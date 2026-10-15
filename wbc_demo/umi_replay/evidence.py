import csv
import hashlib
import json
import math
import shutil
import subprocess
from datetime import datetime, timezone
from html import escape
from pathlib import Path

import numpy as np


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_verified_run(directory):
    directory = Path(directory)
    report = json.loads((directory / "report.json").read_text())
    metrics = json.loads((directory / "eef-metrics.json").read_text())
    if report.get("status") != "passed" or report.get("tracking_pose_frame") != "gripper_eef":
        raise ValueError("Only passed model gripper EEF runs can be packaged")
    if report.get("gripper_eef_model", {}).get("axis_revision") != "camera_side_up_v2":
        raise ValueError("Historical camera-side-inverted EEF runs are not current acceptance evidence")
    if report.get("return_home", {}).get("status") != "passed" or not report.get("replay"):
        raise ValueError("Replay and return-home evidence are required")
    with (directory / "eef_tracking.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    for side in ("left", "right"):
        selected = [row for row in rows if row["side"] == side]
        if not selected:
            raise ValueError(f"Missing measured EEF samples: {side}")
        positions = np.array([float(row["eef_position_error_m"]) for row in selected])
        rotations = np.array([float(row["eef_rotation_error_rad"]) for row in selected])
        expected = {
            "samples": len(selected),
            "max_position_error_m": float(positions.max()),
            "p95_position_error_m": float(np.percentile(positions, 95)),
            "max_rotation_error_rad": float(rotations.max()),
            "source_time_start": min(float(row["source_time_s"]) for row in selected),
            "source_time_end": max(float(row["source_time_s"]) for row in selected),
        }
        for key, value in expected.items():
            if not math.isclose(metrics["sides"][side][key], value, rel_tol=1e-8, abs_tol=1e-10):
                raise ValueError(f"Metrics disagree with EEF CSV: {side}/{key}")
    return report, metrics


def contact_sheet(video, destination, count, columns, label):
    import cv2
    from PIL import Image, ImageDraw

    capture = cv2.VideoCapture(str(video))
    try:
        fps = capture.get(cv2.CAP_PROP_FPS)
        frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        if not capture.isOpened() or fps <= 0 or frames < count:
            raise ValueError(f"Video cannot be decoded: {video}")
        indexes = np.linspace(0, frames - 1, count).astype(int)
        width = 640 if columns > 1 else 960
        height = round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT) * width / capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        tile_height = height + 38
        sheet = Image.new("RGB", (width * columns, tile_height * math.ceil(count / columns)), "#152536")
        drawing = ImageDraw.Draw(sheet)
        entries = []
        for index, frame_index in enumerate(indexes):
            capture.set(cv2.CAP_PROP_POS_FRAMES, int(frame_index))
            success, frame = capture.read()
            if not success:
                raise ValueError(f"Truncated video: {video}, frame {frame_index}")
            image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).resize((width, height))
            column = index % columns
            row = index // columns
            sheet.paste(image, (column * width, row * tile_height + 38))
            timestamp = float(frame_index / fps)
            caption = f"{label} | video {timestamp:.2f}s | frame {frame_index}"
            drawing.text((column * width + 10, row * tile_height + 11), caption, fill="white")
            entries.append({"video_time_s": timestamp, "video_frame": int(frame_index)})
        sheet.save(destination)
        return {"fps": fps, "frames": frames, "duration_s": frames / fps, "thumbnails": entries}
    finally:
        capture.release()


def build_evidence(args):
    report, metrics = load_verified_run(args.run_directory)
    inputs = {
        "robot-summary-input": args.robot_video,
        "source-preview-input": args.source_video,
        "model-diagram-input": args.eef_diagram,
        **{
            name: args.run_directory / name
            for name in ("report.json", "eef-metrics.json", "tracking.csv", "eef_tracking.csv", "tracking-summary.png")
        },
    }
    synchronized = None
    if args.synchronized_directory is not None:
        synchronized = json.loads((args.synchronized_directory / "manifest.json").read_text())
        if synchronized["inputs"].get(str(args.run_directory / "report.json")) != sha256(
            args.run_directory / "report.json"
        ):
            raise ValueError("Synchronized video does not match this run report")
        inputs.update(
            {
                "head_left.mp4": args.synchronized_directory / "head_left.mp4",
                "head-left-frame.png": args.synchronized_directory / "head_left-frame-00625.png",
                "frame_alignment.csv": args.synchronized_directory / "frame_alignment.csv",
                "alignment-manifest.json": args.synchronized_directory / "manifest.json",
            }
        )
    if args.review_directory is not None:
        review = json.loads((args.review_directory / "review.json").read_text())
        if review["runs"]["corrected"]["report_sha256"] != sha256(args.run_directory / "report.json"):
            raise ValueError("Waist review does not match this run report")
        inputs.update(
            {
                name: args.review_directory / name
                for name in ("waist-roll-before-after.png", "camera-axis-before-after.png", "review.json")
            }
        )
    for path in inputs.values():
        if not path.is_file():
            raise FileNotFoundError(path)
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("Activate xense-wbc first: ffmpeg must be on PATH")
    args.output.mkdir(parents=True, exist_ok=False)
    if args.doc_media is not None:
        args.doc_media.mkdir(parents=True, exist_ok=False)
    for name in ("report.json", "eef-metrics.json", "tracking.csv", "eef_tracking.csv", "tracking-summary.png"):
        shutil.copyfile(inputs[name], args.output / name)
    for name in (
        "head_left.mp4",
        "head-left-frame.png",
        "frame_alignment.csv",
        "alignment-manifest.json",
        "waist-roll-before-after.png",
        "camera-axis-before-after.png",
        "review.json",
    ):
        if name in inputs:
            shutil.copyfile(inputs[name], args.output / name)
    shutil.copyfile(args.eef_diagram, args.output / "gripper-eef-frame.png")
    video_info = {
        "robot": contact_sheet(args.robot_video, args.output / "robot-storyboard.png", 6, 3, "SIM / 8x overview"),
        "source": contact_sheet(args.source_video, args.output / "source-storyboard.png", 3, 1, "HF / source 1x"),
    }
    with (args.output / "encoding.log").open("w") as log:
        for source, target in ((args.robot_video, "robot-8x.mp4"), (args.source_video, "source-1x.mp4")):
            subprocess.run(
                [
                    ffmpeg,
                    "-nostdin",
                    "-n",
                    "-i",
                    str(source),
                    "-an",
                    "-c:v",
                    "libx264",
                    "-threads",
                    "2",
                    "-preset",
                    "fast",
                    "-crf",
                    "23",
                    "-pix_fmt",
                    "yuv420p",
                    "-movflags",
                    "+faststart",
                    str(args.output / target),
                ],
                check=True,
                stdout=log,
                stderr=log,
            )
        subprocess.run(
            [
                ffmpeg,
                "-nostdin",
                "-n",
                "-i",
                str(args.robot_video),
                "-filter_complex_threads",
                "1",
                "-filter_complex",
                "fps=4,scale=480:-1:flags=lanczos,split[preview][palette];"
                "[palette]palettegen=max_colors=64[colors];[preview][colors]paletteuse=dither=bayer",
                "-loop",
                "0",
                str(args.output / "robot-preview.gif"),
            ],
            check=True,
            stdout=log,
            stderr=log,
        )
    maximum_position = max(side["max_position_error_m"] for side in metrics["sides"].values()) * 1000
    maximum_rotation = math.degrees(max(side["max_rotation_error_rad"] for side in metrics["sides"].values()))
    body_range = math.degrees(max(metrics["body_joint_range_rad"].values()))
    template = Path(__file__).with_name("evidence_template.html").read_text()
    replacements = {
        "POSITION_MM": f"{maximum_position:.3f}",
        "ROTATION_DEG": f"{maximum_rotation:.3f}",
        "BODY_DEG": f"{body_range:.3f}",
        "BASE_UM": f"{report['replay']['max_base_translation_drift_m'] * 1e6:.3f}",
        "SOURCE_END": f"{metrics['sides']['left']['source_time_end']:.3f}",
        "WALL_SECONDS": f"{report['replay']['wall_seconds']:.3f}",
        "REVISION": escape(report["provenance"]["revision"]),
        "EPISODE_ID": escape(report["provenance"]["source_episode_id"]),
        "RUN_TIME": escape(report["started_at"] + " → " + report["finished_at"]),
        "PREFLIGHT_COUNT": str(len(report["preflight"])),
        "SYNCHRONIZED_SECTION": "",
        "REVIEW_SECTION": "",
    }
    if synchronized is not None:
        replacements["SYNCHRONIZED_SECTION"] = (
            "<section><h2>0. 推荐先看：UMI head_left 与机器人同源相位分屏</h2>"
            '<video controls preload="metadata" playsinline src="head_left.mp4" poster="head-left-frame.png"></video>'
            f'<p>左：原始 UMI head_left；右：本轮实际全身 IK 仿真。{synchronized["frames"]} 帧 / '
            f'{synchronized["fps"]} fps = {synchronized["duration_seconds"]:.6f} 秒，两栏在同一帧中，时长一致。</p>'
            f"<p>根据每条命令源相位和每帧相机 ROS 时间对齐，最大时间差 "
            f'{synchronized["max_camera_timestamp_gap_seconds"] * 1000:.3f} ms；'
            f'换算到源相位的最大差值 {synchronized["max_source_phase_gap_seconds"] * 1000:.3f} ms，'
            f'P95 为 {synchronized["p95_source_phase_gap_seconds"] * 1000:.3f} ms。'
            "不是标定过延迟的硬件同步，也不是实时控制速度。右侧不是机器人头部相机视角。</p>"
            '<p><a href="frame_alignment.csv">逐帧对齐索引</a> · '
            '<a href="alignment-manifest.json">对齐方法与输入哈希</a></p></section>'
        )
    if args.review_directory is not None:
        replacements["REVIEW_SECTION"] = (
            "<section><h2>腰部侧倾与腕部相机：修正前后核查</h2>"
            '<img src="waist-roll-before-after.png" alt="旧新计划及实际腰部 roll">'
            '<img src="camera-axis-before-after.png" alt="相机与夹指网格在旧新 EEF 的侧面投影">'
            "<p>本次同时修正工具轴和姿态偏好，不是单变量权重消融；"
            "原始模型的相机安装未改动。新 IK 请求限制 roll ±0.5°，保留其他腰部与双臂自由度。</p>"
            '<a href="review.json">新旧 roll 数值与因果边界</a></section>'
        )
    for key, value in replacements.items():
        template = template.replace("{{" + key + "}}", value)
    (args.output / "index.html").write_text(template)
    manifest = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "simulation_rerun": False,
        "tracking_pose_frame": "gripper_eef",
        "run_provenance": report["provenance"],
        "video_timeline": "Overview videos use independent PTS; only head_left split-screen, when provided, is command-source-phase aligned using ROS timestamps.",
        "source_license": "TacVerse/xtac-umi-g1-block-to-box / CC-BY-SA-4.0; preview resized and H264 encoded",
        "video_info": video_info,
        "inputs": {name: {"path": str(path), "sha256": sha256(path)} for name, path in inputs.items()},
        "files": {
            path.name: {"bytes": path.stat().st_size, "sha256": sha256(path)}
            for path in sorted(args.output.iterdir())
            if path.is_file()
        },
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    if args.doc_media is not None:
        for name in (
            "robot-storyboard.png",
            "source-storyboard.png",
            "tracking-summary.png",
            "gripper-eef-frame.png",
            "robot-preview.gif",
            "manifest.json",
        ):
            shutil.copyfile(args.output / name, args.doc_media / name)
        for name in ("head-left-frame.png", "waist-roll-before-after.png", "camera-axis-before-after.png"):
            if (args.output / name).exists():
                shutil.copyfile(args.output / name, args.doc_media / name)
    print(f"Offline visual acceptance page: {args.output / 'index.html'}", flush=True)
    return manifest
