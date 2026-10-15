import csv
import json
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pyarrow.parquet as parquet

from .download import sha256


def match_source_frames(source_times, command_rows, camera_rows, maximum_gap):
    phases = np.array([float(row["source_time_s"]) for row in command_rows])
    commands = np.array([float(row["command_ros_time_s"]) for row in command_rows])
    image_times = np.array([float(row["image_ros_time_s"]) for row in camera_rows])
    video_frames = np.array([int(row["video_frame"]) for row in camera_rows])
    if min(len(phases), len(image_times)) < 2:
        raise ValueError(
            "Command and camera timestamp traces are required; old untimestamped videos cannot be synchronized"
        )
    if not np.isfinite(np.concatenate((phases, commands, image_times))).all() or np.any(image_times <= 0):
        raise ValueError("Invalid ROS timestamps")
    if np.any(np.diff(phases) < -1e-9) or np.any(np.diff(commands) <= 0):
        raise ValueError("Command phases and ROS timestamps must be monotonic")
    if source_times[0] < phases[0] - 1e-6 or source_times[-1] > phases[-1] + 1e-6:
        raise ValueError("Command trace does not cover the complete source episode")
    if not np.isfinite(maximum_gap) or maximum_gap <= 0:
        raise ValueError("Positive camera timestamp-gap limit required")
    requested = np.interp(source_times, phases, commands)
    order = np.argsort(image_times, kind="stable")
    ordered = image_times[order]
    following = np.clip(np.searchsorted(ordered, requested), 0, len(ordered) - 1)
    preceding = np.maximum(0, following - 1)
    selected = np.where(
        np.abs(ordered[following] - requested) < np.abs(ordered[preceding] - requested), following, preceding
    )
    gaps = np.abs(ordered[selected] - requested)
    if gaps.max() > maximum_gap:
        raise ValueError(
            f"Camera does not cover source phase without gaps: maximum timestamp mismatch {gaps.max():.3f}s"
        )
    return video_frames[order[selected]], requested, ordered[selected], gaps


def letterbox(image, width, height):
    import cv2

    scale = min(width / image.shape[1], height / image.shape[0])
    resized = cv2.resize(image, (round(image.shape[1] * scale), round(image.shape[0] * scale)))
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    offset_x = (width - resized.shape[1]) // 2
    offset_y = (height - resized.shape[0]) // 2
    canvas[offset_y : offset_y + resized.shape[0], offset_x : offset_x + resized.shape[1]] = resized
    return canvas


def compose_frame(source_image, robot_image, layout, label):
    import cv2

    height = 1064 if layout == "quad" else 544
    composite = np.zeros((height, 1280, 3), dtype=np.uint8)
    if layout == "quad":
        if robot_image.shape[:2] != (480, 1920):
            raise ValueError("Quad layout requires the timestamp-matched 1920x480 three-camera recording")
        images = [source_image, *np.hsplit(robot_image, 3)]
        titles = [
            "UMI head_left / source 1x",
            "Virtual ego / UMI head-pose direction",
            "External A / full-body",
            "External B / full-body",
        ]
    else:
        images = [source_image, robot_image]
        titles = ["UMI head_left / source 1x", "G2 actual IK replay / source-phase retimed"]
    for index, (image, title) in enumerate(zip(images, titles)):
        column = index % 2
        row = index // 2
        offset_x = column * 640
        offset_y = row * 520
        composite[offset_y + 40 : offset_y + 520, offset_x : offset_x + 640] = letterbox(image, 640, 480)
        cv2.putText(composite, title, (offset_x + 12, offset_y + 27), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(composite, label, (12, height - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    return composite


def synchronized_video(args, converted):
    import cv2

    directory = args.run_directory
    report = json.loads((directory / "report.json").read_text())
    if report.get("status") != "passed" or report.get("tracking_pose_frame") != "gripper_eef":
        raise ValueError("A passed actual gripper EEF replay is required")
    if report["provenance"] != converted["provenance"] or report["input_sha256"] != sha256(args.input):
        raise ValueError("Source episode does not match the recorded replay")
    with (directory / "command_phase.csv").open() as stream:
        commands = list(csv.DictReader(stream))
    with args.camera_trace.open() as stream:
        camera = list(csv.DictReader(stream))
    times = np.array([frame["timestamp"] for frame in converted["source_frames"]])
    indexes, command_times, image_times, gaps = match_source_frames(times, commands, camera, args.max_camera_gap)
    image_phases = np.interp(
        image_times,
        [float(row["command_ros_time_s"]) for row in commands],
        [float(row["source_time_s"]) for row in commands],
    )
    phase_gaps = np.abs(image_phases - times)
    if not np.isfinite(args.max_source_phase_gap) or args.max_source_phase_gap <= 0:
        raise ValueError("Positive source-phase mismatch limit required")
    if phase_gaps.max() > args.max_source_phase_gap:
        raise ValueError(f"Source-phase mismatch exceeds the review limit: {phase_gaps.max():.3f}s")
    if np.any(np.diff(indexes) < 0):
        raise ValueError("Matched camera frames move backward in recording order")
    info = json.loads((args.root / "meta/info.json").read_text())
    metadata = []
    for path in sorted((args.root / "meta/episodes").rglob("*.parquet")):
        metadata.extend(parquet.read_table(path).to_pylist())
    episode = next(item for item in metadata if item["episode_index"] == converted["episode_index"])
    key = "observation.images.head_left"
    source_path = args.root / info["video_path"].format(
        video_key=key, chunk_index=episode[f"videos/{key}/chunk_index"], file_index=episode[f"videos/{key}/file_index"]
    )
    fps = info["fps"]
    source_start = round(episode[f"videos/{key}/from_timestamp"] * fps)
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("Activate xense-wbc: ffmpeg is required")
    args.output.mkdir(parents=True, exist_ok=False)
    source_capture = cv2.VideoCapture(str(source_path))
    robot_capture = cv2.VideoCapture(str(args.robot_video))
    encoder = None
    try:
        if not source_capture.isOpened() or not robot_capture.isOpened():
            raise ValueError("Both source head_left and recorded robot videos must be decodable")
        if abs(source_capture.get(cv2.CAP_PROP_FPS) - fps) > 0.01:
            raise ValueError("Unexpected source video FPS")
        if max(indexes) >= robot_capture.get(cv2.CAP_PROP_FRAME_COUNT):
            raise ValueError("Camera timestamp trace indexes exceed video length")
        source_capture.set(cv2.CAP_PROP_POS_FRAMES, source_start)
        with (args.output / "encoding.log").open("w") as log, (args.output / "frame_alignment.csv").open("w") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                [
                    "output_frame",
                    "source_time_s",
                    "source_video_frame",
                    "robot_video_frame",
                    "command_ros_time_s",
                    "image_ros_time_s",
                    "timestamp_gap_s",
                    "robot_image_source_phase_s",
                    "source_phase_gap_s",
                ]
            )
            encoder = subprocess.Popen(
                [
                    ffmpeg,
                    "-nostdin",
                    "-n",
                    "-f",
                    "rawvideo",
                    "-pix_fmt",
                    "bgr24",
                    "-s",
                    "1280x1064" if args.layout == "quad" else "1280x544",
                    "-r",
                    str(fps),
                    "-i",
                    "pipe:0",
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
                    str(args.output / "head_left.mp4"),
                ],
                stdin=subprocess.PIPE,
                stdout=log,
                stderr=log,
            )
            last_index = -1
            robot_image = None
            for index, timestamp in enumerate(times):
                success, source_image = source_capture.read()
                if not success:
                    raise ValueError(f"Truncated source head_left at episode frame {index}")
                selected = int(indexes[index])
                if selected != last_index:
                    robot_capture.set(cv2.CAP_PROP_POS_FRAMES, selected)
                    success, robot_image = robot_capture.read()
                    if not success:
                        raise ValueError(f"Truncated robot recording at video frame {selected}")
                    last_index = selected
                label = f"episode {converted['episode_index']} | frame {index} | phase {timestamp:.3f}s | camera gap {gaps[index]:.3f}s | phase gap {phase_gaps[index]:.3f}s"
                composite = compose_frame(source_image, robot_image, args.layout, label)
                encoder.stdin.write(composite.tobytes())
                writer.writerow(
                    [
                        index,
                        timestamp,
                        source_start + index,
                        selected,
                        command_times[index],
                        image_times[index],
                        gaps[index],
                        image_phases[index],
                        phase_gaps[index],
                    ]
                )
                if index in (0, len(times) // 2, len(times) - 1):
                    cv2.imwrite(str(args.output / f"head_left-frame-{index:05d}.png"), composite)
            encoder.stdin.close()
            if encoder.wait() != 0:
                raise RuntimeError("Video encoder failed; see encoding.log")
    finally:
        if encoder is not None and encoder.poll() is None:
            encoder.terminate()
            encoder.wait()
        source_capture.release()
        robot_capture.release()
    manifest = {
        "comparison": "UMI head_left vs actual physical replay, nearest image ROS timestamp at each commanded source phase",
        "frames": len(times),
        "fps": fps,
        "duration_seconds": len(times) / fps,
        "source_phase_end_seconds": float(times[-1]),
        "physical_replay_seconds": report["replay"]["wall_seconds"],
        "max_camera_timestamp_gap_seconds": float(gaps.max()),
        "p95_camera_timestamp_gap_seconds": float(np.percentile(gaps, 95)),
        "allowed_max_camera_timestamp_gap_seconds": args.max_camera_gap,
        "max_source_phase_gap_seconds": float(phase_gaps.max()),
        "p95_source_phase_gap_seconds": float(np.percentile(phase_gaps, 95)),
        "allowed_max_source_phase_gap_seconds": args.max_source_phase_gap,
        "real_time_control": False,
        "layout": args.layout,
        "views": (
            ["UMI head_left", "virtual head-pose-aligned ego", "external A", "external B"]
            if args.layout == "quad"
            else ["UMI head_left", "external robot"]
        ),
        "alignment_limitations": "Camera timestamps label rendered TF state; this is command-phase alignment, not calibrated sensor latency or identical viewpoints.",
        "source_license": "TacVerse/xtac-umi-g1-block-to-box / CC-BY-SA-4.0; head_left excerpt resized and composed",
        "provenance": report["provenance"],
        "robot_model": report["gripper_eef_model"],
        "inputs": {
            str(path): sha256(path)
            for path in (
                args.input,
                directory / "report.json",
                directory / "command_phase.csv",
                args.camera_trace,
                args.robot_video,
                source_path,
            )
        },
        "output_sha256": sha256(args.output / "head_left.mp4"),
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    print(f"Source-phase split-screen: {args.output / 'head_left.mp4'}", flush=True)
    return manifest
