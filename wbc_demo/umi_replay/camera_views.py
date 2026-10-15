import json
from pathlib import Path

import numpy as np
import pyarrow.parquet as parquet
import yaml

from .download import sha256
from .geometry import interpolate_pose, pose_dict, pose_matrix, rotation_6d

VIEW_NAMES = ("UmiEgo", "ExternalA", "ExternalB")
VIEW_TOPICS = tuple(f"/umi_replay/views/{name}/image_raw" for name in VIEW_NAMES)
GRAPHICS_IN_HEAD = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])


def fixed_render_root(scene):
    from scipy.spatial.transform import Rotation

    robot = scene["robot"]
    if not robot["robot_source"].get("pin_base_to_world"):
        raise ValueError("This render-root bridge requires a pinned robot base")
    initial = robot["init_base_pose"]
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler("z", initial.get("theta", 0.0)).as_matrix()
    matrix[:3, 3] = [initial.get(axis, 0.0) for axis in ("x", "y", "z")]
    return {"prim_path": "/" + robot["robot_prefix"], "pose": pose_dict(matrix)}


def look_at(position, target):
    position = np.array(position, dtype=float)
    backward = position - np.array(target, dtype=float)
    if np.linalg.norm(backward) < 1e-8:
        raise ValueError("Camera position must differ from its target")
    backward /= np.linalg.norm(backward)
    right = np.cross([0, 0, 1], backward)
    if np.linalg.norm(right) < 1e-8:
        raise ValueError("Use a camera direction not parallel to world up")
    right /= np.linalg.norm(right)
    matrix = np.eye(4)
    matrix[:3, :3] = np.column_stack([right, np.cross(backward, right), backward])
    matrix[:3, 3] = position
    return matrix


def head_track(root, converted):
    root = Path(root)
    path = root / converted["provenance"]["data_file"]
    if sha256(path) != converted["provenance"]["data_sha256"]:
        raise ValueError("Head track data does not match the converted EEF source")
    info = json.loads((root / "meta/info.json").read_text())
    names = info["features"]["observation.state"]["names"]
    rows = parquet.read_table(
        path, columns=["observation.state", "episode_index", "frame_index", "timestamp"]
    ).to_pylist()
    rows = sorted(
        (row for row in rows if row["episode_index"] == converted["episode_index"]), key=lambda row: row["frame_index"]
    )
    if len(rows) != len(converted["source_frames"]):
        raise ValueError("Head/EEF episode lengths differ")
    result = []
    for row, source in zip(rows, converted["source_frames"]):
        if row["frame_index"] != source["frame_index"] or abs(row["timestamp"] - source["timestamp"]) > 1e-6:
            raise ValueError("Head/EEF frame indexes or timestamps differ")
        values = dict(zip(names, row["observation.state"]))
        matrix = np.eye(4)
        matrix[:3, 3] = [values[f"head.{axis}"] for axis in ("x", "y", "z")]
        matrix[:3, :3] = rotation_6d([values[f"head.r{index}"] for index in range(1, 7)])
        result.append({"timestamp": row["timestamp"], "pose": pose_dict(matrix)})
    return result


def ego_camera(track, source_time, world):
    times = np.array([frame["timestamp"] for frame in track])
    following = int(np.clip(np.searchsorted(times, source_time, side="right"), 1, len(times) - 1))
    fraction = float(np.clip((source_time - times[following - 1]) / (times[following] - times[following - 1]), 0, 1))
    head = world @ pose_matrix(interpolate_pose(track[following - 1]["pose"], track[following]["pose"], fraction))
    head[:3, :3] = head[:3, :3] @ GRAPHICS_IN_HEAD
    return head


def source_capture(root, converted):
    import cv2

    root = Path(root)
    info = json.loads((root / "meta/info.json").read_text())
    episodes = []
    for path in sorted((root / "meta/episodes").rglob("*.parquet")):
        episodes.extend(parquet.read_table(path).to_pylist())
    episode = next(row for row in episodes if row["episode_index"] == converted["episode_index"])
    key = "observation.images.head_left"
    path = root / info["video_path"].format(
        video_key=key, chunk_index=episode[f"videos/{key}/chunk_index"], file_index=episode[f"videos/{key}/file_index"]
    )
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open source head_left: {path}")
    return capture, round(episode[f"videos/{key}/from_timestamp"] * info["fps"]), info["fps"]


def make_scene(base_scene, root, converted, config, output):
    world = np.eye(4)
    from scipy.spatial.transform import Rotation

    if config["mode"] == "calibrated":
        world = np.array(config["world_to_odom"])
    else:
        if config["mode"] != "shared_world_gripper_eef":
            raise ValueError("Unsupported camera world-alignment policy")
        world[:3, :3] = Rotation.from_euler("z", config["world_yaw_deg"], degrees=True).as_matrix()
        midpoint = np.mean([pose["position"] for pose in converted["source_frames"][0]["eef"]], axis=0)
        world[:3, 3] = np.array(config["initial_midpoint_odom_m"]) - world[:3, :3] @ midpoint
    track = head_track(root, converted)
    poses = (
        ego_camera(track, 0, world),
        look_at([1.25, 1.25, 1.6], [-0.25, 0, 0.95]),
        look_at([-0.25, -1.9, 1.4], [-0.25, 0, 0.95]),
    )
    scene = yaml.safe_load(Path(base_scene).read_text())
    scene["robot"]["robot_name"] = "G2_umi_multiview"
    scene.update({"free_cam_width": 640, "free_cam_height": 480, "cameras": []})
    for name, topic, matrix in zip(VIEW_NAMES, VIEW_TOPICS, poses):
        pose = pose_dict(matrix)
        quaternion = pose["quaternion_xyzw"]
        scene["cameras"].append(
            {
                "prim_path": name,
                "frame_id": "odom",
                "topic": {"rgb": topic.removesuffix("/image_raw")},
                "sensor": {"width": 640, "height": 480, "freq": 30.0, "min_range": 0.01, "max_range": 100},
                "intrinsic": {"fx": 380.0, "fy": 380.0, "cx": 320.0, "cy": 240.0},
                "extrinsic": {"xyz": pose["position"], "wxyz": [quaternion[3], *quaternion[:3]]},
            }
        )
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(scene, sort_keys=False))
    audit = {
        "world_to_odom": world.tolist(),
        "alignment_config": config,
        "head_track": track,
        "source_provenance": converted["provenance"],
        "camera_convention": "head +X forward/+Y left/+Z up -> USD camera -Z forward/+Y image up",
        "calibration_status": "virtual head-pose-aligned camera; head_left extrinsics/intrinsics unavailable; not a calibrated robot head sensor",
        "head_graphics_rotation": GRAPHICS_IN_HEAD.tolist(),
        "scene_sha256": sha256(output),
        "render_robot_root": fixed_render_root(scene),
    }
    output.with_suffix(".json").write_text(json.dumps(audit, indent=2) + "\n")
    return output
