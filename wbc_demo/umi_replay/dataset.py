import json
from pathlib import Path

import numpy as np
import pyarrow.parquet as parquet

from .download import sha256
from .geometry import pose_dict, pose_error, rotation_6d


def read_episode(root, episode_index, feature="observation.state"):
    root = Path(root)
    manifest_path = root / "download-manifest.json"
    if not manifest_path.exists():
        raise ValueError("Pinned download manifest is required; run the download command first")
    manifest = json.loads(manifest_path.read_text())
    for item in manifest["files"]:
        if item["path"].startswith(("data/", "meta/")):
            if sha256(root / item["path"]) != item["sha256"]:
                raise ValueError(f"Dataset checksum mismatch: {item['path']}")
    info = json.loads((root / "meta/info.json").read_text())
    if info["robot_type"] != "xtac_umi_g1" or info["codebase_version"] != "v3.0":
        raise ValueError("This adapter only supports the audited xtac_umi_g1 LeRobot v3 schema")
    if feature not in ("observation.state", "action"):
        raise ValueError("Choose observation.state or action explicitly")
    metadata = []
    for path in sorted((root / "meta/episodes").rglob("*.parquet")):
        metadata.extend(parquet.read_table(path).to_pylist())
    matches = [row for row in metadata if row["episode_index"] == episode_index]
    if len(matches) != 1:
        raise ValueError(f"Episode {episode_index} is missing or ambiguous")
    episode = matches[0]
    path = root / info["data_path"].format(
        chunk_index=episode["data/chunk_index"], file_index=episode["data/file_index"]
    )
    table = parquet.read_table(
        path, columns=[feature, "observation.tracker_confidence", "timestamp", "frame_index", "episode_index", "index"]
    )
    rows = [row for row in table.to_pylist() if row["episode_index"] == episode_index]
    rows.sort(key=lambda row: row["frame_index"])
    if len(rows) != episode["length"] or len(rows) < 2:
        raise ValueError("Episode length mismatch or fewer than two frames")
    if [row["frame_index"] for row in rows] != list(range(len(rows))):
        raise ValueError("Missing, duplicate or nonzero-start frame indexes")
    if [row["index"] for row in rows] != list(range(episode["dataset_from_index"], episode["dataset_to_index"])):
        raise ValueError("Episode global indexes disagree with metadata")
    extrinsics = json.loads((root / "meta/taccap_extrinsics.json").read_text())
    entries = [item for item in extrinsics["episodes"] if item["episode_index"] == episode_index]
    if len(entries) != 1 or not entries[0].get("poses_are_ee") or entries[0].get("legacy_world_pose", True):
        raise ValueError("EEF/world convention is not confirmed by the episode extrinsics metadata")
    names = info["features"][feature]["names"]
    frames = []
    for row in rows:
        values = np.array(row[feature], dtype=float)
        confidence = np.array(row["observation.tracker_confidence"], dtype=float)
        if values.shape != (len(names),) or not np.isfinite(values).all():
            raise ValueError("Non-finite or malformed state")
        if confidence.shape != (2,) or not np.isfinite(confidence).all() or min(confidence) < 0.99:
            raise ValueError(f"Low/invalid tracking confidence at frame {row['frame_index']}")
        named = dict(zip(names, values))
        poses = []
        widths = []
        for side in ("left", "right"):
            matrix = np.eye(4)
            matrix[:3, 3] = [named[f"{side}_tcp.{axis}"] for axis in ("x", "y", "z")]
            matrix[:3, :3] = rotation_6d([named[f"{side}_tcp.r{index}"] for index in range(1, 7)])
            poses.append(pose_dict(matrix))
            width = float(named[f"{side}_gripper.pos"])
            if not 0 <= width <= 1:
                raise ValueError("Gripper normalized opening outside [0,1]")
            widths.append(width)
        frames.append(
            {"frame_index": row["frame_index"], "timestamp": float(row["timestamp"]), "eef": poses, "gripper": widths}
        )
    times = np.array([frame["timestamp"] for frame in frames])
    if (
        not np.isfinite(times).all()
        or abs(times[0]) > 0.002
        or not np.allclose(np.diff(times), 1 / info["fps"], atol=0.002, rtol=0)
    ):
        raise ValueError("Invalid timestamps or gaps in the 30 Hz source timeline")
    jumps = [
        pose_error(first, second)
        for previous, current in zip(frames, frames[1:])
        for first, second in zip(previous["eef"], current["eef"])
    ]
    if max(item[0] for item in jumps) > 0.05 or max(item[1] for item in jumps) > 0.35:
        raise ValueError("Tracker discontinuity exceeds 5 cm or 0.35 rad per source frame")
    provenance = {
        "dataset": "TacVerse/xtac-umi-g1-block-to-box",
        "revision": manifest["revision"],
        "data_file": str(path.relative_to(root)),
        "data_sha256": sha256(path),
        "feature": feature,
        "source_episode_id": entries[0]["source_episode_id"],
        "fps": info["fps"],
        "source_frames": len(frames),
        "extrinsics_already_applied": True,
        "max_source_step_m": max(item[0] for item in jumps),
        "max_source_rotation_step_rad": max(item[1] for item in jumps),
    }
    return frames, provenance
