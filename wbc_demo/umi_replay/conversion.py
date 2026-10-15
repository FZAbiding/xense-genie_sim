import json
from pathlib import Path

import numpy as np

from .dataset import read_episode
from .geometry import interpolate_pose, map_poses, pose_dict, pose_error, pose_matrix


def select_keyframes(
    frames, max_seconds=0.2, position_tolerance=0.005, rotation_tolerance=0.03, gripper_tolerance=0.02
):
    if min(max_seconds, position_tolerance, rotation_tolerance, gripper_tolerance) <= 0:
        raise ValueError("Sampling tolerances must be positive")
    selected = {0, len(frames) - 1}
    anchor = 0
    for index in range(1, len(frames)):
        if frames[index]["timestamp"] - frames[anchor]["timestamp"] >= max_seconds - 1e-5:
            selected.add(index)
            anchor = index
    pending = list(zip(sorted(selected), sorted(selected)[1:]))
    while pending:
        start, end = pending.pop()
        worst = (0.0, None)
        for index in range(start + 1, end):
            fraction = (frames[index]["timestamp"] - frames[start]["timestamp"]) / (
                frames[end]["timestamp"] - frames[start]["timestamp"]
            )
            for side in range(2):
                predicted = interpolate_pose(frames[start]["eef"][side], frames[end]["eef"][side], fraction)
                position, rotation = pose_error(predicted, frames[index]["eef"][side])
                score = max(position / position_tolerance, rotation / rotation_tolerance)
                if score > worst[0]:
                    worst = (score, index)
            if "gripper" in frames[index]:
                predicted = (1 - fraction) * np.array(frames[start]["gripper"]) + fraction * np.array(
                    frames[end]["gripper"]
                )
                score = float(np.max(np.abs(predicted - frames[index]["gripper"]))) / gripper_tolerance
                if score > worst[0]:
                    worst = (score, index)
        if worst[0] > 1:
            selected.add(worst[1])
            pending.extend([(start, worst[1]), (worst[1], end)])
    return sorted(selected)


def convert(root, episode_index, output, feature="observation.state", max_seconds=0.2):
    frames, provenance = read_episode(root, episode_index, feature)
    selected = select_keyframes(frames, max_seconds)
    result = {
        "schema": "xense-umi-eef-v1",
        "episode_index": episode_index,
        "provenance": provenance,
        "frame": "dataset_world",
        "units": "meters, seconds, quaternion_xyzw",
        "rotation_input": "first_two_columns",
        "source_duration_seconds": frames[-1]["timestamp"],
        "source_frames": frames,
        "keyframe_indexes": selected,
        "sampling": {
            "max_interval_seconds": max_seconds,
            "max_position_error_m": 0.005,
            "max_rotation_error_rad": 0.03,
            "max_gripper_opening_error": 0.02,
        },
        "gripper_policy": "normalized_0_closed_1_open; URDF joint-range mapping; opening-dependent TCP",
        "head_policy": "not_replayed",
    }
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite {output}")
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(
        f"Converted episode {episode_index}: {len(frames)} raw frames -> {len(selected)} bounded-error keyframes: {output}",
        flush=True,
    )
    return result


def mapped_frames(converted, world_to_odom, tip_to_eef):
    return [
        {**frame, "tips": map_poses(frame["eef"], world_to_odom, tip_to_eef)}
        for frame in (converted["source_frames"][index] for index in converted["keyframe_indexes"])
    ]


def mapped_eef_frames(converted, world_to_odom):
    return [
        {**frame, "eef_odom": [pose_dict(world_to_odom @ pose_matrix(pose)) for pose in frame["eef"]]}
        for frame in (converted["source_frames"][index] for index in converted["keyframe_indexes"])
    ]
