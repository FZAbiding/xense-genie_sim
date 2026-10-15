import math

import numpy as np

from .geometry import pose_matrix


def validate_realtime_profile(profile):
    if not isinstance(profile, dict):
        raise ValueError("Realtime limits profile must be a JSON object")
    for group in ("waist", "arm", "gripper"):
        limits = profile.get(group)
        if not isinstance(limits, dict):
            raise ValueError(f"Missing realtime limits group: {group}")
        for key in ("velocity_rad_s", "acceleration_rad_s2"):
            value = limits.get(key)
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"Finite positive realtime limit required: {group}.{key}")
    lookahead = profile.get("lookahead_seconds", 0.0)
    if (
        isinstance(lookahead, bool)
        or not isinstance(lookahead, (float, int))
        or not math.isfinite(lookahead)
        or not 0 <= lookahead <= 1 / 30
    ):
        raise ValueError("Replay target lookahead must be between zero and one source frame")
    return profile


def validate_realtime_source(frames):
    if len(frames) < 2:
        raise ValueError("Realtime replay requires at least two source frames")
    timestamps = np.array([frame["timestamp"] for frame in frames], dtype=float)
    if (
        not np.isfinite(timestamps).all()
        or abs(timestamps[0]) > 1e-5
        or not np.allclose(np.diff(timestamps), 1 / 30, atol=2e-5)
    ):
        raise ValueError("Realtime replay requires the complete finite 30 Hz episode starting at zero")
    for frame in frames:
        if len(frame["eef"]) != 2:
            raise ValueError("Two source gripper EEF poses required")
        for pose in frame["eef"]:
            pose_matrix(pose)
        openings = np.asarray(frame["gripper"], dtype=float)
        if openings.shape != (2,) or not np.isfinite(openings).all() or np.any(openings < 0) or np.any(openings > 1):
            raise ValueError("Two finite normalized gripper openings in [0,1] required")
    return timestamps
