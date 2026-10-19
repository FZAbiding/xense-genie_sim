import json
import csv
from pathlib import Path

import numpy as np

from .download import sha256
from .geometry import pose_matrix


def audit_hold_transitions(directory):
    directory = Path(directory)
    report = json.loads((directory / "report.json").read_text())
    replay = report["replay"]
    with (directory / "realtime.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    with (directory / "command_joints.csv").open() as stream:
        commands = list(csv.DictReader(stream))
    names = replay["active_joints"]
    velocities, phases = [], []
    command_index = 0
    for row in rows:
        phases.append(float(row["source_time_s"]))
        if row["published"] == "True":
            if command_index >= len(commands):
                raise ValueError("Missing published command row")
            command = commands[command_index]
            command_index += 1
            if abs(float(command["source_time_s"]) - phases[-1]) > 1e-6:
                raise ValueError("Command and observation phase mismatch")
            velocities.append([float(command[name + "_velocity"]) for name in names])
        elif row["published"] == "False":
            velocities.append([0.0] * len(names))
        else:
            raise ValueError("Unknown publication outcome")
    if (
        not rows
        or command_index != len(commands)
        or command_index != replay["commands"]
        or sum(row["published"] == "False" for row in rows) != replay["deadline_misses"]
        or not np.isfinite(velocities).all()
        or not np.isfinite(phases).all()
    ):
        raise ValueError("Nonempty finite complete command trace required")
    ticks = np.array([int(row["tick"]) for row in rows])
    complete = ticks[0] == 0 and np.all(np.diff(ticks) == 1)
    acceleration = np.diff(np.vstack([np.zeros(len(names)), velocities]), axis=0) * 30
    limits = np.asarray(replay["planned_acceleration_limits_rad_s2"], dtype=float)
    if limits.shape != (len(names),) or not np.isfinite(limits).all() or np.any(limits <= 0):
        raise ValueError("Finite positive acceleration limits required")
    excess = np.abs(acceleration) - limits
    index = np.unravel_index(excess.argmax(), excess.shape)
    result = {
        "status": (
            "failed_nominal_acceleration"
            if complete and excess[index] > 1e-3
            else "passed" if complete else "incomplete_skipped_ticks"
        ),
        "execution_status": report.get("status", "unknown"),
        "partial": report.get("status") != "passed",
        "hold_ticks": [int(row["tick"]) for row in rows if row["published"] == "False"],
        "max_nominal_acceleration_excess_rad_s2": max(0.0, float(excess[index])),
        "worst_joint": names[index[1]],
        "worst_phase_s": phases[index[0]],
        "worst_acceleration_rad_s2": float(acceleration[index]),
        "limit_rad_s2": replay["planned_acceleration_limits_rad_s2"][index[1]],
        "contract": "Nominal 30 Hz position-command integration including zero-advance HOLD transitions; not measured physical acceleration or wall-clock certification.",
    }
    (directory / "hold-aware-audit.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    return result


def decompose_tick(record):
    target = np.array([pose_matrix(pose)[:3, 3] for pose in record["eef_target"]])
    measured = np.array([pose_matrix(pose)[:3, 3] for pose in record["eef_measured"]])
    command = np.array([pose_matrix(pose)[:3, 3] for pose in record["eef_last_command"]])
    if target.shape != (2, 3) or measured.shape != target.shape or command.shape != target.shape:
        raise ValueError("Exactly two EEFs required in each snapshot")
    total, task, following = target - measured, target - command, command - measured
    return total, task, following


def verify_tick_geometry(directory, samples=100):
    from .differential_ik import URDFKinematics, matrix_error
    from .gripper_eef import SwiftpickerEEF

    directory = Path(directory)
    report = json.loads((directory / "report.json").read_text())
    records = [
        json.loads(line) for line in (directory / "control_ticks.jsonl").read_text().splitlines() if line.strip()
    ]
    if not records or samples < 1:
        raise ValueError("Nonempty snapshots and positive sample count required")
    package = Path(__file__).resolve().parents[2] / "source/geniesim_ros/src/ros_ws/src/genie_sim_robot_model"
    urdf = package / "urdf/genie_g2_crsB_swiftpicker.urdf"
    if sha256(urdf) != report["gripper_eef_model"]["urdf_sha256"]:
        raise ValueError("URDF differs from the recorded model")
    eef = SwiftpickerEEF(urdf, package)
    model = URDFKinematics(eef, report["replay"]["model_root_to_odom"], report["replay"]["active_joints"])
    indexes = np.unique(np.linspace(0, len(records) - 1, min(samples, len(records))).astype(int))
    errors = []
    for index in indexes:
        record = records[index]
        for joint_key, pose_key in (
            ("positions_measured", "eef_measured"),
            ("positions_last_command", "eef_last_command"),
        ):
            computed, unused = model.evaluate(record[joint_key], jacobian=False)
            for measured, stored in zip(computed, record[pose_key]):
                error = matrix_error(measured, pose_matrix(stored))
                errors.append([np.linalg.norm(error[:3]), np.linalg.norm(error[3:])])
    maxima = np.max(errors, axis=0)
    result = {
        "status": "passed" if maxima.max() < 1e-8 else "failed",
        "sampled_ticks": len(indexes),
        "max_position_difference_m": float(maxima[0]),
        "max_rotation_difference_rad": float(maxima[1]),
        "urdf_sha256": sha256(urdf),
        "snapshots_sha256": sha256(directory / "control_ticks.jsonl"),
        "contract": "Independent URDF FK checks logged measured and last-command poses against the exact joint snapshots; not actuator acknowledgement.",
    }
    (directory / "tick-geometry.json").write_text(json.dumps(result, indent=2) + "\n")
    if result["status"] != "passed":
        raise ValueError("Logged poses disagree with their joint snapshots")
    return result


def analyze_ticks(directory):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as pyplot

    directory = Path(directory)
    source = directory / "control_ticks.jsonl"
    report = json.loads((directory / "report.json").read_text())
    records = [json.loads(line) for line in source.read_text().splitlines() if line.strip()]
    if not records:
        raise ValueError("No synchronized control snapshots")
    ticks = np.array([record["tick"] for record in records])
    phases = np.array([record["source_time_s"] for record in records])
    if not np.isfinite(phases).all() or np.any(np.diff(ticks) <= 0) or np.any(np.diff(phases) <= 0):
        raise ValueError("Snapshot ticks and phases must be finite and strictly increasing")
    decomposed = np.array([decompose_tick(record) for record in records])
    residual = decomposed[:, 0] - decomposed[:, 1] - decomposed[:, 2]
    identity_error = float(np.max(np.abs(residual)))
    if identity_error > 1e-12:
        raise ValueError("Error-vector identity violated")
    callback_ages = np.array(
        [record["observation_monotonic_s"] - record["feedback"]["receive_monotonic_s"] for record in records]
    )
    publication_ages = np.array(
        [record["observation_monotonic_s"] - record["last_publication"]["publish_monotonic_s"] for record in records]
    )
    if (
        not np.isfinite(np.r_[callback_ages, publication_ages]).all()
        or min(callback_ages.min(), publication_ages.min()) < 0
    ):
        raise ValueError("Invalid callback or publication chronology")
    stamp_ages = [
        record["observation_ros_time_s"] - record["feedback"]["sample_ros_time_s"]
        for record in records
        if record["feedback"]["sample_ros_time_s"] > 0
    ]
    result = {
        "status": "analyzed",
        "execution_status": report["status"],
        "partial": report["status"] != "passed",
        "snapshots": len(records),
        "kinematics_backend": report.get("kinematics_backend", "unknown"),
        "max_vector_identity_error_m": identity_error,
        "callback_age_s_p50_p95_max": np.quantile(callback_ages, [0.5, 0.95, 1]).tolist(),
        "last_publication_age_s_p50_p95_max": np.quantile(publication_ages, [0.5, 0.95, 1]).tolist(),
        "message_stamp_age_s_p50_p95_max": np.quantile(stamp_ages, [0.5, 0.95, 1]).tolist() if stamp_ages else None,
        "per_side": {},
        "contract": "Position-vector decomposition at one controller observation; following is relative to last published command, not acknowledged actuator command. Not causal attribution or physical metrology.",
        "inputs_sha256": {"control_ticks.jsonl": sha256(source), "report.json": sha256(directory / "report.json")},
    }
    norms = np.linalg.norm(decomposed, axis=3)
    figure, axes = pyplot.subplots(1, 2, figsize=(14, 5))
    for side, label in enumerate(("left", "right")):
        result["per_side"][label] = {}
        for index, kind in enumerate(("total", "task", "following")):
            values = norms[:, index, side]
            result["per_side"][label][kind + "_position_m_p50_p95_max"] = np.quantile(values, [0.5, 0.95, 1]).tolist()
            axes[side].plot(phases, values * 1000, label=kind)
        axes[side].set(xlabel="source phase / s", ylabel="position residual / mm", title=label)
        axes[side].legend()
        axes[side].grid(alpha=0.2)
    figure.suptitle(
        f"{result['kinematics_backend']} / {report['status']}: synchronous vector decomposition, not causal shares"
    )
    figure.tight_layout()
    figure.savefig(directory / "error-decomposition.png", dpi=150)
    pyplot.close(figure)
    (directory / "tick-metrics.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    return result
