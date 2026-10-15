import csv
import json
from pathlib import Path

import numpy as np


def analyze_realtime(directory):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as pyplot

    directory = Path(directory)
    report = json.loads((directory / "report.json").read_text())
    if report.get("status") != "passed" or report.get("playback_mode") != "realtime":
        raise ValueError("A passed real-time execution report is required")
    with (directory / "realtime.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    with (directory / "command_joints.csv").open() as stream:
        commands = list(csv.DictReader(stream))
    replay = report["replay"]
    names = replay["active_joints"]
    positions = np.array([[float(row[name + "_position"]) for name in names] for row in commands])
    velocities = np.array([[float(row[name + "_velocity"]) for name in names] for row in commands])
    if not rows or not commands or not np.isfinite(positions).all() or not np.isfinite(velocities).all():
        raise ValueError("Finite nonempty command and tracking logs required")
    finite_fields = [
        "source_time_s",
        "solve_ms",
        "cycle_ms",
        "position_error_left_m",
        "position_error_right_m",
        "rotation_error_left_rad",
        "rotation_error_right_rad",
        "roll_rad",
    ]
    if not np.isfinite([[float(row[field]) for field in finite_fields] for row in rows]).all():
        raise ValueError("Nonfinite tracking sample")
    for field in (
        "planned_velocity_limits_rad_s",
        "planned_acceleration_limits_rad_s2",
        "joint_lower_limits_rad",
        "joint_upper_limits_rad",
        "coupled_matrix",
        "coupled_ceiling",
    ):
        if not np.isfinite(np.asarray(replay[field], dtype=float)).all():
            raise ValueError(f"Nonfinite constraint metadata: {field}")
    if len(commands) != replay["commands"]:
        raise ValueError("Command log length differs from the execution report")
    acceleration = np.diff(np.vstack([np.zeros(len(names)), velocities]), axis=0) * 30
    velocity_excess = float(np.max(np.abs(velocities) - replay["planned_velocity_limits_rad_s"]))
    acceleration_excess = float(np.max(np.abs(acceleration) - replay["planned_acceleration_limits_rad_s2"]))
    position_excess = float(
        max(
            np.max(positions - replay["joint_upper_limits_rad"]),
            np.max(np.array(replay["joint_lower_limits_rad"]) - positions),
        )
    )
    coupling_excess = float(np.max(positions @ np.array(replay["coupled_matrix"]).T - replay["coupled_ceiling"]))
    if max(velocity_excess, position_excess, coupling_excess) > 1e-3 or (
        replay["deadline_misses"] == 0 and acceleration_excess > 1e-3
    ):
        raise ValueError("Recorded commands violate their planned discrete constraints")
    phases = np.array([float(row["source_time_s"]) for row in rows])
    metrics = {
        "status": "passed" if replay["deadline_misses"] == 0 else "passed_with_incomplete_acceleration_audit",
        "report_status": report["status"],
        "source_seconds": replay["source_duration_seconds"],
        "active_wall_seconds": replay["active_wall_seconds"],
        "wall_minus_source_seconds": replay["active_wall_seconds"] - replay["source_duration_seconds"],
        "commands": len(commands),
        "deadline_misses": replay["deadline_misses"],
        "skipped_source_ticks": replay["skipped_source_ticks"],
        "planned_velocity_excess_rad_s": max(0.0, velocity_excess),
        "planned_acceleration_excess_rad_s2": max(0.0, acceleration_excess),
        "joint_position_excess_rad": max(0.0, position_excess),
        "coupling_excess": max(0.0, coupling_excess),
        "constraint_audit": "nominal 1/30 s command integration; not a certification of physical acceleration or wall-clock jitter",
        "acceleration_audit_complete": replay["deadline_misses"] == 0,
        "per_side": {},
        "max_abs_roll_deg": float(np.max(np.abs([float(row["roll_rad"]) for row in rows])) * 180 / np.pi),
    }
    figure, axes = pyplot.subplots(2, 2, figsize=(14, 9))
    for field, label in (("solve_ms", "online IK"), ("cycle_ms", "IK + guards + publish")):
        axes[0, 0].plot(phases, [float(row[field]) for row in rows], label=label)
    axes[0, 0].axhline(1000 / 30, color="red", linestyle="--", label="33.3 ms budget")
    axes[0, 0].set(ylabel="elapsed (ms)", title="Actual per-tick calculation cost")
    for side in ("left", "right"):
        errors = np.array([float(row[f"position_error_{side}_m"]) for row in rows])
        rotation = np.array([float(row[f"rotation_error_{side}_rad"]) for row in rows])
        axes[0, 1].plot(phases, errors * 1000, label=side)
        metrics["per_side"][side] = {
            "max_position_error_m": float(errors.max()),
            "p95_position_error_m": float(np.quantile(errors, 0.95)),
            "max_rotation_error_deg": float(np.degrees(rotation.max())),
        }
        if "actual_opening_left" in rows[0]:
            axes[1, 0].plot(phases, [float(row[f"actual_opening_{side}"]) for row in rows], label=side + " measured")
            axes[1, 0].plot(
                phases, [float(row[f"target_opening_{side}"]) for row in rows], "--", label=side + " source"
            )
    axes[0, 1].set(ylabel="EEF error (mm)", title="Measured PhysX gripper EEF vs current source phase")
    axes[1, 0].set(ylabel="normalized opening", title="0 closed / 1 open")
    axes[1, 1].plot(phases, np.degrees([float(row["roll_rad"]) for row in rows]), label="measured roll")
    axes[1, 1].set(ylabel="waist roll (degrees)", title="Soft upright preference; base fixed")
    for axis in axes.flat:
        axis.set_xlabel("source phase (s), running at wall-clock 1x")
        axis.legend(fontsize=8)
        axis.grid(alpha=0.2)
    figure.suptitle(
        f"Source {metrics['source_seconds']:.3f}s / active wall {metrics['active_wall_seconds']:.3f}s / {len(commands)} online commands"
    )
    figure.tight_layout()
    figure.savefig(directory / "realtime-summary.png", dpi=150)
    pyplot.close(figure)
    (directory / "realtime-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    return metrics
