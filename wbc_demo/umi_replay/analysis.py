import csv
import json
from pathlib import Path

import numpy as np

from .geometry import interpolate_pose, pose_dict, pose_error, pose_matrix


def source_at(frames, source_time, side):
    times = np.array([frame["timestamp"] for frame in frames])
    end = int(np.clip(np.searchsorted(times, source_time, side="right"), 1, len(times) - 1))
    fraction = float(np.clip((source_time - times[end - 1]) / (times[end] - times[end - 1]), 0, 1))
    return interpolate_pose(frames[end - 1]["eef"][side], frames[end]["eef"][side], fraction)


def analyze(run_directory, converted):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as pyplot

    output = Path(run_directory)
    report = json.loads((output / "report.json").read_text())
    if converted["provenance"] != report["provenance"]:
        raise ValueError("Converted source does not match this run's recorded provenance")
    if report.get("playback_mode") == "realtime":
        from .realtime_analysis import analyze_realtime

        return analyze_realtime(output)
    world = np.array(report["resolved_alignment"]["world_to_odom"])
    offsets = [np.array(item) for item in report["resolved_alignment"]["tip_to_eef"]]
    gripper_eef_tracking = report.get("tracking_pose_frame") == "gripper_eef"
    reference_label = "model gripper EEF" if gripper_eef_tracking else "historical virtual EEF"
    with (output / "tracking.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("No measured tracking samples")
    figure = pyplot.figure(figsize=(15, 9))
    spatial = figure.add_subplot(221, projection="3d")
    position_axis = figure.add_subplot(222)
    rotation_axis = figure.add_subplot(223)
    body_axis = figure.add_subplot(224)
    metrics = {
        "comparison": f"actual {reference_label} vs full 30Hz original EEF interpolated at commanded source phase",
        "sides": {},
    }
    errors_path = output / "eef_tracking.csv"
    with errors_path.open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "wall_time_s",
                "feedback_ros_time_s",
                "source_time_s",
                "side",
                "eef_position_error_m",
                "eef_rotation_error_rad",
            ]
        )
        for side, name, color in zip(range(2), ("left", "right"), ("tab:blue", "tab:orange")):
            selected = [row for row in rows if row["side"] == name]
            source_times = [float(row["source_time_s"]) for row in selected]
            wanted_positions = []
            actual_positions = []
            errors = []
            for row, source_time in zip(selected, source_times):
                tip = {
                    "position": [float(row[f"actual_{axis}"]) for axis in ("x", "y", "z")],
                    "quaternion_xyzw": [float(row[f"actual_{axis}"]) for axis in ("qx", "qy", "qz", "qw")],
                }
                actual = tip if gripper_eef_tracking else pose_dict(pose_matrix(tip) @ offsets[side])
                wanted = pose_dict(world @ pose_matrix(source_at(converted["source_frames"], source_time, side)))
                error = pose_error(wanted, actual)
                errors.append(error)
                wanted_positions.append(wanted["position"])
                actual_positions.append(actual["position"])
                writer.writerow([row["wall_time_s"], row["feedback_ros_time_s"], source_time, name, *error])
            errors = np.array(errors)
            metrics["sides"][name] = {
                "samples": len(selected),
                "max_position_error_m": float(errors[:, 0].max()),
                "p95_position_error_m": float(np.percentile(errors[:, 0], 95)),
                "max_rotation_error_rad": float(errors[:, 1].max()),
                "source_time_start": min(source_times),
                "source_time_end": max(source_times),
            }
            spatial.plot(*np.array(wanted_positions).T, color=color, label=name + " source")
            spatial.plot(
                *np.array(actual_positions).T, color=color, linestyle="--", label=name + " measured " + reference_label
            )
            position_axis.plot(source_times, errors[:, 0] * 1000, color=color, label=name)
            rotation_axis.plot(source_times, np.degrees(errors[:, 1]), color=color, label=name)
    left_rows = [row for row in rows if row["side"] == "left"]
    body_names = [key for key in left_rows[0] if "_body_joint" in key]
    metrics["body_joint_range_rad"] = {}
    for name in body_names:
        values = np.array([float(row[name]) for row in left_rows])
        metrics["body_joint_range_rad"][name] = float(np.ptp(values))
        body_axis.plot([float(row["source_time_s"]) for row in left_rows], np.degrees(values), label=name)
    spatial.set(
        xlabel="odom x (m)",
        ylabel="odom y (m)",
        zlabel="odom z (m)",
        title=f"Tracking reference: {reference_label}",
    )
    position_axis.set(
        xlabel="commanded source phase (s)",
        ylabel="EEF position error (mm)",
        title="Includes source sampling + joint interpolation + measured tracking",
    )
    rotation_axis.set(xlabel="commanded source phase (s)", ylabel="EEF orientation error (deg)")
    body_axis.set(xlabel="commanded source phase (s)", ylabel="measured body joints (deg)")
    for axis in (position_axis, rotation_axis, body_axis):
        axis.legend(fontsize=7)
    handles, labels = spatial.get_legend_handles_labels()
    figure.legend(handles, labels, loc="lower center", bbox_to_anchor=(0.5, 0.01), ncol=2, fontsize=7)
    figure.tight_layout(rect=(0, 0.07, 1, 1))
    figure.savefig(output / "tracking-summary.png", dpi=160)
    pyplot.close(figure)
    gripper_path = output / "gripper_tracking.csv"
    if gripper_path.exists():
        with gripper_path.open() as stream:
            gripper_rows = list(csv.DictReader(stream))
        figure, axes = pyplot.subplots(2, 1, figsize=(12, 7), sharex=True)
        source_times = np.array([frame["timestamp"] for frame in converted["source_frames"]])
        source_openings = np.array([frame["gripper"] for frame in converted["source_frames"]])
        metrics["grippers"] = {}
        for side, name, axis in zip(range(2), ("left", "right"), axes):
            selected = [row for row in gripper_rows if row["side"] == name]
            phases = np.array([float(row["source_time_s"]) for row in selected])
            measured = np.array([float(row["actual_opening"]) for row in selected])
            commanded = np.array([float(row["target_opening"]) for row in selected])
            wanted = np.interp(phases, source_times, source_openings[:, side])
            errors = np.abs(measured - wanted)
            metrics["grippers"][name] = {
                "samples": len(selected),
                "max_error_vs_original_normalized": float(errors.max()),
                "p95_error_vs_original_normalized": float(np.percentile(errors, 95)),
                "measured_opening_min_max": [float(measured.min()), float(measured.max())],
                "max_command_tracking_error": float(np.abs(measured - commanded).max()),
            }
            axis.plot(source_times, source_openings[:, side], label="original 30Hz opening", color="black", alpha=0.5)
            axis.plot(phases, commanded, label="command (resampled)", linestyle="--")
            axis.plot(phases, measured, label="measured master joint -> opening")
            axis.set(ylabel=name + " opening (0 closed / 1 open)", ylim=(-0.03, 1.03))
            axis.legend(fontsize=8)
            axis.grid(alpha=0.2)
        axes[-1].set_xlabel("commanded source phase (s)")
        figure.tight_layout()
        figure.savefig(output / "gripper-summary.png", dpi=160)
        pyplot.close(figure)
    (output / "eef-metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    print(f"Measured EEF analysis: {output / 'eef-metrics.json'}", flush=True)
    return metrics
