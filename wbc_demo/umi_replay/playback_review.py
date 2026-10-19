import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .download import sha256
from .gripper_eef import SwiftpickerEEF


def stream_summary(value):
    statuses = value["statuses"]
    first, last = statuses[0], statuses[-1]
    seconds = last["monotonic_s"] - first["monotonic_s"]
    return {
        "received_fps": {
            name: (last["state"]["received"][name] - first["state"]["received"][name]) / seconds
            for name in first["state"]["received"]
        },
        "render_bundle_age_ms_p95": float(
            np.percentile([row["state"]["render_bundle_age_s"] * 1000 for row in statuses], 95)
        ),
        "streams": [{key: value for key, value in stream.items() if key != "samples"} for stream in value["streams"]],
    }


def browser_summary(value):
    frames = value["presentations"]
    samples = [row for row in value["samples"] if row["state"]["stage"] == "replaying"]
    return {
        "browser": value["browser"],
        "presented_source_fps": (len(frames) - 1) / ((frames[-1]["wall_ms"] - frames[0]["wall_ms"]) / 1000),
        "sampled_source_phase_gap_ms_max": max(
            abs(row["video_time_s"] - row["state"]["source_time_s"] - row["state"]["source_video_start_s"]) * 1000
            for row in samples
        ),
        "events": value["events"],
        "sampled_stages": [row["state"]["stage"] for row in value["samples"]],
        "errors": value["errors"],
    }


def generate(args):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as pyplot

    args.output.mkdir(parents=True, exist_ok=True)
    report_path = args.reference / "report.json"
    commands_path = args.reference / "command_joints.csv"
    report = json.loads(report_path.read_text())
    commands = list(csv.DictReader(commands_path.open()))
    first_frame = json.loads((args.reference / "mapped_targets.json").read_text())[0]
    arm_only = json.loads(args.arm_only.read_text())
    probes = {name: json.loads((args.probes / f"{name}-streams.json").read_text()) for name in ("before", "after")}
    summary = {
        "scope": "local diagnostic windows, not a full-episode acceptance or a causal A/B of controller backends",
        "streams": {name: stream_summary(value) for name, value in probes.items()},
        "browsers": {
            name: browser_summary(json.loads((args.probes / f"{name}-browser/browser-probe.json").read_text()))
            for name in ("before", "after")
        },
        "reference_replay": {
            key: report["replay"][key] for key in ("wall_seconds", "pause_seconds", "active_wall_seconds")
        },
        "arm_only": arm_only,
        "inputs_sha256": {str(path): sha256(path) for path in (report_path, commands_path, args.arm_only)},
    }
    figure, axes = pyplot.subplots(1, 2, figsize=(12, 4))
    for name, color in (("before", "#d87937"), ("after", "#158582")):
        for stream in probes[name]["streams"]:
            times = np.array([row["monotonic_s"] for row in stream["samples"]])
            if stream["view"] == "UmiEgo":
                axes[0].plot(times[1:] - times[0], np.diff(times) * 1000, label=name, color=color, alpha=0.8)
        axes[1].bar(
            np.arange(3) + (-0.18 if name == "before" else 0.18),
            [stream["delivered_fps"] for stream in probes[name]["streams"]],
            width=0.36,
            label=name,
            color=color,
        )
    axes[0].axhline(1000 / 30, linestyle="--", color="#777777", label="30 fps period")
    axes[0].set(
        xlabel="Seconds within each idle probe",
        ylabel="MJPEG arrival gap (ms)",
        title="UmiEgo: average rate is not worst-case latency",
    )
    axes[1].axhline(30, linestyle="--", color="#777777")
    axes[1].set(
        xticks=np.arange(3),
        xticklabels=["Ego", "External A", "External B"],
        ylabel="Delivered MJPEG fps",
        title="Same robot pose / same external camera orbits",
    )
    for axis in axes:
        axis.legend()
        axis.grid(alpha=0.2)
    figure.tight_layout()
    figure.savefig(args.output / "delivery.png", dpi=150)
    pyplot.close(figure)
    package = Path(__file__).resolve().parents[2] / "source/geniesim_ros/src/ros_ws/src/genie_sim_robot_model"
    urdf = package / "urdf/genie_g2_crsB_swiftpicker.urdf"
    if sha256(urdf) != report["gripper_eef_model"]["urdf_sha256"]:
        raise ValueError("Cannot draw the reference with a different URDF")
    eef = SwiftpickerEEF(urdf, package)
    root = np.array(report["replay"]["model_root_to_odom"])

    def chain(tip, positions):
        joints = []
        while tip != "base_link":
            joint = eef.parents[tip]
            joints.append(joint)
            tip = joint.find("parent").get("link")
        matrix = root.copy()
        points = [matrix[:3, 3].copy()]
        for joint in reversed(joints):
            matrix = matrix @ eef.origins[joint.get("name")]
            if joint.get("type") in ("revolute", "continuous"):
                axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                matrix[:3, :3] = (
                    matrix[:3, :3] @ Rotation.from_rotvec(axis * eef.joint_value(joint, positions)).as_matrix()
                )
            points.append(matrix[:3, 3].copy())
        return np.array(points)

    figure, axes = pyplot.subplots(1, 3, figsize=(15, 4.5))
    frozen = dict(report["initial_measured_positions_rad"])
    frozen.update(report["initial_approach_goal_rad"])
    for axis, positions, title in zip(
        axes[:2],
        (frozen, arm_only["joint_positions_rad"]),
        ("Existing first-frame joint goal", "Arm-only candidate: waist joints = 0"),
    ):
        for tip, color, target in zip(eef.tip_names, ("#247db5", "#d67433"), first_frame["eef_odom"]):
            points = chain(tip, positions)
            axis.plot(points[:, 0], points[:, 2], "o-", color=color, markersize=3)
            target_point = target["position"]
            axis.plot([points[-1, 0], target_point[0]], [points[-1, 2], target_point[2]], "--", color=color)
            axis.plot(target_point[0], target_point[2], "*", color=color, markersize=9)
        axis.set(xlim=(-0.7, 0.65), ylim=(0, 1.6), xlabel="Odom X (m), front ->", ylabel="Odom Z (m)", title=title)
        axis.set_aspect("equal")
        axis.grid(alpha=0.2)
    times = [float(row["source_time_s"]) for row in commands]
    for index in range(1, 4):
        name = f"idx0{index}_body_joint{index}_position"
        axes[2].plot(times, np.rad2deg([float(row[name]) for row in commands]), label=f"body joint {index}")
    sagittal = np.rad2deg(
        [sum(float(row[f"idx0{index}_body_joint{index}_position"]) for index in range(1, 4)) for row in commands]
    )
    axes[2].plot(times, sagittal, "k--", label="sum of three pitch joints")
    axes[2].set(
        xlabel="Source phase (s)", ylabel="Command angle (degrees)", title="Recorded run-002; not a new waist policy"
    )
    axes[2].legend(fontsize=8)
    axes[2].grid(alpha=0.2)
    summary["waist_pitch_sum_deg"] = {
        "initial_goal": float(np.rad2deg(sum(frozen[f"idx0{index}_body_joint{index}"] for index in range(1, 4)))),
        "replay_min": float(sagittal.min()),
        "replay_max": float(sagittal.max()),
    }
    figure.suptitle(
        "URDF FK skeletons (not simulation screenshots); same EEF targets, different redundant joint solutions"
    )
    figure.tight_layout()
    figure.savefig(args.output / "waist-initial.png", dpi=150)
    pyplot.close(figure)
    (args.output / "metrics.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description="Generate measured playback and first-frame waist review figures")
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--probes", required=True, type=Path)
    parser.add_argument("--arm-only", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    generate(parser.parse_args())


if __name__ == "__main__":
    main()
