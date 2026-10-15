import argparse
import csv
import json
from pathlib import Path

import numpy as np

from .download import sha256
from .gripper_eef import SwiftpickerEEF, binary_stl_vertices


def review(old_run, new_run, package, output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as pyplot

    old_report = json.loads((old_run / "report.json").read_text())
    new_report = json.loads((new_run / "report.json").read_text())
    if old_report["provenance"] != new_report["provenance"]:
        raise ValueError("Review must compare the same source episode")
    np.testing.assert_allclose(
        old_report["resolved_alignment"]["world_to_odom"], new_report["resolved_alignment"]["world_to_odom"], atol=1e-12
    )
    output.mkdir(parents=True, exist_ok=False)
    model = SwiftpickerEEF(package / "urdf/genie_g2_crsB_swiftpicker.urdf", package)
    if model.evidence != new_report["gripper_eef_model"]:
        raise ValueError("Current model does not match the new measured run")
    (output / "model-audit.json").write_text(json.dumps(model.evidence, indent=2) + "\n")
    figure, axes = pyplot.subplots(1, 2, figsize=(12, 5))
    old_tool = np.array(old_report["resolved_alignment"]["tip_to_eef"][0])
    for axis, tool, title in zip(
        axes, (old_tool, model.tip_to_eef[0]), ("Previous EEF: camera-side sign wrong", "Corrected EEF: +Z camera side")
    ):
        for name, resource, color in (
            ("camera housing", "camera_link", "tab:green"),
            ("left finger", "left_support_link", "tab:orange"),
            ("right finger", "right_support_link", "tab:blue"),
        ):
            link_name = "gripper_l_camera_link" if resource == "camera_link" else f"gripper_l_{resource}"
            vertices = binary_stl_vertices(package / f"robots/genie/g2/meshes/gripper/swiftpicker/{resource}.STL")
            placement = np.linalg.inv(tool) @ model.relative_zero("arm_l_end_link", link_name)
            values = vertices @ placement[:3, :3].T + placement[:3, 3]
            values = values[:: max(1, len(values) // 2500)] * 1000
            axis.scatter(values[:, 0], values[:, 2], s=1, alpha=0.25, color=color, label=name)
        flange = np.linalg.inv(tool)[:3, 3] * 1000
        axis.scatter(flange[0], flange[2], color="red", marker="x", s=60, label="flange origin")
        axis.scatter(0, 0, color="black", marker="+", s=80, label="EEF fingertip midpoint")
        axis.axhline(0, color="gray", linewidth=0.5)
        axis.set(xlabel="EEF X toward tips (mm)", ylabel="EEF Z (mm)", title=title, ylim=(-130, 130))
        axis.set_aspect("equal")
        axis.grid(alpha=0.2)
    handles, labels = axes[1].get_legend_handles_labels()
    figure.legend(handles, labels, loc="lower center", ncol=3, fontsize=8)
    figure.tight_layout(rect=(0, 0.15, 1, 1))
    figure.savefig(output / "camera-axis-before-after.png", dpi=160)
    pyplot.close(figure)
    figure, axes = pyplot.subplots(2, 1, figsize=(12, 7), sharex=True)
    result = {"same_source_and_world_alignment": True, "new_axis_revision": model.evidence["axis_revision"], "runs": {}}
    for directory, label, color in ((old_run, "previous", "tab:orange"), (new_run, "corrected", "tab:blue")):
        plan = json.loads((directory / "joint_plan.json").read_text())
        roll = np.degrees(np.array(plan["positions"])[:, plan["joint_names"].index("idx04_body_joint4")])
        axes[0].plot(plan["source_times"], roll, color=color, label=label + " IK plan")
        with (directory / "tracking.csv").open() as stream:
            rows = [row for row in csv.DictReader(stream) if row["side"] == "left"]
        actual = np.degrees([float(row["idx04_body_joint4"]) for row in rows])
        axes[1].plot(
            [float(row["source_time_s"]) for row in rows], actual, color=color, label=label + " physical feedback"
        )
        result["runs"][label] = {
            "directory": str(directory),
            "report_sha256": sha256(directory / "report.json"),
            "max_absolute_planned_roll_deg": float(abs(roll).max()),
            "max_absolute_measured_roll_deg": float(abs(actual).max()),
            "planned_roll_min_max_deg": [float(roll.min()), float(roll.max())],
            "measured_roll_min_max_deg": [float(actual.min()), float(actual.max())],
        }
    for axis in axes:
        axis.axhspan(-0.5, 0.5, color="tab:green", alpha=0.15, label="corrected IK bound +/-0.5 deg")
        axis.set(ylabel="waist roll (deg)")
        axis.grid(alpha=0.2)
        axis.legend(fontsize=8)
    axes[0].set_title("Same source EEF targets: old model axes/prior vs corrected camera-side axes/upright prior")
    axes[1].set_xlabel("commanded source phase (s)")
    figure.tight_layout()
    figure.savefig(output / "waist-roll-before-after.png", dpi=160)
    pyplot.close(figure)
    result["causal_limit"] = (
        "Tool-axis convention and posture preference both changed; this comparison is not a single-variable weight ablation."
    )
    (output / "review.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description="Audit waist roll and the wrist-camera side of the gripper EEF")
    parser.add_argument("--old-run", type=Path, required=True)
    parser.add_argument("--new-run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--package",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "source/geniesim_ros/src/ros_ws/src/genie_sim_robot_model",
    )
    args = parser.parse_args()
    print(json.dumps(review(args.old_run, args.new_run, args.package, args.output), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
