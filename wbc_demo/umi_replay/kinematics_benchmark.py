import csv
import json
import platform
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .differential_ik import URDFKinematics, matrix_error
from .download import sha256
from .gripper_eef import SwiftpickerEEF
from .pinocchio_backend import PinocchioKinematics


def benchmark_kinematics(args):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as pyplot

    if args.samples < 2:
        raise ValueError("At least two recorded joint states required")
    report_path = args.reference_run / "report.json"
    trace_path = args.reference_run / "joint_trace.csv"
    reference_report = json.loads(report_path.read_text())
    if reference_report.get("status") != "passed":
        raise ValueError("Use a passed actual simulator run as the reference")
    with trace_path.open() as stream:
        rows = list(csv.DictReader(stream))
    indexes = np.unique(np.linspace(0, len(rows) - 1, min(args.samples, len(rows))).astype(int))
    states = [{name: float(value) for name, value in rows[index].items() if name != "wall_time_s"} for index in indexes]
    names = reference_report["replay"]["active_joints"]
    eef = SwiftpickerEEF(args.urdf, args.package_root)
    if eef.evidence["urdf_sha256"] != reference_report["gripper_eef_model"]["urdf_sha256"]:
        raise ValueError("URDF differs from the model used for the recorded robot run")
    root = np.eye(4)
    root[:3, :3] = Rotation.from_euler("xyz", [0.13, -0.2, 0.31]).as_matrix()
    root[:3, 3] = [0.12, -0.07, 0.8]
    original = URDFKinematics(eef, root, names)
    candidate = PinocchioKinematics(eef, root, names, args.urdf)
    errors, jacobian_errors, original_costs, candidate_costs, eigenvalues = [], [], [], [], []
    for state in states:
        started = time.perf_counter()
        reference_poses, reference_jacobians = original.evaluate(state)
        original_costs.append((time.perf_counter() - started) * 1000)
        started = time.perf_counter()
        poses, jacobians = candidate.evaluate(state)
        candidate_costs.append((time.perf_counter() - started) * 1000)
        for expected, measured, expected_jacobian, measured_jacobian in zip(
            reference_poses, poses, reference_jacobians, jacobians
        ):
            residual = matrix_error(expected, measured)
            errors.append([float(np.linalg.norm(residual[:3])), float(np.linalg.norm(residual[3:]))])
            jacobian_errors.append(float(np.max(np.abs(expected_jacobian - measured_jacobian))))
        eigenvalues.append(float(np.linalg.eigvalsh(candidate.mass_matrix(state)).min()))
    errors = np.array(errors)
    passed = (
        errors[:, 0].max() < 1e-8 and errors[:, 1].max() < 1e-8 and max(jacobian_errors) < 2e-6 and min(eigenvalues) > 0
    )
    result = {
        "status": "passed" if passed else "failed",
        "kind": "offline kinematic model compatibility; no robot command published",
        "reference_run": str(args.reference_run),
        "sampled_actual_joint_states": len(states),
        "command_dofs": len(names),
        "pinocchio_version": candidate.pin.__version__,
        "python_version": platform.python_version(),
        "pinocchio_nq": candidate.model.nq,
        "pinocchio_nv": candidate.model.nv,
        "mimic_policy": "independent URDF joints retain measured follower deflections; command velocities mapped through URDF mimic multipliers",
        "synthetic_base_link_to_odom_for_frame_convention_test": root.tolist(),
        "max_eef_position_difference_m": float(errors[:, 0].max()),
        "max_eef_rotation_difference_rad": float(errors[:, 1].max()),
        "max_jacobian_element_difference": max(jacobian_errors),
        "min_reduced_mass_matrix_eigenvalue": min(eigenvalues),
        "reference_fk_jacobian_ms_p50_p95_max": np.quantile(original_costs, [0.5, 0.95, 1]).tolist(),
        "pinocchio_fk_jacobian_ms_p50_p95_max": np.quantile(candidate_costs, [0.5, 0.95, 1]).tolist(),
        "limits": "Model check only: not an IK convergence, collision, torque tracking, or hard realtime validation",
        "inputs_sha256": {str(path): sha256(path) for path in [report_path, trace_path, args.urdf]},
    }
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "metrics.json").write_text(json.dumps(result, indent=2) + "\n")
    figure, axes = pyplot.subplots(1, 2, figsize=(12, 4))
    axes[0].plot(original_costs, label="reference Python FK/J")
    axes[0].plot(candidate_costs, label="Pinocchio FK/J")
    axes[0].set(
        xlabel="recorded state sample", ylabel="milliseconds", title="Same actual robot states and EEF definitions"
    )
    axes[0].legend()
    axes[1].semilogy(np.maximum(jacobian_errors, 1e-17), label="max Jacobian element difference")
    axes[1].axhline(2e-6, color="red", linestyle="--", label="acceptance bound")
    axes[1].set(xlabel="hand/state comparison", title="Includes dynamic TCP and measured mimic deflections")
    axes[1].legend()
    figure.tight_layout()
    figure.savefig(args.output / "kinematics-comparison.png", dpi=150)
    pyplot.close(figure)
    print(json.dumps(result, indent=2), flush=True)
    if not passed:
        raise RuntimeError("Pinocchio model compatibility benchmark did not pass")
    return result
