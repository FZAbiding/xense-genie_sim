import argparse
import json
import time
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import least_squares

from .differential_ik import matrix_error
from .download import sha256
from .geometry import pose_matrix
from .gripper_eef import SwiftpickerEEF
from .kinematics import create_kinematics


def check_candidate(positions, reference_root, package):
    import rclpy
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetStateValidity
    from rclpy.node import Node
    from sensor_msgs.msg import JointState, MultiDOFJointState
    from tf2_ros import Buffer, TransformException, TransformListener

    from dual_arm_ik import GROUP

    constraints = yaml.safe_load(
        (package.parent / "genie_sim_moveit_plugins/config/coupled_constraints.yaml").read_text()
    )["coupled_constraints"]
    for constraint in constraints:
        values = [positions[name] for name in constraint["joint_names"]]
        if np.max(np.array(constraint["A"]) @ values - constraint["b"]) > 1e-4:
            raise ValueError("Arm-only candidate violates mechanical coupling")
    rclpy.init()
    node = Node("umi_readonly_state_check")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    client = node.create_client(GetStateValidity, "/check_state_validity")
    try:
        end = time.monotonic() + 15
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                base = buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time())
                root = buffer.lookup_transform("odom", "base_link", rclpy.time.Time()).transform
                break
            except TransformException:
                if time.monotonic() > end:
                    raise TimeoutError("Read-only probe could not receive the robot root TF")
        current_root = pose_matrix(
            {
                "position": [root.translation.x, root.translation.y, root.translation.z],
                "quaternion_xyzw": [root.rotation.x, root.rotation.y, root.rotation.z, root.rotation.w],
            }
        )
        discrepancy = matrix_error(current_root, reference_root)
        if max(np.linalg.norm(discrepancy[:3]), np.linalg.norm(discrepancy[3:])) > 0.001:
            raise ValueError("Live base differs from the reference; candidate FK cannot be reused")
        candidate = RobotState(joint_state=JointState(name=list(positions), position=list(positions.values())))
        candidate.multi_dof_joint_state = MultiDOFJointState(
            header=base.header, joint_names=["planar_joint"], transforms=[base.transform]
        )
        if not client.wait_for_service(timeout_sec=10):
            raise TimeoutError("MoveIt state-validity service is unavailable")
        publishers = node.count_publishers("/joint_command")
        future = client.call_async(GetStateValidity.Request(robot_state=candidate, group_name=GROUP))
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or future.result() is None:
            future.cancel()
            raise TimeoutError("MoveIt state-validity request timed out")
        return {
            "coupled_constraints_valid": True,
            "moveit_state_valid": bool(future.result().valid),
            "joint_command_publishers_during_readonly_probe": publishers,
        }
    finally:
        listener.unregister()
        node.destroy_node()
        rclpy.shutdown()


def probe(reference, output, backend="urdf", validate_moveit=False):
    report = json.loads((reference / "report.json").read_text())
    frame = json.loads((reference / "mapped_targets.json").read_text())[0]
    package = Path(__file__).resolve().parents[2] / "source/geniesim_ros/src/ros_ws/src/genie_sim_robot_model"
    urdf = package / "urdf/genie_g2_crsB_swiftpicker.urdf"
    if sha256(urdf) != report["gripper_eef_model"]["urdf_sha256"]:
        raise ValueError("Reference robot model differs")
    eef = SwiftpickerEEF(urdf, package)
    names = report["replay"]["active_joints"]
    model = create_kinematics(backend, eef, np.array(report["replay"]["model_root_to_odom"]), names)
    arms = [name for name in names if "_arm_l_joint" in name or "_arm_r_joint" in name]
    waist = [name for name in names if "_body_joint" in name]
    positions = dict(report["initial_measured_positions_rad"])
    positions.update(report["initial_approach_goal_rad"])
    positions.update({name: 0.0 for name in waist})
    positions.update(eef.opening_positions(frame["gripper"]))
    lower = [float(eef.joints[name].find("limit").get("lower")) for name in arms]
    upper = [float(eef.joints[name].find("limit").get("upper")) for name in arms]
    targets = [pose_matrix(target) for target in frame["eef_odom"]]

    def residual(values):
        state = dict(positions)
        state.update(zip(arms, values))
        actual, unused = model.evaluate(state, jacobian=False)
        return np.concatenate([matrix_error(pose, target) for pose, target in zip(actual, targets)])

    solution = least_squares(
        residual,
        np.clip([positions[name] for name in arms], lower, upper),
        bounds=(lower, upper),
        max_nfev=2000,
        ftol=1e-12,
        xtol=1e-12,
        gtol=1e-12,
    )
    positions.update(zip(arms, solution.x))
    errors = residual(solution.x).reshape(2, 6)
    result = {
        "scope": "read-only first-frame candidate; no trajectory or robot commands; local failure is not proof of unreachability",
        "backend": backend,
        "reference_sha256": {name: sha256(reference / name) for name in ("report.json", "mapped_targets.json")},
        "waist_fixed_rad": {name: positions[name] for name in waist},
        "position_errors_m": np.linalg.norm(errors[:, :3], axis=1).tolist(),
        "rotation_errors_rad": np.linalg.norm(errors[:, 3:], axis=1).tolist(),
        "optimizer_success": bool(solution.success),
        "optimizer_message": solution.message,
        "joint_positions_rad": positions,
        "moveit_state_valid": None,
        "approach_path_validated": False,
        "full_episode_validated": False,
    }
    if validate_moveit:
        result.update(check_candidate(positions, model.root, package))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({key: value for key, value in result.items() if key != "joint_positions_rad"}))


def main():
    parser = argparse.ArgumentParser(
        description="Read-only arm-only first-frame feasibility probe, not a replay controller"
    )
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--backend", choices=("urdf", "pinocchio"), default="urdf")
    parser.add_argument("--validate-moveit", action="store_true")
    args = parser.parse_args()
    probe(args.reference, args.output, args.backend, args.validate_moveit)


if __name__ == "__main__":
    main()
