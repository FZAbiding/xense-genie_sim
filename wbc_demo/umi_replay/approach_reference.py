import copy
import math
import time

from .geometry import pose_error


def validate_reference(reference, input_sha256, config):
    if reference.get("status") != "passed" or reference.get("input_sha256") != input_sha256:
        raise ValueError("Approach reference must be a passed run of the exact same input")
    if reference.get("config") != config:
        raise ValueError("Approach reference world/safety/posture config differs")
    positions = reference.get("initial_approach_goal_rad", {})
    if not positions or not all(
        isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)
        for value in positions.values()
    ):
        raise ValueError("Finite recorded initial joint goal required")
    return reference


def prepare_reference(node, frame, reference):
    from dual_arm_ik import GROUP
    from lab_core import pose_values
    from .execution import ros_targets

    started = time.monotonic()
    if reference["gripper_eef_model"]["urdf_sha256"] != node.eef.evidence["urdf_sha256"]:
        raise ValueError("Approach reference URDF differs")
    start = copy.deepcopy(node.home)
    goal = copy.deepcopy(start)
    values = reference["initial_approach_goal_rad"]
    if set(values) != set(goal.joint_state.name):
        raise ValueError("Approach reference joint names differ")
    goal.joint_state.position = [values[name] for name in goal.joint_state.name]
    node.validate_constraints(goal, GROUP)
    node.check_gripper_reference(goal)
    predicted = pose_values(node.forward(goal))
    errors = [pose_error(actual, target) for actual, target in zip(predicted, frame["eef_odom"])]
    if (
        len(errors) != 2
        or max(error[0] for error in errors) > 0.005
        or max(error[1] for error in errors) > math.radians(1)
    ):
        raise ValueError("Frozen joint goal no longer reaches the requested initial EEFs")
    node.valid_state(goal, GROUP)
    path = [start, goal]
    samples = node.validate_path(path, GROUP)
    return {
        "start": start,
        "goal": goal,
        "targets": ros_targets(frame["eef_odom"]),
        "group": GROUP,
        "path": path,
        "planning_method": "checked_frozen_reference_joint_goal",
        "collision_samples": samples,
        "ik_error_m": [error[0] for error in errors],
        "ik_orientation_error_rad": [error[1] for error in errors],
        "solve_seconds": time.monotonic() - started,
        "attempts": 0,
    }
