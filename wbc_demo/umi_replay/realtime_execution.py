import copy
import csv
import json
import math
import time

import numpy as np
import rclpy
from moveit_msgs.srv import GetStateValidity

from dual_arm_ik import GROUP
from lab_core import pose_values, save_json

from .differential_ik import DifferentialIK, PlaybackClock, matrix_error
from .geometry import interpolate_pose, pose_matrix, pose_dict
from .kinematics import create_kinematics
from .validation import validate_realtime_profile, validate_realtime_source


def state_positions(state):
    return dict(zip(state.joint_state.name, state.joint_state.position))


def with_positions(state, positions):
    result = copy.deepcopy(state)
    result.joint_state.position = [positions[name] for name in result.joint_state.name]
    return result


def prepare_initial(node, frame):
    from .execution import ros_targets

    node.requested_openings = frame["gripper"]
    last_error = None
    for attempt in range(5):
        if node.cancel.is_set():
            raise RuntimeError("Cancelled during initial approach preparation")
        try:
            prepared = node.prepare(ros_targets(frame["eef_odom"]), seed=node.home, plan=False)
            prepared["collision_samples"] = node.validate_path(prepared["path"], GROUP)
            prepared["planning_method"] = "checked_direct_joint_interpolation"
            return prepared
        except RuntimeError as error:
            last_error = error
    raise RuntimeError(f"No safe direct initial approach after five attempts: {last_error}")


def sample_targets(frames, phase):
    times = np.array([frame["timestamp"] for frame in frames])
    index = min(len(frames) - 2, max(0, int(np.searchsorted(times, phase, side="right") - 1)))
    first, second = frames[index : index + 2]
    fraction = float(np.clip((phase - first["timestamp"]) / (second["timestamp"] - first["timestamp"]), 0, 1))
    targets = [interpolate_pose(start, goal, fraction) for start, goal in zip(first["eef_odom"], second["eef_odom"])]
    openings = (1 - fraction) * np.array(first["gripper"]) + fraction * np.array(second["gripper"])
    return targets, openings


def checked_step(node, previous, candidate, deadline):
    for state in (node.interpolate(previous, candidate, 0.5), candidate):
        if time.monotonic() >= deadline:
            raise TimeoutError("Collision request deadline already expired")
        node.validate_constraints(state, GROUP, tolerance=1e-4)
        future = node.validity.call_async(GetStateValidity.Request(robot_state=state, group_name=GROUP))
        rclpy.spin_until_future_complete(node, future, timeout_sec=max(0.0, deadline - time.monotonic()))
        if not future.done():
            future.cancel()
            raise TimeoutError("Collision service missed the current command deadline")
        response = future.result()
        if response is None or not response.valid:
            raise RuntimeError("Online collision/state validity guard rejected the next command")


def prepare_model(node, backend):
    positions = state_positions(node.state)
    names = [name for name in node.state.joint_state.name if node.is_command_joint(name)]
    root = node.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time()).transform
    root_matrix = pose_matrix(
        {
            "position": [root.translation.x, root.translation.y, root.translation.z],
            "quaternion_xyzw": [root.rotation.x, root.rotation.y, root.rotation.z, root.rotation.w],
        }
    )
    model = create_kinematics(backend, node.eef, root_matrix, names)
    local, unused = model.evaluate(positions, jacobian=False)
    service = pose_values(node.forward(node.state))
    discrepancy = [matrix_error(actual, pose_matrix(expected)) for actual, expected in zip(local, service)]
    if (
        max(np.linalg.norm(item[:3]) for item in discrepancy) > 1e-5
        or max(np.linalg.norm(item[3:]) for item in discrepancy) > 1e-5
    ):
        raise RuntimeError(f"Local URDF FK disagrees with MoveIt: {discrepancy}")
    model.crosscheck = [float(np.linalg.norm(item[:3])) for item in discrepancy]
    return model


def stream(node, frames, config, profile, output, report):
    validate_realtime_profile(profile)
    period = 1 / 30
    lookahead = profile.get("lookahead_seconds", 0.0)
    timestamps = validate_realtime_source(frames)
    duration = float(timestamps[-1] + period)
    positions = state_positions(node.state)
    model = node.realtime_model
    names = model.names
    solver = DifferentialIK(
        model, positions, node.limits, node.velocities, node.coupled, profile, node.joint_position_bounds
    )
    velocity = np.zeros(len(names))
    result = {
        "control_hz_requested": 30,
        "source_duration_seconds": duration,
        "time_scale": 1.0,
        "solver": "repository differential Cartesian IK + OSQP; initial approach method recorded separately",
        "limits_profile": profile,
        "active_joints": names,
        "planned_velocity_limits_rad_s": solver.velocity.tolist(),
        "planned_acceleration_limits_rad_s2": solver.acceleration.tolist(),
        "joint_lower_limits_rad": solver.lower.tolist(),
        "joint_upper_limits_rad": solver.upper.tolist(),
        "coupled_matrix": solver.coupled.tolist(),
        "coupled_ceiling": solver.ceiling.tolist(),
        "target_lookahead_s": lookahead,
        "fk_crosscheck_max_position_m": max(model.crosscheck),
        "kinematics_backend": report["kinematics_backend"],
        "model_version": getattr(getattr(model, "pin", None), "__version__", "repository URDF geometry"),
        "model_root_to_odom": model.root.tolist(),
        "tick_snapshot_contract": "same callback state snapshot vs last published command; no actuator acknowledgement or causal latency attribution",
        "commands": 0,
        "deadline_misses": 0,
        "skipped_source_ticks": 0,
        "qp_solved_inaccurate": 0,
        "max_position_error_m": 0.0,
        "max_rotation_error_rad": 0.0,
        "max_gripper_opening_error": 0.0,
        "max_command_velocity_rad_s": 0.0,
        "max_command_acceleration_rad_s2": 0.0,
        "max_command_step_rad": 0.0,
        "clock_policy": "monotonic wall clock at source 1x; pauses excluded; no backlog or global retiming",
        "collision_policy": "MoveIt midpoint and endpoint checks before each command; not continuous swept-volume checking",
        "feedback_policy": "warm-started command-space differential IK; actual PhysX joint feedback monitored for tracking and base drift",
    }
    report["replay"] = result
    clock = PlaybackClock(time.monotonic(), duration)
    last_tick = -1
    consecutive_misses = 0
    costs, solve_costs = [], []
    last_save = 0.0
    commanded = copy.deepcopy(node.state)
    with (output / "realtime.csv").open("w") as trace, (output / "command_joints.csv").open("w") as joint_trace, (
        output / "command_phase.csv"
    ).open("w") as phase_trace, (output / "control_ticks.jsonl").open("w") as snapshots:
        writer = csv.writer(trace)
        joint_writer = csv.writer(joint_trace)
        joint_writer.writerow(
            ["source_time_s", *[name + "_position" for name in names], *[name + "_velocity" for name in names]]
        )
        phase_writer = csv.writer(phase_trace)
        phase_writer.writerow(["wall_time_s", "command_ros_time_s", "source_time_s", "keyframe"])
        last_publication = node.publish_positions(commanded, GROUP)
        phase_writer.writerow([time.time(), node.get_clock().now().nanoseconds * 1e-9, 0.0, 0])
        writer.writerow(
            [
                "wall_time_s",
                "source_time_s",
                "tick",
                "solve_ms",
                "cycle_ms",
                "published",
                "position_error_left_m",
                "position_error_right_m",
                "rotation_error_left_rad",
                "rotation_error_right_rad",
                "gripper_error",
                "feedback_age_s",
                "roll_rad",
                "target_opening_left",
                "target_opening_right",
                "actual_opening_left",
                "actual_opening_right",
                "observation_ms",
                "collision_ms",
                "publication_ms",
            ]
        )
        while rclpy.ok():
            for callback in range(12):
                rclpy.spin_once(node, timeout_sec=0)
            now = time.monotonic()
            clock.pause(now, node.pause_requested)
            phase = clock.phase(now)
            if node.pause_requested or int(phase / period) != last_tick:
                node.publish_progress(
                    min(phase, timestamps[-1]),
                    "paused" if node.pause_requested else "replaying",
                    playback_rate=0.0 if node.pause_requested else 1.0,
                    phase_monotonic_s=now,
                    source_duration_s=duration,
                    playback_mode="realtime",
                    deadline_misses=result["deadline_misses"],
                )
            if node.cancel.is_set():
                raise RuntimeError("Realtime replay cancelled; holding measured position")
            if now - node.last_state_time > 0.2 or node.count_publishers("/joint_command") > 1:
                raise RuntimeError("Stale joint feedback or competing joint-command publisher")
            node.observe_base_drift()
            if node.pause_requested:
                last_publication = node.publish_positions(commanded, GROUP)
                time.sleep(0.01)
                continue
            if phase >= duration:
                phase_writer.writerow(
                    [time.time(), node.get_clock().now().nanoseconds * 1e-9, duration, len(frames) - 1]
                )
                break
            tick = int(phase / period)
            if tick == last_tick:
                time.sleep(min(0.002, max(0.0, (tick + 1) * period - phase)))
                continue
            result["skipped_source_ticks"] += max(0, tick - last_tick - 1)
            last_tick = tick
            targets, openings = sample_targets(frames, min(phase, timestamps[-1]))
            actual_positions = state_positions(node.state)
            feedback_metadata = dict(node.feedback_metadata)
            measured_velocities = dict(zip(node.state.joint_state.name, node.state.joint_state.velocity))
            observation_ros_time = node.get_clock().now().nanoseconds * 1e-9
            observation_monotonic = time.monotonic()
            feedback_age = time.monotonic() - node.last_state_time
            actual, unused = model.evaluate(actual_positions, jacobian=False)
            command_before, unused = model.evaluate(positions, jacobian=False)
            snapshot = {
                "tick": tick,
                "source_time_s": phase,
                "observation_ros_time_s": observation_ros_time,
                "observation_monotonic_s": observation_monotonic,
                "observation_wall_time_s": time.time(),
                "feedback": feedback_metadata,
                "last_publication": dict(last_publication),
                "positions_measured": actual_positions,
                "velocities_measured": measured_velocities,
                "positions_last_command": dict(positions),
                "eef_target": targets,
                "eef_measured": [pose_dict(pose) for pose in actual],
                "eef_last_command": [pose_dict(pose) for pose in command_before],
                "target_gripper_openings": openings.tolist(),
                "model_observation_ms": (time.monotonic() - observation_monotonic) * 1000,
            }
            snapshots.write(json.dumps(snapshot, allow_nan=False, separators=(",", ":")) + "\n")
            errors = [matrix_error(measured, pose_matrix(wanted)) for measured, wanted in zip(actual, targets)]
            position_errors = [float(np.linalg.norm(item[:3])) for item in errors]
            rotation_errors = [float(np.linalg.norm(item[3:])) for item in errors]
            opening_error = float(np.max(np.abs(node.eef.openings_from_positions(actual_positions) - openings)))
            result["max_position_error_m"] = max(result["max_position_error_m"], max(position_errors))
            result["max_rotation_error_rad"] = max(result["max_rotation_error_rad"], max(rotation_errors))
            result["max_gripper_opening_error"] = max(result["max_gripper_opening_error"], opening_error)
            if (
                max(position_errors) > config["max_tracking_error_m"]
                or max(rotation_errors) > config["max_tracking_rotation_rad"]
                or opening_error > 0.1
            ):
                raise RuntimeError(
                    f"Realtime tracking guard: position={position_errors}, rotation={rotation_errors}, gripper={opening_error}"
                )
            beginning = time.monotonic()
            observation_ms = (beginning - now) * 1000
            command_targets, command_openings = sample_targets(frames, min(phase + lookahead, timestamps[-1]))
            candidate_positions, next_velocity = solver.step(
                positions, command_targets, command_openings, velocity, period
            )
            result["qp_solved_inaccurate"] += int(solver.last_status == "solved inaccurate")
            solve_ms = (time.monotonic() - beginning) * 1000
            candidate = with_positions(commanded, candidate_positions)
            published = False
            collision_ms = 0.0
            publication_ms = 0.0
            try:
                collision_begin = time.monotonic()
                try:
                    checked_step(node, commanded, candidate, now + period)
                finally:
                    collision_ms = (time.monotonic() - collision_begin) * 1000
                if time.monotonic() >= now + period:
                    raise TimeoutError("Command calculation exceeded the current 33.3 ms deadline")
                if node.cancel.is_set() or node.pause_requested:
                    continue
                publication_begin = time.monotonic()
                last_publication = node.publish_positions(candidate, GROUP)
                publication_ms = (time.monotonic() - publication_begin) * 1000
                phase_writer.writerow([time.time(), node.get_clock().now().nanoseconds * 1e-9, phase, tick])
                result["max_command_velocity_rad_s"] = max(
                    result["max_command_velocity_rad_s"], float(np.max(np.abs(next_velocity)))
                )
                result["max_command_acceleration_rad_s2"] = max(
                    result["max_command_acceleration_rad_s2"], float(np.max(np.abs(next_velocity - velocity)) / period)
                )
                result["max_command_step_rad"] = max(
                    result["max_command_step_rad"],
                    max(abs(candidate_positions[name] - positions[name]) for name in names),
                )
                commanded, positions, velocity = candidate, candidate_positions, next_velocity
                joint_writer.writerow([phase, *[positions[name] for name in names], *velocity])
                result["commands"] += 1
                consecutive_misses = 0
                published = True
            except TimeoutError:
                result["deadline_misses"] += 1
                consecutive_misses += 1
                velocity = np.zeros(len(names))
                publication_begin = time.monotonic()
                last_publication = node.publish_positions(commanded, GROUP)
                publication_ms = (time.monotonic() - publication_begin) * 1000
            cycle_ms = (time.monotonic() - now) * 1000
            costs.append(cycle_ms)
            solve_costs.append(solve_ms)
            writer.writerow(
                [
                    time.time(),
                    phase,
                    tick,
                    solve_ms,
                    cycle_ms,
                    published,
                    *position_errors,
                    *rotation_errors,
                    opening_error,
                    feedback_age,
                    actual_positions["idx04_body_joint4"],
                    *openings,
                    *node.eef.openings_from_positions(actual_positions),
                    observation_ms,
                    collision_ms,
                    publication_ms,
                ]
            )
            if time.monotonic() - last_save >= 1:
                trace.flush()
                joint_trace.flush()
                phase_trace.flush()
                save_json(output / "report.json", report)
                last_save = time.monotonic()
            if consecutive_misses >= 3:
                raise RuntimeError("Three consecutive realtime command deadlines missed; refusing stale targets")
    final_poses, unused = model.evaluate(state_positions(node.state), jacobian=False)
    final_errors = [
        matrix_error(actual, pose_matrix(target)) for actual, target in zip(final_poses, frames[-1]["eef_odom"])
    ]
    result["final_position_errors_m"] = [float(np.linalg.norm(item[:3])) for item in final_errors]
    result["final_rotation_errors_rad"] = [float(np.linalg.norm(item[3:])) for item in final_errors]
    if max(result["final_position_errors_m"]) > 0.02 or max(result["final_rotation_errors_rad"]) > math.radians(3):
        raise RuntimeError("Realtime final tracking error exceeds 2 cm / 3 degrees")
    result.update(
        {
            "wall_seconds": time.monotonic() - clock.start,
            "pause_seconds": clock.pause_seconds,
            "active_wall_seconds": time.monotonic() - clock.start - clock.pause_seconds,
            "solve_ms_p50_p95_max": np.quantile(solve_costs, [0.5, 0.95, 1]).tolist(),
            "cycle_ms_p50_p95_max": np.quantile(costs, [0.5, 0.95, 1]).tolist(),
            "max_base_translation_drift_m": node.max_base_translation_drift,
            "max_base_rotation_drift_rad": node.max_base_rotation_drift,
        }
    )
    return result
