import copy
import csv
import hashlib
import json
import math
import os
import signal
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from dual_arm_ik import GROUP
from lab_core import active_joint, pose_values, save_json, segment_duration, timestamp

from .conversion import mapped_eef_frames, select_keyframes
from .download import sha256
from .geometry import check_transform, interpolate_pose, pose_error
from .robot import GripperEEFLab
from .validation import validate_realtime_profile, validate_realtime_source
from .kinematics import validate_backend


def ros_targets(poses):
    targets = []
    for pose in poses:
        target = PoseStamped()
        target.header.frame_id = "odom"
        target.pose.position.x, target.pose.position.y, target.pose.position.z = pose["position"]
        target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w = (
            pose["quaternion_xyzw"]
        )
        targets.append(target)
    return targets


def alignment(node, converted, config):
    if config["mode"] == "calibrated":
        if "tip_to_eef" in config:
            raise ValueError("Tool transforms come from the gripper model, not dataset alignment")
        return check_transform(config["world_to_odom"]), node.eef.tip_to_eef
    if config["mode"] != "shared_world_gripper_eef":
        raise ValueError("Unsupported alignment policy")
    first = converted["source_frames"][0]["eef"]
    world_to_odom = np.eye(4)
    world_to_odom[:3, :3] = Rotation.from_euler("z", config["world_yaw_deg"], degrees=True).as_matrix()
    midpoint = np.mean([pose["position"] for pose in first], axis=0)
    world_to_odom[:3, 3] = np.array(config["initial_midpoint_odom_m"]) - world_to_odom[:3, :3] @ midpoint
    return check_transform(world_to_odom), node.eef.tip_to_eef


def preflight(node, frames, report, output):
    seed = copy.deepcopy(node.home)
    states = []
    first_prepared = None
    for index, frame in enumerate(frames):
        report["current_preflight_keyframe"] = index
        report["current_preflight_source_frame"] = frame["frame_index"]
        report["current_preflight_seed"] = {
            "joint_names": list(seed.joint_state.name),
            "positions": list(seed.joint_state.position),
        }
        targets = ros_targets(frame["eef_odom"])
        node.requested_openings = frame["gripper"]
        prepared = node.prepare(targets, seed=seed, plan=index == 0)
        if index == 0:
            first_prepared = prepared
        else:
            node.validate_path([seed, prepared["goal"]], GROUP)
            predicted = pose_values(node.forward(node.interpolate(seed, prepared["goal"], 0.5)))
            expected = [
                interpolate_pose(first, second, 0.5)
                for first, second in zip(frames[index - 1]["eef_odom"], frame["eef_odom"])
            ]
            midpoint_errors = [pose_error(first, second) for first, second in zip(predicted, expected)]
            if max(item[0] for item in midpoint_errors) > 0.015 or max(
                item[1] for item in midpoint_errors
            ) > math.radians(3):
                raise RuntimeError(
                    f"Joint interpolation departs from Cartesian replay at keyframe {index}: {midpoint_errors}"
                )
        states.append(prepared["goal"])
        seed = prepared["goal"]
        report["preflight"].append(
            {
                "keyframe": index,
                "source_frame": frame["frame_index"],
                "source_timestamp": frame["timestamp"],
                "ik_error_m": prepared["ik_error_m"],
                "ik_orientation_error_rad": prepared["ik_orientation_error_rad"],
            }
        )
        if index % 10 == 0 or index + 1 == len(frames):
            node.publish_progress(0.0, "preflight")
            save_json(output / "report.json", report)
            print(f"Preflight {index + 1}/{len(frames)}: source frame {frame['frame_index']}", flush=True)
    return states, first_prepared


def retime(node, states, frames, speed):
    if not math.isfinite(speed) or speed <= 0 or speed > 1:
        raise ValueError("Replay speed must be in (0,1]")
    scale = 1 / speed
    for start, goal, previous, current in zip(states, states[1:], frames, frames[1:]):
        source_duration = current["timestamp"] - previous["timestamp"]
        if source_duration <= 0:
            raise ValueError("Replay timestamps must strictly increase")
        required = 0.0
        for name, initial, target in zip(start.joint_state.name, start.joint_state.position, goal.joint_state.position):
            if active_joint(name, GROUP) or name in getattr(getattr(node, "eef", None), "master_names", ()):
                velocity = min(node.max_velocity, node.velocities[name])
                required = max(required, segment_duration(abs(target - initial), 0, velocity, node.max_acceleration))
        scale = max(scale, required / source_duration)
    return float(scale)


def execute(node, states, frames, scale, config, output):
    fields = [
        "wall_time_s",
        "feedback_ros_time_s",
        "source_time_s",
        "keyframe",
        "side",
        "position_error_m",
        "rotation_error_rad",
    ]
    fields += [f"{kind}_{axis}" for kind in ("target", "actual") for axis in ("x", "y", "z", "qx", "qy", "qz", "qw")]
    body_names = [name for name in node.home.joint_state.name if "_body_joint" in name]
    fields += body_names
    result = {
        "tracking_samples": 0,
        "max_position_error_m": 0.0,
        "max_rotation_error_rad": 0.0,
        "max_command_step_rad": 0.0,
        "max_body_displacement_rad": 0.0,
        "max_gripper_opening_error": 0.0,
    }
    home = dict(zip(node.home.joint_state.name, node.home.joint_state.position))
    last_observation = 0.0
    start_time = time.monotonic()
    with (output / "tracking.csv").open("w") as stream, (output / "command_phase.csv").open("w") as phase_stream, (
        output / "gripper_tracking.csv"
    ).open("w") as gripper_stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        phase_writer = csv.writer(phase_stream)
        phase_writer.writerow(["wall_time_s", "command_ros_time_s", "source_time_s", "keyframe"])
        gripper_writer = csv.writer(gripper_stream)
        gripper_writer.writerow(
            ["feedback_ros_time_s", "source_time_s", "side", "target_opening", "actual_opening", "master_angle_rad"]
        )
        for index in range(1, len(states)):
            start, goal = states[index - 1], states[index]
            duration = scale * (frames[index]["timestamp"] - frames[index - 1]["timestamp"])
            elapsed = 0.0
            previous_tick = time.monotonic()
            previous_blend = 0.0
            delta = max(
                abs(second - first)
                for name, first, second in zip(
                    start.joint_state.name, start.joint_state.position, goal.joint_state.position
                )
                if node.is_command_joint(name)
            )
            while rclpy.ok():
                tick = time.monotonic()
                if node.cancel.is_set():
                    raise RuntimeError("Replay cancelled; holding measured position")
                if tick - node.last_state_time > 1 or node.count_publishers("/joint_command") > 1:
                    raise RuntimeError("Stale feedback or competing command publisher")
                node.observe_base_drift()
                if getattr(node, "pause_requested", False):
                    node.publish_positions(node.interpolate(start, goal, previous_blend), GROUP)
                    held_phase = frames[index - 1]["timestamp"] + previous_blend * (
                        frames[index]["timestamp"] - frames[index - 1]["timestamp"]
                    )
                    node.publish_progress(held_phase, "paused")
                    phase_writer.writerow([time.time(), node.get_clock().now().nanoseconds * 1e-9, held_phase, index])
                    phase_stream.flush()
                    previous_tick = time.monotonic()
                    rclpy.spin_once(node, timeout_sec=0.02)
                    continue
                elapsed += min(tick - previous_tick, 0.04)
                previous_tick = tick
                fraction = min(1.0, elapsed / duration)
                blend = fraction * fraction * (3 - 2 * fraction)
                step = (blend - previous_blend) * delta
                if step > 0.02:
                    raise RuntimeError("Joint command discontinuity exceeds 0.02 rad")
                result["max_command_step_rad"] = max(result["max_command_step_rad"], step)
                node.publish_positions(node.interpolate(start, goal, blend), GROUP)
                source_time = frames[index - 1]["timestamp"] + blend * (
                    frames[index]["timestamp"] - frames[index - 1]["timestamp"]
                )
                phase_writer.writerow([time.time(), node.get_clock().now().nanoseconds * 1e-9, source_time, index])
                node.publish_progress(source_time, "replaying")
                previous_blend = blend
                rclpy.spin_once(node, timeout_sec=0.001)
                if tick - last_observation > 0.2 or fraction >= 1:
                    target = [
                        interpolate_pose(first, second, blend)
                        for first, second in zip(frames[index - 1]["eef_odom"], frames[index]["eef_odom"])
                    ]
                    actual = pose_values(node.forward(node.state))
                    header = node.state.joint_state.header.stamp
                    feedback_time = header.sec + header.nanosec * 1e-9
                    positions = dict(zip(node.state.joint_state.name, node.state.joint_state.position))
                    actual_openings = node.eef.openings_from_positions(positions)
                    target_openings = (1 - blend) * np.array(frames[index - 1]["gripper"]) + blend * np.array(
                        frames[index]["gripper"]
                    )
                    opening_error = float(np.max(np.abs(target_openings - actual_openings)))
                    result["max_gripper_opening_error"] = max(result["max_gripper_opening_error"], opening_error)
                    if opening_error > 0.1:
                        raise RuntimeError(f"Gripper opening tracking error exceeds 0.1: {opening_error}")
                    for side, desired, actual_opening, master in zip(
                        ("left", "right"), target_openings, actual_openings, node.eef.master_names
                    ):
                        gripper_writer.writerow(
                            [feedback_time, source_time, side, desired, actual_opening, positions[master]]
                        )
                    gripper_stream.flush()
                    for side, wanted, measured in zip(("left", "right"), target, actual):
                        position, rotation = pose_error(wanted, measured)
                        result["max_position_error_m"] = max(result["max_position_error_m"], position)
                        result["max_rotation_error_rad"] = max(result["max_rotation_error_rad"], rotation)
                        source_time = frames[index - 1]["timestamp"] + blend * (
                            frames[index]["timestamp"] - frames[index - 1]["timestamp"]
                        )
                        writer.writerow(
                            [
                                time.time(),
                                feedback_time,
                                source_time,
                                index,
                                side,
                                position,
                                rotation,
                                *wanted["position"],
                                *wanted["quaternion_xyzw"],
                                *measured["position"],
                                *measured["quaternion_xyzw"],
                                *[positions[name] for name in body_names],
                            ]
                        )
                        if position > config["max_tracking_error_m"] or rotation > config["max_tracking_rotation_rad"]:
                            raise RuntimeError(
                                f"Measured replay tracking error exceeds tolerance: {side}: {position} m / {rotation} rad"
                            )
                    result["tracking_samples"] += 1
                    result["max_body_displacement_rad"] = max(
                        result["max_body_displacement_rad"],
                        max(abs(positions[name] - home[name]) for name in body_names),
                    )
                    node.show_targets(ros_targets(target), node.forward(node.state))
                    last_observation = time.monotonic()
                    stream.flush()
                    phase_stream.flush()
                if fraction >= 1:
                    break
                time.sleep(max(0.0, 0.02 - (time.monotonic() - tick)))
            if index % 20 == 0 or index + 1 == len(states):
                print(f"Replay {index + 1}/{len(states)}; source time {frames[index]['timestamp']:.2f}s", flush=True)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            if node.cancel.is_set():
                raise RuntimeError("Cancelled during final settling")
            if time.monotonic() - node.last_state_time > 1 or node.count_publishers("/joint_command") > 1:
                raise RuntimeError("Feedback/publisher guard failed during final settling")
            node.publish_positions(states[-1], GROUP)
            rclpy.spin_once(node, timeout_sec=0.02)
            node.observe_base_drift()
    errors = [
        pose_error(first, second)
        for first, second in zip(frames[-1]["eef_odom"], pose_values(node.forward(node.state)))
    ]
    if max(item[0] for item in errors) > 0.02 or max(item[1] for item in errors) > math.radians(3):
        raise RuntimeError(f"Final tracking error exceeds 2cm/3deg: {errors}")
    result.update(
        {
            "final_errors": errors,
            "wall_seconds": time.monotonic() - start_time,
            "time_scale": scale,
            "max_base_translation_drift_m": node.max_base_translation_drift,
            "max_base_rotation_drift_rad": node.max_base_rotation_drift,
        }
    )
    return result


def posture_config(config, policy):
    config = copy.deepcopy(config)
    if policy not in ("config", "soft", "upright"):
        raise ValueError("Unsupported posture policy")
    bounds = config.get("joint_position_bounds_rad", {})
    if policy == "soft":
        bounds.pop("idx04_body_joint4", None)
        if not bounds:
            config.pop("joint_position_bounds_rad", None)
    elif policy == "upright":
        bounds["idx04_body_joint4"] = [-math.radians(0.5), math.radians(0.5)]
        config["joint_position_bounds_rad"] = bounds
    return config


def run(args, converted, config):
    policy = getattr(args, "posture_policy", "config")
    realtime = getattr(args, "playback_mode", "validated") == "realtime"
    config = posture_config(config, policy)
    realtime_profile = None
    backend = validate_backend(getattr(args, "kinematics_backend", "urdf"))
    if not realtime and backend != "urdf":
        raise ValueError("Kinematics backend selection only applies to realtime playback")
    approach_reference = None
    if getattr(args, "approach_reference", None):
        from .approach_reference import validate_reference

        if not realtime:
            raise ValueError("Frozen approach reference only applies to realtime playback")
        approach_bytes = args.approach_reference.read_bytes()
        approach_reference = validate_reference(json.loads(approach_bytes), sha256(args.input), config)
    if realtime:
        profile_bytes = args.limits_profile.read_bytes()
        realtime_profile = validate_realtime_profile(json.loads(profile_bytes))
        validate_realtime_source(converted["source_frames"])
    converted = copy.deepcopy(converted)
    converted["keyframe_indexes"] = select_keyframes(
        converted["source_frames"], converted["sampling"]["max_interval_seconds"]
    )
    if realtime:
        converted["keyframe_indexes"] = list(range(len(converted["source_frames"])))
    if converted["schema"] != "xense-umi-eef-v1" or converted["frame"] != "dataset_world":
        raise ValueError("Unsupported converted trajectory")
    if not math.isfinite(args.speed) or not 0 < args.speed <= 1:
        raise ValueError("Replay speed must be in (0,1]")
    for key in (
        "max_joint_velocity_rad_s",
        "max_joint_acceleration_rad_s2",
        "max_tracking_error_m",
        "max_tracking_rotation_rad",
    ):
        if not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f"Invalid safety limit: {key}")
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    report = {
        "status": "starting",
        "started_at": timestamp(),
        "input_sha256": sha256(args.input),
        "provenance": converted["provenance"],
        "config": config,
        "preflight": [],
        "execution_requested": args.execute,
        "planning_group": GROUP,
        "physical_chassis": "must be pinned to world by scene_stress.yaml",
        "grippers": "0 closed / 1 open; linear native master-joint mapping; mimic followers; dynamic fingertip-midpoint TCP",
        "head": "not replayed",
        "posture_policy": policy,
        "effective_keyframes": len(converted["keyframe_indexes"]),
        "gripper_sampling_tolerance": 0.02,
        "playback_mode": "realtime" if realtime else "validated",
        "preflight_scope": "initial_pose_only" if realtime else "all_sampled_keyframes",
        "kinematics_backend": backend,
        "motion_started": False,
    }
    if realtime:
        report["realtime_limits_profile"] = realtime_profile
        report["realtime_limits_profile_sha256"] = hashlib.sha256(profile_bytes).hexdigest()
    if approach_reference:
        report["approach_reference_report_sha256"] = hashlib.sha256(approach_bytes).hexdigest()
        report["approach_reference_path"] = str(args.approach_reference)
    report["backend_log_directory"] = os.environ.get("WBC_BACKEND_LOG_DIR")
    save_json(output / "report.json", report)
    os.environ.setdefault("ROS_DOMAIN_ID", "42")
    rclpy.init()
    node = GripperEEFLab(
        config["max_joint_velocity_rad_s"],
        config["max_joint_acceleration_rad_s2"],
        config.get("joint_position_bounds_rad"),
    )
    handlers = {signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)}
    for signum in handlers:
        signal.signal(signum, lambda received, frame: node.cancel.set())
    motion_started = False
    try:
        if getattr(args, "interactive_controls", False):
            node.enable_playback_controls()
        report["interactive_control_events"] = node.control_events
        node.publish_progress(0.0, "initializing")
        node.acquire_control()
        node.initialize()
        report["initial_measured_positions_rad"] = dict(
            zip(node.state.joint_state.name, node.state.joint_state.position)
        )
        if realtime:
            from .realtime_execution import prepare_model

            node.realtime_model = prepare_model(node, backend)
        report["tracking_pose_frame"] = "gripper_eef"
        report["robot_eef_frames"] = node.eef.frame_names
        report["solver_internal_links"] = node.eef.tip_names
        report["gripper_eef_model"] = node.eef.evidence
        if args.execute:
            node.require_sim_camera()
        node.trace_file = (output / "joint_trace.csv").open("w")
        world_to_odom, offsets = alignment(node, converted, config)
        frames = mapped_eef_frames(converted, world_to_odom)
        if len(offsets) != 2 or len(frames) < 2:
            raise ValueError("Exactly two tools and at least two frames required")
        report["resolved_alignment"] = {
            "world_to_odom": world_to_odom.tolist(),
            "tip_to_eef": [item.tolist() for item in offsets],
            "source_start_midpoint_m": np.mean(
                [pose["position"] for pose in converted["source_frames"][0]["eef"]], axis=0
            ).tolist(),
        }
        save_json(output / "mapped_targets.json", frames)
        report["status"] = "preflight_running"
        node.publish_progress(0.0, "preflight")
        if realtime:
            from .realtime_execution import prepare_initial

            if approach_reference:
                from .approach_reference import prepare_reference

                first = prepare_reference(node, frames[0], approach_reference)
            else:
                first = prepare_initial(node, frames[0])
            report["initial_approach_goal_rad"] = dict(
                zip(first["goal"].joint_state.name, first["goal"].joint_state.position)
            )
            report["initial_approach_ik"] = {
                key: first[key] for key in ("ik_error_m", "ik_orientation_error_rad", "solve_seconds")
            }
            report["initial_approach_ik"]["method"] = first["planning_method"]
            report["time_scale"] = 1.0
        else:
            states, first = preflight(node, frames, report, output)
            scale = retime(node, states, frames, args.speed)
            report["retimed_duration_seconds"] = scale * converted["source_duration_seconds"]
            report["time_scale"] = scale
            save_json(
                output / "joint_plan.json",
                {
                    "joint_names": states[0].joint_state.name,
                    "positions": [list(state.joint_state.position) for state in states],
                    "source_times": [frame["timestamp"] for frame in frames],
                    "time_scale": scale,
                },
            )
        if args.execute:
            report["status"] = "executing"
            save_json(output / "report.json", report)
            motion_started = True
            report["motion_started"] = True
            node.publish_progress(0.0, "approaching")
            report["approach"] = node.execute_prepared(first)
            if realtime:
                from .realtime_execution import stream

                report["replay"] = stream(node, frames, config, realtime_profile, output, report)
            else:
                report["replay"] = execute(node, states, frames, scale, config, output)
            node.publish_progress(frames[-1]["timestamp"], "returning_home")
            report["return_home"] = node.return_home()
        report["status"] = "passed" if args.execute else "preflight_passed_no_execution"
        if realtime and not args.execute:
            report["status"] = "initial_pose_preflight_passed_no_execution"
        node.publish_progress(converted["source_duration_seconds"] if args.execute else 0.0, "finished")
        print(f"{report['status']}: {output}", flush=True)
    except BaseException as error:
        node.publish_progress(
            getattr(node, "last_source_time", 0.0),
            "failed",
            deadline_misses=report.get("replay", {}).get("deadline_misses", 0),
            replay_error=str(error),
        )
        report["status"] = "failed"
        report["error"] = str(error)
        if motion_started and node.command_lock is not None:
            node.hold(GROUP)
        raise
    finally:
        report["finished_at"] = timestamp()
        save_json(output / "report.json", report)
        node.release()
        node.destroy_node()
        for signum, handler in handlers.items():
            signal.signal(signum, handler)
        rclpy.shutdown()
