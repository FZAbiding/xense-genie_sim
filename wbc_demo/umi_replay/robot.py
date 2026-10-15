from pathlib import Path

import copy
import math
import time

import rclpy

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped
from moveit_msgs.msg import JointConstraint
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from lab_core import WholeBodyLab, pose_values, active_joint, MotionCancelled

from .geometry import pose_dict
from .gripper_eef import SwiftpickerEEF


def stamped_poses(poses, frame="odom"):
    result = []
    for pose in poses:
        target = PoseStamped()
        target.header.frame_id = frame
        target.pose.position.x, target.pose.position.y, target.pose.position.z = pose["position"]
        target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w = (
            pose["quaternion_xyzw"]
        )
        result.append(target)
    return result


class GripperEEFLab(WholeBodyLab):
    def __init__(self, max_velocity=0.3, max_acceleration=0.5, joint_position_bounds=None):
        super().__init__(max_velocity, max_acceleration)
        self.pause_requested = False
        self.control_events = []
        self.progress_publisher = self.create_publisher(String, "/umi_replay/progress", 10)
        self.joint_position_bounds = joint_position_bounds or {}
        for name, bounds in self.joint_position_bounds.items():
            if name not in self.limits or len(bounds) != 2 or not all(math.isfinite(value) for value in bounds):
                raise ValueError(f"Invalid posture bound: {name}")
            if bounds[0] >= bounds[1] or bounds[0] < self.limits[name][0] or bounds[1] > self.limits[name][1]:
                raise ValueError(f"Posture bounds must lie inside native limits: {name}")
        package = Path(get_package_share_directory("genie_sim_robot_model"))
        self.eef = SwiftpickerEEF(package / "urdf/genie_g2_crsB_swiftpicker.urdf", package)
        self.requested_openings = None
        self.eef.evidence["runtime_tcp"] = (
            "fingertip midpoint recomputed from measured gripper joints; zero-joint fully-open reference retained above"
        )
        self.eef_broadcaster = TransformBroadcaster(self)

    def on_state(self, message):
        super().on_state(message)
        if hasattr(self, "eef_broadcaster"):
            self.broadcast_eef(dict(zip(message.name, message.position)), message.header.stamp)

    def broadcast_eef(self, positions, stamp):
        transforms = []
        for parent, child, matrix in zip(self.eef.tip_names, self.eef.frame_names, self.eef.offsets_at(positions)):
            value = pose_dict(matrix)
            message = TransformStamped()
            message.header.stamp = stamp
            message.header.frame_id = parent
            message.child_frame_id = child
            message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = value[
                "position"
            ]
            (
                message.transform.rotation.x,
                message.transform.rotation.y,
                message.transform.rotation.z,
                message.transform.rotation.w,
            ) = value["quaternion_xyzw"]
            transforms.append(message)
        self.eef_broadcaster.sendTransform(transforms)

    def enable_playback_controls(self):
        self.control_subscription = self.create_subscription(String, "/umi_replay/control", self.playback_control, 10)

    def require_sim_camera(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.count_publishers("/genie_sim/free_camera_rgb/image_raw") > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError("Genie Sim camera publisher was not discovered; no motion is permitted")

    def playback_control(self, message):
        command = message.data
        if command not in ("pause", "resume", "stop"):
            return
        self.control_events.append({"command": command, "ros_time_s": self.get_clock().now().nanoseconds * 1e-9})
        if command == "stop":
            self.cancel.set()
        else:
            self.pause_requested = command == "pause"

    def publish_progress(self, source_time, stage):
        import json

        self.progress_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "source_time_s": source_time,
                        "stage": stage,
                        "paused": self.pause_requested,
                    }
                )
            )
        )

    def check_gripper_reference(self, state):
        values = dict(zip(state.joint_state.name, state.joint_state.position))
        for name, value in values.items():
            if "_gripper_" in name and name in self.limits:
                lower, upper = self.limits[name]
                if not math.isfinite(value) or not lower - 0.02 <= value <= upper + 0.02:
                    raise RuntimeError(f"Measured gripper joint outside native range: {name}")

    def is_command_joint(self, name):
        return active_joint(name, "wbc_fixed_headless") or name in self.eef.master_names

    def with_openings(self, state, openings):
        result = copy.deepcopy(state)
        desired = self.eef.opening_positions(openings)
        result.joint_state.position = [
            desired.get(name, value) for name, value in zip(state.joint_state.name, state.joint_state.position)
        ]
        return result

    def publish_positions(self, state, group):
        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        values = [
            (name, value)
            for name, value in zip(state.joint_state.name, state.joint_state.position)
            if self.is_command_joint(name)
        ]
        command.name = [name for name, value in values]
        command.position = [value for name, value in values]
        self.command.publish(command)

    def execute_prepared(self, prepared, seconds=4.0):
        if len(prepared["path"]) != 2:
            raise RuntimeError("Gripper replay requires the checked direct approach/return path")
        beginning = dict(zip(prepared["start"].joint_state.name, prepared["start"].joint_state.position))
        ending = dict(zip(prepared["goal"].joint_state.name, prepared["goal"].joint_state.position))
        for name in self.eef.master_names:
            delta = abs(ending[name] - beginning[name])
            seconds = max(
                seconds,
                1.5 * delta / min(self.max_velocity, self.velocities[name]),
                math.sqrt(6 * delta / self.max_acceleration),
            )
        return super().execute_prepared(prepared, seconds)

    def validate_path(self, path, group):
        samples = 0
        for start, goal in zip(path, path[1:]):
            delta = max(
                abs(target - initial)
                for name, initial, target in zip(
                    start.joint_state.name, start.joint_state.position, goal.joint_state.position
                )
                if self.is_command_joint(name)
            )
            count = max(21, math.ceil(delta / 0.025) + 1)
            for index in range(count):
                if self.cancel.is_set():
                    raise MotionCancelled("Cancelled during gripper-inclusive path validation")
                self.valid_state(self.interpolate(start, goal, index / (count - 1)), group)
                samples += 1
        return samples

    def forward(self, state, frame="odom"):
        self.check_gripper_reference(state)
        tips = super().forward(state, frame)
        return stamped_poses(
            self.eef.to_eef(pose_values(tips), dict(zip(state.joint_state.name, state.joint_state.position))), frame
        )

    def solve(self, state, targets):
        self.check_gripper_reference(state)
        openings = getattr(self, "requested_openings", None)
        state = self.with_openings(state, openings) if openings is not None else state
        positions = dict(zip(state.joint_state.name, state.joint_state.position))
        flange_targets = stamped_poses(self.eef.to_flange(pose_values(targets), positions))
        bounds = getattr(self, "joint_position_bounds", {})
        if not bounds:
            solution = super().solve(state, flange_targets)
            return self.with_openings(solution, openings) if openings is not None else solution
        constraints = [
            JointConstraint(
                joint_name=name,
                position=(lower + upper) / 2,
                tolerance_above=(upper - lower) / 2,
                tolerance_below=(upper - lower) / 2,
                weight=1.0,
            )
            for name, (lower, upper) in bounds.items()
        ]
        solution = super().solve(state, flange_targets, joint_constraints=constraints)
        positions = dict(zip(solution.joint_state.name, solution.joint_state.position))
        for name, (lower, upper) in bounds.items():
            if not lower - 1e-6 <= positions[name] <= upper + 1e-6:
                raise RuntimeError(f"IK returned a posture outside the requested bound: {name}")
        return self.with_openings(solution, openings) if openings is not None else solution
