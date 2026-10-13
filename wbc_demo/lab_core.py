import copy
import fcntl
import json
import math
import os
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan, GetStateValidity
from sensor_msgs.msg import JointState
from transforms3d.euler import euler2quat
from transforms3d.quaternions import qmult
from visualization_msgs.msg import Marker, MarkerArray

from dual_arm_ik import DualArmDemo, GROUP, TIPS, orientation_error, position_error

ARM_GROUP = "wbc_fixed_arms"
ROOT = Path(__file__).parent


class MotionCancelled(RuntimeError):
    pass


def active_joint(name, group):
    return "_arm_l_joint" in name or "_arm_r_joint" in name or ("_body_joint" in name and group == GROUP)


def finite_targets(targets):
    if len(targets) != 2:
        raise ValueError("Exactly two TCP targets are required")
    for target in targets:
        if target.header.frame_id != "odom":
            raise ValueError("Targets must use the odom frame")
        pose = target.pose
        values = [pose.position.x, pose.position.y, pose.position.z]
        quaternion = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        if not all(math.isfinite(value) for value in values + quaternion):
            raise ValueError("Non-finite target rejected")
        if abs(sum(value * value for value in quaternion) - 1.0) > 0.001:
            raise ValueError("Target orientation must be a unit quaternion")


def make_targets(reference, specification, center):
    targets = copy.deepcopy(reference)
    orbit = math.radians(float(specification.get("orbit_deg", 0.0)))
    for side, target in zip(("left", "right"), targets):
        offset = specification.get(side, [0.0, 0.0, 0.0])
        position = np.array([target.pose.position.x, target.pose.position.y, target.pose.position.z])
        if orbit:
            rotation = np.array([[math.cos(orbit), -math.sin(orbit)], [math.sin(orbit), math.cos(orbit)]])
            position[:2] = rotation @ (position[:2] - np.array(center[:2])) + np.array(center[:2])
        position += np.array(offset, dtype=float)
        target.pose.position.x, target.pose.position.y, target.pose.position.z = position.tolist()
        original = target.pose.orientation
        quaternion = [original.w, original.x, original.y, original.z]
        local = euler2quat(*[math.radians(value) for value in specification.get(side + "_rpy_deg", [0, 0, 0])])
        quaternion = qmult(euler2quat(0, 0, orbit), qmult(quaternion, local))
        original.w, original.x, original.y, original.z = [float(value) for value in quaternion]
    finite_targets(targets)
    return targets


def segment_duration(delta, seconds, max_velocity, max_acceleration):
    return max(seconds, 1.5 * delta / max_velocity, math.sqrt(6.0 * delta / max_acceleration))


def pose_values(targets):
    return [
        {
            "position": [target.pose.position.x, target.pose.position.y, target.pose.position.z],
            "quaternion_xyzw": [
                target.pose.orientation.x,
                target.pose.orientation.y,
                target.pose.orientation.z,
                target.pose.orientation.w,
            ],
        }
        for target in targets
    ]


class WholeBodyLab(DualArmDemo):
    def __init__(self, max_velocity=0.3, max_acceleration=0.5):
        super().__init__()
        if max_velocity <= 0 or max_acceleration <= 0:
            raise ValueError("Velocity and acceleration limits must be positive")
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.cancel = threading.Event()
        self.progress = None
        self.reference = None
        self.home = None
        self.center = None
        self.command_steps = []
        self.marker_publisher = self.create_publisher(MarkerArray, "/wbc_demo/markers", 10)
        self.planner = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.trails = [[], []]
        self.last_visualization = 0.0
        self.command_lock = None
        model = Path(get_package_share_directory("genie_sim_robot_model")) / "urdf/genie_g2_crsB_swiftpicker.urdf"
        self.robot = ET.parse(model).getroot()
        self.limits = {
            joint.attrib["name"]: (float(joint.find("limit").get("lower")), float(joint.find("limit").get("upper")))
            for joint in self.robot.findall("joint")
            if joint.find("limit") is not None and joint.find("limit").get("lower") is not None
        }
        self.velocities = {
            joint.attrib["name"]: float(joint.find("limit").get("velocity"))
            for joint in self.robot.findall("joint")
            if joint.find("limit") is not None and joint.find("limit").get("velocity") is not None
        }
        constraints = Path(get_package_share_directory("genie_sim_moveit_plugins")) / "config/coupled_constraints.yaml"
        self.coupled = yaml.safe_load(constraints.read_text())["coupled_constraints"]

    def acquire_control(self):
        lock_path = Path(tempfile.gettempdir()) / f"xense-wbc-command-{os.environ.get('ROS_DOMAIN_ID', '0')}.lock"
        self.command_lock = lock_path.open("a")
        try:
            fcntl.flock(self.command_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            self.command_lock.close()
            self.command_lock = None
            raise RuntimeError("Another lab/stress controller owns this ROS domain") from error

    def initialize(self):
        self.wait_for_state()
        self.home = copy.deepcopy(self.state)
        self.clamp_limits(self.home, GROUP)
        self.reference = self.forward(self.home)
        self.center = self.base_position()
        self.base_initial_transform = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time()).transform
        self.show_targets(self.reference, self.forward(self.state))

    def clamp_limits(self, state, group):
        values = list(state.joint_state.position)
        for index, name in enumerate(state.joint_state.name):
            if active_joint(name, group) and name in self.limits:
                lower, upper = self.limits[name]
                if values[index] < lower - 0.0101 or values[index] > upper + 0.0101:
                    raise RuntimeError(f"Native joint limit violated: {name}")
                values[index] = min(upper, max(lower, values[index]))
        state.joint_state.position = values

    def validate_constraints(self, state, group, tolerance=0.0001):
        positions = dict(zip(state.joint_state.name, state.joint_state.position))
        for constraint in self.coupled:
            if not any(active_joint(name, group) for name in constraint["joint_names"]):
                continue
            values = np.array([positions[name] for name in constraint["joint_names"]])
            residual = np.array(constraint["A"]) @ values - np.array(constraint["b"])
            if float(max(residual)) > tolerance:
                raise RuntimeError(f"Mechanical coupled constraint violated: {constraint['joint_names']}")

    def valid_state(self, state, group):
        self.validate_constraints(state, group, tolerance=0.015)
        request = GetStateValidity.Request(robot_state=state, group_name=group)
        if not self.call(self.validity, request).valid:
            raise RuntimeError("Collision or MoveIt state bounds violation")

    def validate_path(self, path, group):
        samples = 0
        for start, goal in zip(path, path[1:]):
            delta = max(
                abs(target - initial)
                for name, initial, target in zip(
                    start.joint_state.name, start.joint_state.position, goal.joint_state.position
                )
                if active_joint(name, group)
            )
            count = max(21, math.ceil(delta / 0.025) + 1)
            for index in range(count):
                if self.cancel.is_set():
                    raise MotionCancelled("Cancelled during path validation")
                self.valid_state(self.interpolate(start, goal, index / (count - 1)), group)
                samples += 1
        return samples

    def plan_path(self, start, goal, group):
        path = [start, goal]
        try:
            samples = self.validate_path(path, group)
            return path, "joint_interpolation", samples
        except MotionCancelled:
            raise
        except RuntimeError as direct_error:
            if not self.planner.wait_for_service(timeout_sec=2.0):
                raise RuntimeError(f"Direct path rejected and OMPL unavailable: {direct_error}") from direct_error
        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = group
        request.motion_plan_request.start_state = start
        request.motion_plan_request.pipeline_id = "ompl"
        request.motion_plan_request.planner_id = "RRTConnectkConfigDefault"
        request.motion_plan_request.allowed_planning_time = 5.0
        request.motion_plan_request.num_planning_attempts = 3
        request.motion_plan_request.max_velocity_scaling_factor = 0.1
        request.motion_plan_request.max_acceleration_scaling_factor = 0.1
        constraints = Constraints()
        for name, value in zip(goal.joint_state.name, goal.joint_state.position):
            if active_joint(name, group):
                constraints.joint_constraints.append(
                    JointConstraint(
                        joint_name=name, position=value, tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0
                    )
                )
        request.motion_plan_request.goal_constraints = [constraints]
        result = self.call(self.planner, request).motion_plan_response
        if result.error_code.val != 1:
            raise RuntimeError(f"OMPL planning failed: {result.error_code.val}")
        path = [start]
        for point in result.trajectory.joint_trajectory.points:
            state = copy.deepcopy(start)
            values = dict(zip(result.trajectory.joint_trajectory.joint_names, point.positions))
            state.joint_state.position = [
                values.get(name, value) for name, value in zip(state.joint_state.name, state.joint_state.position)
            ]
            path.append(state)
        path.append(goal)
        return path, "ompl_rrt_connect", self.validate_path(path, group)

    def prepare(self, targets, group=GROUP, seed=None, plan=True):
        finite_targets(targets)
        self.group = group
        start = copy.deepcopy(seed if seed is not None else self.state)
        beginning = time.monotonic()
        last_error = None
        for attempt in range(3):
            if self.cancel.is_set():
                raise MotionCancelled("Cancelled before IK")
            try:
                goal = self.solve(start, targets)
                self.clamp_limits(goal, group)
                self.validate_constraints(goal, group)
                predicted = self.forward(goal)
                errors = [position_error(actual.pose, target.pose) for actual, target in zip(predicted, targets)]
                rotations = [
                    orientation_error(actual.pose.orientation, target.pose.orientation)
                    for actual, target in zip(predicted, targets)
                ]
                if max(errors) > 0.005 or max(rotations) > math.radians(1.0):
                    raise RuntimeError(f"IK residual after native-limit projection: {errors}, {rotations}")
                self.valid_state(goal, group)
                if plan:
                    path, method, samples = self.plan_path(start, goal, group)
                else:
                    path, method, samples = [start, goal], "not_executed", 0
                return {
                    "start": start,
                    "goal": goal,
                    "targets": copy.deepcopy(targets),
                    "group": group,
                    "path": path,
                    "planning_method": method,
                    "collision_samples": samples,
                    "ik_error_m": errors,
                    "ik_orientation_error_rad": rotations,
                    "solve_seconds": time.monotonic() - beginning,
                    "attempts": attempt + 1,
                }
            except MotionCancelled:
                raise
            except RuntimeError as error:
                last_error = error
        raise RuntimeError(f"Target rejected after 3 attempts: {last_error}")

    def publish_positions(self, state, group):
        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        values = [
            (name, value)
            for name, value in zip(state.joint_state.name, state.joint_state.position)
            if active_joint(name, GROUP)
        ]
        command.name = [name for name, value in values]
        command.position = [value for name, value in values]
        self.command.publish(command)

    def hold(self, group):
        if self.state is not None and rclpy.ok():
            held = copy.deepcopy(self.state)
            for attempt in range(3):
                self.publish_positions(held, group)
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.02)

    def execute_prepared(self, prepared, seconds=4.0):
        group = prepared["group"]
        if self.command_lock is None:
            raise RuntimeError("Control lease not acquired")
        if self.count_publishers("/joint_command") > 1:
            raise RuntimeError("Multiple /joint_command publishers detected; refusing competing control")
        current = dict(zip(self.state.joint_state.name, self.state.joint_state.position))
        stale = max(
            abs(current[name] - value)
            for name, value in zip(prepared["start"].joint_state.name, prepared["start"].joint_state.position)
            if active_joint(name, GROUP)
        )
        if stale > 0.03:
            raise RuntimeError("Preview is stale; solve again from the current robot state")
        beginning = time.monotonic()
        peak_step = 0.0
        duration_sum = 0.0
        try:
            for start, goal in zip(prepared["path"], prepared["path"][1:]):
                delta = max(
                    abs(target - initial)
                    for name, initial, target in zip(
                        start.joint_state.name, start.joint_state.position, goal.joint_state.position
                    )
                    if active_joint(name, group)
                )
                if delta < 0.00001:
                    continue
                duration = segment_duration(
                    delta, seconds if len(prepared["path"]) == 2 else 0.1, self.max_velocity, self.max_acceleration
                )
                for name, initial, target in zip(
                    start.joint_state.name, start.joint_state.position, goal.joint_state.position
                ):
                    if active_joint(name, group):
                        velocity = min(self.max_velocity, self.velocities.get(name, self.max_velocity))
                        if velocity <= 0:
                            raise RuntimeError(f"Invalid native velocity limit: {name}")
                        duration = max(duration, 1.5 * abs(target - initial) / velocity)
                duration_sum += duration
                segment_start = time.monotonic()
                previous_blend = 0.0
                previous_tick = segment_start
                virtual_elapsed = 0.0
                while rclpy.ok():
                    tick = time.monotonic()
                    if self.cancel.is_set():
                        raise MotionCancelled("Software stop requested; holding measured joint positions")
                    if tick - self.last_state_time > 1.0:
                        raise RuntimeError("Joint-state feedback stale")
                    self.observe_base_drift()
                    virtual_elapsed += min(tick - previous_tick, 0.04)
                    previous_tick = tick
                    fraction = min(virtual_elapsed / duration, 1.0)
                    blend = min(fraction * fraction * (3.0 - 2.0 * fraction), previous_blend + 0.02 / delta)
                    peak_step = max(peak_step, (blend - previous_blend) * delta)
                    command = self.interpolate(start, goal, blend)
                    self.publish_positions(command, group)
                    previous_blend = blend
                    rclpy.spin_once(self, timeout_sec=0.001)
                    if self.progress is not None and tick - self.last_visualization > 0.2:
                        self.progress(self.forward(self.state), prepared["targets"])
                        self.last_visualization = tick
                    if blend >= 1.0:
                        break
                    time.sleep(max(0.0, 0.02 - (time.monotonic() - tick)))
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                if self.cancel.is_set():
                    raise MotionCancelled("Software stop requested during settling")
                if time.monotonic() - self.last_state_time > 1.0:
                    raise RuntimeError("Joint-state feedback stale during settling")
                self.publish_positions(prepared["goal"], group)
                rclpy.spin_once(self, timeout_sec=0.02)
                self.observe_base_drift()
        except BaseException:
            self.hold(group)
            raise
        measured = self.forward(self.state)
        errors = [position_error(actual.pose, target.pose) for actual, target in zip(measured, prepared["targets"])]
        rotations = [
            orientation_error(actual.pose.orientation, target.pose.orientation)
            for actual, target in zip(measured, prepared["targets"])
        ]
        if max(errors) > 0.02 or max(rotations) > math.radians(3.0):
            raise RuntimeError(f"Measured tracking error exceeds tolerance: {errors}, {rotations}")
        body_start = dict(zip(self.home.joint_state.name, self.home.joint_state.position))
        body_delta = {
            name: value - body_start[name]
            for name, value in zip(self.state.joint_state.name, self.state.joint_state.position)
            if "_body_joint" in name
        }
        start_positions = dict(zip(prepared["start"].joint_state.name, prepared["start"].joint_state.position))
        locked_body_drift = max(
            abs(value - start_positions[name])
            for name, value in zip(self.state.joint_state.name, self.state.joint_state.position)
            if "_body_joint" in name
        )
        if group == ARM_GROUP and locked_body_drift > 0.015:
            raise RuntimeError(f"Position-held waist drift exceeded 0.015 rad: {locked_body_drift}")
        result = {
            key: prepared[key]
            for key in (
                "group",
                "planning_method",
                "collision_samples",
                "ik_error_m",
                "ik_orientation_error_rad",
                "solve_seconds",
                "attempts",
            )
        }
        result.update(
            {
                "status": "passed",
                "targets": pose_values(prepared["targets"]),
                "measured_tcp_error_m": errors,
                "measured_tcp_orientation_error_rad": rotations,
                "body_joint_delta_rad": body_delta,
                "locked_body_drift_rad": locked_body_drift if group == ARM_GROUP else None,
                "max_command_step_rad": peak_step,
                "retimed_seconds": duration_sum,
                "execution_seconds": time.monotonic() - beginning,
                "max_base_translation_drift_m": self.max_base_translation_drift,
                "max_base_rotation_drift_rad": self.max_base_rotation_drift,
            }
        )
        self.command_steps.append(peak_step)
        self.show_targets(prepared["targets"], measured)
        return result

    def return_home(self):
        start = copy.deepcopy(self.state)
        path, method, samples = self.plan_path(start, self.home, GROUP)
        return self.execute_prepared(
            {
                "start": start,
                "goal": self.home,
                "targets": self.reference,
                "group": GROUP,
                "path": path,
                "planning_method": method,
                "collision_samples": samples,
                "ik_error_m": [0.0, 0.0],
                "ik_orientation_error_rad": [0.0, 0.0],
                "solve_seconds": 0.0,
                "attempts": 0,
            }
        )

    def show_targets(self, targets, measured):
        markers = []
        for index, (target, actual) in enumerate(zip(targets, measured)):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.ns = "targets"
            marker.id = index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = target.pose
            marker.scale.x, marker.scale.y, marker.scale.z = 0.12, 0.018, 0.018
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.2, 0.8, 1.0, 1.0)
            markers.append(marker)
            position = actual.pose.position
            self.trails[index].append(Point(x=position.x, y=position.y, z=position.z))
            self.trails[index] = self.trails[index][-2500:]
            trail = Marker()
            trail.header.frame_id = "odom"
            trail.ns, trail.id, trail.type, trail.action = "actual_tcp", index, Marker.LINE_STRIP, Marker.ADD
            trail.pose.orientation.w = 1.0
            trail.scale.x = 0.006
            trail.color.r, trail.color.g, trail.color.b, trail.color.a = (1.0, 0.4 + 0.5 * index, 0.2, 1.0)
            trail.points = self.trails[index]
            markers.append(trail)
        self.marker_publisher.publish(MarkerArray(markers=markers))

    def release(self):
        if self.trace_file is not None:
            self.trace_file.close()
            self.trace_file = None
        if self.command_lock is not None:
            self.command_lock.close()
            self.command_lock = None


def save_json(path, report):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2) + "\n")


def timestamp():
    return datetime.now(timezone.utc).isoformat()
