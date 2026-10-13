import argparse
import copy
import csv
import json
import math
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, MultiDOFJointState
from tf2_ros import Buffer, TransformException, TransformListener

TIPS = ["arm_l_end_link", "arm_r_end_link"]
GROUP = "wbc_fixed_headless"


def position_error(actual, target):
    return math.dist(
        [actual.position.x, actual.position.y, actual.position.z],
        [target.position.x, target.position.y, target.position.z],
    )


def orientation_error(actual, target):
    actual_values = [actual.x, actual.y, actual.z, actual.w]
    target_values = [target.x, target.y, target.z, target.w]
    scale = math.sqrt(sum(value * value for value in actual_values) * sum(value * value for value in target_values))
    cosine = abs(sum(first * second for first, second in zip(actual_values, target_values))) / scale
    return 2.0 * math.acos(min(1.0, cosine))


class DualArmDemo(Node):
    def __init__(self, group=GROUP):
        super().__init__("fixed_base_dual_arm_ik_demo")
        self.group = group
        self.state = None
        self.last_state_time = 0.0
        self.report = None
        self.trace_file = None
        self.trace_writer = None
        self.base_initial_transform = None
        self.max_base_translation_drift = 0.0
        self.max_base_rotation_drift = 0.0
        self.create_subscription(JointState, "/moveit/joint_states", self.on_state, 10)
        self.command = self.create_publisher(JointState, "/joint_command", qos_profile_sensor_data)
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.validity = self.create_client(GetStateValidity, "/check_state_validity")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_state(self, message):
        self.state = RobotState(joint_state=message)
        try:
            base = self.tf_buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time())
            self.state.multi_dof_joint_state = MultiDOFJointState(
                header=base.header, joint_names=["planar_joint"], transforms=[base.transform]
            )
        except TransformException:
            pass
        self.last_state_time = time.monotonic()
        if self.trace_file is not None:
            if self.trace_writer is None:
                self.trace_writer = csv.writer(self.trace_file)
                self.trace_writer.writerow(["wall_time_s", *message.name])
            self.trace_writer.writerow([time.time(), *message.position])

    def save_report(self):
        Path(__file__).with_name("latest_report.json").write_text(json.dumps(self.report, indent=2) + "\n")

    def observe_base_drift(self):
        if self.base_initial_transform is None:
            return
        transform = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time()).transform
        translation = math.dist(
            [transform.translation.x, transform.translation.y, transform.translation.z],
            [
                self.base_initial_transform.translation.x,
                self.base_initial_transform.translation.y,
                self.base_initial_transform.translation.z,
            ],
        )
        rotation = orientation_error(transform.rotation, self.base_initial_transform.rotation)
        self.max_base_translation_drift = max(self.max_base_translation_drift, translation)
        self.max_base_rotation_drift = max(self.max_base_rotation_drift, rotation)
        if translation > 0.001 or rotation > 0.001:
            raise RuntimeError(f"Fixed base moved: translation={translation} m, rotation={rotation} rad")

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError(f"Service unavailable: {client.srv_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"Service failed: {client.srv_name}")
        return future.result()

    def wait_for_state(self):
        deadline = time.monotonic() + 300.0
        while rclpy.ok() and time.monotonic() < deadline:
            if getattr(self, "cancel", None) is not None and self.cancel.is_set():
                raise RuntimeError("Cancelled while waiting for simulator readiness")
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.state is not None
                and time.monotonic() - self.last_state_time < 1.0
                and self.command.get_subscription_count() > 0
                and self.state.multi_dof_joint_state.joint_names == ["planar_joint"]
                and self.tf_buffer.can_transform("odom", "base_link", rclpy.time.Time())
            ):
                positions = dict(zip(self.state.joint_state.name, self.state.joint_state.position))
                height = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time()).transform.translation.z
                if abs(positions.get("base_footprint_to_base_link", math.inf) - height) < 0.0001:
                    return
        raise RuntimeError("Simulator joint states or /joint_command subscriber unavailable")

    def forward(self, state, frame="odom"):
        request = GetPositionFK.Request()
        request.header.frame_id = frame
        request.fk_link_names = TIPS
        request.robot_state = state
        response = self.call(self.fk, request)
        if response.error_code.val != 1:
            raise RuntimeError(f"FK failed: {response.error_code.val}")
        return response.pose_stamped

    def solve(self, state, targets):
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group
        request.ik_request.robot_state = state
        request.ik_request.avoid_collisions = True
        request.ik_request.ik_link_names = TIPS
        request.ik_request.pose_stamped_vector = targets
        request.ik_request.timeout.sec = 2
        response = self.call(self.ik, request)
        if response.error_code.val != 1:
            raise RuntimeError(f"Dual-arm IK failed: {response.error_code.val}")
        initial = dict(zip(state.joint_state.name, state.joint_state.position))
        result = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        for name, position in initial.items():
            active = any(part in name for part in ("_arm_l_joint", "_arm_r_joint")) or (
                "_body_joint" in name and self.group != "wbc_fixed_arms"
            )
            if not active and abs(result.get(name, position) - position) > 1e-7:
                raise RuntimeError(f"IK unexpectedly moved an excluded joint: {name}")
        solution = copy.deepcopy(state)
        solution.joint_state.position = [result.get(name, initial[name]) for name in solution.joint_state.name]
        return solution

    def base_position(self):
        try:
            transform = self.tf_buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time())
        except TransformException as error:
            raise RuntimeError("Fixed-base transform unavailable") from error
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return [translation.x, translation.y, yaw]

    def interpolate(self, start, goal, fraction):
        state = copy.deepcopy(start)
        state.joint_state.position = [
            initial + fraction * (target - initial)
            for initial, target in zip(start.joint_state.position, goal.joint_state.position)
        ]
        return state

    def execute(self, start, goal, seconds):
        if self.count_publishers("/joint_command") > 1:
            raise RuntimeError("Multiple joint-command publishers; refusing competing execution")
        for sample in range(21):
            request = GetStateValidity.Request()
            request.group_name = self.group
            request.robot_state = self.interpolate(start, goal, sample / 20.0)
            if not self.call(self.validity, request).valid:
                raise RuntimeError(f"Self-collision detected along joint interpolation at sample {sample}")
        beginning = time.monotonic()
        while rclpy.ok():
            fraction = min((time.monotonic() - beginning) / seconds, 1.0)
            blend = fraction * fraction * (3.0 - 2.0 * fraction)
            command = self.interpolate(start, goal, blend).joint_state
            active = [
                (name, position)
                for name, position in zip(command.name, command.position)
                if any(part in name for part in ("_body_joint", "_arm_l_joint", "_arm_r_joint"))
            ]
            command.name = [name for name, position in active]
            command.position = [position for name, position in active]
            command.header.stamp = self.get_clock().now().to_msg()
            command.velocity = []
            command.effort = []
            self.command.publish(command)
            rclpy.spin_once(self, timeout_sec=0.02)
            self.observe_base_drift()
            if time.monotonic() - self.last_state_time > 2.0:
                raise RuntimeError("Simulator joint-state stream stopped")
            if fraction >= 1.0:
                break
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self.command.publish(command)
            rclpy.spin_once(self, timeout_sec=0.02)
            self.observe_base_drift()

    def run_demo(self, seconds, cycles):
        self.report = {
            "status": "starting",
            "started_at": datetime.now(timezone.utc).isoformat(),
            "group": GROUP,
            "tips": TIPS,
            "waypoints": [],
        }
        self.save_report()
        self.wait_for_state()
        initial = copy.deepcopy(self.state)
        reference = self.forward(initial)
        base_initial = self.base_position()
        self.base_initial_transform = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time()).transform
        self.report["status"] = "running"
        self.report["base_initial_xy_yaw"] = base_initial
        self.report["target_frame"] = "odom"
        trace_path = Path(__file__).parent / "logs" / "joint_trace.csv"
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        self.trace_file = trace_path.open("w", newline="")
        for cycle in range(cycles):
            for offset in [(0.08, 0.0, -0.08), (0.16, 0.0, -0.16), (0.08, 0.0, -0.08), (0.0, 0.0, 0.0)]:
                targets = copy.deepcopy(reference)
                for target in targets:
                    target.pose.position.x += offset[0]
                    target.pose.position.y += offset[1]
                    target.pose.position.z += offset[2]
                start = copy.deepcopy(self.state)
                solution = self.solve(start, targets)
                predicted = self.forward(solution)
                ik_errors = [position_error(actual.pose, target.pose) for actual, target in zip(predicted, targets)]
                ik_rotation_errors = [
                    orientation_error(actual.pose.orientation, target.pose.orientation)
                    for actual, target in zip(predicted, targets)
                ]
                if max(ik_errors) > 0.005:
                    raise RuntimeError(f"IK FK residual exceeds 5 mm: {ik_errors}")
                if max(ik_rotation_errors) > math.radians(1.0):
                    raise RuntimeError(f"IK orientation residual exceeds 1 degree: {ik_rotation_errors}")
                self.execute(start, solution, seconds)
                measured = self.forward(copy.deepcopy(self.state))
                measured_errors = [
                    position_error(actual.pose, target.pose) for actual, target in zip(measured, targets)
                ]
                measured_rotation_errors = [
                    orientation_error(actual.pose.orientation, target.pose.orientation)
                    for actual, target in zip(measured, targets)
                ]
                initial_positions = dict(zip(initial.joint_state.name, initial.joint_state.position))
                body_delta = {
                    name: position - initial_positions[name]
                    for name, position in zip(self.state.joint_state.name, self.state.joint_state.position)
                    if "_body_joint" in name
                }
                base = self.base_position()
                base_drift = math.dist(base[:2], base_initial[:2])
                yaw_drift = abs(math.remainder(base[2] - base_initial[2], 2.0 * math.pi))
                waypoint = {
                    "cycle": cycle + 1,
                    "tcp_offset_m": offset,
                    "ik_error_m": ik_errors,
                    "ik_orientation_error_rad": ik_rotation_errors,
                    "measured_tcp_error_m": measured_errors,
                    "measured_tcp_orientation_error_rad": measured_rotation_errors,
                    "body_joint_delta_rad": body_delta,
                    "base_xy_drift_m": base_drift,
                    "base_yaw_drift_rad": yaw_drift,
                    "max_base_translation_drift_m": self.max_base_translation_drift,
                    "max_base_rotation_drift_rad": self.max_base_rotation_drift,
                }
                self.report["waypoints"].append(waypoint)
                print(json.dumps(waypoint), flush=True)
                self.save_report()
                self.trace_file.flush()
                if base_drift > 0.001 or yaw_drift > 0.001:
                    raise RuntimeError("Base moved despite fixed-base configuration")
                if max(measured_errors) > 0.02:
                    raise RuntimeError(f"Measured TCP tracking error exceeds 2 cm: {measured_errors}")
                if max(measured_rotation_errors) > math.radians(3.0):
                    raise RuntimeError(f"Measured TCP orientation error exceeds 3 degrees: {measured_rotation_errors}")
        if not any(
            abs(delta) > 0.001
            for waypoint in self.report["waypoints"]
            for delta in waypoint["body_joint_delta_rad"].values()
        ):
            raise RuntimeError("No measurable waist/body motion; demo did not exercise whole-body IK")
        self.report["status"] = "passed"
        self.report["finished_at"] = datetime.now(timezone.utc).isoformat()
        self.save_report()
        print("PASS: both TCPs tracked, waist/body moved, chassis remained fixed.", flush=True)

    def run_ik_probe(self):
        model = Path(get_package_share_directory("genie_sim_robot_model")) / "urdf/genie_g2_crsB_swiftpicker.urdf"
        robot = ET.parse(model).getroot()
        config = yaml.safe_load(Path(__file__).with_name("scene_fixed.yaml").read_text())
        initial = config["robot"]["init_joint_pos"]
        names = [joint.attrib["name"] for joint in robot.findall("joint") if joint.attrib["type"] != "fixed"]
        state = RobotState(
            joint_state=JointState(name=names, position=[math.radians(initial.get(name, 0.0)) for name in names])
        )
        targets = self.forward(state, frame="odom")
        for target in targets:
            target.pose.position.x += 0.08
            target.pose.position.z -= 0.08
        solution = self.solve(state, targets)
        actual = self.forward(solution, frame="odom")
        residuals = [position_error(pose.pose, target.pose) for pose, target in zip(actual, targets)]
        body = {
            name: position
            for name, position in zip(solution.joint_state.name, solution.joint_state.position)
            if "_body_joint" in name
        }
        report = {
            "mode": "kinematics_only_no_physics_or_execution",
            "group": GROUP,
            "tips": TIPS,
            "tcp_offset_m": [0.08, 0.0, -0.08],
            "fk_position_residual_m": residuals,
            "body_joint_solution_rad": body,
        }
        print(json.dumps(report, indent=2), flush=True)
        Path(__file__).with_name("ik_probe_report.json").write_text(json.dumps(report, indent=2) + "\n")
        if max(residuals) > 0.005 or max(abs(value) for value in body.values()) < 0.001:
            raise RuntimeError("IK probe failed: excessive TCP residual or no waist/body motion")
        print("PASS: kinematic dual-arm + waist IK only; no physics simulation was performed.", flush=True)


def main():
    parser = argparse.ArgumentParser(description="Simulation-only fixed-base G2 dual-arm + waist IK demo")
    parser.add_argument("--segment-seconds", type=float, default=4.0)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--ik-only", action="store_true", help="Validate IK/FK without physics or joint commands")
    args = parser.parse_args()
    if args.segment_seconds < 1.0 or args.cycles < 1:
        parser.error("Use segment-seconds >= 1 and cycles >= 1")
    rclpy.init()
    node = DualArmDemo()
    try:
        if args.ik_only:
            node.run_ik_probe()
        else:
            node.run_demo(args.segment_seconds, args.cycles)
    except Exception as error:
        if node.report is not None:
            node.report["status"] = "failed"
            node.report["error"] = str(error)
            node.report["finished_at"] = datetime.now(timezone.utc).isoformat()
            node.save_report()
        raise
    finally:
        if node.trace_file is not None:
            node.trace_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
