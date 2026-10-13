import argparse
import json
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from transforms3d.quaternions import mat2quat


class CameraRecorder(Node):
    def __init__(self, output, name="physx-demo", scene="scene_stress.yaml"):
        super().__init__("fixed_base_camera_recorder")
        self.output = output
        self.name = name
        self.image_prefix = "physx" if name == "physx-demo" else name
        self.writer = None
        self.frames = 0
        self.received_frames = 0
        self.latest_frame = None
        self.ready_after = time.monotonic() + 2.0
        self.camera_pose = self.create_publisher(
            PoseStamped,
            "/genie_sim_engine/viewer/camera_pose",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        config = yaml.safe_load((Path(__file__).parent / scene).read_text())["viewer_camera"]
        position = np.array(config["pos"], dtype=float)
        backward = position - np.array(config["lookat"], dtype=float)
        backward /= np.linalg.norm(backward)
        right = np.cross([0, 0, 1], backward)
        right /= np.linalg.norm(right)
        up = np.cross(backward, right)
        quaternion = mat2quat(np.column_stack([right, up, backward]))
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position.tolist()
        (
            pose.pose.orientation.w,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
        ) = quaternion.tolist()
        self.camera_pose.publish(pose)
        self.create_subscription(Image, "/genie_sim/free_camera_rgb/image_raw", self.on_image, qos_profile_sensor_data)

    def on_image(self, message):
        if time.monotonic() < self.ready_after:
            return
        conversions = {
            "rgb8": (3, cv2.COLOR_RGB2BGR),
            "rgba8": (4, cv2.COLOR_RGBA2BGR),
            "bgr8": (3, None),
            "bgra8": (4, cv2.COLOR_BGRA2BGR),
        }
        if message.encoding not in conversions:
            raise RuntimeError(f"Unsupported camera encoding: {message.encoding}")
        channels, conversion = conversions[message.encoding]
        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        frame = rows[:, : message.width * channels].reshape(message.height, message.width, channels)
        if conversion is not None:
            frame = cv2.cvtColor(frame, conversion)
        if self.writer is None:
            self.writer = cv2.VideoWriter(
                str(self.output / (self.name + ".mp4")),
                cv2.VideoWriter_fourcc(*"mp4v"),
                30,
                (message.width, message.height),
            )
            if not self.writer.isOpened():
                raise RuntimeError("Cannot create camera recording")
            cv2.imwrite(str(self.output / (self.image_prefix + "-start.png")), frame)
            print(f"Camera ready: {message.width}x{message.height}, {message.encoding}", flush=True)
        self.latest_frame = frame
        self.received_frames += 1

    def capture(self):
        if self.latest_frame is None:
            return
        self.writer.write(self.latest_frame)
        self.frames += 1
        if self.frames % 30 == 0:
            cv2.imwrite(str(self.output / (self.image_prefix + "-latest.png")), self.latest_frame)


def main():
    parser = argparse.ArgumentParser(description="Record the simulator's rendered camera, not the desktop")
    parser.add_argument("--seconds", type=float, default=90.0)
    parser.add_argument("--output-name", default="physx-demo")
    parser.add_argument("--scene", default="scene_stress.yaml")
    args = parser.parse_args()
    if "/" in args.output_name or args.output_name in (".", "..") or args.seconds <= 0:
        parser.error("Use a simple output name and positive duration")
    output = Path(__file__).parent / "logs"
    output.mkdir(exist_ok=True)
    os.environ.setdefault("ROS_DOMAIN_ID", "42")
    rclpy.init()
    node = CameraRecorder(output, args.output_name, args.scene)
    beginning = time.monotonic()
    next_capture = beginning
    try:
        while time.monotonic() - beginning < args.seconds:
            rclpy.spin_once(node, timeout_sec=max(0.0, min(0.01, next_capture - time.monotonic())))
            if time.monotonic() >= next_capture:
                node.capture()
                next_capture = max(next_capture + 1.0 / 30.0, time.monotonic())
        if node.frames == 0:
            raise RuntimeError("No rendered camera frames received")
    finally:
        if node.writer is not None:
            node.writer.release()
        node.destroy_node()
        rclpy.shutdown()
    report = {
        "topic": "/genie_sim/free_camera_rgb/image_raw",
        "frames": node.frames,
        "received_frames": node.received_frames,
        "elapsed_seconds": time.monotonic() - beginning,
        "video_fps": 30,
    }
    metadata = "camera-recording.json" if args.output_name == "physx-demo" else args.output_name + ".json"
    (output / metadata).write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
