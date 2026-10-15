import math
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from umi_replay.execution import posture_config, retime, run


class PosturePolicyTests(unittest.TestCase):
    def setUp(self):
        self.config = {"joint_position_bounds_rad": {"idx04_body_joint4": [-0.1, 0.1], "other_joint": [-1, 1]}}

    def test_soft_removes_only_roll_boundary_without_mutating_input(self):
        result = posture_config(self.config, "soft")
        self.assertEqual(result["joint_position_bounds_rad"], {"other_joint": [-1, 1]})
        self.assertEqual(self.config["joint_position_bounds_rad"]["idx04_body_joint4"], [-0.1, 0.1])
        self.assertNotIn(
            "joint_position_bounds_rad",
            posture_config({"joint_position_bounds_rad": {"idx04_body_joint4": [-0.1, 0.1]}}, "soft"),
        )

    def test_upright_keeps_other_boundaries(self):
        result = posture_config(self.config, "upright")
        self.assertEqual(result["joint_position_bounds_rad"]["other_joint"], [-1, 1])
        self.assertEqual(
            result["joint_position_bounds_rad"]["idx04_body_joint4"], [-math.radians(0.5), math.radians(0.5)]
        )

    def test_config_is_preserved_and_invalid_policy_rejected(self):
        self.assertEqual(posture_config(self.config, "config"), self.config)
        with self.assertRaises(ValueError):
            posture_config(self.config, "invalid")


class RetimeTests(unittest.TestCase):
    def test_cubic_joint_limits_determine_global_time_scale(self):
        name = "idx21_arm_l_joint1"
        node = SimpleNamespace(max_velocity=0.3, max_acceleration=0.5, velocities={name: 1.0})
        start = SimpleNamespace(joint_state=SimpleNamespace(name=[name], position=[0.0]))
        goal = SimpleNamespace(joint_state=SimpleNamespace(name=[name], position=[0.1]))
        frames = [{"timestamp": 0}, {"timestamp": 0.2}]
        scale = retime(node, [start, goal], frames, 0.5)
        duration = 0.2 * scale
        self.assertLessEqual(1.5 * 0.1 / duration, 0.3)
        self.assertLessEqual(6 * 0.1 / duration**2, 0.500001)
        self.assertGreaterEqual(scale, 2)

    def test_invalid_speed_rejected(self):
        for speed in [0, -1, 1.1, math.nan, math.inf]:
            with self.assertRaises(ValueError):
                retime(None, [], [], speed)


class EarlyRealtimeValidationTests(unittest.TestCase):
    def test_invalid_profile_is_rejected_before_ros_or_motion(self):
        pose = {"position": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]}
        converted = {
            "schema": "xense-umi-eef-v1",
            "frame": "dataset_world",
            "provenance": {},
            "sampling": {"max_interval_seconds": 0.2},
            "source_frames": [
                {"timestamp": index / 30, "frame_index": index, "eef": [pose, pose], "gripper": [0.5, 0.5]}
                for index in range(2)
            ],
        }
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            source = directory / "input.json"
            source.write_text(json.dumps(converted))
            profile = directory / "bad-profile.json"
            profile.write_text("{}")
            args = SimpleNamespace(
                input=source,
                output=directory / "run",
                speed=1.0,
                execute=False,
                playback_mode="realtime",
                posture_policy="soft",
                limits_profile=profile,
            )
            config = {
                "max_joint_velocity_rad_s": 0.3,
                "max_joint_acceleration_rad_s2": 0.5,
                "max_tracking_error_m": 0.05,
                "max_tracking_rotation_rad": 0.17,
            }
            with patch("umi_replay.execution.rclpy.init", side_effect=AssertionError("ROS initialized too early")):
                with self.assertRaises(ValueError):
                    run(args, converted, config)


if __name__ == "__main__":
    unittest.main()
