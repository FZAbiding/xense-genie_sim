import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from umi_replay.geometry import pose_dict
from umi_replay.realtime_execution import checked_step, prepare_initial, sample_targets


class RealtimeExecutionTests(unittest.TestCase):
    def test_targets_interpolate_both_pose_and_normalized_opening(self):
        first = pose_dict(np.eye(4))
        second = pose_dict(np.eye(4))
        second["position"][0] = 0.1
        frames = [
            {"timestamp": 0.0, "eef_odom": [first, first], "gripper": [0.0, 1.0]},
            {"timestamp": 0.1, "eef_odom": [second, second], "gripper": [1.0, 0.0]},
        ]
        poses, openings = sample_targets(frames, 0.05)
        np.testing.assert_allclose(poses[0]["position"], [0.05, 0, 0])
        np.testing.assert_allclose(openings, [0.5, 0.5])
        poses, openings = sample_targets(frames, 1.0)
        np.testing.assert_allclose(openings, [1.0, 0.0])

    def test_initial_approach_retries_without_falling_back_to_unverified_rrt(self):
        node = SimpleNamespace(
            cancel=SimpleNamespace(is_set=lambda: False),
            home=object(),
            prepare=Mock(return_value={"path": [object(), object()]}),
            validate_path=Mock(side_effect=[RuntimeError("collision"), 21]),
        )
        frame = {"eef_odom": [pose_dict(np.eye(4))] * 2, "gripper": [0.5, 0.5]}
        prepared = prepare_initial(node, frame)
        self.assertEqual(node.prepare.call_count, 2)
        self.assertFalse(node.prepare.call_args.kwargs["plan"])
        self.assertEqual(prepared["collision_samples"], 21)

    def test_collision_check_rejects_invalid_candidate(self):
        future = Mock()
        future.done.return_value = True
        future.result.return_value = SimpleNamespace(valid=False)
        node = SimpleNamespace(
            interpolate=Mock(return_value=object()),
            validate_constraints=Mock(),
            validity=SimpleNamespace(call_async=Mock(return_value=future)),
        )
        with patch("umi_replay.realtime_execution.rclpy.spin_until_future_complete"):
            with self.assertRaisesRegex(RuntimeError, "validity guard"):
                checked_step(node, object(), object(), 100.0)

    def test_collision_timeout_cancels_future_and_never_accepts_stale_result(self):
        future = Mock()
        future.done.return_value = False
        node = SimpleNamespace(
            interpolate=Mock(return_value=object()),
            validate_constraints=Mock(),
            validity=SimpleNamespace(call_async=Mock(return_value=future)),
        )
        with patch("umi_replay.realtime_execution.rclpy.spin_until_future_complete"):
            with self.assertRaises(TimeoutError):
                checked_step(node, object(), object(), 100.0)
        future.cancel.assert_called_once()


if __name__ == "__main__":
    unittest.main()
