import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from umi_replay.conversion import select_keyframes
from umi_replay.geometry import (
    check_transform,
    interpolate_pose,
    map_poses,
    pose_dict,
    pose_error,
    pose_matrix,
    rotation_6d,
    transform,
)


class GeometryTests(unittest.TestCase):
    def test_rotation_uses_columns_not_rows(self):
        matrix = Rotation.from_euler("xyz", [0.4, -0.7, 1.1]).as_matrix()
        decoded = rotation_6d(np.r_[matrix[:, 0], matrix[:, 1]])
        np.testing.assert_allclose(decoded, matrix, atol=1e-12)

    def test_bad_rotation_rejected(self):
        for values in ([0] * 6, [1, 0, 0, 1, 0, 0], [1, 0, 0, 0, float("nan"), 0]):
            with self.assertRaises(ValueError):
                rotation_6d(values)

    def test_reflection_rejected(self):
        matrix = np.eye(4)
        matrix[0, 0] = -1
        with self.assertRaises(ValueError):
            check_transform(matrix)

    def test_world_left_tool_right_inverse(self):
        world = transform([1, 2, 3], Rotation.from_euler("z", 0.6).as_quat())
        tool = transform([0, 0, 0.2], Rotation.from_euler("x", 0.4).as_quat())
        source = transform([0.3, -0.1, 0.2], Rotation.from_euler("y", -0.8).as_quat())
        mapped = map_poses([pose_dict(source)] * 2, world, [tool] * 2)
        np.testing.assert_allclose(pose_matrix(mapped[0]) @ tool, world @ source, atol=1e-12)

    def test_shared_world_preserves_bimanual_distance(self):
        first = pose_dict(transform([1, 2, 0], [0, 0, 0, 1]))
        second = pose_dict(transform([-1, 1, 0], [0, 0, 0, 1]))
        world = transform([3, -2, 1], Rotation.from_euler("z", 0.8).as_quat())
        mapped = map_poses([first, second], world, [np.eye(4), np.eye(4)])
        self.assertAlmostEqual(pose_error(first, second)[0], pose_error(*mapped)[0])

    def test_slerp_handles_quaternion_sign(self):
        pose = {"position": [0, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]}
        opposite = {**pose, "quaternion_xyzw": [0, 0, 0, -1]}
        self.assertAlmostEqual(pose_error(pose, interpolate_pose(pose, opposite, 0.5))[1], 0)

    def test_sampling_keeps_nonlinear_peak_and_endpoints(self):
        frames = []
        for index in range(31):
            pose = {"position": [0.04 if index == 7 else 0, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]}
            frames.append({"timestamp": index / 30, "eef": [pose, pose]})
        selected = select_keyframes(frames)
        self.assertEqual(selected[0], 0)
        self.assertEqual(selected[-1], 30)
        self.assertIn(7, selected)
        for start, end in zip(selected, selected[1:]):
            self.assertLessEqual(frames[end]["timestamp"] - frames[start]["timestamp"], 0.20001)
            for index in range(start, end + 1):
                predicted = interpolate_pose(
                    frames[start]["eef"][0], frames[end]["eef"][0], (index - start) / (end - start)
                )
                self.assertLessEqual(pose_error(predicted, frames[index]["eef"][0])[0], 0.005001)

    def test_zero_sampling_interval_rejected(self):
        with self.assertRaises(ValueError):
            select_keyframes([], max_seconds=0)

    def test_sampling_keeps_gripper_pulse_when_eef_is_stationary(self):
        pose = pose_dict(np.eye(4))
        frames = [
            {"timestamp": index / 30, "eef": [pose, pose], "gripper": [0 if index == 7 else 1, 1]}
            for index in range(31)
        ]
        selected = select_keyframes(frames)
        self.assertIn(7, selected)
        for start, end in zip(selected, selected[1:]):
            for index in range(start, end + 1):
                fraction = (index - start) / (end - start)
                predicted = (1 - fraction) * np.array(frames[start]["gripper"]) + fraction * np.array(
                    frames[end]["gripper"]
                )
                self.assertLessEqual(float(np.max(abs(predicted - frames[index]["gripper"]))), 0.020001)


if __name__ == "__main__":
    unittest.main()
