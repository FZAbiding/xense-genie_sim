import unittest

import numpy as np

from umi_replay.synchronized_video import match_source_frames


class SourceVideoAlignmentTests(unittest.TestCase):
    def setUp(self):
        self.commands = [
            {"source_time_s": phase, "command_ros_time_s": timestamp}
            for phase, timestamp in ((0, 100), (1, 110), (2, 120))
        ]
        self.camera = [
            {"video_frame": index, "image_ros_time_s": timestamp}
            for index, timestamp in enumerate((99.98, 100.02, 105.02, 110.02, 115.02, 120.02))
        ]

    def test_aligns_by_source_phase_not_video_duration_ratio(self):
        indexes, requested, measured, gaps = match_source_frames(
            np.array([0.0, 0.5, 1.0, 2.0]), self.commands, self.camera, 0.1
        )
        np.testing.assert_array_equal(indexes, [0, 2, 3, 5])
        np.testing.assert_allclose(requested, [100, 105, 110, 120])
        np.testing.assert_allclose(gaps, abs(measured - requested))

    def test_rejects_recorder_gap(self):
        with self.assertRaisesRegex(ValueError, "without gaps"):
            match_source_frames(np.array([0, 0.5, 2]), self.commands, self.camera[:2] + self.camera[-1:], 0.4)

    def test_rejects_incomplete_source_coverage(self):
        with self.assertRaisesRegex(ValueError, "complete source episode"):
            match_source_frames(np.array([0, 3]), self.commands, self.camera, 0.4)

    def test_rejects_camera_without_timestamps(self):
        camera = [{"video_frame": index, "image_ros_time_s": 0} for index in range(5)]
        with self.assertRaisesRegex(ValueError, "Invalid ROS timestamps"):
            match_source_frames(np.array([0, 2]), self.commands, camera, 0.4)

    def test_rejects_backward_source_phases(self):
        self.commands[1]["source_time_s"] = 3
        with self.assertRaisesRegex(ValueError, "monotonic"):
            match_source_frames(np.array([0, 2]), self.commands, self.camera, 0.4)

    def test_duplicate_camera_frames_do_not_change_image_time(self):
        camera = [{"video_frame": index, "image_ros_time_s": 100 + index // 2} for index in range(42)]
        indexes, requested, measured, gaps = match_source_frames(np.array([0, 1, 2]), self.commands, camera, 0.1)
        np.testing.assert_allclose(requested, measured)
        self.assertEqual(gaps.max(), 0)
        self.assertTrue(np.all(np.diff(indexes) >= 0))


if __name__ == "__main__":
    unittest.main()
