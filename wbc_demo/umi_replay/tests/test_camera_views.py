import unittest

import numpy as np

from umi_replay.camera_views import GRAPHICS_IN_HEAD, ego_camera, fixed_render_root, look_at
from umi_replay.geometry import pose_dict, pose_matrix
from umi_replay.synchronized_video import compose_frame


class CameraViewsTests(unittest.TestCase):
    def test_render_root_matches_pinned_physics_root_including_yaw_radians(self):
        scene = {
            "robot": {
                "robot_prefix": "genie",
                "robot_source": {"pin_base_to_world": True},
                "init_base_pose": {"x": -0.25, "z": 0.04, "theta": np.pi / 2},
            }
        }
        root = fixed_render_root(scene)
        self.assertEqual(root["prim_path"], "/genie")
        matrix = pose_matrix(root["pose"])
        np.testing.assert_allclose(matrix[:3, 3], [-0.25, 0, 0.04])
        np.testing.assert_allclose(matrix[:3, 0], [0, 1, 0], atol=1e-12)

    def test_render_root_rejects_moving_base(self):
        with self.assertRaisesRegex(ValueError, "pinned"):
            fixed_render_root({"robot": {"robot_source": {"pin_base_to_world": False}}})

    def test_ego_forward_matches_head_forward_not_external_front(self):
        track = [{"timestamp": index, "pose": pose_dict(np.eye(4))} for index in range(2)]
        camera = ego_camera(track, 0.5, np.eye(4))
        np.testing.assert_allclose(-camera[:3, 2], [1, 0, 0])
        np.testing.assert_allclose(camera[:3, 0], [0, -1, 0])
        np.testing.assert_allclose(camera[:3, 1], [0, 0, 1])
        self.assertAlmostEqual(np.linalg.det(GRAPHICS_IN_HEAD), 1)

    def test_ego_applies_same_world_transform_as_eef(self):
        world = np.eye(4)
        world[:3, 3] = [0.2, -0.1, 1.7]
        track = [{"timestamp": index, "pose": pose_dict(np.eye(4))} for index in range(2)]
        np.testing.assert_allclose(ego_camera(track, 1, world)[:3, 3], world[:3, 3])

    def test_look_at_graphics_negative_z_faces_target(self):
        camera = look_at([2, 1, 1], [0, 0, 0.5])
        forward = -camera[:3, 2]
        expected = np.array([-2, -1, -0.5])
        np.testing.assert_allclose(forward, expected / np.linalg.norm(expected))
        self.assertAlmostEqual(np.linalg.det(camera[:3, :3]), 1)

    def test_quad_layout_places_ego_next_to_source_without_mirroring(self):
        source = np.full((480, 640, 3), [11, 22, 33], dtype=np.uint8)
        views = np.hstack([np.full_like(source, value) for value in (40, 80, 120)])
        result = compose_frame(source, views, "quad", "test")
        self.assertEqual(result.shape, (1064, 1280, 3))
        np.testing.assert_array_equal(result[200, 200], [11, 22, 33])
        np.testing.assert_array_equal(result[200, 900], [40, 40, 40])
        np.testing.assert_array_equal(result[700, 200], [80, 80, 80])
        np.testing.assert_array_equal(result[700, 900], [120, 120, 120])

    def test_quad_rejects_single_external_camera_recording(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        with self.assertRaisesRegex(ValueError, "three-camera"):
            compose_frame(image, image, "quad", "invalid")


if __name__ == "__main__":
    unittest.main()
