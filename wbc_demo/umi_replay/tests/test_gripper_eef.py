import copy
import unittest
import xml.etree.ElementTree as element_tree
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from umi_replay.conversion import mapped_eef_frames
from umi_replay.execution import alignment
from umi_replay.geometry import pose_dict, pose_matrix, transform
from umi_replay.gripper_eef import SwiftpickerEEF
from umi_replay.robot import GripperEEFLab, stamped_poses
from lab_core import WholeBodyLab, pose_values

PACKAGE = Path(__file__).resolve().parents[3] / "source/geniesim_ros/src/ros_ws/src/genie_sim_robot_model"


@unittest.skipUnless((PACKAGE / "urdf/genie_g2_crsB_swiftpicker.urdf").exists(), "Build/download the G2 model first")
class GripperEEFTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = SwiftpickerEEF(PACKAGE / "urdf/genie_g2_crsB_swiftpicker.urdf", PACKAGE)

    def test_tcp_is_between_mesh_fingertips_not_at_flange(self):
        for side in self.model.evidence["sides"]:
            centers = [item["distal_face_center_in_base_m"] for item in side["meshes"]]
            np.testing.assert_allclose(np.array(side["base_to_eef"])[:3, 3], np.mean(centers, axis=0))
        for offset in self.model.tip_to_eef:
            self.assertAlmostEqual(offset[2, 3], 0.2120057335, places=8)
            np.testing.assert_allclose(
                offset[:3, :3],
                Rotation.from_euler("y", -90, degrees=True).as_matrix()
                @ Rotation.from_euler("x", 180, degrees=True).as_matrix(),
                atol=1e-12,
            )

    def test_tcp_ik_fk_roundtrip_uses_rotation_of_offset(self):
        targets = [pose_dict(transform([0.4, 0.1, 1.0], Rotation.from_euler("xyz", [0.4, -0.6, 0.2]).as_quat()))] * 2
        flanges = self.model.to_flange(targets)
        restored = self.model.to_eef(flanges)
        for wanted, actual, flange in zip(targets, restored, flanges):
            np.testing.assert_allclose(pose_matrix(wanted), pose_matrix(actual), atol=1e-12)
            self.assertGreater(np.linalg.norm(np.array(wanted["position"]) - flange["position"]), 0.21)

    def test_eef_x_is_physical_tool_axis(self):
        target = pose_dict(np.eye(4))
        flange = self.model.to_flange([target, target])[0]
        self.assertAlmostEqual(flange["position"][0], -0.2120057335, places=8)
        self.assertAlmostEqual(flange["position"][2], -self.model.tip_to_eef[0][0, 3], places=12)

    def test_camera_housing_is_above_the_eef_not_below(self):
        for side in self.model.evidence["sides"]:
            self.assertGreater(side["camera_housing"]["bbox_center_in_eef_m"][2], 0.03)
            self.assertAlmostEqual(np.linalg.det(np.array(side["base_to_eef"])[:3, :3]), 1.0)

    def test_tool_transform_does_not_fit_the_dataset(self):
        node = SimpleNamespace(eef=self.model)
        source = {"source_frames": [{"eef": [pose_dict(np.eye(4)), pose_dict(np.eye(4))]}]}
        config = {"mode": "shared_world_gripper_eef", "initial_midpoint_odom_m": [0.3, 0, 1.05], "world_yaw_deg": 0}
        first_world, first_tools = alignment(node, source, config)
        changed = copy.deepcopy(source)
        changed["source_frames"][0]["eef"][0] = pose_dict(
            transform([0.1, 0.2, 0.3], Rotation.from_euler("x", 1).as_quat())
        )
        second_world, second_tools = alignment(node, changed, config)
        for first, second in zip(first_tools, second_tools):
            np.testing.assert_array_equal(first, second)
        self.assertFalse(np.array_equal(first_world, second_world))

    def test_world_mapping_has_no_independent_hand_rotation(self):
        source_pose = pose_dict(transform([0.1, -0.1, 0.3], Rotation.from_euler("z", 0.4).as_quat()))
        source = {"source_frames": [{"eef": [source_pose, source_pose]}], "keyframe_indexes": [0]}
        world = transform([1, 2, 3], Rotation.from_euler("z", -0.2).as_quat())
        mapped = mapped_eef_frames(source, world)[0]["eef_odom"]
        for pose in mapped:
            np.testing.assert_allclose(pose_matrix(pose), world @ pose_matrix(source_pose), atol=1e-12)

    def test_old_fitted_tool_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            alignment(SimpleNamespace(eef=self.model), {}, {"mode": "shared_world_virtual_tools"})

    def test_robot_adapter_uses_eef_for_both_ik_and_fk(self):
        node = object.__new__(GripperEEFLab)
        node.eef = self.model
        node.limits = {}
        state = SimpleNamespace(joint_state=SimpleNamespace(name=[], position=[]))
        targets = stamped_poses([pose_dict(np.eye(4))] * 2)
        with patch.object(WholeBodyLab, "solve", return_value=state) as backend:
            node.solve(state, targets)
            flanges = backend.call_args.args[1]
            self.assertLess(flanges[0].pose.position.x, -0.21)
        with patch.object(WholeBodyLab, "forward", return_value=flanges):
            restored = pose_values(node.forward(state))
        np.testing.assert_allclose(pose_matrix(restored[0]), np.eye(4), atol=1e-12)

    def test_closed_gripper_is_supported_but_native_limit_violation_is_rejected(self):
        node = SimpleNamespace(limits={"idx31_gripper_l_inner_joint1": (-0.91, 0)})
        state = SimpleNamespace(joint_state=SimpleNamespace(name=["idx31_gripper_l_inner_joint1"], position=[-0.03]))
        GripperEEFLab.check_gripper_reference(node, state)
        state.joint_state.position = [-1.0]
        with self.assertRaises(RuntimeError):
            GripperEEFLab.check_gripper_reference(node, state)

    def test_normalized_opening_and_mimic_signs(self):
        closed = self.model.opening_positions([0, 0])
        opened = self.model.opening_positions([1, 1])
        self.assertEqual(closed[self.model.master_names[0]], -0.91)
        self.assertEqual(opened[self.model.master_names[0]], 0.0)
        self.assertEqual(closed["idx34_gripper_l_inner_joint2"], 0.91)
        np.testing.assert_allclose(
            self.model.openings_from_positions(self.model.opening_positions([0.2, 0.8])), [0.2, 0.8]
        )

    def test_tcp_changes_with_opening_and_roundtrip_still_matches(self):
        positions = self.model.opening_positions([0, 0.5])
        dynamic = self.model.offsets_at(positions)
        self.assertGreater(dynamic[0][2, 3] - self.model.tip_to_eef[0][2, 3], 0.017)
        targets = [pose_dict(np.eye(4))] * 2
        flanges = self.model.to_flange(targets, positions)
        for pose in self.model.to_eef(flanges, positions):
            np.testing.assert_allclose(pose_matrix(pose), np.eye(4), atol=1e-12)

    def test_invalid_normalized_opening_is_not_silently_clamped(self):
        for values in ([0, 1.1], [-0.1, 1], [float("nan"), 0], [0]):
            with self.assertRaises(ValueError):
                self.model.opening_positions(values)

    def test_setting_opening_keeps_input_unchanged_and_updates_followers(self):
        node = object.__new__(GripperEEFLab)
        node.eef = self.model
        names = list(self.model.opening_positions([1, 1]))
        state = SimpleNamespace(joint_state=SimpleNamespace(name=names, position=[0.0] * len(names)))
        result = node.with_openings(state, [0.25, 0.75])
        self.assertEqual(state.joint_state.position, [0.0] * len(names))
        positions = dict(zip(result.joint_state.name, result.joint_state.position))
        self.assertAlmostEqual(positions["idx31_gripper_l_inner_joint1"], -0.6825)
        self.assertAlmostEqual(positions["idx34_gripper_l_inner_joint2"], 0.6825)
        self.assertFalse(node.is_command_joint("idx34_gripper_l_inner_joint2"))
        self.assertTrue(node.is_command_joint("idx31_gripper_l_inner_joint1"))

    def test_srdf_allows_only_each_grippers_intended_opposing_finger_contact(self):
        path = PACKAGE.parent / "genie_sim_moveit/config/srdf_grippers/swiftpicker.srdf.xacro"
        root = element_tree.parse(path).getroot()
        pairs = {frozenset((item.get("link1"), item.get("link2"))) for item in root.findall("disable_collisions")}
        for side in ("l", "r"):
            self.assertIn(frozenset((f"gripper_{side}_left_support_link", f"gripper_{side}_right_support_link")), pairs)
        self.assertNotIn(frozenset(("gripper_l_left_support_link", "gripper_r_right_support_link")), pairs)

    def test_upright_bounds_are_sent_to_ik_and_checked(self):
        node = object.__new__(GripperEEFLab)
        node.eef = self.model
        node.limits = {}
        node.joint_position_bounds = {"idx04_body_joint4": [-0.01, 0.01]}
        state = SimpleNamespace(joint_state=SimpleNamespace(name=["idx04_body_joint4"], position=[0.0]))
        targets = stamped_poses([pose_dict(np.eye(4))] * 2)
        with patch.object(WholeBodyLab, "solve", return_value=state) as backend:
            node.solve(state, targets)
            bound = backend.call_args.kwargs["joint_constraints"][0]
            self.assertEqual(bound.joint_name, "idx04_body_joint4")
            self.assertEqual(bound.position, 0)
            self.assertEqual(bound.tolerance_above, 0.01)
        state.joint_state.position = [0.1]
        with patch.object(WholeBodyLab, "solve", return_value=state):
            with self.assertRaisesRegex(RuntimeError, "outside the requested bound"):
                node.solve(state, targets)


if __name__ == "__main__":
    unittest.main()
