import importlib.util
import json
import unittest
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from umi_replay.differential_ik import DifferentialIK, URDFKinematics
from umi_replay.geometry import pose_dict
from umi_replay.gripper_eef import SwiftpickerEEF
from umi_replay.pinocchio_backend import PinocchioKinematics

PACKAGE = Path(__file__).resolve().parents[3] / "source/geniesim_ros/src/ros_ws/src/genie_sim_robot_model"


@unittest.skipUnless(importlib.util.find_spec("pinocchio"), "Optional Pinocchio probe environment required")
class PinocchioBackendTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        urdf = PACKAGE / "urdf/genie_g2_crsB_swiftpicker.urdf"
        cls.eef = SwiftpickerEEF(urdf, PACKAGE)
        cls.names = [
            name
            for name in cls.eef.joints
            if any(part in name for part in ("_body_joint", "_arm_l_joint", "_arm_r_joint"))
        ] + cls.eef.master_names
        cls.positions = {name: 0.0 for name in cls.eef.joints}
        cls.positions.update(
            {
                "idx01_body_joint1": -0.4,
                "idx02_body_joint2": 0.6,
                "idx03_body_joint3": -0.3,
                "idx22_arm_l_joint2": -0.4,
                "idx24_arm_l_joint4": -0.7,
                "idx62_arm_r_joint2": 0.4,
                "idx64_arm_r_joint4": -0.7,
            }
        )
        cls.positions.update(cls.eef.opening_positions([0.4, 0.7]))
        cls.positions["idx33_gripper_l_left_support_joint"] += 0.002
        root = np.eye(4)
        root[:3, :3] = Rotation.from_euler("xyz", [0.2, -0.1, 0.3]).as_matrix()
        root[:3, 3] = [0.12, -0.06, 0.8]
        cls.original = URDFKinematics(cls.eef, root, cls.names)
        cls.candidate = PinocchioKinematics(cls.eef, root, cls.names, urdf)

    def test_poses_and_jacobians_match_with_measured_mimic_deflection(self):
        poses, jacobians = self.original.evaluate(self.positions)
        actual, derivatives = self.candidate.evaluate(self.positions)
        for expected, measured, expected_derivative, measured_derivative in zip(poses, actual, jacobians, derivatives):
            np.testing.assert_allclose(expected, measured, atol=1e-12)
            np.testing.assert_allclose(expected_derivative, measured_derivative, atol=2e-6)

    def test_joint_mapping_has_full_command_rank_and_positive_mass_matrix(self):
        self.assertEqual(np.linalg.matrix_rank(self.candidate.velocity_map), 21)
        self.assertGreater(np.linalg.eigvalsh(self.candidate.mass_matrix(self.positions)).min(), 0)

    def test_same_qp_can_use_pinocchio_without_losing_existing_constraints(self):
        limits = {
            name: (
                float(self.eef.joints[name].find("limit").get("lower")),
                float(self.eef.joints[name].find("limit").get("upper")),
            )
            for name in self.names
        }
        velocities = {name: float(self.eef.joints[name].find("limit").get("velocity")) for name in self.names}
        coupled = yaml.safe_load(
            (PACKAGE.parent / "genie_sim_moveit_plugins/config/coupled_constraints.yaml").read_text()
        )["coupled_constraints"]
        profile = json.loads((Path(__file__).resolve().parents[1] / "configs/g2_realtime_limits.json").read_text())
        poses, unused = self.original.evaluate(self.positions)
        targets = [pose_dict(pose) for pose in poses]
        targets[0]["position"][0] += 0.002
        results = []
        for model in (self.original, self.candidate):
            solver = DifferentialIK(model, self.positions, limits, velocities, coupled, profile)
            positions, velocity = solver.step(self.positions, targets, [0.42, 0.68], np.zeros(21), 1 / 30)
            results.append([positions[name] for name in self.names])
        np.testing.assert_allclose(results[0], results[1], atol=2e-5)

    def test_nonfinite_measured_position_is_rejected(self):
        positions = dict(self.positions)
        positions[self.names[0]] = float("nan")
        with self.assertRaises(ValueError):
            self.candidate.evaluate(positions)

    def test_missing_command_joint_is_rejected(self):
        positions = dict(self.positions)
        positions.pop(self.names[0])
        with self.assertRaisesRegex(ValueError, "Missing measured command joints"):
            self.candidate.evaluate(positions)
