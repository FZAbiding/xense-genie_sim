import json
import copy
import unittest
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from umi_replay.differential_ik import DifferentialIK, PlaybackClock, URDFKinematics, matrix_error
from umi_replay.geometry import pose_dict
from umi_replay.gripper_eef import SwiftpickerEEF

PACKAGE = Path(__file__).resolve().parents[3] / "source/geniesim_ros/src/ros_ws/src/genie_sim_robot_model"


class ClockTests(unittest.TestCase):
    def test_stalls_skip_time_instead_of_slowing_source(self):
        clock = PlaybackClock(100, 4)
        self.assertAlmostEqual(clock.phase(100.03), 0.03)
        self.assertAlmostEqual(clock.phase(101.5), 1.5)
        self.assertEqual(clock.phase(105), 4)

    def test_pause_preserves_source_phase_without_counting_paused_time(self):
        clock = PlaybackClock(100, 4)
        clock.pause(101, True)
        clock.pause(102, True)
        self.assertEqual(clock.phase(103), 1)
        clock.pause(103, False)
        self.assertEqual(clock.phase(104), 2)
        clock.pause(104, False)
        self.assertEqual(clock.pause_seconds, 2)


@unittest.skipUnless((PACKAGE / "urdf/genie_g2_crsB_swiftpicker.urdf").exists(), "G2 model required")
class DifferentialIKTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.eef = SwiftpickerEEF(PACKAGE / "urdf/genie_g2_crsB_swiftpicker.urdf", PACKAGE)
        cls.names = [
            name
            for name in cls.eef.joints
            if any(part in name for part in ("_body_joint", "_arm_l_joint", "_arm_r_joint"))
        ] + cls.eef.master_names
        cls.model = URDFKinematics(cls.eef, np.eye(4), cls.names)
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
        cls.limits = {
            name: (
                float(cls.eef.joints[name].find("limit").get("lower")),
                float(cls.eef.joints[name].find("limit").get("upper")),
            )
            for name in cls.names
        }
        cls.velocities = {name: float(cls.eef.joints[name].find("limit").get("velocity")) for name in cls.names}
        cls.profile = json.loads((Path(__file__).resolve().parents[1] / "configs/g2_realtime_limits.json").read_text())
        cls.coupled = yaml.safe_load(
            (PACKAGE.parent / "genie_sim_moveit_plugins/config/coupled_constraints.yaml").read_text()
        )["coupled_constraints"]

    def test_analytic_jacobians_agree_with_finite_differences_including_dynamic_tcp(self):
        poses, jacobians = self.model.evaluate(self.positions)
        for index, name in enumerate(self.names):
            shifted = dict(self.positions)
            shifted[name] += 1e-6
            for follower, joint in self.eef.joints.items():
                if joint.find("mimic") is not None:
                    shifted.pop(follower, None)
            moved, unused = self.model.evaluate(shifted, jacobian=False)
            for actual, target, jacobian in zip(poses, moved, jacobians):
                np.testing.assert_allclose(matrix_error(actual, target) / 1e-6, jacobian[:, index], atol=2e-6)

    def test_joint_and_coupled_limits_and_acceleration_are_enforced(self):
        solver = DifferentialIK(self.model, self.positions, self.limits, self.velocities, self.coupled, self.profile)
        poses, unused = self.model.evaluate(self.positions)
        targets = [pose_dict(pose) for pose in poses]
        targets[0]["position"][0] += 0.1
        previous = np.zeros(len(self.names))
        result, velocity = solver.step(self.positions, targets, [0.6, 0.3], previous, 1 / 30)
        values = np.array([result[name] for name in self.names])
        self.assertTrue(np.all(np.abs(velocity) <= solver.velocity + 1e-5))
        self.assertTrue(np.all(np.abs(velocity - previous) <= solver.acceleration / 30 + 1e-5))
        self.assertTrue(np.all(values >= solver.lower - 1e-6))
        self.assertTrue(np.all(values <= solver.upper + 1e-6))
        self.assertTrue(np.all(solver.coupled @ values <= solver.ceiling + 1e-6))

    def test_nonfinite_profile_velocity_is_not_masked_by_native_limit(self):
        for value in (float("nan"), float("inf")):
            profile = copy.deepcopy(self.profile)
            profile["arm"]["velocity_rad_s"] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                DifferentialIK(self.model, self.positions, self.limits, self.velocities, self.coupled, profile)

    def test_infeasible_braking_state_is_rejected_not_silently_clipped(self):
        positions = dict(self.positions)
        joint = "idx01_body_joint1"
        positions[joint] = self.limits[joint][1]
        solver = DifferentialIK(self.model, positions, self.limits, self.velocities, self.coupled, self.profile)
        previous = np.zeros(len(self.names))
        previous[self.names.index(joint)] = self.profile["waist"]["acceleration_rad_s2"] / 30 + 0.000672977154299842
        poses, unused = self.model.evaluate(positions, jacobian=False)
        with self.assertRaisesRegex(RuntimeError, "bounds are infeasible"):
            solver.step(
                positions,
                [pose_dict(pose) for pose in poses],
                self.eef.openings_from_positions(positions),
                previous,
                1 / 30,
            )

    def test_gripper_jacobian_preserves_measured_mimic_deflection(self):
        positions = dict(self.positions)
        positions["idx33_gripper_l_left_support_joint"] += 0.002
        master = self.eef.master_names[0]
        poses, jacobians = self.model.evaluate(positions)
        shifted = dict(positions)
        shifted[master] += 1e-6
        for name, joint in self.eef.joints.items():
            mimic = joint.find("mimic")
            if mimic is not None and mimic.get("joint") == master:
                shifted[name] += 1e-6 * float(mimic.get("multiplier", "1"))
        moved, unused = self.model.evaluate(shifted, jacobian=False)
        np.testing.assert_allclose(
            matrix_error(poses[0], moved[0]) / 1e-6, jacobians[0][:, self.names.index(master)], atol=2e-6
        )

    def test_roll_bound_is_preserved_and_gripper_mimics_follow_master(self):
        solver = DifferentialIK(
            self.model,
            self.positions,
            self.limits,
            self.velocities,
            self.coupled,
            self.profile,
            {"idx04_body_joint4": [-0.0087, 0.0087]},
        )
        poses, unused = self.model.evaluate(self.positions)
        for pose in poses:
            pose[:3, :3] = Rotation.from_euler("x", 0.2).as_matrix() @ pose[:3, :3]
        result, velocity = solver.step(
            self.positions, [pose_dict(pose) for pose in poses], [0, 1], np.zeros(len(self.names)), 1 / 30
        )
        self.assertLessEqual(abs(result["idx04_body_joint4"]), 0.008701)
        self.assertAlmostEqual(result["idx34_gripper_l_inner_joint2"], -result["idx31_gripper_l_inner_joint1"])


if __name__ == "__main__":
    unittest.main()
