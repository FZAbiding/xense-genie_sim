import copy
import math
import os
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

from lab_core import ARM_GROUP, GROUP, WholeBodyLab, active_joint, finite_targets, make_targets, segment_duration


class LabContracts(unittest.TestCase):
    def setUp(self):
        target = PoseStamped()
        target.header.frame_id = "odom"
        target.pose.orientation.w = 1.0
        target.pose.position.x = 0.5
        target.pose.position.y = 0.2
        target.pose.position.z = 1.0
        self.targets = [target, copy.deepcopy(target)]
        self.targets[1].pose.position.y = -0.2

    def test_offsets_do_not_mutate_reference(self):
        actual = make_targets(self.targets, {"left": [0.1, 0.2, -0.5]}, [0, 0, 0])
        self.assertAlmostEqual(actual[0].pose.position.z, 0.5)
        self.assertAlmostEqual(self.targets[0].pose.position.z, 1.0)
        self.assertAlmostEqual(actual[1].pose.position.x, 0.5)

    def test_orbit_rotates_position_and_orientation(self):
        actual = make_targets(self.targets, {"orbit_deg": 90}, [0, 0, 0])
        self.assertAlmostEqual(actual[0].pose.position.x, -0.2)
        self.assertAlmostEqual(actual[0].pose.position.y, 0.5)
        self.assertAlmostEqual(actual[0].pose.orientation.z, math.sqrt(0.5))

    def test_nonfinite_input_is_rejected(self):
        self.targets[0].pose.position.x = math.nan
        with self.assertRaises(ValueError):
            finite_targets(self.targets)

    def test_invalid_orientation_is_rejected(self):
        self.targets[0].pose.orientation.w = 0.0
        with self.assertRaises(ValueError):
            finite_targets(self.targets)

    def test_wrong_frame_is_rejected(self):
        self.targets[0].header.frame_id = "base_link"
        with self.assertRaises(ValueError):
            finite_targets(self.targets)

    def test_group_excludes_waist_and_chassis(self):
        self.assertTrue(active_joint("idx01_body_joint1", GROUP))
        self.assertFalse(active_joint("idx01_body_joint1", ARM_GROUP))
        self.assertTrue(active_joint("idx21_arm_l_joint1", ARM_GROUP))
        self.assertFalse(active_joint("planar_joint", GROUP))
        self.assertFalse(active_joint("idx11_head_joint1", GROUP))

    def test_retiming_bounds_cubic_profile(self):
        for delta in (0.01, 0.5, 2.5):
            duration = segment_duration(delta, 1.0, 0.3, 0.5)
            self.assertLessEqual(1.5 * delta / duration, 0.3 + 1e-12)
            self.assertLessEqual(6 * delta / duration**2, 0.5 + 1e-12)

    def test_padded_limit_is_projected_to_native_limit(self):
        model = SimpleNamespace(limits={"idx01_body_joint1": (-1.0, 0.0)})
        state = RobotState(joint_state=JointState(name=["idx01_body_joint1"], position=[0.005]))
        WholeBodyLab.clamp_limits(model, state, GROUP)
        self.assertEqual(state.joint_state.position[0], 0.0)

    def test_large_limit_violation_is_rejected(self):
        model = SimpleNamespace(limits={"idx01_body_joint1": (-1.0, 0.0)})
        state = RobotState(joint_state=JointState(name=["idx01_body_joint1"], position=[0.1]))
        with self.assertRaises(RuntimeError):
            WholeBodyLab.clamp_limits(model, state, GROUP)

    def test_control_lease_prevents_competing_labs(self):
        first = SimpleNamespace(command_lock=None)
        second = SimpleNamespace(command_lock=None)
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "unit-test-" + str(os.getpid())}):
            try:
                WholeBodyLab.acquire_control(first)
                with self.assertRaises(RuntimeError):
                    WholeBodyLab.acquire_control(second)
            finally:
                if first.command_lock is not None:
                    first.command_lock.close()


if __name__ == "__main__":
    unittest.main()
