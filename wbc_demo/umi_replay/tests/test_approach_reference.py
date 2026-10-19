import copy
import unittest

from umi_replay.approach_reference import validate_reference


class ApproachReferenceTests(unittest.TestCase):
    def setUp(self):
        self.config = {"mode": "calibrated", "max_tracking_error_m": 0.05}
        self.reference = {
            "status": "passed",
            "input_sha256": "same",
            "config": self.config,
            "initial_approach_goal_rad": {"joint": 0.1},
        }

    def test_same_input_and_config_can_reuse_recorded_goal(self):
        original = copy.deepcopy(self.reference)
        self.assertEqual(validate_reference(self.reference, "same", self.config), original)
        self.assertEqual(self.reference, original)

    def test_different_episode_is_rejected(self):
        with self.assertRaises(ValueError):
            validate_reference(self.reference, "other", self.config)

    def test_different_world_or_safety_settings_are_rejected(self):
        with self.assertRaises(ValueError):
            validate_reference(self.reference, "same", {"mode": "calibrated", "max_tracking_error_m": 0.1})

    def test_failed_or_nonfinite_reference_is_rejected(self):
        for changes in ({"status": "failed"}, {"initial_approach_goal_rad": {"joint": float("nan")}}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                validate_reference({**self.reference, **changes}, "same", self.config)
