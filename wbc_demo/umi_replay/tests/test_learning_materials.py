import unittest

import numpy as np

from umi_replay.learning_materials import planar_analytic, planar_fk, planar_iterative, planar_jacobian


class LearningExampleTests(unittest.TestCase):
    def test_both_analytic_branches_reach_same_target(self):
        target = np.array([0.6, 0.4])
        solutions = planar_analytic(target)
        self.assertGreater(solutions[0][1], 0)
        self.assertLess(solutions[1][1], 0)
        for solution in solutions:
            np.testing.assert_allclose(planar_fk(solution), target, atol=1e-12)

    def test_unreachable_target_is_rejected(self):
        with self.assertRaises(ValueError):
            planar_analytic([1.5, 0.0])

    def test_geometric_jacobian_matches_finite_difference(self):
        joints = np.radians([30.0, 60.0])
        derivative = planar_jacobian(joints)
        for index in range(2):
            shifted = joints.copy()
            shifted[index] += 1e-7
            np.testing.assert_allclose((planar_fk(shifted) - planar_fk(joints)) / 1e-7, derivative[:, index], atol=1e-7)

    def test_numerical_iteration_converges_with_analytic_jacobian(self):
        target = np.array([0.6, 0.4])
        history = planar_iterative(target, np.radians([-20.0, 80.0]))
        self.assertGreater(len(history), 2)
        self.assertLess(np.linalg.norm(planar_fk(history[-1]) - target), 1e-9)

    def test_failed_iteration_does_not_claim_unreachable(self):
        with self.assertRaisesRegex(RuntimeError, "not proof of unreachability"):
            planar_iterative([0.6, 0.4], np.radians([-20.0, 80.0]), max_iterations=0)
