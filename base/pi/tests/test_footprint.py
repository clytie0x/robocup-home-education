import math
import unittest

import numpy

import footprint


class BodyGapTest(unittest.TestCase):
    def test_a_point_at_the_cut_corner(self):
        gap = footprint.body_gap(numpy.array([[0.25, 0.25]]), 0, 0, 0)[0]
        self.assertAlmostEqual(gap, math.dist((0.25, 0.25), (0.18, 0.18)), places=3)

    def test_beside_a_side_and_inside(self):
        gaps = footprint.body_gap(numpy.array([[0.0, 0.25], [0.0, -0.30], [0.0, 0.0], [0.19, 0.0]]), 0, 0, 0)
        numpy.testing.assert_allclose(gaps, [0.05, 0.10, -0.20, -0.01], atol=1e-9)

    def test_the_pose_moves_and_turns_the_outline(self):
        # robot at (1, 0.3) facing +y: the point (1, 0) is 0.3 m behind its centre, 0.10 m behind its back
        self.assertAlmostEqual(footprint.body_gap([(1.0, 0.0)], 1.0, 0.3, math.pi / 2)[0], 0.10, places=9)


class SweepTest(unittest.TestCase):
    def test_no_poses_and_no_points_are_clear(self):
        self.assertTrue(footprint.sweep_clear(numpy.empty((0, 2)), [(0, 0, 0.5)]))
        self.assertTrue(footprint.sweep_clear([(0.3, 0.0)], []))

    def test_a_point_the_outline_reaches_is_not_clear(self):
        self.assertFalse(footprint.sweep_clear([(0.30, 0.0)], [(0.05, 0, 0), (0.08, 0, 0)]))
        self.assertTrue(footprint.sweep_clear([(0.30, 0.0)], [(0.05, 0, 0)]))  # 0.05 m left, over the margin

    def test_a_point_already_inside_the_margin_only_blocks_getting_closer(self):
        wall = [(x, -0.21) for x in numpy.arange(-0.6, 0.15, 0.02)]  # 1 cm beside the right side, ending at its front
        self.assertTrue(footprint.sweep_clear(wall, [(0.1, 0, 0), (0.2, 0, 0)]))  # along it
        self.assertFalse(footprint.sweep_clear(wall, [(0, -0.005, 0)]))  # into it


if __name__ == "__main__":
    unittest.main()
