"""The 18 Nov phantom: a return ~0.26 m away was the robot, and it stopped every cycle."""
import math
import unittest

import numpy as np

import depth_obstacles
import footprint
import params
import safety
from test_depth_obstacles import scene_depth

P = params.load()


class SelfBody(unittest.TestCase):
    def test_the_corner_return_stops_until_it_is_masked(self):
        point = np.array([[0.22, 0.17]])  # range 0.278 m, bearing ~38 deg: the arm, past the 0.18 m body box
        self.assertTrue(safety.must_stop(point, 0.1, 0.0, P))
        self.assertEqual(len(footprint.drop_self(point, P, sensor="lidar")), 0)
        self.assertEqual(footprint.LAST["lidar"]["nearest"]["sensor"], "lidar")
        self.assertGreater(footprint.LAST["lidar"]["count"], 0)
        self.assertFalse(safety.must_stop(footprint.drop_self(point, P), 0.1, 0.0, P))

    def test_straight_ahead_at_the_outline_is_the_bag_and_a_real_obstacle_is_kept(self):
        bag = footprint.drop_self([[0.26, 0.0]], P, sensor="depth")
        self.assertEqual(len(bag), 0)
        self.assertEqual(footprint.LAST["depth"]["nearest"]["sensor"], "depth")
        kept = footprint.drop_self([[0.40, 0.0], [0.325, 0.0], [0.0, 0.22]], P)
        self.assertEqual(len(kept), 3)  # past the bag, the braking strip, and 2 cm off the side
        self.assertTrue(safety.must_stop(np.array([[0.325, 0.0]]), 0.1, 0.0, P))

    def test_a_configured_polygon_masks_only_inside_it(self):
        p = dict(P, self_mask={"skin_m": 0.0, "sectors": [], "polygons": [[[0.5, -0.1], [0.7, -0.1], [0.7, 0.1], [0.5, 0.1]]]})
        kept = footprint.drop_self([[0.6, 0.0], [0.9, 0.0]], p)
        self.assertEqual(kept.tolist(), [[0.9, 0.0]])

    def test_stop_cause_names_the_sensor_and_the_bearing(self):
        cause = safety.stop_cause([[0.45, 0.2]], [[0.30, 0.0]], 0.1, 0.0, P)
        self.assertEqual(cause["sensor"], "depth")
        self.assertTrue(cause["blocking"])
        self.assertLess(abs(cause["bearing_deg"]), 1.0)
        other = safety.nearest_return([[2.0, 2.0]], None)
        self.assertEqual(other["sensor"], "lidar")
        self.assertFalse(other["blocking"])

    def test_depth_of_the_carried_bag_is_dropped_and_a_box_ahead_is_not(self):
        p = dict(P)
        close = depth_obstacles.points(scene_depth(p, [(0.26, -0.04, 0.04, 0.05, 0.4)]), p)
        self.assertEqual(len(close), 0)
        bare = dict(p, self_mask={"skin_m": 0.0, "sectors": [], "polygons": []})
        self.assertGreater(len(depth_obstacles.points(scene_depth(bare, [(0.26, -0.04, 0.04, 0.05, 0.4)]), bare)), 0)
        box = depth_obstacles.points(scene_depth(p, [(1.2, -0.1, 0.1, 0.0, 0.4)]), p)
        self.assertGreater(len(box), 0)
        self.assertTrue(np.all(np.abs(box[:, 0] - 1.2) < 0.15))
