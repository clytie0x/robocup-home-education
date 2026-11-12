import math
import unittest

import geometry
import lidar
import params

P = params.load()


class Geometry(unittest.TestCase):
    def test_base_and_odom_frames_round_trip(self):
        pose = (1.0, 2.0, math.pi / 2)
        p = geometry.base_to_odom((1.0, 0.0), pose)
        self.assertAlmostEqual(p[0], 1.0)
        self.assertAlmostEqual(p[1], 3.0)
        back = geometry.odom_to_base(p, pose)
        self.assertAlmostEqual(back[0], 1.0)
        self.assertAlmostEqual(back[1], 0.0)

    def test_lidar_point_is_shifted_ahead_by_lidar_x(self):
        x, y = geometry.lidar_to_base((1.0, 0.5), P)
        self.assertAlmostEqual(x, 1.0 + P["lidar_x"])
        self.assertAlmostEqual(y, 0.5)

    def test_polar_zero_is_front_and_left_is_positive(self):
        deg, metres = geometry.polar((0.0, 2.0))
        self.assertAlmostEqual(deg, 90.0)
        self.assertAlmostEqual(metres, 2.0)
        self.assertAlmostEqual(geometry.polar((3.0, 0.0))[0], 0.0)


class LidarUsable(unittest.TestCase):
    def test_body_point_is_dropped_and_far_point_kept(self):
        self.assertEqual(lidar.usable([(111.0, 0.18), (0.0, 2.0)]), [(0.0, 2.0)])


if __name__ == "__main__":
    unittest.main()
