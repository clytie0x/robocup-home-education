import unittest

import numpy

import params
import safety

P = params.load()
Q = dict(P, decel=0.5, stop_delay=0.2, robot_half_width=0.20)
# At v = 0.4 the braking distance is 0.39, so the strip is 0.20 < x < 0.59, |y| < 0.25.


class SafetyTest(unittest.TestCase):
    def test_braking_distance_formula(self):
        self.assertAlmostEqual(safety.braking_distance(0.4, Q), 0.16 + 0.08 + 0.15)

    def test_braking_distance_refuses_unmeasured_numbers(self):
        with self.assertRaises(ValueError):
            safety.braking_distance(0.4, dict(P, decel=None, stop_delay=0.2))
        with self.assertRaises(ValueError):
            safety.braking_distance(0.4, dict(P, decel=0.5, stop_delay=None))

    def test_stops_for_a_point_inside_the_braking_strip(self):
        self.assertTrue(safety.must_stop([(0.4, 0.0)], 0.4, 0.0, Q))
        self.assertTrue(safety.must_stop(numpy.array([[0.5, -0.2], [3.0, 0.0]]), 0.4, 0.0, Q))

    def test_ignores_a_point_beside_the_strip_or_behind(self):
        for pt in [(0.4, 0.30), (0.4, -0.30), (-0.4, 0.0), (0.10, 0.0), (0.8, 0.0)]:
            self.assertFalse(safety.must_stop([pt], 0.4, 0.0, Q), pt)
        self.assertFalse(safety.must_stop(numpy.empty((0, 2)), 0.4, 0.0, Q))

    def test_turning_checks_both_sides(self):
        for w in (0.3, -0.3):
            for pt in [(0.2, 0.2), (0.2, -0.2), (0.0, -0.27), (0.0, 0.27)]:
                self.assertTrue(safety.must_stop([pt], 0.0, w, Q), (w, pt))
        self.assertFalse(safety.must_stop([(0.2, 0.3)], 0.0, 0.3, Q))  # 0.36 m, outside

    def test_rolling_and_turning_checks_the_disc_too(self):
        self.assertTrue(safety.must_stop([(0.2, 0.26)], 0.01, 0.49, Q))
        self.assertFalse(safety.must_stop([(0.2, 0.26)], 0.01, 0.0, Q))

    def test_nan_means_stop(self):
        nan = float("nan")
        self.assertTrue(safety.must_stop([(nan, 0.0)], 0.4, 0.0, Q))
        self.assertTrue(safety.must_stop([], nan, 0.0, Q))
        self.assertTrue(safety.must_stop([], 0.0, nan, Q))

    def test_standing_still_never_stops(self):
        self.assertFalse(safety.must_stop([(0.3, 0.0)], 0.0, 0.0, Q))

    def test_backward_speed_is_an_error(self):
        with self.assertRaises(ValueError):
            safety.must_stop([(0.3, 0.0)], -0.1, 0.0, Q)

    def test_no_depth_lowers_the_speed_cap(self):
        self.assertEqual(safety.speed_cap(None, dict(P, max_speed=0.5)), 0.3)
        self.assertEqual(safety.speed_cap(numpy.empty((0, 2)), dict(P, max_speed=0.5)), 0.5)


if __name__ == "__main__":
    unittest.main()
