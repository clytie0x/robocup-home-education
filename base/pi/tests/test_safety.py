import math
import unittest

import numpy

import footprint
import params
import safety

P = params.load()
Q = dict(P, decel=0.5, stop_delay=0.2, robot_half_width=0.20)
# At v = 0.4 the base runs on 0.24 after S#; with the 0.30 clearance the strip is 0.20 < x < 0.74, |y| < 0.25.


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

    def test_always_leaves_the_clearance_ahead_even_when_creeping(self):
        for v in (P["max_speed"], 0.04, 0.01):  # the robot's measured decel and stop_delay
            edge = safety.FRONT + safety.braking_distance(v, P, margin=0.0) + safety.CLEARANCE
            self.assertTrue(safety.must_stop([(edge - 0.01, 0.0)], v, 0.0, P), v)
            self.assertFalse(safety.must_stop([(edge + 0.01, 0.0)], v, 0.0, P), v)

    def test_the_operator_at_the_follow_gap_does_not_stop_the_robot_at_top_speed(self):
        # Fails if max_speed is raised before braking is measured again: the clearance would
        # then reach past the follow gap and the robot would stop behind every operator.
        operator = safety.FRONT + P["follow_gap"]
        self.assertFalse(safety.must_stop([(operator, 0.0)], P["max_speed"], 0.0, P))

    def test_turning_into_a_point_beside_the_front_stops_turning_away_does_not(self):
        # 0.06 m beside the left side; the front-left corner swings toward it when turning left
        self.assertTrue(safety.must_stop([(0.08, 0.26)], 0.0, 0.49, P))
        self.assertFalse(safety.must_stop([(0.08, 0.26)], 0.0, -0.49, P))

    def test_a_wall_3_cm_beside_the_body_stops_any_turn_on_the_spot(self):
        # the base turns about its centre: one corner swings toward the wall either way
        wall = [(x, -0.23) for x in numpy.arange(-0.4, 0.19, 0.02)]
        for w in (0.3, -0.3):
            self.assertTrue(safety.must_stop(wall, 0.0, w, P), w)
        self.assertFalse(safety.must_stop(wall, P["max_speed"], 0.0, P))  # driving straight along it

    def test_turning_in_the_open_is_not_stopped_by_a_point_the_outline_never_reaches(self):
        # inside the old 0.33 m spin circle, 0.10 m outside the side: the turn is allowed now
        for w in (0.3, -0.3):
            self.assertFalse(safety.must_stop([(0.0, 0.30)], 0.0, w, P), w)

    def test_a_point_entering_the_swept_outline_stops(self):
        self.assertTrue(safety.must_stop([(0.35, 0.10)], 0.10, 0.4, P))

    def test_rolling_and_turning_sweeps_the_tail(self):
        # turning left swings the rear-right corner out to the right
        self.assertTrue(safety.must_stop([(-0.08, -0.26)], 0.01, 0.49, P))
        self.assertFalse(safety.must_stop([(-0.08, -0.26)], 0.01, 0.0, P))

    def test_a_point_in_a_cut_front_corner_stops_driving_straight(self):
        self.assertTrue(safety.must_stop([(0.19, 0.19)], P["max_speed"], 0.0, P))

    def test_a_slow_turn_cannot_creep_into_a_wall_one_cycle_at_a_time(self):
        wall = numpy.array([(x, -0.231) for x in numpy.arange(-0.6, 0.6, 0.01)])
        for w in (0.012, 0.02, 0.03, 0.05):
            th = 0.0
            for _ in range(400):  # 40 s of 0.1 s cycles, turning while nothing stops it
                c, s = math.cos(th), math.sin(th)
                local = numpy.c_[c * wall[:, 0] + s * wall[:, 1], -s * wall[:, 0] + c * wall[:, 1]]
                if safety.must_stop(local, 0.0, w, P):
                    break
                th += w * 0.1
            self.assertGreater(footprint.body_gap(local, 0, 0, 0).min(), 0.02, w)

    def test_a_longer_cycle_sweeps_further(self):
        # 0.27 m beside the centre: a corner (0.256 m out) reaches it only after turning about 50 degrees
        self.assertFalse(safety.must_stop([(0.0, 0.27)], 0.0, 0.49, P))
        self.assertTrue(safety.must_stop([(0.0, 0.27)], 0.0, 0.49, P, cycle=1.5))

    def test_unmeasured_turn_braking_is_an_error(self):
        with self.assertRaises(ValueError):
            safety.must_stop([(0.0, 0.30)], 0.0, 0.3, dict(P, turn_accel=None))

    def test_the_base_rolls_on_after_a_stop_and_that_is_checked(self):
        coast = safety.Coast(P)
        coast.sent(0.09, 0.0, 0.0)
        # 0.1 s later the brain asks to turn on the spot: the roll forward is checked with it
        self.assertEqual(coast.motions(0.0, 0.49, 0.1), [(0.09, 0.49)])
        self.assertTrue(any(safety.must_stop([(0.27, 0.10)], v, w, P) for v, w in coast.motions(0.0, 0.49, 0.1)))
        self.assertFalse(safety.must_stop([(0.27, 0.10)], 0.0, 0.49, P))  # the command alone passes
        # it keeps its speed for stop_delay, then slows at decel until it stands
        self.assertEqual(coast.motions(0.0, 0.0, 0.27), [(0.09, 0.0)])
        self.assertAlmostEqual(coast.motions(0.0, 0.0, 0.27 + 1.0)[0][0], 0.09 - P["decel"])
        self.assertEqual(coast.motions(0.0, 0.0, 10.0), [(0.0, 0.0)])

    def test_a_turn_that_flips_direction_still_checks_the_old_one(self):
        coast = safety.Coast(P)
        coast.sent(0.0, 0.49, 0.0)
        coast.sent(0.0, -0.49, 0.1)
        self.assertEqual(coast.motions(0.0, -0.49, 0.2), [(0.0, -0.49), (0.0, 0.49)])
        self.assertEqual(coast.motions(0.0, -0.49, 5.0), [(0.0, -0.49)])  # died away

    def test_unmeasured_braking_is_refused_when_the_run_starts(self):
        with self.assertRaises(ValueError):
            safety.Coast(dict(P, turn_accel=None))

    def test_nan_in_the_command_is_kept_so_it_stops(self):
        coast = safety.Coast(P)
        coast.sent(0.0, 0.2, 0.0)
        nan = float("nan")
        self.assertTrue(any(safety.must_stop([], v, w, P) for v, w in coast.motions(0.05, nan, 0.1)))
        self.assertTrue(any(safety.must_stop([], v, w, P) for v, w in coast.motions(nan, 0.0, 0.1)))

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



def wall_behind(x):
    return [(x, y) for y in numpy.arange(-0.30, 0.31, 0.02)]


class RearClearTest(unittest.TestCase):
    """Backing up straight: the back edge is at x = -0.20; GRID adds 7 mm to the margin."""

    def test_a_wall_far_behind_is_clear_a_near_one_is_not(self):
        self.assertTrue(safety.rear_clear(wall_behind(-0.20 - 0.20 - 0.10 - 0.06), 0.04, P, 0.20))
        self.assertFalse(safety.rear_clear(wall_behind(-0.20 - 0.20 - 0.10 + 0.02), 0.04, P, 0.20))
        self.assertFalse(safety.rear_clear(wall_behind(-0.25), 0.04, P, 0.0))  # 5 cm behind: not even a start

    def test_the_run_on_after_s_is_included(self):
        run_on = 0.04 * (safety.CYCLE + P["stop_delay"]) + 0.04 ** 2 / (2 * P["decel"])  # about 0.027 m
        self.assertGreater(run_on, 0.02)
        wall = wall_behind(-0.20 - 0.20 - 0.10 - 0.015)  # clear of the 0.20 m alone, not of 0.20 m + run-on
        self.assertTrue(footprint.sweep_clear(wall, [(-0.20, 0.0, 0.0)], 0.10))
        self.assertFalse(safety.rear_clear(wall, 0.04, P, 0.20))
        self.assertTrue(safety.rear_clear(wall, 0.04, dict(P, decel=100.0, stop_delay=0.0), 0.19))  # run-on only the last cycle

    def test_the_margin_is_a_parameter(self):
        wall = wall_behind(-0.20 - 0.10 - 0.15)
        self.assertTrue(safety.rear_clear(wall, 0.04, P, 0.10))
        self.assertFalse(safety.rear_clear(wall, 0.04, P, 0.10, margin=0.20))

    def test_points_beside_or_ahead_do_not_count(self):
        beside = [(x, s * 0.25) for x in numpy.arange(-0.14, 0.15, 0.02) for s in (1, -1)]  # 5 cm off each side
        ahead = [(0.25, 0.0), (0.5, 0.1)]
        self.assertTrue(safety.rear_clear(beside + ahead, 0.04, P, 0.30))
        self.assertFalse(safety.rear_clear(beside + ahead + [(-0.45, 0.0)], 0.04, P, 0.30))

    def test_nothing_seen_is_clear_nan_is_not(self):
        self.assertTrue(safety.rear_clear(numpy.empty((0, 2)), 0.04, P, 0.30))
        self.assertFalse(safety.rear_clear([(float("nan"), 0.0)], 0.04, P, 0.30))
        self.assertFalse(safety.rear_clear([(float("-inf"), 0.0)], 0.04, P, 0.30))

    def test_bad_speed_or_distance_is_an_error(self):
        for speed, distance in ((0.0, 0.1), (-0.04, 0.1), (float("nan"), 0.1), (0.04, -0.1), (0.04, float("inf"))):
            with self.assertRaises(ValueError):
                safety.rear_clear([], speed, P, distance)

    def test_must_stop_still_refuses_backward(self):
        with self.assertRaises(ValueError):
            safety.must_stop([], -0.04, 0.0, P)

if __name__ == "__main__":
    unittest.main()
