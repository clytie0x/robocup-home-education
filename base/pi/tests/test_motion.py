import math
import unittest

import motion
import params

P = params.load()
SCAN_A = [(float(d), 2.0) for d in range(-180, 180)]  # two different scans (their contents do not matter here)
SCAN_B = [(float(d), 2.1) for d in range(-180, 180)]


def fake_match(step, agree):
    calls = []

    def match(before, after, p, guess):
        calls.append(guess)
        return step + (agree,)
    match.calls = calls
    return match


class MotionTest(unittest.TestCase):
    def test_a_trusted_scan_match_corrects_the_odometry_step(self):
        est = motion.MotionEstimator(P, match=fake_match((0.012, 0.0, 0.0), 0.9))
        est.update(0.0, (0.0, 0.0, 0.0), SCAN_A)
        state = est.update(0.1, (0.010, 0.0, 0.0), SCAN_B)
        self.assertAlmostEqual(state.pose[0], 0.012, delta=0.001)
        self.assertTrue(state.trusted)
        self.assertAlmostEqual(state.v, 0.12, delta=0.01)
        self.assertEqual(est.match.calls[-1], (0.010, 0.0, 0.0))  # the odometry step is the guess

    def test_a_poor_match_leaves_the_odometry(self):
        est = motion.MotionEstimator(P, match=fake_match((0.012, 0.0, 0.0), 0.4))
        est.update(0.0, (0.0, 0.0, 0.0), SCAN_A)
        state = est.update(0.1, (0.010, 0.0, 0.0), SCAN_B)
        self.assertAlmostEqual(state.pose[0], 0.010, places=9)
        self.assertFalse(state.trusted)
        self.assertEqual(state.agree, 0.4)

    def test_a_match_far_from_the_odometry_is_refused(self):
        for step in ((0.07, 0.0, 0.0), (0.01, 0.0, math.radians(6))):
            est = motion.MotionEstimator(P, match=fake_match(step, 0.95))
            est.update(0.0, (0.0, 0.0, 0.0), SCAN_A)
            state = est.update(0.1, (0.010, 0.0, 0.0), SCAN_B)
            self.assertAlmostEqual(state.pose[0], 0.010, places=9)
            self.assertFalse(state.trusted)
            self.assertEqual(est.refused, 1)

    def test_the_same_scan_again_is_not_matched(self):
        match = fake_match((0.012, 0.0, 0.0), 0.9)
        est = motion.MotionEstimator(P, match=match)
        est.update(0.0, (0.0, 0.0, 0.0), SCAN_A)
        state = est.update(0.05, (0.005, 0.0, 0.0), SCAN_A)  # the LiDAR has not turned once yet
        self.assertEqual(match.calls, [])
        self.assertAlmostEqual(state.pose[0], 0.005, places=9)

    def test_steps_are_taken_in_the_robot_frame(self):
        # facing +y, the odometry says 0.01 forward; the match says 0.012 forward: +y grows
        est = motion.MotionEstimator(P, match=fake_match((0.012, 0.0, 0.0), 0.9))
        est.update(0.0, (1.0, 1.0, math.pi / 2), SCAN_A)
        state = est.update(0.1, (1.0, 1.01, math.pi / 2), SCAN_B)
        self.assertAlmostEqual(state.pose[0], 1.0, places=6)
        self.assertAlmostEqual(state.pose[1], 1.012, places=6)

    def test_time_going_backward_is_an_error(self):
        est = motion.MotionEstimator(P, match=fake_match((0.0, 0.0, 0.0), 0.9))
        est.update(1.0, (0.0, 0.0, 0.0), SCAN_A)
        with self.assertRaises(ValueError):
            est.update(0.9, (0.0, 0.0, 0.0), SCAN_B)

    def test_a_scan_that_cannot_be_matched_keeps_the_odometry(self):
        def broken(before, after, p, guess):
            raise ValueError("too few usable points")
        est = motion.MotionEstimator(P, match=broken)
        est.update(0.0, (0.0, 0.0, 0.0), SCAN_A)
        state = est.update(0.1, (0.010, 0.0, 0.0), SCAN_B)
        self.assertAlmostEqual(state.pose[0], 0.010, places=9)
        self.assertFalse(state.trusted)

    def test_the_default_matcher_thins_inside_scanmatch(self):
        seen = []
        real = motion.scanmatch.match

        def spy(before, after, p, guess, max_points=None):
            seen.append((len(before), len(after), max_points))
            return (0.0, 0.0, 0.0, 1.0)
        motion.scanmatch.match = spy
        try:
            est = motion.MotionEstimator(P)
            est.update(0.0, (0.0, 0.0, 0.0), [(d / 2.0, 2.0) for d in range(-360, 360)])
            est.update(0.1, (0.0, 0.0, 0.0), [(d / 2.0, 2.1) for d in range(-360, 360)])
        finally:
            motion.scanmatch.match = real
        self.assertEqual(seen, [(720, 720, motion.MAX_POINTS)])  # whole scans in: the surfaces need them

    def test_far_walls_are_not_trusted(self):
        from test_scanmatch import offset_scan
        for half, trusted in ((2.5, True), (5.0, False)):  # 5 m and 10 m square rooms
            est = motion.MotionEstimator(P)
            est.update(0.0, (0.0, 0.0, 0.0), offset_scan((0, 0, 0), half, 0.9, 0.0))
            state = est.update(0.5, (0.06, 0.0, 0.0), offset_scan((0.06, 0.0, 0.0), half, 0.9, 0.4))
            self.assertEqual(state.trusted, trusted, half)
            if trusted:
                self.assertAlmostEqual(state.v, 0.12, delta=0.002)

    def test_motion_is_matched_against_the_scan_a_baseline_ago(self):
        before = []

        def match(a, b, p, guess):
            before.append(a[0][1])
            return guess + (0.9,)  # agrees with the odometry
        est = motion.MotionEstimator(P, match=match)
        for k in range(8):  # a new scan every 0.1 s, 0.012 m forward each
            state = est.update(0.1 * k, (0.012 * k, 0.0, 0.0), [(0.0, 2.0 + k)])
        self.assertEqual(before[-1], 2.0 + 2)  # at 0.7 s: the scan of 0.2 s, 0.5 s before
        self.assertTrue(state.trusted)
        self.assertAlmostEqual(state.span, motion.BASELINE)
        self.assertAlmostEqual(state.v, 0.12, places=6)
        self.assertAlmostEqual(state.pose[0], 0.012 * 7, places=6)

    def test_an_odometry_only_update_spans_its_own_step(self):
        est = motion.MotionEstimator(P, match=fake_match((0.012, 0.0, 0.0), 0.9))
        est.update(0.0, (0.0, 0.0, 0.0), SCAN_A)
        state = est.update(0.05, (0.005, 0.0, 0.0), SCAN_A)
        self.assertFalse(state.trusted)
        self.assertAlmostEqual(state.span, 0.05)
        self.assertAlmostEqual(state.v, 0.1, places=6)


if __name__ == "__main__":
    unittest.main()
