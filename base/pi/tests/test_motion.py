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

    def test_long_scans_are_thinned_before_matching(self):
        seen = []

        def match(before, after, p, guess):
            seen.append((len(before), len(after)))
            return (0.0, 0.0, 0.0, 1.0)
        est = motion.MotionEstimator(P, match=match)
        est.update(0.0, (0.0, 0.0, 0.0), [(d / 2.0, 2.0) for d in range(-360, 360)])
        est.update(0.1, (0.0, 0.0, 0.0), [(d / 2.0, 2.1) for d in range(-360, 360)])
        self.assertLessEqual(max(max(s) for s in seen), motion.MAX_POINTS)


if __name__ == "__main__":
    unittest.main()
