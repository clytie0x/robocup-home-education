import math
import unittest

import calib
import params
from motion import MotionState

P = params.load()
DT = 0.1


def drive(est, commands, gain=1.0, turn_gain=1.0, lag=0.0, decel=None, delay=0.0, drift=0.0, t0=0.0):
    """A simulated base: it does gain * the command `lag` seconds late; after a stop it rolls on
    `delay` seconds, then slows at `decel`. drift: rad per metre it turns by itself."""
    sent = []  # (time, v, w)
    v_now = 0.0
    t = t0
    for v_cmd, w_cmd in commands:
        sent.append((t, v_cmd, w_cmd))
        acting = [s for s in sent if s[0] <= t - lag + 1e-9]
        _, va, wa = acting[-1] if acting else (t, 0.0, 0.0)
        if decel is not None and va == 0.0 and v_now > 0:
            stopped_at = max(s[0] for s in acting if s[1] > 0) + DT if any(s[1] > 0 for s in acting) else t
            if t - stopped_at - lag >= delay:
                v_now = max(0.0, v_now - decel * DT)
        else:
            v_now = gain * va
        w_now = turn_gain * wa + drift * v_now
        est.observe(MotionState(t, (0.0, 0.0, 0.0), v_now, w_now, True, 0.9), (v_cmd, w_cmd), t)
        t = round(t + DT, 6)
    return t


class EstimatesTest(unittest.TestCase):
    def test_too_few_samples_report_the_params_values_untrusted(self):
        est = calib.BaseEstimates(P)
        for name, key in (("decel", "decel"), ("stop_delay", "stop_delay"), ("latency", "command_latency")):
            value, samples, trusted = getattr(est, name)
            self.assertEqual((value, samples, trusted), (P[key], 0, False), name)
        self.assertEqual(est.speed_gain, (1.0, 0, False))

    def test_speed_gain_converges(self):
        est = calib.BaseEstimates(P)
        drive(est, [(0.10, 0.0)] * 300, gain=0.8)  # 30 s
        value, samples, trusted = est.speed_gain
        self.assertTrue(trusted)
        self.assertAlmostEqual(value, 0.80, delta=0.03)

    def test_turn_gain_and_drift(self):
        est = calib.BaseEstimates(P)
        drive(est, [(0.0, 0.4)] * 200, turn_gain=0.9)
        self.assertAlmostEqual(est.turn_gain[0], 0.9, delta=0.03)
        est = calib.BaseEstimates(P)
        drive(est, [(0.10, 0.0)] * 300, drift=0.02)  # 2 cm of turn per metre: 0.02 rad/m
        self.assertAlmostEqual(est.drift_per_m[0], 0.02, delta=0.005)

    def test_latency_from_the_lag_between_command_and_turn(self):
        est = calib.BaseEstimates(P)
        pattern = ([(0.0, 0.4)] * 15 + [(0.0, -0.4)] * 15) * 5  # 15 s of turning back and forth
        drive(est, pattern, lag=0.4)
        value, _, trusted = est.latency
        self.assertTrue(trusted)
        self.assertAlmostEqual(value, 0.40, delta=0.1)

    def test_braking_from_stops(self):
        est = calib.BaseEstimates(dict(P, decel=0.5, stop_delay=0.0))  # a generous floor, so the estimate shows
        t = 0.0
        for _ in range(4):
            t = drive(est, [(0.12, 0.0)] * 30 + [(0.0, 0.0)] * 40, decel=0.064, delay=0.27, t0=t)
        decel, _, trusted = est.decel
        delay, _, _ = est.stop_delay
        self.assertTrue(trusted)
        self.assertAlmostEqual(decel, 0.064, delta=0.0064)
        self.assertAlmostEqual(delay, 0.27, delta=0.1)

    def test_safe_braking_is_never_better_than_the_floor(self):
        est = calib.BaseEstimates(P)  # floor: decel 0.064, delay 0.27
        t = 0.0
        for _ in range(4):
            t = drive(est, [(0.12, 0.0)] * 30 + [(0.0, 0.0)] * 40, decel=0.3, delay=0.05, t0=t)  # a better base
        decel, delay = est.safe_braking()
        self.assertLessEqual(decel, P["decel"])
        self.assertGreaterEqual(delay, P["stop_delay"])

    def test_standing_after_a_stop_makes_no_more_stops(self):
        est = calib.BaseEstimates(dict(P, decel=0.5, stop_delay=0.0))
        t = drive(est, [(0.04, 0.0)] * 20 + [(0.0, 0.0)] * 20, decel=0.064, delay=0.27)
        samples = est.decel[1]
        for i in range(600):  # 60 s standing with a little noise in the measured speed
            est.observe(MotionState(t + 0.1 * i, (0, 0, 0), 0.01 * (i % 2), 0.0, True, 0.9), (0.0, 0.0), t)
        self.assertEqual(est.decel[1], samples)

    def test_no_latency_from_noise_without_turn_commands(self):
        est = calib.BaseEstimates(P)
        for i in range(300):
            w = 0.15 if i % 37 == 0 else 0.0
            est.observe(MotionState(0.1 * i, (0, 0, 0), 0.05, w, True, 0.9), (0.05, 0.0), 0.0)
        self.assertEqual(est.latency[1], 0)

    def test_a_command_the_velocity_loop_moves_a_little_still_counts_as_steady(self):
        est = calib.BaseEstimates(P)
        for i in range(400):  # the loop moves the command by under SAME every cycle
            v_cmd = 0.10 + 0.004 * ((i % 3) - 1)
            est.observe(MotionState(0.1 * i, (0, 0, 0), 0.8 * 0.10, 0.003, True, 0.9), (v_cmd, 0.005), 0.1 * i)
        self.assertTrue(est.speed_gain[2])
        self.assertAlmostEqual(est.speed_gain[0], 0.8, delta=0.05)

    def test_estimates_stay_inside_their_bounds(self):
        est = calib.BaseEstimates(P)
        drive(est, [(0.10, 0.0)] * 300, gain=3.0)
        self.assertLessEqual(est.speed_gain[0], calib.GAIN_BOUNDS[1])


if __name__ == "__main__":
    unittest.main()
