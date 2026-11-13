import unittest

import params
import velocity
from motion import MotionState

P = params.load()
DT = 0.1


class FakeEstimates:
    def __init__(self, speed_gain=(1.0, 0, False), turn_gain=(1.0, 0, False), drift=(0.0, 0, False), latency=(0.0, 0, False)):
        self.speed_gain, self.turn_gain, self.drift_per_m, self.latency = speed_gain, turn_gain, drift, latency


def run(loop, wanted, seconds, gain=0.8, cap=0.12, drift=0.0, turn_gain=1.0):
    """A base that drives gain * the command, never faster than cap, and turns drift rad per metre."""
    v = w = 0.0
    sent = []
    t = 0.0
    while t < seconds:
        state = MotionState(t, (0.0, 0.0, 0.0), v, w, True, 0.9)
        cmd = loop.command(*wanted, state)
        sent.append(cmd)
        v = min(cap, gain * cmd[0])
        w = turn_gain * cmd[1] + drift * v
        t = round(t + DT, 6)
    return v, w, sent


class VelocityLoopTest(unittest.TestCase):
    def test_a_span_averaged_speed_is_compared_with_the_wish_at_its_middle(self):
        loop = velocity.VelocityLoop(P, FakeEstimates())
        for k in range(11):
            loop.command(0.10, 0.0, MotionState(0.1 * k, (0, 0, 0), 0.10, 0.0, True, 0.9))
        # asked less at 1.1 s; the 0.5 s average still shows 0.10, which was right for its middle
        v, _ = loop.command(0.05, 0.0, MotionState(1.1, (0, 0, 0), 0.10, 0.0, True, 0.9, 0.5))
        self.assertAlmostEqual(v, 0.05, places=6)

    def test_a_slow_base_is_brought_to_the_wanted_speed(self):
        loop = velocity.VelocityLoop(P, FakeEstimates())
        v, _, _ = run(loop, (0.10, 0.0), 3.0, gain=0.8)
        self.assertAlmostEqual(v, 0.10, delta=0.005)

    def test_the_trusted_gain_is_fed_forward(self):
        loop = velocity.VelocityLoop(P, FakeEstimates(speed_gain=(0.8, 100, True), latency=(0.3, 0, False)))
        _, _, sent = run(loop, (0.08, 0.0), 0.1, gain=0.8)
        self.assertAlmostEqual(sent[0][0], 0.10, delta=0.001)  # 0.08 / 0.8 at once; no feedback before the base can react

    def test_never_above_the_cap_and_no_wind_up(self):
        loop = velocity.VelocityLoop(P, FakeEstimates())
        _, _, sent = run(loop, (0.20, 0.0), 10.0, gain=0.8, cap=0.10)
        self.assertLessEqual(max(s[0] for s in sent), P["max_speed"] + 1e-9)
        self.assertLessEqual(abs(loop.integral_v), velocity.I_LIMIT + 1e-9)
        # then asked for less: it must come down at once, not after unwinding
        _, _, sent = run(loop, (0.05, 0.0), 0.3, gain=0.8, cap=0.10)
        self.assertLess(sent[-1][0], 0.09)

    def test_drift_is_turned_out(self):
        loop = velocity.VelocityLoop(P, FakeEstimates())
        v, w, _ = run(loop, (0.10, 0.0), 5.0, gain=1.0, drift=0.02)  # 2 cm per metre
        self.assertLess(abs(w / v), 0.005)

    def test_a_stop_is_never_delayed(self):
        loop = velocity.VelocityLoop(P, FakeEstimates(speed_gain=(0.8, 100, True)))
        run(loop, (0.10, 0.2), 2.0)
        state = MotionState(2.0, (0.0, 0.0, 0.0), 0.1, 0.2, True, 0.9)
        self.assertEqual(loop.command(0.0, 0.0, state), (0.0, 0.0))

    def test_a_hold_does_not_build_up_the_integral(self):
        loop = velocity.VelocityLoop(P, FakeEstimates())
        still = MotionState(0.0, (0.0, 0.0, 0.0), 0.0, 0.0, True, 0.9)
        for i in range(30):  # 3 s held by a stop: the brain asks 0.025, the base stands
            t = 0.1 * i
            loop.command(0.025, 0.0, MotionState(t, (0, 0, 0), 0.0, 0.0, True, 0.9))
            loop.reset(t)
        first = loop.command(0.025, 0.0, MotionState(3.0, (0, 0, 0), 0.0, 0.0, True, 0.9))
        self.assertLess(first[0], 0.04)

    def test_the_cycle_cap_is_kept(self):
        loop = velocity.VelocityLoop(P, FakeEstimates())
        _, _, sent = run(loop, (0.05, 0.0), 3.0, gain=0.5)
        loop2 = velocity.VelocityLoop(P, FakeEstimates())
        state = MotionState(3.1, (0, 0, 0), 0.0, 0.0, True, 0.9)
        self.assertLessEqual(loop2.command(0.05, 0.0, state, top=0.05)[0], 0.05)

    def test_an_untrusted_measurement_only_feeds_forward(self):
        loop = velocity.VelocityLoop(P, FakeEstimates(speed_gain=(0.8, 100, True)))
        state = MotionState(0.0, (0.0, 0.0, 0.0), 0.0, 0.0, False, 0.2)
        for _ in range(20):
            cmd = loop.command(0.08, 0.0, state)
        self.assertAlmostEqual(cmd[0], 0.10, delta=0.001)
        self.assertEqual(loop.integral_v, 0.0)


if __name__ == "__main__":
    unittest.main()
