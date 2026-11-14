import math
import unittest

import latency
import run_follow


def log_that_moves_after(lag, dt=0.02, speed=0.12):
    t0 = 100.0
    odom = [[t0 - 0.1, 1.0, 2.0, 0.5], [t0, 1.0, 2.0, 0.5]]
    t = t0
    while t < t0 + 2.0:
        t += dt
        moved = round(max(0.0, t - t0 - lag) * speed, 2)  # 0.01 m steps, like the firmware
        odom.append([t, 1.0 + moved * math.cos(0.5), 2.0 + moved * math.sin(0.5), 0.5])
    return {"drive_start": t0, "odom": odom}


class LatencyTest(unittest.TestCase):
    def test_first_motion_finds_the_lag(self):
        got = latency.first_motion(log_that_moves_after(0.30))
        self.assertGreaterEqual(got, 0.30)
        self.assertLess(got, 0.30 + 0.015 / 0.12 + 0.05)  # the 0.015 m it takes to see it, one sample

    def test_no_motion_is_none(self):
        self.assertIsNone(latency.first_motion({"drive_start": 1.0, "odom": [[0.5, 0, 0, 0], [1.5, 0, 0, 0]]}))


class PredictTest(unittest.TestCase):
    def test_straight(self):
        self.assertEqual(run_follow.predict((0, 0, 0), 0.1, 0.0, 0.5), (0.05, 0.0, 0.0))

    def test_an_arc_ends_on_the_circle(self):
        x, y, th = run_follow.predict((0, 0, 0), 0.1, 0.5, math.pi)  # a quarter circle of radius 0.2
        self.assertAlmostEqual(x, 0.2, places=9)
        self.assertAlmostEqual(y, 0.2, places=9)
        self.assertAlmostEqual(th, math.pi / 2, places=9)

    def test_the_commands_still_on_their_way_are_played_out(self):
        sent = [(9.0, 0.1, 0.0), (9.8, 0.1, 0.0), (9.9, 0.0, 0.0)]  # (time sent, v, w)
        # latency 0.3 s at 10.0: what was sent from 9.7 to 10.0 plays out: 0.2 s at 0.1, 0.1 s standing
        x, y, th = run_follow.ahead((0, 0, 0), sent, 10.0, 0.3)
        self.assertAlmostEqual(x, 0.02, places=9)
        self.assertEqual((y, th), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
