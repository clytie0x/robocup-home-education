import math
import sys
import unittest
from pathlib import Path

import params
from test_scanmatch import room_scan

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import base_check  # noqa: E402

P = params.load()


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, seconds):
        self.t += max(seconds, 0.001)


class FakeWheels:
    """Moves forward by v * dt per odometry read; read number `fail_at` raises, read `slow_at` takes 0.4 s.

    odom_zero: the odometry reads 0 whatever the wheels do (the open -026 case).
    """

    def __init__(self, clock, fail_at=None, slow_at=None, odom_zero=False):
        self.clock, self.fail_at, self.slow_at, self.odom_zero = clock, fail_at, slow_at, odom_zero
        self.calls, self.v, self.x, self.reads = [], 0.0, 0.0, 0
        self.last_t = clock()

    def send(self, v, w):
        self.calls.append("send")
        self.v = v

    def read_odom(self):
        self.reads += 1
        if self.reads == self.fail_at:
            raise RuntimeError("serial unplugged")
        if self.reads == self.slow_at:
            self.clock.t += 0.4
        self.x += self.v * (self.clock() - self.last_t)
        self.last_t = self.clock()
        return (0.0 if self.odom_zero else self.x, 0.0, 0.0)

    def stop(self):
        self.calls.append("stop")
        self.v = 0.0


class FakeStream:
    updated_at = None


class FakeSession:
    """A new revolution every 0.1 s of the fake clock."""

    def __init__(self, clock, scan):
        self.clock, self.scan, self.stream = clock, scan, FakeStream()

    def latest(self):
        self.stream.updated_at = math.floor(self.clock() / 0.1) * 0.1
        return list(self.scan)


def new_log():
    return {"odom": [], "scans": []}


class ClearToDrive(unittest.TestCase):
    def test_box_in_the_strip_refuses(self):
        scan = [(0.0, 1.0)] + room_scan((0, 0, 0), half=3.0)  # 0.975 m ahead of the front
        ok, nearest = base_check.clear_to_drive(scan, P, "line", 1.5)
        self.assertFalse(ok)
        self.assertAlmostEqual(nearest, 1.0 + P["lidar_x"] - 0.20, places=6)

    def test_far_wall_is_clear(self):
        ok, nearest = base_check.clear_to_drive(room_scan((0, 0, 0), half=3.0), P, "line", 1.5)
        self.assertTrue(ok)  # wall ahead at 3 - 0.20 = 2.8 m > 1.5 + 0.6
        self.assertAlmostEqual(nearest, 2.8, delta=0.01)

    def test_side_point_blocks_a_turn_not_a_line(self):
        scan = [(90.0, 0.40)] + room_scan((0, 0, 0), half=3.0)  # 0.4 m left of the LiDAR
        self.assertTrue(base_check.clear_to_drive(scan, P, "line", 1.5)[0])
        self.assertFalse(base_check.clear_to_drive(scan, P, "turn", math.pi)[0])

    def test_no_scan_refuses(self):
        self.assertFalse(base_check.clear_to_drive([], P, "line", 1.5)[0])


class Drive(unittest.TestCase):
    def test_exception_sends_stop_last(self):
        clock = Clock()
        wheels = FakeWheels(clock, fail_at=5)
        session = FakeSession(clock, room_scan((0, 0, 0), half=3.0))
        log = new_log()
        with self.assertRaises(RuntimeError):
            base_check.drive(wheels, session, P, "line", 0.15, 1.5, log, clock, clock.sleep)
        self.assertIn("send", wheels.calls)
        self.assertEqual(wheels.calls[-1], "stop")
        self.assertEqual(log["stop_reason"], "error")
        self.assertIn("serial unplugged", log["error"])

    def test_reaches_the_target_and_stops(self):
        clock = Clock()
        wheels = FakeWheels(clock)
        session = FakeSession(clock, room_scan((0, 0, 0), half=3.0))
        log = new_log()
        base_check.drive(wheels, session, P, "line", 0.15, 0.3, log, clock, clock.sleep)
        self.assertEqual(log["stop_reason"], "target")
        self.assertEqual(wheels.calls[-1], "stop")
        self.assertGreaterEqual(wheels.x, 0.3)
        self.assertGreater(len(log["odom"]), 30)  # 20 Hz for about 2 s
        self.assertGreater(len(log["scans"]), 15)

    def test_point_close_ahead_is_an_emergency(self):
        clock = Clock()
        wheels = FakeWheels(clock)
        session = FakeSession(clock, [(0.0, 0.30)] + room_scan((0, 0, 0), half=3.0))
        log = new_log()
        base_check.drive(wheels, session, P, "line", 0.15, 1.5, log, clock, clock.sleep)
        self.assertEqual(log["stop_reason"], "emergency")
        self.assertEqual(wheels.calls[-1], "stop")


class Guards(unittest.TestCase):
    def test_slow_odometry_read_ends_the_run(self):
        clock = Clock()
        wheels = FakeWheels(clock, slow_at=10)
        log = new_log()
        base_check.drive(wheels, FakeSession(clock, room_scan((0, 0, 0), half=3.0)), P, "line", 0.15, 1.5, log, clock, clock.sleep)
        self.assertEqual(log["stop_reason"], "send gap")
        self.assertEqual(wheels.calls.count("send"), 9)  # no send after the gap
        self.assertEqual(wheels.calls[-1], "stop")

    def test_dead_odometry_stays_inside_the_clear_strip(self):
        clock = Clock()
        wheels = FakeWheels(clock, odom_zero=True)
        log = new_log()
        base_check.drive(wheels, FakeSession(clock, room_scan((0, 0, 0), half=3.0)), P, "line", 0.6, 1.5, log, clock, clock.sleep)
        self.assertEqual(log["stop_reason"], "timeout")
        self.assertLessEqual(wheels.x, 1.5 + 0.3 + 0.6 * 0.06)  # one loop period of slack, well under 1.5 + 0.6

    def test_emergency_distance_grows_with_speed(self):
        self.assertEqual(base_check.emergency_distance(0.15), 0.35)
        self.assertAlmostEqual(base_check.emergency_distance(0.6), 0.6 * 0.4 + 0.36 + 0.15)
        scan = [(0.0, 0.62 + 0.20 - P["lidar_x"])] + room_scan((0, 0, 0), half=3.0)  # 0.62 m ahead of the front
        self.assertFalse(base_check.emergency(scan, P, "line", 0.15))
        self.assertTrue(base_check.emergency(scan, P, "line", 0.6))

    def test_first_revolution_is_dropped(self):
        clock = Clock()
        log = new_log()
        base_check.first_full_scan(FakeSession(clock, room_scan((0, 0, 0))), log, clock, clock.sleep)
        self.assertEqual(len(log["scans"]), 1)
        self.assertGreater(log["scans"][0][0], 0.0)  # the second revolution, not the one stamped at 0

    def test_flags_out_of_range_refuse_before_any_port(self):
        def no_port(*_args, **_kwargs):
            raise AssertionError("a port was opened")

        saved = base_check.lidar.LidarSession, base_check.wheels_module.Wheels, sys.argv
        base_check.lidar.LidarSession = base_check.wheels_module.Wheels = no_port
        try:
            for argv in (["--line", "0.3", "--max-speed", "0.8"], ["--line", "0.15", "--linear-scale", "10"],
                         ["--line", "0.15", "--linear-scale", "0.2"], ["--line", "0.3"], ["--line", "0.15"],
                         ["--turn", "0.8"], ["--turn", "0.8", "--max-turn-rate", "2.0"]):
                sys.argv = ["base_check.py"] + argv + (["--yes-owner-at-robot"] if argv != ["--line", "0.15"] else [])
                with self.assertRaises(SystemExit) as stopped:
                    base_check.main()
                self.assertIn("REFUSED", str(stopped.exception.code))
        finally:
            base_check.lidar.LidarSession, base_check.wheels_module.Wheels, sys.argv = saved


def straight_log(v=0.3, drift=0.02, stop_time=5.0, decel=1.0, odom_scale=0.97):
    """A run at v m/s from t = 0, S# at stop_time, then braking at decel with no delay.

    The base slides sideways by `drift` m per metre forward. Scans every 0.1 s, stamped at
    the end of the revolution (pose time + 0.05 s); odometry at 20 Hz reads odom_scale * real.
    """
    coast = v * v / (2 * decel)

    def forward(t):
        if t <= 0:
            return 0.0
        if t <= stop_time:
            return v * t
        u = min(t - stop_time, v / decel)
        return v * stop_time + v * u - decel * u * u / 2

    start = (-0.8, 0.0)
    pose = lambda t: (start[0] + forward(t), start[1] + drift * forward(t), 0.0)  # noqa: E731
    log = {"mode": "line", "rate": v, "drive_start": 0.0, "stop_time": stop_time, "odom": [], "scans": []}
    t = -0.5
    while t < stop_time + 2.2:
        log["scans"].append([t + 0.05, room_scan(pose(t))])
        t = round(t + 0.1, 6)
    for i in range(int((stop_time + 2.2) * 20) + 1):
        x, y, _ = pose(i / 20)
        log["odom"].append([i / 20, odom_scale * (x - start[0]), odom_scale * (y - start[1]), 0.0])
    return log, v * stop_time + coast, coast


class Report(unittest.TestCase):
    def test_straight_run(self):
        log, distance, coast = straight_log()
        r = base_check.report(log, P)
        self.assertAlmostEqual(r["real_distance"], distance * math.hypot(1, 0.02), delta=0.01)
        self.assertAlmostEqual(r["odom_distance"], 0.97 * distance * math.hypot(1, 0.02), delta=0.005)
        self.assertAlmostEqual(r["drift_per_m"], 0.02, delta=0.005)
        self.assertAlmostEqual(r["middle_speed"], 0.3, delta=0.01)
        self.assertAlmostEqual(r["after_stop"], coast, delta=0.01)
        self.assertAlmostEqual(math.degrees(r["scan_yaw"]), 0.0, delta=0.5)
        self.assertLess(r["stop_delay"], 0.06)  # no delay in this log; the steps are 0.1 s long

    def test_half_turn_right(self):
        w, decel, stop_time = 0.49, 2.0, math.pi / 0.49  # S# after 180 deg, then 0.06 rad of coast

        def yaw(t):
            if t <= 0:
                return 0.0
            u = min(max(t - stop_time, 0.0), w / decel)
            return -(w * min(t, stop_time) + w * u - decel * u * u / 2)

        log = {"mode": "turn", "rate": -w, "drive_start": 0.0, "stop_time": stop_time, "odom": [], "scans": []}
        t = -0.5
        while t < stop_time + 2.2:
            log["scans"].append([t + 0.05, room_scan((0.0, 0.0, yaw(t)))])
            t = round(t + 0.1, 6)
        for i in range(int((stop_time + 2.2) * 20) + 1):
            log["odom"].append([i / 20, 0.0, 0.0, math.atan2(math.sin(0.9 * yaw(i / 20)), math.cos(0.9 * yaw(i / 20)))])
        r = base_check.report(log, P)
        coast = w * w / (2 * decel)
        self.assertAlmostEqual(r["scan_yaw"], -math.pi - coast, delta=math.radians(0.5))
        self.assertAlmostEqual(r["odom_yaw"], 0.9 * (-math.pi - coast), delta=0.01)  # unwrapped past 180 deg
        self.assertAlmostEqual(r["middle_speed"], w, delta=0.01)
        self.assertAlmostEqual(r["after_stop"], coast, delta=math.radians(0.5))


if __name__ == "__main__":
    unittest.main()
