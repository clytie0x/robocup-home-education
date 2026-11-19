"""The brake tool records odometry and LiDAR, and it refuses to move without the owner and safe_run."""
import math
import sys
import unittest
from pathlib import Path
from unittest import mock

import params
from test_base_check import Clock, FakeSession, FakeWheels
from test_scanmatch import room_scan

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import brake_distance  # noqa: E402
import follow_speed  # noqa: E402
import wheels as wheels_module  # noqa: E402

P = params.load()


class Refuse(unittest.TestCase):
    def test_no_owner_and_no_safe_run(self):
        self.assertIn("yes-owner-at-robot", brake_distance.refuse(False, True))
        self.assertIn(wheels_module.NOT_SAFE_RUN, brake_distance.refuse(True, False))
        self.assertIsNone(brake_distance.refuse(True, True))

    def test_main_stops_before_the_lidar_port(self):
        with self.assertRaises(SystemExit) as missing:
            brake_distance.main([])
        self.assertIn("yes-owner-at-robot", str(missing.exception))
        with mock.patch.object(wheels_module, "under_safe_run", return_value=False):
            with self.assertRaises(SystemExit) as unsafe:
                brake_distance.main(["--yes-owner-at-robot"])
        self.assertIn("safe_run", str(unsafe.exception))
        with mock.patch.object(wheels_module, "under_safe_run", return_value=True):
            with self.assertRaises(SystemExit) as short:
                brake_distance.main(["--yes-owner-at-robot", "--metres", "0.10"])
        self.assertIn("0.20", str(short.exception))


class Record(unittest.TestCase):
    def test_stops_from_odometry_and_a_closer_wall(self):
        log = {
            "rate": 0.12,
            "stop_time": 1.0,
            "stop_reason": "target",
            "odom": [(0.5, 0.0, 0.0, 0.0), (1.0, 0.40, 0.0, 0.0), (1.5, 0.47, 0.0, 0.0)],
            "scans": [(0.9, room_scan((0.0, 0.0, 0.0), half=2.0)),
                      (1.2, room_scan((0.05, 0.0, 0.0), half=2.0))],
        }
        got = brake_distance.stops_from_log(log, P)
        self.assertAlmostEqual(got["odom_stop_m"], 0.07)
        self.assertAlmostEqual(got["lidar_stop_m"], 0.05, delta=0.02)
        self.assertEqual(got["speed"], 0.12)

    def test_run_params_allows_015_in_memory_only(self):
        original = dict(P)
        copy = brake_distance.run_params(P)
        self.assertGreaterEqual(copy["max_speed"], 0.15)
        self.assertEqual(P["max_speed"], original["max_speed"])
        self.assertIsNot(copy, P)

    def test_the_report_names_the_switch_and_the_cap_now(self):
        runs = [{"speed": v, "odom_stop_m": 0.05, "lidar_stop_m": 0.05} for v in follow_speed.RUN_SPEEDS]
        report = brake_distance.build_report(runs, P, brake_distance.METRES)
        self.assertEqual(report["follow_max_speed_now"], 0.12)
        self.assertIn("max_speed", report["switch"])
        self.assertEqual(report["metres"], 0.40)
        self.assertIn("gate", report)

    def test_three_straight_runs_are_commanded(self):
        clock = Clock()
        wheels = FakeWheels(clock)
        session = FakeSession(clock, room_scan((0.0, 0.0, 0.0), half=3.0))
        asked = []
        for speed in follow_speed.RUN_SPEEDS:
            brake_distance.measure_run(wheels, session, brake_distance.run_params(P), speed, 0.30,
                                       clock, clock.sleep)
            asked.append(max(v for v, _w in wheels.sends))
            wheels.sends.clear()
        for speed, seen in zip(follow_speed.RUN_SPEEDS, asked):
            self.assertAlmostEqual(seen, speed, places=2)
        self.assertIn("stop", wheels.calls)


if __name__ == "__main__":
    unittest.main()
