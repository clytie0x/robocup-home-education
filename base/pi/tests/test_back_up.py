import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path

import lidar
import params
import tether
import wheels
from test_scanmatch import room_scan

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import back_up  # noqa: E402

P = params.load()


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, seconds):
        self.t += max(seconds, 0.001)


class Ticker:
    """A clock that moves on 10 ms per read, so every scan of main()'s session is a new one."""

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        self.t += 0.01
        return self.t


class FakeWheels:
    """Backs along -x of `pose` by speed * dt per odometry read; send() would be a forward drive.

    The odometry reads `gain` times the real travel (0: frozen); read number `fail_at` raises.
    """

    def __init__(self, clock, pose=(0.0, 0.0, 0.0), direction=-1.0, gain=1.0, fail_at=None):
        self.clock, self.pose, self.direction, self.gain, self.fail_at = clock, pose, direction, gain, fail_at
        self.calls, self.speed, self.travel, self.last_t, self.reads = [], 0.0, 0.0, clock(), 0

    def send(self, v, w):
        raise AssertionError("back_up drove forward")

    def send_reverse(self, speed, w):
        self.calls.append(("reverse", speed, w))
        self.speed = speed

    def read_odom(self):
        self.reads += 1
        if self.reads == self.fail_at:
            raise RuntimeError("serial unplugged")
        self.travel += self.direction * self.speed * (self.clock() - self.last_t)
        self.last_t = self.clock()
        x, y, th = self.pose
        odom = self.gain * self.travel
        return (x + odom * math.cos(th), y + odom * math.sin(th), th)

    def stop(self):
        self.calls.append("stop")
        self.speed = 0.0

    def close(self):
        self.calls.append("close")


class FakeSession:
    """A real lidar.ScanStream fed scan_at(travel) on every read until the robot backed `freeze_at`;
    after that no revolution comes in, and the stream's own max_age decides when the scan is gone."""

    def __init__(self, scan_at, clock, freeze_at=math.inf):
        self.scan_at, self.stream, self.freeze_at = scan_at, lidar.ScanStream(clock=clock), freeze_at
        self.travel, self.closed = lambda: 0.0, False

    def latest(self):
        if self.travel() < self.freeze_at:
            self.stream.last, self.stream.updated_at = list(self.scan_at(self.travel())), self.stream.clock()
        return self.stream.latest()

    def close(self):
        self.closed = True


def open_room(_travel=0.0):
    return room_scan((0.0, 0.0, 0.0), half=2.0)  # 1.8 m free behind the back edge


def rear_wall_at(gap):
    """The back edge `gap` m from a wall behind (room of side 4, robot moved back towards it)."""
    return lambda travel: room_scan((-(2.0 - 0.20 - gap) - travel, 0.0, 0.0), half=2.0)


class Tool(unittest.TestCase):
    def setUp(self):
        self.saved = back_up.lidar.LidarSession, back_up.wheels_module.Wheels, sys.argv, os.environ.get(wheels.SAFE_RUN_ENV)
        self.opened = []

    def tearDown(self):
        back_up.lidar.LidarSession, back_up.wheels_module.Wheels, sys.argv, env = self.saved
        if env is None:
            os.environ.pop(wheels.SAFE_RUN_ENV, None)
        else:
            os.environ[wheels.SAFE_RUN_ENV] = env

    def run_main(self, argv, safe_run=True, scan_at=open_room, pose=(0.0, 0.0, 0.0)):
        if safe_run:
            os.environ[wheels.SAFE_RUN_ENV] = "1"
        else:
            os.environ.pop(wheels.SAFE_RUN_ENV, None)
        self.session = FakeSession(scan_at, Ticker())
        self.wheels = FakeWheels(Clock(), pose)

        def lidar_session():
            self.opened.append("lidar")
            return self.session

        def open_wheels(_p):
            self.opened.append("wheels")
            return self.wheels

        back_up.lidar.LidarSession, back_up.wheels_module.Wheels = lidar_session, open_wheels
        sys.argv = ["back_up.py"] + argv
        with self.assertRaises(SystemExit) as stopped:
            back_up.main()
        self.assertIn("REFUSED", str(stopped.exception.code))
        return str(stopped.exception.code)

    def test_refuses_without_the_flag_too_far_or_without_safe_run_before_any_port(self):
        for argv, safe_run in ((["--metres", "0.2"], True), (["--metres", "0.31", "--yes-back-up"], True),
                               (["--metres", "0", "--yes-back-up"], True), (["--metres", "-0.1", "--yes-back-up"], True),
                               (["--metres", "nan", "--yes-back-up"], True), (["--metres", "0.2", "--yes-back-up"], False)):
            self.run_main(argv, safe_run)
            self.assertEqual(self.opened, [], argv)

    def test_refuses_when_the_rear_is_blocked_and_never_moves(self):
        message = self.run_main(["--metres", "0.2", "--yes-back-up"], scan_at=rear_wall_at(0.25))
        self.assertIn("rear", message)
        self.assertNotIn(("reverse", back_up.SPEED, 0.0), self.wheels.calls)
        self.assertEqual(self.wheels.calls[-2:], ["stop", "close"])
        self.assertTrue(self.session.closed)

    def test_refuses_an_avoid_point_on_the_path_in_the_odometry_frame(self):
        pose = (1.0, 2.0, math.pi / 2)  # facing +y: behind is -y
        with tempfile.TemporaryDirectory() as tmp:
            trail = Path(tmp) / "trail.json"
            trail.write_text(json.dumps([[5.0, 5.0], [1.0, 2.0 - 0.45]]))  # 0.25 m behind the back edge
            message = self.run_main(["--metres", "0.2", "--yes-back-up", "--avoid", str(trail)], pose=pose)
            self.assertIn("avoid point", message)
            self.assertNotIn(("reverse", back_up.SPEED, 0.0), self.wheels.calls)
            for bad in ("[[1.0, 2.0], [3]]", "[[1.0, NaN]]", '{"trail": 3}', "nonsense"):
                trail.write_text(bad)
                self.assertIn("avoid", self.run_main(["--metres", "0.2", "--yes-back-up", "--avoid", str(trail)]))

    def test_refuses_a_saved_tether_whose_cable_lies_behind(self):
        pose = (1.0, 2.0, math.pi / 2)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "tether.json"
            tether.Tether(P, pose).save(path)  # just laid: 1 m of cable straight behind the attach post
            self.assertIn("avoid point", self.run_main(["--metres", "0.2", "--yes-back-up", "--avoid", str(path)], pose=pose))
            self.assertNotIn(("reverse", back_up.SPEED, 0.0), self.wheels.calls)


class Avoid(unittest.TestCase):
    """The rule of footprint.sweep_clear with AVOID_MARGIN: a point already nearer only blocks getting closer."""

    def setUp(self):
        self.pts = back_up.run_follow.scan_points(open_room(), P)
        self.pose = (1.0, 2.0, math.pi / 2)  # facing +y: base (x, y) is odometry (1 - y, 2 + x)

    def odom(self, base_points):
        return back_up.numpy.array([(1.0 - y, 2.0 + x) for x, y in base_points]).reshape(-1, 2)

    def check(self, base_points, metres=0.3):
        return back_up.blocked(self.pts, self.pose, metres, self.odom(base_points), P)

    def test_a_trail_ending_at_the_robot_does_not_block_backing_away_from_it(self):
        self.assertIsNone(self.check([(x, 0.0) for x in (1.0, 0.6, 0.3, 0.21)]))  # ahead, ends 1 cm from the front
        self.assertIsNone(self.check([(-0.08, 0.20)]))  # on the attach post itself, alongside the path

    def test_a_trail_point_on_the_path_blocks(self):
        self.assertIn("avoid", self.check([(1.0, 0.0), (0.21, 0.0), (-0.45, 0.0)]))
        self.assertIn("avoid", self.check([(-0.62, 0.10)]))  # 0.3 m + run-on + 0.10 m behind: inside 0.20 m

    def test_a_side_point_beyond_the_margin_passes(self):
        self.assertIsNone(self.check([(-0.40, 0.20 + back_up.AVOID_MARGIN + 0.03)]))
        self.assertIsNone(self.check([(5.0, 5.0)]))

    def test_a_tether_file_gives_its_cable_points_without_the_attached_part(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "tether.json"
            lay = self.odom([(1.0, 0.20), (0.5, 0.20), (-0.08, 0.20)]).tolist()  # from ahead to the post
            tether.Tether(P, self.pose, lay=lay).save(path)
            avoid = back_up.load_avoid(path, P)
            self.assertEqual(avoid.shape[1], 2)
            local = back_up.numpy.array([back_up.geometry.odom_to_base(q, self.pose) for q in avoid])
            self.assertGreater(local[:, 0].min(), -0.08 + tether.TRAIL_ATTACHED - 0.01)  # the newest 0.4 m left out
            self.assertIsNone(back_up.blocked(self.pts, self.pose, 0.3, avoid, P))
            path.write_text(json.dumps([[1.0, 2.0]]))  # the plain list still works
            self.assertEqual(back_up.load_avoid(path, P).tolist(), [[1.0, 2.0]])


class Drive(unittest.TestCase):
    def drive(self, scan_at, metres=0.2, freeze_at=math.inf, **wheels_kw):
        self.clock = Clock()
        w = FakeWheels(self.clock, **wheels_kw)
        session = FakeSession(scan_at, self.clock, freeze_at)
        session.travel = lambda: -w.travel
        result = back_up.drive(w, session, P, metres, back_up.numpy.empty((0, 2)), self.clock, self.clock.sleep)
        return result, w

    def test_backs_up_the_distance_slowly_and_stops(self):
        result, w = self.drive(open_room)
        self.assertEqual(result["stop"], "there")
        self.assertGreaterEqual(-w.travel, 0.2)
        self.assertLess(-w.travel, 0.2 + back_up.SPEED * back_up.PERIOD + 1e-9)
        self.assertEqual({c for c in w.calls if c != "stop"}, {("reverse", back_up.SPEED, 0.0)})
        self.assertEqual(w.calls[-1], "stop")

    def test_stops_when_the_rear_closes_in(self):
        # 0.40 m behind: clear for 0.2 m; after 5 cm something steps in 0.15 m behind the back edge
        result, w = self.drive(lambda travel: rear_wall_at(0.40 if travel < 0.05 else 0.15 + travel)(travel))
        self.assertEqual(result["stop"], "rear blocked")
        self.assertEqual(w.calls[-1], "stop")
        self.assertLess(-w.travel, 0.06)

    def test_stops_when_the_scan_grows_too_old(self):
        result, w = self.drive(open_room, freeze_at=0.05)  # no new revolution after 5 cm
        self.assertEqual(result["stop"], "no LiDAR points")
        age = lidar.ScanStream().max_age
        self.assertGreater(-w.travel, 0.05 + 0.5 * age * back_up.SPEED)  # went on while the last scan was young
        self.assertLess(-w.travel, 0.05 + (age + 2 * back_up.PERIOD) * back_up.SPEED)

    def test_stops_when_the_odometry_goes_forward(self):
        result, w = self.drive(open_room, direction=1.0)
        self.assertIn("forward", result["stop"])
        self.assertLess(w.travel, 0.03)
        self.assertEqual(w.calls[-1], "stop")

    def test_stops_at_the_time_limit_when_the_odometry_is_slow(self):
        result, w = self.drive(open_room, gain=0.3)  # reads 0.3 of the real travel: never "there"
        self.assertEqual(result["stop"], "time")
        limit = 0.2 / back_up.SPEED * 1.5 + 1.0
        self.assertGreater(self.clock.t, limit + 1.0)  # + the 1 s roll-out wait
        self.assertLess(self.clock.t, limit + back_up.PERIOD + 1.0 + 1e-6)
        self.assertEqual(w.calls[-1], "stop")

    def test_stops_when_the_odometry_is_frozen(self):
        result, w = self.drive(open_room, metres=0.05, gain=0.0)
        self.assertEqual(result["stop"], "odometry not moving")
        self.assertLess(-w.travel, back_up.SPEED * (back_up.STALL_S + 2 * back_up.PERIOD))
        self.assertEqual(w.calls[-1], "stop")

    def test_an_odometry_error_still_sends_stop(self):
        clock = Clock()
        w = FakeWheels(clock, fail_at=5)
        session = FakeSession(open_room, clock)
        with self.assertRaises(RuntimeError):
            back_up.drive(w, session, P, 0.2, back_up.numpy.empty((0, 2)), clock, clock.sleep)
        self.assertIn(("reverse", back_up.SPEED, 0.0), w.calls)
        self.assertEqual(w.calls[-1], "stop")


if __name__ == "__main__":
    unittest.main()
