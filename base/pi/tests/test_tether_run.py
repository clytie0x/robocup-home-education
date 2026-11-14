import contextlib
import io
import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

import geometry
import params
import tether
import wheels
from test_floor_cable import scene
from test_scanmatch import room_scan
from test_tether import drive

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import tether_run  # noqa: E402

P = params.load()
FLOOR = scene()  # tiles, no cable
CABLE = scene(cable=(-0.3, 0.3))  # a cable across the path about 0.85 m ahead of the base centre
BLIND = np.zeros_like(FLOOR)
LAY_LEFT = [(0.5, 1.5), (-0.08, 0.20)]  # the cable leaves the post forward-left: nothing behind the robot


def big_room(pose):
    return room_scan(pose, half=3.0)


def wall_ahead(pose):
    """A wall 0.30 m ahead of the front edge (heading 0), 5.5 m free behind."""
    return room_scan((pose[0] + 3.0 - 0.20 - 0.30, pose[1], pose[2]), half=3.0)


def boxed_in(pose):
    """0.30 m free ahead of the front edge, 0.25 m behind the back edge."""
    return room_scan((pose[0] - 0.025, pose[1], pose[2]), half=0.475)


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, seconds):
        self.t += max(seconds, 0.001)


class FakeWheels:
    """Moves at the commanded (v, w); after S# holds it for stop_delay, then slows at decel / turn_accel."""

    def __init__(self, clock, pose=(0.0, 0.0, 0.0)):
        self.clock, self.pose = clock, list(pose)
        self.cmd, self.stopped_at, self.t, self.calls = (0.0, 0.0), None, clock(), []
        self.max_yaw = abs(pose[2])

    def _speed(self, t):
        v, w = self.cmd
        if self.stopped_at is None:
            return v, w
        late = max(0.0, t - self.stopped_at - P["stop_delay"])
        return (math.copysign(max(0.0, abs(v) - P["decel"] * late), v),
                math.copysign(max(0.0, abs(w) - P["turn_accel"] * late), w))

    def _advance(self):
        now = self.clock()
        while self.t < now - 1e-12:
            dt = min(0.01, now - self.t)
            v, w = self._speed(self.t)
            self.pose[2] += w * dt
            self.pose[0] += v * math.cos(self.pose[2]) * dt
            self.pose[1] += v * math.sin(self.pose[2]) * dt
            self.max_yaw = max(self.max_yaw, abs(self.pose[2]))
            self.t += dt

    def send(self, v, w):
        self._advance()
        self.calls.append(("send", v, w))
        self.cmd, self.stopped_at = (v, w), None

    def send_reverse(self, speed, w):
        self._advance()
        self.calls.append(("reverse", speed, w))
        self.cmd, self.stopped_at = (-speed, w), None

    def read_odom(self):
        self._advance()
        return tuple(self.pose)

    def stop(self):
        self._advance()
        self.calls.append("stop")
        if self.stopped_at is None:
            self.stopped_at = self.clock()

    def close(self):
        self.calls.append("close")


class FakeStream:
    updated_at = 0


class FakeSession:
    def __init__(self, test):
        self.test, self.stream = test, FakeStream()

    def latest(self):
        self.stream.updated_at += 1
        return list(self.test.scan(self.test.wheels.read_odom()))

    def close(self):
        pass


class FakeCamera:
    error = None

    def __init__(self, test):
        self.test = test

    def start(self):
        pass

    def latest(self):
        return self.test.frame(self.test.wheels.read_odom())

    def stop(self):
        pass


class Runner(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.state = Path(self.tmp.name) / "tether.json"
        self.clock = Clock()
        self.wheels = FakeWheels(self.clock)
        self.scan, self.frame, self.opened = big_room, (lambda pose: FLOOR), []

        def opener(name, make):
            def open_(*_args):
                self.opened.append(name)
                return make()
            return open_

        for target, name, value in ((tether_run, "clock", self.clock), (tether_run, "sleep", self.clock.sleep),
                                    (tether_run.lidar, "LidarSession", opener("lidar", lambda: FakeSession(self))),
                                    (tether_run.wheels_module, "Wheels", opener("wheels", lambda: self.wheels)),
                                    (tether_run.camera, "Camera", opener("camera", lambda: FakeCamera(self)))):
            patcher = mock.patch.object(target, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        env = mock.patch.dict(os.environ, {wheels.SAFE_RUN_ENV: "1"})
        env.start()
        self.addCleanup(env.stop)

    def save_state(self, pose=(0.0, 0.0, 0.0), lay=None, net_turn=0.0):
        t = tether.Tether(P, pose, lay=lay)
        t.net_turn = net_turn
        t.save(self.state)
        self.wheels.pose = list(pose)

    def run_tool(self, *argv):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = tether_run.main(["--state", str(self.state)] + [str(a) for a in argv])
        return code, json.loads(out.getvalue().strip().splitlines()[-1])

    def refused(self, *argv):
        with self.assertRaises(SystemExit) as stopped:
            tether_run.main(["--state", str(self.state)] + [str(a) for a in argv])
        self.assertIn("REFUSED", str(stopped.exception.code))
        return str(stopped.exception.code)

    def loaded(self):
        return tether.Tether.load(self.state, P)

    def moves(self, kind="send"):
        return [c for c in self.wheels.calls if c != "stop" and c != "close" and c[0] == kind and (c[1] or c[2])]


class Refusals(Runner):
    def test_refuses_without_safe_run_or_past_the_limits_before_any_port(self):
        self.save_state()
        for argv in (["init"], ["turn", 30], ["line", 0.3], ["uturn"], ["precheck-line", 0.5, 0.08]):
            with mock.patch.dict(os.environ, {wheels.SAFE_RUN_ENV: ""}):
                self.assertIn("safe_run", self.refused(*argv))
        for argv in (["line", 0.3, "--speed", 0.15], ["line", 3.0], ["line", "nan"], ["turn", 200], ["turn", "nan"],
                     ["precheck-line", 0.5, 0.2]):
            self.refused(*argv)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            tether_run.main(["uturn", "up"])
        self.assertEqual(self.opened, [])

    def test_refuses_without_a_state_or_when_the_odometry_moved_since_it_was_saved(self):
        self.assertIn("init", self.refused("line", 0.3))
        self.save_state()
        self.wheels.pose = [0.2, 0.0, 0.0]
        code, result = self.run_tool("line", 0.3)
        self.assertEqual(code, 1)
        self.assertIn("odometry", result["refused"])
        self.assertEqual(self.moves(), [])
        self.assertEqual(self.loaded().pose, (0.0, 0.0, 0.0))

    def test_net_turn_limit_refuses_a_turn_that_would_wind_the_cable(self):
        self.save_state(pose=(0.0, 0.0, math.radians(150)), net_turn=math.radians(150))
        code, result = self.run_tool("turn", 45)
        self.assertEqual(code, 1)
        self.assertIn("wind", result["refused"])
        self.assertEqual(self.moves(), [])
        code, result = self.run_tool("turn", -45)  # unwinding is fine
        self.assertEqual((code, result["stop"]), (0, "there"))
        self.assertAlmostEqual(result["net_turn_deg"], 105, delta=2)


class Line(Runner):
    def test_a_blind_camera_stops_a_line_and_the_trail_is_saved(self):
        self.save_state()
        self.frame = lambda pose: FLOOR if pose[0] < 0.1 else BLIND
        code, result = self.run_tool("line", 0.8)
        self.assertEqual((code, result["stop"]), (1, "camera blind"))
        self.assertFalse(result["blocked_ahead"])
        self.assertEqual(self.moves("reverse"), [])
        self.assertTrue(0.1 <= self.wheels.pose[0] < 0.25, self.wheels.pose)
        self.assertEqual(self.wheels.calls[-1], "close")
        self.assertAlmostEqual(self.loaded().pose[0], self.wheels.pose[0], places=6)

    def test_cable_seen_ahead_stops_a_line_short_of_it(self):
        self.save_state()
        self.frame = lambda pose: FLOOR if pose[0] < 0.1 else CABLE
        code, result = self.run_tool("line", 1.0)
        self.assertEqual((code, result["stop"]), (1, "cable ahead"))
        seen = np.array(self.loaded().seen)
        self.assertGreater(len(seen), 5)
        self.assertGreater(seen[:, 0].min(), self.wheels.pose[0] + 0.5)  # stopped well short of it
        code, result = self.run_tool("precheck-line", 0.5, 0.08)  # the cable is remembered
        self.assertEqual(code, 1)

    def test_precheck_passes_only_with_a_visible_floor_and_no_cable(self):
        self.save_state()
        self.assertEqual(self.run_tool("precheck-line", 0.5, 0.08)[0], 0)
        self.frame = lambda pose: BLIND
        self.assertEqual(self.run_tool("precheck-line", 0.5, 0.08)[1]["stop"], "camera blind")
        self.frame = lambda pose: CABLE
        self.assertEqual(self.run_tool("precheck-line", 0.5, 0.08)[1]["stop"], "cable ahead")
        self.assertEqual(self.moves(), [])

    def test_a_straight_return_over_the_trail_is_refused(self):
        self.save_state()
        self.assertEqual(self.run_tool("line", 1.0)[0], 0)
        self.assertAlmostEqual(self.wheels.pose[0], 1.0, delta=0.03)
        code, result = self.run_tool("turn", 180)  # on the spot: the trail it lays comes round the body
        self.assertEqual(code, 1)
        self.assertIn("cable", result["refused"])
        self.assertEqual(self.moves(), [("send", 0.1, 0.0)] * len(self.moves()))
        t = tether.Tether(P, (0.0, 0.0, 0.0))
        pose = drive(t, drive(t, (0.0, 0.0, 0.0), 0.1, 0.0, 10.0), 0.0, 0.3, math.pi / 0.3)  # facing back, however
        t.save(self.state)
        self.wheels.pose = list(pose)
        sent = len(self.wheels.calls)
        code, result = self.run_tool("line", 0.5)
        self.assertEqual(code, 1)
        self.assertIn("cable", result["refused"])
        self.assertEqual([c for c in self.wheels.calls[sent:] if c[0] == "send"], [])

    def test_uturn_right_after_a_straight_run_is_clear_and_ends_beside_the_trail(self):
        self.save_state()
        self.assertEqual(self.run_tool("line", 1.0)[0], 0)
        code, result = self.run_tool("uturn")
        self.assertEqual((code, result["stop"]), (0, "there"))
        x, y, th = self.wheels.pose
        self.assertAlmostEqual(abs(math.remainder(th, math.tau)), math.pi, delta=0.08)
        self.assertAlmostEqual(y, -0.6, delta=0.08)
        self.assertAlmostEqual(result["net_turn_deg"], -180, delta=5)
        self.assertEqual(self.run_tool("precheck-line", 0.5, 0.08)[0], 0)  # the way back beside the trail is clear

    def test_uturn_left_after_a_straight_run_leaves_a_way_back_over_the_cable(self):
        self.save_state()
        self.assertEqual(self.run_tool("line", 1.0)[0], 0)
        self.assertEqual(self.run_tool("uturn", "left")[0], 0)
        self.assertAlmostEqual(self.wheels.pose[1], 0.6, delta=0.08)
        self.assertEqual(self.run_tool("precheck-line", 0.5, 0.08)[1]["refused"], "the cable is in the way")


class BackUp(Runner):
    def blocked_line(self, lay, scan, *flags):
        self.save_state(lay=lay)
        self.scan = scan
        code, result = self.run_tool("line", 0.5, *flags)
        self.assertEqual((code, result["stop"]), (1, "blocked ahead"))
        self.assertTrue(result["blocked_ahead"])
        self.assertEqual(self.moves(), [])
        return result

    def test_backs_up_when_ahead_is_blocked_and_the_rear_and_the_cable_are_clear(self):
        result = self.blocked_line(LAY_LEFT, wall_ahead, "--back-up-if-blocked")
        self.assertEqual(result["back_up"], "there")
        self.assertEqual(set(self.moves("reverse")), {("reverse", 0.04, 0.0)})
        self.assertAlmostEqual(self.wheels.pose[0], -0.2, delta=0.04)
        self.assertAlmostEqual(self.loaded().pose[0], self.wheels.pose[0], places=6)

    def test_no_back_up_without_the_flag(self):
        result = self.blocked_line(LAY_LEFT, wall_ahead)
        self.assertNotIn("back_up", result)
        self.assertEqual(self.moves("reverse"), [])

    def test_no_back_up_when_the_rear_is_blocked(self):
        result = self.blocked_line(LAY_LEFT, boxed_in, "--back-up-if-blocked")
        self.assertIn("rear", result["back_up"])
        self.assertEqual(self.moves("reverse"), [])

    def test_no_back_up_onto_the_cable_behind(self):
        result = self.blocked_line(None, wall_ahead, "--back-up-if-blocked")  # the lay straight behind
        self.assertIn("avoid", result["back_up"])
        self.assertEqual(self.moves("reverse"), [])

    def test_the_cable_hanging_from_the_post_behind_blocks_backing(self):
        t = tether.Tether(P, (0.0, 0.0, 0.0))
        pose = drive(t, (0.0, 0.0, 0.0), 0.1, 0.0, 10.0)
        pose = drive(t, pose, 0.0, 0.3, math.radians(30) / 0.3, dt=0.05)  # the near cable still runs behind the body
        pts = tether_run.run_follow.scan_points(big_room(pose), P)
        obstacles = np.array([geometry.base_to_odom(q, pose) for q in t.points(pose)])  # the attached part left out
        self.assertIsNone(tether_run.back_up.blocked(pts, pose, tether_run.BACK_METRES, obstacles, P))
        self.assertIn("avoid", tether_run.back_up.blocked(pts, pose, tether_run.BACK_METRES, tether_run.whole_cable(t, pose), P))


class Survey(Runner):
    def test_init_surveys_within_the_net_turn_limit_and_returns_to_the_start_heading(self):
        code, result = self.run_tool("init")
        self.assertEqual(code, 0)
        self.assertEqual([s["deg"] for s in result["survey"]], [-60, -30, 0, 30, 60, 0])
        self.assertTrue(all(s["floor_visible"] for s in result["survey"]))
        self.assertLessEqual(self.wheels.max_yaw, math.radians(60) + 0.05)
        self.assertLess(abs(self.wheels.pose[2]), 0.03)
        t = self.loaded()
        self.assertLess(abs(t.net_turn), 0.03)
        self.assertEqual(t.length, 10.0)

    def test_init_records_the_cable_it_sees(self):
        self.frame = lambda pose: CABLE if abs(pose[2]) < 0.05 else FLOOR
        code, result = self.run_tool("init")
        self.assertGreater(len(self.loaded().seen), 5)
        self.assertGreater(max(s["cable_points"] for s in result["survey"]), 0)


class AfterBaseCheck(Runner):
    def test_feeds_the_logged_odometry_into_the_trail_without_any_port(self):
        self.save_state()
        log = Path(self.tmp.name) / "line.json"
        log.write_text(json.dumps({"odom": [[0.1 * i, 0.01 * i, 0.0, 0.0] for i in range(51)],
                                   "report": {"stop_reason": "target"}}))
        with mock.patch.dict(os.environ, {wheels.SAFE_RUN_ENV: ""}):
            code, result = self.run_tool("after-base-check", log)
        self.assertEqual(code, 0)
        t = self.loaded()
        self.assertEqual(t.pose, (0.5, 0.0, 0.0))
        self.assertAlmostEqual(t.used_length(), 1.5, places=6)
        self.assertAlmostEqual(result["used_length"], 1.5, places=2)
        self.assertEqual(self.opened, [])

    def test_refuses_a_log_that_does_not_start_at_the_saved_pose(self):
        self.save_state()
        log = Path(self.tmp.name) / "line.json"
        log.write_text(json.dumps({"odom": [[0.0, 0.3, 0.0, 0.0], [0.1, 0.4, 0.0, 0.0]]}))
        code, result = self.run_tool("after-base-check", log)
        self.assertEqual(code, 1)
        self.assertIn("odometry", result["refused"])
        self.assertEqual(self.loaded().pose, (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
