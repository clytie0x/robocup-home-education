"""The control loop with fake devices. Nothing here opens a serial port or drives the base."""
import json
import math
import signal
import socket
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy

from hmc import hri, route
from hmc.follower import Follower, Mode, Senses
from hmc.perception import MotionTracker
from hmc.world import Circle

import cable as cable_module
import command_server
import escape
import params
import run_follow
import safety
import wheels as wheels_module
from link import Answer, Seen
from test_command_server import Broken, Client
from test_legs import person, scan_of
from test_scanmatch import box, world_scan
from test_wheels import FakePort
from wheels import Wheels

P = params.load()
LOOK = (1.0, 0.0, 0.0)  # the walker's look: the fake Jetson always sends the same one
WALL = 8.0  # m, the far wall behind the walker
WALL_SPAN = 30.0  # degrees each side of the walker where that wall returns; elsewhere the hall is open
WALL_ONLY = scan_of()


class FakeBase:
    """Stands in for wheels.Wheels: remembers the command and drives a perfect base with it."""

    def __init__(self):
        self.pose = (0.0, 0.0, 0.0)
        self.command = (0.0, 0.0)
        self.sent = []
        self.stops = 0  # S# sent (stop() also counts as a (0, 0) command)
        self.asked = []  # times read_status was called (E#), by self.clock
        self.clock = lambda: None

    def send(self, v, w):
        self.command = (v, w)
        self.sent.append((v, w))

    def send_reverse(self, speed, w):
        """wheels.Wheels.send_reverse: B packets, backward at `speed`."""
        self.backed = getattr(self, "backed", 0) + 1
        self.send(-speed, w)

    def read_odom(self):
        return self.pose

    def read_status(self):
        """E# of the new firmware, from a base whose wheels turn: no fault."""
        self.asked.append(self.clock())
        return {"left": 0, "right": 0, "pwm": [0, 0], "flags": 0}

    def stop(self):
        self.stops += 1
        self.send(0.0, 0.0)

    def advance(self, dt):
        v, w = self.command
        x, y, yaw = self.pose
        self.pose = (x + v * dt * math.cos(yaw), y + v * dt * math.sin(yaw), yaw + w * dt)


class Walker:
    """One person walking in the odometry frame, seen by a fake LiDAR, camera and Jetson."""

    def __init__(self, base, at):
        self.base, self.at, self.t = base, at, 0.0  # at(t) -> (x, y) in the odometry frame

    def in_base(self):
        x, y, yaw = self.base.pose
        wx, wy = self.at(self.t)
        dx, dy = wx - x, wy - y
        return (math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy)

    # LiDAR: two legs in front of a far wall. The legs need a farther background to count as
    # legs; the rest of the scan is "no return" (0 m), so the planner has few points and the
    # test stays fast. The wall moves with the robot, but it is too far to matter.
    def latest(self):
        lx, ly = self.in_base()[0] - P["lidar_x"], self.in_base()[1]
        ahead = math.degrees(math.atan2(ly, lx))
        return [(deg, m if abs((deg - ahead + 180) % 360 - 180) <= WALL_SPAN else 0.0)
                for deg, m in scan_of(person(lx, ly), wall=WALL)]


class Jetson:
    """Answers every frame at once with the walker's bearing and look, while in the camera view."""

    def __init__(self, walker):
        self.walker = walker
        self.look = LOOK
        self.frames = 0

    def send(self, frame):
        self.frames += 1

    def latest(self):
        bx, by = self.walker.in_base()
        bearing = math.degrees(math.atan2(by - P["camera_y"], bx - P["camera_x"]))
        people = [Seen(bearing, 0.9, self.look, [0, 0, 1, 1])] if abs(bearing) <= P["camera_hfov_deg"] / 2 else []
        return Answer(self.walker.t, people)

    def alive(self, now, silence):
        return True


class Camera:
    def __init__(self):
        self.t = 0.0

    def latest(self):
        # An empty room: every pixel 4 m away, past the depth layer's 3 m, so "looked, nothing there".
        return SimpleNamespace(time=self.t, depth=numpy.full((240, 320), 4000, dtype=numpy.uint16))


class Scans:
    def __init__(self, scan):
        self.scan = scan

    def latest(self):
        return self.scan


class Pushy:
    """A brain that always wants to drive straight on at 0.1 m/s."""

    def __init__(self, robot):
        self.robot = robot
        self.mode = Mode.FOLLOW
        self.operator_look = SimpleNamespace(count=20)
        self.last_operator_pos = None
        self.people_trackers = {}  # the crowd memory reads them, as from hmc's Follower

    def score(self, look):
        return 0.0

    def watch(self, senses):
        for pid, pos in senses.people.items():
            self.people_trackers.setdefault(pid, MotionTracker()).update(senses.time, pos)

    def step(self, senses, dt):
        self.robot.speed, self.robot.turn_speed = 0.1, 0.0


class Stalling(Pushy):
    """A brain that drives, but takes too long to decide (a planner stall)."""

    def step(self, senses, dt):
        super().step(senses, dt)
        time.sleep(0.5)


class Crowded:
    """Senses that say someone else stands right at the operator."""

    locked, ambiguous, learned = True, True, 0

    def read(self, now, *rest):
        return Senses(now, None, None, {}, [])


def wheels_unwatched(state):
    """For a test of something else whose FakeBase never moves while it drives: the Pi's wheel check
    (wheels.StallWatch) would stop it after a second."""
    state.stall.update = lambda now, sent, pose: False
    return state


class Logged(unittest.TestCase):
    """A log folder and the cycle helpers; no tests of its own."""

    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.folder = Path(folder.name)
        self.log = run_follow.Log(self.folder / "run")
        self.addCleanup(self.log.close)  # cleanups run last-in first: the file closes before the folder goes

    def state(self, wheels, scan=WALL_ONLY, brain=Pushy, clock=None):
        # The cycle clock: by default the test time passed to cycle(), so nothing is ever slow.
        clock = clock or (lambda: self.now)
        state = run_follow.State(P, wheels, Scans(scan), Camera(), None, self.log, clock=clock)
        if brain is not None:
            state.follower = brain(state.body.robot)
        return state

    def cycle(self, state, now):
        self.now = now
        return run_follow.cycle(state, now)

    def records(self):
        lines = (self.folder / "run" / "log.jsonl").read_text().splitlines()
        return [json.loads(line) for line in lines]


class Loop(Logged):
    def test_dry_run_never_opens_the_wheel_port(self):
        old = {s: signal.getsignal(s) for s in run_follow.STOP_SIGNALS}
        self.addCleanup(lambda: [signal.signal(s, h) for s, h in old.items()])
        base = FakeBase()
        base.pose = (1.0, 2.0, 0.5)  # would show up in the log if the loop read the wheels
        lidar_session = Walker(base, lambda t: (1.5, 0.0))
        lidar_session.close = lambda: None
        cam = SimpleNamespace(start=lambda: None, stop=lambda: None, latest=lambda: None)
        with mock.patch.object(run_follow.wheels_module, "Wheels", side_effect=AssertionError("port opened")) as opened, \
                mock.patch.object(run_follow.lidar, "LidarSession", return_value=lidar_session), \
                mock.patch.object(run_follow.camera, "Camera", return_value=cam), \
                mock.patch.object(run_follow, "LOG_ROOT", self.folder / "logs"), \
                mock.patch("sys.stdin", SimpleNamespace(isatty=lambda: True)),                 mock.patch("builtins.input", return_value=""), mock.patch("builtins.print"):  # a console: Enter
            run_follow.main(["--seconds", "0.35", "--no-jetson"])
        opened.assert_not_called()
        logs = list((self.folder / "logs").iterdir())
        cycles = [json.loads(line) for line in (logs[0] / "log.jsonl").read_text().splitlines()]
        self.assertGreaterEqual(len(cycles), 3)
        self.assertTrue(all(r["pose"] == [0.0, 0.0, 0.0] for r in cycles))  # zero motion

    def test_broken_odometry_stops_and_ends_the_run(self):
        port = FakePort(odom=b"garbage#")
        clock = SimpleNamespace(now=0.0)
        state = self.state(Wheels(P, port=port, start_thread=False), clock=lambda: clock.now)
        end = run_follow.run(state, 5.0, clock=lambda: clock.now, sleep=lambda s: setattr(clock, "now", clock.now + s))
        self.assertTrue(end.startswith("odometry"), end)
        self.assertEqual(port.written[-1], b"S#")
        self.assertEqual(self.records()[-1]["stop"], "odometry")

    def test_slow_cycle_stops_the_wheels(self):
        port = FakePort()
        state = self.state(Wheels(P, port=port, start_thread=False))
        self.assertNotIn("end", self.cycle(state, 0.0))
        self.assertTrue(port.written[-1].startswith(b"F"))  # the brain was driving
        record = self.cycle(state, 0.5)
        self.assertEqual(record["stop"], "slow cycle")
        self.assertIn("end", record)
        self.assertEqual(port.written[-1], b"S#")

    def test_a_stalled_planner_never_sends_its_stale_command(self):
        port = FakePort()
        state = self.state(Wheels(P, port=port, start_thread=False), brain=Stalling, clock=time.monotonic)
        record = run_follow.cycle(state, time.monotonic())
        self.assertEqual(record["stop"], "slow cycle")
        self.assertIn("end", record)
        self.assertEqual(port.written[-1], b"S#")
        self.assertFalse(any(packet.startswith(b"F") for packet in port.written))

    def test_a_stranger_is_not_enrolled_mid_learn(self):
        base = FakeBase()
        walker = Walker(base, lambda t: (1.5, 0.0))
        jetson = Jetson(walker)
        state = run_follow.State(P, base, walker, Camera(), jetson, self.log, clock=lambda: walker.t)
        for i in range(5):  # learn a few looks of the operator
            walker.t = state.camera.t = i * run_follow.PERIOD
            run_follow.cycle(state, walker.t)
        learned = state.follower.operator_look.count
        self.assertTrue(0 < learned < 20)

        state.senses = run_follow.RealSenses(P)  # the track is lost
        jetson.look = (0.0, 1.0, 0.0)  # and someone else stands there now
        walker.t = state.camera.t = 0.5
        record = run_follow.cycle(state, walker.t)
        self.assertEqual(record["enrol"], "the person in view does not look like the operator")
        self.assertFalse(record["locked"])
        self.assertEqual(state.follower.operator_look.count, learned)

        jetson.look = LOOK  # the operator comes back: enrolled again
        walker.t = state.camera.t = 0.6
        self.assertEqual(run_follow.cycle(state, walker.t)["enrol"], "locked")

    def test_obstacle_in_the_braking_strip_overrides_the_brain(self):
        base = FakeBase()
        state = self.state(base)
        clear = self.cycle(state, 0.0)
        self.assertEqual((clear["v"], clear["stop"]), (0.1, None))
        self.assertEqual(state.coast.parts["v"], (0.1, 0.0))  # the last-resort stop knows the base now rolls

        base = FakeBase()
        # 0.125 m ahead of the front, inside the strip (0.163 m at 0.1 m/s with decel 0.14; 0.225 m was inside at 0.064)
        state = self.state(base, scan=scan_of([(0.15, 0.0)]))
        self.assertLess(0.15 + P["lidar_x"] - safety.FRONT, safety.braking_distance(0.1, P))
        record = self.cycle(state, 0.0)
        self.assertEqual((record["v"], record["w"], record["stop"]), (0.0, 0.0, "obstacle"))
        self.assertEqual(base.sent[-1], (0.0, 0.0))
        self.assertEqual(state.body.robot.speed, 0.0)

    def search_state(self, base, people, scan=WALL_ONLY):
        """Not locked yet, the Jetson link up, the camera answering `people` (bearings in degrees)."""
        link = SimpleNamespace(latest=lambda: Answer(self.now, [Seen(b, 0.9, LOOK, [0, 0, 1, 1]) for b in people]),
                               alive=lambda now, silence: True, send=lambda frame: None)
        state = run_follow.State(P, base, Scans(scan), Camera(), link, self.log, clock=lambda: self.now)
        state.follower = Pushy(state.body.robot)
        state.follower.operator_look = SimpleNamespace(count=0)  # nothing learned yet: enrol may lock
        return state

    def test_before_the_first_lock_it_turns_on_the_spot_to_find_the_operator(self):
        base = FakeBase()
        record = self.cycle(self.search_state(base, []), 0.0)
        self.assertEqual((record["v"], record["w"], record["search"]), (0.0, run_follow.SEARCH_TURN, "turn"))
        self.assertEqual(base.sent[-1], (0.0, run_follow.SEARCH_TURN))

    def test_a_person_near_the_middle_of_the_image_stops_the_turn_and_enrol_is_tried(self):
        base = FakeBase()
        record = self.cycle(self.search_state(base, [5.0]), 0.0)
        self.assertEqual((record["v"], record["w"], record["search"]), (0.0, 0.0, "hold"))
        self.assertIsNotNone(record["enrol"])  # enrol ran this cycle (nobody on the LiDAR there: refused)
        record = self.cycle(self.search_state(FakeBase(), [25.0]), 0.0)  # off to the side: keep turning
        self.assertEqual(record["search"], "turn")

    def test_the_hold_lasts_between_answers_and_ends_on_a_person_who_never_locks(self):
        # The link hands each answer out once; the next cycles get None. The hold must last anyway.
        answers = [Answer(0.0, [Seen(3.0, 0.9, LOOK, [0, 0, 1, 1])])]
        link = SimpleNamespace(latest=lambda: answers.pop() if answers else None,
                               alive=lambda now, silence: True, send=lambda frame: None)
        state = run_follow.State(P, FakeBase(), Scans(WALL_ONLY), Camera(), link, self.log, clock=lambda: self.now)
        state.follower = Pushy(state.body.robot)
        state.follower.operator_look = SimpleNamespace(count=0)
        got = [self.cycle(state, t)["search"] for t in (0.0, 0.1, 0.2, 0.3)]
        self.assertEqual(got, ["hold"] * 4)
        got = self.cycle(state, 0.6)["search"]  # the answer is 0.6 s old now: no fresh view, no blind turn
        self.assertEqual(got, "hold")
        self.assertEqual(state.body.robot.turn_speed, 0.0)

    def test_a_centred_person_who_never_locks_is_turned_past_after_a_while(self):
        state = self.search_state(FakeBase(), [3.0])
        holds = [self.cycle(state, round(0.1 * i, 1))["search"] for i in range(0, 35)]
        self.assertEqual(holds[:30], ["hold"] * 30)  # 3 s
        self.assertEqual(holds[30:], ["turn"] * 5)

    def test_the_search_turn_still_stops_for_what_its_outline_would_hit(self):
        r, deg = 0.30, math.radians(65)  # a leg beside the front-left corner the left turn swings toward
        scan = scan_of([(r * math.cos(deg) - P["lidar_x"], r * math.sin(deg))])
        record = self.cycle(self.search_state(FakeBase(), [], scan=scan), 0.0)
        self.assertEqual((record["v"], record["w"], record["stop"]), (0.0, 0.0, "obstacle"))

    def test_after_two_full_turns_it_stands_and_waits(self):
        base = FakeBase()
        state = self.search_state(base, [])
        state.searched = run_follow.SEARCH_MAX
        self.assertEqual(self.cycle(state, 0.0)["search"], "hold")

    def test_the_motion_is_measured_and_the_velocity_loop_corrects_a_slow_base(self):
        base = FakeBase()
        state = run_follow.State(P, base, Scans(WALL_ONLY), Camera(), None, self.log, clock=lambda: self.now,
                                 velocity_loop=True)
        state.follower = Pushy(state.body.robot)  # always wants 0.1 m/s
        state.estimates._avg["speed_gain"].value, state.estimates._avg["speed_gain"].samples = 0.8, 100
        record = None
        for i in range(3):
            record = self.cycle(state, 0.1 * i)
            base.advance(0.1)
        self.assertIsNotNone(record["measured"])
        self.assertEqual(len(record["measured"]), 5)  # v, w, trusted, agree, span
        self.assertEqual(record["estimates"]["speed_gain"], [0.8, True])
        self.assertAlmostEqual(base.sent[-1][0], 0.12, delta=0.001)  # 0.1 / 0.8 = 0.125, capped at max_speed

    def test_without_the_flag_the_brain_command_goes_out_as_it_is(self):
        base = FakeBase()
        state = self.state(base)
        self.cycle(state, 0.0)
        self.assertEqual(base.sent[-1], (0.1, 0.0))

    def test_the_page_can_pause_slow_and_end_a_run(self):
        base = FakeBase()
        state = self.state(base)
        state.controls.apply({"pause": True})
        record = self.cycle(state, 0.0)
        self.assertEqual((record["v"], record["w"], record["stop"]), (0.0, 0.0, "paused"))
        state.controls.apply({"pause": False})
        state.controls.apply({"max_speed": 0.05})
        self.cycle(state, 0.1)
        self.assertEqual(state.body.robot.max_speed, 0.05)
        state.controls.apply({"end": True})
        record = self.cycle(state, 0.2)
        self.assertIn("dashboard", record["end"])
        self.assertEqual(base.sent[-1], (0.0, 0.0))

    def test_each_cycle_puts_the_frame_scan_and_record_in_the_live_feed(self):
        state = self.state(FakeBase())
        put = []
        state.dashboard = SimpleNamespace(latest=None, feed=SimpleNamespace(put=lambda *args: put.append(args)))
        record = self.cycle(state, 0.0)
        (frame, points, overlay, now), = put
        self.assertIsNotNone(frame)  # the fake camera's frame
        self.assertIs(overlay, record)
        self.assertEqual(now, 0.0)
        numpy.testing.assert_array_equal(points, run_follow.scan_points(WALL_ONLY, P))  # the LiDAR, robot frame

    def test_a_broken_live_feed_never_stops_the_cycle(self):
        base = FakeBase()
        state = self.state(base)

        def broken(*args):
            raise RuntimeError("the live view broke")
        state.dashboard = SimpleNamespace(latest=None, feed=SimpleNamespace(put=broken))
        for i in range(3):
            record = self.cycle(state, 0.1 * i)
            self.assertNotIn("end", record)
        self.assertIs(state.dashboard.latest, record)
        self.assertEqual(base.sent[-1], (0.1, 0.0))  # still driving

    def test_each_cycle_logs_the_camera_answer(self):
        class Link:
            host = None

            def latest(self):
                return Answer(0.40, [Seen(11.0, 0.9, LOOK, [0, 0, 1, 1])])

            def alive(self, now, silence):
                return True

            def send(self, frame):
                pass

            def replies_per_s(self, now):
                return 2

        state = self.state(FakeBase())
        state.link = Link()
        record = self.cycle(state, 0.55)
        self.assertEqual(record["cam"], {"age": 0.15, "people": 1, "replies_per_s": 2})

    def test_the_camera_fields_are_empty_without_an_answer(self):
        record = self.cycle(self.state(FakeBase()), 0.0)
        self.assertEqual(record["cam"], {"age": None, "people": 0, "replies_per_s": 0})

    def test_the_page_can_switch_the_search_turn_off(self):
        state = self.search_state(FakeBase(), [])
        state.controls.apply({"search": False})
        self.assertIsNone(self.cycle(state, 0.0)["search"])

    def test_the_brain_plans_from_where_the_commands_on_their_way_will_take_it(self):
        base = FakeBase()
        state = self.state(base)
        state.history = [(9.0, 0.1, 0.0)]  # driving straight at 0.1 since 9.0 s
        self.cycle(state, 10.0)
        self.assertAlmostEqual(state.body.robot.x, base.read_odom()[0] + 0.1 * P["command_latency"], places=9)

    def test_ambiguous_scene_holds_still(self):
        base = FakeBase()
        state = self.state(base)
        state.senses = Crowded()
        record = self.cycle(state, 0.0)
        self.assertEqual((record["v"], record["w"], record["stop"]), (0.0, 0.0, "ambiguous"))
        self.assertEqual(base.sent[-1], (0.0, 0.0))

    def test_the_senses_get_the_camera_frame_for_the_depth_at_the_people(self):
        class Recording(Crowded):
            def read(self, now, *rest):
                got.append(rest)
                return super().read(now, *rest)

        got = []
        state = self.state(FakeBase())
        state.senses = Recording()
        self.cycle(state, 0.0)
        self.assertEqual(got[0][-1].depth.shape, (240, 320))  # the Xtion frame, depth and all
        state.virtual = SimpleNamespace(apply=lambda now, pose, scan, answer, alive: (scan, answer, alive), report={})
        self.cycle(state, 0.1)
        self.assertIsNone(got[1][-1])  # a simulated person has no depth: the real image would lie about them

    def test_drive_is_refused_above_0_15_with_closed_gates(self):
        closed = dict(P, max_speed=0.3, decel=None)
        self.assertIn("needs", run_follow.refusal(True, 0.16, closed))
        self.assertIsNone(run_follow.refusal(True, 0.15, closed))
        self.assertIsNone(run_follow.refusal(False, 0.16, closed))  # a dry run never moves
        self.assertIsNone(run_follow.refusal(True, 0.16, dict(P, max_speed=0.3)))  # gates open
        self.assertIn("above max_speed", run_follow.refusal(True, P["max_speed"] + 0.01, P))
        with mock.patch.object(run_follow.params, "load", return_value=closed), \
                mock.patch.object(run_follow.wheels_module, "Wheels") as opened, \
                mock.patch.object(run_follow.lidar, "LidarSession") as lidar_opened:
            with self.assertRaises(SystemExit):
                run_follow.main(["--seconds", "1", "--drive", "--max-speed", "0.16"])
        opened.assert_not_called()
        lidar_opened.assert_not_called()

    def walk(self, path, seconds=30.0):
        """Run the loop with a walker on `path` (odometry frame). Returns the records and front gaps."""
        base = FakeBase()
        walker = Walker(base, path)
        state = run_follow.State(P, base, walker, Camera(), Jetson(walker), self.log, clock=lambda: walker.t)
        records, gaps = [], []
        for i in range(int(seconds / run_follow.PERIOD)):
            now = i * run_follow.PERIOD
            walker.t = state.camera.t = now
            base.advance(run_follow.PERIOD)
            records.append(run_follow.cycle(state, now))
            gaps.append(math.dist(base.pose[:2], path(now)) - safety.FRONT)  # robot front edge to the walker
        return state, records, gaps

    def test_follows_a_simulated_walker_with_fakes(self):
        def path(t):
            # Starts 1.5 m ahead, walks a gentle S at about 0.1 m/s for 20 s, then stands still.
            t = min(t, 20.0)
            return (1.5 + 0.1 * t, 0.4 * math.sin(t / 20.0 * math.pi))

        started = time.monotonic()
        state, records, gaps = self.walk(path)
        self.assertLess(time.monotonic() - started, 10.0)
        self.assertTrue(all("end" not in r for r in records))
        locked = sum(r["locked"] for r in records) / len(records)
        self.assertGreaterEqual(locked, 0.95)
        self.assertGreaterEqual(min(gaps), 0.3)
        self.assertEqual(state.follower.mode, Mode.FOLLOW)  # learned the look, never lost the walker
        self.assertGreater(max(r["v"] for r in records), 0.0)

    # Known brain problem, kept visible: the operator's circle blocks the goal cell, so
    # plan_path ends at "the free cell closest to the goal", which jumps around the operator
    # (behind, left, right) as the estimate moves by millimetres. With the short robot
    # follow distance (0.8 m centre to centre) the robot turns on the spot chasing it.
    # It passes once hmc/navigation.py picks the near side; hmc/ is not changed in this task.
    def test_closes_up_to_a_standing_walker(self):
        state, records, gaps = self.walk(lambda t: (1.5, 0.0))
        self.assertLess(gaps[-1], P["follow_gap"] + 0.1)


class DriftingBase(FakeBase):
    """A perfect base whose odometry is not: distances 2 % long, turns 1 % large, and a heading
    drift of DRIFT rad per metre (as hmc's --odometry wheel). `pose` is the belief, `truth` the world."""

    DRIFT = 0.005

    def __init__(self):
        super().__init__()
        self.truth = (0.0, 0.0, 0.0)

    def advance(self, dt):
        v, w = self.command
        x, y, yaw = self.truth
        self.truth = (x + v * dt * math.cos(yaw), y + v * dt * math.sin(yaw), yaw + w * dt)
        x, y, yaw = self.pose
        yaw += 1.01 * w * dt + self.DRIFT * v * dt
        self.pose = (x + 1.02 * v * dt * math.cos(yaw), y + 1.02 * v * dt * math.sin(yaw), yaw)


class Room:
    """The LiDAR among fixed walls and legs (world frame), seen from the base's true pose."""

    def __init__(self, base, walls, legs=lambda t: ()):
        self.base, self.walls, self.legs = base, walls, legs
        self.t, self.n = 0.0, 0

    def latest(self):
        self.n += 1  # each revolution of the LDS-03 starts at another angle
        return world_scan(getattr(self.base, "truth", self.base.pose), self.walls, self.legs(self.t),
                          start_deg=(0.37 * self.n) % 0.9)


class Ways(Follower):
    """Drives through way points (believed frame) instead of following someone. The breadcrumbs
    are dropped as in hmc's Follower.step."""

    def __init__(self, robot, speech, ways):
        super().__init__(robot, speech)
        self.ways = list(ways)

    def step(self, senses, dt):
        self.route.add(self.body.pos)
        while self.ways and math.dist(self.body.pos, self.ways[0]) < 0.1:
            self.ways.pop(0)
        if self.ways:
            self.body.drive_toward(self.ways[0], dt)
        else:
            self.body.brake(dt)


class Shifted:
    """Keyframes whose next correction moves the believed pose sideways by `dy` and turns it by
    `turn` (Review Focus 2). dy None: no correction."""

    def __init__(self, dy, turn=0.0):
        self.dy, self.turn = dy, turn

    def add(self, pose, scan):
        pass

    def correct(self, pose, scan):
        if self.dy is None:
            return pose, {"used": False, "index": None, "share": 0.0, "shift": 0.0, "turn": 0.0, "why": "far"}
        dy, self.dy = self.dy, None
        return (pose[0], pose[1] + dy, pose[2] + self.turn), {"used": True, "index": 0, "share": 1.0,
                                                              "shift": abs(dy), "turn": self.turn, "why": "used"}


class Eyes:
    """The Jetson: every person in the camera's view, by bearing, with their look. people(t) -> [(x, y, look)]
    in the world frame, seen from the base's pose."""

    def __init__(self, base, world, people):
        self.base, self.world, self.people = base, world, people

    def send(self, frame):
        pass

    def latest(self):
        x, y, yaw = self.base.pose
        seen = []
        for px, py, look in self.people(self.world.t):
            dx, dy = px - x, py - y
            bx, by = math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy
            bearing = math.degrees(math.atan2(by - P["camera_y"], bx - P["camera_x"]))
            if abs(bearing) <= P["camera_hfov_deg"] / 2:
                seen.append(Seen(bearing, 0.9, look, [0, 0, 1, 1]))
        return Answer(self.world.t, seen)

    def alive(self, now, silence):
        return True


class Planner:
    """Stands in for remote_plan.RemotePlanner: answers every request with a straight line, and
    counts forget()."""

    def __init__(self):
        self.forgot, self.asked, self.stale = 0, 0, False
        self.close_plan = True

    def alive(self):
        return True

    def waiting(self):
        return self.stale

    def forget(self):
        self.forgot += 1

    def plan(self, start, goal, shapes, near_side=False, heading=None):
        self.asked += 1
        return None if self.stale else [tuple(start), tuple(goal)]


# A 12 m L of corridors 2 m wide: 6 m along x, a left turn, 6 m along y. Pillars on the walls
# give the scan matcher corners; a plain corridor only corrects sideways (keyframes.SLIDE_MAX).
def _pillar(x, y):
    return box(x - 0.15, y - 0.15, x + 0.15, y + 0.15)


L_WALLS = ([((-1.0, -1.0), (7.0, -1.0)), ((-1.0, 1.0), (5.0, 1.0)), ((-1.0, -1.0), (-1.0, 1.0)),
            ((7.0, -1.0), (7.0, 7.0)), ((5.0, 1.0), (5.0, 7.0)), ((5.0, 7.0), (7.0, 7.0))]
           + _pillar(1.5, -0.85) + _pillar(4.0, -0.85) + _pillar(0.5, 0.85) + _pillar(3.0, 0.85)
           + _pillar(6.85, 2.0) + _pillar(6.85, 5.0) + _pillar(5.15, 3.5) + _pillar(5.15, 1.5))
FAST = dict(P, max_speed=0.25, cable_free_m=0.0)  # the simulated base is perfect: drive faster, so the test runs in seconds; its
# 2 m corridor with pillars has no comfortable room beside the cable at 0.25 m/s: no lay (cable.Lay) there


class Return(Logged):
    def speech(self):
        return [r["say"] for r in self.records() if "say" in r]

    def carried_out(self, state, base, metres, start=0.0):
        """Following, while the base is carried 0.1 m a cycle along x. Returns the time after it."""
        now = start
        for i in range(int(round(metres / 0.1)) + 1):
            now = start + 0.1 * i
            base.pose = (0.1 * i, base.pose[1], base.pose[2])
            self.cycle(state, now)
        return now

    def drive(self, state, base, start, seconds, world=None):
        """Cycle with the base driving its commands. Returns the records; stops at an end record."""
        records = []
        for i in range(int(round(seconds / run_follow.PERIOD))):
            now = start + run_follow.PERIOD * (i + 1)
            if world is not None:
                world.t = now
            base.advance(run_follow.PERIOD)
            records.append(self.cycle(state, now))
            if "end" in records[-1]:
                break
        return records

    def test_enter_switches_to_return_and_says_so(self):
        base = FakeBase()
        state = self.state(base, brain=None)
        now = self.carried_out(state, base, 2.0)
        self.assertEqual(self.records()[-1]["phase"], "follow")
        state.go_back.set()  # Enter on the console
        record = self.cycle(state, now + 0.1)
        self.assertEqual(record["phase"], "return")
        self.assertEqual(self.speech(), [hri.RETURN_TEXT])
        self.assertIsNotNone(record["target"])

        # --return-after: by itself after that many seconds of following
        base = FakeBase()
        state = self.state(base, brain=None)
        state.return_after = 0.3
        self.assertEqual([self.cycle(state, t)["phase"] for t in (0.0, 0.1, 0.2)], ["follow"] * 3)
        self.assertIn(self.cycle(state, 0.3)["phase"], ("return", "done"))

    def test_enter_is_a_line_on_the_console_read_without_waiting(self):
        ours, theirs = socket.socketpair()  # select works on sockets everywhere, a pipe only on POSIX
        console = ours.makefile("rb", buffering=0)
        self.addCleanup(ours.close)
        self.addCleanup(console.close)
        self.assertFalse(run_follow.enter_pressed(console))  # nothing typed: back at once
        theirs.sendall(b"\n\n")  # Enter pressed twice more while the run was starting
        time.sleep(0.05)
        run_follow.discard_typed(console)
        self.assertFalse(run_follow.enter_pressed(console))  # so the run does not end at once
        theirs.sendall(b"\n")
        time.sleep(0.05)
        self.assertTrue(run_follow.enter_pressed(console))
        theirs.close()  # the end of input is not Enter
        time.sleep(0.05)
        self.assertFalse(run_follow.enter_pressed(console))
        run_follow.discard_typed(console)  # and does not hang

    def test_the_console_is_polled_in_the_cycle(self):
        base = FakeBase()
        state = self.state(base, brain=None)
        presses = [False] * 20 + [True]
        state.enter = lambda: presses.pop(0)
        self.carried_out(state, base, 2.0)  # 21 cycles
        self.assertEqual(self.records()[-1]["phase"], "return")
        self.assertEqual(presses, [])

    def test_go_back_on_the_page_is_enter_and_wait_holds_without_remote(self):
        base = FakeBase()
        state = self.state(base, brain=None)
        now = self.carried_out(state, base, 2.0)
        state.controls.command("wait")
        record = self.cycle(state, now + 0.1)
        self.assertEqual((record["phase"], record["v"], record["w"], record["stop"]), ("follow", 0.0, 0.0, "page"))
        state.controls.command("resume")
        self.assertIsNone(self.cycle(state, now + 0.2)["stop"])
        state.controls.command("return")  # Go back
        record = self.cycle(state, now + 0.3)
        self.assertEqual(record["phase"], "return")
        self.assertEqual(self.speech(), [hri.RETURN_TEXT])
        self.assertEqual([r["page_cmd"] for r in self.records() if "page_cmd" in r], ["wait", "resume", "return"])

    def test_enter_after_a_page_wait_goes_back_and_ends_the_wait(self):
        base = FakeBase()
        state = self.state(base, brain=None)
        now = self.carried_out(state, base, 2.0)
        state.controls.command("wait")
        self.assertEqual(self.cycle(state, now + 0.1)["stop"], "page")
        presses = [True]
        state.enter = lambda: bool(presses) and presses.pop()
        record = self.cycle(state, now + 0.2)
        self.assertEqual(record["phase"], "return")
        self.assertNotIn(record["stop"], ("page", "wait"))
        self.assertEqual((state.waiting, state.page_wait), (False, False))

    def test_a_console_that_went_away_is_no_enter(self):
        ours, theirs = socket.socketpair()
        console = ours.makefile("rb", buffering=0)
        theirs.close()
        console.close()
        ours.close()
        self.assertFalse(run_follow.enter_pressed(console))  # closed: no Enter, and no crash
        self.assertFalse(run_follow.console_is_tty(None))
        self.assertFalse(run_follow.console_is_tty(console))
        self.assertFalse(run_follow.console_is_tty(SimpleNamespace(isatty=lambda: False)))
        self.assertTrue(run_follow.console_is_tty(SimpleNamespace(isatty=lambda: True)))

    def test_a_keyframe_is_stored_with_each_breadcrumb(self):
        base = FakeBase()
        state = self.state(base, brain=None)
        self.carried_out(state, base, 2.0)
        points = state.follower.route.points
        self.assertEqual(len(points), 5)  # the start and every 0.5 m
        self.assertEqual(self.records()[-1]["breadcrumbs"], 5)
        self.assertEqual([tuple(round(v, 6) for v in pose[:2]) for pose, _ in state.keyframes.frames],
                         [tuple(round(v, 6) for v in pt) for pt in points])
        self.assertTrue(all(scan for _, scan in state.keyframes.frames))

    def returning(self, legs=lambda t: (), seconds=15.0):
        """Carried 3 m out in a room, then driving back for `seconds` without corrections."""
        base = FakeBase()
        world = Room(base, box(-1.5, -2.0, 4.5, 2.0), legs)
        state = run_follow.State(P, base, world, Camera(), None, self.log, clock=lambda: self.now)
        now = self.carried_out(state, base, 3.0)
        state.go_back.set()
        state.keyframes = Shifted(None)
        records = self.drive(state, base, now, seconds, world)
        return state, base, world, records

    def lay_gap(self, record):
        """m from the robot's centre to the cable laid on the way out: the left post's path, y = ATTACH[1]."""
        x, y = record["pose"][:2]
        return math.hypot(max(0.0, cable_module.ATTACH[0] - x, x - 3.0 - cable_module.ATTACH[0]), y - cable_module.ATTACH[1])

    def test_the_way_back_goes_beside_the_cable_not_on_it(self):
        state, base, world, records = self.returning(seconds=40.0)
        self.assertTrue(records[-1]["end"].startswith("done"), records[-1].get("end"))
        self.assertAlmostEqual(records[0]["cable_lay_m"], 3.0, delta=0.1)
        middle = [r for r in records if 0.5 <= r["pose"][0] <= 2.0]
        self.assertGreater(len(middle), 10)
        self.assertGreaterEqual(min(self.lay_gap(r) for r in middle), P["robot_half_width"] + 0.05)
        self.assertTrue(all(r["cable_cross"] is None for r in records))

    def test_with_no_way_beside_the_cable_it_crosses_slowly_and_logs_it(self):
        def straight(start, goal, shapes, near_side=False, heading=None):  # no way at all past the cable
            return None if any(getattr(s, "r", None) == cable_module.RADIUS for s in shapes) else [start, goal]

        with mock.patch.object(run_follow, "_plan_path", straight), mock.patch.object(run_follow, "REMOTE", None):
            state, base, world, records = self.returning(seconds=20.0)
        crossing = [r for r in records if r["cable_cross"] is not None]
        self.assertTrue(crossing)
        near = [r for r in crossing if self.lay_gap(r) < P["robot_radius"] + cable_module.CROSS_NEAR - 0.05
                and r["time"] - r["cable_cross"]["since"] >= 1.0]  # a second to slow down at the brain's ramp
        self.assertTrue(near)
        self.assertLessEqual(max(r["v"] for r in near), cable_module.CROSS_SPEED + 1e-9)
        self.assertGreater(max(r["v"] for r in records), cable_module.CROSS_SPEED)  # elsewhere at its own speed

    def test_a_pose_correction_clears_the_brains_memory_and_keeps_the_speed_limits(self):
        # Review Focus 2: the believed pose jumps 0.3 m sideways while driving back
        state, base, world, records = self.returning()
        self.assertGreater(records[-1]["v"], 0.05)  # turned round and driving back
        now = records[-1]["time"]
        state.follower.obstacle_memory[("old",)] = (now, Circle(50.0, 50.0, 0.05))  # the brain's memory: the believed frame
        state.senses.memory._depth[("old",)] = (2.0, 0.0, now)  # the senses' memory: the odometry frame
        state.keyframes = Shifted(0.3)
        state.follower.navigator._planned_at = now  # planned last cycle: only the correction can make it plan again
        records = records + self.drive(state, base, now, 0.1, world)
        jump = len(records) - 1
        self.assertEqual(records[jump]["keyframe"]["why"], "used")
        self.assertAlmostEqual(records[jump]["believed"][1] - records[jump]["pose"][1], 0.3, places=6)
        self.assertNotIn(("old",), state.follower.obstacle_memory)
        # The senses' memory stays where the odometry put it; the brain gets it moved with the correction.
        self.assertIn(("old",), state.senses.memory._depth)
        self.assertTrue(any(abs(s.x - 2.0) < 1e-9 and abs(s.y - 0.3) < 1e-9
                            for _, s in state.follower.obstacle_memory.values()))
        self.assertEqual(state.follower.navigator._planned_at, records[jump]["time"])  # replanned at once
        records = records + self.drive(state, base, records[jump]["time"], 3.0, world)
        for before, after in zip(records[jump - 1:], records[jump:]):
            self.assertIsNone(after["stop"])
            self.assertLessEqual(abs(after["v"] - before["v"]), P["accel"] * run_follow.PERIOD + 1e-9)
            self.assertLessEqual(abs(after["w"] - before["w"]), P["turn_accel"] * run_follow.PERIOD + 1e-9)
            self.assertGreaterEqual(after["v"], 0.0)  # never backward

    def test_a_turn_correction_clears_and_replans_like_a_shift(self):
        # 0.1 rad moves a point 2 m away by 0.2 m; 0.02 rad by 0.04 m, under CORRECTED
        for turn, clears in ((0.1, True), (0.02, False)):
            state, base, world, records = self.returning(seconds=12.0)
            now = records[-1]["time"]
            state.follower.obstacle_memory[("old",)] = (now, Circle(50.0, 50.0, 0.05))
            state.keyframes = Shifted(0.0, turn)
            record = self.drive(state, base, now, 0.1, world)[-1]
            self.assertEqual(record["keyframe"]["why"], "used")
            self.assertAlmostEqual(record["believed"][2] - record["pose"][2], turn, places=9)
            self.assertEqual(("old",) not in state.follower.obstacle_memory, clears, turn)

    def test_furniture_stays_furniture_across_a_correction(self):
        # A still pair of legs (a chair) is furniture. A 0.3 m jump and a 10 deg turn of the believed pose
        # must not make it look as if it moved (senses.MOVED), nor the camera bearings shift.
        state, base, world, records = self.returning(lambda t: person(2.6, 0.9), seconds=3.0)
        state.keyframes = Shifted(0.3)
        records += self.drive(state, base, records[-1]["time"], 1.0, world)
        state.keyframes = Shifted(0.0, math.radians(10))
        records += self.drive(state, base, records[-1]["time"], 2.0, world)
        self.assertEqual([r["people"] for r in records if r["people"]], [])
        self.assertEqual([r["keyframe"]["why"] for r in records].count("used"), 2)

    def test_a_standing_person_does_not_seem_to_walk_after_a_correction(self):
        # The people trackers keep their past in the believed frame: a jump must move it along, or the
        # person seems to walk (hmc's MotionTracker.direction), and the queue could take them for its tail.
        def walks_then_stands(t):
            return person(2.0 + 0.3 * min(max(t - 3.0, 0.0), 2.0), 1.2)  # 0.6 m in 2 s once driving back: a person

        state, base, world, records = self.returning(walks_then_stands, seconds=6.0)  # still 4 s, past the window
        self.assertTrue(records[-1]["people"])
        state.keyframes = Shifted(0.35)
        for i in range(20):  # the 2 s the tracker's window would hold the jump
            record = self.drive(state, base, records[-1]["time"], 0.1, world)[-1]
            records.append(record)
            self.assertEqual(record["keyframe"]["why"], "far" if i else "used")
            self.assertTrue(record["people"])
            trackers = [t for t in state.follower.people_trackers.values() if t.seen and record["time"] - t.last_time < 0.05]
            self.assertTrue(trackers)
            self.assertEqual([t.direction() for t in trackers], [None] * len(trackers), record["time"])
            self.assertTrue(all(t.is_stationary() for t in trackers))

    def test_a_new_phase_or_a_correction_drops_the_remote_answer(self):
        # remote_plan answers a request from a cycle or two ago: after a jump or a new goal it is out of date
        planner = Planner()
        with mock.patch.object(run_follow, "REMOTE", planner):
            base = FakeBase()
            state = self.state(base, brain=None)
            now = self.carried_out(state, base, 2.0)
            self.assertEqual(planner.forgot, 0)
            state.go_back.set()
            state.keyframes = Shifted(None)
            self.cycle(state, now + 0.1)
            self.assertEqual(planner.forgot, 1)  # the goal is a breadcrumb now, not the operator
            state.keyframes = Shifted(0.3)
            self.cycle(state, now + 0.2)
            self.assertEqual(planner.forgot, 2)
            planner.stale = True  # no answer since: ask again every cycle, not after REPLAN_EVERY
            asked = planner.asked
            for i in (3, 4, 5):
                self.cycle(state, now + 0.1 * i)
            self.assertEqual(planner.asked, asked + 3)

    def test_return_with_fakes_ends_within_arrive_of_the_start(self):
        # A 12 m L route out, 2 % odometry error, the corrections on: back to where it started.
        base = DriftingBase()
        world = Room(base, L_WALLS)
        state = run_follow.State(FAST, base, world, Camera(), None, self.log, clock=lambda: self.now)
        state.follower = Ways(state.body.robot, state.speech, [(6.0, 0.0), (6.0, 6.0)])
        started = time.monotonic()
        out = self.drive(state, base, -run_follow.PERIOD, 90.0, world)
        self.assertEqual(state.follower.ways, [])
        self.assertLess(math.dist(base.truth[:2], (6.0, 6.0)), 0.6)
        state.go_back.set()
        back = self.drive(state, base, out[-1]["time"], 150.0, world)
        self.assertTrue(back[-1]["end"].startswith("done"), back[-1].get("end"))
        # The corrections keep the belief on the truth (without them, measured once: 0.73 m apart and
        # 0.45 m from the start). The brain stops ARRIVE from its believed start, or a little further
        # when the last crawl takes over route.SKIP_AFTER and route.ARRIVE_BLOCKED takes over (0.31 m here).
        self.assertLess(math.dist(base.truth[:2], back[-1]["believed"][:2]), 0.03)
        self.assertLess(math.dist(base.truth[:2], (0.0, 0.0)), route.ARRIVE + 0.03)
        self.assertEqual(self.speech()[-2:], [hri.RETURN_TEXT, hri.ARRIVED_TEXT])
        used = [r for r in back if r["keyframe"] and r["keyframe"]["used"]]
        self.assertGreater(len(used), 10)
        self.assertTrue(all(r["stop"] is None for r in out + back))  # never near enough to anything to stop
        self.assertTrue(all(r["v"] >= 0.0 for r in out + back))  # never backward
        # 1488 simulated cycles; their clock is the test's, so a slow CPU changes nothing above, only the
        # wall time (about 25 s on a PC, 160 s on the Pi). A hang guard, not a speed test.
        self.assertLess(time.monotonic() - started, 600.0)

    def test_without_a_link_the_queue_is_never_joined(self):
        # Three people walk up and stand as a group beside the route for 7 s: no looks, no crowd.
        base = FakeBase()

        def group(t):
            walked = min(t, 2.0) * 0.25  # 0.5 m in 2 s, so the senses know they are people
            return sum((person(x, y + walked) for x, y in ((1.2, 0.6), (1.7, 0.7), (1.5, 1.1))), [])

        world = Room(base, box(-1.5, -2.0, 4.0, 2.5), group)
        state = run_follow.State(P, base, world, Camera(), None, self.log, clock=lambda: self.now)
        now = self.carried_out(state, base, 2.0)
        for i in range(1, 71):
            world.t = now + 0.1 * i
            self.cycle(state, world.t)
        self.assertEqual(state.queue.crowd, [])
        state.go_back.set()
        back = self.drive(state, base, world.t, 60.0, world)
        self.assertTrue(back[-1]["end"].startswith("done"), back[-1].get("end"))
        self.assertTrue(all(r["queue"] is None for r in back))
        self.assertFalse(state.joined)

    def test_enter_during_a_search_returns_from_where_it_stands(self):
        # Review Focus 3: the search's turn stops, the way back starts at the breadcrumb nearest the robot
        link = SimpleNamespace(latest=lambda: Answer(self.now, []), alive=lambda now, silence: True,
                               send=lambda frame: None)  # up, nobody in view: before the first lock it turns
        base = FakeBase()
        state = run_follow.State(P, base, Scans(WALL_ONLY), Camera(), link, self.log, clock=lambda: self.now)
        now = self.carried_out(state, base, 2.0)
        self.assertEqual(self.records()[-1]["search"], "turn")
        state.follower.mode = Mode.SEARCH
        base.pose = (1.0, 0.6, math.pi / 2)  # it drove off to look for the operator, past REACHED from (1.0, 0)
        state.go_back.set()
        record = self.cycle(state, now + 0.1)
        self.assertEqual(record["phase"], "return")
        self.assertIsNone(record["search"])
        self.assertNotEqual(record["w"], run_follow.SEARCH_TURN)
        self.assertEqual(record["target"], [1.0, 0.0])

    def test_an_ambiguous_scene_does_not_hold_the_way_back(self):
        # "someone is right at the operator" is about whom to follow; on the way back nobody is followed
        base = FakeBase()
        state = self.state(base, brain=None)
        now = self.carried_out(state, base, 2.0)
        state.go_back.set()
        self.cycle(state, now + 0.1)
        state.senses = Crowded()
        record = self.cycle(state, now + 0.2)
        self.assertIsNone(record["stop"])
        self.assertNotEqual((record["v"], record["w"]), (0.0, 0.0))  # turning round toward the breadcrumbs

    def test_done_stops_the_wheels_and_ends_the_run(self):
        # Review Focus 3 as well: Enter before the robot has moved, so it is at the start at once
        base = FakeBase()
        state = self.state(base, brain=None)
        clock = SimpleNamespace(now=0.0)
        state.clock = lambda: clock.now
        state.go_back.set()
        end = run_follow.run(state, 5.0, clock=lambda: clock.now, sleep=lambda s: setattr(clock, "now", clock.now + s))
        self.assertTrue(end.startswith("done"), end)
        record = self.records()[-1]
        self.assertEqual((record["phase"], record["v"], record["w"]), ("done", 0.0, 0.0))
        self.assertEqual(base.sent[-1], (0.0, 0.0))
        self.assertEqual(base.stops, 1)  # S#
        self.assertEqual(self.speech(), [hri.RETURN_TEXT, hri.ARRIVED_TEXT])

    def test_arriving_while_still_rolling_sends_a_stop_not_the_brains_braking(self):
        base = FakeBase()
        state = self.state(base, brain=None)
        self.cycle(state, 0.0)  # the start: (0, 0), heading 0
        base.pose = (-0.25, 0.0, 0.0)  # back within ARRIVE, facing the start heading
        state.body.robot.speed = 0.1  # still rolling: the brain brakes from 0.1 (0.075 after this cycle)
        state.go_back.set()
        record = self.cycle(state, 0.1)
        self.assertTrue(record["end"].startswith("done"))
        self.assertEqual((record["v"], record["w"]), (0.0, 0.0))
        self.assertEqual(base.sent[-2:], [(0.0, 0.0), (0.0, 0.0)])  # this cycle's command, then S#
        self.assertEqual(base.stops, 1)

    def test_joins_the_queue_of_the_crowd_on_the_way_back(self):
        # Three people stand as a group ahead (the crowd, remembered by look); on the way back one of them
        # walks toward the start past the robot: the robot walks behind them, then finishes on its own.
        crowd = [(4.88, 0.69, (0.0, 1.0, 0.0)), (4.53, 1.29, (0.0, 0.0, 1.0)), (4.15, 1.99, (0.0, 0.7071, 0.7071))]
        walk = {"from": None}

        def people(t):
            x, y, look = crowd[0]
            if walk["from"] is not None:  # walks to (-3, 0.6) at 0.45 m/s, past the robot into its view
                s = min(0.45 * (t - walk["from"]), math.dist((x, y), (-3.0, 0.6)))
                a = math.atan2(0.6 - y, -3.0 - x)
                x, y = x + s * math.cos(a), y + s * math.sin(a)
            return [(x, y, look)] + crowd[1:]

        base = FakeBase()
        world = Room(base, box(-4.0, -2.5, 7.0, 3.0), lambda t: sum((person(x, y) for x, y, _ in people(t)), []))
        # No cable lay (cable.Lay): beside it the turnaround is a goal behind, it unwinds to the right and the camera
        # never sees the one who walks by on the left (measured 2026-11-15).
        state = run_follow.State(dict(P, cable_free_m=0.0), base, world, Camera(), Eyes(base, world, people), self.log,
                                 clock=lambda: self.now)
        wheels_unwatched(state)  # the base stands still for the 7 s facing the crowd below, whatever it is sent
        for _ in range(20):
            state.follower.operator_look.add(LOOK)  # learned already: nobody in the crowd is enrolled
        now = self.carried_out(state, base, 3.0)
        base.pose = (3.0, 0.0, 0.7)  # facing the crowd
        for i in range(1, 71):  # 7 s
            world.t = now + 0.1 * i
            self.cycle(state, world.t)
        self.assertGreaterEqual(len(state.queue.crowd), 1)
        walk["from"] = world.t
        state.go_back.set()
        back = self.drive(state, base, world.t, 90.0, world)
        joined = [r for r in back if r["queue"] is not None]
        self.assertTrue(joined)
        self.assertTrue(back[-1]["end"].startswith("done"), back[-1].get("end"))
        self.assertTrue(all(r["v"] >= 0.0 for r in back))


STATUS_FIELDS = {"t", "phase", "waiting", "held", "locked", "operator_m", "breadcrumbs", "queue_tail", "at_start",
                 "stop", "ack", "ack_by", "wheels", "cable_deg", "bag", "mode"}


class DeadMotors(FakeBase):
    """The motor supply is gone (2026-11-14): the firmware still answers, the wheels never turn.
    old_firmware: E# does not parse. Else the firmware's encoder-fault flags (03) are up while driving,
    until S# (a stop, or a zero command, which is S#) clears them."""

    def __init__(self, old_firmware=False):
        super().__init__()
        self.old_firmware = old_firmware
        self.fault = False
        self.alive = False  # True: the power is back, the wheels turn

    def send(self, v, w):
        super().send(v, w)
        self.fault = (v, w) != (0.0, 0.0)

    def advance(self, dt):
        if self.alive:
            super().advance(dt)  # else no motor power

    def read_status(self):
        self.asked.append(self.clock())
        if self.old_firmware:
            raise ValueError("bad status 'Ox:+000.00,y:+000.00,th:+000.00#'")
        return {"left": 0, "right": 0, "pwm": [76, 76] if self.fault else [0, 0], "flags": 3 if self.fault else 0}


class WheelWatch(Logged):
    """Task 1: wheels that do not turn are found two ways, stop the robot, say so and end the run at the third time."""

    def run_dead(self, base, seconds, state=None, resume_at=()):
        """Cycles every 0.1 s; Resume pressed on the page at the times in resume_at."""
        state = state or self.state(base)
        if base is not None:
            base.clock = lambda: self.now
        records = []
        for i in range(round(seconds / 0.1)):
            if round(0.1 * i, 1) in resume_at:
                state.controls.command("resume")
            records.append(self.cycle(state, round(0.1 * i, 1)))
            if "end" in records[-1]:
                break
            if base is not None:
                base.advance(0.1)
        return state, records

    def said(self):
        return [r["say"] for r in self.records() if "say" in r]

    def test_wheels_that_do_not_turn_stop_say_hold_and_end_the_run_the_third_time(self):
        base = DeadMotors(old_firmware=True)  # only the Pi's own check can find it
        state, records = self.run_dead(base, 20.0, resume_at=(4.0, 8.0))
        by_time = {r["time"]: r for r in records}
        self.assertEqual((by_time[0.9]["v"], by_time[0.9]["stop"], by_time[0.9]["wheels"]), (0.1, None, None))
        first = by_time[1.0]  # one second of 0.1 m/s that moved nothing
        self.assertEqual((first["v"], first["w"], first["stop"], first["wheels"]), (0.0, 0.0, "wheels", "not turning"))
        self.assertGreaterEqual(base.stops, 1)  # S#: clears the firmware's fault
        held = [r for r in records if 1.0 <= r["time"] < 4.0]
        self.assertTrue(all((r["v"], r["stop"]) == (0.0, "wheels") for r in held))
        self.assertEqual((by_time[4.0]["v"], by_time[4.0]["stop"], by_time[4.0]["wheels"]), (0.1, None, None))  # resumed
        self.assertEqual(by_time[5.0]["stop"], "wheels")  # the second time
        end = records[-1]
        self.assertEqual(end["time"], 9.0)  # the third time ends the run
        self.assertEqual(end["stop"], "wheels not turning")
        self.assertIn("motor power", end["end"])
        self.assertEqual(base.sent[-1], (0.0, 0.0))
        self.assertEqual(self.said(), [run_follow.WHEELS_TEXT] * 3)  # once per time, not every held cycle
        # the old firmware's E# does not parse: asked WHEELS_MISSES times, then never again this run
        self.assertEqual(len(base.asked), run_follow.WHEELS_MISSES)
        self.assertEqual(len([r for r in self.records() if "firmware" in str(r.get("wheels"))]), 1)

    def test_the_count_is_forgotten_after_good_motion(self):
        base = DeadMotors(old_firmware=True)
        state = self.state(base)
        base.clock = lambda: self.now
        records = []
        for i in range(250):
            now = round(0.1 * i, 1)
            base.alive = 4.0 <= now < 8.0  # dead, then 4 s of good driving (0.4 m), then dead again
            if now in (4.0, 12.0, 16.0):
                state.controls.command("resume")  # someone checked the power each time
            records.append(self.cycle(state, now))
            if "end" in records[-1]:
                break
            base.advance(0.1)
        episodes = [r["episode"] for r in self.records() if "episode" in r]
        self.assertEqual(episodes, [1, 1, 2, 3])  # 0.3 m of driving forgot the first
        self.assertEqual(records[-1]["time"], 17.0)
        self.assertTrue(any("forgotten" in str(r.get("wheels")) for r in self.records()))

    def test_the_wheel_hold_stays_until_an_explicit_resume(self):
        # the robot asks to check the motor power; someone plugs the battery in with a hand on the robot
        base = DeadMotors(old_firmware=True)
        state, records = self.run_dead(base, 1.1)
        self.assertEqual(records[-1]["stop"], "wheels")
        base.alive = True  # the power is back: the robot must not drive off by itself
        for i in range(11, 120):
            record = self.cycle(state, round(0.1 * i, 1))
            base.advance(0.1)
        self.assertEqual((record["v"], record["w"], record["stop"], record["wheels"]), (0.0, 0.0, "wheels", "not turning"))
        state.controls.command("resume")
        record = self.cycle(state, 12.0)
        self.assertEqual((record["v"], record["stop"], record["wheels"]), (0.1, None, None))
        self.assertEqual([r["episode"] for r in self.records() if "episode" in r], [1])

    def test_a_failed_wheel_write_ends_as_wheels_port(self):
        class Unplugged(FakeBase):
            def send(self, v, w):
                raise OSError("write failed")

        record = self.cycle(self.state(Unplugged()), 0.0)
        self.assertEqual(record["stop"], "wheels port")

    def test_the_firmware_fault_flags_find_it_first(self):
        base = DeadMotors()
        state, records = self.run_dead(base, 2.0)
        by_time = {r["time"]: r for r in records}
        self.assertEqual(by_time[0.5]["stop"], "wheels")  # the second E#, 0.5 s after the first
        self.assertEqual(by_time[0.5]["wheels"], "fault 03")
        self.assertTrue(all(b - a >= run_follow.WHEELS_POLL - 1e-9 for a, b in zip(base.asked, base.asked[1:])))
        self.assertEqual(base.asked[:2], [0.0, 0.5])
        self.assertEqual(self.said(), [run_follow.WHEELS_TEXT])

    def test_turning_wheels_and_a_dry_run_are_left_alone(self):
        base = FakeBase()
        _, records = self.run_dead(base, 3.0)
        self.assertTrue(all(r["stop"] is None and r["wheels"] is None for r in records))
        self.assertEqual(len(base.asked), 6)  # every 0.5 s
        _, records = self.run_dead(None, 3.0, state=self.state(None))  # no wheels: nothing to watch
        self.assertTrue(all(r["stop"] is None and r["wheels"] is None for r in records))
        self.assertEqual(self.said(), [])



class Remote(Logged):
    """--remote: the Jetson's commands over TCP instead of Enter (integration design D2)."""

    def setUp(self):
        super().setUp()
        planner = mock.patch.object(run_follow, "REMOTE", None)  # main() in another test leaves its planner there
        planner.start()
        self.addCleanup(planner.stop)
        self.server = command_server.CommandServer("127.0.0.1", 0)
        self.server.start()
        self.addCleanup(self.server.stop)

    def remote_state(self, base, brain=Pushy, lidar=None, link=None):
        state = run_follow.State(P, base, lidar or Scans(WALL_ONLY), Camera(), link, self.log, clock=lambda: self.now,
                                 remote=self.server)
        if brain is not None:
            state.follower = brain(state.body.robot)
        return state

    def client(self):
        client = Client(self.server.port)
        self.addCleanup(client.close)
        return client

    def test_without_remote_nothing_changes(self):
        record = self.cycle(self.state(FakeBase()), 0.0)
        self.assertEqual(record["phase"], "follow")
        self.assertNotIn("remote", record)

    def test_idle_does_not_move_until_follow(self):
        base = FakeBase()
        state = self.remote_state(base)
        jetson = self.client()
        for t in (0.0, 0.1, 0.2):
            record = self.cycle(state, t)
            self.assertEqual((record["phase"], record["v"], record["w"], record["stop"]), ("idle", 0.0, 0.0, "idle"))
        self.assertEqual(set(base.sent), {(0.0, 0.0)})
        self.assertEqual(jetson.read()[-1]["phase"], "IDLE")
        jetson.cmd("follow")
        record = self.cycle(state, 0.3)
        self.assertEqual((record["phase"], record["v"], record["stop"]), ("follow", 0.1, None))
        self.assertEqual(base.sent[-1], (0.1, 0.0))
        status = jetson.read()[-1]
        self.assertEqual((status["phase"], status["ack"], status["waiting"]), ("FOLLOW", "follow", False))
        self.assertEqual(record["remote"], {"waiting": False, "held": None, "silent_s": 0.0, "last_cmd": "follow"})

    def test_idle_senses_and_enrols_the_operator_without_moving(self):
        base = FakeBase()
        walker = Walker(base, lambda t: (1.5, 0.0))
        state = self.remote_state(base, brain=None, lidar=walker, link=Jetson(walker))
        jetson = self.client()
        for i in range(5):
            walker.t = state.camera.t = i * run_follow.PERIOD
            record = self.cycle(state, walker.t)
        self.assertEqual((record["phase"], record["locked"]), ("idle", True))
        self.assertGreater(state.follower.operator_look.count, 0)
        self.assertEqual(set(base.sent), {(0.0, 0.0)})
        self.assertEqual(state.follower.route.points, [(0.0, 0.0)])  # no breadcrumbs while standing
        status = jetson.read()[-1]
        self.assertTrue(status["locked"])
        self.assertAlmostEqual(status["operator_m"], 1.5, delta=0.1)  # base centre to the operator

    def test_wait_holds_and_resume_continues(self):
        base = FakeBase()
        state = self.remote_state(base)
        jetson = self.client()
        jetson.cmd("follow")
        self.assertEqual(self.cycle(state, 0.0)["v"], 0.1)
        jetson.cmd("wait")
        for t in (0.1, 0.2):
            record = self.cycle(state, t)
            self.assertEqual((record["phase"], record["v"], record["w"], record["stop"]), ("follow", 0.0, 0.0, "wait"))
        self.assertEqual(base.sent[-1], (0.0, 0.0))
        self.assertEqual(state.body.robot.speed, 0.0)
        status = jetson.read()[-1]
        self.assertEqual((status["phase"], status["waiting"], status["ack"], status["stop"]), ("FOLLOW", True, "wait", "wait"))
        jetson.cmd("resume")
        record = self.cycle(state, 0.3)
        self.assertEqual((record["v"], record["stop"]), (0.1, None))
        self.assertEqual(jetson.read()[-1]["waiting"], False)

    def test_return_switches_phase_and_says_the_text(self):
        base = FakeBase()
        state = self.remote_state(base, brain=None)
        jetson = self.client()
        jetson.cmd("follow")
        for i in range(21):  # carried 2 m out while following
            base.pose = (0.1 * i, 0.0, 0.0)
            self.cycle(state, 0.1 * i)
        jetson.cmd("wait")  # at the car: the flow holds the robot, then sends return without resume
        self.cycle(state, 2.1)
        jetson.cmd("return")
        record = self.cycle(state, 2.2)
        self.assertEqual(record["phase"], "return")
        self.assertIsNotNone(record["target"])
        self.assertIsNone(record["stop"])  # return ends the wait
        self.assertEqual([r["say"] for r in self.records() if "say" in r], [hri.RETURN_TEXT])
        status = jetson.read()[-1]
        self.assertEqual((status["phase"], status["ack"], status["waiting"], status["breadcrumbs"], status["at_start"]),
                         ("RETURN", "return", False, 5, False))

    def test_back_at_the_start_reports_at_start_and_ends(self):
        base = FakeBase()
        state = self.remote_state(base, brain=None)
        jetson = self.client()
        jetson.cmd("follow")
        self.cycle(state, 0.0)
        jetson.cmd("return")  # before it moved: it is at the start at once
        record = self.cycle(state, 0.1)
        self.assertTrue(record["end"].startswith("done"), record)
        self.assertEqual(base.stops, 1)  # S#
        status = jetson.read()[-1]
        self.assertEqual((status["phase"], status["at_start"], status["ack"]), ("DONE", True, "return"))

    def test_stop_ends_the_run_with_s_hash(self):
        base = FakeBase()
        state = self.remote_state(base)
        jetson = self.client()
        jetson.cmd("follow")
        self.cycle(state, 0.0)
        jetson.cmd("stop")
        record = self.cycle(state, 0.1)
        self.assertEqual(record["stop"], "remote stop")
        self.assertIn("end", record)
        self.assertEqual(base.stops, 1)
        self.assertEqual(base.sent[-1], (0.0, 0.0))
        status = jetson.read()[-1]
        self.assertEqual((status["phase"], status["ack"], status["at_start"], status["stop"]),
                         ("DONE", "stop", False, "remote stop"))
        # and run() ends with it
        state = self.remote_state(FakeBase())
        jetson.send(b'{"cmd": "follow"}\n{"cmd": "stop"}\n')  # in order: following, so stop is taken
        clock = SimpleNamespace(now=0.0)
        state.clock = lambda: clock.now
        end = run_follow.run(state, 5.0, clock=lambda: clock.now, sleep=lambda s: setattr(clock, "now", clock.now + s))
        self.assertTrue(end.startswith("remote stop"), end)

    def test_two_seconds_of_silence_hold_until_resume_and_a_ping_does_not_release(self):
        base = FakeBase()
        state = wheels_unwatched(self.remote_state(base))
        jetson = self.client()
        jetson.cmd("follow")
        for i in range(21):  # 0.0 .. 2.0 s: still heard
            record = self.cycle(state, round(0.1 * i, 1))
            self.assertEqual((record["v"], record["stop"]), (0.1, None), record["time"])
        record = self.cycle(state, 2.1)
        self.assertEqual((record["phase"], record["v"], record["w"], record["stop"]), ("follow", 0.0, 0.0, "silent"))
        self.assertAlmostEqual(record["remote"]["silent_s"], 2.1)
        self.assertEqual(base.sent[-1], (0.0, 0.0))
        self.assertTrue(any("silent" in str(r.get("remote")) for r in self.records() if "pose" not in r))
        jetson.cmd("ping")
        record = self.cycle(state, 2.2)
        self.assertEqual((record["v"], record["stop"], record["remote"]["held"]), (0.0, "silent", "silent"))
        self.assertEqual(record["remote"]["last_cmd"], "follow")  # a ping is not acked as a command
        self.assertEqual(jetson.read()[-1]["held"], "silent")
        record = self.cycle(state, 2.3)  # heard again 0.1 s ago, but only a ping: still held
        self.assertEqual((record["v"], record["stop"], record["remote"]["silent_s"]), (0.0, "silent", 0.1))
        jetson.cmd("resume")
        record = self.cycle(state, 2.4)
        self.assertEqual((record["v"], record["stop"], record["remote"]["held"]), (0.1, None, None))
        self.assertTrue(any("released" in str(r.get("remote")) for r in self.records()))

    def test_a_malformed_line_gets_an_error_and_changes_nothing(self):
        base = FakeBase()
        state = self.remote_state(base)
        jetson = self.client()
        jetson.cmd("follow")
        self.cycle(state, 0.0)
        jetson.read()
        jetson.send(b'not json\n{"cmd": "fly"}\n["wait"]\n{"cmd": "wait"\n\n')
        record = self.cycle(state, 0.1)
        replies = jetson.read()
        self.assertEqual(len([r for r in replies if "error" in r]), 5)
        self.assertEqual((record["phase"], record["v"], record["stop"]), ("follow", 0.1, None))
        self.assertEqual(record["remote"], {"waiting": False, "held": None, "silent_s": 0.1, "last_cmd": "follow"})  # not heard

    def test_a_second_client_replaces_the_first(self):
        state = self.remote_state(FakeBase())
        first = self.client()
        first.cmd("follow")
        self.cycle(state, 0.0)
        second = self.client()
        second.cmd("wait")
        record = self.cycle(state, 0.1)
        self.assertEqual(record["stop"], "wait")
        first.read()
        self.assertTrue(first.closed)
        self.assertEqual(second.read()[-1]["ack"], "wait")
        self.assertTrue(any("replaced" in str(r.get("remote")) for r in self.records() if "pose" not in r))

    def test_status_lines_carry_the_fields_at_most_5_hz(self):
        state = self.remote_state(FakeBase())
        jetson = self.client()
        jetson.cmd("follow")
        for i in range(10):
            self.cycle(state, i * 0.1)
        statuses = jetson.read()
        self.assertEqual([round(s["t"], 1) for s in statuses], [0.0, 0.2, 0.4, 0.6, 0.8])
        for s in statuses:
            self.assertEqual(set(s), STATUS_FIELDS)
            self.assertEqual((s["phase"], s["waiting"], s["held"], s["locked"], s["operator_m"], s["queue_tail"],
                              s["at_start"], s["stop"], s["ack"], s["ack_by"], s["cable_deg"]),
                             ("FOLLOW", False, None, False, None, None, False, None, "follow", "client", 0.0))
            self.assertIsInstance(s["breadcrumbs"], int)
        # The cycle log carries the camera fields too (the status line above does not need them).
        self.assertEqual(set(self.records()[-1]["cam"]), {"age", "people", "replies_per_s"})

    def test_operator_m_is_the_kept_operator_while_locked_but_not_seen(self):
        class LockedBlind:
            locked, ambiguous, learned = True, False, 20

            def read(self, now, *rest):
                return Senses(now, None, None, {}, [])

        state = self.remote_state(FakeBase())
        state.senses = LockedBlind()
        state.follower.last_operator_pos = (1.2, 0.0)
        jetson = self.client()
        jetson.cmd("follow")
        self.cycle(state, 0.0)
        self.assertAlmostEqual(jetson.read()[-1]["operator_m"], 1.2)
        state.senses.locked = False
        jetson.cmd("ping")
        self.cycle(state, 0.1)
        self.assertIsNone(jetson.read()[-1]["operator_m"])

    def quiet(self, state, start, until):
        """Cycles every 0.1 s from `start` to `until` with nothing sent. Returns the last record."""
        record = None
        for i in range(int(round(start * 10)), int(round(until * 10)) + 1):
            record = self.cycle(state, round(0.1 * i, 1))
        return record

    def test_a_disconnect_holds_still_after_two_seconds_until_a_command(self):
        base = FakeBase()
        state = wheels_unwatched(self.remote_state(base))
        jetson = self.client()
        jetson.cmd("follow")
        self.cycle(state, 0.0)
        jetson.close()  # the Wi-Fi drops
        record = self.quiet(state, 0.1, 2.0)
        self.assertEqual((record["v"], record["stop"]), (0.1, None))
        record = self.cycle(state, 2.1)
        self.assertEqual((record["v"], record["w"], record["stop"]), (0.0, 0.0, "silent"))
        self.assertIsNone(state.remote.client)  # "closed by the client", or a reset: it had unread status lines
        self.assertEqual(len([r for r in self.records() if "remote" in r and "pose" not in r]), 3)  # connect, gone, silent
        again = self.client()
        again.cmd("ping")
        record = self.cycle(state, 2.2)
        self.assertEqual((record["stop"], record["remote"]["held"]), ("silent", "silent"))
        self.assertEqual(again.read()[-1]["held"], "silent")
        self.assertEqual(self.cycle(state, 2.3)["stop"], "silent")  # pings alone never release it
        again.cmd("follow")  # the bridge sends its intended state again after a reconnect
        record = self.cycle(state, 2.4)
        self.assertEqual((record["v"], record["stop"], record["remote"]["held"]), (0.1, None, None))

    def test_no_client_at_all_latches_the_silence_hold(self):
        base = FakeBase()
        state = self.remote_state(base)
        record = self.quiet(state, 0.0, 2.1)
        self.assertEqual((record["phase"], record["stop"], record["remote"]["held"]), ("idle", "idle", "silent"))
        jetson = self.client()
        jetson.cmd("follow")
        record = self.cycle(state, 2.2)
        self.assertEqual((record["phase"], record["v"], record["stop"]), ("follow", 0.1, None))

    def test_only_malformed_lines_past_two_seconds_still_hold(self):
        base = FakeBase()
        state = wheels_unwatched(self.remote_state(base))
        jetson = self.client()
        jetson.cmd("follow")
        for i in range(22):
            if i % 5 == 1:
                jetson.send(b"hello\n")
            record = self.cycle(state, round(0.1 * i, 1))
        self.assertEqual((record["v"], record["stop"]), (0.0, "silent"))
        self.assertGreater(len(jetson.errors()), 0)

    def returning_state(self, jetson):
        """Remote, the real brain, carried 2 m out while following, then return sent. Returns (state, base, now)."""
        base = FakeBase()
        state = self.remote_state(base, brain=None)
        jetson.cmd("follow")
        for i in range(21):
            base.pose = (0.1 * i, 0.0, 0.0)
            self.cycle(state, round(0.1 * i, 1))
        jetson.cmd("return")
        self.assertEqual(self.cycle(state, 2.1)["phase"], "return")
        return state, base, 2.1

    def test_follow_during_return_keeps_returning(self):
        jetson = self.client()
        state, base, now = self.returning_state(jetson)
        target = state.returner.target
        jetson.cmd("follow")
        record = self.cycle(state, now + 0.1)
        self.assertEqual((record["phase"], record["remote"]["last_cmd"]), ("return", "follow"))
        self.assertEqual(state.returner.target, target)
        self.assertEqual([r["say"] for r in self.records() if "say" in r], [hri.RETURN_TEXT])

    def test_wait_during_return_holds_and_resume_drives_on(self):
        jetson = self.client()
        state, base, now = self.returning_state(jetson)
        jetson.cmd("wait")
        record = self.cycle(state, now + 0.1)
        self.assertEqual((record["phase"], record["v"], record["w"], record["stop"]), ("return", 0.0, 0.0, "wait"))
        self.assertEqual(jetson.read()[-1]["held"], "wait")
        jetson.cmd("resume")
        record = self.cycle(state, now + 0.2)
        self.assertEqual((record["phase"], record["stop"]), ("return", None))
        self.assertNotEqual((record["v"], record["w"]), (0.0, 0.0))  # turning round toward the breadcrumbs

    def test_return_while_idle_is_refused(self):
        state = self.remote_state(FakeBase())
        jetson = self.client()
        self.cycle(state, 0.0)
        jetson.cmd("return")
        record = self.cycle(state, 0.1)
        self.assertEqual((record["phase"], record["remote"]["last_cmd"]), ("idle", None))
        self.assertIn("not following yet", jetson.errors())
        self.assertEqual([r for r in self.records() if "say" in r], [])
        self.assertFalse(state.go_back.is_set())

    def test_stop_while_idle_is_refused(self):
        base = FakeBase()
        state = self.remote_state(base)
        jetson = self.client()
        self.cycle(state, 0.0)
        jetson.cmd("stop")
        record = self.cycle(state, 0.1)
        self.assertNotIn("end", record)
        self.assertEqual(record["phase"], "idle")
        self.assertEqual(base.stops, 0)
        self.assertIn("idle: nothing to stop", jetson.errors())

    def test_stop_while_waiting_ends_the_run(self):
        base = FakeBase()
        state = self.remote_state(base)
        jetson = self.client()
        jetson.cmd("follow")
        self.cycle(state, 0.0)
        jetson.cmd("wait")
        self.cycle(state, 0.1)
        jetson.cmd("stop")
        record = self.cycle(state, 0.2)
        self.assertEqual(record["stop"], "remote stop")
        self.assertEqual(base.stops, 1)

    def test_the_way_back_starts_where_follow_came_not_where_the_program_started(self):
        base = FakeBase()
        state = self.remote_state(base, brain=None)
        jetson = self.client()
        self.cycle(state, 0.0)
        base.pose = (1.0, 0.5, 0.3)  # carried to the start line before follow
        self.cycle(state, 0.1)
        jetson.cmd("follow")
        self.cycle(state, 0.2)
        self.assertEqual(state.start, (1.0, 0.5, 0.3))
        self.assertEqual(state.started, 0.2)
        self.assertEqual(state.follower.route.points, [(1.0, 0.5)])
        self.assertEqual([pose for pose, _ in state.keyframes.frames], [(1.0, 0.5, 0.3)])
        jetson.cmd("return")  # straight back: it is at the start already, not 1.1 m from it
        record = self.cycle(state, 0.3)
        self.assertTrue(record["end"].startswith("done"), record)
        self.assertEqual(jetson.read()[-1]["at_start"], True)

    def test_the_remote_holds_send_zero_with_the_velocity_loop(self):
        base = FakeBase()
        state = run_follow.State(P, base, Scans(WALL_ONLY), Camera(), None, self.log, clock=lambda: self.now,
                                 velocity_loop=True, remote=self.server)
        state.follower = Pushy(state.body.robot)
        wheels_unwatched(state)
        state.estimates._avg["speed_gain"].value, state.estimates._avg["speed_gain"].samples = 0.8, 100
        jetson = self.client()
        self.assertEqual(self.cycle(state, 0.0)["stop"], "idle")
        self.assertEqual(base.sent[-1], (0.0, 0.0))
        jetson.cmd("follow")
        self.cycle(state, 0.1)
        base.advance(0.1)
        self.assertGreater(base.sent[-1][0], 0.1)  # the loop corrects the slow base
        jetson.cmd("wait")
        self.assertEqual(self.cycle(state, 0.2)["stop"], "wait")
        self.assertEqual(base.sent[-1], (0.0, 0.0))
        jetson.cmd("resume")
        self.cycle(state, 0.3)
        record = self.quiet(state, 0.4, 2.4)  # resume heard at 0.3
        self.assertEqual(record["stop"], "silent")
        self.assertEqual(base.sent[-1], (0.0, 0.0))

    def test_a_socket_error_does_not_escape_the_cycle(self):
        base = FakeBase()
        state = self.remote_state(base)
        jetson = self.client()
        jetson.cmd("follow")
        self.cycle(state, 0.0)
        state.remote.client.close()
        state.remote.client = Broken()
        record = self.cycle(state, 0.1)  # no exception
        self.assertEqual((record["phase"], record["v"]), ("follow", 0.1))
        self.assertTrue(any("broke" in str(r.get("remote")) for r in self.records()))

    def test_the_wheel_hold_says_held_wheels_and_the_clients_resume_releases_it(self):
        base = DeadMotors()
        base.clock = lambda: self.now
        state = self.remote_state(base)
        jetson = self.client()
        jetson.cmd("follow")
        for i in range(30):  # found at 0.5 s, then held for good
            record = self.cycle(state, round(0.1 * i, 1))
            if i % 10 == 9:
                jetson.cmd("ping")
        self.assertEqual((record["stop"], record["remote"]["held"]), ("wheels", "wheels"))
        self.assertEqual(jetson.read()[-1]["held"], "wheels")
        jetson.cmd("resume")
        record = self.cycle(state, 3.0)
        self.assertEqual((record["v"], record["stop"], record["remote"]["held"]), (0.1, None, None))

    def test_the_status_says_why_the_wheels_stopped(self):
        base = DeadMotors()
        base.clock = lambda: self.now
        state = self.remote_state(base)
        jetson = self.client()
        jetson.cmd("follow")
        for i in range(7):  # found at 0.5 s; the next 5 Hz status line goes at 0.6 s
            record = self.cycle(state, round(0.1 * i, 1))
        self.assertEqual((record["stop"], record["wheels"]), ("wheels", "fault 03"))
        status = jetson.read()[-1]
        self.assertEqual((status["phase"], status["stop"], status["wheels"]), ("FOLLOW", "wheels", "fault 03"))

    def test_a_page_wait_holds_until_the_page_resumes_whatever_the_client_sends(self):
        base = FakeBase()
        state = self.remote_state(base)
        jetson = self.client()
        jetson.cmd("follow")
        self.assertEqual(self.cycle(state, 0.0)["v"], 0.1)
        state.controls.command("wait")
        record = self.cycle(state, 0.1)
        self.assertEqual((record["v"], record["stop"], record["remote"]["held"]), (0.0, "page", "page"))
        self.assertEqual(jetson.read()[-1]["held"], "page")
        for t, cmd in ((0.2, "resume"), (0.3, "follow")):  # the bridge restoring its intent: not a release
            jetson.cmd(cmd)
            record = self.cycle(state, t)
            self.assertEqual((record["v"], record["stop"]), (0.0, "page"), cmd)
            self.assertEqual(jetson.read()[-1]["held"], "page")
        state.controls.command("resume")
        record = self.cycle(state, 0.4)
        self.assertEqual((record["v"], record["stop"], record["remote"]["held"]), (0.1, None, None))

    def test_a_client_return_still_acts_under_a_page_wait_but_does_not_drive(self):
        base = FakeBase()
        state = self.remote_state(base, brain=None)
        jetson = self.client()
        jetson.cmd("follow")
        for i in range(21):
            base.pose = (0.1 * i, 0.0, 0.0)
            self.cycle(state, round(0.1 * i, 1))
        state.controls.command("wait")
        self.assertEqual(self.cycle(state, 2.1)["stop"], "page")
        jetson.cmd("return")
        record = self.cycle(state, 2.2)
        self.assertEqual((record["phase"], record["v"], record["w"], record["stop"]), ("return", 0.0, 0.0, "page"))
        state.controls.command("resume")
        self.assertNotIn(self.cycle(state, 2.3)["stop"], ("page", "wait", "silent"))

    def test_the_pages_buttons_take_the_clients_path(self):
        base = FakeBase()
        state = self.remote_state(base)
        jetson = self.client()
        state.controls.command("return")  # refused while idle, as from the client; the client gets no error for it
        record = self.cycle(state, 0.0)
        self.assertEqual((record["phase"], record["stop"]), ("idle", "idle"))
        self.assertEqual([r["why"] for r in self.records() if "remote_refused" in r], ["not following yet"])
        self.assertFalse(jetson.errors())
        state.controls.command("follow")
        record = self.cycle(state, 0.1)
        self.assertEqual((record["phase"], record["v"], record["stop"]), ("follow", 0.1, None))
        status = jetson.read()[-1]
        self.assertEqual((status["ack"], status["ack_by"]), (None, None))  # ack is the client's own last command: a page press is not one
        jetson.cmd("wait")
        self.cycle(state, 0.15)
        state.controls.command("resume")
        self.cycle(state, 0.18)
        status = jetson.read()[-1]
        self.assertEqual((status["ack"], status["ack_by"]), ("wait", "client"))  # base_bridge takes a wait ack only from the client
        state.controls.command("wait")
        self.assertEqual(self.cycle(state, 0.2)["stop"], "page")
        state.controls.command("resume")
        self.assertEqual(self.cycle(state, 0.3)["stop"], None)
        self.assertEqual([r["page_cmd"] for r in self.records() if "page_cmd" in r], ["follow", "resume", "wait", "resume"])

    def polled(self, state, start, until, poll=True):
        """As quiet(), with the page polling /status every cycle when poll. Returns the last record."""
        record = None
        for i in range(int(round(start * 10)), int(round(until * 10)) + 1):
            self.now = round(0.1 * i, 1)
            if poll:
                state.controls.polled()
            record = self.cycle(state, self.now)
        return record

    def test_with_no_jetson_the_page_runs_the_task_while_it_polls(self):
        state = wheels_unwatched(self.remote_state(FakeBase()))
        state.controls.clock = lambda: self.now
        record = self.quiet(state, 0.0, 2.1)  # nobody connects: held for the silence
        self.assertEqual(record["remote"]["held"], "silent")
        state.controls.command("follow")
        record = self.polled(state, 2.2, 5.0)  # the page leads while it polls: the client's silence holds nothing
        self.assertEqual((record["phase"], record["v"], record["stop"]), ("follow", 0.1, None))
        record = self.polled(state, 5.1, 6.9, poll=False)  # the phone lost the Wi-Fi: under 2 s, still driving
        self.assertEqual(record["stop"], None)
        record = self.polled(state, 7.0, 7.0, poll=False)  # 2 s without a poll: the silence hold is back
        self.assertEqual((record["v"], record["stop"]), (0.0, "silent"))
        record = self.polled(state, 7.1, 7.5)  # polling again does not release it by itself
        self.assertEqual(record["stop"], "silent")
        state.controls.command("resume")
        self.assertEqual(self.polled(state, 7.6, 7.6)["stop"], None)

    def test_a_client_heard_again_after_the_page_led_is_not_held_for_its_silence(self):
        state = wheels_unwatched(self.remote_state(FakeBase()))
        state.controls.clock = lambda: self.now
        self.quiet(state, 0.0, 2.1)
        state.controls.command("follow")
        self.polled(state, 2.2, 5.0)
        jetson = self.client()  # a bridge just restarted: it pings, with no intent to send
        jetson.cmd("ping")
        record = self.polled(state, 5.1, 5.1)
        self.assertEqual((record["v"], record["stop"], record["remote"]["held"]), (0.1, None, None))
        jetson.cmd("ping")
        self.assertEqual(self.polled(state, 5.2, 5.2)["stop"], None)

    def test_main_with_remote_starts_in_idle_without_enter(self):
        old = {s: signal.getsignal(s) for s in run_follow.STOP_SIGNALS}
        self.addCleanup(lambda: [signal.signal(s, h) for s, h in old.items()])
        lidar_session = Scans(WALL_ONLY)
        lidar_session.close = lambda: None
        cam = SimpleNamespace(start=lambda: None, stop=lambda: None, latest=lambda: None)
        made, real = [], command_server.CommandServer

        def server(port, allow):
            made.append(real("127.0.0.1", port, allow=allow))  # not on every interface in a test
            return made[-1]

        with mock.patch.object(run_follow.lidar, "LidarSession", return_value=lidar_session), \
                mock.patch.object(run_follow.camera, "Camera", return_value=cam), \
                mock.patch.object(run_follow.command_server, "CommandServer", side_effect=server), \
                mock.patch.object(run_follow, "LOG_ROOT", self.folder / "logs"), \
                mock.patch("builtins.input", side_effect=AssertionError("Enter asked")), mock.patch("builtins.print"):
            run_follow.main(["--seconds", "0.35", "--no-jetson", "--planner", "here", "--dashboard", "0",
                             "--remote", "0"])
        self.assertEqual(len(made), 1)
        self.assertEqual(made[0].listener.fileno(), -1)  # closed at the end of the run
        # only the Jetson's addresses (cable, Wi-Fi, Tailscale) and this computer may send commands
        self.assertEqual(list(made[0].allow), [P["jetson_host"]] + P["jetson_hosts_backup"] + ["127.0.0.1"])
        logs = list((self.folder / "logs").iterdir())
        cycles = [json.loads(line) for line in (logs[0] / "log.jsonl").read_text().splitlines()]
        cycles = [r for r in cycles if "pose" in r]
        self.assertGreaterEqual(len(cycles), 3)
        self.assertTrue(all((r["phase"], r["v"], r["w"]) == ("idle", 0.0, 0.0) for r in cycles))
        with self.assertRaises(SystemExit), mock.patch("sys.stderr"):
            run_follow.main(["--seconds", "1", "--remote", "x"])


    def test_remote_allow_overrides_the_allow_list(self):
        old = {s: signal.getsignal(s) for s in run_follow.STOP_SIGNALS}
        self.addCleanup(lambda: [signal.signal(s, h) for s, h in old.items()])
        lidar_session = Scans(WALL_ONLY)
        lidar_session.close = lambda: None
        cam = SimpleNamespace(start=lambda: None, stop=lambda: None, latest=lambda: None)
        made, real = [], command_server.CommandServer

        def server(port, allow):
            made.append(real("127.0.0.1", port, allow=allow))  # not on every interface in a test
            return made[-1]

        with mock.patch.object(run_follow.lidar, "LidarSession", return_value=lidar_session), \
                mock.patch.object(run_follow.camera, "Camera", return_value=cam), \
                mock.patch.object(run_follow.command_server, "CommandServer", side_effect=server), \
                mock.patch.object(run_follow, "LOG_ROOT", self.folder / "logs"), \
                mock.patch("builtins.input", side_effect=AssertionError("Enter asked")), mock.patch("builtins.print"):
            run_follow.main(["--seconds", "0.35", "--no-jetson", "--planner", "here", "--dashboard", "0",
                             "--remote", "0", "--remote-allow", "127.0.0.1, 10.1.2.3"])
        self.assertEqual(len(made), 1)
        self.assertEqual(made[0].listener.fileno(), -1)  # closed at the end of the run
        # --remote-allow: the lab's list instead
        self.assertEqual(list(made[0].allow), ["127.0.0.1", "10.1.2.3"])

    def test_main_keeps_the_page_to_this_computer_unless_asked(self):
        # by default only the console (and its PIN) reaches the switches: anyone on the Wi-Fi could drive
        old = {s: signal.getsignal(s) for s in run_follow.STOP_SIGNALS}
        self.addCleanup(lambda: [signal.signal(s, h) for s, h in old.items()])
        lidar_session = Scans(WALL_ONLY)
        lidar_session.close = lambda: None
        cam = SimpleNamespace(start=lambda: None, stop=lambda: None, latest=lambda: None)
        pages, real, real_server = [], run_follow.dashboard_module.Dashboard, command_server.CommandServer

        def page(controls, run_info, port, remote, host):
            pages.append(real(controls, run_info, port=0, host=host, remote=remote))
            return pages[-1]

        with mock.patch.object(run_follow.lidar, "LidarSession", return_value=lidar_session), \
                mock.patch.object(run_follow.camera, "Camera", return_value=cam), \
                mock.patch.object(run_follow.dashboard_module, "Dashboard", side_effect=page), \
                mock.patch.object(run_follow.command_server, "CommandServer",
                                  side_effect=lambda port, allow: real_server("127.0.0.1", port, allow=allow)), \
                mock.patch.object(run_follow, "LOG_ROOT", self.folder / "logs"), \
                mock.patch("builtins.input", side_effect=AssertionError("Enter asked")), mock.patch("builtins.print"):
            run_follow.main(["--seconds", "0.35", "--no-jetson", "--planner", "here", "--remote", "0"])
            run_follow.main(["--seconds", "0.35", "--no-jetson", "--planner", "here", "--remote", "0",
                             "--dashboard-host", "0.0.0.0"])  # opened on purpose
        self.assertEqual(len(pages), 2)
        self.assertEqual(pages[0].server.server_address[0], "127.0.0.1")
        self.assertEqual(pages[1].server.server_address[0], "0.0.0.0")

    def test_main_without_a_console_waits_for_follow_on_the_page(self):
        # under nohup stdin is no terminal and input() raises EOFError (2026-11-14: the run died of it)
        old = {s: signal.getsignal(s) for s in run_follow.STOP_SIGNALS}
        self.addCleanup(lambda: [signal.signal(s, h) for s, h in old.items()])
        lidar_session = Scans(WALL_ONLY)
        lidar_session.close = lambda: None
        cam = SimpleNamespace(start=lambda: None, stop=lambda: None, latest=lambda: None)
        pages, real, pressed = [], run_follow.dashboard_module.Dashboard, []

        def page(controls, run_info, port, remote, host):
            self.assertEqual(host, "127.0.0.1")  # by hand too: only the console reaches the switches
            pages.append(real(controls, run_info, port=0, host="127.0.0.1", remote=remote))
            return pages[-1]

        def follow():  # Follow on the page, a while after the start
            pressed.append(time.monotonic())
            pages[0].controls.command("follow")

        press = threading.Timer(0.3, follow)
        self.addCleanup(press.cancel)
        no_tty = SimpleNamespace(isatty=lambda: False)
        with mock.patch.object(run_follow.lidar, "LidarSession", return_value=lidar_session), \
                mock.patch.object(run_follow.camera, "Camera", return_value=cam), \
                mock.patch.object(run_follow.dashboard_module, "Dashboard", side_effect=page), \
                mock.patch.object(run_follow, "LOG_ROOT", self.folder / "logs"), \
                mock.patch("sys.stdin", no_tty), \
                mock.patch("builtins.input", side_effect=EOFError), mock.patch("builtins.print"):
            press.start()
            run_follow.main(["--seconds", "0.35", "--no-jetson", "--planner", "here"])
        self.assertEqual(len(pages), 1)
        self.assertIsInstance(pages[0].feed, run_follow.live_feed.LiveFeed)
        logs = list((self.folder / "logs").iterdir())
        cycles = [json.loads(line) for line in (logs[0] / "log.jsonl").read_text().splitlines()]
        cycles = [r for r in cycles if "pose" in r]
        self.assertGreaterEqual(len(cycles), 1)
        self.assertGreaterEqual(cycles[0]["time"], pressed[0])  # the run waited for Follow
        self.assertTrue(all(r["phase"] == "follow" for r in cycles))
        # nothing could start it: no console, no --remote and no page
        with self.assertRaises(SystemExit) as refused, mock.patch("sys.stdin", no_tty):
            run_follow.main(["--seconds", "1", "--dashboard", "0"])
        self.assertIn("REFUSED", str(refused.exception))


class Blocked(Pushy):
    """A brain that follows an operator 2 m away (believed frame) but finds no way there: it stands."""

    def __init__(self, robot):
        super().__init__(robot)
        self.last_operator_pos = (2.0, 0.0)
        self.follow_distance = P["follow_gap"] + safety.FRONT
        self.navigator = SimpleNamespace(path=[], _goal=None)

    def step(self, senses, dt):
        self.robot.speed = self.robot.turn_speed = 0.0


CORNER_WALLS = [((0.45, -1.5), (0.45, 1.5)), ((-1.5, 0.45), (0.45, 0.45))]  # ahead and to the left
DEAD_END_WALLS = [((0.3, -0.27), (0.3, 0.27)), ((-0.15, 0.27), (0.6, 0.27)), ((-0.15, -0.27), (0.6, -0.27))]


class Stuck(Logged):
    """Owner's request A (2026-11-15): out of a corner when stuck, then plan again."""

    def run_stuck(self, walls, seconds, state=None, base=None, page=None):
        base = base or FakeBase()
        world = Room(base, walls)
        state = state or run_follow.State(P, base, world, Camera(), None, self.log, clock=lambda: self.now)
        if not isinstance(state.follower, Blocked):
            state.follower = Blocked(state.body.robot)
        records = []
        for i in range(round(seconds / 0.1)):
            if page and round(0.1 * i, 1) == 0.0:
                state.controls.command(page)
            records.append(self.cycle(state, round(0.1 * i, 1)))
            base.advance(0.1)
        return state, base, records

    def test_four_seconds_stuck_in_a_corner_turn_out_and_log_it(self):
        state, base, records = self.run_stuck(CORNER_WALLS, 12.0)
        events = [(r["time"], r["escape"]) for r in records if r["escape"]]
        self.assertEqual(events[0][0], escape.STUCK_S)
        self.assertEqual(events[0][1]["start"], "turn")
        self.assertIn("no path", events[0][1]["why"])
        self.assertTrue(any(e.get("end") == "turn" and e["result"] == "done" for _, e in events))
        turning = [r for r in records if r["w"]]
        self.assertTrue(turning)
        self.assertTrue(all(r["w"] == -escape.TURN_W and r["v"] == 0.0 for r in turning))  # right: away from both walls
        self.assertTrue(all(r["stop"] is None for r in records))
        self.assertLess(base.pose[2], math.radians(-60))
        self.assertLessEqual(len([e for _, e in events if "start" in e]), escape.MAX_ESCAPES)

    def test_no_room_to_turn_backs_up_with_reverse_packets(self):
        state, base, records = self.run_stuck(DEAD_END_WALLS, 12.0)
        start = next(r["escape"] for r in records if r["escape"])
        self.assertEqual(start["start"], "back", start)
        self.assertGreater(base.backed, 0)
        self.assertIn((-wheels_module.REVERSE_MAX, 0.0), base.sent)
        self.assertAlmostEqual(base.pose[0], -start["m"], delta=0.02)
        self.assertTrue(any(r["escape"] == {"end": "back", "result": "done"} for r in records))
        self.assertTrue(all(r["stop"] is None for r in records))

    def test_held_or_in_a_dry_run_it_never_escapes(self):
        state, base, records = self.run_stuck(CORNER_WALLS, 10.0, page="wait")  # the page's Wait
        self.assertEqual([r["escape"] for r in records if r["escape"]], [])
        self.assertEqual(base.pose, (0.0, 0.0, 0.0))
        world = Room(FakeBase(), CORNER_WALLS)
        state = run_follow.State(P, None, world, Camera(), None, self.log, clock=lambda: self.now)
        state.follower = Blocked(state.body.robot)
        records = [self.cycle(state, round(0.1 * i, 1)) for i in range(100)]
        self.assertEqual([r["escape"] for r in records if r["escape"]], [])

    def test_the_cable_is_counted_saved_and_reported(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        path = Path(folder.name) / "cable.json"
        cable_module.save(path, 140.0, 12.0)
        base = FakeBase()
        state = self.search_spin(base, path)
        records = []
        for i in range(60):  # 6 s of the search turn to the left at 0.3 rad/s
            records.append(self.cycle(state, round(0.1 * i, 1)))
            base.advance(0.1)
        self.assertTrue(all("cable_deg" in r for r in records))
        self.assertLessEqual(max(r["cable_deg"] for r in records), cable_module.WARN_DEG + 2.0)
        self.assertIn(-run_follow.SEARCH_TURN, [r["w"] for r in records])  # turned round at 150 deg
        self.assertIn("the other way round", [r["cable"] for r in records])
        saved = json.loads(path.read_text())
        self.assertEqual(saved["set_at"], 12.0)
        self.assertAlmostEqual(saved["net_deg"], records[-1]["cable_deg"], delta=math.degrees(0.3) * 1.1)

    def search_spin(self, base, path):
        """Not locked yet, the link up and nobody in view: run_follow's own search turns on the spot."""
        link = SimpleNamespace(latest=lambda: Answer(self.now, []), alive=lambda now, silence: True,
                               send=lambda frame: None)
        state = run_follow.State(P, base, Scans(WALL_ONLY), Camera(), link, self.log, clock=lambda: self.now,
                                 cable_path=path)
        state.follower = Pushy(state.body.robot)
        state.follower.operator_look = SimpleNamespace(count=0)
        return state


class Creep(Pushy):
    """A brain that turns on the spot at `w` toward a heading `error` rad away (wheels.least)."""

    w, error = 0.07, math.radians(44)

    def step(self, senses, dt):
        self.robot.speed, self.robot.turn_speed, self.robot.turn_error = 0.0, self.w, self.error


class Least(Logged):
    """Task 1 of 2026-11-15: every command moves the wheels or is 0, made so in one place before the stops."""

    def test_a_crawl_on_the_spot_becomes_min_turn_and_the_obstacle_check_judges_that(self):
        base = FakeBase()
        state = self.state(base, brain=Creep)
        with mock.patch.object(run_follow.safety, "must_stop", wraps=safety.must_stop) as checked:
            record = self.cycle(state, 0.0)
        self.assertEqual(base.sent[-1], (0.0, wheels_module.MIN_TURN))
        self.assertEqual((record["v"], record["w"]), (0.0, wheels_module.MIN_TURN))
        self.assertTrue(checked.called)
        self.assertTrue(all(abs(call.args[2]) == wheels_module.MIN_TURN for call in checked.call_args_list))
        self.assertEqual(state.coast.parts["left"][0], wheels_module.MIN_TURN)  # the coast knows the real command

    def test_a_small_heading_error_on_the_spot_sends_0(self):
        base = FakeBase()
        state = self.state(base, brain=Creep)
        state.follower.error = math.radians(1.0)
        record = self.cycle(state, 0.0)
        self.assertEqual((record["v"], record["w"]), (0.0, 0.0))
        self.assertEqual(base.sent[-1], (0.0, 0.0))

    def test_the_raised_turn_still_stops_for_what_its_outline_would_hit(self):
        base = FakeBase()
        r, deg = 0.30, math.radians(65)  # a leg beside the front-left corner the left turn swings toward
        state = self.state(base, scan=scan_of([(r * math.cos(deg) - P["lidar_x"], r * math.sin(deg))]), brain=Creep)
        record = self.cycle(state, 0.0)
        self.assertEqual(record["stop"], "obstacle")
        self.assertEqual(base.sent[-1], (0.0, 0.0))

    def test_the_real_base_follows_with_its_own_gaps(self):
        state = run_follow.State(P, FakeBase(), Scans(WALL_ONLY), Camera(), None, self.log, clock=lambda: self.now)
        follower = state.follower
        self.assertEqual((follower.restart_gap, follower.wait_gap, follower.go_on_gap), (0.25, 2.0, 1.2))
        self.assertEqual((run_follow.RESTART_GAP, run_follow.WAIT_GAP, run_follow.GO_ON_GAP), (0.25, 2.0, 1.2))


class SearchSpin(Logged):
    """Cable review major 2: the brain's own search spin (Mode.SEARCH) is an open turn, not a turn to +90 deg."""

    def test_from_net_100_a_minute_of_search_turns_unwinds_first_and_never_shakes(self):
        base = FakeBase()
        state = run_follow.State(P, base, Scans(WALL_ONLY), Camera(), None, self.log, clock=lambda: self.now)
        follower = state.follower
        follower.mode, follower.look_since = Mode.SEARCH, 0.0  # lost: looking around on the spot
        # Not this test: the Pi's wheel check takes the turn round at -200 deg (net yaw about 0 over its 1 s
        # window) for wheels that do not turn.
        wheels_unwatched(state)
        state.cable.net_deg = 100.0
        records = []
        with mock.patch.object(run_follow.hmc_follower, "LOOK_AROUND", 1e9):  # keep looking round all the minute
            for i in range(600):
                records.append(self.cycle(state, round(0.1 * i, 1)))
                base.advance(0.1)
        ws = [r["w"] for r in records]
        self.assertLess(next(w for w in ws if w), 0.0)  # to the right first: it unwinds from +100
        self.assertLess(records[30]["cable_deg"], 100.0)
        self.assertNotIn("wound", [r["cable"] for r in records])
        self.assertLessEqual(max(abs(r["cable_deg"]) for r in records), cable_module.LIMIT_DEG)
        turning = [w for w in ws if w]
        self.assertGreater(len(turning), 590)  # it kept turning
        runs, n = [], 1
        for a, b in zip(turning, turning[1:]):
            if a * b < 0:
                runs.append(n)
                n = 0
            n += 1
        self.assertTrue(runs)  # it turned round at the warning
        self.assertGreaterEqual(min(runs[1:] or runs), 50)  # each way for seconds, never back and forth on the spot


class BackingOverTheCable(Logged):
    """Cable review major 3: backing never drives over the cable lying behind (cable.Lay)."""

    def backing(self, trail):
        base = FakeBase()
        state = self.state(base, brain=Blocked)
        state.cable.lay.trail = list(trail)  # the post's path, oldest first, odometry frame
        state.escape.command = lambda *a: (-wheels_module.REVERSE_MAX, 0.0)  # an escape backs straight
        return base, self.cycle(state, 0.0)

    def test_the_obstacle_stop_holds_a_backing_over_the_cable_behind(self):
        base, record = self.backing([(-0.24, -1.0), (-0.24, -0.1)])  # from the post round the back, then right
        self.assertEqual(record["stop"], "obstacle")
        self.assertEqual(base.sent[-1], (0.0, 0.0))

    def test_a_cable_beside_the_robot_lets_it_back(self):
        base, record = self.backing([(-2.0, cable_module.ATTACH[1]), (-0.2, cable_module.ATTACH[1])])
        self.assertIsNone(record["stop"])
        self.assertEqual(base.sent[-1], (-wheels_module.REVERSE_MAX, 0.0))


if __name__ == "__main__":
    unittest.main()
