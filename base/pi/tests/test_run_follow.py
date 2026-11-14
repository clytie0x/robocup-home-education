"""The control loop with fake devices. Nothing here opens a serial port or drives the base."""
import json
import math
import signal
import socket
import tempfile
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

import params
import run_follow
import safety
from link import Answer, Seen
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

    def send(self, v, w):
        self.command = (v, w)
        self.sent.append((v, w))

    def read_odom(self):
        return self.pose

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
                mock.patch("builtins.input", return_value=""), mock.patch("builtins.print"):
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
        state = self.state(base, scan=scan_of([(0.25, 0.0)]))  # 0.225 m ahead of the front, inside the strip
        self.assertLess(0.25 + P["lidar_x"] - safety.FRONT, safety.braking_distance(0.1, P))
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
FAST = dict(P, max_speed=0.25)  # the simulated base is perfect: drive faster, so the test runs in seconds


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
        self.assertLess(time.monotonic() - started, 60.0)

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
        state = run_follow.State(P, base, world, Camera(), Eyes(base, world, people), self.log, clock=lambda: self.now)
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


if __name__ == "__main__":
    unittest.main()
