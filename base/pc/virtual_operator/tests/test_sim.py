"""The PC side's map, walks and auto loop, without a robot or a network.

    cd base/pc/virtual_operator && python -m unittest discover -s tests
"""
import csv
import math
import random
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import sim  # noqa: E402

HALF = 2.5  # m, half the side of the square test room around (0, 0)


def room_scan(pose, half=HALF, wall=None, step=1.0):
    """Robot-frame points of a square room (and an optional wall segment ((x1, y1), (x2, y2)) in it)."""
    x0, y0, th = pose
    segments = [((-half, -half), (half, -half)), ((half, -half), (half, half)), ((half, half), (-half, half)),
                ((-half, half), (-half, -half))] + ([wall] if wall else [])
    points = []
    a = 0.0
    while a < 360.0:
        d = (math.cos(math.radians(a) + th), math.sin(math.radians(a) + th))
        best = None
        for (ax, ay), (bx, by) in segments:
            ex, ey = bx - ax, by - ay
            den = d[0] * ey - d[1] * ex
            if abs(den) < 1e-12:
                continue
            t = ((ax - x0) * ey - (ay - y0) * ex) / den
            u = ((ax - x0) * d[1] - (ay - y0) * d[0]) / den
            if t > 0 and 0 <= u <= 1 and (best is None or t < best):
                best = t
        if best is not None and best <= sim.SCAN_RANGE:
            points.append((best * math.cos(math.radians(a)), best * math.sin(math.radians(a))))
        a += step
    return points


def mapped(pose=(0.0, 0.0, 0.0), wall=None, scans=3):
    grid = sim.Grid()
    for _ in range(scans):
        grid.add_scan(pose, room_scan(pose, wall=wall))
    return grid


class GridTest(unittest.TestCase):
    def test_walls_are_occupied_and_the_floor_is_seen(self):
        grid = mapped()
        walls = grid.occupied()
        self.assertTrue(walls)
        self.assertTrue(all(abs(abs(max(abs(x), abs(y))) - HALF) <= 0.08 for x, y in map(sim.centre, walls)))
        self.assertIn(sim.cell_of(1.0, 1.0), grid.free)
        self.assertNotIn(sim.cell_of(1.0, 1.0), walls)

    def test_one_scan_is_not_yet_a_wall(self):
        grid = sim.Grid()
        grid.add_scan((0.0, 0.0, 0.0), room_scan((0.0, 0.0, 0.0)))
        self.assertEqual(grid.occupied(), set())

    def test_something_gone_is_cleared_by_the_rays(self):
        box = ((1.0, -0.5), (1.0, 0.5))
        grid = mapped(wall=box)
        self.assertIn(sim.cell_of(1.0, 0.0), grid.occupied())
        for _ in range(10):  # the box was carried away
            grid.add_scan((0.0, 0.0, 0.0), room_scan((0.0, 0.0, 0.0)))
        self.assertNotIn(sim.cell_of(1.0, 0.0), grid.occupied())

    def test_the_pose_puts_the_points_in_the_odometry_frame(self):
        a = mapped(pose=(0.0, 0.0, 0.0)).occupied()
        b = mapped(pose=(0.5, -0.3, 1.0)).occupied()  # the same room seen from elsewhere
        self.assertGreater(len(a & b) / len(a), 0.6)


class WalkTest(unittest.TestCase):
    def test_a_walk_keeps_its_clearance_and_goes_round_a_wall(self):
        wall = ((0.0, -2.5), (0.0, 1.0))  # a wall from the bottom side up to y = 1
        grid = mapped(pose=(-1.5, 1.8, 0.0), wall=wall)
        for pose in ((1.5, 1.8, 0.0), (-1.5, -1.0, 0.0), (1.5, -1.0, 0.0)):  # look round the room
            for _ in range(3):
                grid.add_scan(pose, room_scan(pose, wall=wall))
        path = sim.astar(grid, (-1.5, -1.5), (1.5, -1.5))
        self.assertIsNotNone(path)
        self.assertEqual((path[0], path[-1]), ((-1.5, -1.5), (1.5, -1.5)))
        self.assertGreater(max(y for _, y in path), 1.0 + sim.CLEARANCE - 0.06)  # over the end of the wall
        walls = [sim.centre(c) for c in grid.occupied()]
        for a, b in zip(path, path[1:]):
            for k in range(21):
                q = (a[0] + (b[0] - a[0]) * k / 20, a[1] + (b[1] - a[1]) * k / 20)
                self.assertGreater(min(math.dist(q, w) for w in walls), sim.CLEARANCE - 0.05)

    def test_no_walk_into_a_wall_or_unseen_floor(self):
        grid = mapped()
        self.assertIsNone(sim.astar(grid, (0.5, 0.0), (HALF - 0.1, 0.0)))  # too close to the wall
        self.assertIsNone(sim.astar(grid, (0.5, 0.0), (6.0, 0.0)))  # outside the room: never seen

    def test_the_walk_keeps_away_from_the_robot(self):
        grid = mapped()
        path = sim.astar(grid, (-1.5, 0.0), (1.5, 0.0), robot_xy=(0.0, 0.0))
        self.assertIsNotNone(path)
        for a, b in zip(path, path[1:]):
            for k in range(21):
                q = (a[0] + (b[0] - a[0]) * k / 20, a[1] + (b[1] - a[1]) * k / 20)
                self.assertGreater(math.hypot(*q), sim.ROBOT_KEEP - 0.08)

    def test_a_random_goal_is_2_to_4_m_away_and_reachable(self):
        grid = mapped(scans=3)
        rng = random.Random(1)
        for _ in range(5):
            goal, path = sim.random_goal(grid, (-1.0, -1.0), (0.0, 0.0), rng)
            self.assertTrue(sim.NEAR - 0.05 <= math.dist(goal, (-1.0, -1.0)) <= sim.FAR + 0.05)
            self.assertEqual(path[-1], goal)


class PersonTest(unittest.TestCase):
    def test_walks_the_path_at_its_speed_and_faces_the_way(self):
        p = sim.Person(0.0, 0.0, 0.0, speed=0.24)
        p.go([(0.0, 0.0), (0.0, 1.0), (1.0, 1.0)])
        for _ in range(10):
            p.step(0.1)
        self.assertAlmostEqual(p.y, 0.24, places=6)
        self.assertAlmostEqual(p.th, math.pi / 2, places=6)
        for _ in range(100):
            p.step(0.1)
        self.assertEqual((p.x, p.y, p.walking), (1.0, 1.0, False))
        self.assertAlmostEqual(p.th, 0.0, places=6)

    def test_the_datagram_and_the_spoken_word(self):
        p = sim.Person(1.0, 2.0, 0.5)
        p.say("stop", 10.0)
        self.assertEqual(p.state(10.5), {"t": 10.5, "x": 1.0, "y": 2.0, "th": 0.5, "speaking": "stop", "pointing": None})
        self.assertIsNone(p.state(10.0 + sim.SAY_S)["speaking"])
        with self.assertRaises(ValueError):
            p.say("dance", 0.0)

    def test_pointing_at_a_floor_spot_for_a_while(self):
        p = sim.Person(0.0, 0.0, 0.0)
        p.point((1.0, 1.0), 10.0)
        self.assertEqual(p.state(11.0)["pointing"], [1.0, 1.0])
        self.assertAlmostEqual(p.th, math.pi / 4)  # standing: it turns to the spot
        self.assertIsNone(p.state(10.0 + sim.POINT_S)["pointing"])
        p.point((2.0, 0.0), 20.0, seconds=60.0)
        p.point(None, 21.0)
        self.assertIsNone(p.state(21.0)["pointing"])
        with self.assertRaises(ValueError):
            p.point((1.0, float("nan")), 0.0)

    def test_the_default_speed_is_the_owners(self):
        self.assertEqual(sim.Person().speed, 0.24)


class GapTest(unittest.TestCase):
    def test_gap_from_the_square_body(self):
        self.assertAlmostEqual(sim.body_gap([(0.5, 0.0)]), 0.30, places=6)
        self.assertAlmostEqual(sim.body_gap([(0.23, 0.24)]), math.hypot(0.03, 0.04), places=6)
        self.assertIsNone(sim.body_gap([]))


class FakeRun:
    """A console and a robot that follow perfectly: it locks at once, drives to 0.9 m of the person, goes home."""

    def __init__(self, person):
        self.person = person
        self.running = False
        self.record = None
        self.started = 0
        self.stopped = 0
        self.heard = None

    def start_run(self):
        self.started += 1
        self.running = True
        self.record = {"pose": [0.0, 0.0, 0.0], "phase": "idle", "locked": False, "link": True, "wheels": None,
                       "cable_deg": 0.0}
        return None

    def stop_run(self):
        self.stopped += 1
        self.running = False

    def robot(self):
        return sim.Robot(self.running, True, dict(self.record) if self.record else None, gap=0.5)

    def advance(self, now, dt):
        if not self.running:
            return
        r = self.record
        word = self.person.speaking(now)
        if word != self.heard and word is not None:
            r["phase"] = {"follow": "follow", "back": "return"}.get(word, r["phase"])
        self.heard = word
        r["locked"] = True
        x, y, _ = r["pose"]
        if r["phase"] == "follow":
            target = (self.person.x, self.person.y)
            far = math.dist((x, y), target) - 0.9
        else:
            target, far = (0.0, 0.0), math.dist((x, y), (0.0, 0.0)) if r["phase"] == "return" else 0.0
        if far > 0.01:
            step = min(far, 0.12 * dt)
            th = math.atan2(target[1] - y, target[0] - x)
            r["pose"] = [x + step * math.cos(th), y + step * math.sin(th), th]
        elif r["phase"] == "return":
            r["phase"], r["end"] = "done", "done: back at the start"
            self.running = False


class AutoTest(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.csv = Path(folder.name) / "episodes.csv"
        self.grid = mapped(scans=3)
        self.person = sim.Person(-1.5, 1.5, 0.0)
        self.run = FakeRun(self.person)
        self.auto = sim.Auto(self.person, self.grid, self.run, self.csv, rng=random.Random(2))

    def play(self, seconds, until=None, start=0.0):
        now = start
        while now < start + seconds:
            self.person.step(0.1)
            self.run.advance(now, 0.1)
            self.auto.tick(now, self.run.robot())
            if until is not None and until():
                break
            now += 0.1
        return now

    def rows(self):
        with open(self.csv, newline="", encoding="utf-8") as f:
            return list(csv.DictReader(f))

    def test_one_whole_episode_is_logged_ok_and_the_next_starts(self):
        self.auto.start(0.0)
        end = self.play(400.0, until=lambda: self.csv.exists())
        rows = self.rows()
        self.assertEqual(len(rows), 1, self.auto.message)
        row = rows[0]
        self.assertEqual(row["result"], "ok")
        self.assertGreaterEqual(float(row["walk_m"]), sim.NEAR - 0.05)  # a walk is never shorter than the way
        self.assertGreater(float(row["time_to_reach_s"]), 0.0)
        self.assertEqual(float(row["nearest_m"]), 0.5)
        self.assertEqual(self.auto.step, "starting")  # on to the next episode, with a new run
        self.play(10.0, start=end + 0.1)
        self.assertEqual(self.run.started, 2)
        self.assertEqual(self.run.stopped, 0)

    def test_the_person_stands_in_front_until_the_lock(self):
        self.auto.start(0.0)
        self.play(30.0, until=lambda: self.auto.step == "lock")
        self.assertEqual(self.auto.step, "lock")
        self.assertLess(math.dist((self.person.x, self.person.y), (sim.FRONT, 0.0)), 0.16)

    def fault_after_follow(self, **change):
        self.auto.start(0.0)
        now = self.play(60.0, until=lambda: self.auto.step == "walk")
        self.assertEqual(self.auto.step, "walk")
        self.run.record.update(change)
        robot = self.run.robot()
        if "gap" in change:
            robot.gap = change.pop("gap")
            self.run.record.pop("gap")
        self.auto.tick(now + 0.1, robot)
        self.assertEqual(self.auto.step, "off")
        self.assertEqual(self.run.stopped, 1)  # the console's STOP
        self.assertFalse(self.person.walking)  # and the person stops
        return self.rows()[-1]["result"]

    def test_a_wheel_fault_stops_the_loop_and_the_run(self):
        self.assertIn("ล้อ", self.fault_after_follow(wheels="not turning"))

    def test_too_close_to_something_stops(self):
        self.assertIn("ใกล้", self.fault_after_follow(gap=0.04))

    def test_a_cable_turn_past_150_stops(self):
        self.assertIn("สายไฟ", self.fault_after_follow(cable_deg=-151.0))

    def test_no_cable_turn_in_the_record_is_fine(self):
        self.auto.start(0.0)
        self.run.start_run()
        self.run.record.pop("cable_deg")
        self.assertIsNone(self.auto.fault(1.0, self.run.robot()))

    def test_a_link_lost_for_2_s_stops(self):
        self.auto.start(0.0)
        now = self.play(60.0, until=lambda: self.auto.step == "walk")
        self.run.record["link"] = False
        self.auto.tick(now + 0.1, self.run.robot())
        self.assertNotEqual(self.auto.step, "off")  # a moment is not yet a fault
        self.auto.tick(now + 2.2, self.run.robot())
        self.assertEqual(self.auto.step, "off")

    def test_the_run_ending_by_itself_stops_the_loop(self):
        self.auto.start(0.0)
        now = self.play(60.0, until=lambda: self.auto.step == "walk")
        self.run.running = False
        self.run.record["end"] = "slow cycle: 0.50 s"
        self.auto.tick(now + 0.1, self.run.robot())
        self.assertEqual(self.auto.step, "off")
        self.assertIn("slow cycle", self.rows()[-1]["result"])

    def test_stop_by_the_owner(self):
        self.auto.start(0.0)
        now = self.play(60.0, until=lambda: self.auto.step == "walk")
        self.auto.stop(now)
        self.assertEqual((self.auto.step, self.run.stopped), ("off", 1))
        self.assertEqual(self.rows()[-1]["result"], "หยุดโดยผู้ใช้")

    def test_a_running_plain_run_is_not_taken(self):
        self.run.start_run()
        self.auto.start(0.0)
        self.auto.tick(0.1, sim.Robot(True, False, self.run.record))
        self.assertEqual(self.auto.step, "off")
        self.assertEqual(self.run.stopped, 0)  # someone else's run: not stopped from here


if __name__ == "__main__":
    unittest.main()
