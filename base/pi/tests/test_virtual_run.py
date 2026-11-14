"""run_follow with --virtual-operator: the senses see the virtual person, the safety stop only the real room."""
import json
import math
import unittest
from unittest import mock

import run_follow
import safety
import virtual_operator as vo
from test_run_follow import P, Camera, FakeBase, Logged, Pushy, Scans
from test_virtual_operator import room

SECTOR = 30.0  # degrees each side of the person where the far wall returns; elsewhere no return (a fast planner)
WALL = 6.0


class SectorLidar:
    """The real LiDAR of an open hall: a far wall behind wherever the virtual person stands, nothing else."""

    def __init__(self, base, person):
        self.base, self.person = base, person

    def latest(self):
        x, y, yaw = self.base.pose
        px, py = self.person()
        bx = math.cos(yaw) * (px - x) + math.sin(yaw) * (py - y) - P["lidar_x"]
        by = -math.sin(yaw) * (px - x) + math.cos(yaw) * (py - y)
        ahead = math.degrees(math.atan2(by, bx))
        return [(deg, WALL if abs((deg - ahead + 180) % 360 - 180) <= SECTOR else 0.0) for deg, _ in room(step=0.8)]


class Hook(Logged):
    def virtual_state(self, scan, person, brain=Pushy):
        state = self.state(FakeBase(), scan=scan, brain=brain)
        state.controls.apply({"search": False})  # Pushy never locks: the search would hold on the centred person
        state.virtual = vo.VirtualOperator(P, port=0, controls=state.controls, clock=lambda: self.now, seed=2)
        self.now = 0.0
        state.virtual.receive(json.dumps({"t": 0.0, "x": person[0], "y": person[1], "th": math.pi,
                                          "speaking": None}).encode(), ("127.0.0.1", 1))
        return state

    def test_the_senses_read_the_virtual_legs_and_the_record_says_so(self):
        state = self.virtual_state(room(), (1.5, 0.0))
        got = []
        read = state.senses.read
        state.senses.read = lambda now, pose, v, w, dt, scan, *rest: got.append(scan) or read(now, pose, v, w, dt, scan, *rest)
        record = self.cycle(state, 0.1)
        self.assertNotEqual(got[0], room())  # the legs are in the scan the senses read
        self.assertEqual(record["virtual"]["person"], [1.5, 0.0, round(math.pi, 3)])
        self.assertGreater(record["virtual"]["returns"], 0)
        self.assertTrue(record["virtual"]["seen"])
        self.assertTrue(record["link"])  # the made-up camera answers
        self.assertEqual(json.loads(json.dumps(record))["virtual"], record["virtual"])

    def test_must_stop_decides_on_the_real_points_only(self):
        # The virtual person 0.25 m in front of the front edge: a real person there would stop the drive.
        state = self.virtual_state(room(), (0.45, 0.0))
        checked = []
        real_stop = safety.must_stop
        with mock.patch.object(run_follow.safety, "must_stop",
                               side_effect=lambda pts, *a, **k: checked.append(pts) or real_stop(pts, *a, **k)):
            record = self.cycle(state, 0.1)
        self.assertGreater(record["virtual"]["returns"], 0)
        self.assertIsNone(record["stop"])  # the real hall is clear: the brain's 0.1 m/s goes out
        self.assertEqual(record["v"], 0.1)
        real = run_follow.scan_points(room(), P)
        self.assertTrue(checked and all(len(pts) == len(real) and (pts == real).all() for pts in checked))
        self.assertAlmostEqual(record["nearest"], min(math.hypot(x, y) for x, y in real), places=6)

    def test_a_real_obstacle_still_stops_with_a_virtual_person_ahead(self):
        state = self.virtual_state(room(box=(-10.0, 10.0, 0.18)), (2.5, 0.0))  # a real box 0.18 m from the laser
        record = self.cycle(state, 0.1)
        self.assertEqual(record["stop"], "obstacle")
        self.assertEqual(record["v"], 0.0)

    def test_the_pointed_floor_spot_is_in_the_record(self):
        state = self.virtual_state(room(), (1.5, 0.0))
        state.virtual.receive(json.dumps({"t": 0.05, "x": 1.5, "y": 0.0, "th": math.pi, "speaking": None,
                                          "pointing": [2.5, 1.0]}).encode(), ("127.0.0.1", 1))
        self.assertEqual(self.cycle(state, 0.1)["virtual_pointing"], [2.5, 1.0])

    def test_a_vanished_person_is_no_camera_answer(self):
        state = self.virtual_state(room(), (1.5, 0.0))
        for i in range(1, 6):
            self.assertIsNotNone(self.cycle(state, 0.1 * i)["virtual"]["person"])  # up to 0.5 s after the state
        record = self.cycle(state, 0.6)  # no state since 0.0
        self.assertIsNone(record["virtual"]["person"])
        self.assertFalse(record["link"])

    def test_back_said_by_the_virtual_person_starts_the_way_back(self):
        state = self.virtual_state(room(), (1.5, 0.0), brain=None)  # hmc's own Follower
        self.cycle(state, 0.1)
        state.virtual.receive(json.dumps({"t": 0.15, "x": 1.5, "y": 0.0, "th": math.pi, "speaking": "back"}).encode(),
                              ("127.0.0.1", 1))
        self.cycle(state, 0.2)  # the word is queued as a page button in this cycle
        record = self.cycle(state, 0.3)  # and taken in the next, as the page's Go back
        self.assertIn({"time": 0.3, "page_cmd": "return"}, self.records())
        self.assertIn(record["phase"], (run_follow.RETURN, run_follow.DONE))  # it stands at the start already

    def test_without_the_flag_there_is_no_virtual_record(self):
        record = self.cycle(self.state(FakeBase(), scan=room()), 0.1)
        self.assertNotIn("virtual", record)
        self.assertNotIn("virtual_pointing", record)


class Follows(Logged):
    def test_the_real_loop_follows_a_walking_virtual_person(self):
        def path(t):
            t = min(t, 20.0)  # 1.5 m ahead, walks a gentle S at 0.1 m/s for 20 s, then stands
            return (1.5 + 0.1 * t, 0.4 * math.sin(t / 20.0 * math.pi))

        base = FakeBase()
        clock = {"t": 0.0}
        lidar = SectorLidar(base, lambda: path(clock["t"]))
        state = run_follow.State(P, base, lidar, Camera(), None, self.log, clock=lambda: clock["t"])
        state.virtual = vo.VirtualOperator(P, port=0, controls=state.controls, clock=lambda: clock["t"], seed=5)
        records, gaps = [], []
        for i in range(int(30.0 / run_follow.PERIOD)):
            now = clock["t"] = state.camera.t = i * run_follow.PERIOD
            x, y = path(now)
            heading = math.atan2(*reversed([a - b for a, b in zip(path(now + 0.1), path(now))])) if now < 20 else 0.0
            state.virtual.receive(json.dumps({"t": now, "x": x, "y": y, "th": heading, "speaking": None}).encode(),
                                  ("127.0.0.1", 1))
            base.advance(run_follow.PERIOD)
            records.append(run_follow.cycle(state, now))
            gaps.append(math.dist(base.pose[:2], path(now)) - safety.FRONT)
        self.assertTrue(all("end" not in r for r in records))
        self.assertGreaterEqual(sum(r["locked"] for r in records) / len(records), 0.9)
        self.assertGreater(base.pose[0], 1.0)  # it drove after the person
        self.assertGreaterEqual(min(gaps), 0.3)


class Flags(unittest.TestCase):
    def test_the_virtual_operator_needs_remote(self):
        with mock.patch("builtins.print"), self.assertRaises(SystemExit) as stopped:
            run_follow.main(["--seconds", "1", "--virtual-operator", "5610"])
        self.assertIn("--virtual-operator", str(stopped.exception))

    def test_the_allow_list_is_the_flag_and_never_everyone(self):
        self.assertEqual(vo.parse_allow("192.168.31.50, 100.73.1.2,,"), ["192.168.31.50", "100.73.1.2"])
        self.assertEqual(vo.parse_allow(None), [])
        for bad in ("0.0.0.0", "not-an-address", "192.168.31.255", "224.0.0.1"):
            with self.assertRaises(ValueError):
                vo.parse_allow(bad)
        with mock.patch("builtins.print"), self.assertRaises(SystemExit) as stopped:
            run_follow.main(["--seconds", "1", "--remote", "--virtual-operator", "5610", "--virtual-allow", "0.0.0.0"])
        self.assertIn("--virtual-allow", str(stopped.exception))

if __name__ == "__main__":
    unittest.main()
