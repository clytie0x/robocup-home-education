"""The real run_follow loop in rooms rebuilt from recorded scans, a virtual operator standing where the log had them.

The room is the world points of one saved scan (the operator's own returns cut out), seen again from wherever
the base drives: each bearing of that scan returns its nearest point. The base is FakeBase (it drives exactly
as commanded), the operator a virtual_operator person (legs in the scan, a camera answer while in view).
"""
import json
import math
import unittest
from pathlib import Path

import geometry
import lidar
import run_follow
import virtual_operator as vo
from test_run_follow import P, Camera, FakeBase, Logged

FIXTURES = Path(__file__).resolve().parent / "fixtures"


class RecordedRoom:
    """A LiDAR that sees the world points of one recorded scan from wherever the base now stands."""

    def __init__(self, base, scan, pose, clear=(), clear_r=0.45):
        self.base = base
        self.bearings = [deg for deg, _ in scan]
        self.step = abs(self.bearings[0] - self.bearings[1])
        points = [geometry.base_to_odom(geometry.lidar_to_base(
            (m * math.cos(math.radians(d)), m * math.sin(math.radians(d))), P), pose)
            for d, m in lidar.usable(scan) if m > 0.3]
        self.points = [pt for pt in points if all(math.dist(pt, c) > clear_r for c in clear)]

    def latest(self):
        nearest = {}
        for pt in self.points:
            bx, by = geometry.odom_to_base(pt, self.base.pose)
            lx = bx - P["lidar_x"]
            k = round(math.degrees(math.atan2(by, lx)) / self.step)
            nearest[k] = min(nearest.get(k, math.inf), math.hypot(lx, by))
        return [(deg, nearest.get(round(deg / self.step), 0.0)) for deg in self.bearings]


class Scenes(Logged):
    def scene(self, name, seconds):
        """Cycle records of the loop with the operator standing still where the fixture's log had them."""
        fixture = json.loads((FIXTURES / name).read_text(encoding="utf-8"))
        pose, operator = tuple(fixture["pose"]), tuple(fixture["operator"])
        base = FakeBase()
        base.pose = pose
        clock = {"t": 0.0}
        room = RecordedRoom(base, fixture["scan"], pose, clear=[operator])
        state = run_follow.State(P, base, room, Camera(), None, self.log, clock=lambda: clock["t"])
        state.virtual = vo.VirtualOperator(P, port=0, controls=state.controls, clock=lambda: clock["t"], seed=5)
        facing = math.atan2(pose[1] - operator[1], pose[0] - operator[0])
        records = []
        for i in range(int(seconds / run_follow.PERIOD)):
            now = clock["t"] = state.camera.t = i * run_follow.PERIOD
            state.virtual.receive(json.dumps({"t": now, "x": operator[0], "y": operator[1], "th": facing,
                                              "speaking": None}).encode(), ("127.0.0.1", 1))
            base.advance(run_follow.PERIOD)
            record = run_follow.cycle(state, now)
            record["gap"] = math.dist(base.pose[:2], operator)
            records.append(record)
        return records

    def test_arriving_beside_a_wall_keeps_the_operator(self):
        """Run 20261115-023855, 160 s: the operator stood 0.5 m from a cabinet. Turning toward them on arrival,
        the base saw a leg-sized piece of that cabinet as a new id next to them; the split rule dropped the
        track, the merged rule refused every re-lock and the robot searched for a person in front of it."""
        records = self.scene("wall-beside-operator-2026-11-15.json", 22.0)
        first = next(i for i, r in enumerate(records) if r["locked"])
        after = records[first:]
        self.assertLessEqual(records[-1]["gap"], 1.0)  # it went to them
        self.assertEqual([r["why"] for r in after if r["why"] and "split" in r["why"]], [])
        self.assertNotIn("search", {r["mode"] for r in after})
        self.assertGreaterEqual(sum(r["locked"] for r in after) / len(after), 0.97)

    def test_goes_to_an_operator_it_stood_2_25_m_from(self):
        """Run 20261115-023855, the last 59 s: locked on the operator 2.25 m away, the planner found no path
        (code of 2026-11-15 00:44) and the robot stood. The same room must take it there and keep the lock."""
        records = self.scene("stood-2-25m-from-operator-2026-11-15.json", 16.0)
        locked = [r for r in records if r["locked"]]
        self.assertLessEqual(records[-1]["gap"], 1.0)
        self.assertEqual([r["time"] for r in locked if r["mode"] == "follow" and not r["path"]], [])
        self.assertNotIn("search", {r["mode"] for r in records})


if __name__ == "__main__":
    unittest.main()
