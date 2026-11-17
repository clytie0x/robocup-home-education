import json
import math
import socket
import threading
import time
import unittest
from pathlib import Path

import plan_server
import remote_plan
from hmc.world import Circle


def wait_for(check, seconds=5.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if check():
            return True
        time.sleep(0.01)
    return False


class PlanServerTest(unittest.TestCase):
    def test_answer_plans_with_the_close_plan(self):
        posts = [Circle(1.5, y, 0.05) for y in (0.375, 0.475, 0.575, -0.375, -0.475, -0.575)]  # a 0.65 m gap
        request = {"id": 7, "start": [0, 0], "goal": [3, 0], "near_side": False, "heading": 0.0, "robot_radius": 0.29,
                   "shapes": [["Circle", c.x, c.y, c.r] for c in posts]}
        got = plan_server.answer(request)
        self.assertEqual(got["id"], 7)
        self.assertIsNotNone(got["path"])
        self.assertGreater(got["path"][-1][0], 2.5)

    def test_answer_drives_toward_an_operator_it_cannot_reach(self):
        # Recorded 2026-11-15: the operator 2.5 m ahead, open floor between (photo), but stray
        # remembered circles closed every way; the planner answered no path and the robot stood still.
        recorded = json.loads((Path(__file__).parent / "fixtures" / "no-path-2026-11-15.json").read_text())
        request = dict(recorded, id=3, close_plan=True)
        shapes = [Circle(*values) for _, *values in recorded["shapes"]]
        start, goal = recorded["start"], recorded["goal"]
        path = plan_server.answer(request)["path"]
        self.assertIsNotNone(path)  # before the fix: None, so the robot stood 2.5 m away
        self.assertLess(math.dist(path[-1], goal), math.dist(start, goal) - 0.3)  # it comes closer (0.43 m here)
        length = sum(math.dist(a, b) for a, b in zip([start] + path, path))
        self.assertLessEqual(length, math.dist(start, goal) + 0.1)  # toward the operator, no detour out of view
        # every waypoint keeps the wide berth (circle plus SAFETY) from every shape, or near the start
        # (it stands inside that berth) no less than it has now
        here = min(s.clearance(tuple(start)) for s in shapes)
        for point in path:
            self.assertGreaterEqual(min(s.clearance(tuple(point)) for s in shapes), min(here, 0.29 + 0.1) - 1e-9)

    def test_answer_finds_the_way_out_of_a_berth_boxed_in_by_other_zones(self):
        # Recorded 2026-11-15 (134.9 to 139.0 s): a remembered point 0.31 m from the base put it inside the wide
        # berth; the cells freed round it (a disc of the berth) all ended inside the zones of the shapes beside
        # it, so the operator 1.4 to 1.7 m away had no path and the robot stood. The way out now follows the
        # cells no closer to anything than the base is (navigation.ESCAPE).
        recorded = json.loads((Path(__file__).parent / "fixtures" / "no-path-start-in-zone-2026-11-15.json").read_text())
        for i, request in enumerate(recorded["requests"]):
            with self.subTest(i):
                shapes = [Circle(*values) for _, *values in request["shapes"]]
                start, goal = tuple(request["start"]), tuple(request["goal"])
                path = plan_server.answer(dict(request, id=i, robot_radius=0.29, close_plan=True))["path"]
                self.assertIsNotNone(path)
                self.assertLess(math.dist(path[-1], goal), math.dist(start, goal) - 0.3)
                here = min(s.clearance(start) for s in shapes)
                for point in path:  # never nearer to anything than where it stood
                    self.assertGreaterEqual(min(s.clearance(tuple(point)) for s in shapes), min(here, 0.29 + 0.1) - 1e-9)

    def test_only_the_newest_waiting_request_counts(self):
        self.assertEqual(plan_server.newest([b'{"id": 1}', b'{"id": 2}'])["id"], 2)
        self.assertIsNone(plan_server.newest([]))


class RemotePlannerTest(unittest.TestCase):
    def test_plans_in_the_background_and_reports_when_the_server_is_gone(self):
        ours, theirs = socket.socketpair()
        threading.Thread(target=plan_server.serve, args=(theirs,), daemon=True).start()
        now = [0.0]
        remote = remote_plan.RemotePlanner("x", 0, 0.29, clock=lambda: now[0], connect=lambda *a, **k: ours)
        remote.start()
        self.addCleanup(remote.stop)
        self.assertFalse(remote.alive())
        self.assertIsNone(remote.plan((0, 0), (2, 0), []))  # nothing answered yet: no path this cycle
        self.assertTrue(wait_for(remote.alive))
        path = remote.plan((0, 0), (2, 0), [])
        self.assertIsNotNone(path)
        self.assertAlmostEqual(path[-1][0], 2.0, places=6)
        now[0] += remote_plan.MAX_AGE + 0.1  # no answer for longer than MAX_AGE: the caller plans itself
        self.assertFalse(remote.alive())

    def test_after_forget_only_an_answer_to_a_newer_request_counts(self):
        # A correction moved the believed pose (or the goal changed): the answer on its way is out of date.
        ours, theirs = socket.socketpair()
        server = theirs.makefile("rb")
        remote = remote_plan.RemotePlanner("x", 0, 0.29, connect=lambda *a, **k: ours)
        remote.start()
        self.addCleanup(remote.stop)
        self.assertFalse(remote.waiting())
        remote.plan((0, 0), (2, 0), [])
        old = json.loads(server.readline())  # on its way to the planner
        remote.forget()
        self.assertTrue(remote.waiting())
        theirs.sendall((json.dumps({"id": old["id"], "path": [[0, 0], [2, 0]], "seconds": 0.1}) + "\n").encode())
        self.assertTrue(wait_for(remote.alive))  # the server answered, but for the old frame
        self.assertIsNone(remote.plan((0, 0.3), (2, 0.3), []))
        self.assertTrue(remote.waiting())
        new = json.loads(server.readline())
        theirs.sendall((json.dumps({"id": new["id"], "path": [[0, 0.3], [2, 0.3]], "seconds": 0.1}) + "\n").encode())
        self.assertTrue(wait_for(lambda: not remote.waiting()))
        self.assertEqual(remote.plan((0, 0.3), (2, 0.3), []), [(0, 0.3), (2, 0.3)])


if __name__ == "__main__":
    unittest.main()
