import json
import socket
import threading
import time
import unittest

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
