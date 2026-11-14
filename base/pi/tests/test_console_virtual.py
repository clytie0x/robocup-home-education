"""The console's virtual operator run: run_follow with --virtual-operator, behind the PIN."""
import json
import unittest
from pathlib import Path

import console
import test_console
from test_console import P, FakeJobs


class CommandTest(unittest.TestCase):
    def test_the_virtual_run_adds_the_port_and_the_pcs_address(self):
        cmd, error = console.task_command(True, 0.08, 5, P, py="py", virtual="192.168.31.50")
        self.assertIsNone(error)
        self.assertEqual([Path(a).name for a in cmd[1:6:2]], ["job_guard.py", "safe_run.py", "run_follow.py"])
        self.assertEqual(cmd[cmd.index("--virtual-operator") + 1], str(console.VIRTUAL_PORT))
        self.assertEqual(cmd[cmd.index("--virtual-allow") + 1], "192.168.31.50")
        self.assertIn("--remote", cmd)
        self.assertEqual(cmd[cmd.index("--dashboard-host") + 1], "127.0.0.1")

    def test_without_it_nothing_changes(self):
        self.assertEqual(console.task_command(False, 0.08, 2, P, py="py"),
                         console.task_command(False, 0.08, 2, P, py="py", virtual=None))
        self.assertNotIn("--virtual-operator", console.task_command(False, 0.08, 2, P, py="py")[0])

    def test_only_one_computers_ipv4_address(self):
        for bad in ("0.0.0.0", "192.168.31.255", "pc.local", "", "1.2.3.4; rm -rf", 5):
            self.assertIsNotNone(console.task_command(False, 0.08, 2, P, virtual=bad)[1], bad)


class Starts(FakeJobs):
    def __init__(self):
        super().__init__()
        self.calls = []

    def start(self, kind, label, cmd, info=None):
        self.calls.append((kind, label, cmd, info))
        return super().start(kind, label, cmd, info)


class VirtualHttpTest(unittest.TestCase):
    tearDown, request = test_console.HttpTest.tearDown, test_console.HttpTest.request  # the console on a free port, as in test_console

    def setUp(self):
        test_console.HttpTest.setUp(self)
        self.jobs = self.console.jobs = Starts()

    def test_a_virtual_run_needs_the_pin_and_allows_the_pc_that_asked(self):
        body = {"drive": False, "speed": 0.08, "minutes": 5, "virtual": True}
        self.assertEqual(self.request("/api/task/start", body)[0], 401)
        code, reply = self.request("/api/task/start", body, pin="2468")
        self.assertEqual((code, json.loads(reply)["error"]), (200, None))
        kind, label, cmd, info = self.jobs.calls[-1]
        self.assertEqual(kind, "task")
        self.assertIn("คนเสมือน", label)
        self.assertEqual(cmd[cmd.index("--virtual-allow") + 1], "127.0.0.1")  # the computer that sent the start
        self.assertEqual(info["virtual"], {"host": "127.0.0.1", "port": console.VIRTUAL_PORT})

    def test_the_pc_address_can_be_given(self):
        body = {"drive": True, "speed": 0.08, "minutes": 5, "virtual": True, "virtual_host": "192.168.31.77"}
        self.assertEqual(self.request("/api/task/start", body, pin="2468")[0], 200)
        _, _, cmd, info = self.jobs.calls[-1]
        self.assertEqual(cmd[cmd.index("--virtual-allow") + 1], "192.168.31.77")
        self.assertEqual(Path(cmd[3]).name, "safe_run.py")  # a drive is still under safe_run
        code, reply = self.request("/api/task/start", dict(body, virtual_host="0.0.0.0"), pin="2468")
        self.assertEqual(code, 400)

    def test_never_without_a_pin_file(self):
        self.console.pin = None
        code, reply = self.request("/api/task/start", {"drive": False, "speed": 0.08, "minutes": 5, "virtual": True})
        self.assertEqual(code, 403)
        self.assertIn("PIN", json.loads(reply)["error"])
        self.assertEqual(self.jobs.calls, [])

    def test_virtual_takes_true_or_false(self):
        code, _ = self.request("/api/task/start", {"drive": False, "speed": 0.08, "minutes": 5, "virtual": "yes"},
                               pin="2468")
        self.assertEqual(code, 400)
        self.assertEqual(self.jobs.calls, [])

    def test_a_plain_run_has_no_virtual_operator(self):
        self.assertEqual(self.request("/api/task/start", {"drive": False, "speed": 0.08, "minutes": 5}, pin="2468")[0], 200)
        _, label, cmd, info = self.jobs.calls[-1]
        self.assertNotIn("--virtual-operator", cmd)
        self.assertNotIn("คนเสมือน", label)
        self.assertIsNone(info.get("virtual"))

    def test_the_state_says_the_port(self):
        self.assertEqual(json.loads(self.request("/api/state")[1])["virtual_port"], console.VIRTUAL_PORT)

    def test_the_page_has_the_option(self):
        page = (console.STATIC / "index.html").read_text(encoding="utf-8")
        self.assertIn('id="seg-person"', page)
        self.assertIn("คนเสมือน", page)
        self.assertIn("virtual", (console.STATIC / "app.js").read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
