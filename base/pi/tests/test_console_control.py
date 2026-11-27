"""The console as the one way in (design 2026-11-16): programs over X-HMC-Client, the command log, the arm relay,
STOP for the arm, the hold, checks by name, and robotctl's mapping. A fake agent: nothing reaches a Jetson."""
import http.client
import io
import json
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
from http.server import ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "tools"))
import console  # noqa: E402
import robotctl  # noqa: E402
from test_console import FakeJobs, P  # noqa: E402


class Jobs(FakeJobs):
    busy = False  # a job running or starting

    def start(self, kind, label, cmd, info=None):
        self.cmds = getattr(self, "cmds", []) + [cmd]
        return super().start(kind, label, cmd, info)


class FakeAgent:
    """The Jetson agent's GET and POST /arm, as console.Jetson's fetch sees them."""

    def __init__(self):
        self.posts, self.down, self.arm = [], False, {"busy": False, "state": "resting", "error": None}
        self.timeouts = []  # of the POSTs

    def __call__(self, url, body=None, timeout=2.0):
        if self.down:
            raise urllib.error.URLError("refused")
        if not url.endswith("/arm"):
            return 404, {"error": "not found"}
        if body is None:
            return 200, dict(self.arm)
        self.posts.append(body)
        self.timeouts.append(timeout)
        return 200, {"ok": True, "error": None}


class FakeRun:
    """run_follow's dashboard as Console.fetch sees it: /command queues, /status gives record."""

    def __init__(self):
        self.commands, self.record, self.down = [], None, False

    def __call__(self, url, body=None, timeout=2.0):
        if self.down:
            raise OSError("refused")
        if url.endswith("/command"):
            self.commands.append(body["command"])
            return 200, {"error": None, "command": body["command"]}
        return 200, {"record": self.record}


class ControlTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.jobs, self.agent = Jobs(), FakeAgent()
        jetson = console.Jetson(hosts=("10.0.0.2",), fetch=self.agent)
        self.console = console.Console(P, console.Devices(P), self.jobs, jetson, run_url="http://127.0.0.1:9",
                                       pin="2468", log_root=Path(self.tmp.name))
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), console.handler_for(self.console))
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.port = self.server.server_address[1]

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.tmp.cleanup()

    def request(self, path, body=None, pin="2468", page=True, program=None, source="127.0.0.1"):
        headers = {"Content-Type": "application/json"}
        if page:
            headers["X-HMC-Console"] = "1"
        if program:
            headers["X-HMC-Client"] = program
        if pin:
            headers["X-Console-Pin"] = pin
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5, source_address=(source, 0))
        try:
            conn.request("GET" if body is None else "POST", path, None if body is None else json.dumps(body), headers)
            r = conn.getresponse()
            return r.status, json.loads(r.read() or b"null")
        finally:
            conn.close()

    def log(self, n=None):
        """The command lines written so far. The server writes a line before it sends the answer, so a
        request() that has returned already has its line. `n` still waits briefly for a read that races
        a handler which has not answered yet."""
        path = Path(self.tmp.name) / "console" / "commands.jsonl"
        deadline = time.monotonic() + 1.0
        while True:
            lines = []
            if path.is_file():
                lines = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
            if n is None or len(lines) >= n or time.monotonic() >= deadline:
                return lines
            time.sleep(0.01)

    def test_a_program_is_taken_only_from_the_pi_itself(self):
        dry = {"drive": False, "speed": 0.08, "minutes": 1}
        self.assertEqual(self.request("/api/task/start", dry, page=False, program="xohm@10.0.0.9")[0], 200)
        try:
            code = self.request("/api/task/start", dry, page=False, program="xohm@10.0.0.9", source="127.0.0.2")[0]
        except OSError as error:  # no second loopback address here
            self.skipTest("cannot send from 127.0.0.2: %s" % error)
        self.assertEqual(code, 403)  # anyone else on the network: the page header or nothing
        self.assertEqual(self.request("/api/task/start", dry, page=False, program="bad name!")[0], 403)
        self.assertEqual(self.request("/api/task/start", dry, page=False, program="xohm", pin="1111")[0], 401)
        self.assertEqual(self.jobs.started, ["task"])

    def test_every_post_is_logged_without_the_pin(self):
        self.request("/api/run/command", {"command": "follow", "pin": "2468"}, page=False, program="xohm@10.0.0.9")
        self.request("/api/task/start", {"drive": True}, pin="0000")
        self.request("/api/stop", {})
        first, wrong, stop = self.log(3)
        for line in (first, wrong, stop):  # not the raw line: its "time" can contain the digits 2468
            self.assertNotIn("2468", json.dumps({k: v for k, v in line.items() if k != "time"}))
        self.assertEqual((first["client"], first["address"], first["path"], first["code"]),
                         ("xohm@10.0.0.9", "127.0.0.1", "/api/run/command", 409))
        self.assertEqual(first["body"], {"command": "follow"})
        self.assertEqual(first["error"], "no task is running")
        self.assertEqual((wrong["client"], wrong["code"], wrong["body"]), ("page", 401, None))  # refused before the body
        self.assertEqual((stop["path"], stop["code"], stop["error"]), ("/api/stop", 200, None))
        code, body = self.request("/api/commands?n=2")
        self.assertEqual((code, [c["path"] for c in body["commands"]]), (200, ["/api/task/start", "/api/stop"]))

    def run_log(self, name, records):
        folder = Path(self.tmp.name) / name
        folder.mkdir(parents=True)
        (folder / "log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in records), encoding="utf-8")

    def test_the_voice_flows_commands_join_the_list(self):
        now = time.time()
        self.run_log("20261115-090000", [{"time": 5.0, "remote_cmd": "wait", "wall": now - 9}])  # an older run
        self.run_log("20261116-100000", [
            {"time": 10.0, "remote_cmd": "follow", "wall": now - 5},
            {"time": 10.1, "pose": [0, 0, 0]},  # a cycle
            {"time": 11.0, "remote_refused": "bag", "why": "not following yet", "by": "client", "wall": now - 4},
            {"time": 11.5, "remote_refused": "bag", "why": "not following yet", "by": "page", "wall": now - 3},
            {"time": 11.6, "page_cmd": "wait", "wall": now - 3},  # the console's own, logged there already
            {"time": 12.0, "remote_cmd": "return"},  # from before "wall": no time to place it by
            {"time": 13.0, "remote_cmd": "return", "wall": now + 60}])
        self.request("/api/stop", {})
        code, body = self.request("/api/commands?n=20")
        self.assertEqual(code, 200)
        got = [(c["client"], c["path"], c["code"], c["error"]) for c in body["commands"]]
        self.assertEqual(got, [("voice (Jetson)", "follow", 200, None),
                               ("voice (Jetson)", "bag", None, "not following yet"),
                               ("page", "/api/stop", 200, None),
                               ("voice (Jetson)", "return", 200, None)])
        self.assertEqual(set(body["commands"][0]), set(self.log()[0]))  # the shape of the console's own lines
        self.assertEqual([c["path"] for c in self.request("/api/commands?n=2")[1]["commands"]], ["/api/stop", "return"])

    def test_only_the_end_of_the_run_log_is_read(self):
        self.run_log("20261116-100000", [{"time": 1.0, "remote_cmd": "follow", "wall": 1.0}]
                     + [{"time": 2.0, "cycle": "x" * 100}] * 50 + [{"time": 3.0, "remote_cmd": "wait", "wall": 3.0}])
        self.assertEqual([c["path"] for c in console.voice_commands(self.tmp.name, size=2000)], ["wait"])
        self.assertEqual([c["path"] for c in console.voice_commands(self.tmp.name)], ["follow", "wait"])
        self.assertEqual(console.voice_commands(Path(self.tmp.name) / "missing"), [])

    def test_arm_commands_are_relayed_to_the_agent(self):
        self.assertEqual(self.request("/api/arm", {"cmd": "center"})[0], 200)
        self.assertEqual(self.request("/api/arm", {"cmd": "torque", "on": False})[0], 200)
        self.assertEqual(self.request("/api/arm", {"cmd": "torque"})[0], 400)
        self.assertEqual(self.request("/api/arm", {"cmd": "dance"})[0], 400)
        self.assertEqual(self.request("/api/arm", {"cmd": "home"}, pin=None)[0], 401)
        self.assertEqual(self.request("/api/arm/stop", {}, pin=None)[0], 200)  # a STOP: no PIN
        self.assertEqual(self.agent.posts, [{"cmd": "center"}, {"cmd": "torque", "on": False}, {"cmd": "stop"}])
        self.assertEqual(self.request("/api/arm"), (200, self.agent.arm))

    def test_no_arm_moves_while_a_task_runs_but_its_stop_goes(self):
        self.jobs.running = {"kind": "task", "label": "Drive task"}
        code, body = self.request("/api/arm", {"cmd": "home"})
        self.assertEqual(code, 409)
        self.assertIn("task", body["error"])
        self.assertEqual(self.request("/api/arm/stop", {})[0], 200)
        self.assertEqual(self.agent.posts, [{"cmd": "stop"}])

    def test_the_bag_commands_go_to_the_run(self):
        self.console.fetch = run = FakeRun()
        self.jobs.running = {"kind": "task", "label": "Drive task"}
        for command in ("bag", "bag_again", "bag_near", "bag_left", "bag_right"):
            self.assertEqual(self.request("/api/run/command", {"command": command})[0], 200, command)
        self.assertEqual(self.request("/api/run/command", {"command": "bag_far"})[0], 400)
        self.assertEqual(run.commands, ["bag", "bag_again", "bag_near", "bag_left", "bag_right"])

    def test_pick_and_drop_at_a_given_point_wait_for_the_whole_bag_job(self):
        pick = {"cmd": "pick_bag", "x": 0.31, "y": 0.01, "top": 0.27, "yaw": 0.4}
        code, body = self.request("/api/arm", pick)
        self.assertEqual((code, body["sent"]), (200, pick))
        self.assertEqual(self.request("/api/arm", {"cmd": "drop_bag", "x": 0.3, "y": -0.1, "top": 9})[0], 200)
        self.assertEqual(self.agent.posts, [pick, {"cmd": "drop_bag", "x": 0.3, "y": -0.1}])  # a drop has no top
        self.assertEqual(self.agent.timeouts, [console.ARM_JOB_TIMEOUT] * 2)
        self.assertGreaterEqual(console.ARM_JOB_TIMEOUT, 60 + 150)  # bag_arm's HOME_TIMEOUT + PICK_TIMEOUT
        for bad in ({"cmd": "drop_bag", "x": 0.3}, {"cmd": "pick_bag", "x": True, "y": 0.0},
                    {"cmd": "pick_bag", "x": float("nan"), "y": 0.0}, {"cmd": "pick_bag", "x": 0.3, "y": 0.0, "top": "high"}):
            self.assertEqual(self.request("/api/arm", bad)[0], 400, bad)
        code, body = self.request("/api/arm", {"cmd": "pick_bag"})  # no task, no point
        self.assertEqual(code, 409)
        self.assertIn("no reached bag", body["error"])
        self.assertEqual(len(self.agent.posts), 2)

    def test_the_console_default_bag_point_is_not_a_measurement(self):
        code, body = self.request("/api/arm", {"cmd": "pick_bag", "x": 0.36, "y": 0.0})
        self.assertEqual(code, 409)
        self.assertEqual(body["reason"], "DEFAULT_NOT_MEASURED")
        self.assertIn("0.36", body["error"])
        code, body = self.request("/api/arm", {"cmd": "pick_bag", "x": 0.36, "y": 0.00, "measured": True})
        self.assertEqual(code, 200)
        self.assertEqual(body["sent"], {"cmd": "pick_bag", "x": 0.36, "y": 0.0})
        self.assertEqual(self.request("/api/arm", {"cmd": "drop_bag", "x": 0.36, "y": 0.0})[0], 200)
        self.assertEqual(len(self.agent.posts), 2)

    def test_a_reached_bag_at_the_default_point_is_still_a_measurement(self):
        self.jobs.running = {"kind": "task", "label": "Drive task"}
        run = FakeRun()
        run.record = {"phase": "wait", "bag_task": {"result": {"ok": True, "x": 0.36, "y": 0.0}}}
        self.console.fetch = run
        code, body = self.console.bag_job("pick_bag", {}, task=True)
        self.assertEqual(code, 200)
        self.assertEqual(body["sent"], {"cmd": "pick_bag", "x": 0.36, "y": 0.0})

    def test_arm_and_bag_buttons_are_refused_while_a_task_runs(self):
        self.console.fetch = FakeRun()
        self.jobs.running = {"kind": "task", "label": "Drive task"}
        for cmd in ({"cmd": "pick_bag"}, {"cmd": "drop_bag", "x": 0.3, "y": 0.0},
                    {"cmd": "pick_bag", "x": 0.3, "y": 0.0}, {"cmd": "home"}, {"cmd": "bagpick"}):
            code, body = self.request("/api/arm", cmd)
            self.assertEqual(code, 409, cmd)
            self.assertIn("task", body["error"])
        self.assertEqual(self.agent.posts, [])

    def test_bagpick_waits_for_the_whole_run_with_the_pick_rules(self):
        code, body = self.request("/api/arm", {"cmd": "bagpick"})
        self.assertEqual((code, body["sent"]), (200, {"cmd": "bagpick", "putback": False}))
        self.assertEqual(self.request("/api/arm", {"cmd": "bagpick", "putback": True})[0], 200)
        self.assertEqual(self.request("/api/arm", {"cmd": "bagpick", "putback": "yes"})[0], 400)
        self.assertEqual(self.request("/api/arm", {"cmd": "bagpick"}, pin=None)[0], 401)
        self.assertEqual(self.agent.posts[1], {"cmd": "bagpick", "putback": True})
        self.assertEqual(self.agent.timeouts, [console.ARM_BAGPICK_TIMEOUT] * 2)
        self.assertGreaterEqual(console.ARM_BAGPICK_TIMEOUT, 330.0)  # the agent's ARM_BAGPICK_TIMEOUT
        self.console.fetch = FakeRun()
        self.jobs.running = {"kind": "task", "label": "Drive task"}
        code, body = self.request("/api/arm", {"cmd": "bagpick"})
        self.assertEqual(code, 409)
        self.assertIn("task", body["error"])
        self.assertEqual(self.request("/api/arm", {"cmd": "bagpick"})[0], 409)
        self.assertEqual(len(self.agent.posts), 2)
        self.jobs.running = None
        self.console.hold = {"who": "deploy", "since": 0.0}
        self.assertEqual(self.request("/api/arm", {"cmd": "bagpick"})[0], 409)
        self.assertEqual(len(self.agent.posts), 2)

    def test_stop_stops_the_arm_too_and_the_base_without_the_agent(self):
        code, body = self.request("/api/stop", {}, pin=None, page=False)
        self.assertEqual((code, body["error"], body["arm"]), (200, None, "arm stopped"))
        self.assertEqual((self.jobs.stopped, self.agent.posts), (1, [{"cmd": "stop"}]))
        self.agent.down = True
        code, body = self.request("/api/stop", {}, pin=None, page=False)
        self.assertEqual((code, body["error"], self.jobs.stopped), (200, None, 2))
        self.assertIn("arm stop", body["arm"])

    def test_a_command_not_answered_in_time_is_never_sent_to_the_other_address(self):
        sent = []

        def slow(url, body=None, timeout=2.0):
            sent.append(url)
            raise TimeoutError("timed out")
        jetson = console.Jetson(hosts=("10.0.0.2", "192.168.31.137"), fetch=slow)
        self.assertEqual(jetson.arm({"cmd": "center"})[0], 504)
        self.assertEqual(sent, ["http://10.0.0.2:8090/arm"])

    def test_hold_and_release(self):
        code, body = self.request("/api/hold", {}, page=False, program="xohm@10.0.0.9")
        self.assertEqual((code, body["hold"]["who"]), (200, "xohm@10.0.0.9"))
        self.assertEqual(self.request("/api/state")[1]["hold"]["who"], "xohm@10.0.0.9")
        self.assertEqual(self.request("/api/hold", {"who": "deploy"})[0], 409)  # one hold at a time
        for path, body in (("/api/task/start", {"drive": False, "speed": 0.08, "minutes": 1}),
                           ("/api/check/start", {"check": "floor"}), ("/api/arm", {"cmd": "home"}),
                           ("/api/jetson/start", {"name": "detector"}), ("/api/jetson/stop-all", {})):
            code, answer = self.request(path, body)
            self.assertEqual(code, 409, path)
            self.assertIn("held by xohm@10.0.0.9", answer["error"])
        self.assertEqual(self.request("/api/stop", {})[0], 200)  # STOP is never held
        self.assertEqual(self.request("/api/arm/stop", {})[0], 200)
        self.assertEqual((self.jobs.started, self.agent.posts[-1]), ([], {"cmd": "stop"}))
        self.assertEqual(self.request("/api/release", {})[0], 200)
        self.assertIsNone(self.request("/api/state")[1]["hold"])
        self.assertEqual(self.request("/api/check/start", {"check": "floor"})[0], 200)

    def test_no_hold_while_busy(self):
        self.jobs.busy = True  # a job running, or one starting (not in status() yet)
        code, body = self.request("/api/hold", {"who": "deploy"})
        self.assertEqual(code, 409)
        self.assertIn("job", body["error"])
        self.jobs.busy = False
        self.agent.arm["busy"] = True
        self.assertEqual(self.request("/api/hold", {"who": "deploy"})[0], 409)
        self.assertIsNone(self.console.hold)
        self.agent.arm["busy"], self.agent.down = False, True  # no Jetson: no arm move of the console's
        self.assertEqual(self.request("/api/hold", {"who": "deploy"})[0], 200)
        self.assertEqual(self.request("/api/hold", {"who": "x y"})[0], 400)

    def test_checks_by_name_from_a_program_and_from_the_page(self):
        line = {"check": "base_check", "args": ["--line", "0.1", "--metres", "1.0", "--yes-owner-at-robot"]}
        code, body = self.request("/api/check/start", line, page=False, program="xohm@10.0.0.9")
        self.assertEqual(code, 400)
        self.assertIn("--yes-owner-at-robot", body["error"])
        self.assertEqual(self.request("/api/check/start", line)[0], 200)  # a person pressed it on the page
        self.assertEqual([Path(a).name for a in self.jobs.cmds[0][1:6:2]], ["job_guard.py", "safe_run.py", "base_check.py"])
        fast = {"check": "base_check", "args": ["--line", "0.2", "--yes-owner-at-robot"]}
        code, body = self.request("/api/check/start", fast)
        self.assertEqual(code, 400)
        self.assertIn("max_speed", body["error"])
        walk = {"check": "walk_check", "args": ["--seconds", "60"]}
        self.assertEqual(self.request("/api/check/start", walk, page=False, program="xohm@10.0.0.9")[0], 200)
        self.assertEqual(self.request("/api/check/start", {"check": "air_check"})[0], 400)
        self.assertEqual(self.jobs.started, ["check-base_check", "check-walk_check"])


class CheckArgsTest(unittest.TestCase):
    def check(self, name, args, page=False):
        return console.check_command(name, py="py", args=args, max_speed=0.12, page=page)

    def test_allowed(self):
        cmd, label, moves = self.check("tether_run", ["line", "1.0", "--speed", "0.1", "--back-up-if-blocked"])
        self.assertTrue(moves)
        self.assertEqual([Path(a).name for a in cmd[1::2][:3]], ["job_guard.py", "safe_run.py", "tether_run.py"])
        self.assertEqual(cmd[-4:], ["1.0", "--speed", "0.1", "--back-up-if-blocked"])
        self.assertEqual(label, "tether_run line 1.0 --speed 0.1 --back-up-if-blocked")
        for name, args in (("tether_run", ["turn", "-90"]), ("tether_run", ["uturn", "left"]),
                           ("tether_run", ["precheck-line", "1.5", "0.12"]), ("tether_run", ["init", "--length=10"]),
                           ("base_check", ["--turn", "-0.26", "--degrees", "180"]), ("walk_check", []),
                           ("lap_check", ["--seconds", "120"])):
            self.assertIsNotNone(self.check(name, args), (name, args))
        cmd, _, moves = self.check("walk_check", ["--seconds", "30"])
        self.assertFalse(moves)
        self.assertNotIn("safe_run", " ".join(cmd))
        self.assertIsNotNone(self.check("approach", ["--to", "0.37", "--yes-close-approach"], page=True))

    def test_refused(self):
        for name, args in (("tether_run", ["line", "1.0", "--speed", "0.2"]), ("tether_run", ["precheck-line", "1", "0.3"]),
                           ("tether_run", ["after-base-check", "log.json"]), ("tether_run", ["--state", "x", "init"]),
                           ("tether_run", []), ("base_check", ["--max-speed", "0.3"]), ("base_check", ["--line=0.5"]),
                           ("base_check", ["--linear-scale", "2"]), ("base_check", ["--out", "/tmp"]),
                           ("back_up", ["--metres", "0.2", "--avoid", "trail.json"]), ("lap_check", ["--seconds", "1e9"]),
                           ("walk_check", ["--seconds", "60;reboot"]), ("walk_check", ["--seconds"]),
                           ("walk_check", ["extra"]), ("walk_check", "--seconds 60"), ("wheel", ["--fast"]),
                           ("tether_run", [["line"]])):
            with self.assertRaises(ValueError, msg=(name, args)):
                self.check(name, args, page=True)
        for flag in ("--yes-back-up", "--yes-floor-run", "--yes-close-approach", "--yes-owner-at-robot"):
            tool = {"--yes-back-up": "back_up", "--yes-floor-run": "lap_check", "--yes-close-approach": "approach",
                    "--yes-owner-at-robot": "base_check"}[flag]
            with self.assertRaises(ValueError):
                self.check(tool, [flag])  # never from a program
            self.assertIsNotNone(self.check(tool, [flag], page=True))
        self.assertIsNone(self.check("air_check", []))
        self.assertIsNone(self.check(["wheel"], []))

    def test_joystick(self):
        flag = "--yes-owner-at-robot"
        cmd, label, moves = self.check("joystick_drive", ["--max-speed", "0.08", "--seconds", "120", flag], page=True)
        self.assertTrue(moves)
        self.assertEqual([Path(a).name for a in cmd[1::2][:3]], ["job_guard.py", "safe_run.py", "joystick_drive.py"])
        for args in (["--dry"], ["--device", "/dev/input/js1"], ["--max-speed", "0.3"], ["--seconds", "1e9"], ["extra"]):
            with self.assertRaises(ValueError, msg=args):
                self.check("joystick_drive", args + [flag], page=True)
        with self.assertRaises(ValueError):
            self.check("joystick_drive", [flag])  # the presence flag never comes from a program


class RobotctlTest(unittest.TestCase):
    def test_mapping(self):
        cases = [
            (["state"], ("GET", "/api/state", None)), (["stop"], ("POST", "/api/stop", {})),
            (["check", "wheel"], ("POST", "/api/check/start", {"check": "wheel", "args": []})),
            (["check", "tether_run", "turn", "-90"],
             ("POST", "/api/check/start", {"check": "tether_run", "args": ["turn", "-90"]})),
            (["task", "drive", "0.12", "5"], ("POST", "/api/task/start", {"drive": True, "speed": 0.12, "minutes": 5.0})),
            (["task", "dry", "0.08", "2", "--virtual", "192.168.31.50"],
             ("POST", "/api/task/start", {"drive": False, "speed": 0.08, "minutes": 2.0, "virtual": True,
                                          "virtual_host": "192.168.31.50"})),
            (["run", "follow"], ("POST", "/api/run/command", {"command": "follow"})),
            (["run", "stop"], ("POST", "/api/stop", {})),
            (["arm", "center"], ("POST", "/api/arm", {"cmd": "center"})),
            (["arm", "torque", "off"], ("POST", "/api/arm", {"cmd": "torque", "on": False})),
            (["arm", "stop"], ("POST", "/api/arm/stop", {})), (["arm", "state"], ("GET", "/api/arm", None)),
            (["jetson", "restart", "detector"], ("POST", "/api/jetson/restart", {"name": "detector"})),
            (["jetson", "start-all"], ("POST", "/api/jetson/start-all", {})),
            (["hold"], ("POST", "/api/hold", {"who": "me"})), (["release"], ("POST", "/api/release", {})),
            (["log"], ("GET", "/api/commands?n=20", None)), (["log", "5"], ("GET", "/api/commands?n=5", None)),
            (["run", "bag_near"], ("POST", "/api/run/command", {"command": "bag_near"})),
            (["run", "bag_again"], ("POST", "/api/run/command", {"command": "bag_again"})),
            (["arm", "pick"], ("POST", "/api/arm", {"cmd": "pick_bag"})),
            (["arm", "pick", "0.31", "0.01"], ("POST", "/api/arm", {"cmd": "pick_bag", "x": 0.31, "y": 0.01})),
            (["arm", "pick", "0.31", "0", "0.27", "-0.5"],
             ("POST", "/api/arm", {"cmd": "pick_bag", "x": 0.31, "y": 0.0, "top": 0.27, "yaw": -0.5})),
            (["arm", "drop", "0.3", "-0.1"], ("POST", "/api/arm", {"cmd": "drop_bag", "x": 0.3, "y": -0.1})),
            (["arm", "bagpick"], ("POST", "/api/arm", {"cmd": "bagpick", "putback": False})),
            (["arm", "bagpick", "--putback"], ("POST", "/api/arm", {"cmd": "bagpick", "putback": True})),
        ]
        for argv, want in cases:
            self.assertEqual(robotctl.request(argv, who="me"), want, argv)
        for argv in ([], ["dance"], ["task", "drive", "fast", "5"], ["task", "drive", "0.12"], ["run", "jump"],
                     ["arm", "torque"], ["arm", "wave"], ["jetson", "start"], ["log", "x"], ["state", "extra"],
                     ["task", "dry", "0.08", "2", "--other", "1.2.3.4"], ["arm", "pick", "0.3"],
                     ["arm", "pick", "0.3", "0", "0.27"], ["arm", "drop"], ["arm", "drop", "x", "0"],
                     ["task", "drop", "1", "2"], ["run", "bag_far"], ["arm", "bagpick", "--hold"],
                     ["arm", "bagpick", "--putback", "x"], ["arm", "--putback"]):
            with self.assertRaises(ValueError, msg=argv):
                robotctl.request(argv)

    def fetch(self, statuses, answers=None):
        """robotctl fetch over a fake console: GET /api/run gives statuses one by one (the last one stays)."""
        sent, now = [], [0.0]

        def send(method, path, body, heads):
            sent.append((method, path, body, heads.get("X-HMC-Session")))
            if path.startswith("/api/run") and method == "GET":
                status = statuses.pop(0) if len(statuses) > 1 else statuses[0]
                if isinstance(status, BaseException):
                    raise status
                return 200, {"record": {"phase": "bag", "bag_task": status}}
            return (answers or {}).get(path, (200, {"error": None, "session": "ab" * 16}))

        def sleep(s):
            now[0] += s
        out, err = io.StringIO(), io.StringIO()
        code = robotctl.fetch(send, {"X-HMC-Client": "me"}, out, err, clock=lambda: now[0], sleep=sleep)
        return code, sent, out.getvalue(), err.getvalue()

    def test_fetch_drives_to_the_bag_then_picks_it(self):
        old = {"id": 5.0, "state": "reached", "result": {"ok": True, "x": 9.0, "y": 9.0}}  # an earlier task
        reached = {"ok": True, "code": "OK", "x": 0.305, "y": 0.012, "top": 0.26, "odometry_m": 0.0}
        code, sent, out, _ = self.fetch([old, old, {"id": 7.0, "state": "far", "result": None},
                                         {"id": 7.0, "state": "reached", "result": reached}])
        self.assertEqual(code, 0, out)
        self.assertEqual(sent[1][:3], ("POST", "/api/run/command", {"command": "bag_near"}))
        self.assertTrue(all(s[3] == "ab" * 16 for s in sent[2:-1]))  # the polls are the run's page heartbeat
        self.assertEqual(sent[-1][:3], ("POST", "/api/arm", {"cmd": "pick_bag", "x": 0.305, "y": 0.012, "top": 0.26}))
        self.assertIn("bag task far", out)
        self.assertIn("bag task reached", out)

    def test_fetch_stops_on_a_failure_a_timeout_a_refusal_and_ctrl_c(self):
        failed = {"id": 7.0, "state": "failed", "result": {"ok": False, "code": "NO_BAG", "message": "none"}}
        code, sent, _, err = self.fetch([{}, failed])
        self.assertEqual((code, sent[-1][1]), (1, "/api/run?heartbeat=1"))
        self.assertIn("NO_BAG", err)
        code, sent, _, err = self.fetch([{}])  # the run never starts it
        self.assertEqual(code, 1)
        self.assertIn("no bag result", err)
        self.assertLessEqual(len(sent), 2 + robotctl.FETCH_S / robotctl.POLL_S + 1)
        code, sent, _, err = self.fetch([{}], {"/api/run/command": (409, {"error": "no task is running"})})
        self.assertEqual((code, len(sent), err.strip()), (1, 2, "no task is running"))
        code, sent, _, _ = self.fetch([{}, KeyboardInterrupt()])
        self.assertEqual((code, sent[-1][:2]), (130, ("POST", "/api/stop")))

    def test_who(self):
        self.assertEqual(robotctl.default_who({"USER": "xohm", "SSH_CLIENT": "192.168.31.50 51234 22"}),
                         "xohm@192.168.31.50")
        self.assertEqual(robotctl.default_who({"USER": "xohm"}), "xohm@local")
        self.assertTrue(console.CLIENT.match(robotctl.default_who({"USER": "xohm", "SSH_CLIENT": "fe80::1%eth0 1 22"})))

    def test_the_pin_is_sent_and_never_printed(self):
        with tempfile.TemporaryDirectory() as tmp:
            pin = Path(tmp) / "pin"
            self.assertNotIn("X-Console-Pin", robotctl.headers("me", pin))  # no file: no PIN header
            pin.write_text("4821\n")
            self.assertEqual(robotctl.headers("me", pin)["X-Console-Pin"], "4821")
        seen = []

        def send(method, path, body, heads):
            seen.append((method, path, body, heads["X-HMC-Client"]))
            return 409, {"error": "held by deploy", "pin": True}
        out, err = io.StringIO(), io.StringIO()
        self.assertEqual(robotctl.main(["--who", "ai", "arm", "center"], send=send, out=out, err=err), 1)
        self.assertEqual(seen, [("POST", "/api/arm", {"cmd": "center"}, "ai")])
        self.assertEqual((json.loads(out.getvalue())["error"], err.getvalue().strip()), ("held by deploy", "held by deploy"))
        self.assertEqual(robotctl.main(["state"], send=lambda *a: (200, {"host": "pi"}), out=out, err=err), 0)

        def down(*a):
            raise ConnectionRefusedError("refused")
        self.assertEqual(robotctl.main(["state"], send=down, out=out, err=err), 2)
        self.assertEqual(robotctl.main(["dance"], send=send, out=out, err=err), 3)


if __name__ == "__main__":
    unittest.main()
