"""The PC app's ssh control and its local API, with a fake ssh: nothing here logs in to the robot."""
import json
import math
import shutil
import subprocess
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import app as app_module  # noqa: E402
import remote  # noqa: E402

DRY = {"drive": False, "speed": 0.08, "minutes": 10}
DRIVE = {"drive": True, "speed": 0.12, "minutes": 5}
BASH = shutil.which("bash")


class Scripts(unittest.TestCase):
    def test_a_drive_runs_under_safe_run_with_the_virtual_operator(self):
        script = remote.start_script(DRIVE, "192.168.31.50", 5610)
        self.assertIn(".venv/bin/python tools/safe_run.py .venv/bin/python run_follow.py --drive --seconds 300"
                      " --max-speed 0.12 --remote --dashboard-host 127.0.0.1 --virtual-operator 5610"
                      " --virtual-allow 192.168.31.50", script)
        self.assertIn("setsid ", script)
        self.assertIn("< /dev/null &", script)
        self.assertIn("echo $! > logs/virtual/run.pid", script)

    def test_a_dry_run_has_no_drive_and_no_safe_run(self):
        script = remote.start_script(DRY, "192.168.31.50", 5610)
        self.assertNotIn("--drive", script)
        self.assertNotIn("safe_run", script)

    def test_through_the_console_when_it_runs_with_the_pin_read_on_the_pi(self):
        script = remote.start_script(DRY, "192.168.31.50", 5610)
        self.assertIn("cat ~/.config/hmc-console-pin", script)
        self.assertIn('"virtual": true', script)
        self.assertIn('"virtual_host": "192.168.31.50"', script)

    def test_bad_settings_never_reach_the_shell(self):
        for settings, pc in ((dict(DRY, speed=0.5), "192.168.31.50"), (dict(DRY, minutes=999), "192.168.31.50"),
                             (dict(DRY, drive="yes"), "192.168.31.50"), (DRY, "1.2.3.4; rm -rf ~"), (DRY, "pc.local")):
            with self.assertRaises((ValueError, OSError)):
                remote.start_script(settings, pc, 5610)

    def test_nothing_is_killed_by_a_name_pattern(self):
        for script in (remote.start_script(DRIVE, "10.0.0.5", 5610), remote.stop_script(), remote.last_record_script()):
            for word in ("pkill", "pgrep", "killall", "kill -9 $(", "ps "):
                self.assertNotIn(word, script)
        stop = remote.stop_script()
        self.assertIn('kill -TERM -- "-$PID"', stop)  # the process group from the pid file
        self.assertLess(stop.index("/api/stop"), stop.index("kill -TERM"))  # STOP and the run's own end first
        self.assertLess(stop.index('{"end": true}'), stop.index("kill -TERM"))

    @unittest.skipIf(BASH is None, "no bash here")
    def test_the_scripts_are_valid_bash(self):
        for script in (remote.start_script(DRIVE, "10.0.0.5", 5610), remote.start_script(DRY, "10.0.0.5", 5610),
                       remote.stop_script(), remote.last_record_script()):
            done = subprocess.run([BASH, "-n"], input=script, capture_output=True, text=True)
            self.assertEqual(done.returncode, 0, done.stderr)


class FakeRun:
    def __init__(self, answers):
        self.answers, self.calls = list(answers), []

    def __call__(self, argv, input=None, **kw):
        self.calls.append((argv, input))
        code, out = self.answers.pop(0)
        return SimpleNamespace(returncode=code, stdout=out, stderr="")


class SshTest(unittest.TestCase):
    def test_the_lab_wifi_first_then_tailscale(self):
        run = FakeRun([(255, ""), (0, "STARTED 4242 logs/virtual/x.log\n")])
        ssh = remote.Ssh(run=run)
        self.assertIsNone(ssh.start(DRY, "192.168.31.50", 5610))
        self.assertEqual([c[0][-3] for c in run.calls], ["xohm@192.168.31.21", "xohm@100.73.128.36"])
        self.assertEqual(run.calls[0][0][-2:], ["bash", "-s"])  # the script goes on stdin
        self.assertIn("BatchMode=yes", run.calls[0][0])
        self.assertEqual(ssh.host, "100.73.128.36")  # remembered for the datagrams and the next call

    def test_the_answers(self):
        cases = [("CONSOLE {\"error\": null}\n", None), ("CONSOLE {\"error\": \"busy\"}\n", "busy"),
                 ("ERROR a virtual operator run is already running (pid 7)\n", "a virtual operator run is already running (pid 7)")]
        for out, want in cases:
            self.assertEqual(remote.Ssh(run=FakeRun([(0, out)])).start(DRY, "10.0.0.5", 5610), want)

    def test_the_end_from_the_newest_log(self):
        line = json.dumps({"time": 1.0, "stop": None, "end": "done: back at the start"})
        self.assertEqual(remote.Ssh(run=FakeRun([(0, line + "\n")])).last_end(), "done: back at the start")
        self.assertIsNone(remote.Ssh(run=FakeRun([(0, "")])).last_end())


class FakeSsh:
    user, host = "xohm", "127.0.0.1"
    tunnel = None

    def __init__(self):
        self.started, self.stopped = [], 0

    def start(self, settings, pc, port):
        self.started.append((settings, pc, port))
        return None

    def stop(self):
        self.stopped += 1
        return None

    def ensure_tunnel(self):
        return True

    def close(self):
        pass


class ApiTest(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.ssh = FakeSsh()
        self.app = app_module.App("http://127.0.0.1:9", csv_path=Path(folder.name) / "e.csv", ssh=self.ssh)
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), None)
        self.port = self.server.server_address[1]
        self.server.RequestHandlerClass = app_module.handler_for(self.app, self.port)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)

    def call(self, path, body=None, header=True, host=None):
        headers = {"Content-Type": "application/json"}
        if header:
            headers[app_module.APP_HEADER] = "1"
        if host:
            headers["Host"] = host
        req = urllib.request.Request("http://127.0.0.1:%d%s" % (self.port, path),
                                     None if body is None else json.dumps(body).encode(), headers)
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    def test_scripts_start_and_stop_the_auto_loop(self):
        code, body = self.call("/api/auto/start", {"drive": True, "speed": 0.12, "minutes": 5})
        self.assertEqual((code, body["run"]), (200, DRIVE))
        self.assertTrue(self.call("/api/auto/status")[1]["auto"]["on"])
        self.assertEqual(self.call("/api/auto/stop", {})[0], 200)
        self.assertEqual(self.ssh.stopped, 1)  # the robot's run got its STOP over ssh
        status = self.call("/api/auto/status")[1]
        self.assertFalse(status["auto"]["on"])
        self.assertNotIn("scan", status)
        self.assertEqual(status["config"]["control"], "ssh")

    def test_bad_run_settings_are_refused(self):
        self.assertEqual(self.call("/api/auto/start", {"speed": 0.5})[0], 400)
        self.assertFalse(self.app.auto.on)

    def test_a_run_start_goes_over_ssh_with_this_pcs_address(self):
        self.assertEqual(self.call("/api/run/start", {})[0], 200)
        settings, pc, port = self.ssh.started[0]
        self.assertEqual((settings, pc, port), (DRY, "127.0.0.1", 5610))  # the way to 127.0.0.1 is the loopback

    def test_say_goal_and_point(self):
        self.assertEqual(self.call("/api/say", {"word": "follow"})[0], 200)
        self.assertEqual(self.call("/api/say", {"word": "dance"})[0], 400)
        self.assertEqual(self.call("/api/goal", {"x": 2.0, "y": 0.0})[0], 409)  # nothing mapped yet
        code, body = self.call("/api/point", {"x": 1.5, "y": -0.5, "seconds": 30})
        self.assertEqual((code, body["pointing"]), (200, [1.5, -0.5]))
        self.assertEqual(self.app.person.state(self.app.clock())["pointing"], [1.5, -0.5])  # in the datagram
        self.assertEqual(self.call("/api/point", {"clear": True})[1]["pointing"], None)

    def test_point_at_the_bag_the_robot_reports(self):
        self.assertEqual(self.call("/api/point", {"bag": True})[0], 409)  # no bag in the record yet
        self.app.robot.record = {"pose": [0.0, 0.0, 0.0], "bag": [2.0, 1.0]}
        self.assertEqual(self.call("/api/point", {"bag": True})[1]["pointing"], [2.0, 1.0])

    def test_virtual_bags_are_placed_sent_and_cleared(self):
        self.assertEqual(self.call("/api/bags", {"x": 2.0, "y": 0.8})[1]["bags"], [[2.0, 0.8]])
        self.assertEqual(self.call("/api/bags", {"x": 2.0, "y": -0.2})[0], 200)
        self.assertEqual(self.app.datagram(self.app.clock())["bags"], [[2.0, 0.8], [2.0, -0.2]])  # to the robot
        self.assertEqual(self.call("/api/view")[1]["bags"], [[2.0, 0.8], [2.0, -0.2]])
        self.assertEqual(self.call("/api/bags", {"x": 1.0, "y": 0.0})[0], 200)
        self.assertEqual(self.call("/api/bags", {"x": 3.0, "y": 0.0})[0], 409)  # sim.MAX_BAGS
        for bad in ({"x": "a", "y": 0.0}, {"x": 1.0}, {"x": True, "y": 0.0}):
            self.assertEqual(self.call("/api/bags", bad)[0], 400, bad)
        self.assertEqual(self.call("/api/bags", {"clear": True})[1]["bags"], [])
        self.assertEqual(self.app.datagram(self.app.clock())["bags"], [])

    def test_the_bags_stay_where_they_lie_when_the_odometry_starts_anew(self):
        self.app.bags = [[2.0, 1.0]]
        self.app._check_frame([0.0, 0.0, 0.0])
        self.app._check_frame([5.0, 5.0, math.pi / 2])  # a new run: the same robot, a new frame
        self.assertEqual([round(v, 6) for v in self.app.bags[0]], [4.0, 7.0])  # still 2 m ahead, 1 m left of it

    def test_the_robots_bag_task_is_on_the_view(self):
        task = {"state": "reached", "result": {"ok": True, "code": "OK", "x": 0.31, "y": 0.015}}
        self.app.robot.record = {"pose": [0.0, 0.0, 0.0], "bag": [2.0, 1.0], "bag_task": task}
        robot = self.call("/api/view")[1]["robot"]
        self.assertEqual((robot["bag"], robot["bag_task"]), ([2.0, 1.0], task))

    def test_no_post_without_the_apps_header_or_from_another_host_name(self):
        self.assertEqual(self.call("/api/auto/start", {}, header=False)[0], 403)
        self.assertEqual(self.call("/api/auto/status", host="evil.example:%d" % self.port)[0], 403)
        self.assertFalse(self.app.auto.on)


if __name__ == "__main__":
    unittest.main()
