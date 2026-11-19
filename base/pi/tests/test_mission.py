"""The console executive: start, GET /api/state, arm refusal, STOP without stop-all."""
import json
import tempfile
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
import unittest

import console
from test_console import FakeJobs, P
from test_console_control import FakeAgent, FakeRun


class MissionTest(unittest.TestCase):
    def setUp(self):
        self.tmp_obj = tempfile.TemporaryDirectory()
        self.tmp = Path(self.tmp_obj.name)
        self.jobs = FakeJobs()
        self.agent = FakeAgent()
        self.acts = []
        jetson = console.Jetson(hosts=("10.0.0.2",), fetch=self.agent)
        jetson.act = lambda action, name=None: self.acts.append((action, name)) or (200, {})
        self.run = FakeRun()
        self.console = console.Console(P, console.Devices(P), self.jobs, jetson, fetch=self.run,
                                       pin="2468", log_root=self.tmp)
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), console.handler_for(self.console))
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.base = "http://127.0.0.1:%d" % self.server.server_address[1]

    def tearDown(self):
        self.console._mission_stop.set()
        self.server.shutdown()
        self.server.server_close()
        self.tmp_obj.cleanup()

    def request(self, path, body=None, pin=None):
        headers = {"Content-Type": "application/json", "X-HMC-Console": "1"}
        if pin:
            headers["X-Console-Pin"] = pin
        req = urllib.request.Request(self.base + path, None if body is None else json.dumps(body).encode(), headers)
        try:
            with urllib.request.urlopen(req, timeout=5) as response:
                return response.status, response.read()
        except urllib.error.HTTPError as error:
            return error.code, error.read()

    def start(self):
        code, raw = self.request("/api/task/start", {"drive": False, "speed": 0.08, "minutes": 5}, pin="2468")
        self.assertEqual(code, 200, raw)
        self.jobs.running = {"kind": "task", "label": "Drive task"}

    def mission(self):
        code, raw = self.request("/api/state")
        self.assertEqual(code, 200)
        return json.loads(raw)["mission"]

    def test_start_resets_the_executive_and_state_shows_the_step(self):
        idle = self.mission()
        self.assertEqual(idle["state"], "IDLE")
        self.assertFalse(idle["active"])
        self.assertEqual(idle["step_index"], 0)
        self.assertEqual(idle["step_total"], 11)
        self.console.jetson.host = "10.0.0.2"
        parked = []
        self.console.jetson.fetch = lambda url, body=None, timeout=2.0: parked.append((url, body, timeout)) or (200, {})
        self.start()
        snap = self.mission()
        self.assertTrue(snap["active"])
        self.assertEqual(snap["state"], "READY")
        self.assertEqual(snap["step_index"], 2)
        self.assertEqual(snap["step_total"], 11)
        self.assertEqual(snap["limit_s"], 300)
        self.assertGreater(snap["remaining_s"], 290)
        self.assertIsNone(snap["reason"])
        self.assertIsNone(snap["outcome"])
        self.assertIn("follow me", snap["say"])
        log = Path(snap["log"])
        self.assertTrue(log.is_file())
        line = json.loads(log.read_text(encoding="utf-8").splitlines()[0])
        self.assertIn("+07:00", line["ts"])
        self.assertEqual(line["src"], "hmc.flow")
        self.assertEqual(parked, [("http://10.0.0.2:8090/stop", {"name": "flow"}, 0.4)])
        self.assertEqual(self.acts, [])

    def test_speech_drives_follow_and_stop_records_operator_stop(self):
        self.start()
        code, raw = self.request("/api/mission/hear", {"text": "follow me"}, pin="2468")
        self.assertEqual(code, 200, raw)
        body = json.loads(raw)
        self.assertEqual(body["mission"]["state"], "ENROLLING")
        self.assertIn("follow", self.run.commands)
        code, raw = self.request("/api/stop", {})
        self.assertEqual(code, 200)
        snap = self.mission()
        self.assertEqual(snap["state"], "ABORTED")
        self.assertEqual(snap["outcome"], "ABORTED")
        self.assertEqual(snap["reason"]["code"], "OPERATOR_STOP")
        self.assertTrue(snap["reason"]["th"])
        self.assertTrue(snap["reason"]["fix"])
        self.assertFalse(snap["active"])
        self.assertEqual(self.jobs.stopped, 1)
        self.assertEqual(self.acts, [])
        self.assertNotIn("stop-all", self.run.commands)
        self.jobs.running = None
        code, raw = self.request("/api/arm", {"cmd": "home"}, pin="2468")
        self.assertEqual(code, 200, raw)

    def test_arm_buttons_are_refused_during_the_run(self):
        self.start()
        for cmd in ({"cmd": "home"}, {"cmd": "pick_bag", "x": 0.3, "y": 0.0}, {"cmd": "bagpick"}):
            code, raw = self.request("/api/arm", cmd, pin="2468")
            body = json.loads(raw)
            self.assertEqual(code, 409, cmd)
            self.assertIn("task", body["error"])
        self.assertEqual(self.agent.posts, [])

    def test_a_down_link_does_not_follow_until_it_is_back(self):
        self.start()
        code, raw = self.request("/api/mission/event", {"event": "base_link_lost"}, pin="2468")
        self.assertEqual(code, 200, raw)
        for _ in range(3):
            code, raw = self.request("/api/mission/hear", {"text": "follow me"}, pin="2468")
            self.assertEqual(code, 200, raw)
            body = json.loads(raw)
            self.assertEqual(body["mission"]["state"], "READY")
            self.assertEqual(body["mission"]["reason"]["code"], "BASE_LINK_DOWN")
            self.assertEqual(body["mission"]["reason"]["fix"], "Start the Pi base program.")
        self.assertNotIn("follow", self.run.commands)
        code, raw = self.request("/api/mission/event", {"event": "base_link_ok"}, pin="2468")
        self.assertEqual(code, 200, raw)
        code, raw = self.request("/api/mission/hear", {"text": "follow me"}, pin="2468")
        body = json.loads(raw)
        self.assertEqual(body["mission"]["state"], "ENROLLING")
        self.assertIn("follow", self.run.commands)
