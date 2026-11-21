"""The console executive: start, GET /api/state, arm refusal, STOP without stop-all."""
import json
import tempfile
import threading
import time
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
        self.assertEqual(parked, [("http://10.0.0.2:8090/flow", {"cmd": "own", "ttl": console.FLOW_LEASE},
                                   console.FLOW_OWN_TIMEOUT), ("http://10.0.0.2:8090/stop", {"name": "flow"}, 0.4)])
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


class FlowAgent:
    """agent.py's POST /flow (the parked Jetson flow: own, heard, say, release), /stop and /arm, as console.Jetson's
    fetch sees them. Every call is kept as (path, body, timeout)."""

    def __init__(self):
        self.calls, self.lines, self.seq = [], [], 0
        self.flow_code = 200  # 404: an older agent without /flow
        self.fail = False  # heard and say do not answer in time
        self.arm_answer = (200, {"ok": True, "error": None, "code": "SUCCESS", "message": "holding"})
        self.arm_error = None  # raised by a POST /arm instead of its answer
        self.gate = threading.Event()  # a POST /arm answers once it is set
        self.gate.set()

    def hear(self, text):
        self.seq += 1
        self.lines.append({"seq": self.seq, "text": text, "t": time.time()})

    def __call__(self, url, body=None, timeout=2.0):
        path = url.split(":8090", 1)[1]
        self.calls.append((path, body, timeout))
        if path == "/arm":
            if body is None:
                return 200, {"busy": False, "state": "resting", "error": None}
            self.gate.wait(10)
            if self.arm_error is not None:
                raise self.arm_error
            return self.arm_answer
        if path != "/flow":
            return 200, {"error": None}
        if self.flow_code != 200:
            return self.flow_code, {"error": "not found"}
        if body["cmd"] == "own":
            return 200, {"ok": True, "parked": True, "seq": self.seq}
        if body["cmd"] == "release":
            return 200, {"ok": True, "parked": False}
        if self.fail:
            raise OSError("timed out")
        if body["cmd"] == "heard":
            items = [line for line in self.lines if line["seq"] > body["after"]][:20]
            return 200, {"ok": True, "parked": True, "seq": self.seq, "items": items}
        return 200, {"ok": True}

    def sent(self, cmd):
        return [body for path, body, _ in self.calls if path == "/flow" and body["cmd"] == cmd]

    def says(self):
        return [body["text"] for body in self.sent("say")]


class RelayTest(unittest.TestCase):
    """The console's half of the Jetson flow relay: speech in, say out, the lease; the arm's answer and the bag task's
    progress fed back to the executive."""

    setUp_console = MissionTest.setUp
    tearDown = MissionTest.tearDown
    request, start, mission = MissionTest.request, MissionTest.start, MissionTest.mission

    def setUp(self):
        self.setUp_console()
        self.flow = FlowAgent()
        self.console.jetson.fetch = self.flow
        self.console.jetson.host = "10.0.0.2"
        self.events = []
        on_event = self.console.mission.on_event
        self.console.mission.on_event = lambda ev, now: self.events.append(ev) or on_event(ev, now)

    def wait_until(self, test, why, timeout=4.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = test()
            if value:
                return value
            time.sleep(0.02)
        self.fail("not within %.0f s: %s" % (timeout, why))

    def state(self):
        return self.console.mission.state

    def logged(self, name):
        """How many lines of the run's events.jsonl note an ignored event of this name."""
        lines = [json.loads(line) for line in
                 Path(self.console.mission.log_path).read_text(encoding="utf-8").splitlines()]
        return sum(1 for line in lines if line["event"] == "ignored" and line["detail"] == name)

    def to_pointing(self):
        self.start()
        for path, key, value in (("hear", "text", "follow me"), ("event", "event", "locked"), ("hear", "text", "stop"),
                                 ("hear", "text", "yes"), ("hear", "text", "ready")):
            code, raw = self.request("/api/mission/" + path, {key: value}, pin="2468")
            self.assertEqual(code, 200, raw)
        self.assertEqual(self.state(), "POINTING")
        self.assertEqual(self.run.commands[-1], "bag")

    def reach_bag(self, bag_id=12.5):
        self.run.record = {"phase": "follow", "bag_task": {
            "id": bag_id, "seq": 1, "mode": "point", "state": "reached",
            "chosen": {"x": 0.9, "y": 0.1, "side": "left", "say": "I see the bag on the left."},
            "result": {"ok": True, "code": "OK", "x": 0.31, "y": 0.02, "top": 0.2}}}

    def arm_posts(self):
        return [(body, timeout) for path, body, timeout in self.flow.calls if path == "/arm" and body is not None]

    def test_start_owns_the_flow_and_its_lines_go_out_in_order(self):
        self.start()
        self.assertEqual(self.flow.calls[0], ("/flow", {"cmd": "own", "ttl": console.FLOW_LEASE},
                                              console.FLOW_OWN_TIMEOUT))
        self.wait_until(lambda: len(self.flow.sent("heard")) >= 2, "two reads of the speech")
        self.assertEqual(self.flow.says(), ["Checking the base before we start.", "Say follow me when you are ready."])
        self.assertEqual(self.flow.sent("heard")[0], {"cmd": "heard", "after": 0, "ttl": console.FLOW_LEASE})
        self.assertNotIn("/stop", [path for path, _, _ in self.flow.calls])

    def test_an_agent_without_flow_is_stopped_as_before_and_the_log_says_so(self):
        self.flow.flow_code = 404
        self.start()
        self.assertEqual([(path, body) for path, body, _ in self.flow.calls],
                         [("/flow", {"cmd": "own", "ttl": console.FLOW_LEASE}), ("/stop", {"name": "flow"})])
        time.sleep(0.6)
        self.assertEqual(len(self.flow.calls), 2)
        self.assertEqual(self.logged("voice_relay_off"), 1)

    def test_a_follow_only_run_keeps_voice_out_as_before(self):
        code, raw = self.request("/api/task/start", {"drive": False, "speed": 0.08, "minutes": 5,
                                                     "mission": "follow_only"}, pin="2468")
        self.assertEqual(code, 200, raw)
        self.assertEqual([(path, body) for path, body, _ in self.flow.calls], [("/stop", {"name": "flow"})])
        time.sleep(0.6)
        self.assertEqual(len(self.flow.calls), 1)
        self.assertEqual(self.logged("voice_relay_off"), 1)

    def test_heard_follow_me_moves_the_executive_on(self):
        self.start()
        self.flow.hear("follow me")
        self.wait_until(lambda: self.state() == "ENROLLING", "ENROLLING")
        self.assertIn("follow", self.run.commands)
        self.wait_until(lambda: "I will follow you. Please stand in front of me." in self.flow.says(), "its line said")
        self.wait_until(lambda: self.flow.sent("heard")[-1]["after"] == 1, "the next read after seq 1")

    def test_a_failed_poll_does_not_end_the_run(self):
        self.start()
        self.wait_until(lambda: self.flow.sent("heard"), "a first read")
        self.flow.fail = True
        failed = len(self.flow.sent("heard"))
        self.wait_until(lambda: len(self.flow.sent("heard")) >= failed + 3, "three failed reads")
        self.assertTrue(self.mission()["active"])
        self.assertEqual(self.state(), "READY")
        self.flow.fail = False
        self.flow.hear("follow me")
        self.wait_until(lambda: self.state() == "ENROLLING", "ENROLLING after the relay is back")
        self.assertEqual(self.logged("voice_relay_lost"), 1)
        self.assertEqual(self.logged("voice_relay_ok"), 1)

    def test_stop_releases_the_flow_once(self):
        self.start()
        self.wait_until(lambda: self.flow.sent("heard"), "a first read")
        code, _ = self.request("/api/stop", {})
        self.assertEqual(code, 200)
        self.wait_until(lambda: self.flow.sent("release"), "release")
        reads = len(self.flow.sent("heard"))
        time.sleep(0.6)
        self.assertEqual(len(self.flow.sent("release")), 1)
        self.assertEqual(len(self.flow.sent("heard")), reads)
        self.assertEqual(self.flow.says()[-1], "STOP was pressed. Motion has stopped.")

    def test_the_end_of_the_run_job_releases_the_flow(self):
        self.start()
        self.wait_until(lambda: self.flow.sent("heard"), "a first read")
        self.jobs.running = None
        self.wait_until(lambda: self.flow.sent("release"), "release")
        time.sleep(0.6)
        self.assertEqual(len(self.flow.sent("release")), 1)

    def test_a_reached_bag_is_picked_brought_back_and_dropped(self):
        self.to_pointing()
        self.reach_bag()
        self.wait_until(lambda: self.state() == "RETURNING", "RETURNING after grab_success")
        self.assertEqual(self.arm_posts(), [({"cmd": "pick_bag", "x": 0.31, "y": 0.02, "top": 0.2},
                                             console.ARM_JOB_TIMEOUT)])
        self.assertTrue(self.console.mission.carrying)
        self.assertEqual(self.run.commands[-1], "return")
        time.sleep(1.2)  # more status reads of the same bag task
        self.assertEqual([e for e in self.events if e.startswith("bag_")],
                         ["bag_chosen|LEFT|I see the bag on the left.", "bag_reached|OK|0.310,0.020,top=0.200"])
        self.assertEqual(self.flow.says().count("I see the bag on the left."), 1)
        self.console._drop = {"x": 0.38, "y": 0.0}  # so100_ik.reachable_drop
        # as the real run_follow: at_start and its end in the same record, the base stopped (DONE)
        self.run.record = dict(self.run.record, phase="done", at_start=True, end="done: back at the start")
        self.wait_until(lambda: self.state() == "DONE", "DONE after drop_done")
        self.assertEqual(self.arm_posts()[1], ({"cmd": "drop_bag", "x": 0.38, "y": 0.0}, console.ARM_JOB_TIMEOUT))
        self.assertIn("drop_done", self.events)
        self.assertEqual(self.mission()["outcome"], "SUCCESS")
        self.wait_until(lambda: self.flow.sent("release"), "release at DONE")

    def test_a_run_that_ended_at_the_start_still_drops_and_says_so(self):
        # run_follow ends in the cycle it arrives and its page goes with it: the console reads that end in its log
        self.console._drop = {"x": 0.38, "y": 0.0}
        self.to_pointing()
        self.reach_bag()
        self.wait_until(lambda: self.state() == "RETURNING", "RETURNING after grab_success")
        self.run.record = dict(self.run.record, phase="return")
        log = Path(self.console.mission.log_path).parent / "log.jsonl"
        log.write_text(json.dumps({"time": 1.0, "phase": "done", "at_start": True,
                                   "end": "done: back at the start"}) + "\n", encoding="utf-8")
        self.jobs.running = None
        self.wait_until(lambda: self.state() == "DONE", "DONE after drop_done")
        self.assertEqual(self.arm_posts()[1], ({"cmd": "drop_bag", "x": 0.38, "y": 0.0}, console.ARM_JOB_TIMEOUT))
        self.assertEqual(self.mission()["outcome"], "SUCCESS")
        self.wait_until(lambda: "Here is your bag." in self.flow.says(), "the last line said")
        self.assertIn("I am back at the start.", self.flow.says())
        self.wait_until(lambda: self.flow.sent("release"), "release at DONE")
        time.sleep(0.6)
        self.assertEqual(len(self.flow.sent("release")), 1)

    def test_a_missed_pick_measures_the_bag_again(self):
        self.flow.arm_answer = (409, {"ok": False, "error": "MISSED: closed on air", "code": "MISSED",
                                      "message": "closed on air"})
        self.to_pointing()
        self.reach_bag()
        self.wait_until(lambda: "bag_again" in self.run.commands, "bag_again after the miss")
        self.assertIn("grab_failed|MISSED|closed on air", self.events)
        self.assertEqual(self.state(), "POINTING")
        self.assertEqual(self.mission()["reason"]["code"], "PICK_MISSED")
        self.reach_bag(bag_id=40.0)  # the second bag task: reported too
        self.wait_until(lambda: len(self.arm_posts()) == 2, "the second pick")

    def test_an_unreachable_bag_and_a_lost_agent_are_handed_over(self):
        for answer, error, reason in (((409, {"ok": False, "code": "UNREACHABLE", "message": "too far"}), None,
                                       "PICK_UNREACHABLE"), (None, OSError("timed out"), "PICK_TIMEOUT")):
            with self.subTest(reason):
                self.flow.arm_answer, self.flow.arm_error = answer, error
                self.to_pointing()
                self.reach_bag()
                self.wait_until(lambda: self.state() == "HANDOVER", "HANDOVER")
                self.assertEqual(self.mission()["reason"]["code"], reason)
                self.request("/api/stop", {})
                self.jobs.running = None
                self.run.record = None

    def test_the_mission_loop_ticks_while_a_slow_pick_runs(self):
        self.flow.gate.clear()
        self.to_pointing()
        ticks = []
        on_tick = self.console.mission.on_tick
        self.console.mission.on_tick = lambda now: ticks.append(now) or on_tick(now)
        self.reach_bag()
        self.wait_until(self.arm_posts, "the pick sent")
        ticks.clear()
        time.sleep(1.3)
        self.assertGreaterEqual(len(ticks), 2)
        self.assertEqual(self.state(), "PICKING")
        self.flow.gate.set()
        self.wait_until(lambda: self.state() == "RETURNING", "RETURNING once the arm answered")
