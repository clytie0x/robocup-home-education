import json
import os
import shlex
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import agent  # noqa: E402

SLEEPER = [sys.executable, "-c", "import time; print('hello from the fake', flush=True); time.sleep(30)"]


def shell(args):
    """args as one shell command line for Program (bash -c on Linux, cmd.exe on Windows)."""
    return subprocess.list2cmdline(args) if os.name != "posix" else shlex.join(args)


def wait_for(check, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        time.sleep(0.05)
    return check()


class AgentTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.logs = Path(self.tmp.name)
        fakes = {name: (shell(SLEEPER), False, None) for name in ("detector", "bridge", "flow")}
        # find_outside: never the real process list (a real Jetson may run these programs by hand)
        # hold: never a datagram to a live bridge on this machine
        self.agent = agent.Agent(fakes, logs=self.logs, order=["detector", "bridge", "flow"],
                                 find_outside=lambda ours: {}, hold=lambda: None)
        self.outside = None

    def tearDown(self):
        self.agent.stop_all()
        if self.outside is not None and self.outside.poll() is None:
            self.outside.kill()
            self.outside.wait()
        self.tmp.cleanup()

    def test_start_status_stop(self):
        code, body = self.agent.act("start", "flow")
        self.assertEqual((code, body["error"]), (200, None))
        flow = body["status"]["programs"]["flow"]
        self.assertTrue(flow["running"])
        self.assertIsInstance(flow["pid"], int)
        self.assertFalse(body["status"]["programs"]["bridge"]["running"])
        self.assertTrue(wait_for(lambda: any("hello from the fake" in line
                                             for line in self.agent.status()["programs"]["flow"]["log"])))
        code, body = self.agent.act("start", "flow")
        self.assertEqual(code, 409)
        self.assertIn("already running", body["error"])
        code, body = self.agent.act("stop", "flow")
        self.assertEqual((code, body["error"]), (200, None))
        self.assertFalse(body["status"]["programs"]["flow"]["running"])
        self.assertEqual(self.agent.act("stop", "flow")[0], 409)

    def test_restart_gives_a_new_process(self):
        self.agent.act("start", "bridge")
        first = self.agent.programs["bridge"].proc.pid
        code, body = self.agent.act("restart", "bridge")
        self.assertEqual(code, 200)
        self.assertTrue(body["status"]["programs"]["bridge"]["running"])
        self.assertNotEqual(body["status"]["programs"]["bridge"]["pid"], first)

    def test_unknown_name_and_action(self):
        self.assertEqual(self.agent.act("start", "rm -rf")[0], 400)
        self.assertEqual(self.agent.act("reboot")[0], 404)

    def test_start_all_in_order(self):
        saved = agent.SETTLE
        agent.SETTLE = 0.2
        try:
            self.assertEqual(self.agent.act("start-all")[0], 200)
            self.assertTrue(wait_for(lambda: self.agent.sequence == "all started", 10))
        finally:
            agent.SETTLE = saved
        started = [self.agent.programs[n].started for n in ("detector", "bridge", "flow")]
        self.assertEqual(started, sorted(started))
        self.assertTrue(all(self.agent.programs[n].running() for n in ("detector", "bridge", "flow")))
        self.agent.act("stop-all")
        self.assertFalse(any(self.agent.programs[n].running() for n in ("detector", "bridge", "flow")))

    def test_never_touches_a_process_it_did_not_start(self):
        self.outside = subprocess.Popen(SLEEPER, stdout=subprocess.DEVNULL)
        self.agent.act("start", "detector")
        self.agent.stop_all()
        self.assertEqual(self.agent.act("stop", "flow")[0], 409)  # not started by us: nothing to stop
        time.sleep(0.3)
        self.assertIsNone(self.outside.poll())

    def test_start_refused_while_a_copy_started_by_hand_runs(self):
        a = agent.Agent({"bridge": (shell(SLEEPER), False, None)}, logs=self.logs,
                        find_outside=lambda ours: {"bridge": [4321]})
        self.addCleanup(a.stop_all)
        for action in ("start", "restart"):
            code, body = a.act(action, "bridge")
            self.assertEqual(code, 409, action)
            self.assertIn("4321", body["error"])
            self.assertIn("started by hand", body["error"])
            self.assertIn("ด้วยมือ", body["error"])
            self.assertFalse(a.programs["bridge"].running())
        saved = agent.SETTLE
        agent.SETTLE = 0.0
        try:
            self.assertEqual(a.act("start-all")[0], 200)
            self.assertTrue(wait_for(lambda: a.sequence is not None and "4321" in a.sequence))
        finally:
            agent.SETTLE = saved
        self.assertFalse(a.programs["bridge"].running())

    def test_outside_leaves_out_the_children_of_our_own_groups(self):
        proc = Path(self.tmp.name) / "proc"
        for pid, pgrp, script in ((100, 50, "base_bridge.py"), (200, 200, "base_bridge.py"), (300, 300, "other.py")):
            d = proc / str(pid)
            d.mkdir(parents=True)
            (d / "cmdline").write_bytes(b"python3\0" + script.encode() + b"\0--pi-host\0x\0")
            (d / "stat").write_text("%d (python3) S 1 %d %d 0 -1 4194560 0 0" % (pid, pgrp, pgrp))
        self.assertEqual(agent.outside({50}, proc), {"bridge": [200]})

    def test_outside_finds_any_python_and_the_script_anywhere_in_argv(self):
        proc = Path(self.tmp.name) / "proc"
        cases = {101: [b"python3", b"-u", b"base_bridge.py"], 102: [b"/usr/bin/python3.12", b"base_bridge.py"],
                 103: [b"python3", b"/home/u/hmc-follow/hmc_flow.py", b"--x"], 104: [b"python3", b"-c", b"print('base_bridge.py')"],
                 105: [b"bash", b"base_bridge.py"], 106: [b"python", b"-X", b"dev", b"server.py"]}
        for pid, args in cases.items():
            d = proc / str(pid)
            d.mkdir(parents=True)
            (d / "cmdline").write_bytes(b"\0".join(args) + b"\0")
            (d / "stat").write_text("%d (x) S 1 %d %d 0" % (pid, pid, pid))
        found = agent.outside({106}, proc)  # 106 leads one of our groups
        self.assertEqual(found, {"bridge": [101, 102], "flow": [103]})

    def holding_agent(self):
        holds = []
        a = self.agent
        a.hold = lambda: holds.append({n: a.programs[n].running() for n in a.order})
        return a, holds

    def test_stopping_the_flow_or_the_bridge_asks_the_bridge_to_hold_first(self):
        a, holds = self.holding_agent()
        a.act("start", "bridge")
        a.act("start", "flow")
        code, body = a.act("stop", "flow")
        self.assertEqual(code, 200)
        self.assertEqual(holds, [{"detector": False, "bridge": True, "flow": True}])  # before the stop
        self.assertIn("hold", body["hold"])
        a.act("stop", "detector")
        self.assertEqual(len(holds), 1)  # the detector does not drive the robot
        a.act("restart", "bridge")
        self.assertEqual(len(holds), 2)
        a.act("start", "flow")
        a.act("stop-all")
        self.assertEqual(holds[-1], {"detector": False, "bridge": True, "flow": True})

    def test_no_hold_without_a_running_bridge(self):
        a, holds = self.holding_agent()
        a.act("start", "flow")
        code, body = a.act("stop", "flow")
        self.assertEqual((code, holds), (200, []))
        self.assertIsNone(body["hold"])

    def test_send_hold_is_one_datagram_to_the_bridge(self):
        import socket
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind(("127.0.0.1", 0))
            s.settimeout(2.0)
            agent.send_hold(s.getsockname()[1], settle=0.0)
            self.assertEqual(s.recv(64), b"hold")
        agent.send_hold(1, settle=0.0)  # nobody listens: no error

    def fake_group(self, dies_after):
        """A group whose leader (bash) ends at the first signal while a child lives through dies_after signals."""
        group = {"alive": True, "sent": []}

        class Proc:
            pid = 777
            def poll(self):
                return 0 if group["sent"] else None

        def killpg(pgid, sig):
            self.assertEqual(pgid, 777)
            if sig == 0:
                if not group["alive"]:
                    raise ProcessLookupError()
                return
            group["sent"].append(sig)
            if len(group["sent"]) == dies_after:
                group["alive"] = False
        now = [0.0]

        def sleep(s):
            now[0] += s
        p = agent.Program("bridge", "x", self.logs, popen=lambda *a, **k: Proc(), clock=lambda: now[0],
                          sleep=sleep, killpg=killpg)
        return p, group

    def test_stop_kills_the_group_when_a_child_ignores_ctrl_c_and_sigterm(self):
        p, group = self.fake_group(dies_after=3)
        self.assertIsNone(p.start())
        self.assertIsNone(p.stop())
        self.assertEqual(group["sent"], [agent.signal.SIGINT, agent.signal.SIGTERM, agent.KILL])
        self.assertFalse(group["alive"])

    def test_stop_ends_early_when_the_whole_group_is_gone(self):
        p, group = self.fake_group(dies_after=1)
        p.start()
        self.assertIsNone(p.stop())
        self.assertEqual(group["sent"], [agent.signal.SIGINT])

    def test_stop_says_so_when_the_group_outlives_sigkill(self):
        p, group = self.fake_group(dies_after=None)
        p.start()
        self.assertIn("still", p.stop())

    def test_start_refused_while_the_last_group_still_has_a_process(self):
        p, group = self.fake_group(dies_after=3)
        p.start()
        group["sent"].append("bash ended by itself")  # the leader is gone, its child is not
        self.assertIn("already running", p.start())

    def fake_root(self):
        """A Jetson-like /sys and /proc: two readable zones, cv0 without data, three INA3221 rails."""
        root = Path(self.tmp.name) / "root"
        zones = {"0": ("cpu-thermal", "48500\n"), "1": ("gpu-thermal", "51250\n"), "2": ("cv0-thermal", None),
                 "10": ("cpu-thermal", "47000\n")}
        for n, (kind, temp) in zones.items():
            z = root / ("sys/class/thermal/thermal_zone%s" % n)
            z.mkdir(parents=True)
            (z / "type").write_text(kind + "\n")
            if temp is not None:
                (z / "temp").write_text(temp)
            else:
                (z / "temp").mkdir()  # reading it fails, as "No data available" does
        hw = root / "sys/bus/i2c/drivers/ina3221/1-0040/hwmon/hwmon3"
        hw.mkdir(parents=True)
        for n, label, mv, ma in ((1, "VDD_IN", 5080, 1460), (2, "VDD_CPU_GPU_CV", 5072, 412), (3, "VDD_SOC", 5064, 668),
                                 (7, "sum of shunt voltages", 120, None)):
            (hw / ("in%d_label" % n)).write_text(label + "\n")
            (hw / ("in%d_input" % n)).write_text("%d\n" % mv)
            if ma is not None:
                (hw / ("curr%d_input" % n)).write_text("%d\n" % ma)
        (root / "sys/devices/platform/gpu.0").mkdir(parents=True)
        (root / "sys/devices/platform/gpu.0/load").write_text("373\n")
        (root / "proc").mkdir()
        (root / "proc/meminfo").write_text("MemTotal:  7800000 kB\nMemFree: 100 kB\nMemAvailable: 5752000 kB\n")
        (root / "proc/loadavg").write_text("0.50 0.40 0.30 1/200 999\n")
        (root / "proc/uptime").write_text("3600.4 100.0\n")
        (root / "proc/stat").write_text("cpu  100 0 100 700 100 0 0 0 0 0\ncpu0 1 2 3 4\n")
        return root

    def test_metrics_shape_for_telemetry(self):
        root = self.fake_root()
        m = agent.Metrics(root, disk=self.tmp.name)
        first = m.read()
        self.assertEqual(first["thermal"], {"cpu-thermal": 48.5, "gpu-thermal": 51.2, "cpu-thermal-10": 47.0})
        self.assertEqual(first["power"], {"VDD_IN": {"mV": 5080.0, "mA": 1460.0, "mW": 7416.8},
                                          "VDD_CPU_GPU_CV": {"mV": 5072.0, "mA": 412.0, "mW": 2089.7},
                                          "VDD_SOC": {"mV": 5064.0, "mA": 668.0, "mW": 3382.8}})
        self.assertEqual(first["gpu_load_pct"], 37.3)
        self.assertIsNone(first["cpu_pct"])  # needs a second reading
        self.assertEqual((first["load1"], first["mem_total_mb"], first["mem_used_mb"], first["uptime_s"]),
                         (0.5, 7617.2, 2000.0, 3600))
        self.assertIsInstance(first["disk_free_gb"], float)
        (root / "proc/stat").write_text("cpu  250 0 150 800 100 0 0 0 0 0\n")  # +200 busy, +100 idle
        self.assertAlmostEqual(m.read()["cpu_pct"], 66.7)

        def numbers_only(value):
            if isinstance(value, dict):
                return all(numbers_only(v) for v in value.values())
            return value is None or (isinstance(value, (int, float)) and not isinstance(value, bool))
        self.assertTrue(numbers_only(first))

    def test_metrics_missing_sources_are_null(self):
        m = agent.Metrics(Path(self.tmp.name) / "missing", disk=self.tmp.name)
        r = m.read()
        self.assertEqual((r["thermal"], r["power"]), ({}, {}))
        for key in ("gpu_load_pct", "cpu_pct", "load1", "mem_used_mb", "mem_total_mb", "uptime_s"):
            self.assertIsNone(r[key], key)

    def test_status_carries_metrics_and_programs(self):
        self.agent.metrics = agent.Metrics(self.fake_root(), disk=self.tmp.name)
        s = self.agent.status()
        self.assertEqual(s["gpu_load_pct"], 37.3)
        self.assertEqual(set(s["programs"]), {"detector", "bridge", "flow"})
        self.assertIsInstance(s["time"], float)


class HttpTest(unittest.TestCase):
    def serve(self, allow):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        a = agent.Agent({"flow": (shell(SLEEPER), False, None)}, logs=Path(tmp.name), find_outside=lambda ours: {})
        self.addCleanup(a.stop_all)
        server = ThreadingHTTPServer(("127.0.0.1", 0), agent.handler_for(a, allow))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        return "http://127.0.0.1:%d" % server.server_address[1]

    def post(self, url, body):
        req = urllib.request.Request(url, json.dumps(body).encode(), {"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.load(r)
        except urllib.error.HTTPError as e:
            return e.code, json.load(e)

    def test_status_has_cors_and_control_is_allowed_from_localhost(self):
        base = self.serve(("127.0.0.1",))
        with urllib.request.urlopen(base + "/status", timeout=5) as r:
            self.assertEqual(r.headers["Access-Control-Allow-Origin"], "*")
            self.assertIn("flow", json.load(r)["programs"])
        code, body = self.post(base + "/start", {"name": "flow"})
        self.assertEqual(code, 200)
        self.assertTrue(body["status"]["programs"]["flow"]["running"])
        self.assertEqual(self.post(base + "/stop", {"name": "flow"})[0], 200)

    def test_control_refused_from_other_addresses(self):
        base = self.serve(("10.0.0.1",))
        code, body = self.post(base + "/start", {"name": "flow"})
        self.assertEqual(code, 403)
        self.assertIn("console", body["error"])


if __name__ == "__main__":
    unittest.main()
