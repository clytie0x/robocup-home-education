import json
import os
import shlex
import socket
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

    def test_outside_pids_are_sorted(self):
        """The Jetson's /proc lists 102 before 101 at times; the answer must not depend on that order."""
        proc = Path(self.tmp.name) / "proc"
        for pid in (99, 100, 102, 101):  # "100" < "99" by name, so a name-ordered listing is not numeric
            d = proc / str(pid)
            d.mkdir(parents=True)
            (d / "cmdline").write_bytes(b"python3\0base_bridge.py\0")
            (d / "stat").write_text("%d (x) S 1 %d %d 0" % (pid, pid, pid))
        self.assertEqual(agent.outside(set(), proc), {"bridge": [99, 100, 101, 102]})

    def test_outside_finds_the_whisper_server_the_voice_launch_and_the_arm_job(self):
        proc = Path(self.tmp.name) / "proc"
        cases = {201: [b"/opt/robot_voice/src/whisper.cpp/build/bin/whisper-server", b"-m", b"x.bin"],
                 202: [b"/usr/bin/python3", b"/opt/ros/jazzy/bin/ros2", b"launch", b"voice_assistant",
                       b"hmc_voice.launch.py"],
                 203: [b"python3", b"bag_arm.py"], 204: [b"bash", b"-c", b"whisper-server"],
                 205: [b"python3", b"whisper-server"]}
        for pid, args in cases.items():
            d = proc / str(pid)
            d.mkdir(parents=True)
            (d / "cmdline").write_bytes(b"\0".join(args) + b"\0")
            (d / "stat").write_text("%d (x) S 1 %d %d 0" % (pid, pid, pid))
        self.assertEqual(agent.outside(set(), proc), {"whisper": [201], "voice": [202], "arm": [203]})

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

    def test_every_program_gets_its_console_job_name(self):
        seen = []
        p = agent.Program("arm", "x", self.logs, popen=lambda *a, **k: seen.append(k["env"]) or object())
        self.assertIsNone(p.start())
        self.assertEqual(seen[0]["HMC_CONSOLE_JOB"], "agent:arm")
        self.assertEqual(seen[0].get("PATH"), os.environ.get("PATH"))  # the rest of the environment as before

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
        self.assertIn("version", s)  # the commit deploy.sh wrote next to agent.py, or None

    def test_deployed_version(self):
        path = os.path.join(self.tmp.name, "VERSION.json")
        self.assertIsNone(agent.deployed_version(path))
        with open(path, "w") as f:
            json.dump({"commit": "a" * 40, "describe": "aaaaaaa"}, f)
        self.assertEqual(agent.deployed_version(path)["describe"], "aaaaaaa")
        with open(path, "w") as f:
            f.write('{"commit": null}')
        self.assertIsNone(agent.deployed_version(path))


class TaskProgramsTest(unittest.TestCase):
    """The programs start-all runs for the whole task, in the order each needs the one before."""

    def test_start_all_order(self):
        self.assertEqual(agent.ORDER, ["detector", "whisper", "voice", "bridge", "armstack", "webcam", "arm", "flow"])
        self.assertIn("armstack", agent.SHARED)  # the arm stack started by hand before 2026-11-15 is used as it is
        self.assertIn("webcam", agent.SHARED)  # a hand-started webcam_view already holds 8082 and /dev/video0
        self.assertIn("arm.launch.py mount:=robot", agent.PROGRAMS["armstack"][0])
        self.assertEqual(list(agent.PROGRAMS), agent.ORDER)
        self.assertEqual(set(agent.SCRIPTS) | set(agent.EXECUTABLES), set(agent.ORDER))

    def test_webcam_runs_the_arm_teams_view_before_bag_arm(self):
        cmd, ros, port = agent.PROGRAMS["webcam"]
        self.assertEqual((ros, port), (False, 8082))  # bag_arm and the console read :8082/snap.jpg
        self.assertIn("webcam_view.py", cmd)
        self.assertLess(agent.ORDER.index("webcam"), agent.ORDER.index("arm"))

    def test_whisper_runs_the_server_of_servers_sh_in_the_foreground(self):
        cmd, ros, port = agent.PROGRAMS["whisper"]
        self.assertEqual((ros, port), (False, 8080))  # servers.sh: --host 127.0.0.1 --port 8080
        self.assertIn(agent.VOICE_ROOT + "/src/whisper.cpp/build/bin/whisper-server", cmd)
        self.assertIn("exec ", cmd)  # the server is the group leader: no "&", so the agent sees it run
        self.assertNotIn("&", cmd.replace("&&", ""))
        for part in ("-l en", "-t 4", "--host 127.0.0.1", "--port 8080", "ggml-small.en-q5_1.bin",
                     "ggml-base.en-q5_1.bin"):
            self.assertIn(part, cmd)

    def test_voice_is_the_help_me_carry_launch(self):
        cmd, ros, port = agent.PROGRAMS["voice"]
        self.assertTrue(ros)
        self.assertIsNone(port)
        self.assertIn(agent.VOICE_WS + "/install/setup.bash", cmd)
        self.assertTrue(cmd.endswith("ros2 launch voice_assistant hmc_voice.launch.py"))

    def test_arm_is_bag_arm_with_the_arm_workspace(self):
        cmd, ros, port = agent.PROGRAMS["arm"]
        self.assertTrue(ros)
        self.assertIn("so100_ws/install/setup.bash", cmd)
        self.assertTrue(cmd.endswith("python3 bag_arm.py"))

    def test_flow_uses_the_arm_with_the_measured_timeouts(self):
        cmd, ros, port = agent.PROGRAMS["flow"]
        self.assertTrue(cmd.endswith(
            "python3 hmc_flow.py --ros-args -p use_arm:=true -p grab_timeout:=240.0 -p drop_timeout:=60.0"))

    def test_start_all_uses_a_whisper_server_started_by_hand(self):
        """servers.sh start-stt (the voice docs) leaves one on port 8080: use it, never start a second."""
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        fakes = {name: (shell(SLEEPER), False, 8080 if name == "whisper" else None) for name in ("whisper", "flow")}
        a = agent.Agent(fakes, logs=Path(tmp.name), find_outside=lambda ours: {"whisper": [4321]},
                        hold=lambda: None, port_open=lambda port: True)
        self.addCleanup(a.stop_all)
        self.assertEqual(a.act("start", "whisper")[0], 409)  # a single start is still refused
        saved = agent.SETTLE
        agent.SETTLE = 0.0
        try:
            self.assertEqual(a.act("start-all")[0], 200)
            self.assertTrue(wait_for(lambda: a.sequence == "all started"))
        finally:
            agent.SETTLE = saved
        self.assertFalse(a.programs["whisper"].running())
        self.assertTrue(a.programs["flow"].running())


    def test_start_all_uses_an_arm_stack_started_by_hand(self):
        """The arm stack (no port of its own) started from a terminal before 2026-11-15: used, never doubled."""
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        fakes = {name: (shell(SLEEPER), False, None) for name in ("armstack", "flow")}
        a = agent.Agent(fakes, logs=Path(tmp.name), find_outside=lambda ours: {"armstack": [5555]},
                        hold=lambda: None, port_open=lambda port: True,
                        read_args=lambda pid: ["/usr/bin/python3", "/opt/ros/humble/bin/ros2", "launch",
                                               "so100_moveit_config", "arm.launch.py", "mount:=robot"])
        self.addCleanup(a.stop_all)
        saved = agent.SETTLE
        agent.SETTLE = 0.0
        try:
            self.assertEqual(a.act("start-all")[0], 200)
            self.assertTrue(wait_for(lambda: a.sequence == "all started"), a.sequence)
        finally:
            agent.SETTLE = saved
        self.assertFalse(a.programs["armstack"].running())
        self.assertTrue(a.programs["flow"].running())


    def test_start_all_refuses_a_hand_started_arm_stack_on_another_mount(self):
        """A copy launched with mount:=desk (or none) has the wrong floor for the robot: nothing after it starts."""
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        fakes = {name: (shell(SLEEPER), False, None) for name in ("armstack", "flow")}
        a = agent.Agent(fakes, logs=Path(tmp.name), find_outside=lambda ours: {"armstack": [5555]},
                        hold=lambda: None, port_open=lambda port: True,
                        read_args=lambda pid: ["python3", "ros2", "launch", "so100_moveit_config", "arm.launch.py",
                                               "mount:=desk"])
        self.addCleanup(a.stop_all)
        self.assertEqual(a.act("start-all")[0], 200)
        self.assertTrue(wait_for(lambda: "lacks mount:=robot" in a.sequence), a.sequence)
        self.assertIn("5555", a.sequence)
        self.assertFalse(a.programs["armstack"].running())
        self.assertFalse(a.programs["flow"].running())

    def test_cmdline_reads_proc(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        (Path(tmp.name) / "77").mkdir()
        (Path(tmp.name) / "77" / "cmdline").write_bytes(b"python3\0arm.launch.py\0mount:=robot\0")
        self.assertEqual(agent.cmdline(77, Path(tmp.name)), ["python3", "arm.launch.py", "mount:=robot"])
        self.assertEqual(agent.cmdline(78, Path(tmp.name)), [])


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
        self.assertEqual(self.post(base + "/arm", {"cmd": "home"})[0], 403)


class FakeBagArm:
    """bag_arm.py's command server: answers each JSON line with `answer(cmd)`, records what came."""

    def __init__(self, answer):
        self.got = []
        self.sock = socket.socket()
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(4)
        self.port = self.sock.getsockname()[1]
        self.answer = answer
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self):
        while True:
            try:
                conn, _ = self.sock.accept()
            except OSError:
                return
            with conn, conn.makefile("rwb") as f:
                cmd = json.loads(f.readline())
                self.got.append(cmd)
                f.write((json.dumps(self.answer(cmd)) + "\n").encode())
                f.flush()

    def close(self):
        self.sock.close()


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class ArmRelayTest(unittest.TestCase):
    def serve(self, arm_port):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        a = agent.Agent({}, logs=Path(tmp.name), find_outside=lambda ours: {}, arm_port=arm_port)
        server = ThreadingHTTPServer(("127.0.0.1", 0), agent.handler_for(a, ("127.0.0.1",)))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        return "http://127.0.0.1:%d" % server.server_address[1]

    def test_post_and_get_arm_go_to_bag_arm(self):
        fake = FakeBagArm(lambda cmd: {"ok": cmd["cmd"] != "center", "error": None if cmd["cmd"] != "center"
                                       else "a bag job is running", "busy": False})
        self.addCleanup(fake.close)
        base = self.serve(fake.port)
        code, body = HttpTest.post(None, base + "/arm", {"cmd": "torque", "on": True})
        self.assertEqual((code, body["ok"]), (200, True))
        code, body = HttpTest.post(None, base + "/arm", {"cmd": "center"})
        self.assertEqual((code, body["error"]), (409, "a bag job is running"))
        with urllib.request.urlopen(base + "/arm", timeout=5) as r:
            self.assertEqual(json.load(r)["busy"], False)
        self.assertEqual(fake.got, [{"cmd": "torque", "on": True}, {"cmd": "center"}, {"cmd": "state"}])

    def test_no_bag_arm_is_a_clear_error(self):
        base = self.serve(free_port())
        code, body = HttpTest.post(None, base + "/arm", {"cmd": "home"})
        self.assertEqual(code, 503)
        self.assertIn("bag_arm.py is not answering", body["error"])
        try:
            urllib.request.urlopen(base + "/arm", timeout=5)
            self.fail("GET /arm answered without bag_arm")
        except urllib.error.HTTPError as e:
            self.assertEqual(e.code, 503)

    def test_a_silent_bag_arm_times_out(self):
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            s.listen(1)
            code, body = agent.arm_command({"cmd": "center"}, s.getsockname()[1], timeout=0.3)
        self.assertEqual(code, 504)
        self.assertIn("did not answer", body["error"])
        self.assertEqual(agent.ARM_CMD_PORT, 5607)
        self.assertGreaterEqual(agent.ARM_TIMEOUT, 85.0)

    def test_a_bag_job_gets_its_own_long_timeout(self):
        from unittest import mock
        self.assertGreaterEqual(agent.ARM_JOB_TIMEOUT, 60.0 + 150.0)  # bag_arm's HOME_TIMEOUT + PICK_TIMEOUT
        with socket.socket() as s, mock.patch.object(agent, "ARM_TIMEOUT", 0.2), \
                mock.patch.object(agent, "ARM_JOB_TIMEOUT", 0.8):
            s.bind(("127.0.0.1", 0))
            s.listen(4)
            took = {}
            for cmd in ("center", "pick_bag", "drop_bag"):
                t0 = time.monotonic()
                self.assertEqual(agent.arm_command({"cmd": cmd}, s.getsockname()[1])[0], 504)
                took[cmd] = time.monotonic() - t0
        self.assertLess(took["center"], 0.7)
        self.assertGreaterEqual(min(took["pick_bag"], took["drop_bag"]), 0.75)


if __name__ == "__main__":
    unittest.main()
