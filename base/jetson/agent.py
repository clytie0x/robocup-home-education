"""Start, stop and report the Jetson's Help Me Carry programs over HTTP, for the Pi's robot console.

  python3 agent.py [--port 8090] [--allow 127.0.0.1,10.0.0.1,192.168.31.21]

Programs (run from this folder, ~/hmc-follow; the ROS ones with /opt/ros/jazzy and ROS_DOMAIN_ID=42):
  detector  python3 server.py                                    person detector, TCP 5601
  whisper   VOICE_ROOT/src/whisper.cpp/build/bin/whisper-server  speech to text, TCP 8080 (servers.sh start-stt
            in the foreground; VOICE_ROOT must be checked on the robot)
  voice     ros2 launch voice_assistant hmc_voice.launch.py      stt, tts and the rule brain (VOICE_WS sourced)
  bridge    python3 base_bridge.py --pi-host 10.0.0.1,192.168.31.21   ROS <-> run_follow --remote
  arm       python3 bag_arm.py                                   the arm side's pick_bag and drop_bag jobs
                                                                 (~/so100_ws sourced)
  flow      python3 hmc_flow.py --ros-args -p use_arm:=true -p grab_timeout:=240.0 -p drop_timeout:=60.0
                                                                 the task flow (speech, says, state)

  GET  /status                     the shape below
  POST /start {"name": "flow"}     also /stop and /restart; one name from the list above
  POST /start-all                  detector, whisper, voice, bridge, armstack, webcam, arm, then flow (each waits
                                   for the one before); a whisper-server, arm stack or webcam_view started by hand
                                   is used as it is
  POST /stop-all                   flow, arm, webcam, armstack, bridge, voice, whisper, detector
  POST /arm {"cmd": ...}           the console's arm buttons, relayed as one JSON line to bag_arm.py's command
                                   server (127.0.0.1:5607, see its docstring) and its answer back: 200 when it
                                   says ok, 409 when it refuses or fails, 503 when bag_arm.py does not answer,
                                   502 when it drops the line, 504 after ARM_TIMEOUT (center: home, a MoveIt move and the gripper)
                                   or, for pick_bag and drop_bag (a whole bag job), ARM_JOB_TIMEOUT, for bagpick
                                   (the arm team's camera pick) ARM_BAGPICK_TIMEOUT
  GET  /arm                        {"cmd": "state"}'s answer: /arm/state, the joints (rad), "busy"
  POST /flow {"cmd": ...}          the console executive's own, heard, say and release, relayed as one JSON line to
                                   hmc_flow.py's command server (127.0.0.1:5608, see its docstring) and its answer
                                   back with 200 (ok or not), 503 when the flow program does not answer, 502 when
                                   it drops the line or answers no JSON object, 504 after FLOW_TIMEOUT

GET /status (the robot console shows it; base/pi/telemetry.py stores the metrics). Every metric is a
number or null, never a string; a missing source gives null (or an empty object):
  "thermal": {zone type: deg C}     every /sys/class/thermal/thermal_zone*/ whose temp reads (cv0..cv2 answer
                                    "No data available" and are left out); a repeated type gets "-<zone n>"
  "power": {label: {"mV", "mA", "mW"}}  the INA3221 rails (/sys/bus/i2c/drivers/ina3221/*/hwmon/hwmon*/:
                                    in<N>_label, in<N>_input mV, curr<N>_input mA; "sum of shunt voltages" left
                                    out); on this Jetson VDD_IN, VDD_CPU_GPU_CV, VDD_SOC
  "gpu_load_pct"                    /sys/devices/platform/gpu.0/load / 10 (the file is per mille)
  "cpu_pct"                         busy share of all CPUs since the previous /status (/proc/stat); null the first time
  "load1", "mem_used_mb", "mem_total_mb", "disk_free_gb" (this folder's disk), "uptime_s", "time" (epoch s)
  "programs": {name: {"running": bool, "pid", "started" (epoch s), "uptime" (s), "code", "command", "log": [lines]}}
  "order": [names], "sequence": start-all's progress text or null, "outside": {name: [pid]}, "host": hostname

Each program is its own process group, its output in logs/<name>.log here. The agent only ever
signals the processes it started itself; a copy started by hand is listed under "outside" (read
from /proc, children of the agent's own groups left out) and left alone, and while one runs the
agent refuses to start that program (409), so two bridges never fight for the Pi. Stop waits for
the whole group to be gone (Ctrl+C, SIGTERM, then SIGKILL to the group), not only for bash.
A copy counts by its argv: any python* executable with the script's file name in any position
(python3 -u base_bridge.py, python3.12 ...; ros2 is a python script, so its hmc_voice.launch.py
counts), or for whisper an argv[0] named whisper-server. The pids are listed in numeric order. Our
python programs are no ROS packages, so there is no ros2 run form to look for. Before stopping or restarting the flow or the bridge (and in stop-all)
while the agent's bridge runs, the agent sends b"hold" to the bridge's UDP 127.0.0.1:5605 and waits
0.3 s: a Pi in FOLLOW or RETURN gets wait at once, not 2 s after the bridge is gone. The answer's
"hold" says when that was done. On SIGTERM or Ctrl+C the agent stops what it started.
GET is open to anyone; POST only from the --allow addresses (the Pi and this machine), so a phone
on the lab Wi-Fi goes through the Pi's console. CORS headers are sent so a page can read /status.
Plain Python 3 standard library.
"""
import argparse
import fcntl
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import ports

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"
PORT = ports.AGENT
ALLOW = ("127.0.0.1", "10.0.0.1", "192.168.31.21")  # localhost and the Pi on the LAN cable and the Wi-Fi
ROS = "source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=42 && "
# Where the voice side lives on the Jetson. Both must be checked on the robot before the first start-all:
# VOICE_ROOT is ROOT of voice/scripts/servers.sh (whisper.cpp build and models, voice/CLAUDE.md), VOICE_WS the
# colcon workspace with voice_assistant built in it (voice/README.md: ~/ros2_ws).
VOICE_ROOT = "/opt/robot_voice"
VOICE_WS = "~/ros2_ws"
# servers.sh start_whisper without its "&" (the agent must see the server run) and without the download:
# small.en when it is there, else base.en, as pick_model does offline
WHISPER = ("M=%(r)s/models/whisper/ggml-small.en-q5_1.bin; [ -s \"$M\" ] || M=%(r)s/models/whisper/ggml-base.en-q5_1.bin; "
           "exec %(r)s/src/whisper.cpp/build/bin/whisper-server -m \"$M\" -l en -t 4 --host 127.0.0.1 --port %(p)d"
           % {"r": VOICE_ROOT, "p": ports.WHISPER})
# bag_arm: a real pick took 30.4 s and a real place 30.0 s on the arm, but bag_arm may need /arm/home 60 s +
# pick 150 s, so the flow waits 240 s for a pick and 60 s for a place
FLOW_ARGS = "--ros-args -p use_arm:=true -p grab_timeout:=240.0 -p drop_timeout:=60.0"
PROGRAMS = {  # name: (shell command, needs ROS, TCP port that says it is up, or None)
    "detector": ("python3 server.py", False, ports.DETECTOR),
    "whisper": (WHISPER, False, ports.WHISPER),
    "voice": ("source %s/install/setup.bash && ros2 launch voice_assistant hmc_voice.launch.py" % VOICE_WS, True, None),
    "bridge": ("python3 base_bridge.py --pi-host 10.0.0.1,192.168.31.21", True, None),
    # the arm stack (driver, MoveIt, pick_place with /arm/*): before 2026-11-15 it was started by hand from a
    # terminal; a copy started that way is used as it is (SHARED)
    "armstack": ("source ~/so100_ws/install/setup.bash && ros2 launch so100_moveit_config arm.launch.py mount:=robot"
                 " rviz:=false", True, None),
    # the arm's webcam (/dev/video0) for the console's Live tab and bag_arm's /snap.jpg; before 2026-11-16 only
    # ever started by hand, so a copy started that way is used as it is (SHARED)
    "webcam": ("cd ~/so100_ws && exec python3 webcam_view.py", False, ports.WEBCAM),
    "arm": ("source ~/so100_ws/install/setup.bash && python3 bag_arm.py", True, None),
    "flow": ("python3 hmc_flow.py " + FLOW_ARGS, True, None),
}
ORDER = ["detector", "whisper", "voice", "bridge", "armstack", "webcam", "arm", "flow"]
SCRIPTS = {"detector": "server.py", "voice": "hmc_voice.launch.py", "bridge": "base_bridge.py", "arm": "bag_arm.py",
           "armstack": "arm.launch.py", "webcam": "webcam_view.py",
           "flow": "hmc_flow.py"}  # a python* argv with this file name (ros2 is python: its launch file counts)
EXECUTABLES = {"whisper": "whisper-server"}  # argv[0]'s file name
# Programs of voice's generic stack (voice_assistant.launch.py, as robot_voice/scripts/control_panel.py starts it,
# or its nodes run one by one) that clash with ours on domain 42: its flow_node answers /speech/heard next to
# hmc_flow (both say /tts/say and send /robot/command), its stt/tts would share the mic and speaker with ours,
# command_bridge_node drives /cmd_vel and calls /arm/stop, arm_action_bridge calls /arm/pick and /arm/place on
# drop_bag next to bag_arm. A python* argv with one of these file names counts as an outside copy of each name.
CLASHES = {"voice_assistant.launch.py": ("voice", "flow", "arm"), "flow_node": ("flow",),
           "command_bridge_node": ("flow", "arm"), "arm_action_bridge": ("arm",)}
SHARED = ("whisper", "armstack", "webcam")  # start-all uses a copy started by hand (servers.sh start-stt) instead of stopping there
LOG_LINES = 20
STOP_STEPS = ((signal.SIGINT, 5.0), (signal.SIGTERM, 3.0))  # ROS nodes end cleanly on Ctrl+C; SIGKILL after
KILL = getattr(signal, "SIGKILL", signal.SIGTERM)
HOLD_PORT = ports.HOLD  # base_bridge.py's HoldListener
HOLD_SETTLE = 0.3  # s for the bridge (a 50 ms timer) to pass the wait on
ARM_CMD_PORT = ports.ARM_CMD  # bag_arm.py's command server (TCP 127.0.0.1)
ARM_TIMEOUT = 90.0  # s: bag_arm's center takes at most 30 + 45 + 10 s
ARM_JOB_TIMEOUT = 240.0  # s for pick_bag and drop_bag: home 60 + pick 150 + the camera looks (the flow's grab_timeout)
ARM_BAGPICK_TIMEOUT = 330.0  # s for bagpick: bag_arm's BAGPICK_TIMEOUT 300 + its kill and answer
FLOW_CMD_PORT = ports.FLOW_CMD  # hmc_flow.py's command server (TCP 127.0.0.1)
FLOW_TIMEOUT = 3.0  # s: its answers come at once (no move waits)
UP_WAIT = 60.0  # s start-all waits for the detector's port (the TensorRT engines take a while)
SETTLE = 3.0  # s start-all waits after a program without a port, before the next


def tail(path, n=LOG_LINES, start=0):
    """The last n lines of path from byte offset start, as text."""
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(start, size - 32768))
            data = f.read()
    except OSError:
        return []
    return data.decode("utf-8", "replace").splitlines()[-n:]



def deployed_version(path=None):
    """The commit base/tools/deploy.sh wrote next to this file ({"commit", "describe", "branch", "deployed"}),
    or None (never deployed that way, or an unreadable file)."""
    try:
        with open(path or HERE / "VERSION.json", encoding="utf-8") as f:
            v = json.load(f)
    except (OSError, ValueError):
        return None
    return v if isinstance(v, dict) and isinstance(v.get("commit"), str) else None


class Program:
    """One program the agent starts and stops. It never signals a process it did not start."""

    def __init__(self, name, cmd, logs=LOGS, popen=subprocess.Popen, clock=time.time, sleep=time.sleep,
                 killpg=None):
        self.name, self.cmd = name, cmd
        self.killpg = killpg or (os.killpg if os.name == "posix" else None)  # None: Windows, no groups
        self.log = Path(logs) / ("%s.log" % name)
        self.popen, self.clock, self.sleep = popen, clock, sleep
        self.lock = threading.Lock()
        self.proc = None
        self.started = None
        self.offset = 0  # where this run's output starts in the log
        self.code = None  # exit code of the last run

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def _group_alive(self):
        """Any process left in the group this program leads (bash may be gone while python lives)."""
        if self.proc is None:
            return False
        if self.killpg is None:
            return self.proc.poll() is None
        self.proc.poll()  # reaps bash, so a finished leader does not count
        try:
            self.killpg(self.proc.pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def start(self):
        """None, or why it was not started."""
        with self.lock:
            if self._group_alive():
                return "%s is already running (pid %d)" % (self.name, self.proc.pid)
            self.log.parent.mkdir(parents=True, exist_ok=True)
            out = open(self.log, "ab")
            self.offset = out.tell()
            out.write(("=== %s started by agent.py at %s: %s\n" % (
                self.name, time.strftime("%Y-%m-%d %H:%M:%S"), self.cmd)).encode())
            out.flush()
            kwargs = {"start_new_session": True} if os.name == "posix" else {
                "creationflags": subprocess.CREATE_NEW_PROCESS_GROUP}
            shell = ["bash", "-c", self.cmd] if os.name == "posix" else self.cmd
            try:
                # the console starts every program through the agent: the arm gate lets HMC_CONSOLE_JOB through
                self.proc = self.popen(shell, cwd=str(HERE), stdin=subprocess.DEVNULL, stdout=out,
                                       stderr=subprocess.STDOUT, shell=os.name != "posix",
                                       env=dict(os.environ, HMC_CONSOLE_JOB="agent:%s" % self.name), **kwargs)
            except OSError as error:
                return "%s did not start: %s" % (self.name, error)
            finally:
                out.close()
            self.started, self.code = self.clock(), None
            return None

    def _signal(self, sig):
        if self.killpg is not None:
            self.killpg(self.proc.pid, sig)  # the group it leads: bash and the python under it
        else:  # Windows (tests on a laptop): no groups to signal, end the whole tree
            subprocess.run(["taskkill", "/T", "/F", "/PID", str(self.proc.pid)], capture_output=True)

    def stop(self):
        """Ctrl+C to its group, then SIGTERM, then SIGKILL. None, or why nothing was done."""
        with self.lock:
            if not self._group_alive():
                return "%s is not running" % self.name
            for sig, wait in STOP_STEPS + ((KILL, 2.0),):
                try:
                    self._signal(sig)
                except OSError:
                    pass
                end = self.clock() + wait
                while self._group_alive() and self.clock() < end:
                    self.sleep(0.05)
                if not self._group_alive():
                    break
            self.code = self.proc.poll()
            if self._group_alive():
                return "%s: processes still in group %d after SIGKILL" % (self.name, self.proc.pid)
            return None

    def status(self):
        running = self.running()
        if self.proc is not None and not running:
            self.code = self.proc.poll()
        return {"running": running, "pid": self.proc.pid if self.proc is not None else None,
                "started": self.started, "uptime": round(self.clock() - self.started, 1) if running else None,
                "code": None if running else self.code, "command": self.cmd,
                "log": tail(self.log, LOG_LINES, self.offset) if self.started is not None else tail(self.log)}


def port_open(port, host="127.0.0.1", timeout=0.5):
    try:
        with socket.create_connection((host, port), timeout):
            return True
    except OSError:
        return False


def outside(ours, proc=Path("/proc")):
    """{name: [pid, ...]} of copies of the programs this agent did not start (read only)."""
    found = {}
    if not proc.is_dir():
        return found
    for entry in proc.iterdir():
        if not entry.name.isdigit() or int(entry.name) in ours:
            continue
        try:
            args = (entry / "cmdline").read_bytes().split(b"\0")
            pgrp = int((entry / "stat").read_text().rsplit(")", 1)[1].split()[2])
        except (OSError, ValueError, IndexError):
            continue
        if pgrp in ours:  # the python under one of our bash groups
            continue
        first = os.path.basename(args[0].decode("utf-8", "replace")) if args else ""
        for name, exe in EXECUTABLES.items():
            if first == exe:
                found.setdefault(name, []).append(int(entry.name))
        if not first.startswith("python"):
            continue
        names = {os.path.basename(a.decode("utf-8", "replace")) for a in args[1:]}
        for name, script in SCRIPTS.items():
            if script in names:
                found.setdefault(name, []).append(int(entry.name))
        for script, blocked in CLASHES.items():
            if script in names:
                for name in blocked:
                    found.setdefault(name, []).append(int(entry.name))
    return {name: sorted(pids) for name, pids in found.items()}  # /proc's own order differs between machines


def cmdline(pid, proc=Path("/proc")):
    """The argv of pid as text, [] when it does not read (gone, no /proc)."""
    try:
        return [a.decode("utf-8", "replace") for a in (proc / str(pid) / "cmdline").read_bytes().split(b"\0") if a]
    except OSError:
        return []


SHARED_NEEDS = {"armstack": "mount:=robot"}  # a hand-started arm stack with another mount (the desk) has the
# wrong floor and arm pose for the robot (review 2026-11-15): start-all refuses it instead of using it


def send_hold(port=HOLD_PORT, settle=HOLD_SETTLE):
    """Ask the bridge to put a moving Pi on wait; nobody listening is no error."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(b"hold", ("127.0.0.1", port))
    except OSError:
        pass
    time.sleep(settle)


def arm_command(body, port=ARM_CMD_PORT, timeout=None):
    """(http code, answer) of one command sent to bag_arm.py's command server (timeout None: the command's own)."""
    if timeout is None:
        cmd = body.get("cmd")
        timeout = (ARM_BAGPICK_TIMEOUT if cmd == "bagpick" else ARM_JOB_TIMEOUT if cmd in ("pick_bag", "drop_bag")
                   else ARM_TIMEOUT)
    try:
        s = socket.create_connection(("127.0.0.1", port), 2.0)
    except OSError as error:  # refused, or on Windows a refused localhost connect that times out
        return 503, {"ok": False, "error": "bag_arm.py is not answering on 127.0.0.1:%d (start the arm program): %s"
                     % (port, error)}
    try:
        with s:
            s.settimeout(timeout)
            s.sendall((json.dumps(body) + "\n").encode())
            line = s.makefile("rb").readline(65536)
    except socket.timeout:
        return 504, {"ok": False, "error": "bag_arm.py did not answer within %.0f s" % timeout}
    except OSError as error:
        return 502, {"ok": False, "error": "bag_arm.py dropped the connection: %s" % error}
    try:
        out = json.loads(line)
    except ValueError:
        return 502, {"ok": False, "error": "bag_arm.py closed without an answer"}
    return (200 if out.get("ok") else 409), out


def flow_command(body, port=FLOW_CMD_PORT, timeout=FLOW_TIMEOUT):
    """(http code, answer) of one request sent to hmc_flow.py's command server: its answer is 200, ok or not."""
    try:
        s = socket.create_connection(("127.0.0.1", port), 2.0)
    except OSError as error:  # refused, or on Windows a refused localhost connect that times out
        return 503, {"ok": False, "error": "the flow program (hmc_flow.py) does not answer on 127.0.0.1:%d (start the "
                     "flow program): %s" % (port, error)}
    try:
        with s:
            s.settimeout(timeout)
            s.sendall((json.dumps(body) + "\n").encode())
            line = s.makefile("rb").readline(65536)
    except socket.timeout:
        return 504, {"ok": False, "error": "the flow program did not answer within %.0f s" % timeout}
    except OSError as error:
        return 502, {"ok": False, "error": "the flow program dropped the connection: %s" % error}
    try:
        out = json.loads(line)
    except ValueError:
        out = None
    if not isinstance(out, dict):
        return 502, {"ok": False, "error": "the flow program closed without a JSON object answer"}
    return 200, out


def _number(path, scale=1.0):
    """float(text of path) * scale, or None when it does not read (missing, "No data available", ...)."""
    try:
        return float(Path(path).read_text().split()[0]) * scale
    except (OSError, ValueError, IndexError):
        return None


def thermal(root=Path("/")):
    """{zone type: deg C} of every zone whose temp reads."""
    out = {}
    zones = sorted((root / "sys/class/thermal").glob("thermal_zone*"), key=lambda z: int(z.name[12:] or 0))
    for zone in zones:
        c = _number(zone / "temp", 0.001)
        if c is None:
            continue
        try:
            name = (zone / "type").read_text().strip() or zone.name
        except OSError:
            name = zone.name
        if name in out:
            name = "%s-%s" % (name, zone.name[12:])
        out[name] = round(c, 1)
    return out


def power(root=Path("/")):
    """{rail label: {"mV", "mA", "mW"}} from the INA3221 monitor."""
    out = {}
    for hwmon in sorted(root.glob("sys/bus/i2c/drivers/ina3221/*/hwmon/hwmon*")):
        for label_file in sorted(hwmon.glob("in*_label")):
            n = label_file.name[2:-len("_label")]
            try:
                label = label_file.read_text().strip()
            except OSError:
                continue
            if not label or label.lower().startswith("sum of shunt"):
                continue
            mv, ma = _number(hwmon / ("in%s_input" % n)), _number(hwmon / ("curr%s_input" % n))
            out[label] = {"mV": mv, "mA": ma, "mW": None if mv is None or ma is None else round(mv * ma / 1000.0, 1)}
    return out


def cpu_times(root=Path("/")):
    """(busy, total) jiffies from the first line of /proc/stat, or None."""
    try:
        fields = [int(x) for x in (root / "proc/stat").read_text().splitlines()[0].split()[1:]]
        idle = fields[3] + (fields[4] if len(fields) > 4 else 0)  # idle + iowait
    except (OSError, ValueError, IndexError):
        return None
    return sum(fields) - idle, sum(fields)


class Metrics:
    """The flat numbers of /status. cpu_pct needs two /proc/stat readings: null on the first call."""

    def __init__(self, root=Path("/"), disk=HERE):
        self.root, self.disk = Path(root), disk
        self.last_cpu = None

    def read(self):
        root = self.root
        out = {"thermal": thermal(root), "power": power(root), "gpu_load_pct": None, "cpu_pct": None, "load1": None,
               "mem_used_mb": None, "mem_total_mb": None, "disk_free_gb": None, "uptime_s": None}
        for gpu in ("sys/devices/platform/gpu.0/load", "sys/devices/platform/17000000.gpu/load"):
            load = _number(root / gpu, 0.1)
            if load is not None:
                out["gpu_load_pct"] = round(load, 1)
                break
        cpu = cpu_times(root)
        if cpu is not None and self.last_cpu is not None and cpu[1] > self.last_cpu[1]:
            out["cpu_pct"] = round(100.0 * (cpu[0] - self.last_cpu[0]) / (cpu[1] - self.last_cpu[1]), 1)
        if cpu is not None:
            self.last_cpu = cpu
        out["load1"] = _number(root / "proc/loadavg")
        try:
            mem = {}
            for line in (root / "proc/meminfo").read_text().splitlines():
                key, _, value = line.partition(":")
                if value.split():
                    mem[key] = int(value.split()[0])
            out["mem_total_mb"] = round(mem["MemTotal"] / 1024.0, 1)
            out["mem_used_mb"] = round((mem["MemTotal"] - mem["MemAvailable"]) / 1024.0, 1)
        except (OSError, KeyError, ValueError):
            pass
        try:
            out["disk_free_gb"] = round(shutil.disk_usage(self.disk).free / 1e9, 2)
        except OSError:
            pass
        uptime = _number(root / "proc/uptime")
        out["uptime_s"] = None if uptime is None else round(uptime)
        return out


class Agent:
    def __init__(self, programs=None, logs=LOGS, order=None, popen=subprocess.Popen, clock=time.time,
                 sleep=time.sleep, port_open=port_open, metrics=None, find_outside=outside, hold=send_hold,
                 read_args=cmdline, arm_port=ARM_CMD_PORT, flow_port=FLOW_CMD_PORT):
        programs = PROGRAMS if programs is None else programs
        self.arm_port, self.flow_port = arm_port, flow_port
        self.programs = {}
        for name, (cmd, ros, port) in programs.items():
            self.programs[name] = Program(name, (ROS + cmd) if ros else cmd, logs, popen, clock, sleep)
            self.programs[name].port = port
        self.order = list(order or programs)
        self.clock, self.sleep, self.port_open = clock, sleep, port_open
        self.find_outside = find_outside
        self.read_args = read_args
        self.hold = hold
        self.metrics = metrics or Metrics()
        self.metrics_lock = threading.Lock()
        self.sequence = None  # what start-all is doing, for the page
        self.seq_lock = threading.Lock()

    def act(self, action, name=None):
        """(http code, body) for one POST."""
        if action in ("start", "stop", "restart"):
            if name not in self.programs:
                return 400, {"error": "name must be one of %s" % ", ".join(self.order)}
            p = self.programs[name]
            held = self._hold_first() if action != "start" and name in ("flow", "bridge") else None
            if action == "restart":
                p.stop()
                error = self._outside_error(name) or p.start()
            elif action == "start":
                error = self._outside_error(name) or p.start()
            else:
                error = p.stop()
            return (409 if error else 200), {"error": error, "hold": held, "status": self.status()}
        if action == "start-all":
            if not self.seq_lock.acquire(blocking=False):
                return 409, {"error": "start-all is already running", "status": self.status()}
            threading.Thread(target=self._start_all, daemon=True).start()
            return 200, {"error": None, "status": self.status()}
        if action == "stop-all":
            held = self._hold_first()
            for n in reversed(self.order):
                self.programs[n].stop()
            return 200, {"error": None, "hold": held, "status": self.status()}
        return 404, {"error": "unknown action %r" % action}

    def _hold_first(self):
        """Before the flow or the bridge stops: the bridge puts a moving Pi on wait. Its note, or None."""
        bridge = self.programs.get("bridge")
        if bridge is None or not bridge.running():
            return None  # no bridge of ours: the Pi already holds after 2 s of silence
        self.hold()
        return "asked the bridge to hold the Pi first (wait if it was in FOLLOW or RETURN)"

    def _ours(self):
        return {p.proc.pid for p in self.programs.values() if p.proc is not None}

    def _outside_error(self, name):
        """Why name must not start: a copy started by hand runs (two bridges would fight for the Pi)."""
        pids = self.find_outside(self._ours()).get(name)
        if not pids:
            return None
        listed = ", ".join(str(pid) for pid in pids)
        return ("มี %s หรือโปรแกรมที่ชนกับมัน (voice_assistant.launch.py, flow_node, ...) เปิดเองด้วยมืออยู่แล้ว "
                "(pid %s) ให้ปิดตัวนั้นก่อน / "
                "%s, or a program that clashes with it (voice_assistant.launch.py, flow_node, ...), is already "
                "running, started by hand (pid %s): stop that copy first" % (name, listed, name, listed))

    def _start_all(self):
        try:
            for name in self.order:
                p = self.programs[name]
                self.sequence = "starting %s" % name
                shared = name in SHARED and self.find_outside(self._ours()).get(name)
                need = SHARED_NEEDS.get(name)
                if shared and need and not all(need in self.read_args(pid) for pid in shared):
                    listed = ", ".join(str(pid) for pid in shared)
                    self.sequence = ("%s เปิดด้วยมือ (pid %s) แต่ไม่มี %s ให้ปิดตัวนั้นก่อน / "
                                     "%s started by hand (pid %s) lacks %s: stop that copy first"
                                     % (name, listed, need, name, listed, need))
                    return
                if not p.running() and not shared:  # a shared copy by hand: only its port is waited for
                    error = self._outside_error(name) or p.start()
                    if error:
                        self.sequence = error
                        return
                if p.port:
                    end = self.clock() + UP_WAIT
                    while not self.port_open(p.port) and self.clock() < end:
                        if not shared and not p.running():
                            self.sequence = "%s ended before it was up: see its log" % name
                            return
                        self.sleep(0.5)
                    if not self.port_open(p.port):
                        self.sequence = "%s not listening on port %d after %d s" % (name, p.port, UP_WAIT)
                        return
                else:
                    end = self.clock() + SETTLE
                    while self.clock() < end:
                        self.sleep(0.2)
                    if not shared and not p.running():
                        self.sequence = "%s ended at once: see its log" % name
                        return
            self.sequence = "all started"
        finally:
            self.seq_lock.release()

    def status(self):
        ours = self._ours()
        with self.metrics_lock:  # cpu_pct is a difference from the previous reading
            out = self.metrics.read()
        out.update({"time": self.clock(), "programs": {n: self.programs[n].status() for n in self.order},
                    "order": self.order, "sequence": self.sequence, "outside": self.find_outside(ours),
                    "host": socket.gethostname(), "version": deployed_version()})
        return out

    def stop_all(self):
        self._hold_first()
        for n in reversed(self.order):
            self.programs[n].stop()


def handler_for(agent, allow):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, body):
            data = json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_OPTIONS(self):
            self._send(200, {})

        def do_GET(self):
            if self.path.split("?")[0] == "/status":
                self._send(200, agent.status())
            elif self.path.split("?")[0] == "/arm":
                self._send(*arm_command({"cmd": "state"}, agent.arm_port))
            else:
                self._send(404, {"error": "not found"})

        def do_POST(self):
            if self.client_address[0] not in allow:
                return self._send(403, {"error": "control only from %s; use the Pi's console" % ", ".join(allow)})
            try:
                length = int(self.headers.get("Content-Length", "0"))
                body = json.loads(self.rfile.read(min(length, 4096)) or b"{}")
            except ValueError:
                return self._send(400, {"error": "not JSON"})
            if not isinstance(body, dict):
                return self._send(400, {"error": "send a JSON object"})
            if self.path.strip("/") == "arm":
                return self._send(*arm_command(body, agent.arm_port))
            if self.path.strip("/") == "flow":
                return self._send(*flow_command(body, agent.flow_port))
            code, out = agent.act(self.path.strip("/"), body.get("name"))
            self._send(code, out)
    return Handler


def claim_instance(port, directory=None):
    """One agent per port. Returns the lock file; the caller keeps it open. SystemExit if another holds it.

    The kernel releases the lock when this process exits, so a crashed agent does not block the next start.
    A live listener is not stolen: the socket is bound only after the lock, and SO_REUSEADDR is not set.
    """
    directory = directory or os.environ.get("HMC_LOCK_DIR", "/tmp")
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, "hmc-agent-%s.lock" % port)
    fd = open(path, "a+")
    try:
        fcntl.flock(fd.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        fd.seek(0)
        holder = fd.read().strip() or "unknown"
        fd.close()
        raise SystemExit("agent already running on port %d (pid %s); not starting a second one" % (port, holder))
    fd.seek(0)
    fd.truncate()
    fd.write("%s\n" % os.getpid())
    fd.flush()
    return fd


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--allow", default=",".join(ALLOW), help="addresses that may start and stop (comma separated)")
    args = ap.parse_args(argv)
    lock = claim_instance(args.port)
    agent = Agent()
    try:
        server = ThreadingHTTPServer(("0.0.0.0", args.port), handler_for(agent, tuple(args.allow.split(","))))
    except OSError as exc:
        lock.close()
        raise SystemExit("agent cannot listen on port %d: %s" % (args.port, exc)) from exc
    server._instance_lock = lock  # kept open so the lock lives as long as the server
    server.daemon_threads = True

    def finish(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, finish)
    print("agent on port %d; control from %s" % (args.port, args.allow), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        agent.stop_all()
        server.server_close()


if __name__ == "__main__":
    sys.exit(main())
