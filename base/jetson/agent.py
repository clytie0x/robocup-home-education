"""Start, stop and report the Jetson's three Help Me Carry programs over HTTP, for the Pi's robot console.

  python3 agent.py [--port 8090] [--allow 127.0.0.1,10.0.0.1,192.168.31.21]

Programs (run from this folder, ~/hmc-follow; the ROS ones with /opt/ros/jazzy and ROS_DOMAIN_ID=42):
  detector  python3 server.py                                    person detector, TCP 5601
  bridge    python3 base_bridge.py --pi-host 10.0.0.1,192.168.31.21   ROS <-> run_follow --remote
  flow      python3 hmc_flow.py                                  the task flow (speech, says, state)

  GET  /status                     the shape below
  POST /start {"name": "flow"}     also /stop and /restart; one name from the list above
  POST /start-all                  detector, then bridge, then flow (each waits for the one before)
  POST /stop-all                   flow, bridge, detector

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
(python3 -u base_bridge.py, python3.12 ...). None of the three is a ROS package, so there is no
ros2 run form to look for. Before stopping or restarting the flow or the bridge (and in stop-all)
while the agent's bridge runs, the agent sends b"hold" to the bridge's UDP 127.0.0.1:5605 and waits
0.3 s: a Pi in FOLLOW or RETURN gets wait at once, not 2 s after the bridge is gone. The answer's
"hold" says when that was done. On SIGTERM or Ctrl+C the agent stops what it started.
GET is open to anyone; POST only from the --allow addresses (the Pi and this machine), so a phone
on the lab Wi-Fi goes through the Pi's console. CORS headers are sent so a page can read /status.
Plain Python 3 standard library.
"""
import argparse
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

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"
PORT = 8090
ALLOW = ("127.0.0.1", "10.0.0.1", "192.168.31.21")  # localhost and the Pi on the LAN cable and the Wi-Fi
ROS = "source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=42 && "
PROGRAMS = {  # name: (shell command, needs ROS, TCP port that says it is up, or None)
    "detector": ("python3 server.py", False, 5601),
    "bridge": ("python3 base_bridge.py --pi-host 10.0.0.1,192.168.31.21", True, None),
    "flow": ("python3 hmc_flow.py", True, None),
}
ORDER = ["detector", "bridge", "flow"]
SCRIPTS = {"detector": "server.py", "bridge": "base_bridge.py", "flow": "hmc_flow.py"}
LOG_LINES = 20
STOP_STEPS = ((signal.SIGINT, 5.0), (signal.SIGTERM, 3.0))  # ROS nodes end cleanly on Ctrl+C; SIGKILL after
KILL = getattr(signal, "SIGKILL", signal.SIGTERM)
HOLD_PORT = 5605  # base_bridge.py's HoldListener
HOLD_SETTLE = 0.3  # s for the bridge (a 50 ms timer) to pass the wait on
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
                self.proc = self.popen(shell, cwd=str(HERE), stdin=subprocess.DEVNULL, stdout=out,
                                       stderr=subprocess.STDOUT, shell=os.name != "posix", **kwargs)
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
        if not args or not os.path.basename(args[0].decode("utf-8", "replace")).startswith("python"):
            continue
        names = {os.path.basename(a.decode("utf-8", "replace")) for a in args[1:]}
        for name, script in SCRIPTS.items():
            if script in names:
                found.setdefault(name, []).append(int(entry.name))
    return found


def send_hold(port=HOLD_PORT, settle=HOLD_SETTLE):
    """Ask the bridge to put a moving Pi on wait; nobody listening is no error."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(b"hold", ("127.0.0.1", port))
    except OSError:
        pass
    time.sleep(settle)


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
                 sleep=time.sleep, port_open=port_open, metrics=None, find_outside=outside, hold=send_hold):
        programs = PROGRAMS if programs is None else programs
        self.programs = {}
        for name, (cmd, ros, port) in programs.items():
            self.programs[name] = Program(name, (ROS + cmd) if ros else cmd, logs, popen, clock, sleep)
            self.programs[name].port = port
        self.order = list(order or programs)
        self.clock, self.sleep, self.port_open = clock, sleep, port_open
        self.find_outside = find_outside
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
        return ("มี %s ที่เปิดเองด้วยมืออยู่แล้ว (pid %s) "
                "ให้ปิดตัวนั้นก่อน / "
                "%s is already running, started by hand (pid %s): stop that copy first" % (name, listed, name, listed))

    def _start_all(self):
        try:
            for name in self.order:
                p = self.programs[name]
                self.sequence = "starting %s" % name
                if not p.running():
                    error = self._outside_error(name) or p.start()
                    if error:
                        self.sequence = error
                        return
                if p.port:
                    end = self.clock() + UP_WAIT
                    while not self.port_open(p.port) and self.clock() < end:
                        if not p.running():
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
                    if not p.running():
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
            code, out = agent.act(self.path.strip("/"), body.get("name"))
            self._send(code, out)
    return Handler


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--allow", default=",".join(ALLOW), help="addresses that may start and stop (comma separated)")
    args = ap.parse_args(argv)
    agent = Agent()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), handler_for(agent, tuple(args.allow.split(","))))
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
