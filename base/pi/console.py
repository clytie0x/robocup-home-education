"""Robot console: one web page to run and check the whole robot from a phone or a laptop.

  .venv/bin/python console.py [--port 8081] [--no-devices]
  then open http://192.168.31.21:8081/ (Wi-Fi) or http://10.0.0.1:8081/ (LAN cable)

Tabs: Robot (the default: Xtion, arm webcam, Jetson snapshot, LiDAR, status, events, pick and drop,
STOP), Overview (state, faults, health list, power bank estimate, motor battery use), Live (Xtion camera, LiDAR from above,
the arm team's webcam), Task (start run_follow, Follow / Wait / Resume / Go back, STOP), Data (charts and
runs from telemetry.py's database, read only), Jetson (detector, bridge, flow through the Jetson's
agent.py), Checks (wheel test, camera pitch, link test), Logs, Help. The
page is plain files in console/ next to this file. It replaces tools/camera_view.py on port 8081
(that one still works on its own; this reuses its stream pieces).

Devices: the camera and the LiDAR can each be open in one program at a time. The console reads
them for the live view and closes both (threads joined) before it starts any job, and opens them
again when the job has ended. During a run, /stream and /scan relay run_follow's own (its dashboard,
fed by live_feed.py), which listens on 127.0.0.1 only: the PIN cannot be skipped through it.
One job at a time: run_follow or one check tool, started through tools/job_guard.py as the leader
of its own process group, output to logs/console/. Anything that moves the wheels runs under tools/safe_run.py. STOP: POST /control {"end": true} to the
run's own dashboard (port 8080), SIGTERM to the group after END_GRACE s, SIGKILL after TERM_GRACE
more, then S# from here. When the console ends (Ctrl+C, SIGTERM) it stops the job the same way;
when it is killed outright, job_guard notices and stops the job.

Speeds are only SPEEDS and never above params.json max_speed. Anyone who can reach port 8081 can
use the page; put a PIN in ~/.config/hmc-console-pin to require it for every start and command
(STOP and the battery marks never need it).
"""
import argparse
import hmac
import ipaddress
import json
import math
import os
import queue
import re
import secrets
import shutil
import signal
import socket
import sqlite3
import subprocess
import sys
import tempfile
import threading
import time
import types
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
TOOLS = HERE / "tools"
sys.path.insert(0, str(TOOLS))  # camera_view (Latest, Scan, jpeg_of, relay)
import camera_view  # noqa: E402
import console_gate  # noqa: E402
import ports  # noqa: E402

try:  # the telemetry database (base/pi/telemetry.py) for the Data tab and the power bank estimate
    import telemetry
except Exception:  # not there: the Data tab says so, the rest works
    telemetry = None

STATIC = HERE / "console"
LOG_ROOT = HERE / "logs"  # run_follow's logs/<start time>/ folders; on the Pi ~/hmc-base/logs
JOB_LOGS = LOG_ROOT / "console"
WHEEL_TEST_LAST = LOG_ROOT / "wheel-test" / "last.json"  # written by tools/wheel_test.py
PIN_FILE = Path.home() / ".config" / "hmc-console-pin"
PORT = ports.CONSOLE
RUN_URL = "http://127.0.0.1:%d" % ports.DASHBOARD  # run_follow's own dashboard while it runs
AGENT_PORT = ports.AGENT  # base/jetson/agent.py
WEBCAM_PORT = ports.WEBCAM  # the arm team's ~/so100_ws/webcam_view.py: only shown, never started or stopped
JETSON_HOSTS = ("10.0.0.2", "192.168.31.137")  # LAN cable first, then Wi-Fi
SPEEDS = (0.08, 0.12)  # m/s the page offers
MINUTES_MAX = 30
END_GRACE = 3.0  # s after {"end": true} before the group gets SIGTERM
TERM_GRACE = 3.0  # s after SIGTERM before SIGKILL
CANCEL_S = 3.0  # s: a start refuses to run a job when STOP came within this long before it
RETRY = 5.0  # s between tries to open a missing camera or LiDAR
REFRESH = 3.0  # s between Jetson checks
LIVE_AGE = 1.0  # s; newer than this is live
SKELETON_HZ = 5.0  # frames a second to the Jetson pose detector while the page shows the skeleton
STALE_WHEEL_TEST = 6 * 3600  # s; an older wheel test is a warning
RUN_NAME = re.compile(r"^\d{8}-\d{6}$")  # run_follow's log folder names
FILES = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/style.css": ("style.css", "text/css; charset=utf-8"),
    "/app.js": ("app.js", "text/javascript; charset=utf-8"),
    "/preact-htm.js": ("preact-htm.js", "text/javascript; charset=utf-8"),
    "/robot.js": ("robot.js", "text/javascript; charset=utf-8"),
}
COMMANDS = ("follow", "wait", "resume", "return", "bag", "bag_again", "bag_near")
MISSIONS = ("full", "follow_only", "bag_only")
RETURN_S = 180  # s more for run_follow in the full mission: hmc.flow RETURN_BUDGET, the way back after the clock
MISSION_LABEL = {"full": "โจทย์จริง HMC", "follow_only": "ตามคนอย่างเดียว", "bag_only": "หาและหยิบถุง"}
NO_PIN = ("/api/stop", "/api/arm/stop", "/api/bank/full", "/api/bank/reading")  # STOPs, and the owner's battery marks
CLIENT = re.compile(r"^[\w.@:%-]{1,64}$")  # a program's name (X-HMC-Client), e.g. robotctl's "xohm@192.168.31.50"
ARM_CMDS = ("home", "rest", "center", "torque", "pick_bag", "drop_bag", "bagpick")
ARM_JOBS = ("pick_bag", "drop_bag", "bagpick")  # bag jobs: refused on the page while a task run is active
ARM_TIMEOUT = 95.0  # s: the agent answers an arm command when it is done, within its own ARM_TIMEOUT (90 s)
ARM_JOB_TIMEOUT = 245.0  # s: and a bag job within its ARM_JOB_TIMEOUT (240 s: home 60 + pick 150 + the camera looks)
ARM_BAGPICK_TIMEOUT = 335.0  # s: and the arm team's bagpick within its ARM_BAGPICK_TIMEOUT (330 s)
ARM_STOP_WAIT = 0.5  # s STOP's answer waits for the arm's stop; the base's stop is already under way
FLOW_LEASE = 5.0  # s the Jetson flow stays parked (agent.py POST /flow) without the console's next read
FLOW_POLL = 0.25  # s between two reads of what the parked flow heard
FLOW_TIMEOUT = 4.0  # s for one /flow call: the agent answers 504 after 3 s without the flow program's reply
FLOW_OWN_TIMEOUT = 1.0  # s a run's start waits for the own (and the end for the release); no answer: stopped as before
COMMANDS_MAX = 200  # lines GET /api/commands gives at most
RUN_TAIL = 1 << 20  # bytes at the end of the newest run log that GET /api/commands reads for the voice flow's commands
VOICE_CLIENT = "voice (Jetson)"  # their client name in the command list
HELD = ("the console is held by %s (a deploy or a repair): no starts, arm moves or Jetson start/stop until"
        " robotctl release (STOP still works)")
MAX_BODY = 65536  # bytes of a POST body
SESSION = re.compile(r"^[0-9a-f]{32}$")  # a page tab's token (X-HMC-Session), from its first command
WHEEL_TEST_EVERY = 10.0  # s at least between two wheel tests
WHEEL_CLEAR = (0.35, 0.25)  # m: nothing may be in 0 < x < 0.35, |y| < 0.25 (robot frame) for a wheel test
SCAN_FRESH = 0.5  # s; the wheel test needs a scan of the console's own LiDAR at most this old showing a clear front
BAG_KEEP_S = 30.0  # s a front-camera bag reading is kept when a later look has no camera handle
EVENT_CAP = 80  # lines kept on the Robot page's event list
HURRY_MIN = 0.3  # s a second STOP still leaves the run after asking it to end (it sends its own S# when it ends)
PIN_TRIES = 5  # wrong PINs (from anyone) before the PIN is locked out
PIN_LOCK_S = 60.0  # s of the first lockout; each next one twice as long, at most PIN_LOCK_MAX_S
PIN_LOCK_MAX_S = 900.0
DATA_QUERIES = 2  # /api/data/* at once; 1 while a drive job runs (a run's cycle must not starve)
JOB_SCRIPTS = ("run_follow.py", "safe_run.py", "wheel_test.py", "job_guard.py", "sensor_check.py")
NEED_PIN = ("ตั้ง PIN ก่อน ถึงจะขับจริงหรือทดสอบล้อได้ ใส่ PIN ในไฟล์ ~/.config/hmc-console-pin บน Pi"
            " แล้วเปิดคอนโซลใหม่ (ดู base/docs/robot-console.md) ส่วนงานทดลองเริ่มได้เลย")
JETSON_ACTIONS = ("start", "stop", "restart", "start-all", "stop-all")
VIRTUAL_PORT = ports.VIRTUAL  # UDP: run_follow --virtual-operator takes the PC's simulated person here (virtual_operator.py)
NEED_PIN_VIRTUAL = ("ตั้ง PIN ก่อน ถึงจะทดสอบด้วยคนเสมือนได้ ใส่ PIN ในไฟล์ ~/.config/hmc-console-pin บน Pi"
                    " แล้วเปิดคอนโซลใหม่ (ดู base/docs/robot-console.md)")


# ---------------------------------------------------------------- jobs

def task_command(drive, speed, minutes, p, py=sys.executable, virtual=None, mission="full"):
    """(argv, None) for run_follow, or (None, why not). virtual: the PC's IPv4 address for a virtual operator run."""
    if not isinstance(drive, bool):
        return None, "drive takes true or false"
    if mission not in MISSIONS:
        return None, "mission must be one of %s" % ", ".join(MISSIONS)
    if isinstance(speed, bool) or speed not in SPEEDS:
        return None, "top speed must be one of %s m/s" % ", ".join("%.2f" % s for s in SPEEDS)
    if speed > p["max_speed"] + 1e-9:
        return None, "%.2f m/s is above max_speed %.2f in params.json" % (speed, p["max_speed"])
    if isinstance(minutes, bool) or not isinstance(minutes, (int, float)) or not 0 < minutes <= MINUTES_MAX:
        return None, "length must be 1 to %d minutes" % MINUTES_MAX
    # follow_only omits --remote so Jetson hmc_flow / voice cannot trigger bag picking or interfere
    remote_flag = [] if mission == "follow_only" else ["--remote"]
    run = [py, str(HERE / "run_follow.py")] + (["--drive"] if drive else []) + [
        "--seconds", str(int(round(minutes * 60 + (RETURN_S if mission == "full" else 0)))),
        "--max-speed", "%.2f" % speed, *remote_flag,
        "--dashboard-host", "127.0.0.1"]  # its page skips the PIN: only the console may reach it
    if virtual is not None:
        if not _one_computer(virtual):
            return None, "the virtual operator's PC must be one IPv4 address"
        run += ["--virtual-operator", str(VIRTUAL_PORT), "--virtual-allow", virtual]
    if drive:
        run = [py, str(TOOLS / "safe_run.py")] + run
    return [py, str(TOOLS / "job_guard.py")] + run, None


def _one_computer(address):
    """True for one computer's IPv4 address (not 0.0.0.0, a broadcast or a multicast group)."""
    if not isinstance(address, str):
        return False
    try:
        a = ipaddress.IPv4Address(address)
    except ValueError:
        return False
    return not (a.is_unspecified or a.is_multicast or address.endswith(".255"))


ARG_KINDS = {"number": re.compile(r"^\d+(\.\d+)?$"), "signed": re.compile(r"^-?\d+(\.\d+)?$"),
             "speed": re.compile(r"^\d+(\.\d+)?$"), "side": re.compile(r"^(right|left)$")}
# The tools the console runs as checks by name: (moves, positional kinds, {option: kind}). A "speed" is m/s and never
# above params.json max_speed. Paths (--out, --avoid, --state, after-base-check) and the ladder's limit raisers
# (--max-turn-rate, --linear-scale) are left out: those stay tools for a person at the robot; air_check (raw packets) too.
TOOL_CHECKS = {
    "base_check": (True, [], {"--line": "speed", "--turn": "signed", "--metres": "number", "--degrees": "number",
                              "--max-speed": "speed", "--velocity-loop": "flag", "--yes-owner-at-robot": "flag"}),
    "approach": (True, [], {"--to": "number", "--yes-close-approach": "flag"}),
    "back_up": (True, [], {"--metres": "number", "--yes-back-up": "flag"}),
    "joystick_drive": (True, [], {"--max-speed": "speed", "--seconds": "number", "--yes-owner-at-robot": "flag"}),  # --dry and --device stay out
    "lap_check": (True, [], {"--seconds": "number", "--yes-floor-run": "flag"}),
    "walk_check": (False, [], {"--seconds": "number"}),
}
TETHER = {  # tether_run's subcommands: (positional kinds, {option: kind}); every one runs under safe_run
    "init": ([], {"--length": "number"}), "turn": (["signed"], {}),
    "line": (["number"], {"--speed": "speed", "--back-up-if-blocked": "flag"}),
    "uturn": (["side"], {}), "precheck-line": (["number", "speed"], {})}
PRESENCE = "%s says a person is at the robot: only the console's page may send it, never a program"


def tool_args(args, positional, options, max_speed, page):
    """Checks one tool's arguments against its allow-list; ValueError names the one refused."""
    if not isinstance(args, list) or not all(isinstance(a, str) for a in args):
        raise ValueError("args must be a list of strings")
    left, i = list(positional), 0
    while i < len(args):
        if args[i].startswith("--"):
            name, eq, value = args[i].partition("=")
            kind = options.get(name)
            if kind is None:
                raise ValueError("%s is not allowed through the console" % name)
            if name.startswith("--yes-") and not page:
                raise ValueError(PRESENCE % name)
            if kind == "flag":
                if eq:
                    raise ValueError("%s takes no value" % name)
                i += 1
                continue
            if not eq:
                i += 1
                if i == len(args):
                    raise ValueError("%s needs a value" % name)
                value = args[i]
        elif left:
            name, kind, value = "argument", left.pop(0), args[i]
        else:
            raise ValueError("%r is not allowed through the console" % args[i])
        if not ARG_KINDS[kind].match(value):
            raise ValueError("%s: %r is not a %s" % (name, value, "plain number" if kind != "side" else "right or left"))
        if kind == "speed" and float(value) > max_speed + 1e-9:
            raise ValueError("%s %s m/s is above max_speed %.2f in params.json" % (name, value, max_speed))
        i += 1


def check_command(name, py=sys.executable, args=(), max_speed=0.0, page=True):
    """(argv, label, moves) for a check, or None for an unknown name; ValueError for arguments it does not allow.
    page: sent from the console's own page (a person pressed it), not by a program."""
    guard = [py, str(TOOLS / "job_guard.py")]
    if name in ("wheel", "floor") and args:
        raise ValueError("the %s check takes no arguments" % name)
    if name == "wheel":
        return guard + [py, str(TOOLS / "safe_run.py"), py, str(TOOLS / "wheel_test.py")], "ทดสอบล้อ", True
    if name == "floor":
        return guard + [py, str(TOOLS / "sensor_check.py"), "--floor"], "ตรวจมุมกล้อง", False
    if name == "tether_run":
        if not isinstance(args, list) or not args or not isinstance(args[0], str) or args[0] not in TETHER:
            raise ValueError("tether_run takes one of %s" % ", ".join(TETHER))
        moves, (positional, options), rest = True, TETHER[args[0]], args[1:]
    elif isinstance(name, str) and name in TOOL_CHECKS:
        moves, positional, options = TOOL_CHECKS[name]
        rest = args
    else:
        return None
    tool_args(rest, positional, options, max_speed, page)
    run = [py, str(TOOLS / (name + ".py"))] + list(args)
    return guard + ([py, str(TOOLS / "safe_run.py")] if moves else []) + run, " ".join([name] + list(args)), moves


def signal_group(proc, hard):
    """SIGTERM (or SIGKILL when hard) to the process group proc leads."""
    try:
        if os.name == "posix":
            os.killpg(proc.pid, signal.SIGKILL if hard else signal.SIGTERM)
        elif hard:
            subprocess.run(["taskkill", "/T", "/F", "/PID", str(proc.pid)], capture_output=True)
        else:
            proc.send_signal(signal.CTRL_BREAK_EVENT)  # Windows: the whole new process group
    except OSError:
        pass


def tail_lines(path, n=30, size=16384):
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - size))
            data = f.read()
    except OSError:
        return []
    return data.decode("utf-8", "replace").replace("\r", "").splitlines()[-n:]


def job_group_alive(pgid, proc_root=Path("/proc")):
    """True while a process of process group pgid runs one of our job scripts (JOB_SCRIPTS). The group's leader
    was reaped, so its number may belong to someone else's group by now: then it is not ours and never signalled.
    Without /proc (not Linux) it cannot be told: False."""
    try:
        entries = [e for e in Path(proc_root).iterdir() if e.name.isdigit()]
    except OSError:
        return False
    for entry in entries:
        try:
            fields = (entry / "stat").read_text().rpartition(")")[2].split()
            if int(fields[2]) != pgid:
                continue
            argv = (entry / "cmdline").read_bytes().split(b"\0")
        except (OSError, ValueError, IndexError):
            continue  # gone meanwhile, or unreadable
        if any(Path(a.decode("utf-8", "replace")).name in JOB_SCRIPTS for a in argv if a):
            return True
    return False


def ask_run_end(url=RUN_URL, fetch=None):
    """POST {"end": true} to run_follow's page. OSError when it does not answer or refuses (any non-2xx)."""
    code, body = (fetch or http_json)(url + "/control", {"end": True}, timeout=1.5)
    if not 200 <= code < 300:
        raise OSError("the run's page answered HTTP %d (%s)" % (code, body))


class _Refused(Exception):
    """A job start refused with a reason for the page."""


class Jobs:
    """One job at a time (run_follow or a check tool), each the leader of its own process group.

    release() frees the camera and LiDAR before the job starts; reclaim() takes them back after it
    ended. ask_end() asks a running task to end itself (run_follow's POST /control {"end": true});
    stop_wheels() sends S# after a SIGKILL, when safe_run may not have had the chance.

    STOP always does something: with a job, its stop sequence (one at a time; pressed again, the same sequence
    goes to SIGKILL at once); without one, the sweep: ask run_follow's page to end (a run started by hand), end
    what is left of the last job's process group (its guard died, a child lives), send S#. A STOP during a start
    (while the camera and LiDAR close) or within cancel_s before it keeps that job from starting."""

    def __init__(self, folder=JOB_LOGS, release=lambda: None, reclaim=lambda: None, ask_end=None,
                 stop_wheels=None, popen=subprocess.Popen, clock=time.monotonic, sleep=time.sleep,
                 end_grace=END_GRACE, term_grace=TERM_GRACE, cancel_s=CANCEL_S, group_alive=job_group_alive,
                 signal=signal_group):
        self.folder = Path(folder)
        self.release, self.reclaim, self.ask_end, self.stop_wheels = release, reclaim, ask_end, stop_wheels
        self.popen, self.clock, self.sleep = popen, clock, sleep
        self.end_grace, self.term_grace, self.cancel_s = end_grace, term_grace, cancel_s
        self.group_alive, self.signal = group_alive, signal
        self.lock = threading.Lock()
        self.busy = False  # from start() until the job ended and the devices are back
        self.proc = None
        self.job = None  # the running or last job, for the page
        self.watcher = None
        self.stopped = threading.Event()  # set when a STOP's whole sequence (S# included) is over
        self.hurry = threading.Event()  # a second STOP: the sequence goes to SIGKILL now
        self.sequence = threading.Lock()  # one stop sequence or sweep at a time
        self.cancel_at = None  # the last STOP

    def _cancelled(self, started):
        at = self.cancel_at
        return at is not None and (at >= started or self.clock() - at < self.cancel_s)

    def start(self, kind, label, cmd, info=None):
        """None, or why the job was not started. Whatever fails, the devices come back and the console is free."""
        with self.lock:
            if self.busy:
                return "%s is running: stop it first" % (self.job["label"] if self.job else "another job")
            self.busy = True
            started = self.clock()
        out = None
        try:
            if self._cancelled(started):
                raise _Refused("STOP was pressed just now: wait a few seconds, then start again")
            try:
                self.release()
            except Exception as error:
                raise _Refused("could not free the camera and LiDAR: %s" % error)
            self.folder.mkdir(parents=True, exist_ok=True)
            path = self.folder / ("%s-%s.log" % (time.strftime("%Y%m%d-%H%M%S"), kind))
            out = open(path, "ab")
            out.write(("$ %s\n" % " ".join(cmd)).encode())
            out.flush()
            group = {"start_new_session": True} if os.name == "posix" else {
                "creationflags": subprocess.CREATE_NEW_PROCESS_GROUP}
            with self.lock:  # a STOP sees either no job (and cancels this one) or the started one
                if self._cancelled(started):
                    raise _Refused("STOP was pressed while %s was starting: not started" % label)
                proc = self.popen(cmd, cwd=str(HERE), stdin=subprocess.DEVNULL, stdout=out,
                                  stderr=subprocess.STDOUT, env=dict(os.environ, PYTHONUNBUFFERED="1",
                                                                     **{console_gate.ENV: path.stem}), **group)
                self.proc = proc
                self.stopped, self.hurry = threading.Event(), threading.Event()
                self.job = {"kind": kind, "label": label, "info": info or {}, "pid": proc.pid, "log": str(path),
                            "started": self.clock(), "ended": None, "code": None, "stopping": None, "notes": []}
        except Exception as error:
            if out is not None:
                out.close()
            self._reclaim()
            with self.lock:
                self.busy = False
            return "%s" % error if isinstance(error, _Refused) else "could not start %s: %s" % (label, error)
        self.watcher = threading.Thread(target=self._watch, args=(proc, out, self.job, self.stopped), daemon=True)
        self.watcher.start()
        return None

    def _reclaim(self):
        try:
            self.reclaim()
        except Exception:
            pass

    def _watch(self, proc, out, job, stopped):
        code = proc.wait()
        out.close()
        job["code"], job["ended"] = code, self.clock()
        if job["stopping"] is not None:
            stopped.wait(15)  # the devices come back only after the STOP sequence, S# included
        self._reclaim()
        with self.lock:
            self.busy = False

    def _alive(self, proc):
        return proc.poll() is None

    def _wait(self, proc, seconds, hurry=None, at_least=0.0):
        """Until proc ended, seconds passed, or hurry is set (but not before at_least seconds)."""
        start = self.clock()
        end = start + seconds
        while self._alive(proc) and self.clock() < end and not (
                hurry is not None and hurry.is_set() and self.clock() - start >= at_least):
            self.sleep(0.05)
        return not self._alive(proc)

    def _note(self, job, text):
        job["notes"].append("%s %s" % (time.strftime("%H:%M:%S"), text))

    def stop(self, wait=False):
        """STOP. None while a job is being stopped, else a note of the sweep. Never refuses."""
        with self.lock:
            self.cancel_at = self.clock()
            proc, job = self.proc, self.job
            alive = proc is not None and self._alive(proc)
            if alive:
                again = job["stopping"] is not None
                job["stopping"] = job["stopping"] or self.clock()
                stopped, hurry = self.stopped, self.hurry
        if alive and again:
            hurry.set()  # the sequence already running goes to SIGKILL; it alone sends S#
            if wait:
                stopped.wait(15)
            return None
        if alive:
            run, args = self._sequence, (proc, job, stopped, hurry)
        else:
            run, args = self._sweep, (proc, job)
        if wait:
            run(*args)
        else:
            threading.Thread(target=run, args=args, daemon=True).start()
        return None if alive else "nothing was running: asked the run's page to end, and sent S#"

    def _sequence(self, proc, job, stopped, hurry):
        with self.sequence:
            try:
                self._stop(proc, job, hurry)
            finally:
                stopped.set()

    def _sweep(self, proc, job):
        if not self.sequence.acquire(blocking=False):
            return  # a stop sequence is still running: it sends S# itself
        try:
            if self.ask_end is not None:
                try:
                    self.ask_end()  # a run_follow started by hand, or one whose guard is gone
                except Exception:
                    pass
            if proc is not None and self.group_alive(proc.pid):
                if job is not None:
                    self._note(job, "STOP: the job's group was still running: SIGTERM")
                self.signal(proc, hard=False)
                end = self.clock() + self.term_grace
                while self.group_alive(proc.pid) and self.clock() < end:
                    self.sleep(0.05)
                if self.group_alive(proc.pid):
                    self.signal(proc, hard=True)
                    self.sleep(0.3)
            if self.stop_wheels is not None:
                try:
                    self.stop_wheels()
                except Exception:
                    pass  # the port is held (a run started by hand stops its own wheels) or absent
        finally:
            self.sequence.release()

    def _stop(self, proc, job, hurry):
        if job["kind"] == "task" and self.ask_end is not None:
            try:
                self.ask_end()
                self._note(job, "asked the run to end")
                if self._wait(proc, self.end_grace, hurry, at_least=HURRY_MIN):  # its own S# first
                    return
            except Exception as error:  # nothing to wait for: straight to SIGTERM
                self._note(job, "the run's page did not answer (%s)" % error)
        if not self._alive(proc):
            return
        if not hurry.is_set():
            self.signal(proc, hard=False)
            self._note(job, "sent SIGTERM to the job")
            if self._wait(proc, self.term_grace, hurry):
                return
        self._kill(proc, job)

    def _kill(self, proc, job):
        if not self._alive(proc):
            return
        self.signal(proc, hard=True)
        self._note(job, "killed the job (SIGKILL)")
        self._wait(proc, 2.0)
        self.sleep(0.3)  # the rest of the group goes with the leader; let the wheel port close
        if self.stop_wheels is not None:
            try:
                self.stop_wheels()
                self._note(job, "S# sent by the console")
            except Exception as error:
                self._note(job, "COULD NOT SEND S# (%s): press the motor cut-off" % error)

    def shutdown(self):
        """The console is ending: stop the job, and wait for it."""
        self.reclaim = lambda: None
        self.stop(wait=True)
        if self.watcher is not None:
            self.watcher.join(5)

    def status(self, lines=30):
        with self.lock:
            job = dict(self.job) if self.job else None
            busy = self.busy
        if job is None:
            return {"running": False, "job": None, "tail": []}
        job["notes"] = list(job["notes"])
        end = job["ended"] or self.clock()
        job["elapsed"] = round(end - job["started"], 1)
        return {"running": busy, "job": job, "tail": tail_lines(job["log"], lines)}


# ---------------------------------------------------------------- devices

def _open_camera():
    import camera
    return camera.Camera()


class ChildCamera:
    """The camera in a child process (tools/camera_pipe.py): same start / latest / error / stop as
    camera.Camera, but latest() gives JPEG bytes. OpenNI stopped and started again inside the console
    crashed it twice (Segmentation fault, 2026-11-15 03:23 and 03:28, as jobs took the camera); a crash
    now ends only the child."""

    def __init__(self, argv=None, term_s=3.0, err_path=None):
        self.argv = argv or [sys.executable, str(TOOLS / "camera_pipe.py")]
        self.term_s = term_s
        # the child's stderr goes to a file: an unread pipe would fill with OpenNI warnings and block it
        self.err_path = Path(err_path) if err_path else HERE / "logs" / "console" / "camera_pipe.err"
        self.proc = None
        self.error = None
        self._jpeg = None
        self._lock = threading.Lock()
        self._thread = None

    def start(self):
        self.err_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.err_path, "wb") as err:
            self.proc = subprocess.Popen(self.argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                         stderr=err, cwd=str(HERE))
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    def _read(self):
        out = self.proc.stdout
        try:
            while True:
                head = out.read(4)
                if len(head) < 4:
                    break
                size = int.from_bytes(head, "big")
                body = out.read(size)
                if len(body) < size:
                    break
                with self._lock:
                    self._jpeg = body
        except (OSError, ValueError):
            pass
        code = self.proc.wait()
        try:
            tail = self.err_path.read_bytes()[-2000:].decode("utf-8", "replace").strip().splitlines()
        except OSError:
            tail = []
        self.error = RuntimeError("camera process ended (code %s)%s" % (code, ": " + tail[-1] if tail else ""))

    def latest(self):
        with self._lock:
            jpeg, self._jpeg = self._jpeg, None
        return jpeg

    def stop(self):
        if self.proc is None:
            return
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(self.term_s)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(5)
        if self._thread is not None:
            self._thread.join(5)


def _open_child_camera():
    return ChildCamera()


def _as_is(jpeg):
    return jpeg


def _open_lidar():
    import lidar
    return lidar.LidarSession()


def _open_pose_link(p):
    import link
    return link.JetsonLink(link.hosts(p), p["jetson_port"], p["camera_hfov_deg"], encode=lambda f: f.jpeg)


def jpeg_size(jpeg):
    """(width, height) of JPEG bytes."""
    import cv2
    import numpy as np
    image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError("not a JPEG")
    return image.shape[1], image.shape[0]


class Devices:
    """The camera and LiDAR readers of the live view. stop() closes both before a job; start() reopens.

    With skeleton on, the shown frames also go to the Jetson detector with pose, SKELETON_HZ a second. The
    detector serves one client and a new one replaces the old, so this link lives only while the devices
    are open: a job (run_follow) takes the camera and then the detector link for itself."""

    def __init__(self, p, open_camera=_open_child_camera, open_lidar=_open_lidar, encode=_as_is,
                 points=None, clock=time.monotonic, retry=RETRY, open_link=_open_pose_link, size_of=jpeg_size):
        self.p = p
        self.open_camera, self.open_lidar, self.encode = open_camera, open_lidar, encode
        self.open_link, self.size_of = open_link, size_of
        self.skeleton_on = False
        self.skeleton = {"people": [], "w": None, "h": None, "at": None, "error": None}
        self.points = points or self._scan_points
        self.clock, self.retry = clock, retry
        self.latest, self.scan = camera_view.Latest(), camera_view.Scan()
        self.held = True  # nothing open until start()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.threads = []
        self.camera = {"error": "not opened yet", "at": None}
        self.lidar = {"error": "not opened yet", "at": None}

    def _scan_points(self, scan):
        import run_follow
        return [tuple(q) for q in run_follow.scan_points(scan, self.p)]

    def start(self):
        with self.lock:
            if self.threads:
                return
            self.stop_event = event = threading.Event()
            self.held = False
            self.camera = {"error": "opening", "at": None}
            self.lidar = {"error": "opening", "at": None}
            self.threads = [threading.Thread(target=self._camera_loop, args=(event,), daemon=True),
                            threading.Thread(target=self._lidar_loop, args=(event,), daemon=True),
                            threading.Thread(target=self._skeleton_loop, args=(event,), daemon=True)]
            for t in self.threads:
                t.start()

    def stop(self):
        """Close the camera and the LiDAR; returns when both are closed."""
        with self.lock:
            self.held = True
            self.stop_event.set()
            threads, self.threads = self.threads, []
        for t in threads:
            t.join(10)
            if t.is_alive():
                raise RuntimeError("the %s reader did not close" % ("camera", "LiDAR", "skeleton")[threads.index(t)])

    def _camera_loop(self, event):
        while not event.is_set():
            try:
                cam = self.open_camera()
            except Exception as error:
                self.camera = {"error": "%s" % (error,), "at": None}
                event.wait(self.retry)
                continue
            try:
                cam.start()
                while not event.is_set():
                    frame = cam.latest()
                    if frame is not None:
                        self.latest.put(self.encode(frame))
                        self.camera = {"error": None, "at": self.clock()}
                    elif cam.error is not None:
                        raise cam.error
                    event.wait(0.005)
            except Exception as error:
                self.camera = {"error": "%s" % (error,), "at": self.camera.get("at")}
            finally:
                try:
                    cam.stop()
                except Exception:
                    pass
            event.wait(self.retry)

    def _lidar_loop(self, event):
        while not event.is_set():
            try:
                session = self.open_lidar()
            except Exception as error:
                self.lidar = {"error": "%s" % (error,), "at": None}
                event.wait(self.retry)
                continue
            try:
                last = None
                while not event.is_set():
                    s = session.latest()
                    if s and s is not last:
                        self.scan.put(self.points(s), self.clock())
                        self.lidar = {"error": None, "at": self.clock()}
                        last = s
                    event.wait(0.02)
            except Exception as error:
                self.lidar = {"error": "%s" % (error,), "at": self.lidar.get("at")}
            finally:
                try:
                    session.close()
                except Exception:
                    pass
            event.wait(self.retry)

    def _skeleton_loop(self, event):
        link, n = None, 0
        try:
            while not event.is_set():
                if not self.skeleton_on:
                    if link is not None:
                        link.stop()
                        link = None
                    self.skeleton = dict(self.skeleton, people=[], at=None)
                    event.wait(0.2)
                    continue
                try:
                    if link is None:
                        link = self.open_link(self.p)
                        link.start()
                    jpeg, m = self.latest.jpeg, self.latest.n
                    if jpeg is not None and m != n:
                        n = m
                        w, h = self.size_of(jpeg)
                        self.skeleton.update(w=w, h=h)
                        link.send(types.SimpleNamespace(time=self.clock(), width=w, height=h, jpeg=jpeg), pose=True)
                    answer = link.latest()
                    if answer is not None:
                        people = [{"box": s.box, "kp": s.kp} for s in answer.people]
                        self.skeleton.update(people=people, at=self.clock(), error=None)
                    elif link.error:
                        self.skeleton["error"] = link.error
                except Exception as error:
                    self.skeleton["error"] = "%s" % (error,)
                event.wait(1 / SKELETON_HZ)
        finally:
            if link is not None:
                link.stop()

    def skeleton_status(self):
        s = self.skeleton
        age = None if s["at"] is None else round(self.clock() - s["at"], 2)
        return {"on": self.skeleton_on, "held": self.held, "w": s["w"], "h": s["h"], "age": age,
                "people": s["people"] if age is not None and age < LIVE_AGE else [], "error": s["error"]}

    def status(self):
        now = self.clock()

        def one(d):
            age = None if d["at"] is None else round(now - d["at"], 1)
            if self.held:
                state = "held"
            elif age is not None and age < LIVE_AGE:
                state = "live"
            elif age is not None:
                state = "stale"
            else:
                state = "off"
            return {"state": state, "age": age, "error": d["error"]}
        return {"held": self.held, "camera": one(self.camera), "lidar": one(self.lidar)}


# ---------------------------------------------------------------- the Jetson, the Pi, the logs

def probe(host, port=AGENT_PORT, timeout=0.8):
    """{"host", "ok", "ms"}: a TCP connect; refused still means the host answered."""
    t = time.perf_counter()
    try:
        with socket.create_connection((host, port), timeout):
            pass
        ok = True
    except ConnectionRefusedError:
        ok = True
    except OSError:
        ok = False
    return {"host": host, "ok": ok, "ms": round((time.perf_counter() - t) * 1000, 1) if ok else None}


def port_open(host, port, timeout=0.8):
    try:
        with socket.create_connection((host, port), timeout):
            return True
    except OSError:
        return False


def http_json(url, body=None, timeout=2.0):
    """(status, JSON body) of a GET, or a POST when body is given. OSError when nothing answered."""
    data = None if body is None else json.dumps(body).encode()
    # X-HMC-Console: run_follow's dashboard takes a POST only with it (dashboard.CONSOLE_HEADER)
    req = urllib.request.Request(url, data, {"Content-Type": "application/json", "X-HMC-Console": "1"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read() or b"null")
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read() or b"null")
        except ValueError:
            return e.code, {"error": "HTTP %d" % e.code}


class Jetson:
    """Reachability of both Jetson addresses, the agent's status (proxied) and the arm webcam."""

    def __init__(self, hosts=JETSON_HOSTS, fetch=http_json, probe=probe, port_open=port_open, clock=time.time):
        self.hosts, self.fetch, self.probe, self.port_open, self.clock = hosts, fetch, probe, port_open, clock
        self.host = None  # the address the agent last answered on
        self.links = [{"host": h, "ok": None, "ms": None} for h in hosts]
        self.agent, self.agent_error, self.agent_at = None, "not asked yet", None
        self.arm_state, self.arm_at = None, None  # last GET /arm, cached with the 3 s checker
        self.webcam = None
        self.checked = None

    def order(self):
        reachable = [link["host"] for link in self.links if link["ok"]]
        hosts = ([self.host] if self.host else []) + reachable + list(self.hosts)
        return list(dict.fromkeys(hosts))

    def refresh(self):
        self.links = [self.probe(h) for h in self.hosts]
        for host in self.order():
            if not any(link["host"] == host and link["ok"] for link in self.links):
                continue
            try:
                code, body = self.fetch("http://%s:%d/status" % (host, AGENT_PORT), timeout=2.0)
            except (OSError, ValueError) as error:
                self.agent_error = "no answer from agent.py on %s (%s)" % (host, error)
                continue
            if code == 200 and isinstance(body, dict):
                self.host, self.agent, self.agent_error, self.agent_at = host, body, None, self.clock()
                break
        else:
            self.agent = None
            if not any(link["ok"] for link in self.links):
                self.agent_error = "the Jetson does not answer on %s" % " or ".join(self.hosts)
        host = self.host or next((link["host"] for link in self.links if link["ok"]), None)
        self.webcam = host is not None and self.port_open(host, WEBCAM_PORT)
        if self.host:
            try:
                code, body = self.arm()
            except (OSError, ValueError):
                code, body = 0, None
            if code == 200 and isinstance(body, dict):
                self.arm_state, self.arm_at = body, self.clock()
        self.checked = self.clock()

    def act(self, action, name=None):
        if action not in JETSON_ACTIONS:
            return 404, {"error": "unknown action"}
        last = "the Jetson does not answer"
        for host in self.order():
            try:
                code, body = self.fetch("http://%s:%d/%s" % (host, AGENT_PORT, action), {"name": name}, timeout=8.0)
            except (OSError, ValueError) as error:
                last = "no answer from agent.py on %s (%s)" % (host, error)
                continue
            self.host = host
            if isinstance(body, dict) and isinstance(body.get("status"), dict):
                self.agent, self.agent_error, self.agent_at = body["status"], None, self.clock()
            return code, body
        return 502, {"error": last}

    def arm(self, body=None, timeout=None):
        """(code, body) of the agent's GET /arm, or its POST /arm with body. A command goes to the next address only
        when this one could not be reached: one sent and not answered in time is never sent a second time."""
        last = "the Jetson does not answer"
        for host in self.order():
            try:
                code, answer = self.fetch("http://%s:%d/arm" % (host, AGENT_PORT), body,
                                          timeout=timeout or (2.0 if body is None else ARM_TIMEOUT))
            except urllib.error.URLError as error:  # no connection: the command was never sent
                last = "no answer from agent.py on %s (%s)" % (host, error.reason)
                continue
            except (OSError, ValueError) as error:
                return 504, {"error": "agent.py on %s did not answer in time (%s)" % (host, error)}
            self.host = host
            return code, answer
        return 502, {"error": last}

    def flow(self, body, timeout=FLOW_TIMEOUT):
        """(code, body) of the agent's POST /flow (the parked flow program: own, heard, say, release), sent as arm
        sends a command: to the next address only when this one could not be reached, never a second time."""
        last = "the Jetson does not answer"
        for host in self.order():
            try:
                code, answer = self.fetch("http://%s:%d/flow" % (host, AGENT_PORT), body, timeout=timeout)
            except urllib.error.URLError as error:  # no connection: the call was never sent
                last = "no answer from agent.py on %s (%s)" % (host, error.reason)
                continue
            except (OSError, ValueError) as error:
                return 504, {"error": "agent.py on %s did not answer in time (%s)" % (host, error)}
            self.host = host
            return code, answer
        return 502, {"error": last}

    def webcam_url(self):
        return "http://%s:%d/stream" % (self.host or self.hosts[0], WEBCAM_PORT)

    def webcam_snap(self):
        """(code, body, content type) of one JPEG from the arm webcam's /snap.jpg. The page shows it with an img tag."""
        url = "http://%s:%d/snap.jpg" % (self.host or self.hosts[0], WEBCAM_PORT)
        try:
            with urllib.request.urlopen(url, timeout=1.5) as r:
                data = r.read()
                kind = (r.headers.get("Content-Type") or "image/jpeg").split(";")[0]
        except (OSError, ValueError) as error:
            return 503, {"error": "the arm webcam snapshot does not answer (%s)" % error}, "application/json"
        if not data:
            return 502, {"error": "the arm webcam snapshot was empty"}, "application/json"
        return 200, data, kind or "image/jpeg"

    def snapshot(self):
        return {"host": self.host, "links": self.links, "agent": self.agent, "agent_error": self.agent_error,
                "agent_at": self.agent_at, "webcam": self.webcam, "checked": self.checked,
                "arm": self.arm_state, "arm_at": self.arm_at}


def link_test(hosts=JETSON_HOSTS, tries=3, ports=(AGENT_PORT, 22), probe=probe):
    """TCP connect times (ms, None for no answer) to every address and port, tries times each, in parallel."""
    out = [{"host": h, "port": port, "ms": []} for h in hosts for port in ports]

    def one(item):
        item["ms"] = [probe(item["host"], item["port"], timeout=1.0)["ms"] for _ in range(tries)]
        item["ok"] = any(t is not None for t in item["ms"])
    threads = [threading.Thread(target=one, args=(item,)) for item in out]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    return out



def deployed_version(path=None):
    """The commit base/tools/deploy.sh wrote next to this file ({"commit", "describe", "branch", "deployed"}),
    or None (never deployed that way, or an unreadable file)."""
    try:
        with open(path or HERE / "VERSION.json", encoding="utf-8") as f:
            v = json.load(f)
    except (OSError, ValueError):
        return None
    return v if isinstance(v, dict) and isinstance(v.get("commit"), str) else None


def pi_health(root=Path("/"), disk=HERE):
    out = {"temp_c": None, "load": None, "disk_free_gb": None, "uptime_s": None}
    try:
        out["temp_c"] = round(int((root / "sys/class/thermal/thermal_zone0/temp").read_text().strip()) / 1000, 1)
    except (OSError, ValueError):
        pass
    try:
        out["load"] = [float(x) for x in (root / "proc/loadavg").read_text().split()[:3]]
    except (OSError, ValueError):
        pass
    try:
        out["uptime_s"] = round(float((root / "proc/uptime").read_text().split()[0]))
    except (OSError, ValueError, IndexError):
        pass
    try:
        out["disk_free_gb"] = round(shutil.disk_usage(disk).free / 1e9, 1)
    except OSError:
        pass
    return out


def wheel_test_last(path=WHEEL_TEST_LAST):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def _ago(seconds):
    if seconds < 90:
        return "%d วิ. ที่แล้ว" % seconds
    if seconds < 5400:
        return "%d นาทีที่แล้ว" % (seconds / 60)
    if seconds < 2 * 86400:
        return "%.1f ชม. ที่แล้ว" % (seconds / 3600)
    return "%d วันที่แล้ว" % (seconds / 86400)


def _duration(seconds):
    seconds = int(seconds)
    if seconds < 3600:
        return "%d:%02d" % (seconds // 60, seconds % 60)
    return "%d ชม. %02d นาที" % (seconds // 3600, seconds % 3600 // 60)


WHEEL_TEXT = {  # tools/wheel_test.py's verdicts, in Thai for the page
    "ok": "ล้อหมุนทั้งสองข้าง และ encoder นับทั้งคู่",
    "no_power": "ล้อไม่มีไฟเข้า ตรวจแบต 21V ว่าเสียบอยู่และมีไฟ",
    "one_encoder": "ล้อข้างหนึ่งไม่นับ encoder ตรวจ encoder หรือสายมอเตอร์ของข้างนั้น",
    "no_answer": "บอร์ดมอเตอร์ไม่ตอบ ตรวจสาย USB ไปบอร์ดมอเตอร์ และดูว่าบอร์ดเปิดอยู่",
    "unclear": "firmware ไม่ได้เพิ่ม PWM ทดสอบใหม่อีกครั้ง แล้วดูค่าที่อ่านได้",
    "refused": "ต้องเริ่มผ่าน tools/safe_run.py",
}
WHEEL_WORD = {"ok": "ผ่าน", "no_power": "ไม่มีไฟ", "one_encoder": "encoder ข้างเดียว", "no_answer": "ไม่ตอบ",
              "unclear": "ไม่ชัด", "refused": "ถูกปฏิเสธ"}


def cards(pi, devices, wheel, jetson, now):
    """The Overview's health cards: [{"id", "title", "level": ok|warn|bad|idle, "value", "text"}]."""
    out = []
    # Pi
    level, texts = "ok", []
    t, disk = pi["temp_c"], pi["disk_free_gb"]
    if t is not None and t >= 80:
        level, texts = "bad", texts + ["Pi ร้อนมากและช้าลง ให้มีลมผ่าน ดูพัดลม"]
    elif t is not None and t >= 70:
        level, texts = "warn", texts + ["Pi อุ่น อย่าให้อะไรบังพัดลม"]
    if disk is not None and disk < 0.5:
        level, texts = "bad", texts + ["SD card ใกล้เต็ม เขียน log ไม่ได้ ลบงานเก่าออก"]
    elif disk is not None and disk < 2:
        level = "bad" if level == "bad" else "warn"
        texts.append("SD card เหลือที่น้อย ควรลบงานเก่าเร็วๆ นี้")
    parts = []
    if t is not None:
        parts.append("%.0f °C" % t)
    if pi["load"]:
        parts.append("โหลด %.1f" % pi["load"][0])
    if disk is not None:
        parts.append("ว่าง %.1f GB" % disk)
    if pi["uptime_s"] is not None:
        parts.append("เปิดมา " + _duration(pi["uptime_s"]))
    if t is None and pi["load"] is None:
        level, texts = "idle", ["อ่านอุณหภูมิและโหลดไม่ได้ (ไม่ใช่ Pi หรือเปล่า)"]
    out.append({"id": "pi", "title": "Raspberry Pi", "level": level, "value": " · ".join(parts) or "-",
                "text": " · ".join(texts) or "อุณหภูมิ โหลด และพื้นที่ดิสก์ปกติ"})
    # LiDAR and camera
    for key, title, cable in (("lidar", "LiDAR", "สาย USB ของ LiDAR (/dev/rct_ahr_lidar)"),
                              ("camera", "กล้อง Xtion", "สาย USB ของ Xtion (พอร์ต USB 2 ของ Pi)")):
        d = devices[key]
        if d["state"] == "held":
            card = ("idle", "งานกำลังใช้อยู่", "งานที่ทำอยู่ใช้อยู่ เปิดได้ทีละโปรแกรม")
        elif d["state"] == "live":
            card = ("ok", "สด", "ทำงานปกติ" if key == "camera" else "กำลังสแกน")
        elif d["state"] == "stale":
            card = ("warn", "ไม่มีข้อมูลใหม่ %.0f วิ." % d["age"], "หยุดส่งข้อมูล ตรวจ%s" % cable)
        else:
            card = ("bad", "ไม่พบ", "หาไม่เจอ ตรวจ%s (%s)" % (cable, d["error"] or "ไม่มีข้อมูล"))
        out.append({"id": key, "title": title, "level": card[0], "value": card[1], "text": card[2]})
    # wheels
    if wheel is None:
        out.append({"id": "wheels", "title": "ล้อและไฟมอเตอร์", "level": "warn", "value": "ยังไม่ทดสอบ",
                    "text": "ยังไม่เคยทดสอบล้อ ทดสอบได้ที่แท็บตรวจเช็ก (หุ่นขยับ 4 ซม.)"})
    else:
        age = now - wheel.get("time", 0)
        ok = wheel.get("verdict") == "ok"
        level = ("ok" if age < STALE_WHEEL_TEST else "warn") if ok else "bad"
        text = WHEEL_TEXT.get(wheel.get("verdict")) or wheel.get("text") or wheel.get("verdict", "?")
        if ok and age >= STALE_WHEEL_TEST:
            text += " · ทดสอบนานแล้ว ทดสอบใหม่ก่อนวิ่ง"
        verdict = wheel.get("verdict", "?")
        out.append({"id": "wheels", "title": "ล้อและไฟมอเตอร์", "level": level,
                    "value": "%s %s" % (WHEEL_WORD.get(verdict) or verdict.replace("_", " "), _ago(age)),
                    "text": text})
    # Jetson link
    links = jetson["links"]
    lan = next((l for l in links if l["host"] == JETSON_HOSTS[0]), None)
    up = [l for l in links if l["ok"]]
    value = " · ".join("%s %s" % (l["host"], ("%.0f ms" % l["ms"]) if l["ok"] else ("-" if l["ok"] is None else "ไม่ตอบ"))
                      for l in links)
    if not up:
        level, text = ("idle", "กำลังตรวจ") if all(l["ok"] is None for l in links) else (
            "bad", "Jetson ไม่ตอบ เปิดอยู่ไหม ตรวจสาย LAN และ Wi-Fi")
    elif lan is not None and lan["ok"]:
        level, text = "ok", "สาย LAN ใช้ได้ ใช้อยู่ %s" % (jetson["host"] or JETSON_HOSTS[0])
    else:
        level, text = "warn", "ใช้ได้แค่ Wi-Fi ตรวจสาย LAN (run_follow ใช้ 10.0.0.2)"
    out.append({"id": "link", "title": "ลิงก์ Jetson", "level": level, "value": value, "text": text})
    # Jetson services
    agent = jetson["agent"]
    if agent is None:
        out.append({"id": "services", "title": "โปรแกรมบน Jetson", "level": "bad" if up else "idle",
                    "value": "agent ไม่ตอบ",
                    "text": (jetson["agent_error"] or "") + " เปิด agent.py บน Jetson (ดูวิธีใช้)"})
    else:
        progs = agent.get("programs", {})
        stopped = [n for n, s in progs.items() if not s.get("running")]
        level = "ok" if not stopped else "warn"
        out.append({"id": "services", "title": "โปรแกรมบน Jetson", "level": level,
                    "value": "ทำงาน %d จาก %d" % (len(progs) - len(stopped), len(progs)),
                    "text": "detector bridge และ flow ทำงานอยู่" if not stopped
                    else "หยุดอยู่ %s เปิดได้ที่แท็บ Jetson" % " ".join(stopped)})
    # arm webcam
    cam = jetson["webcam"]
    out.append({"id": "webcam", "title": "เว็บแคมของแขน", "level": "idle" if cam is None else ("ok" if cam else "warn"),
                "value": "กำลังตรวจ" if cam is None else ("สด" if cam else "ปิดอยู่"),
                "text": "webcam_view ของทีมแขนบน Jetson" + ("" if cam or cam is None else
                                                            " ไม่ได้ทำงาน เป็นของทีมแขน ให้ถามเขา")})
    return out


def _first_record(path):
    try:
        with open(path, "rb") as f:
            return json.loads(f.readline())
    except (OSError, ValueError):
        return None


def _last_record(path):
    """The last complete JSON line of path, reading from the end."""
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            block = 65536
            while True:
                start = max(0, size - block)
                f.seek(start)
                lines = f.read(size - start).splitlines()
                for line in reversed(lines if start == 0 else lines[1:]):
                    try:
                        return json.loads(line)
                    except ValueError:
                        continue  # a half-written last line
                if start == 0:
                    return None
                block *= 4
    except OSError:
        return None


def voice_commands(root=LOG_ROOT, size=RUN_TAIL):
    """The voice flow's commands (run_follow's remote_cmd lines, and its remote_refused ones from the client) in the
    last `size` bytes of the newest run log, as command-log entries. Their "wall" is the time: a line from before
    run_follow wrote it has only its monotonic "time" and is left out."""
    try:
        newest = max((d for d in Path(root).iterdir() if RUN_NAME.match(d.name) and d.is_dir()), default=None)
    except OSError:
        return []
    out = []
    for text in tail_lines(newest / "log.jsonl", size, size) if newest else []:  # every line of the window
        if '"remote_cmd"' not in text and '"remote_refused"' not in text:
            continue  # the cycles: most of the log, never parsed
        try:
            r = json.loads(text)
        except ValueError:
            continue  # the first line, cut by the window
        if not isinstance(r, dict) or not isinstance(r.get("wall"), (int, float)):
            continue
        if "remote_cmd" in r:
            cmd, code, error = r["remote_cmd"], 200, None
        elif r.get("by") == "client":  # a refused page press is in the console's own log already
            cmd, code, error = r.get("remote_refused"), None, r.get("why") or "refused"
        else:
            continue
        out.append({"time": r["wall"], "client": VOICE_CLIENT, "address": None, "path": str(cmd), "body": None,
                    "code": code, "error": error})
    return out


def list_runs(root=LOG_ROOT, limit=20):
    """The newest run_follow log folders: [{"name", "start", "seconds", "end", "size"}]."""
    try:
        folders = sorted((d for d in Path(root).iterdir() if d.is_dir() and RUN_NAME.match(d.name)), reverse=True)
    except OSError:
        return []
    out = []
    for d in folders[:limit]:
        log = d / "log.jsonl"
        n = d.name
        item = {"name": n, "start": "%s-%s-%s %s:%s:%s" % (n[0:4], n[4:6], n[6:8], n[9:11], n[11:13], n[13:15]),
                "seconds": None, "end": None, "size": None, "has_log": log.is_file()}
        if item["has_log"]:
            item["size"] = log.stat().st_size
            first, last = _first_record(log), _last_record(log)
            if first and last and isinstance(first.get("time"), (int, float)) and isinstance(last.get("time"), (int, float)):
                item["seconds"] = round(last["time"] - first["time"], 1)
            if last is not None:
                item["end"] = last.get("end")
        out.append(item)
    return out


def load_pin(path=PIN_FILE):
    try:
        return Path(path).read_text().strip() or None
    except OSError:
        return None


def send_stop():
    """S# to the base from here, after a job was killed outright."""
    import params
    import wheels
    w = wheels.Wheels(params.load(), start_thread=False)  # opening sends S# three times
    w.close()  # and closing again


# ---------------------------------------------------------------- the Data tab: telemetry's database, read only

METRIC_NAME = re.compile(r"^[a-z0-9_]{1,64}$")
METRICS_MAX = 12  # metrics in one request
POINTS_DEFAULT, POINTS_MAX = 200, 1000
RUNS_MAX = 500
RUN_COLUMNS = ("t", "v_cmd", "v_meas", "w_cmd", "w_meas", "operator_m", "nearest_m", "locked")
DB_TIMEOUT = 2.0  # s a read waits for the recorder's write lock; the job manager never waits on any of this


def read_db(path):
    """A read-only connection to the telemetry database, or None when the file does not exist yet."""
    path = Path(path)
    if not path.is_file():
        return None
    return sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True, timeout=DB_TIMEOUT,
                           check_same_thread=False)


def _number_arg(query, name, default):
    """query[name] as a float (default when absent); ValueError for text or a non-finite number."""
    text = (query.get(name) or [""])[0].strip()
    if not text:
        return default
    value = float(text)
    if value != value or value in (float("inf"), float("-inf")):
        raise ValueError(name)
    return value


def _metrics_arg(query):
    names = [m for m in (query.get("metrics") or [""])[0].split(",") if m]
    if not names or len(names) > METRICS_MAX or not all(METRIC_NAME.match(m) for m in names):
        raise ValueError("metrics: 1 to %d names of a-z, 0-9 and _" % METRICS_MAX)
    return list(dict.fromkeys(names))


def _window(query, now):
    since = _number_arg(query, "since", now - 3600.0)
    until = _number_arg(query, "until", now)
    if until <= since:
        raise ValueError("until must be after since")
    return since, until


def series(db, metrics, since, until, points):
    """{metric: [[ts, value], ...]}: the samples in [since, until) averaged into at most `points` equal buckets
    (the mean time and the mean value of each bucket that has samples)."""
    out = {m: [] for m in metrics}
    if db is None:
        return out
    width = (until - since) / points
    rows = db.execute("SELECT metric, CAST((ts - ?) / ? AS INTEGER) AS b, avg(ts), avg(value) FROM samples"
                      " WHERE metric IN (%s) AND ts >= ? AND ts < ? AND value IS NOT NULL"
                      " GROUP BY metric, b ORDER BY metric, b" % ",".join("?" * len(metrics)),
                      [since, width] + metrics + [since, until]).fetchall()
    for metric, _, ts, value in rows:
        out[metric].append([round(ts, 1), round(value, 4)])
    return out


def _db_info(path):
    try:
        return {"db": Path(path).is_file(), "db_bytes": Path(path).stat().st_size}
    except OSError:
        return {"db": False, "db_bytes": None}


# ---------------------------------------------------------------- the web server

def _load_mission():
    """hmc.flow.HmcFlow. On the Pi the package sits next to this file; in the repo it is ../../hmc/src."""
    try:
        from hmc.flow import HmcFlow
        return HmcFlow
    except ImportError:
        pass
    for folder in (HERE.parent.parent / "hmc" / "src", HERE):
        if (folder / "hmc" / "flow.py").exists() and str(folder) not in sys.path:
            sys.path.insert(0, str(folder))
            break
    from hmc.flow import HmcFlow
    return HmcFlow


def _mission_status(body):
    """run_follow's status record as the executive reads it, or None when the page has no record yet."""
    if not isinstance(body, dict):
        return None
    record = body.get("record") if isinstance(body.get("record"), dict) else None
    if record is None and any(key in body for key in ("locked", "phase", "at_start", "end")):
        record = body
    if record is None:
        return None
    status = {"link": True}
    if "locked" in record:
        status["locked"] = bool(record.get("locked"))
    if "at_start" in record:
        status["at_start"] = bool(record.get("at_start"))
    end = record.get("end")
    if end:
        status["end"] = str(end)
    wheels = record.get("wheels")
    if isinstance(wheels, str) and wheels:
        status["wheels"] = wheels
    pose = record.get("believed") if isinstance(record.get("believed"), dict) else record.get("pose")
    if isinstance(pose, dict):
        status["pose"] = pose
    if isinstance(record.get("bag_task"), dict):
        status["bag_task"] = record["bag_task"]  # the executive does not read it: Console._bag_events does
    return status


_RUN_ACTION = {"follow_me": "follow", "wait": "wait", "resume": "resume", "go_back": "return", "stop": "wait"}


class Console:
    def __init__(self, p, devices, jobs, jetson, run_url=RUN_URL, fetch=http_json, pin=None, log_root=LOG_ROOT,
                 db_path=None):
        self.p, self.devices, self.jobs, self.jetson = p, devices, jobs, jetson
        self.run_url, self.fetch, self.pin, self.log_root = run_url, fetch, pin, log_root
        self.clock = time.monotonic
        self.wheel_at = None  # when the last wheel test started (clock)
        self.commander = None  # the token of the page tab that sent the last command to the run
        self.guard = threading.Lock()  # the PIN lockout and the data query count
        self.pin_fails, self.pin_locked_until, self.pin_lock_s = 0, None, PIN_LOCK_S
        self.data_running = 0  # /api/data/* queries now
        self.hold = None  # {"who", "since"} while a deploy or a repair holds the console; a restart clears it
        self.db_path = Path(db_path) if db_path is not None else (
            telemetry.DB_PATH if telemetry is not None else HERE / "data" / "robot.db")
        self.events = []  # {"time", "level", "text"}, oldest first, capped; the Robot page shows them newest first
        self.bag = None  # last measured bag, or None; never a hard-coded x
        self._bank, self._bank_at = None, 0.0
        self._drop = None
        self.mission = _load_mission()()
        self._mission_stop = threading.Event()
        self._mission_saw_run = False
        self._voice_on = False  # this run's say lines go to the parked Jetson flow (_own_flow, _voice_loop)
        self._voice_run = 0  # the run the voice thread serves: a newer run's start ends it
        self._says = queue.SimpleQueue()  # say lines for the voice thread, in order
        self._arm_lock = threading.Lock()  # the executive's pick and drop: one at a time (_arm_job)
        self._bag_seen = self._bag_asked = self._bag_said = None  # bag task ids (_bag_events)
        self._mission_thread = threading.Thread(target=self._mission_loop, daemon=True)
        self._mission_thread.start()

    def data(self, path, query):
        """(code, body[, content type, headers]) of GET /api/data/series, /runs, /run/<id>, /export.csv and
        /api/bank (the power bank's estimate, and "motor": the 21V battery's use). Read-only queries on the telemetry database; a database not made yet gives empty answers."""
        if telemetry is None:
            return 503, {"error": "telemetry.py is not next to console.py: no data to show", "telemetry": False}
        if path == "/api/data/runs":
            self._import_finished_runs()
        if not path.startswith("/api/data/"):
            return self._query(path, query)
        job = self.jobs.status()
        info = (job.get("job") or {}).get("info") or {}
        limit = 1 if job["running"] and (info.get("drive") or info.get("moves")) else DATA_QUERIES
        with self.guard:
            if self.data_running >= limit:
                return 429, {"error": "ข้อมูลกำลังโหลดอยู่ รอสักครู่แล้วลองใหม่" + (" (หุ่นกำลังขับ)" if limit == 1 else "")}
            self.data_running += 1
        try:
            return self._query(path, query)
        finally:
            with self.guard:
                self.data_running -= 1

    def _query(self, path, query):
        now = time.time()
        try:
            db = read_db(self.db_path)
        except sqlite3.Error as error:
            return 503, {"error": "the telemetry database cannot be opened (%s)" % error}
        try:
            return self._data(db, path, query, now)
        except ValueError as error:
            return 400, {"error": "%s" % error}
        except sqlite3.Error as error:
            return 503, {"error": "the telemetry database cannot be read now (%s)" % error}
        finally:
            if db is not None:
                db.close()

    def _data(self, db, path, query, now):
        info = _db_info(self.db_path)
        if path == "/api/data/series":
            metrics = _metrics_arg(query)
            since, until = _window(query, now)
            points = max(2, min(POINTS_MAX, int(_number_arg(query, "points", POINTS_DEFAULT))))
            return 200, dict(info, since=since, until=until, points=points, bucket_s=(until - since) / points,
                             series=series(db, metrics, since, until, points))
        if path == "/api/data/runs":
            limit = max(1, min(RUNS_MAX, int(_number_arg(query, "limit", 50))))
            if db is None:
                return 200, dict(info, runs=[], total={"runs": 0, "distance_m": 0.0})
            cursor = db.execute("SELECT run_id, started, ended, seconds, end_reason, end_detail, drive, cycles,"
                                " distance_m, max_v, mean_v_moving, locked_pct FROM runs"
                                " ORDER BY started DESC, run_id DESC LIMIT ?", (limit,))
            names = [d[0] for d in cursor.description]
            runs = [dict(zip(names, row)) for row in cursor]
            count, distance = db.execute("SELECT count(*), coalesce(sum(distance_m), 0) FROM runs").fetchone()
            return 200, dict(info, runs=runs, total={"runs": count, "distance_m": distance})
        if path.startswith("/api/data/run/"):
            run_id = path[len("/api/data/run/"):]
            points = max(2, min(POINTS_MAX, int(_number_arg(query, "points", 400))))
            row = None if db is None or not RUN_NAME.match(run_id) else db.execute(
                "SELECT run_id, started, ended, seconds, end_reason, end_detail, drive, distance_m, max_v"
                " FROM runs WHERE run_id = ?", (run_id,)).fetchone()
            if row is None:
                return 404, {"error": "no such run"}
            run = dict(zip(("run_id", "started", "ended", "seconds", "end_reason", "end_detail", "drive",
                            "distance_m", "max_v"), row))
            rows = db.execute("SELECT %s FROM run_samples WHERE run_id = ? ORDER BY t" % ", ".join(RUN_COLUMNS),
                              (run_id,)).fetchall()
            step = max(1, -(-len(rows) // points))
            kept = rows[::step]
            samples = {c: [None if r[i] is None else round(r[i], 4) for r in kept] for i, c in enumerate(RUN_COLUMNS)}
            return 200, {"run": run, "samples": samples, "rows": len(rows), "step": step}
        if path == "/api/data/export.csv":
            metrics = _metrics_arg(query)
            since, until = _window(query, now)
            if db is None:
                data = ("ts,time,%s\r\n" % ",".join(metrics)).encode()
            else:
                fd, tmp = tempfile.mkstemp(suffix=".csv")  # export_csv writes a file: the system temp folder
                os.close(fd)
                try:
                    telemetry.export_csv(db, metrics, since, until, tmp)
                    data = Path(tmp).read_bytes()
                finally:
                    os.unlink(tmp)
            name = "robot-%s-%s.csv" % (time.strftime("%Y%m%d-%H%M", time.localtime(since)),
                                        time.strftime("%Y%m%d-%H%M", time.localtime(until)))
            return 200, data, "text/csv; charset=utf-8", [("Content-Disposition", 'attachment; filename="%s"' % name)]
        if path == "/api/bank":
            if db is None:
                bank = dict.fromkeys(("percent", "low", "high", "wh_used", "mark_ts", "mark_kind", "mark_percent",
                                      "hours_on", "driving_min", "warning"))
                full = None
                motor = {"since": None, "wh_used": None, "drive_minutes": None}
            else:
                bank = telemetry.estimate(db, now)
                full = db.execute("SELECT max(ts) FROM charge_marks WHERE kind = 'full' AND battery = 'bank'"
                                  ).fetchone()[0]
                motor = telemetry.motor_usage(db, now)
            return 200, dict(bank, **info, full_ts=full, warn_pct=telemetry.WARN_PCT, now=now, motor=motor)
        return 404, {"error": "not found"}

    def bank_post(self, path, body):
        """(code, body) of POST /api/bank/full {"battery": "bank" (the default) or "motor"} and /api/bank/reading
        {"percent": 0..100} (the power bank only): the owner's charge marks."""
        if telemetry is None:
            return 503, {"error": "telemetry.py is not next to console.py", "telemetry": False}
        percent = body.get("percent")
        if path == "/api/bank/reading" and (isinstance(percent, bool) or not isinstance(percent, (int, float))
                                            or not 0 <= percent <= 100):
            return 400, {"error": "percent must be a number from 0 to 100"}
        battery = body.get("battery", "bank")
        if path == "/api/bank/full" and not (isinstance(battery, str) and battery in ("bank", "motor")):
            return 400, {"error": "battery must be \"bank\" or \"motor\""}
        try:
            db = telemetry.open_db(self.db_path)
            try:
                if path == "/api/bank/full":
                    telemetry.mark_full(db, note="console", battery=battery)
                else:
                    telemetry.add_reading(db, percent, note="console")
            finally:
                db.close()
        except (sqlite3.Error, OSError, ValueError) as error:
            return 503, {"error": "could not write the mark (%s)" % error}
        code, bank = self.data("/api/bank", {})
        self._bank = None  # the next /api/state reads the mark just written
        return 200, {"error": None, "bank": bank if code == 200 else None}

    def note(self, level, text):
        """Append one line to the Robot page's event list. An error identical to the last line is kept once."""
        text = " ".join(str(text).split())[:400]
        if not text:
            return
        with self.guard:
            if (level == "error" and self.events and self.events[-1]["level"] == "error"
                    and self.events[-1]["text"] == text):
                return
            self.events.append({"time": round(time.time(), 3), "level": level, "text": text})
            if len(self.events) > EVENT_CAP:
                del self.events[:-EVENT_CAP]

    def _import_finished_runs(self):
        """Pull finished run folders into the telemetry database so the Data tab does not wait for the recorder.
        Skipped while a drive is running, and when there is no run folder (reading must not create the file)."""
        if telemetry is None:
            return
        job = self.jobs.status()
        info = (job.get("job") or {}).get("info") or {}
        if job.get("running") and (info.get("drive") or info.get("moves")):
            return
        try:
            folders = [f for f in Path(self.log_root).iterdir() if f.is_dir() and RUN_NAME.match(f.name)]
        except OSError:
            return
        if not folders:
            return
        try:
            db = telemetry.open_db(self.db_path)
            try:
                telemetry.import_runs(db, self.log_root)
                db.execute("PRAGMA wal_checkpoint(PASSIVE)")
            finally:
                db.close()
        except (sqlite3.Error, OSError):
            pass

    def bank_cached(self):
        """The /api/bank body, refreshed at most every 15 s, so the Robot page does not poll it itself."""
        now = time.time()
        if self._bank is not None and now - self._bank_at < 15.0:
            return self._bank
        code, body = self.data("/api/bank", {})
        self._bank = body if code == 200 else None
        self._bank_at = now
        return self._bank

    def drop_default(self):
        """The reachable place, from so100_ik.reachable_drop. Cached. None x when that module cannot be imported."""
        if self._drop is not None:
            return self._drop
        try:
            import importlib.util
            path = HERE / "jetson" / "so100_ik.py"  # on the Pi: deploy.sh puts it beside console.py
            if not path.exists():
                path = HERE.parent / "jetson" / "so100_ik.py"  # in the repo
            spec = importlib.util.spec_from_file_location("so100_ik_console", path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            self._drop = mod.reachable_drop()
        except Exception as error:  # noqa: BLE001 (the page still works; it shows why there is no default)
            self._drop = {"x": None, "y": 0.0, "error": "%s" % error}
        return self._drop

    def _set_bag(self, x, y, top, yaw, source, at):
        bag = {"x": round(float(x), 3), "y": round(float(y), 3), "source": source, "at": round(float(at), 3)}
        if isinstance(top, (int, float)) and not isinstance(top, bool):
            bag["top"] = round(float(top), 3)
        if isinstance(yaw, (int, float)) and not isinstance(yaw, bool):
            bag["yaw"] = round(float(yaw), 3)
        with self.guard:
            self.bag = bag

    def _ask_front_camera(self, points):
        """One JSON line to front_camera.serve (TCP LOOK) and its answer, or None. Short timeouts: this is a probe."""
        hosts = list(getattr(self.jetson, "hosts", ()) or ())
        host = getattr(self.jetson, "host", None)
        if host:
            hosts = [host] + [h for h in hosts if h != host]
        req = (json.dumps({"id": "console", "points": points}) + "\n").encode("utf-8")
        for one in hosts:
            try:
                with socket.create_connection((one, ports.LOOK), timeout=0.4) as s:
                    s.settimeout(0.8)
                    s.sendall(req)
                    line = s.makefile("rb").readline(65536)
                answer = json.loads(line.decode("utf-8"))
            except (OSError, ValueError):
                continue
            if isinstance(answer, dict):
                return answer
        return None

    def probe_bag(self):
        """Remember the newest bag the front camera (or, failing that, the LiDAR) measured. Never writes 0.36.
        While a task holds the LiDAR, take the run's reached bag and do not open the camera port."""
        if self.live_from_run():
            self._bag_from_run()
            return
        devices = self.devices
        if devices.held:
            return
        with devices.scan.lock:
            points, at = list(devices.scan.points), devices.scan.at
        if at is None or devices.clock() - at > 2.0 or not points:
            return
        front = [(x, y) for x, y in points
                 if 0.25 < x < 1.6 and abs(y) < 0.5 and abs(math.degrees(math.atan2(y, x))) <= 40]
        if not front:
            return
        near = min(front, key=lambda p: math.hypot(p[0], p[1]))
        cluster = [p for p in front if math.hypot(p[0] - near[0], p[1] - near[1]) <= 0.12]
        answer = self._ask_front_camera([[round(x, 3), round(y, 3)] for x, y in cluster])
        now = time.time()
        handle = answer.get("handle") if isinstance(answer, dict) else None
        if isinstance(answer, dict) and answer.get("ok") and isinstance(handle, list) and len(handle) >= 2:
            self._set_bag(handle[0], handle[1], answer.get("top"), answer.get("yaw"), "front-camera", now)
            return
        with self.guard:
            bag = self.bag
        if bag and bag.get("source") == "front-camera" and now - bag["at"] < BAG_KEEP_S:
            return
        self._set_bag(sum(p[0] for p in cluster) / len(cluster), sum(p[1] for p in cluster) / len(cluster),
                      None, None, "lidar", now)

    def _bag_from_run(self):
        try:
            code, body = self.run_status()
        except (OSError, ValueError):
            return
        if code != 200 or not isinstance(body, dict):
            return
        record = body.get("record") if isinstance(body.get("record"), dict) else {}
        result = (record.get("bag_task") or {}).get("result") if isinstance(record.get("bag_task"), dict) else None
        if not isinstance(result, dict) or not result.get("ok"):
            return
        if not isinstance(result.get("x"), (int, float)) or isinstance(result.get("x"), bool):
            return
        if not isinstance(result.get("y"), (int, float)) or isinstance(result.get("y"), bool):
            return
        self._set_bag(result["x"], result["y"], result.get("top"), result.get("yaw"), "run", time.time())

    def state(self, heartbeat=False, session=None):
        devices = self.devices.status()
        jetson = self.jetson.snapshot()
        wheel = wheel_test_last(self.log_root / "wheel-test" / "last.json")
        pi = pi_health()
        _code, run = self.run_status(heartbeat=heartbeat, session=session)
        if isinstance(run, dict) and run.get("error"):
            self.note("error", str(run["error"]))
        with self.guard:
            events = list(self.events)
            bag = dict(self.bag) if self.bag else None
        return {"time": time.time(), "host": socket.gethostname(), "cards": cards(pi, devices, wheel, jetson, time.time()),
                "pi": pi, "devices": devices, "job": self.jobs.status(), "jetson": jetson, "wheel_test": wheel,
                "max_speed": self.p["max_speed"], "speeds": [s for s in SPEEDS if s <= self.p["max_speed"] + 1e-9],
                "minutes_max": MINUTES_MAX, "pin": self.pin is not None, "virtual_port": VIRTUAL_PORT,
                "version": deployed_version(), "hold": self.hold,
                "run": run if isinstance(run, dict) else {"state": "unknown"},
                "mission": self.mission.snapshot(time.monotonic()),
                "bag": bag, "drop": self.drop_default(), "events": events, "bank": self.bank_cached()}

    def live_from_run(self):
        """True while run_follow holds the camera and the LiDAR: its own page serves them (live_feed.py)."""
        job = self.jobs.status()
        return bool(job["running"] and job["job"] and job["job"]["kind"] == "task")

    def scan(self):
        """(code, body) of /scan: the run's LiDAR view during a run, else the console's own LiDAR."""
        if not self.live_from_run():
            return 200, self.devices.scan.json(self.devices.clock())
        try:
            return self.fetch(self.run_url + "/scan", timeout=1.0)
        except (OSError, ValueError) as error:
            return 503, {"error": "the run's live view does not answer yet (%s)" % error}

    def run_status(self, heartbeat=False, session=None):
        """run_follow's /status.         heartbeat: only for a read a browser just asked for (GET /api/run?heartbeat=1 or GET /api/state?heartbeat=1,
        sent by the page while its tab is in view), so the run's page-alive heartbeat stops with the phone; never
        from a thread or timer of the console itself. And only from the tab that sent the last command (its
        session token): another tab left open on a laptop must not keep the run's page alive for the phone.
        With no task running the answer is 200 {"state": "idle"} and the run's page is not asked."""
        if not self.live_from_run():
            return 200, {"state": "idle"}
        heartbeat = bool(heartbeat and self.commander and isinstance(session, str)
                         and hmac.compare_digest(session.encode(), self.commander.encode()))
        try:
            code, body = self.fetch(self.run_url + ("/status?heartbeat=1" if heartbeat else "/status"), timeout=1.0)
        except (OSError, ValueError) as error:
            return 200, {"error": "the run's status page does not answer yet (%s)" % error}
        if isinstance(body, dict) and "state" not in body:
            record = body.get("record") if isinstance(body.get("record"), dict) else None
            phase = record.get("phase") if record else None
            body = dict(body, state=phase or "running")
        return code, body

    def wheel_refusal(self):
        """(code, why) when a wheel test may not start now, else None. It drives 4 cm without looking."""
        now = self.clock()
        if self.wheel_at is not None and now - self.wheel_at < WHEEL_TEST_EVERY:
            return 429, "ทดสอบล้อได้ทุก 10 วินาที รออีก %.0f วินาที" % (WHEEL_TEST_EVERY - (now - self.wheel_at) + 0.5)
        d = self.devices
        with d.scan.lock:
            points, at = list(d.scan.points), d.scan.at
        if d.held or at is None or d.clock() - at > SCAN_FRESH:  # look first; without a fresh scan, no test
            return 409, "LiDAR ของคอนโซลยังไม่มีข้อมูลสด มองหน้าหุ่นไม่ได้ จึงทดสอบล้อไม่ได้ รอสักครู่แล้วลองใหม่"
        near = [(x, y) for x, y in points if 0 < x < WHEEL_CLEAR[0] and abs(y) < WHEEL_CLEAR[1]]
        if near:
            return 409, ("มีของอยู่หน้าหุ่นใกล้กว่า %.0f ซม. (ใกล้สุด %.0f ซม.) ย้ายออกก่อนทดสอบล้อ"
                         % (WHEEL_CLEAR[0] * 100, min(x for x, _ in near) * 100))
        return None

    def check_pin(self, given):
        """None when the PIN given is right; else (code, body). PIN_TRIES wrong ones lock it out for a while."""
        now = self.clock()
        with self.guard:
            if self.pin_locked_until is not None and now < self.pin_locked_until:
                return 429, {"error": "ใส่ PIN ผิดหลายครั้ง ล็อกไว้ %.0f วินาที (STOP ยังใช้ได้)"
                                      % (self.pin_locked_until - now + 0.5)}
            if hmac.compare_digest(given.encode(), self.pin.encode()):
                self.pin_fails = 0
                return None
            if given:
                self.pin_fails += 1
                if self.pin_fails >= PIN_TRIES:
                    self.pin_fails, self.pin_locked_until = 0, now + self.pin_lock_s
                    print("console: %d wrong PINs: PIN locked for %.0f s" % (PIN_TRIES, self.pin_lock_s),
                          file=sys.stderr, flush=True)
                    self.pin_lock_s = min(self.pin_lock_s * 2, PIN_LOCK_MAX_S)
        return 401, {"error": "the PIN is wrong or missing", "pin": True}

    def busy(self):
        """Why the console is busy now, or None: a job runs or is starting, or the agent's GET /arm says busy. A Jetson
        that does not answer has no arm command of the console's under way: not busy."""
        if self.jobs.busy:  # set from the start of start(), before the job is in status()
            return "a job is running or starting"
        try:
            code, arm = self.jetson.arm()
        except Exception:
            return None
        if code == 200 and isinstance(arm, dict) and arm.get("busy") is True:
            return "the arm is busy (%s)" % (arm.get("state") or "moving")
        return None

    def bag_job(self, cmd, body, task):
        """POST /api/arm pick_bag {"x", "y" (robot frame m)[, "top", "yaw"]}, drop_bag {"x", "y"} or bagpick
        {"putback": bool} (the arm team's camera pick): bag_arm's own bag job, answered when it is done. A pick without x
        and y takes the running task's last reached bag. Refused while the task's bag task moves the base, and while a
        task runs whose status does not say."""
        record = None
        from_run = False
        if task:
            code, status = self.run_status()
            record = status.get("record") if code == 200 and isinstance(status, dict) else None
            if not isinstance(record, dict):
                return 503, {"error": "the run's status does not answer: cannot tell whether a bag task moves the base"}
            if record.get("phase") == "bag":
                return 409, {"error": "a bag task is moving the base: the arm waits until it ended"}
        if cmd == "bagpick":
            putback = body.get("putback", False)
            if not isinstance(putback, bool):
                return 400, {"error": "bagpick takes putback: true or false"}
            out = {"cmd": cmd, "putback": putback}
            code, answer = self.jetson.arm(out, timeout=ARM_BAGPICK_TIMEOUT)
            return code, (dict(answer, sent=out) if isinstance(answer, dict) else answer)
        if cmd == "pick_bag" and body.get("x") is None and body.get("y") is None:
            result = ((record or {}).get("bag_task") or {}).get("result")
            if not (isinstance(result, dict) and result.get("ok")):
                return 409, {"error": "no reached bag in a running task: give x and y, or run bag or bag_near first"}
            body = result
            from_run = True
        out = {"cmd": cmd}
        for key in ("x", "y", "top", "yaw") if cmd == "pick_bag" else ("x", "y"):
            value = body.get(key)
            if value is None and key in ("top", "yaw"):
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not -10.0 < value < 10.0:  # NaN too
                return 400, {"error": "%s takes x and y in metres (robot frame)%s" % (
                    cmd, ", top (m) and yaw (rad) when known" if cmd == "pick_bag" else "")}
            out[key] = float(value)
        # The page's empty bag fields are 0.36, 0.00. That point is not a measurement.
        if cmd == "pick_bag" and abs(out["x"] - 0.36) < 0.001 and abs(out["y"]) < 0.001:
            measured = body.get("measured") is True or (from_run and body.get("ok") is True)
            if not measured:
                return 409, {
                    "error": "0.36, 0.00 is the console default, not a measured bag",
                    "reason": "DEFAULT_NOT_MEASURED",
                }
        code, answer = self.jetson.arm(out, timeout=ARM_JOB_TIMEOUT)
        return code, (dict(answer, sent=out) if isinstance(answer, dict) else answer)

    def _bag_only_worker(self, drive):
        """Auto-sequence for mission='bag_only': wait for run_follow -> send bag_near -> when reached, trigger pick_bag."""
        deadline = self.clock() + 15.0
        sent = False
        while self.clock() < deadline:
            job = self.jobs.status()
            if not (job["running"] and job["job"] and job["job"]["kind"] == "task"):
                return
            try:
                code, _ = self.fetch(self.run_url + "/command", {"command": "bag_near"}, timeout=2.0)
                if code == 200:
                    sent = True
                    break
            except (OSError, ValueError):
                pass
            time.sleep(0.5)
        if not sent:
            return
        # Poll until bag_near finishes (up to 120 s)
        pick_deadline = self.clock() + 120.0
        while self.clock() < pick_deadline:
            job = self.jobs.status()
            if not (job["running"] and job["job"] and job["job"]["kind"] == "task"):
                return
            try:
                code, status = self.fetch(self.run_url + "/status?heartbeat=1", timeout=2.0)
            except (OSError, ValueError):
                time.sleep(0.5)
                continue
            if code == 200 and isinstance(status, dict):
                result = ((status.get("record") or {}).get("bag_task") or {}).get("result")
                if isinstance(result, dict) and "ok" in result:
                    if result.get("ok") and drive:
                        self.bag_job("pick_bag", {}, task=True)
                    return
            time.sleep(0.5)

    def stop_arm(self):
        """The arm's stop beside the base's STOP, best effort: a note, never an error, never waits long."""
        out = []

        def ask():
            try:
                code, answer = self.jetson.arm({"cmd": "stop"}, timeout=5.0)
                out.append("arm stopped" if code == 200 else "arm stop: HTTP %d (%s)" % (
                    code, answer.get("error") if isinstance(answer, dict) else answer))
            except Exception as error:
                out.append("arm stop failed: %s" % error)
        asked = threading.Thread(target=ask, daemon=True)
        asked.start()
        asked.join(ARM_STOP_WAIT)
        return out[0] if out else "arm stop sent, no answer yet"

    def log_command(self, client, address, path, body, code, answer):
        """One line for one POST in logs/console/commands.jsonl (console_gate writes its maintenance lines there too).
        Never a PIN: it travels in a header, and a "pin" key in a body is dropped."""
        line = {"time": time.time(), "client": client, "address": address, "path": path,
                "body": {k: v for k, v in body.items() if k.lower() != "pin"} if isinstance(body, dict) else None,
                "code": code, "error": answer.get("error") if isinstance(answer, dict) else None}
        log = self.log_root / "console" / "commands.jsonl"
        try:
            log.parent.mkdir(parents=True, exist_ok=True)
            with self.guard, open(log, "a", encoding="utf-8") as f:
                f.write(json.dumps(line, ensure_ascii=False) + "\n")
        except (OSError, TypeError, ValueError):
            pass  # a full disk must not cost the answer

    def commands(self, n):
        """The last n lines of the command log (those in its last 16 KB) and the voice flow's commands in the newest
        run log, oldest first (the page turns it round)."""
        out = []
        for text in tail_lines(self.log_root / "console" / "commands.jsonl", n):
            try:
                out.append(json.loads(text))
            except ValueError:
                pass  # the first line, cut by the 16 KB window
        out += voice_commands(self.log_root)
        out.sort(key=lambda c: c.get("time") or 0)
        return {"commands": out[-n:]}

    def _mission_loop(self):
        """Clock and run_follow status for the executive. A failed read before the robot has left the start
        is not a lost link: the page is often not up yet."""
        while not self._mission_stop.wait(0.5):
            try:
                if not self.mission.running:
                    continue
                now = time.monotonic()
                live = self.live_from_run()
                if live:
                    self._mission_saw_run = True
                elif self._mission_saw_run:
                    self._mission_saw_run = False
                    self._apply(self.mission.on_event(
                        "base_run_ended|BASE_RUN_ENDED|the run ended", now))
                    continue
                self._apply(self.mission.on_tick(now))
                if not self.live_from_run():
                    continue
                try:
                    code, body = self.fetch(self.run_url + "/status", timeout=0.3)
                except (OSError, ValueError):
                    if getattr(self.mission, "left_start", False):
                        self._apply(self.mission.on_status({"link": False}, time.monotonic()))
                    continue
                status = _mission_status(body) if code == 200 else None
                if status is None:
                    continue
                self._apply(self.mission.on_status(status, time.monotonic()))
                for event in self._bag_events(status.get("bag_task")):
                    self._apply(self.mission.on_event(event, time.monotonic()))
            except Exception:
                continue

    def _apply(self, outputs):
        """Drive run_follow and the arm from the executive; its say lines go to the voice thread. Never stop-all."""
        for kind, value in outputs or ():
            if kind == "say" and self._voice_on and isinstance(value, str):
                self._says.put(value)
            if kind != "cmd" or not isinstance(value, dict):
                continue
            action = value.get("action")
            params = value.get("params") if isinstance(value.get("params"), dict) else {}
            try:
                if action in ("pick_bag", "drop_bag"):  # a thread: the mission loop keeps the clock meanwhile
                    threading.Thread(target=self._arm_job, args=(action, params), daemon=True).start()
                    continue
                if action == "find_bag":
                    command = "bag_again" if params.get("again") else "bag"
                    self._bag_asked = (self._bag_seen,)  # before it is sent: its task is the next new id
                else:
                    command = _RUN_ACTION.get(action)
                if command:
                    self.fetch(self.run_url + "/command", {"command": command},
                               timeout=0.4 if action == "stop" else 2.0)
            except Exception:
                continue

    def _begin_mission(self, minutes, mission="full"):
        """Reset the executive for the run that just started. The task clock is at most 5:00."""
        job = (self.jobs.status().get("job") or {})
        stem = Path(str(job.get("log") or "")).stem
        run_id = stem[:15] if RUN_NAME.match(stem[:15]) else time.strftime("%Y%m%d-%H%M%S")
        try:
            limit = min(300.0, float(minutes) * 60.0)
        except (TypeError, ValueError):
            limit = 300.0
        try:
            self.mission.open_log(Path(self.log_root) / run_id, run_id)
        except OSError:
            pass
        self._mission_saw_run = False
        self._bag_seen = self._bag_asked = self._bag_said = None
        voice_off = self._own_flow(mission == "full")  # first: the start's own lines are said through it
        # This call runs only after the Pi base program has started, so the local link is up.
        # A later status or NO_LINK can take it down again.
        self._apply(self.mission.start(time.monotonic(), report={"link": True}, run_id=run_id, limit_s=limit))
        if voice_off:
            self._park_flow()
            self._apply(self.mission.on_event("voice_relay_off|" + voice_off, time.monotonic()))

    def _stop_mission(self):
        self._apply(self.mission.stop(time.monotonic()))

    def _park_flow(self):
        """Ask the Jetson agent to stop only the flow program, when its host is already known."""
        host = getattr(self.jetson, "host", None)
        if not host:
            return
        fetch = getattr(self.jetson, "fetch", None) or self.fetch
        try:
            fetch("http://%s:%d/stop" % (host, AGENT_PORT), {"name": "flow"}, timeout=0.4)
        except Exception:
            pass

    def _own_flow(self, wanted=True):
        """Park the Jetson flow program for this run (POST /flow own) and start the voice thread: None. Else why not
        ("CODE|detail"): the caller then stops the flow program as before, and no speech reaches the executive."""
        self._voice_run += 1  # an older run's voice thread ends, without its release
        self._voice_on = False
        if not wanted:  # follow_only and bag_only: speech must not steer them (task_command's --remote note)
            return "NOT_FULL|speech reaches the executive in the full mission only"
        if not getattr(self.jetson, "host", None):
            return "NO_AGENT|the Jetson agent has not answered yet"
        try:
            code, answer = self.jetson.flow({"cmd": "own", "ttl": FLOW_LEASE}, timeout=FLOW_OWN_TIMEOUT)
        except Exception as error:
            code, answer = 0, {"error": str(error)}
        if code != 200 or not isinstance(answer, dict) or answer.get("parked") is not True:
            return "HTTP_%d|%s" % (code, answer.get("error") if isinstance(answer, dict) else answer)
        seq = answer.get("seq") if isinstance(answer.get("seq"), int) else 0
        self._says = queue.SimpleQueue()
        self._voice_on = True
        threading.Thread(target=self._voice_loop, args=(self._voice_run, seq), daemon=True).start()
        return None

    def _voice_loop(self, run, seq):
        """One run's voice thread, every FLOW_POLL s: the executive's say lines out in order, then one read of what
        the parked flow heard after seq (it renews the lease), each line to the executive. A failed call never ends
        the run: logged once when lost and once when back. It ends one round after the run did (a terminal state,
        STOP, or the run job's end) with one release, at a newer run's start, or with the console."""
        lost = saw_run = ending = False
        while not self._mission_stop.wait(FLOW_POLL) and self._voice_run == run:
            live = self.live_from_run()
            saw_run = saw_run or live
            over = not self.mission.running or (saw_run and not live)
            calls = []
            while not self._says.empty():
                calls.append({"cmd": "say", "text": self._says.get()})
            if not over:
                calls.append({"cmd": "heard", "after": seq, "ttl": FLOW_LEASE})
            for body in calls:
                try:
                    code, answer = self.jetson.flow(body)
                except Exception as error:
                    code, answer = 0, {"error": str(error)}
                ok = code == 200 and isinstance(answer, dict) and answer.get("ok") is True
                if ok == lost:  # it changed: lost now, or back
                    lost = not ok
                    self._apply(self.mission.on_event("voice_relay_ok" if ok else "voice_relay_lost|HTTP_%d|%s" % (
                        code, answer.get("error") if isinstance(answer, dict) else answer), time.monotonic()))
                if not (ok and body["cmd"] == "heard"):
                    continue
                for item in answer.get("items") or ():
                    if isinstance(item, dict) and isinstance(item.get("seq"), int) and item["seq"] > seq:
                        seq = item["seq"]
                        if isinstance(item.get("text"), str) and item["text"].strip():
                            self._apply(self.mission.on_heard(item["text"], time.monotonic()))
                if isinstance(answer.get("seq"), int) and answer["seq"] < seq:
                    seq = answer["seq"]  # the flow program started again and counts from 0
            if over and ending:
                break
            ending = over
        if self._voice_run == run:
            self._voice_on = False
            try:
                self.jetson.flow({"cmd": "release"}, timeout=FLOW_OWN_TIMEOUT)
            except Exception:
                pass

    def _arm_job(self, action, params):
        """The executive's pick_bag or drop_bag, one at a time, as the page's bag job of the task (bag_job: its checks,
        its timeout), its answer back as the event the executive reads: grab_success or grab_failed|CODE|message,
        drop_done or drop_failed|CODE|message. No answer in time, or no agent: TIMEOUT."""
        with self._arm_lock:
            if not self.mission.running:
                return  # STOP or the run's end came while an earlier job held the arm
            try:  # pick_bag's x and y are the run's reached bag (bag_reached): measured, as bag_job's own from the run
                code, answer = self.bag_job(action, dict(params, measured=True), task=True)
            except Exception as error:
                code, answer = 0, {"error": str(error)}
        answer = answer if isinstance(answer, dict) else {}
        if code == 200 and answer.get("ok") is True:
            event = "grab_success" if action == "pick_bag" else "drop_done"
        else:
            event = "%s|%s|%s" % ("grab_failed" if action == "pick_bag" else "drop_failed",
                                  answer.get("code") or answer.get("reason")
                                  or ("TIMEOUT" if code in (0, 502, 503, 504) else "REFUSED"),
                                  answer.get("message") or answer.get("error") or "HTTP %d" % code)
        self._apply(self.mission.on_event(event, time.monotonic()))

    def _bag_events(self, bag):
        """bag_chosen|SIDE|say, then bag_reached|CODE|x,y[,top=T][,yaw=Y] or bag_failed|CODE|message, once per bag
        task that the executive's find_bag asked for (one whose id differs from the one seen when it was sent): the
        rules and formats of base/jetson/base_bridge.py BridgeLogic._bag_events, from run_follow's record.bag_task."""
        if not isinstance(bag, dict):
            return []
        self._bag_seen = bag.get("id")
        if self._bag_asked is None or bag.get("id") == self._bag_asked[0]:
            return []
        out = []
        chosen, result = bag.get("chosen"), bag.get("result")
        if isinstance(chosen, dict) and self._bag_said != bag.get("id"):
            self._bag_said = bag.get("id")
            out.append("bag_chosen|%s|%s" % (str(chosen.get("side") or "none").upper(), chosen.get("say", "")))
        if isinstance(result, dict):
            self._bag_asked = None
            if result.get("ok"):
                extra = "".join(",%s=%.3f" % (k, result[k]) for k in ("top", "yaw") if result.get(k) is not None)
                out.append("bag_reached|%s|%.3f,%.3f%s" % (result.get("code", "OK"), result["x"], result["y"], extra))
            else:
                out.append("bag_failed|%s|%s" % (result.get("code", "FAILED"), result.get("message", "")))
        return out

    def post(self, path, body, session=None, client=None, program=None):
        """(code, body) for one POST from the page. session: the tab's X-HMC-Session token, if it has one.
        client: the address the POST came from (a virtual operator run takes the person from there).
        program: the X-HMC-Client name of a program on the Pi (robotctl), None for the console's own page."""
        if path == "/api/stop":
            self._stop_mission()  # motion only: records OPERATOR_STOP, never stop-all
            note = self.jobs.stop()  # the base first: the arm's stop never delays it
            return 200, {"error": None, "note": note, "arm": self.stop_arm()}
        if path == "/api/arm/stop":
            return self.jetson.arm({"cmd": "stop"}, timeout=5.0)
        if path in ("/api/bank/full", "/api/bank/reading"):
            return self.bank_post(path, body)
        if path == "/api/hold":
            who = body.get("who") or program or "page"
            if not isinstance(who, str) or not CLIENT.match(who):
                return 400, {"error": "who: 1 to 64 letters, digits and . @ : % - _"}
            with self.guard:
                if self.hold is not None:
                    return 409, {"error": "already held by %s" % self.hold["who"], "hold": dict(self.hold)}
                self.hold = {"who": who, "since": time.time()}  # first: a start from now on is refused
            why = self.busy()
            if why:
                with self.guard:
                    self.hold = None
                return 409, {"error": "not held: %s" % why}
            return 200, {"error": None, "hold": dict(self.hold)}
        if path == "/api/release":
            with self.guard:
                was, self.hold = self.hold, None
            return 200, {"error": None, "released": was}
        hold = self.hold
        if hold is not None and (path in ("/api/task/start", "/api/check/start", "/api/arm")
                                 or path.startswith("/api/jetson/")):
            return 409, {"error": HELD % hold["who"], "hold": dict(hold)}
        if path == "/api/arm":
            cmd = body.get("cmd")
            if cmd not in ARM_CMDS:
                return 400, {"error": "cmd must be one of %s" % ", ".join(ARM_CMDS)}
            if cmd == "torque" and not isinstance(body.get("on"), bool):
                return 400, {"error": "torque takes on: true or false"}
            job = self.jobs.status()
            task = bool(job["running"] and job["job"] and job["job"]["kind"] == "task")
            if task or self.mission.running:
                return 409, {"error": "a task run is active: arm and bag buttons are refused until the run ends"
                                      " (STOP still stops motion)"}
            if cmd in ARM_JOBS:
                return self.bag_job(cmd, body, task)
            return self.jetson.arm({"cmd": cmd, "on": body["on"]} if cmd == "torque" else {"cmd": cmd})
        if path == "/api/task/start":
            virtual = body.get("virtual", False)
            if not isinstance(virtual, bool):
                return 400, {"error": "virtual takes true or false"}
            mission = body.get("mission", "full")
            pc = (body.get("virtual_host") or client) if virtual else None  # the PC that sends the virtual person
            cmd, error = task_command(body.get("drive"), body.get("speed"), body.get("minutes"), self.p,
                                      virtual=pc, mission=mission)
            if error:
                return 400, {"error": error}
            if body["drive"] and self.pin is None:
                return 403, {"error": NEED_PIN}
            if virtual and self.pin is None:
                return 403, {"error": NEED_PIN_VIRTUAL}
            m_tag = MISSION_LABEL.get(mission, mission)
            label = "%s (%s%s %.2f m/s %s นาที)" % (
                m_tag, "ขับจริง" if body["drive"] else "ทดลอง", " คนเสมือน" if virtual else "",
                body["speed"], ("%g" % body["minutes"]))
            error = self.jobs.start("task", label, cmd, {"drive": body["drive"], "speed": body["speed"],
                                                         "minutes": body["minutes"], "mission": mission,
                                                         "virtual": {"host": pc, "port": VIRTUAL_PORT} if virtual else None})
            if not error:
                self.commander = None  # a new run: no tab leads it until one sends a command
                self._begin_mission(body.get("minutes"), mission)
                if mission == "bag_only":
                    threading.Thread(target=self._bag_only_worker, args=(body["drive"],), daemon=True).start()
            return (409 if error else 200), {"error": error}
        if path == "/api/check/start":
            try:
                found = check_command(body.get("check"), args=body.get("args", []), max_speed=self.p["max_speed"],
                                      page=program is None)
            except ValueError as error:
                return 400, {"error": "%s" % error}
            if found is None:
                return 400, {"error": "unknown check"}
            cmd, label, moves = found
            if moves:
                if self.pin is None:
                    return 403, {"error": NEED_PIN}
                # the wheel test drives without looking; the other tools watch the LiDAR themselves
                refused = self.wheel_refusal() if body["check"] == "wheel" else None
                if refused:
                    return refused[0], {"error": refused[1]}
            error = self.jobs.start("check-%s" % body["check"], label, cmd, {"moves": moves})
            if not error and moves:
                self.wheel_at = self.clock()
            return (409 if error else 200), {"error": error}
        if path == "/api/run/command":
            command = body.get("command")
            if command not in COMMANDS:
                return 400, {"error": "command must be one of %s" % ", ".join(COMMANDS)}
            job = self.jobs.status()
            if not job["running"] or job["job"]["kind"] != "task":
                return 409, {"error": "no task is running"}
            if self.mission.running:
                self._apply(self.mission.on_button(command, time.monotonic()))
                token = session if isinstance(session, str) and SESSION.match(session) else secrets.token_hex(16)
                self.commander = token
                snap = self.mission.snapshot()
                return 200, {"error": None, "session": token, "state": snap["state"], "say": snap["say"]}
            token = session if isinstance(session, str) and SESSION.match(session) else secrets.token_hex(16)
            try:
                code, reply = self.fetch(self.run_url + "/command", {"command": command}, timeout=2.0)
            except (OSError, ValueError) as error:
                return 502, {"error": "the run did not answer (%s)" % error}
            if code == 200:
                self.commander = token  # this tab leads now: only its status reads are the run's heartbeat
                try:  # and at once for it, so the run's next cycle already sees the page alive (no Jetson)
                    self.fetch(self.run_url + "/status?heartbeat=1", timeout=1.0)
                except (OSError, ValueError):
                    pass
            return code, (dict(reply, session=token) if isinstance(reply, dict) else reply)
        if path.startswith("/api/jetson/"):
            return self.jetson.act(path[len("/api/jetson/"):], body.get("name"))
        if path == "/api/mission/hear":
            text = body.get("text")
            if not isinstance(text, str) or not text.strip():
                return 400, {"error": "text must be a string"}
            self._apply(self.mission.on_heard(text, time.monotonic()))
            return 200, {"error": None, "mission": self.mission.snapshot()}
        if path == "/api/mission/event":
            self._apply(self.mission.on_event(body.get("event", body), time.monotonic()))
            return 200, {"error": None, "mission": self.mission.snapshot()}
        if path == "/api/skeleton":
            if not isinstance(body.get("on"), bool):
                return 400, {"error": "send on: true or false"}
            self.devices.skeleton_on = body["on"]
            return 200, dict(self.devices.skeleton_status(), error=None)
        return 404, {"error": "not found"}


def handler_for(console):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"

        def log_message(self, *args):
            pass

        def _send(self, code, body, kind="application/json", extra=()):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            for k, v in extra:
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split("?")[0]
            try:
                if path in FILES:
                    name, kind = FILES[path]
                    self._send(200, (STATIC / name).read_bytes(), kind)  # read each time: edit, reload
                elif path == "/api/state":
                    query = urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)
                    session = self.headers.get("X-HMC-Session") or (query.get("session") or [None])[0]
                    self._send(200, console.state(heartbeat=query.get("heartbeat") == ["1"], session=session))
                elif path == "/usb-snap":
                    snap = getattr(console.jetson, "webcam_snap", None)
                    if snap is None:
                        self._send(503, {"error": "no snapshot"})
                    else:
                        self._send(*snap())
                elif path == "/api/run":
                    query = urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)
                    session = self.headers.get("X-HMC-Session") or (query.get("session") or [None])[0]
                    self._send(*console.run_status(heartbeat=query.get("heartbeat") == ["1"], session=session))
                elif path == "/api/logs":
                    self._send(200, {"runs": list_runs(console.log_root)})
                elif path == "/api/commands":
                    query = urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)
                    try:
                        n = max(1, min(COMMANDS_MAX, int((query.get("n") or ["20"])[0])))
                    except ValueError:
                        n = 20
                    self._send(200, console.commands(n))
                elif path == "/api/arm":
                    self._send(*console.jetson.arm())
                elif path == "/api/linktest":
                    self._send(200, {"results": link_test(console.jetson.hosts)})
                elif path == "/scan":
                    self._send(*console.scan())
                elif path == "/api/skeleton":
                    self._send(200, console.devices.skeleton_status())
                elif path == "/stream":
                    self._stream()
                elif path == "/usb-stream":
                    camera_view.relay(self, console.jetson.webcam_url())
                elif path.startswith("/logs/") and path.endswith("/log.jsonl"):
                    self._download(path.split("/")[2])
                elif path.startswith("/api/data/") or path == "/api/bank":
                    self._send(*console.data(path, urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)))
                else:
                    self._send(404, {"error": "not found"})
            except OSError:  # the viewer went away (broken pipe, reset, aborted)
                pass

        def _stream(self):
            if console.live_from_run():  # run_follow has the camera: relay its page's stream
                return camera_view.relay(self, console.run_url + "/stream", 503,
                                         "the run's camera view does not answer yet")
            devices = console.devices
            if devices.held:
                return self._send(503, {"error": "the camera is in use by the job"})
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            n = 0
            while not devices.held:
                jpeg, m = devices.latest.wait_newer(n)
                if jpeg is None or m == n:
                    continue
                n = m
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg))
                self.wfile.write(jpeg + b"\r\n")
                time.sleep(1 / 15)  # 15 frames a second is plenty on a phone

        def _download(self, name):
            log = console.log_root / name / "log.jsonl"
            if not RUN_NAME.match(name) or not log.is_file():
                return self._send(404, {"error": "no such run"})
            size = log.stat().st_size
            self.send_response(200)
            self.send_header("Content-Type", "application/x-ndjson")
            self.send_header("Content-Length", str(size))
            self.send_header("Content-Disposition", 'attachment; filename="run-%s-log.jsonl"' % name)
            self.end_headers()
            with open(log, "rb") as f:
                shutil.copyfileobj(f, self.wfile, size if size < 1 << 20 else 1 << 20)

        def do_POST(self):
            path = self.path.split("?")[0]
            program = self.headers.get("X-HMC-Client")
            self.body = None
            code, answer = self._post(path, program)
            if isinstance(answer, dict) and answer.get("error"):
                console.note("error", "%s: %s" % (path, answer["error"]))
            elif path in ("/api/stop", "/api/arm/stop", "/api/jetson/stop-all"):
                console.note("info", "%s ok" % path)
            session = self.headers.get("X-HMC-Session") or ""
            try:
                self._send(code, answer)
            finally:  # the page's session token only in part: it also picks the run's heartbeat
                console.log_command(program[:64] if program is not None else "page" + (" " + session[:6] if session else ""),
                                    self.client_address[0], path, self.body, code, answer)

        def _post(self, path, program):
            try:
                length = max(0, int(self.headers.get("Content-Length", "0")))
            except ValueError:
                length = 0
            raw = self.rfile.read(min(length, MAX_BODY)) if length else b""  # read first: an answer before the
            if length > MAX_BODY:                                             # body makes Windows reset the socket
                return 413, {"error": "too big"}
            if path != "/api/stop":  # STOP always works; anything else only from the console's own page, or a program
                kind = self.headers.get("Content-Type", "").split(";")[0].strip().lower()  # on the Pi itself (ssh)
                own = (self.client_address[0] == "127.0.0.1" and bool(CLIENT.match(program))) if program is not None \
                    else self.headers.get("X-HMC-Console") == "1"
                if kind != "application/json" or not own:
                    return 403, {"error": "only the console's own page, or robotctl on the Pi, may send this"}
            if console.pin is not None and path not in NO_PIN:
                refused = console.check_pin(self.headers.get("X-Console-Pin", ""))
                if refused:
                    return refused
            try:
                body = json.loads(raw or b"{}")
            except ValueError:
                body = {} if path == "/api/stop" else None
            if not isinstance(body, dict):
                return 400, {"error": "send a JSON object"}
            self.body = body
            return console.post(path, body, session=self.headers.get("X-HMC-Session"), client=self.client_address[0],
                                program=program)
    return Handler


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--no-devices", action="store_true", help="never open the camera or the LiDAR (viewing elsewhere)")
    args = ap.parse_args(argv)
    os.environ[console_gate.CONSOLE_ENV] = "1"  # its own live view, camera_pipe and S# open the devices
    import params
    p = params.load()
    devices = Devices(p)
    if not args.no_devices:
        devices.start()

    jobs = Jobs(release=devices.stop, reclaim=(lambda: None) if args.no_devices else devices.start,
                ask_end=ask_run_end, stop_wheels=send_stop)
    jetson = Jetson()

    def keep_checking():
        while True:
            try:
                jetson.refresh()
            except Exception as error:  # never let the checker die
                jetson.agent_error = "check failed: %s" % error
            time.sleep(REFRESH)
    threading.Thread(target=keep_checking, daemon=True).start()
    console = Console(p, devices, jobs, jetson, pin=load_pin())

    def keep_bag():
        while True:
            try:
                console.probe_bag()
            except Exception:  # noqa: BLE001 (a failed look must not stop the console)
                pass
            time.sleep(2.0)
    threading.Thread(target=keep_bag, daemon=True).start()
    recorder = None
    try:  # telemetry.py (base/pi, the telemetry recorder's own file) records metrics while the console runs
        import telemetry
        recorder = telemetry.Recorder(logs_dir=console.log_root, db_path=console.db_path)
        recorder.start()
    except Exception:  # not there, or it failed: the console works without it
        recorder = None
    server = ThreadingHTTPServer(("0.0.0.0", args.port), handler_for(console))
    server.daemon_threads = True

    def finish(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, finish)
    print("robot console on port %d%s" % (args.port, " (PIN on)" if console.pin else ""), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        jobs.shutdown()
        devices.stop()
        if recorder is not None:
            try:
                recorder.stop()
            except Exception:
                pass
        server.server_close()


if __name__ == "__main__":
    main()
