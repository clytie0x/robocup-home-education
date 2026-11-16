"""Robot console: one web page to run and check the whole robot from a phone or a laptop.

  .venv/bin/python console.py [--port 8081] [--no-devices]
  then open http://192.168.31.21:8081/ (Wi-Fi) or http://10.0.0.1:8081/ (LAN cable)

Tabs: Overview (state, faults, health list, power bank estimate, motor battery use), Live (Xtion camera, LiDAR from above,
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
import os
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

try:  # the telemetry database (base/pi/telemetry.py) for the Data tab and the power bank estimate
    import telemetry
except Exception:  # not there: the Data tab says so, the rest works
    telemetry = None

STATIC = HERE / "console"
LOG_ROOT = HERE / "logs"  # run_follow's logs/<start time>/ folders; on the Pi ~/hmc-base/logs
JOB_LOGS = LOG_ROOT / "console"
WHEEL_TEST_LAST = LOG_ROOT / "wheel-test" / "last.json"  # written by tools/wheel_test.py
PIN_FILE = Path.home() / ".config" / "hmc-console-pin"
PORT = 8081
RUN_URL = "http://127.0.0.1:8080"  # run_follow's own dashboard while it runs
AGENT_PORT = 8090  # base/jetson/agent.py
WEBCAM_PORT = 8082  # the arm team's ~/so100_ws/webcam_view.py: only shown, never started or stopped
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
}
COMMANDS = ("follow", "wait", "resume", "return")
NO_PIN = ("/api/stop", "/api/bank/full", "/api/bank/reading")  # STOP, and the owner's battery marks
MAX_BODY = 65536  # bytes of a POST body
SESSION = re.compile(r"^[0-9a-f]{32}$")  # a page tab's token (X-HMC-Session), from its first command
WHEEL_TEST_EVERY = 10.0  # s at least between two wheel tests
WHEEL_CLEAR = (0.35, 0.25)  # m: nothing may be in 0 < x < 0.35, |y| < 0.25 (robot frame) for a wheel test
SCAN_FRESH = 0.5  # s; the wheel test needs a scan of the console's own LiDAR at most this old showing a clear front
HURRY_MIN = 0.3  # s a second STOP still leaves the run after asking it to end (it sends its own S# when it ends)
PIN_TRIES = 5  # wrong PINs (from anyone) before the PIN is locked out
PIN_LOCK_S = 60.0  # s of the first lockout; each next one twice as long, at most PIN_LOCK_MAX_S
PIN_LOCK_MAX_S = 900.0
DATA_QUERIES = 2  # /api/data/* at once; 1 while a drive job runs (a run's cycle must not starve)
JOB_SCRIPTS = ("run_follow.py", "safe_run.py", "wheel_test.py", "job_guard.py", "sensor_check.py")
NEED_PIN = ("ตั้ง PIN ก่อน ถึงจะขับจริงหรือทดสอบล้อได้ ใส่ PIN ในไฟล์ ~/.config/hmc-console-pin บน Pi"
            " แล้วเปิดคอนโซลใหม่ (ดู base/docs/robot-console.md) ส่วนงานทดลองเริ่มได้เลย")
JETSON_ACTIONS = ("start", "stop", "restart", "start-all", "stop-all")
VIRTUAL_PORT = 5610  # UDP: run_follow --virtual-operator takes the PC's simulated person here (virtual_operator.py)
NEED_PIN_VIRTUAL = ("ตั้ง PIN ก่อน ถึงจะทดสอบด้วยคนเสมือนได้ ใส่ PIN ในไฟล์ ~/.config/hmc-console-pin บน Pi"
                    " แล้วเปิดคอนโซลใหม่ (ดู base/docs/robot-console.md)")


# ---------------------------------------------------------------- jobs

def task_command(drive, speed, minutes, p, py=sys.executable, virtual=None):
    """(argv, None) for run_follow, or (None, why not). virtual: the PC's IPv4 address for a virtual operator run."""
    if not isinstance(drive, bool):
        return None, "drive takes true or false"
    if isinstance(speed, bool) or speed not in SPEEDS:
        return None, "top speed must be one of %s m/s" % ", ".join("%.2f" % s for s in SPEEDS)
    if speed > p["max_speed"] + 1e-9:
        return None, "%.2f m/s is above max_speed %.2f in params.json" % (speed, p["max_speed"])
    if isinstance(minutes, bool) or not isinstance(minutes, (int, float)) or not 0 < minutes <= MINUTES_MAX:
        return None, "length must be 1 to %d minutes" % MINUTES_MAX
    run = [py, str(HERE / "run_follow.py")] + (["--drive"] if drive else []) + [
        "--seconds", str(int(round(minutes * 60))), "--max-speed", "%.2f" % speed, "--remote",
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


def check_command(name, py=sys.executable):
    """(argv, label, moves) for a check, or None."""
    guard = [py, str(TOOLS / "job_guard.py")]
    if name == "wheel":
        return guard + [py, str(TOOLS / "safe_run.py"), py, str(TOOLS / "wheel_test.py")], "ทดสอบล้อ", True
    if name == "floor":
        return guard + [py, str(TOOLS / "sensor_check.py"), "--floor"], "ตรวจมุมกล้อง", False
    return None


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


def tail_lines(path, n=30):
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - 16384))
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
                                  stderr=subprocess.STDOUT, env=dict(os.environ, PYTHONUNBUFFERED="1"), **group)
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

    def webcam_url(self):
        return "http://%s:%d/stream" % (self.host or self.hosts[0], WEBCAM_PORT)

    def snapshot(self):
        return {"host": self.host, "links": self.links, "agent": self.agent, "agent_error": self.agent_error,
                "agent_at": self.agent_at, "webcam": self.webcam, "checked": self.checked}


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
        self.db_path = Path(db_path) if db_path is not None else (
            telemetry.DB_PATH if telemetry is not None else HERE / "data" / "robot.db")

    def data(self, path, query):
        """(code, body[, content type, headers]) of GET /api/data/series, /runs, /run/<id>, /export.csv and
        /api/bank (the power bank's estimate, and "motor": the 21V battery's use). Read-only queries on the telemetry database; a database not made yet gives empty answers."""
        if telemetry is None:
            return 503, {"error": "telemetry.py is not next to console.py: no data to show", "telemetry": False}
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
        return 200, {"error": None, "bank": bank if code == 200 else None}

    def state(self):
        devices = self.devices.status()
        jetson = self.jetson.snapshot()
        wheel = wheel_test_last(self.log_root / "wheel-test" / "last.json")
        pi = pi_health()
        return {"time": time.time(), "host": socket.gethostname(), "cards": cards(pi, devices, wheel, jetson, time.time()),
                "pi": pi, "devices": devices, "job": self.jobs.status(), "jetson": jetson, "wheel_test": wheel,
                "max_speed": self.p["max_speed"], "speeds": [s for s in SPEEDS if s <= self.p["max_speed"] + 1e-9],
                "minutes_max": MINUTES_MAX, "pin": self.pin is not None, "virtual_port": VIRTUAL_PORT,
                "version": deployed_version()}

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
        """run_follow's /status. heartbeat: only for a read a browser just asked for (GET /api/run?heartbeat=1,
        sent by the page while its tab is in view), so the run's page-alive heartbeat stops with the phone; never
        from a thread or timer of the console itself. And only from the tab that sent the last command (its
        session token): another tab left open on a laptop must not keep the run's page alive for the phone."""
        heartbeat = bool(heartbeat and self.commander and isinstance(session, str)
                         and hmac.compare_digest(session.encode(), self.commander.encode()))
        try:
            code, body = self.fetch(self.run_url + ("/status?heartbeat=1" if heartbeat else "/status"), timeout=1.0)
        except (OSError, ValueError) as error:
            return 200, {"error": "the run's status page does not answer yet (%s)" % error}
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

    def post(self, path, body, session=None, client=None):
        """(code, body) for one POST from the page. session: the tab's X-HMC-Session token, if it has one.
        client: the address the POST came from (a virtual operator run takes the person from there)."""
        if path == "/api/stop":
            return 200, {"error": None, "note": self.jobs.stop()}
        if path in ("/api/bank/full", "/api/bank/reading"):
            return self.bank_post(path, body)
        if path == "/api/task/start":
            virtual = body.get("virtual", False)
            if not isinstance(virtual, bool):
                return 400, {"error": "virtual takes true or false"}
            pc = (body.get("virtual_host") or client) if virtual else None  # the PC that sends the virtual person
            cmd, error = task_command(body.get("drive"), body.get("speed"), body.get("minutes"), self.p, virtual=pc)
            if error:
                return 400, {"error": error}
            if body["drive"] and self.pin is None:
                return 403, {"error": NEED_PIN}
            if virtual and self.pin is None:
                return 403, {"error": NEED_PIN_VIRTUAL}
            label = "งาน%s%s %.2f m/s %s นาที" % ("ขับจริง" if body["drive"] else "ทดลอง", " คนเสมือน" if virtual else "",
                                                 body["speed"], ("%g" % body["minutes"]))
            error = self.jobs.start("task", label, cmd, {"drive": body["drive"], "speed": body["speed"],
                                                         "minutes": body["minutes"],
                                                         "virtual": {"host": pc, "port": VIRTUAL_PORT} if virtual else None})
            if not error:
                self.commander = None  # a new run: no tab leads it until one sends a command
            return (409 if error else 200), {"error": error}
        if path == "/api/check/start":
            found = check_command(body.get("check"))
            if found is None:
                return 400, {"error": "unknown check"}
            cmd, label, moves = found
            if moves:
                if self.pin is None:
                    return 403, {"error": NEED_PIN}
                refused = self.wheel_refusal()
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
                    self._send(200, console.state())
                elif path == "/api/run":
                    query = urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)
                    session = self.headers.get("X-HMC-Session") or (query.get("session") or [None])[0]
                    self._send(*console.run_status(heartbeat=query.get("heartbeat") == ["1"], session=session))
                elif path == "/api/logs":
                    self._send(200, {"runs": list_runs(console.log_root)})
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
            try:
                length = max(0, int(self.headers.get("Content-Length", "0")))
            except ValueError:
                length = 0
            raw = self.rfile.read(min(length, MAX_BODY)) if length else b""  # read first: an answer before the
            if length > MAX_BODY:                                             # body makes Windows reset the socket
                return self._send(413, {"error": "too big"})
            if path != "/api/stop":  # STOP always works; anything else only from the console's own page
                kind = self.headers.get("Content-Type", "").split(";")[0].strip().lower()
                if kind != "application/json" or self.headers.get("X-HMC-Console") != "1":
                    return self._send(403, {"error": "only the console's own page may send this"})
            if console.pin is not None and path not in NO_PIN:
                refused = console.check_pin(self.headers.get("X-Console-Pin", ""))
                if refused:
                    return self._send(*refused)
            try:
                body = json.loads(raw or b"{}")
            except ValueError:
                body = {} if path == "/api/stop" else None
            if not isinstance(body, dict):
                return self._send(400, {"error": "send a JSON object"})
            self._send(*console.post(path, body, session=self.headers.get("X-HMC-Session"), client=self.client_address[0]))
    return Handler


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--no-devices", action="store_true", help="never open the camera or the LiDAR (viewing elsewhere)")
    args = ap.parse_args(argv)
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
    recorder = None
    try:  # telemetry.py (base/pi, the telemetry recorder's own file) records metrics while the console runs
        import telemetry
        recorder = telemetry.Recorder()
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
