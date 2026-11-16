"""Virtual operator on the owner's PC: a 3D person the real robot follows in the real room.

    python base/pc/virtual_operator/app.py [--control ssh|console] [--console http://192.168.31.21:8081] [--port 8095]
    then open http://127.0.0.1:8095/

--control ssh (default): runs are started and stopped over ssh to the Pi and read through an ssh tunnel to
run_follow's own page (remote.py): no person and no PIN needed. --control console: through the robot console's
API with its PIN, as the phone does.

Standard library only. It reads the robot console (base/pi/console.py): the run's status (/api/run), its LiDAR
(/scan, robot frame, with the odometry pose of that cycle) and the job (/api/state). From the scans and poses
it keeps a map of the room (sim.Grid). The person walks on that map (sim.astar) at the owner's walking speed,
and its state goes 10 times a second as UDP to run_follow --virtual-operator on the Pi (virtual_operator.py,
port 5610), with the virtual bags placed on the page (the robot sees them as LiDAR circles and its BAG
phase finds and approaches the pointed one; the chosen bag comes back in the record as "bag"). Runs are started through the console (POST /api/task/start with "virtual": true) with the PIN the
owner types in the page: it stays in this process's memory only, never on disk. Episodes of the auto loop
go to episodes.csv next to this file.

The page and its API answer only on 127.0.0.1; every POST needs JSON and the header X-VO-App: 1, so no
other web site can drive the robot through this app.
"""
import argparse
import json
import math
import re
import socket
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import remote  # noqa: E402
import sim  # noqa: E402

PORT = 8095
CONSOLES = tuple("http://%s:%d" % (host, remote.ports.CONSOLE) for host in remote.HOSTS)  # lab Wi-Fi, Tailscale
VIRTUAL_PORT = remote.ports.VIRTUAL  # the console's VIRTUAL_PORT, until /api/state says otherwise
CSV_PATH = HERE / "episodes.csv"
SEND_EVERY = 0.1  # s between datagrams to the robot
RUN_EVERY = 0.2  # s between status reads
SCAN_EVERY = 0.2  # s between /scan reads
STATE_EVERY = 1.0  # s between /api/state reads
MAP_EVERY = 0.5  # s between scans put in the map
JUMP = 0.5  # m of pose change between two reads: a new odometry frame (the run restarted its count)
APP_HEADER = "X-VO-App"
SPEEDS = (0.08, 0.12)
STOPPED = re.compile(r"STOPPED: (.*)")


class ConsoleClient:
    """The robot console's API, as its own page uses it (JSON, X-HMC-Console: 1, the PIN as X-Console-Pin)."""

    def __init__(self, base):
        self.base = base.rstrip("/")
        self.pin = None  # memory only

    @property
    def host(self):
        return urllib.parse.urlsplit(self.base).hostname

    def _call(self, path, body=None, timeout=2.0):
        headers = {}
        data = None
        if body is not None:
            data = json.dumps(body).encode()
            headers = {"Content-Type": "application/json", "X-HMC-Console": "1"}
            if self.pin:
                headers["X-Console-Pin"] = self.pin
        req = urllib.request.Request(self.base + path, data, headers)
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return r.status, json.loads(r.read() or b"null")
        except urllib.error.HTTPError as e:
            try:
                return e.code, json.loads(e.read() or b"null")
            except ValueError:
                return e.code, None

    def get(self, path, timeout=1.5):
        return self._call(path, None, timeout)

    def post(self, path, body, timeout=8.0):
        return self._call(path, body, timeout)


class App:
    def __init__(self, console, csv_path=CSV_PATH, clock=time.monotonic, control="ssh", ssh=None):
        self.lock = threading.RLock()
        self.clock = clock
        self.control = control  # "ssh" or "console"
        self.ssh = ssh or remote.Ssh()
        self.tunnel = ConsoleClient("http://127.0.0.1:%d" % remote.TUNNEL_PORT)  # run_follow's page, over ssh
        self.client = ConsoleClient(console)
        self.person = sim.Person()
        self.bags = []  # virtual bags on the floor, odometry frame: [x, y] each
        self.grid = sim.Grid()
        self.robot = sim.Robot()
        self.auto = sim.Auto(self.person, self.grid, Background(self), csv_path)
        self.run_settings = {"drive": False, "speed": 0.08, "minutes": 10}
        self.scan = []  # the newest scan, odometry frame
        self.job_started = None  # the console's start time of the job, to see a new run
        self.virtual_port = VIRTUAL_PORT
        self.connected = False
        self.error = None  # the newest problem talking to the console
        self.last_pose = None
        self.map_at = -math.inf
        self.sent, self.udp_error = 0, None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.stopping = threading.Event()

    # ------------------------------------------------------------ the console

    def start_run(self):
        """Auto loop, the page and scripts: a virtual operator run. None or why not."""
        if self.control == "ssh":
            try:
                pc = remote.local_address(self.ssh.host)
            except OSError as error:
                return "ไม่มีทางไป Pi (%s)" % error
            error = self.ssh.start(dict(self.run_settings), pc, self.virtual_port)
            if error is None:
                self.ssh.ensure_tunnel()
            return error
        body = dict(self.run_settings, virtual=True)
        if not self.client.pin:
            return "ใส่ PIN ของคอนโซลก่อน"
        try:
            code, reply = self.client.post("/api/task/start", body)
        except (OSError, ValueError) as error:
            return "คอนโซลไม่ตอบ (%s)" % error
        if code == 401:
            return "PIN ผิด"
        if code != 200 or (reply or {}).get("error"):
            return (reply or {}).get("error") or "HTTP %d" % code
        return None

    def stop_run(self):
        """ssh: the console's STOP, the run's own end, then SIGTERM to its group; console: its STOP (no PIN)."""
        if self.control == "ssh":
            error = self.ssh.stop()
            if error:
                with self.lock:
                    self.error = "ส่ง STOP ทาง ssh ไม่ได้ (%s)" % error
            return
        try:
            self.client.post("/api/stop", {}, timeout=4.0)
        except (OSError, ValueError) as error:
            with self.lock:
                self.error = "ส่ง STOP ไม่ได้ (%s)" % error

    def _read_state(self):
        code, state = self.client.get("/api/state")
        if code != 200 or not isinstance(state, dict):
            raise OSError("/api/state HTTP %d" % code)
        job = state.get("job") or {}
        info = (job.get("job") or {})
        running = bool(job.get("running")) and info.get("kind") == "task"
        tail = job.get("tail") or []
        ended = next((m.group(1) for line in reversed(tail) if (m := STOPPED.search(line))), None)
        with self.lock:
            if info.get("started") != self.job_started:
                self.job_started = info.get("started")
                self.robot.record = None  # a new job: its records only
            self.robot.running = running
            self.robot.virtual = bool((info.get("info") or {}).get("virtual"))
            self.robot.label = info.get("label")
            self.virtual_port = int(state.get("virtual_port") or VIRTUAL_PORT)
            if not running and ended and self.robot.record is not None and not self.robot.record.get("end"):
                self.robot.record = dict(self.robot.record, end=ended)

    def _read_run(self):
        code, body = self.client.get("/api/run")
        record = (body or {}).get("record") if isinstance(body, dict) else None
        if isinstance(record, dict):
            with self.lock:
                self.robot.record = record
                self._check_frame(record.get("pose"))

    def _read_ssh(self):
        """--control ssh: run_follow's own /status and /scan through the tunnel. No answer: no run."""
        self.ssh.ensure_tunnel()
        try:
            code, body = self.tunnel.get("/status", timeout=1.0)
            record = (body or {}).get("record") if code == 200 and isinstance(body, dict) else None
        except (OSError, ValueError):
            record = None
        with self.lock:
            was = self.robot.running
            self.robot.running = record is not None
            if record is not None:
                self.robot.record, self.robot.virtual = record, "virtual" in record
                self.robot.label = "ssh %s" % self.ssh.host
                self._check_frame(record.get("pose"))
            last = self.robot.record
        if record is None and was and last is not None and not last.get("end"):
            end = self.ssh.last_end()  # the end record may have come between two reads
            with self.lock:
                if self.robot.record is last:
                    self.robot.record = dict(last, end=end or "the run's page stopped answering")
        if record is not None:
            self._read_scan(self.tunnel)

    def _read_scan(self, client=None):
        code, body = (client or self.client).get("/scan")
        if code != 200 or not isinstance(body, dict):
            return
        points = [tuple(q) for q in body.get("points") or []]
        with self.lock:
            pose = body.get("pose") or (self.robot.record or {}).get("pose")
            self.robot.gap = sim.body_gap(points)
            if pose is None:
                return
            self._check_frame(pose)
            self.scan = [sim.to_world(pose, q) for q in points]
            now = self.clock()
            if now - self.map_at >= MAP_EVERY:
                self.map_at = now
                self.grid.add_scan(pose, points)

    def _check_frame(self, pose):
        """A jump in the odometry (a run that counts from zero again): keep the person where they stand
        relative to the robot, start the map anew."""
        if pose is None:
            return
        last, self.last_pose = self.last_pose, list(pose)
        if last is None or math.dist(last[:2], pose[:2]) <= JUMP:
            return
        p = self.person
        dx, dy = p.x - last[0], p.y - last[1]
        c, s = math.cos(-last[2]), math.sin(-last[2])
        rx, ry = c * dx - s * dy, s * dx + c * dy  # in the robot frame
        c, s = math.cos(pose[2]), math.sin(pose[2])
        p.place(pose[0] + c * rx - s * ry, pose[1] + s * rx + c * ry, p.th - last[2] + pose[2])
        self.bags = [self._moved(b, last, pose) for b in self.bags]  # the bags stay where they lie
        self.grid.clear()

    @staticmethod
    def _moved(point, old, new):
        """A point kept where it is relative to the robot when its odometry frame jumps from old to new."""
        dx, dy = point[0] - old[0], point[1] - old[1]
        c, s = math.cos(-old[2]), math.sin(-old[2])
        rx, ry = c * dx - s * dy, s * dx + c * dy
        c, s = math.cos(new[2]), math.sin(new[2])
        return [new[0] + c * rx - s * ry, new[1] + s * rx + c * ry]

    def poll_loop(self):
        next_state = next_run = next_scan = 0.0
        while not self.stopping.is_set():
            now = self.clock()
            try:
                if self.control == "ssh":
                    if now >= next_run:
                        next_run = now + RUN_EVERY
                        self._read_ssh()
                    with self.lock:
                        self.connected = self.ssh.tunnel is not None
                    self.stopping.wait(0.05)
                    continue
                if now >= next_state:
                    next_state = now + STATE_EVERY
                    self._read_state()
                if now >= next_run and self.robot.running:
                    next_run = now + RUN_EVERY
                    self._read_run()
                if now >= next_scan and self.robot.running:
                    next_scan = now + SCAN_EVERY
                    self._read_scan()
                with self.lock:
                    self.connected = True
                    if self.error and self.error.startswith("คอนโซลไม่ตอบ"):
                        self.error = None
            except (OSError, ValueError) as error:
                with self.lock:  # no console: no run as far as this app knows, so nothing is sent to the robot
                    self.connected, self.error, self.robot.running = False, "คอนโซลไม่ตอบ: %s" % error, False
                self.stopping.wait(1.0)
            self.stopping.wait(0.05)

    # ------------------------------------------------------------ the person

    def sim_loop(self):
        last = self.clock()
        while not self.stopping.is_set():
            now = self.clock()
            with self.lock:
                self.person.step(now - last)
                self.auto.tick(now, self.robot)
                state = self.datagram(now)
                send = self.robot.running and self.robot.virtual
                target = (self.ssh.host if self.control == "ssh" else self.client.host, self.virtual_port)
            last = now
            if send:
                try:
                    self.sock.sendto(json.dumps(state).encode(), target)
                    self.sent += 1
                    self.udp_error = None
                except OSError as error:  # Windows: an ICMP "port unreachable" earlier shows up here
                    self.udp_error = str(error)
            self.stopping.wait(max(0.0, SEND_EVERY - (self.clock() - now)))

    def datagram(self, now):
        """The person's state and the virtual bags, as virtual_operator.py on the Pi takes them (call with the lock)."""
        return dict(self.person.state(now), bags=[[round(v, 4) for v in b] for b in self.bags])

    def place_in_front(self):
        pose = self.robot.pose
        if pose is None:
            return "ยังไม่รู้ตำแหน่งหุ่น"
        x, y, th = pose
        self.person.place(x + sim.FRONT * math.cos(th), y + sim.FRONT * math.sin(th), th + math.pi)
        return None

    # ------------------------------------------------------------ the page

    def view(self):
        with self.lock:
            p, r, rec = self.person, self.robot, self.robot.record or {}
            now = self.clock()
            return {
                "person": {"x": p.x, "y": p.y, "th": p.th, "walking": p.walking, "speaking": p.speaking(now),
                           "pointing": p.pointing(now),
                           "path": [[p.x, p.y]] + [list(q) for q in p.path], "goal": p.goal, "speed": p.speed},
                "bags": [list(b) for b in self.bags],
                "robot": {"running": r.running, "virtual": r.virtual, "label": r.label, "pose": rec.get("pose"),
                          "phase": rec.get("phase"), "locked": rec.get("locked"), "stop": rec.get("stop"),
                          "nearest": rec.get("nearest"), "gap": r.gap, "cable_deg": rec.get("cable_deg"),
                          "link": rec.get("link"), "wheels": rec.get("wheels"), "virtual_report": rec.get("virtual"),
                          "operator": rec.get("operator") or rec.get("operator_kept"), "end": rec.get("end"),
                          "v": rec.get("v"), "w": rec.get("w"), "bag": rec.get("bag"), "bag_task": rec.get("bag_task"),
                          "virtual_pointing": rec.get("virtual_pointing")},
                "scan": [[round(x, 3), round(y, 3)] for x, y in self.scan],
                "auto": {"on": self.auto.on, "step": self.auto.step, "message": self.auto.message},
                "episodes": self.auto.episodes[-8:],
                "config": {"control": self.control, "ssh": "%s@%s" % (self.ssh.user, self.ssh.host),
                           "console": self.client.base, "consoles": list(CONSOLES), "pin": bool(self.client.pin),
                           "run": dict(self.run_settings), "speeds": list(SPEEDS)},
                "connected": self.connected, "error": self.error, "sent": self.sent, "udp_error": self.udp_error,
                "map_version": self.grid.version,
            }

    def map_json(self, since):
        with self.lock:
            if since == self.grid.version:
                return {"version": since, "same": True}
            return {"version": self.grid.version, "cell": sim.CELL, "occupied": [list(c) for c in self.grid.occupied()]}

    def post(self, path, body):
        """(code, answer) for one POST from the page."""
        now = self.clock()
        if path == "/api/stop":  # first and without the lock's wait: STOP
            self.stop_run()
            with self.lock:
                self.auto.stop(now, stop_robot=False)
                self.person.halt()
            return 200, {"error": None}
        with self.lock:
            if path == "/api/config":
                return self._config(body)
            if path == "/api/say":
                try:
                    self.person.say(body.get("word"), now)
                except ValueError as error:
                    return 400, {"error": str(error)}
                return 200, {"error": None}
            if path == "/api/goal":
                x, y = body.get("x"), body.get("y")
                if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (x, y)):
                    return 400, {"error": "x and y are numbers"}
                if self.auto.on:
                    return 409, {"error": "รอบอัตโนมัติกำลังวิ่ง กดหยุดก่อน"}
                robot = self.robot.pose[:2] if self.robot.pose else None
                path_ = sim.astar(self.grid, (self.person.x, self.person.y), (x, y), robot_xy=robot)
                if path_ is None:
                    return 409, {"error": "ไปไม่ได้ จุดนั้นยังไม่เห็นบนแผนที่หรือชิดของเกิน 0.3 m"}
                self.person.go(path_)
                return 200, {"error": None, "path": path_}
            if path == "/api/point":
                return self._point(body, now)
            if path == "/api/bags":
                return self._bags(body)
            if path == "/api/place":
                error = self.place_in_front()
                return (409 if error else 200), {"error": error}
            if path == "/api/auto":
                if body.get("on") is True:
                    self.auto.start(now)
                else:
                    self.auto.stop(now, stop_robot=False)
                return 200, {"error": None}
            if path == "/api/auto/start":  # for scripts: optional run settings, then the loop
                if body:
                    code, answer = self._config({"run": dict(self.run_settings, **body)})
                    if code != 200:
                        return code, answer
                self.auto.start(now)
                return 200, {"error": None, "run": dict(self.run_settings)}
        if path == "/api/auto/stop":  # the loop off, the person stops, the run gets STOP
            self.stop_run()
            with self.lock:
                self.auto.stop(now, stop_robot=False)
                self.person.halt()
            return 200, {"error": None}
        if path == "/api/run/start":
            error = self.start_run()
            return (409 if error else 200), {"error": error}
        return 404, {"error": "not found"}

    def _point(self, body, now):
        """{"x", "y"}: point at that floor spot; {"bag": true}: at the bag the robot reports; {"clear": true}."""
        if body.get("clear"):
            self.person.point(None, now)
            return 200, {"error": None, "pointing": None}
        if body.get("bag"):
            bag = (self.robot.record or {}).get("bag")
            if not (isinstance(bag, list) and len(bag) >= 2):
                return 409, {"error": "หุ่นยังไม่รายงานตำแหน่งถุง คลิกพื้นจุดที่จะชี้แทน", "bag": None}
            spot = bag[:2]
        else:
            spot = [body.get("x"), body.get("y")]
        seconds = body.get("seconds", sim.POINT_S)
        if isinstance(seconds, bool) or not isinstance(seconds, (int, float)) or not 0 < seconds <= 600:
            return 400, {"error": "seconds 0 to 600"}
        try:
            self.person.point(spot, now, seconds)
        except (ValueError, TypeError):
            return 400, {"error": "point at x, y numbers"}
        return 200, {"error": None, "pointing": self.person.pointing(now)}

    def _bags(self, body):
        """{"x", "y"}: a virtual bag there (at most sim.MAX_BAGS); {"clear": true}: none."""
        if body.get("clear"):
            self.bags = []
            return 200, {"error": None, "bags": []}
        x, y = body.get("x"), body.get("y")
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in (x, y)):
            return 400, {"error": "x and y are numbers"}
        if len(self.bags) >= sim.MAX_BAGS:
            return 409, {"error": "วางถุงได้ไม่เกิน %d ใบ ล้างก่อน" % sim.MAX_BAGS, "bags": [list(b) for b in self.bags]}
        self.bags.append([float(x), float(y)])
        return 200, {"error": None, "bags": [list(b) for b in self.bags]}

    def status(self):
        """GET /api/auto/status: the loop, the robot, the person and the episodes, without the scan."""
        v = self.view()
        v.pop("scan")
        return v

    def _config(self, body):
        if "control" in body:
            if body["control"] not in ("ssh", "console"):
                return 400, {"error": "control is ssh or console"}
            self.control, self.robot, self.last_pose = body["control"], sim.Robot(), None
        if "console" in body:
            url = str(body["console"]).strip().rstrip("/")
            parts = urllib.parse.urlsplit(url)
            if parts.scheme != "http" or not parts.hostname:
                return 400, {"error": "ที่อยู่คอนโซลต้องเป็น http://host:port"}
            self.client.base = url
            self.connected, self.robot, self.last_pose, self.job_started = False, sim.Robot(), None, None
        if "pin" in body:
            self.client.pin = str(body["pin"]) or None
        if "speed" in body:
            speed = body["speed"]
            if not isinstance(speed, (int, float)) or not 0.05 <= speed <= 1.5:
                return 400, {"error": "ความเร็วเดิน 0.05 ถึง 1.5 m/s"}
            self.person.speed = float(speed)
        if "run" in body:
            run = body["run"]
            if not (isinstance(run, dict) and isinstance(run.get("drive"), bool) and run.get("speed") in SPEEDS
                    and isinstance(run.get("minutes"), (int, float)) and 1 <= run["minutes"] <= 30):
                return 400, {"error": "run: drive true/false, speed 0.08 or 0.12, minutes 1 to 30"}
            self.run_settings = {"drive": run["drive"], "speed": run["speed"], "minutes": run["minutes"]}
        return 200, {"error": None}

    def start(self):
        for loop in (self.poll_loop, self.sim_loop):
            threading.Thread(target=loop, daemon=True).start()


class Background:
    """The auto loop's actions, off its 10 Hz tick: a run start can take 10 s (the console frees the camera)."""

    def __init__(self, app):
        self.app = app

    def _later(self, work):
        threading.Thread(target=work, daemon=True).start()

    def start_run(self):
        def work():
            error = self.app.start_run()
            if error:
                with self.app.lock:
                    self.app.auto.message = "เริ่มงานไม่ได้: %s" % error
        self._later(work)
        return None

    def stop_run(self):
        self._later(self.app.stop_run)


def handler_for(app, port):
    hosts = {"127.0.0.1:%d" % port, "localhost:%d" % port}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, body, kind="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _host_ok(self):
            if self.headers.get("Host") in hosts:  # another name for this address (DNS rebinding) gets nothing
                return True
            self._send(403, {"error": "open http://127.0.0.1:%d/" % port})
            return False

        def do_GET(self):
            if not self._host_ok():
                return
            path = self.path.split("?")[0]
            try:
                if path in ("/", "/index.html"):
                    self._send(200, (HERE / "index.html").read_bytes(), "text/html; charset=utf-8")
                elif path == "/api/view":
                    self._send(200, app.view())
                elif path == "/api/auto/status":
                    self._send(200, app.status())
                elif path == "/api/map":
                    query = urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)
                    try:
                        since = int((query.get("v") or ["-1"])[0])
                    except ValueError:
                        since = -1
                    self._send(200, app.map_json(since))
                else:
                    self._send(404, {"error": "not found"})
            except OSError:
                pass

        def do_POST(self):
            if not self._host_ok():
                return
            length = max(0, min(int(self.headers.get("Content-Length") or 0), 65536))
            raw = self.rfile.read(length) if length else b""
            kind = self.headers.get("Content-Type", "").split(";")[0].strip().lower()
            if kind != "application/json" or self.headers.get(APP_HEADER) != "1":
                return self._send(403, {"error": "only this app's own page may send this"})
            try:
                body = json.loads(raw or b"{}")
            except ValueError:
                body = None
            if not isinstance(body, dict):
                return self._send(400, {"error": "send a JSON object"})
            self._send(*app.post(self.path.split("?")[0], body))
    return Handler


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--console", default=CONSOLES[0],
                    help="the robot console (default %%(default)s; over Tailscale %s)" % CONSOLES[1])
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--control", choices=("ssh", "console"), default="ssh",
                    help="start and stop runs over ssh to the Pi (default) or through the console with its PIN")
    args = ap.parse_args(argv)
    app = App(args.console, control=args.control)
    app.start()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler_for(app, args.port))
    server.daemon_threads = True
    how = "ssh %s@%s" % (app.ssh.user, app.ssh.host) if args.control == "ssh" else "console " + args.console
    print("virtual operator: http://127.0.0.1:%d/  (%s)" % (args.port, how), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        app.stopping.set()
        app.ssh.close()
        server.server_close()


if __name__ == "__main__":
    main()
