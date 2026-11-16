"""A web page for run_follow.py: the robot's state, and a few switches.

  http://<pi>:8080/          the page (status twice a second, buttons). run_follow serves it on 127.0.0.1
                              (the console proxies it, behind its PIN) unless --dashboard-host opens it
  GET  /status               {"record": the newest cycle record (run_follow.cycle; "phase" idle|follow|return|done,
                              "stop", "wheels", ...) or null before the first cycle, "controls": the switches,
                              "run": {"drive", "seconds"}, "remote": true when run_follow has --remote, "uptime": s}
                              GET /status?heartbeat=1 is also the page's heartbeat (Controls.polled): with
                              --remote the page leads the task while it polls so at least every PAGE_ALIVE
                              seconds. A plain GET /status answers the same but is no heartbeat (console.py
                              adds heartbeat=1 only to a status read a browser just asked it for).
  POST /control  {"pause": true} | {"search": false} | {"close_plan": true} | {"max_speed": 0.08} | {"end": true}
                 -> {"error": null or text, "controls": the switches}; 400 when refused
  POST /command  {"command": "follow" | "wait" | "resume" | "return" | "bag" | "bag_again" | "bag_near"}
                 -> {"error": null, "command": name},
                 400 {"error": text} for anything else. The task buttons, when voice or the Jetson is down:
                 taken in the next cycle as the --remote client's commands are, except that a page "wait"
                 holds (status held "page") until the page's own resume, follow or return; without --remote
                 "return" is Enter (the way back), and with no console (nohup) "follow" starts the run.
                 Both POSTs need Content-Type: application/json and X-HMC-Console: 1 (403 or 415 without),
                 so no page of another site can send them; {"end": true} (STOP) needs neither.
  GET  /stream               the camera, MJPEG (multipart/x-mixed-replace), at most live_feed.LIVE_FPS a second,
                              encoded only while someone watches; 503 before the first frame or without cv2
  GET  /scan                 the LiDAR from above (live_feed.LiveFeed.scan_json: points, older, nearest, age,
                              operator, people, stop, phase); 503 when the run has no live feed
  OPTIONS        any path: the CORS answer, for GET only

Every JSON answer to a GET carries Access-Control-Allow-Origin: *; a POST answer does not (console.py
proxies its POSTs from the same computer). The switches only make the robot more careful
or stop it; driving still has to be started with tools/safe_run.py, and the speed can never go above
params.json's max_speed. The page is not an emergency stop: if the Wi-Fi drops the page can do nothing.
Anyone on the network can use it.
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import ports

PORT = ports.DASHBOARD
PAGE = Path(__file__).resolve().parent / "dashboard.html"
# the task buttons: the --remote client's commands but stop and ping ("bag": the virtual operator's word for the
# flow's find_bag, run_follow's BAG phase; all three bag commands: the console's step commands)
COMMANDS = ("follow", "wait", "resume", "return", "bag", "bag_again", "bag_near")
COMMANDS_KEPT = 10  # presses kept while nobody takes them (run_follow waiting for Enter): the newest
CORS = {"Access-Control-Allow-Origin": "*", "Access-Control-Allow-Methods": "GET, OPTIONS"}  # reading only
CONSOLE_HEADER = "X-HMC-Console"  # a custom header: a page of another site cannot send it without a CORS yes
PAGE_ALIVE = 2.0  # s; the page (or console.py) polls /status at least this often while it leads the task
# The page is served from the control process: viewers cost the cycle time. Enough of them ended runs as "slow cycle".
MAX_STREAMS = 3  # camera viewers at once; one more gets 503
SCAN_CACHE = 0.1  # s; one /scan answer serves every viewer of the same scan for at most this long


class Controls:
    """The switches the page sets and run_follow.cycle reads, every cycle."""

    def __init__(self, max_speed, clock=time.monotonic):
        self.lock = threading.Lock()
        self.clock = clock  # the clock of run_follow's cycle `now`
        self.polled_at = -float("inf")  # the last GET /status: the page's heartbeat
        self.top = max_speed  # the most the page may set
        self.paused = False  # True: hold still (the senses and the brain keep running)
        self.search = True  # turn on the spot to find the operator before the first lock
        self.close_plan = True  # plan through gaps with the outline (hmc navigation.CLOSE_PLAN)
        self.max_speed = max_speed
        self.end = False  # True: stop the wheels and end the run
        self.commands = []  # task buttons pressed since run_follow last took them, in order

    def snapshot(self):
        with self.lock:
            return {"paused": self.paused, "search": self.search, "close_plan": self.close_plan,
                    "max_speed": self.max_speed, "top_speed": self.top, "end": self.end}

    def apply(self, change):
        """Apply one change from the page. Returns an error text, or None when it was taken."""
        if not isinstance(change, dict) or len(change) != 1:
            return "send one switch at a time, as a JSON object"
        (key, value), = change.items()
        with self.lock:
            if key in ("pause", "search", "close_plan", "end"):
                if not isinstance(value, bool):
                    return "%s takes true or false" % key
                if key == "end" and not value:
                    return "an ended run cannot be taken back"
                setattr(self, "paused" if key == "pause" else key, value)
                return None
            if key == "max_speed":
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 <= value <= self.top:
                    return "max_speed takes a number from 0 to %.2f" % self.top
                self.max_speed = float(value)
                return None
        return "unknown switch %r" % key

    def command(self, name):
        """Queue one task button for run_follow's next cycle. Returns an error text, or None when it was taken."""
        if not isinstance(name, str) or name not in COMMANDS:
            return "command takes one of %s" % ", ".join(COMMANDS)
        with self.lock:
            self.commands = (self.commands + [name])[-COMMANDS_KEPT:]
        return None

    def take_commands(self):
        """The task buttons pressed since the last call, in order."""
        with self.lock:
            taken, self.commands = self.commands, []
        return taken

    def polled(self):
        """A GET /status came: the page (or console.py) is still there."""
        with self.lock:
            self.polled_at = self.clock()

    def page_alive(self, now):
        """True when /status was polled less than PAGE_ALIVE before now."""
        with self.lock:
            return now - self.polled_at < PAGE_ALIVE


class Dashboard:
    """Serves the page from a background thread. latest is set by run_follow after every cycle."""

    def __init__(self, controls, run_info, port=PORT, host="0.0.0.0", remote=False, feed=None):
        self.controls, self.run_info, self.remote = controls, run_info, remote
        self.latest = None
        self.feed = feed  # live_feed.LiveFeed: the camera and the LiDAR for /stream and /scan, or None
        self.streams = 0  # /stream viewers now
        self.scan_cache = (None, None, -float("inf"), None)  # (feed, its scan time, made at, JSON)
        self.lock = threading.Lock()
        self.started = time.monotonic()
        dashboard = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass  # quiet: run_follow prints its own lines

            def _send(self, code, body, kind="application/json"):
                data = body if isinstance(body, bytes) else json.dumps(body).encode()
                self.send_response(code)
                self.send_header("Content-Type", kind)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                if kind == "application/json" and self.command != "POST":
                    for name, value in CORS.items():
                        self.send_header(name, value)
                self.end_headers()
                self.wfile.write(data)

            def do_OPTIONS(self):
                self.send_response(204)
                for name, value in CORS.items():
                    self.send_header(name, value)
                self.send_header("Content-Length", "0")
                self.end_headers()

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    self._send(200, PAGE.read_bytes(), "text/html; charset=utf-8")
                elif self.path.split("?")[0] == "/status":
                    if "heartbeat=1" in self.path.partition("?")[2].split("&"):
                        dashboard.controls.polled()
                    self._send(200, dashboard.status())
                elif self.path.split("?")[0] == "/scan":
                    feed = dashboard.feed
                    if feed is None:
                        return self._send(503, {"error": "this run has no live view"})
                    self._send(200, dashboard.scan_answer(feed))
                elif self.path.split("?")[0] == "/stream":
                    self._stream()
                else:
                    self._send(404, {"error": "not found"})

            def _stream(self):
                feed = dashboard.feed
                if feed is None or not feed.has_frame():
                    return self._send(503, {"error": "no camera image yet"})
                with dashboard.lock:
                    full = dashboard.streams >= MAX_STREAMS
                    dashboard.streams += 0 if full else 1
                if full:
                    return self._send(503, {"error": "at most %d camera viewers at once" % MAX_STREAMS})
                try:
                    self._frames(feed)
                finally:
                    with dashboard.lock:
                        dashboard.streams -= 1

            def _frames(self, feed):
                frames = feed.stream()
                try:
                    jpeg = next(frames)
                except Exception as error:  # no cv2 here, or the image would not encode: the run goes on
                    return self._send(503, {"error": "the camera image cannot be encoded (%s)" % error})
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    while True:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg))
                        self.wfile.write(jpeg + b"\r\n")
                        jpeg = next(frames)
                except Exception:  # the viewer went away, or an image would not encode
                    return

            def do_POST(self):
                if self.path not in ("/control", "/command"):
                    return self._send(404, {"error": "not found"})
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    change = json.loads(self.rfile.read(min(length, 4096)) or b"null")
                except ValueError:
                    return self._send(400, {"error": "not JSON"})
                if not (self.path == "/control" and change == {"end": True}):  # STOP must always work
                    if self.headers.get(CONSOLE_HEADER) != "1":
                        return self._send(403, {"error": "%s: 1 is needed" % CONSOLE_HEADER})
                    if self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
                        return self._send(415, {"error": "Content-Type: application/json is needed"})
                if self.path == "/command":
                    if not isinstance(change, dict) or set(change) != {"command"}:
                        return self._send(400, {"error": 'send {"command": NAME}, NAME one of ' + ", ".join(COMMANDS)})
                    error = dashboard.controls.command(change["command"])
                    if error:
                        return self._send(400, {"error": error})
                    return self._send(200, {"error": None, "command": change["command"]})
                error = dashboard.controls.apply(change)
                self._send(400 if error else 200, {"error": error, "controls": dashboard.controls.snapshot()})

        self.server = ThreadingHTTPServer((host, port), Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.server.shutdown()
        self.server.server_close()

    def scan_answer(self, feed):
        """feed.scan_json, made once per scan for every viewer (and anew after SCAN_CACHE, so "age" stays true)."""
        now = feed.clock()
        with self.lock:
            cached_feed, at, made, body = self.scan_cache
            if cached_feed is feed and at == feed.at and now - made < SCAN_CACHE:
                return body
        body = feed.scan_json(now)
        with self.lock:
            self.scan_cache = (feed, feed.at, now, body)
        return body

    def status(self):
        return {"record": self.latest, "controls": self.controls.snapshot(), "run": self.run_info,
                "remote": self.remote, "uptime": round(time.monotonic() - self.started, 1)}
