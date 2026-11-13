"""A web page for run_follow.py: the robot's state, and a few switches.

  http://<pi>:8080/          the page (status twice a second, buttons)
  GET  /status               the newest cycle record, the switches and the run, as JSON
  POST /control  {"pause": true} | {"search": false} | {"close_plan": true} | {"max_speed": 0.08} | {"end": true}

The switches only make the robot more careful or stop it; driving still has to be started with
tools/safe_run.py, and the speed can never go above params.json's max_speed. The page is not an
emergency stop: if the Wi-Fi drops the page can do nothing. Anyone on the network can use it.
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

PORT = 8080
PAGE = Path(__file__).resolve().parent / "dashboard.html"


class Controls:
    """The switches the page sets and run_follow.cycle reads, every cycle."""

    def __init__(self, max_speed):
        self.lock = threading.Lock()
        self.top = max_speed  # the most the page may set
        self.paused = False  # True: hold still (the senses and the brain keep running)
        self.search = True  # turn on the spot to find the operator before the first lock
        self.close_plan = True  # plan through gaps with the outline (hmc navigation.CLOSE_PLAN)
        self.max_speed = max_speed
        self.end = False  # True: stop the wheels and end the run

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


class Dashboard:
    """Serves the page from a background thread. latest is set by run_follow after every cycle."""

    def __init__(self, controls, run_info, port=PORT, host="0.0.0.0"):
        self.controls, self.run_info = controls, run_info
        self.latest = None
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
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    self._send(200, PAGE.read_bytes(), "text/html; charset=utf-8")
                elif self.path == "/status":
                    self._send(200, dashboard.status())
                else:
                    self._send(404, {"error": "not found"})

            def do_POST(self):
                if self.path != "/control":
                    return self._send(404, {"error": "not found"})
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    change = json.loads(self.rfile.read(min(length, 4096)) or b"null")
                except ValueError:
                    return self._send(400, {"error": "not JSON"})
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

    def status(self):
        return {"record": self.latest, "controls": self.controls.snapshot(), "run": self.run_info,
                "uptime": round(time.monotonic() - self.started, 1)}
