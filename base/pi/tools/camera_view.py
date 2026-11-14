"""Live view of the robot's camera and LiDAR scan in a browser, for people watching the robot.

  .venv/bin/python tools/camera_view.py [--port 8081] [--fps 5] [--no-lidar]
  then open http://<pi address>:8081/ (camera) or http://<pi address>:8081/lidar (scan) on any
  device on the same network.

Read-only: it never opens the wheel port. The camera and the LiDAR can each be open in one
program at a time, so stop this before starting run_follow or any drive tool (and the other way
round). The image is shown flipped back to how a person sees the scene (the Xtion sends it
mirrored). The scan is drawn from above in the robot frame: forward up, left to the left.
"""
import argparse
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import camera  # noqa: E402

SCAN_RANGE = 4.0  # m drawn from the centre

PAGE = b"""<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Robot camera</title><style>body{margin:0;background:#111;color:#ccc;font:14px sans-serif;text-align:center}
img{width:100%;max-width:960px;image-rendering:auto}</style></head>
<body><img src="/stream" alt="camera"><p>Robot camera (Xtion, 320x240) &middot; <a href="/lidar" style="color:#4af">LiDAR scan</a></p></body></html>"""

LIDAR_PAGE = b"""<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Robot LiDAR</title><style>body{margin:0;background:#111;color:#ccc;font:14px sans-serif;text-align:center}
img{width:100%;max-width:960px;image-rendering:auto}</style></head>
<body><canvas id="scan" width="480" height="480" style="width:100%;max-width:480px;background:#000"></canvas>
<p id="info">LiDAR scan from above: forward is up, grid every 1 m, robot outline in the centre</p>
<script>
const c = document.getElementById("scan"), g = c.getContext("2d"), R = 4.0, s = c.width / (2 * R);
function px(x, y) { return [c.width / 2 - y * s, c.height / 2 - x * s]; }
async function tick() {
  try {
    const r = await fetch("/scan", {cache: "no-store"}); const d = await r.json();
    g.fillStyle = "#000"; g.fillRect(0, 0, c.width, c.height);
    g.strokeStyle = "#222"; for (let m = 1; m <= R; m++) { g.beginPath(); g.arc(c.width/2, c.height/2, m*s, 0, 7); g.stroke(); }
    g.strokeStyle = "#4af"; g.beginPath(); const o = [[0.2,0.16],[0.16,0.2],[-0.16,0.2],[-0.2,0.16],[-0.2,-0.16],[-0.16,-0.2],[0.16,-0.2],[0.2,-0.16]];
    o.forEach((p, i) => { const q = px(p[0], p[1]); i ? g.lineTo(q[0], q[1]) : g.moveTo(q[0], q[1]); }); g.closePath(); g.stroke();
    const f = px(0.35, 0); g.beginPath(); g.moveTo(c.width/2, c.height/2); g.lineTo(f[0], f[1]); g.stroke();
    g.fillStyle = "#f44"; for (const p of d.points) { const q = px(p[0], p[1]); g.fillRect(q[0]-1, q[1]-1, 3, 3); }
    document.getElementById("info").textContent = d.points.length + " points, nearest " + (d.nearest == null ? "-" : d.nearest.toFixed(2) + " m") + (d.age == null ? "" : ", " + d.age.toFixed(1) + " s old");
  } catch (e) {}
  setTimeout(tick, 200);
}
tick();
</script>
<p><a href="/" style="color:#4af">camera</a></p></body></html>"""


def jpeg_of(frame, flip=True):
    """JPEG bytes of a camera.Frame, mirrored back when flip."""
    import cv2
    import numpy as np

    rgb = np.frombuffer(frame.rgb, dtype=np.uint8).reshape(frame.height, frame.width, 3)
    if flip:
        rgb = rgb[:, ::-1]
    ok, jpeg = cv2.imencode(".jpg", np.ascontiguousarray(rgb[:, :, ::-1]), [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        raise ValueError("JPEG encoding failed")
    return jpeg.tobytes()


class Scan:
    """The newest LiDAR points (robot frame), shared with the web clients as JSON."""

    def __init__(self):
        self.lock = threading.Lock()
        self.points, self.at = [], None

    def put(self, points, now):
        with self.lock:
            self.points, self.at = points, now

    def json(self, now):
        import json
        import math
        with self.lock:
            pts = [[round(x, 3), round(y, 3)] for x, y in self.points if math.hypot(x, y) <= SCAN_RANGE]
            at = self.at
        near = min((math.hypot(x, y) for x, y in pts), default=None)
        return json.dumps({"points": pts, "nearest": near, "age": None if at is None else now - at}).encode()


class Latest:
    """The newest JPEG, shared between the camera thread and the web clients."""

    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg, self.n = None, 0

    def put(self, jpeg):
        with self.cond:
            self.jpeg, self.n = jpeg, self.n + 1
            self.cond.notify_all()

    def wait_newer(self, n, timeout=2.0):
        with self.cond:
            self.cond.wait_for(lambda: self.n > n, timeout)
            return self.jpeg, self.n


def handler_for(latest, fps, scan=None):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(PAGE)
            elif self.path == "/lidar":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(LIDAR_PAGE)
            elif self.path == "/scan":
                body = (scan or Scan()).json(time.monotonic())
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                n = 0
                try:
                    while True:
                        jpeg, n = latest.wait_newer(n)
                        if jpeg is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg))
                        self.wfile.write(jpeg + b"\r\n")
                        time.sleep(1.0 / fps)
                except (BrokenPipeError, ConnectionResetError):
                    return
            else:
                self.send_error(404)
    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8081)
    ap.add_argument("--fps", type=float, default=5.0)
    ap.add_argument("--no-lidar", action="store_true", help="camera only (the LiDAR is in use elsewhere)")
    args = ap.parse_args()
    # a background job ignores SIGINT: let `kill` (SIGTERM) close the camera and LiDAR cleanly too
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    latest, scan = Latest(), Scan()
    cam = camera.Camera()
    cam.start()
    session = None
    if not args.no_lidar:
        import lidar
        import params
        import run_follow
        p = params.load()
        session = lidar.LidarSession()

        def read_scans():
            last = None
            while True:
                s = session.latest()
                if s and s is not last:
                    scan.put([tuple(q) for q in run_follow.scan_points(s, p)], time.monotonic())
                    last = s
                time.sleep(0.1)
        threading.Thread(target=read_scans, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), handler_for(latest, args.fps, scan))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print("camera view on port %d" % args.port, flush=True)
    try:
        while True:
            frame = cam.latest()
            if frame is not None:
                latest.put(jpeg_of(frame))
            elif cam.error is not None:
                sys.exit("camera stopped: %r" % (cam.error,))
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        cam.stop()
        if session is not None:
            session.close()


if __name__ == "__main__":
    main()
