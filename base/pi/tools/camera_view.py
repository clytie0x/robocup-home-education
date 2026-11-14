"""Live view of the robot's camera and LiDAR scan in a browser, for people watching the robot.

  .venv/bin/python tools/camera_view.py [--port 8081] [--fps 30] [--no-lidar]
  then open http://<pi address>:8081/ on any device on the same network: the home page shows
  everything; /camera and /lidar show one each. The pages are plain files in tools/viewer/.
  The arm's USB webcam on the Jetson (the arm team's ~/so100_ws/webcam_view.py, port 8082) is
  relayed over the LAN as /usb-stream, so this never opens that camera itself.

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
KEEP_SCANS = 5  # revolutions drawn together (0.5 s at 10 Hz), the older ones fainter

VIEWER = Path(__file__).resolve().parent / "viewer"  # the pages: plain HTML, CSS and JS
FILES = {  # URL path -> (file in VIEWER, content type)
    "/": ("index.html", "text/html; charset=utf-8"),
    "/camera": ("camera.html", "text/html; charset=utf-8"),
    "/lidar": ("lidar.html", "text/html; charset=utf-8"),
    "/style.css": ("style.css", "text/css; charset=utf-8"),
    "/lidar.js": ("lidar.js", "text/javascript; charset=utf-8"),
}


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
        self.points, self.at, self.older = [], None, []

    def put(self, points, now):
        with self.lock:
            if self.at is not None:
                self.older = ([self.points] + self.older)[:KEEP_SCANS - 1]
            self.points, self.at = points, now

    def json(self, now):
        import json
        import math
        def keep(points):
            return [[round(x, 3), round(y, 3)] for x, y in points if math.hypot(x, y) <= SCAN_RANGE]
        with self.lock:
            pts, at = keep(self.points), self.at
            older = [keep(o) for o in self.older]
        near = min((math.hypot(x, y) for x, y in pts), default=None)
        return json.dumps({"points": pts, "older": older, "nearest": near,
                           "age": None if at is None else now - at}).encode()


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


def relay(handler, url):
    """Copy the Jetson's USB camera stream to one viewer, byte for byte (no decoding on the Pi)."""
    import urllib.request
    try:
        upstream = urllib.request.urlopen(url, timeout=3)
    except OSError:
        handler.send_error(502, "USB camera not reachable")
        return
    with upstream:
        handler.send_response(200)
        handler.send_header("Content-Type", upstream.headers.get("Content-Type", "application/octet-stream"))
        handler.send_header("Cache-Control", "no-store")
        handler.end_headers()
        try:
            while True:
                chunk = upstream.read1(65536)
                if not chunk:
                    return
                handler.wfile.write(chunk)
        except (OSError, BrokenPipeError, ConnectionResetError):
            return


def handler_for(latest, fps, scan=None, usb_url=None):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path in FILES:
                name, kind = FILES[self.path]
                body = (VIEWER / name).read_bytes()  # read each time: edit a page, reload the browser
                self.send_response(200)
                self.send_header("Content-Type", kind)
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
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
            elif self.path.split("?")[0] == "/usb-stream" and usb_url:  # ?t= forces a fresh retry
                relay(self, usb_url)
            else:
                self.send_error(404)
    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8081)
    ap.add_argument("--fps", type=float, default=30.0, help="frames a second to each viewer (the Xtion gives 30)")
    ap.add_argument("--no-lidar", action="store_true", help="camera only (the LiDAR is in use elsewhere)")
    ap.add_argument("--usb-port", type=int, default=8082,
                    help="port of the arm team's webcam_view.py on jetson_host, relayed as /usb-stream")
    args = ap.parse_args()
    import params
    usb_url = "http://%s:%d/stream" % (params.load()["jetson_host"], args.usb_port)
    # a background job ignores SIGINT: let `kill` (SIGTERM) close the camera and LiDAR cleanly too
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    latest, scan = Latest(), Scan()
    cam = camera.Camera()
    cam.start()
    session = None
    if not args.no_lidar:
        import lidar
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
                time.sleep(0.02)
        threading.Thread(target=read_scans, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), handler_for(latest, args.fps, scan, usb_url))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print("camera view on port %d" % args.port, flush=True)
    try:
        while True:
            frame = cam.latest()
            if frame is not None:
                latest.put(jpeg_of(frame))
            elif cam.error is not None:
                sys.exit("camera stopped: %r" % (cam.error,))
            time.sleep(0.005)  # the Xtion delivers every 33 ms: poll often enough to encode each frame
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        cam.stop()
        if session is not None:
            session.close()


if __name__ == "__main__":
    main()
