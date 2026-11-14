"""Live view of the robot's Xtion colour camera in a browser, for people watching the robot.

  .venv/bin/python tools/camera_view.py [--port 8081] [--fps 5]
  then open http://<pi address>:8081/ on any device on the same network.

Read-only: it never opens the wheel port. The camera can be open in one program at a time, so
stop this before starting run_follow (and the other way round). The image is shown flipped back
to how a person sees the scene (the Xtion sends it mirrored).
"""
import argparse
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import camera  # noqa: E402

PAGE = b"""<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Robot camera</title><style>body{margin:0;background:#111;color:#ccc;font:14px sans-serif;text-align:center}
img{width:100%;max-width:960px;image-rendering:auto}</style></head>
<body><img src="/stream" alt="camera"><p>Robot camera (Xtion, 320x240)</p></body></html>"""


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


def handler_for(latest, fps):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(PAGE)
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
    args = ap.parse_args()
    latest = Latest()
    cam = camera.Camera()
    cam.start()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), handler_for(latest, args.fps))
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


if __name__ == "__main__":
    main()
