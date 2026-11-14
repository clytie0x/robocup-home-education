"""Live view of the USB camera on the Jetson (AX1080P), as an MJPEG stream for the Pi's viewer page.

  python3 usb_camera_view.py [--device PATH] [--port 8083] [--width 640] [--fps 30]
  then http://<jetson>:8083/stream is the stream; the Pi's home page shows it through
  /usb-stream on the Pi (tools/camera_view.py relays it over the LAN cable).

Read-only. A V4L2 camera can be open in one program at a time, so stop this before another
program needs the camera.
"""
import argparse
import glob
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

DEVICE_GLOB = "/dev/v4l/by-id/*AX1080P*-video-index0"


class Latest:
    """The newest JPEG, shared between the camera loop and the web clients."""

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


def handler_for(latest):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path != "/stream":
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            n = 0
            try:
                while True:
                    jpeg, n = latest.wait_newer(n)
                    if jpeg is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg))
                    self.wfile.write(jpeg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                return
    return Handler


def find_device(given):
    if given:
        return given
    found = sorted(glob.glob(DEVICE_GLOB))
    if not found:
        sys.exit("no AX1080P camera found (%s): is it plugged in?" % DEVICE_GLOB)
    return found[0]


def main():
    import cv2

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--device", help="V4L2 device (default: the AX1080P by id)")
    ap.add_argument("--port", type=int, default=8083)
    ap.add_argument("--width", type=int, default=640, help="width sent to viewers (height keeps the aspect)")
    ap.add_argument("--fps", type=float, default=30.0)
    args = ap.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    device = find_device(args.device)
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    if not cap.isOpened():
        sys.exit("cannot open %s (in use by another program?)" % device)

    latest = Latest()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), handler_for(latest))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print("usb camera %s, %dx%d, on port %d" % (device, cap.get(cv2.CAP_PROP_FRAME_WIDTH),
                                                cap.get(cv2.CAP_PROP_FRAME_HEIGHT), args.port), flush=True)
    failures = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                failures += 1
                if failures > 50:
                    sys.exit("camera stopped giving frames")
                time.sleep(0.02)
                continue
            failures = 0
            h, w = frame.shape[:2]
            if w > args.width:
                frame = cv2.resize(frame, (args.width, round(h * args.width / w)), interpolation=cv2.INTER_AREA)
            ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                latest.put(jpeg.tobytes())
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        cap.release()


if __name__ == "__main__":
    main()
