"""Live view of the arm's webcam in a browser: http://<jetson>:8082/  (also /snap.jpg = one frame).

The AX1080P is opened as MJPG 640x480 at 30 fps. That is the size front_camera.json was
calibrated at (front_camera.WIDTH, HEIGHT and fx). A larger mode would change fx unless
the pixels are scaled back. YUYV at this size is about 18 MB/s on USB; MJPG is what the
device already offers at 640x480 and 1280x720.

One process owns the camera. A second start exits with the holder's pid instead of fighting for
/dev/video0 (OpenCV then prints "can't open camera by index" in a loop: 768 times over 17 restarts
on 19 Nov). A failed open waits 0, 0.5, 1, 2, then 4 s, and after that 5 s between tries.
"""
import fcntl
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# the console gate without a sourced workspace (the agent runs "cd ~/so100_ws && python3 webcam_view.py")
_here = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(_here, "src", "so100_driver"), os.path.join(_here, "so100_driver")]

# 640x480 matches front_camera.json / front_camera.WIDTH, HEIGHT. Do not change it here
# without rescaling fx, cx and cy (front_camera.for_image does that only for a uniform scale).
WIDTH, HEIGHT = 640, 480
FPS = 30
FOURCC = "MJPG"
JPEG_QUALITY = 80

OPEN_DELAYS = (0, 0.5, 1, 2, 4)
RETRY_WAIT = 5.0
latest = {"jpg": None}


def lock_path(index=0, directory=None):
    directory = directory or os.environ.get("HMC_LOCK_DIR", "/tmp")
    return os.path.join(directory, "hmc-webcam-video%s.lock" % index)


def claim_camera(index=0, directory=None):
    """Exclusive lock for this webcam. Returns the open file (keep it). SystemExit if another process holds it."""
    path = lock_path(index, directory)
    fd = open(path, "a+")
    try:
        fcntl.flock(fd.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        fd.seek(0)
        holder = fd.read().strip() or "unknown"
        fd.close()
        raise SystemExit("webcam already open (video%s, pid %s); not starting a second one" % (index, holder))
    fd.seek(0)
    fd.truncate()
    fd.write("%s\n" % os.getpid())
    fd.flush()
    return fd


def configure_capture(cap, cv2, width=WIDTH, height=HEIGHT, fps=FPS, fourcc=FOURCC):
    """Ask the camera for MJPG at the calibration size. FOURCC is set before the size: V4L2
    keeps the previous pixel format if the size is set first, and that format is YUYV."""
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def _open_v4l2(index):
    """V4L2 first (one device, no backend probe). The default backend if V4L2 does not open."""
    import cv2
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if cap is not None and cap.isOpened():
        return cap
    if cap is not None:
        cap.release()
    return cv2.VideoCapture(index)


def open_capture(index=0, open_fn=None, sleep=None, delays=OPEN_DELAYS):
    """Open the webcam, retrying with backoff. open_fn(index) -> a capture. None when every try fails."""
    open_fn = _open_v4l2 if open_fn is None else open_fn
    sleep = time.sleep if sleep is None else sleep
    for delay in delays:
        if delay:
            sleep(delay)
        cap = open_fn(index)
        opened = cap is not None and (not hasattr(cap, "isOpened") or cap.isOpened())
        if opened:
            return cap
        if cap is not None and hasattr(cap, "release"):
            cap.release()
    return None


def grab(index=0, sleep=None):
    import cv2
    sleep = time.sleep if sleep is None else sleep
    while True:
        cap = open_capture(index, sleep=sleep)
        if cap is None:
            latest["jpg"] = None
            sleep(RETRY_WAIT)
            continue
        configure_capture(cap, cv2)
        while True:
            ok, frame = cap.read()
            if not ok:
                latest["jpg"] = None  # camera gone: do not keep showing the last frame
                cap.release()
                sleep(RETRY_WAIT)
                break
            latest["jpg"] = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])[1].tobytes()


PAGE = b"""<!doctype html><html><head><meta charset="utf-8"><title>Arm webcam</title>
<style>body{background:#111;color:#ddd;font-family:sans-serif;text-align:center;margin:12px}
img{max-width:100%;border:1px solid #444}</style></head>
<body><div>Arm webcam (640x480 MJPG)</div><img src="/stream"></body></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
            self.end_headers()
            try:
                while True:
                    j = latest["jpg"]
                    if j:
                        self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(j)).encode() + b"\r\n\r\n" + j + b"\r\n")
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                return
        elif self.path == "/snap.jpg" and latest["jpg"]:
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.end_headers()
            self.wfile.write(latest["jpg"])
        elif self.path == "/snap.jpg":
            self.send_response(503)
            self.end_headers()
            self.wfile.write(b"no camera frame (is the webcam plugged in?)")
        else:
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(PAGE)


def main():
    from so100_driver.console_gate import require as console_gate
    lock = claim_camera(0)  # kept open while we serve; the kernel drops it when this process exits
    try:
        console_gate("webcam_view")  # /dev/video0 has one owner: the agent's webcam program
        threading.Thread(target=grab, daemon=True).start()
        ThreadingHTTPServer(("0.0.0.0", 8082), H).serve_forever()
    finally:
        lock.close()


if __name__ == "__main__":
    main()
