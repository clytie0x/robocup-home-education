"""Live view of the arm's webcam in a browser: http://<jetson>:8082/  (also /snap.jpg = one frame).

The AX1080P is opened as MJPG 640x480 at 30 fps. That is the size front_camera.json was
calibrated at (front_camera.WIDTH, HEIGHT and fx). A larger mode would change fx unless
the pixels are scaled back. YUYV at this size is about 18 MB/s on USB; MJPG is what the
device already offers at 640x480 and 1280x720.
"""
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# 640x480 matches front_camera.json / front_camera.WIDTH, HEIGHT. Do not change it here
# without rescaling fx, cx and cy (front_camera.for_image does that only for a uniform scale).
WIDTH, HEIGHT = 640, 480
FPS = 30
FOURCC = "MJPG"
JPEG_QUALITY = 80

latest = {"jpg": None}


def configure_capture(cap, cv2, width=WIDTH, height=HEIGHT, fps=FPS, fourcc=FOURCC):
    """Ask the camera for MJPG at the calibration size. FOURCC is set before the size: V4L2
    keeps the previous pixel format if the size is set first, and that format is YUYV."""
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def open_capture(index=0, cv2=None, opener=None):
    if cv2 is None:
        import cv2 as cv2  # noqa: PLC0415
    if opener is None:
        opener = lambda i: cv2.VideoCapture(i, cv2.CAP_V4L2)
    return configure_capture(opener(index), cv2)


def grab(open_capture=open_capture):
    import cv2
    cap = open_capture()
    while True:
        ok, frame = cap.read()
        if not ok:
            latest["jpg"] = None          # camera gone: do not keep showing the last frame
            time.sleep(0.5)
            cap.release()
            cap = open_capture()
            continue
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
    # the console gate without a sourced workspace (the agent runs "cd ~/so100_ws && python3 webcam_view.py")
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path[:0] = [os.path.join(here, "src", "so100_driver"), os.path.join(here, "so100_driver")]
    from so100_driver.console_gate import require as console_gate  # noqa: E402
    console_gate("webcam_view")  # /dev/video0 has one owner: the agent's webcam program
    threading.Thread(target=grab, daemon=True).start()
    ThreadingHTTPServer(("0.0.0.0", 8082), H).serve_forever()


if __name__ == "__main__":
    main()
