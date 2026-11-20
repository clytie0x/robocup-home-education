"""The camera in its own process, for the console's live view: JPEG frames on stdout, each as a 4-byte
big-endian length and the bytes. Ends on SIGTERM, on stdin closing, or when stdout is closed.

Why a process: OpenNI shut down and started again inside one long-lived process crashed the console
twice on 2026-11-15 (Segmentation fault at 03:23:16 and 03:28:51, as a job took the camera). Here the
crash, if any, ends only this process; the console starts a fresh one later.

The page is a preview. Encoding every Xtion frame (about 22/s) at JPEG quality 80 was the top
userspace cost on the Pi while the console was idle (camera_pipe 17-29 %). This process encodes
CONSOLE_FPS frames a second at CONSOLE_JPEG_QUALITY. Follow does not read this pipe: run_follow
opens camera.Camera() itself, at the sensor rate, in its own process.

  .venv/bin/python tools/camera_pipe.py      # run by console.py, not by hand
"""
import signal
import struct
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

# Console preview only. Follow's camera.Camera() is a different process and is not capped here.
CONSOLE_JPEG_QUALITY = 50
CONSOLE_FPS = 8.0


def next_encode_at(now, scheduled, fps=CONSOLE_FPS):
    """The time the next JPEG may be sent, or None while `now` is still inside the last period."""
    if now + 1e-9 < scheduled:
        return None
    return now + 1.0 / fps


def main():
    import camera
    import camera_view

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    threading.Thread(target=lambda: (sys.stdin.read(), stop.set()), daemon=True).start()   # parent gone
    out = sys.stdout.buffer
    cam = camera.Camera()
    cam.start()
    scheduled = 0.0
    try:
        while not stop.is_set():
            frame = cam.latest()
            now = time.monotonic()
            due = next_encode_at(now, scheduled)
            if frame is not None and due is not None:
                jpeg = camera_view.jpeg_of(frame, quality=CONSOLE_JPEG_QUALITY)
                out.write(struct.pack(">I", len(jpeg)) + jpeg)
                out.flush()
                scheduled = due
            elif cam.error is not None:
                print("camera stopped: %s" % (cam.error,), file=sys.stderr)
                return 1
            time.sleep(0.02)
    except BrokenPipeError:
        return 0
    finally:
        cam.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
