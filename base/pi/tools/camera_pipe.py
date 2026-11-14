"""The camera in its own process, for the console's live view: JPEG frames on stdout, each as a 4-byte
big-endian length and the bytes. Ends on SIGTERM, on stdin closing, or when stdout is closed.

Why a process: OpenNI shut down and started again inside one long-lived process crashed the console
twice on 2026-11-15 (Segmentation fault at 03:23:16 and 03:28:51, as a job took the camera). Here the
crash, if any, ends only this process; the console starts a fresh one later.

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


def main():
    import camera
    import camera_view

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    threading.Thread(target=lambda: (sys.stdin.read(), stop.set()), daemon=True).start()   # parent gone
    out = sys.stdout.buffer
    cam = camera.Camera()
    cam.start()
    try:
        while not stop.is_set():
            frame = cam.latest()
            if frame is not None:
                jpeg = camera_view.jpeg_of(frame)
                out.write(struct.pack(">I", len(jpeg)) + jpeg)
                out.flush()
            elif cam.error is not None:
                print("camera stopped: %s" % (cam.error,), file=sys.stderr)
                return 1
            time.sleep(0.005)
    except BrokenPipeError:
        return 0
    finally:
        cam.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
