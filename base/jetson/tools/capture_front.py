"""Capture new front camera frames for the bag segmenter's training set: GET the Jetson's /snap.jpg again and again
and keep a frame only when the view changed since the last kept one, so a still scene gives one frame, not hundreds.

Read-only on the robot: GET http://<host>:8082/snap.jpg (the arm team's webcam_view.py), nothing else, no other
port, never a POST. "Changed" is the mean absolute difference of two small grey thumbnails (THUMB), in grey levels
0-255, of at least --min-diff (the rule of the v4 capture). A kept frame is the camera's JPEG bytes as sent.

  python tools/capture_front.py OUT_DIR [--host 100.108.202.48|192.168.31.137] [--every S] [--min-diff L]
Each kept frame is OUT_DIR/front_YYYYMMDD-HHMMSS.jpg (laptop local time) and one printed line. Ctrl-C stops.
"""
import argparse
import http.client
import os
import sys
import time
import urllib.request

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import ports  # noqa: E402

HOSTS = ("100.108.202.48", "192.168.31.137")  # the Jetson on Tailscale, then on the lab Wi-Fi
THUMB = (32, 24)  # px (w, h): the grey thumbnail the change test compares
MIN_DIFF = 6.0    # grey levels (0-255): mean abs difference that makes a new view
EVERY = 1.0       # s between two GETs
TIMEOUT = 3.0     # s for one GET


def snap_url(host):
    return "http://%s:%d/snap.jpg" % (host, ports.WEBCAM)


def fetch(url, timeout=TIMEOUT):
    """The JPEG bytes of one GET (no data argument: urllib sends a GET)."""
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read()


def thumb(jpeg):
    """JPEG bytes -> the small grey float thumbnail, or None when the bytes are no image."""
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE) if jpeg else None
    return None if img is None else cv2.resize(img, THUMB, interpolation=cv2.INTER_AREA).astype(np.float32)


class Gate:
    """Lets a frame through when it differs from the last one let through by at least min_diff."""

    def __init__(self, min_diff=MIN_DIFF):
        self.min_diff, self.last = min_diff, None

    def offer(self, jpeg):
        """The mean abs difference to the last kept frame (inf for the first) when this one is kept, else None."""
        t = thumb(jpeg)
        if t is None:
            return None
        diff = float("inf") if self.last is None else float(np.abs(t - self.last).mean())
        if diff < self.min_diff:
            return None
        self.last = t
        return diff


def frame_path(out_dir, t):
    """out_dir/front_YYYYMMDD-HHMMSS.jpg for time t; -2, -3 ... when that second has a frame already."""
    stem = os.path.join(out_dir, time.strftime("front_%Y%m%d-%H%M%S", time.localtime(t)))
    path, n = stem + ".jpg", 1
    while os.path.exists(path):
        n += 1
        path = "%s-%d.jpg" % (stem, n)
    return path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out", help="folder for the kept frames (made when missing)")
    ap.add_argument("--host", choices=HOSTS, default=HOSTS[0])
    ap.add_argument("--every", type=float, default=EVERY, help="seconds between two GETs")
    ap.add_argument("--min-diff", type=float, default=MIN_DIFF, help="grey levels: mean abs difference to keep")
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    url, gate, kept = snap_url(a.host), Gate(a.min_diff), 0
    print("GET %s every %.1f s, keeping views that differ by %.1f grey levels or more; Ctrl-C stops"
          % (url, a.every, a.min_diff), flush=True)
    try:
        while True:
            t = time.time()
            try:
                jpeg = fetch(url)
            except (OSError, http.client.HTTPException) as e:
                print("no frame: %s" % e, flush=True)
            else:
                diff = gate.offer(jpeg)
                if diff is not None:
                    path = frame_path(a.out, t)
                    with open(path, "wb") as f:
                        f.write(jpeg)
                    kept += 1
                    print("%3d %s diff %.1f" % (kept, os.path.basename(path), diff), flush=True)
            time.sleep(max(0.0, a.every - (time.time() - t)))
    except KeyboardInterrupt:
        print("stopped: %d frames kept in %s" % (kept, a.out))
    return kept


if __name__ == "__main__":
    main()
