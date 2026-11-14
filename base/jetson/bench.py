"""Frames-per-second bench on the Jetson: detector plus look vectors on 320 x 240 frames with two people.

Run: .venv/bin/python bench.py --detector models/yolo11n_256x320.engine --reid models/osnet_x0_25_msmt17_256x128.engine \
         --image models/two_people_forest.jpg [--frames 300] [--detector-only]
Prints one JSON line. Frames are 320 x 240 windows sliding across the image (already decoded, no JPEG cost).
"""

import argparse
import json
import time

import cv2
import numpy as np

from detector import Detector
from engine import gpu_free_bytes
from reid import ReId


def frames(path, count):
    img = cv2.imread(path)
    if img is None:
        raise SystemExit(f"cannot read {path}")
    wide = cv2.resize(img, (360, 240))  # 40 px of room to slide the 320 px window
    return [np.ascontiguousarray(wide[:, x:x + 320]) for x in (np.arange(count) % 41)]


def ram_mib(field):
    with open("/proc/self/status") as f:
        for line in f:
            if line.startswith(field + ":"):
                return int(line.split()[1]) / 1024
    return float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--detector", required=True)
    ap.add_argument("--reid", required=True)
    ap.add_argument("--image", required=True)
    ap.add_argument("--frames", type=int, default=300)
    ap.add_argument("--detector-only", action="store_true")
    args = ap.parse_args()

    shots = frames(args.image, args.frames)
    free_before, _ = gpu_free_bytes()
    det = Detector(args.detector)
    reid = None if args.detector_only else ReId(args.reid)
    for shot in shots[:20]:  # warm up
        boxes = det.detect(shot)
        if reid:
            reid.looks(shot, [b[:4] for b in boxes])

    times, people = [], []
    for shot in shots:
        t = time.perf_counter()
        boxes = det.detect(shot)
        if reid:
            looks = reid.looks(shot, [b[:4] for b in boxes])
            assert all(abs(sum(v * v for v in look) - 1) < 1e-3 for look in looks)
        times.append(time.perf_counter() - t)
        people.append(len(boxes))
    free_after, _ = gpu_free_bytes()

    fps = 1.0 / np.array(times)
    print(json.dumps({
        "detector": args.detector, "reid": None if args.detector_only else args.reid, "frames": len(times),
        "fps_median": round(float(np.median(fps)), 1), "fps_p5": round(float(np.percentile(fps, 5)), 1),
        "people_per_frame": {str(n): people.count(n) for n in sorted(set(people))},
        # engines + buffers, excluding the CUDA context (the first cudaMemGetInfo creates it before the baseline);
        # Orin shares RAM between CPU and GPU, so the drop in free memory also includes other teams' changes
        "gpu_engines_buffers_mib": round((free_before - free_after) / 2**20, 1),
        "ram_rss_mib": round(ram_mib("VmRSS"), 1), "ram_peak_mib": round(ram_mib("VmHWM"), 1),
    }))


if __name__ == "__main__":
    main()
