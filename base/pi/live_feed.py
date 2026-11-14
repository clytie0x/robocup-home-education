"""The camera image and the LiDAR scan of a running run_follow, for the web console's Live tab.

The camera and the LiDAR can each be open in one program at a time, so while run_follow runs the
console cannot read them. run_follow already reads both every cycle: cycle() puts the newest frame,
the robot-frame LiDAR points and its record here (references only, it costs microseconds), and
dashboard.py's GET /stream and /scan take them out. The JPEGs are made on the web side, only while
someone watches, at most LIVE_FPS a second, and one image serves every viewer. The console relays
both as its own /stream and /scan.
"""
import json
import math
import sys
import threading
import time
from pathlib import Path

import numpy

sys.path.insert(0, str(Path(__file__).resolve().parent / "tools"))  # camera_view (jpeg_of, the scan's look)
import camera_view  # noqa: E402

LIVE_FPS = 10  # JPEGs a second at most to one viewer (the cycle gives a new frame 10 times a second)
JPEG_QUALITY = 70
STREAM_IDLE = 2.0  # s without a new frame: the stream ends, so a viewer gone meanwhile frees its /stream place


def _jpeg(frame):
    return camera_view.jpeg_of(frame, quality=JPEG_QUALITY)  # mirrored back, as a person sees the scene


def _keep(points):
    """[[x, y], ...] of the points within camera_view.SCAN_RANGE, rounded to mm."""
    if points is None:
        return []
    a = numpy.asarray(points, dtype=float).reshape(-1, 2)
    return numpy.round(a[numpy.hypot(a[:, 0], a[:, 1]) <= camera_view.SCAN_RANGE], 3).tolist()


def _in_robot(point, pose):
    """A point of the brain's frame in the robot frame (x forward, y left), or None."""
    if point is None or pose is None:
        return None
    x, y, th = pose
    dx, dy = point[0] - x, point[1] - y
    return [math.cos(th) * dx + math.sin(th) * dy, -math.sin(th) * dx + math.cos(th) * dy]


class LiveFeed:
    """The newest frame, scan points and cycle record, shared between the cycle and the web clients.

    encode: frame -> JPEG bytes (needs cv2: without it the first image raises, and /stream answers 503).
    clock, sleep: for the stream's pace; clock is the cycle's clock."""

    def __init__(self, encode=_jpeg, clock=time.monotonic, sleep=time.sleep, fps=LIVE_FPS):
        self.encode, self.clock, self.sleep, self.period = encode, clock, sleep, 1.0 / fps
        self.lock = threading.Lock()  # held only to swap references
        self.encoding = threading.Lock()  # one encode at a time: viewers asking together share its JPEG
        self.frame = None
        self.points, self.older, self.overlay, self.at = None, [], {}, None
        self.jpeg = (None, None)  # (frame, its JPEG): one encode serves every viewer

    def put(self, frame, points, overlay, now):
        """From the cycle: the frame (None keeps the last one), the robot-frame LiDAR points, the record."""
        with self.lock:
            if frame is not None:
                self.frame = frame
            if self.points is not None:
                self.older = [self.points] + self.older[:camera_view.KEEP_SCANS - 2]
            self.points, self.overlay, self.at = points, overlay, now

    def has_frame(self):
        with self.lock:
            return self.frame is not None

    def _jpeg_of(self, frame):
        with self.encoding:
            with self.lock:
                done, jpeg = self.jpeg
            if done is not frame:
                jpeg = self.encode(frame)
                with self.lock:
                    self.jpeg = (frame, jpeg)
        return jpeg

    def stream(self):
        """JPEGs for one viewer: each new frame at most fps a second, encoded only when the viewer takes it.
        Ends after STREAM_IDLE without a new frame: only a write notices a viewer that went away."""
        last, next_at, fresh_at = None, -math.inf, self.clock()
        while self.clock() - fresh_at < STREAM_IDLE:
            wait = next_at - self.clock()
            if wait > 0:
                self.sleep(wait)
            next_at = self.clock() + self.period
            with self.lock:
                frame = self.frame
            if frame is not None and frame is not last:
                last, fresh_at = frame, self.clock()
                yield self._jpeg_of(frame)

    def scan_json(self, now):
        """camera_view.Scan.json's answer, and from the record: "operator" [x, y] or null, "people"
        [[x, y], ...] (both in the robot frame), "stop" (the reason the robot holds, or null), "phase" and "pose"
        (the odometry pose of these points' cycle, for a map built from the scans: base/pc/virtual_operator)."""
        with self.lock:
            points, older, overlay, at = self.points, self.older, self.overlay, self.at
        pts = _keep(points)
        pose = overlay.get("believed")
        operator = overlay.get("operator")
        if operator is None and overlay.get("locked"):
            operator = overlay.get("operator_kept")  # not seen this cycle, but the senses still hold the track
        people = [_in_robot(q, pose) for q in overlay.get("people") or []] if pose is not None else []
        return json.dumps({"points": pts, "older": [_keep(o) for o in older],
                           "nearest": min((math.hypot(x, y) for x, y in pts), default=None),
                           "age": None if at is None else now - at, "operator": _in_robot(operator, pose),
                           "people": people, "stop": overlay.get("stop"), "phase": overlay.get("phase"),
                           "pose": overlay.get("pose")}).encode()
