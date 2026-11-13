"""The base's measured motion: wheel odometry, corrected by LiDAR scan matching (Task M1).

Odometry steps by 0.01 m and turns with the measured scales; the LiDAR sees the room. Every new
revolution is matched to the one about BASELINE seconds before, with the odometry motion since
then as the starting guess. A match counts only when enough points agree and it stays near the
odometry; otherwise the odometry step is kept. The pose is built in the odometry frame, and v, w
are the matched motion over its span (or the last odometry step over its time).
"""
import math
from dataclasses import dataclass

import scanmatch

AGREE_MIN = 0.6  # share of points that must agree (spec: below 60 % the estimate is not trusted)
# A match further than this from the odometry step is refused: a floor, plus half the step itself
# (a step is about 0.012 m at 0.12 m/s; a fixed 0.05 m let a match the wrong way through, review).
MAX_POS = 0.02  # m
MAX_YAW = math.radians(2)
# Returns per scan given to the matcher. On the Pi (235 pairs of saved scans 1 s apart, no guess):
# 150 points took 26.9 ms median, 91 ms worst (over the 80 ms budget), 84 % trusted; 120 points
# 14.4 ms median, 63 ms worst, 71 % trusted. Scans 0.1 s apart with the odometry guess agree more.
MAX_POINTS = 120
# s: v and w come from a match against the scan this long ago. Matched 0.1 s apart (one revolution),
# each step read short: on the base_check line of 2026-11-14 05:13 the 0.1 s steps summed to
# -0.0015 rad/s where the start-to-end match said -0.0071; 0.5 s apart they gave -0.0065 (18.7 ms median).
BASELINE = 0.5
KEEP = 2.0  # s of recent scans kept to match against


@dataclass
class MotionState:
    t: float
    pose: tuple  # (x, y, yaw), odometry frame
    v: float  # m/s, forward over span
    w: float  # rad/s over span
    trusted: bool  # this step came from a scan match that agreed
    agree: float  # the match's share of agreeing points (0 when none was made)
    span: float = 0.0  # s that v and w average over, ending at t


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _match(before, after, p, guess):
    return scanmatch.match(before, after, p, guess, max_points=MAX_POINTS)


def _relative(a, b):
    """Pose b in the frame of pose a."""
    (x0, y0, th0), (x1, y1, th1) = a, b
    c, s = math.cos(th0), math.sin(th0)
    dx, dy = x1 - x0, y1 - y0
    return (c * dx + s * dy, -s * dx + c * dy, _wrap(th1 - th0))


def _compose(pose, step):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return (x + c * step[0] - s * step[1], y + s * step[0] + c * step[1], th + step[2])


class MotionEstimator:
    def __init__(self, p, match=_match):
        self.p, self.match = p, match
        self.t = self.odom = self.scan = self.pose = None
        self.keys = []  # (t, odometry pose, scan, pose) at each new scan, the last KEEP seconds
        self.refused = 0  # matches refused for being too far from the odometry

    def update(self, t, odom_pose, scan, command=None):
        """The motion state at time t from the odometry pose and the newest scan.

        command: the (v, w) sent, unused for now (the online estimates, Task M2, will use it).
        """
        if self.t is not None and t < self.t:
            raise ValueError("time went backward: %.3f after %.3f" % (t, self.t))
        if self.t is None:
            self.t, self.odom, self.scan, self.pose = t, odom_pose, scan, tuple(odom_pose)
            if scan:
                self.keys = [(t, odom_pose, scan, self.pose)]
            return MotionState(t, self.pose, 0.0, 0.0, False, 0.0)
        step = _relative(self.odom, odom_pose)  # the odometry step, in the frame of the previous update
        dt = t - self.t
        pose = _compose(self.pose, step)
        v = math.copysign(math.hypot(step[0], step[1]), step[0]) / dt if dt > 0 else 0.0
        w = step[2] / dt if dt > 0 else 0.0
        trusted, agree, span = False, 0.0, dt
        if scan and self.scan and scan is not self.scan and scan != self.scan:
            older = [k for k in self.keys if k[0] <= t - BASELINE + 1e-9]
            key = older[-1] if older else (self.keys[0] if self.keys else None)
            if key is not None:
                guess = _relative(key[1], odom_pose)
                try:
                    mx, my, mth, agree = self.match(key[2], scan, self.p, guess)
                except ValueError:
                    pass  # too few points, or no overlap: keep the odometry
                else:
                    if agree >= AGREE_MIN:
                        near = (math.hypot(mx - guess[0], my - guess[1]) <= MAX_POS + 0.5 * math.hypot(guess[0], guess[1])
                                and abs(_wrap(mth - guess[2])) <= MAX_YAW + 0.5 * abs(guess[2]))
                        span = t - key[0]
                        if near and span > 0:
                            pose, trusted = _compose(key[3], (mx, my, mth)), True
                            v = math.copysign(math.hypot(mx, my), mx) / span
                            w = mth / span
                        else:
                            self.refused += 1
                            span = dt
            self.keys = [k for k in self.keys if k[0] > t - KEEP] + [(t, odom_pose, scan, pose)]
        self.pose = pose
        self.t, self.odom = t, odom_pose
        if scan:
            self.scan = scan
        return MotionState(t, self.pose, v, w, trusted, agree, span)
