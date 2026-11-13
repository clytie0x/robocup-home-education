"""The base's measured motion: wheel odometry, corrected by LiDAR scan matching (Task M1).

Odometry steps by 0.01 m and turns with the measured scales; the LiDAR sees the room. Every new
revolution is matched to the one before, with the odometry step as the starting guess. A match
counts only when enough points agree and it stays near the odometry; otherwise the odometry step
is kept. The pose is built from those steps in the odometry frame, and v, w are the last step
over its time.
"""
import math
from dataclasses import dataclass

import scanmatch

AGREE_MIN = 0.6  # share of points that must agree (spec: below 60 % the estimate is not trusted)
MAX_POS = 0.05  # m; a match further than this from the odometry step is refused
MAX_YAW = math.radians(5)
# Returns per scan given to the matcher. On the Pi (235 pairs of saved scans 1 s apart, no guess):
# 150 points took 26.9 ms median, 91 ms worst (over the 80 ms budget), 84 % trusted; 120 points
# 14.4 ms median, 63 ms worst, 71 % trusted. Scans 0.1 s apart with the odometry guess agree more.
MAX_POINTS = 120


@dataclass
class MotionState:
    t: float
    pose: tuple  # (x, y, yaw), odometry frame
    v: float  # m/s, forward over the last step
    w: float  # rad/s over the last step
    trusted: bool  # this step came from a scan match that agreed
    agree: float  # the match's share of agreeing points (0 when none was made)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _thin(scan):
    """At most MAX_POINTS returns, keeping their order (the matcher uses neighbours)."""
    step = max(1, math.ceil(len(scan) / MAX_POINTS))
    return scan[::step]


class MotionEstimator:
    def __init__(self, p, match=scanmatch.match):
        self.p, self.match = p, match
        self.t = self.odom = self.scan = self.pose = None
        self.refused = 0  # matches refused for being too far from the odometry

    def update(self, t, odom_pose, scan, command=None):
        """The motion state at time t from the odometry pose and the newest scan.

        command: the (v, w) sent, unused for now (the online estimates, Task M2, will use it).
        """
        if self.t is not None and t < self.t:
            raise ValueError("time went backward: %.3f after %.3f" % (t, self.t))
        if self.t is None:
            self.t, self.odom, self.scan, self.pose = t, odom_pose, scan, tuple(odom_pose)
            return MotionState(t, self.pose, 0.0, 0.0, False, 0.0)
        # the odometry step, in the frame of the robot at the previous update
        (x0, y0, th0), (x1, y1, th1) = self.odom, odom_pose
        c, s = math.cos(th0), math.sin(th0)
        dx, dy = x1 - x0, y1 - y0
        step = (c * dx + s * dy, -s * dx + c * dy, _wrap(th1 - th0))
        trusted, agree = False, 0.0
        if scan and self.scan and scan is not self.scan and scan != self.scan:
            try:
                mx, my, mth, agree = self.match(_thin(self.scan), _thin(scan), self.p, step)
            except ValueError:
                pass  # too few points, or no overlap: keep the odometry
            else:
                if agree >= AGREE_MIN:
                    if math.hypot(mx - step[0], my - step[1]) <= MAX_POS and abs(_wrap(mth - step[2])) <= MAX_YAW:
                        step, trusted = (mx, my, mth), True
                    else:
                        self.refused += 1
        x, y, th = self.pose
        c, s = math.cos(th), math.sin(th)
        self.pose = (x + c * step[0] - s * step[1], y + s * step[0] + c * step[1], th + step[2])
        dt = t - self.t
        v = math.copysign(math.hypot(step[0], step[1]), step[0]) / dt if dt > 0 else 0.0
        w = step[2] / dt if dt > 0 else 0.0
        self.t, self.odom = t, odom_pose
        if scan:
            self.scan = scan
        return MotionState(t, self.pose, v, w, trusted, agree)
