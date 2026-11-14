"""The power cable the base drags: where it lies on the floor, worked out from the base's own motion.

The cable runs from an anchor along the floor to a post on the robot (ATTACH, base frame). No sensor
sees it behind the robot, so the floor path of the attach point (the trail) is taken as the cable.
The newest TRAIL_ATTACHED of it swings with the robot and is not an obstacle; everything older is.
"""
import json
import math

import numpy

import footprint
import geometry
import safety

ATTACH = (-0.08, 0.20)  # m, base frame: the post the cable is taped to
STEP = 0.05  # m, trail and output spacing, and the camera points' merge distance
TRAIL_ATTACHED = 0.4  # m of the newest trail that moves with the robot
ASSUMED_LAY = 1.0  # m, unknown lay at the start: straight behind the attach point
SLACK_MARGIN = 1.0  # m of cable always left unused


def _length(poly):
    return float(numpy.hypot(*numpy.diff(poly, axis=0).T).sum()) if len(poly) > 1 else 0.0


class Tether:
    def __init__(self, p, start_pose, length=10.0, lay=None):
        """`lay`: known cable floor points (odom), anchor side first, ending at the attach point."""
        self.p, self.length = p, length
        self.reset(start_pose, lay)

    def reset(self, pose, lay=None):
        """The cable is laid out again behind the robot at `pose`: no trail, no turn, no camera points."""
        a = geometry.base_to_odom(ATTACH, pose)
        behind = (a[0] - ASSUMED_LAY * math.cos(pose[2]), a[1] - ASSUMED_LAY * math.sin(pose[2]))
        self.trail = [tuple(q) for q in lay] if lay is not None else [behind, a]
        self.seen, self.pose, self.net_turn = [], tuple(pose), 0.0

    def update(self, pose):
        """Each cycle, with the odometry pose (x, y, th)."""
        dth = math.remainder(pose[2] - self.pose[2], math.tau)
        moved = math.hypot(pose[0] - self.pose[0], pose[1] - self.pose[1])
        self.net_turn += dth
        self.pose = tuple(pose)
        a = geometry.base_to_odom(ATTACH, pose)
        if abs(dth) * math.hypot(*ATTACH) > moved:  # turning on the spot: the near cable swings back with it
            along = 0.0
            for i in range(len(self.trail) - 2, -1, -1):
                along += math.dist(self.trail[i], self.trail[i + 1])
                if along > TRAIL_ATTACHED:
                    break
                if math.dist(self.trail[i], a) <= STEP:
                    del self.trail[i + 1:]
                    break
        if math.dist(self.trail[-1], a) > STEP:
            self.trail.append(a)

    def _cable(self, pose):
        """The trail polyline (odom) ending at the attach point at `pose`."""
        poly = numpy.array(self.trail + [geometry.base_to_odom(ATTACH, pose)])
        keep = numpy.r_[True, numpy.hypot(*numpy.diff(poly, axis=0).T) > 0]
        return poly[keep]

    def points(self, pose):
        """(N, 2) cable floor points in the base frame at `pose`: the trail older than the attached
        part, every STEP, plus the camera's points."""
        poly = self._cable(pose)
        along = numpy.r_[0.0, numpy.cumsum(numpy.hypot(*numpy.diff(poly, axis=0).T))]
        at = along[-1] - numpy.arange(TRAIL_ATTACHED, along[-1] + 1e-9, STEP)
        odom = numpy.stack([numpy.interp(at, along, poly[:, 0]), numpy.interp(at, along, poly[:, 1])], axis=1)
        odom = numpy.vstack([odom, numpy.array(self.seen).reshape(-1, 2)])
        return numpy.array([geometry.odom_to_base(q, pose) for q in odom]).reshape(-1, 2)

    def add_seen(self, points_odom):
        """Cable points the camera saw (odom); kept, one per STEP."""
        for q in points_odom:
            if all(math.dist(q, s) > STEP for s in self.seen):
                self.seen.append((float(q[0]), float(q[1])))

    def turn_allowed(self, dw_total):
        """False when turning dw_total more (rad, left positive) would wind the cable past half a turn."""
        return abs(self.net_turn + dw_total) <= math.pi

    def used_length(self):
        """Cable on the floor so far (m): the assumed lay plus the attach point's path."""
        return _length(self._cable(self.pose))

    def slack_ok(self, extra):
        return self.used_length() + extra <= self.length - SLACK_MARGIN

    def clear(self, pose, v, w, p, seconds=1.0, margin=0.20):
        """True when (v, w) for `seconds`, then the measured stop, keeps the outline `margin` from the cable.

        Backing up (v < 0) is the forward motion (-v, -w) mirrored front to back.
        """
        poses = safety.run_on_poses(abs(v), w if v >= 0 else -w, p, cycle=seconds)
        if v < 0:
            poses = [(-x, y, -th) for x, y, th in poses]
        return footprint.sweep_clear(self.points(pose), poses, margin)

    def save(self, path):
        with open(path, "w") as f:
            json.dump({"length": self.length, "trail": self.trail, "seen": self.seen,
                       "pose": self.pose, "net_turn": self.net_turn}, f)

    @classmethod
    def load(cls, path, p):
        with open(path) as f:
            d = json.load(f)
        t = cls(p, d["pose"], d["length"], lay=d["trail"])
        t.seen, t.net_turn = [tuple(q) for q in d["seen"]], d["net_turn"]
        return t
