"""SO-100 arm kinematics in the robot base frame, for picking a bag from the floor.

No ROS. Numbers come from the arm team's branch (read with `git show origin/arm-system:<path>`):
  URDF   arm/so100_moveit_config/urdf/so100.urdf (joint origins, axes, limits, velocities; lines below)
  meshes arm/so100_moveit_config/meshes/*.stl  (per-link bounding boxes, computed from every vertex)
  calib  arm/calibration.json + arm/so100_driver/so100_driver/calib.py (calibrated joint ranges)
  jaws   arm/so100_driver/so100_driver/ik.py (FIXED_INNER_X, TIP_Y, GRASP_Y, jaw_gap)

Frames
  arm frame   the URDF `base` link: origin at the bottom of the arm's base plate, z up, the pan axis at
              (0, -0.0452), the arm's "forward" is -y and its "left" is +x (arm/API.md).
  robot frame the wheeled base's frame (x forward, y left, z up from the floor).
  robot = pan_xy + Rz(yaw) * (forward, left), forward = -(y_arm + 0.0452), left = x_arm,
  z_robot = z_arm + base_z + z_offset.
  pan_xy = (0.012, 0.0145) from the LiDAR, base_z = 0.305 (tape: 30 to 31 cm), z_offset = +0.026
  (the real arm sits 1.9 to 3.4 cm higher than its model): see base/docs/floor-pick.md.

A target is a point between the open jaws (GRASP_Y along the tool axis, at the middle of the jaw
opening) plus the tool pitch (angle of the tool axis below the horizontal: -90 deg = straight down).
"""
import json
import math
import os

import numpy as np

ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
JOINTS = ARM + ("gripper",)

# (joint, parent, child, origin xyz, origin rpy, axis, lower, upper, velocity rad/s)
URDF_JOINTS = (
    ("shoulder_pan", "base", "shoulder", (0, -0.0452, 0.0165), (1.57079, 0, 0), (0, 1, 0), -2.0, 2.0, 1.0),  # so100.urdf:191-196
    ("shoulder_lift", "shoulder", "upper_arm", (0, 0.1025, 0.0306), (-1.8, 0, 0), (1, 0, 0), 0.0, 3.5, 1.0),  # :200-205
    ("elbow_flex", "upper_arm", "lower_arm", (0, 0.11257, 0.028), (1.57079, 0, 0), (1, 0, 0), -3.14158, 0.0, 1.0),  # :209-214
    ("wrist_flex", "lower_arm", "wrist", (0, 0.0052, 0.1349), (-1, 0, 0), (1, 0, 0), -2.5, 1.2, 1.0),  # :218-223
    ("wrist_roll", "wrist", "gripper", (0, -0.0601, 0), (0, 1.57079, 0), (0, 1, 0), -3.14158, 3.14158, 1.0),  # :227-232
    ("gripper", "gripper", "jaw", (-0.0202, -0.0244, 0), (0, 3.14158, 0), (0, 0, 1), -0.2, 2.0, 1.0),  # :236-241
)
TCP = (0, -0.09, 0)  # so100.urdf:246-250, fixed joint gripper -> tcp

# Bounding boxes of each link's visual meshes in the link frame (min xyz, max xyz), from all vertices of
# Base(+_Motor), Rotation_Pitch(+_Motor), Upper_Arm(+_Motor), Lower_Arm(+_Motor), Wrist_Pitch_Roll(+_Motor),
# Fixed_Jaw(+_Motor) and Moving_Jaw .stl (so100.urdf visuals have no origin offset).
LINK_BOXES = {
    "upper_arm": ((-0.0351, -0.0119, -0.012), (0.0349, 0.1302, 0.0382)),
    "lower_arm": ((-0.0351, -0.015, -0.012), (0.0261, 0.0219, 0.1451)),
    "wrist": ((-0.0352, -0.0626, -0.016), (0.0281, 0.012, 0.016)),
    "gripper": ((-0.0304, -0.1064, -0.024), (0.0352, 0.0, 0.024)),
    "jaw": ((-0.0123, -0.082, -0.0239), (0.01, 0.01, 0.0241)),
}
# The AX1080P camera under the jaws (gripper frame): not measured yet. A box around the gripper body,
# both z sides, so the clearance checks keep room for it wherever it sits.
CAMERA_BOX = ((-0.035, -0.075, -0.055), (0.04, -0.02, 0.055))

# Jaws (arm team's ik.py, measured from Fixed_Jaw.stl / Moving_Jaw.stl): tool axis = gripper -y,
# the moving jaw opens towards -x, fixed finger inner face at x = +0.008, finger tips at y = -0.106.
FIXED_INNER_X = 0.008
TIP_Y = -0.106
GRASP_Y = -0.092           # where the held part sits along the tool axis (1.4 cm inside the tips)
JAW_OPEN = 0.9             # rad: their JAW_G_MAX, widest useful opening
CLOSED = 0.05              # rad: below this after closing = nothing in the jaws (pick_place_node.py:745)


def jaw_gap(g):
    """Gap between the finger faces (m) at gripper angle g (rad), valid 0..1 (ik.py)."""
    return 0.016 + 0.066 * g


# arm/calibration.json (raw servo counts) and calib.py: angle = sign*(raw-zero)*2pi/4096,
# usable range = [min+20, max-20] counts (gripper: [min, max-20]).
CALIBRATION = {
    "shoulder_pan": {"sign": -1, "zero": 2089, "min": 698, "max": 3485},
    "shoulder_lift": {"sign": 1, "zero": 905, "min": 901, "max": 3203},
    "elbow_flex": {"sign": 1, "zero": 3124, "min": 926, "max": 3123},
    "wrist_flex": {"sign": 1, "zero": 2486, "min": 823, "max": 2873},
    "wrist_roll": {"sign": 1, "zero": 986, "min": 0, "max": 4095},
    "gripper": {"sign": 1, "zero": 907, "min": 908, "max": 2354},
}
ROBOT_CALIBRATION = os.path.expanduser("~/so100_ws/calibration.json")   # the robot's own copy, if present

# Robot body: the base outline (hmc/src/hmc/world.py FOOTPRINT, a 0.40 m square) up to the top deck,
# on which the arm's base plate stands (arm base 0.30 to 0.31 m above the floor).
BODY_HALF = 0.20
DECK_TOP = 0.30
DECK_MARGIN = 0.01
FLOOR_CLEAR = 0.02
LIDAR_Z = 0.245            # LiDAR scan plane above the floor (0.24 to 0.25 m)


class Unreachable(Exception):
    """The target cannot be reached, or only by breaking a limit or hitting the floor or the deck."""


def calibrated_limits(cal=None):
    cal = cal or CALIBRATION
    out = {}
    for name, _p, _c, _x, _r, _a, lower, upper, _v in URDF_JOINTS:
        j = cal.get(name)
        if not j or j.get("min") is None:
            out[name] = (lower, upper)
            continue
        lo_raw = j["min"] if name == "gripper" else j["min"] + 20
        hi_raw = j["max"] - 20
        a, b = sorted(j["sign"] * (r - j["zero"]) * 2 * math.pi / 4096 for r in (lo_raw, hi_raw))
        out[name] = (max(a, lower), min(b, upper))
    return out


def load_limits(path=ROBOT_CALIBRATION):
    """Calibrated limits from the robot's calibration file when it exists, else the copy above."""
    try:
        with open(path) as f:
            return calibrated_limits(json.load(f))
    except (OSError, ValueError, KeyError):
        return calibrated_limits()


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _tf(xyz, rot):
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = xyz
    return m


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _box_corners(lo, hi):
    return np.array([[x, y, z, 1.0] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


_ORIGINS = {j[0]: _tf(j[3], _rpy(*j[4])) for j in URDF_JOINTS}
_CHAIN = [(j[0], j[1], j[2], _ORIGINS[j[0]], list(j[5]).index(1)) for j in URDF_JOINTS]   # every axis is +x, +y or +z


def _rot4(axis, th):
    """4x4 rotation by th about the link's x (0), y (1) or z (2) axis."""
    c, s = math.cos(th), math.sin(th)
    m = np.eye(4)
    i, j = [(1, 2), (2, 0), (0, 1)][axis]
    m[i, i], m[i, j], m[j, i], m[j, j] = c, -s, s, c
    return m
_TCP = _tf(TCP, np.eye(3))
PAN_POINT = np.array([0.0, -0.0452])   # arm frame (API.md, so100.urdf:194)


class Mount:
    """Where the arm stands on the robot: arm frame <-> robot base frame."""

    def __init__(self, pan_xy=(0.012, 0.0145), base_z=0.305, z_offset=0.026, yaw=0.0):
        self.pan_xy = np.asarray(pan_xy, float)
        self.base_z, self.z_offset, self.yaw = float(base_z), float(z_offset), float(yaw)

    def to_robot(self, p):
        p = np.asarray(p, float)
        f, left = -(p[1] - PAN_POINT[1]), p[0] - PAN_POINT[0]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.array([self.pan_xy[0] + c * f - s * left, self.pan_xy[1] + s * f + c * left,
                         p[2] + self.base_z + self.z_offset])

    def to_arm(self, p):
        p = np.asarray(p, float)
        d = p[:2] - self.pan_xy
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        f, left = c * d[0] + s * d[1], -s * d[0] + c * d[1]
        return np.array([PAN_POINT[0] + left, PAN_POINT[1] - f, p[2] - self.base_z - self.z_offset])

    def to_dict(self):
        return {"pan_xy": self.pan_xy.tolist(), "base_z": self.base_z, "z_offset": self.z_offset, "yaw": self.yaw}


class Kinematics:
    """FK/IK of the SO-100 in the robot base frame with limit, floor and deck checks."""

    def __init__(self, mount=None, limits=None, jaw_open=JAW_OPEN):
        self.mount = mount or Mount()
        self.limits = dict(limits or calibrated_limits())
        self.velocity = {j[0]: j[8] for j in URDF_JOINTS}
        self.jaw_open = jaw_open
        # the part to grasp sits mid-way between the open jaws
        self.grasp_local = np.array([FIXED_INNER_X - jaw_gap(jaw_open) / 2, GRASP_Y, 0.0, 1.0])
        self._measure()

    # ----------------------------------------------------------------------------- forward
    def fk(self, q):
        """4x4 pose of every link in the ARM frame."""
        w = {"base": np.eye(4)}
        for name, parent, child, origin, axis in _CHAIN:
            w[child] = w[parent] @ origin @ _rot4(axis, q.get(name, 0.0))
        w["tcp"] = w["gripper"] @ _TCP
        return w

    def point(self, q, local, link="gripper"):
        """Robot-frame position of a point given in a link frame."""
        return self.mount.to_robot((self.fk(q)[link] @ np.append(np.asarray(local, float)[:3], 1.0))[:3])

    def grasp_point(self, q):
        return self.mount.to_robot((self.fk(q)["gripper"] @ self.grasp_local)[:3])

    def tool_dir(self, q, frames=None):
        """Unit tool axis (gripper -y) in the robot frame."""
        t = -(frames or self.fk(q))["gripper"][:3, 1]
        c, s = math.cos(self.mount.yaw), math.sin(self.mount.yaw)
        # arm (x, y) -> robot: forward = -y, left = x
        f, left = -t[1], t[0]
        return np.array([c * f - s * left, s * f + c * left, t[2]])

    def heading(self, q):
        """Direction the arm points in the arm frame (0 = -y), as an angle in the arm frame."""
        return self.heading0 + q.get("shoulder_pan", 0.0)

    def pitch(self, q, frames=None):
        t = -(frames or self.fk(q))["gripper"][:3, 1]
        h = self.heading(q)
        return math.atan2(t[2], t[0] * math.cos(h) + t[1] * math.sin(h))

    # ------------------------------------------------------------------ planar model (closed form)
    def _rz(self, q, link, w=None):
        p = (w or self.fk(q))[link][:3, 3]
        h = self.heading(q)
        return np.array([float(np.dot(p[:2] - PAN_POINT, (math.cos(h), math.sin(h)))), p[2]])

    def _angles(self, q):
        w = self.fk(q)
        s, e, wr = self._rz(q, "upper_arm", w), self._rz(q, "lower_arm", w), self._rz(q, "wrist", w)
        return np.array([math.atan2(*(e - s)[::-1]), math.atan2(*(wr - e)[::-1]), self.pitch(q, w)])

    def _measure(self):
        z = {n: 0.0 for n in ARM}
        self.heading0 = 0.0
        ref = {**z, "shoulder_lift": 1.2, "elbow_flex": -1.2}
        p = self.fk(ref)["wrist"][:3, 3][:2] - PAN_POINT
        self.heading0 = math.atan2(p[1], p[0])           # -pi/2: the arm points along -y at pan 0
        self.s = self._rz(z, "upper_arm")
        self.l1 = float(np.linalg.norm(self._rz(z, "lower_arm") - self.s))
        self.l2 = float(np.linalg.norm(self._rz(z, "wrist") - self._rz(z, "lower_arm")))
        self.reach_tool = float(np.linalg.norm(self.fk(z)["gripper"][:3, 3] - self.fk(z)["wrist"][:3, 3]))
        base = self._angles(z)
        d = 0.2
        self.a = np.round([(self._angles({**z, n: s * d}) - base) / (s * d)
                           for n, s in (("shoulder_lift", 1), ("elbow_flex", -1), ("wrist_flex", -1))])
        self.c = base

    def _closed_form(self, p_arm, pitch, branch):
        v = p_arm[:2] - PAN_POINT
        r = float(np.hypot(*v))
        pan = _wrap(math.atan2(v[1], v[0]) - self.heading0)
        depth = self.reach_tool - GRASP_Y
        wr = np.array([r, p_arm[2]]) - depth * np.array([math.cos(pitch), math.sin(pitch)])
        dv = wr - self.s
        n = float(np.hypot(*dv))
        cg = (n * n - self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * self.l2)
        if abs(cg) > 1:
            raise Unreachable("out of reach: the wrist would be %.3f m from the shoulder, the arm spans %.3f to %.3f m"
                              % (n, abs(self.l1 - self.l2), self.l1 + self.l2))
        g = branch * math.acos(cg)
        b1 = math.atan2(dv[1], dv[0]) - math.atan2(self.l2 * math.sin(g), self.l1 + self.l2 * math.cos(g))
        b2 = b1 + g
        a = self.a
        lift = _wrap(b1 - self.c[0]) / a[0][0]
        elbow = _wrap(b2 - self.c[1] - a[0][1] * lift) / a[1][1]
        wflex = _wrap(pitch - self.c[2] - a[0][2] * lift - a[1][2] * elbow) / a[2][2]
        return {"shoulder_pan": pan, "shoulder_lift": lift, "elbow_flex": elbow, "wrist_flex": wflex}

    # ----------------------------------------------------------------------------- inverse
    def ik(self, target, pitch, roll=0.0, check=True, jaw=None):
        """Joint angles (dict of the 5 arm joints) that put the grasp point at `target` (robot frame, m)
        with the tool at `pitch` (rad). Raises Unreachable with the reason; never clamps."""
        p_arm = self.mount.to_arm(target)
        errors = []
        for branch in (1, -1):
            try:
                q = self._closed_form(p_arm, pitch, branch)
                q["wrist_roll"] = roll
                q = self._refine(q, p_arm, pitch)
                q = self._into_limits(q)
                if check:
                    self.check_pose(q, jaw=self.jaw_open if jaw is None else jaw)
                return q
            except Unreachable as exc:
                errors.append(exc)
        raise max(errors, key=lambda e: ("floor" in str(e) or "deck" in str(e), "limit" in str(e)))

    def _residual(self, q, p_arm, pitch):
        w = self.fk(q)
        return np.append((w["gripper"] @ self.grasp_local)[:3] - p_arm, _wrap(self.pitch(q, w) - pitch))

    def _refine(self, q, p_arm, pitch):
        names = ARM[:4]
        q = dict(q)
        for _ in range(30):
            r = self._residual(q, p_arm, pitch)
            if np.linalg.norm(r[:3]) < 1e-7 and abs(r[3]) < 1e-7:
                return q
            jac = np.zeros((4, 4))
            for i, n in enumerate(names):
                dq = dict(q)
                dq[n] += 1e-6
                jac[:, i] = (self._residual(dq, p_arm, pitch) - r) / 1e-6
            step = np.linalg.lstsq(jac, -r, rcond=None)[0]
            step *= min(1.0, 0.3 / max(1e-9, float(np.abs(step).max())))
            for i, n in enumerate(names):
                q[n] += float(step[i])
        r = self._residual(q, p_arm, pitch)
        if np.linalg.norm(r[:3]) > 1e-4 or abs(r[3]) > 1e-3:
            raise Unreachable("out of reach: no exact solution (%.1f mm off)" % (np.linalg.norm(r[:3]) * 1000))
        return q

    def _into_limits(self, q):
        out = dict(q)
        for n, v in q.items():
            lo, hi = self.limits[n]
            for k in (0, 1, -1):
                if lo - 1e-9 <= v + 2 * math.pi * k <= hi + 1e-9:
                    out[n] = v + 2 * math.pi * k
                    break
        self.check_limits(out)
        return out

    def check_limits(self, q):
        for n, v in q.items():
            if n not in self.limits:
                continue
            lo, hi = self.limits[n]
            if not lo - 1e-9 <= v <= hi + 1e-9:
                raise Unreachable("joint limit: %s %.3f rad outside %.3f..%.3f" % (n, v, lo, hi))

    def ik_any_pitch(self, target, pitches=None, roll=0.0, jaw=None):
        """The steepest pitch (from straight down towards 20 deg down) that works: (q, pitch)."""
        errors = []
        for deg in pitches or range(-90, -19, 5):
            pitch = math.radians(deg)
            try:
                return self.ik(target, pitch, roll, jaw=jaw), pitch
            except Unreachable as exc:
                errors.append(exc)
        raise max(errors, key=lambda e: ("floor" in str(e) or "deck" in str(e), "limit" in str(e)))

    # ---------------------------------------------------------------------------- clearance
    def body_points(self, q, jaw=None, camera=True):
        """Corners of the link bounding boxes (and the camera box) in the robot frame, MODEL height
        (the z offset is left out when it is positive, so the checks hold even if it is wrong)."""
        qq = dict(q)
        qq["gripper"] = self.jaw_open if jaw is None else jaw
        w = self.fk(qq)
        pts = []
        for link, (lo, hi) in LINK_BOXES.items():
            pts.append((_box_corners(lo, hi) @ w[link].T)[:, :3])
        if camera:
            pts.append((_box_corners(*CAMERA_BOX) @ w["gripper"].T)[:, :3])
        pts = np.vstack(pts)
        out = np.array([self.mount.to_robot(p) for p in pts])
        out[:, 2] -= max(self.mount.z_offset, 0.0)
        return out

    def check_pose(self, q, jaw=None):
        pts = self.body_points(q, jaw)
        low = float(pts[:, 2].min())
        if low < FLOOR_CLEAR:
            raise Unreachable("floor: a part of the arm would be %.3f m above the floor (min %.2f)" % (low, FLOOR_CLEAR))
        inside = ((np.abs(pts[:, 0]) < BODY_HALF + DECK_MARGIN) & (np.abs(pts[:, 1]) < BODY_HALF + DECK_MARGIN)
                  & (pts[:, 2] < DECK_TOP + DECK_MARGIN))
        if inside.any():
            p = pts[inside][np.argmin(pts[inside][:, 2])]
            raise Unreachable("deck: a part of the arm would go into the robot's top at (%.3f, %.3f, %.3f)" % tuple(p))

    def pose_ok(self, q, jaw=None):
        try:
            self.check_limits(q)
            self.check_pose(q, jaw)
            return True
        except Unreachable:
            return False
