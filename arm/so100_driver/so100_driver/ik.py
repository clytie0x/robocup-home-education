"""Closed-form inverse kinematics for the SO-100 arm and a simple pick-and-place grasp planner.

No ROS imports: tested on Windows and on the Jetson.

Geometry: shoulder_pan turns about a vertical axis; shoulder_lift, elbow_flex and wrist_flex are three
parallel axes, so after choosing the pan angle the arm is a planar 3-link chain.  Given a point on the
tool axis and the tool pitch, the wrist centre follows, and lift/elbow come from the 2-link triangle.
wrist_roll then turns the jaws about the tool axis.  All constants are measured from the URDF at start-up
(finite differences), so nothing here depends on the URDF's odd frame conventions.

Gripper geometry (gripper link frame, measured from Fixed_Jaw.stl / Moving_Jaw.stl):
  tool axis = -y, jaws close along x, fixed finger inner face at x=+0.008, finger tips at y=-0.106,
  gap between the finger faces ~= 0.016 + 0.066 * gripper_angle (valid 0..1 rad).
"""
import math
import os
import struct
import xml.etree.ElementTree as ET

import numpy as np

ARM = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')

FIXED_INNER_X = 0.008     # fixed finger inner face (gripper frame x)
TIP_Y = -0.106            # finger tips (gripper frame y)
GRASP_Y = -0.092          # where the object centre sits along the tool axis (~1.4 cm inside the tips)
JAW_G_MAX = 0.9           # widest useful jaw angle (tips start to pull back beyond this)
MAX_OBJECT = 0.065        # widest object (m, across the jaws) the planner accepts


# The SO-100 has ONE moving jaw. Closing pushes the object towards the fixed finger, so the object is
# placed close to the fixed finger (small margin) and the moving jaw gets the big margin. Centring it
# between the open jaws (first version) shoved tall objects 1+ cm sideways and knocked them over.
FIXED_MARGIN = 0.006      # gap between the object and the fixed finger before closing
MOVING_MARGIN = 0.018     # gap on the moving-jaw side
GRIP_DEPTH = 0.028        # how far the fingers reach down over a tall object (finger length ~3 cm)


def object_offset(gap):
    """Gripper-frame x of the object centre for a jaw opening `gap` (object = gap - both margins)."""
    extent = max(gap - FIXED_MARGIN - MOVING_MARGIN, 0.0)
    return FIXED_INNER_X - FIXED_MARGIN - extent / 2


def opening_for(extent):
    return min(extent + FIXED_MARGIN + MOVING_MARGIN, jaw_gap(JAW_G_MAX))


def jaw_gap(g):
    return 0.016 + 0.066 * g


def jaw_for_gap(gap):
    return (gap - 0.016) / 0.066


# ----------------------------------------------------------------------------------- forward kin.
def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(ax, th):
    ax = np.asarray(ax, float) / np.linalg.norm(ax)
    k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + math.sin(th) * k + (1 - math.cos(th)) * k @ k


def _tf(xyz, rot):
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = xyz
    return m


def _wrap_into(a, lo, hi):
    """Shift `a` by multiples of 2*pi into [lo, hi]; None if impossible."""
    for k in (0, 1, -1, 2, -2):
        b = a + 2 * math.pi * k
        if lo - 1e-9 <= b <= hi + 1e-9:
            return b
    return None


def read_stl(path, max_points=400):
    """Vertices of a binary STL (subsampled), as an (n, 3) array."""
    with open(path, 'rb') as f:
        data = f.read()
    n = struct.unpack('<I', data[80:84])[0]
    tri = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', 9), ('a', '<u2')]))
    v = np.unique(tri['v'].reshape(-1, 3).astype(float).round(4), axis=0)
    if len(v) > max_points:   # keep the extremes along z plus an even sample
        idx = np.unique(np.r_[np.linspace(0, len(v) - 1, max_points).astype(int), np.argsort(v[:, 2])[:40]])
        v = v[idx]
    return v


class Kinematics:
    def __init__(self, urdf_xml, limits=None, mesh_dir=None):
        root = ET.fromstring(urdf_xml)
        self.meshes = {}          # link -> (n, 3) vertices in the link frame (for the table check)
        if mesh_dir:
            for link in root.findall('link'):
                pts = [read_stl(os.path.join(mesh_dir, os.path.basename(m.get('filename'))))
                       for m in link.findall('visual/geometry/mesh')]
                if pts:
                    self.meshes[link.get('name')] = np.vstack(pts)
        self.joints = []          # (name, parent, child, origin 4x4, axis or None)
        self.urdf_limits = {}
        for j in root.findall('joint'):
            o = j.find('origin')
            xyz = [float(v) for v in o.get('xyz', '0 0 0').split()] if o is not None else [0, 0, 0]
            rpy = [float(v) for v in o.get('rpy', '0 0 0').split()] if o is not None else [0, 0, 0]
            axis = None
            if j.get('type') != 'fixed':
                axis = [float(v) for v in j.find('axis').get('xyz').split()]
                lim = j.find('limit')
                self.urdf_limits[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))
            self.joints.append((j.get('name'), j.find('parent').get('link'), j.find('child').get('link'),
                                _tf(xyz, _rpy(*rpy)), axis))
        self.limits = dict(self.urdf_limits)
        if limits:
            self.limits.update(limits)
        self._measure()

    def frames(self, q):
        world = {'base': np.eye(4)}
        todo = list(self.joints)
        while todo:
            rest = []
            for name, parent, child, origin, axis in todo:
                if parent not in world:
                    rest.append((name, parent, child, origin, axis))
                    continue
                m = world[parent] @ origin
                if axis is not None:
                    m = m @ _tf([0, 0, 0], _axis_angle(axis, q.get(name, 0.0)))
                world[child] = m
            if len(rest) == len(todo):
                break
            todo = rest
        return world

    def lowest(self, q, links):
        """Lowest z (base frame) of the given links' meshes."""
        w = self.frames(q)
        low = math.inf
        for link in links:
            v = self.meshes.get(link)
            if v is not None:
                low = min(low, float((v @ w[link][:3, :3].T + w[link][:3, 3])[:, 2].min()))
        return low

    def gripper_point(self, q, local):
        """World position of a point given in the gripper link frame."""
        return (self.frames(q)['gripper'] @ np.array([*local, 1.0]))[:3]

    # ------------------------------------------------------------------- planar model from the URDF
    def _measure(self):
        z = {n: 0.0 for n in ARM}
        w = self.frames(z)
        self.pan_point = w['shoulder'][:3, 3][:2].copy()
        self.pan_axis = w['shoulder'][:3, :3] @ np.array(self.joints_axis('shoulder_pan'))
        if abs(abs(self.pan_axis[2]) - 1) > 1e-3:
            raise ValueError('shoulder_pan axis is not vertical')
        ref = {**z, 'shoulder_lift': 1.2, 'elbow_flex': -1.2}
        self.heading0 = self._heading(ref)
        self.pan_sign = 1.0 if self._ang(self._heading({**ref, 'shoulder_pan': 0.3}) - self.heading0) > 0 else -1.0
        self.l1 = np.linalg.norm(self._rz(z, 'lower_arm') - self._rz(z, 'upper_arm'))
        self.l2 = np.linalg.norm(self._rz(z, 'wrist') - self._rz(z, 'lower_arm'))
        self.wrist_to_gripper = np.linalg.norm(w['gripper'][:3, 3] - w['wrist'][:3, 3])
        self.s = self._rz(z, 'upper_arm')
        # angle model: b1 = c1 + a1*lift, b2 = c2 + a2*lift + a3*elbow, phi = c3 + a4*lift + a5*elbow + a6*wflex
        base = self._angles(z)
        d = 0.2
        dl = (self._angles({**z, 'shoulder_lift': d}) - base) / d
        de = (self._angles({**z, 'elbow_flex': -d}) - base) / -d
        dw = (self._angles({**z, 'wrist_flex': -d}) - base) / -d
        self.c = base
        self.a = np.array([dl, de, dw]).round()      # each entry is -1, 0 or +1
        if not (np.allclose(self.a, np.array([dl, de, dw]), atol=1e-6)):
            raise ValueError('lift/elbow/wrist axes are not parallel')

    def joints_axis(self, name):
        return next(a for n, _, _, _, a in self.joints if n == name)

    @staticmethod
    def _ang(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    def _heading(self, q):
        p = self.frames(q)['wrist'][:3, 3][:2] - self.pan_point
        return math.atan2(p[1], p[0])

    def _rz(self, q, link):
        """(r, z) of a link origin in the arm plane (r measured along the arm heading)."""
        p = self.frames(q)[link][:3, 3]
        h = self.heading0 + self.pan_sign * q.get('shoulder_pan', 0.0)
        u = np.array([math.cos(h), math.sin(h)])
        return np.array([float(np.dot(p[:2] - self.pan_point, u)), p[2]])

    def _angles(self, q):
        w = self.frames(q)
        s, e, wr = self._rz(q, 'upper_arm'), self._rz(q, 'lower_arm'), self._rz(q, 'wrist')
        h = self.heading0 + self.pan_sign * q.get('shoulder_pan', 0.0)
        tool = -w['wrist'][:3, 1]
        t = np.array([tool[0] * math.cos(h) + tool[1] * math.sin(h), tool[2]])
        return np.array([math.atan2(*(e - s)[::-1]), math.atan2(*(wr - e)[::-1]), math.atan2(t[1], t[0])])

    # ------------------------------------------------------------------------------------------ IK
    def heading(self, q):
        return self.heading0 + self.pan_sign * q['shoulder_pan']

    def tool_dir(self, heading, pitch):
        return np.array([math.cos(pitch) * math.cos(heading), math.cos(pitch) * math.sin(heading), math.sin(pitch)])

    def solve(self, point, pitch, depth=-GRASP_Y, branch=None):
        """Joint angles (no roll) that put the tool-axis point `depth` m beyond the gripper origin at
        `point` (base frame) with the tool pointing at `pitch` (rad, -pi/2 = straight down).
        Returns a list of solutions (dicts), best first."""
        p = np.asarray(point, float)
        v = p[:2] - self.pan_point
        r = float(np.hypot(*v))
        if r < 1e-6:
            return []
        pan = _wrap_into(self._ang((math.atan2(v[1], v[0]) - self.heading0) * self.pan_sign), *self.limits['shoulder_pan'])
        if pan is None:
            return []
        reach = self.wrist_to_gripper + depth
        wr = np.array([r, p[2]]) - reach * np.array([math.cos(pitch), math.sin(pitch)])
        dv = wr - self.s
        n = float(np.hypot(*dv))
        cg = (n * n - self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * self.l2)
        if abs(cg) > 1:
            return []
        out = []
        for sgn in ((branch,) if branch else (1, -1)):
            g = sgn * math.acos(cg)
            b1 = math.atan2(dv[1], dv[0]) - math.atan2(self.l2 * math.sin(g), self.l1 + self.l2 * math.cos(g))
            b2 = b1 + g
            lift = self._ang(b1 - self.c[0]) / self.a[0][0]
            elbow = self._ang(b2 - self.c[1] - self.a[0][1] * lift) / self.a[1][1]
            wflex = self._ang(pitch - self.c[2] - self.a[0][2] * lift - self.a[1][2] * elbow) / self.a[2][2]
            q = {'shoulder_pan': pan}
            ok = True
            for name, val in (('shoulder_lift', lift), ('elbow_flex', elbow), ('wrist_flex', wflex)):
                q[name] = _wrap_into(val, *self.limits[name])
                ok = ok and q[name] is not None
            if ok:
                q['wrist_roll'] = 0.0
                q['_branch'] = sgn
                out.append(q)
        return out

    def roll_for(self, q, close_dir, prefer=0.0):
        """wrist_roll that lines the jaw closing axis up with `close_dir` (either sign). (roll, alignment)."""
        qa = {**q, 'wrist_roll': 0.0}
        qb = {**q, 'wrist_roll': math.pi / 2}
        a = self.frames(qa)['gripper'][:3, 0]
        b = self.frames(qb)['gripper'][:3, 0]
        c = np.asarray(close_dir, float) / np.linalg.norm(close_dir)
        best = math.atan2(float(b @ c), float(a @ c))
        lo, hi = self.limits['wrist_roll']
        cands = [x for x in (best + k * math.pi for k in range(-3, 4)) if lo <= x <= hi]
        if not cands:
            return None, 0.0
        roll = min(cands, key=lambda x: abs(x - prefer))
        return roll, abs(float(a @ c) * math.cos(roll) + float(b @ c) * math.sin(roll))

    def close_axis(self, q):
        return self.frames(q)['gripper'][:3, 0]

    # ------------------------------------------------------------------------------- user frame
    def to_user(self, p):
        """Base-frame point -> (forward, left) metres from the pan axis, as measured with a tape."""
        v = np.asarray(p[:2], float) - self.pan_point
        f = np.array([math.cos(self.heading0), math.sin(self.heading0)])
        left = np.array([-f[1], f[0]])          # 90 deg counter-clockwise seen from above
        return float(v @ f), float(v @ left)

    def from_user(self, fwd, left):
        f = np.array([math.cos(self.heading0), math.sin(self.heading0)])
        lv = np.array([-f[1], f[0]])
        return self.pan_point + fwd * f + left * lv


# ------------------------------------------------------------------------------ grasp planning
class Box:
    """Object on the table: centre (x, y) in the base frame, yaw (rad), size (along own x, own y, height)."""

    def __init__(self, x, y, yaw, size):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        self.size = tuple(float(s) for s in size)

    def axes(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.array([c, s, 0.0]), np.array([-s, c, 0.0])

    def extent_along(self, d):
        ax, ay = self.axes()
        return abs(self.size[0] * float(ax @ d)) + abs(self.size[1] * float(ay @ d))


class PlanError(Exception):
    def __init__(self, msg, stage=0, code=None):
        super().__init__(msg)
        self.stage = stage      # how far planning got (higher = more informative reason)
        # API error code (see so100_interfaces): table hits COLLISION, width TOO_WIDE, otherwise UNREACHABLE
        self.code = code or ('COLLISION' if 'ชน' in msg else 'TOO_WIDE' if 'เกินก้าม' in msg else 'UNREACHABLE')


class GraspPlanner:
    APPROACH = 0.05      # pre-grasp distance back along the tool axis (m)
    LIFT = 0.05          # lift after grasping (m)
    NEAR_DOWN = math.radians(-65)   # at or below this pitch the jaws may turn to the object's own axes
    STEP = 0.005         # straight-line interpolation step (m)
    TABLE_CLEAR = 0.006  # finger tips stay this far above the table top
    ARM_CLEAR = 0.012    # arm links (motors, wrist) stay this far above the table top
    ARM_LINKS = ('shoulder', 'upper_arm', 'lower_arm', 'wrist')
    HAND_LINKS = ('gripper', 'jaw')

    def __init__(self, kin, table_z):
        self.k = kin
        self.table_z = table_z

    # pitch candidates: straight down first, then tilting up towards a horizontal side grasp
    PITCHES = [math.radians(a) for a in range(-90, 1, 5)]

    def pitches(self, mode='auto'):
        """Pitch candidates for a grasp mode: 'top' = (nearly) straight down, 'side' = tilted/horizontal."""
        if mode in ('', 'auto', None):
            return self.PITCHES
        if mode == 'top':
            return [p for p in self.PITCHES if p <= self.NEAR_DOWN]
        if mode == 'side':
            return [p for p in self.PITCHES if p > self.NEAR_DOWN]
        raise PlanError(f'grasp_mode ไม่รู้จัก: {mode!r} (ใช้ auto / top / side)', 9, 'FAILED')

    def picks(self, obj, current=None, mode='auto', errors=None, target=None):
        """All fully checked ways to pick `obj`, best first (generator)."""
        k = self.k
        errors = [] if errors is None else errors
        prefer = (current or {}).get('wrist_roll', 0.0)
        for pitch in self.pitches(mode):
            for branch in (1, -1):
                centre = np.array([obj.x, obj.y, self._grasp_height(obj, pitch)])
                sols = k.solve(centre, pitch, branch=branch)
                if not sols:
                    errors.append(PlanError('เอื้อมไม่ถึงตำแหน่งของ', 0))
                    continue
                if target is not None and not k.solve(np.array([target.x, target.y, centre[2] + 0.005]), pitch,
                                                      branch=branch):
                    errors.append(PlanError('เอื้อมไม่ถึงจุดวาง (ด้วยท่าจับเดียวกัน)', 4))   # cheap early reject
                    continue
                for extent, close_dir in self._close_dirs(obj, k.heading(sols[0]), pitch):
                    if extent > MAX_OBJECT + 1e-4:     # (rounding: a 6.5 cm side may come out as 6.50001)
                        errors.append(PlanError(
                            f'ของกว้าง {extent * 100:.1f} ซม. เกินก้าม (สูงสุด {MAX_OBJECT * 100:.1f})', 1))
                        continue
                    try:
                        yield self._pick_part(obj, pitch, branch, centre, close_dir, extent, opening_for(extent),
                                              prefer)
                    except PlanError as exc:
                        errors.append(exc)

    @staticmethod
    def _best(errors):
        return max(errors, key=lambda e: e.stage) if errors else PlanError('เอื้อมไม่ถึงตำแหน่งของ', 0)

    @staticmethod
    def _join(held, place):
        return {**held, **place, 'steps': {**held['steps'], **place['steps']}}

    def plan_pick(self, obj, current=None, mode='auto'):
        """Pick only (Pick action / voice 'carry')."""
        errors = []
        for held in self.picks(obj, current, mode, errors):
            return held
        raise self._best(errors)

    def plan_place(self, obj, held, target):
        """Place only, keeping the grasp `held` from plan_pick (Place action / voice 'drop')."""
        return self._join(held, self._place_part(obj, held, target))

    def plan(self, obj, target, current=None, mode='auto'):
        """Pick and place: the first grasp that also works at the target (same grasp at both ends)."""
        errors = []
        for held in self.picks(obj, current, mode, errors, target):
            try:
                return self._join(held, self._place_part(obj, held, target))
            except PlanError as exc:
                errors.append(exc)
        raise self._best(errors)

    def _close_dirs(self, box, heading, pitch):
        side = np.array([-math.sin(heading), math.cos(heading), 0.0])
        if pitch <= self.NEAR_DOWN:
            ax, ay = box.axes()
            dirs = sorted([(box.size[0], ax), (box.size[1], ay)], key=lambda t: t[0])
            return [(e, d) for e, d in dirs]
        return [(box.extent_along(side), side)]

    def _grasp_height(self, box, pitch):
        h = box.size[2]
        if pitch <= self.NEAR_DOWN:
            depth = min(GRIP_DEPTH, 0.6 * h)
            return self.table_z + h - depth + (GRASP_Y - TIP_Y)   # finger tips at (top - depth)
        return self.table_z + min(max(0.5 * h, 0.015), max(h - 0.01, 0.01))

    def _pose(self, centre, pitch, close_dir, gap, branch, prefer_roll, x_off=None):
        """IK for 'object centre held in the jaws at `centre`' -> joint dict incl. roll.
        x_off: gripper-frame x of `centre` (default: next to the fixed finger, see object_offset)."""
        if x_off is None:
            x_off = object_offset(gap)
        point = np.asarray(centre, float)
        q = None
        for _ in range(4):           # offset depends on roll, roll depends on pan: converge
            sols = self.k.solve(point, pitch, branch=branch)
            if not sols:
                raise PlanError('เอื้อมไม่ถึงบางช่วงของการหยิบ/วาง', 3)
            q = sols[0]
            roll, align = self.k.roll_for(q, close_dir, prefer_roll)
            if roll is None or align < 0.95:
                raise PlanError('หมุนข้อมือให้ก้ามตรงกับของไม่ได้', 3)
            q['wrist_roll'] = roll
            prefer_roll = roll   # stick to this jaw side (+90 vs -90 can tie and flip every iteration)
            point = np.asarray(centre, float) - x_off * self.k.close_axis(q)
        err = np.linalg.norm(self.k.gripper_point(q, (x_off, GRASP_Y, 0)) - centre)
        if err > 0.002:
            raise PlanError(f'IK ไม่ลงตัว ({err * 1000:.0f} มม.)', 3)
        return q

    def describe(self, pitch):
        d = math.degrees(-pitch)        # 90 = pointing straight down, 0 = horizontal
        if d >= 89.5:
            return 'จับจากด้านบน (ก้ามชี้ลงตรง)'
        if pitch <= self.NEAR_DOWN:
            return f'จับจากด้านบน (ก้ามเอียงจากแนวดิ่ง {90 - d:.0f}°)'
        return f'จับจากด้านข้าง (ก้ามก้มลงจากแนวนอน {d:.0f}°)'

    def _tips_low(self, q, gap):
        pts = [(FIXED_INNER_X + 0.005, TIP_Y, 0.0), (FIXED_INNER_X - gap - 0.005, TIP_Y + 0.003, 0.0)]
        return min(self.k.gripper_point(q, p)[2] for p in pts)

    def check_table(self, qs, gap, what, stage):
        """Every link of every pose stays above the table (whole meshes, not just the finger tips)."""
        jaw = min(max(jaw_for_gap(gap), 0.0), JAW_G_MAX)
        for q in qs:
            if not self.k.meshes:
                if self._tips_low(q, gap) < self.table_z + self.TABLE_CLEAR:
                    raise PlanError(f'{what}: ปลายก้ามจะชนโต๊ะ', stage)
                continue
            qj = {**q, 'gripper': jaw}
            if self.k.lowest(qj, self.ARM_LINKS) < self.table_z + self.ARM_CLEAR:
                raise PlanError(f'{what}: ข้อแขน/ข้อมือจะชนโต๊ะ', stage)
            if self.k.lowest(qj, self.HAND_LINKS) < self.table_z + self.TABLE_CLEAR - 0.002:
                raise PlanError(f'{what}: ก้ามจะชนโต๊ะ', stage)

    def _line(self, a, b, pitch, close_dir, gap, branch, roll, x_off=None):
        """Straight line of the object centre from a to b -> list of joint dicts (a excluded)."""
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / self.STEP)))
        out, prev = [], None
        for i in range(1, n + 1):
            q = self._pose(a + (b - a) * i / n, pitch, close_dir, gap, branch, roll, x_off)
            if prev is not None and max(abs(q[j] - prev[j]) for j in ARM) > 0.25:
                raise PlanError('ท่าแขนกระโดดกลางทาง', 3)
            out.append(q)
            prev = q
        return out

    def _pick_part(self, obj, pitch, branch, centre, close_dir, extent, gap, prefer):
        k = self.k
        grasp = self._pose(centre, pitch, close_dir, gap, branch, prefer)
        self.check_table([grasp], gap, 'ตอนหยิบ', 2)
        back = self.approach_dist(obj, centre, pitch)
        pre = centre - back * k.tool_dir(k.heading(grasp), pitch)
        lift = centre + np.array([0, 0, self.LIFT])
        steps = {
            'pre': self._pose(pre, pitch, close_dir, gap, branch, grasp['wrist_roll']),
            'approach': self._line(pre, centre, pitch, close_dir, gap, branch, grasp['wrist_roll']),
            'lift': self._line(centre, lift, pitch, close_dir, gap, branch, grasp['wrist_roll']),
        }
        self.check_table([steps['pre']] + steps['approach'] + steps['lift'], gap, 'ช่วงหยิบ', 2)
        return {
            'pitch': pitch, 'branch': branch, 'extent': extent, 'gap': gap, 'close_dir': close_dir,
            'jaw_open': min(jaw_for_gap(gap), JAW_G_MAX), 'grasp_centre': centre, 'grasp': grasp,
            'back': back, 'object': (obj.x, obj.y, obj.yaw, obj.size), 'steps': steps,
            'mode': self.describe(pitch),
        }

    def _place_part(self, obj, held, target):
        k = self.k
        pitch, branch, gap, back = held['pitch'], held['branch'], held['gap'], held['back']
        # place: same pitch keeps the object upright; bottom 5 mm above the table
        place = np.array([target.x, target.y, held['grasp_centre'][2] + 0.005])
        sols = k.solve(place, pitch, branch=branch)
        if not sols:
            raise PlanError('เอื้อมไม่ถึงจุดวาง (ด้วยท่าจับเดียวกัน)', 4)
        p_head = k.heading(sols[0])
        place_close, final_yaw = self._place_close(obj, target.yaw, pitch, held['close_dir'], p_head, held['grasp'])
        place_q = self._pose(place, pitch, place_close, gap, branch, held['grasp']['wrist_roll'])
        self.check_table([place_q], gap, 'ตอนวาง', 4)
        above_place = place + np.array([0, 0, self.LIFT])
        retreat = place - back * k.tool_dir(p_head, pitch)
        steps = {
            'above_place': self._pose(above_place, pitch, place_close, gap, branch, place_q['wrist_roll']),
            'lower': self._line(above_place, place, pitch, place_close, gap, branch, place_q['wrist_roll']),
            'retreat': self._line(place, retreat, pitch, place_close, gap, branch, place_q['wrist_roll']),
        }
        self.check_table([steps['above_place']] + steps['lower'] + steps['retreat'], gap, 'ช่วงวาง', 4)
        return {'place_centre': place, 'final_yaw': final_yaw, 'steps': steps}

    def approach_dist(self, obj, centre, pitch):
        """How far back along the tool axis the approach starts: far enough that the finger tips start
        2 cm above a tall object's top (a short fixed distance clipped tall objects and tipped them over)."""
        down = math.sin(-pitch)
        if down < 0.5:                       # side grasp: comes in sideways, the fixed distance is fine
            return self.APPROACH
        tips_below_top = obj.size[2] + self.table_z - (centre[2] - (GRASP_Y - TIP_Y) * down)
        return max(self.APPROACH, (tips_below_top + 0.02) / down)

    def _place_close(self, obj, target_yaw, pitch, close_dir, p_head, grasp):
        """Jaw closing direction at the target and the object's final yaw."""
        if pitch <= self.NEAR_DOWN:
            # keep the object's yaw relative to the jaws, then turn it to the target yaw
            d_yaw = target_yaw - obj.yaw
            c, s = math.cos(d_yaw), math.sin(d_yaw)
            return (np.array([c * close_dir[0] - s * close_dir[1], s * close_dir[0] + c * close_dir[1], 0.0]),
                    target_yaw)
        return np.array([-math.sin(p_head), math.cos(p_head), 0.0]), obj.yaw + (p_head - self.k.heading(grasp))

    # ------------------------------------------------------------- "where can it go" map for RViz
    def pick_options(self, obj, current=None):
        """Ways to pick `obj` (grasp + pre-grasp checked; the straight lines are not), best first."""
        k = self.k
        prefer = (current or {}).get('wrist_roll', 0.0)
        opts = []
        for pitch in self.PITCHES:
            for branch in (1, -1):
                centre = np.array([obj.x, obj.y, self._grasp_height(obj, pitch)])
                sols = k.solve(centre, pitch, branch=branch)
                if not sols:
                    continue
                for extent, close_dir in self._close_dirs(obj, k.heading(sols[0]), pitch):
                    if extent > MAX_OBJECT + 1e-4:
                        continue
                    gap = opening_for(extent)
                    try:
                        grasp = self._pose(centre, pitch, close_dir, gap, branch, prefer)
                        pre = centre - self.approach_dist(obj, centre, pitch) * k.tool_dir(k.heading(grasp), pitch)
                        self.check_table([grasp, self._pose(pre, pitch, close_dir, gap, branch, grasp['wrist_roll'])],
                                         gap, '', 2)
                    except PlanError:
                        continue
                    opts.append((pitch, branch, centre, close_dir, gap, grasp))
                    break
        return opts

    def can_place(self, obj, opt, x, y, yaw):
        """Quick check of the place poses (place, above it, retreat) for one pick option."""
        k = self.k
        pitch, branch, centre, close_dir, gap, grasp = opt
        place = np.array([x, y, centre[2] + 0.005])
        sols = k.solve(place, pitch, branch=branch)
        if not sols:
            return False
        p_head = k.heading(sols[0])
        place_close, _ = self._place_close(obj, yaw, pitch, close_dir, p_head, grasp)
        try:
            q = self._pose(place, pitch, place_close, gap, branch, grasp['wrist_roll'])
            above = self._pose(place + np.array([0, 0, self.LIFT]), pitch, place_close, gap, branch, q['wrist_roll'])
            back = self._pose(place - self.approach_dist(obj, centre, pitch) * k.tool_dir(p_head, pitch), pitch,
                              place_close, gap, branch, q['wrist_roll'])
            self.check_table([q, above, back], gap, '', 4)
        except PlanError:
            return False
        return True

    def place_map(self, obj, target_yaw, cells, current=None, cancelled=None):
        """For each (x, y) in `cells`: can `obj` be put down there? None if cancelled."""
        opts = self.pick_options(obj, current)
        out = []
        for x, y in cells:
            if cancelled and cancelled():
                return None
            out.append(any(self.can_place(obj, o, x, y, target_yaw) for o in opts))
        return out
