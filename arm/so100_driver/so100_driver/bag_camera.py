"""Camera bag pick for /arm/pick_bag (bag_method = camera) and its put-down for /arm/place.

Since 2026-11-18 the webcam (webcam_view.py, port 8082) is FIXED under the robot's top deck, looking ahead and
down; it no longer rides on the forearm. Turning the arm does not change its picture, so the old "turn the
arm until the bag is in the middle of the image" step is gone. The camera's pinhole model comes from the HMC
team's calibration (front_camera.json, robot frame with its origin at the turning centre).

  1. where (before EVERY try: a miss or a drop may have pushed the bag): the arm goes to LOOK if it would be
     in the picture, then the bag's centre comes from the image (locate: the front wall's rim, bag_cam_rim_h
     above the floor; its foot on the floor when that is in view). The goal's x, y only say roughly where:
     more than GOAL_TOL apart = an error. A bag whose rim is not about bag_cam_width wide (fallen over) = an
     error. No bag in the picture: the goal on the first try, an error later. Orientation: the goal's yaw =
     the handles' line; no yaw = the bag faces the robot (handles parallel to the robot's y axis).
  2. what: the bag's FRONT wall (the one nearer the arm), bag_cam_half_depth in front of the centre. The jaws
     (pitched -60 deg) close across that wall near its rim: finger tips bag_cam_close_h above the floor (the
     paper rim, not the handle arcs: a handle has rope only at its very top in the middle). One jaw moves, so
     the middle of the open gap (gripper x GAP_MID), not the tool axis, is put on the wall. The wrist roll
     lines the closing axis up with the wall's normal (Kinematics.roll_for), not a guessed sign.
  3. check: lift STRAIGHT up 20 cm, slowly (same reach and tilt); the Xtion must see the floor where the bag
     stood (its bottom off the floor). Not the front camera's top edge: it also goes up when the jaws only
     stretch the wall up (video of 2026-11-18). A miss: open, back off, look again (1.), the next try a
     little further in / out (TRIES) from where the bag is now.
  4. raise: tilt up in small steps at the same reach; then ALWAYS the front camera: the bag must have left
     its place on the floor, else (it slipped out) open, look again, next try.
bag_hover_only: stop with the jaws OPEN at the close height and hold still there (a test: hang a plumb line,
measure how far the gap is from the wall); the log has the asked and the real (forward kinematics) gap middle.
Put-down: turn towards the target at the hold height, come down along the same tilt steps, open, back off.
"""
import json
import math
import os
import time
import urllib.request

import numpy as np

from . import bag_vision as bv
from .ik import ARM, FIXED_INNER_X, JAW_G_MAX, TIP_Y, jaw_gap
from .task_rules import Failed

TIP = -TIP_Y                      # gripper origin -> finger tips along the tool axis (m)
GAP_MID = FIXED_INNER_X - jaw_gap(JAW_G_MAX) / 2   # gripper-frame x of the middle of the fully open jaw gap
# (extra reach along the bearing m, pitch deg), best first: the open gap is 7.5 cm, so +-1.5 cm still keeps a
# wall that is where it should be inside the jaws
TRIES = ((0.0, -60), (0.015, -60), (-0.015, -60), (0.03, -50))
GOAL_TOL = 0.10                   # m: the image wins over the goal's x, y; further apart than this = an error
# arm pose for looking: tips high over the base, every link out of the front camera's picture (READY is in it,
# right where a near bag shows) and clear of the deck (checked against the deck model, 2026-11-18)
LOOK = {"shoulder_pan": 0.0, "shoulder_lift": 0.5, "elbow_flex": -1.6, "wrist_flex": 0.0, "wrist_roll": 0.0}
LIFT_DZ = (0.02, 0.04, 0.06, 0.08, 0.10)
STRAIGHT_UP = tuple(0.02 * k for k in range(1, 11))   # m: the first lift, straight up to 20 cm
LIFT_SPEED = 0.25                 # rad/s of the fastest joint on that lift
UP = [(-52, 0.0), (-45, 0.02), (-38, 0.04), (-30, 0.06), (-22, 0.08), (-15, 0.10), (-8, 0.12), (0, 0.14),
      (0, 0.17), (0, 0.20), (0, 0.23), (0, 0.26), (0, 0.29), (0, 0.32)]   # (pitch, tip z): raise, small tilt steps
HOLD_R = 0.32                     # finger tips from the pan axis while held up
DOWN = [(0, 0.14), (-30, 0.06), (-60, -0.02)]   # put-down: few, longer moves (the task gives it 20 s)
PUT_DROP = 0.03                   # m above the grasp height the put-down lets go (the bag drops that far)
JOINT_STEP = 0.03                 # rad between path points
FREE_SPEED, HELD_SPEED, PUT_SPEED = 0.8, 0.4, 0.6  # rad/s of the fastest joint: empty / lifting / putting down


# ---------------------------------------------------------------------- geometry (no ROS: unit-tested)
class FrontCamera:
    """The fixed front camera from front_camera.json (HMC team, hmc-follow/front_camera.py): pinhole fx = fy,
    at (x, y, z) in the robot frame (x ahead, y left, z up from the floor, origin at the turning centre),
    R = Rz(yaw) Ry(pitch) Rx(roll) R0, R0 = OpenCV camera axes -> robot with all angles zero."""

    R0 = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])

    def __init__(self, d):
        if not d.get("calibrated"):
            raise ValueError("front camera not calibrated")
        self.fx, self.cx, self.cy = float(d["fx"]), float(d["cx"]), float(d["cy"])
        self.pos = np.array([float(d["x"]), float(d["y"]), float(d["z"])])
        self.mirror = bool(d.get("mirror", False))
        y, p, r = (math.radians(float(d.get(k, 0.0))) for k in ("yaw_deg", "pitch_deg", "roll_deg"))
        cy, sy, cp, sp, cr, sr = math.cos(y), math.sin(y), math.cos(p), math.sin(p), math.cos(r), math.sin(r)
        rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
        self.rot = rz @ ry @ rx @ self.R0

    @classmethod
    def load(cls, path):
        with open(os.path.expanduser(path)) as f:
            return cls(json.load(f))

    def ray(self, u, v):
        d = np.array([(u - self.cx) / self.fx * (-1.0 if self.mirror else 1.0), (v - self.cy) / self.fx, 1.0])
        d = self.rot @ d
        return d / np.linalg.norm(d)

    def project(self, p):
        pc = (np.asarray(p, float) - self.pos) @ self.rot
        u = self.fx * pc[0] / pc[2]
        return (self.cx - u if self.mirror else self.cx + u), self.cy + self.fx * pc[1] / pc[2]

    def side_at(self, u, v, x):
        """Robot-frame y where the ray of pixel (u, v) passes the forward distance x (None: never)."""
        d = self.ray(u, v)
        if d[0] <= 1e-6:
            return None
        return float(self.pos[1] + (x - self.pos[0]) / d[0] * d[1])


def front_wall(centre, line_yaw, half_depth):
    """Bag centre (forward, left from the pan axis), its handles' line (rad, same frame) -> (point on the wall
    nearer the arm, unit normal pointing away from the arm)."""
    c = np.asarray(centre, float)
    n = np.array([math.sin(line_yaw), -math.cos(line_yaw)])
    if n @ c < 0:
        n = -n
    return c - half_depth * n, n


def _at_height(cam, u, v, z):
    """Where the ray of pixel (u, v) comes down to height z (camera's robot frame), or None."""
    d = cam.ray(u, v)
    if d[2] >= -1e-6:
        return None
    s = (z - cam.pos[2]) / d[2]
    return cam.pos + s * d if s > 0 else None


def locate(cam, b, pan, half_depth, top_h, foot=None, lying_h=0.12):
    """The bag's centre from a front-camera blob (bag_vision.find_bag), the bag standing and facing the arm.
    pan: the arm's pan axis (x, y) in the camera's robot frame. Two ways:
      bottom: foot = image row of the front wall's foot on the floor (bag_vision.foot_y; z = 0, known):
              centre half_depth beyond it. Used when the foot is in the picture (not cut off by the frame).
      top:    the blob's top edge (median over its columns: the handles' arcs are thin) is the front wall's
              rim, top_h above the floor (the camera looks steeply down on the front wall; its rim is the
              highest bag-coloured line): centre half_depth beyond it. Used when the foot is out of the
              picture, which is the usual case at pick distances (less exact: top_h is per bag).
    With the foot in view it also checks the bag stands: the top edge seen right above the foot must be
    higher than lying_h. Returns dict(centre (forward, left) from the pan axis, from, standing True / False /
    None = cannot tell, top_seen_h, centre_top, centre_bottom) or None when no ray hits."""
    pan = np.asarray(pan, float)
    # every column's top (the gripper-hidden filter of find_bag drops most of them here: the bag's red-orange
    # face looks like the gripper's orange), across the middle of the blob's width
    t = b.get("all_tops", b["tops"])
    ut, vt = (b["x0"] + b["x1"]) / 2, float(np.median(t[:, 1]))
    out = {"standing": None}
    rim = _at_height(cam, ut, vt, top_h)
    if rim is not None:
        rel = rim[:2] - pan
        out["centre_top"] = rel + half_depth * rel / np.linalg.norm(rel)
        a, c = _at_height(cam, b["x0"], vt, top_h), _at_height(cam, b["x1"], vt, top_h)
        if a is not None and c is not None:
            out["width"] = float(np.linalg.norm(a[:2] - c[:2]))     # m across the rim (log; lying check)
    if foot is not None:
        foot = _at_height(cam, (b["x0"] + b["x1"]) / 2, foot, 0.0)
        if foot is not None:
            rel = foot[:2] - pan
            n = rel / np.linalg.norm(rel)
            out["centre_bottom"] = rel + half_depth * n
            d = cam.ray(ut, vt)
            if abs(d[:2] @ n) > 1e-6:
                s = (rel @ n - (cam.pos[:2] - pan) @ n) / (d[:2] @ n)
                out["top_seen_h"] = float(cam.pos[2] + s * d[2])
                out["standing"] = out["top_seen_h"] > lying_h
    for k in ("centre_bottom", "centre_top"):
        if k in out:
            out["centre"], out["from"] = tuple(float(v) for v in out[k]), k.split("_")[1]
            return out
    return None


# ---------------------------------------------------------------------- the pick
class CameraBag:
    def __init__(self, node):
        self.n = node
        self.kin = node.kin
        self.theta = 0.0

    # ------------------------------------------------------------------ helpers
    def p(self, name):
        return self.n.get_parameter(name).value

    def web(self):
        """A frame of the fixed front camera, or None."""
        import cv2
        for _ in range(3):
            try:
                d = urllib.request.urlopen(str(self.p("bag_cam_url")), timeout=4).read()
                im = cv2.imdecode(np.frombuffer(d, np.uint8), cv2.IMREAD_COLOR)
                if im is not None:
                    return im
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.3)
        return None

    def seen(self):
        """The bag in the front camera now (bag_vision.find_bag), or None (no frame / not seen)."""
        im = self.web()
        return None if im is None else bv.find_bag(im)

    def xtion(self):
        import cv2
        try:
            r = urllib.request.urlopen(str(self.p("bag_xtion_url")), timeout=5)
            d, k = b"", 0
            end = time.monotonic() + 5
            while time.monotonic() < end:
                d += r.read(4096)
                a = d.find(b"\xff\xd8")
                b = d.find(b"\xff\xd9", a + 2) if a >= 0 else -1
                if b >= 0:
                    k += 1
                    if k == 2:
                        return cv2.imdecode(np.frombuffer(d[a:b + 2], np.uint8), cv2.IMREAD_COLOR)
                    d = d[b + 2:]
        except Exception:  # noqa: BLE001
            pass
        return None

    def user_dir(self, v):
        """User-frame (forward, left) direction -> base-frame 3-vector."""
        h = self.kin.heading0
        f, lv = np.array([math.cos(h), math.sin(h)]), np.array([-math.sin(h), math.cos(h)])
        return np.array([*(v[0] * f + v[1] * lv), 0.0])

    def solve(self, f, l, z, pitch, roll, xoff=0.0, cur=None):
        """Joint path (from cur, default the arm now) putting gripper-frame (xoff, 0, 0) at user (f, l), base z,
        tool pitch/roll (deg); None if out of reach or it would touch the robot body."""
        x, y = self.kin.from_user(f, l)
        target = np.array([x, y, z])
        if xoff:
            for q0 in self.kin.solve(target, math.radians(pitch), depth=0.0)[:1]:
                q0.pop("_branch", None)
                q0["wrist_roll"] = math.radians(roll)
                # the whole shift (up / down too): the gap middle at the tips lands at the asked height (the
                # forearm-camera recipe shifted sideways only, 1.4 cm higher at pitch -60)
                target = target - xoff * self.kin.frames(q0)["gripper"][:3, 0]
        cur = {j: float((cur or self.n.joints)[j]) for j in ARM}
        for q in self.kin.solve(target, math.radians(pitch), depth=0.0):
            q.pop("_branch", None)
            q["wrist_roll"] = math.radians(roll)
            lo, hi = self.kin.limits["wrist_roll"]
            if not lo <= q["wrist_roll"] <= hi:
                continue
            steps = max(2, int(math.ceil(max(abs(q[j] - cur[j]) for j in ARM) / JOINT_STEP)))
            path = [{j: cur[j] + (q[j] - cur[j]) * t for j in ARM} for t in np.linspace(0, 1, steps + 1)[1:]]
            if self.n.planner.hits_body(q) or any(self.n.planner.hits_body(s) for s in path):
                continue
            return path
        return None

    def tip(self, r, tz, pitch, roll, what, pull_in=0.0, speed=FREE_SPEED, sim=None):
        """Finger tips r m out along the bag's bearing self.theta, tz high (base frame), the gap middle on the
        line; if out of reach, up to pull_in m closer (same bearing). sim: a joint dict to move on paper only
        (dry run of a sequence: updated in place, the arm does not move)."""
        c, s = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
        for d in np.arange(0.0, pull_in + 1e-6, 0.01):
            ro = r - d - TIP * c
            path = self.solve(ro * math.cos(self.theta), ro * math.sin(self.theta), tz - TIP * s, pitch, roll,
                              GAP_MID, cur=sim)
            if path is not None:
                if sim is not None:
                    sim.update(path[-1])
                else:
                    self.n.line(path, what, speed, JOINT_STEP)
                return
        raise Failed(f"{what}: แขนไปไม่ถึง (ปลายก้าม {r:.3f} ม. สูง {tz:.3f})", "UNREACHABLE")

    def roll_for(self, r, tz, pitch, close_dir):
        """Wrist roll (deg) lining the closing axis up with close_dir at the grasp pose, and the alignment."""
        c, s = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
        ro = r - TIP * c
        x, y = self.kin.from_user(ro * math.cos(self.theta), ro * math.sin(self.theta))
        sols = self.kin.solve(np.array([x, y, tz - TIP * s]), math.radians(pitch), depth=0.0)
        if not sols:
            raise Failed(f"เอื้อมไม่ถึงผนังหน้าถุง (ปลายก้าม {r:.3f} ม. จากแกนหมุน)", "UNREACHABLE")
        roll, align = self.kin.roll_for(sols[0], close_dir, 0.0)
        if roll is None:
            raise Failed("หมุนข้อมือให้ก้ามหนีบขวางผนังถุงไม่ได้", "UNREACHABLE")
        return math.degrees(roll), align

    def above(self, r, close_z, pitch, roll, sim=None):
        """To 10 cm above the close pose: from the robot's side if the arm gets there (a near wall: that pose
        is in the deck), else straight from above, else through a higher pose first."""
        for pre_r, pre_z in ((r - 0.04, 0.10), (r, 0.10), (r, 0.20)):
            if sim is None and not self.feasible(r, close_z, pitch, roll, (pre_r, pre_z)):
                continue                           # (the dry run picks the way; never stop half-way there)
            try:
                self.tip(pre_r, close_z + pre_z, pitch, roll, "ไปเหนือถุง", sim=sim)
            except Failed:
                continue
            try:
                self.tip(r, close_z + 0.10, pitch, roll, "เหนือถุง", sim=sim)
                return
            except Failed:
                continue
        raise Failed(f"ไปเหนือผนังหน้าถุงไม่ได้ (ปลายก้าม {r:.3f} ม. จากแกนหมุน)", "UNREACHABLE")

    def feasible(self, r, close_z, pitch, roll, pre=None):
        """Dry run from the arm now: above the wall (by `pre` = (r, dz), or any way), down to the close, the
        10 cm lift. True if every move is in reach and clear of the robot body."""
        sim = {j: float(self.n.joints[j]) for j in ARM}
        try:
            if pre is None:
                self.above(r, close_z, pitch, roll, sim)
            else:
                self.tip(pre[0], close_z + pre[1], pitch, roll, "", sim=sim)
                self.tip(r, close_z + 0.10, pitch, roll, "", sim=sim)
            for dz in (0.08, 0.06, 0.04, 0.02, 0.0):
                self.tip(r, close_z + dz, pitch, roll, "", sim=sim)
            for dz in LIFT_DZ:
                self.tip(r, close_z + dz, pitch, roll, "", pull_in=0.05, sim=sim)
        except Failed:
            return False
        return True

    def closing_sign(self, r, close_z, pitch, roll, close_dir):
        """The jaws' closing axis at the close pose . close_dir (+ = pointing away from the arm); -2 when the
        pose is out of reach."""
        sim = {j: float(self.n.joints[j]) for j in ARM}
        try:
            self.tip(r, close_z, pitch, roll, "", sim=sim)
        except Failed:
            return -2.0
        a = self.kin.close_axis(sim)
        return float(a @ close_dir / (np.linalg.norm(a) * np.linalg.norm(close_dir)))

    def grasp_pose(self, r, close_z, pitch, close_dir):
        """(pitch, roll deg, alignment) for a try: the wrist roll that closes the jaws across the wall, or that
        turned half a turn (the fixed jaw on the far side: near the robot the gripper body would touch the deck);
        the try's pitch, else steeper, else flatter. Raises UNREACHABLE when nothing gets there."""
        lo, hi = (math.degrees(v) for v in self.kin.limits["wrist_roll"])
        min_align = float(self.p("bag_cam_min_align"))
        for pt in dict.fromkeys((pitch, pitch - 10, pitch + 10)):
            try:
                roll, align = self.roll_for(r, close_z, pt, close_dir)
            except Failed:
                continue
            if align < min_align:                  # the jaws would pinch the wall slanted (a flatter pitch)
                continue
            # the jaws' closing axis pointing AWAY from the arm (along the wall normal) first: on the robot,
            # 2026-11-18, every close with it pointing back at the robot (roll ~+20) came back empty (8 of 8),
            # both closes with it pointing away (roll ~+200..209) held the wall
            rolls = [rl for rl in (roll, roll + 180.0, roll - 180.0) if lo <= rl <= hi]
            rolls.sort(key=lambda rl: -self.closing_sign(r, close_z, pt, rl, close_dir))
            for rl in rolls:
                if self.feasible(r, close_z, pt, rl):
                    if self.closing_sign(r, close_z, pt, rl, close_dir) < 0:
                        self.n.say(f"ท่าแนวหุบชี้ออกจากหุ่นไปไม่ได้ที่ปลายก้าม {r * 100:.1f} ซม.: ใช้แนวกลับด้าน "
                                   "(เคยหุบเปล่า)")
                    return pt, rl, align
        raise Failed(f"เอื้อมไปหนีบขวางผนังหน้าถุงไม่ได้ (ปลายก้าม {r:.3f} ม. จากแกนหมุน สูง "
                     f"{close_z - self.n.table_z:.2f} ม. จากพื้น)", "UNREACHABLE")

    # ------------------------------------------------------------------ where is the bag
    def in_view(self, cam, q):
        """Does the arm (its link meshes, joints q) show in the front camera's picture?"""
        px, py = (float(v) for v in self.p("bag_front_cam_pan"))
        pts = self.kin.link_points({j: float(q[j]) for j in ARM}, list(self.kin.meshes))
        for p in pts[::5]:
            f, l = self.kin.to_user(p)
            c = np.array([f + px, l + py, p[2] - self.n.table_z])
            if (c - cam.pos) @ cam.rot[:, 2] <= 0:
                continue
            u, v = cam.project(c)
            if 0 <= u < 640 and 0 <= v < 480:
                return True
        return False

    def look(self, cam):
        """The arm out of the camera's way (LOOK) if it is in the picture, then a frame -> (blob, located)."""
        if cam is not None and self.n.joints and self.in_view(cam, self.n.joints):
            self.n.moveit_to(LOOK, "ยกแขนพ้นภาพกล้องหน้า (ดูถุง)")
            time.sleep(0.5)
        im = self.web()
        if im is None:
            self.n.say("ไม่มีภาพจากกล้องหน้า (webcam_view.py พอร์ต 8082)")
            return None, None
        b = bv.find_bag(im)
        if b is None or cam is None:
            return b, None
        px, py = (float(v) for v in self.p("bag_front_cam_pan"))
        loc = locate(cam, b, (px, py), float(self.p("bag_cam_half_depth")), float(self.p("bag_cam_rim_h")),
                     foot=bv.foot_y(im, b))
        return b, loc

    def where(self, obj, first=True):
        """Look at the bag (front camera: the 1st try and after every miss or drop) -> dict: centre (user f, l),
        wall point, normal, seen (blob), from. The goal's x, y only say roughly where: the image wins, and an
        image more than GOAL_TOL from the goal is an error. No bag in the image on the first look: the goal; on
        a later look: an error (the bag is gone or hidden). A bag that does not stand facing the arm: an error."""
        f, l = self.kin.to_user(np.array([obj.x, obj.y]))
        line = (obj.yaw - self.kin.heading0) if getattr(obj, "yaw_given", True) else math.pi / 2
        out = {"goal": (f, l), "from": "goal", "line": line, "seen": None}
        cam = None
        try:
            cam = FrontCamera.load(str(self.p("bag_front_cam_file")))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.n.say(f"ไม่มีค่าคาลิเบรตกล้องหน้า ({exc})")
        b, loc = self.look(cam)
        out["seen"] = b
        if loc is None:
            if not first:
                raise Failed("ดูกล้องหน้าใหม่แล้วไม่เห็นถุง (ถุงหลุดไปไหน หรือมีอะไรบัง): หยุด", "MISSED")
            self.n.say("กล้องหน้าหาถุงไม่ได้: ใช้ตำแหน่งจากคำสั่งอย่างเดียว")
        else:
            c = loc["centre"]
            off = math.hypot(c[0] - f, c[1] - l)
            width, want = loc.get("width"), float(self.p("bag_cam_width"))
            self.n.get_logger().info(
                f"front camera bag: centre fwd {c[0]:.3f} left {c[1]:+.3f} (from the {loc['from']}; goal fwd {f:.3f} "
                f"left {l:+.3f}, {off * 100:.1f} cm apart); rim width {width if width else float('nan'):.3f} m, "
                f"standing {loc['standing']} (top seen {loc.get('top_seen_h', float('nan')):.2f} m)")
            if loc["standing"] is False or (width is not None and not 0.55 * want < width < 2.2 * want):
                raise Failed(f"ถุงล้มหรือไม่ได้ตั้งหันหน้าเข้าหุ่น (กล้องเห็นขอบปากถุงกว้าง {(width or 0) * 100:.0f} "
                             f"ซม. ควรราว {want * 100:.0f}): ตั้งถุงใหม่แล้วสั่งอีกครั้ง", "FAILED")
            if off > GOAL_TOL:
                raise Failed(f"กล้องหน้าเห็นถุงห่างจากตำแหน่งที่สั่ง {off * 100:.0f} ซม. (เกิน "
                             f"{GOAL_TOL * 100:.0f}): เช็กตำแหน่งที่ส่งมา", "FAILED")
            f, l, out["from"] = c[0], c[1], "image " + loc["from"]
        wall, normal = front_wall((f, l), line, float(self.p("bag_cam_half_depth")))
        out.update(centre=(f, l), wall=wall, normal=normal)
        return out

    def close_h(self, obj):
        """Finger-tip height above the floor at the close: the paper rim, lower for a bag lower than that."""
        h = float(self.p("bag_cam_close_h"))
        top = obj.size[2]
        if math.isfinite(top) and top < h + 0.03:
            h = max(0.05, top - 0.03)
        return h

    # ------------------------------------------------------------------ pick
    def pick(self, obj):
        """obj: Box from the PickBag goal (x, y = roughly the bag's centre). Every try looks at the bag again
        (the last one may have pushed it), plans the grasp there, closes, lifts, checks; a miss or a drop:
        open, look again, next try. Returns the result text (holding)."""
        if not self.n.joints:
            raise Failed("ไม่รู้ท่าแขน (ไม่มี joint_states)", "FAILED")
        tries = TRIES[:max(1, int(self.p("bag_cam_tries")))]
        hover = bool(self.p("bag_hover_only"))
        closed, why = 0, "ถุงไม่ลอยจากพื้น"
        close_z = self.n.table_z + self.close_h(obj)
        for i, (dt, pitch) in enumerate(tries):
            self.n.progress(1, 4, "look", f"ลองที่ {i + 1}/{len(tries)}: ดูถุงจากกล้องหน้า...")
            w = self.where(obj, first=(i == 0))
            start = w["seen"]
            wall, normal = w["wall"], w["normal"]
            self.theta = math.atan2(wall[1], wall[0])
            r_wall = float(np.hypot(*wall))
            close_dir = self.user_dir(normal)
            fl = w["centre"]
            self.n.get_logger().info(
                f"camera bag try {i + 1}: centre fwd {fl[0]:.3f} left {fl[1]:+.3f} (from {w['from']}, goal fwd "
                f"{w['goal'][0]:.3f} left {w['goal'][1]:+.3f}); handles' line {math.degrees(w['line']):.0f} deg; "
                f"front wall fwd {wall[0]:.3f} left {wall[1]:+.3f}, bearing {math.degrees(self.theta):+.1f} deg, "
                f"{r_wall:.3f} m from the pan axis; tips close {close_z - self.n.table_z:.3f} m above the floor")
            r = r_wall + dt
            # where the gap middle should end up (log), then the pose asked of the arm: the measured bias of the
            # close pose taken off (bag_cam_close_bias)
            want = (r * math.cos(self.theta), r * math.sin(self.theta), close_z - self.n.table_z)
            bias_f, bias_up = (float(v) for v in self.p("bag_cam_close_bias"))
            r, cz = r + bias_f, close_z + bias_up
            try:
                pitch, roll, align = self.grasp_pose(r, cz, pitch, close_dir)
            except Failed as exc:
                if i + 1 < len(tries):
                    self.n.say(f"ลองที่ {i + 1}: {exc} — ข้าม")
                    continue
                raise
            self.n.progress(2, 4, "grasp", f"ลองที่ {i + 1}/{len(tries)}: ลงจับผนังหน้าถุง (ทิศ "
                            f"{math.degrees(self.theta):+.0f}°, ปลายก้าม {r * 100:.1f} ซม., หมุนก้าม {roll:+.0f}°)")
            self.n.get_logger().info(f"camera try {i + 1}: tips {r:.3f} m out, pitch {pitch}, roll {roll:+.1f} deg "
                                     f"(closing axis vs wall normal {align:.2f})")
            self.n.gripper(0.9, "อ้าก้าม")
            try:
                self.above(r, cz, pitch, roll)
                time.sleep(0.3)
                before = self.xtion()
                for dz in (0.08, 0.06, 0.04, 0.02, 0.0):
                    self.tip(r, cz + dz, pitch, roll, "ลงไปที่ถุง")
            except Failed as exc:
                if exc.code != "UNREACHABLE":
                    raise
                raise Failed(f"ถุงอยู่ไกลหรือใกล้เกินเอื้อม ({exc}): ขยับหุ่น", "UNREACHABLE")
            report = self.n.grasp_report(f"camera try {i + 1} at the close", want)
            if hover:
                self.n.hold_still = "ทดสอบ bag_hover_only: ค้างไว้ให้วัด"
                raise Failed(f"bag_hover_only: หยุดค้าง อ้าก้ามไว้ที่ระดับหนีบ ({report})", "STOPPED")
            res = self.n.gripper(0.0, "หนีบ")
            closed += 1
            self.n.grip_reading = (float(res.position), float(res.effort))
            self.n.get_logger().info(f"camera grip {i + 1}: position {res.position:.3f} rad, "
                                     f"load {res.effort:.1f} %, stalled {res.stalled}")
            self.n.progress(3, 4, "lift", "ยกตรงขึ้นช้าๆ แล้วเช็กว่าก้นถุงพ้นพื้น...")
            # straight up at the same reach and tilt, slowly: on 2026-11-18 (video) the paper wall slipped out
            # when the old raise swung out to HOLD_R and tilted the wrist right after a 10 cm lift
            lifted_to = 0.0
            for dz in STRAIGHT_UP:
                try:
                    self.tip(r, cz + dz, pitch, roll, "ยกถุงตรงขึ้น", pull_in=0.03, speed=LIFT_SPEED)
                    lifted_to = dz
                except Failed:
                    break
            time.sleep(0.5)
            # the BOTTOM must be off the floor: the Xtion (under the deck) sees the floor where the bag stood.
            # The front camera only sees the top edge, which also goes up when the jaws just stretch the wall
            # up while the bag stays on the floor (video, 2026-11-18: 5 false "lifted" of 37-82 px), so it is
            # not used for this check any more.
            after = self.xtion()
            share, cols = bv.lifted(before, after) if before is not None and after is not None else (0.0, 0)
            self.n.get_logger().info(f"xtion lift check {i + 1}: lifted {lifted_to * 100:.0f} cm, floor under "
                                     f"the bag {share:.2f} ({cols} columns)")
            if cols >= 8 and share <= 0.25:
                why = "ก้นถุงยังอยู่บนพื้น (Xtion)"
                self.n.say(f"ลองที่ {i + 1}: ก้นถุงไม่พ้นพื้น — อ้าก้าม ดูกล้องใหม่แล้วลองอีกครั้ง")
                self.n.gripper(0.9, "อ้าก้าม")
                self.tip(r - 0.03, cz + lifted_to + 0.04, pitch, roll, "ถอยขึ้น", pull_in=0.05)
                continue
            verified = cols >= 8
            if not verified:
                self.n.say("Xtion ยืนยันไม่ได้ว่าก้นถุงพ้นพื้น: ยกต่อ แล้วเช็กด้วยกล้องหน้าตอนยกสูง")
            held = {"mode": "camera", "camera": True, "theta": self.theta, "roll": roll, "pitch": pitch, "r": r,
                    "hold_r": r, "verified": verified, "close_z": cz, "object": (obj.x, obj.y, obj.yaw, obj.size)}
            self.n.holding = held                # from here on the arm never folds or forgets it by itself
            self.n.progress(4, 4, "raise", "ยกถุงขึ้นสูง...")
            top = cz + lifted_to
            for pt, tz in UP:                    # tilt up in small steps at the SAME reach (no swing out)
                if tz <= top + 1e-6:
                    continue
                try:
                    self.tip(r, tz, pt, roll, "ยกถุงขึ้นสูง", pull_in=0.06, speed=HELD_SPEED)
                    top, held["hold"] = tz, (pt, tz)
                except Failed:
                    self.n.say(f"ยกได้สูงสุดแค่ปลายก้าม {top - self.n.table_z:.2f} ม. จากพื้นที่ทิศนี้")
                    break
            # held up: the bag must have left its place on the floor (ALWAYS checked: a slip in the raise)
            time.sleep(0.5)
            gone = bv.fixed_lifted(start, self.seen(), min_up=20.0)
            self.n.get_logger().info(f"front camera hold check {i + 1}: {gone}")
            if gone["lifted"] is False:
                self.n.holding = None
                why = "ยกขึ้นแล้วถุงหลุด (ยังอยู่ที่เดิมบนพื้น)"
                self.n.say(f"ลองที่ {i + 1}: ถุงยังอยู่ที่เดิมบนพื้น (หลุด) — อ้าก้าม ดูกล้องใหม่แล้วลองอีกครั้ง")
                self.n.gripper(0.9, "อ้าก้าม")
                continue
            verified = held["verified"] = verified or bool(gone["lifted"])
            note = "" if verified else " (ยังไม่ได้ยืนยันด้วยกล้อง)"
            return f"ถือถุงอยู่ด้านหน้า ปลายก้ามสูงจากพื้น {top - self.n.table_z:.2f} ม. (ลองที่ {i + 1}){note}"
        raise Failed(f"จับถุงไม่ได้ใน {len(tries)} ครั้ง (หนีบ {closed} ครั้ง; ล่าสุด: {why})", "MISSED")

    # ------------------------------------------------------------------ put down
    def place(self, target):
        """Put the held bag down at target (Box, base frame x, y) and let go."""
        held = self.n.holding
        self.theta, roll = held["theta"], held["roll"]
        pt, tz = held.get("hold", (0, 0.14))
        f, l = self.kin.to_user(np.array([target.x, target.y]))
        new = math.atan2(l, f)
        # the jaws hold the FRONT wall: tips half a bag before the target, so the bag's centre lands on it (the
        # tips at the target put the bag 10-17 cm too far, 2026-11-18); within the reach of the hold pose
        r = max(0.26, min(0.35, math.hypot(f, l) - float(self.p("bag_cam_half_depth"))))
        # let go PUT_DROP above the grasp height: the bag slips a little in the jaws while held, so at the grasp
        # height its bottom is already on the floor and the arm pressed it down and forward
        put_z = held.get("close_z", self.n.table_z + float(self.p("bag_cam_close_h"))) + PUT_DROP
        self.n.progress(1, 3, "turn", "หันไปทางจุดวาง...")
        steps = max(1, int(math.ceil(abs(new - self.theta) / math.radians(10))))
        start = self.theta
        for k in range(1, steps + 1):            # turn at the hold height, a little at a time (the bag swings)
            self.theta = start + (new - start) * k / steps
            self.tip(held.get("hold_r", HOLD_R), tz, pt, roll, "หันถุงไปทางจุดวาง", pull_in=0.06, speed=PUT_SPEED)
        self.n.progress(2, 3, "lower", "วางลง...")
        for p2, z2 in DOWN:
            if z2 < tz or p2 < pt:
                self.tip(held.get("hold_r", HOLD_R) if p2 >= 0 else r, z2, p2, roll, "วางถุงลง", pull_in=0.06,
                         speed=PUT_SPEED)
        self.tip(r, put_z, -60, roll, "วางถุงลงพื้น", pull_in=0.05, speed=HELD_SPEED)
        self.n.gripper(0.9, "ปล่อย")
        self.n.holding = self.n.grasped = None
        self.n.progress(3, 3, "retreat", "ถอยออกแล้วกลับท่า ready...")
        self.tip(r - 0.03, put_z + 0.13, -60, roll, "ถอยขึ้น", pull_in=0.05)
        return "วางถุงแล้ว"
