"""Camera bag pick for /arm/pick_bag (bag_method = camera) and its put-down for /arm/place.

Tested by hand on the robot on 2026-11-15 (arm/tools/bagpick). The forearm webcam (webcam_view.py, port
8082) looks at the floor in front from a viewing pose; the Xtion (base Pi stream) checks the lift.

  1. viewing pose turned towards the bag (the goal's x, y; then straight ahead and +-10 cm if not seen),
     centre the bag in the image, read its top edge: level = broad side facing (jaws open front-back),
     peak = narrow side facing (jaws open left-right), slanted = diagonal (jaws turned +-45 deg)
  2. along that bearing, jaws pitched -60 deg come down in 2 cm steps and close near the top. One jaw moves,
     so the middle of the open gap (gripper x GAP_MID), not the tool axis, is aimed at the bag
  3. lift 10 cm; the bag-coloured bottom columns of the Xtion frame must now show floor. A miss: open where it
     is, look again, try further out (bag_cam_tries)
  4. tilt up in small steps while rising: the bag hangs in front, its bottom well off the floor
Put-down: turn towards the target at the hold height, come down along the same tilt steps, open, back off.
"""
import math
import time
import urllib.request

import numpy as np

from . import bag_vision as bv
from .ik import ARM
from .task_rules import Failed

VIEW = (0.18, 0.09, -50.0)        # viewing pose: gripper forward (m), height (m, base frame), pitch (deg)
TIP = 0.106                       # gripper origin -> finger tips along the tool axis (m)
GAP_MID = 0.008 - (0.016 + 0.066 * 0.9) / 2   # gripper-frame x of the middle of the fully open jaw gap
PX_PER_M = 4700.0                 # image shift per metre of viewing-pose sideways move (measured)
REACH = {"across": 0.275, "away": 0.27, "diag": 0.27}   # gripper origin forward for the grasp at pitch -60
TRIES = ((0.007, -60), (0.022, -60), (-0.008, -60), (0.037, -50))   # (extra tip reach m, pitch deg), best first
CLOSE_TZ = -0.062                 # finger tip height at the close (base frame, ~27 cm above the floor)
UP = [(-52, 0.0), (-45, 0.02), (-38, 0.04), (-30, 0.06), (-22, 0.08), (-15, 0.10), (-8, 0.12), (0, 0.14),
      (0, 0.17), (0, 0.20), (0, 0.23), (0, 0.26), (0, 0.29), (0, 0.32)]   # (pitch, tip z): raise, small tilt steps
HOLD_R = 0.32                     # finger tips from the pan axis while held up
DOWN = [(0, 0.14), (-30, 0.06), (-60, -0.02)]   # put-down: few, longer moves (the task gives it 20 s)
JOINT_STEP = 0.03                 # rad between path points
FREE_SPEED, HELD_SPEED, PUT_SPEED = 0.8, 0.4, 0.6  # rad/s of the fastest joint: empty / lifting / putting down


class CameraBag:
    def __init__(self, node):
        self.n = node
        self.kin = node.kin

    # ------------------------------------------------------------------ helpers
    def p(self, name):
        return self.n.get_parameter(name).value

    def web(self):
        url = str(self.p("bag_cam_url"))
        import cv2
        for _ in range(5):
            try:
                d = urllib.request.urlopen(url, timeout=4).read()
                im = cv2.imdecode(np.frombuffer(d, np.uint8), cv2.IMREAD_COLOR)
                if im is not None:
                    return im
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.3)
        raise Failed("ไม่มีภาพจากกล้องบนแขน (webcam_view.py ไม่ทำงาน?)", "NOT_FOUND")

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

    def solve(self, f, l, z, pitch, roll, xoff=0.0):
        """Joint pose putting gripper-frame (xoff, 0, 0) at user (f, l), base z, tool pitch/roll (deg)."""
        x, y = self.kin.from_user(f, l)
        target = np.array([x, y, z])
        if xoff:
            for q0 in self.kin.solve(target, math.radians(pitch), depth=0.0)[:1]:
                q0.pop("_branch", None)
                q0["wrist_roll"] = math.radians(roll)
                a = self.kin.frames(q0)["gripper"][:3, 0].copy()
                a[2] = 0.0          # sideways / along only: the close height stays where it was tested
                target = target - xoff * a
        cur = {j: float(self.n.joints[j]) for j in ARM}
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

    def go(self, f, l, z, pitch, roll, what, xoff=0.0, speed=FREE_SPEED):
        path = self.solve(f, l, z, pitch, roll, xoff)
        if path is None:
            raise Failed(f"{what}: แขนไปไม่ถึง (fwd {f:.3f} left {l:+.3f} z {z:.3f} pitch {pitch:.0f})", "UNREACHABLE")
        self.n.line(path, what, speed, JOINT_STEP)

    def tip(self, r, tz, pitch, roll, what, pull_in=0.0, speed=FREE_SPEED):
        """Finger tips r m out along the bag's bearing self.theta, tz high (base frame), the gap middle on the
        line; if out of reach, up to pull_in m closer (same bearing)."""
        c, s = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
        for d in np.arange(0.0, pull_in + 1e-6, 0.01):
            ro = r - d - TIP * c
            path = self.solve(ro * math.cos(self.theta), ro * math.sin(self.theta), tz - TIP * s, pitch, roll,
                              GAP_MID)
            if path is not None:
                self.n.line(path, what, speed, JOINT_STEP)
                return
        raise Failed(f"{what}: แขนไปไม่ถึง (ปลายก้าม {r:.3f} ม. สูง {tz:.3f})", "UNREACHABLE")

    # ------------------------------------------------------------------ find the bag
    def view(self, left):
        self.go(VIEW[0], left, VIEW[1], VIEW[2], 0.0, "ท่าดูถุง")
        time.sleep(0.3)                          # the webcam frame lags the move a little
        im = self.web()
        return im, bv.find_bag(im)

    def locate(self, first):
        """Viewing pose turned to `first` (m sideways), then straight ahead and +-10 cm. Returns (left, kind,
        roll) of the centred bag, or None."""
        found = None
        for left in [first] + [v for v in (0.0, 0.10, -0.10) if abs(v - first) > 0.03]:
            im, b = self.view(left)
            self.n.say(f"มองหาถุงที่ {left * 100:+.0f} ซม.: " + (f'เห็น ({b["area"]} px)' if b else "ไม่เห็น"))
            if b:
                found = (left, b, im)
                break
        if found is None:
            return None
        left, b, im = found
        for _ in range(5):                       # centre it
            w = im.shape[1]
            err_px = (b["x0"] + b["x1"]) / 2 - b["grip_x"]
            if b["x0"] <= 3 and b["x1"] >= w - 4:
                break                            # fills the view: any spot on the ridge will do
            if b["x0"] <= 3 or b["x1"] >= w - 4:
                err_px = math.copysign(max(abs(err_px), 120), 1 if b["x1"] >= w - 4 else -1)
            err = err_px / PX_PER_M
            if abs(err) < 0.01:
                break
            left = max(-0.20, min(0.20, left - max(-0.04, min(0.04, 0.5 * err))))
            im, nb = self.view(left)
            if nb is None:
                return None
            b = nb
        kind, roll, ang = bv.ridge(b)
        self.n.say(f"ถุง: {kind} (ขอบบน {ang:+.0f}°) -> หมุนก้าม {roll}°")
        return left, kind, roll

    # ------------------------------------------------------------------ pick
    def pick(self, obj):
        """obj: Box from the PickBag goal (only its x, y are used: the first look). Returns the holding dict."""
        if not self.n.joints:
            raise Failed("ไม่รู้ท่าแขน (ไม่มี joint_states)", "FAILED")
        f, l = self.kin.to_user(np.array([obj.x, obj.y]))
        first = max(-0.20, min(0.20, VIEW[0] * math.tan(math.atan2(l, f)))) if f > 0.05 else 0.0
        tries = TRIES[:max(1, int(self.p("bag_cam_tries")))]
        left = first
        for i, (dt, pitch) in enumerate(tries):
            self.n.progress(1, 4, "look", f"ลองที่ {i + 1}/{len(tries)}: มองหาถุง...")
            loc = self.locate(left)
            if loc is None:
                raise Failed("มองไม่เห็นถุงจากกล้องบนแขน", "NOT_FOUND")
            left, kind, roll = loc
            self.theta = math.atan2(left, VIEW[0])
            before = self.xtion()
            r = REACH[kind] + TIP * math.cos(math.radians(60)) + dt
            self.n.progress(2, 4, "grasp", f"ลงจับ (ทิศ {math.degrees(self.theta):+.0f}°, ปลายก้าม {r * 100:.1f} ซม.)")
            self.n.gripper(0.9, "อ้าก้าม")
            try:
                self.tip(r - 0.04, CLOSE_TZ + 0.10, pitch, roll, "ไปเหนือถุง", pull_in=0.04)
                for dz in (0.08, 0.06, 0.04, 0.02, 0.0):
                    self.tip(r, CLOSE_TZ + dz, pitch, roll, "ลงไปที่ถุง", pull_in=0.02)
            except Failed as exc:
                if exc.code != "UNREACHABLE":
                    raise
                self.go(VIEW[0], left, VIEW[1], VIEW[2], 0.0, "กลับท่าดูถุง")
                raise Failed(f"ถุงอยู่ไกลเกินเอื้อม ({exc}): ขยับหุ่นเข้าใกล้", "UNREACHABLE")
            res = self.n.gripper(0.0, "หนีบ")
            self.n.grip_reading = (float(res.position), float(res.effort))
            self.n.get_logger().info(f"camera grip {i + 1}: position {res.position:.3f} rad, "
                                     f"load {res.effort:.1f} %, stalled {res.stalled}")
            self.n.progress(3, 4, "lift", "ยกขึ้น 10 ซม. แล้วเช็กจาก Xtion...")
            for dz in (0.02, 0.04, 0.06, 0.08, 0.10):
                self.tip(r, CLOSE_TZ + dz, pitch, roll, "ยกถุงขึ้น", pull_in=0.05, speed=HELD_SPEED)
            after = self.xtion()
            if before is None or after is None:
                share, cols = 0.0, 0
            else:
                share, cols = bv.lifted(before, after)
            self.n.get_logger().info(f"camera lift check {i + 1}: floor under the bag {share:.2f} ({cols} columns)")
            if cols < 8:
                self.n.say("Xtion มองไม่เห็นก้นถุง: ยกต่อโดยไม่ได้ยืนยัน")
                verified = False
                break
            if share > 0.25:
                verified = True
                break
            self.n.say(f"ลองที่ {i + 1}: ถุงไม่ลอย (เห็นพื้นใต้ถุง {share:.2f}) — อ้าก้ามแล้วลองใหม่")
            self.n.gripper(0.9, "อ้าก้าม")
            self.tip(r - 0.03, CLOSE_TZ + 0.14, pitch, roll, "ถอยขึ้น", pull_in=0.05)
        else:
            self.go(VIEW[0], left, VIEW[1], VIEW[2], 0.0, "กลับท่าดูถุง")
            raise Failed(f"จับถุงไม่ได้ใน {len(tries)} ครั้ง (ถุงไม่ลอยจากพื้น)", "MISSED")
        held = {"mode": "camera", "camera": True, "theta": self.theta, "roll": roll, "pitch": pitch, "r": r,
                "kind": kind, "verified": verified, "object": (obj.x, obj.y, obj.yaw, obj.size)}
        self.n.holding = held                    # from here on the arm never folds or forgets it by itself
        self.n.progress(4, 4, "raise", "ยกถุงขึ้นสูง...")
        self.tip(min(r, 0.33), CLOSE_TZ + 0.10, pitch, roll, "ยกถุง", pull_in=0.05, speed=HELD_SPEED)
        top = CLOSE_TZ + 0.10
        for pt, tz in UP:
            try:
                self.tip(HOLD_R, tz, pt, roll, "ยกถุงขึ้นสูง", pull_in=0.06, speed=HELD_SPEED)
                top, held["hold"] = tz, (pt, tz)
            except Failed:
                self.n.say(f"ยกได้สูงสุดแค่ปลายก้าม {top - self.n.table_z:.2f} ม. จากพื้นที่ทิศนี้")
                break
        if verified and before is not None:
            again = self.xtion()
            if again is not None:
                share, cols = bv.lifted(before, again)
                if cols >= 8 and share < 0.25:
                    self.n.holding = None
                    raise Failed("ถุงหลุดระหว่างยกขึ้นสูง", "MISSED")
        note = "" if verified else " (ยังไม่ได้ยืนยันด้วย Xtion)"
        return f"ถือถุงอยู่ด้านหน้า ปลายก้ามสูงจากพื้น {top - self.n.table_z:.2f} ม.{note}"

    # ------------------------------------------------------------------ put down
    def place(self, target):
        """Put the held bag down at target (Box, base frame x, y) and let go."""
        held = self.n.holding
        self.theta, roll = held["theta"], held["roll"]
        pt, tz = held.get("hold", (0, 0.14))
        f, l = self.kin.to_user(np.array([target.x, target.y]))
        new = math.atan2(l, f)
        r = max(0.30, min(0.35, math.hypot(f, l)))
        self.n.progress(1, 3, "turn", "หันไปทางจุดวาง...")
        steps = max(1, int(math.ceil(abs(new - self.theta) / math.radians(10))))
        start = self.theta
        for k in range(1, steps + 1):            # turn at the hold height, a little at a time (the bag swings)
            self.theta = start + (new - start) * k / steps
            self.tip(HOLD_R, tz, pt, roll, "หันถุงไปทางจุดวาง", pull_in=0.06, speed=PUT_SPEED)
        self.n.progress(2, 3, "lower", "วางลง...")
        for p2, z2 in DOWN:
            if z2 < tz or p2 < pt:
                self.tip(HOLD_R if p2 >= 0 else r, z2, p2, roll, "วางถุงลง", pull_in=0.06, speed=PUT_SPEED)
        self.tip(r, CLOSE_TZ + 0.01, -60, roll, "วางถุงลงพื้น", pull_in=0.05, speed=HELD_SPEED)
        self.n.gripper(0.9, "ปล่อย")
        self.n.holding = self.n.grasped = None
        self.n.progress(3, 3, "retreat", "ถอยออกแล้วกลับท่า ready...")
        self.tip(r - 0.03, CLOSE_TZ + 0.14, -60, roll, "ถอยขึ้น", pull_in=0.05)
        return "วางถุงแล้ว"
