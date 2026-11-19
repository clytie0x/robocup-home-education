"""Automatic bag pick with the forearm webcam + the Xtion (test script, not part of the arm package).

python3 bagpick.py [--putback] [--hold S]
  1. look for the bag from the viewing pose at three pan angles, centre it in the image
  2. read the ridge direction from the bag's top edge -> jaw roll (0 / 90 / +-45)
  3. pitched -60 deg, open jaws come down over the ridge in 2 cm steps, close near the top
  4. lift 10 cm, check in the Xtion that the floor shows under the bag; if not: put down, one retry
  5. tilt up and raise the bag high (front clear for the lidar); --putback brings it back down afterwards
Run on the Jetson with the arm stack up (domain 42, mount:=robot). Writes bp_*.jpg for each step.
"""
import json
import math
import os
import sys
import time
import urllib.request

import cv2
import numpy as np
import rclpy

import bagvision as bv
import gcam
from so100_driver.console_gate import require as console_gate
from so100_driver.ik import ARM, GraspPlanner, Kinematics, deck_box

XTION = os.environ.get("BAG_XTION_URL", "http://10.0.0.1:8081/stream")
VIEW = (0.18, 0.09, -50)            # viewing pose: fwd, z, pitch (left varies)
REACH = {"across": 0.275, "away": 0.27, "diag": 0.27}   # gripper origin fwd for the grasp (pitch -60)
DOWN = [0.11, 0.09, 0.07, 0.05, 0.03]                    # origin z steps; close at the last
RAISE = [(-0.025, 0.095, -45), (-0.042, 0.113, -30), (-0.052, 0.127, -15), (-0.056, 0.140, 0),
         (-0.056, 0.20, 0), (-0.056, 0.26, 0), (-0.056, 0.32, 0)]  # (fwd offset from REACH, z, pitch)
PX_PER_M = 4700.0                   # image shift per metre of viewing-pose 'left' (measured)
# one moving jaw: the open gap (7.5 cm) lies to one side of the tool axis; its middle in gripper-frame x
GAP_MID = 0.008 - (0.016 + 0.066 * 0.9) / 2


def web():
    for _ in range(5):
        try:
            d = urllib.request.urlopen(gcam.WEB, timeout=5).read()
            return cv2.imdecode(np.frombuffer(d, np.uint8), cv2.IMREAD_COLOR)
        except Exception:  # noqa: BLE001
            time.sleep(0.3)
    raise RuntimeError("no webcam frame")


def xtion():
    r = urllib.request.urlopen(XTION, timeout=8)
    d, n = b"", 0
    while True:
        d += r.read(4096)
        a = d.find(b"\xff\xd8")
        b = d.find(b"\xff\xd9", a + 2) if a >= 0 else -1
        if b >= 0:
            n += 1
            if n == 2:
                return cv2.imdecode(np.frombuffer(d[a:b + 2], np.uint8), cv2.IMREAD_COLOR)
            d = d[b + 2:]


def main():
    console_gate("bagpick")
    putback = "--putback" in sys.argv
    hold = float(sys.argv[sys.argv.index("--hold") + 1]) if "--hold" in sys.argv else 2.0
    rclpy.init()
    n = gcam.G()
    if not n.spin_until(lambda: n.urdf and all(j in n.js for j in ARM), 10):
        sys.exit("no /robot_description or /joint_states")
    from ament_index_python.packages import get_package_share_directory
    kin = Kinematics(n.urdf, None, get_package_share_directory("so100_moveit_config") + "/meshes")
    pl = GraspPlanner(kin, gcam.FLOOR, [deck_box(kin, *gcam.DECK, gcam.DECK_Z, gcam.FLOOR)])
    k = [0]

    def snap(im, label):
        cv2.imwrite(f"bp_{k[0]:02d}.jpg", im)
        k[0] += 1
        print(label, flush=True)

    def goto(f, l, z, pitch, roll, xoff=0.0, speed=1.0):
        cur = n.q_now()
        x, y = kin.from_user(f, l)
        target = np.array([x, y, z])
        if xoff:            # put gripper-frame x = xoff (between the jaws) on the target, not the tool axis
            for q0 in kin.solve(target, math.radians(pitch), depth=0.0)[:1]:
                q0.pop("_branch", None)
                q0["wrist_roll"] = math.radians(roll)
                target = target - xoff * kin.frames(q0)["gripper"][:3, 0]
        for s in kin.solve(target, math.radians(pitch), depth=0.0):
            s.pop("_branch", None)
            s["wrist_roll"] = math.radians(roll)
            steps = max(2, int(max(abs(s[j] - cur[j]) for j in ARM) / 0.03))
            path = [{j: cur[j] + (s[j] - cur[j]) * t for j in ARM} for t in np.linspace(0, 1, steps)]
            if pl.hits_body(s) or any(pl.hits_body(p) for p in path):
                continue
            n.move(s, max(1.0, max(abs(s[j] - cur[j]) for j in ARM) / (gcam.SPEED * speed)))
            time.sleep(0.5)
            return
        raise RuntimeError(f"cannot reach fwd {f:.3f} left {l:.3f} z {z:.3f} pitch {pitch} roll {roll}")

    def centre(L, b, im):
        for it in range(6):
            w = im.shape[1]
            mid = (b["x0"] + b["x1"]) / 2
            err_px = mid - b["grip_x"]
            if b["x0"] <= 3 or b["x1"] >= w - 4:          # runs off the frame: its middle is further out
                side = 1 if b["x1"] >= w - 4 and b["x0"] > 3 else (-1 if b["x0"] <= 3 and b["x1"] < w - 4 else 0)
                err_px = side * max(abs(err_px), 120) if side else err_px
            err = err_px / PX_PER_M
            if abs(err) < 0.01 or (b["x0"] <= 3 and b["x1"] >= w - 4):
                break                                 # centred, or so wide that any spot on the ridge will do
            L = max(-0.20, min(0.20, L - max(-0.04, min(0.04, 0.5 * err))))
            goto(VIEW[0], L, VIEW[1], VIEW[2], 0)
            im = web()
            b = bv.find_bag(im)
            if b is None:
                return None
            snap(bv.draw(im, b, f"centre left {L:+.3f}"), f"centre {it}: left {L:+.3f} off {err * 100:+.1f} cm")
        return L, b, im

    def locate(near=None):
        """Find the bag from the viewing pose (first where it was last seen), centre it, read its ridge."""
        found = None
        for L in ([near] if near is not None else []) + [0.0, 0.10, -0.10]:
            goto(VIEW[0], L, VIEW[1], VIEW[2], 0)
            im = web()
            b = bv.find_bag(im)
            snap(bv.draw(im, b, f"scan left {L:+.2f}"), f"scan left {L:+.2f}: " + (f'bag area {b["area"]}' if b else "no bag"))
            if b and (found is None or abs((b["x0"] + b["x1"]) / 2 - b["grip_x"])
                      < abs((found[1]["x0"] + found[1]["x1"]) / 2 - found[1]["grip_x"])):
                found = (L, b, im)
            if found and near is not None:
                break                                # seen again where it was: no need to sweep
        if found is None:
            return None
        c = centre(*found)
        if c is None:
            return None
        L, b, im = c
        kind, roll, ang = bv.ridge(b)
        snap(bv.draw(im, b, f"{kind} roll {roll} edge {ang:+.0f}"), f"ridge: {kind} (edge {ang:+.0f} deg) -> jaw roll {roll}")
        return L, kind, roll

    # 1) find the bag
    loc = locate()
    if loc is None:
        sys.exit("RESULT no bag in view")
    L, kind, roll = loc
    theta = 0.0

    # 2) grasp, lift 10 cm, check (fingertip coordinates from here on)
    def tip(r, tz, p, rl, pull_in=0.0, speed=1.0):
        """Fingertips r m from the pan axis along the bag's bearing, tz high; if out of reach, up to
        pull_in m closer to the robot (same bearing)."""
        c, s_ = math.cos(math.radians(p)), math.sin(math.radians(p))
        for d in np.arange(0.0, pull_in + 1e-6, 0.01):
            ro = r - d - 0.106 * c                # gripper origin, radially
            try:
                goto(ro * math.cos(theta), ro * math.sin(theta), tz - 0.106 * s_, p, rl, xoff=GAP_MID, speed=speed)
                return
            except RuntimeError:
                if d + 0.01 > pull_in + 1e-6:
                    raise

    TZ = -0.062                              # fingertip height at the close (~27 cm above the floor)
    attempts = ((0.007, -60), (0.022, -60), (-0.008, -60), (0.037, -50))   # best first: tips at ~33 cm
    held, unverified = False, False
    for attempt, (dt, pitch) in enumerate(attempts):
        if attempt:                          # the bag may have moved: look again before the next try
            loc = locate(near=L)
            if loc is None:
                sys.exit("RESULT lost the bag after a try (knocked over or pushed away?)")
            L, kind, roll = loc
        view_left = L
        theta = math.atan2(L, VIEW[0])       # bearing the camera found the bag on (0 = straight ahead)
        xim0 = xtion()
        cv2.imwrite(f"bp_xtion_before{attempt + 1}.jpg", xim0)
        tf = REACH[kind] + 0.053 + dt
        print(f"try {attempt + 1}: bearing {math.degrees(theta):+.1f} deg, {kind}, jaw roll {roll}, tips at {tf:.3f}", flush=True)
        n.gripper(0.9)
        try:
            tip(tf - 0.04, TZ + 0.10, pitch, roll, pull_in=0.04)
            for dz in (0.08, 0.06, 0.04, 0.02, 0.0):
                tip(tf, TZ + dz, pitch, roll, pull_in=0.02)
        except RuntimeError as exc:
            print(f"attempt {attempt + 1}: {exc}", flush=True)
            goto(VIEW[0], view_left, VIEW[1], VIEW[2], 0)
            sys.exit("RESULT too far: the bag is beyond reach of the arm, move the robot closer")
        r = n.gripper(0.0)
        print(f"grip {attempt + 1}: position {r.position:.3f} effort {r.effort:.1f} stalled {r.stalled}", flush=True)
        for dz in (0.02, 0.04, 0.06, 0.08, 0.10):
            tip(tf, TZ + dz, pitch, roll, pull_in=0.05, speed=0.5)
        xim = xtion()
        cv2.imwrite(f"bp_xtion_lift{attempt + 1}.jpg", xim)
        share, cols = bv.lifted(xim0, xim)
        print(f"lift check {attempt + 1}: floor under the bag {share:.2f} ({cols} columns)", flush=True)
        if cols < 8:
            print("the Xtion cannot see the bag bottom: carrying on without the lift check", flush=True)
            held, unverified = True, True
            break
        if share > 0.25:
            held = True
            break
        n.gripper(0.9)                       # nothing in the jaws: open right here, no put-down
        tip(tf - 0.03, TZ + 0.14, pitch, roll, pull_in=0.05)
    if not held:
        goto(VIEW[0], view_left, VIEW[1], VIEW[2], 0)
        sys.exit(f"RESULT missed {len(attempts)} times (bag not off the floor): it may be too far, move the robot closer")

    # 3) raise high: tilt up while rising, fingertips pulled in to 32 cm. Far to the side the arm cannot reach
    #    the top heights: raise as far as it goes there, swing the bag in front of the robot, then go on.
    up = [(-52, 0.0), (-45, 0.02), (-38, 0.04), (-30, 0.06), (-22, 0.08), (-15, 0.10), (-8, 0.12), (0, 0.14),
          (0, 0.17), (0, 0.20), (0, 0.23), (0, 0.26), (0, 0.29), (0, 0.32)]   # small steps: the pinch must not twist
    tip(min(tf, 0.33), TZ + 0.10, pitch, roll, pull_in=0.05)
    path = []
    for p, tz in up:
        try:
            tip(0.32, tz, p, roll, pull_in=0.06, speed=0.5)
            path.append((p, tz))
        except RuntimeError:
            print(f"cannot raise further than {path[-1][1] + 0.331 if path else 0:.2f} m tip height here", flush=True)
            break
    time.sleep(hold)
    xim = xtion()
    cv2.imwrite("bp_xtion_high.jpg", xim)
    if not unverified:
        share_h, _ = bv.lifted(xim0, xim)
        print(f"high check: floor under the bag {share_h:.2f}" + ("  -> DROPPED while raising" if share_h < 0.25 else ""), flush=True)
    top = path[-1][1] + 0.331 if path else 0
    print("RESULT holding the bag" + (" (lift NOT verified by the Xtion)" if unverified else ""),
          json.dumps({"left": round(L, 3), "tips": round(tf, 3), "ridge": kind, "roll": roll, "pitch": pitch,
                      "tip_height_m": round(top, 2)}), flush=True)
    if putback:
        for p, tz in reversed(path[:-1]):
            tip(0.32, tz, p, roll, pull_in=0.06)
        tip(tf, TZ + 0.10, pitch, roll, pull_in=0.05)
        for dz in (0.08, 0.06, 0.04, 0.02, 0.01):
            tip(tf, TZ + dz, pitch, roll, pull_in=0.05)
        n.gripper(0.9)
        tip(tf - 0.02, TZ + 0.10, pitch, roll, pull_in=0.05)
        goto(VIEW[0], view_left, VIEW[1], VIEW[2], 0)
        snap(web(), "put back")


if __name__ == "__main__":
    main()
