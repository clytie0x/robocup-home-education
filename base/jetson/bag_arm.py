"""Help Me Carry, arm side: pick the pointed bag from the floor and put it down at the start.

The grasp itself is the arm team's: /arm/pick_bag and /arm/place (~/so100_ws/src/API.md), so their
collision model of the robot body and their carry pose are used, and their nodes are never changed.
Ours is what they need as input:
  - where: the bag's x, y from the Pi (robot frame, LiDAR after the approach), refined by the gripper
    camera when it is calibrated (floor_pick.py --calibrate-self);
  - how high its top is: triangulated from 3 gripper camera views (gripper_camera.triangulate), else
    DEFAULT_TOP. Too low is safe (the jaws close lower on the handles or the gathered mouth),
    too high closes on air (MISSED), so the fallback is just above the LiDAR plane the bag crossed.
Before anything moves, /arm/home (an API task) restarts the arm team's idle timer: their auto rest
fires 120 s after their last task and would replace our camera moves (seen 2026-11-15 04:15).
The top goes to them in the arm frame of their floor: z = ARM_FLOOR_Z + top (their floor_z for mount:=robot
is the measured -0.331 itself, so no offset of ours is added); x, y go through Mount.to_arm.

/robot/command (JSON {"action", "params"}):
  pick_bag {"x", "y"[, "top"]}   robot frame, metres -> grab_success | grab_failed|CODE|message
                                 (with their grasp reading: grab_success|SUCCESS|gripper_pos=R gripper_load=L,
                                 grab_failed|CODE|message [gripper_pos=R gripper_load=L])
  drop_bag {"x", "y"}            floor point, robot frame -> drop_done | drop_failed|CODE|message
  stop                           while a job runs: /arm/stop (the arm holds where it is)
Events go out on /robot/event, the names hmc_flow.py reads.

  source /opt/ros/jazzy/setup.bash; source ~/so100_ws/install/setup.bash; export ROS_DOMAIN_ID=42
  python3 bag_arm.py                        # the node
  python3 bag_arm.py --pick 0.31 0.0        # one pick from the shell, then exit
  python3 bag_arm.py --place 0.31 0.0
"""
import argparse
import json
import math
import sys
import threading
import time

import numpy as np

import gripper_camera as gc
from floor_pick import JAW_OPEN, READY, Failure, FloorPick, Planner, Result
from so100_ik import Kinematics, Mount, Unreachable, load_limits

DEFAULT_TOP = 0.26        # m above the floor: just above the LiDAR plane (0.24 to 0.25) the bag crossed
ARM_FLOOR_Z = -0.331      # floor in the arm frame = their launch floor_z for mount:=robot (arm.launch.py,
                          # base_z 0.305 + z_offset 0.026, base/docs/floor-pick.md Frames)
TOP_RANGE = (0.15, 0.50)  # a triangulated top outside this is a wrong match
MAX_SPREAD = 0.02         # m, RMS distance of the triangulated point from the view rays
MAX_SHIFT = 0.08          # m, the camera may move the grasp point this far from the LiDAR's bag centre
LOOK_CLEAR = 0.08         # m, the gripper stays this far above the assumed top while it looks
HOME_TIMEOUT = 60.0
PICK_TIMEOUT = 150.0      # their pick: up to 3 tries, slow carry (bag_speed 0.08)
PLACE_TIMEOUT = 120.0


def look_poses(plan, kin, target, count=3):
    """Up to `count` poses above and behind `target` (robot frame x, y, z) with the tool axis within
    22 deg of it, spread in height, reach and pitch for a stereo baseline (the tool always looks out
    along the pan direction, so poses beside the target fail the 22 deg test). The grasp point stays
    LOOK_CLEAR above the target."""
    t = np.asarray(target, float)
    cands = []
    for dx in (-0.12, -0.08, -0.04):
        for dy in (-0.07, 0.0, 0.07):
            for grip_z in (t[2] + 0.10, t[2] + 0.14, t[2] + 0.18):
                for pitch_d in (-75, -60, -45):
                    g = np.array([t[0] + dx, t[1] + dy, grip_z])
                    try:
                        q = kin.ik(g, math.radians(pitch_d), jaw=JAW_OPEN)
                        plan.check(q, JAW_OPEN, False)
                    except (Unreachable, Failure):
                        continue
                    if kin.grasp_point(q)[2] < t[2] + LOOK_CLEAR:
                        continue
                    v = t - g
                    if float(kin.tool_dir(q) @ v / np.linalg.norm(v)) > math.cos(math.radians(22)):
                        cands.append(((dy / 0.035, grip_z / 0.04, pitch_d / 15.0, dx / 0.04), q))
    return Planner._spread(cands, count)


class BagArm:
    """Pick and place jobs over a transport with arm_task / arm_state / moving / stop (floor_pick.RosArm)."""

    def __init__(self, kin, transport, camera=None, detector=None, log=print, event=None):
        self.k, self.t = kin, transport
        self.fp = FloorPick(kin, transport, Planner(kin), log=log)
        self.camera, self.detector = camera, detector or gc.BagHandleDetector()
        self.log = log
        self.event = event or getattr(transport, "event", lambda _e: None)
        self.reading = {}          # the last API result's grasp reading (gripper_pos rad, gripper_load %)

    def _ready(self):
        st = self.t.arm_state()
        if st is None:
            raise Failure("FAILED", "no /arm/state from the arm team's nodes")
        if st.get("holding"):
            raise Failure("HOLDING", "the arm already holds something")
        if st.get("state") == "busy":
            raise Failure("BUSY", "the arm team's node is busy (%s)" % (st.get("task") or "?"))
        moving = getattr(self.t, "moving", None)
        if moving is not None and moving():
            raise Failure("BUSY", "someone else is moving the arm")

    def _task(self, name, timeout, **goal):
        res = self.t.arm_task(name, timeout, **goal)
        ok, code, msg = res[:3]
        self.reading = res[3] if len(res) > 3 else {}     # their grasp reading (pick_bag), if they send one
        if not ok:
            raise Failure(code or "FAILED", "/arm/%s: %s" % (name, msg))
        return msg

    def measure_top(self, x, y):
        """Triangulated bag point (robot frame) seen by the detector from look_poses, or None (logged)."""
        if self.camera is None:
            self.log("no gripper camera calibration: top not measured")
            return None
        target = np.array([x, y, DEFAULT_TOP + 0.05])
        poses = look_poses(self.fp.plan, self.k, target)
        if len(poses) < 2:
            self.log("fewer than 2 look poses reach the bag at (%.2f, %.2f)" % (x, y))
            return None
        views = []
        for i, q in enumerate(poses):
            self.fp._goto("look at the bag %d" % i, q, JAW_OPEN)
            self.t.sleep(0.6)
            det = self.detector.detect(self.t.snapshot())
            if det is None:
                self.log("bag not seen from look %d" % i)
                continue
            views.append((self.fp._current(), (det.u, det.v)))
        if len(views) < 2:
            self.log("bag seen from %d look(s): top not measured" % len(views))
            return None
        p, spread = gc.triangulate(self.k, self.camera, views)
        self.log("bag point (%.3f, %.3f, %.3f), spread %.3f m from %d views" % (p[0], p[1], p[2], spread, len(views)))
        if spread > MAX_SPREAD or not TOP_RANGE[0] <= p[2] <= TOP_RANGE[1] \
                or math.hypot(p[0] - x, p[1] - y) > MAX_SHIFT:
            self.log("bag point rejected (spread, height or shift out of range)")
            return None
        return p

    def _reading(self):
        """'gripper_pos=R gripper_load=L' from the last pick_bag result, '' without a finite reading."""
        pos, load = self.reading.get("gripper_pos"), self.reading.get("gripper_load")
        if pos is None or load is None or not (math.isfinite(pos) and math.isfinite(load)):
            return ""
        return "gripper_pos=%.3f gripper_load=%.1f" % (pos, load)

    def pick(self, x, y, top=None):
        """Pick the bag at robot (x, y). Sends grab_success or grab_failed|CODE|message (with their grasp
        reading when they send one, for the flow log)."""
        self.reading = {}
        try:
            self._ready()
            self._task("home", HOME_TIMEOUT)
            where = np.array([x, y], float)
            if top is None:
                p = self.measure_top(x, y)
                if p is not None:
                    where, top = p[:2], float(p[2])
                    self.fp._goto("back to ready", READY, JAW_OPEN)
                else:
                    top = DEFAULT_TOP
            grasp = np.array([*self.k.mount.to_arm([where[0], where[1], 0.0])[:2], ARM_FLOOR_Z + top])
            self.log("pick_bag at robot (%.3f, %.3f), top %.3f m -> arm (%.3f, %.3f, %.3f)"
                     % (where[0], where[1], top, grasp[0], grasp[1], grasp[2]))
            msg = self._task("pick_bag", PICK_TIMEOUT, grasp=tuple(grasp), yaw=None)
        except Failure as exc:
            self.log("FAILED %s: %s" % (exc.code, exc.message))
            note = self._reading()
            self.event("grab_failed|%s|%s%s" % (exc.code, exc.message.replace("|", "/"),
                                                " [%s]" % note if note else ""))
            return Result(False, exc.code, exc.message)
        note = self._reading()
        self.event("grab_success|SUCCESS|%s" % note if note else "grab_success")
        return Result(True, "SUCCESS", msg)

    def place(self, x, y):
        """Put the held bag down at robot floor point (x, y). Sends drop_done or drop_failed|CODE|message."""
        try:
            target = self.k.mount.to_arm([x, y, 0.0])
            self.log("place at robot (%.3f, %.3f) -> arm (%.3f, %.3f)" % (x, y, target[0], target[1]))
            msg = self._task("place", PLACE_TIMEOUT, target=(target[0], target[1]))
        except Failure as exc:
            self.log("FAILED %s: %s" % (exc.code, exc.message))
            self.event("drop_failed|%s|%s" % (exc.code, exc.message.replace("|", "/")))
            return Result(False, exc.code, exc.message)
        self.event("drop_done")
        return Result(True, "SUCCESS", msg)


class Jobs:
    """/robot/command -> one BagArm job at a time on a thread (pure: tests drive it without ROS)."""

    def __init__(self, bag_arm, log=print):
        self.arm, self.log = bag_arm, log
        self.thread = None

    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    def on_command(self, text):
        try:
            cmd = json.loads(text)
        except ValueError:
            return
        action, p = cmd.get("action"), cmd.get("params") or {}
        if action == "stop":
            if self.busy():
                self.log("stop: /arm/stop")
                self.arm.t.stop()
            return
        if action not in ("pick_bag", "drop_bag"):
            return
        fail = "grab_failed" if action == "pick_bag" else "drop_failed"
        try:
            x, y = float(p["x"]), float(p["y"])
            top = float(p["top"]) if p.get("top") is not None else None
        except (KeyError, TypeError, ValueError):
            self.arm.event("%s|FAILED|%s needs x and y" % (fail, action))
            return
        if not (math.isfinite(x) and math.isfinite(y)):
            self.arm.event("%s|FAILED|%s needs finite x and y" % (fail, action))
            return
        if self.busy():
            self.arm.event("%s|BUSY|another arm job is running" % fail)
            return
        job = (lambda: self.arm.pick(x, y, top)) if action == "pick_bag" else (lambda: self.arm.place(x, y))
        self.thread = threading.Thread(target=job, daemon=True)
        self.thread.start()


def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--pick", nargs=2, type=float, metavar=("X", "Y"))
    ap.add_argument("--place", nargs=2, type=float, metavar=("X", "Y"))
    ap.add_argument("--top", type=float, help="pick: the bag top above the floor (m), skip the camera")
    ap.add_argument("--camera-file", default=gc.DEFAULT_FILE)
    ap.add_argument("--snap-url", default="http://127.0.0.1:8082/snap.jpg")
    return ap.parse_args(argv)


def main(argv=None):
    a = parse(argv)
    from floor_pick import RosArm  # noqa: PLC0415 (ROS only here)
    kin = Kinematics(Mount(), load_limits())
    try:
        camera = gc.CameraModel.load(a.camera_file)
    except (OSError, ValueError, KeyError):
        camera = None
        print("no gripper camera calibration at %s: the bag top is not measured" % a.camera_file)
    t = RosArm(a.snap_url)
    arm = BagArm(kin, t, camera)
    try:
        if a.pick or a.place:
            res = arm.pick(*a.pick, a.top) if a.pick else arm.place(*a.place)
            print("result: %s %s %s" % ("OK" if res.ok else "FAILED", res.code, res.message))
            return 0 if res.ok else 1
        from std_msgs.msg import String  # noqa: PLC0415
        jobs = Jobs(arm)
        t.node.create_subscription(String, "/robot/command", lambda m: jobs.on_command(m.data), 10)
        print("bag_arm: waiting for pick_bag / drop_bag on /robot/command")
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        return 130
    finally:
        t.close()


if __name__ == "__main__":
    sys.exit(main())
