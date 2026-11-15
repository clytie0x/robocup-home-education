"""Pick the operator's bag from the floor with the SO-100 arm (RoboCup@Home "Help Me Carry").

The arm team's /arm/pick assumes objects on a table (object z ignored), so it cannot pick from the floor.
This drives their running driver through its standard ROS 2 interfaces (domain 42), while their
/arm/pick is idle, and never changes or restarts their nodes:
  /arm_controller/follow_joint_trajectory  control_msgs/FollowJointTrajectory (5 arm joints)
  /gripper_controller/gripper_cmd          control_msgs/GripperCommand (their driver squeezes with its
                                           own gripper_torque_limit, 500; our max_effort is not used)
  /arm/state (2 Hz), /arm/stop, /joint_states; events on /robot/event like their bridge:
  grab_success, grab_failed|CODE|message, drop_done, drop_failed|CODE|message (hmc_flow.py).

  python3 floor_pick.py --dry-run --pick 0.32 0.0      print the trajectories, send nothing
  python3 floor_pick.py --pick 0.32 0.0 --hover-only   go above the bag with open jaws and stop
  python3 floor_pick.py --pick 0.32 0.0 [--camera]     full pick (camera: visual servo first)
  python3 floor_pick.py --place 0.32 0.0 | --carry
  python3 floor_pick.py --calibrate --marker 0.32 0.0 --marker 0.32 0.10
Run on the Jetson: source /opt/ros/jazzy/setup.bash; source ~/so100_ws/install/setup.bash;
export ROS_DOMAIN_ID=42. Positions are robot base frame metres (x forward, y left, from the base centre).
See base/docs/floor-pick.md.
"""
import argparse
import collections
import math
import sys
import time

import numpy as np

import gripper_camera as gc
from so100_ik import (ARM, BODY_HALF, CLOSED, DECK_TOP, JAW_OPEN, LIDAR_Z, Kinematics, Mount, Unreachable,
                      load_limits)

# arm team's poses (pick_place_node.py:60-61)
READY = {"shoulder_pan": 0.0, "shoulder_lift": 1.2, "elbow_flex": -1.2, "wrist_flex": 0.3, "wrist_roll": 0.0}
REST = {"shoulder_pan": 0.0, "shoulder_lift": 0.04, "elbow_flex": -0.04, "wrist_flex": 0.0, "wrist_roll": 0.0}

SPEED_SCALE = 0.30          # of the URDF joint velocity limits (1 rad/s each, so100.urdf)
LINE_SPEED = 0.05           # m/s, cap on the grasp point's average speed on straight lines
STEP = 0.005                # m between points on straight lines
APPROACH = 0.08             # m back along the tool axis for the pre-grasp
NECK_Z = LIDAR_Z            # bag neck / handles height above the floor (earlier pick: 0.24 to 0.25 m)
PLACE_ABOVE = 0.01          # place: the grip this much above the height it was picked at
BAG_HANG = 0.27             # m from the grip down to the bottom of the lifted bag (hardware assessment)
BAG_HALF = (0.06, 0.12)     # half size of the hanging bag along robot x and y (to be measured)
GRIP_HELD = 0.3             # rad: about where the jaws stop on the bag (for planning clearance only)
START_TOLERANCE = 0.12      # rad: the arm must be this close to where a move starts
SETTLE = 0.5                # s after the lift before checking the bag is still held
OK_STATES = ("idle", "resting")

Result = collections.namedtuple("Result", "ok code message")


class Failure(Exception):
    def __init__(self, code, message, stop=False):
        super().__init__(message)
        self.code, self.message, self.stop = code, message, stop


def minjerk_time(s):
    """Time fraction at which a min-jerk move has covered the fraction s of its path."""
    lo, hi = 0.0, 1.0
    for _ in range(40):
        m = (lo + hi) / 2
        if 10 * m ** 3 - 15 * m ** 4 + 6 * m ** 5 < s:
            lo = m
        else:
            hi = m
    return (lo + hi) / 2


def qvec(q):
    return np.array([q[n] for n in ARM])


# ===================================================================================== planning
class Planner:
    def __init__(self, kin, speed=SPEED_SCALE, bag_hang=BAG_HANG, bag_half=BAG_HALF):
        self.k = kin
        self.vmax = {n: kin.velocity[n] * speed for n in ARM}
        self.bag_hang, self.bag_half = bag_hang, bag_half
        self._carry = None

    # ---------------------------------------------------------------------------- timing
    def timed(self, start, qs):
        """[(t, q)] with min-jerk timing, joint speeds <= vmax and the grasp point <= LINE_SPEED."""
        prev, prev_p = start, self.k.grasp_point(start)
        cost = []
        for q in qs:
            p = self.k.grasp_point(q)
            c = max(abs(q[n] - prev[n]) / self.vmax[n] for n in ARM)
            cost.append(max(c, float(np.linalg.norm(p - prev_p)) / LINE_SPEED))
            prev, prev_p = q, p
        total = float(sum(cost))
        if total < 1e-9:
            return [(0.2, dict(qs[-1]))] if qs else []
        dur = 1.875 * total          # min-jerk peak speed is 1.875 times the average
        out, acc = [], 0.0
        for q, c in zip(qs, cost):
            acc += c
            out.append((max(dur * minjerk_time(acc / total), 0.02), dict(q)))
        return out

    # ------------------------------------------------------------------------- clearance
    def bag_problem(self, q):
        """Why the hanging bag (below the grip) would hit the robot's top or the arm, or None."""
        g = self.k.grasp_point(q)
        hx, hy = self.bag_half
        bottom = g[2] - self.bag_hang
        over_body = abs(g[0]) - hx < BODY_HALF and abs(g[1]) - hy < BODY_HALF
        if over_body and bottom < DECK_TOP + 0.02:
            return "the bag would hit the robot's top (bag bottom %.3f m)" % bottom
        pts = self.k.body_points(q, GRIP_HELD, camera=False)[:24]     # upper_arm, lower_arm, wrist corners
        inside = ((np.abs(pts[:, 0] - g[0]) < hx) & (np.abs(pts[:, 1] - g[1]) < hy)
                  & (pts[:, 2] > bottom) & (pts[:, 2] < g[2] - 0.04))
        if inside.any():
            return "the arm would pass through the hanging bag"
        return None

    def check(self, q, jaw, bag=False):
        try:
            self.k.check_limits(q)
            self.k.check_pose(q, jaw)
        except Unreachable as exc:
            raise Failure("COLLISION" if ("floor" in str(exc) or "deck" in str(exc)) else "UNREACHABLE", str(exc))
        if bag:
            why = self.bag_problem(q)
            if why:
                raise Failure("COLLISION", why)

    def ik(self, p, pitch, jaw, roll=0.0):
        try:
            return self.k.ik(p, pitch, roll, jaw=jaw)
        except Unreachable as exc:
            raise Failure("COLLISION" if ("floor" in str(exc) or "deck" in str(exc)) else "UNREACHABLE",
                          "(%.3f, %.3f, %.3f) at %.0f deg: %s" % (p[0], p[1], p[2], math.degrees(pitch), exc))

    def line(self, a, b, pitch_a, pitch_b, jaw, roll=0.0, bag=False):
        """Straight line of the grasp point from a to b (pitch interpolated): joint poses, a excluded."""
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / STEP)))
        out, prev = [], None
        for i in range(1, n + 1):
            s = i / n
            q = self.ik(a + (b - a) * s, pitch_a + (pitch_b - pitch_a) * s, jaw, roll)
            if bag:
                self.check(q, jaw, bag=True)
            if prev is not None and max(abs(q[j] - prev[j]) for j in ARM) > 0.25:
                raise Failure("UNREACHABLE", "the arm would jump between two points of a straight line")
            out.append(q)
            prev = q
        return out

    def joint_path(self, q0, q1, jaw, bag=False, via_ready=True):
        """Joint-space path q0 -> q1, every 0.03 rad checked; through READY if the direct one hits."""
        def samples(a, b):
            d = max(abs(b[n] - a[n]) for n in ARM)
            n = max(1, int(math.ceil(d / 0.03)))
            return [{j: a[j] + (b[j] - a[j]) * i / n for j in ARM} for i in range(1, n + 1)]
        try:
            path = samples(q0, q1)
            for q in path:
                self.check(q, jaw, bag)
            return path
        except Failure as exc:
            if not via_ready:
                raise
            first = exc
        try:
            path = samples(q0, READY) + samples(READY, q1)
            for q in path:
                self.check(q, jaw, bag)
            return path
        except Failure:
            raise first

    # ----------------------------------------------------------------------------- poses
    def pre_point(self, target, q_grasp, back=APPROACH):
        return np.asarray(target, float) - back * self.k.tool_dir(q_grasp)

    def grasp_pose(self, target, pitch=None, prefer=None):
        """(q, pitch) for the grasp at target: the given pitch (deg), else `prefer` (rad) if it works,
        else the steepest that works for both the grasp and its pre-grasp."""
        pitches = [math.radians(pitch)] if pitch is not None else [math.radians(d) for d in range(-90, -19, 5)]
        if pitch is None and prefer is not None:
            pitches.insert(0, prefer)
        errors = []
        for p in pitches:
            try:
                q = self.ik(target, p, JAW_OPEN)
                self.ik(self.pre_point(target, q), p, JAW_OPEN)
                return q, p
            except Failure as exc:
                errors.append(exc)
        raise max(errors, key=lambda e: e.code == "COLLISION")

    def carry_plan(self):
        """Carry pose: grip as high as possible with the bag inside the footprint and above the deck;
        if no pose does that, the one with the bag highest (and the reason)."""
        if self._carry:
            return self._carry
        y0 = float(self.k.mount.pan_xy[1])
        best = None
        for x in np.arange(-0.06, 0.32, 0.02):
            for pitch_d in (0, -10, -20, -30, -45, -60, -90):
                for z in np.arange(0.74, 0.44, -0.02):
                    p = np.array([x, y0, z])
                    try:
                        q = self.k.ik(p, math.radians(pitch_d), jaw=GRIP_HELD)
                    except Unreachable:
                        continue
                    if self.bag_problem(q):
                        continue
                    hx, hy = self.bag_half
                    inside = abs(x) + hx <= BODY_HALF + 1e-9 and abs(y0) + hy <= BODY_HALF + 1e-9
                    bottom = z - self.bag_hang
                    clears = bottom >= LIDAR_Z + 0.02
                    key = (inside, clears, bottom)
                    if best is None or key > best[0]:
                        best = (key, q, p, math.radians(pitch_d))
                    break
        if best is None:
            raise Failure("UNREACHABLE", "no carry pose found")
        (inside, clears, bottom), q, p, pitch = best
        note = []
        note.append("bag inside the footprint" if inside else
                    "IMPOSSIBLE to keep the bag inside the footprint (bag half size %.2f x %.2f m)" % self.bag_half)
        note.append("bag bottom %.3f m above the floor, %s the LiDAR plane (%.3f m)"
                    % (bottom, "clear of" if clears else "BELOW", LIDAR_Z))
        self._carry = {"q": q, "grip": p, "pitch": pitch, "inside_footprint": inside, "bag_bottom": bottom,
                       "clears_lidar": clears, "note": "; ".join(note)}
        return self._carry

    def lift_point(self, target, carry):
        """Highest point straight above the target (at most the carry height) and its pitch."""
        for z in np.arange(carry["grip"][2], target[2] + 0.05, -0.01):
            for pd in (math.degrees(carry["pitch"]), 0, -10, -20, -30, -45, -60):
                p = np.array([target[0], target[1], z])
                try:
                    q = self.k.ik(p, math.radians(pd), jaw=GRIP_HELD)
                except Unreachable:
                    continue
                if not self.bag_problem(q):
                    return p, math.radians(pd)
        raise Failure("UNREACHABLE", "cannot lift the bag above (%.3f, %.3f)" % (target[0], target[1]))

    def pick_plan(self, target, pitch=None, prefer=None):
        """Every pose of a pick, planned before anything moves."""
        target = np.asarray(target, float)
        q_grasp, pitch = self.grasp_pose(target, pitch, prefer)
        pre = self.pre_point(target, q_grasp)
        q_pre = self.ik(pre, pitch, JAW_OPEN)
        descend = self.line(pre, target, pitch, pitch, JAW_OPEN)
        carry = self.carry_plan()
        # lift straight up, as high as the arm goes there (up to the carry height), then across to it
        up, pitch_up = self.lift_point(target, carry)
        lift = self.line(target, up, pitch, pitch_up, GRIP_HELD, bag=True)
        across = self.joint_path(lift[-1], carry["q"], GRIP_HELD, bag=True, via_ready=False)
        back_up = self.line(target, pre, pitch, pitch, JAW_OPEN)
        return {"target": target, "pitch": pitch, "pre": q_pre, "descend": descend, "lift": lift,
                "across": across, "back_up": back_up, "carry": carry}

    def place_plan(self, target, start_q):
        """From the carry pose: across above the spot, down until the bag stands, open, back off."""
        target = np.asarray(target, float)
        carry = self.carry_plan()
        q_place, pitch = self.grasp_pose(target)
        above = np.array([target[0], target[1], carry["grip"][2]])
        q_above = self.ik(above, carry["pitch"], GRIP_HELD)
        to_above = self.joint_path(start_q, q_above, GRIP_HELD, bag=True)
        lower = self.line(above, target, carry["pitch"], pitch, GRIP_HELD)
        away = self.line(target, self.pre_point(target, q_place), pitch, pitch, JAW_OPEN)
        return {"target": target, "pitch": pitch, "to_above": to_above, "lower": lower, "away": away,
                "carry": carry}

    def calibration_poses(self, marker, count=4):
        """Up to `count` reachable poses whose tool axis passes within 20 deg of the marker (the camera
        looks along the jaws), varied in pitch, height and side offset so the fit sees the marker from
        different places in the image."""
        m = np.asarray(marker, float)
        cands = []
        for dx in (-0.10, -0.07, -0.04, -0.01, 0.02):
            for dy in (-0.04, 0.0, 0.04):
                for grip_z in (0.22, 0.26, 0.30):
                    for pitch_d in (-90, -75, -60, -45):
                        g = np.array([m[0] + dx, m[1] + dy, grip_z])
                        try:
                            q = self.k.ik(g, math.radians(pitch_d), jaw=JAW_OPEN)
                        except Unreachable:
                            continue
                        v = m - g
                        cosang = float(self.k.tool_dir(q) @ v / np.linalg.norm(v))
                        if cosang > math.cos(math.radians(20)):
                            cands.append(((pitch_d / 15.0, grip_z / 0.04, dx / 0.03, dy / 0.03), q))
        return self._spread(cands, count)

    @staticmethod
    def _spread(cands, count):
        chosen = []
        while cands and len(chosen) < count:      # greedy: farthest from those already chosen
            if not chosen:
                pick = cands[len(cands) // 2]
            else:
                pick = max(cands, key=lambda c: min(math.dist(c[0], o[0]) for o in chosen))
            chosen.append(pick)
            cands.remove(pick)
        return [q for _k, q in chosen]

    def self_calibration_poses(self, marker, count=12):
        """Like calibration_poses but wider, with the wrist rolled +-0.8 rad too: with the marker's
        place unknown, only views from many heights, sides and rolls pin the camera down (one floor
        crossing, 12 poses, 0.5 px noise: camera within 4 mm, a point 0.3 m up triangulated within
        3 mm in the tests). Every pose is checked against the floor and the deck."""
        m = np.asarray(marker, float)
        cands = []
        for dx in (-0.14, -0.10, -0.06, -0.02, 0.02):
            for dy in (-0.08, -0.04, 0.0, 0.04, 0.08):
                for grip_z in (0.22, 0.28, 0.34, 0.40):
                    for pitch_d in (-90, -75, -60, -45, -30):
                        for roll in (-0.8, 0.0, 0.8):
                            g = np.array([m[0] + dx, m[1] + dy, grip_z])
                            try:
                                q = self.k.ik(g, math.radians(pitch_d), roll=roll, jaw=JAW_OPEN)
                                self.check(q, JAW_OPEN, False)
                            except (Unreachable, Failure):
                                continue
                            v = m - g
                            if float(self.k.tool_dir(q) @ v / np.linalg.norm(v)) > math.cos(math.radians(22)):
                                cands.append(((pitch_d / 15.0, grip_z / 0.04, dx / 0.03, dy / 0.03, roll / 0.3), q))
        return self._spread(cands, count)


# ==================================================================================== sequence
class FloorPick:
    """Pick/place/carry/calibrate on an injectable transport (see RosArm / DryRunArm / tests' FakeArm):
      arm_state() -> {"state", "task", ...} or None     joints() -> {joint: rad} incl. gripper, or None
      follow([(t, q)], timeout) -> (ok, code, msg)       grip(position, timeout) -> {ok, position, stalled, reached}
      stop()   event(text)   snapshot() -> BGR image or None   sleep(s)
    """

    def __init__(self, kin, transport, planner=None, log=print, grip_timeout=10.0):
        self.k = kin
        self.t = transport
        self.plan = planner or Planner(kin)
        self.log = log
        self.grip_timeout = grip_timeout
        self.moves = []          # (step, duration s) of this run

    # ------------------------------------------------------------------------------ steps
    def _check_state(self, what, holding_ok=True):
        st = self.t.arm_state()
        if st is None:
            raise Failure("FAILED", "no /arm/state from the arm team's nodes", stop=False)
        if st.get("state") == "busy":
            raise Failure("BUSY", "the arm team's node is busy (%s) before '%s'" % (st.get("task") or "?", what),
                          stop=True)
        if st.get("state") == "holding" and not holding_ok:
            raise Failure("HOLDING", "the arm team's pick holds something")
        if st.get("state") not in OK_STATES + ("holding",):
            raise Failure("FAILED", "arm state is '%s' (%s)" % (st.get("state"), st.get("fault") or ""))

    def _current(self):
        q = self.t.joints()
        if q is None or any(n not in q for n in ARM):
            raise Failure("FAILED", "no /joint_states from the arm driver")
        return q

    def _move(self, what, qs, start=None):
        if not qs:
            return
        self._check_state(what)
        now = self._current()
        start = start or now
        off = max(abs(now[n] - start[n]) for n in ARM)
        if off > START_TOLERANCE:
            raise Failure("FAILED", "%s: the arm is %.2f rad away from where the move starts" % (what, off), stop=True)
        pts = self.plan.timed(start, qs)
        dur = pts[-1][0]
        moving = getattr(self.t, "moving", None)
        if moving is not None and moving():
            raise Failure("BUSY", "%s: someone else is moving the arm" % what, stop=False)
        self.moves.append((what, dur))
        self.log("move: %s (%d points, %.1f s)" % (what, len(pts), dur))
        ok, code, msg = self.t.follow(pts, dur + 5.0)
        if not ok:
            # replaced by another program's command: that program has the arm now, do not stop it
            raise Failure(code or "FAILED", "%s: %s" % (what, msg), stop=not (code == "BUSY" and "replaced" in str(msg)))

    def _goto(self, what, q, jaw, bag=False):
        start = self._current()
        self._move(what, self.plan.joint_path(start, q, jaw, bag), start)

    def _grip(self, what, position):
        self._check_state(what)
        self.log("gripper: %s (%.2f rad)" % (what, position))
        r = self.t.grip(position, self.grip_timeout)
        if not r or not r.get("ok"):
            raise Failure("FAILED", "%s: the gripper did not answer" % what, stop=True)
        return r

    def _fail(self, exc, kind, sent_success=False):
        if exc.stop:
            self.t.stop()
            self.log("/arm/stop called")
        self.log("FAILED %s: %s" % (exc.code, exc.message))
        if not sent_success:
            self.t.event("%s_failed|%s|%s" % (kind, exc.code, exc.message.replace("|", "/")))
        return Result(False, exc.code, exc.message)

    # ------------------------------------------------------------------------------- pick
    def pick(self, x, y, z=NECK_Z, pitch=None, hover_only=False, camera=None, detector=None):
        target = np.array([x, y, z], float)
        sent = False
        self.moves = []
        try:
            self._check_state("pick", holding_ok=False)
            plan = self.plan.pick_plan(target, pitch)
            carry = plan["carry"]
            self.log("plan: pitch %.0f deg; carry: %s" % (math.degrees(plan["pitch"]), carry["note"]))
            self._goto("above the bag", plan["pre"], JAW_OPEN)
            self._grip("open", JAW_OPEN)
            if camera is not None:
                plan = self._servo(plan, camera, detector or gc.BagHandleDetector())
            if hover_only:
                self.log("hover only: stopped above (%.3f, %.3f, %.3f) with the jaws open" % tuple(plan["target"]))
                return Result(True, "HOVER", "above the bag, jaws open")
            self._move("descend", plan["descend"], plan["pre"])
            r = self._grip("close", 0.0)
            if r.get("reached") or r.get("position", 0.0) < CLOSED:
                self.log("jaws closed fully (%.3f rad): missed" % r.get("position", 0.0))
                self._grip("open", JAW_OPEN)
                self._move("back up", plan["back_up"], plan["descend"][-1])
                raise Failure("MISSED", "I missed the bag.")
            self._move("lift", plan["lift"], plan["descend"][-1])
            self.t.sleep(SETTLE)
            g = self._current().get("gripper", 0.0)
            if g < CLOSED:
                raise Failure("DROPPED", "The bag slipped out after the lift.", stop=True)
            self.t.event("grab_success")
            sent = True
            self._move("to carry", plan["across"], plan["lift"][-1])
            self.log("holding the bag in the carry pose (%s)" % carry["note"])
            return Result(True, "SUCCESS", "I have the bag.")
        except Failure as exc:
            return self._fail(exc, "grab", sent)

    def _servo(self, plan, camera, detector, tol=0.004, max_step=0.03, iterations=8):
        """Centre the grasp point over the handle seen by the gripper camera, in small steps, at the
        pre-grasp height; then return a fresh plan for the refined target."""
        target, pitch = plan["target"].copy(), plan["pitch"]
        misses, err = 0, None
        for i in range(iterations):
            self.t.sleep(0.6)            # let the arm settle and the camera stream catch up
            img = self.t.snapshot()
            det = detector.detect(img) if img is not None else None
            if det is None:
                misses += 1
                self.log("camera: no handle seen (%d)" % misses)
                if misses >= 2:
                    raise Failure("NOT_FOUND", "I cannot see the bag handles.")
                continue
            q = self._current()
            est = camera.pixel_to_plane(self.k, q, det.u, det.v, target[2])
            if est is None:
                raise Failure("NOT_FOUND", "the handle pixel does not meet the handle height plane")
            want = camera.project(self.k, q, target)
            err = est[:2] - target[:2]
            self.log("camera %d: %s, target pixel %s, handle at (%.3f, %.3f), off %.1f mm"
                     % (i, det, None if want is None else "(%.0f, %.0f)" % tuple(want), est[0], est[1],
                        1000 * float(np.linalg.norm(err))))
            if np.linalg.norm(err) < tol:
                break
            step = err * min(1.0, max_step / float(np.linalg.norm(err)))
            target[:2] += step
            q_grasp, pitch = self.plan.grasp_pose(target, prefer=pitch)
            q_pre = self.plan.ik(self.plan.pre_point(target, q_grasp), pitch, JAW_OPEN)
            self._goto("servo step %d" % i, q_pre, JAW_OPEN)
        else:
            if err is None or np.linalg.norm(err) > 0.015:
                raise Failure("NOT_CENTRED", "I could not line up with the bag handles.")
        new = self.plan.pick_plan(target, prefer=pitch)
        cur = self._current()
        if max(abs(cur[n] - new["pre"][n]) for n in ARM) > 1e-3:
            self._goto("above the handle", new["pre"], JAW_OPEN)
        return new

    # ------------------------------------------------------------------------------ place
    def place(self, x, y, z=NECK_Z + PLACE_ABOVE):
        target = np.array([x, y, z], float)
        self.moves = []
        sent = False
        try:
            self._check_state("place")
            q0 = self._current()
            if q0.get("gripper", 0.0) < CLOSED:
                raise Failure("NOT_HOLDING", "I am not holding anything.")
            plan = self.plan.place_plan(target, q0)
            self._move("above the spot", plan["to_above"], q0)
            self._move("lower", plan["lower"], plan["to_above"][-1])
            self._grip("open", JAW_OPEN)
            self._move("away", plan["away"], plan["lower"][-1])
            self.t.event("drop_done")
            sent = True
            self._grip("close", 0.0)
            return Result(True, "SUCCESS", "I have put it down.")
        except Failure as exc:
            return self._fail(exc, "drop", sent)

    def carry(self):
        self.moves = []
        try:
            self._check_state("carry")
            c = self.plan.carry_plan()
            self.log("carry: %s" % c["note"])
            q0 = self._current()
            holding = q0.get("gripper", 0.0) >= CLOSED
            self._goto("to carry", c["q"], GRIP_HELD if holding else JAW_OPEN, bag=holding)
            return Result(True, "SUCCESS", c["note"])
        except Failure as exc:
            if exc.stop:
                self.t.stop()
            self.log("FAILED %s: %s" % (exc.code, exc.message))
            return Result(False, exc.code, exc.message)

    # -------------------------------------------------------------------------- calibration
    def calibrate(self, markers, detector, prompt=input, path=gc.DEFAULT_FILE, base=None):
        """Owner puts a marker at each known robot-frame floor position; the arm looks at it from 3-4
        poses; the camera-to-gripper transform (and focal length) is fitted and saved."""
        obs = []
        try:
            self._check_state("calibrate", holding_ok=False)
            for m in markers:
                m = np.asarray(m, float)
                poses = self.plan.calibration_poses(m)
                if len(poses) < 2:
                    raise Failure("UNREACHABLE", "marker (%.2f, %.2f) is outside the arm's view poses" % (m[0], m[1]))
                prompt("Put the marker at x=%.3f y=%.3f (z=%.3f), keep hands clear, then press Enter: "
                       % tuple(m))
                for i, q in enumerate(poses):
                    self._goto("look at marker %d" % i, q, JAW_OPEN)
                    self.t.sleep(0.6)
                    det = detector.detect(self.t.snapshot())
                    if det is None:
                        self.log("marker not seen from pose %d" % i)
                        continue
                    obs.append((self._current(), m, (det.u, det.v)))
                    self.log("marker seen at (%.0f, %.0f) px" % (det.u, det.v))
            self._goto("back to ready", READY, JAW_OPEN)
            cam, rms = gc.fit_camera(self.k, obs, base)
        except Failure as exc:
            if exc.stop:
                self.t.stop()
            self.log("FAILED %s: %s" % (exc.code, exc.message))
            return None
        except ValueError as exc:
            self.log("FAILED: %s" % exc)
            return None
        cam.info = {"date": time.strftime("%Y-%m-%d %H:%M:%S"), "sightings": len(obs),
                    "mount": self.k.mount.to_dict(), "markers": [list(map(float, m)) for m in markers]}
        cam.save(path)
        self.log("camera fitted: %d sightings, rms %.2f px, f %.0f px, saved %s" % (len(obs), rms, cam.fx, path))
        return cam

    TRACK_MOVES = ({"shoulder_pan": 0.12}, {"shoulder_pan": -0.12}, {"wrist_flex": 0.15}, {"wrist_flex": -0.15},
                   {"wrist_roll": 0.5}, {"wrist_roll": -0.5}, {"shoulder_lift": -0.10, "elbow_flex": 0.10},
                   {"wrist_roll": 0.5, "shoulder_pan": 0.10}, {"wrist_roll": -0.5, "wrist_flex": 0.12})
    TRACK_GATE = 220.0   # px from the first sighting: farther is another crossing

    def _track_cross(self, detector, obs):
        """Sightings of the crossing nearest the image centre, from here and from small joint moves
        round here (no camera model needed: the crossing barely moves in the image)."""
        q0 = self._current()
        det = detector.detect(self.t.snapshot())
        if det is None:
            raise Failure("NOT_FOUND", "no floor crossing seen from the current pose")
        first = (det.u, det.v)
        obs.append((q0, "cross", first))
        for i, d in enumerate(self.TRACK_MOVES):
            q = {n: q0[n] + d.get(n, 0.0) for n in ARM}
            try:
                self.k.check_limits(q)
                self.plan.check(q, JAW_OPEN)
            except (Unreachable, Failure):
                continue
            self._goto("track the crossing %d" % i, q, JAW_OPEN)
            self.t.sleep(0.6)
            img = self.t.snapshot()
            try:
                det = detector.detect(img, near=first)
            except TypeError:
                det = detector.detect(img)
            if det is None or math.hypot(det.u - first[0], det.v - first[1]) > self.TRACK_GATE:
                self.log("track %d: crossing not seen near (%.0f, %.0f) px" % (i, first[0], first[1]))
                continue
            obs.append((self._current(), "cross", (det.u, det.v)))

    def calibrate_self(self, detector, path=gc.DEFAULT_FILE, base=None, start=None, count=12, gate_px=250.0):
        """No one in the room: a floor-tile grout crossing in front is the marker, its place fitted with
        the camera (gc.fit_camera_free). `start` is a rough floor x, y of the crossing; without it the
        crossing seen from the current pose is put on the floor with the `base` (uncalibrated) model.
        A sighting more than `gate_px` from where the rough model expects it is another crossing: skipped."""
        base = base or gc.CameraModel()
        obs = []
        try:
            self._check_state("calibrate", holding_ok=False)
            if start is None:               # coarse: small moves round here, the same crossing tracked
                self._track_cross(detector, obs)
                base, rms, where = gc.fit_camera_free(self.k, obs, base)
                start = where["cross"]
                self.log("coarse fit: %d sightings, rms %.1f px" % (len(obs), rms))
            m = np.array([start[0], start[1], 0.0])
            self.log("floor crossing roughly at (%.3f, %.3f)" % (m[0], m[1]))
            poses = self.plan.self_calibration_poses(m, count)
            if len(poses) < 4:
                raise Failure("UNREACHABLE", "crossing (%.2f, %.2f) is outside the arm's view poses" % (m[0], m[1]))
            for i, q in enumerate(poses):
                self._goto("look at the crossing %d" % i, q, JAW_OPEN)
                self.t.sleep(0.6)
                now = self._current()
                expect = base.project(self.k, now, m)
                img = self.t.snapshot()
                try:
                    det = detector.detect(img, near=expect)
                except TypeError:                 # a detector without `near`
                    det = detector.detect(img)
                if det is None:
                    self.log("crossing not seen from pose %d" % i)
                    continue
                if expect is not None and math.hypot(det.u - expect[0], det.v - expect[1]) > gate_px:
                    self.log("pose %d: a crossing at (%.0f, %.0f) px, expected near (%.0f, %.0f): skipped"
                             % (i, det.u, det.v, expect[0], expect[1]))
                    continue
                obs.append((now, "cross", (det.u, det.v)))
                self.log("crossing seen at (%.0f, %.0f) px" % (det.u, det.v))
            self._goto("back to ready", READY, JAW_OPEN)
            cam, rms, where = gc.fit_camera_free(self.k, obs, base)
        except Failure as exc:
            if exc.stop:
                self.t.stop()
            self.log("FAILED %s: %s" % (exc.code, exc.message))
            return None
        except ValueError as exc:
            self.log("FAILED: %s" % exc)
            return None
        cross = where["cross"]
        cam.info = {"date": time.strftime("%Y-%m-%d %H:%M:%S"), "sightings": len(obs),
                    "mount": self.k.mount.to_dict(), "method": "floor cross, position fitted",
                    "markers": [[float(cross[0]), float(cross[1]), 0.0]]}
        cam.save(path)
        self.log("camera fitted: %d sightings, rms %.2f px, f %.0f px, crossing at (%.3f, %.3f), saved %s"
                 % (len(obs), rms, cam.fx, cross[0], cross[1], path))
        return cam


# ================================================================================== transports
class DryRunArm:
    """Prints what would be sent; the gripper 'stops on the bag' so the whole sequence is shown."""

    def __init__(self, kin, start=None, out=print, gripper=0.0):
        self.k = kin
        self.q = dict(start or REST)
        self.q["gripper"] = gripper
        self.out = out
        self.total = 0.0

    def arm_state(self):
        return {"state": "idle", "task": ""}

    def joints(self):
        return dict(self.q)

    def follow(self, points, timeout):
        self.total += points[-1][0]
        self.out("  trajectory: %d points, %.2f s (timeout %.1f s)   %s" % (len(points), points[-1][0], timeout,
                                                                         " ".join("%13s" % n for n in ARM)))
        for t, q in points:
            g = self.k.grasp_point(q)
            self.out("    t=%6.2f  grip (%.3f %.3f %.3f)  %s" % (t, g[0], g[1], g[2],
                                                               " ".join("%+13.4f" % q[n] for n in ARM)))
        self.q.update(points[-1][1])
        return True, "", ""

    def grip(self, position, timeout):
        self.out("  gripper -> %.3f rad (timeout %.0f s)" % (position, timeout))
        self.q["gripper"] = GRIP_HELD if position < CLOSED else position
        return {"ok": True, "position": self.q["gripper"], "stalled": position < CLOSED, "reached": False}

    def stop(self):
        self.out("  /arm/stop")

    def event(self, text):
        self.out("  /robot/event: %s" % text)

    def snapshot(self):
        return None

    def sleep(self, s):
        pass


class RosArm:
    """The arm team's running driver over ROS 2 (domain 42). Needs their so100_interfaces for /arm/state."""

    def __init__(self, snap_url="http://127.0.0.1:8082/snap.jpg"):
        import threading  # noqa: PLC0415
        import rclpy  # noqa: PLC0415
        from control_msgs.action import FollowJointTrajectory, GripperCommand  # noqa: PLC0415
        from rclpy.action import ActionClient  # noqa: PLC0415
        from rclpy.executors import MultiThreadedExecutor  # noqa: PLC0415
        from sensor_msgs.msg import JointState  # noqa: PLC0415
        from std_msgs.msg import String  # noqa: PLC0415
        from std_srvs.srv import Trigger  # noqa: PLC0415
        try:
            from so100_interfaces.msg import ArmState  # noqa: PLC0415
        except ImportError:
            raise SystemExit("so100_interfaces not found: source ~/so100_ws/install/setup.bash first "
                             "(needed to read /arm/state)")
        self.FJT, self.GC = FollowJointTrajectory, GripperCommand
        self.String = String
        self.Trigger = Trigger
        self.snap_url = snap_url
        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node("floor_pick")
        self._traj = ActionClient(self.node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self._grip = ActionClient(self.node, GripperCommand, "/gripper_controller/gripper_cmd")
        self._stop = self.node.create_client(Trigger, "/arm/stop")
        self._event = self.node.create_publisher(String, "/robot/event", 10)
        self._state, self._state_t = None, 0.0
        self._joints, self._joints_t = None, 0.0
        self._history = collections.deque(maxlen=200)
        self._api = {}
        self._goal = None
        self.node.create_subscription(ArmState, "/arm/state", self._on_state, 10)
        self.node.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.ex = MultiThreadedExecutor()
        self.ex.add_node(self.node)
        threading.Thread(target=self.ex.spin, daemon=True).start()

    def _on_state(self, msg):
        self._state = {"state": msg.state, "task": msg.task, "holding": msg.holding, "torque_on": msg.torque_on,
                       "fault": msg.fault, "step": msg.step}
        self._state_t = time.monotonic()

    def _on_joints(self, msg):
        self._joints = dict(zip(msg.name, msg.position))
        self._joints_t = time.monotonic()
        self._history.append((self._joints_t, self._joints))

    def moving(self, seconds=1.5, tol=0.01):
        """True when a joint moved more than `tol` rad in the last `seconds` while no goal of ours ran:
        someone else (another program on the driver) is moving the arm."""
        if self._goal is not None:
            return False
        now = time.monotonic()
        recent = [q for t, q in list(self._history) if now - t <= seconds]
        if len(recent) < 2:
            return False
        return any(max(q.get(n, 0.0) for q in recent) - min(q.get(n, 0.0) for q in recent) > tol for n in ARM)

    def arm_task(self, name, timeout, **goal):
        """Run one of the arm team's API tasks (/arm/home, /arm/pick_bag, /arm/place: API.md) and wait.
        goal: pick_bag -> grasp=(x, y, z) arm base frame, yaw (rad or None), thickness, length;
        place -> target=(x, y). Returns (success, error_code, message[, reading]): reading (only when the
        arm answered) = the result's gripper_pos (rad) and gripper_load (%) after their last grasp, {} when
        their result has no such fields."""
        from geometry_msgs.msg import PoseStamped  # noqa: PLC0415
        from rclpy.action import ActionClient  # noqa: PLC0415
        from so100_interfaces.action import Home, PickBag, Place  # noqa: PLC0415
        kind = {"home": Home, "pick_bag": PickBag, "place": Place}[name]
        client = self._api.get(name)
        if client is None:
            client = self._api[name] = ActionClient(self.node, kind, "/arm/" + name)
        if not client.wait_for_server(timeout_sec=5.0):
            return False, "FAILED", "/arm/%s not available" % name

        def pose(x, y, z=0.0, yaw=None):
            ps = PoseStamped()
            ps.header.frame_id = "base"
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(x), float(y), float(z)
            if yaw is None:        # all zero: "unknown", the arm team's default
                ps.pose.orientation.w = 0.0
            else:
                ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            return ps
        g = kind.Goal()
        if name == "pick_bag":
            g.grasp_pose = pose(*goal["grasp"], yaw=goal.get("yaw"))
            g.thickness, g.length = float(goal.get("thickness", 0.0)), float(goal.get("length", 0.0))
        elif name == "place":
            g.target_pose = pose(*goal["target"])
        handle, why = self._wait(client.send_goal_async(g), 5.0)
        if handle is None or not handle.accepted:
            return False, "FAILED", "/arm/%s goal not accepted (%s)" % (name, why or "rejected")
        res, why = self._wait(handle.get_result_async(), timeout)
        if res is None:
            handle.cancel_goal_async()
            return False, "TIMEOUT", "/arm/%s took longer than %.0f s" % (name, timeout)
        r = res.result
        reading = {k: float(getattr(r, k)) for k in ("gripper_pos", "gripper_load") if hasattr(r, k)}
        return bool(r.success), r.error_code, r.message, reading

    def arm_state(self, wait=3.0):
        end = time.monotonic() + wait
        while time.monotonic() < end:
            if self._state is not None and time.monotonic() - self._state_t < 2.0:   # 2 Hz topic
                return dict(self._state)
            time.sleep(0.05)
        return None

    def joints(self, wait=1.0):
        end = time.monotonic() + wait
        while time.monotonic() < end:
            if self._joints is not None and time.monotonic() - self._joints_t < 0.5:
                return dict(self._joints)
            time.sleep(0.02)
        return None

    def _wait(self, fut, timeout, watch_busy=False):
        end = time.monotonic() + timeout
        while not fut.done():
            if time.monotonic() > end:
                return None, "TIMEOUT"
            if watch_busy:
                st = self._state
                if st is None or time.monotonic() - self._state_t > 2.5:
                    return None, "NO_STATE"
                if st["state"] == "busy":
                    return None, "BUSY"
            time.sleep(0.02)
        return fut.result(), ""

    def follow(self, points, timeout):
        from builtin_interfaces.msg import Duration  # noqa: PLC0415
        from control_msgs.msg import JointTolerance  # noqa: PLC0415
        from trajectory_msgs.msg import JointTrajectoryPoint  # noqa: PLC0415
        if not self._traj.wait_for_server(timeout_sec=3.0):
            return False, "FAILED", "arm driver trajectory action not available"
        goal = self.FJT.Goal()
        goal.trajectory.joint_names = list(ARM)
        goal.path_tolerance = [JointTolerance(name=j, position=0.2) for j in ARM]   # exact path: no retime
        for t, q in points:
            pt = JointTrajectoryPoint(positions=[float(q[j]) for j in ARM])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        handle, why = self._wait(self._traj.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            return False, "FAILED", "trajectory not accepted (%s)" % (why or "rejected")
        self._goal = handle
        res, why = self._wait(handle.get_result_async(), timeout, watch_busy=True)
        self._goal = None
        if res is None:
            handle.cancel_goal_async()
            return False, why, {"TIMEOUT": "timed out", "BUSY": "the arm team's node became busy",
                                "NO_STATE": "/arm/state stopped"}.get(why, why)
        r = res.result
        if r.error_code != 0:
            if "replaced by another" in (r.error_string or ""):   # another program took the arm
                return False, "BUSY", r.error_string
            return False, "FAILED", r.error_string or "error code %d" % r.error_code
        return True, "", ""

    def grip(self, position, timeout):
        if not self._grip.wait_for_server(timeout_sec=3.0):
            return {"ok": False}
        goal = self.GC.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 1.0     # not read by their driver: it uses gripper_torque_limit (500)
        handle, _ = self._wait(self._grip.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            return {"ok": False}
        res, _ = self._wait(handle.get_result_async(), timeout)
        if res is None:
            handle.cancel_goal_async()
            return {"ok": False}
        r = res.result
        return {"ok": True, "position": float(r.position), "stalled": bool(r.stalled), "reached": bool(r.reached_goal)}

    def stop(self):
        if self._goal is not None:
            self._goal.cancel_goal_async()
        if self._stop.wait_for_service(timeout_sec=1.0):
            self._wait(self._stop.call_async(self.Trigger.Request()), 2.0)

    def event(self, text):
        self._event.publish(self.String(data=text))

    def snapshot(self):
        return gc.wait_fresh_snapshot(self.snap_url, settle=0.0)

    def sleep(self, s):
        time.sleep(s)

    def close(self):
        self.ex.shutdown()
        self.node.destroy_node()
        self.rclpy.try_shutdown()


# ========================================================================================= CLI
def parse(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    what = ap.add_mutually_exclusive_group(required=True)
    what.add_argument("--pick", nargs=2, type=float, metavar=("X", "Y"))
    what.add_argument("--place", nargs=2, type=float, metavar=("X", "Y"))
    what.add_argument("--carry", action="store_true")
    what.add_argument("--calibrate", action="store_true")
    what.add_argument("--calibrate-self", action="store_true",
                      help="calibrate the camera on the floor-tile crossing in view, no one needed")
    ap.add_argument("--z", type=float, help="grip height above the floor (pick %.3f, place +%.2f)" % (NECK_Z, PLACE_ABOVE))
    ap.add_argument("--pitch", type=float, help="tool pitch in deg (-90 = straight down); default: steepest")
    ap.add_argument("--hover-only", action="store_true", help="pick: stop above the bag with open jaws")
    ap.add_argument("--camera", action="store_true", help="pick: visual servo with the gripper camera")
    ap.add_argument("--dry-run", action="store_true", help="print the joint trajectories, send nothing")
    ap.add_argument("--marker", nargs=2, type=float, action="append", metavar=("X", "Y"), help="calibration marker")
    ap.add_argument("--marker-z", type=float, default=0.0)
    ap.add_argument("--marker-kind", choices=("red", "grout"), default="red")
    ap.add_argument("--z-offset", type=float, default=0.026, help="real arm height above its model (m)")
    ap.add_argument("--pan-xy", nargs=2, type=float, default=(0.012, 0.0145))
    ap.add_argument("--base-z", type=float, default=0.305)
    ap.add_argument("--yaw", type=float, default=0.0, help="arm yaw on the robot (deg)")
    ap.add_argument("--speed", type=float, default=SPEED_SCALE, help="fraction of the URDF joint speeds")
    ap.add_argument("--camera-file", default=gc.DEFAULT_FILE)
    ap.add_argument("--snap-url", default="http://127.0.0.1:8082/snap.jpg")
    ap.add_argument("--detector", help="module:Class with .detect(bgr) (default: colour + compact blob)")
    return ap.parse_args(argv)


def main(argv=None, transport=None):
    a = parse(argv)
    mount = Mount(tuple(a.pan_xy), a.base_z, a.z_offset, math.radians(a.yaw))
    kin = Kinematics(mount, load_limits())
    planner = Planner(kin, min(a.speed, 1.0))
    own = transport is None
    if transport is None:
        if a.dry_run and a.place:      # as if the bag were held in the carry pose
            transport = DryRunArm(kin, planner.carry_plan()["q"], gripper=GRIP_HELD)
        else:
            transport = DryRunArm(kin) if a.dry_run else RosArm(a.snap_url)
    fp = FloorPick(kin, transport, planner)
    try:
        if a.pick:
            camera = detector = None
            if a.camera and not a.dry_run:
                try:
                    camera = gc.CameraModel.load(a.camera_file)
                except (OSError, ValueError, KeyError):
                    print("no camera calibration at %s: run --calibrate first" % a.camera_file)
                    return 2
                detector = gc.load_detector(a.detector) if a.detector else gc.BagHandleDetector()
            elif a.camera:
                print("dry run: the camera servo is skipped (no frames)")
            res = fp.pick(a.pick[0], a.pick[1], NECK_Z if a.z is None else a.z, a.pitch, a.hover_only, camera, detector)
        elif a.place:
            res = fp.place(a.place[0], a.place[1], NECK_Z + PLACE_ABOVE if a.z is None else a.z)
        elif a.carry:
            res = fp.carry()
        elif a.calibrate_self:
            cam = fp.calibrate_self(gc.GroutCrossDetector(), a.camera_file)
            res = Result(cam is not None, "SUCCESS" if cam else "FAILED", "")
        else:
            if not a.marker:
                print("--calibrate needs at least one --marker X Y")
                return 2
            det = gc.GroutCrossDetector() if a.marker_kind == "grout" else gc.ColourBlobDetector()
            markers = [(x, y, a.marker_z) for x, y in a.marker]
            cam = fp.calibrate(markers, det, (lambda s: print(s)) if a.dry_run else input, a.camera_file)
            res = Result(cam is not None, "SUCCESS" if cam else "FAILED", "")
        total = sum(d for _w, d in fp.moves)
        print("result: %s %s %s (motion %.1f s)" % ("OK" if res.ok else "FAILED", res.code, res.message, total))
        return 0 if res.ok else 1
    except KeyboardInterrupt:
        transport.stop()          # Ctrl-C: cancel our goal and freeze the arm (/arm/stop)
        print("interrupted: /arm/stop called")
        return 130
    finally:
        if own and isinstance(transport, RosArm):
            transport.close()


if __name__ == "__main__":
    sys.exit(main())
