"""The bag at the car (Help Me Carry 5.1): which bag the operator points at, then up to it until the arm reaches it.

run_follow's BAG phase runs one BagTask, made by the client's "bag" (or "bag_again") while it waits in FOLLOW at
the car, or "bag_near" (no pointing: the nearest bag in front, also from IDLE, a step test from the console).
Pure logic: run_follow gives it each cycle's points, the operator and the pointing (View), sends the command it
returns through the usual safety stop and the cable rules, and tells it what really went out (after).

  pointing   stand still, asking the Jetson for pose; base/jetson/pointing.py turns the operator's keypoints (or,
             with --virtual-operator, the PC's pointed spot) into a floor point. Steady: about 1 s still.
             Candidates: LiDAR clusters (a gift bag crosses the LiDAR plane at about 0.245 m) and the detector's
             bag boxes (pointing.bag_candidates; the clear plastic bag may stay under the plane). choose_bag
             picks one: "chosen" with its side and the sentence to say.
  turn       on the spot until the bag lies on the line the robot drives along, y = ARM_Y (the arm's pan axis,
             robot (0.012, 0.0145)); the cable rules apply (run_follow passes the turn through cable.steer).
  far        straight on at FAR_SPEED, steering the bag onto that line, every point counted by the normal
             stop (the bag too: it ends CREEP_DIST before the goal, well outside the clearance).
  creep      straight at CREEP (0.03 m/s) until the bag centre is at x = TARGET_X (the arm reaches a grasp point
             0.24 to 0.34 m ahead of its pan axis). Only the locked bag may come closer than the normal
             clearance: the points within EXCLUDE_R of it are left out of the stop, anything else in the front
             strip stops it (BLOCKED). A virtual bag leaves nothing out: there is nothing real to come close to.
  settle     stand SETTLE_S, measure the bag from the LiDAR, then decide: turn, far, creep, reached or failed.
  look       (look=True) once, at a settle with the bag LOOK_RANGE from the cameras (a far drive from farther stops
             at LOOK_AT for it): the top from the Xtion depth (bag_depth.measure_top, against the local floor),
             else (rgb=True) from the Jetson's front camera (Look -> base/jetson/front_camera.py), else none (the
             arm's default top); the yaw from the bag's LiDAR points (bag_depth.cluster_yaw). Nothing sees the
             bag at the pick distance, so this is the only measurement (base/docs/bag-sensing.md).

Results (status and record "bag"): reached with the bag centre measured after the last stop (code OK from the
LiDAR, ESTIMATED from odometry when the LiDAR never saw it), with "top" (m, "top_by" depth or rgb) and "yaw" (rad,
base frame at the end, the long axis as a line in [-pi/2, pi/2)) when the look measured them, or failed with
NOT_POINTED, NO_BAG, BLOCKED, TIMEOUT, LOST_BAG or CANCELLED (a wait, resume, follow or return came).

The bag centre: the LiDAR sees the side facing it; the centre is taken BAG_HALF_DEPTH behind the middle of those
points, along the ray from the laser (an assumption: a gift bag about 0.10 m deep side-on; measure it).
Frames: base (x ahead, y left, origin at the turning centre) for everything given and reported; the bag is kept
in the odometry frame, which never jumps, so the robot's own motion moves it in the base frame.
"""
import importlib.util
import json
import math
import socket
import sys
import threading
from pathlib import Path

import numpy

import bag_depth
import geometry
import legs
import ports
import safety

# The arm (owner's numbers, 2026-11-15)
ARM_Y = 0.0145  # m, the arm's pan axis is at robot (0.012, 0.0145)
TARGET_X = 0.36  # m, bag centre 0.348 m ahead of pan axis -> front wall ~0.293 m (sweet spot of 0.24..0.34 reach & visible in front webcam)
X_TOL = 0.03  # m, reached when the centre is this close to TARGET_X
Y_TOL = 0.03  # m, and this close to ARM_Y
Y_AIM = 0.015  # m, before the creep a turn puts the bag this close to ARM_Y (straight on keeps y)
# Motion
CREEP = 0.03  # m/s, the close approach (tools/approach.py: under 1.5 cm of run-on after S#)
MIN_CREEP = 0.01  # m/s
FAR_SPEED = 0.08  # m/s, the way up to the creep
FAR_STEER = 1.0  # rad/s per rad of heading error while driving far
FAR_TURN = 0.15  # rad/s, at most, while driving far
CREEP_DIST = 0.25  # m of creep at the end: the far drive stops with the centre at TARGET_X + this
TURN = 0.2  # rad/s on the spot
MIN_TURN = 0.1  # rad/s
TURN_LEAD_S = 0.7  # s: the turn slows over its last TURN * this radians
ALIGN_DEG = 3.0  # far: a heading error under this is steered out while driving, not turned out first
FACE_DEG = 20.0  # a bag this far off the drive line (beside, behind) is always turned to first
MAX_TURNS = 5  # turns on the spot per task
SETTLE_S = 0.8  # s standing before a measurement
# Limits
POINTING_S = 20.0  # s to find a steady pointing at a bag
NO_BAG_S = 2.0  # s of steady pointing with no bag near it: NO_BAG
APPROACH_S = 120.0  # s from the choice to reached
MAX_TRAVEL = 4.0  # m driven from the choice
LOST_S = 1.5  # s without a LiDAR return from a bag the LiDAR had seen: LOST_BAG
EXCLUDE_R = 0.15  # m around the bag centre: its own points, left out of the stop while creeping
MIN_FACE_GAP = 0.02  # m between the front edge and the bag's nearest point: the creep stops there whatever x says
# Finding and tracking
BAG_HALF_DEPTH = 0.05  # m from the middle of the side the LiDAR sees to the bag centre (an assumption)
NEAR_M = 0.6  # m, a bag must be this close to the pointed floor point (pointing.choose_bag)
CANDIDATE_R = 1.5  # m around the pointed point where bags count, for the side words ("the left bag")
OPERATOR_KEEP = 0.35  # m around the operator: their legs, never a bag
MERGE = 0.15  # m, a box's floor point this close to a cluster is that cluster
TRACK_R = 0.15  # m around the predicted centre where the bag's points are looked for
TRACK_JUMP = 0.10  # m a measurement may move the bag; farther is something else
MIN_POINTS = 3  # LiDAR points that make a sighting
# The LiDAR gives nothing within 0.12 m of the laser (lidar.is_body), so for the last few centimetres the bag's
# side is out of its sight: a sighting needs the middle of the side SIGHT_MIN from the laser, and from there on
# the bag is where the last sighting put it, moved by the odometry (a few centimetres of straight creep).
SIGHT_MIN = 0.16  # m from the laser
BLIND_MARGIN = 0.03  # m: a bag predicted this near SIGHT_MIN is not lost when it is not seen
AGAIN_R = 0.25  # m: bag_again looks for the last bag this close to where it was
NEAR_BAG_R = 2.0  # m: bag_near takes the nearest bag within this of the turning centre
NEAR_BAG_DEG = 40.0  # and within this of straight ahead
LAY_KEEP = 0.05  # m beside the body's half width: the cable on the floor nearer the drive line blocks it
VIRTUAL_R = 0.1  # m, a bag this close to a virtual bag is the virtual one
# The look (base/docs/bag-sensing.md)
LOOK_CAM_X = 0.205  # m: both cameras stand about this far ahead of the turning centre (params camera_x; the
                    # front camera "about 0.2 m"), so ranges to the bag are counted from there
LOOK_RANGE = (0.7, 1.5)  # m from there to the bag centre: the Xtion has no depth nearer than about 0.5 m, the
                         # front camera sees the bag whole from 0.7 m
LOOK_AT = 1.1  # m: a far drive from farther stops here to look
LOOK_DEPTH_S = 1.0  # s at the look stop waiting for a depth frame
LOOK_RGB_S = 3.0  # s waiting for the Jetson's front camera answer (a frame over HTTP and the segmentation)
YAW_OK = 0.5  # cluster_yaw confidence for a yaw to be passed on
LOOK_PORT = ports.LOOK  # TCP on the Jetson: front_camera.serve (started by bag_arm.py)
POINTING, TURN_STATE, FAR, CREEP_STATE, SETTLE, REACHED, FAILED = (
    "pointing", "turn", "far", "creep", "settle", "reached", "failed")


def _load_pointing():
    """base/jetson/pointing.py, the one source: next to the Pi code in jetson/ (deploy.sh) or in ../jetson (laptop)."""
    if "pointing" in sys.modules:
        return sys.modules["pointing"]
    here = Path(__file__).resolve().parent
    for path in (here / "jetson" / "pointing.py", here.parent / "jetson" / "pointing.py"):
        if path.exists():
            spec = importlib.util.spec_from_file_location("pointing", path)
            module = importlib.util.module_from_spec(spec)
            sys.modules["pointing"] = module  # dataclasses look their module up there
            spec.loader.exec_module(module)
            return module
    raise ImportError("pointing.py not found in jetson/ or ../jetson/ (deploy.sh copies it)")


pointing = _load_pointing()


class View:
    """One cycle as the task sees it. Points and positions are base frame.

    pose: odometry (x, y, yaw). seen: (N, 2) points the senses read (with --virtual-operator: its legs and bags
    too), for finding and tracking. real: the real points (and depth points) the stop decides on; seen when not
    given. clusters: LiDAR cluster centres (the side facing the laser) from that scan.
    operator: (x, y) or None. found: a pointing.Pointing this cycle, or None. boxes: bag boxes [x1, y1, x2, y2]
    in the frame's pixels, frame (width, height). virtual_bags: centres of the virtual bags. lay: (x, y) circles of
    the cable on the floor. speed_cap: m/s the body may drive this cycle. cable_holding: the cable rules stopped
    the last turn. depth: the camera's depth image of this cycle, or None."""

    def __init__(self, now, pose, seen=None, clusters=(), operator=None, found=None, boxes=(), frame=(320, 240),
                 virtual_bags=(), lay=(), speed_cap=FAR_SPEED, cable_holding=False, real=None, depth=None):
        self.now, self.pose = now, tuple(pose)
        self.depth = depth  # this cycle's depth image (uint16 mm) or None: only the look reads it
        self.seen = numpy.zeros((0, 2)) if seen is None else numpy.asarray(seen, dtype=float).reshape(-1, 2)
        self.real = self.seen if real is None else numpy.asarray(real, dtype=float).reshape(-1, 2)
        self.clusters, self.operator, self.found = [tuple(c) for c in clusters], operator, found
        self.boxes, self.frame = list(boxes), tuple(frame)
        self.virtual_bags, self.lay = [tuple(b) for b in virtual_bags], [tuple(c) for c in lay]
        self.speed_cap, self.cable_holding = speed_cap, cable_holding


def centre_of(face, p):
    """The bag centre BAG_HALF_DEPTH behind the middle of the side the laser sees (base frame)."""
    lx = p["lidar_x"]
    dx, dy = face[0] - lx, face[1]
    n = math.hypot(dx, dy) or 1e-9
    return (face[0] + BAG_HALF_DEPTH * dx / n, face[1] + BAG_HALF_DEPTH * dy / n)


def lidar_clusters(scan, p):
    """legs.clusters (the small objects standing in front of what is behind them) as base-frame centres."""
    return [geometry.lidar_to_base(c, p) for c in legs.clusters(scan)]


def turn_for(bag, y=ARM_Y):
    """rad to turn on the spot (+ left) so that driving straight on brings the bag to robot y."""
    r = math.hypot(*bag)
    if r < 1e-6:
        return 0.0
    return math.atan2(bag[1], bag[0]) - math.asin(max(-1.0, min(1.0, y / r)))


def stop_point(bag, y=ARM_Y, x=TARGET_X):
    """Where the turning centre ends (base frame now) after the turn and the straight drive."""
    th = turn_for(bag, y)
    d = math.sqrt(max(0.0, math.hypot(*bag) ** 2 - y * y)) - x
    return (d * math.cos(th), d * math.sin(th)), d


def crosses(lay, end, p, keep=LAY_KEEP):
    """True when a cable circle lies within the body's half width + keep of the drive from here to end."""
    reach = p["robot_half_width"] + keep
    ex, ey = end
    n2 = ex * ex + ey * ey
    for cx, cy in lay:
        t = 0.0 if n2 < 1e-12 else max(0.0, min(1.0, (cx * ex + cy * ey) / n2))
        if math.hypot(cx - t * ex, cy - t * ey) < reach:
            return True
    return False


def run_on(v, p):
    """m the base rolls after S# from v."""
    return v * p["stop_delay"] + v * v / (2 * p["decel"])


def turn_run_on(w, p):
    """rad the base turns on after S# from w."""
    return abs(w) * p["stop_delay"] + w * w / (2 * p["turn_accel"])


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def virtual_pointing(spot, operator, height=1.4):
    """The PC's pointed floor spot (base frame) as a pointing.Pointing from the operator's shoulder."""
    ox, oy = operator
    return pointing.Pointing(tuple(spot), 1.0, "right", (ox, oy, height), (ox, oy), (ox, oy, height - 0.3))


def camera_pointing(answer, operator, frame, p):
    """The operator's pointing from the Jetson's keypoints: the camera person nearest the operator's bearing."""
    if answer is None or operator is None:
        return None
    bearing = math.degrees(math.atan2(operator[1] - p["camera_y"], operator[0] - p["camera_x"]))
    people = [s for s in answer.people if getattr(s, "kp", None) is not None and abs(s.bearing_deg - bearing) <= 10.0]
    if not people:
        return None
    best = min(people, key=lambda s: abs(s.bearing_deg - bearing))
    r = math.hypot(*operator)
    try:
        return pointing.pointing(best.kp, r, math.degrees(math.atan2(operator[1], operator[0])), p, frame[0], frame[1])
    except (ValueError, TypeError, IndexError):
        return None


class BagTask:
    """One "bag" (mode "point"), "bag_again" (mode "again": the last bag, measured and approached once more) or
    "bag_near" (mode "near": the nearest bag in front, nobody points)."""

    def __init__(self, p, now, seq, mode="point", last=None, look=False, rgb=False):
        self.p, self.seq, self.mode = p, seq, mode
        self.look, self.rgb = look, rgb  # measure the bag in the far stage; ask the Jetson when depth fails
        self.id = round(now, 3)  # tells this task's status from an earlier one (the bridge reads it)
        self.t0 = now
        self.state = POINTING
        self.steady = pointing.Steady()
        self.no_bag_since = None
        self.bag = None  # odometry (x, y) of the bag centre
        self.last = last  # odometry (x, y) of the previous task's bag (mode "again")
        self.lidar = False  # the LiDAR saw the bag since the choice
        self.virtual = False
        self.seen_at = None  # when the LiDAR last saw it (or it was out of sight in the blind range)
        self.seen_travel = None  # self.travelled at the last sighting
        self.chosen = None  # {"x", "y", "side", "say"} in the base frame at the choice
        self.result = None  # {"ok", "code", "x", "y"} or {"ok", "code", "message"}
        self.chosen_at = None
        self.start_pose = None  # odometry pose at the choice (MAX_TRAVEL)
        self.travelled = 0.0
        self.last_pose = None
        self.turns = 0
        self.turn_to = None  # odometry yaw the turn goes for
        self.turn_sent = False
        self.settle_since = None
        self.sent = (0.0, 0.0)
        self.why = None  # what the last step did, for the log
        self.wants_pose = mode == "point"  # ask the Jetson for keypoints (only while looking for the pointing)
        self.measured = None  # the look's outcome {"top", "top_by", "yaw_odom"}; {} when not looked; None: not yet
        self.look_state = None  # {"stage", "since", "pose", "range", "centre", "points", "yaw", ...} once looking
        self.look_request = None  # {"id", "points"} for the Jetson's front camera (Look sends it)
        self.rgb_answer = None

    # ------------------------------------------------------------ outputs
    @property
    def done(self):
        return self.state in (REACHED, FAILED)

    def status(self):
        """For the status line and the record: base-frame chosen bag and the result."""
        out = {"id": self.id, "seq": self.seq, "mode": self.mode, "state": self.state, "chosen": self.chosen,
               "result": self.result}
        if self.look_state is not None:
            out["look"] = {k: v for k, v in self.look_state.items() if k not in ("points", "since")}
        return out

    @property
    def wants_depth(self):
        """True while the look waits for a depth frame (run_follow passes every frame's depth anyway)."""
        return self.look_state is not None and self.look_state["stage"] == "depth" and not self.done

    def looked(self, answer):
        """The Jetson's front camera answer for look_request (Look calls it; another id is ignored)."""
        if self.look_request is not None and isinstance(answer, dict) and answer.get("id") == self.look_request["id"]:
            self.rgb_answer = answer

    def odom_bag(self):
        return None if self.bag is None else [round(self.bag[0], 3), round(self.bag[1], 3)]

    def cancel(self, now, by):
        if not self.done:
            self._fail("CANCELLED", "%s came" % by)

    def safety_points(self, points):
        """The points the stop checks this cycle: while creeping, the bag's own points left out (not a virtual bag)."""
        if self.state != CREEP_STATE or self.bag is None or self.virtual or self.last_pose is None:
            return points
        pts = numpy.asarray(points, dtype=float).reshape(-1, 2)
        if not len(pts):
            return pts
        bx, by = geometry.odom_to_base(self.bag, self.last_pose)
        return pts[numpy.hypot(pts[:, 0] - bx, pts[:, 1] - by) > EXCLUDE_R]

    # ------------------------------------------------------------ the step
    def step(self, view):
        """(v, w, turn error or None) for this cycle."""
        now, pose = view.now, view.pose
        if self.last_pose is not None and self.start_pose is not None:
            self.travelled += math.dist(pose[:2], self.last_pose[:2])
        self.last_pose = pose
        self.why = None
        if self.done:
            return self._stand()
        if self.state == POINTING:
            return self._pointing(view)
        if now - self.chosen_at > APPROACH_S:
            return self._fail("TIMEOUT", "no bag reached in %.0f s" % APPROACH_S)
        if self.travelled > MAX_TRAVEL:
            return self._fail("BLOCKED", "drove %.1f m without reaching the bag" % self.travelled)
        if view.cable_holding:
            return self._fail("BLOCKED", "my cable is wound")
        seen = self._track(view)
        if not seen and self._blind(view):
            seen, self.seen_at = True, now  # out of sight as expected: the last sighting stands
        if self.lidar and not seen and now - self.seen_at > LOST_S:
            return self._fail("LOST_BAG", "no LiDAR return from the bag for %.1f s" % (now - self.seen_at))
        if self.state == TURN_STATE:
            return self._turn(view)
        if self.state == FAR:
            return self._far(view)
        if self.state == CREEP_STATE:
            return self._creep(view)
        return self._settle(view)

    def after(self, now, stop, sent):
        """What run_follow really sent this cycle, and why it stopped the command (its stop reason) or None."""
        self.sent = tuple(sent)
        if self.done or stop is None:
            return
        if stop == "obstacle" and self.state in (FAR, CREEP_STATE, TURN_STATE):
            self._fail("BLOCKED", "something is in my way")
        elif self.state == SETTLE and (sent[0] or sent[1]):
            self.settle_since = None

    # ------------------------------------------------------------ states
    def _stand(self):
        return 0.0, 0.0, None

    def _base(self, view):
        return geometry.odom_to_base(self.bag, view.pose)

    def _pointing(self, view):
        now = view.now
        if self.mode == "again":
            return self._again(view)
        if self.mode == "near":
            return self._ahead(view)
        if now - self.t0 > POINTING_S:
            if self.no_bag_since is not None:
                return self._fail("NO_BAG", "no bag where you point")
            return self._fail("NOT_POINTED", "no steady pointing in %.0f s" % POINTING_S)
        found = self.steady.add(now, view.found)
        if found is None:
            return self._stand()
        candidates = self._candidates(view, found.point)
        choice = pointing.choose_bag(found, candidates, near_m=NEAR_M)
        if choice is None:
            self.no_bag_since = now if self.no_bag_since is None else self.no_bag_since
            self.why = "pointed at (%.2f, %.2f): no bag within %.1f m" % (found.point[0], found.point[1], NEAR_M)
            if now - self.no_bag_since >= NO_BAG_S:
                return self._fail("NO_BAG", "no bag within %.1f m of where you point" % NEAR_M)
            return self._stand()
        side = choice.side
        self._choose(view, choice.xy)
        self.chosen = {"x": round(choice.xy[0], 3), "y": round(choice.xy[1], 3), "side": side, "say": choice.say,
                       "point": [round(found.point[0], 3), round(found.point[1], 3)], "candidates": len(candidates)}
        return self._plan(view)

    def _again(self, view):
        """bag_again: the last bag, looked for near where it was."""
        if self.last is None:
            return self._fail("NO_BAG", "no bag chosen before")
        last = geometry.odom_to_base(self.last, view.pose)
        near = [c for c in self._bags(view) if math.dist(c, last) <= AGAIN_R]
        if not near:
            return self._fail("NO_BAG", "the bag is not where it was")
        bag = min(near, key=lambda c: math.dist(c, last))
        self._choose(view, bag)
        self.chosen = {"x": round(bag[0], 3), "y": round(bag[1], 3), "side": None, "say": "I will look at the bag again.",
                       "point": None, "candidates": len(near)}
        return self._plan(view)

    def _ahead(self, view):
        """bag_near: the nearest bag within NEAR_BAG_R and NEAR_BAG_DEG of straight ahead."""
        near = [c for c in self._bags(view)
                if math.hypot(*c) <= NEAR_BAG_R and abs(math.degrees(math.atan2(c[1], c[0]))) <= NEAR_BAG_DEG]
        if not near:
            return self._fail("NO_BAG", "no bag within %.1f m in front of me" % NEAR_BAG_R)
        bag = min(near, key=lambda c: math.hypot(*c))
        self._choose(view, bag)
        self.chosen = {"x": round(bag[0], 3), "y": round(bag[1], 3), "side": None,
                       "say": "I will go to the bag in front of me.", "point": None, "candidates": len(near)}
        return self._plan(view)

    def _bags(self, view):
        """Bag-sized LiDAR clusters (centres, base frame), the operator's legs left out."""
        out = []
        for face in view.clusters:
            if view.operator is not None and math.dist(face, view.operator) <= OPERATOR_KEEP:
                continue
            if face[0] <= safety.FRONT - 0.05 and abs(face[1]) < 0.3:
                continue  # the robot's own body
            out.append(centre_of(face, self.p))
        return out

    def _candidates(self, view, point):
        bags = [c for c in self._bags(view) if math.dist(c, point) <= CANDIDATE_R]
        faces = [c for c in view.clusters]
        for xy in pointing.bag_candidates(view.boxes, self.p, view.frame[0], view.frame[1], clusters=faces):
            centre = centre_of(xy, self.p)
            if view.operator is not None and math.dist(centre, view.operator) <= OPERATOR_KEEP:
                continue
            if math.dist(centre, point) <= CANDIDATE_R and all(math.dist(centre, b) > MERGE for b in bags):
                bags.append(centre)
        return bags

    def _choose(self, view, bag):
        self.wants_pose = False
        self.bag = geometry.base_to_odom(bag, view.pose)
        self.chosen_at, self.start_pose = view.now, view.pose
        self.virtual = any(math.dist(bag, v) <= VIRTUAL_R for v in view.virtual_bags)
        if self._sighting(view, bag) is not None:
            self.lidar, self.seen_at, self.seen_travel = True, view.now, 0.0

    def _plan(self, view):
        """After the choice: too far, or across the cable, or on to the first turn."""
        bag = self._base(view)
        end, d = stop_point(bag)
        if d > MAX_TRAVEL:
            return self._fail("BLOCKED", "the bag is %.1f m away, more than %.1f m" % (d, MAX_TRAVEL))
        if d > 0 and crosses(view.lay, end, self.p):
            return self._fail("BLOCKED", "my cable lies on the way to the bag")
        return self._decide(view)

    def _near(self, view, predicted):
        """The seen points within TRACK_R of `predicted` (base frame), the operator's left out."""
        pts = view.seen
        if not len(pts):
            return pts
        near = pts[numpy.hypot(pts[:, 0] - predicted[0], pts[:, 1] - predicted[1]) <= TRACK_R]
        if view.operator is not None and len(near):
            near = near[numpy.hypot(near[:, 0] - view.operator[0], near[:, 1] - view.operator[1]) > OPERATOR_KEEP]
        return near

    def _sighting(self, view, predicted):
        """The bag centre from the points near `predicted` (base frame), or None."""
        near = self._near(view, predicted)
        if len(near) < MIN_POINTS:
            return None
        face = (float(near[:, 0].mean()), float(near[:, 1].mean()))
        if math.hypot(face[0] - self.p["lidar_x"], face[1]) < SIGHT_MIN:
            return None  # only part of it is out of the blind range: the middle would be wrong
        centre = centre_of(face, self.p)
        return centre if math.dist(centre, predicted) <= TRACK_JUMP else None

    def _blind(self, view):
        """True when the kept bag is too near the laser to be seen (SIGHT_MIN): not seeing it is no loss."""
        bx, by = self._base(view)
        lx = self.p["lidar_x"]
        r = math.hypot(bx - lx, by)
        return r - BAG_HALF_DEPTH < SIGHT_MIN + BLIND_MARGIN

    def _track(self, view):
        """Move the kept bag to this cycle's LiDAR sighting. True when the LiDAR saw it."""
        got = self._sighting(view, self._base(view))
        if got is None:
            return False
        new = geometry.base_to_odom(got, view.pose)
        self.bag = ((self.bag[0] + new[0]) / 2, (self.bag[1] + new[1]) / 2)
        self.lidar, self.seen_at, self.seen_travel = True, view.now, self.travelled
        return True

    def _decide(self, view):
        """From standing: the next move for where the bag is now."""
        bx, by = self._base(view)
        left, ey = bx - TARGET_X, by - ARM_Y
        if abs(math.degrees(turn_for((bx, by)))) > FACE_DEG and math.hypot(bx, by) > TARGET_X + X_TOL:
            return self._start_turn(view)  # beside or behind: face it first
        if left - run_on(min(FAR_SPEED, view.speed_cap), self.p) > CREEP_DIST + X_TOL:  # _far would drive
            if abs(math.degrees(turn_for((bx, by)))) > ALIGN_DEG:
                return self._start_turn(view)
            return self._go(FAR, view)
        if left > X_TOL:
            if abs(ey) > Y_AIM and self.turns < MAX_TURNS:
                return self._start_turn(view)
            return self._go(CREEP_STATE, view)
        if left < -X_TOL:
            return self._fail("BLOCKED", "the bag is too close: x %.3f" % bx)
        if abs(ey) <= Y_TOL:
            dead = None if self.seen_travel is None else round(self.travelled - self.seen_travel, 3)
            self.result = {"ok": True, "code": "OK" if self.lidar else "ESTIMATED", "x": round(bx, 3), "y": round(by, 3),
                           "odometry_m": dead}  # m driven since the last LiDAR sighting (None: never seen by it)
            if self.measured and self.measured.get("top") is not None:
                self.result["top"], self.result["top_by"] = self.measured["top"], self.measured["top_by"]
            if self.measured and self.measured.get("yaw_odom") is not None:
                self.result["yaw"] = round(bag_depth.wrap_line(self.measured["yaw_odom"] - view.pose[2]), 3)
            self.state = REACHED
            self.why = "reached (%.3f, %.3f)" % (bx, by)
            return self._stand()
        if self.turns < MAX_TURNS:
            return self._start_turn(view)
        return self._fail("BLOCKED", "could not line up with the bag: y %.3f" % by)

    def _go(self, state, view):
        self.state = state
        return self.step_state(view)

    def step_state(self, view):
        return {FAR: self._far, CREEP_STATE: self._creep, TURN_STATE: self._turn}[self.state](view)

    def _start_turn(self, view):
        if self.turns >= MAX_TURNS:
            return self._fail("BLOCKED", "could not face the bag in %d turns" % MAX_TURNS)
        self.turns += 1
        self.turn_to = view.pose[2] + turn_for(self._base(view))
        self.turn_sent = False
        return self._go(TURN_STATE, view)

    def _turn(self, view):
        rem = _wrap(self.turn_to - view.pose[2])
        w = math.copysign(min(TURN, self.p["max_turn_rate"], max(MIN_TURN, abs(rem) / TURN_LEAD_S)), rem)
        if self.turn_sent and abs(rem) <= turn_run_on(w, self.p):
            return self._to_settle()
        self.turn_sent = True
        self.why = "turn %.1f deg left" % math.degrees(rem)
        return 0.0, w, rem

    def _far(self, view):
        bx, by = self._base(view)
        v = min(FAR_SPEED, view.speed_cap)
        if bx - run_on(v, self.p) <= TARGET_X + CREEP_DIST:
            return self._to_settle()
        if self._look_pending():
            r = math.hypot(bx - LOOK_CAM_X, by)
            if LOOK_RANGE[0] < r and r - run_on(v, self.p) <= LOOK_AT:
                return self._to_settle()  # stand to look on the way
        err = turn_for((bx, by))
        w = max(-FAR_TURN, min(FAR_TURN, FAR_STEER * err))
        self.why = "far, %.2f m to go" % (bx - TARGET_X)
        return v, w, None

    def _creep(self, view):
        bx, by = self._base(view)
        left = bx - TARGET_X
        v = min(CREEP, view.speed_cap, max(MIN_CREEP, left))
        if left <= run_on(v, self.p) or left <= 0.003:
            return self._to_settle()
        if self._face_gap(view) < MIN_FACE_GAP:
            return self._to_settle()
        if safety.must_stop(self.safety_points(view.real), v, 0.0, self.p):
            return self._fail("BLOCKED", "something else is in front of me")
        self.why = "creep, %.3f m to go" % left
        return v, 0.0, None

    def _face_gap(self, view):
        """m from the front edge to the bag's nearest point ahead (inf when the LiDAR sees none of it)."""
        if self.virtual or not len(view.seen):
            return math.inf
        bx, by = self._base(view)
        pts = view.seen
        own = pts[(numpy.hypot(pts[:, 0] - bx, pts[:, 1] - by) <= EXCLUDE_R)
                  & (numpy.abs(pts[:, 1]) < self.p["robot_half_width"] + safety.SIDE_MARGIN)]
        return float(own[:, 0].min()) - safety.FRONT if len(own) else math.inf

    def _to_settle(self):
        self.state, self.settle_since = SETTLE, None
        return self._stand()

    def _settle(self, view):
        if self.sent != (0.0, 0.0):
            self.settle_since = None
        if self.settle_since is None:
            self.settle_since = view.now
        if view.now - self.settle_since < SETTLE_S - 1e-6:
            return self._stand()
        if self.lidar and view.now - self.seen_at > LOST_S / 2:
            return self._stand()  # the LiDAR has not seen it lately: wait for a sighting (or LOST_BAG)
        if self._look_pending():
            bx, by = self._base(view)
            r = math.hypot(bx - LOOK_CAM_X, by)
            if self.look_state is not None or LOOK_RANGE[0] <= r <= LOOK_RANGE[1]:
                return self._look(view, r)
            if r < LOOK_RANGE[0]:
                self.look_state = {"stage": "done", "range": round(r, 3),
                                   "why": "the bag was nearer than %.1f m at the first stop: not looked at"
                                   % LOOK_RANGE[0]}
                self.measured = {}
        return self._decide(view)

    # ------------------------------------------------------------ the look
    def _look_pending(self):
        return self.look and self.measured is None and not self.virtual

    def _look(self, view, r):
        """Stand while the bag is measured (depth, then the front camera), then decide as always."""
        now, ls = view.now, self.look_state
        if ls is None:
            bx, by = self._base(view)
            pts = self._near(view, (bx, by))
            yaw, conf, why = bag_depth.cluster_yaw(pts)
            ls = self.look_state = {"stage": "depth", "since": now, "pose": [round(c, 4) for c in view.pose],
                                    "range": round(r, 3), "centre": [round(bx, 3), round(by, 3)],
                                    "points": [[round(float(x), 3), round(float(y), 3)] for x, y in pts],
                                    "yaw": None if yaw is None else round(yaw, 3), "yaw_conf": conf, "yaw_why": why,
                                    "depth": None, "rgb": None, "why": None}
        if ls["stage"] == "depth":
            if view.depth is not None:
                ls["depth"] = bag_depth.measure_top(view.depth, ls["centre"], self.p)
            elif now - ls["since"] >= LOOK_DEPTH_S:
                ls["depth"] = {"ok": False, "why": "no depth frame in %.1f s" % LOOK_DEPTH_S}
            else:
                return self._stand()
            if ls["depth"]["ok"]:
                return self._look_done(view, ls["depth"]["top"], "depth")
            if not self.rgb:
                return self._look_done(view, None, None, "depth: %s" % ls["depth"]["why"])
            ls["stage"], ls["since"] = "rgb", now
            self.look_request = {"id": "%s-%d" % (self.id, self.seq), "points": ls["points"]}
        answer = self.rgb_answer
        if answer is not None:
            ls["rgb"] = {k: answer.get(k) for k in ("ok", "top", "top_by", "conf", "handle", "why")}
            if answer.get("ok") and answer.get("top") is not None:
                return self._look_done(view, float(answer["top"]), "rgb")
            return self._look_done(view, None, None, "front camera: %s" % answer.get("why"))
        if now - ls["since"] >= LOOK_RGB_S:
            return self._look_done(view, None, None, "front camera: no answer in %.1f s" % LOOK_RGB_S)
        return self._stand()

    def _look_done(self, view, top, by, why=None):
        ls = self.look_state
        ls["stage"], ls["why"] = "done", why
        yaw_odom = None
        if ls["yaw"] is not None and ls["yaw_conf"] >= YAW_OK:
            yaw_odom = ls["yaw"] + ls["pose"][2]
        self.measured = {"top": None if top is None else round(top, 3), "top_by": by, "yaw_odom": yaw_odom}
        self.why = "looked: top %s (%s), yaw %s" % (top, by or why, ls["yaw"])
        return self._decide(view)

    def _fail(self, code, message):
        self.state, self.wants_pose = FAILED, False
        self.result = {"ok": False, "code": code, "message": message}
        self.why = "%s: %s" % (code, message)
        return self._stand()


class Look:
    """The one part of this module with a socket: hands a task's look_request to the Jetson's front camera
    service (front_camera.serve, TCP LOOK_PORT, one JSON line each way) on a thread, and its answer back to the
    task. run_follow calls serve(task, now) every BAG cycle; it never blocks the cycle. No service, or no
    answer: an answer {"ok": false, "why"} (the task gives up after LOOK_RGB_S anyway)."""

    def __init__(self, hosts, port=LOOK_PORT, connect_s=0.5, answer_s=LOOK_RGB_S):
        self.hosts, self.port = list(hosts), port
        self.connect_s, self.answer_s = connect_s, answer_s
        self.asked = None
        self.answer = None
        self.lock = threading.Lock()

    def serve(self, task, now):
        req = getattr(task, "look_request", None)
        if req is None:
            return
        if req["id"] != self.asked:
            self.asked = req["id"]
            with self.lock:
                self.answer = None
            threading.Thread(target=self._ask, args=(req,), daemon=True).start()
        with self.lock:
            answer = self.answer
        if answer is not None and answer.get("id") == req["id"]:
            task.looked(answer)

    def _ask(self, req):
        answer, why = None, []
        for host in self.hosts:
            try:
                with socket.create_connection((host, self.port), timeout=self.connect_s) as s:
                    s.settimeout(self.answer_s)
                    s.sendall((json.dumps(req) + "\n").encode("utf-8"))
                    line = s.makefile("rb").readline(65536)
                answer = json.loads(line.decode("utf-8"))
                if isinstance(answer, dict):
                    break
                answer = None
            except (OSError, ValueError) as exc:
                why.append("%s: %s" % (host, exc))
        if answer is None:
            answer = {"ok": False, "why": "no front camera answer (%s)" % "; ".join(why)}
        answer["id"] = req["id"]
        with self.lock:
            self.answer = answer
