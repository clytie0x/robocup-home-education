"""Camera bag pick for /arm/pick_bag (bag_method = camera) and its put-down for /arm/place.

Since 2026-11-18 the webcam (webcam_view.py, port 8082) is FIXED under the robot's top deck, looking ahead and
down; it no longer rides on the forearm. Turning the arm does not change its picture, so the old "turn the
arm until the bag is in the middle of the image" step is gone. The camera model comes from the HMC team's
calibration (front_camera.json, robot frame with its origin at the turning centre): a pinhole, or OpenCV
fisheye when that file says dist_model "fisheye".

  1. where (before EVERY try: a miss or a drop may have pushed the bag): the arm goes to LOOK if it would be
     in the picture, then the bag's centre comes from the image (locate). Two edge estimates, the rim at
     bag_cam_rim_h and the foot on the floor, are reconciled (reconcile_edges). The camera centre is used
     when its confidence is at least CONF_MIN; otherwise the goal's x, y are used: always with only the rim
     in the picture, or a blob that is not a bag detection (trusted). A camera centre more
     than LIDAR_DISAGREE from the Pi LiDAR goal, or a rim wider than WIDTH_MAX, is looked at once more and
     then the LiDAR goal is used (the source is recorded). A bag that is not standing, or whose rim is not
     about bag_cam_width wide, is still an error. No bag in the picture: the goal on the first try, an error
     later. The rim height in the 3D estimate is the Pi's measured top when it is between RIM_LO and RIM_HI,
     otherwise bag_cam_rim_h. Orientation: the goal's yaw = the handles' line; no yaw = the bag faces the
     robot (handles parallel to the robot's y axis).
  2. what: the bag's BACK wall (bag_cam_half_depth beyond the centre) when the arm reaches it, else its FRONT wall
     (the one nearer the arm) (WALLS: on the back wall the jaw inside the bag gets under the rim's inner fold, which
     hooks a heavy bag; the bag then hangs towards the robot, which held_clear keeps off the deck). The jaws
     (pitched -60 deg) close across that wall. When the tuned model gives a handle centroid, the grasp
     keeps the rim's forward distance and shifts sideways onto the handle; otherwise the paper rim
     (a handle has rope only at its very top). Finger tips are bag_cam_grip_depth under the rim (1 or 1.5 cm
     shallower when that is out of reach: SHALLOWER). One jaw moves, so
     the middle of the open gap (gripper x GAP_MID), not the tool axis, is put on the wall. The wrist roll
     lines the closing axis up with the wall's normal (Kinematics.roll_for), not a guessed sign.
     A wall further than wall_reach(tips height) from the pan axis fails at once: move the robot closer (a low
     bag must be nearer than a tall one).
  3. move: before anything moves, the held move (4.) must exist on paper from the close pose. Then one planned
     trajectory from above the wall down onto it (the old stepwise moves when that plan is not all in reach),
     and the gripper closes.
  4. carry: ONE held move from the close pose: up, out to the side (CARRY_SIDES) and the jaws tilted only as far
     as needed (CARRY_PITCHES) until the bag's bottom is bag_carry_bottom over the floor (over the LiDAR plane).
     No straight lift first, no Xtion check on the way (robot, 2026-11-20: the bag held in every pick). Then,
     from the side (the arm out of the front camera's picture): the bag must have left its place on the floor
     (spot_left), else (it slipped out) open, look again, next try. The result code is SUCCESS_UNVERIFIED when
     the gripper closed but the camera could not tell (no picture, the bag carried in front, or no bag found while
     its old spot does not show the floor: bag_vision.spot_floor). The Xtion frames before the down move and after
     the carry are saved with the trial (record only).
bag_hover_only: stop with the jaws OPEN at the close height and hold still there (a test: hang a plumb line,
measure how far the gap is from the wall); the log has the asked and the real (forward kinematics) gap middle.
Put-down: one held move to 8 cm over the release height at the target, straight down until the bag's bottom is on
the floor (PUT_DROP 0, or earlier: a stall on that move is the touchdown), open, back off.
"""
import copy
import json
import math
import os
import time
import urllib.request

import numpy as np

from . import bag_vision as bv
from . import pick_trial as pt
from .ik import ARM, FIXED_INNER_X, JAW_G_MAX, TIP_Y, jaw_gap
from .task_rules import Failed

TIP = -TIP_Y                      # gripper origin -> finger tips along the tool axis (m)
GAP_MID = FIXED_INNER_X - jaw_gap(JAW_G_MAX) / 2   # gripper-frame x of the middle of the fully open jaw gap
# (extra reach along the bearing m, pitch deg), best first: the open gap is 7.5 cm, so +-1.5 cm still keeps a
# wall that is where it should be inside the jaws
TRIES = ((0.0, -60), (0.015, -60), (-0.015, -60), (0.03, -50))
# The two edge estimates (reconcile_edges). They agree at or under EDGE_AGREE (their mean). Further apart,
# the foot wins: it meets the known floor, the rim uses an assumed height. Confidence is 0 at EDGE_LOST.
# The camera centre is used only at confidence >= CONF_MIN; otherwise the request's x, y are used.
EDGE_AGREE = 0.04
EDGE_LOST = 0.12
CONF_MIN = 0.55
# The rim alone is under CONF_MIN, so it never replaces the goal's x, y. The steep camera (pitch 47.7 deg, 2026-11-20)
# sees the far rim above the near one and the rim height is assumed: on a 36 cm bag whose foot projected to 0.29 m
# it put the front wall 0.52 m out (frames 2026-11-20 22:43 and 22:47; the true distance was not taped).
RIM_ONLY_CONF = 0.50
# Blob sources that are not a bag detection. The floor-difference mask takes lamp glare on the floor for the bag: one
# scene gave 0.52 and 0.73 m (2026-11-20 22:47), and its outline does not move when the bag is lifted.
UNTRUSTED_SOURCES = ("floor-diff",)
# m: a foot this much farther than the blob's top rim is a piece of the bag, not its foot (locate)
FOOT_PAST_TOP = 0.02
# the front rim found inside an open bag's outline (bag_vision.front_rim) with the rim height measured: over
# CONF_MIN, so it replaces the goal (2026-11-27: 19.1 to 19.2 cm for bags taped at 20, 23.9 to 24.3 at 25). With
# the rim height only assumed it stays RIM_ONLY_CONF (front_rim_trust): 2 cm of height is about 1 cm of distance.
FRONT_RIM_CONF = 0.65
# m, the gap middle's reach from which bag_cam_close_bias applies. It was measured at about 0.31 m; at 0.255 m it made
# the close 2.1 cm short and 1.8 cm low (pick 2026-11-20 22:53).
BIAS_FROM_R = 0.28
# arm pose for looking: folded REST pose stays completely out of the front camera's picture (0 pts in view;
# READY puts 55 pts in view), so the arm never lifts high just to look at the bag
LOOK = {"shoulder_pan": 0.0, "shoulder_lift": 0.04, "elbow_flex": -0.04, "wrist_flex": 0.0, "wrist_roll": 0.0}
LIFT_DZ = (0.02, 0.04, 0.06, 0.08, 0.10)
STRAIGHT_UP = tuple(0.02 * k for k in range(1, 11))   # m: the first lift, straight up to 20 cm
LIFT_SPEED = 0.25                 # rad/s of the fastest joint on that lift
# The carry (carry_plan): ONE held move from the close pose, up, out to the side and the jaws tilted a little, to
# where the bag's bottom is bag_carry_bottom above the floor (clear of the LiDAR plane LIDAR_Z). The jaws hold a
# paper wall by friction, so the least tilt that gets there wins (the old tilt to 0 deg twisted it out, video
# 2026-11-19; at the grasp pitch -60 the tips get no higher than 0.39 m). Simulated over 109 bag placements and
# tried on the robot 2026-11-20 (the bag held in every pick; side 60 deg, tilt 25 deg).
CARRY_SIDES = (60.0, 45.0, -30.0, None)   # deg from straight ahead (+ = left); None = in front (hides the camera)
# None = the grasp pitch. Down to -15 (45 deg of tilt): a tall bag (37 cm rim) only clears the LiDAR with the tips
# about 0.59 m up, which the arm reaches at -15 deg (robot 2026-11-20: a heavy 37 cm bag held at 45 deg, back wall)
CARRY_PITCHES = (None, -45.0, -35.0, -25.0, -15.0)
CARRY_R = (0.32, 0.28, 0.24, 0.20)   # m, finger tips from the pan axis at the carry pose: the shortest move of these
# Which wall the jaws close on, in this order. "back" (the far wall): the jaw inside the bag is the lower one (the
# closing axis tilts 30 deg at pitch -60), so it reaches under the rim's inner fold and the fold hooks on it; a
# heavy bag held that way, while on its front wall it slipped out, with or without tilt (robot 2026-11-20).
# "front" when the back wall is out of reach (a front grip lets the bag stand one bag-depth further out).
WALLS = ("back", "front")
DRAG_MAX = 0.015                  # m the grip may move sideways while the bag's bottom is still on the floor
# Grip depth (rim - finger tips at the close) when the bag_cam_grip_depth pose is out of reach (a low bag further
# away): 1 and 1.5 cm shallower, so at least 3.5 cm of wall stays in the jaws.
SHALLOWER = (0.0, 0.01, 0.015)
# Farthest front wall (m from the pan axis) by the tips' height over the floor at the close: the arm reaches less
# far the lower it has to go (simulated 2026-11-20, pitch -50 to -80, outward close). Over 0.21 m: REACH_WALL.
WALL_REACH = ((0.13, 0.24), (0.15, 0.28), (0.17, 0.30), (0.19, 0.31), (0.21, 0.32))
LIDAR_Z = 0.245                   # m above the floor, the LiDAR scan plane (base/jetson/so100_ik.py LIDAR_Z)
NEARER = (0.0, 0.015, 0.03)       # m: the gap middle moved nearer for an outward close (grasp_outward)
# m above the grasp height the put-down lets go. 0: the bag's bottom is on the floor first. At 3 cm the bag fell
# out in 4 of 6 put-downs on the robot (2026-11-20: it hangs tilted); at 0, 3 of 3 stood.
PUT_DROP = 0.0
PUT_PITCHES = (None, -50.0, -45.0, -40.0, -35.0)   # deg over the put-down spot; None = the grasp pitch
JOINT_STEP = 0.03                 # rad between path points
FREE_SPEED, HELD_SPEED, PUT_SPEED = 0.8, 0.4, 0.6  # rad/s of the fastest joint: empty / lifting / putting down
DESCEND_DZ = (0.08, 0.06, 0.04, 0.02, 0.0)  # m above the close pose, the stepwise way down
LIFT_CONTINUOUS = (0.05, 0.10, 0.15, 0.20)  # m, one straight lift: every 5 cm, same 20 cm
PATH_BOW = 0.02                 # m, a direct move may pass its end reaches by this much
REACH_WALL = 0.32                 # m, bag front from the pan axis: further, tell the operator to move closer
LIDAR_DISAGREE = 0.10             # m, camera centre vs the Pi LiDAR goal: look once more, then use the LiDAR
WIDTH_MAX = 0.30                  # m, a wider rim is not a bag
RIM_LO, RIM_HI = 0.15, 0.50       # m, a Pi-measured top inside this replaces bag_cam_rim_h
PICK_TEMP_C = 55.0                # a pick waits while the hottest motor is at or above this
DRIVER_TEMP_C = 60.0              # the driver's max_temp: it refuses a move at or above this
HELD_CODES = ("SUCCESS", "SUCCESS_UNVERIFIED")
# Share of the bag's old spot that must show something other than the bag (bag_vision.spot_floor) before a picture
# with no bag found counts as "lifted". Robot Xtion frames 2026-11-19: 0.00 with the bag still standing (15:17),
# 0.99 with it lifted (17:29).
SPOT_FLOOR = 0.5


def spot_left(start, now, floor=None, min_overlap=0.5, same_rows=25.0, min_floor=SPOT_FLOOR):
    """Has the bag left the place it stood? start = its blob before the grasp (arm out of the picture), now
    = the bag blob in a picture with the arm turned aside (None: that picture shows no bag; a missing picture
    must not get here). False when a blob still covers most of the same columns with its top at about the
    same rows (the bag stayed on the floor); True otherwise. No bag found is True only when its spot shows the
    floor (floor = bag_vision.spot_floor, at least min_floor); else None: a missed detection is no lift."""
    if now is None:
        return True if floor is not None and floor >= min_floor else None
    w = max(1.0, start["x1"] - start["x0"])
    overlap = (min(start["x1"], now["x1"]) - max(start["x0"], now["x0"])) / w
    return not (overlap > min_overlap and abs(now["y0"] - start["y0"]) < same_rows)


def trusted(blob):
    """True when the blob is a bag detection (not the floor-difference fallback). None is not."""
    return blob is not None and blob.get("source") not in UNTRUSTED_SOURCES


def front_rim_trust(loc, rim_from):
    """loc with a front-rim reading kept at RIM_ONLY_CONF unless the rim height was measured (rim_height)."""
    if loc is not None and loc.get("rim_edge") == "front" and rim_from != "measured":
        loc = dict(loc, confidence=RIM_ONLY_CONF)
    return loc


def close_bias(r, bias, r_from=BIAS_FROM_R):
    """(forward, up) added to the close pose asked of the arm at reach r: the measured bias, (0, 0) nearer than it
    was measured."""
    return (float(bias[0]), float(bias[1])) if r >= r_from else (0.0, 0.0)


def tip_overshoot(kin, path):
    """How far the finger tips bulge outward past the start and end reaches of a joint path (m).

    Moving from one reach to another is the move itself. Only the bulge past both ends counts:
    that is the swing that pulled the bag off the wall on 2026-11-18.
    """
    if not path:
        return 0.0
    reaches = []
    for q in path:
        point = kin.gripper_point(q, (0.0, TIP_Y, 0.0))
        forward, left = kin.to_user(point)
        reaches.append(math.hypot(float(forward), float(left)))
    limit = max(reaches[0], reaches[-1])
    return max(0.0, max(reaches) - limit)


def line_seconds(n_points, speed, step=JOINT_STEP):
    """Seconds of one line() (pick_place_node.line: min-jerk peak = 1.875 x average)."""
    if n_points <= 0 or speed <= 0:
        return 0.0
    return 1.875 * float(n_points) * float(step) / float(speed)


def rim_height(top, fallback, lo=RIM_LO, hi=RIM_HI):
    """(metres, "measured" or "fallback"). The Pi top when it is finite and inside [lo, hi]."""
    try:
        h = float(top)
    except (TypeError, ValueError):
        return float(fallback), "fallback"
    if math.isfinite(h) and lo <= h <= hi:
        return h, "measured"
    return float(fallback), "fallback"


def wall_reach(tip_h, table=WALL_REACH, most=REACH_WALL):
    """Farthest front wall (m from the pan axis) the arm closes on with the finger tips tip_h over the floor:
    WALL_REACH, linear in between, its first value below it, REACH_WALL above its last height."""
    if tip_h <= table[0][0]:
        return table[0][1]
    for (h0, r0), (h1, r1) in zip(table, table[1:]):
        if tip_h <= h1:
            return min(most, r0 + (r1 - r0) * (tip_h - h0) / (h1 - h0))
    return most


def pose_outlier(offset_m, width_m, disagree=LIDAR_DISAGREE, width_max=WIDTH_MAX):
    """The camera centre is more than `disagree` from the LiDAR goal, or the rim is wider than a bag."""
    try:
        wide = width_m is not None and float(width_m) > width_max
    except (TypeError, ValueError):
        wide = False
    try:
        far = offset_m is not None and float(offset_m) > disagree
    except (TypeError, ValueError):
        far = False
    return bool(wide or far)


def look_offset(loc, goal):
    """Metres between a look's centre and the LiDAR goal, or None when the look has no centre."""
    if loc is None or loc.get("centre") is None:
        return None
    c = loc["centre"]
    return math.hypot(float(c[0]) - float(goal[0]), float(c[1]) - float(goal[1]))


def needs_relook(loc, goal):
    """True when this look should be taken again before the grasp trusts it."""
    if loc is None:
        return False
    return pose_outlier(look_offset(loc, goal), loc.get("width"))


def _confidence(loc):
    if loc is None:
        return 0.0
    try:
        return float(loc.get("confidence") or 0.0)
    except (TypeError, ValueError):
        return 0.0


def bag_fallen(loc, want):
    """The old hard error: not standing, or the rim is outside the bag_cam_width band.

    A rim wider than WIDTH_MAX is an outlier (re-look, then LiDAR), not this error.
    """
    if loc is None:
        return False
    width = loc.get("width")
    try:
        width_f = None if width is None else float(width)
    except (TypeError, ValueError):
        width_f = None
    if width_f is not None and width_f > WIDTH_MAX:
        return False
    off_band = width_f is not None and not (0.55 * want < width_f < 2.2 * want)
    return loc.get("standing") is False or off_band


def resolve_grasp(looks, goal, want):
    """Choose the grasp centre after one look, or a second look when the first is an outlier.

    `looks` is [first] or [first, second]. Each entry is a locate() dict or None.
    Returns xy, source ("camera", "request" or "lidar"), relook, loc, failure ("fallen" or None),
    confidence and offset. A fallen bag is an error. An outlier that is still an outlier after the
    second look uses the LiDAR goal.
    """
    goal = (float(goal[0]), float(goal[1]))
    first = looks[0] if looks else None
    relook = needs_relook(first, goal)
    loc = first
    if relook:
        loc = looks[1] if len(looks) > 1 else None
        if loc is None or needs_relook(loc, goal):
            kept = loc if loc is not None else first
            return {"xy": goal, "source": "lidar", "relook": True, "loc": kept, "failure": None,
                    "confidence": _confidence(kept), "offset": look_offset(kept, goal)}
    if loc is None:
        return {"xy": goal, "source": "request", "relook": False, "loc": None, "failure": None,
                "confidence": 0.0, "offset": None}
    if bag_fallen(loc, want):
        return {"xy": None, "source": "request", "relook": relook, "loc": loc, "failure": "fallen",
                "confidence": _confidence(loc), "offset": look_offset(loc, goal)}
    xy, source = choose_grasp_xy(loc["centre"], _confidence(loc), goal)
    return {"xy": xy, "source": "camera" if source == "camera" else "request", "relook": relook, "loc": loc,
            "failure": None, "confidence": _confidence(loc), "offset": look_offset(loc, goal)}


def cool_reason(temp_c, pick=PICK_TEMP_C, driver=DRIVER_TEMP_C):
    """Why a pick waits, or "" when the motors are cool enough to start. Never starts at `driver`."""
    if temp_c is None:
        return ""
    try:
        temp = float(temp_c)
    except (TypeError, ValueError):
        return ""
    if not math.isfinite(temp) or temp < pick:
        return ""
    return ("waiting: arm motors are %.0f C; a pick starts only below %.0f C "
            "(the driver refuses at %.0f C)" % (temp, pick, driver))


def wait_until_cool(read_temp, sleep, now, limit_s=300.0, pick=PICK_TEMP_C):
    """Wait while read_temp() is at or above `pick`. Returns (waited_s, reason, end_temp).

    A missing first reading does not wait (no status yet). A gap after a hot reading keeps waiting:
    the driver refuses a move at DRIVER_TEMP_C, and a pick starts only below `pick`. Raises Failed HOT
    when it is still hot after limit_s.
    """
    temp = read_temp()
    reason = cool_reason(temp, pick=pick)
    if not reason:
        return 0.0, "", temp
    start = now()
    while True:
        temp = read_temp()
        if temp is not None and float(temp) < pick:
            return now() - start, reason, float(temp)
        if now() - start >= limit_s:
            shown = float("nan") if temp is None else float(temp)
            raise Failed(reason + "; still %.0f C" % shown, "HOT")
        sleep(0.5)


def pick_result(done):
    """(code, detail) of a task function. A held bag the camera did not confirm is SUCCESS_UNVERIFIED."""
    if isinstance(done, tuple) and len(done) == 2 and done[0] in HELD_CODES:
        return done[0], done[1]
    return "SUCCESS", done or "เสร็จ"


# ---------------------------------------------------------------------- geometry (no ROS: unit-tested)
# The same three helpers are copied in base/jetson/front_camera.py. The two trees do not import each other.
def _lens(dist_model, dist):
    """("pinhole" or "fisheye", (k1, k2, k3, k4)). An unknown name is an error, not a silent pinhole."""
    model = "pinhole" if dist_model is None else str(dist_model)
    if model not in ("pinhole", "fisheye"):
        raise ValueError("unknown dist_model %r (expected \"pinhole\" or \"fisheye\")" % model)
    coeffs = (list(dist) if dist is not None else []) + [0.0, 0.0, 0.0, 0.0]
    return model, tuple(float(k) for k in coeffs[:4])


def _fish_r(theta, k):
    """OpenCV fisheye radius over f: theta * (1 + k1 theta^2 + k2 theta^4 + k3 theta^6 + k4 theta^8)."""
    t2 = theta * theta
    return theta * (1.0 + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3]))))


def _fish_theta(rd, k, iterations=20):
    """Ray angle whose fisheye radius over f is rd (rd >= 0). Newton, held inside [0, pi/2].

    OpenCV's model stops at 180 deg, so rd is clipped to pi/2. A derivative near zero, or a step
    that is not a finite number, stops the loop instead of dividing.
    """
    rd = float(rd)
    if not math.isfinite(rd) or rd <= 0.0:
        return 0.0
    rd = min(rd, math.pi / 2.0)
    theta = rd
    for _ in range(iterations):
        t2 = theta * theta
        f = _fish_r(theta, k) - rd
        df = 1.0 + t2 * (3.0 * k[0] + t2 * (5.0 * k[1] + t2 * (7.0 * k[2] + t2 * 9.0 * k[3])))
        if not math.isfinite(df) or abs(df) < 1e-14:
            break
        step = f / df
        if not math.isfinite(step):
            break
        theta = theta - step
        if theta < 0.0:
            theta = 0.0
        elif theta > math.pi / 2.0:
            theta = math.pi / 2.0
        if abs(step) < 1e-15:
            break
    return theta if math.isfinite(theta) else 0.0


class FrontCamera:
    """The fixed front camera from front_camera.json (HMC team, hmc-follow/front_camera.py): fx = fy,
    at (x, y, z) in the robot frame (x ahead, y left, z up from the floor, origin at the turning centre),
    R = Rz(yaw) Ry(pitch) Rx(roll) R0, R0 = OpenCV camera axes -> robot with all angles zero.
    dist_model "fisheye" (OpenCV k1..k4) is applied in ray and project; otherwise the lens is a plain pinhole."""

    R0 = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])

    def __init__(self, d):
        if not d.get("calibrated"):
            raise ValueError("front camera not calibrated")
        self.fx, self.cx, self.cy = float(d["fx"]), float(d["cx"]), float(d["cy"])
        # webcam_view captures MJPG at this size. fx is for these pixels (front_camera.json).
        self.width, self.height = int(d.get("width", 640)), int(d.get("height", 480))
        self.pos = np.array([float(d["x"]), float(d["y"]), float(d["z"])])
        self.mirror = bool(d.get("mirror", False))
        self.dist_model, self.dist = _lens(d.get("dist_model", "pinhole"), d.get("dist"))
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

    def for_image(self, image):
        """This camera, or a copy whose fx, cx, cy match a uniformly scaled frame. None if the aspect changed."""
        height, width = int(image.shape[0]), int(image.shape[1])
        if width == self.width and height == self.height:
            return self
        if self.width <= 0 or self.height <= 0:
            return None
        sx, sy = width / float(self.width), height / float(self.height)
        if abs(sx - sy) > 0.02:
            return None
        scale = (sx + sy) / 2.0
        twin = copy.copy(self)
        twin.fx = self.fx * scale
        twin.cx = (self.cx + 0.5) * scale - 0.5
        twin.cy = (self.cy + 0.5) * scale - 0.5
        twin.width, twin.height = width, height
        return twin

    def ray(self, u, v):
        if self.dist_model == "fisheye":
            mx = (u - self.cx) / self.fx * (-1.0 if self.mirror else 1.0)
            my = (v - self.cy) / self.fx
            rd = math.hypot(mx, my)
            if rd > 1e-12:
                scale = math.tan(min(_fish_theta(rd, self.dist), math.radians(89.0))) / rd
                mx, my = mx * scale, my * scale
            d = np.array([mx, my, 1.0])
        else:
            d = np.array([(u - self.cx) / self.fx * (-1.0 if self.mirror else 1.0), (v - self.cy) / self.fx, 1.0])
        d = self.rot @ d
        return d / np.linalg.norm(d)

    def project(self, p):
        pc = (np.asarray(p, float) - self.pos) @ self.rot
        if self.dist_model == "fisheye":
            r = math.hypot(float(pc[0]), float(pc[1]))
            if r > 1e-12:
                s = _fish_r(math.atan2(r, float(pc[2])), self.dist) / r
            else:
                s = 1.0 / float(pc[2])
            u = self.fx * pc[0] * s
            return (self.cx - u if self.mirror else self.cx + u), self.cy + self.fx * pc[1] * s
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


def rim_span(tops, rim_row, blob_top, blob_bottom, band=None):
    """(left column, right column) of the rim, or None.

    The widest row of a standing bag is often the foot. Projecting the blob's x0/x1 at rim height then
    reads 47-56 cm for an 18 cm bag. The rim is the columns whose top pixel is within `band` of the
    median top row (band defaults to max(6 px, 12 % of the blob height)). Fewer than two such columns:
    the width is unknown, and the fallen-bag check does not use the blob width.
    """
    if tops is None or len(tops) < 2:
        return None
    height = abs(float(blob_bottom) - float(blob_top))
    band = max(6.0, 0.12 * height) if band is None else float(band)
    keep = tops[np.abs(tops[:, 1] - float(rim_row)) <= band]
    if len(keep) < 2:
        return None
    return float(keep[:, 0].min()), float(keep[:, 0].max())


def reconcile_edges(centre_top, centre_bottom, agree=EDGE_AGREE, lost=EDGE_LOST):
    """The bag centre from the rim estimate and the foot estimate.

    centre_top / centre_bottom are (forward, left) metres, or None when that edge was not in the picture.

    method:
      neither: no centre, confidence 0.
      foot only: the foot, confidence 0.80 (the floor height is known).
      rim only: the rim, confidence RIM_ONLY_CONF (usual: the foot is out of the frame; the rim height is assumed
                and the edge may be the far rim, so it stays under CONF_MIN: the goal's x, y are used).
      both, spread <= agree: the mean. Confidence 0.98 - 0.20 * (spread / agree).
      both, spread > agree: the foot. Confidence 0.75 * max(0, 1 - (spread - agree) / (lost - agree)),
                so 0 once the estimates are `lost` metres apart (8 cm is already under CONF_MIN).

    At the 5 cm disagreement from the 19 Nov audit the foot is used and the confidence stays above the
    gate. spread is the distance between the two estimates (0 with only one).
    """
    top = None if centre_top is None else (float(centre_top[0]), float(centre_top[1]))
    bot = None if centre_bottom is None else (float(centre_bottom[0]), float(centre_bottom[1]))
    if top is None and bot is None:
        return {"centre": None, "method": "none", "spread": None, "confidence": 0.0}
    if top is None:
        return {"centre": bot, "method": "foot", "spread": 0.0, "confidence": 0.80}
    if bot is None:
        return {"centre": top, "method": "rim", "spread": 0.0, "confidence": RIM_ONLY_CONF}
    spread = math.hypot(top[0] - bot[0], top[1] - bot[1])
    if spread <= agree:
        ratio = (spread / agree) if agree else 0.0
        return {"centre": ((top[0] + bot[0]) / 2.0, (top[1] + bot[1]) / 2.0), "method": "mean",
                "spread": spread, "confidence": 0.98 - 0.20 * ratio}
    span = lost - agree
    fade = 0.0 if span <= 0 else max(0.0, 1.0 - (spread - agree) / span)
    return {"centre": bot, "method": "foot", "spread": spread, "confidence": 0.75 * fade}


def choose_grasp_xy(camera_xy, confidence, request_xy, min_conf=CONF_MIN):
    """(x, y) to grasp, and "camera" or "request".

    The camera wins only when it has a centre and confidence >= min_conf. Otherwise the request's x, y:
    the arm API still accepts an explicit goal, and a low-confidence picture must not move the grasp.
    """
    request = (float(request_xy[0]), float(request_xy[1]))
    if camera_xy is not None and confidence >= min_conf:
        return (float(camera_xy[0]), float(camera_xy[1])), "camera"
    return request, "request"


def _num(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _requested_pose(obj):
    """The goal the caller sent: x, y, yaw (None when the caller did not give one), height."""
    size = getattr(obj, "size", None)
    height = _num(size[2]) if size is not None and len(size) > 2 else None
    yaw = _num(getattr(obj, "yaw", None)) if getattr(obj, "yaw_given", True) else None
    return {"x": _num(getattr(obj, "x", None)), "y": _num(getattr(obj, "y", None)), "yaw": yaw, "height": height}


def _centre_beyond(point_xy, pan, half_depth):
    """Bag centre half_depth past a wall point, (forward, left) from the pan axis."""
    rel = np.asarray(point_xy, float) - np.asarray(pan[:2], float)
    nrm = float(np.linalg.norm(rel))
    if nrm < 1e-9:
        return None
    return rel + half_depth * rel / nrm


def _wall_forward(centre, half_depth):
    """Forward distance of the wall point that _centre_beyond would turn into `centre`."""
    c = np.asarray(centre, float)
    nrm = float(np.linalg.norm(c))
    if nrm <= half_depth + 1e-6:
        return None
    return float(c[0] * (nrm - half_depth) / nrm)


def _handle_left_at(cam, hx, hy, pan, half_depth, centre):
    """Left of the bag centre from the handle ray, read at the rim/foot wall depth.

    The handle centroid is above the rim. Intersecting that ray with the rim-height plane lands
    too far forward, by about (handle rise) / tan(pitch). The forward distance stays the rim/foot
    estimate; only the left comes from the handle.
    """
    wall_fwd = _wall_forward(centre, half_depth)
    if wall_fwd is None:
        return None
    d = cam.ray(hx, hy)
    if d[0] <= 1e-6:
        return None
    s = (float(pan[0]) + wall_fwd - float(cam.pos[0])) / float(d[0])
    if s <= 0:
        return None
    at = cam.pos + s * d
    shifted = _centre_beyond(at[:2], pan, half_depth)
    if shifted is None:
        return None
    return float(shifted[1])


def locate(cam, b, pan, half_depth, top_h, foot=None, lying_h=0.12):
    """The bag's centre from a front-camera blob (bag_vision.find_bag), the bag standing and facing the arm.
    pan: the arm's pan axis (x, y) in the camera's robot frame. Two ways, then reconcile_edges:
      bottom: foot = image row of the front wall's foot on the floor (bag_vision.foot_row; z = 0, known):
              centre half_depth beyond it.
      top:    the blob's top edge (median over its columns) is the front wall's rim, top_h above the floor
              (the camera looks steeply down on the front wall): centre half_depth beyond it. The usual case
              at pick distances, when the foot is out of the picture (less exact: top_h is per bag).
    When the blob has a handle centroid (the tuned model), the centre keeps this forward distance and
    takes its left from the handle ray at that depth. The handle sits above the rim, so the ray is not
    intersected with z = top_h. The rim width is rim_span of the top edge, not the blob's widest row, and
    the standing check stays on the rim. Returns dict(centre (forward, left) from the pan axis, from, aim
    "handle" or "rim", confidence, spread, standing True / False / None, top_seen_h, centre_top,
    centre_bottom, width) or None when no ray hits."""
    pan = np.asarray(pan, float)
    # every column's top (a gripper in front of the rim still hides those columns), across the middle
    t = b.get("all_tops", b["tops"])
    rim_u, rim_v = (b["x0"] + b["x1"]) / 2, float(np.median(t[:, 1]))
    fr = b.get("front_rim") if isinstance(b.get("front_rim"), dict) else None
    front = fr is not None and fr.get("kind") == "open"
    if front:                                  # the front wall's rim, not the outline's top (bag_vision.front_rim)
        rim_v = float(fr["row"])
    handle = b.get("handle") if isinstance(b.get("handle"), dict) else None
    hx = _num(None if handle is None else handle.get("cx"))
    hy = _num(None if handle is None else handle.get("cy"))
    out = {"standing": None, "aim": "rim"}
    point = _at_height(cam, rim_u, rim_v, top_h)
    if point is not None:
        out["centre_top"] = _centre_beyond(point[:2], pan, half_depth)
        span = rim_span(t, rim_v, float(np.min(t[:, 1])), float(b.get("y1", np.max(t[:, 1]))))
        if span is not None:
            a, c = _at_height(cam, span[0], rim_v, top_h), _at_height(cam, span[1], rim_v, top_h)
            if a is not None and c is not None:
                out["width"] = float(np.linalg.norm(a[:2] - c[:2]))     # m across the rim (log; lying check)
    if foot is not None:
        foot = _at_height(cam, (b["x0"] + b["x1"]) / 2, foot, 0.0)
        if foot is not None:
            rel = foot[:2] - pan
            n = rel / np.linalg.norm(rel)
            out["centre_bottom"] = rel + half_depth * n
            d = cam.ray(rim_u, rim_v)
            if abs(d[:2] @ n) > 1e-6:
                s = (rel @ n - (cam.pos[:2] - pan) @ n) / (d[:2] @ n)
                out["top_seen_h"] = float(cam.pos[2] + s * d[2])
                out["standing"] = out["top_seen_h"] > lying_h
    # The foot, when found, is the distance: the floor height is known. The blob's top is the far (back) rim
    # of an open bag seen from above, plus the segmenter's overshoot: on the robot it read 10 to 12 cm beyond
    # the front wall of bags taped at 20 and 25 cm (2026-11-24) and 6 to 14 cm beyond a 29 cm bag's foot
    # (2026-11-27), so averaging the two, or fading the foot out when they disagree, threw the foot away.
    # The rim stays for the standing check, the width and the log (spread). A foot farther than the blob's top
    # is not a foot: the blob is a piece of the bag (the arm across it; 41.8 cm read for a bag at 25, the
    # piece's top at 25.8), so then only the rim is left.
    top, bot = out.get("centre_top"), out.get("centre_bottom")
    if top is not None and bot is not None and bot[0] > top[0] + FOOT_PAST_TOP:
        out["foot_dropped"] = float(bot[0])
        out.pop("centre_bottom")
    if out.get("centre_bottom") is not None:
        edges = reconcile_edges(None, out["centre_bottom"])
        if out.get("centre_top") is not None:
            edges["spread"] = float(np.hypot(*(np.asarray(out["centre_top"]) - np.asarray(out["centre_bottom"]))))
    else:
        edges = reconcile_edges(out.get("centre_top"), None)
        if front and edges["centre"] is not None:
            edges = dict(edges, method="front rim", confidence=FRONT_RIM_CONF)
            out["rim_edge"] = "front"           # front_rim_trust reads this (a handle renames the method)
    if edges["centre"] is None:
        return None
    if hx is not None and hy is not None:
        left = _handle_left_at(cam, hx, hy, pan, half_depth, edges["centre"])
        if left is not None:
            edges = {"centre": (float(edges["centre"][0]), left), "method": "handle",
                     "spread": edges["spread"], "confidence": edges["confidence"]}
            out["aim"] = "handle"
    out["centre"] = edges["centre"]
    out["from"] = edges["method"]
    out["spread"] = edges["spread"]
    out["confidence"] = edges["confidence"]
    return out


# ---------------------------------------------------------------------- the pick
class CameraBag:
    def __init__(self, node):
        self.n = node
        self.kin = node.kin
        self.theta = 0.0

    # ------------------------------------------------------------------ helpers
    def p(self, name):
        return self.n.get_parameter(name).value

    def speed(self, kind):
        """rad/s of the fastest joint. The old constants, unless bag_cam_*_speed is set (0 or missing = the constant)."""
        name, default = {"free": ("bag_cam_free_speed", FREE_SPEED), "held": ("bag_cam_held_speed", HELD_SPEED),
                         "put": ("bag_cam_put_speed", PUT_SPEED), "lift": ("bag_cam_lift_speed", LIFT_SPEED)}[kind]
        try:
            value = float(self.p(name))
        except Exception:  # noqa: BLE001 (a node without the parameter keeps the old speed)
            return default
        return value if value > 0 else default

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

    def vision_model(self):
        """(model, tuned engine path) for find_bag. A missing or empty tuned file is coco, logged once."""
        try:
            model = str(self.p("bag_model") or "coco")
        except Exception:  # noqa: BLE001 (a node without the parameter stays on coco)
            model = "coco"
        try:
            engine = str(self.p("bag_tuned_engine") or "")
        except Exception:  # noqa: BLE001
            engine = ""
        chosen, path, reason = bv.resolve_bag_model(model, engine or None)
        if reason and reason != getattr(self, "_model_reason", None):
            self._model_reason = reason
            self.n.get_logger().warning(reason)
        return chosen, path

    def seen(self):
        """The bag in the front camera now (bag_vision.find_bag), or None (no frame / not seen)."""
        im = self.web()
        if im is None:
            return None
        model, path = self.vision_model()
        return bv.find_bag(im, model=model, tuned_engine=path)

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

    def tip(self, r, tz, pitch, roll, what, pull_in=0.0, speed=None, sim=None):
        """Finger tips r m out along the bag's bearing self.theta, tz high (base frame), the gap middle on the
        line; if out of reach, up to pull_in m closer (same bearing). sim: a joint dict to move on paper only
        (dry run of a sequence: updated in place, the arm does not move). speed None = speed("free")."""
        if speed is None:
            speed = self.speed("free")
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
                return path
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
            except Failed as exc:
                if exc.code != "UNREACHABLE":      # a stop, a driver fault, too hot: end here (review 2026-11-19)
                    raise
                continue
            try:
                self.tip(r, close_z + 0.10, pitch, roll, "เหนือถุง", sim=sim)
                return
            except Failed as exc:
                if exc.code != "UNREACHABLE":
                    raise
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
            for dz in DESCEND_DZ:
                self.tip(r, close_z + dz, pitch, roll, "", sim=sim)
            for dz in LIFT_DZ:
                self.tip(r, close_z + dz, pitch, roll, "", pull_in=0.05, sim=sim)
        except Failed:
            return False
        return True

    def descend_steps(self, r, cz, pitch, roll):
        return [(r, cz + dz, pitch, roll, 0.0) for dz in DESCEND_DZ]

    def lift_steps(self, r, cz, pitch, roll):
        return [(r, cz + dz, pitch, roll, 0.03) for dz in STRAIGHT_UP]

    def dry_steps(self, steps, sim):
        """tip(sim=) each step. The joint paths of the steps that stay in reach, stopping at the first miss."""
        out = []
        for r, tz, pitch, roll, pull in steps:
            try:
                out.append(self.tip(r, tz, pitch, roll, "", pull_in=pull, sim=sim))
            except Failed as exc:
                if exc.code != "UNREACHABLE":
                    raise
                break
        return out

    def _approach(self, r, cz, pitch, roll):
        """(pre_r, pre_z) of the pre-grasp above the handle, or None. Same order as above()."""
        for pre_r, pre_z in ((r - 0.04, 0.10), (r, 0.10), (r, 0.20)):
            if self.feasible(r, cz, pitch, roll, (pre_r, pre_z)):
                return pre_r, pre_z
        return None

    def plan_handle_lift(self, r, cz, pitch, roll):
        """One plan: pre-grasp above the handle, down onto it, then the straight lift.

        None when the way down is not all in reach (the caller uses the stepwise path). Otherwise
        (down_path, lift_path, lifted_to, planned_s). The gripper close is the only pause, between the
        two paths. The descent is one move when the tips stay within PATH_BOW of its ends, and the
        2 cm steps otherwise. The lift is one straight rise, sampled every 5 cm (the 2 cm steps when
        a 5 cm sample is out of reach).
        """
        start = {j: float(self.n.joints[j]) for j in ARM}
        chosen = self._approach(r, cz, pitch, roll)
        if chosen is None:
            return None
        pre_r, pre_z = chosen
        sim = dict(start)
        down_segs = self.dry_steps(
            [(pre_r, cz + pre_z, pitch, roll, 0.0), (r, cz + 0.10, pitch, roll, 0.0)], sim)
        if len(down_segs) < 2:
            return None
        grasp_from = dict(sim)
        direct = None
        try:
            direct = self.tip(r, cz, pitch, roll, "", sim=sim)
        except Failed as exc:
            if exc.code != "UNREACHABLE":
                raise
            sim = dict(grasp_from)
        if direct is not None and tip_overshoot(self.kin, direct) <= PATH_BOW:
            down_segs.append(direct)
        else:
            sim = dict(grasp_from)
            stepped = self.dry_steps(self.descend_steps(r, cz, pitch, roll), sim)
            if len(stepped) < len(DESCEND_DZ):
                return None
            down_segs.extend(stepped)
        grasp = dict(sim)
        lift_steps = [(r, cz + dz, pitch, roll, 0.03) for dz in LIFT_CONTINUOUS]
        lift_segs = self.dry_steps(lift_steps, sim)
        if len(lift_segs) < len(lift_steps):
            sim = dict(grasp)
            lift_segs = self.dry_steps(self.lift_steps(r, cz, pitch, roll), sim)
            lifted_to = 0.0 if not lift_segs else STRAIGHT_UP[len(lift_segs) - 1]
        else:
            lifted_to = LIFT_CONTINUOUS[len(lift_segs) - 1]
        down_path = [q for seg in down_segs for q in seg]
        lift_path = [q for seg in lift_segs for q in seg]
        planned = {"down_s": line_seconds(len(down_path), self.speed("free")),
                   "lift_s": line_seconds(len(lift_path), self.speed("lift"))}
        return down_path, lift_path, lifted_to, planned

    def held_clear(self, path, grip_h, grasp_xy, side=1.0, stood=None):
        """The bag hanging from the gap middle along a joint path (its wall in the jaws, the bag behind it along the
        closing axis, or in front of it for side -1: a back-wall grip): False when it is dragged on the floor (the
        grip moves sideways more than DRAG_MAX while the bag's bottom is still down) or swings into the robot body
        (the deck box) below the deck's top deeper than where it stood (the joints `stood`, default the path's start:
        the bag stood there, so the deck parameter is no tighter than that)."""
        deck = getattr(self.n, "deck", None)
        width = float(self.p("bag_cam_width"))
        depth = side * 2.0 * float(self.p("bag_cam_half_depth"))

        def into_deck(q):
            g = self.kin.gripper_point(q, (GAP_MID, TIP_Y, 0.0))
            if g[2] - grip_h >= deck[1][2] + 0.01:
                return 0.0
            a = self.kin.close_axis(q)[:2]
            if np.linalg.norm(a) < 1e-6:
                return 0.0
            u = a / np.linalg.norm(a)
            v = np.array([-u[1], u[0]])
            worst = 0.0
            for along in np.linspace(0.0, depth, 4):
                for across in np.linspace(-width / 2, width / 2, 5):
                    c = g[:2] + along * u + across * v
                    worst = max(worst, float(np.min(np.minimum(c - deck[0][:2], deck[1][:2] - c))) + 0.01)
            return worst

        ref = stood if stood is not None else (path[0] if path else None)
        allowed = into_deck(ref) if deck is not None and ref is not None else 0.0
        for q in path:
            g = self.kin.gripper_point(q, (GAP_MID, TIP_Y, 0.0))
            bottom = g[2] - grip_h
            if bottom < self.n.table_z + 0.01 and math.hypot(g[0] - grasp_xy[0], g[1] - grasp_xy[1]) > DRAG_MAX:
                return False
            if deck is not None and into_deck(q) > max(0.0, allowed) + 0.005:
                return False
        return True

    def carry_plan(self, r, cz, pitch, roll, sim=None, side=1.0):
        """(path, theta, pitch, reach) of ONE held move from the close pose (the arm now, or the joint dict sim): up,
        out to the carry side and the jaws tilted, to where the bag's bottom is bag_carry_bottom over the floor.
        Sides in CARRY_SIDES order, the least tilt first (CARRY_PITCHES), the shortest move of the reaches CARRY_R;
        the bag must stay clear of the floor and the deck all the way (held_clear). None when nothing works."""
        start = {j: float((sim or self.n.joints)[j]) for j in ARM}
        grip_h = cz - self.n.table_z
        want = self.n.table_z + float(self.p("bag_carry_bottom")) + grip_h
        grasp_xy = self.kin.gripper_point(start, (GAP_MID, TIP_Y, 0.0))[:2]
        saved = self.theta
        try:
            for bearing in CARRY_SIDES:
                theta = saved if bearing is None else math.radians(bearing)
                for tilt in CARRY_PITCHES:
                    jaw_pitch = pitch if tilt is None else tilt
                    best = None
                    for reach in CARRY_R:
                        self.theta = theta
                        try:
                            path = self.tip(reach, want, jaw_pitch, roll, "", sim=dict(start))
                        except Failed as exc:
                            if exc.code != "UNREACHABLE":
                                raise
                            continue
                        if ((best is None or len(path) < len(best[0]))
                                and self.held_clear([start] + path, grip_h, grasp_xy, side)):
                            best = (path, theta, jaw_pitch, reach)
                    if best:
                        return best
        finally:
            self.theta = saved
        return None

    def _remember_plan(self, mode, **seconds):
        """Keep the planned line() seconds on the trial and on the node (the pick JSON)."""
        motion = getattr(self, "last_motion", None) or {"mode": mode, "planned_s": {}}
        motion["mode"] = mode
        motion["planned_s"].update({k: round(float(v), 3) for k, v in seconds.items()})
        self.last_motion = motion
        self.n.planned_motion = motion
        trial = getattr(self, "_trial", None)
        if trial is not None:
            trial["motion"] = motion

    def _down_stepwise(self, r, cz, pitch, roll):
        """The previous approach: above the bag, then one short move per step down."""
        self.above(r, cz, pitch, roll)
        for dz in DESCEND_DZ:
            self.tip(r, cz + dz, pitch, roll, "ลงไปที่ถุง")

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

    def grasp_outward(self, r, close_z, pitch, close_dir, nearer=NEARER, allow_inward=True):
        """grasp_pose with the closing axis pointing away from the arm, moving the gap middle up to max(NEARER)
        nearer when that is out of reach at r (the open gap is 7.5 cm: the wall stays between the jaws); only
        then the other way round. Returns (pitch, roll, align, r used)."""
        for d in nearer:
            try:
                pt, rl, al = self.grasp_pose(r - d, close_z, pitch, close_dir, allow_inward=False)
            except Failed:
                continue
            if d:
                self.n.say(f"ท่าแนวหุบชี้ออกจากหุ่นเอื้อมไม่ถึงที่ {r * 100:.1f} ซม.: เลื่อนช่องก้ามเข้ามา {d * 100:.1f} ซม.")
            return pt, rl, al, r - d
        pt, rl, al = self.grasp_pose(r, close_z, pitch, close_dir, allow_inward=allow_inward)
        return pt, rl, al, r

    def grasp_pose(self, r, close_z, pitch, close_dir, allow_inward=True):
        """(pitch, roll deg, alignment) for a try. The closing axis must point AWAY from the arm (along the
        wall normal): on the robot, 2026-11-18, every close with it pointing back at the robot came back empty
        (8 of 8). Among those, the steepest pitch first: the jaws then close more square on the wall (alignment
        0.94 at -70 against 0.87 at -60; the closed jaws keep a 16 mm gap, so a 1 mm paper wall is held by the
        jaw tips' friction only). Only when no pitch gets there with the axis pointing away, the other way
        round (said in the log). Raises UNREACHABLE when nothing gets there."""
        lo, hi = (math.degrees(v) for v in self.kin.limits["wrist_roll"])
        min_align = float(self.p("bag_cam_min_align"))
        cands = []                                 # (pitch, roll, align, closing sign)
        for pt in sorted(dict.fromkeys((pitch - 10, pitch, pitch + 10))):     # steepest first
            try:
                roll, align = self.roll_for(r, close_z, pt, close_dir)
            except Failed:
                continue
            if align < min_align:                  # the jaws would pinch the wall slanted (a flatter pitch)
                continue
            for rl in (roll, roll + 180.0, roll - 180.0):
                if lo <= rl <= hi:
                    cands.append((pt, rl, align, self.closing_sign(r, close_z, pt, rl, close_dir)))
        for outward in ((True, False) if allow_inward else (True,)):
            for pt, rl, align, sign in cands:
                if (sign > 0) == outward and self.feasible(r, close_z, pt, rl):
                    if not outward:
                        self.n.say(f"ท่าแนวหุบชี้ออกจากหุ่นไปไม่ได้ที่ปลายก้าม {r * 100:.1f} ซม.: ใช้แนวกลับด้าน "
                                   "(เคยหุบเปล่า)")
                    return pt, rl, align
        raise Failed(f"เอื้อมไปหนีบขวางผนังหน้าถุงไม่ได้ (ปลายก้าม {r:.3f} ม. จากแกนหมุน สูง "
                     f"{close_z - self.n.table_z:.2f} ม. จากพื้น)", "UNREACHABLE")

    def place_left(self, start):
        """With the arm at the side: True = the place the bag stood is empty (the bag is in the jaws), False =
        the bag is still there (it slipped out), None = cannot tell (no calibration, no picture before or now,
        or the arm still in the picture). A missing picture is no evidence either way, and neither is a blob that
        is not a bag detection (trusted): False opens the jaws, so glare must not say "still there".
        self.hold_why says why a None could not tell. The picture is kept on the trial ("hold") even with the arm
        in it: in both slips on the robot (2026-11-24) the arm at the carry pose was in the picture, so the check
        never ran, and a check that can tell a held bag from a dropped one needs such pictures of both."""
        self.hold_why = ""
        if not trusted(start):
            self.hold_why = "ภาพก่อนหยิบไม่ใช่ถุงที่โมเดลหาเจอ"
            return None
        try:
            cam = FrontCamera.load(str(self.p("bag_front_cam_file")))
        except (OSError, ValueError, KeyError, TypeError):
            self.hold_why = "ไม่มีค่าคาลิเบรตกล้องหน้า"
            return None
        time.sleep(0.4)
        im = self._keep_frame("hold", self.web())
        if self.in_view(cam, self.n.joints):
            self.hold_why = "แขนท่าถือถุงบังภาพกล้องหน้า"
            return None
        if im is None:
            self.hold_why = "ไม่มีภาพจากกล้องหน้า"
            return None
        model, path = self.vision_model()
        now = bv.find_bag(im, model=model, tuned_engine=path)
        if not trusted(now):
            now = None
        floor = None
        if now is None and start.get("frame") is not None:
            floor = bv.spot_floor(start["frame"], im, (start["x0"], start["y0"], start["x1"], start["y1"]))
            self._note_spot("front", floor)
        left = spot_left(start, now, floor)
        if left is None:
            self.hold_why = "กล้องไม่เห็นถุง และที่เดิมยังดูไม่ออกว่าว่าง"
        return left

    def _note_spot(self, camera, floor):
        """Keep a spot_floor share on the trial JSON (the robot test compares it with the pictures)."""
        trial = getattr(self, "_trial", None)
        if trial is not None and floor is not None:
            trial.setdefault("spot_floor", {})[camera] = round(float(floor), 3)

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
            if 0 <= u < cam.width and 0 <= v < cam.height:
                return True
        return False

    def look(self, cam, top_h=None):
        """The arm out of the camera's way (LOOK) if it is in the picture, then a frame -> (blob, located).

        top_h is the rim height for the 3D estimate. None uses bag_cam_rim_h.
        """
        if cam is not None and self.n.joints and self.in_view(cam, self.n.joints):
            self.n.moveit_to(LOOK, "ยกแขนพ้นภาพกล้องหน้า (ดูถุง)")
            time.sleep(0.5)
        im = self._keep_frame("look", self.web())
        if im is None:
            self.n.say("ไม่มีภาพจากกล้องหน้า (webcam_view.py พอร์ต 8082)")
            return None, None
        model, path = self.vision_model()
        b = bv.find_bag(im, model=model, tuned_engine=path)
        if b is not None:
            b["frame"] = im                       # the hold check compares the bag's spot with this picture
            b["front_rim"] = bv.front_rim(im, b)
        if b is None or cam is None:
            return b, None
        if not trusted(b):
            self.n.say("กล้องหน้าหาถุงด้วยโมเดลไม่เจอ (เห็นแค่สีต่างจากพื้น): ไม่ใช้ระยะจากกล้อง")
            return b, None
        cam = cam.for_image(im)
        if cam is None:
            self.n.say("ภาพกล้องหน้าไม่ตรงขนาดคาลิเบรต (ต้องเป็นขนาดเดิม หรือสเกลเท่ากันทั้งสองแกน)")
            return b, None
        px, py = (float(v) for v in self.p("bag_front_cam_pan"))
        rim = float(self.p("bag_cam_rim_h") if top_h is None else top_h)
        loc = locate(cam, b, (px, py), float(self.p("bag_cam_half_depth")), rim, foot=bv.foot_row(im, b))
        if loc is not None:
            loc["rim_h"] = rim
        return b, loc

    def _tag_measure(self, relook):
        trial = getattr(self, "_trial", None)
        measured = None if not trial else trial.get("measured")
        if not isinstance(measured, dict):
            return
        measured["relook"] = bool(relook)
        rim = trial.get("rim_h")
        if isinstance(rim, dict):
            measured["rim_h_m"] = rim.get("m")
            measured["rim_h_source"] = rim.get("source")

    def where(self, obj, first=True):
        """Look at the bag (front camera: the 1st try and after every miss or drop) -> dict: centre (user f, l),
        wall point, normal, seen (blob), from. A confident camera centre (choose_grasp_xy) replaces the goal's
        x, y; a low confidence, or no bag on the first look, keeps the goal. No bag on a later look: an error
        (the bag is gone or hidden). A camera centre more than 10 cm from the Pi LiDAR goal, or a rim wider
        than 30 cm: look once more, then use the LiDAR goal and record the source. A bag that does not stand,
        or whose rim width is far from bag_cam_width (and not that too-wide outlier), is still an error.
        The rim height is the Pi's measured top when it is usable, else bag_cam_rim_h."""
        f, l = self.kin.to_user(np.array([obj.x, obj.y]))
        line = (obj.yaw - self.kin.heading0) if getattr(obj, "yaw_given", True) else math.pi / 2
        goal = (f, l)
        out = {"goal": goal, "from": "goal", "line": line, "seen": None, "confidence": 0.0}
        top_h, rim_from = rim_height(obj.size[2], float(self.p("bag_cam_rim_h")))
        trial = getattr(self, "_trial", None)
        if trial is not None:
            trial["rim_h"] = {"m": top_h, "source": rim_from}
        cam = None
        try:
            cam = FrontCamera.load(str(self.p("bag_front_cam_file")))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.n.say(f"ไม่มีค่าคาลิเบรตกล้องหน้า ({exc})")
        b, loc = self.look(cam, top_h)
        looks, seen = [loc], b
        if needs_relook(loc, goal):
            self.n.say("กล้องกับ LiDAR ห่างเกิน 10 ซม. หรือขอบถุงกว้างเกิน 30 ซม.: มองอีกครั้ง")
            b2, loc2 = self.look(cam, top_h)
            looks.append(loc2)
            if b2 is not None:
                seen = b2
        want = float(self.p("bag_cam_width"))
        looks = [front_rim_trust(one, rim_from) for one in looks]
        decision = resolve_grasp(looks, goal, want)
        out["seen"] = seen
        loc = decision["loc"]
        if loc is None and not decision["relook"]:
            self._note_measure(None, goal, "request", 0.0, None)
            self._tag_measure(False)
            if not first and (seen is None or trusted(seen)):
                raise Failed("ดูกล้องหน้าใหม่แล้วไม่เห็นถุง (ถุงหลุดไปไหน หรือมีอะไรบัง): หยุด", "MISSED")
            self.n.say("กล้องหน้าหาถุงไม่ได้: ใช้ตำแหน่งจากคำสั่งอย่างเดียว")
        elif decision["failure"] == "fallen":
            width = None if loc is None else loc.get("width")
            off = decision["offset"]
            self._note_measure(loc, goal, "request", decision["confidence"], off)
            self._tag_measure(decision["relook"])
            raise Failed(f"ถุงล้มหรือไม่ได้ตั้งหันหน้าเข้าหุ่น (กล้องเห็นขอบปากถุงกว้าง {(width or 0) * 100:.0f} "
                         f"ซม. ควรราว {want * 100:.0f}): ตั้งถุงใหม่แล้วสั่งอีกครั้ง", "FAILED")
        else:
            if loc is not None:
                c = loc["centre"]
                width, off = loc.get("width"), decision["offset"]
                spread = loc.get("spread")
                spread_s = float("nan") if spread is None else spread
                apart = float("nan") if off is None else off * 100
                self.n.get_logger().info(
                    f"front camera bag: centre fwd {c[0]:.3f} left {c[1]:+.3f} (from the {loc['from']}, "
                    f"spread {spread_s:.3f} m, confidence {decision['confidence']:.2f}; goal fwd {goal[0]:.3f} "
                    f"left {goal[1]:+.3f}, {apart:.1f} cm apart); rim width {width if width else float('nan'):.3f} m, "
                    f"standing {loc['standing']} (top seen {loc.get('top_seen_h', float('nan')):.2f} m, "
                    f"rim height {top_h:.2f} m from {rim_from}); source {decision['source']}; "
                    f"aim {loc.get('aim', 'rim')}"
                    + (" after a second look" if decision["relook"] else ""))
            f, l = decision["xy"]
            if decision["source"] == "camera":
                out["from"] = "image " + loc["from"]
            elif decision["source"] == "lidar":
                out["from"] = "lidar"
                self.n.say("กล้องยังไม่ตรงกับ LiDAR: ใช้ตำแหน่ง LiDAR")
            elif loc is not None:
                self.n.say(f"กล้องหน้าไม่มั่นใจพอ ({decision['confidence']:.2f}): ใช้ตำแหน่งจากคำสั่ง")
            out["confidence"] = decision["confidence"]
            self._note_measure(loc, (f, l), decision["source"], decision["confidence"], decision["offset"])
            self._tag_measure(decision["relook"])
        wall, normal = front_wall((f, l), line, float(self.p("bag_cam_half_depth")))
        out.update(centre=(f, l), wall=wall, normal=normal)
        return out

    def close_h(self, obj):
        """Finger-tip height above the floor at the close: bag_cam_grip_depth under the rim (the Pi's measured top
        when inside RIM_LO..RIM_HI, else bag_cam_rim_h). With no grip depth set, the old fixed bag_cam_close_h,
        lower for a bag lower than that."""
        try:
            depth = float(self.p("bag_cam_grip_depth"))
        except Exception:  # noqa: BLE001 (a node without the parameter keeps the fixed height)
            depth = 0.0
        if depth > 0:
            rim, _source = rim_height(obj.size[2], float(self.p("bag_cam_rim_h")))
            return max(0.05, rim - depth)
        h = float(self.p("bag_cam_close_h"))
        top = obj.size[2]
        if math.isfinite(top) and top < h + 0.03:
            h = max(0.05, top - 0.03)
        return h

    # ------------------------------------------------------------------ one JSON file per trial
    def _begin_trial(self, obj):
        self._trial_n = getattr(self, "_trial_n", 0) + 1
        now = time.time()
        self._clock = pt.PhaseClock()
        self._trial = {"id": pt.new_trial_id(now, os.getpid(), self._trial_n), "ict": pt.ict_timestamp(now),
                       "requested": _requested_pose(obj), "measured": None, "confidences": {}, "frames": [],
                       "success": False, "failure": None, "open": True}

    def _phase(self, name):
        clock = getattr(self, "_clock", None)
        if clock is not None:
            clock.phase(name)

    def _note_measure(self, loc, xy, source, confidence, offset):
        trial = getattr(self, "_trial", None)
        if not trial:
            return

        def pair(key):
            value = None if loc is None else loc.get(key)
            if value is None:
                return None
            return [float(value[0]), float(value[1])]

        trial["measured"] = {
            "centre": None if loc is None else [float(loc["centre"][0]), float(loc["centre"][1])],
            "centre_top": pair("centre_top"), "centre_bottom": pair("centre_bottom"),
            "method": None if loc is None else loc.get("from"),
            "spread": None if loc is None or loc.get("spread") is None else float(loc["spread"]),
            "confidence": float(confidence), "source": source,
            "used": [float(xy[0]), float(xy[1])],
            "offset_m": None if offset is None else float(offset),
            "width": None if loc is None or loc.get("width") is None else float(loc["width"]),
            "standing": None if loc is None else loc.get("standing"),
        }
        trial["confidences"]["pose"] = float(confidence)

    def _keep_frame(self, kind, image):
        """Save a JPEG next to the trial JSON and remember its file name. The image is returned either way."""
        trial = getattr(self, "_trial", None)
        if trial is None or image is None:
            return image
        try:
            import cv2
            directory = os.path.expanduser(str(self.p("bag_trial_dir")))
            os.makedirs(directory, exist_ok=True)
            stem = "%s_%s" % (trial["id"], kind)
            n = sum(1 for name in trial["frames"] if name.startswith(stem))
            filename = ("%s.jpg" % stem) if n == 0 else ("%s%d.jpg" % (stem, n + 1))
            if cv2.imwrite(os.path.join(directory, filename), image):
                trial["frames"].append(filename)
        except Exception:  # noqa: BLE001 (a frame that cannot be saved must not fail the pick)
            pass
        return image

    def _trial_failed(self, exc):
        trial = getattr(self, "_trial", None)
        if trial:
            trial["success"] = False
            trial["failure"] = "%s: %s" % (getattr(exc, "code", "ERROR"), exc)
        self._finish_trial()

    def _trial_ok(self):
        trial = getattr(self, "_trial", None)
        if trial:
            trial["success"] = True
            trial["failure"] = None
        self._finish_trial()

    def _finish_trial(self):
        trial = getattr(self, "_trial", None)
        if not trial or not trial.get("open"):
            return
        trial["open"] = False
        clock = getattr(self, "_clock", None)
        if clock is not None:
            clock.close()
        try:
            params = {}
            try:
                with open(os.path.expanduser(str(self.p("bag_front_cam_file"))), encoding="utf-8") as f:
                    params = json.load(f)
            except (OSError, ValueError, TypeError):
                params = {}
            record = pt.build_trial(
                trial["id"], trial["ict"], pt.read_commit(), pt.camera_params_hash(params),
                trial["requested"], trial["measured"], clock.phases if clock else {}, trial["confidences"],
                trial["success"], trial["failure"], trial["frames"])
            for key in ("motion", "result_code", "rim_h"):
                if trial.get(key) is not None:
                    record[key] = trial[key]
            path = pt.write_trial(record, os.path.expanduser(str(self.p("bag_trial_dir"))))
            self.n.get_logger().info("pick trial %s" % path)
        except Exception as exc:  # noqa: BLE001 (the pick result stands even when the trial file cannot be written)
            try:
                self.n.get_logger().warning("pick trial not written: %s" % exc)
            except Exception:  # noqa: BLE001
                pass

    # ------------------------------------------------------------------ pick
    def pick(self, obj):
        """obj: Box from the PickBag goal (x, y = roughly the bag's centre). Every try looks at the bag again.
        One JSON trial is written under bag_trial_dir (a disk error does not fail the pick)."""
        self._begin_trial(obj)
        try:
            text = self._pick_impl(obj)
        except Exception as exc:
            self._trial_failed(exc)
            raise
        self._trial_ok()
        return text

    def _pick_impl(self, obj):
        """obj: Box from the PickBag goal (x, y = roughly the bag's centre). Every try looks at the bag again
        (the last one may have pushed it), plans the grasp there, closes, lifts, checks; a miss or a drop:
        open, look again, next try. Returns the result text (holding)."""
        if not self.n.joints:
            raise Failed("ไม่รู้ท่าแขน (ไม่มี joint_states)", "FAILED")
        tries = TRIES[:max(1, int(self.p("bag_cam_tries")))]
        hover = bool(self.p("bag_hover_only"))
        closed, why = 0, "ถุงไม่ลอยจากพื้น"
        close_z = self.n.table_z + self.close_h(obj)
        need_look = True
        w = None
        for i, (dt, pitch) in enumerate(tries):
            if need_look:
                self._phase("look")
                self.n.progress(1, 4, "look", f"ลองที่ {i + 1}/{len(tries)}: ดูถุงจากกล้องหน้า...")
                w = self.where(obj, first=(i == 0))
                need_look = False
            start = w["seen"]
            normal = np.asarray(w["normal"], float)
            front = np.asarray(w["wall"], float)
            close_dir = self.user_dir(normal)
            fl = w["centre"]
            half = float(self.p("bag_cam_half_depth"))
            # a low bag must be nearer: the arm reaches less far the lower it closes (WALL_REACH; the shallower
            # grip below adds at most 1.5 cm of height)
            reach = wall_reach(close_z - self.n.table_z + SHALLOWER[-1])
            r_front = float(np.hypot(*front))
            self.n.get_logger().info(
                f"camera bag try {i + 1}: centre fwd {fl[0]:.3f} left {fl[1]:+.3f} (from {w['from']}, goal fwd "
                f"{w['goal'][0]:.3f} left {w['goal'][1]:+.3f}); handles' line {math.degrees(w['line']):.0f} deg; "
                f"front wall fwd {front[0]:.3f} left {front[1]:+.3f}, {r_front:.3f} m from the pan axis; tips close "
                f"{close_z - self.n.table_z:.3f} m above the floor")
            if r_front > reach:
                raise Failed(
                    "move closer: the bag front is %.2f m from the arm base; the arm reaches %.2f m "
                    "(ขยับหุ่นเข้าใกล้ถุง)" % (r_front, reach), "UNREACHABLE")
            plan, last = None, None
            for which in WALLS:
                wall = front + (2.0 * half * normal if which == "back" else 0.0)
                side = -1.0 if which == "back" else 1.0     # the bag's body: in front of the gripped wall or behind
                theta = math.atan2(wall[1], wall[0])
                r_wall = float(np.hypot(*wall))
                if r_wall > reach:
                    continue
                self.theta = theta
                r = r_wall + dt
                bias_f, bias_up = close_bias(r, self.p("bag_cam_close_bias"))
                # where the gap middle should end up (log), then the pose asked of the arm: the measured bias of the
                # close pose taken off (bag_cam_close_bias)
                want = (r * math.cos(theta), r * math.sin(theta), close_z - self.n.table_z)
                rb, cz0 = r + bias_f, close_z + bias_up
                for up in SHALLOWER:               # a low bag further away: grip a little higher (shallower)
                    try:
                        # the back wall: no shift nearer (that puts the inner jaw on the front wall: a push)
                        # (nor the jaws turned round: the jaw nearer the robot would close on the front wall)
                        pt_, roll, align, rr = self.grasp_outward(rb, cz0 + up, pitch, close_dir,
                                                                  (0.0,) if which == "back" else NEARER,
                                                                  allow_inward=which != "back")
                    except Failed as exc:
                        if exc.code != "UNREACHABLE":
                            raise
                        last = exc
                        continue
                    # the held move must exist before the jaws close on the bag (from the close pose, on paper)
                    close_q = {j: float(self.n.joints[j]) for j in ARM}
                    try:
                        self.tip(rr, cz0 + up, pt_, roll, "", sim=close_q)
                    except Failed as exc:
                        if exc.code != "UNREACHABLE":
                            raise
                        continue
                    carry = self.carry_plan(rr, cz0 + up, pt_, roll, sim=close_q, side=side)
                    if carry is None:
                        last = Failed(f"ยกถุงไปถือไม่ได้เมื่อหนีบผนัง{'หลัง' if side < 0 else 'หน้า'} (ทุกด้านชนตัวหุ่น"
                                      "หรือเอื้อมไม่ถึง)", "UNREACHABLE")
                        continue
                    plan = (which, side, theta, want, pt_, roll, align, rr, cz0 + up, cz0, carry)
                    break
                if plan:
                    break
            if plan is None:
                why = str(last) if last else "เอื้อมไม่ถึงผนังถุงทั้งสองด้าน"
                if i + 1 < len(tries):
                    self.n.say(f"ลองที่ {i + 1}: {why} — ข้าม")
                    continue
                raise last if last else Failed(why, "UNREACHABLE")
            which, side, self.theta, want, pitch, roll, align, r, cz, cz0, carry = plan
            if cz > cz0:
                self.n.say(f"หนีบตื้นขึ้น {(cz - cz0) * 100:.1f} ซม. (เอื้อมไม่ถึงที่ความลึกปกติ)")
            self.n.get_logger().info(f"camera try {i + 1}: the {which} wall, tips {r:.3f} m out, bearing "
                                     f"{math.degrees(self.theta):+.1f} deg, pitch {pitch}, roll {roll:+.1f} deg "
                                     f"(closing axis vs wall normal {align:.2f})")
            self.n.gripper(0.9, "อ้าก้าม")
            need_look = True
            # One plan: above the handle, down onto it, then (only pause: the gripper close) the straight lift.
            # The dry run is the same tip(sim=) sequence as the old steps. A plan that cannot reach the handle
            # falls back to those steps.
            planned = self.plan_handle_lift(r, cz, pitch, roll)
            # the Xtion (under the deck) before and after the pick, kept for the record: does the bag leave the floor
            xtion_before = self._keep_frame("xtion_before", self.xtion())
            self._phase("down")
            self.n.progress(2, 5, "down", f"ลองที่ {i + 1}/{len(tries)}: เล็งที่หูถุงแล้วลงจับในท่าเดียว (ทิศ "
                            f"{math.degrees(self.theta):+.0f}°, ปลายก้าม {r * 100:.1f} ซม., หมุนก้าม {roll:+.0f}°)")
            try:
                if planned is None:
                    self.n.get_logger().info("continuous handle plan failed; using the stepwise path")
                    self._remember_plan("stepwise")
                    self._down_stepwise(r, cz, pitch, roll)
                else:
                    down_path, seconds = planned[0], planned[3]
                    self._remember_plan("continuous", down_s=seconds["down_s"])
                    self.n.line(down_path, "ลงไปที่ถุง", self.speed("free"), JOINT_STEP)
            except Failed as exc:
                if exc.code != "UNREACHABLE":
                    raise
                raise Failed(f"ถุงอยู่ไกลหรือใกล้เกินเอื้อม ({exc}): ขยับหุ่น", "UNREACHABLE")
            report = self.n.grasp_report(f"camera try {i + 1} at the close", want)
            if hover:
                self.n.hold_still = "ทดสอบ bag_hover_only: ค้างไว้ให้วัด"
                raise Failed(f"bag_hover_only: หยุดค้าง อ้าก้ามไว้ที่ระดับหนีบ ({report})", "STOPPED")
            self._phase("grasp")
            self.n.progress(3, 5, "grasp", "หนีบ")
            res = self.n.gripper(0.0, "หนีบ")
            closed += 1
            self.n.grip_reading = (float(res.position), float(res.effort))
            self.n.get_logger().info(f"camera grip {i + 1}: position {res.position:.3f} rad, "
                                     f"load {res.effort:.1f} %, stalled {res.stalled}")
            # ONE held move: up, out to the side and the jaws tilted a little (carry_plan, planned before the
            # close). No straight lift first and no stop on the way: on the robot (2026-11-20) the bag held in
            # every pick this way, and the bag is checked once, from the side, below.
            self._phase("carry")
            phase_log = getattr(self.n, "phase_log", None)
            if phase_log is not None:
                phase_log.mark("carry")
            self.n.progress(4, 5, "carry", "ยก หันไปด้านข้าง และเงยก้ามในท่าเดียว...")
            path, hold_th, hold_pt, hold_r = self.carry_plan(r, cz, pitch, roll, side=side) or carry
            grip_h = cz - self.n.table_z           # the bag's bottom hangs this far below the tips
            top = self.n.table_z + float(self.p("bag_carry_bottom")) + grip_h
            held = {"mode": "camera", "camera": True, "theta": hold_th, "roll": roll, "pitch": pitch, "r": r,
                    "hold_r": hold_r, "hold": (hold_pt, top), "verified": False, "close_z": cz, "side": side,
                    "grasp_q": {j: float(self.n.joints[j]) for j in ARM},
                    "grasp_theta": self.theta, "object": (obj.x, obj.y, obj.yaw, obj.size)}
            self.n.holding = held                # from here on the arm never folds or forgets it by itself
            self._remember_plan(self.last_motion["mode"] if getattr(self, "last_motion", None) else "stepwise",
                                carry_s=line_seconds(len(path), self.speed("held")))
            self.n.line(path, "ยกถุงไปถือด้านข้าง (หลบ LiDAR)", self.speed("held"), JOINT_STEP)
            self.theta = hold_th
            xtion_floor = bv.spot_floor(xtion_before, self._keep_frame("xtion_after", self.xtion()))
            self._note_spot("xtion", xtion_floor)
            self.n.get_logger().info(f"xtion spot {i + 1}: share no longer the bag {xtion_floor} (record only)")
            self._phase("check")
            self.n.progress(5, 5, "check", "เช็กด้วยกล้องว่าถุงพ้นจากที่เดิม...")
            # Held up at the side: the bag must have left its place on the floor (ALWAYS checked: a slip in the
            # raise). Checked from here, the arm out of the front camera's picture: in front it hides that place
            # and the old check read "nothing to compare" (2026-11-19), or saw only the wall stretched up.
            gone = self.place_left(start)
            self.n.get_logger().info(f"front camera hold check {i + 1}: bag left its place: {gone}")
            if gone is False:
                self.n.holding = None
                why = "ยกขึ้นแล้วถุงหลุด (ยังอยู่ที่เดิมบนพื้น)"
                self.n.say(f"ลองที่ {i + 1}: ถุงยังอยู่ที่เดิมบนพื้น (หลุด) — อ้าก้าม ดูกล้องใหม่แล้วลองอีกครั้ง")
                self.n.gripper(0.9, "อ้าก้าม")
                continue
            verified = held["verified"] = bool(gone)
            if verified:
                text = f"ถือถุงไว้ด้านข้าง ปลายก้ามสูงจากพื้น {top - self.n.table_z:.2f} ม. (ลองที่ {i + 1})"
            else:
                # not "holding the bag": on the robot (2026-11-24) both picks that said so had dropped it
                text = (f"หนีบแล้วยกขึ้น ปลายก้ามสูงจากพื้น {top - self.n.table_z:.2f} ม. แต่ยืนยันไม่ได้ว่าถุงติดก้ามมา "
                        f"({getattr(self, 'hold_why', '') or 'กล้องตรวจไม่ได้'}) (ลองที่ {i + 1})")
            code = "SUCCESS" if verified else "SUCCESS_UNVERIFIED"
            trial = getattr(self, "_trial", None)
            if trial is not None:
                trial["result_code"] = code
                trial["hold_check"] = "verified" if verified else (getattr(self, "hold_why", "") or "unknown")
            if code == "SUCCESS":
                return text
            return (code, text)
        raise Failed(f"จับถุงไม่ได้ใน {len(tries)} ครั้ง (หนีบ {closed} ครั้ง; ล่าสุด: {why})", "MISSED")

    # ------------------------------------------------------------------ put down
    def place(self, target):
        """Put the held bag down at target (Box, base frame x, y) and let go: ONE held move from the carry pose to 8 cm
        over the release height at the target (the grasp pitch, or a little flatter when that is out of reach), the
        bag kept clear of the floor and the deck; straight down until the tips are PUT_DROP over the grasp height
        (the bag's bottom on the floor); open; back off."""
        held = self.n.holding
        roll = held["roll"]
        f, l = self.kin.to_user(np.array([target.x, target.y]))
        new = math.atan2(l, f)
        # the jaws hold the FRONT wall: tips half a bag before the target, so the bag's centre lands on it (the
        # tips at the target put the bag 10-17 cm too far, 2026-11-18)
        side = held.get("side", 1.0)
        r = max(0.20, min(0.32, math.hypot(f, l) - side * float(self.p("bag_cam_half_depth"))))
        put_z = held.get("close_z", self.n.table_z + float(self.p("bag_cam_close_h"))) + PUT_DROP
        grip_h = held.get("close_z", put_z) - self.n.table_z
        gp = held.get("pitch", -60)
        self.n.progress(1, 3, "over", "ถือถุงไปเหนือจุดวาง...")
        start = {j: float(self.n.joints[j]) for j in ARM}
        no_drag = (math.inf, math.inf)           # the bag is off the floor here: only the deck matters
        # one move over the spot; when that would swing the bag into the deck on the way (a back-wall grip: the bag
        # hangs towards the robot), first turn towards the spot at the carry height and reach (the bag's inner edge
        # stays off the deck), then in and down to it
        hold_pt, hold_tz = held.get("hold", (gp, put_z + 0.08))
        hold_r = max(r, held.get("hold_r", r))
        over = None
        for two in (False, True):
            for pp in PUT_PITCHES:
                jaw_pitch = gp if pp is None else pp
                self.theta = new
                sim = dict(start)
                try:
                    turn = self.tip(hold_r, hold_tz, hold_pt, roll, "", pull_in=0.06, sim=sim) if two else []
                    path = self.tip(r, put_z + 0.08, jaw_pitch, roll, "", pull_in=0.06, sim=sim)
                except Failed as exc:
                    if exc.code != "UNREACHABLE":
                        raise
                    continue
                if self.held_clear([start] + turn + path, grip_h, no_drag, side, stood=held.get("grasp_q")):
                    over = (turn, path, jaw_pitch)
                    break
            if over:
                break
        if over is None:
            raise Failed(f"ถือถุงไปเหนือจุดวางไม่ได้ (ปลายก้าม {r:.3f} ม. จากแกนหมุน)", "UNREACHABLE")
        turn, path, jaw_pitch = over
        if turn:
            self.n.line(turn, "หันถุงไปเหนือจุดวาง", self.speed("put"), JOINT_STEP)
        self.n.line(path, "ถือถุงไปเหนือจุดวาง", self.speed("put"), JOINT_STEP)
        self.n.progress(2, 3, "lower", "วางลง...")
        placed_r = r
        for cand_r in (r, held.get("r", r), r + 0.03, r + 0.06):
            try:
                self.tip(cand_r, put_z, jaw_pitch, roll, "วางถุงลงพื้น", pull_in=0.05, speed=self.speed("put"))
                placed_r = cand_r
                break
            except Failed as exc:
                if exc.code == "COLLISION":
                    # the arm stalled on the way straight down: the bag's bottom is on the floor already (its wall
                    # slid down the jaws on the carry). That is the touchdown PUT_DROP 0 asks for: let go here
                    # (robot 2026-11-20: shoulder_lift 9 and 8 deg short, the bag kept pressed on the floor). The
                    # gripper command clears the driver's fault hold.
                    self.n.say(f"ก้นถุงถึงพื้นก่อนถึงระดับวาง ({exc}): ปล่อยตรงนี้")
                    placed_r = cand_r
                    break
                if exc.code != "UNREACHABLE":
                    raise
        else:
            raise Failed(f"วางถุงลงพื้น: แขนไปไม่ถึง (ปลายก้าม {r:.3f} ม. สูง {put_z:.3f})", "UNREACHABLE")
        self.n.gripper(0.9, "ปล่อย")
        self.n.holding = self.n.grasped = None
        self.n.progress(3, 3, "retreat", "ถอยออกแล้วกลับท่า ready...")
        # up and out of the bag: back towards the robot (a front-wall grip) or away from it (a back-wall grip: the
        # tall bag's mouth is in the way nearer the robot, and a high pose that near is out of reach)
        for back_r, up, back_pt in ((placed_r - 0.03 * side, 0.12, jaw_pitch), (placed_r, 0.10, jaw_pitch),
                                    (placed_r + 0.03 * side, 0.08, jaw_pitch), (placed_r, 0.08, -45.0)):
            try:
                self.tip(back_r, put_z + up, back_pt, roll, "ถอยขึ้น", pull_in=0.05)
                break
            except Failed as exc:
                if exc.code != "UNREACHABLE":
                    raise
        else:
            self.n.say("ถอยก้ามขึ้นจากถุงไม่ได้ (เอื้อมไม่ถึง): ก้ามอ้าค้างไว้ที่ถุง")
        return "วางถุงแล้ว"
