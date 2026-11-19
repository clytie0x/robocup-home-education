"""Rules of the task server (pick_place_node) and the driver (driver_node) that need no ROS.

Tested on Windows (test/test_rules.py); the nodes only call them.
"""
import ast
import json
import math
import os


class Failed(Exception):
    """A task step failed. code = its API error code (the error_code list in so100_interfaces/action),
    always given where it is raised: nothing is guessed from the words of the message."""

    def __init__(self, msg, code):
        super().__init__(msg)
        self.code = code


# FollowJointTrajectory result error_code values the driver sends. -4 and -5 are control_msgs' own; the
# others are this driver's (control_msgs leaves other values free), so the task server reads why a move
# ended as a value.
PATH_TOLERANCE_VIOLATED = -4   # a joint could not follow: blocked / collision / overload
GOAL_TOLERANCE_VIOLATED = -5   # a joint stopped short of the end
ARM_FAULT = -100               # lost contact with the servos, or a servo switched its torque off itself
ARM_HOT = -101                 # a servo is too hot: refused, or stopped by the heat check
ARM_REPLACED = -102            # another command took the arm (a newer trajectory, the joint sliders)
ARM_STOPPED = -103             # /so100/stop or torque off
ARM_BUSY = -104                # refused: another program's trajectory runs (see may_take_arm)

API_CODES = {PATH_TOLERANCE_VIOLATED: "COLLISION", GOAL_TOLERANCE_VIOLATED: "COLLISION", ARM_FAULT: "FAILED",
             ARM_HOT: "HOT", ARM_REPLACED: "REPLACED", ARM_STOPPED: "STOPPED", ARM_BUSY: "BUSY"}


def api_code(error_code):
    """Driver FollowJointTrajectory error_code -> API error code (anything else: FAILED)."""
    return API_CODES.get(int(error_code), "FAILED")


# One commander at a time. rclpy does not tell an action server which client sent a goal, so the goal
# names its commander in trajectory.header.frame_id: pick_place_node's straight lines send PICK_PLACE,
# MoveIt sends its planning frame ('base', also for RViz's MoveIt moves), most scripts send ''.
PICK_PLACE = "so100_pick_place"
BUSY_TEXT = "arm busy: another program has it"


def may_take_arm(running, new):
    """May a trajectory from commander `new` start while one from `running` runs (None = nothing runs)?
    The same commander may replace its own goal; the task server (PICK_PLACE) always may."""
    return running is None or new == running or new == PICK_PLACE


def still_held(gripper_pos, miss_below, jaw_open=None):
    """After a failed or stopped task: is something still in the jaws? Closed on air the gripper reads
    0.031 to 0.046 rad (2026-11-15), so a reading above the miss threshold means something keeps them open.
    Jaws read within 0.05 rad of their open angle `jaw_open` were opened, not held open. No reading (no
    joint_states) counts as not held."""
    if gripper_pos is None or miss_below is None:
        return False
    return miss_below < gripper_pos and (jaw_open is None or gripper_pos < jaw_open - 0.05)


def allowed_while_hot(target_rad, present_rad):
    """While a servo is hot every command is refused except opening the gripper (it lowers the load).
    Gripper angle: larger = wider open (ik.jaw_gap)."""
    return target_rad > present_rad


def gripper_load(status):
    """Gripper servo (ID 6, the last of the six) load in % from the driver's so100/status line
    ('... load%=[l1, l2, l3, l4, l5, l6]'), NaN when it is not there."""
    if "load%=" not in status:
        return math.nan
    try:
        loads = ast.literal_eval(status.split("load%=", 1)[1].split("]", 1)[0] + "]")
    except (ValueError, SyntaxError):
        return math.nan
    return float(loads[5]) if len(loads) == 6 else math.nan


# ---- touch probe for a bag top of unknown height (PickBag grasp_pose z = NaN)
# The closed jaws go straight down over the bag in small steps. A push up on the finger tips turns the joints
# by (force x horizontal distance from their axis to the tips): over a bag 22-34 cm in front, shoulder_lift
# (lever 19-30 cm) and elbow_flex (8-20 cm) carry it; wrist_flex has the shortest lever (0-7 cm; test_bag).
PROBE_JOINTS = ("shoulder_lift", "elbow_flex")
PROBE_SERVOS = (2, 3)        # their servo IDs = places 2 and 3 in the driver's load% list (calib.IDS)


def top_unknown(z):
    """PickBag grasp_pose z NaN = the caller does not know the height of the bag top: find it by touch.
    Every real z is used as given (as before)."""
    return math.isnan(float(z))


def gripper_camera_calibrated(path):
    """True when `path` is a gripper_camera.json the calibrator wrote (it has T_gripper_camera).

    A missing or broken file is not a calibration: the 0.02 m/s touch probe still runs.
    """
    try:
        with open(os.path.expanduser(path)) as f:
            data = json.load(f)
    except (OSError, ValueError, UnicodeError):
        return False
    return isinstance(data, dict) and "T_gripper_camera" in data


def descent_loads(status):
    """Load % of the PROBE_JOINTS from the driver's so100/status line ('... load%=[l1, ..., l6]'),
    NaN where it is not there."""
    nan = (math.nan,) * len(PROBE_SERVOS)
    if "load%=" not in status:
        return nan
    try:
        loads = ast.literal_eval(status.split("load%=", 1)[1].split("]", 1)[0] + "]")
    except (ValueError, SyntaxError):
        return nan
    return tuple(float(loads[i - 1]) for i in PROBE_SERVOS) if len(loads) == 6 else nan


def probe_contact(samples, base_steps, load_jump, error_jump, skip=0):
    """First probe step (index into samples) at which the closed jaws touched something, or None.
    samples: one (loads, errors) per step, read with the arm standing still at the end of the step: load %
    and tracking error (rad, |commanded - present|) of each PROBE_JOINT. The first `skip` steps are the arm
    still settling after the MoveIt move to the start (real run 2026-11-15: shoulder 13, 12, then 2-4 %); the
    next base_steps steps are in free air (the probe starts above the tallest bag) and their mean per joint is
    the baseline. Contact: a joint's load is more than load_jump % away from its baseline (either sign: a push
    from below takes weight off a joint), or its tracking error is more than error_jump rad above its baseline.
    A threshold <= 0 switches that check off. NaN readings never count; skipped and baseline steps are not
    judged."""
    skip, base_steps = max(0, int(skip)), max(1, int(base_steps))
    first = skip + base_steps
    if len(samples) <= first:
        return None

    def baseline(kind):
        out = []
        for j in range(len(samples[0][kind])):
            vals = [s[kind][j] for s in samples[skip:first] if math.isfinite(s[kind][j])]
            out.append(sum(vals) / len(vals) if vals else math.nan)
        return out

    base_load, base_err = baseline(0), baseline(1)
    for i in range(first, len(samples)):
        loads, errors = samples[i]
        for j in range(len(loads)):
            if load_jump > 0 and abs(loads[j] - base_load[j]) > load_jump:
                return i
            if error_jump > 0 and errors[j] - base_err[j] > error_jump:
                return i
    return None


def settled_load(readings, tol):
    """A load reading that counts: the mean of the last two readings (PROBE_JOINTS loads %, one status line
    each) when every joint changed by at most tol % between them, else None (still settling: on 2026-11-15
    the shoulder read 14, 11, 8 after a MoveIt move). NaN never settles."""
    if len(readings) < 2:
        return None
    a, b = readings[-2], readings[-1]
    if all(math.isfinite(x) and math.isfinite(y) and abs(y - x) <= tol for x, y in zip(a, b)):
        return tuple((x + y) / 2 for x, y in zip(a, b))
    return None


def held_by_weight(free, loaded, load_jump):
    """Did a bag grasp whose jaws read closed (a thin rim) lift the bag? free / loaded: PROBE_JOINTS loads %
    at the same pose, both arriving there from below (servo friction: the shoulder read ~8 % more after rising
    into a pose than after lowering into it, 2026-11-15), jaws empty before going down and closed after coming
    back up, each settled (settled_load). Held = some joint's load
    changed by more than load_jump % (the bag's weight on its lever). NaN never counts, load_jump <= 0 = off.
    Returns (held, per-joint change)."""
    change = tuple(abs(b - a) for a, b in zip(free, loaded))
    return load_jump > 0 and any(math.isfinite(c) and c > load_jump for c in change), change
