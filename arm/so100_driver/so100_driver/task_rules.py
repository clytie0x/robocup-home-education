"""Rules of the task server (pick_place_node) and the driver (driver_node) that need no ROS.

Tested on Windows (test/test_rules.py); the nodes only call them.
"""
import ast
import math


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

API_CODES = {PATH_TOLERANCE_VIOLATED: 'COLLISION', GOAL_TOLERANCE_VIOLATED: 'COLLISION', ARM_FAULT: 'FAILED',
             ARM_HOT: 'HOT', ARM_REPLACED: 'REPLACED', ARM_STOPPED: 'STOPPED', ARM_BUSY: 'BUSY'}


def api_code(error_code):
    """Driver FollowJointTrajectory error_code -> API error code (anything else: FAILED)."""
    return API_CODES.get(int(error_code), 'FAILED')


# One commander at a time. rclpy does not tell an action server which client sent a goal, so the goal
# names its commander in trajectory.header.frame_id: pick_place_node's straight lines send PICK_PLACE,
# MoveIt sends its planning frame ('base', also for RViz's MoveIt moves), most scripts send ''.
PICK_PLACE = 'so100_pick_place'
BUSY_TEXT = 'arm busy: another program has it'


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
    if 'load%=' not in status:
        return math.nan
    try:
        loads = ast.literal_eval(status.split('load%=', 1)[1].split(']', 1)[0] + ']')
    except (ValueError, SyntaxError):
        return math.nan
    return float(loads[5]) if len(loads) == 6 else math.nan
