"""Numbers from the arm stack, and placeholders where the robot was not measured.

A comment that says PLACEHOLDER is not a measurement and not a datasheet value.
"""

import math

# mount:=robot. arm/so100_moveit_config/launch/arm.launch.py
FLOOR_Z = -0.331
DECK_Z = -0.068
# launch `deck` default "0.18,0.22,0.18,0.22" (front, back, left, right).
# The launch file says the back edge is a guess. pick_place_node's own default is 0,0,0,0;
# mount:=robot passes the launch value.
DECK_FRONT, DECK_BACK, DECK_LEFT, DECK_RIGHT = 0.18, 0.22, 0.18, 0.22

# pick_place_node / bag_camera.py
CLOSE_H = 0.19
HALF_DEPTH = 0.055
RIM_H = 0.27
BAG_WIDTH = 0.18
BAG_DEPTH = 0.11
BIAS_FORWARD = -0.012
BIAS_UP = -0.014
TRIES = ((0.0, -60.0), (0.015, -60.0), (-0.015, -60.0), (0.03, -50.0))
REACH_WALL = 0.32
FREE_SPEED = 0.8
LIFT_SPEED = 0.25
HELD_SPEED = 0.4
CONF_MIN = 0.55
LOOK = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.04,
    "elbow_flex": -0.04,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
}
DESCEND_DZ = (0.08, 0.06, 0.04, 0.02, 0.0)
LIFT_DZ = (0.05, 0.10, 0.15, 0.20)

# ik.py
JAW_OPEN = 0.9
FIXED_INNER_X = 0.008
TIP_Y = -0.106

# joint_limits.yaml (MoveIt). URDF velocity="1" and effort="35" are not used.
ARM_VEL = 1.5
GRIPPER_VEL = 3.0

# driver_node publishes /joint_states at 50 Hz. The bench stepped at 2 ms.
CONTROL_DT = 0.02
PHYSICS_DT = 0.002

# Owner tape: handle top 0.36 to 0.37 m. The middle of that range.
HANDLE_TOP = 0.365

# Wall distance for the nominal paper bag. The close bias asks the fingers 1.2 cm
# closer than the wall, so the commanded reach is 0.300 m. At 0.28 m the wrist
# mesh sits in the deck box; at 0.30 m and pitch -60 deg it does not.
NOMINAL_WALL = 0.312

# PLAN.md success.
HOLD_S = 3.0
LIFT_CLEAR = 0.02
SLIP_M = 0.01
SUCCESS_BONUS = 10.0
COLLISION_PENALTY = 5.0
DROP_PENALTY = 2.0
# Tier-2 reward is -0.01 per 10 Hz step, which is -0.1 per second. Tier 1 uses the
# episode outcome, so the same time term is applied once for the whole try.
TIME_PENALTY_PER_S = 0.1

# PLACEHOLDER actuators. The 20 Nov bench used kp 20, kv 1, forcerange ±3 and the
# arm tracked a slow sine. kp and kv are higher here so the -60 deg reach holds
# against gravity (still not a sysid fit). The STS3215 stall torque is unknown.
# URDF effort 35 is not used.
ACTUATOR_KP = 40.0
ACTUATOR_KV = 2.0
ACTUATOR_FORCE = 3.0

# PLACEHOLDER bag mass. The bench used 0.15 kg. Empty and loaded masses were not weighed.
BAG_MASS = 0.15

# PLACEHOLDER rope radius. The bench used a 3 mm capsule. The real rope was not measured.
ROPE_RADIUS = 0.003

# PLACEHOLDER ribbon half-sizes (wide, thin), metres. Not measured. The thin half is
# also the flex radius, which MuJoCo requires for a box-section rod.
RIBBON_HALF = (0.005, 0.001)

# PLACEHOLDER Young's modulus (Pa). 1e7 let the arch fall to the rim; 1e8 left about
# 6 mm of sag, inside the 1 cm tape range. Not a measured modulus.
ROPE_YOUNG = 1.0e8

# PLACEHOLDER. Contacts lighter than this are not counted, so a tiny numerical force
# is not a collision. Not measured on the robot.
COLLISION_FORCE_N = 5.0

# The new front camera is 640x480 (PLAN.md). Focal length, distortion and pose are
# not calibrated. 262 px is the equidistant f that would spread a 140 deg field
# across 640 px. That field was not measured.
IMAGE_W = 640
IMAGE_H = 480
NOMINAL_F_PX = 262.0
F_RANGE = (160.0, 420.0)
# PLACEHOLDER mount: a point above the arm base, not the old 0.89 m note and not
# front_camera.json. Randomised widely around it.
NOMINAL_CAM_H = 0.70
NOMINAL_CAM_FWD = -0.05
CAM_H_RANGE = (0.45, 1.10)
CAM_FWD_RANGE = (-0.15, 0.10)
CAM_LEFT_RANGE = (-0.08, 0.08)
# PLACEHOLDER pixel noise and the chance a frame is dropped.
PIXEL_NOISE_PX = 1.5
DROPOUT_P = 0.15

# Grasp-parameter bounds. Offsets cover TRIES (-0.015..0.03 m, -60 and -50 deg)
# with room around them. The speed scale multiplies FREE_SPEED and LIFT_SPEED.
ACTION_LOW = (-0.04, -0.05, 0.12, -80.0, -30.0, 0.5)
ACTION_HIGH = (0.04, 0.05, 0.28, -40.0, 30.0, 1.5)

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
ALL_JOINTS = ARM_JOINTS + ("gripper",)

OBS_NAMES = (
    "q_shoulder_pan", "q_shoulder_lift", "q_elbow_flex", "q_wrist_flex", "q_wrist_roll", "q_gripper",
    "qd_shoulder_pan", "qd_shoulder_lift", "qd_elbow_flex", "qd_wrist_flex", "qd_wrist_roll", "qd_gripper",
    "jaw_angle", "jaw_load",
    "bag_forward", "bag_left",
    "handle_forward", "handle_left", "handle_up", "handle_valid",
    "rim_width", "standing", "confidence", "source_camera",
)

# Jaw gap at 0.10 rad is 2.26 cm (ik.jaw_gap). The plate is this thick along the
# closing axis so a jaw commanded to 0 rad, the real close position, stalls on it
# instead of shutting. Bunched-paper thickness was not measured. PLACEHOLDER.
PINCH_THICKNESS = 0.016 + 0.066 * 0.10


def jaw_gap(angle):
    """Same fit as so100_driver.ik.jaw_gap."""
    return 0.016 + 0.066 * angle


def gap_mid():
    """Gripper-frame x of the middle of the fully open jaw gap. bag_camera.GAP_MID."""
    return FIXED_INNER_X - jaw_gap(JAW_OPEN) / 2.0


def handle_rise():
    """How far the handle arch stands above the rim, from the tape and the rim height."""
    return HANDLE_TOP - RIM_H


def pitch_rad(deg):
    return math.radians(deg)
