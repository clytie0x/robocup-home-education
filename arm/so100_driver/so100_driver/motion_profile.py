"""Speed profiles and Pilz LIN request fields for the SO-100 pick pipeline.

'safe' is the profile on the robot today: joint scale 0.30, approach lines at the
line_speed parameter (0.03 m/s), bag carry at bag_speed (0.08 m/s), OMPL 5 s x 3.
'fast' is the bench step from the hardware-efficiency plan (joint 0.45, straight
lines 0.08 m/s, bag carry 0.12 m/s, OMPL 1 s x 1). It is selected with the
speed_profile parameter (or bag_arm.py --speed-profile) after a supervised arm test.
The default stays 'safe'.

Straight approach and lift ask MoveIt for Pilz LIN (already loaded in arm.launch.py).
Awkward joint goals stay on OMPL. Pilz cartesian_limits max_trans_vel is 0.3 m/s, so
the velocity scale is the wanted line speed divided by that cap.
"""
import math

SAFE = "safe"
FAST = "fast"
PROFILES = (SAFE, FAST)

PILZ_PIPELINE = "pilz_industrial_motion_planner"
PILZ_LIN = "LIN"
OMPL_PIPELINE = "ompl"
TIP_LINKS = ("tcp", "gripper")  # SRDF tip is tcp; gripper is the link the IK frames always have
CARTESIAN_MAX_VEL = 0.3  # m/s, arm/so100_moveit_config/config/pilz_cartesian_limits.yaml
PLANNING_FRAME = "base"

# Numbers the running node already uses when speed_profile is safe. fast replaces them.
SAFE_DEFAULTS = {
    "joint_scale": 0.30,
    "approach_speed": 0.03,  # pick_place line_speed parameter, m/s on approach/lift
    "bag_speed": 0.08,       # m/s of the grasp point on the carry
    "line_speed": 0.05,      # floor_pick.LINE_SPEED, m/s cap on a straight grasp-point path
    "planning_time": 5.0,    # s, OMPL allowed_planning_time
    "planning_attempts": 3,
}
FAST_PROFILE = {
    "joint_scale": 0.45,
    "approach_speed": 0.08,
    "bag_speed": 0.12,
    "line_speed": 0.08,
    "planning_time": 1.0,
    "planning_attempts": 1,
}


def resolve(name, current=None):
    """The numbers for `name`. 'safe' keeps `current` (the live parameters); 'fast' is FAST_PROFILE.

    `current` supplies the safe values so a launch file that already set line_speed or bag_speed
    is unchanged until someone selects fast. Missing keys fall back to SAFE_DEFAULTS.
    """
    if name not in PROFILES:
        raise ValueError("speed profile must be safe or fast, not %r" % (name,))
    base = dict(SAFE_DEFAULTS)
    if current:
        base.update({k: current[k] for k in SAFE_DEFAULTS if k in current and current[k] is not None})
    if name == SAFE:
        return base
    return dict(FAST_PROFILE)


def velocity_scale(line_speed, cap=CARTESIAN_MAX_VEL):
    """Pilz max_velocity_scaling_factor for a wanted Cartesian speed (m/s)."""
    if not (line_speed > 0 and cap > 0):
        raise ValueError("line speed and cartesian cap must be above 0")
    return min(1.0, float(line_speed) / float(cap))


def tip_frame(frames):
    """(link name, 4x4) of the arm tip. tcp is the MoveIt tip; gripper is the IK frame."""
    for name in TIP_LINKS:
        if name in frames:
            return name, frames[name]
    raise KeyError("no tcp or gripper frame")


def quat_xyzw(rotation):
    """Unit quaternion (x, y, z, w) of a 3x3 rotation matrix."""
    r = rotation
    trace = float(r[0][0] + r[1][1] + r[2][2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2][1] - r[1][2]) / s
        y = (r[0][2] - r[2][0]) / s
        z = (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        w = (r[2][1] - r[1][2]) / s
        x = 0.25 * s
        y = (r[0][1] + r[1][0]) / s
        z = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        w = (r[0][2] - r[2][0]) / s
        x = (r[0][1] + r[1][0]) / s
        y = 0.25 * s
        z = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        w = (r[1][0] - r[0][1]) / s
        x = (r[0][2] + r[2][0]) / s
        y = (r[1][2] + r[2][1]) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    return (x / n, y / n, z / n, w / n)


def lin_request(matrix, line_speed, link):
    """Fields of a Pilz LIN MotionPlanRequest to the tip pose `matrix` (4x4) at `line_speed` m/s.

    The node turns this into moveit_msgs. Straight lines only: joint goals stay on OMPL.
    """
    position = (float(matrix[0][3]), float(matrix[1][3]), float(matrix[2][3]))
    rotation = [[float(matrix[i][j]) for j in range(3)] for i in range(3)]
    return {
        "pipeline_id": PILZ_PIPELINE,
        "planner_id": PILZ_LIN,
        "group_name": "arm",
        "frame_id": PLANNING_FRAME,
        "link": link,
        "position": position,
        "orientation_xyzw": quat_xyzw(rotation),
        "velocity_scale": velocity_scale(line_speed),
        "position_tolerance_m": 0.001,
        "orientation_tolerance_rad": 0.01,
    }


def ompl_request(speeds):
    """Fields of an OMPL joint-goal request (awkward goals: home, carry, above the bag)."""
    return {
        "pipeline_id": OMPL_PIPELINE,
        "planner_id": "",
        "allowed_planning_time": float(speeds["planning_time"]),
        "num_planning_attempts": int(speeds["planning_attempts"]),
        "max_velocity_scaling_factor": float(speeds["joint_scale"]),
    }
