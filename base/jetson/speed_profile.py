"""Floor-pick half of the speed profiles. The arm half is so100_driver.motion_profile.

'safe' is today's motion: joint scale 0.30 (SPEED_SCALE) and LINE_SPEED 0.05 m/s.
'fast' is joint scale 0.45 and LINE_SPEED 0.08 m/s, selected with
floor_pick.py --speed-profile fast after a supervised test. The default is safe.
bag_speed and the OMPL budget live on the arm node (motion_profile.FAST_PROFILE);
they are repeated here so one JSON record can name every number that changes.
"""

SAFE = "safe"
FAST = "fast"

SAFE_PROFILE = {
    "joint_scale": 0.30,
    "line_speed": 0.05,
    "approach_speed": 0.03,
    "bag_speed": 0.08,
    "planning_time": 5.0,
    "planning_attempts": 3,
}
FAST_PROFILE = {
    "joint_scale": 0.45,
    "line_speed": 0.08,
    "approach_speed": 0.08,
    "bag_speed": 0.12,
    "planning_time": 1.0,
    "planning_attempts": 1,
}


def get(name):
    if name == SAFE:
        return dict(SAFE_PROFILE)
    if name == FAST:
        return dict(FAST_PROFILE)
    raise ValueError("speed profile must be safe or fast, not %r" % (name,))
