"""One camera-pick try, the same order as bag_camera.CameraBag.

Look (the arm is already at LOOK), aim at the handle centroid when the detector
has one and at the rim otherwise, tips at the close height, pitch from the try,
jaw open, move above, descend, close to jaw 0, lift, hold 3 s.
A wall past REACH_WALL is a miss with no motion.
"""

import math

import mujoco
import numpy as np

from . import constants as C
from .reward import judge
from .scene import bag_bottom, geom_id, qpos_of, set_controls

_PAD = ("pad_fixed", "pad_jaw")


def scripted_action(index):
    """The hand-picked TRIES entry, plus the nominal close height, roll and speed."""
    along, pitch = C.TRIES[index % len(C.TRIES)]
    return np.array((along, 0.0, C.CLOSE_H, pitch, 0.0, 1.0), dtype=float)


def clip_action(action):
    raw = np.asarray(action, dtype=float).reshape(6)
    low = np.asarray(C.ACTION_LOW, dtype=float)
    high = np.asarray(C.ACTION_HIGH, dtype=float)
    return np.minimum(np.maximum(raw, low), high)


def _read_arm(model, data):
    return {name: qpos_of(model, data, name) for name in C.ALL_JOINTS}


def _speed_limit(name, asked):
    cap = C.GRIPPER_VEL if name == "gripper" else C.ARM_VEL
    return min(abs(asked), cap)


def _move(model, data, target, speed, grip):
    """Move each joint toward `target` no faster than `speed` or the MoveIt cap."""
    current = _read_arm(model, data)
    target = dict(target)
    target["gripper"] = grip
    gap = max(abs(target[name] - current[name]) for name in C.ALL_JOINTS)
    limited = min(speed, C.ARM_VEL)
    duration = max(C.CONTROL_DT, gap / max(limited, 1e-3))
    steps = max(1, int(math.ceil(duration / C.CONTROL_DT)))
    sub = int(round(C.CONTROL_DT / C.PHYSICS_DT))
    for step in range(1, steps + 1):
        blend = step / steps
        command = {
            name: current[name] + (target[name] - current[name]) * blend
            for name in C.ALL_JOINTS
        }
        set_controls(model, data, command)
        for _ in range(sub):
            mujoco.mj_step(model, data)
    return duration


def _solve(kin, reach, height_z, pitch_deg, roll_deg, bearing):
    """CameraBag.tip / solve: tips at `reach` along `bearing`, gap middle on that line."""
    pitch = math.radians(pitch_deg)
    cosine, sine = math.cos(pitch), math.sin(pitch)
    tip = -C.TIP_Y
    radial = reach - tip * cosine
    forward = radial * math.cos(bearing)
    left = radial * math.sin(bearing)
    x, y = kin.from_user(forward, left)
    point = np.array((x, y, height_z - tip * sine))
    solutions = kin.solve(point, pitch, depth=0.0)
    if not solutions:
        return None
    pose = dict(solutions[0])
    pose.pop("_branch", None)
    pose["wrist_roll"] = math.radians(roll_deg)
    point = point - C.gap_mid() * kin.frames(pose)["gripper"][:3, 0]
    solutions = kin.solve(point, pitch, depth=0.0)
    if not solutions:
        return None
    pose = dict(solutions[0])
    pose.pop("_branch", None)
    pose["wrist_roll"] = math.radians(roll_deg)
    low, high = kin.limits["wrist_roll"]
    if not low <= pose["wrist_roll"] <= high:
        return None
    return pose


def _user_dir(kin, vector):
    """bag_camera.user_dir: a (forward, left) direction as a base-frame vector."""
    heading = kin.heading0
    forward = np.array((math.cos(heading), math.sin(heading)))
    left = np.array((-math.sin(heading), math.cos(heading)))
    return np.array((*(vector[0] * forward + vector[1] * left), 0.0))


def _aim(reading, action):
    """Front wall from the detector reading, then the action's sideways shift.

    Tuned detections keep the rim's forward distance and take the handle's left,
    which is what bag_camera does with a handle centroid.
    """
    centre = np.array((reading["forward"], reading["left"]), dtype=float)
    # No yaw in the detector output: the bag faces the arm, handles along left.
    line = math.pi / 2.0
    normal = np.array((math.sin(line), -math.cos(line)))
    if normal @ centre < 0:
        normal = -normal
    wall = centre - C.HALF_DEPTH * normal
    if reading["handle"] is not None and reading["confidence"] >= C.CONF_MIN:
        wall = np.array((wall[0], reading["handle"][1]))
    tangent = np.array((-normal[1], normal[0]))
    wall = wall + float(action[1]) * tangent
    return wall, normal


def _contacts(model, data):
    grasp = set()
    collision = False
    for index in range(data.ncon):
        contact = data.contact[index]
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, force)
        if force[0] < C.COLLISION_FORCE_N:
            continue
        names = []
        for gid in (int(contact.geom1), int(contact.geom2)):
            if gid < 0:
                names.append("rope")
            else:
                names.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or "")
        pads = [name for name in names if name in _PAD]
        other = [name for name in names if name not in _PAD]
        if pads and any(name == "rim" or name == "rope" for name in other):
            grasp.update(pads)
            continue
        if pads and "bag_wall" in other:
            collision = True
            continue
        arm = any(name.startswith("col_") for name in names)
        blocked = any(name in ("bag_wall", "deck", "floor") for name in names)
        if arm and blocked:
            collision = True
    return grasp, collision


def _hold(model, data, seconds):
    """Stay on the current command. Return how long the bag stayed up, the slip, the bottom."""
    start = np.array(data.geom_xpos[geom_id(model, "bag_wall")][:2], dtype=float)
    held = 0.0
    bottom = bag_bottom(model, data)
    slip = 0.0
    steps = int(round(seconds / C.PHYSICS_DT))
    for count in range(steps):
        mujoco.mj_step(model, data)
        bottom = bag_bottom(model, data)
        now = data.geom_xpos[geom_id(model, "bag_wall")][:2]
        slip = max(slip, float(np.linalg.norm(now - start)))
        if bottom >= C.LIFT_CLEAR and slip <= C.SLIP_M:
            held = (count + 1) * C.PHYSICS_DT
        elif bottom < C.LIFT_CLEAR:
            break
    return held, slip, bottom


def run_try(model, data, kin, reading, action, clock):
    """Execute one try. `clock` is data.time at the start of the pick, after settle."""
    action = clip_action(action)
    wall, normal = _aim(reading, action)
    bearing = math.atan2(wall[1], wall[0])
    distance = float(np.hypot(wall[0], wall[1]))
    if distance > C.REACH_WALL:
        return _result(False, False, False, data.time - clock, False, bag_bottom(model, data))
    reach = distance + float(action[0]) + C.BIAS_FORWARD
    close_z = C.FLOOR_Z + float(action[2]) + C.BIAS_UP
    pitch = float(action[3])
    speed = C.FREE_SPEED * float(action[5])
    lift_speed = C.LIFT_SPEED * float(action[5])
    seed = _solve(kin, reach, close_z, pitch, 0.0, bearing)
    if seed is None:
        return _result(False, False, False, data.time - clock, False, bag_bottom(model, data))
    roll, _align = kin.roll_for(seed, _user_dir(kin, normal), 0.0)
    if roll is None:
        return _result(False, False, False, data.time - clock, False, bag_bottom(model, data))
    roll_deg = math.degrees(roll) + float(action[4])

    def pose_at(extra_h):
        return _solve(kin, reach, close_z + extra_h, pitch, roll_deg, bearing)

    above = pose_at(0.10) or pose_at(0.20)
    if above is None:
        return _result(False, False, False, data.time - clock, False, bag_bottom(model, data))
    collision = False
    _move(model, data, above, speed, C.JAW_OPEN)
    _grasp, hit = _contacts(model, data)
    collision = collision or hit
    for extra in C.DESCEND_DZ:
        pose = pose_at(extra)
        if pose is None:
            break
        _move(model, data, pose, speed, C.JAW_OPEN)
        _grasp, hit = _contacts(model, data)
        collision = collision or hit
    _move(model, data, _read_arm(model, data), C.GRIPPER_VEL, 0.0)
    # The real close is the only pause. 0.4 s is long enough for the jaw to stall.
    for _ in range(int(0.4 / C.PHYSICS_DT)):
        mujoco.mj_step(model, data)
    grasp, hit = _contacts(model, data)
    collision = collision or hit
    lifted = bag_bottom(model, data) >= C.LIFT_CLEAR
    for extra in C.LIFT_DZ:
        pose = pose_at(extra)
        if pose is None:
            break
        _move(model, data, pose, lift_speed, 0.0)
        grasp_now, hit = _contacts(model, data)
        grasp = grasp_now or grasp
        collision = collision or hit
        if bag_bottom(model, data) >= C.LIFT_CLEAR + 0.01 and len(grasp_now) == 2:
            grasp = grasp_now
            lifted = True
            break
    held_s, slip, bottom = _hold(model, data, C.HOLD_S)
    grasp, hit = _contacts(model, data)
    collision = collision or hit
    both = "pad_fixed" in grasp and "pad_jaw" in grasp
    success, dropped = judge(both, bottom, held_s, slip, collision)
    if collision:
        success = False
    return _result(success, collision, dropped, data.time - clock, both, bottom)


def _result(success, collision, dropped, time_s, grasped, bottom):
    return {
        "success": bool(success),
        "collision": bool(collision),
        "dropped": bool(dropped),
        "time_s": float(time_s),
        "grasped": bool(grasped),
        "bottom": float(bottom),
    }
