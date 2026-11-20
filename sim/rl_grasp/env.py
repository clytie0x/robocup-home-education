"""One grasp try per step. reset() and step() are our own; this is not a Gymnasium env."""

import math

import mujoco
import numpy as np

from . import camera, constants as C
from .executor import run_try
from .reward import outcome_reward
from .scene import actuator_id, joint_id, load_scene, qpos_of, qvel_of

class GraspEnv:
    def __init__(self, nominal=False, max_tries=1, seed=0, hard=False, tuned=False):
        self.nominal = bool(nominal)
        self.hard = bool(hard)
        self.tuned = bool(tuned)
        self.max_tries = int(max_tries)
        self.rng = np.random.default_rng(seed)
        self._scenes = {}
        self.model = None
        self.data = None
        self.kin = None
        self.kind = "paper"
        self._nominal_dyn = None
        self.cam_true = None
        self.cam_nominal = None
        self.tries = 0
        self._clock = 0.0
        self._cached = None
        self._load("paper")

    def _load(self, kind):
        if kind not in self._scenes:
            model, kin = load_scene(kind)
            self._scenes[kind] = (model, kin, _snapshot(model))
        self.kind = kind
        self.model, self.kin, self._nominal_dyn = self._scenes[kind]
        self.data = mujoco.MjData(self.model)
        self.cam_nominal = camera.nominal_camera(self.kin)

    def reset(self, seed=None):
        if seed is not None:
            self.rng = np.random.default_rng(int(seed))
        kind = "paper" if self.nominal else ("gift" if self.rng.random() < 0.5 else "paper")
        if kind != self.kind or self.data is None:
            self._load(kind)
        _restore(self.model, self._nominal_dyn)
        mujoco.mj_resetData(self.model, self.data)
        wall, left, yaw, floor = self._sample_pose()
        self._apply_dynamics(floor)
        self._place_bag(wall, left, yaw, floor)
        self._park_arm()
        if self.nominal:
            self.cam_true = self.cam_nominal
        else:
            self.cam_true = camera.sample_camera(self.kin, self.rng, hard=self.hard)
        self._settle()
        self.tries = 0
        self._clock = float(self.data.time)
        self._cached = self.reading()
        return self._obs_from(self._cached), {"kind": self.kind}

    def step(self, action):
        result = run_try(self.model, self.data, self.kin, self._cached, action, self._clock)
        self.tries += 1
        done = bool(result["success"] or self.tries >= self.max_tries)
        reward = outcome_reward(
            result["success"], result["collision"], result["dropped"], result["time_s"],
        )
        result["reward"] = reward
        self._cached = self.reading()
        return self._obs_from(self._cached), reward, done, result

    def observation(self):
        if self._cached is None:
            self._cached = self.reading()
        return self._obs_from(self._cached)

    def _obs_from(self, reading):
        position = [qpos_of(self.model, self.data, name) for name in C.ALL_JOINTS]
        velocity = [qvel_of(self.model, self.data, name) for name in C.ALL_JOINTS]
        load = _jaw_load(self.model, self.data)
        if reading["handle"] is None:
            handle = (0.0, 0.0, 0.0, 0.0)
        else:
            handle = (*reading["handle"], 1.0)
        raw = np.asarray((
            *position, *velocity, position[-1], load,
            reading["forward"], reading["left"], *handle,
            reading["width"], 1.0 if reading["standing"] else 0.0,
            reading["confidence"], 1.0 if reading["source"] == "camera" else 0.0,
        ), dtype=np.float64)
        return raw / np.asarray(C.OBS_SCALE, dtype=np.float64)

    def reading(self):
        truth = _truth(self.model, self.data)
        if self.nominal:
            noise, dropout = 0.0, 0.0
        elif self.hard:
            noise, dropout = C.HARD_PIXEL_NOISE_PX, C.HARD_DROPOUT_P
        else:
            noise, dropout = C.PIXEL_NOISE_PX, C.DROPOUT_P
        reading = camera.detect(
            truth, self.cam_true, self.cam_nominal, self.rng, self.kin, noise, dropout,
        )
        # coco, the real default, aims at the rim. The handle shift is the tuned model.
        if not self.tuned:
            reading["handle"] = None
        return reading

    def _sample_pose(self):
        if self.nominal:
            return C.NOMINAL_WALL, 0.0, 0.0, C.FLOOR_Z
        # 0.305–0.312 m: the close collides near 0.28 m and REACH_WALL is 0.32 m.
        wall = float(self.rng.uniform(0.305, 0.312))
        left = float(self.rng.uniform(-0.04, 0.04))
        yaw = math.radians(float(self.rng.uniform(-30.0, 30.0)))
        floor = C.FLOOR_Z + float(self.rng.uniform(-0.01, 0.01))
        return wall, left, yaw, floor

    def _apply_dynamics(self, floor):
        if self.nominal:
            return
        snap = self._nominal_dyn
        mass_range = C.HARD_MASS_SCALE if self.hard else C.MASS_SCALE
        friction_range = C.HARD_FRICTION_SCALE if self.hard else C.FRICTION_SCALE
        actuator_range = C.HARD_ACTUATOR_SCALE if self.hard else C.ACTUATOR_SCALE
        bend_range = C.HARD_BEND_SCALE if self.hard else C.BEND_SCALE
        mass_scale = float(self.rng.uniform(*mass_range))
        bag = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "bag")
        self.model.body_mass[bag] = snap["mass"][bag] * mass_scale
        slide = float(self.rng.uniform(*friction_range))
        for name in ("bag_wall", "rim", "floor", "pad_fixed", "pad_jaw"):
            gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            self.model.geom_friction[gid, 0] = snap["friction"][gid, 0] * slide
        for index in range(self.model.nu):
            scale = float(self.rng.uniform(*actuator_range))
            self.model.actuator_gainprm[index, 0] = snap["gain"][index, 0] * scale
            self.model.actuator_biasprm[index, 1] = snap["bias"][index, 1] * scale
            self.model.actuator_biasprm[index, 2] = snap["bias"][index, 2] * scale
            self.model.actuator_forcerange[index] = snap["force"][index] * scale
        if self.model.flex_bending.size:
            # Coefficients above the compiled value segfault MuJoCo 4.0.1.
            # Entries of -1 are unused slots, not stiffnesses, so they stay.
            bend = float(self.rng.uniform(*bend_range))
            scaled = snap["bend"].copy()
            positive = scaled > 0.0
            scaled[positive] *= bend
            self.model.flex_bending[:] = scaled
        gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.model.geom_pos[gid, 2] = floor

    def _place_bag(self, wall, left, yaw, floor):
        centre = np.array(self.kin.from_user(wall + C.HALF_DEPTH, left))
        address = self.model.jnt_qposadr[joint_id(self.model, "bag_free")]
        self.data.qpos[address:address + 3] = (centre[0], centre[1], floor + C.RIM_H / 2.0)
        half = yaw / 2.0
        self.data.qpos[address + 3:address + 7] = (math.cos(half), 0.0, 0.0, math.sin(half))

    def _park_arm(self):
        pose = dict(C.LOOK)
        pose["gripper"] = C.JAW_OPEN
        for name, value in pose.items():
            address = self.model.jnt_qposadr[joint_id(self.model, name)]
            self.data.qpos[address] = value
        from .scene import set_controls
        set_controls(self.model, self.data, pose)

    def _settle(self):
        steps = int(0.3 / C.PHYSICS_DT)
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)


def _snapshot(model):
    return {
        "mass": model.body_mass.copy(),
        "friction": model.geom_friction.copy(),
        "gain": model.actuator_gainprm.copy(),
        "bias": model.actuator_biasprm.copy(),
        "force": model.actuator_forcerange.copy(),
        "bend": model.flex_bending.copy(),
    }


def _restore(model, snap):
    model.body_mass[:] = snap["mass"]
    model.geom_friction[:] = snap["friction"]
    model.actuator_gainprm[:] = snap["gain"]
    model.actuator_biasprm[:] = snap["bias"]
    model.actuator_forcerange[:] = snap["force"]
    if snap["bend"].size:
        model.flex_bending[:] = snap["bend"]
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    model.geom_pos[floor, 2] = C.FLOOR_Z


def _jaw_load(model, data):
    force = abs(float(data.actuator_force[actuator_id(model, "gripper")]))
    return min(100.0, 100.0 * force / C.ACTUATOR_FORCE)


def _truth(model, data):
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "bag")
    origin = data.xpos[body]
    rotation = data.xmat[body].reshape(3, 3)
    half = C.BAG_WIDTH / 2.0
    # The detector reads the rim, so the centre point it projects sits at rim height.
    rim = np.array((0.0, C.HALF_DEPTH, C.RIM_H / 2.0))
    centre = origin + rotation @ np.array((0.0, 0.0, C.RIM_H / 2.0))
    left = origin + rotation @ (rim + np.array((half, 0.0, 0.0)))
    right = origin + rotation @ (rim + np.array((-half, 0.0, 0.0)))
    handle = None
    if model.nflexvert:
        handle = np.array(data.flexvert_xpos[int(np.argmax(data.flexvert_xpos[:, 2]))])
    return {
        "centre": centre,
        "handle": handle,
        "rim_left": left,
        "rim_right": right,
        "width": C.BAG_WIDTH,
        "standing": True,
    }
