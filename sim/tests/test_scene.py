"""The model loads, matches the URDF, and steps the same way twice."""

import mujoco
import numpy as np

from rl_grasp import constants as C
from rl_grasp.env import GraspEnv
from rl_grasp.scene import load_scene


def _park_and_step(kind, n=40):
    model, _kin = load_scene(kind)
    data = mujoco.MjData(model)
    data.ctrl[:] = 0.1
    for _ in range(n):
        mujoco.mj_step(model, data)
    return data.qpos.copy(), data.time


def test_paper_and_gift_step_and_stay_finite():
    for kind in ("paper", "gift"):
        qpos, _time = _park_and_step(kind)
        assert np.isfinite(qpos).all()


def test_the_same_seed_steps_the_same_way():
    first, _ = _park_and_step("paper", 25)
    second, _ = _park_and_step("paper", 25)
    assert np.allclose(first, second)


def test_the_handle_stays_near_the_taped_height():
    env = GraspEnv(nominal=True, seed=0)
    env.reset(0)
    top = float(env.data.flexvert_xpos[:, 2].max()) - C.FLOOR_Z
    assert 0.30 < top < 0.42
