"""The scripted pick lifts a nominal paper bag."""

import numpy as np

from rl_grasp.env import GraspEnv
from rl_grasp.evaluate import run
from rl_grasp.executor import scripted_action


def test_the_scripted_baseline_grasps_the_nominal_bag():
    text = run("scripted", episodes=1, seed=0, nominal=True, max_tries=1)
    assert "success_rate: 1.000000" in text
    assert "collision_count: 0" in text
    assert "successes: 1" in text
    # The first TRIES entry is the nominal close: no offset, pitch -60.
    action = scripted_action(0)
    assert action[0] == 0.0
    assert action[3] == -60.0


def test_randomised_episodes_do_not_crash():
    for seed in (1000, 1002):
        env = GraspEnv(nominal=False, max_tries=1, seed=seed)
        env.reset(seed)
        _obs, _reward, _done, result = env.step(scripted_action(0))
        assert np.isfinite(result["time_s"])
        assert np.isfinite(env.data.qpos).all()
