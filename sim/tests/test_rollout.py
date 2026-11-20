"""A training step of the scripted action is the same full try evaluate runs."""

import numpy as np

from rl_grasp.env import GraspEnv
from rl_grasp.evaluate import run
from rl_grasp.executor import scripted_action
from rl_grasp.policy import to_normal, to_physical


def _median(text):
    for line in text.splitlines():
        if line.startswith("median_time_s:"):
            return float(line.split(": ", 1)[1])
    raise AssertionError(text)


def test_zero_network_output_is_the_scripted_pick():
    assert np.allclose(to_normal(scripted_action(0)), 0.0, atol=1e-6)
    assert np.allclose(to_physical(np.zeros(6)), scripted_action(0), atol=1e-6)


def test_training_step_matches_evaluate():
    env = GraspEnv(nominal=True, max_tries=1, seed=0)
    env.reset(0)
    _obs, _reward, _done, info = env.step(to_physical(np.zeros(6)))
    report = run("scripted", episodes=1, seed=0, nominal=True, max_tries=1)
    assert info["success"]
    assert info["time_s"] > 8.0
    assert abs(info["time_s"] - _median(report)) < 0.05


def test_the_training_wrapper_records_a_full_try():
    """train() steps through _Physical. A zero action there is a full simulated try."""
    from rl_grasp.train import _Physical

    env = _Physical(GraspEnv(nominal=True, max_tries=1, seed=0))
    env.reset(0)
    _obs, _reward, _done, info = env.step(np.zeros(6))
    assert env.times == [info["time_s"]]
    assert info["success"]
    assert info["time_s"] > 8.0
    report = run("scripted", episodes=1, seed=0, nominal=True, max_tries=1)
    assert abs(info["time_s"] - _median(report)) < 0.05
