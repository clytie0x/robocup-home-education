"""PPO and SAC improve a tiny bandit. No robot, so this stays under a second."""

import numpy as np

from rl_grasp import ppo, sac


class _Match:
    """Action should copy the observation. Reward is minus the absolute error."""

    def __init__(self, seed=0):
        self.rng = np.random.default_rng(seed)
        self.target = np.zeros(2)

    def reset(self, seed=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.target = self.rng.uniform(-1.0, 1.0, size=2)
        return self.target.astype(np.float64), {}

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        reward = -float(np.abs(action - self.target).sum())
        return self.target.copy(), reward, True, {}


def _improved(returns):
    early = float(np.mean(returns[:40]))
    late = float(np.mean(returns[-40:]))
    return early, late


def test_ppo_improves_the_toy():
    _net, returns = ppo.train(_Match(), steps=512, seed=0, act_dim=2, hidden=32, batch=32, epochs=10, lr=3e-3)
    early, late = _improved(returns)
    assert late > early + 0.15


def test_sac_improves_the_toy():
    _actor, returns = sac.train(
        _Match(), steps=700, seed=0, act_dim=2, hidden=32, batch=64, lr=1e-3, warmup=80,
    )
    early, late = _improved(returns)
    assert late > early + 0.15
