"""Success rate, median time to a held lift, and collision count."""

import statistics

import numpy as np

from .env import GraspEnv
from .executor import scripted_action
from .policy import deterministic_action
from .train import load_module


def format_report(name, episodes, successes, times, collisions):
    rate = (successes / episodes) if episodes else 0.0
    if times:
        median = "%.3f" % statistics.median(times)
    else:
        median = "nan"
    return "\n".join((
        "policy: %s" % name,
        "episodes: %d" % episodes,
        "success_rate: %.6f" % rate,
        "median_time_s: %s" % median,
        "collision_count: %d" % collisions,
        "successes: %d" % successes,
    ))


def run(policy, episodes, seed, nominal, max_tries):
    """`policy` is "scripted" or a checkpoint path."""
    name, choose = _chooser(policy)
    env = GraspEnv(nominal=nominal, max_tries=max_tries, seed=seed)
    successes = 0
    collisions = 0
    times = []
    for index in range(episodes):
        obs, _info = env.reset(seed + index)
        result = None
        for try_index in range(max_tries):
            action = choose(obs, try_index)
            obs, _reward, done, result = env.step(action)
            if done:
                break
        if result["success"]:
            successes += 1
            times.append(result["time_s"])
        if result["collision"]:
            collisions += 1
    return format_report(name, episodes, successes, times, collisions)


def _chooser(policy):
    if policy == "scripted":
        return "scripted", lambda _obs, index: scripted_action(index)
    module, blob = load_module(policy)

    def choose(obs, _index):
        return deterministic_action(module, np.asarray(obs, dtype=np.float64))

    return blob["algo"], choose
