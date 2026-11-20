"""Train a tier-1 grasp policy. `steps` counts environment transitions (one try each)."""

import multiprocessing as mp
from pathlib import Path

import numpy as np
import torch

from . import ppo, sac
from .env import GraspEnv
from .policy import Actor, ActorCritic, to_physical


class _Physical:
    """Network actions are in [-1, 1]. The env wants grasp parameters."""

    def __init__(self, env):
        self.env = env

    def reset(self, seed=None):
        return self.env.reset(seed)

    def step(self, normal):
        return self.env.step(to_physical(normal))


def train_policy(algo, steps, seed, workers, nominal, max_tries, hidden=64):
    if workers <= 1:
        env = _Physical(GraspEnv(nominal=nominal, max_tries=max_tries, seed=seed))
        trainer = ppo.train if algo == "ppo" else sac.train
        if algo not in ("ppo", "sac"):
            raise ValueError("algo must be ppo or sac, got %r" % algo)
        module, returns = trainer(
            env, steps, seed, act_dim=6, hidden=hidden,
            batch=min(32, max(1, steps)), lr=1e-3,
        )
        return module, returns
    if algo == "sac":
        # The replay buffer lives in this process. Parallel rollouts are PPO's.
        return train_policy(algo, steps, seed, 1, nominal, max_tries, hidden)
    return _train_ppo_parallel(steps, seed, workers, nominal, max_tries, hidden)


def _train_ppo_parallel(steps, seed, workers, nominal, max_tries, hidden):
    obs0 = _one_obs(nominal, seed)
    net = ActorCritic(len(obs0), 6, hidden)
    optimizer = torch.optim.Adam(net.parameters(), lr=1e-3)
    pool = _Pool(workers, nominal, max_tries, seed)
    try:
        observations = pool.reset([seed + index for index in range(workers)])
        returns = []
        buffer = []
        taken = 0
        while taken < steps:
            actions = [_act(net, obs) for obs in observations]
            results = pool.step([item[0] for item in actions])
            next_obs = []
            for obs, action, result in zip(observations, actions, results):
                new_obs, reward, done, _info = result
                buffer.append((obs, action[0], action[1], float(reward), action[2], bool(done)))
                returns.append(float(reward))
                taken += 1
                # tries=1 ends every step, so the next observation comes from reset.
                next_obs.append(None if done else new_obs)
            if any(obs is None for obs in next_obs):
                fresh = pool.reset([
                    None if obs is not None else seed + taken + index
                    for index, obs in enumerate(next_obs)
                ])
                next_obs = [old if old is not None else new for old, new in zip(next_obs, fresh)]
            observations = next_obs
            if len(buffer) >= min(32, steps):
                _apply_update(net, optimizer, buffer)
                buffer.clear()
        if buffer:
            _apply_update(net, optimizer, buffer)
        return net, returns
    finally:
        pool.close()


def _one_obs(nominal, seed):
    env = GraspEnv(nominal=nominal, max_tries=1, seed=seed)
    obs, _info = env.reset(seed)
    return obs


def _act(net, obs):
    with torch.no_grad():
        tensor = torch.as_tensor(np.asarray(obs), dtype=torch.float32).unsqueeze(0)
        action, log_prob, value = net.sample(tensor)
    return action.squeeze(0).numpy(), float(log_prob), float(value)


def _apply_update(net, optimizer, buffer):
    from .ppo import _update

    _update(net, optimizer, buffer, epochs=4, clip=0.2)


def _serve(conn, nominal, max_tries, seed):
    env = _Physical(GraspEnv(nominal=nominal, max_tries=max_tries, seed=seed))
    try:
        while True:
            command, payload = conn.recv()
            if command == "close":
                return
            if command == "reset":
                conn.send(env.reset(payload))
            elif command == "step":
                conn.send(env.step(payload))
    finally:
        conn.close()


class _Pool:
    def __init__(self, workers, nominal, max_tries, seed):
        ctx = mp.get_context("spawn")
        self.pipes = []
        self.procs = []
        for index in range(workers):
            parent, child = ctx.Pipe()
            proc = ctx.Process(target=_serve, args=(child, nominal, max_tries, seed + index))
            proc.start()
            child.close()
            self.pipes.append(parent)
            self.procs.append(proc)

    def reset(self, seeds):
        """`seeds` is an int per worker, or None to leave that worker in its current episode."""
        waiting = []
        for pipe, seed in zip(self.pipes, seeds):
            if seed is None:
                continue
            pipe.send(("reset", int(seed)))
            waiting.append(pipe)
        fresh = {id(pipe): pipe.recv()[0] for pipe in waiting}
        return [fresh.get(id(pipe)) for pipe in self.pipes]

    def step(self, actions):
        for pipe, action in zip(self.pipes, actions):
            pipe.send(("step", action))
        return [pipe.recv() for pipe in self.pipes]

    def close(self):
        for pipe in self.pipes:
            pipe.send(("close", None))
        for proc in self.procs:
            proc.join(timeout=30)


def save_checkpoint(path, algo, module, steps, seed):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save({
        "algo": algo,
        "state_dict": module.state_dict(),
        "obs_dim": int(module.body[0].in_features),
        "act_dim": 6,
        "hidden": int(module.body[0].out_features),
        "steps": int(steps),
        "seed": int(seed),
    }, path)


def load_module(path):
    blob = torch.load(path, map_location="cpu", weights_only=False)
    hidden = int(blob["hidden"])
    obs_dim = int(blob["obs_dim"])
    act_dim = int(blob["act_dim"])
    if blob["algo"] == "sac":
        module = Actor(obs_dim, act_dim, hidden)
    else:
        module = ActorCritic(obs_dim, act_dim, hidden)
    module.load_state_dict(blob["state_dict"])
    module.eval()
    return module, blob
