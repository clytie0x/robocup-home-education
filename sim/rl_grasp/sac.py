"""Soft actor-critic with a fixed temperature. Continuous actions in [-1, 1]."""

import numpy as np
import torch
from torch import optim

from .policy import Actor, QNet, seed_all


def train(env, steps, seed, act_dim, hidden=64, batch=64, lr=3e-4, alpha=0.05, warmup=64):
    """`steps` environment transitions. Returns the actor and each episode's return."""
    seed_all(seed)
    obs, _info = env.reset(seed)
    obs_dim = int(np.asarray(obs).shape[0])
    actor = Actor(obs_dim, act_dim, hidden)
    q1, q2 = QNet(obs_dim, act_dim, hidden), QNet(obs_dim, act_dim, hidden)
    q1_t, q2_t = QNet(obs_dim, act_dim, hidden), QNet(obs_dim, act_dim, hidden)
    q1_t.load_state_dict(q1.state_dict())
    q2_t.load_state_dict(q2.state_dict())
    actor_opt = optim.Adam(actor.parameters(), lr=lr)
    q_opt = optim.Adam(list(q1.parameters()) + list(q2.parameters()), lr=lr)
    replay = []
    episode_returns = []
    running = 0.0
    for _ in range(steps):
        action = _act(actor, obs, warmup=len(replay) < warmup)
        next_obs, reward, done, _info = env.step(action)
        replay.append((obs, action, float(reward), next_obs, bool(done)))
        running += float(reward)
        if done:
            episode_returns.append(running)
            running = 0.0
            obs, _info = env.reset()
        else:
            obs = next_obs
        if len(replay) >= warmup:
            _update(actor, q1, q2, q1_t, q2_t, actor_opt, q_opt, replay, batch, alpha)
    return actor, episode_returns


def _act(actor, obs, warmup):
    if warmup:
        return np.random.uniform(-1.0, 1.0, size=actor.mean.out_features)
    with torch.no_grad():
        tensor = torch.as_tensor(np.asarray(obs), dtype=torch.float32).unsqueeze(0)
        action, _log_prob = actor.sample(tensor)
    return action.squeeze(0).numpy()


def _update(actor, q1, q2, q1_t, q2_t, actor_opt, q_opt, replay, batch, alpha, gamma=0.99, tau=0.005):
    pick = np.random.randint(0, len(replay), size=batch)
    rows = [replay[index] for index in pick]
    obs = torch.as_tensor(np.asarray([row[0] for row in rows], dtype=np.float32))
    action = torch.as_tensor(np.asarray([row[1] for row in rows], dtype=np.float32))
    reward = torch.as_tensor([row[2] for row in rows], dtype=torch.float32)
    nxt = torch.as_tensor(np.asarray([row[3] for row in rows], dtype=np.float32))
    done = torch.as_tensor([float(row[4]) for row in rows], dtype=torch.float32)
    with torch.no_grad():
        next_action, next_log = actor.sample(nxt)
        target_q = torch.minimum(q1_t(nxt, next_action), q2_t(nxt, next_action))
        target = reward + gamma * (1.0 - done) * (target_q - alpha * next_log)
    loss_q = (q1(obs, action) - target).pow(2).mean() + (q2(obs, action) - target).pow(2).mean()
    q_opt.zero_grad()
    loss_q.backward()
    q_opt.step()
    new_action, log_prob = actor.sample(obs)
    loss_actor = (alpha * log_prob - torch.minimum(q1(obs, new_action), q2(obs, new_action))).mean()
    actor_opt.zero_grad()
    loss_actor.backward()
    actor_opt.step()
    _soft(q1, q1_t, tau)
    _soft(q2, q2_t, tau)


def _soft(source, target, tau):
    for online, frozen in zip(source.parameters(), target.parameters()):
        frozen.data.mul_(1.0 - tau).add_(online.data, alpha=tau)
