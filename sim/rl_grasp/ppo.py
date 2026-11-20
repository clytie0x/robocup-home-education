"""Clipped PPO for a continuous action in [-1, 1]."""

import numpy as np
import torch
from torch import optim

from .policy import ActorCritic, seed_all


def train(env, steps, seed, act_dim, hidden=64, batch=64, epochs=8, clip=0.2, lr=3e-4,
          zero_mean=False, log_std_init=-0.5):
    """`steps` environment transitions. Returns the network and each episode's return."""
    seed_all(seed)
    obs, _info = env.reset(seed)
    obs_dim = int(np.asarray(obs).shape[0])
    net = ActorCritic(obs_dim, act_dim, hidden, zero_mean=zero_mean, log_std_init=log_std_init)
    optimizer = optim.Adam(net.parameters(), lr=lr)
    episode_returns = []
    running = 0.0
    buffer = []
    taken = 0
    while taken < steps:
        action, log_prob, value = _act(net, obs)
        next_obs, reward, done, _info = env.step(action)
        buffer.append((obs, action, log_prob, float(reward), value, bool(done)))
        running += float(reward)
        taken += 1
        if done:
            episode_returns.append(running)
            running = 0.0
            obs, _info = env.reset()
        else:
            obs = next_obs
        if len(buffer) >= batch:
            _update(net, optimizer, buffer, epochs, clip)
            buffer.clear()
    if buffer:
        _update(net, optimizer, buffer, epochs, clip)
    return net, episode_returns


def _act(net, obs):
    with torch.no_grad():
        tensor = torch.as_tensor(np.asarray(obs), dtype=torch.float32).unsqueeze(0)
        action, log_prob, value = net.sample(tensor)
    return action.squeeze(0).numpy(), float(log_prob), float(value)


def _update(net, optimizer, buffer, epochs, clip):
    obs = torch.as_tensor(np.asarray([row[0] for row in buffer], dtype=np.float32))
    action = torch.as_tensor(np.asarray([row[1] for row in buffer], dtype=np.float32))
    old_log = torch.as_tensor([row[2] for row in buffer], dtype=torch.float32)
    reward = torch.as_tensor([row[3] for row in buffer], dtype=torch.float32)
    value = torch.as_tensor([row[4] for row in buffer], dtype=torch.float32)
    done = torch.as_tensor([float(row[5]) for row in buffer], dtype=torch.float32)
    returns = _discount(reward, done)
    advantage = returns - value
    # One-step tries often share a reward. Dividing by a tiny std turns that
    # noise into a huge step and walks the mean off the scripted action.
    spread = advantage.std()
    advantage = advantage - advantage.mean()
    if float(spread) > 1e-3:
        advantage = advantage / (spread + 1e-8)
    for _ in range(epochs):
        log_prob, entropy, new_value = net.log_prob(obs, action)
        ratio = torch.exp(log_prob - old_log)
        unclipped = ratio * advantage
        clipped = torch.clamp(ratio, 1.0 - clip, 1.0 + clip) * advantage
        policy_loss = -torch.minimum(unclipped, clipped).mean()
        value_loss = (new_value - returns).pow(2).mean()
        loss = policy_loss + 0.5 * value_loss - 0.01 * entropy.mean()
        optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(net.parameters(), 0.5)
        optimizer.step()


def _discount(reward, done, gamma=0.99):
    out = torch.zeros_like(reward)
    running = 0.0
    for index in range(len(reward) - 1, -1, -1):
        if bool(done[index]):
            running = 0.0
        running = float(reward[index]) + gamma * running
        out[index] = running
    return out
