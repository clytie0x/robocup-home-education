"""Small Gaussian policies. Actions inside the network are in [-1, 1]."""

import numpy as np
import torch
from torch import nn
from torch.distributions import Normal

from . import constants as C


def seed_all(seed):
    np.random.seed(int(seed))
    torch.manual_seed(int(seed))


def to_physical(normal):
    normal = np.clip(np.asarray(normal, dtype=float), -1.0, 1.0)
    low = np.asarray(C.ACTION_LOW, dtype=float)
    high = np.asarray(C.ACTION_HIGH, dtype=float)
    return low + (normal + 1.0) * 0.5 * (high - low)


def to_normal(physical):
    low = np.asarray(C.ACTION_LOW, dtype=float)
    high = np.asarray(C.ACTION_HIGH, dtype=float)
    scaled = 2.0 * (np.asarray(physical, dtype=float) - low) / (high - low) - 1.0
    return np.clip(scaled, -1.0, 1.0)


def _trunk(in_dim, hidden):
    return nn.Sequential(
        nn.Linear(in_dim, hidden),
        nn.Tanh(),
        nn.Linear(hidden, hidden),
        nn.Tanh(),
    )


class ActorCritic(nn.Module):
    """PPO actor and value, one trunk."""

    def __init__(self, obs_dim, act_dim, hidden=64):
        super().__init__()
        self.body = _trunk(obs_dim, hidden)
        self.mean = nn.Linear(hidden, act_dim)
        self.value = nn.Linear(hidden, 1)
        self.log_std = nn.Parameter(torch.full((act_dim,), -0.5))

    def forward(self, obs):
        features = self.body(obs)
        mean = self.mean(features)
        std = self.log_std.clamp(-4.0, 1.0).exp()
        return Normal(mean, std), self.value(features).squeeze(-1)

    def sample(self, obs):
        dist, value = self.forward(obs)
        raw = dist.rsample()
        action = torch.tanh(raw)
        log_prob = dist.log_prob(raw).sum(-1)
        log_prob = log_prob - torch.log(1.0 - action.pow(2) + 1e-6).sum(-1)
        return action, log_prob, value

    def log_prob(self, obs, action):
        dist, value = self.forward(obs)
        clipped = action.clamp(-0.999, 0.999)
        raw = 0.5 * torch.log((1.0 + clipped) / (1.0 - clipped))
        log_prob = dist.log_prob(raw).sum(-1)
        log_prob = log_prob - torch.log(1.0 - clipped.pow(2) + 1e-6).sum(-1)
        entropy = dist.entropy().sum(-1)
        return log_prob, entropy, value

    def act_mean(self, obs):
        dist, _value = self.forward(obs)
        return torch.tanh(dist.mean)


class Actor(nn.Module):
    """SAC actor. Twin Q networks live next to the update."""

    def __init__(self, obs_dim, act_dim, hidden=64):
        super().__init__()
        self.body = _trunk(obs_dim, hidden)
        self.mean = nn.Linear(hidden, act_dim)
        self.log_std = nn.Linear(hidden, act_dim)

    def forward(self, obs):
        features = self.body(obs)
        mean = self.mean(features)
        log_std = self.log_std(features).clamp(-4.0, 1.0)
        return mean, log_std

    def sample(self, obs):
        mean, log_std = self.forward(obs)
        dist = Normal(mean, log_std.exp())
        raw = dist.rsample()
        action = torch.tanh(raw)
        log_prob = dist.log_prob(raw).sum(-1)
        log_prob = log_prob - torch.log(1.0 - action.pow(2) + 1e-6).sum(-1)
        return action, log_prob

    def act_mean(self, obs):
        mean, _log_std = self.forward(obs)
        return torch.tanh(mean)


class QNet(nn.Module):
    def __init__(self, obs_dim, act_dim, hidden=64):
        super().__init__()
        self.body = nn.Sequential(
            nn.Linear(obs_dim + act_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, 1),
        )

    def forward(self, obs, action):
        return self.body(torch.cat((obs, action), dim=-1)).squeeze(-1)


def deterministic_action(module, obs):
    """Physical grasp parameters from one observation vector."""
    with torch.no_grad():
        tensor = torch.as_tensor(obs, dtype=torch.float32).unsqueeze(0)
        normal = module.act_mean(tensor).squeeze(0).numpy()
    return to_physical(normal)
