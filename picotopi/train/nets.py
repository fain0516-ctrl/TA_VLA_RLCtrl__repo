# -*- coding: utf-8 -*-
"""Actor(관측만) / 비대칭 Critic(관측 + 특권 ps·pd + 행동) / PopArt."""
import torch
import torch.nn as nn
import torch.nn.functional as F

from .. import config as C


class Actor(nn.Module):
    """파라미터 순서는 policy.numpy_actor.PARAM_NAMES와 같아야 합니다 (워커·실기 추론 공용)."""

    def __init__(s, i, h):
        super().__init__()
        s.fc1, s.fc2 = nn.Linear(i, h), nn.Linear(h, h)
        s.mu, s.ls = nn.Linear(h, C.ACT_DIM), nn.Linear(h, C.ACT_DIM)
        nn.init.uniform_(s.mu.weight, -1e-3, 1e-3); nn.init.zeros_(s.mu.bias)
        nn.init.constant_(s.ls.bias, -0.5)

    def forward(s, x):
        h = F.relu(s.fc2(F.relu(s.fc1(x))))
        mu, ls = s.mu(h), s.ls(h).clamp(-5, 2)
        u = mu + ls.exp() * torch.randn_like(mu)
        a = torch.tanh(u)
        lp = (-0.5 * ((u - mu) / ls.exp()) ** 2 - ls - 0.9189385).sum(-1, keepdim=True)
        return a, lp - torch.log(1 - a ** 2 + 1e-6).sum(-1, keepdim=True)


class Critic(nn.Module):
    def __init__(s, i, h):
        super().__init__()
        mk = lambda: nn.Sequential(nn.Linear(i, h), nn.ReLU(), nn.Linear(h, h), nn.ReLU())
        s.t1, s.t2, s.h1, s.h2 = mk(), mk(), nn.Linear(h, 1), nn.Linear(h, 1)
        s.register_buffer("mu", torch.zeros(1))
        s.register_buffer("nu", torch.ones(1))
        s.register_buffer("sigma", torch.ones(1))

    def forward(s, x):
        return s.h1(s.t1(x)), s.h2(s.t2(x))

    def denorm(s, q):
        return q * s.sigma + s.mu


@torch.no_grad()
def popart_update(on, tg, y, beta):
    """PopArt: 타깃 통계 적응 정규화 + 출력 보존 재조정."""
    mu_o, sg_o = on.mu.clone(), on.sigma.clone()
    on.mu.lerp_(y.mean().reshape(1), beta)
    on.nu.lerp_((y * y).mean().reshape(1), beta)
    on.sigma.copy_((on.nu - on.mu * on.mu).clamp(min=1e-4).sqrt())
    scale, shift = sg_o / on.sigma, (mu_o - on.mu) / on.sigma
    for net in (on, tg):
        for hd in (net.h1, net.h2):
            hd.weight.mul_(scale)
            hd.bias.mul_(scale).add_(shift)
    tg.mu.copy_(on.mu); tg.nu.copy_(on.nu); tg.sigma.copy_(on.sigma)
