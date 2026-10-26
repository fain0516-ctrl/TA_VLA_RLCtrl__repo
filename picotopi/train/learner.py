# -*- coding: utf-8 -*-
"""SAC 업데이트: twin Q + 타깃 네트워크(soft update) + 자동 온도 α + PopArt 정규화 타깃.

CaT 확률 종료만 term=1(부트스트랩 차단), 시간 초과는 부트스트랩합니다.
"""
import copy
import math

import torch
import torch.nn.functional as F

from .. import config as C
from ..sim.env import OBS_DIM, PS_DIM, PD_DIM
from .nets import Actor, Critic, popart_update


class Learner:
    def __init__(s, cfg, dev):
        h, s.cfg, s.dev = cfg["hidden"], cfg, dev
        s.actor = Actor(OBS_DIM, h).to(dev)
        s.critic = Critic(OBS_DIM + PS_DIM + PD_DIM + C.ACT_DIM, h).to(dev)
        s.critic_t = copy.deepcopy(s.critic)
        for p in s.critic_t.parameters():
            p.requires_grad_(False)
        s.log_alpha = torch.full((1,), math.log(cfg["alpha0"]), device=dev, requires_grad=True)
        s.oa = torch.optim.Adam(s.actor.parameters(), lr=cfg["lr"])
        s.oc = torch.optim.Adam(s.critic.parameters(), lr=cfg["lr"])
        s.ol = torch.optim.Adam([s.log_alpha], lr=cfg["lr"])
        s.H_t = -float(C.ACT_DIM)
        s.stat = [0.0] * 4

    def update(s, B):
        c = s.cfg
        with torch.no_grad():
            alpha = s.log_alpha.exp()
            a2, lp2 = s.actor(B["obs2"])
            q1t, q2t = s.critic_t(torch.cat([B["obs2"], B["psi2"], a2], -1))
            qt = torch.min(s.critic_t.denorm(q1t), s.critic_t.denorm(q2t))
            y = B["r"] + c["gamma"] * (1 - B["term"]) * (qt - alpha * lp2)
            popart_update(s.critic, s.critic_t, y, c["popart_beta"])
            yn = (y - s.critic.mu) / s.critic.sigma
        q1, q2 = s.critic(torch.cat([B["obs"], B["psi"], B["act"]], -1))
        lc = F.mse_loss(q1, yn) + F.mse_loss(q2, yn)
        s.oc.zero_grad(set_to_none=True); lc.backward(); s.oc.step()

        for p in s.critic.parameters():
            p.requires_grad_(False)
        a, lp = s.actor(B["obs"])
        q1p, q2p = s.critic(torch.cat([B["obs"], B["psi"], a], -1))
        la = (alpha * lp - torch.min(s.critic.denorm(q1p), s.critic.denorm(q2p))).mean()
        s.oa.zero_grad(set_to_none=True); la.backward(); s.oa.step()
        for p in s.critic.parameters():
            p.requires_grad_(True)

        ll = -(s.log_alpha * (lp.detach() + s.H_t)).mean()
        s.ol.zero_grad(set_to_none=True); ll.backward(); s.ol.step()
        with torch.no_grad():
            for p, pt in zip(s.critic.parameters(), s.critic_t.parameters()):
                pt.lerp_(p, c["tau"])
        s.stat = [lc.item(), la.item(), alpha.item(), y.mean().item()]

    def flat_actor(s):
        return torch.cat([p.detach().reshape(-1).cpu() for p in s.actor.parameters()]).numpy()
