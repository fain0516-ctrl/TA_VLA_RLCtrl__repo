# -*- coding: utf-8 -*-
"""에피소드 리플레이 + HER (future 전략, 샘플 시점 목표 의존 보상 재계산).

버퍼에는 목표 무관 보상 r_np만 저장하고, 목표 의존 항(힘 추종·유지)은 샘플할 때 재라벨 목표 g로 다시 계산합니다.
Actor 관측 안의 g 칸(G_IDX)도 함께 교체합니다 [가이드 P1-2, FORGE].
"""
import torch

from .. import config as C
from ..control.observation import G_IDX
from ..sim.env import OBS_DIM, PS_DIM, PD_DIM, AG_DIM


class EpisodeReplay:
    def __init__(s, cap, T, dev):
        s.cap, s.T, s.dev, s.ptr, s.filled = cap, T, dev, 0, 0
        z = lambda *sh, dt=torch.float32: torch.zeros(*sh, dtype=dt, device=dev)
        s.b = dict(obs=z(cap, T + 1, OBS_DIM), ag=z(cap, T + 1, AG_DIM), pd=z(cap, T + 1, PD_DIM),
                   act=z(cap, T, C.ACT_DIM), r_np=z(cap, T), term=z(cap, T), g=z(cap, 1), ps=z(cap, PS_DIM),
                   len=z(cap, dt=torch.long))
        s.cum = z(cap, dt=torch.long)

    def add(s, eps):
        for ep in eps:
            i, L = s.ptr, len(ep["act"])
            for k in ("obs", "ag", "pd"):
                s.b[k][i, :L + 1] = torch.as_tensor(ep[k], device=s.dev)
            for k in ("act", "r_np", "term"):
                s.b[k][i, :L] = torch.as_tensor(ep[k], device=s.dev)
                s.b[k][i, L:] = 0
            s.b["g"][i] = torch.as_tensor(ep["g"], device=s.dev)
            s.b["ps"][i] = torch.as_tensor(ep["ps"], device=s.dev)
            s.b["len"][i] = L
            s.ptr, s.filled = (s.ptr + 1) % s.cap, min(s.filled + 1, s.cap)
        torch.cumsum(s.b["len"], 0, out=s.cum)
        return sum(len(e["act"]) for e in eps)

    def sample(s, N, rew, p_her):
        d, b, m = s.dev, s.b, rew.m_hold
        tot = s.cum[-1]
        flat = torch.minimum((torch.rand(N, device=d) * tot).long(), tot - 1)
        e = torch.searchsorted(s.cum, flat, right=True)
        L = b["len"][e]
        t = flat - (s.cum[e] - L)
        jj = torch.arange(m, device=d)
        ag2 = b["ag"][e, t + 1]
        hist2 = b["ag"][e[:, None], ((t + 1)[:, None] - jj[None]).clamp(min=0)][..., 0]
        # future: 미래 m스텝 창의 평균 힘을 새 목표로, 원 목표 분포 [g_min, 0.8 F_th] 안일 때만
        fut = torch.minimum(t + 1 + (torch.rand(N, device=d) * (L - t).float()).long(), L)
        gwin = b["ag"][e[:, None], torch.minimum(fut[:, None] + jj[None], L[:, None])][..., 0].mean(1)
        F_th = b["ps"][e, -1]
        ok = (torch.rand(N, device=d) < p_her) & (gwin >= rew.g_min) & (gwin <= C.G_HI_FRAC * F_th)
        g = torch.where(ok, gwin, b["g"][e, 0])
        r_g, _ = rew.goal_terms_torch(ag2[:, 0] * C.F_REF, ag2[:, 1], hist2 * C.F_REF, g * C.F_REF)
        r = r_g + b["r_np"][e, t]
        obs, obs2 = b["obs"][e, t].clone(), b["obs"][e, t + 1].clone()
        obs[:, G_IDX], obs2[:, G_IDX] = g, g
        ps = b["ps"][e]
        return dict(obs=obs, obs2=obs2, psi=torch.cat([ps, b["pd"][e, t]], -1),
                    psi2=torch.cat([ps, b["pd"][e, t + 1]], -1), act=b["act"][e, t], r=r[:, None],
                    term=b["term"][e, t][:, None])
