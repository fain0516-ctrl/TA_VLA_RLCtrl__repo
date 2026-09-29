#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""V8 학습기: 목표 조건부 SAC + HER (1지 MVP).

V7 학습기에서 유지한 것: CPU 워커(NumPy 추론) + 에피소드 리플레이 + 샘플 시점 HER 보상 재계산, 비대칭 Critic,
PopArt, 경계 평가 ADR.
V7 대비 바뀐 것 (docs/research/04_team_guide.md):
- 환경/보상/안전층 교체 (v8/env.py, v8/reward.py)
- Actor 관측에 목표 힘 g와 파손 한계 F_th 포함 -> HER 재라벨 시 관측의 g 칸도 함께 교체      [P1-2, FORGE]
- 잔차 base 서보·Dense2Sparse 스케줄 제거: 보상이 유계라 필요 없고 critic 행동 정의가 단순해짐
- CaT 확률 종료만 d=1, 시간 초과는 bootstrap
- 반경 커리큘럼(좁은 범위 -> 1.5~4.0 cm)을 성능 기반으로 확장                             [P1-5]
- 시드 인자, CSV 로그, 체크포인트에 설정 스냅샷 저장                                        [P0-3]
CUDA Graph / pinned 더블버퍼 최적화는 V8 1지 규모에서 병목이 아니라 빼고, 가독성을 우선했습니다.
대규모(4지 이상) 학습은 가이드 P1-4대로 MJX PPO 경로를 권장합니다.
"""
import os
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
import sys
import copy
import json
import math
import time
import queue as pyq
import argparse
import multiprocessing as mp
from collections import deque
from dataclasses import asdict

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from v8 import config as C                                                      # noqa: E402
from v8.env import OBS_DIM, PS_DIM, PD_DIM, AG_DIM                              # noqa: E402
from v8.reward import RewardV8                                                   # noqa: E402

CFG = dict(T=C.T_EP, cap_eps=20000, batch=512, utd=0.5, gamma=0.98, tau=0.005, lr=3e-4, p_her=0.6,
           total=1_000_000, warmup=10_000, sync_every=2000, hidden=256, alpha0=0.1, popart_beta=1e-3,
           p_boundary=0.3, adr_n=20, adr_step=0.05, adr_hi=0.30, adr_lo=0.10,
           curr_thr=0.30, curr_step=0.1, curr_n=100)
G_IDX = OBS_DIM - 2          # 관측 안의 g 위치 (g/F_REF)
ADR_SIDES = [(j, s) for j in range(len(C.DR_KEYS)) for s in (0, 1)
             if C.DR[C.DR_KEYS[j]][s] != C.DR[C.DR_KEYS[j]][s + 2]]


# ============================================================ 공유 상태 / 워커
class Shared:
    def __init__(s, n_par, ctx):
        s.w, s.ver = ctx.Array("f", n_par), ctx.Value("i", 0)
        s.stop = ctx.Value("i", 0)
        s.curr = ctx.Value("d", 0.0)
        s.adr = ctx.Array("d", 2 * len(C.DR_KEYS))

    def publish(s, flat):
        with s.w.get_lock():
            np.frombuffer(s.w.get_obj(), dtype=np.float32)[:] = flat
        s.ver.value += 1

    def fetch(s, spec):
        with s.w.get_lock():
            flat = np.frombuffer(s.w.get_obj(), dtype=np.float32).copy()
        out, o = [], 0
        for shp in spec:
            n = int(np.prod(shp))
            out.append(flat[o:o + n].reshape(shp))
            o += n
        return out


def np_policy(p, x, deterministic=False, rng=np.random):
    h = np.maximum(p[0] @ x + p[1], 0.0)
    h = np.maximum(p[2] @ h + p[3], 0.0)
    mu = p[4] @ h + p[5]
    if deterministic:
        return np.tanh(mu)
    ls = np.clip(p[6] @ h + p[7], -5.0, 2.0)
    return np.tanh(mu + np.exp(ls) * rng.standard_normal(mu.shape[0]))


def worker(rank, sh, spec, q, seed, p_b):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from v8.env import FingerGraspEnvV8
    env = FingerGraspEnvV8(seed=seed * 1000 + rank)
    rng = np.random.default_rng(seed * 1000 + rank + 7)
    pol, ver = None, -1

    def start():
        with sh.adr.get_lock():
            env.adr = np.frombuffer(sh.adr.get_obj(), dtype=np.float64).reshape(-1, 2).copy()
        env.curr = sh.curr.value
        bnd = int(rng.integers(len(ADR_SIDES))) if rng.random() < p_b else -1
        o = env.reset(boundary=None if bnd < 0 else ADR_SIDES[bnd])
        return o, bnd, dict(obs=[o["obs"]], ag=[o["ag"]], pd=[o["pd"]], act=[], r_np=[], term=[],
                            g=np.array([o["g"]]), ps=o["ps"])

    obs, bnd, ep = start()
    stats = dict(hold=0.0, crushed=0.0, peak=0.0)
    while not sh.stop.value:
        if sh.ver.value != ver:
            ver, pol = sh.ver.value, sh.fetch(spec)
        a = np_policy(pol, obs["obs"], rng=rng)
        obs, done, info = env.step(a)
        ep["obs"].append(obs["obs"]); ep["ag"].append(obs["ag"]); ep["pd"].append(obs["pd"])
        ep["act"].append(a.astype(np.float32)); ep["r_np"].append(info["r_np"]); ep["term"].append(float(info["term"]))
        stats["hold"] += info["hold"]
        stats["crushed"] = max(stats["crushed"], float(info["crushed"]))
        stats["peak"] = max(stats["peak"], info["F_peak"])
        if done:
            out = {k: np.asarray(v, dtype=np.float32) for k, v in ep.items()}
            out.update(hold=stats["hold"] / C.T_EP, crushed=stats["crushed"], peak=stats["peak"], bnd=bnd)
            q.put(out)
            obs, bnd, ep = start()
            stats = dict(hold=0.0, crushed=0.0, peak=0.0)


# ============================================================ 네트워크
class Actor(nn.Module):
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
    """PopArt: 타깃 통계 적응 정규화 + 출력 보존 재조정 (V7과 동일)."""
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


# ============================================================ 에피소드 리플레이 + HER
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
        # HER future 전략: 미래 m스텝 창의 평균 힘을 새 목표로, 원 목표 분포 [g_min, 0.8 F_th] 안일 때만
        fut = torch.minimum(t + 1 + (torch.rand(N, device=d) * (L - t).float()).long(), L)
        gwin = b["ag"][e[:, None], torch.minimum(fut[:, None] + jj[None], L[:, None])][..., 0].mean(1)
        F_th = b["ps"][e, -1]
        ok = (torch.rand(N, device=d) < p_her) & (gwin >= rew.g_min) & (gwin <= C.G_HI_FRAC * F_th)
        g = torch.where(ok, gwin, b["g"][e, 0])
        r_g, _ = rew.goal_terms_torch(ag2[:, 0] * C.F_REF, ag2[:, 1], hist2 * C.F_REF, g * C.F_REF)
        r = r_g + b["r_np"][e, t]
        obs, obs2 = b["obs"][e, t].clone(), b["obs"][e, t + 1].clone()
        obs[:, G_IDX], obs2[:, G_IDX] = g, g          # Actor 관측 속 목표도 재라벨
        ps = b["ps"][e]
        return dict(obs=obs, obs2=obs2, psi=torch.cat([ps, b["pd"][e, t]], -1),
                    psi2=torch.cat([ps, b["pd"][e, t + 1]], -1), act=b["act"][e, t], r=r[:, None],
                    term=b["term"][e, t][:, None])


# ============================================================ 학습기
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


# ============================================================ 메인 루프
def train(total=CFG["total"], workers=None, seed=0, out_dir=None, cfg=CFG, log_every=20000, device=None):
    out_dir = out_dir or os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs", f"seed{seed}")
    os.makedirs(out_dir, exist_ok=True)
    dev = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    nw = workers or max(2, min(12, (os.cpu_count() or 4) - 2))
    torch.manual_seed(seed); np.random.seed(seed)
    rew = RewardV8()
    buf = EpisodeReplay(cfg["cap_eps"], cfg["T"], dev)
    lr = Learner(cfg, dev)
    spec = [tuple(p.shape) for p in lr.actor.parameters()]
    ctx = mp.get_context("spawn")         # Windows 기본과 동일, Linux에서도 같은 동작
    sh = Shared(sum(int(np.prod(x)) for x in spec), ctx)
    sh.publish(lr.flat_actor())
    meta = dict(cfg=cfg, reward=asdict(rew), seed=seed, dr=C.DR, tendon_motor=C.TENDON_MOTOR,
                policy_hz=1 / C.POLICY_DT, obs_dim=OBS_DIM)
    with open(os.path.join(out_dir, "config.json"), "w") as f:
        json.dump(meta, f, indent=1, default=str)

    q = ctx.Queue(maxsize=512)
    procs = [ctx.Process(target=worker, args=(i, sh, spec, q, seed, cfg["p_boundary"]), daemon=True)
             for i in range(nw)]
    for p in procs:
        p.start()

    n = upd = last_sync = 0
    phi = np.zeros((len(C.DR_KEYS), 2))
    adr_buf = [[] for _ in ADR_SIDES]
    nb_hold, stats = deque(maxlen=cfg["curr_n"]), deque(maxlen=200)
    next_log, t0 = log_every, time.time()
    log = open(os.path.join(out_dir, "log.csv"), "w")
    log.write("n,upd,hold,crush,peak,curr,adr,Lc,La,alpha,ymean,tps\n")
    try:
        while n < total:
            eps = []
            while len(eps) < 64:
                try:
                    eps.append(q.get(timeout=0.01) if not eps else q.get_nowait())
                except pyq.Empty:
                    break
            if eps:
                n += buf.add(eps)
                for ep in eps:
                    stats.append((ep["hold"], ep["crushed"], ep["peak"]))
                    if ep["bnd"] < 0:
                        nb_hold.append(ep["hold"])
                        continue
                    ab = adr_buf[ep["bnd"]]
                    ab.append(ep["hold"])
                    if len(ab) >= cfg["adr_n"]:
                        mh, (j, sd) = float(np.mean(ab)), ADR_SIDES[ep["bnd"]]
                        ab.clear()
                        if mh > cfg["adr_hi"]:
                            phi[j, sd] = min(1.0, phi[j, sd] + cfg["adr_step"])
                        elif mh < cfg["adr_lo"]:
                            phi[j, sd] = max(0.0, phi[j, sd] - cfg["adr_step"])
                        with sh.adr.get_lock():
                            np.frombuffer(sh.adr.get_obj(), dtype=np.float64)[:] = phi.ravel()
                if len(nb_hold) >= cfg["curr_n"] and np.mean(nb_hold) >= cfg["curr_thr"] and sh.curr.value < 1.0:
                    sh.curr.value = min(1.0, sh.curr.value + cfg["curr_step"])
                    nb_hold.clear()

            if n >= cfg["warmup"]:
                target = int(cfg["utd"] * (n - cfg["warmup"]))
                while upd < target:
                    lr.update(buf.sample(cfg["batch"], rew, cfg["p_her"]))
                    upd += 1
                if upd - last_sync >= cfg["sync_every"]:
                    sh.publish(lr.flat_actor())
                    last_sync = upd

            if n >= next_log and stats:
                st = np.mean(stats, 0)
                row = [n, upd, st[0], st[1], st[2], sh.curr.value, phi.mean(), *lr.stat, n / (time.time() - t0)]
                log.write(",".join(f"{v:.4g}" for v in row) + "\n"); log.flush()
                print(f"[V8] n={n:8d} upd={upd:7d} hold={st[0]:.3f} crush={st[1]:.2f} peak={st[2]:.2f}N "
                      f"curr={sh.curr.value:.1f} adr={phi.mean():.2f} Lc={lr.stat[0]:.3f} La={lr.stat[1]:.3f} "
                      f"alpha={lr.stat[2]:.3f} tps={row[-1]:.0f}", flush=True)
                torch.save(dict(actor=lr.actor.state_dict(), critic=lr.critic.state_dict(), adr=phi,
                                curr=sh.curr.value, n=n, meta=meta), os.path.join(out_dir, "v8_ckpt.pt"))
                next_log += log_every
    finally:
        sh.stop.value = 1
        t_end = time.time() + 3
        while time.time() < t_end and any(p.is_alive() for p in procs):
            try:
                q.get(timeout=0.1)
            except pyq.Empty:
                pass
        for p in procs:
            p.terminate()
        log.close()
        torch.save(lr.actor.state_dict(), os.path.join(out_dir, "final_actor_v8.pth"))
    return lr, out_dir


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--total", type=int, default=CFG["total"])
    ap.add_argument("--workers", type=int, default=None)
    ap.add_argument("--warmup", type=int, default=CFG["warmup"])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--log-every", type=int, default=20000)
    ap.add_argument("--device", default=None)
    a = ap.parse_args()
    cfg = dict(CFG, warmup=a.warmup)
    train(a.total, a.workers, a.seed, a.out, cfg, a.log_every, a.device)
