#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
import sys
import math
import time
import copy
import queue as pyq
import argparse
import multiprocessing as mp
from collections import deque

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from universal_grasp_env_v7_final import (OBS_DIM, GOAL_DIM, ACT_DIM, PS_DIM, PD_DIM, PSI_DIM, FRAME, F_IDX, CMD_IDX,
                                          DELTA_MAX, DELTA_BASE, DELTA_RES, K_F, DR_KEYS, ADR_SIDES)
from train_sac_her_v7_final import RewardTunerV7, F_REF, TH_MAX

# [Opus Fix: CUDA 빌드 torch 필수 (pip install torch --index-url https://download.pytorch.org/whl/cu121), 미설치 시 CPU eager 폴백]
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
CUDA = device.type == "cuda"

CFG = dict(T=150, cap_eps=20000, batch=512, chunk_upd=32, utd=0.5, gamma=0.99, tau=0.005, lr=3e-4, p_her=0.8,
           total=2_000_000, warmup=10000, sync_upd=128, lag_tr=20000, ramp_steps=100_000, alpha0=0.05,
           popart_beta=1e-3, stage_n=64, p_boundary=0.4, adr_n=20, adr_step=0.05, adr_hi=0.20, adr_lo=0.05,
           phase_thr=0.15, phase_min=100_000, max_chunks_per_loop=4, hidden=256)


# ============================================================ shared state / CPU workers
class Shared:
    def __init__(s, n_par):
        s.w, s.ver = mp.Array("f", n_par), mp.Value("i", 0)
        s.ramp, s.phase = mp.Value("d", 0.0), mp.Value("i", 1)
        s.stop, s.pause = mp.Value("i", 0), mp.Value("i", 0)
        s.adr = mp.Array("d", 2 * len(DR_KEYS))

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

    def set_adr(s, phi):
        with s.adr.get_lock():
            np.frombuffer(s.adr.get_obj(), dtype=np.float64)[:] = phi.ravel()

    def get_adr(s):
        with s.adr.get_lock():
            return np.frombuffer(s.adr.get_obj(), dtype=np.float64).reshape(-1, 2).copy()


def np_policy(p, x):
    h = np.maximum(p[0] @ x + p[1], 0.0)
    h = np.maximum(p[2] @ h + p[3], 0.0)
    mu, ls = p[4] @ h + p[5], np.clip(p[6] @ h + p[7], -5.0, 2.0)
    return np.tanh(mu + np.exp(ls) * np.random.randn(mu.shape[0]))


def worker(rank, sh, spec, q, seed, p_b):
    # [Opus Fix: 워커는 NumPy 추론만 수행 -> 프로세스별 CUDA 컨텍스트(수백 MB VRAM) 생성 방지]
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from universal_grasp_env_v7_final import UniversalTactileGraspEnvV7
    np.random.seed(seed + rank)
    env, pol, ver = UniversalTactileGraspEnvV7(1), None, 0

    def start():
        env.adr = sh.get_adr()
        bnd = int(np.random.randint(len(ADR_SIDES))) if np.random.rand() < p_b else -1
        o = env.reset(phase=sh.phase.value, boundary=None if bnd < 0 else bnd)
        return o, bnd, dict(obs=[o["obs"]], ag=[o["ag"]], pd=[o["pd"]], act=[], r_np=[], r_sf=[], term=[],
                            g=o["goal"], ps=o["ps"])

    obs, bnd, ep = start()
    while not sh.stop.value:
        while sh.pause.value and not sh.stop.value:
            time.sleep(0.005)
        if sh.ver.value != ver:
            ver, pol = sh.ver.value, sh.fetch(spec)
        if pol is None:
            time.sleep(0.01)
            continue
        env.delta_res = DELTA_RES * sh.ramp.value
        a = np_policy(pol, np.concatenate([obs["obs"], obs["goal"]]))
        obs, done, info = env.step(a)
        ep["obs"].append(obs["obs"]); ep["ag"].append(obs["ag"]); ep["pd"].append(obs["pd"])
        ep["act"].append(info["act"]); ep["r_np"].append(info["r_np"]); ep["r_sf"].append(info["r_sf"])
        ep["term"].append(float(info["term"]))
        if done:
            q.put({k: np.asarray(v, dtype=np.float32) for k, v in ep.items()} |
                  dict(hold=info["hold"], crushed=info["crushed"], missed=info["missed"], bnd=bnd))
            obs, bnd, ep = start()


# ============================================================ networks
class Actor(nn.Module):
    def __init__(s, i, h):
        super().__init__()
        s.fc1, s.fc2, s.mu, s.ls = nn.Linear(i, h), nn.Linear(h, h), nn.Linear(h, ACT_DIM), nn.Linear(h, ACT_DIM)
        # [Opus Fix: Residual RL 전제 - 잔차 평균 0 초기화 (base 서보가 초기 행동을 지배)]
        nn.init.zeros_(s.mu.weight); nn.init.zeros_(s.mu.bias); nn.init.constant_(s.ls.bias, -1.0)

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
    # [Opus Fix: PopArt - 타깃 통계 적응 정규화 + 출력 보존 재조정 (W'=W s/s', b'=(s b + m - m')/s')]
    # [Opus Fix: 타깃 네트워크 헤드도 동일 재조정 + 통계 복사 (초안의 통계 lerp는 target 출력 정규화 불일치 유발)]
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


# ============================================================ GPU-resident episodic replay
class GPUReplay:
    KEYS = ("obs", "ag", "pd", "act", "r_np", "r_sf", "term", "g", "ps", "len")

    def __init__(s, cap, T, dev, stage_n):
        s.cap, s.T, s.dev, s.stage_n, s.ptr, s.filled, s.k = cap, T, dev, stage_n, 0, 0, 0
        z = lambda *sh, dt=torch.float32: torch.zeros(*sh, dtype=dt, device=dev)
        # [Opus Fix: 리플레이 전체 VRAM 상주 (cap*(T+1)*(obs+ag+pd)*4B ~= 0.6GB @cap=2e4) -> 업데이트당 H2D 0회]
        s.b = dict(obs=z(cap, T + 1, OBS_DIM), ag=z(cap, T + 1, 3), pd=z(cap, T + 1, PD_DIM),
                   act=z(cap, T, ACT_DIM), r_np=z(cap, T), r_sf=z(cap, T), term=z(cap, T), g=z(cap, 1),
                   ps=z(cap, PS_DIM), len=z(cap, dt=torch.long))
        # [Opus Fix: 누적합을 고정 길이(cap)로 유지 -> 샘플러 텐서 형상 정적화 (CUDA Graph 캡처 가능, len=0 에피소드는 searchsorted가 자동 배제)]
        s.cum = z(cap, dt=torch.long)
        s.total = s.cum[-1:]
        s.cs = torch.cuda.Stream() if CUDA else None
        s.slots = []
        for _ in range(2 if CUDA else 1):
            host = {k: torch.zeros((stage_n,) + tuple(s.b[k].shape[1:]), dtype=s.b[k].dtype, pin_memory=CUDA)
                    for k in s.KEYS}
            host["idx"] = torch.zeros(stage_n, dtype=torch.long, pin_memory=CUDA)
            devb = {k: torch.empty_like(v, device=dev) for k, v in host.items()} if CUDA else host
            s.slots.append(dict(host=host, np={k: v.numpy() for k, v in host.items()}, dev=devb,
                                copied=None, consumed=None))

    def add_many(s, eps):
        n = len(eps)
        sl = s.slots[s.k]
        s.k = (s.k + 1) % len(s.slots)
        if sl["copied"] is not None:
            sl["copied"].synchronize()
        Hn = sl["np"]
        for i, ep in enumerate(eps):
            L = len(ep["act"])
            for k in ("obs", "ag", "pd"):
                Hn[k][i, :L + 1] = ep[k]
            for k in ("act", "r_np", "r_sf", "term"):
                Hn[k][i, :L] = ep[k]
            Hn["g"][i], Hn["ps"][i], Hn["len"][i] = ep["g"], ep["ps"], L
        Hn["idx"][:n] = (s.ptr + np.arange(n)) % s.cap
        if CUDA:
            # [Opus Fix: pinned 더블버퍼 + 전용 copy stream 비동기 H2D -> 학습 그래프 재생과 전송 중첩, 호스트 블로킹 제거]
            if sl["consumed"] is not None:
                s.cs.wait_event(sl["consumed"])
            with torch.cuda.stream(s.cs):
                for k, v in sl["host"].items():
                    sl["dev"][k][:n].copy_(v[:n], non_blocking=True)
                sl["copied"] = torch.cuda.Event()
                sl["copied"].record(s.cs)
            torch.cuda.current_stream().wait_event(sl["copied"])
        D = sl["dev"]
        idx = D["idx"][:n]
        for k in s.KEYS:
            s.b[k].index_copy_(0, idx, D[k][:n])
        torch.cumsum(s.b["len"], 0, out=s.cum)
        if CUDA:
            sl["consumed"] = torch.cuda.Event()
            sl["consumed"].record()
        s.ptr, s.filled = (s.ptr + n) % s.cap, min(s.filled + n, s.cap)
        return int(sum(len(e["act"]) for e in eps))

    def sample(s, N, tuner, beta, p_her):
        # [Opus Fix: 샘플링/HER relabel/보상 재계산 전부 디바이스 텐서 연산, 호스트 동기화·가변 형상 없음]
        d, m, b = s.dev, tuner.m_hold, s.b
        tot = s.total
        flat = torch.minimum((torch.rand(N, device=d) * tot).long(), tot - 1)
        e = torch.searchsorted(s.cum, flat, right=True)
        L = b["len"][e]
        t = flat - (s.cum[e] - L)
        jj = torch.arange(m, device=d)
        ag, ag2 = b["ag"][e, t], b["ag"][e, t + 1]
        hist2 = b["ag"][e[:, None], ((t + 1)[:, None] - jj[None]).clamp(min=0)][..., 0]
        # [Opus Fix: future 전략 윈도우 평균 목표 g'=mean(ag_{k..k+m-1}) -> 확률적 전이의 hindsight bias 완화]
        fut = torch.minimum(t + 1 + (torch.rand(N, device=d) * (L - t).float()).long(), L)
        gwin = b["ag"][e[:, None], torch.minimum(fut[:, None] + jj[None], L[:, None])][..., 0].mean(1)
        g0 = b["g"][e, 0]
        # [Opus Fix: relabel 목표를 원 목표 분포 지지집합 [g_min, 0.8 F_lim/F_ref]로 제한 (파손력 목표 학습 차단)]
        ok = (torch.rand(N, device=d) < p_her) & (gwin >= tuner.g_min) & (gwin <= 0.8 * b["ps"][e, 3])
        g = torch.where(ok, gwin, g0)
        term, r_np, r_sf = b["term"][e, t], b["r_np"][e, t], b["r_sf"][e, t]
        # [Opus Fix: 보상은 샘플 시점에 relabel 목표로 재계산 (버퍼에는 목표 무관 항만 존재)]
        r = tuner.reward(ag, ag2, hist2, g, r_np, r_sf, term, beta)
        ps = b["ps"][e]
        return dict(obs=b["obs"][e, t], obs2=b["obs"][e, t + 1], g=g[:, None],
                    psi=torch.cat([ps, b["pd"][e, t]], -1), psi2=torch.cat([ps, b["pd"][e, t + 1]], -1),
                    act=b["act"][e, t], r=r[:, None], term=term[:, None])


# ============================================================ learner
class Learner:
    def __init__(s, cfg, buf, tuner):
        s.cfg, s.buf, s.tuner, xd, h = cfg, buf, tuner, OBS_DIM + GOAL_DIM, cfg["hidden"]
        s.actor = Actor(xd, h).to(device)
        s.critic = Critic(xd + PSI_DIM + ACT_DIM, h).to(device)
        s.critic_t = copy.deepcopy(s.critic)
        for p in s.critic_t.parameters():
            p.requires_grad_(False)
        s.cp, s.tp = list(s.critic.parameters()), list(s.critic_t.parameters())
        s.log_alpha = torch.full((1,), math.log(cfg["alpha0"]), device=device, requires_grad=True)
        # [Opus Fix: capturable foreach Adam -> 옵티마이저 step을 CUDA Graph 내부에 포함]
        kw = dict(lr=cfg["lr"], **(dict(capturable=True, foreach=True) if CUDA else {}))
        s.oa = torch.optim.Adam(s.actor.parameters(), **kw)
        s.oc = torch.optim.Adam(s.critic.parameters(), **kw)
        s.ol = torch.optim.Adam([s.log_alpha], **kw)
        s.beta = torch.ones((), device=device)
        s.dres = torch.zeros((), device=device)
        s.stat = torch.zeros(4, device=device)
        s.H_t = -1.5 * ACT_DIM
        s.graph, s.eager = None, not CUDA

    def executed(s, obs, g, a):
        # [Opus Fix: HER MDP 붕괴 교정 - base 서보가 g에 의존하므로 critic 행동을 실행된 명령 증분 a~로 정의]
        # [Opus Fix: a~ = clip(cmd + dmax*tanh((d_base(o,g) + a_res*d_res)/dmax), 0, th_max) - cmd, 정책 그래디언트는 f_g를 통과]
        f = obs[:, -FRAME:]
        Fo, cmd = f[:, F_IDX:F_IDX + 1], f[:, CMD_IDX:CMD_IDX + 1] * TH_MAX
        d_base = torch.clamp(K_F * F_REF * (g - Fo), -DELTA_BASE, DELTA_BASE)
        dth = DELTA_MAX * torch.tanh((d_base + a[:, :1] * s.dres) / DELTA_MAX)
        dth = torch.clamp(cmd + dth, 0.0, TH_MAX) - cmd
        return torch.cat([dth / DELTA_MAX, a[:, 1:]], -1)

    def _update(s, B):
        c = s.cfg
        x, x2 = torch.cat([B["obs"], B["g"]], -1), torch.cat([B["obs2"], B["g"]], -1)
        with torch.no_grad():
            alpha = s.log_alpha.exp()
            a2, lp2 = s.actor(x2)
            q1t, q2t = s.critic_t(torch.cat([x2, B["psi2"], s.executed(B["obs2"], B["g"], a2)], -1))
            qt = torch.min(s.critic_t.denorm(q1t), s.critic_t.denorm(q2t))
            # [Opus Fix: timeout은 bootstrap, crush/miss만 d=1 (Pardo 2018)]
            y = B["r"] + c["gamma"] * (1 - B["term"]) * (qt - alpha * lp2)
            popart_update(s.critic, s.critic_t, y, c["popart_beta"])
            yn = (y - s.critic.mu) / s.critic.sigma
        q1, q2 = s.critic(torch.cat([x, B["psi"], B["act"]], -1))
        lc = F.mse_loss(q1, yn) + F.mse_loss(q2, yn)
        s.oc.zero_grad(set_to_none=True)
        lc.backward()
        s.oc.step()

        # [Opus Fix: actor 손실 중 critic 동결 -> critic 파라미터 불필요 역전파(~30% 연산) 제거]
        for p in s.cp:
            p.requires_grad_(False)
        a, lp = s.actor(x)
        q1p, q2p = s.critic(torch.cat([x, B["psi"], s.executed(B["obs"], B["g"], a)], -1))
        la = (alpha * lp - torch.min(s.critic.denorm(q1p), s.critic.denorm(q2p))).mean()
        s.oa.zero_grad(set_to_none=True)
        la.backward()
        s.oa.step()
        for p in s.cp:
            p.requires_grad_(True)

        ll = -(s.log_alpha * (lp.detach() + s.H_t)).mean()
        s.ol.zero_grad(set_to_none=True)
        ll.backward()
        s.ol.step()
        with torch.no_grad():
            # [Opus Fix: foreach 융합 soft update, 파라미터만 lerp (PopArt 통계는 popart_update에서 동기화)]
            torch._foreach_lerp_(s.tp, s.cp, c["tau"])
            s.stat.copy_(torch.stack([lc.detach(), la.detach(), alpha.squeeze(), y.mean()]))

    def _chunk(s):
        U, bs = s.cfg["chunk_upd"], s.cfg["batch"]
        C = s.buf.sample(U * bs, s.tuner, s.beta, s.cfg["p_her"])
        for i in range(U):
            s._update({k: v[i * bs:(i + 1) * bs] for k, v in C.items()})

    def _capture(s):
        # [Opus Fix: 샘플링 + U회 SAC 업데이트 전체를 단일 CUDA Graph로 캡처 -> 청크당 커널 launch 1회, 호스트 개입 0]
        st = torch.cuda.Stream()
        st.wait_stream(torch.cuda.current_stream())
        with torch.cuda.stream(st):
            for _ in range(2):
                s._chunk()
        torch.cuda.current_stream().wait_stream(st)
        for o in (s.oa, s.oc, s.ol):
            o.zero_grad(set_to_none=True)
        g = torch.cuda.CUDAGraph()
        with torch.cuda.graph(g):
            s._chunk()
        s.graph = g

    def run_chunk(s):
        if s.graph is None and not s.eager:
            try:
                s._capture()
            except Exception as ex:
                print(f"[V7] CUDA graph capture failed -> eager: {ex}", flush=True)
                s.graph, s.eager = None, True
        if s.graph is not None:
            s.graph.replay()
        else:
            s._chunk()


class Publisher:
    # [Opus Fix: 가중치 D2H를 pinned + non_blocking + event 폴링으로 비동기화 (암묵적 device sync 제거)]
    def __init__(s, actor):
        s.actor, s.sh, s.pending, s.ev = actor, None, False, None
        s.stage = torch.zeros(sum(p.numel() for p in actor.parameters()), pin_memory=CUDA)
        s.spec = [tuple(p.shape) for p in actor.parameters()]

    def kick(s):
        if s.pending:
            return
        s.stage.copy_(torch.cat([p.detach().reshape(-1) for p in s.actor.parameters()]), non_blocking=True)
        if CUDA:
            s.ev = torch.cuda.Event()
            s.ev.record()
        s.pending = True

    def poll(s):
        if s.pending and (s.ev is None or s.ev.query()):
            s.sh.publish(s.stage.numpy())
            s.pending = False


# ============================================================ main loop
def train(total=CFG["total"], workers=None, warmup=CFG["warmup"], log_every=20000, seed=0, out_dir=None, cfg=CFG):
    out_dir = out_dir or os.path.dirname(os.path.abspath(__file__))
    nw = workers or max(2, min(10, (os.cpu_count() or 4) - 4))
    torch.manual_seed(seed)
    if CUDA:
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
        torch.set_float32_matmul_precision("high")
    else:
        print("[V7] WARNING: CUDA unavailable -> CPU eager fallback", flush=True)
    torch.set_num_threads(4)
    tuner = RewardTunerV7()
    buf = GPUReplay(cfg["cap_eps"], cfg["T"], device, cfg["stage_n"])
    lr = Learner(cfg, buf, tuner)
    pub = Publisher(lr.actor)
    sh = Shared(pub.stage.numel())
    pub.sh = sh
    pub.kick()
    if CUDA:
        torch.cuda.synchronize()
    pub.poll()
    q = mp.Queue(maxsize=256)
    procs = [mp.Process(target=worker, args=(i, sh, pub.spec, q, seed, cfg["p_boundary"]), daemon=True)
             for i in range(nw)]
    for p in procs:
        p.start()

    n = upd = last_pub = 0
    phase, phase_n0, ramp = 1, 0, 0.0
    phi = np.zeros((len(DR_KEYS), 2))
    adr_buf = [[] for _ in ADR_SIDES]
    nb_hold, stats = deque(maxlen=100), deque(maxlen=200)
    next_log, t0 = log_every, time.time()
    U = cfg["chunk_upd"]
    try:
        while n < total:
            eps = []
            while len(eps) < cfg["stage_n"]:
                try:
                    eps.append(q.get_nowait())
                except pyq.Empty:
                    break
            if eps:
                n += buf.add_many(eps)
                for ep in eps:
                    stats.append((ep["hold"], float(ep["crushed"]), float(ep["missed"]), len(ep["act"])))
                    if ep["bnd"] < 0:
                        nb_hold.append(ep["hold"])
                        continue
                    ab = adr_buf[ep["bnd"]]
                    ab.append(ep["hold"])
                    if len(ab) >= cfg["adr_n"]:
                        # [Opus Fix: 경계 평가 기반 ADR (OpenAI ADR): 경계 성능 > t_H 확장, < t_L 축소]
                        mh, j, sd = float(np.mean(ab)), *ADR_SIDES[ep["bnd"]]
                        ab.clear()
                        if mh > cfg["adr_hi"]:
                            phi[j, sd] = min(1.0, phi[j, sd] + cfg["adr_step"])
                        elif mh < cfg["adr_lo"]:
                            phi[j, sd] = max(0.0, phi[j, sd] - cfg["adr_step"])
                        sh.set_adr(phi)
                # [Opus Fix: 커리큘럼 phase를 스텝 수가 아닌 비경계 에피소드 hold 성능으로 진급]
                if (phase < 3 and len(nb_hold) >= 100 and np.mean(nb_hold) >= cfg["phase_thr"]
                        and n - phase_n0 >= cfg["phase_min"]):
                    phase, phase_n0 = phase + 1, n
                    sh.phase.value = phase
                    nb_hold.clear()
            pub.poll()

            ran = 0
            if n >= warmup and buf.filled > 0:
                ramp = min(1.0, (n - warmup) / cfg["ramp_steps"])
                sh.ramp.value = ramp
                lr.dres.fill_(DELTA_RES * ramp)
                lr.beta.fill_(tuner.beta(n, 0.6 * total))
                # [Opus Fix: SPI=B*U/N 레이트 리미터 - 실행된 업데이트만 회계 (초안의 carry 누수 제거), 학습기 지연 시 수집기 정지]
                target = int(cfg["utd"] * (n - warmup))
                sh.pause.value = int((n - warmup) - upd / cfg["utd"] > cfg["lag_tr"])
                while upd + U <= target and ran < cfg["max_chunks_per_loop"]:
                    lr.run_chunk()
                    upd += U
                    ran += 1
                    if upd // cfg["sync_upd"] != last_pub:
                        last_pub = upd // cfg["sync_upd"]
                        pub.kick()
            if not eps and ran == 0:
                time.sleep(0.002)

            if n >= next_log and stats:
                st, ls = np.mean(stats, 0), lr.stat.tolist()
                el = time.time() - t0
                print(f"[V7] n={n:8d} upd={upd:7d} hold={st[0]:.3f} crush={st[1]:.2f} miss={st[2]:.2f} len={st[3]:5.1f} "
                      f"ph={phase} adr={phi.mean():.2f} ramp={ramp:.2f} beta={float(lr.beta):.2f} "
                      f"Lc={ls[0]:.3f} La={ls[1]:.3f} alpha={ls[2]:.3f} ymean={ls[3]:.2f} "
                      f"tps={n / el:.0f} ups={upd / el:.0f} graph={lr.graph is not None}", flush=True)
                torch.save(dict(actor=lr.actor.state_dict(), critic=lr.critic.state_dict(),
                                critic_t=lr.critic_t.state_dict(), log_alpha=lr.log_alpha.detach(),
                                adr=phi, phase=phase, n=n, upd=upd), os.path.join(out_dir, "v7_final_ckpt.pt"))
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
        torch.save(lr.actor.state_dict(), os.path.join(out_dir, "final_actor_v7.pth"))
    return lr


if __name__ == "__main__":
    mp.freeze_support()
    ap = argparse.ArgumentParser()
    ap.add_argument("--total", type=int, default=CFG["total"])
    ap.add_argument("--workers", type=int, default=None)
    ap.add_argument("--warmup", type=int, default=CFG["warmup"])
    ap.add_argument("--log-every", type=int, default=20000)
    a = ap.parse_args()
    train(a.total, a.workers, a.warmup, a.log_every)
