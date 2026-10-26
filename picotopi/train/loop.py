# -*- coding: utf-8 -*-
"""메인 학습 루프 (V8 train()과 같은 동작).

    python -m picotopi.train --seed 0 --workers 12
결과: <out>/log.csv, config.json, ckpt.pt, final_actor.pth (+ final_actor.npz: Pi 런타임용)
"""
import os
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
import argparse
import json
import multiprocessing as mp
import queue as pyq
import time
from collections import deque
from dataclasses import asdict

import numpy as np
import torch

from .. import config as C
from ..policy.numpy_actor import export_npz
from ..reward import GraspReward
from ..sim.env import OBS_DIM
from .learner import Learner
from .replay import EpisodeReplay
from .schedulers import ADRController, RadiusCurriculum
from .workers import Shared, worker

CFG = dict(T=C.T_EP, cap_eps=20000, batch=512, utd=0.5, gamma=0.98, tau=0.005, lr=3e-4, p_her=0.6,
           total=1_000_000, warmup=10_000, sync_every=2000, hidden=256, alpha0=0.1, popart_beta=1e-3,
           p_boundary=0.3, adr_n=20, adr_step=0.05, adr_hi=0.30, adr_lo=0.10,
           curr_thr=0.30, curr_step=0.1, curr_n=100)


def train(total=CFG["total"], workers=None, seed=0, out_dir=None, cfg=CFG, log_every=20000, device=None):
    out_dir = out_dir or os.path.join("runs", f"seed{seed}")
    os.makedirs(out_dir, exist_ok=True)
    dev = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    nw = workers or max(2, min(12, (os.cpu_count() or 4) - 2))
    torch.manual_seed(seed); np.random.seed(seed)
    rew = GraspReward()
    buf = EpisodeReplay(cfg["cap_eps"], cfg["T"], dev)
    lr = Learner(cfg, dev)
    adr = ADRController(cfg["adr_n"], cfg["adr_step"], cfg["adr_hi"], cfg["adr_lo"])
    curr = RadiusCurriculum(cfg["curr_thr"], cfg["curr_step"], cfg["curr_n"])
    spec = [tuple(p.shape) for p in lr.actor.parameters()]
    ctx = mp.get_context("spawn")         # Windows 기본과 동일
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
    stats = deque(maxlen=200)
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
                        curr.add(ep["hold"])
                    elif adr.record(int(ep["bnd"]), ep["hold"]):
                        sh.publish_adr(adr.phi)
                if curr.advance():
                    sh.curr.value = curr.value

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
                row = [n, upd, st[0], st[1], st[2], curr.value, adr.phi.mean(), *lr.stat, n / (time.time() - t0)]
                log.write(",".join(f"{v:.4g}" for v in row) + "\n"); log.flush()
                print(f"[PICOtoPI] n={n:8d} upd={upd:7d} hold={st[0]:.3f} crush={st[1]:.2f} peak={st[2]:.2f}N "
                      f"curr={curr.value:.1f} adr={adr.phi.mean():.2f} Lc={lr.stat[0]:.3f} La={lr.stat[1]:.3f} "
                      f"alpha={lr.stat[2]:.3f} tps={row[-1]:.0f}", flush=True)
                torch.save(dict(actor=lr.actor.state_dict(), critic=lr.critic.state_dict(), adr=adr.phi,
                                curr=curr.value, n=n, meta=meta), os.path.join(out_dir, "ckpt.pt"))
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
        pth = os.path.join(out_dir, "final_actor.pth")
        torch.save(lr.actor.state_dict(), pth)
        export_npz(pth, os.path.join(out_dir, "final_actor.npz"))
    return lr, out_dir


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m picotopi.train")
    ap.add_argument("--total", type=int, default=CFG["total"])
    ap.add_argument("--workers", type=int, default=None)
    ap.add_argument("--warmup", type=int, default=CFG["warmup"])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--log-every", type=int, default=20000)
    ap.add_argument("--device", default=None)
    a = ap.parse_args(argv)
    train(a.total, a.workers, a.seed, a.out, dict(CFG, warmup=a.warmup), a.log_every, a.device)
