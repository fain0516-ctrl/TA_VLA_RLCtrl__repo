# -*- coding: utf-8 -*-
"""CPU 롤아웃 워커 + 공유 메모리 (Actor 가중치, ADR 확장도, 커리큘럼 단계, 정지 플래그)."""
import numpy as np

from .. import config as C
from ..policy.numpy_actor import np_policy
from .schedulers import ADR_SIDES


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

    def publish_adr(s, phi):
        with s.adr.get_lock():
            np.frombuffer(s.adr.get_obj(), dtype=np.float64)[:] = phi.ravel()

    def fetch(s, spec):
        with s.w.get_lock():
            flat = np.frombuffer(s.w.get_obj(), dtype=np.float32).copy()
        out, o = [], 0
        for shp in spec:
            n = int(np.prod(shp))
            out.append(flat[o:o + n].reshape(shp))
            o += n
        return out


def worker(rank, sh, spec, q, seed, p_b):
    from ..sim.env import FingerGraspEnv
    env = FingerGraspEnv(seed=seed * 1000 + rank)
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
