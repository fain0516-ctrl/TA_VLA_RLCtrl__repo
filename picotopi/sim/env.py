# -*- coding: utf-8 -*-
"""1지 촉각 파지 학습 환경 (V8 FingerGraspEnvV8을 control/reward/sim 모듈로 재조립).

이 파일에 남는 것은 시뮬 전용 부분뿐입니다:
  - MuJoCo 모델 생성·스텝, 패드 접촉력 측정
  - 센서 오차 모델 (촉각 스케일·노이즈·패드 드롭, 엔코더 영점·노이즈, Load 노이즈)
  - 행동·관측 지연 큐 (DR)
  - 특권 정보(ps, pd)와 achieved goal(ag), 접근 거리
행동 적분(control.action), 안전층(control.safety), 관측 프레임(control.observation), 보상(reward)은
실기 런타임과 같은 코드입니다.
"""
from collections import deque

import mujoco
import numpy as np

from .. import config as C
from ..control.action import ActionIntegrator
from ..control.observation import OBS_DIM, FrameStack, make_frame
from ..control.safety import SafetyLayer, SafetyParams
from ..control.tactile import tactile_threshold
from ..reward import GraspReward
from .model import build_xml
from .params import sample_episode_params

PS_DIM = 16                        # 에피소드 고정 특권 정보
PD_DIM = 12                        # 스텝별 특권 정보
AG_DIM = 2                         # achieved: [평균 힘/F_REF, 접촉]

__all__ = ["FingerGraspEnv", "OBS_DIM", "PS_DIM", "PD_DIM", "AG_DIM"]


def _seg_dist(p, a, b):
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / (np.dot(ab, ab) + 1e-12), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * ab)))


class FingerGraspEnv:
    def __init__(self, seed=None, reward=None):
        self.rng = np.random.default_rng(seed)
        self.rew = reward or GraspReward()
        self.adr = np.zeros((len(C.DR_KEYS), 2))   # 키별 [하한 확장도, 상한 확장도]
        self.curr = 0.0                            # 반경 커리큘럼
        self.act_int = ActionIntegrator()
        self.safety = SafetyLayer()
        self.stack = FrameStack()
        self.model = self.data = None

    # V8 이름 호환 (기준선 FSM·평가 코드가 직접 읽음)
    mcp_ref = property(lambda s: s.act_int.mcp_ref)
    u_ref = property(lambda s: s.act_int.u_ref)
    a_f = property(lambda s: s.act_int.a_f)
    e = property(lambda s: s.safety.e)

    # ================================================================== reset
    def reset(self, boundary=None, params=None):
        """boundary: (DR 키 인덱스, 0|1) - ADR 경계 평가. params: 평가용 고정 파라미터(부분 지정)."""
        p = sample_episode_params(self.rng, self.adr, self.curr, boundary)
        if params:
            p.update(params)
        self.p = p
        self.model = mujoco.MjModel.from_xml_string(build_xml(p))
        self.data = mujoco.MjData(self.model)
        m = self.model
        gid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
        self.g_obj = gid("obj")
        self.g_pads = [gid(n) for n in C.TAC_PADS]
        self.b_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n) for n in ("prox", "mid", "dist", "obj")]
        self.s_tip = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "tip")
        self.data.qpos[0] = self.rng.uniform(0.0, 0.08)
        self.data.ctrl[0] = self.data.qpos[0]
        mujoco.mj_forward(m, self.data)

        q0 = float(self.data.qpos[0])
        self.act_int.reset(q0)
        self.safety.reset(q0, SafetyParams.from_episode(p))
        self.steps, self.has_touched, self.contact_prev = 0, False, False
        ad = p["act_delay"]
        self.act_delay = int(np.floor(ad)) + int(self.rng.random() < ad - np.floor(ad))
        self.act_q = deque([(q0, 0.0)] * (self.act_delay + 1), maxlen=self.act_delay + 1)
        n_od = int(round(p["obs_delay"] / C.CTRL_DT))
        self.sens_q = deque(maxlen=n_od + 1)
        self.F_true = np.zeros(3)
        s = self._measure()
        self.sens_q.extend([s] * (n_od + 1))
        self.d_best = self._wrap_dist()
        self.hist = deque([0.0] * self.rew.m_hold, maxlen=self.rew.m_hold)
        self.stack.reset(self._frame(s))
        self.F_bar, self.F_peak = 0.0, 0.0
        return self._obs()

    # ================================================================== 센서 (시뮬 오차 모델)
    def _pad_forces(self):
        d, f, c6 = self.data, np.zeros(3), np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            if self.g_obj not in (c.geom1, c.geom2):
                continue
            other = c.geom2 if c.geom1 == self.g_obj else c.geom1
            if other in self.g_pads:
                mujoco.mj_contactForce(self.model, d, i, c6)
                f[self.g_pads.index(other)] += abs(c6[0])
        return f

    def _measure(self):
        """실기 제어기가 볼 수 있는 값 (촉각, 엔코더, Load) - runtime.HardwareIO.read()와 같은 형식."""
        p, d = self.p, self.data
        F = self._pad_forces()
        self.F_true = F
        tac = np.maximum(0.0, p["tac_scale"] * F + self.rng.normal(0, p["tac_noise"], 3)) * (1.0 - p["drop"])
        tac = tactile_threshold(tac)
        q = d.qpos[0] + p["zero"] + self.rng.normal(0, p["q_noise"])
        load = d.actuator_force[0] / (C.STS_TAU_MAX * p["tau"]) + self.rng.normal(0, 0.03)
        return dict(tac=tac, q=float(q), load=float(np.clip(load, -1.5, 1.5)))

    def _frame(self, s):
        ai = self.act_int
        return make_frame(s, ai.mcp_ref, ai.u_ref, ai.a_f, self.has_touched)

    def _wrap_dist(self):
        """세 링크와 물체 표면 사이 평균 거리 (접근 보상용, 특권)."""
        x = self.data.xpos
        pts = [x[self.b_ids[0]], x[self.b_ids[1]], x[self.b_ids[2]], self.data.site_xpos[self.s_tip]]
        c = x[self.b_ids[3]]
        r = self.p["radius"] + C.LINK_W / 2
        return float(np.mean([max(_seg_dist(c, pts[i], pts[i + 1]) - r, 0.0) for i in range(3)]))

    # ================================================================== 관측
    def _obs(self):
        p, d = self.p, self.data
        ps = np.array([p["radius"] / 0.04, p["mass"] / 0.2, p["mu"] / 2.0, p["eta"], p["kp"], p["tau"], p["vmax"],
                       p["spring"], p["oppose"], p["act_delay"] / 1.5, p["obs_delay"] / 0.05, p["tac_scale"],
                       *p["drop"], p["F_th"] / C.F_REF], dtype=np.float32)
        tf = -d.actuator_force[1] / (p["tendon_fmax"] * p["eta"] + 1e-6)
        pd = np.clip(np.array([*(d.qpos[:3] / C.TH_MAX), *(self.F_true / C.F_REF), tf, self.safety.e / 0.3,
                               d.qpos[3] / 0.005, d.qpos[4] / 0.005, self.F_bar / C.F_REF,
                               self.F_peak / C.F_REF], dtype=np.float32), -3, 3)
        ag = np.array([self.F_bar / C.F_REF, float(self.F_bar > C.CONTACT_F)], dtype=np.float32)
        obs = self.stack.actor_obs(p["g"], p["F_th"])
        return dict(obs=obs, ps=ps, pd=pd, ag=ag, g=np.float32(p["g"] / C.F_REF))

    # ================================================================== step (25 Hz)
    def step(self, a):
        p, rew = self.p, self.rew
        da = self.act_int.step(a)
        F_sum, F_peak = 0.0, 0.0
        for _ in range(C.DECIM):
            self.act_q.append((self.act_int.mcp_ref, self.act_int.u_ref))
            mref, uref = self.act_q[0]
            F_meas = float(self.sens_q[0]["tac"].sum())
            self.data.ctrl[0], self.data.ctrl[1] = self.safety.tick(mref, uref, F_meas, -self.data.actuator_force[1])
            for _ in range(C.N_SUB):
                mujoco.mj_step(self.model, self.data)
                F_peak = max(F_peak, float(self._pad_forces().sum()))
            self.sens_q.append(self._measure())
            F_sum += float(self.F_true.sum())
        self.steps += 1
        self.F_bar, self.F_peak = F_sum / C.DECIM, F_peak

        s_obs = self.sens_q[0]
        contact = self.F_bar > C.CONTACT_F
        if float(s_obs["tac"].sum()) >= C.LATCH_F:
            self.has_touched = True
        slip_edge = bool(self.contact_prev and not contact and self.has_touched)
        self.contact_prev = contact

        d_now = self._wrap_dist()
        d_imp = 0.0 if contact else self.d_best - d_now
        self.d_best = min(self.d_best, d_now)
        r_np, parts = rew.nongoal_terms(d_imp, da, F_peak, p["F_th"], slip_edge)
        term = bool(self.rng.random() < rew.cat_term_prob(parts["excess"]))
        trunc = self.steps >= C.T_EP

        self.hist.append(self.F_bar)
        r_g, hold = rew.goal_terms_np(self.F_bar, float(contact), list(self.hist), p["g"])
        self.stack.push(self._frame(s_obs))
        info = dict(r_np=r_np, r=r_np + r_g, hold=hold, term=term, trunc=trunc, F_bar=self.F_bar, F_peak=F_peak,
                    crushed=F_peak > p["F_th"], slip_edge=slip_edge, contact=contact, **parts)
        return self._obs(), bool(term or trunc), info

    # ================================================================== 기준선용 측정치
    def measured(self):
        s = self.sens_q[0]
        ai = self.act_int
        return dict(F=float(s["tac"].sum()), q=s["q"], load=s["load"], mcp_ref=ai.mcp_ref, u_ref=ai.u_ref,
                    g=self.p["g"], F_th=self.p["F_th"])
