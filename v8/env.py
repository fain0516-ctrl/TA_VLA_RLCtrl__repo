# -*- coding: utf-8 -*-
"""V8 1지 촉각 파지 환경.

가이드(docs/research/04_team_guide.md) 반영 요약
- 정책 25 Hz, 안전층 100 Hz, 물리 2 ms                                     [P1-2, 03 §2]
- 행동 = [Δq_mcp, Δu_tendon] 증분 + EMA, 텐던을 MCP와 분리 (V7: ctrl[0]=ctrl[1]) [P1-2]
- 안전층(슬루·백래시·초과력 어드미턴스 양보·텐던 ZOH·MCP 연성 피드포워드·해제 인터락)을
  학습 루프 안에서 실행                                                    [P1-3, DextrAH-G]
- Actor 관측에 목표 힘 g와 파손 한계 F_th, 이전 행동, 위치오차, Load, 3스텝 이력  [P1-2, FORGE]
- 촉각: 이진 접촉(보정 임계, 패드 드롭) + 4단계 양자화 + (옵션) 노이즈·스케일 DR 연속값 [P2-6]
- 텐던: 공간 경로 권선반경 맵 + 복귀 스프링 + 전달효율 η + 슬랙               [P2-2]
- DR: 식별값 근처 좁은 nominal에서 ADR로 확장, 행동·관측 지연 포함           [P2-4, P2-5]
- 평가 지표: 순간 최대력(F_peak)을 매 스텝 기록 (3스텝 지속 규칙 폐지)          [P0-3]
"""
from collections import deque

import mujoco
import numpy as np

from . import config as C
from .hand_model import build_xml
from .reward import RewardV8

FRAME = 13
H = 3
OBS_DIM = FRAME * H + 2            # + [g, F_th]
PS_DIM = 16                        # 에피소드 고정 특권 정보
PD_DIM = 12                        # 스텝별 특권 정보
AG_DIM = 2                         # achieved: [평균 힘/F_REF, 접촉]


def _seg_dist(p, a, b):
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / (np.dot(ab, ab) + 1e-12), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * ab)))


class FingerGraspEnvV8:
    def __init__(self, seed=None, reward=None):
        self.rng = np.random.default_rng(seed)
        self.rew = reward or RewardV8()
        self.adr = np.zeros((len(C.DR_KEYS), 2))   # 키별 [하한 확장도, 상한 확장도] 0~1
        self.curr = 0.0                            # 반경 커리큘럼 0(시작 범위)~1(전체 범위)
        self.model = self.data = None

    # ================================================================== reset
    def _sample_params(self, boundary):
        p = {}
        for j, k in enumerate(C.DR_KEYS):
            b = boundary[1] if boundary is not None and boundary[0] == j else None
            p[k] = C.dr_sample(k, self.rng, self.adr[j, 0], self.adr[j, 1], b)
        lo = C.R_START[0] + self.curr * (C.R_FULL[0] - C.R_START[0])
        hi = C.R_START[1] + self.curr * (C.R_FULL[1] - C.R_START[1])
        p["radius"] = float(self.rng.uniform(lo, hi))
        p["tendon_fmax"] = C.tendon_force_max()
        p["F_th"] = float(self.rng.uniform(*C.F_TH_RANGE))
        p["g"] = float(self.rng.uniform(C.G_LO, C.G_HI_FRAC * p["F_th"]))
        p["drop"] = (self.rng.random(3) < C.TAC_DROP_P).astype(np.float64)
        return p

    def reset(self, boundary=None, params=None):
        """boundary: (DR 키 인덱스, 0|1) - ADR 경계 평가 에피소드. params: 평가용 고정 파라미터(부분 지정 가능)."""
        p = self._sample_params(boundary)
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
        self.mcp_ref, self.u_ref = q0, 0.0            # 정책 적분 목표
        self.mcp_cmd, self.mcp_bl = q0, q0            # 슬루 후 명령, 백래시 적용 목표
        self.tendon_ctrl, self.zoh_t = 0.0, 0.0
        self.e, self.ed = 0.0, 0.0                    # 어드미턴스 양보 상태
        self.a_f = np.zeros(C.ACT_DIM)
        self.steps, self.has_touched, self.contact_prev = 0, False, False
        ad = p["act_delay"]
        self.act_delay = int(np.floor(ad)) + int(self.rng.random() < ad - np.floor(ad))
        self.act_q = deque([(self.mcp_ref, self.u_ref)] * (self.act_delay + 1), maxlen=self.act_delay + 1)
        n_od = int(round(p["obs_delay"] / C.CTRL_DT))
        self.sens_q = deque(maxlen=n_od + 1)
        self.F_true = np.zeros(3)
        self.F_meas_tick = 0.0
        s = self._measure()
        self.sens_q.extend([s] * (n_od + 1))
        self.d_best = self._wrap_dist()
        self.hist = deque([0.0] * self.rew.m_hold, maxlen=self.rew.m_hold)
        self.frames = deque([self._frame(s)] * H, maxlen=H)
        self.F_bar = 0.0
        return self._obs()

    # ================================================================== 센서
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
        """실기 제어기가 볼 수 있는 값만 (촉각 측정치, 엔코더, Load)."""
        p, d = self.p, self.data
        F = self._pad_forces()
        self.F_true = F
        tac = np.maximum(0.0, p["tac_scale"] * F + self.rng.normal(0, p["tac_noise"], 3)) * (1.0 - p["drop"])
        tac[tac < C.TAC_THRESH] = 0.0
        q = d.qpos[0] + p["zero"] + self.rng.normal(0, p["q_noise"])
        tau_max = C.STS_TAU_MAX * p["tau"]
        # Load: STS3215 부하 레지스터 대용. 보정 전(가이드 M3)이라 노이즈·오프셋을 크게 둠 [추정]
        load = d.actuator_force[0] / tau_max + self.rng.normal(0, 0.03)
        return dict(tac=tac, q=float(q), load=float(np.clip(load, -1.5, 1.5)))

    def _frame(self, s):
        tac = s["tac"]
        Fm = float(tac.sum())
        lvl = float(np.searchsorted(C.TAC_LEVELS, Fm, side="right"))
        return np.array([
            s["q"] / C.TH_MAX, self.mcp_ref / C.TH_MAX, np.clip((self.mcp_ref - s["q"]) / 0.3, -1, 1), self.u_ref,
            s["load"], *(tac > 0).astype(np.float64), lvl / len(C.TAC_LEVELS),
            min(Fm / C.F_REF, 2.0) if C.OBS_CONT_FORCE else 0.0, *self.a_f, float(self.has_touched)],
            dtype=np.float32)

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
        pd = np.clip(np.array([*(d.qpos[:3] / C.TH_MAX), *(self.F_true / C.F_REF), tf, self.e / 0.3,
                               d.qpos[3] / 0.005, d.qpos[4] / 0.005, self.F_bar / C.F_REF,
                               getattr(self, "F_peak", 0.0) / C.F_REF], dtype=np.float32), -3, 3)
        ag = np.array([self.F_bar / C.F_REF, float(self.F_bar > 0.02)], dtype=np.float32)
        obs = np.concatenate([*self.frames, [p["g"] / C.F_REF, p["F_th"] / C.F_REF]]).astype(np.float32)
        return dict(obs=obs, ps=ps, pd=pd, ag=ag, g=np.float32(p["g"] / C.F_REF))

    # ================================================================== 안전층 1틱 (100 Hz)
    def _safety_tick(self, mcp_ref, u_ref, F_meas):
        p = self.p
        # (1) 초과력 어드미턴스 양보: 0.9·F_th 넘는 측정력만 밀어냄 (정책의 힘 추종과 역할 분리, 09번 중복 양보 방지)
        f_in = C.ADM_GAIN * max(F_meas - C.SAFETY_FRAC * p["F_th"], 0.0)
        edd = (f_in - C.ADM_D * self.ed - C.ADM_K * self.e) / C.ADM_M
        self.ed += edd * C.CTRL_DT
        self.e = max(self.e + self.ed * C.CTRL_DT, 0.0)
        if self.e == 0.0:
            self.ed = max(self.ed, 0.0)
        tgt = float(np.clip(mcp_ref - self.e, 0.0, C.TH_MAX))
        u = float(np.clip(u_ref - self.e / C.PIP_MAX, 0.0, 1.0))
        # (2) 해제 인터락 (ADR 005): 텐던 장력이 남아 있으면 MCP 신전 보류
        tension = -self.data.actuator_force[1]
        if tgt < self.mcp_cmd and tension > C.RELEASE_T:
            tgt = self.mcp_cmd
        # (3) 슬루 제한 (실측 최대 속도 x DR)
        dmax = C.STS_VMAX * p["vmax"] * C.CTRL_DT
        self.mcp_cmd += float(np.clip(tgt - self.mcp_cmd, -dmax, dmax))
        # (4) 백래시: 명령이 데드밴드를 넘을 때만 서보 목표 이동
        bl = p["backlash"] / 2
        if self.mcp_cmd > self.mcp_bl + bl:
            self.mcp_bl = self.mcp_cmd - bl
        elif self.mcp_cmd < self.mcp_bl - bl:
            self.mcp_bl = self.mcp_cmd + bl
        self.data.ctrl[0] = self.mcp_bl
        # (5) 텐던: PWM ZOH + MCP 연성 피드포워드 ΔL = r_mcp·q_mcp (ADR 005) + 슬랙
        self.zoh_t += C.CTRL_DT
        if self.zoh_t >= C.TENDON_ZOH - 1e-9:
            self.zoh_t = 0.0
            rs = p["r_scale"]
            self.tendon_ctrl = -(C.R_MCP * rs * self.mcp_cmd + u * C.TENDON_STROKE * rs) + p["slack"]
        self.data.ctrl[1] = self.tendon_ctrl

    # ================================================================== step (25 Hz)
    def step(self, a):
        p, rew = self.p, self.rew
        a = np.clip(np.asarray(a, dtype=np.float64).reshape(C.ACT_DIM), -1.0, 1.0)
        a_prev = self.a_f.copy()
        self.a_f = C.ACT_EMA * a + (1.0 - C.ACT_EMA) * self.a_f
        self.mcp_ref = float(np.clip(self.mcp_ref + self.a_f[0] * C.DQ_MCP, 0.0, C.TH_MAX))
        self.u_ref = float(np.clip(self.u_ref + self.a_f[1] * C.DU_TENDON, 0.0, 1.0))

        F_sum, F_peak = 0.0, 0.0
        for _ in range(C.DECIM):
            self.act_q.append((self.mcp_ref, self.u_ref))
            mref, uref = self.act_q[0]
            s_delayed = self.sens_q[0]
            self._safety_tick(mref, uref, float(s_delayed["tac"].sum()))
            for _ in range(C.N_SUB):
                mujoco.mj_step(self.model, self.data)
                F_peak = max(F_peak, float(self._pad_forces().sum()))
            s = self._measure()
            self.sens_q.append(s)
            F_sum += float(self.F_true.sum())
        self.steps += 1
        self.F_bar, self.F_peak = F_sum / C.DECIM, F_peak

        s_obs = self.sens_q[0]
        contact = self.F_bar > 0.02
        if float(s_obs["tac"].sum()) >= 0.15:
            self.has_touched = True
        slip_edge = bool(self.contact_prev and not contact and self.has_touched)
        self.contact_prev = contact

        d_now = self._wrap_dist()
        d_imp = 0.0 if contact else self.d_best - d_now
        self.d_best = min(self.d_best, d_now)
        r_np, parts = rew.nongoal_terms(d_imp, self.a_f - a_prev, F_peak, p["F_th"], slip_edge)
        term = bool(self.rng.random() < rew.cat_term_prob(parts["excess"]))
        trunc = self.steps >= C.T_EP

        self.hist.append(self.F_bar)
        r_g, hold = rew.goal_terms_np(self.F_bar, float(contact), list(self.hist), p["g"])
        self.frames.append(self._frame(s_obs))
        info = dict(r_np=r_np, r=r_np + r_g, hold=hold, term=term, trunc=trunc, F_bar=self.F_bar, F_peak=F_peak,
                    crushed=F_peak > p["F_th"], slip_edge=slip_edge, contact=contact, **parts)
        return self._obs(), bool(term or trunc), info

    # ================================================================== 기준선용 측정치 접근
    def measured(self):
        s = self.sens_q[0]
        return dict(F=float(s["tac"].sum()), q=s["q"], load=s["load"], mcp_ref=self.mcp_ref, u_ref=self.u_ref,
                    g=self.p["g"], F_th=self.p["F_th"])
