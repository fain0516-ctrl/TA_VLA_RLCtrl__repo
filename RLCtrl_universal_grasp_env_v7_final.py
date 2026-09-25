#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import io
import contextlib
import numpy as np
import mujoco
from collections import deque

from universal_grasp_env import UniversalTactileGraspEnv
from sts3215_sim2real_env import SimToRealSTS3215Wrapper
import mcp_joint_control.control.grasp_fsm as _gfsm
from mcp_joint_control.control.grasp_fsm import GraspState
from train_sac_her_v7_final import RewardTunerV7, F_REF, TH_MAX

DT = 0.01
H, FRAME = 3, 11
OBS_DIM, GOAL_DIM, ACT_DIM = H * FRAME, 1, 2
PS_DIM, PD_DIM = 11, 18
PSI_DIM = PS_DIM + PD_DIM
DELTA_MAX, DELTA_BASE, DELTA_RES, K_F = 0.06, 0.01, 0.03, 0.02
K_MIN, K_MAX, TH_REF, RHO_K, ETA = 2.0, 20.0, 0.1, 0.8, 0.5
F_IDX, CMD_IDX = 3, 10


class _SimClock:
    t = 0.0

    @staticmethod
    def time():
        return _SimClock.t


# [Opus Fix: GraspFSM 홀드 타이머가 wall-clock 기반 -> 시뮬레이션 시계로 교체 (결정성/Markov성)]
_gfsm.time = _SimClock

# (nominal_lo, nominal_hi, full_lo, full_hi)
DR = dict(
    mu=(0.9, 1.1, 0.3, 2.0), mass=(0.04, 0.06, 0.02, 0.20),
    damp=(0.9, 1.1, 0.4, 2.5), floss=(0.9, 1.1, 0.4, 2.5), kp=(0.95, 1.05, 0.6, 1.4),
    delay=(0, 0, 0, 3), mu_cap=(0.0, 0.05, 0.0, 0.35),
    nF=(0.005, 0.01, 0.005, 0.08), nth=(5e-4, 1e-3, 5e-4, 6e-3),
)
DR_KEYS = list(DR)
# [Opus Fix: ADR 경계 평가용 (파라미터, 측면) 목록]
ADR_SIDES = [(j, s) for j, k in enumerate(DR_KEYS) for s in (0, 1)
             if (DR[k][0] != DR[k][2] if s == 0 else DR[k][1] != DR[k][3])]


class UniversalTactileGraspEnvV7(UniversalTactileGraspEnv):
    def __init__(self, curriculum_phase=1):
        self.tuner = RewardTunerV7()
        self.adr = np.zeros((len(DR_KEYS), 2))
        self.boundary, self.delta_res = None, DELTA_RES
        super().__init__(curriculum_phase)

    def _dr(self, k, log=True):
        j = DR_KEYS.index(k)
        nlo, nhi, flo, fhi = DR[k]
        lo, hi = nlo + self.adr[j, 0] * (flo - nlo), nhi + self.adr[j, 1] * (fhi - nhi)
        if self.boundary is not None and ADR_SIDES[self.boundary][0] == j:
            return float(lo if ADR_SIDES[self.boundary][1] == 0 else hi)
        if log:
            return float(np.exp(np.random.uniform(np.log(lo), np.log(hi))))
        return float(np.random.uniform(lo, hi))

    def _set_kp(self, eta):
        for i in (0, 1):
            kp = self.kp0[i] * self.kp_m * eta[i]
            self.model.actuator_gainprm[i, 0] = kp
            self.model.actuator_biasprm[i, 1] = -kp

    def _tactile(self):
        Fn = Ft = 0.0
        c6 = np.zeros(6)
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            if c.geom1 == self.tid or c.geom2 == self.tid:
                mujoco.mj_contactForce(self.model, self.data, i, c6)
                if abs(c6[0]) > Fn:
                    Fn, Ft = abs(c6[0]), float(np.hypot(c6[1], c6[2]))
        sg = Ft / (self.mu * Fn + 1e-6) if Fn > 0.02 else 0.0
        return Fn, Ft, min(sg, 2.0)

    def _frame(self, th, thd, Fn, Ft, slip):
        Fo = max(0.0, Fn + np.random.randn() * self.nF + 0.03 * Fn * np.random.randn())
        Fto = max(0.0, Ft + np.random.randn() * self.nF)
        tho = th + np.random.randn() * self.nth
        self.Fo_c = min(Fo / F_REF, 2.0)
        # [Opus Fix: actor 관측에서 참값 mu 의존 sigma 제거 -> 실측 가능한 F_t/F_n, 명령 적분기 cmd 및 cmd-theta 포함]
        ratio = min(Fto / (Fo + 0.05), 3.0) / 3.0
        return np.array([tho / TH_MAX, self.data.qpos[1] / TH_MAX, np.clip(thd / 6.0, -1, 1), self.Fo_c,
                         min(Fto / F_REF, 2.0), ratio, slip, self.prev_at, self.prev_aK,
                         np.clip((self.cmd - tho) / 0.4, -1, 1), self.cmd / TH_MAX], dtype=np.float32)

    def _priv_dyn(self, th):
        # [Opus Fix: 은닉 상태(지연 큐, FSM, admittance, TDPA, 모터, overforce 카운터, h)를 critic 특권 입력으로 -> 비Markov 타깃 제거]
        w = self.sim2real_ctrl
        dq = [(c - th) / 0.4 for c in self.dq]
        dq += [0.0] * (4 - len(dq))
        fsm = np.zeros(5)
        fsm[list(GraspState).index(w.fsm.state)] = 1.0
        v = [self.cmd / TH_MAX, *np.clip(dq, -1, 1), *fsm, w.admittance.e_pos / 0.4, w.admittance.e_vel / 3.0,
             w.tdpa.E_obs / 0.5, w.tdpa.alpha_pc / 18.0, w.current_pos / TH_MAX, w.current_vel / 6.0,
             self.overforce_steps / 3.0, float(self.has_touched)]
        return np.clip(np.array(v, dtype=np.float32), -2.0, 2.0)

    def _obs(self, th):
        ag = np.array([self.Fbar / F_REF, th / TH_MAX, float(self.has_touched)], dtype=np.float32)
        return {"obs": np.concatenate(self.frames), "goal": np.array([self.g], dtype=np.float32),
                "ps": self.ps, "pd": self._priv_dyn(th), "ag": ag}

    def reset(self, phase=None, boundary=None):
        if phase is not None:
            self.curriculum_phase = phase
        self.boundary = boundary
        _SimClock.t = 0.0
        self.model = mujoco.MjModel.from_xml_string(self._generate_randomized_xml())
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.tid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "target_geom")
        self.bid = int(m.geom_bodyid[self.tid])
        self.mu, mass = self._dr("mu"), self._dr("mass")
        m.geom_friction[self.tid, 0] = self.mu
        m.body_inertia[self.bid] *= mass / m.body_mass[self.bid]
        m.body_mass[self.bid] = mass
        damp_m, self.kp_m = self._dr("damp"), self._dr("kp")
        # [Opus Fix: 관절 감쇠 0.02는 링크 관성(~1e-5)에서 RK4 발산 -> 0.002]
        m.dof_damping[:3] = 0.002 * damp_m
        m.dof_frictionloss[:3] *= self._dr("floss")
        self.kp0 = m.actuator_gainprm[:, 0].copy()
        self.mu_cap, self.delay = self._dr("mu_cap", False), int(round(self._dr("delay", False)))
        self.nF, self.nth = self._dr("nF"), self._dr("nth")
        self._set_kp((1.0, 1.0))
        mujoco.mj_setConst(m, self.data)
        self.data.qpos[0] = np.random.uniform(0.0, 0.08)
        mujoco.mj_forward(m, self.data)

        with contextlib.redirect_stdout(io.StringIO()):
            self.sim2real_ctrl = SimToRealSTS3215Wrapper(dt=DT)
        self.K_t = 0.5 * (K_MIN + K_MAX)
        self.sim2real_ctrl.admittance.K = self.K_t
        self.g = float(np.random.uniform(0.2, 0.8 * self.current_fragile_limit) / F_REF)
        self.ps = np.concatenate([np.eye(3)[self.current_shape_id],
                                  [self.current_fragile_limit / F_REF, self.target_radius / 0.04, self.mu / 2.0,
                                   mass / 0.2, damp_m / 2.5, self.kp_m / 1.4, self.delay / 3.0,
                                   self.mu_cap / 0.35]]).astype(np.float32)
        th = float(self.data.qpos[0])
        self.cmd, self.prev_dth, self.Fbar = th, 0.0, 0.0
        self.prev_at, self.prev_aK = 0.0, 0.0
        self.dq = deque([th] * (self.delay + 1), maxlen=self.delay + 1)
        self.steps = self.overforce_steps = self.n_succ = 0
        self.has_touched = False
        self.hist = deque([0.0] * self.tuner.m_hold, maxlen=self.tuner.m_hold)
        Fn, Ft, _ = self._tactile()
        f = self._frame(th, 0.0, Fn, Ft, 0.0)
        self.frames = deque([f] * H, maxlen=H)
        return self._obs(th)

    def step(self, a):
        a = np.clip(np.asarray(a, dtype=np.float64).reshape(ACT_DIM), -1.0, 1.0)
        Fn0, _, _ = self._tactile()
        slip_flag = bool(self.has_touched and Fn0 < 0.1 and self.data.qpos[1] > 0.1)

        # [Opus Fix: 가변 컴플라이언스 - 가상 강성 K를 행동으로 승격, K_t = K_min + (a_K+1)/2 (K_max-K_min)]
        self.K_t = K_MIN + 0.5 * (a[1] + 1.0) * (K_MAX - K_MIN)
        self.sim2real_ctrl.admittance.K = self.K_t

        # [Opus Fix: 실행 행동 사상 a~ = clip(cmd + dmax*tanh((d_base(o,g)+a_res*d_res)/dmax)) - cmd, 학습기 executed()와 동일]
        d_base = float(np.clip(K_F * F_REF * (self.g - self.Fo_c), -DELTA_BASE, DELTA_BASE))
        dth = DELTA_MAX * float(np.tanh((d_base + a[0] * self.delta_res) / DELTA_MAX))
        new_cmd = float(np.clip(self.cmd + dth, 0.0, TH_MAX))
        dth, self.cmd = new_cmd - self.cmd, new_cmd
        self.dq.append(self.cmd)

        _SimClock.t += DT
        pos, _ = self.sim2real_ctrl.step(self.dq[0], Fn0, slip_flag)
        self._set_kp((1.0, float(np.exp(-self.mu_cap * self.data.qpos[:3].sum()))))
        self.data.ctrl[0] = self.data.ctrl[1] = pos
        for _ in range(5):
            mujoco.mj_step(self.model, self.data)
        self.steps += 1

        Fn2, Ft2, sg2 = self._tactile()
        th2 = float(self.data.qpos[0])
        Fbar_prev = self.Fbar
        # [Opus Fix: EMA 필터 힘 F_bar를 achieved goal/potential/success의 기준 변수로 -> 채터링 분산 억제]
        self.Fbar = (1.0 - ETA) * self.Fbar + ETA * Fn2
        if Fn2 >= 0.15:
            self.has_touched = True
        # [Opus Fix: slip 레벨 트리거 -> 에지 트리거 (놓친 뒤 지속 벌점으로 인한 종료 유인 제거)]
        slip_edge = float(Fbar_prev >= 0.15 and self.Fbar < 0.10)
        slip2 = float(self.has_touched and Fn2 < 0.1 and self.data.qpos[1] > 0.1)

        self.overforce_steps = self.overforce_steps + 1 if Fn2 > self.current_fragile_limit else 0
        crushed = self.overforce_steps >= 3
        missed = th2 >= 1.45 and Fn2 < 0.01
        term, trunc = bool(crushed or missed), self.steps >= self.max_steps

        K_star = float(np.clip(RHO_K * self.current_fragile_limit / TH_REF, K_MIN, K_MAX))
        r_np, r_sf = self.tuner.penalties(sg2, self.K_t, K_star, K_MAX - K_MIN, dth, self.prev_dth, DELTA_MAX,
                                          slip_edge, Fn2 > self.current_fragile_limit, term)
        self.prev_dth, self.prev_at, self.prev_aK = dth, dth / DELTA_MAX, float(a[1])
        self.hist.append(self.Fbar / F_REF)
        succ = bool(self.tuner.hold_np(self.hist, self.g) and not term)
        self.n_succ += int(succ)

        self.frames.append(self._frame(th2, float(self.data.qvel[0]), Fn2, Ft2, slip2))
        obs = self._obs(th2)
        # [Opus Fix: 버퍼에는 실행된 행동 a~(critic 행동)과 목표 무관 보상 항만 전달]
        info = dict(act=np.array([dth / DELTA_MAX, a[1]], dtype=np.float32), r_np=r_np, r_sf=r_sf, term=term,
                    trunc=trunc, crushed=bool(crushed), missed=bool(missed), succ=succ,
                    hold=self.n_succ / self.max_steps)
        return obs, bool(term or trunc), info
