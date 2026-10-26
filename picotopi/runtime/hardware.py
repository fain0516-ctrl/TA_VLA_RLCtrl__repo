# -*- coding: utf-8 -*-
"""하드웨어 입출력 인터페이스.

ControlLoop는 이 인터페이스만 봅니다. 실기 구현(예: Pi에서 Pico와 통신하는 클래스)은 HardwareIO를 상속해
네 메서드를 채우면 됩니다. Pi<->Pico 통신 형식은 이 저장소에 정의돼 있지 않아 여기서 정하지 않았습니다.

단위 약속 (시뮬과 같음)
- read()  -> dict(tac=(3,) N [prox, mid, dist], q=MCP 각 rad, load=STS3215 부하 / 최대토크 (대략 -1~1))
- tension() -> 텐던 장력 N 추정값. 측정 수단이 없으면 0을 반환 (해제 인터락이 꺼지는 것과 같음)
- write(mcp_rad, tendon_len_m) : MCP 서보 목표각, 텐던 길이 명령 (음수 = 당김, 0 = 이완).
  텐던 길이 -> 텐던 서보 각/PWM 변환은 구현 쪽 책임 (권선 반경 실측 전 [추정], 가이드 M1·M7)
- wait_tick() : 100 Hz 주기 맞춤 (시뮬은 물리 N_SUB 스텝 진행)
"""
import abc

import mujoco
import numpy as np

from .. import config as C
from ..control.tactile import tactile_threshold
from ..sim.model import build_xml


class HardwareIO(abc.ABC):
    @abc.abstractmethod
    def read(self):
        ...

    def tension(self):
        return 0.0

    @abc.abstractmethod
    def write(self, mcp_rad, tendon_len_m):
        ...

    @abc.abstractmethod
    def wait_tick(self):
        ...

    def close(self):
        pass


def nominal_params(radius=0.026, F_th=2.0, g=0.5):
    """DR nominal 구간 중앙값 + 과제값. SimHardwareIO와 런타임 시험용."""
    p = {k: 0.5 * (v[0] + v[1]) for k, v in C.DR.items()}
    p.update(radius=radius, F_th=F_th, g=g, tendon_fmax=C.tendon_force_max(), drop=np.zeros(3))
    return p


class SimHardwareIO(HardwareIO):
    """MuJoCo 모델을 실기 대용으로 쓰는 구현 (런타임 루프 시험, 하드웨어 연결 전 리허설)."""

    def __init__(self, params=None, seed=0):
        self.p = params or nominal_params()
        self.rng = np.random.default_rng(seed)
        self.model = mujoco.MjModel.from_xml_string(build_xml(self.p))
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.g_obj = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "obj")
        self.g_pads = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n) for n in C.TAC_PADS]
        mujoco.mj_forward(m, self.data)
        self.F_true = np.zeros(3)

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

    def read(self):
        p, d = self.p, self.data
        self.F_true = self._pad_forces()
        tac = tactile_threshold(np.maximum(0.0, self.F_true + self.rng.normal(0, p["tac_noise"], 3)))
        load = d.actuator_force[0] / (C.STS_TAU_MAX * p["tau"])
        return dict(tac=tac, q=float(d.qpos[0] + self.rng.normal(0, p["q_noise"])),
                    load=float(np.clip(load, -1.5, 1.5)))

    def tension(self):
        return float(-self.data.actuator_force[1])

    def write(self, mcp_rad, tendon_len_m):
        self.data.ctrl[0], self.data.ctrl[1] = mcp_rad, tendon_len_m

    def wait_tick(self):
        for _ in range(C.N_SUB):
            mujoco.mj_step(self.model, self.data)
