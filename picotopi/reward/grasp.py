# -*- coding: utf-8 -*-
"""파지 보상 (V8 RewardV8와 동일) [가이드 P1-1, 02 §2].

V7 대비 바뀐 점
- 파지 보상: tanh 포텐셜(PBRS) -> 목표 힘 가우시안 추종, 접촉 시에만 지급 (ByteDance 2025)
- 파손: 이진 과힘 -0.5 + 대형 종료 벌점 R_term -> 초과력 선형 패널티 + CaT 확률 종료, 종료 벌점 없음 (FORGE, CaT)
- 접근: 포텐셜 h(s) -> "최고 기록 갱신분만" 지급 (DexPBT, DextrAH-G)
- miss(-10 종료) 제거, 행동 변화 패널티는 규모 점검(python -m picotopi.eval --penalty-check) 통과 값 사용
- Dense2Sparse 스케줄·PopArt 없이도 보상 규모가 유계가 되도록 각 항을 [0,1] 또는 작은 음수로 제한

목표(g) 의존 항(힘 추종·유지 보너스)은 학습기에서 HER 재라벨 목표로 다시 계산하고,
목표 무관 항(r_np)만 버퍼에 저장합니다. 보상 계산식은 numpy/torch 공용입니다.
"""
from dataclasses import dataclass

from .. import config as C


@dataclass
class GraspReward:
    w_F: float = 1.0          # 힘 추종 가중치
    sigma_F: float = 0.15     # N, 가우시안 폭 [가이드 §3.2]
    b_hold: float = 0.5       # m_hold 스텝 연속 추종 시 스텝당 보너스
    m_hold: int = 5           # 0.2 s @25 Hz
    eps_abs: float = 0.05     # N
    eps_rel: float = 0.15     # 목표 대비
    w_c: float = 0.5          # 접근 최고기록 갱신 보상 (1 cm 당)
    w_da: float = 0.02        # 행동 변화 패널티 (0.05에서 penalty-check 실패 -> 0.02)
    beta_over: float = 1.0    # 초과력/F_th 당 패널티 (FORGE β 0.2/N x F_th 5 N 과 같은 규모)
    p_drop: float = 0.5       # 접촉 상실(미끄럼) 에지 패널티
    cat_pmax: float = 0.5     # CaT 최대 종료 확률
    cat_cmax: float = 0.5     # 초과력/F_th 가 이 값이면 p_max
    g_min: float = C.G_LO / C.F_REF

    # ------------------------------------------------------------ 목표 의존
    def goal_terms_np(self, F_N, contact, hist_N, g_N):
        """스칼라 버전 (env 로그·평가용). hist_N: 최근 m_hold 스텝 평균 힘(N)."""
        import numpy as np
        track = self.w_F * contact * float(np.exp(-0.5 * ((F_N - g_N) / self.sigma_F) ** 2))
        tol = max(self.eps_rel * g_N, self.eps_abs)
        hold = float(len(hist_N) >= self.m_hold and np.all(np.abs(np.asarray(hist_N)[-self.m_hold:] - g_N) < tol))
        return track + self.b_hold * hold, hold

    def goal_terms_torch(self, F_N, contact, hist_N, g_N):
        """배치 버전 (학습기 HER 재계산). F_N, contact, g_N: (N,), hist_N: (N, m_hold)."""
        import torch
        track = self.w_F * contact * torch.exp(-0.5 * ((F_N - g_N) / self.sigma_F) ** 2)
        tol = torch.clamp(self.eps_rel * g_N, min=self.eps_abs)
        hold = ((hist_N - g_N[:, None]).abs() < tol[:, None]).all(-1).float()
        return track + self.b_hold * hold, hold

    # ------------------------------------------------------------ 목표 무관 (env에서 numpy로 계산)
    def nongoal_terms(self, d_improve, da, F_peak, F_th, slip_edge):
        r_app = self.w_c * max(d_improve, 0.0) / 0.01
        r_da = -self.w_da * float((da ** 2).sum())
        excess = max(F_peak - F_th, 0.0) / F_th
        r_over = -self.beta_over * excess
        r_slip = -self.p_drop * float(slip_edge)
        return r_app + r_da + r_over + r_slip, dict(app=r_app, da=r_da, over=r_over, slip=r_slip, excess=excess)

    def cat_term_prob(self, excess):
        return self.cat_pmax * min(excess / self.cat_cmax, 1.0)


RewardV8 = GraspReward      # V8 이름 호환
