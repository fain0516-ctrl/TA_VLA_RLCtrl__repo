#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import torch
from dataclasses import dataclass

F_REF, TH_MAX = 2.0, 1.57


@dataclass
class RewardTunerV7:
    gamma: float = 0.99
    horizon: int = 150
    m_hold: int = 3
    eps_abs: float = 0.025
    eps_rel: float = 0.15
    c_s: float = 0.05
    w_F: float = 1.0
    kappa: float = 0.25
    w_a: float = 0.5
    b_c: float = 0.5
    lam_sig: float = 0.2
    sig0: float = 0.8
    lam_K: float = 0.1
    lam_j: float = 0.05
    lam_slip: float = 0.3
    lam_over: float = 0.5
    margin: float = 1.05
    g_min: float = 0.1

    def __post_init__(self):
        # [Opus Fix: 종료 지배 조건 R_term <= -c_max * sum_{k<T} gamma^k -> miss/crush 자살 유인 제거]
        self.c_max = self.c_s + self.lam_sig + self.lam_K + self.lam_j + self.lam_slip + self.lam_over
        self.R_term = -self.margin * self.c_max * (1.0 - self.gamma ** self.horizon) / (1.0 - self.gamma)

    @staticmethod
    def beta(k, K):
        # [Opus Fix: Dense2Sparse 코사인 스케줄 beta_k, 비포텐셜 dense 항에만 적용]
        return 0.5 * (1.0 + float(np.cos(np.pi * min(k / K, 1.0))))

    def tol(self, g):
        return torch.clamp(self.eps_rel * g, min=self.eps_abs)

    def hold_ok(self, hist, g):
        # [Opus Fix: 상대 허용오차 + m스텝 유지 성공 판정 -> 채터링 우연 성공(hindsight bias) 억제]
        return ((hist - g[:, None]).abs() < self.tol(g)[:, None]).all(1)

    def phi(self, ag, g):
        # [Opus Fix: 유계 포텐셜 |Phi| <= w_F + w_a + b_c, 필터 힘 F_bar 기반, 접촉 보너스를 Phi_c = b_c*h(s)로 편입]
        return -self.w_F * torch.tanh((ag[..., 0] - g).abs() / self.kappa) + self.w_a * ag[..., 1] + self.b_c * ag[..., 2]

    def reward(self, ag, ag2, hist2, g, r_np, r_sf, term, beta):
        # [Opus Fix: r = r_sp + [gamma(1-d)Phi(s') - Phi(s)] (beta=1 고정, 정책 불변) + beta_k r_np + r_safe(비감쇠)]
        r_sp = self.c_s * (self.hold_ok(hist2, g).float() - 1.0)
        pbrs = self.gamma * (1.0 - term) * self.phi(ag2, g) - self.phi(ag, g)
        return r_sp + pbrs + beta * r_np + r_sf

    def hold_np(self, hist, g):
        return bool(np.all(np.abs(np.asarray(hist) - g) < max(self.eps_abs, self.eps_rel * g)))

    def penalties(self, sigma, K_t, K_star, K_range, dth, prev_dth, dmax, slip_edge, over, fail):
        # [Opus Fix: 목표 무관 항만 저장 (실행 행동 기준) -> HER relabel 시 보상 일관성 보장]
        # [Opus Fix: 연속형 단일 sigma 항, 행동화된 K에 대한 제어 가능한 컴플라이언스 항, 각 항 상한]
        r_np = (-self.lam_sig * min(max(sigma - self.sig0, 0.0) / (1.0 - self.sig0), 1.0) ** 2
                - self.lam_K * ((K_t - K_star) / K_range) ** 2
                - self.lam_j * ((dth - prev_dth) / (2.0 * dmax)) ** 2)
        r_sf = -self.lam_slip * slip_edge - self.lam_over * float(over) + self.R_term * float(fail)
        return float(r_np), float(r_sf)
