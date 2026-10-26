# -*- coding: utf-8 -*-
"""촉각 센서 [가이드 P2-6, M2]."""
TAC_PADS = ("prox", "mid", "dist")
TAC_THRESH = 0.10         # N, FSR 작동 하한 [추정]
TAC_LEVELS = (0.15, 0.5, 1.5)   # 양자화 경계 N -> 0..3 단계
TAC_DROP_P = 0.10         # 패드별 에피소드 드롭 확률
OBS_CONT_FORCE = True     # 연속 힘 관측 사용 여부
LATCH_F = 0.15            # N, 첫 접촉(has_touched) 판정 및 FSM 래치
CONTACT_F = 0.02          # N, 시뮬 평균력 기준 접촉 판정
