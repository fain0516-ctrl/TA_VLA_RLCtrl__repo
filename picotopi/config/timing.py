# -*- coding: utf-8 -*-
"""시간 구조 [가이드 P1-2, 03 §2]."""
SIM_DT = 0.002            # 물리 dt 2 ms
CTRL_DT = 0.010           # 안전층 100 Hz
POLICY_DT = 0.040         # 정책 25 Hz
N_SUB = int(round(CTRL_DT / SIM_DT))        # 안전층 1틱당 물리 스텝 5
DECIM = int(round(POLICY_DT / CTRL_DT))     # 정책 1스텝당 안전층 틱 4
TENDON_ZOH = 0.020        # PCA9685 50 Hz 갱신 가정 [추정, 가이드 M7]
EP_SEC = 3.0
T_EP = int(round(EP_SEC / POLICY_DT))       # 75 정책 스텝
