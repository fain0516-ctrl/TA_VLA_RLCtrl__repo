# -*- coding: utf-8 -*-
"""목표 조건부 SAC + HER 학습기 (V8 train_sac_her_v8.py 분리본).

nets       Actor / 비대칭 Critic / PopArt
replay     에피소드 리플레이 + 샘플 시점 HER 보상 재계산
schedulers ADR(경계 평가 기반 DR 확장) / 반경 커리큘럼
workers    CPU 워커 (NumPy Actor 추론) + 공유 메모리
learner    SAC 업데이트
loop       메인 루프 + CLI (python -m picotopi.train)
"""
