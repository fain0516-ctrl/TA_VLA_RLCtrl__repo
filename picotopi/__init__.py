# -*- coding: utf-8 -*-
"""PICOtoPI 공용 모듈 (V8 1지 촉각 파지에서 분리).

계층 (아래쪽은 위쪽을 import하지 않음):
    config   설정값 단일 출처 (시간 구조, 기구, 액추에이터, 촉각, 안전층, 과제, DR)
    control  하드웨어 무관 제어 블록 (행동 적분기, 안전층, 촉각 처리, 관측 프레임, FSM 기준선)
    policy   NumPy Actor 추론 (torch 없이 Pi에서 실행) + .pth -> .npz 내보내기
    reward   보상 (numpy/torch 공용 식)
    sim      MuJoCo 모델 생성, 에피소드 파라미터(DR·커리큘럼) 샘플링, 학습 환경
    train    SAC+HER 학습기 (네트워크, PopArt, 에피소드 리플레이, ADR·커리큘럼, 워커)
    eval     평가 프로토콜 (지표, 반경별 표, 패널티 규모 점검)
    runtime  실기/모의 실행 루프 (25 Hz 정책 + 100 Hz 안전층) + 하드웨어 입출력 인터페이스
"""
__version__ = "0.1.0"
