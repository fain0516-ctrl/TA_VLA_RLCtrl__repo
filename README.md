# TA_VLA_RLCtrl__repo
코드_저장소
RLCtrl, 2026-09-26 , 연결 확인

## 구성

| 경로 | 내용 |
|---|---|
| `RLCtrl_*_v7_final.py` | V7 (SAC-HER, 외부 env 의존) |
| `v8/` | V8: 2023~2026 연구 기반으로 재구성한 1지 촉각 파지 환경·학습기·평가 ([v8/README.md](v8/README.md)) |
| `docs/research/` | 연구 정리 01~04 (논문 수집, 보상·훈련, 환경·sim-to-real, **팀 적용 가이드**) |
| `tests/test_v8_smoke.py` | V8 스모크 테스트 |
| `reverseeng/` | 기기 프로필 수집 골격 |
| `ros2_ws/` | **ROS 2 재구성**: 1조(관절각)·4조(촉각) 입력을 받는 3조 Motion Mapping·하위 제어기·서보 버스 ([ros2_ws/README.md](ros2_ws/README.md)) |
| `firmware/pico2_servo_bridge/` | SN74125N + Pico 2 자작 보드 MicroPython 펌웨어 (W5500 UDP ↔ STS3215·DS3225) |
