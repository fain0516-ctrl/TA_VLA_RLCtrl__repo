# TA_VLA_RLCtrl__repo
코드_저장소
RLCtrl, 2026-09-26 , 연결 확인

## 구성

| 경로 | 내용 |
|---|---|
| `RLCtrl_*_v7_final.py` | V7 (SAC-HER, 외부 env 의존) |
| `picotopi/` | V8을 재사용 모듈로 나눈 PICOtoPI 공용 패키지 (설정·제어·정책·보상·시뮬·학습·평가·실행 루프) ([picotopi/README.md](picotopi/README.md)) |
| `tools/deploy_picotopi.ps1` | 로컬 드라이브(기본 `E:\PICOtoPI`)로 정리·복사 |
| `docs/report/` | 모듈화 보고서 (DOCX) |
| `v8/` | V8: 2023~2026 연구 기반으로 재구성한 1지 촉각 파지 환경·학습기·평가 ([v8/README.md](v8/README.md)) |
| `docs/research/` | 연구 정리 01~04 (논문 수집, 보상·훈련, 환경·sim-to-real, **팀 적용 가이드**) |
| `tests/test_v8_smoke.py`, `tests/test_picotopi.py` | V8 스모크 테스트, PICOtoPI 모듈 테스트(V8 동등성 포함) |
| `reverseeng/` | 기기 프로필 수집 골격 |
