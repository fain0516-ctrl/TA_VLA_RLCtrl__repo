# picotopi: V8에서 분리한 PICOtoPI 공용 모듈

V8(`v8/`)의 1지 촉각 파지 환경·학습기·평가를 재사용 단위로 나눈 패키지입니다. `v8/`은 건드리지 않았고,
같은 시드·같은 행동에서 **V8 원본과 비트 단위로 같은 궤적**을 내는 것을 테스트로 확인합니다
(`tests/test_picotopi.py::test_env_identical_to_v8`).

## 구조

| 모듈 | 내용 | 실기(Pi)에서 사용 |
|---|---|---|
| `config/` | timing · mechanics · sensing · control · task · dr 로 나눈 설정값. `from picotopi import config as C` | O |
| `control/action.py` | `ActionIntegrator`: [Δq_mcp, Δu_tendon] + EMA 0.6 적분 | O |
| `control/safety.py` | `SafetyLayer`: 초과력 어드미턴스 양보, 해제 인터락, 슬루, 백래시, 텐던 ZOH + MCP 피드포워드 | O |
| `control/observation.py` | `make_frame`, `FrameStack`: 13차원 프레임 x 3 + [g, F_th] = 41차원 | O |
| `control/tactile.py` | 작동 임계, 4단계 양자화 | O |
| `control/baselines.py` | still / random / 촉각 래치 FSM | O |
| `policy/numpy_actor.py` | torch 없이 도는 Actor 추론, `.pth -> .npz` 내보내기 | O |
| `reward/grasp.py` | `GraspReward` (= V8 `RewardV8`) | - |
| `sim/` | MJCF 생성, 에피소드 파라미터(DR·커리큘럼) 샘플링, `FingerGraspEnv` | - |
| `train/` | nets(Actor·비대칭 Critic·PopArt), replay(HER), schedulers(ADR·커리큘럼), workers, learner(SAC), loop | - |
| `eval/` | 반경별 지표, 패널티 규모 점검 | - |
| `runtime/` | `HardwareIO` 인터페이스, `SimHardwareIO`, `ControlLoop`(25 Hz 정책 + 100 Hz 안전층) | O |

의존 방향: `config <- control <- policy/reward <- sim <- train/eval`, `runtime`은 `config/control/policy`(+시험용 `sim.model`)만 씁니다.

## 실행 (저장소 루트에서)

```bash
python -m pytest tests/test_picotopi.py -q
python -m picotopi.eval --penalty-check
python -m picotopi.eval --policy fsm
python -m picotopi.train --seed 0 --workers 12 --out runs/seed0
python -m picotopi.eval --policy runs/seed0/final_actor.npz --seeds 0 1 2
python -m picotopi.policy.numpy_actor runs/seed0/final_actor.pth actor.npz   # Pi 배포용
```

## 실기 연결

`runtime/hardware.py`의 `HardwareIO`를 상속해 `read / tension / write / wait_tick`을 채우면 `ControlLoop`가
학습 환경과 같은 행동 적분기·안전층·관측 프레임으로 정책을 돌립니다. Pi<->Pico 통신 형식과 텐던 길이 -> 서보 명령
변환은 저장소에 정의돼 있지 않아 비워 두었습니다.

## V8 대비 달라진 점

- `pd`(스텝별 특권 정보)의 마지막 칸 `F_peak`가 V8에서는 reset 때 직전 에피소드 값이 남았습니다. 여기서는 0으로 초기화합니다.
  (첫 에피소드는 V8과 같고, 둘째 에피소드부터 reset 직후 Critic 입력 한 칸만 다릅니다.)
- 학습 산출물 이름: `ckpt.pt`, `final_actor.pth`, `final_actor.npz` (V8: `v8_ckpt.pt`, `final_actor_v8.pth`). 체크포인트 형식은 같습니다.
- 접촉 판정 0.02 N, 래치 0.15 N이 코드 상수에서 `config.sensing.CONTACT_F`, `LATCH_F`로 옮겨졌습니다(값 동일).
