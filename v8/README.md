# V8: 2023~2026 연구 기반 1지 촉각 파지 환경·학습기

V7(`RLCtrl_*_v7_final.py`)을 `docs/research/04_team_guide.md`의 권장 세팅으로 다시 짠 버전입니다.
V7 파일은 건드리지 않았고, V8은 저장소 안에서만 실행됩니다(외부 `universal_grasp_env`, `sts3215_sim2real_env`,
`mcp_joint_control` 의존 없음).

## 실행

```bash
pip install -r requirements-v8.txt
python -m pytest tests/test_v8_smoke.py -q          # 스모크 테스트 (7개)
python -m v8.evaluate --penalty-check               # 패널티 규모 점검 (가이드 P0-2)
python -m v8.evaluate --policy fsm                  # FSM 기준선 (가이드 P0-4)
python -m v8.train_sac_her_v8 --seed 0              # 학습 (시드 3개 이상 권장)
python -m v8.evaluate --policy v8/runs/seed0/final_actor_v8.pth --seeds 0 1 2
```

## RTX 4090 + i9-12th (Windows) 실행 절차

ADR 002의 `.venv_310`(PyTorch 2.0 + DirectML)은 CUDA를 쓰지 못하므로 V8용 가상환경을 따로 만듭니다.
Windows에서 `pip install torch`만 하면 CPU 빌드가 설치되니 CUDA 인덱스를 지정하세요.

```powershell
cd <저장소 루트>
py -3.10 -m venv .venv_v8
.\.venv_v8\Scripts\Activate.ps1
pip install torch --index-url https://download.pytorch.org/whl/cu121
pip install "mujoco>=3.1" "numpy<2" pytest
python -c "import torch; print(torch.cuda.is_available())"   # True 여야 함
python -m pytest tests/test_v8_smoke.py -q
python -m v8.train_sac_her_v8 --seed 0 --workers 12          # 시드 0, 1, 2 각각 실행
```

- 장치: CUDA가 보이면 자동으로 GPU 학습기, 워커는 CPU NumPy 추론이라 GPU 메모리를 나눠 쓰지 않습니다.
- 워커 수: 기본값은 `min(12, 논리코어-2)` = i9-12th(24스레드)에서 12. 병목은 학습기(UTD 0.5)라 더 늘려도 이득이 작습니다.
- 반드시 저장소 루트에서 `python -m v8....` 형태로 실행하세요 (Windows spawn 방식 대응).
- 결과: `v8/runs/seed<N>/` 에 `log.csv`, `config.json`, `v8_ckpt.pt`, `final_actor_v8.pth`.

## V7 → V8 변경점

| 축 | V7 | V8 | 가이드 |
|---|---|---|---|
| 모델 | 외부 env에서 XML 생성 | `hand_model.py`에서 직접 생성. 자기충돌 없음(contype/conaffinity) | 03 §6 F13 |
| 정책 주기 | 100 Hz | **25 Hz** 정책 / 100 Hz 안전층 / 2 ms 물리 | P1-2 |
| 행동 | [잔차 Δθ, 가변 강성 K], 텐던 = MCP 명령 그대로 (`ctrl[0]=ctrl[1]`) | **[Δq_mcp, Δu_tendon]** + EMA 0.6, 텐던 분리 | P1-2 |
| 텐던 | kp 스케일 `exp(-μθ)` | 고정 텐던 경로(MCP·PIP·DIP 권선반경) + PIP 복귀 스프링 + DIP=0.88·PIP + η + 슬랙 + 20 ms ZOH + 모터 사양 전환(`TENDON_MOTOR`) | P2-2, M1 |
| 서보 | kp 배율 | armature, 토크 한계, 속도 한계, 백래시, 영점 오프셋 | P2-1 |
| 안전층 | 외부 FSM·어드미턴스·TDPA | 루프 안에서 슬루 + 초과력 어드미턴스 양보(0.9·F_th부터) + 해제 인터락(ADR 005) + MCP 연성 피드포워드 | P1-3 |
| Actor 관측 | 11차원 x 3 + 목표 | 13차원 x 3 + **g, F_th** (이진 촉각 3패드, 4단계 양자화 힘, Load, 위치오차, 이전 행동) | P1-2, P2-6 |
| 촉각 | 연속 힘 + 노이즈 | 작동 임계 0.1 N, 패드 드롭 10%, 스케일 DR, 양자화 | P2-6, M2 |
| 파지 보상 | tanh 포텐셜(PBRS) + Dense2Sparse | 목표 힘 **가우시안**, 접촉 시에만 + 유지 보너스 | P1-1 |
| 파손 | 이진 과힘 -0.5 + 3스텝 지속 시 대형 종료 벌점 | **선형 초과력 패널티** + **CaT 확률 종료**, 종료 벌점 없음 | P1-1 |
| 접근 | 포텐셜 h(s) | 최고 기록 갱신분만 | P1-1 |
| miss | 종료 | 없음 (시간 초과만) | P1-1 |
| DR | 9종, 넓은 full 범위 | 19종, 식별값 ±10~30% full, 행동·관측 지연 | P2-4, P2-5 |
| 커리큘럼 | phase 1~3 | 반경 2.6~3.4 cm → 1.5~4.0 cm 성능 기반 확장 + ADR | P1-5 |
| 평가 | hold, crush(3스텝) | **순간 최대력** 무파손율, 추종 RMS, 반경별, 시드별, FSM 기준선 | P0-3, §3.4 |
| 학습기 | SAC+HER, PopArt, CUDA Graph | SAC+HER, PopArt 유지. 목표 재라벨 시 Actor 관측 속 g도 교체. CUDA Graph 제거(1지 규모에서 병목 아님) | P1-4 |

## 현재 확인된 것

- 스모크 테스트 7개 통과, 3만 스텝 CPU 학습 루프가 끝까지 돕니다(4코어 CPU에서 약 100 transitions/s).
- 패널티 점검: still 0.00, random 0.65, FSM 43.9 → 통과. `w_da` 0.05에서는 실패해서 0.02로 내렸습니다.
- FSM 기준선 (시드 0~2, 반경당 30 에피소드, nominal DR):

| 반경 | 무파손율 | 유지 비율 | 추종 RMS | 미끄럼 | 최대력 |
|---|---|---|---|---|---|
| 1.5 cm | 0.70 | 0.44 | 0.09 N | 0.10 | 1.54 N |
| 2.0 cm | 0.77 | 0.44 | 0.06 N | 0.23 | 1.64 N |
| 2.6 cm | 0.60 | 0.41 | 0.07 N | 0.33 | 1.99 N |
| 3.4 cm | 0.67 | 0.29 | 0.10 N | 0.43 | 2.52 N |
| 4.0 cm | 0.63 | 0.29 | 0.12 N | 0.40 | 3.03 N |

  FSM도 접촉 순간 충격으로 30~40% 에피소드에서 F_th를 넘습니다. V6 "파손 0%"가 3스텝 규칙에 가려졌다는 가이드 P0-3과 같은 현상입니다.

## 아직 확인하지 못한 것

- **수렴까지의 학습은 돌려보지 않았습니다.** 이 환경의 CPU로는 수백만 스텝이 어려워, RL 정책이 FSM 기준선을 넘는지는 RTX 4090 환경에서 확인이 필요합니다.
- `config.py`의 `[추정]` 값(STS3215 kp·armature·속도, 권선반경, 텐던 모터 토크, 촉각 임계)은 가이드 §4 실측(M1~M7) 후 교체해야 합니다.
- 1지 모델이라 물체 무게·미끄럼은 반대편 지지 스프링으로 근사했습니다. 다지(4지 공유 정책 + 엄지)는 가이드 P3 단계입니다.
- ADR 002(PyTorch 2.0 / NumPy < 2)에서의 실행은 확인하지 않았습니다. 새 API는 쓰지 않았지만 팀 환경에서 한 번 돌려 보세요.
