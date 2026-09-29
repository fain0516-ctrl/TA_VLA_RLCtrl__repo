# 01. 2023~2026 다지 로봇손 강화학습(피지컬 AI) 연구 수집·분류

작성: 2026-09-29 · 1단계(논문 수집) 결과물 · 2단계(보상/훈련), 3단계(환경/sim-to-real), 4단계(팀 적용 가이드)의 입력 자료

## 0. 읽는 법

- **대상 로봇**: 5지 그리퍼, 평면 굴곡만(외전 없음). PIP-DIP 텐던(DS3225), MCP·엄지(2모터) STS3215. 외부 센서: 촉각, 비전/VLA. 내장: Load, 온도, 자기식 엔코더. 현재 파이프라인: MuJoCo 10ms, SAC, 관측 3차원(q, F, slip), 행동 1차원(MCP 각도), Slew limiter + 어드미턴스 FSM (`AI_Priority_Rules/11_*.md`).
- **분류 태그**: `[훈련]` 알고리즘·하이퍼파라미터·커리큘럼·증류 / `[보상]` 보상 설계 / `[환경]` 시뮬레이터·도메인 랜덤화(DR)·액추에이터·센서 모델링 / `[정책]` 정책의 경향성(동결, 해킹, 떨림, 과력 등)과 구조(계층·잔차·VLA).
- **관련도**: ★★★ 우리 손에 거의 그대로 적용 가능, ★★ 원리 차용 가능, ★ 배경 지식.
- **검증 수준**: 링크는 모두 arXiv/공식 페이지를 검색으로 확인했습니다. 수치는 초록·본문에서 확인한 것만 적었고, 본문을 못 읽은 논문은 `(초록/제목 기준)`으로 표시했습니다. 2단계·3단계에서 수치를 표로 옮길 때는 원문 재확인을 권장합니다.

---

## 1. 저가·텐던·서보 하드웨어와 액추에이터 모델링 (우리 손과 가장 가까운 그룹)

| # | 논문 (연도) | 태그 | 관련도 | 핵심 교훈 (한 줄) |
|---|---|---|---|---|
| 1.1 | [Aero Hand Open: A Simulation-Ready Tendon-Driven Hand](https://arxiv.org/abs/2608.28578) (2026) | 환경·훈련 | ★★★ | **Feetech 서보 7개 + 텐던 1케이블/손가락(MCP-PIP-DIP) 구조를 MuJoCo spatial tendon 20개로 재현, 엔코더만으로 in-hand 회전 zero-shot 성공.** 성공 요인은 "액추에이터 수준 모델링" |
| 1.2 | [Tendon Force Modeling for Sim2Real Transfer of RL Policies for Tendon-Driven Robots](https://arxiv.org/abs/2603.04351) (2026, Yuryev & Hughes) | 환경 | ★★★ | 텐던 힘을 이력(history) 기반 Transformer로 학습해 시뮬에 삽입 → sim-to-real 격차 41%↓, 손끝 추종 50% 개선, 예측오차 최대 모터힘의 3% |
| 1.3 | [BAM: Extended Friction Models for the Physics Simulation of Servo Actuators](https://github.com/Rhoban/bam) (ICRA 2025, Duclusaud et al.) | 환경 | ★★★ | **STS3215가 사전 식별 모델로 포함됨.** 기본 Coulomb-점성 마찰 대신 Stribeck·부하 의존 마찰 모델 → "모델 정밀도가 정책 이전성에 직접 영향" |
| 1.4 | [ORCA: Open-Source, Reliable, Cost-Effective Anthropomorphic Hand](https://arxiv.org/abs/2504.04259) (2025, ETH SRL) | 환경 | ★★ | 텐던 구동 저가 손을 "장시간 무중단 학습"에 쓰려면 탈구형 관절·자동 캘리브레이션·텐던 장력 관리 같은 신뢰성 설계가 선행되어야 함 |
| 1.5 | [Getting the Ball Rolling: Dexterous Policy for a Biomimetic Tendon-Driven Hand](https://arxiv.org/abs/2308.02453) (Humanoids 2023, Faive) | 환경·훈련 | ★★ | 롤링 접촉 관절+텐던 손을 IsaacGym에서 관절 공간으로 학습하고 관절→텐던 매핑을 하위 제어기에 둠. 텐던 동역학을 직접 시뮬하지 않고도 이전 가능했던 사례 |
| 1.6 | [LEAP Hand: Low-Cost, Efficient, Anthropomorphic Hand](https://arxiv.org/abs/2309.06440) (RSS 2023) | 환경 | ★★ | 저가 서보 손의 표준 연구 플랫폼. 이후 논문 다수(1.x, 3.x, 5.x)가 LEAP로 재현됨 → 공개 URDF·학습 코드 재사용 가치 큼 |
| 1.7 | [MuJoCo Playground](https://arxiv.org/abs/2502.08844) (2025, Google DeepMind 외) | 환경·훈련 | ★★★ | MJX(GPU)로 LEAP 손 큐브 재배향을 수 분~수십 분에 학습하고 실물 이전. **우리처럼 MuJoCo를 쓰면 CPU 14환경 대신 GPU 수천 환경 PPO로 옮길 수 있는 직접 경로** |
| 1.8 | [ASAP: Aligning Simulation and Real-World Physics](https://arxiv.org/abs/2502.01143) (2025) | 환경 | ★★ | 실기 궤적으로 "delta action 모델"을 학습해 시뮬 동역학 보정 후 재학습. 서보 응답 불일치 보정 방법으로 차용 가능 |

## 2. 촉각·힘 기반 파지 (우리 목표: 파손·미끄럼 없는 포위 파지)

| # | 논문 (연도) | 태그 | 관련도 | 핵심 교훈 |
|---|---|---|---|---|
| 2.1 | [Closing the Reality Gap: Zero-Shot Sim-to-Real for Dexterous Force-Based Grasping](https://arxiv.org/abs/2601.02778) (2025.12, ByteDance Seed, xHand) | 전부 | ★★★ | **목표 파지력을 관측으로 넣고 추종시키는 힘 적응 파지를 zero-shot 이전한 첫 사례.** 모터 전류→토크 보정(토크센서 없음), 액추에이터 DR(정지토크·속도상수·마찰·백래시·지연), "손바닥 위로 떨어지는 물체 받기" 과제로 사전 파지 단계와 보상 해킹 제거. 보상: 토크/접촉력 가우시안 추종 + action rate + 관절속도 패널티 + 손가락 간 일관성 |
| 2.2 | [FORGE: Force-Guided Exploration for Robust Contact-Rich Manipulation](https://arxiv.org/abs/2408.04587) (RA-L 2025, NVIDIA) | 보상·환경·정책 | ★★★ | **허용 최대 접촉력을 정책 입력으로 주고 초과 시 패널티** → 하나의 정책이 실행 시 힘 한계를 바꿔가며 안전하게 동작. 우리의 `fragile_limit`를 Critic이 아니라 **Actor 입력**으로 줘야 한다는 근거 |
| 2.3 | [Rotating without Seeing: In-Hand Dexterity through Touch](https://arxiv.org/abs/2303.10880) (RSS 2023) | 환경 | ★★★ | 손 전체 16개 저가 FSR을 **이진(접촉/비접촉)** 으로만 사용 → 연속 힘값보다 sim-to-real 격차가 작음. 저가 촉각센서의 권장 처리법 |
| 2.4 | [DexTouch: Seek and Manipulate Objects with Tactile Dexterity](https://arxiv.org/abs/2401.12496) (RA-L 2024) | 환경 | ★★ | 이진 촉각만으로 물체 탐색·조작. 2.3의 결론(이진화)을 재확인 |
| 2.4b | [General In-Hand Object Rotation with Vision and Touch (RotateIt)](https://arxiv.org/abs/2309.09979) (CoRL 2023) | 훈련·환경 | ★★ | 특권정보 teacher → 시각·촉각·고유감각 student 증류. 촉각은 이진 접촉 + 위치 임베딩 |
| 2.5 | [AnyRotate: Gravity-Invariant In-Hand Rotation with Sim-to-Real Touch](https://arxiv.org/abs/2405.07391) (CoRL 2024) | 환경 | ★★ | 고해상도 광학 촉각을 "접촉 자세+힘"의 저차원 특징으로 압축해 이전. 원시 촉각 이미지를 그대로 쓰지 말 것 |
| 2.6 | [Robot Synesthesia: In-Hand Manipulation with Visuotactile Sensing](https://arxiv.org/abs/2312.01853) (ICRA 2024) | 환경 | ★★ | 촉각을 점군으로 변환해 비전 점군과 결합. 비전+촉각 융합 표현의 한 방식 |
| 2.7 | [TacSL: Library for Visuotactile Sensor Simulation and Learning](https://arxiv.org/abs/2408.06506) (2024, NVIDIA) | 환경·훈련 | ★ | GPU 촉각 시뮬 + 비대칭 actor-critic 증류(AACD). 광학 촉각을 쓸 경우 참고 |
| 2.8 | [3D-ViTac: Fine-Grained Manipulation with Visuo-Tactile Sensing](https://arxiv.org/abs/2410.24091) (CoRL 2024) | 환경 | ★★ | 저가 압저항 촉각 패드(밀집)로 연약 물체 파지. 모방학습 기반이나, **압저항 센서 사용 시 캘리브레이션·노이즈 처리** 참고 |
| 2.9 | [Learning Gentle Grasping Using Vision, Sound, and Touch](https://arxiv.org/abs/2503.07926) (2025) | 보상 | ★★ | 파지 시 발생하는 소리로 "부드러운 파지" 레이블을 자동 생성해 학습. 과력 판정을 사람 라벨 없이 얻는 방법 |
| 2.10 | [Blind Dexterous Grasping via Real2Sim2Real Tactile Policy Learning](https://arxiv.org/abs/2606.11767) (2026) | 환경·훈련 | ★★ | 실제 촉각 신호를 재현하도록 접촉 보정한 디지털 트윈 → 물체별 RL 전문가 → 촉각 조건 Diffusion Policy로 통합. **비전 없이 촉각만으로는 실제 성공률 27%(20개 물체)** 에 그침 → 촉각 단독은 아직 약함 |
| 2.11 | [See to Reach, Feel to Grasp: Blind Grasp Reflex for Anthropomorphic Hands](https://arxiv.org/abs/2609.31323) (2026.09) | 정책 | ★★★ | (제목/초록 기준, 본문은 요청 제한으로 미확인) 비전으로 접근, 촉각 반사로 파지를 분리하는 구조. **우리 구조(VLA 상위 + 촉각 하위 파지)와 동일한 분업** → 3·4단계에서 본문 재확인 필요 |
| 2.12 | [Adaptive Compliance Policy](https://arxiv.org/abs/2410.09309) (2024) | 정책 | ★★ | 정책이 위치 목표와 함께 **강성(stiffness)** 도 출력. 고정 어드미턴스 파라미터 대신 학습 가능한 컴플라이언스 |

## 3. Sim-to-real 파이프라인 (in-hand 조작·파지 일반)

| # | 논문 (연도) | 태그 | 관련도 | 핵심 교훈 |
|---|---|---|---|---|
| 3.1 | [Sim-to-Real RL for Vision-Based Dexterous Manipulation on Humanoids](https://arxiv.org/abs/2502.20396) (CoRL 2025, Lin et al.) | 전부 | ★★★ | 4가지 레시피: ① 실기 데이터로 시뮬 파라미터 자동 튜닝(real-to-sim autotune) ② 접촉 목표(contact goal) 기반 범용 보상 ③ 과제 분할 후 증류(divide-and-conquer) ④ 희소+밀집 물체 표현 혼합 |
| 3.2 | [DeXtreme: Transfer of Agile In-Hand Manipulation](https://arxiv.org/abs/2210.13702) (ICRA 2023) | 환경·훈련 | ★★ | 물리·관측·행동 지연까지 광범위 DR + 자동 DR(ADR). 비대칭 actor-critic(Critic에만 특권정보)이 표준 |
| 3.3 | [Visual Dexterity](https://arxiv.org/abs/2211.11744) (Science Robotics 2023) | 훈련·환경 | ★★ | 저가 3지 손, teacher-student + 점군. 저가 하드웨어로도 일반 물체 재배향 가능 |
| 3.4 | [Lessons from Learning to Spin "Pens" (PenSpin)](https://arxiv.org/abs/2407.18902) (CoRL 2024) | 훈련 | ★★★ | **시뮬만으로는 실패**: 특권정보 oracle을 시뮬에서 학습 → 그 궤적을 실기에 open-loop 재생해 성공분만 수집 → 소수 실데이터로 정책 미세조정. 순수 zero-shot이 안 될 때의 표준 탈출구 |
| 3.5 | [DextrAH-G](https://arxiv.org/abs/2407.02274) (CoRL 2024) / [DextrAH-RGB](https://arxiv.org/abs/2412.01791) (2024) | 정책·환경 | ★★★ | **정책 출력 아래에 geometric fabric(기하학적 안전 제어층)** 을 두어 관절한계·충돌·속도를 구조적으로 보장. 우리 Slew limiter + 어드미턴스를 "시뮬 학습 시점부터 포함"하는 설계를 지지 |
| 3.6 | [IndustReal](https://arxiv.org/abs/2305.17110) (RSS 2023) | 보상·훈련·정책 | ★★ | SDF 기반 밀집 보상, 샘플링 기반 커리큘럼, **policy-level action integrator**(절대 목표 대신 증분 행동 적분)로 실기 정착 오차 감소 |
| 3.7 | [Twisting Lids Off with Two Hands](https://arxiv.org/abs/2403.02338) (CoRL 2024) | 보상·환경 | ★ | 키포인트 기반 보상과 단순한 물체 표현으로 양손 조작 이전 |
| 3.8 | [RobustDexGrasp](https://arxiv.org/abs/2504.05287) (CoRL 2025) | 훈련·환경 | ★★ | 손 중심(hand-centric) 물체 표현 + 혼합 커리큘럼 + 외란 주입으로 단일 시점 파지 일반화 |
| 3.9 | [UniDexGrasp++](https://arxiv.org/abs/2304.00464) (ICCV 2023) / [CrossDex](https://arxiv.org/abs/2410.02479) (2024) / [DemoGrasp](https://arxiv.org/abs/2509.22149) (ICLR 2026) | 훈련 | ★ | 기하 인식 커리큘럼, 교차 구현체 학습, 단일 시연 기반 파지 일반화. 물체 다양성이 커질 때 참고 |
| 3.10 | [DexterityGen](https://arxiv.org/abs/2502.04307) (2025) | 정책 | ★ | 시뮬 RL로 학습한 저수준 "손재주 기반(foundation) 컨트롤러" 위에 텔레옵/상위 명령. 계층 구조 근거 |

## 4. 보상 설계와 자동화

| # | 논문 (연도) | 태그 | 관련도 | 핵심 교훈 |
|---|---|---|---|---|
| 4.1 | [Eureka: Human-Level Reward Design via Coding LLMs](https://arxiv.org/abs/2310.12931) (ICLR 2024) | 보상 | ★★ | LLM이 보상 코드 생성·반복 개선. 29개 환경에서 83% 과제가 전문가 보상 이상. 보상 항 가중치 탐색 자동화에 활용 |
| 4.2 | [DrEureka: Language Model Guided Sim-to-Real](https://arxiv.org/abs/2406.01967) (RSS 2024) | 보상·환경 | ★★ | 보상과 함께 **DR 범위까지 LLM이 설계**. 먼저 정책이 버티는 물리 범위를 측정(RAPP)한 뒤 DR 범위를 정함 |
| 4.3 | [Text2Reward](https://arxiv.org/abs/2309.11489) (ICLR 2024) | 보상 | ★ | 자연어→밀집 보상 코드 |
| 4.4 | [Lipschitz-Constrained Policies (LCP)](https://arxiv.org/abs/2410.11825) (2024) | 보상·정책 | ★★★ | **떨림 억제를 보상 패널티(action rate/jerk) 대신 정책 그래디언트 패널티로 해결.** 저크 패널티를 키우면 정책이 "안 움직이기"로 수렴하는 문제(우리 V6 실패와 동일 현상)를 피하는 대안 |
| 4.5 | 3.1 Lin et al.의 contact-goal 보상, 2.1 xHand의 가우시안 힘 추종 보상, 3.6 SDF 보상 | 보상 | ★★★ | 공통 경향: **단계별 if-else 보상보다 연속·유계(bounded) 추종 보상**(exp(-e²/σ²) 꼴)을 쓰고, 패널티는 작은 가중치의 정칙화로만 둠 |

## 5. 훈련 알고리즘·스케일·탐색

| # | 논문 (연도) | 태그 | 관련도 | 핵심 교훈 |
|---|---|---|---|---|
| 5.1 | [DexPBT: Population Based Training for Dexterous Manipulation](https://arxiv.org/abs/2305.12127) (RSS 2023) | 훈련 | ★★ | PPO + PBT로 하이퍼파라미터·보상 가중치를 학습 중 자동 탐색 → 단일 실행 대비 큰 향상 |
| 5.2 | [Sampling-based Exploration for RL of Dexterous Manipulation](https://arxiv.org/abs/2303.03486) (RSS 2023) | 훈련 | ★ | 희소 보상에서 샘플링 기반 계획으로 초기 상태 분포를 넓혀 탐색 개선 |
| 5.3 | [SERL](https://arxiv.org/abs/2401.16013) (ICRA 2024) / [HIL-SERL](https://arxiv.org/abs/2410.21845) (2024) | 훈련 | ★★★ | **SAC 계열을 쓰는 사례는 대부분 "실기 직접 학습"**: RLPD(시연+온라인 데이터 50:50), 높은 UTD, 학습형 성공 판별기, 사람 개입으로 1~2.5시간 내 실기 학습. 우리 SAC 파이프라인이 실기 미세조정 단계로 갈 때의 레시피 |
| 5.4 | 공통 관찰 (1.7, 2.1, 3.1, 3.2, 3.5) | 훈련 | ★★★ | **시뮬 대규모 학습은 거의 전부 PPO + GPU 수천 병렬 환경 + 비대칭 actor-critic.** SAC + CPU 14환경은 문헌상 드문 조합이며 수렴 속도·안정성 모두 불리 |

## 6. 상위 VLA·계층 구조·잔차 RL

| # | 논문 (연도) | 태그 | 관련도 | 핵심 교훈 |
|---|---|---|---|---|
| 6.1 | [π0](https://arxiv.org/abs/2410.24164) (2024) / [GR00T N1](https://arxiv.org/abs/2503.14734) (2025) | 정책 | ★★ | VLM(느린 판단) + 행동 전문가(빠른 연속 제어) 이중 시스템. 우리 "VLA → 피지컬 AI → 하위 제어기" 3층 구조와 대응 |
| 6.2 | [DexGraspVLA](https://arxiv.org/abs/2502.20900) (2025) | 정책 | ★★★ | VLM 계획기가 대상 선택·박스 지정, 하위 diffusion 제어기가 파지. **상위는 "무엇을/어디를", 하위는 "어떻게 쥘지"** 로 분리해야 일반화 |
| 6.3 | [ResiP](https://arxiv.org/abs/2407.16677) (2024) / [Policy Decorator](https://arxiv.org/abs/2412.13630) (ICLR 2025) | 정책·훈련 | ★★ | 기반 정책(BC/VLA) 위에 **작은 잔차 RL 정책**을 얹어 정밀도 향상. 잔차 크기 제한이 안정성 핵심 |
| 6.4 | [ConRFT](https://arxiv.org/abs/2502.05450) (2025) / [VLA-RL](https://arxiv.org/abs/2505.18719) (2025) / [SimpleVLA-RL](https://arxiv.org/abs/2509.09674) (2025) | 훈련 | ★ | VLA 자체를 RL로 후학습. 현 단계보다는 추후 과제 |
| 6.5 | [Towards High-DoF Dexterous Manipulation through VLA Post-Training](https://arxiv.org/abs/2609.19666) (2026.09) | 훈련 | ★ | (제목 기준) 고자유도 손에 VLA 후학습 적용 — 최신 동향 |
| 6.6 | [RL-Augmented Teleoperation and Mixture-of-Dexterous-Experts VLA](https://arxiv.org/abs/2603.08122) (2026) | 정책 | ★ | (제목 기준) RL 전문가로 텔레옵 데이터를 보강하고 전문가 혼합 VLA로 통합 |
| 6.7 | [Scaffolding Dexterous Manipulation with VLMs](https://arxiv.org/abs/2506.19212) (2025) | 정책 | ★ | VLM이 키포인트·궤적 같은 "발판"을 주고 RL이 그것을 추종 |

---

## 7. 네 축으로 본 종합 (2·3단계가 이어받을 요약)

### 7.1 훈련 세팅
- ✅ **권장**: GPU 대량 병렬 PPO(1.7, 3.1, 3.2, 2.1), 비대칭 actor-critic(Critic에 특권정보: 물체 크기·질량·마찰·파손 한계), teacher→student 증류(2.4b, 3.1, 3.3), PBT 등 자동 하이퍼파라미터 탐색(5.1).
- ✅ **SAC를 유지할 경우**: 실기 미세조정 단계에서 RLPD·시연 데이터 혼합·학습형 성공 판별기(5.3).
- ⚠️ **실패/비권장 신호**: 순수 zero-shot이 안 되는 과제(PenSpin, 3.4), 촉각 단독 파지(2.10, 실제 27%).
- 🔎 **우리 V6에서 Critic 입력을 Actor와 같게(3차원) 통일한 결정은 문헌 표준(비대칭)과 반대**입니다. 문헌 기준으로 보면 문제는 "비대칭"이 아니라 Phase 1에서 특권 특징이 상수라 정보가 없던 것, 또는 정규화 문제일 가능성이 큽니다 → 2단계에서 재검토 항목.

### 7.2 보상 세팅
- ✅ 연속·유계 추종 보상(가우시안 힘 추종, 접촉 목표, SDF) + 작은 정칙화 패널티(action rate, 관절 속도)(2.1, 3.1, 3.6).
- ✅ 파손 한계를 **정책 입력**으로 주고 초과분만 패널티(2.2 FORGE).
- ✅ 떨림 억제는 보상 대신 구조로: Lipschitz 제약(4.4), 안전 제어층(3.5), 증분 행동 적분(3.6).
- ⚠️ 패널티가 과제 보상을 매 스텝 잠식하면 정책이 동결(Frozen policy)됨 — 우리 V6(저크 -0.5 vs 파지 +2.0)과 같은 현상. 4.4는 이 문제를 동기로 제시.
- ⚠️ 큰 음수 종료 보상(-10, -30)과 조기 종료의 조합은 "빨리 끝내기"나 "접촉 회피"를 유도하기 쉬움(2.1은 이런 해킹을 피하려 과제 자체를 받기 형태로 바꿈).

### 7.3 환경 세팅
- ✅ **액추에이터 모델링이 1순위**: STS3215 마찰 식별(1.3 BAM), 텐던 라우팅을 MuJoCo spatial tendon으로 재현(1.1), 텐던 힘 학습 모델(1.2), 전류/부하→토크 보정(2.1, 우리 Load 센서에 해당).
- ✅ DR 대상: 물체(질량·크기·마찰·형상), 액추에이터(정지토크·속도·마찰·백래시·지연), 관측 노이즈·지연(2.1, 3.2). 범위는 정책 강건성 측정 후 설정(4.2).
- ✅ 저가 촉각은 이진화 또는 저차원 특징으로(2.3, 2.4, 2.5).
- ✅ 실기 데이터로 시뮬 자동 튜닝(3.1, 1.8, 2.10).
- ✅ 시뮬 속도: MuJoCo MJX(1.7)로 GPU 전환이 가장 적은 비용의 개선 경로.

### 7.4 정책 경향성·구조
- 동결 정책(과대 패널티), 보상 해킹(사전 파지 단계 우회), 떨림(action chattering), 과력 파지가 반복 보고되는 실패 모드.
- 구조 권장: 상위(VLA: 무엇을/어디를) + 중위(RL 파지 정책: 얼마나 세게/언제 멈출지) + 하위(안전층·서보 폐루프)(6.1, 6.2, 2.11, 3.5).
- 기반 정책 위 잔차 RL(6.3)은 VLA 출력을 보정하는 형태로 확장 가능.

## 8. 우리 손에 비해 문헌이 비어 있는 부분 (4단계 후보)
1. **혼합 구동 손**(텐던 PIP-DIP + 직결 서보 MCP)을 한 정책으로 학습한 사례는 찾지 못했습니다. 1.1은 전 관절 텐던, LEAP은 전 관절 직결.
2. **평면 굴곡 전용(외전 없음) 5지 손**의 포위 파지 RL은 드뭅니다. 대부분 16~22 DoF 손. 자유도가 적은 만큼 관측/행동 공간을 작게 유지해도 된다는 근거는 1.1(엔코더만 7개)이 가장 가깝습니다.
3. **모터 온도**를 관측이나 제약으로 쓰는 RL 논문은 거의 없습니다(2.1이 DR 이유로 온도 언급만). 과열 방지는 하위 제어기 규칙으로 처리하는 편이 현실적.
4. DS3225(PWM 서보, 피드백 없음)처럼 **상태 피드백이 없는 액추에이터**의 sim-to-real은 문헌 공백. 텐던 쪽 상태는 촉각·STS3215 엔코더로 간접 추정해야 할 가능성.
5. 현재 행동 1차원(MCP만)은 문헌 대비 매우 축소된 형태이므로, 손가락별·관절군별로 확장할 때 커리큘럼 설계가 필요.

## 9. 확인하지 못한 것
- 2.11(arXiv 2609.31323) 본문: 요청 제한(HTTP 429)으로 읽지 못해 제목·초록 기준으로만 분류했습니다.
- 6.5, 6.6은 제목·검색 요약 기준입니다.
- 2.1의 보상 가중치·DR 범위 수치는 본문 발췌에 없어 기재하지 않았습니다(3단계에서 원문 재확인 필요).
