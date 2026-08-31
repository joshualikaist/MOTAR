# MOTAR 교수님 발표용 PPT 제작 마스터 브리프

> **계보 단절 (2026-08-27)** — corrected v2와 그 이전 사이에는 **세 개의 독립 단절**이 있다:
> 배치 기하(중첩→비중첩, 40×40×3), **heading 임계(`1e-5` → `0.10`, 10,000배)**, 밀도 계보(70→300 →
> 70→205). 따라서 corrected v2에서의 PPO 성능은 **NOT RUN**이며, 과거 205-bar 수치는 `historical`
> 로만 인용한다. 상세: `VERIFICATION.md` "계보 단절".
>
> **학습 상한 205는 2026-08-27 기하 감사에서 body+tracking 기준 PASS다** (connectivity
> 99.167%, no-route 0.833%, 생성 실패 0). 300 bars는 같은 기준으로 FAIL(94.661% / 5.339%).
> 본학습 상한은 205로 유지한다. 220/250은 연결 OOD, 300은 단절 스트레스다. 이 감사가 PPO
> 권한을 주지 않으며, corrected-v2 성능은 여전히 **NOT RUN**이다. 슬라이드에서 205를
> "학습된 성능 상한"으로 제시하지 말 것 — 연결된 기하 상한일 뿐이다.

작성 기준: 2026-08-27 revision
청중: 지도교수 및 드론·강화학습·제어 연구자  
권장 발표 시간: 본편 18–22분 + 질의응답 10분  
권장 분량: 본편 18장, appendix 10–12장  

이 문서는 지금까지의 연구 전체를 한 발표로 묶기 위한 **제작 명세**다. Claude는 이 파일을
성과 홍보문이 아니라 근거가 제한된 연구발표 계약으로 사용해야 한다. 결과가 FAIL 또는
INCONCLUSIVE여도 지우지 말고, 왜 그 판정이 다음 연구 방향을 좁혔는지를 설명한다.

## 0. Claude에 그대로 전달할 최상위 요청

> 첨부한 `CLAUDE_PPT_MASTER_BRIEF_2026-08-26.md`를 최상위 계약으로 삼아 16:9 교수님 연구발표용
> PPT를 작성해 주세요. 본편은 18장, appendix는 10–12장으로 구성하고, 하드웨어 예상 플랫폼,
> simulation/task contract, sensor data flow, Transformer high-level policy, low-level flight
> controller, reward/PPO/curriculum parameters, held-out 결과, perception/latency 분석,
> physical-target/recovery 실패, sim-to-real gap과 다음 72시간 계획을 모두 포함하세요.
>
> 성공처럼 보이게 포장하지 마세요. `PASS_32_CELL_INTEGRITY`는 실행 무결성이지 알고리즘 성공이
> 아니며, route mechanism은 `FAIL_ROUTE_MECHANISM`, no-anchor 후속은 `INCONCLUSIVE`, 실제 비행은
> 0회입니다. simulation·synthetic·hardware-pending을 모든 관련 슬라이드에 표시하세요.
> 외부 논문의 제목·수치·DOI는 원문 확인이 없으면 `[CITATION NEEDED]`로 남기고 만들어내지 마세요.
> 각 수치의 speaker note에 이 문서가 지정한 원자료 경로를 적으세요.
>
> **2026-08-27 lineage 정정을 최우선으로 적용하세요.** 기존 capture/crash 수치는 중첩을 허용한
> historical `navrl_band` 환경의 결과이며, 새 `footprint_clearance` 환경의 성능으로 표시하면
> 안 됩니다. 새 환경은 arena 40×40×3 m, `navrl_ref5in_v2_quad`, collision proxy 0.283 m,
> surface clearance 0.45 m, overlap/merge fallback 0, 학습 목표 70→205 bars, 평가 asset ceiling
> 300 bars입니다. 새 환경의 PPO 성능은 아직 `NOT RUN`입니다.
>
> 디자인은 흰색/짙은 녹색/민트의 단순한 연구발표 스타일로 통일하고, 본문 24 pt 이상,
> 제목 34 pt 이상, 표 18 pt 이상을 유지하세요. 한 슬라이드에는 하나의 결론만 두고, 긴 문장 대신
> 도식·그래프·강조 수치를 사용하세요. 기존 `motar-system-overview.svg`와
> `motar-control-stack.svg`는 재사용하되 16:9 안에서 읽히도록 확대·분할하세요.

## 1. 발표의 중심 메시지

### 한 문장

MOTAR는 camera/LiDAR 기반 구조화 관측만 받는 UAV가 밀집 장애물장에서 이동 표적을 요격하는
문제를 다루며, 최고 capture 하나보다 **밀도·표적 운동·인지 지연·동역학 계약이 언제 어떤 실패를
만드는지 재현 가능하게 분해**한다.

### 발표가 답해야 할 세 질문

1. 센서-only actor와 실제적인 비행 명령 범위만으로 이동 표적 요격을 어떻게 구성했는가?
2. density, speed, visibility/latency가 capture·crash·timeout에 각각 어떤 영향을 주는가?
3. simulation에서 잘 보이는 결과를 실제 기체 주장으로 넘기기 전에 어떤 physical/hardware gate가
   실패했으며, 다음에 무엇을 실제로 측정해야 하는가?

### 절대 바꾸지 않을 최종 판정

| 범주 | 최종 판정 | 의미 |
|---|---|---|
| Track A · detection Stage 1 | `RANGE_INCONCLUSIVE_AT_THIS_BUDGET` | 28 m clip이 관측을 개선했지만 사전 primary −15 pp gate 미달 |
| Track A · P2/D1/P3 | `STRICT FAIL / FAIL / BLOCKED` | historical sensor-range Track A의 Stage 2 금지 |
| Historical Track B · recovery-v2 | `PASS_32_CELL_INTEGRITY / FAIL_ROUTE_MECHANISM` | overlap-permitting 환경의 32셀; recovery 0/16 |
| Track B · no-anchor | `INCONCLUSIVE`, primary `n=1` | 전형적 원인을 판정할 표본 없음 |
| Corrected non-overlap v2 | route gate r2 32/32 integrity PASS · `FAIL_ROUTE_MECHANISM` · PPO `NOT RUN`(0 epoch) | 70-bar plan 17.78%, fallback 30.02%, 0.6 goals/env 0.21875; smoke·장기학습 차단 |
| hardware | 미조립·real log 0·flight 0 | sim-to-real 성공 주장 금지 |

기계 판독 원본: `docs/research_authority_2026-08-26.json`  
검증 명령: `python tools/check_research_authority.py --json`

## 2. 권장 서사 구조

발표는 “알고리즘 소개 → 좋은 숫자” 순서가 아니라 다음 인과 흐름으로 진행한다.

1. 이동 표적 요격은 navigation보다 target visibility와 relative motion이 추가된다.
2. 이를 sensor-only structured observation + temporal policy + fixed flight control로 구성했다.
3. historical 환경의 의미론·평가 오류를 분리하고, 중첩 없는 corrected-v2 계약을 새로 세웠다.
4. held-out 결과에서 density 영향이 speed 영향보다 컸다.
5. 인지 지연·미취득을 분해하니 timestamp와 range contract의 중요성이 드러났다.
6. 더 실제적인 target dynamics를 넣자 route/controller feasibility가 먼저 실패했다.
7. 따라서 지금의 기여는 sim-to-real 성공이 아니라 **실패를 계층별로 분리하는 검증 방법과 수치**다.
8. 다음은 더 긴 학습이 아니라 실제 BOM·calibration·210 sensor trials다.

## 3. 본편 18장 상세 구성

### Slide 1 — Title

제목 후보:

> **MOTAR: Failure-Aware Sensor-Only UAV Interception in Dense Obstacle Fields**

부제:

> From structured perception to physically bounded control — and where the contract fails

화면:

- 좌측: 제목, 이름, 연구실, 날짜.
- 우측: 205-bar 3D arena 화면 또는 `docs/assets/motar-system-overview.svg` 일부.
- 하단 badge: `SIMULATION-ONLY · HARDWARE PENDING · 2026-08-26`.

발표 멘트:

> “이 연구는 최고 성공률을 주장하는 발표가 아니라, 센서 기반 이동표적 요격에서 실패 원인을
> 인지·정책·제어·기하·하드웨어 계약으로 나눠 확인한 과정입니다.”

### Slide 2 — Problem and research question

제목: **Why moving-target interception is not ordinary goal navigation**

그림: 세 축을 삼각형으로 표시.

- target visibility: 표적이 sensor range/FOV 밖일 수 있음.
- relative motion: 목표점이 매 step 이동함.
- obstacle density: 빠른 접근과 stopping margin이 충돌함.

Research question을 구체적으로 표시:

> 제한된 camera/LiDAR 표현과 bounded flight command만으로 밀집 장애물 속 이동 표적을 얼마나
> 안정적으로 요격할 수 있으며, density 증가 시 실패는 perception·control·geometry 중 어디에서
> 발생하는가?

피해야 할 표현: “Transformer가 RNN보다 좋기 때문에 사용했다.” 이 비교는 별도 종합 gate가 없다.

### Slide 3 — Scope and information firewall

제목: **What the actor sees — and what it never sees**

2열 도식:

Actor input:

- camera/RGB-D에서 추출된 **target track**, raw 4-channel image가 아님.
- LiDAR `4×72`, nominal 12 m.
- ego-state: body/world velocity, yaw/attitude-derived terms, height, history validity.
- structured obstacle/target history.

Training-only:

- GT target/vehicle state: reward, termination, asymmetric critic, evaluation labels.
- actor에 GT target position, semantic mask, visibility oracle 직접 입력 금지.

교수님 예상 질문 대응:

- “RGB-D와 LiDAR를 왜 같이 쓰나?” → RGB-D 계열은 target track, LiDAR는 obstacle geometry로 역할이
  다르다. raw RGB-D tensor와 LiDAR를 동시에 network에 넣는 구조가 아니다.
- “proprioception이 무엇인가?” → 슬라이드에서는 `ego-state (velocity, yaw/attitude-derived state,
  height)`라고 풀어 쓴다.

원자료: `docs/MOTAR_SYSTEM_SPEC_2026-08-24.md` §3.

### Slide 4 — Task and environment contract

제목: **Corrected-v2 task contract**

표:

| 항목 | 값 |
|---|---:|
| arena | 40×40×3 m, area 1,600 m² |
| obstacles | height 3 m, square footprint 0.4–0.8 m side distribution |
| density | fresh training target 70→205 bars; asset/evaluation ceiling 300; YOPOv2 count-density reference cells 64/100 |
| 205-bar count density | 12.81 / 100 m² |
| 205-bar nominal occupancy | 약 4.681% |
| target speed | base fresh(route off): mixed CV/waypoint; routed fresh(`global_astar_v1`): waypoint-only; both U[0.3,1.5] m/s with a 1-epoch ramp. Historical generic v2 mixed/300-epoch values must not be spliced in |
| goal distance | U[6,28] m; hard diagnostic U[22.5,28] m |
| capture | swept relative segment enters 0.5 m radius |
| episode | exact 600 actions, 60 s |
| rates | 100 Hz physics / 10 Hz policy |

밀도 환산표(40×40 m = 1,600 m², finite bar pool 평균 footprint 0.365313 m²):

| bars | count /100m² | nominal gross occupancy | 발표 역할 |
|---:|---:|---:|---|
| 64 | 4.00 | 1.46% | YOPOv2 5 m count-density reference |
| 100 | 6.25 | 2.28% | YOPOv2 4 m count-density reference |
| 205 | 12.81 | 4.68% | corrected fresh ID training target |
| 250 | 15.63 | 5.71% | OOD evaluation only |
| 300 | 18.75 | 6.85% | OOD/geometry stress only |

그림: arena top view + camera/LiDAR range circles. 12 m LiDAR와 target-camera clip 20 m를 서로 다른
원으로 표시한다.

YOPOv2-Tracker의 simulation navigation은 평균 간격 5 m/4 m를 각각 `4/6.25 trees/100m²`로
명시한다. MOTAR의 같은 개수 밀도는 64/100 bars이고, 205 bars는 12.81/100m²로 YOPOv2 최밀집
조건의 2.05배다. 이는 **count-density context**일 뿐 난이도·성공률 우월성 주장이 아니다.
YOPOv2 본문이 이 비교 절에서 나무 직경 분포를 주지 않으므로 `0.6 m tree` 기반 면적 비교를
공식 수치처럼 쓰지 않는다. 원문: arXiv:2505.06923, Sec. IV-C1.

### Slide 5 — Expected hardware platform

제목: **Hardware-informed simulation candidate, not a built vehicle**

중앙에 5-inch-class quad schematic. 확정값과 미측정값을 색으로 분리한다.

Simulation candidate:

| 항목 | 값 |
|---|---:|
| name | `navrl_ref5in_v2_quad` (fresh-only) |
| nominal mass | 1.20 kg |
| motor-to-motor diagonal | 220 mm |
| collision proxy | 0.283×0.283×0.12 m (0.2826 m prop-tip span을 바깥쪽으로 반올림) |
| maximum thrust | 9.60 N/motor |
| nominal T/W | 3.262 |
| motor lag | 0.04 s first order |
| prop-tip AABB | 약 0.2826 m |

미측정/미선정:

- exact motor/prop/ESC/battery, camera/LiDAR model과 mounting.
- AUW, CG, inertia, thrust curve, thermal/power/endurance.
- intrinsics/extrinsics, FOV, latency/skew, dropout/range error.

슬라이드 우측에 큰 문구: **“Repository-consistent candidate ≠ flight-validated hardware.”**

2026-08-27 정정(본 발표에서 반드시 반영): 기존 정량 결과는 `navrl_band`가 가까운 막대를
의도적으로 겹쳐 compound obstacle로 만들던 historical 환경이다. 300 bars가 실제로 평균 약
264개 독립 component였으므로 이를 “서로 독립인 300개 막대”라고 표현하면 안 된다. 새 코드는
실제 collision footprint 외접원 기준 최소 0.45 m 표면 여유, overlap 0, merge fallback 0을
강제한다. 이 변경은 새로운 task distribution이므로 기존 PPO 성능곡선과 연결하지 말고,
engineering gate 뒤 fresh PPO가 완료되기 전까지 새 환경의 capture 수치는 `NOT RUN`으로 둔다.

원자료: `aerial_gym/config/robot_config/navrl_ref5in_v2_quad_config.py`,
`resources/robots/quad/quad_navrl_ref5in_v2.urdf`,
`resources/models/environment_assets/objects/navrl_target_drone_v2.urdf`.
Historical v1 파일은 과거 checkpoint provenance 때문에 수정하지 않았다.

### Slide 6 — End-to-end high-level architecture

제목: **Perception → structured memory → policy → flight command**

`docs/assets/motar-system-overview.svg`를 전체 폭으로 사용한다. 단, 아래 라벨을 추가한다.

1. Sensing: target track / static LiDAR / ego-state.
2. Packing: 898-D observation.
3. Tokenization: 17×64-D.
4. Temporal Transformer: 4 layers, 4 heads.
5. Actor: bounded 4-D distribution.
6. Fixed controller/plant.

별도 점선:

- GT → reward/central critic only.
- riskcap → post-training inference-only candidate; PPO가 학습한 layer가 아님.

### Slide 7 — Data structure and token semantics

제목: **What is inside the 898-D observation?**

stacked bar 또는 token 그림:

| block | raw dimension | temporal interpretation |
|---|---:|---|
| static scan | 288 | current 4×72 geometry |
| obstacle histories | 480 | 5 samples × 8 obstacles × features |
| robot histories | 50 | 5 samples |
| target histories | 80 | 5 samples |
| total | 898 | 17 learned tokens after projection |

Token path:

- static scan → small CNN → 64-D token.
- obstacle/robot/target block → per-type MLP → 64-D tokens.
- `[CLS] + static + obstacle(5) + robot(5) + target(5)` = 17 tokens.
- learned positional embedding, Transformer FFN 128, dropout 0.0.

History는 5 sample, 약 2 s의 고정 창이다. “Transformer가 무한 history를 보존한다”고 말하지 않는다.
RNN은 더 긴 정보를 압축할 수 있으므로, 여기서는 **field-aligned parallel token processing과 구현된
contract**가 선택 이유라고 설명한다.

원자료: `aerial_gym/task/navrl_task/navrl_perception.py`,
`aerial_gym/rl_training/rl_games/navrl_transformer_network.py`.

### Slide 8 — Policy and action distribution

제목: **What PPO actually learns**

그림: Transformer CLS → actor MLP 256–256 → μ, σ → squashed bounded action.

표:

| 항목 | 값 |
|---|---:|
| actor/critic hidden | 256, 256 ELU |
| action | 4-D bounded squashed Gaussian |
| nominal std contract | [0.35, 0.35, 0.05, 0.08] |
| horizontal command | per-axis ±2.5 m/s |
| yaw-rate | ±3.0 rad/s |
| vertical channel | actor output은 실행하지 않고 altitude PI가 overwrite |

강조:

- PPO가 학습하는 것은 network weight와 action distribution이다.
- arena, sensor geometry, reward coefficient, controller, motor/URDF는 고정 계약이다.
- capture rate 하나만으로 Transformer/RNN 우열을 주장하지 않는다. crash, timeout, latency,
  smoothness, parameter/FLOP와 seed가 함께 필요하다.

### Slide 9 — Low-level flight-control stack

제목: **Learned navigation, fixed low-level control**

`docs/assets/motar-control-stack.svg`를 사용하되 발표에서는 상단 1–4와 하단 5–8을 두 번 확대해
순차 애니메이션한다.

실제 순서:

1. policy action → body-frame `v*xy`, yaw-rate.
2. altitude PI: `z*=1.0 m`, integral clip ±2.5, vertical speed ±2.5 m/s.
3. Lee velocity loop: `Kv=[2.5,2.5,2.5]`.
4. force vector and 45° tilt limit.
5. desired attitude + altitude-priority thrust compensation.
6. attitude/rate torque: `KR=[1.0,1.0,0.5]`, `Kω=[0.15,0.15,0.15]`.
7. fixed allocation matrix → four motors.
8. 0.04 s motor lag → 100 Hz rigid-body physics.

중요 구분:

- 위 gain은 **pursuer** controller다.
- physical target diagnostic의 target-side controller gain `[0.08,0.08,0.04]` 및
  `[0.04,0.04,0.03]`은 별도 계보다. 한 표에 섞지 않는다.
- 큰 각도로 즉시 방향을 바꾸는 virtual point와 6-DoF physical actor를 구분한다.

### Slide 10 — Reward contract and justification

제목: **Reward encourages closing and survival without paying for loitering**

3개 그룹으로 표현:

Dense:

- relative range-rate `+1.0`.
- ego-motion progress `+1.0·(d_prev−0.99d_new)`.
- static safety `1.5·r_static`.

Regularization/cost:

- time `−0.05/step`.
- action smoothness `−0.1`.
- height `−8.0`, yaw alignment `−0.3`, yaw-rate² `−0.02`.
- detector-visible bonus `+0.02/step` (현재 live reward; 후보 항이 아님).

Terminal:

- capture `+30` at 0.5 m.
- collision `−20` overwrite.
- timeout bonus 없음.

정당성 답변:

> “보상은 과제의 유일한 수학적 정의라고 주장하지 않습니다. 목표 접근에 대한 dense credit,
> 충돌/성공의 terminal preference, 제어 가능성 regularization으로 분해했습니다. NavRL의 +1 alive
> bonus는 capture가 terminal인 본 과제에서는 오래 버티기를 보상해 loitering을 만들므로 −0.05 time
> cost로 바꿨습니다. moving target의 progress는 formal policy-invariant PBRS가 아니라 target(t+1)에
> 재고정한 heuristic임을 명시합니다. 최종 성능은 reward가 아니라 held-out capture/crash/timeout으로
> 판정했습니다.”

`no differentiable per-step cost` 같은 표현은 사용하지 않는다. 거리 기반 dense loss/reward는 만들 수
있으며, 본 연구의 선택은 그중 하나다.

### Slide 11 — PPO and curriculum contract

제목: **Training contract: one policy, many fixed conditions**

표:

| parameter | canonical value |
|---|---:|
| envs | 128 (4 GB profile 64) |
| horizon | 32 |
| minibatch / mini-epochs | 2048 / 4 |
| gamma / GAE | 0.99 / 0.95 |
| launcher LR | 3e-5 |
| clip / grad norm / critic coef | 0.2 / 1.0 / 2.0 |
| canonical entropy | 0.0 |
| KL stop/rollback | 0.04 / rollback on, LR×0.5 |
| density | fresh physical training 70→205, +15; `NAVRL_MAX_BARS=300`은 OOD 평가용 asset ceiling |
| dwell / evidence | 1,000 epochs / 16,384 episodes |
| promotion | 70:.82, 85:.77, 100:.72, 115+:.70 |
| target speed | U[0.3,1.5], 1-epoch ramp; base fresh is mixed, routed fresh is waypoint-only |

주의:

- YAML default LR 1e-4·entropy 0.003과 canonical launcher override를 구분한다.
- 205는 ID 학습 상한이고 220/250/300은 새 정책 평가 시 OOD로 표시한다.
- generic historical v2 launcher의 300 기본값과 fresh physical launcher의 205 목표를 섞지 않는다.
- best reward checkpoint가 아니라 terminal/last checkpoint와 SHA를 평가에 사용한다.

### Slide 12 — Evaluation protocol before results

제목: **Why the numbers are comparable**

도식: preregistration → source/checkpoint hash → held-out cells → receipt → verdict/VOID.

필수 설명:

- deterministic held-out evaluation, requested/actual episode counts 기록.
- capture/crash/timeout 합계 검증과 crash cause 분해.
- training/eval robot·sensor·observation·reward contract 비교.
- source manifest와 checkpoint snapshot SHA.
- threshold는 결과 전에 동결, 실패한 셀도 보존.
- `PASS integrity`와 `PASS mechanism/performance`는 다른 축.

이 슬라이드가 있어야 뒤의 FAIL/INCONCLUSIVE가 연구 성과로 이해된다.

### Slide 13 — Main held-out density × speed result

제목: **Historical density map — not the corrected non-overlap result**

왼쪽 그래프: speed 0.3에서 bars별 capture.

| bars | capture | crash |
|---:|---:|---:|
| 130 | 88.29% | 7.42% |
| 160 | 86.10% | 9.80% |
| 190 | 82.63% | 13.95% |
| 205 | 81.21% | 14.89% |
| 220 OOD | 77.94% | 18.50% |

오른쪽 그래프: 205 bars에서 speed별 capture/crash.

| target speed | capture | crash |
|---:|---:|---:|
| 0.3 | 81.21% | 14.89% |
| 0.7 | 81.21% | 16.11% |
| 1.1 | 80.77% | 17.08% |
| 1.5 | 78.15% | 20.10% |

하단 결론:

- density cost 약 −11.36 pp.
- speed cost 약 −2.67 pp.
- trained support 내 interaction 미확인: p=.337/.817.
- 220 bars는 OOD로 별도 표시.

조건: historical overlap-permitting `navrl_band`, ep25000 frozen policy + post-training riskcap
candidate, seed 47, 약 2,050 ep/cell. 슬라이드 전체에 `HISTORICAL · NOT VALID FOR NEW LINEAGE`
watermark를 둔다.
riskcap은 학습 layer가 아님을 각주로 명시한다.

### Slide 14 — Curriculum ceiling and representation limit

제목: **Historical trainable ceiling near 100 bars**

그래프:

- 85: 0.737 promoted.
- 90: 0.718 promoted.
- 95: 0.670→0.709 promoted.
- 100: 0.521–0.631, 17 windows, plateau≈0.56.

해석:

- 특정 cluster-sector sensor-only policy/training contract의 ceiling.
- geometry의 절대 한계나 모든 algorithm의 한계가 아님.
- 8 obstacle token은 밀집 장면을 압축하므로 representation bottleneck 후보지만 인과 확정은 아님.
- corridor-token pilot은 capture 66.10%, gain +1.57 pp로 사전 gate를 통과하지 못했다.
- 새 비중첩 physical lineage의 trainable ceiling은 아직 측정하지 않았다(`NOT RUN`).

### Slide 15 — Perception and timing lessons

제목: **Visibility and timestamp semantics change the diagnosis**

세 패널:

1. Camera-range diagnostic Stage 1:
   - never-acquired 8.443→3.172%.
   - capture 82.235→88.677%는 secondary.
   - primary Δ−5.271 pp, required ≤−15 pp → `INCONCLUSIVE`.
2. Latency correction:
   - naive 0.1 s: −42.7 pp, **superseded** due wrong pose lifting.
   - timestamp-aligned 0.1 s residual 약 −2.5 pp.
   - aligned 0.5 s 약 −15.8 pp.
3. dropout 0.3: 약 −12.7 pp, synthetic distribution.

메시지:

> “센서 지연 자체만큼, detection timestamp와 ego pose를 같은 시간축으로 결합하는 것이 중요하다.”

실제 camera model/latency/range error는 미측정이므로 sim-to-real 수치로 쓰지 않는다.

### Slide 16 — Physical target and route recovery failure

제목: **Historical route gate: a valid 32-cell run can still fail the mechanism**

상단 흐름:

virtual/bounded target → 6-DoF physical target → global route → local recovery → controller.

핵심 수치:

- lower-1.25 grid: 32/32 integrity PASS.
- 전체 7/32 PASS, 모두 route-off.
- recovery 0/16.
- 70-bar plan success 190/203 = 93.60%, gate≥99%.
- fallback 18,381/38,400 = 47.87%, gate≤1%.
- recovery occupancy 63.06% `NO_CONNECTOR`.
- no-anchor follow-up primary n=1 → `INCONCLUSIVE`.

해석:

- motor saturation/tilt/contact만으로 설명되지 않는다.
- fail-closed route/recovery state machine이 executable connector를 안정적으로 만들지 못했다.
- 1.25 contract는 canonical 1.5 성공이 아니다.
- planner는 pursuer planner가 아니라 **target-side environment motion controller**다.
- 이 결과는 old `navrl_band` geometry이며 corrected non-overlap gate 결과가 아니다.

### Slide 17 — What is proven, what is not

제목: **Evidence boundary**

초록/빨강 2열:

Supported:

- historical semantic/receipt pipeline의 교정 기록과 corrected-v2 계약(성능 NOT RUN).
- density/speed held-out map under frozen simulation contract.
- learned detector non-inferiority: −0.0145 pp, CI [−1.752,+1.723].
- timestamp-aware latency diagnosis.
- physical target route mechanism failure and bottleneck counters.
- corrected v2 airframe open-arena GPU envelope and non-overlap placement implementation.

Not supported:

- actual flight or sim-to-real performance.
- successful route recovery or physical target PPO.
- 300-bar full connectivity/performance.
- corrected non-overlap PPO capture/crash/timeout 또는 205-bar mastery.
- universal Transformer superiority over RNN.
- real sensor range/noise/latency distribution.
- exact 5-inch hardware feasibility.

중앙 문구: **“The result is a reproducible failure map, not a deployment claim.”**

### Slide 18 — Corrected-environment execution plan / closing

제목: **Validate geometry, smoke once, then train only to 205**

Immediate simulation sequence:

1. 64/70/100/130/160/190/205/220/250/300 bars exact topology audit: **완료**. 205 PASS,
   300 FAIL이며 이는 geometry 판정이지 PPO 성능이 아니다.
2. corrected non-overlap route/physical engineering gate: **완료, FAIL_ROUTE_MECHANISM**.
3. 70 bars fresh PPO 500-epoch smoke: **미실행, 선수 gate 실패로 차단**.
4. single-seed 70→205 fresh PPO: **미실행, smoke 이전 단계에서 차단**.
5. 다음 software 후보는 braking-aware safe-state를 보존하는 target route/controller를 새로
   사전등록한 뒤 mechanism gate부터 다시 통과하는 것이다. 현재 결과로 threshold 완화 금지.
6. held-out 64/70/100/130/160/190/205 + OOD 220/250/300: **fresh PPO 이전 단계에서 차단**.

300 bars는 asset/evaluation ceiling이며 자동 학습 목표가 아니다. 205→300 continuation은 topology와
205 held-out 결과를 보기 전에는 승인하지 않는다. 실제 hardware track의 BOM/calibration/210 trials는
이 simulation sequence와 별도이며 여전히 미실행이다.

마지막 문장:

> “먼저 중첩 없는 환경이 실제로 풀 수 있는 과제인지 확인하고, 205 bars까지만 새 계보로 학습한 뒤,
> 300 bars는 성능 절벽과 기하학적 한계를 측정하는 OOD 조건으로 남깁니다.”

## 4. Appendix 권장 구성

### A1 — Full hardware/URDF table

mass, box, inertia, motor geometry, max thrust, T/W, motor time constant, unidentified parameters와
source SHA를 표로 둔다. `hardware-informed candidate` watermark 필수.

### A2 — Full observation schema

898 fields의 block order, unit, normalization, clipping, validity mask, history time offset을 표시한다.
실제 field-by-field 값은 `navrl_perception.py`에서 재검증한다.

### A3 — Network parameter diagram

static CNN, four projection MLP, 17×64 token, 4×Transformer, actor/value heads. parameter count와 FLOPs는
실측 스크립트가 없으면 빈칸 또는 `[MEASURE]`로 둔다.

### A4 — Full action/controller parameters

body/world transform, altitude PI, velocity/attitude/rate gain, yaw/tilt limits, allocation, motor lag.
target-side physical controller는 별도 작은 표로 분리한다.

### A5 — Reward equation

모든 항과 정확 계수를 한 줄 수식으로 표시한다. moving-target progress는 heuristic 표식.

### A6 — PPO/curriculum table

launcher override와 YAML default를 두 열로 나눠 혼동을 방지한다. 특히 generic historical
v2 `70→300`과 corrected physical fresh `70→205`, `MAX_BARS=300`을 세 개의 독립 필드로 표시한다.

### A7 — Crash taxonomy

capture / bar contact / below / above / OOB / timeout의 mutual attribution 순서와 episode count 합계 검사를
flow chart로 표시한다.

### A8 — Detector and latency details

analytic detector, learned-v2 detector, target track, range clipping/normalization, timestamp lifting을 구분한다.
“detector가 raw GT인가?” 질문에 답할 수 있어야 한다.

### A9 — Riskcap

- post-training inference-only safety governor.
- PPO reward/network에 포함되지 않음.
- frozen parameter derivation과 causal/confounded comparison을 구분.
- riskcap 사후 tuning으로 결과를 만들지 않았음을 명시.

### A10 — Historical route mechanism cell map

bars 70/150/205/300 × speed 0.6/0.9/1.2/1.25 × route off/on의 32셀 pass/fail 표.
integrity와 mechanism verdict를 색상으로 분리한다.
`HISTORICAL navrl_band · NOT THE NEW NON-OVERLAP GATE` 라벨을 표 위에 둔다.

### A11 — Superseded/VOID registry

- v1 24×24 results: chirality/phantom-wall 때문에 main claim에서 제외.
- naive 0.1 s latency −42.7 pp: timestamp confound로 superseded.
- routed attempt 1: conda ninja PATH failure로 VOID.
- mode probe: `INCONCLUSIVE_POLICY_CHIRALITY`.

### A12 — Exact next data contract

실기에서 기록할 필수 항목과 210-trial matrix, GO/NO-GO 기준을 넣는다.

## 5. 교수님 예상 질문과 답변

### Q1. 왜 Transformer인가? RNN보다 수치적으로 좋은가?

답변:

> 이 발표는 Transformer의 보편적 우월성을 주장하지 않습니다. 현재 구현은 sensor type과 time step을
> 17개의 고정 token으로 정렬해 병렬 처리하기 편해 Transformer를 사용했습니다. architecture 비교는
> capture뿐 아니라 crash/timeout, latency, smoothness, parameter/FLOP, seed를 맞춘 종합 gate가 있어야
> 합니다. RNN은 더 긴 history를 압축할 수 있으므로 이 결과만으로 열등하다고 말할 수 없습니다.

### Q2. Capture rate만으로 모델이 좋다고 판단했나?

> 아닙니다. primary는 held-out capture지만 crash와 timeout, cause attribution, action/stopping margin,
> latency, confidence interval, execution integrity를 함께 봅니다. capture가 늘어도 crash가 함께 늘면
> 실패모드 이동으로 판정합니다.

### Q3. Reward는 왜 정당한가?

Slide 10 답변을 사용한다. 특히 formal PBRS 과대주장 금지, held-out metric으로 검증했다고 말한다.

### Q4. RGB-D와 LiDAR를 동시에 쓰는가? 입력이 4-channel인가?

> actor가 raw RGB-D 4-channel tensor를 직접 받는 구조가 아닙니다. camera/RGB-D 계열에서 target track을
> 만들고, LiDAR는 static obstacle geometry를 제공합니다. 두 역할을 structured feature로 합칩니다.

### Q5. riskcap은 학습 중인가 inference-only인가?

> 학습 후 적용하는 inference-only filter입니다. policy weight나 reward에 포함되지 않습니다. 따라서
> riskcap 결과를 learned-policy 단독 효과로 말하지 않습니다.

### Q6. 205 bars가 NavRL/YOPOv2보다 정말 어려운가?

> YOPOv2 navigation simulation은 평균 간격 5 m와 4 m를 각각 4와 6.25 trees/100m²로 정의합니다.
> MOTAR의 동일 count-density cell은 64와 100 bars이고, 205 bars는 12.81/100m²로 YOPOv2 최밀집
> 조건의 2.05배입니다. 그러나 obstacle shape, speed(4–10 m/s 대 0.3–1.5 m/s target), sensor,
> success definition과 moving-target interception task가 달라 “2배 어렵다”고 말하지 않습니다.
> 새 비중첩 계보의 205-bar 성능은 아직 NOT RUN입니다.

### Q7. 표적이 화면에서 부자연스럽게 움직이던데 학습 환경도 그런가?

> 브라우저 viewer는 설명용이며 PhysX/PPO replay가 아닙니다. 학습에는 legacy/bounded 계보가 있고,
> physical/routed target은 별도 feasibility diagnostic입니다. corrected non-overlap 6-DoF target
> 계보도 2026-08-31 32셀 route mechanism gate를 통과하지 못해 PPO를 시작하지 않았습니다(0 epoch).

### Q8. Timeout을 줄이려고 episode를 무한히 늘리면 되지 않나?

> 시간만 늘리면 미취득/search 문제를 가리고 task difficulty를 바꿉니다. 60 s/600 action을 고정하고
> never-acquired와 first-acquisition을 분리했습니다. 실제 endurance와 mission time을 측정한 뒤 horizon을
> 정해야 합니다.

### Q9. 속도 제한을 더 풀면 되지 않나?

> 속도를 올리면 stopping distance와 curvature 요구가 증가합니다. physical target strict gate에서
> 고밀도는 0.6 m/s 이상부터 tracking/planner 조건이 실패했고, recovery-v2도 1.25 contract에서 실패했습니다.
> 실제 thrust/drag/motor lag 측정 없이 limit만 높이지 않습니다.

### Q10. 왜 sim-to-real 연구라고 부르나? 실제 비행이 없는데?

> 현재는 sim-to-real **준비와 실패 분석 단계**입니다. hardware-informed candidate, provenance-bound
> contract, timestamp/latency schema와 hardware measurement plan을 만들었지만 transfer 성능은 아직
> 주장하지 않습니다. 발표 제목/결론에서도 hardware pending을 명시합니다.

## 6. 시각 디자인 계약

- 16:9, 1920×1080 기준.
- background: off-white `#F7FAF8`; main ink `#17231F`; green `#087F6D`; fail red `#C4554D`.
- title 34–42 pt, body 24–28 pt, tables minimum 18 pt.
- 본편 한 장 최대 bullet 5개, 표 최대 6행. 전체 파라미터는 appendix.
- PASS는 초록색 하나로 쓰지 않는다. `integrity PASS / mechanism FAIL`은 각각 다른 badge.
- simulation-only는 좌상단 작은 고정 badge, hardware pending은 결과 슬라이드 하단 footer.
- pp와 %를 혼용하지 않는다: rate는 `%`, 차이는 `pp`.
- 그래프 y축은 필요 없이 0부터 시작하지 않아도 되지만 범위를 명시하고 변화를 과장하지 않는다.
- 모든 OOD point는 다른 marker와 `OOD` label.
- FAIL/INCONCLUSIVE 슬라이드를 회색 처리하지 말고 연구 결론으로 동일한 시각적 무게를 준다.
- SVG를 raster screenshot으로 축소하지 말고 vector로 유지한다.
- 한글 폰트가 깨지면 Pretendard/Noto Sans KR/Apple SD Gothic Neo 중 포함 가능한 것으로 통일한다.

## 7. Claude가 사용해야 할 파일

최소 전달 묶음:

1. `docs/CLAUDE_PPT_MASTER_BRIEF_2026-08-26.md` — 최상위 제작 계약.
2. `docs/assets/motar-system-overview.svg` — high-level 구조.
3. `docs/assets/motar-control-stack.svg` — low-level 구조.
4. `docs/MOTAR_SYSTEM_SPEC_2026-08-24.md` — 정확한 파라미터.
5. `docs/PAPER_WRITING_BRIEF_2026-08-24.md` — 논문 claim boundary.
6. `docs/SIM2REAL_3DAY_EXECUTION_PLAN.md` — 하드웨어 다음 단계.
7. `docs/research_authority_2026-08-26.json` — 최종 실행 authority.
8. `VERIFICATION.md` — gate 결과.
9. `README.md` — 공개 요약.
10. `docs/PPT_PACKAGE_README_2026-08-27.md` — 이번 revision의 우선순위·금지사항·파일 SHA.
11. `results/navrl_ref_platform_verification_20260827_v2/summary.json` — 새 v2 airframe GPU envelope.
12. `docs/status/data/platform.json` — v1/v2 platform 구조화 사양.

원자료를 추가로 요구할 때만:

- `docs/status/status.json`.
- `results/navrl_ref5in_detection_range_stage1_s457/summary.json`.
- `results/navrl_physical_target_recovery_v2_gate_lower1p25_seed827/summary.json`.
- `results/navrl_physical_target_recovery_v2_no_connector_forensics_seed827/summary.json`.

## 8. Claude의 최종 산출물 체크리스트

- [ ] 본편 18장 + appendix 10–12장.
- [ ] 발표자 note에 슬라이드별 40–70초 설명과 원자료 경로.
- [ ] 모든 결과에 simulation-only/held-out/OOD/synthetic 표기.
- [ ] integrity와 performance/mechanism verdict 분리.
- [ ] expected hardware와 measured hardware를 색으로 분리.
- [ ] high-level과 low-level architecture를 별도 슬라이드로 설명.
- [ ] reward와 PPO/curriculum 정확 계수 포함.
- [ ] Track A/B 최종 판정 및 금지된 다음 실험 포함.
- [ ] limitation을 마지막 한 장에 몰지 말고 관련 결과 옆에도 표시.
- [ ] 참고문헌은 실제 DOI 확인 전 placeholder.
- [ ] export 후 모든 한글/수식/SVG/표가 16:9 화면 밖으로 잘리지 않는지 100% zoom QA.
- [ ] `pp`와 `%` 단위 일관성.
- [ ] “hardware validated”, “sim-to-real achieved”, “route recovery solved”, “Transformer is better” 금지.
- [ ] historical 결과 슬라이드에 `HISTORICAL navrl_band` watermark.
- [ ] corrected non-overlap 성능은 전부 `NOT RUN`; 임의 그래프/보간 금지.
- [ ] fresh training target `205`와 asset/evaluation ceiling `300`을 서로 바꾸지 않음.
- [ ] YOPOv2 대응 count-density는 `64/100`, 205는 2.05× context이며 난이도 우월성 주장이 아님.

## 9. 발표 직전 실행할 검증 명령

```bash
cd /home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator

# 최종 연구 authority와 원자료 SHA 확인
/home/fair/miniconda3/envs/aerialgym/bin/python tools/check_research_authority.py --json

# 사이트/그림/3D 계약
node tests/test_status_site.js
node tests/test_status_webgl_headless.js
/home/fair/miniconda3/envs/aerialgym/bin/python tests/test_status_snapshot.py

# JSON 문법
/home/fair/miniconda3/envs/aerialgym/bin/python -m json.tool \
  docs/research_authority_2026-08-26.json >/dev/null
```

이 검증이 PASS하더라도 실제 hardware claim은 열리지 않는다. 발표에서 가장 중요한 정직성 문장은
다음과 같다.

> **현재까지 확보한 것은 재현 가능한 simulation failure map이며, sim-to-real 성능은 다음 실제 측정의
> 대상이지 이미 달성한 결과가 아니다.**
