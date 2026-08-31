# MOTAR PPT v5 재제작 지시서 — 독립 검수 반영본

작성: 2026-08-30
검수 대상: `MOTAR_deck (4).pptx` 26장 + 발표자 노트 26장
검수 저장소 HEAD: `4113832`
원 검증 요청: `MOTAR_검증요청.md`의 slide 23 C1–C6

이 문서는 기존 deck의 디자인 취향을 평가하는 문서가 아니라, **Claude가 다음 deck을 다시 만들 때
틀리면 안 되는 사실·계보·표현·그림 구조를 고정하는 제작 계약**이다. 기존 PPT의 문장을 그대로
복사한 뒤 일부 숫자만 고치는 방식은 금지한다. 아래 `REBUILD` 슬라이드는 구조부터 다시 그린다.

---

## 0. 최종 판정

현재 deck은 export/render 자체는 정상이나 발표용으로는 **조건부 FAIL**이다.

필수 재작성:

- slide 6–7: YOPOv2 inference input/privileged training 정보가 뒤섞임.
- slide 10: `센서 기반 표적 상태` 열이 target이 없는 NavRL/MAVRL에도 `○`라 의미가 성립하지 않음.
- slide 11: PPO reward가 detector/tracker까지 공동학습하는 것처럼 읽힘.
- slide 12: training-only GT/reward/critic 관계와 actor 4-D z 채널의 의미가 불명확.
- slide 14: 898-D 분해가 틀렸고, live visibility reward를 후보로 잘못 표시.
- slide 16: historical generic v2와 fresh physical target pattern/ramp가 섞임.
- slide 21: YOPOv2의 actual/reference mismatch를 timestamp alignment 선행결과처럼 표현.
- slide 22: 160×90 software-clip 진단을 일반적인 광학 한계처럼 표현.
- slide 23: carve-out 인과를 직접 계측한 것처럼 단정.
- slide 24: 서로 다른 lineage의 결과가 badge 없이 한 표에 혼합.
- slide 25: 현재 research authority보다 E1/E2를 먼저 실행하는 것처럼 서술.

추가로 slide 3의 분류 편수는 표와 참고문헌이 불일치한다. 현재 참고문헌의 실제 표식은
`S=6, A=10, B=3, F=1`인데 slide 3에는 `6/9/4/1`로 적혀 있다.

---

## 1. Claude가 따라야 할 데이터 authority

1. 결과 숫자는 이 문서에 적힌 값과 첨부된 원자료만 사용한다.
2. historical `navrl_band` 결과와 corrected non-overlap fresh 계보를 한 곡선/한 주장으로 잇지 않는다.
3. `205 bars`는 fresh training target이자 frozen geometry PASS cell이다.
4. `300 bars`는 asset/evaluation ceiling이며 frozen geometry FAIL cell이다. 300-bar PPO 성능은 없다.
5. corrected non-overlap fresh PPO capture/crash/timeout은 `NOT RUN`이다.
6. 실제 기체는 미조립이고 실제 센서 로그는 없다. 공식 상태는 `SYNTHETIC_ONLY`다.
7. current authority check:
   - Track A: `RANGE_INCONCLUSIVE_AT_THIS_BUDGET`
   - Track A stage 2: not authorised
   - Track B: `FAIL_ROUTE_MECHANISM`
   - Track B long training: not authorised
8. frozen geometry receipt는 `205: 99.167% PASS`, `300: 94.661% FAIL`을 유지한다.
   이후 footprint-aware spawn/goal 수정의 scratch 값으로 이 숫자를 교체하지 않는다.

공통 badge:

- `HISTORICAL · OVERLAP-PERMITTING`: 주황색 얇은 띠.
- `CORRECTED NON-OVERLAP · PPO NOT RUN`: 청색 띠.
- `SYNTHETIC DIAGNOSTIC`: 회색 띠.
- `HARDWARE-INFORMED CANDIDATE · NOT BUILT`: 회색 점선 watermark.
- `FAIL / BLOCKED`: 적색.
- `PASS WITHIN STATED SCOPE`: 청록색. 녹색은 실제 하드웨어 측정 전까지 사용하지 않는다.

---

## 2. 권장 deck 구조

현재 26장 본편은 텍스트가 너무 작다. 본편 18장 + appendix 8–10장으로 재구성한다.

### 본편 18장

1. Title / research question
2. Why interception in clutter
3. Related-work gap — tracking vs interception
4. System boundary — what is sensed, learned, fixed
5. Correct YOPOv2 comparison
6. 898-D → 17-token representation
7. Learned high-level policy + fixed low-level controller
8. Hardware-informed candidate / unmeasured hardware
9. Environment and lineage split
10. Reward contract and justification
11. Experiment matrix and authority
12. Historical density × speed result
13. Initial non-acquisition failure result
14. Timestamp alignment mechanism
15. Latency/pose premise results
16. Evidence ledger — pass, fail, inconclusive
17. Limits and currently authorised next work
18. Conclusion

### Appendix

- literature selection and 20 references
- full comparison matrix
- full PPO/curriculum table
- full reward equation
- observation field schema
- platform/URDF parameters
- crash taxonomy
- detailed route/recovery receipts
- camera range diagnostics
- C1–C6 latency verification table

발표 본문은 title 30–34 pt, body 17–20 pt, table 14–16 pt, footnote 11–12 pt를 하한으로 한다.
8–11 pt 본문은 금지한다. 한 장에 표와 장문 해석과 각주를 동시에 넣지 않는다.

---

## 3. 슬라이드별 교정 지시

### Slide 1 — KEEP, 단 범위 표시

- 날짜는 새 export 날짜로 바꾼다.
- `camera 87° / 20 m`는 simulation contract임을 작은 `model` badge로 표시한다.
- `실제로 접촉까지`는 실제 하드웨어 접촉을 수행했다는 뜻이 아니라 task termination 정의임을
  발표자 note에서 명시한다.

### Slide 2 — KEEP

- 적용 예시는 동기 부여이지 검증된 deployment가 아니다.
- `외부 측위를 기대할 수 없다`보다 `actor가 외부 GT target state를 받지 않는다`가 코드 계약에 더
  정확하다. 실제 odometry 자체를 사용하지 않는다는 뜻으로 읽히지 않게 한다.

### Slide 3 — FIX COUNT

현재 reference category count의 올바른 합:

| S | A | B | F | total |
|---:|---:|---:|---:|---:|
| 6 | 10 | 3 | 1 | 20 |

`6 / 9 / 4 / 1`을 사용하지 않는다. 분류 자체가 주관적 screening tier임을 유지한다.

### Slide 4–5 — KEEP WITH BADGES

- UAST는 CVPR 2026 peer-reviewed.
- YOPOv2-Tracker는 arXiv 2025 frontier comparator.
- `tracking/following`과 `physical capture termination`의 구분은 유지한다.

### Slide 6 — REBUILD: YOPOv2 실제 구조

현재 잘못된 점:

- ESDF map과 target pose를 inference input으로 그림.
- target pose가 네트워크에 직접 주어지는 것처럼 그림.

정확한 구조:

```text
INFERENCE
RGB-D 160×96 + 6-D state(initial velocity, acceleration)
  → modified ResNet-18 / 5×3 primitive grid
  → per-primitive 14-D prediction
     [offsets 3, end velocity 3, end acceleration 3,
      trajectory cost 1, objectness 1, target image/depth position 3]
  → objectness/NMS + EKF consistency
  → select primitive and solve trajectory
  → disturbance compensation
  → desired attitude + thrust

TRAINING ONLY
GT point cloud / ESDF + GT target state
  → smoothness/safety/tracking costs and labels
  → cost gradients back-propagated into the network
```

ESDF와 GT target은 반드시 `TRAINING ONLY` 점선 영역에 둔다.

### Slide 7 — REBUILD: apples-to-apples comparison

YOPOv2에 `target pose input`이라고 쓰지 않는다.

| 축 | YOPOv2-Tracker | MOTAR |
|---|---|---|
| inference perception | raw RGB-D + velocity/acceleration state | camera-derived target track + LiDAR geometry + ego history |
| privileged training | ESDF/point cloud + GT target labels/cost | GT target/vehicle state → reward, termination, asymmetric critic |
| learned output | primitive refinement/cost/objectness/target estimate | bounded body velocity/yaw-rate policy |
| temporal mechanism | EKF target-state continuity | 5-sample structured history + tracker |
| training signal | differentiable trajectory cost gradients | PPO reward/advantage |
| endpoint | persistent tracking/navigation success | 0.5 m capture termination |

성능 우열 표가 아니라 구조·목표·학습신호 차이임을 제목에 표시한다.

### Slide 8–9 — KEEP, 표현 범위 제한

- `저속/밀집`과 `고속/성김`은 conceptual regime split이지 이 deck의 직접 A/B 결과가 아니다.
- `latency가 중요하지 않다`는 표현은 계속 금지한다.

### Slide 10 — REBUILD: literature matrix semantics

현재 `센서 기반 표적 상태` 열은 의미가 불명확하다. NavRL과 MAVRL은 moving target task가 없으므로
`○`가 될 수 없다.

열을 다음처럼 바꾼다:

```text
moving target | cluttered obstacles | single UAV | actor target source
capture endpoint | learned decision | real-flight validation
```

`actor target source`는 `onboard / prior / N/A` 세 값으로 쓴다.

- NavRL, MAVRL: `N/A` — predefined goal navigation/obstacle avoidance이지 target tracker가 아님.
- YOPOv2, D-VAT, UAST: onboard visual observation.
- Pliska: onboard tracking/state estimation, physical interception.

표 하단 주장은 다음 범위로 제한한다:

> 이 표의 비교군 안에서는 cluttered moving-target task, learned decision, physical capture endpoint를
> 동시에 평가한 선행행을 확인하지 못했다. 이는 전체 문헌에 대한 부재 증명이 아니다.

### Slide 11 — FIX: reward가 학습하는 범위

현재 `하나의 reward로 공동 최적화`가 detector/tracker/low-level까지 PPO가 같이 학습하는 것으로
읽힌다. 다음으로 교체한다:

> Detector, tracker, state estimator, Lee controller는 고정한다. PPO reward는 policy 내부의
> search / avoid / approach / capture 행동 절충만 공동 최적화한다.

`RL이 modular보다 우월`은 E1 전까지 가설로 유지한다.

### Slide 12 — REBUILD: system boundary figure

아래 §4 Figure A를 사용한다. 핵심 수정:

- GT state는 reward/termination과 privileged critic으로 **분기**한다. 순차 파이프라인처럼 그리지 않는다.
- actor는 GT를 받지 않는다.
- actor 4-D 중 z action은 actuator command가 아니라 checkpoint-compatible previous-action state channel이다.
  실제 수직 명령은 altitude PI가 덮어쓴다.
- policy 10 Hz와 controller/physics 100 Hz를 분리 표시한다.
- safety governor는 canonical에서 `off`, 별도 evaluation-only 블록으로 빼낸다.

### Slide 13 — KEEP, detector variant를 분리

`Detector`를 하나의 확정 실물 detector처럼 그리지 않는다.

- analytic detector: simulation diagnostic.
- learned-v2 detector: offline trained candidate/evaluation result.
- real detector/log: 없음.

RGB-D raw tensor가 actor로 직접 들어가지 않고 camera 측정에서 만든 compact target track만 들어간다고
한 문장으로 명시한다. LiDAR는 obstacle geometry 역할이다.

### Slide 14 — REBUILD: observation + reward

현재 `target state 4`와 입력 합산은 틀렸다.

정확한 898-D:

| block | shape | dim | Transformer token |
|---|---:|---:|---:|
| current static LiDAR | 4×72 | 288 | 1 |
| obstacle history | 5×8×12 | 480 | 5 |
| robot history | 5×10 | 50 | 5 |
| target history | 5×16 | 80 | 5 |
| CLS | – | – | 1 |
| total | | 898 | 17×64-D |

5 samples at 0.5 s spacing span approximately 2.0 s.

현재 live reward를 후보와 섞지 않는다:

```text
r_t = 1.0 r_range_rate
    + 1.0 r_ego_progress(*)
    + 1.5 r_static
    + 0.02 I_visible
    - 0.05
    - 0.1 ||Δv||
    - 8.0 p_height
    - 0.3 p_yaw_align
    - 0.02 ψ_cmd²
```

- capture: shaped reward에 +30, 0.5 m에서 종료.
- collision: reward를 −20으로 overwrite하고 종료.
- timeout bonus 없음.
- `(*)` moving-target progress는 target(t+1)에 재고정한 heuristic이며 formal policy-invariant PBRS로
  부르지 않는다.
- visibility `+0.02/step`는 **현재 live reward**다. `후보/on-off 후 결정`이라고 쓰지 않는다.

### Slide 15 — FIX: exact platform lineage

`ref5in candidate` 대신:

> `navrl_ref5in_v2_quad` — fresh-only hardware-informed simulation candidate, not built

표에 포함:

- nominal mass 1.20 kg.
- motor diagonal 220 mm.
- collision proxy 0.283×0.283×0.12 m.
- nominal T/W 3.262.
- motor time constant 0.04 s.
- max thrust 9.60 N/motor.
- values are synthetic/model design points, not identified hardware.

현재 미측정: actual AUW, CG/inertia, thrust curve, ESC/motor response, camera intrinsics/extrinsics,
sensor-to-command latency, timestamp synchronization, power/thermal/endurance.

### Slide 16 — REBUILD: environment contract by lineage

한 표에 pattern/ramp를 하나만 쓰지 말고 다음 두 열로 분리한다.

| item | historical evaluated v2 | corrected fresh physical intent |
|---|---|---|
| placement | overlap-permitting `navrl_band` | footprint-aware non-overlap, 0.45 m surface clearance |
| target dynamics | historical receipt별 legacy/bounded | physical 6-DoF |
| target pattern | mixed CV/waypoint | base fresh(route off): mixed; routed fresh: waypoint-only |
| speed ramp | training: `U[0.3, v_max]`, `v_max` 0.3→1.5 m/s over epoch 0–300; held-out evaluation: exact fixed speed, ramp N/A | 1 epoch |
| density results | 130–220 held-out map exists | PPO NOT RUN |
| density target | historical policy lineage | training 70→205, +15; 300 evaluation/asset ceiling |
| route | historical condition | routed candidate `global_astar_v1`, mechanism FAIL, long training blocked |

공통 arena는 40×40×3 m, bars_h3 높이 3 m, side 0.4–0.8 m, goal distance 6–28 m,
episode 600 actions/60 s, policy 10 Hz, physics 100 Hz다.

Historical ramp provenance는 ep25000 체크포인트의 `env_state`에 기록된
`cfg_target_speed_min=0.3`, `cfg_target_speed_final=1.5`,
`cfg_target_speed_ramp_start_epochs=0`, `cfg_target_speed_ramp_epochs=300`이다.
Slide 19의 0.3/0.7/1.1/1.5 m/s 셀은 평가 시 속도를 각각 고정했으므로, 300-epoch ramp를
평가 조건으로 설명하면 안 된다.

### Slide 17–18 — KEEP AS DESIGN, authority badge 추가

- E1/E2는 아직 연구 설계이며 실행 권한이 열린 현재 작업으로 표현하지 않는다.
- 결과 칸은 계속 비워 둔다.
- E1 전에 `RL is better` 금지.

### Slide 19 — KEEP

수치 재검증 완료. 다음 라벨 유지:

- `HISTORICAL · OVERLAP-PERMITTING · HELD-OUT SIMULATION`.
- 220 bars는 OOD.
- density cost −11.4 pp, speed cost −2.7 pp.
- interaction은 확인되지 않음(`p=.337/.817`).
- `density가 높을수록 speed penalty가 커진다` 금지.

### Slide 20 — FIX TITLE SCOPE

제목을 다음으로 바꾼다:

> 이 historical diagnostic에서는 밀도보다 초기 미관측이 지배적이었다

seed 359 / away-heading cohort 범위를 제목 아래에 둔다. 전체 정책·전체 환경의 보편적 원인으로
확장하지 않는다.

### Slide 21 — FIX YOPOv2 ANALOGY

YOPOv2 Fig. 7은 actual position과 reference position의 controller tracking error다. delayed detection
timestamp/capture-time pose를 직접 실험한 것이 아니다.

교체 문구:

> YOPOv2가 다룬 actual–reference state inconsistency와 구조적으로 유사하지만, 본 연구의 delayed
> detection timestamp alignment와 동일한 문제나 선행 실험은 아니다.

### Slide 22 — FIX CAMERA RANGE LINEAGE

우측 패널 제목:

> Historical 160×90 software-clip diagnostic — superseded as a hardware proposal

- 20→28 m에서 timeout −37.65 pp는 그 historical contract의 결과.
- 0.90 px / 2 px² gate 0%도 그 160×90 detector model의 계산.
- 이를 모든 28 m camera가 물리적으로 불가능하다는 주장으로 일반화하지 않는다.
- later high-resolution synthetic Stage 1은 never-acquired 8.443→3.172%였지만 primary −15 pp gate를
  통과하지 못해 `INCONCLUSIVE`다.

### Slide 23 — FIX CAUSAL WORDING

C1–C4, C6은 PASS. C5만 관측과 해석을 분리한다.

사용 가능한 수치:

- clean 80.54%.
- aligned 0.1 s 78.04% (−2.50 pp).
- naive 0.1 s 37.82% (−42.71 pp).
- exact 79.06% campaign 기준: −0.05 s −2.82 pp, +0.05 s −17.28 pp,
  +0.10 s −39.77 pp.
- isolated-RNG seed181: position 0.10 m −1.47 pp CI[−4.02,+1.07], yaw 2° −3.28 pp
  CI[−5.86,−0.70], yaw 5° −12.75 pp CI[−15.47,−10.03].
- crash: clock −0.05 s 21.27%, +0.05 s 35.48%.

금지 문장:

> 늦은 자세가 실제 막대를 지도에서 지웠다.

교체 문장:

> 자세 offset의 부호에 따라 crash가 21.27%에서 35.48%로 증가했다. 코드에는 stale target
> estimate가 obstacle-map carve-out을 구동하는 경로가 있어 이 해석과 일치하지만, 삭제된 실제
> bar 수를 직접 계측하지 않아 인과는 아직 확정하지 않았다.

`+20 ms`, `yaw≤1°`, `position≤10 cm`를 사전등록 gate로 부르지 않는다.

### Slide 24 — REBUILD: evidence ledger with per-card lineage

현재 카드마다 다음 계보를 붙인다.

- 600 exact actions: historical v2 semantic engineering validation.
- 333/333 endpoint oracle: historical selected-contact endpoint oracle; global reachability 아님.
- 99.167% / 94.661%: corrected non-overlap frozen geometry receipt.
- learned vs analytic −0.0145 pp: synthetic detector evaluation.
- 8.443→3.172%: high-resolution synthetic camera-range diagnostic, primary gate missed.
- route/recovery cells: historical/evaluation-only physical-route lineage; mechanism FAIL.

`corrected-v2 semantics`라는 이름은 `corrected fresh non-overlap`과 충돌하므로
`historical v2 semantic engineering fix`로 바꾼다.

### Slide 25 — REBUILD: current authority-aligned next steps

현재 authority가 허용하는 다음 순서:

1. exact hardware BOM and geometry.
2. sensor calibration and timestamp contract.
3. 210 independent sensor trials.
4. real-log profile and offline replay.

E1 modular baseline과 E2 perception ablation은 중요 연구 backlog지만, 현재 실행 authority의
즉시 다음 단계처럼 날짜를 확정하지 않는다. 실제 hardware/BOM/log가 없으므로 `1–2일`, `2–4일`을
확정 일정으로 쓰지 않는다.

### Slide 26 — FIX CATEGORY COUNT, bibliography 유지

- category count는 `S6/A10/B3/F1`.
- UAST: CVPR 2026, pp. 13464–13473.
- YOPOv2-Tracker: arXiv:2505.06923, 2025 frontier.
- NavRL: RA-L 2025로 표기 가능하지만 arXiv 최초 공개는 2024임을 bibliography metadata에서 구분.
- Pliska DOI는 `10.1109/LRA.2024.3451768`.
- 논문 제목 축약은 본편에서 허용하되 appendix bibliography에는 정식 제목·저자·venue·year·DOI/URL을 둔다.

---

## 4. 반드시 새로 그릴 figure 설계

### Figure A — End-to-end system boundary and data authority

목적: 교수님이 `무엇이 학습되고, 무엇이 센서이며, GT가 어디에만 쓰이는가`를 한 번에 이해.

```text
DEPLOYMENT / ACTOR PATH (solid blue)
RGB-D → detector/track/depth → target feature history ┐
LiDAR 4×72 → scan + obstacle proposals/history       ├→ 898-D → 17×64 tokens
ego state + previous action/history                   ┘        → Transformer actor 10 Hz
                                                               → vx, vy, z_compat, yaw-rate
                                                               → z_compat discarded for actuation
                                                               → altitude PI + Lee velocity control 100 Hz
                                                               → motor allocator → 4 motors → rigid body

TRAINING ONLY (red dashed)
GT target/vehicle state ─┬→ reward + termination → returns/advantages ┐
                         └→ privileged critic observations            ├→ PPO update
actor observations contain NO GT target pose/velocity/visibility      ┘

EVALUATION ONLY (gray dotted)
riskcap / forensics / oracle / metrics — not learned, canonical governor off
```

그림 규칙:

- actor path는 실선 청색.
- training-only는 붉은 점선 상단 띠.
- evaluation-only는 회색 점선 옆가지.
- `10 Hz`와 `100 Hz` 사이에 sample/hold 아이콘.
- z action에 `compatibility state channel; overwritten by altitude PI` 라벨.

### Figure B — 898-D packing to 17 tokens

두 단계로 그린다.

1. 왼쪽: raw structured blocks와 정확 shape.
2. 오른쪽: projection 후 token sequence.

```text
static 288 ─────────────────────────────→ static token 1
obstacle [t0..t4] each 8×12 ─MLP each──→ obstacle tokens 5
robot    [t0..t4] each 10   ─MLP each──→ robot tokens 5
target   [t0..t4] each 16   ─MLP each──→ target tokens 5
                                         + learned CLS 1
                                         = 17 tokens × 64-D
                                         → 4-layer / 4-head Transformer
```

`8 obstacle slots = 8 Transformer tokens`처럼 그리지 않는다. 각 시점의 8×12가 MLP로 하나의
obstacle-history token이 된다.

### Figure C — Timestamp alignment, two timelines

상단 `correct`:

```text
t−Δ: image acquired + pose buffered
      → target ray lifted with pose(t−Δ)
      → tracker predicts state forward to t
```

하단 `naive/superseded`:

```text
t−Δ image arrives at t
      → ray lifted with pose(t)
      → ego translation/yaw is falsely assigned to target motion
```

오른쪽에는 수치만:

- aligned −2.50 pp.
- naive −42.71 pp.
- clock asymmetry −2.82 vs −17.28 pp.

carve-out은 `hypothesized code-consistent channel; not directly instrumented`로 점선 표시한다.

### Figure D — Evidence lineage rail

가로 3개 rail:

```text
Historical overlap-permitting ── held-out density/speed, latency, detector results
Corrected non-overlap fresh  ─── geometry PASS205 / FAIL300, PPO NOT RUN
Hardware track              ──── candidate model only → BOM/calibration/logs pending
```

모든 결과 카드를 하나의 rail에만 붙인다. rail 사이 화살표로 성능을 이어 붙이지 않는다.

### Figure E — Reward contract

저울/파이프라인 그림보다 세 그룹 카드가 안전하다.

- Dense: range-rate +1, ego-progress +1*, static safety +1.5, visible +0.02.
- Regularization/cost: time −0.05, smooth −0.1, height −8, yaw align −0.3, yaw-rate² −0.02.
- Terminal: capture +30 add, collision −20 overwrite, timeout bonus none.

별표: moving-target progress is heuristic, not policy-invariant PBRS.

### Figure F — High-level / low-level control timing

```text
Transformer actor (10 Hz)
  → normalized 4-D action
  → body vx/vy ±2.5 m/s per axis
  → yaw-rate ±3.0 rad/s
  → z action retained only in prev_action history

altitude PI (z*=1.0 m, vertical cap 2.5 m/s)
  + body velocity/yaw commands
  → Lee controller, tilt≤45° (100 Hz)
  → force/attitude/rate torque
  → 4-motor allocation
  → motor lag τ=0.04 s
  → 100 Hz rigid-body physics
```

canonical safety governor는 이 그림의 주경로에 넣지 말고 `off / evaluation-only` 옆가지로 둔다.

---

## 5. 발표자 note 작성 계약

각 슬라이드 note는 다음 네 줄 구조로 쓴다.

1. 이 장의 한 문장 결론.
2. 숫자의 조건/lineage.
3. 이 숫자로 말할 수 없는 것.
4. 원자료 파일 경로 또는 논문 URL.

slide 23에서 말할 안전한 문장:

> 시뮬레이션에서는 측정 시점 자세를 사용하는 변환이 기본값으로 고정돼 있고, 동일한 0.1초 지연의
> capture 비용이 naive −42.71 pp에서 aligned −2.50 pp로 바뀌었습니다. 다만 이 값은 정확한 검출
> timestamp와 capture-time pose를 전제로 합니다. clock offset은 부호 비대칭이었고 crash도 함께
> 증가했습니다. 코드상 carve-out 경로와 일치하지만 삭제된 실제 bar 수를 직접 계측하지 않았으므로
> 그 인과는 아직 가설입니다.

---

## 6. 시각 QA 계약

- 16:9, white background, 2개 accent 색 이하.
- title 30–34 pt, body 17–20 pt, table 14–16 pt, footnote 11–12 pt.
- 한 슬라이드 최대 핵심 주장 1개.
- 모든 `pp`는 차이, `%`는 절대 비율로 통일.
- `PASS`, `FAIL`, `INCONCLUSIVE`, `NOT RUN`을 색뿐 아니라 텍스트로도 표기.
- red는 fail/block에만 사용.
- historical 결과마다 주황 watermark.
- SVG/수식/한글은 PowerPoint export와 PDF export 모두 확인.
- 100% zoom에서 잘림, 겹침, 2줄 표 제목, 흐린 회색 글씨를 전수 검사.

---

## 7. 금지 주장

- Transformer가 RNN보다 보편적으로 우월하다.
- RL이 modular baseline보다 우월하다.
- sim-to-real을 달성했다 / hardware validated.
- 300 bars에서 policy performance를 검증했다.
- corrected non-overlap fresh PPO가 205 bars를 mastery했다.
- YOPOv2가 target pose/ESDF를 inference input으로 받는다.
- late pose가 실제 막대를 지웠다는 인과를 직접 증명했다.
- +20 ms, yaw≤1°, position≤10 cm가 preregistered tolerance gate다.
- 28 m target은 어떤 camera에서도 검출 불가능하다.
- historical overlap-permitting curve를 corrected fresh 성능으로 재사용한다.

---

## 8. 이번 독립 검수에서 실행한 검사

- PPT 26장 LibreOffice PDF export: 성공.
- 26장 PNG render/contact sheet visual QA: clipping/font corruption 없음.
- research authority: verified.
- Python unit tests: 851 OK, skipped 2.
- site contract/headless WebGL/snapshot: PASS.
- slide 23 C1–C4/C6: PASS.
- C5: numeric/code channel PASS, direct causal instrumentation 없음.
- central literature source audit:
  - YOPOv2 official arXiv method/input/training privilege 확인.
  - UAST CVPR 2026 official CVF entry 확인.
  - NavRL/MAVRL target-free navigation scope 확인.
  - Pliska RA-L 2024 physical interception and DOI 확인.
  - Fast-Tracker/Elastic/Intention-Aware/D-VAT venue and task scope 확인.

이 테스트 통과는 software consistency만 뜻하며 실기 성능을 열지 않는다.

---

## 9. Claude가 사용할 원자료

### 첨부된 저장소 문서

- `MOTAR_SYSTEM_SPEC_2026-08-24.md`: current observation/action/control/reward contract.
- `CLAUDE_PPT_MASTER_BRIEF_2026-08-26.md`: deck scope, result ledger, forbidden claims.
- `PPT_PACKAGE_README_2026-08-27.md`: historical/corrected lineage rules.
- `MOTAR_검증요청.md`: slide 23 C1–C6 exact verification request.

### primary literature links

- YOPOv2-Tracker: <https://arxiv.org/html/2505.06923>
- UAST official CVF: <https://openaccess.thecvf.com/content/CVPR2026/html/Qin_UAST_Unified_Active_Search_and_Tracking_for_Arbitrary_Targets_with_CVPR_2026_paper.html>
- NavRL: <https://arxiv.org/abs/2409.15634>
- MAVRL: <https://doi.org/10.1109/LRA.2024.3522778>
- Pliska interception: <https://arxiv.org/abs/2405.13542>, DOI `10.1109/LRA.2024.3451768`
- D-VAT: <https://arxiv.org/abs/2308.16874>, DOI `10.1109/LRA.2024.3385700`
- Fast-Tracker: <https://arxiv.org/abs/2011.03968>, DOI `10.1109/ICRA48506.2021.9561948`
- Elastic Tracker: <https://arxiv.org/abs/2109.07111>
- Intention-Aware Planner: <https://arxiv.org/abs/2309.08854>
- Resource-Efficient RGBD Aerial Tracking official CVF:
  <https://openaccess.thecvf.com/content/CVPR2023/html/Yang_Resource-Efficient_RGBD_Aerial_Tracking_CVPR_2023_paper.html>

YOPOv2 그림을 재작성할 때는 원문 Fig. 3과 §III-C/III-E를 기준으로 하고, Fig. 7은 timestamp
alignment의 직접 근거가 아니라 actual/reference mismatch의 analogy로만 사용한다.
