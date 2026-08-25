# VERIFICATION — ref5in 검증 단계 (실행 authority)

검증 gate·판정·다음 실험은 **이 문서**가 규정한다. 연구 charter(가설·방법)는
[`RESEARCH_PLAN.md`](RESEARCH_PLAN.md), 날짜별 기록은 [`WORKLOG.md`](WORKLOG.md),
명령어는 [`OPERATIONS.md`](OPERATIONS.md), 라이브 지표는 [`docs/status/`](docs/status/)를 본다.

> 기준일: 2026-08-23

## 후보 R0 — global-routed physical target (기존 P0–P3와 독립)

새 fresh-only model id는 `physx_ref5in_6dof_global_astar_aabb_v1`이다. 기존 physical/legacy
checkpoint에 소급 적용하지 않는다. actual AABB, all-orientation support, same-component goal,
fail-closed A*의 CPU unit/launcher/latency gate는 **PASS**했다. 70/150/205/300 bars에서 순차
평균은 46.18/48.71/58.34/64.01 ms/env, 128-env 직렬 보수 투영은 최대 8.19 s였다.

판정은 `PASS_CPU_ENGINEERING_GATE / SIMULATOR_UNMEASURED`다. 다음 권한은 fresh short PhysX
smoke뿐이며 PPO 본학습은 금지한다. 300 bars arena-wide connectivity 주장은 금지한다. 근거:
[`preregistration`](docs/preregistration_physical_target_global_route_2026-08-25.md),
[`CPU benchmark`](results/navrl_target_route_cpu_benchmark_seed825/summary.md).

## 한 줄 상태

`navrl_ref5in_quad`는 **hardware-informed simulation candidate**다. P0·P1c는 PASS, **P2·D1은 FAIL**,
**P3 장기학습은 차단**. seed 367은 초기 미관측의 인과 기여를 지지했지만 해결책 채택은 아니다.
정직한 고해상도 검출 조건의 Stage 1(각 1,000 epoch, 2,049 ep)은 never-acquired
`8.443→3.172%`(**−5.271 pp**)로 사전 `−15 pp` gate를 못 넘어
`RANGE_INCONCLUSIVE_AT_THIS_BUDGET`; **Stage 2 권한 없음**. 다음 authority는
[`docs/SIM2REAL_3DAY_EXECUTION_PLAN.md`](docs/SIM2REAL_3DAY_EXECUTION_PLAN.md)다.

## 지금 막혀 있는 것

| gate | 판정 | 핵심 수치 |
|---|---|---|
| P2 held-out (seed 313) | **STRICT FAIL** | 68.28 / 26.16 / **5.56%** timeout (허용 102건, 실제 114) |
| D1 adaptation (seed 331) | **FAIL** | q3/CV timeout **15.98%** > 12% |
| P3 full-budget | **BLOCKED** | P2 PASS 전까지 실행 금지 |

어느 진단 결과도 P2/D1 FAIL을 소급 변경하거나 P3를 자동 해제하지 **않는다**.

## 완료 gate 요약

| 단계 | 결과 | canonical 결과 |
|---|---|---|
| P0 repository + simulator | PASS | [`results/navrl_ref_platform_verification/`](results/navrl_ref_platform_verification/summary.md) |
| P1c fresh 900 epoch smoke | PASS | [`results/navrl_ref5in_smoke_seed197/p1c/`](results/navrl_ref5in_smoke_seed197/p1c/summary.md) |
| P1a / P1b | FAIL | [`p1a/`](results/navrl_ref5in_smoke_seed197/summary.md), [`p1b/`](results/navrl_ref5in_smoke_seed197/p1b/summary.md) |
| P2 | STRICT FAIL | [`results/navrl_ref5in_p2_seed313/`](results/navrl_ref5in_p2_seed313/summary.md) |
| D0 outcome strata (seed 317) | descriptive | [`results/navrl_ref5in_outcome_diagnostic_v2_seed317/`](results/navrl_ref5in_outcome_diagnostic_v2_seed317/summary.md) |
| D1 probe (seed 331) | FAIL | [`results/navrl_ref5in_d1_eval_seed331/`](results/navrl_ref5in_d1_eval_seed331/summary.md) |

### P2 실패가 의미하는 것

경계 실패다. timeout Wilson 95% CI는 4.65–6.64%로 5%를 포함한다. capture는 통과, crash도 통과.
**같은 seed로 P2 재시도하지 않는다.**

### D0/D1이 좁힌 병목

- 표적 속도만의 병목: **기각** (빠를수록 timeout 감소).
- 장거리(22.5–28 m): crash↑ + CV-specific timeout↑ (waypoint timeout은 상대적으로 낮음).
- radial heading(away vs toward): timeout +23 pp — dense obstacle **불필요** (1-bar에서도 +54 pp).
- seed 359 first-acquisition: away timeout의 **87.52%**가 never-acquired (capture cohort 0.00%).
- fused == camera 최초취득 (6 cohort 전부) → LiDAR 12 m보다 camera 20 m가 먼저 지배.

### 관측 계약 (코드 감사)

hard-distance 평가 `[22.5, 28] m`는 target camera max **20 m**, LiDAR **12 m**를 넘는다.
모든 episode는 target token 0으로 시작하며, 초기 행동은 spawn 방향 prior에 의존한다.

## 완료: camera range 인과 대조 (§8.29)

동결 checkpoint: D1 terminal epoch 1900 (SHA `197ea269…a278e`).

| arm | 개입 | seed | 상태 |
|---|---|---|---|
| A | camera range **20 m** (대조) | 367 | 원자료 [`camera_20m/`](results/navrl_ref5in_camera_range_control_seed367/cells/camera_20m/) |
| B | camera range **28 m** (개입 1값만) | 367 | 원자료 [`camera_28m/`](results/navrl_ref5in_camera_range_control_seed367/cells/camera_28m/) |

조건: 1 bar, CV-only, away heading, goal `[22.5,28] m`, deterministic, governor off, 2,049 ep/cell.

**Primary gate:** B timeout ≤ A timeout − **20 pp**.

**Secondary:** crash +10 pp 이상이면 timeout 감소만으로 성공 주장 금지.

**해석 한계:** 정책은 20 m로 학습됐다. timeout이 안 줄어도 “비관측이 원인 아님”으로 읽을 수 없다.

### seed 367 결과 (동결, `summary.json`)

| arm | camera range | capture | crash | timeout | pooled never-acq |
|---|---:|---:|---:|---:|---:|
| A | 20 m | 36.39% | 7.80% | **55.80%** | 57.22% |
| B | 28 m | 74.96% | 6.88% | **18.16%** | 20.30% |
| Δ (B−A) | — | +38.57 pp | −0.92 pp | **−37.65 pp** | −36.92 pp |

**Primary gate 통과** (임계 20 pp). **Guard 깨끗** — crash가 오르기는커녕 0.92 pp 내렸으므로
실패모드 이동이 아니다. 판정 `initial_unobservability_dominant_cause_supported`.

조작이 인지 계층에 실제로 먹혔는지는 receipt가 아니라 **행동**으로 확인했다: pooled
never-acquired `57.22% → 20.30%`. receipt는 env var를 요청했다는 것만 증명하므로 이 확인이
필요하며, 임계 없는 **방향 검정**으로 두어 사후 조정 여지를 없앴다.

**독립 재검증:** `verify` PASS, raw 재계산 불일치 0건, 두 gate 독립 재도출 일치. 두 셀의
`condition` 46키 중 다른 것은 `evaluation_nonce` 하나뿐이고, `v2_evaluation_contract` 93키와
receipt 63키까지 넓혀도 차이는 `target_camera_max_range_m` **하나뿐**이다 — 교란 없는 단일 변수
대조임이 확인됐다. 두 셀이 동일 source bundle(314 파일)을 공유하고 runtime git status는 clean이다.

**메커니즘:** capture arm의 first-visible 중앙값이 `313 → 28 step`으로 붕괴한다 — 28 m 카메라는
22.5–28 m 시작 거리에서 사실상 spawn부터 표적을 본다. 반면 timeout arm의 first-visible 평균은
`561.66 → 548.86`으로 거의 그대로다. 효과는 "timeout이 더 일찍 취득한다"가 아니라 **"episode가
timeout에서 빠져나간다"**는 형태다.

**말할 수 있는 것:** 이 조건에서 초기 표적 미관측이 timeout의 지배적 원인이다.

**말할 수 없는 것:**
1. camera range 확장이 **해결책**이라는 것 — 진단이지 채택이 아니다. 28 m 검출은 학습 분포
   밖이라 정책이 그것을 잘 활용한다는 보장이 없다(capture 75%는 부수 관측이지 gate가 아니다).
2. 실기 함의 — 20 m는 시뮬 파라미터이지 특정 센서 사양이 아니다.
3. P2/D1 재판정이나 P3 해제 — 전부 그대로다.

### 2026-08-22 정정 — 그 선택지 둘 다 전제가 틀렸다

이전 판(2026-08-21)은 다음을 **(a) 과제를 센서에 맞춘다** / **(b) 센서를 과제에 맞춘다(장거리
검출 전제로 재학습)** 의 선택으로 적었다. camera-first 1단계가 그 전제를 무너뜨렸다.

seed 367은 **광학을 바꾸지 않았다.** 두 arm의 소스 스냅샷이 md5 동일이고 소프트웨어 far-plane만
20→28 m로 풀렸다. 따라서 그 결과는 **정보의 가치**를 보인 것이지 28 m 검출 하드웨어의 실현성이
아니다. 그리고 시뮬레이터 자신의 기하로는 28 m 광축 검출이 **불가능**하다 — 160×90 @ 87°에서
fx 84.3 px/rad, 표적이 28 m에 0.90 px(0.62 px²)이고, 마스크가 서브픽셀 커버리지 없이 화소 중심
표본화라 광축 표적이 2 px² 임계를 20 m에서 24.6%, 28 m에서 **0%** 통과한다. 이 광학 사슬의
신뢰 검출 거리는 12–15 m다.

**교란 (한계로 기록, 판정 소급 변경 아님)**: `detector_max_range` 변경은 순수 clip 변경이 아니다.
actor 표적 토큰이 함께 재정규화된다(`navrl_perception.py:1574,1578`). 28 m arm은 20 m로 학습된
정책에 0.714배 스케일 위치를 먹였으므로 timeout −37.65 pp 중 일부는 관측 인코딩 변화다.
**§8.29의 공식 판정은 변경하지 않는다.**

따라서 실제 결함은 과제/센서 정합이 아니라 **센서 모델이 세 방향으로 동시에 틀려 있다**는 것이다:
해상도(실기 대비 13배 조악), 검출 임계(면적 2 px² ≈ 지름 1.6 px — Johnson 검출 기준 미만),
거리(해석적 정확값, 오차 0 — 28 m 스테레오 시차는 1.2–2.4 px로 측정 불가).

### 다음 실험 (in force) — 센서 충실도, 평가 전용

사전등록: [`docs/prereg_2026-08-22_sensor_fidelity.md`](docs/prereg_2026-08-22_sensor_fidelity.md)
(2026-08-22 동결, 커밋 `e2b95f8` — 구현 기계가 존재하기 전).

| | |
|---|---|
| 성격 | **평가 전용.** 학습은 이 사전등록의 권한 밖 |
| 정책 | frozen ref5in D1 ep1900 (`197ea269…`) |
| seed | **421** (미사용), 70막대, arm당 2,049 ep |
| arm A | detect 160×90, `NAVRL_DETECTOR_MIN_PIXELS=2` — 현행 |
| arm B | detect **1920×1200**, `MIN_PIXELS=50` (지름 8 px, CNN 하늘배경 하한) |
| 고정 | `NAVRL_DETECTOR_MAX_RANGE=20.0` **변경 금지** (토큰 재정규화 교란 회피), RGB 해상도 양 arm 160×90 |
| 판정 | never-acquired가 A 대비 **+10 pp 이상** → `FIDELITY_COST_CONFIRMED` / ±3 pp → `FIDELITY_NEUTRAL` / 그 외 INCONCLUSIVE |

**판정 방향에 주의**: 임계가 25배 오르므로 arm B가 나빠지는 것이 **예상된 결과**다. 그 크기가
"지금까지의 성적 중 얼마가 존재할 수 없는 센서 덕분이었는가"의 추정치이며, 이 실험의 값어치는
개선이 아니라 **정직한 기준선의 확립**이다. capture/crash/timeout은 원값 보고하되 판정에 쓰지 않는다.

**게이트 0(구현 타당성)이 판정보다 먼저다.** 검출 해상도를 RGB/perception 해상도와 분리하는 것은
knob이 아니라 설계 변경이며, detect == camera에서 bit-identical, appearance 교란 ≠ 0인데
detect ≠ camera면 fail-closed, 모든 조합에서 관측 898-D 유지를 증명해야 한다. 실패 시
`FAIL_CLOSED_IMPLEMENTATION`이며 센서 모델에 대한 주장을 하지 않는다.

### 다음 실험 (in force, 2026-08-22 갱신) — 검출 거리 2단계

[`docs/prereg_2026-08-22_detection_range_2stage.md`](docs/prereg_2026-08-22_detection_range_2stage.md).

seed 421이 `FIDELITY_NEUTRAL`(never-acquired +0.195 pp, `target_hidden_fraction` 양 arm 0.82)로
**검출 임계가 아니라 20 m 클립이 구속 조건**임을 확정했다. 이제 정직한 고해상도 센서로 28 m를
물리적으로 정당하게 볼 수 있으므로(28 m에서 92 px² > 임계 50), 조작 축을 **클립**으로 옮긴다.

| | 1단계 (스크리닝) | 2단계 (확증, 조건부) |
|---|---|---|
| 초기화 | frozen ep1900 warm-start | **fresh** |
| seed (학습/평가) | 457 / 461 | 463 / 467 |
| 예산 | 1,000 epoch, 합 1.7 h | 10,000 epoch, 합 17 h |
| arm | 클립 20 m vs 28 m, 그 외 전부 동일(detect 1920×1200 / `MIN_PIXELS=50`) | 동일 |
| 게이트 | never-acquired `≤ −15 pp` → `RANGE_HELPS` | `≤ −15 pp` **및** capture `≥ +5 pp` → `RANGE_CONFIRMED` |

**1단계 음성은 "효과 없음"이 아니라 "이 예산에서 미결"이다** — 양 arm이 20 m 정책에서 출발하므로
설계가 arm B에 불리하다. `RANGE_HELPS`가 아니면 2단계를 실행하지 않는다.

2단계는 70막대 고정 10k이므로 **P3가 아니며**(P3 = 70→205막대·30k·seed 211) P3 차단은 유지된다.

### 2026-08-23 Stage 1 결과 — 종료, Stage 2 미승인

| arm | pooled never-acquired | capture | crash | timeout |
|---|---:|---:|---:|---:|
| clip 20 m | 8.443% | 82.235% | 15.666% | 2.099% |
| clip 28 m | 3.172% | 88.677% | 11.274% | 0.049% |
| Δ (28−20) | **−5.271 pp** | +6.442 pp | −4.392 pp | −2.050 pp |

두 arm은 train seed 457, eval seed 461, detect 1920×1200/50 px, 70 bars, hard-distance
`[22.5,28] m`, deterministic/governor-off 조건이다. provenance/quality gate는 **17/17 PASS**이고
각 arm은 terminal epoch 2900/frame 11,878,400에서 정상 종료했다.

Primary `−5.271 pp`는 `−15 pp`에 미달했다. capture는 사전등록상 판정에서 제외된 부수 관측이므로
공식 verdict는 `RANGE_INCONCLUSIVE_AT_THIS_BUDGET`, `stage2_authorised=false`다. 임계를 사후 완화하거나
fresh 10k Stage 2를 실행하지 않는다. 원자료:
[`results/navrl_ref5in_detection_range_stage1_s457/summary.md`](results/navrl_ref5in_detection_range_stage1_s457/summary.md).

양 arm은 실기 far-range 오차를 넣지 않은 analytic exact range를 사용했다. 따라서 다음은 PPO가 아니라
exact BOM/calibration/time-sync와 real-log bearing/range/latency/dropout profile을 닫는 72시간 계측이다.

### 대기 중 (조건 미충족, 사전등록 동결됨)

**정직한 센서에서의 적응 학습** —
[`docs/prereg_2026-08-22_honest_sensor_adaptation.md`](docs/prereg_2026-08-22_honest_sensor_adaptation.md)
(2026-08-22 동결, **센서 충실도 결과를 보기 전**).

**센서 충실도가 `FIDELITY_COST_CONFIRMED`일 때만 실행한다.** `FIDELITY_NEUTRAL`이나
`INCONCLUSIVE`이면 실행하지 않으며, 이 조건을 결과를 본 뒤 완화하지 않는다.

| | |
|---|---|
| 학습 seed **433** / 평가 seed **449** | 둘 다 미사용 |
| 예산 | 1,000 epoch / 4.096M samples (riskcap 적응 선례) |
| arm A | detect 160×90 / `MIN_PIXELS=2` — 예산 효과 분리용 control |
| arm B | detect 1920×1200 / `MIN_PIXELS=50` |
| 판정 | `NA_B ≤ NA_frozen − 10 pp` **및** `NA_B ≤ NA_A − 5 pp` → `ADAPTATION_RECOVERS` / `NA_B > NA_A − 5 pp` → `ADAPTATION_INEFFECTIVE` / 그 외 INCONCLUSIVE |

**이것은 P3가 아니다.** P3는 70→205 bars · 30k epoch · seed 211이며, 본 실험은 70 bars 고정
1,000 epoch다. 따라서 fail-closed 5(P3 금지)를 위반하지 않으며, **P3 차단은 그대로 유지된다.**
`ADAPTATION_RECOVERS`가 나와도 정책을 채택하지 않는다 — 채택은 P2 gate 통과가 필요하고 별개 실행이다.

### 대기 중 (실행 결정 안 됨)

- **paired-reflection consistency** — 사전등록
  [`docs/prereg_2026-08-22_paired_reflection_consistency.md`](docs/prereg_2026-08-22_paired_reflection_consistency.md)
  동결(`e8f9b3e`), 미실행. GPU 2.5–3 h. N1이 `CHIRALITY_CONFIRMED_REAL_FRAME`(실제 프레임 15,488,
  median conj_err_lat 1.454, sign agreement 2.49%)을 냈으므로 자격은 있으나, 인과 근거가 있는
  쪽은 센서이므로 우선순위가 낮다.
- **거리 충실도(#3)** — 별도 사전등록 필요. 검출 임계와 달리 물리적으로 독립된 양이므로 함께
  바꾸지 않는다.

## fail-closed 규칙

> 각 규칙은 **사유**와 **재검 조건**을 함께 갖는다(2026-08-22,
> [`docs/discipline_review_2026-08-22.md`](docs/discipline_review_2026-08-22.md)). 원인이 소멸한
> 규칙은 만료 대상이며, 재검 없이 남은 금지는 규율이 아니라 퇴적물이다.

1. 앞 gate **명시적 PASS** 없이 다음 단계 시작 금지.
2. 실패를 epoch 추가·threshold 사후 완화·parameter sweep으로 덮지 않음.
3. 한 run에서 airframe, reward, horizon, representation, governor **동시 변경 금지**.
4. runtime git tree dirty면 fail-closed (VOID 후 clean 재실행).
5. P3는 P2 PASS 전까지 **실행 금지**.

## P3 조건 (참고만 — 현재 차단)

P2 PASS 후에만: training seed **211**, eval seed 313 재사용 금지, 70→205 bars,
30k epoch budget, unique checkpoint 250 epoch 간격.

## 하드웨어 gate (별도)

P0–P3 PASS ≠ 실기 검증. BOM/CAD/CG, thrust stand, inertia 식별, FOV/latency, 실제 비행은
[`docs/archive/reference_platform_proposal_2026-08.md`](docs/archive/reference_platform_proposal_2026-08.md),
[`docs/archive/sim_vs_hardware_gap_2026-08.md`](docs/archive/sim_vs_hardware_gap_2026-08.md) 참고.

## Physical-target fresh lineage (2026-08-21)

`NAVRL_TARGET_DYNAMICS=physical`의 1–6단계 구현은 완료했지만, fresh PPO는 **BLOCKED**다.
dynamic PhysX actor, 0.01 s four-motor controller, 동일 camera/LiDAR/contact OBB, exact bar AABB,
fresh-only checkpoint guard까지 구현했다. 고정 seed 503 / 32 env / density별 280 measured step /
1.5 m/s mixed target의 사전등록 gate에서 70/150/205/300 bars 모두 전체 PASS하지 못했다.

| bars | speed ratio | tracking RMSE | contact | immediate infeasible | invalid OBB | verdict |
|---:|---:|---:|---:|---:|---:|---|
| 70 | 0.919 | 0.233 m/s | 0.257% | 1.574% | 0.089% | FAIL |
| 150 | 0.880 | 0.279 m/s | 0.525% | 1.417% | 0.089% | FAIL |
| 205 | 0.832 | 0.316 m/s | 0.904% | 1.384% | 0.045% | FAIL |
| 300 | 0.738 | 0.351 m/s | 1.373% | 2.042% | 0.011% | FAIL |

Authority: [`docs/navrl_physical_target_audit_2026-08-21.md`](docs/navrl_physical_target_audit_2026-08-21.md),
raw summary: [`results/navrl_physical_target_verification/summary.json`](results/navrl_physical_target_verification/summary.json).
다음은 장기학습이 아니라 global/corridor target route 또는 density-conditioned speed envelope를
사전등록하고 같은 gate를 재실행하는 것이다. 실기 검증은 hardware identification manifest가
미완료이므로 별도로 차단된다.

### 2026-08-24 속도 포락선 진단 — 완료, physical PPO 해제 아님

`docs/prereg_2026-08-24_physical_target_speed_envelope.md`에 고정한 4×4 grid를 실행했다.
seed 509, mixed target, 32 env, 280 measured steps/cell이며 기존 physical-target gate를 그대로
사용했다. 각 speed arm은 Isaac Gym 프로세스를 분리해 실행했다.

| bars | 0.6 m/s | 0.9 m/s | 1.2 m/s | 1.5 m/s | 최고 passing speed |
|---:|:---:|:---:|:---:|:---:|---:|
| 70 | PASS | FAIL | FAIL | FAIL | 0.6 |
| 150 | PASS | PASS | FAIL | FAIL | 0.9 |
| 205 | FAIL* | FAIL* | FAIL | FAIL | 없음 |
| 300 | PASS | PASS | FAIL | FAIL | 0.9 |

`*` 205 bars의 0.6/0.9 m/s는 tracking·contact·planner는 통과했지만 strict invalid-state
`=0` gate에서 각각 1 sample(0.011%)이 남아 FAIL이다. 따라서 205 bars는 이 표본에서 “안전한
속도”를 채택하지 않는다. 이 결과는 높은 속도를 무조건 풀자는 근거가 아니라, 205-bar의 희귀
OBB/boundary event와 route feasibility를 먼저 고쳐야 한다는 진단이다. 모든 결과 원자료는
[`results/navrl_physical_target_speed_envelope_seed509/summary.json`](results/navrl_physical_target_speed_envelope_seed509/summary.json).

이 진단만으로 fixed-speed task를 density-conditioned task로 바꾸거나 PPO를 시작하지 않는다.
physical-target fresh lineage는 여전히 **BLOCKED**다.

### 2026-08-24 physical OBB boundary forensic 및 engineering fix — gate 여전히 BLOCKED

205 bars, seed 509, mixed, 32 env, 0.6/0.9/1.2/1.5 m/s의 고정 contract에서 invalid
non-contact event의 원자료를 기록했다. pre-fix 이벤트는 모두 finite position이었고, 주된
음수 margin은 x/y arena boundary였다. NaN, contact 귀속 누락, target teleport 증거는 없었다.
따라서 이를 경로 planner의 “진입 거부”로 설명하지 않는다. 이 코드는 planner가 아니라 velocity
controller이며, 이전 구현은 infeasible first-step에서 least-bad command를 계속 실행했다.

수정은 두 가지로 제한했다: (1) OBB의 현재 world-axis support를 반영한 center bounds,
(2) infeasible first-step에서 zero planar command를 제출해 물리 controller가 감속하도록 하는
fallback. strict counter와 gate는 그대로다. 결과는 기존 summary를 덮어쓰지 않고 다음에 보존했다.

| bars | 0.6 m/s | 0.9 m/s | 1.2 m/s | 1.5 m/s | 최고 passing speed |
|---:|:---:|:---:|:---:|:---:|---:|
| 70 | PASS | PASS | FAIL | FAIL | 0.9 |
| 150 | PASS | FAIL | FAIL | FAIL | 0.6 |
| 205 | PASS | FAIL | FAIL | FAIL | 0.6 |
| 300 | PASS | FAIL | FAIL | FAIL | 0.6 |

post-fix forensic은 0.6/0.9/1.2 m/s에서 invalid event 0건, 1.5 m/s에서 1건(유한 OBB,
x-margin 약 −0.00006 m)을 보였다. 이는 개선이지 해결이 아니다. physical PPO와 speed
완화는 여전히 금지한다. 원자료: `results/navrl_physical_target_invalid_forensics_post_wall_brake_seed509/summary.json`,
`results/navrl_physical_target_speed_envelope_post_wall_brake_seed509/summary.json`.

### 2026-08-24 synthetic corridor mode probe — 판정 보류

고정 ref5in checkpoint/seed 431의 6-arm synthetic corridor screen은 policy action의 좌우
reflection error가 사전 gate(0.15)를 초과해 `INCONCLUSIVE_POLICY_CHIRALITY`로 종료됐다.
probe action은 실제 환경에 실행하지 않았고, 학습·reward·checkpoint를 바꾸지 않았다. 따라서
이 결과로 mode averaging을 적용하거나 고밀도 병목의 원인을 확정하지 않는다. 원자료:
`results/navrl_ref5in_symmetric_corridor_mode_probe_seed431/summary.json`.

### 2026-08-24 sim-to-real software-only preflight — 구조 PASS, 실기 주장 불가

한 번의 preflight에서 telemetry 계약, CSV ingest, trial-level sensor profile, two-zone replay를
연결했다. 네 단계 모두 구조상 `PASS`였고, synthetic fixture임을 각 결과와 receipt에 남겼다.
mode probe는 기존 결과대로 `INCONCLUSIVE_POLICY_CHIRALITY`, physical-target speed envelope는
`all_cells_pass=false`였다. 따라서 fresh PPO는 `BLOCKED`로 유지한다. 원자료:
`results/navrl_sim2real_software_preflight_2026-08-24/summary.json`.

### 2026-08-24 reflection·distance 상태

- 기존 N1 real-frame reflection audit는 receipt를 다시 검증해 `CHIRALITY_CONFIRMED_REAL_FRAME`
  PASS를 확인했다. paired-reflection consistency A/B는 별도 학습 계약이며, 사전등록 §5-b의
  loss-profile 산출물과 dedicated arm launcher가 없어 실행하지 않았다. `NAVRL_REFLECTION_COEF=0.01`
  을 임의 재사용하지 않는다.
- 거리 충실도는 [`docs/prereg_2026-08-24_distance_fidelity.md`](docs/prereg_2026-08-24_distance_fidelity.md)
  로 설계만 고정했다. 실제 range ground truth/profile이 없으므로 합성 noise를 만들거나 평가하지
  않는다(`BLOCKED_NO_REAL_RANGE_PROFILE`).

## 아카이브

2026-08-20 통합으로 아래를 `docs/archive/`로 이동했다.

- 발표/PPT/review/handoff/prereg 일자별 문서 17개
- RESEARCH_PLAN §8.1–8.22 (v2/TTC/riskcap 역사): [`RESEARCH_PLAN_v2_history.md`](docs/archive/RESEARCH_PLAN_v2_history.md)
- ref5in 감사 초안: [`ref5in_audit_and_next_steps_2026-08-13.md`](docs/archive/ref5in_audit_and_next_steps_2026-08-13.md)

코드·결과 manifest에 예전 `docs/*.md` 경로가 남아 있을 수 있다. authority는 이 파일과 WORKLOG를 따른다.
