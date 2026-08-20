# VERIFICATION — ref5in 검증 단계 (실행 authority)

검증 gate·판정·다음 실험은 **이 문서**가 규정한다. 연구 charter(가설·방법)는
[`RESEARCH_PLAN.md`](RESEARCH_PLAN.md), 날짜별 기록은 [`WORKLOG.md`](WORKLOG.md),
명령어는 [`OPERATIONS.md`](OPERATIONS.md), 라이브 지표는 [`docs/status/`](docs/status/)를 본다.

> 기준일: 2026-08-20

## 한 줄 상태

`navrl_ref5in_quad`는 **hardware-informed simulation candidate**다. P0·P1c는 PASS, **P2·D1은 FAIL**,
**P3 장기학습은 차단**. 병목은 장거리 CV에서의 **초기 표적 미관측(camera 20 m vs goal 22.5–28 m)** 으로
좁혀졌다. seed 367 camera-range 인과 대조가 **완료·동결**됐고 primary gate를 통과했다 — camera 20→28 m에서 timeout `55.80% → 18.16%`(**−37.65 pp**). 이는 **진단**이며 P2/D1 FAIL과 P3 차단은 그대로다.

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

### 이제 남은 것은 설계 결정이다

병목이 특정된 이상 다음은 진단이 아니라 선택이다. **(a) 과제를 센서에 맞춘다**(goal 거리를
관측 범위 안으로) 또는 **(b) 센서를 과제에 맞춘다**(장거리 검출 전제로 재학습). 둘 다 재학습이
필요하므로 P3 차단 해제 조건과 함께 **별도 사전등록**한다. 이번 결과만으로 재학습을 시작하지
않는다.

## fail-closed 규칙

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

## 아카이브

2026-08-20 통합으로 아래를 `docs/archive/`로 이동했다.

- 발표/PPT/review/handoff/prereg 일자별 문서 17개
- RESEARCH_PLAN §8.1–8.22 (v2/TTC/riskcap 역사): [`RESEARCH_PLAN_v2_history.md`](docs/archive/RESEARCH_PLAN_v2_history.md)
- ref5in 감사 초안: [`ref5in_audit_and_next_steps_2026-08-13.md`](docs/archive/ref5in_audit_and_next_steps_2026-08-13.md)

코드·결과 manifest에 예전 `docs/*.md` 경로가 남아 있을 수 있다. authority는 이 파일과 WORKLOG를 따른다.
