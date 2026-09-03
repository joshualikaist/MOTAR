# VERIFICATION — ref5in 검증 단계 (실행 authority)

검증 gate·판정·다음 실험은 **이 문서**가 규정한다. 연구 charter(가설·방법)는
[`RESEARCH_PLAN.md`](RESEARCH_PLAN.md), 날짜별 기록은 [`WORKLOG.md`](WORKLOG.md),
명령어는 [`OPERATIONS.md`](OPERATIONS.md), 라이브 지표는 [`docs/status/`](docs/status/)를 본다.

> 기준일: 2026-09-03

현재 판정의 기계 판독 원본은
[`docs/research_authority_2026-08-26.json`](docs/research_authority_2026-08-26.json)이다.
`python tools/check_research_authority.py --json`은 frozen summary의 SHA와 Track A/B 및 corrected
non-overlap 판정 필드를
직접 대조하며 어떤 학습·평가도 실행하지 않는다.

## 2026-09-03 software-only next work (GPU 권한 없음)

- **다음 software 작업**은 오프라인 instance adapter CPU 계약이다. 사전등록
  [`docs/preregistration_sam_instance_adapter_offline_2026-09-03.md`](docs/preregistration_sam_instance_adapter_offline_2026-09-03.md).
  SAM 3는 설치하지 않았고 Isaac 제어루프에 연결하지 않는다. PPO/GPU 평가 권한은 없다.
- Track A/B, corrected non-overlap, braking-v3, route-off held-out의 **기록된 FAIL/INCONCLUSIVE/BLOCKED 판정은 수정하지 않는다.**
- S1 blind-search state와 이 축을 한 실험에서 섞지 않는다.

## 2026-09-01 현재 실행 authority

- **Track A — perception/sim-to-real:** 아래 2026-08-26 절의 판정 그대로다. P2 `STRICT FAIL`,
  D1 `FAIL`, P3 `BLOCKED`. 실제 hardware나 real log가 없으면 GPU 작업 권한이 없다.
- **Track B — routed physical target:** recovery-v2 계보는 닫혔다. 기록된
  `FAIL_ROUTE_MECHANISM` / no-anchor `INCONCLUSIVE` 수치는 수정하지 않는다.
- **Corrected non-overlap — current fresh lineage:** seed 829 route/physical gate r2는
  `PASS_32_CELL_INTEGRITY / FAIL_ROUTE_MECHANISM / BLOCKED_PHYSICAL_TRAINING`이다. 70-bar plan
  `17.7831%`(gate ≥99%), fallback `30.0156%`(gate ≤1%), 0.6 m/s goals/env `0.21875`(gate ≥0.5)다.
  이 FAIL 판정과 수치는 유지한다. 500-epoch smoke와 70→205 장기학습은 모두 미실행(새 PPO 0 epoch).
- **Braking-aware route v3:** `global_astar_braking_v3` 구현은 CPU 계약까지 완료·커밋됐다
  (`6b13441`). 사전등록
  [`docs/preregistration_braking_aware_route_v3_2026-09-01.md`](docs/preregistration_braking_aware_route_v3_2026-09-01.md)
  (SHA-256 `cceecb9ad4a538e7bc2bc9171436e823ef18652e9c971e0d6fa8174279df6056`)는 결과를 보기 전에
  고정됐다. **2026-09-01 재측정 결과: canonical 1.5 m/s braking receipt는 NO-GO다.**
  커밋 `fb9fa50`에서 0.6/0.9/1.2 m/s는 통과했으나 1.5 m/s warmup이 mean 1.442577 m/s
  (절대 오차 0.057423 > 게이트 0.05)로 실패했고, 이 수치는 2026-08-26 측정과 동일한 결정론적
  controller 한계다. 이 판정과 수치는 유지한다. threshold/controller/0.05 게이트는 바꾸지 않았다.
  **현재 다음 software 작업은 별도 사전등록된 lower-contract v3다.**
  [`docs/preregistration_braking_aware_route_v3_lower1p25_2026-09-01.md`](docs/preregistration_braking_aware_route_v3_lower1p25_2026-09-01.md)
  (SHA-256 `cd1347121c24ecd10273189360bed9ca76ffa80673aa89addf3ff0eaebc16252`).
  속도 grid는 0.6/0.9/1.2/1.25이며 `NAVRL_TARGET_BRAKING_CONTRACT_VARIANT=baseline_1p25`로만
  무장한다. 기본값은 여전히 `canonical_1p5`다.
  08-26 lower receipt는 옛 core 바이트에 묶여 재사용하지 않는다. PID나 0.05 완화는 이 계보에서
  하지 않는다. **2026-09-01 현재 커밋 `dd8b4a4`에서 새 `baseline_1p25` raw receipt가
  검증됐다** (`results/navrl_physical_target_braking_lower1p25_seed827_2026-09-01/receipt.json`,
  SHA-256 `6d71b0be34ffb166d23aff9f6897cf41b5bb82a4a488d24403d735cf97852485`).
  속도 0.6/0.9/1.2/1.25, decel p05 0.505257, stop-time p95 0.89 s, 정지거리 p95
  0.32814/0.49835/0.67084/0.69944 m. training source bundle SHA
  `5de7fdbfa56811b71284d454195239d8490e8a5db0b54a22d3e8b9c58263964c`.
  **2026-09-01 lower 8-cell pilot는 `VOID_EXECUTION`이다** (reason:
  `matched-arm identity drift: initial_target_pose_sha256`). 레이아웃·로봇 자세 해시는
  off/v3가 일치했으나 표적 초기 자세가 갈라졌다. v3 `_sample_general_target` /
  `_sample_waypoints`가 support를 추가로 inset하고 spawn AABB 필터를 썼기 때문이다.
  공식 판정은 NOT_INTERPRETED이며 그 VOID를 FAIL로 바꾸지 않는다.
  **현재 software 작업은 별도 사전등록된 matched-spawn 수정이다.**
  [`docs/preregistration_braking_aware_route_v3_lower1p25_matched_spawn_2026-09-01.md`](docs/preregistration_braking_aware_route_v3_lower1p25_matched_spawn_2026-09-01.md)
  (SHA-256 `17e7f35e350087bf2733aca70dfca7210efe667ad1845dd053f7334ddf1645b4`).
  v3 spawn/waypoint는 off와 같은 physical wall+boundary 상자·bar-clearance를 쓰고,
  envelope는 A*/rollout/watchdog에만 남긴다. `initial_target_pose_sha256` 매칭은 유지한다.
  CPU forensic 123개가 통과했고, 별도 execution addendum
  [`docs/preregistration_braking_aware_route_v3_lower1p25_matched_spawn_gpu_authority_2026-09-01.md`](docs/preregistration_braking_aware_route_v3_lower1p25_matched_spawn_gpu_authority_2026-09-01.md)
  (SHA-256 `70b8a08f0c95040a86c43e1be5ac11d0b688b9a6874f3cfc4548f914882f085f`)가 새
  `baseline_1p25` receipt 1회와 seed-829 8-cell pilot 1회만 허용했다. spawn 바이트가
  바뀌었으므로 `dd8b4a4` receipt는 사용하지 않았다.
  **새 pilot은 `PASS_8_CELL_INTEGRITY / FAIL_BLOCKS_CONFIRMATORY`로 종료됐다.** 모든 속도에서
  off/v3 layout·robot pose·target pose 해시가 일치해 옛 VOID 원인은 해결됐다. 그러나 routed
  speed ratio는 0.7872/0.7681/0.5984/0.6297로 모두 0.80 게이트 미달, soft exits 6,825
  (gate 0), fallback 8.914%(gate ≤1%), 0.6 m/s goals/env 0.21875(gate ≥0.5)다. plan success
  99.468%와 terminal certificate fraction 1.0은 통과했다. 원자료와 판정은
  [`docs/braking_aware_route_v3_lower1p25_matched_spawn_result_2026-09-01.md`](docs/braking_aware_route_v3_lower1p25_matched_spawn_result_2026-09-01.md)에
  고정했다. GPU authority는 소비·폐쇄됐고 confirmatory와 PPO는 열리지 않는다. 현재 stage는
  `MECHANISM_GATE_FAIL_CLOSED`다.
- **Corrected non-overlap route-off learning baseline (2026-09-01 user-priority split):** routed
  method의 FAIL을 덮지 않는 별도 70-bar/500-epoch fresh smoke를 실행했다. 근거는 동일-layout
  70-bar route-off 0.6/0.9/1.2/1.25 m/s 셀이 모두 physical PASS였다는 독립 계측이다. 계약은
  [`docs/preregistration_corrected_nonoverlap_physical_off_smoke_2026-09-01.md`](docs/preregistration_corrected_nonoverlap_physical_off_smoke_2026-09-01.md),
  launcher는 `train_navrl_corrected_nonoverlap_physical_smoke.sh`다. 공식 판정은
  `PASS_LEARNING_VIABILITY`: 초기→후기 100-epoch capture `11.01→60.39%`(+49.38pp), reward
  `-131.00→+93.08`, KL max `0.01119`, rollback/skipped 0이다. 이 PASS로 별도 사전등록된 fresh
  route-off 70→205 curriculum 1회만 열렸다. 그 1회는 2026-09-02 사용자 요청으로
  **운영자 중지**됐다 (`OPERATOR_STOPPED_INCOMPLETE`). 재개 금지, 두 번째 curriculum 금지,
  routed PPO 금지. `global_astar_*`, 1.5 m/s와 hardware claim은 계속 차단한다.
  - run `ppo_260901_1431_navrl_corrected-nonoverlap-physical-off-curriculum-s911`
  - 중지 epoch 21973/30000, 막대 145 (160+ 미도달), 마지막 공식 hold capture 0.647 / gate 0.70
  - 평가용 체크포인트는 `nn/last_gen_ppo_ep_21750_rew_83.1572.pth`뿐이며 `gen_ppo.pth`는 쓰지 않는다
  - 학습 로그 capture를 held-out로 부르지 않는다
  held-out 평가 사전등록은
  [`docs/preregistration_corrected_nonoverlap_physical_off_heldout_eval_2026-09-02.md`](docs/preregistration_corrected_nonoverlap_physical_off_heldout_eval_2026-09-02.md)
  이다. 셀은 학습한 70/85/100/115/130/145뿐이고, 205는 넣지 않았다. 체크포인트는
  `last_gen_ppo_ep_21750`만, seed 313, `gen_ppo.pth` 금지. 기본 evaluator 밀도
  `70 150 210 280`과 `navrl_band`/`U[0.3,1.5]`는 이 체크포인트에 쓰면 VOID다.
  평가는 완료되어 `COMPLETE_VALID_WITH_METADATA_ERRATUM`으로 봉인됐다. capture는
  70/85/100/115/130/145 bars에서 각각
  **83.70/80.94/77.75/73.45/69.17/65.54%**, timeout은 전 셀 0.4% 이하다.
  70→145의 −18.16 pp는 거의 전부 bar contact 증가(+18.25 pp)다. 보조 contract JSON의
  speed max `1.5`는 serializer 하드코딩 오류이고, 실제 condition/log/validator/속도 strata는
  모두 `1.25`를 증명한다. 원본은 수정하지 않았고 향후 serializer만 고쳤다. 현재 GPU 권한은 없다.
  결과: [`summary.md`](results/navrl_corrected_nonoverlap_physical_off_heldout_seed313/summary.md).

## 2026-08-26 기록된 실행 authority (판정 보존)

- **Track A — perception/sim-to-real:** P2 `STRICT FAIL`, D1 `FAIL`, P3 `BLOCKED`; detection
  Stage 1은 `RANGE_INCONCLUSIVE_AT_THIS_BUDGET`, Stage 2는 미승인이다. 유일한 다음 authority는
  [`docs/SIM2REAL_3DAY_EXECUTION_PLAN.md`](docs/SIM2REAL_3DAY_EXECUTION_PLAN.md)의 exact
  BOM/calibration, 210개 독립 sensor trial, real-log profile/replay다. 실제 hardware나 real log가
  없으면 GPU 작업 권한이 없다.
- **Track B — routed physical target:** recovery-v2 lower-1.25는
  `PASS_32_CELL_INTEGRITY / FAIL_ROUTE_MECHANISM`이고, 후속 no-anchor probe는 primary `n=1`,
  observer identity disagreement `0`으로 `INCONCLUSIVE`다. 이 계보는 닫혔으며 PPO, retune,
  1.5 m/s, env 수 변경, 32-cell 재실행 또는 다른 GPU probe 권한이 없다.
- **Corrected non-overlap — current fresh lineage:** seed 829 route/physical gate r2는
  `PASS_32_CELL_INTEGRITY / FAIL_ROUTE_MECHANISM / BLOCKED_PHYSICAL_TRAINING`이다. 70-bar plan
  `17.7831%`(gate ≥99%), fallback `30.0156%`(gate ≤1%), 0.6 m/s goals/env `0.21875`(gate ≥0.5)다.
  500-epoch smoke와 70→205 장기학습은 모두 미실행(새 PPO 0 epoch)이다. 막대 겹침 수정 때문에
  fresh PPO는 결국 필요하지만, 새 braking-aware route/controller gate가 먼저 통과해야 한다.

R0 CPU gate, 최초 0/32 VOID, attempt 2, recovery forensics와 recovery-v2의 실행 이력과 수치는
아래 날짜별 절에 보존한다. 역사적 preregistration은 현재 실행 authority가 아니다.

## 계보 단절 (2026-08-27 고정) — 과거 결과는 corrected v2로 이월되지 않는다

corrected v2 계보(40×40×3 m, **비중첩 배치**, surface clearance 0.45 m, 학습 70→205)와 그 이전
결과 사이에는 **세 개의 독립적인 단절**이 있다. 하나만으로도 이월이 불가능하며, 셋이 겹친다.

| 단절 | 내용 | 영향 |
|---|---|---|
| 배치 기하 | 중첩 허용 → **비중첩**, 아레나 40×40×3 | 관측 분포가 다르다 |
| **heading 임계** | `HEADING_VALID_SPEED_MPS`가 인라인 `1e-5`를 **0.10으로 대체**(`0d1def2`, 2026-08-26) | 표적 heading 판정이 **10,000배** 달라진다. 2026-08-26 **이전 모든 체크포인트**는 `1e-5`로 학습됐다 |
| 밀도 계보 | 70→300 → **70→205** | 커리큘럼 수준 구성 자체가 다르다 |

따라서 **corrected v2에서의 PPO 성능은 NOT RUN(0 epoch)이다.** 과거 205-bar 수치·capture·density curve는
`historical`로만 인용하며, 새 계보의 성능 주장으로 쓰지 않는다. 이 규칙은 문서·PPT·논문 브리프에
동일하게 적용된다.

heading 단절은 재개(warm-start)에도 적용된다. `env_state`가 이제
`cfg_target_motion_heading_valid_speed_mps`와 `..._provenance`를 함께 기록하며, 키가 없는 과거
체크포인트는 `assumed_pre_key_default`로 로드되되 그 메시지가 `1e-5` 사실을 직접 명시한다.
**추정을 측정으로 읽지 않는다.**

### 205 학습 상한은 기하 감사 PASS (2026-08-27 측정)

학습 상한 205의 **선택 이유**는 여전히 YOPOv2 count density 대비 약 2.05배라는 밀도 문맥이다.
연결 여부는 더 이상 유추가 아니다. 게이트(connectivity ≥ 95%, no-route ≤ 5%, 생성 실패 0)를
결과 파일보다 먼저 `VERIFICATION.md`에 고정한 뒤 CPU 감사를 돌렸다.

Canonical 6–28 m, **body+tracking** 팽창(0.650 m), 60 layout × 128 pair:

| bars | connectivity | no-route | 판정 |
|---:|---:|---:|---|
| 205 | 99.167% | 0.833% | **PASS** |
| 250 | 97.813% | 2.187% | PASS (최고 통과 밀도) |
| 300 | 94.661% | 5.339% | **FAIL** |

원자료
[`results/navrl_v2_density_geometry_audit_2026-08-27/`](results/navrl_v2_density_geometry_audit_2026-08-27/summary.md).
본학습 상한은 약속대로 **205로 유지**한다. 220/250은 연결 OOD, 300은 단절 스트레스다.
이 측정은 PPO·smoke·Track A/B GPU 권한을 만들지 않는다. 밀도 계약:
[`docs/preregistration_navrl_v2_corrected_density_geometry_2026-08-27.md`](docs/preregistration_navrl_v2_corrected_density_geometry_2026-08-27.md).

## 한 줄 상태

`navrl_ref5in_quad`는 **hardware-informed simulation candidate**다. P0·P1c는 PASS, **P2·D1은 FAIL**,
**P3 장기학습은 차단**. seed 367은 초기 미관측의 인과 기여를 지지했지만 해결책 채택은 아니다.
정직한 고해상도 검출 조건의 Stage 1(각 1,000 epoch, 2,049 ep)은 never-acquired
`8.443→3.172%`(**−5.271 pp**)로 사전 `−15 pp` gate를 못 넘어
`RANGE_INCONCLUSIVE_AT_THIS_BUDGET`; **Stage 2 권한 없음**. Track A 다음 authority는
[`docs/SIM2REAL_3DAY_EXECUTION_PLAN.md`](docs/SIM2REAL_3DAY_EXECUTION_PLAN.md).
Track B (R0) recovery-v2 lower-1.25 32-cell은 **`PASS_32_CELL_INTEGRITY` / `FAIL_ROUTE_MECHANISM`**;
후속 70-bar no-anchor geometry probe는 primary `n=1`로 **`INCONCLUSIVE`**다. 32-cell FAIL은
유지되며 재실행·게인·PPO·1.5·env 수 변경 권한은 없다.

## 지금 막혀 있는 것

| gate | 판정 | 핵심 수치 |
|---|---|---|
| P2 held-out (seed 313) | **STRICT FAIL** | 68.28 / 26.16 / **5.56%** timeout (허용 102건, 실제 114) |
| D1 adaptation (seed 331) | **FAIL** | q3/CV timeout **15.98%** > 12% |
| P3 full-budget | **BLOCKED** | P2 PASS 전까지 실행 금지 |
| R0 recovery-v2 lower-1.25 | **FAIL_ROUTE_MECHANISM** | 7/32 (off만); recovery 0/16; 70-bar plan 93.60%, fallback 47.87% |
| R0 no-anchor forensics | **INCONCLUSIVE** | primary 1/106; identity disagreement 0; Wilson 최소 n=20 미달 |
| corrected non-overlap route r2 | **FAIL_ROUTE_MECHANISM** | 32/32 integrity; plan 17.78%, fallback 30.02%, goals/env 0.21875; PPO 0 epoch |

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

### 역사적 계약 — 센서 충실도 평가(실행 완료)

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

### 역사적 계약 — 검출 거리 2단계(Stage 1 완료, Stage 2 미승인)

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

### 동결된 역사적 계약 (현행 실행 authority 아님)

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

### 동결된 후보 (현행 실행 authority 아님)

- **paired-reflection consistency** — 사전등록
  [`docs/prereg_2026-08-22_paired_reflection_consistency.md`](docs/prereg_2026-08-22_paired_reflection_consistency.md)
  동결(`e8f9b3e`), 미실행. GPU 2.5–3 h. N1이 `CHIRALITY_CONFIRMED_REAL_FRAME`(실제 프레임 15,488,
  median conj_err_lat 1.454, sign agreement 2.49%)을 냈으므로 자격은 있으나, 인과 근거가 있는
  쪽은 센서이므로 우선순위가 낮다.
- **거리 충실도(#3)** — 별도 사전등록 필요. 검출 임계와 달리 물리적으로 독립된 양이므로 함께
  바꾸지 않는다.

## 2026-09-02 distractor envelope 판정 — 색 지름길 정량화 (PLAN SYNC)

**두 detector 모두 `COLOR_SHORTCUT_CONFIRMED`.** 사전등록
[`prereg`](docs/prereg_2026-09-01_distractor_envelope.md), 원자료
[`summary`](results/navrl_detector_distractor_envelope_seed479/summary.md).

seed 479, 2 detector x N=0/1/3/5 = 8 cell, 2,049 ep/cell. FTLR = (DISTRACTOR_LOCK + GHOST_LOCK) /
가시 프레임, 분류 반경 0.5 m.

| detector | N=1 | N=3 | N=5 | 판정 |
|---|---:|---:|---:|---|
| default (5-param 색 규칙) | 52.7% | 79.7% | 88.5% | `COLOR_SHORTCUT_CONFIRMED` |
| **v7 (11,329-param 학습 CNN)** | 60.7% | 83.1% | **90.3%** | `COLOR_SHORTCUT_CONFIRMED` |

**학습된 인지가 시뮬레이터의 색 지름길을 학습했다.** frame precision 0.99766인 v7이 동색 디코이
앞에서는 가시 프레임의 90.27%에서 틀린 물체를 잡는다. 게다가 **distractor 수가 늘수록 confidence가
올라간다**(0.826 → 0.892) — `count`가 픽셀 합이라 디코이가 많을수록 점수가 커지기 때문이다.
평균 픽셀 수 147–181인데 표적 자체는 2–5 px다.

Gate 0 셋 다 PASS(N=0 계보 회귀는 기기 변경에도 ±3.75 pp 이내). **detector 간 FTLR·outcome 비교는
금지**(prereg §3-c, L6): 서로 다른 궤적 → 서로 다른 프레임 분포.

이 결과는 개선이 아니라 **결함의 정량화**이며, 후보 기반 detector(C/D 단계) 설계 근거다.
이 판정을 보고 임계값·detector·prereg를 바꾸지 않는다.

### 검증 이력 (2기기)

GTX 1650 Ti에서 측정·verify PASS → gitignore 아티팩트(61 MB) 전송 후 **RTX 3070에서도 verify PASS**.
재검증 과정에서 이식성 결함 2건을 고쳤다(host 절대 심링크 24개, `gate0.artifact_path`의 host 절대경로).
**게이트·임계값·판정 무변경**이며, 수정 전 recorded-vs-recomputed 전수 비교에서 차이는
`artifact_path` 한 필드뿐이었음을 확인했다. 상세는 WORKLOG 2026-09-02 병합 노트.

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
당시 후속 후보는 장기학습이 아니라 global/corridor target route 또는 density-conditioned speed
envelope를 사전등록하고 같은 gate를 재실행하는 것이었다. 이후 실행 이력은 아래 날짜별 절에
보존하며, 이 문장은 현재 rerun authority가 아니다. 실기 검증은 hardware identification manifest가
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

## 2026-08-25 routed physical gate — attempt 2 판정

별도 attempt 2는 preregistered route-off/on × 4 speeds × 4 densities의 32 JSON cell을 모두
기록해 `PASS_32_CELL_INTEGRITY`를 얻었다. 그러나 route mechanism은 `FAIL_ROUTE_MECHANISM`,
density-conditioned envelope는 FAIL, physical training은 `BLOCKED_PHYSICAL_TRAINING`이다.
attempt 1은 `ninja` PATH 오류로 0/32 `VOID_EXECUTION`이며 수치 결과가 아니다.

| bars \ speed | 0.6 | 0.9 | 1.2 | 1.5 |
|---:|:---:|:---:|:---:|:---:|
| 70 | off P / on F | off P / on F | off F / on F | off F / on F |
| 150 | off P / on F | off P / on F | off P / on F | off F / on F |
| 205 | off P / on F | off P / on F | off F / on F | off F / on F |
| 300 | off P / on F | off F / on F | off F / on F | off F / on F |

The four 70-bar route-on speed cells pooled together give plan success `0.1454668471` vs gate
≥0.99 and fallback `0.359296875` vs gate ≤0.01. The separately gated 70 bars × 0.6 m/s cell gives
goal completions/env `0.25` vs gate ≥0.5; same-goal reselection is 0. Across route-on cells, local
invalidation is 0.125–0.635% while fallback is 32.6–85.0%.
The route status counters are dominated by `unsafe_start` (420 vs 80 `ok`; 6 `no_path`, 6
`local_step_infeasible`), supporting a soft-envelope recovery deadlock diagnosis. This does not
prove global disconnection, and does not make motor/tilt/contact the primary cause; tracking remains
a separately gated metric.

No PPO policy was loaded, no hardware was validated, and no 300-bar arena-wide connectivity claim
is authorized. Do not alter evaluator code, preregistered thresholds, or the physical-training
block. Raw evidence and hashes: [`attempt 2 summary`](results/navrl_physical_target_routed_gate_seed827_attempt2/summary.md),
[`attempt 1 VOID`](results/navrl_physical_target_routed_gate_seed827/VOID.md).

## 2026-08-26 recovery-v2 lower-1.25 32-cell 판정

별도 `baseline_1p25` 계약의 recovery-v2 32-cell이 원자료 32개를 모두 기록해
`PASS_32_CELL_INTEGRITY`를 얻었다. route mechanism은 `FAIL_ROUTE_MECHANISM`이고 physical
training은 계속 `long_training_authorized=false`다. 7/32만 통과했고 전부 route-off다.
recovery 16 cell은 0통과. 70-bar 4-speed pool: plan success `190/203=0.9360` (gate ≥0.99),
fallback `18381/38400=0.4787` (gate ≤0.01), 70×0.6 goals/env `0.21875` (gate ≥0.5).
canonical 1.5 결과를 대체하거나 1.25를 1.5 성공으로 읽지 않는다. VOID/incomplete 형제는
합치지 않는다.

| bars \ speed | 0.6 | 0.9 | 1.2 | 1.25 |
|---:|:---:|:---:|:---:|:---:|
| 70 | off P / on F | off P / on F | off F / on F | off F / on F |
| 150 | off P / on F | off P / on F | off F / on F | off F / on F |
| 205 | off P / on F | off P / on F | off F / on F | off F / on F |
| 300 | off P / on F | off F / on F | off F / on F | off F / on F |

v1 attempt 2의 `unsafe_start` deadlock과 다른 FAIL이다. recovery-v2는 합법
`BRAKE→CONNECT→ROUTE`와 93.6% plan success까지 갔고, 그다음 `NO_CONNECTOR`에 63% occupancy로
붙는다 (hard breach 0/534). CONNECT rest-heading 수정은 명령이 앵커를 향하게 했지만 실현 속도는
정지 근처에서 ~0.055 m/s이고 CONNECT 자체는 recovery-arm 시간의 1.22%다. 게인 2.5 / `0.45 m` /
env 32 / 1.5를 이 FAIL 보고 바꾸지 않는다. 32-cell을 재실행하지 않는다.

원자료: [`gate summary`](results/navrl_physical_target_recovery_v2_gate_lower1p25_seed827/summary.json),
[`result note`](docs/physical_target_recovery_v2_lower1p25_result_2026-08-26.md).
[`no-anchor forensics preregistration`](docs/preregistration_physical_target_recovery_v2_no_connector_forensics_2026-08-26.md)
의 70-bar 4-speed GPU probe도 완료·검증됐다. 106개 `NO_CONNECTOR` 중 사전 primary는 1개뿐이고
runtime anchor boolean과 CPU replica가 일치해 VOID는 아니다. 최소 `n=20`을 못 채워
`INCONCLUSIVE`; [`result note`](docs/physical_target_recovery_v2_no_connector_forensics_result_2026-08-26.md)를
따른다. 이 결과를 보고 셀을 재실행하거나 32-cell grid, gain 2.5, `0.45 m`, PPO, 1.5, env 수를
변경하지 않는다. Track B에서 추가 GPU authority는 없다.

## 아카이브

2026-08-20 통합으로 아래를 `docs/archive/`로 이동했다.

- 발표/PPT/review/handoff/prereg 일자별 문서 17개
- RESEARCH_PLAN §8.1–8.22 (v2/TTC/riskcap 역사): [`RESEARCH_PLAN_v2_history.md`](docs/archive/RESEARCH_PLAN_v2_history.md)
- ref5in 감사 초안: [`ref5in_audit_and_next_steps_2026-08-13.md`](docs/archive/ref5in_audit_and_next_steps_2026-08-13.md)

코드·결과 manifest에 예전 `docs/*.md` 경로가 남아 있을 수 있다. authority는 이 파일과 WORKLOG를 따른다.
