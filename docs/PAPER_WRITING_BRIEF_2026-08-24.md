# MOTAR 논문 작성용 근거 브리프

작성 기준: 2026-08-24
코드 기준 commit: `1804a0a`
목적: 이 파일과 링크된 원자료만 사용해 초안 논문을 작성한다. 없는 실험·하드웨어·문헌 서지는 만들지 않는다.

## 0. GPT에게 전달할 작성 규칙

다음 규칙을 지켜 논문을 작성한다.

1. **simulation 결과와 sim-to-real 결과를 절대 섞지 않는다.** 현재 실제 기체 조립, 센서 calibration,
   실제 비행 및 real sensor log는 없다.
2. historical `navrl_ref5in_quad`와 fresh-only `navrl_ref5in_v2_quad`는 모두 실제 기체가
   아니라 **hardware-informed simulation candidate**다. 질량·추력·관성은 설계점이며
   hardware identification 결과가 아니다.
3. v1 결과(24×24 m, 478 m²)는 chirality/phantom-wall 문제로 superseded다. 논문의 주 결과로 쓰지 않는다.
4. v2의 held-out capture/crash/timeout은 반드시 density, target speed, arena, evaluator semantics,
   checkpoint, seed, episode 수와 함께 쓴다.
5. reward/PPO loss/KL은 성공률이 아니다. 주 결과는 held-out capture rate와 crash/timeout이다.
6. mode probe의 `INCONCLUSIVE_POLICY_CHIRALITY`는 mode averaging의 근거가 아니다.
7. 외부 논문 제목·저자·DOI는 확인된 자료가 없으면 `[CITATION NEEDED]` placeholder로 남긴다.
8. 모든 표와 그림에는 `simulation-only`, `synthetic-only`, `held-out`, `OOD` 여부를 표시한다.

## 1. 연구를 한 문장으로 설명

MOTAR는 카메라·LiDAR와 ego-state만을 actor에 제공하는 UAV가 무작위로 배치된 막대 장애물장 안에서
움직이는 표적을 요격하도록 학습하고, 밀도·표적 속도·인지 지연이 실패 모드에 미치는 영향을 재현 가능한
held-out simulation protocol로 분해한다.

권장 제목 후보:

- **MOTAR: Failure-Aware Sensor-Only UAV Interception in Dense Random Obstacle Fields**
- **How Density, Target Motion, and Perception Delay Limit Sensor-Only UAV Interception**

## 2. 논문의 핵심 기여 후보

1. camera/LiDAR target track과 structured obstacle representation을 이용하는 sensor-only UAV
   interception task와 provenance-bound evaluation protocol.
2. density × target-speed held-out map에서 density effect가 target-speed effect보다 큰지와 interaction을
   통계적으로 분리하는 분석.
3. crash를 bar contact, below, out-of-bounds, timeout으로 분해하고, latency의 naive transform과
   timestamp-aware transform을 구분하는 failure analysis.
4. physical-target controller, OBB boundary validity, motor/tilt/tracking/planner gate를 별도로 검증해
   PPO 성능과 target/vehicle feasibility를 혼동하지 않는 방법.
5. 실제 하드웨어가 없을 때 simulation 결과를 sim-to-real 증거로 과장하지 않는 measurement-before-training
   workflow.

## 3. Task와 simulation contract

| 항목 | 현재 기준 |
|---|---|
| simulator | Isaac Gym/Aerial Gym/Warp/rl_games |
| corrected-v2 arena | 40×40×3 m |
| placement area | 1,600 m² |
| obstacle | historical: 높이 3 m + overlap-permitting `navrl_band`; corrected fresh lineage: footprint-aware non-overlap, 0.45 m surface clearance |
| actor observation | camera/LiDAR structured scene + simulator ego-state |
| privileged actor input | GT target position/velocity/visibility/semantic mask 금지 |
| LiDAR | 4×72 beams, 12 m nominal range |
| obstacle representation | 8 cluster-sector tokens, 240° selection FOV, ±10° suppression (계보별 차이 명시) |
| target | mixed motion; learned-task distribution U[0.3, 1.5] m/s, goal distance U[6, 28] m |
| pursuer command | bounded squashed-Gaussian velocity/yaw action |
| episode | corrected-v2 exact 600 actions, `time_outs` 전달 |
| primary metric | held-out capture: pursuer가 target 0.5 m 이내로 종료한 episode 비율 |
| secondary metrics | crash, timeout, bar contact, OOB/below cause, latency/track metrics |

새 `navrl_ref5in_v2_quad`는 1.20 kg, 220 mm motor diagonal, 0.283 m XY collision proxy를 사용하는
fresh-only hardware-informed simulation candidate다. historical `navrl_ref5in_quad`의 0.280 m
자산은 과거 체크포인트 provenance 때문에 그대로 보존한다. 실제 AUW/CG/inertia/thrust
curve/thermal/power는 미측정이다.

## 4. Reward와 정책

현재 frozen audit에 기록된 reward 구성:

- range-rate closing term: `+1.0`
- time cost: `−0.05` per step
- static safety: `+1.5 · mean(log(d/range))` under the configured safety condition
- ego-progress PBRS: `+1.0 · (d_prev − 0.99·d_new)`
- smoothness: `−0.1·Δv`
- height penalty: `−8.0·height²` outside the allowed band
- terminal capture: `+30`
- collision: `−20` (terminal overwrite)
- timeout bonus: none

Transformer/temporal design은 history를 structured token으로 압축한다. 논문에서는 “Transformer가 항상
RNN보다 우월하다”고 쓰지 말고, 본 연구에서 비교한 지표와 history contract가 있는 경우에만 제한적으로
주장한다. capture 하나만으로 architecture superiority를 판정하지 않는다; smoothness, latency, crash,
OOD density, parameter/FLOP budget을 함께 보고한다.

## 5. 주 결과 A — v2 density × speed held-out map

원자료: `docs/status/status.json`의 `density_speed_map`, 관련 evaluator receipt/summary.
조건: v2 40×40 m arena, ep25000 + riskcap frozen candidate, deterministic, 약 2,050 episodes/cell,
seed 47. 130–205 bars는 trained density support, 220 bars는 OOD다.

### Density effect at target speed 0.3 m/s

| bars | density / 100 m² | capture | crash |
|---:|---:|---:|---:|
| 130 | 8.12 | 88.29% | 7.42% |
| 160 | 10.00 | 86.10% | 9.80% |
| 190 | 11.88 | 82.63% | 13.95% |
| 205 | 12.81 | 81.21% | 14.89% |
| 220 (OOD) | 13.75 | 77.94% | 18.50% |

### Speed effect at 205 bars

| target speed | capture | crash |
|---:|---:|---:|
| 0.3 m/s | 81.21% | 14.89% |
| 0.7 m/s | 81.21% | 16.11% |
| 1.1 m/s | 80.77% | 17.08% |
| 1.5 m/s | 78.15% | 20.10% |

요약 effect size: density cost 약 **−11.36 percentage points**, speed cost 약 **−2.67 pp**.
훈련된 density support 안에서 density×speed interaction은 확인되지 않았다(continuous likelihood-ratio
`p=0.337`, categorical omnibus `p=0.817`). 220 bars 결과는 OOD generalization으로 분리한다.

## 6. 주 결과 B — curriculum ceiling

원자료: `docs/status/status.json`의 `density_ceiling`.

| bars | capture window | 판정 |
|---:|---:|---|
| 85 | 0.737 | promoted |
| 90 | 0.718 | promoted |
| 95 | 0.670 → 0.709 | first hold 후 promoted |
| 100 | 0.521–0.631, 17 windows | ceiling; plateau 약 0.56 |

해석: 100 bars는 해당 sensor-only cluster-sector policy의 **관측된 trainable ceiling**이다. 기하학적
절대 한계나 모든 알고리즘의 한계라고 쓰지 않는다.

## 7. 주 결과 C — physical-target feasibility envelope

원자료: `results/navrl_physical_target_speed_envelope_post_wall_brake_seed509/summary.json`.
이 실험은 PPO capture가 아니라 physical target controller의 feasibility diagnostic이다. seed 509,
mixed pattern, 32 env, speed grid `{0.6, 0.9, 1.2, 1.5}` m/s, measured 280 steps/cell이며 기존 strict
gate를 그대로 사용했다.

| density | 0.6 | 0.9 | 1.2 | 1.5 | 최고 passing speed |
|---:|:---:|:---:|:---:|:---:|---:|
| 70 | PASS | PASS | FAIL | FAIL | 0.9 |
| 150 | PASS | FAIL | FAIL | FAIL | 0.6 |
| 205 | PASS | FAIL | FAIL | FAIL | 0.6 |
| 300 | PASS | FAIL | FAIL | FAIL | 0.6 |

OBB support-aware wall reserve와 infeasible-step braking fallback을 추가했지만, high-speed cells는
여전히 planner/tracking/state/contact 중 하나 이상 실패했다. 따라서 이 결과는 “속도 제한을 풀자”는
근거가 아니라, density-conditioned feasibility와 physical target contract가 PPO 전에 닫혀야 한다는
근거다.

## 8. 주 결과 D — perception/latency failure analysis

원자료: `status.json`의 `perception_robustness`, latency summaries.

- naive delayed-frame transform의 0.1 s latency 결과는 capture −42.7 pp였지만, delayed detection을
  pose-at-t로 잘못 lifting한 confound가 원인이었다. 이 값은 superseded다.
- timestamp/pose-aligned correction 후 0.1 s residual은 약 −2.5 pp.
- 0.5 s timestamp-aware latency에서는 약 −15.8 pp.
- detection dropout 0.3 조건에서는 capture 약 −12.7 pp.

논문 메시지: latency가 무조건 치명적이라는 주장이 아니라, timestamp alignment가 없으면 latency 효과를
과대평가하고, alignment 후에도 긴 delay는 실제로 손실을 만든다는 결론이다.

## 9. 보조 결과와 제외할 결과

### Corridor-token pilot

K=6 corridor token은 observation 898→946 차원으로 확장됐고 bar contact는 약 0.93 pp 줄었지만,
capture는 66.10%, gain은 +1.57 pp로 사전 gate(capture ≥68%, gain ≥3 pp)를 통과하지 못했다.
동일 표현을 더 오래 학습해 해결됐다고 쓰지 않는다.

### Synthetic mode probe

결과: `INCONCLUSIVE_POLICY_CHIRALITY`. synthetic fixture에서 좌우 reflection error가 gate를 초과했다.
probe action은 실제 환경에 실행하지 않았다. mode averaging을 채택했다거나 고밀도 병목 원인을
확정했다고 쓰지 않는다.

### Superseded v1

24×24 m/478 m² v1 density curves는 chirality와 phantom-wall 문제 때문에 역사적 기록으로만 보존한다.
v2 주 결과와 직접 결합하지 않는다.

## 10. 실패 원인에 대한 현재 결론

1. 학습된 sensor-only policy는 density가 증가할수록 capture가 감소하고 bar contact가 지배적 crash
   cause가 된다.
2. target speed 효과는 현재 v2 trained support 안에서 density 효과보다 작다.
3. 8-token/selector representation, stopping margin, target boundary feasibility는 서로 다른 축이다.
   하나의 원인으로 합치지 않는다.
4. physical-target gate 실패는 PPO 발산과 같은 현상이 아니다. target controller/arena OBB validity와
   planner infeasibility를 먼저 검사해야 한다.
5. 실제 센서가 없으므로 range noise, latency, dropout parameter를 실기 분포라고 주장할 수 없다.

## 11. 논문 구성 권장안

1. Introduction — dense moving-target interception의 gap과 failure-first evaluation 필요성
2. Related Work — NavRL/NavRL++/UAV tracking/occlusion/active perception (확인된 서지만 인용)
3. Problem Formulation — sensor-only actor, no-GT information firewall, moving target and dense bars
4. Method — camera/LiDAR structured observation, token selector, temporal policy, bounded action, reward
5. Evaluation Protocol — corrected-v2 contract, held-out seeds/episodes, provenance/VOID rules
6. Results — density×speed map, curriculum ceiling, crash causes, latency correction
7. Feasibility and Limitations — physical target envelope, synthetic ref5in, no hardware/logs
8. Discussion — density dominates speed in the tested support; representation and stopping are separate
9. Conclusion — reproducible simulation evidence, not sim-to-real claim

## 12. 초록 초안 방향

다음 수준으로 작성한다:

> We study sensor-only UAV interception of a moving target in randomly populated obstacle fields. The
> actor receives camera/LiDAR-derived structured observations and ego-state, while privileged target
> state is excluded from the policy path. Using a provenance-bound Isaac Gym protocol, we evaluate
> held-out capture, crash causes, timeout, density, target speed, and timestamp-aware perception delay.
> In the corrected-v2 arena, density produced a substantially larger performance change than target
> speed within the trained support, while the density-by-speed interaction was not statistically
> confirmed. A separate physical-target feasibility audit exposed strict planner/OBB boundary failures
> at high command speeds. These results establish a reproducible simulation benchmark and failure
> decomposition; they do not claim sim-to-real transfer because the reference airframe and sensor
> distribution remain unmeasured.

## 13. 반드시 포함할 한계

- 실제 기체 미조립, 실제 센서 로그 0개, 실제 비행 0회.
- ref5in은 hardware-informed simulation candidate.
- 220 bars는 OOD; density ceiling은 특정 policy/training contract의 관측값.
- 일부 historical curves는 superseded이며 main claim에서 제외.
- single/few seed 결과는 generalization proof가 아님.
- target/controller feasibility와 learned policy capture를 별도 실험으로 다룸.
- synthetic preflight는 schema PASS이지 sensor performance PASS가 아님.

## 14. 원자료 링크

- [canonical system specification](MOTAR_SYSTEM_SPEC_2026-08-24.md)
- [VERIFICATION.md](../VERIFICATION.md)
- [WORKLOG.md](../WORKLOG.md)
- [RESEARCH_PLAN.md](../RESEARCH_PLAN.md)
- [SIM2REAL_3DAY_EXECUTION_PLAN.md](SIM2REAL_3DAY_EXECUTION_PLAN.md)
- [status.json](status/status.json)
- [software preflight summary](../results/navrl_sim2real_software_preflight_2026-08-24/summary.json)
- [physical target envelope](../results/navrl_physical_target_speed_envelope_post_wall_brake_seed509/summary.json)
- [mode probe](../results/navrl_ref5in_symmetric_corridor_mode_probe_seed431/summary.json)

이 파일 자체가 논문 원고는 아니다. GPT에게 전달할 때 “위 근거와 제한을 지켜 6–8쪽 논문 초안을
작성하고, 외부 인용은 placeholder로 표시하며, 모든 수치 옆에 원자료 경로를 각주로 달라”고 요청한다.

## 2026-08-25 physical-target route-gate addendum

The seed-827 routed physical-target run is a **feasibility diagnostic**, not a PPO experiment. Attempt
1 is `VOID_EXECUTION` (0/32) because the conda `ninja` executable was absent from inherited `PATH`.
Attempt 2 has `PASS_32_CELL_INTEGRITY`, but `FAIL_ROUTE_MECHANISM` and
`BLOCKED_PHYSICAL_TRAINING`; its JSON claim boundary says no PPO policy, no hardware validation, and
no arena-wide 300-bar connectivity claim.

Compact strict cell map (`route-off / route-on`, P/F):

| bars \ speed | 0.6 | 0.9 | 1.2 | 1.5 |
|---:|:---:|:---:|:---:|:---:|
| 70 | P / F | P / F | F / F | F / F |
| 150 | P / F | P / F | P / F | F / F |
| 205 | P / F | P / F | F / F | F / F |
| 300 | P / F | F / F | F / F | F / F |

Across the four routed 70-bar speed cells, pooled plan success is `0.1454668471` (gate ≥0.99) and
fallback is `0.359296875` (gate ≤0.01). The separately gated 70 bars × 0.6 m/s cell has
goal completions/env `0.25` (gate ≥0.5), and same-goal reselection is `0`. Across route-on cells,
local invalidation 0.125–0.635% is amplified to 32.6–85.0%
fallback; `unsafe_start` dominates the status counters. Write this as a soft-envelope recovery
deadlock diagnosis, not as proof of global disconnection. Motor/tilt/contact are not the primary
diagnosis; tracking remains separately gated, and route-off cells are not learned-policy results.

Use the raw [attempt 2 summary](../results/navrl_physical_target_routed_gate_seed827_attempt2/summary.md)
and [attempt 1 VOID](../results/navrl_physical_target_routed_gate_seed827/VOID.md) with their listed
hashes. Do not lower preregistered thresholds or promote this gate to PPO/hardware evidence.

## 2026-08-26 claim-boundary addendum

For the current paper boundary, separate the two tracks:

- **Track A:** P2 is `STRICT FAIL`, D1 is `FAIL`, and P3 is `BLOCKED`. Detection Stage 1 is
  `RANGE_INCONCLUSIVE_AT_THIS_BUDGET`; Stage 2 was not authorized. The only next evidence contract is
  exact hardware BOM/calibration, 210 independent sensor trials, and real-log profile/offline replay.
  With no hardware or real logs, report that there is no authorized GPU work.
- **Track B:** recovery-v2 lower-1.25 is `PASS_32_CELL_INTEGRITY / FAIL_ROUTE_MECHANISM`, not a
  1.5 m/s result. Only 7/32 cells passed (all route-off), recovery passed 0/16, and the 70-bar
  diagnostics were plan `93.60%`, fallback `47.87%`, 70×0.6 goals/env `0.21875`, and
  `NO_CONNECTOR` occupancy `63.06%`. The no-anchor follow-up was valid but `INCONCLUSIVE`
  (primary `n=1`, observer identity disagreement `0`).

Do not write the no-anchor result as evidence for or against typical anchor availability, and do not
turn it into authority for another probe, 32-cell rerun, PPO, retuning, 1.5 m/s, or an env-count
change. The supported contribution remains simulation failure analysis with explicit blocked and
inconclusive outcomes, not route recovery, physical training, hardware validation, or sim-to-real
performance. Current evidence:
[recovery-v2 32-cell result](physical_target_recovery_v2_lower1p25_result_2026-08-26.md) and
[no-anchor result](physical_target_recovery_v2_no_connector_forensics_result_2026-08-26.md).
