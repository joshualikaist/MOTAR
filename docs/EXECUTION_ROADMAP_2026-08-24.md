# MOTAR 실행 로드맵 — 2026-08-24 snapshot

기준일: 2026-08-24. 이 문서는 실험을 조금씩 추가하지 않고, **다음 의사결정까지 필요한 전체 순서**를
한 곳에 고정한다. 현재 상태는 실기 미조립·센서 원자료 없음이며, 새 PPO는 허가하지 않는다.

> **Supersession notice · 2026-08-26:** 아래 A–F는 2026-08-24 의사결정 snapshot이다. 실행된
> route/recovery 후속과 현행 authority는 문서 끝 addendum 및
> [`../VERIFICATION.md`](../VERIFICATION.md)를 따른다.

## 현재 판정

- P2: STRICT FAIL, D1: FAIL, P3: BLOCKED.
- physical target 1–6 구현과 기본 동역학 검증은 완료됐지만, fixed 1.5 m/s physical gate는 실패했다.
- 205 bars의 낮은 속도 셀에서 보인 실패는 NaN이나 contact 누락이 아니라, 유한한 OBB가
  arena 경계를 아주 조금 넘는 strict state failure였다. 원인 분리를 위해 고정 표본 forensic을
  끝냈고, center-only reserve를 OBB-support-aware reserve로 바꾸고 infeasible step에는
  zero planar command를 보내는 물리적 braking fallback을 적용했다. 다만 수정 후에도 1.5 m/s
  한 셀에서 1건의 경계 초과가 남으므로 gate는 아직 닫히지 않았다.
- reflection A/B는 profile/전용 launcher가 없어 실행하지 않는다. 과거 `0.01` 계수를 재사용하지 않는다.
- distance fidelity는 실측 range profile이 없으므로 합성 noise를 넣지 않는다.

## 단계별 실행 순서

### A. 증거·계약 동결 — 완료

기존 checkpoint/source/robot provenance, P2/D1/P3 판정, 센서 72시간 계획을 동결했다. 모든 후속 결과는
이 판정을 소급해 바꾸지 않는다.

### B. physical target gate 닫기 — 현재 진행

1. 205 bars, seed 509, mixed, 0.6/0.9/1.2/1.5 m/s에서 invalid OBB 이벤트를 좌표·support·margin·속도·step으로 기록한다.
2. center-only wall reserve 결함이 확인되면 OBB support-aware planner bounds를 적용한다.
3. infeasible first-step에서 least-bad outward command를 보내지 않고 zero planar command로
   감속하게 한다. 이것은 teleport/position clamp가 아니며 strict event는 계속 기록한다.
4. 기존 gate를 같은 seed/grid로 재실행한다. gate 완화, 속도 추가, target teleport는 금지한다.
5. strict gate 통과 시에만 500 epoch physical PPO smoke를 별도 preregistration한다. 실패하면
   physical PPO는 계속 차단한다.

현재 소요: forensic + support-aware 수정 + 2회 재실행을 완료하는 데 약 1시간 40분의 GPU wall time이
들었다. 수정 후에도 1.5 m/s 셀이 남아 있으므로 B는 아직 미완료다. 이미 통과한 셀을 성과로
과장하지 않고, strict gate를 닫을 때까지 PPO를 시작하지 않는다.

현재 post-fix 결과(새 output root, 기존 결과 보존):

| bars | 0.6 m/s | 0.9 m/s | 1.2 m/s | 1.5 m/s | 최고 passing speed |
|---:|:---:|:---:|:---:|:---:|---:|
| 70 | PASS | PASS | FAIL | FAIL | 0.9 |
| 150 | PASS | FAIL | FAIL | FAIL | 0.6 |
| 205 | PASS | FAIL | FAIL | FAIL | 0.6 |
| 300 | PASS | FAIL | FAIL | FAIL | 0.6 |

post-fix forensic에서는 0.6/0.9/1.2 m/s의 invalid non-contact event가 0건, 1.5 m/s에서
1건(OBB x-margin 약 −0.00006 m, finite, contact 없음)이었다. 이는 “해결”이 아니라
boundary inertia의 잔여 strict failure를 수치로 확인한 것이다.

### C. 실제 기체·센서 계약 — 하드웨어 대기

사용자가 기체를 조립하고 BOM/AUW/CG/inertia, thrust curve, camera/LiDAR calibration, timestamp 계약,
210개 sensor trial을 취득한다. 그 전에는 URDF/질량/센서 noise를 임의로 수정하지 않는다.

하드웨어가 없는 동안 software-only preflight를 실행했다. telemetry, CSV ingest, trial profile,
two-zone replay는 모두 구조적으로 PASS했지만 결과의 claim status는 `SYNTHETIC_ONLY`다. 실제
기체·센서가 생기면 같은 산출물 자리에 real-log 입력을 넣어 다시 실행한다.

### D. 실측 sensor profile과 two-zone replay — C 이후

trial 단위 bearing/range error, valid fraction, dropout burst, latency/skew를 계산하고, far bearing-only /
near range-fusion observation contract를 고정한다. 이 단계가 끝나기 전에는 distance perturbation이나
sim-to-real PPO를 실행하지 않는다.

### E. 선택적 reflection consistency — 우선순위 낮음

합성 symmetric-corridor mode probe는 이미 고정 checkpoint로 실행했고 결과는
`INCONCLUSIVE_POLICY_CHIRALITY`였다. 즉 좌우 reflection error가 gate를 넘었고, synthetic fixture에서
mode averaging을 지지하는 근거가 나오지 않았다. 이 결과는 high-density 원인이나 capture 개선의
증거가 아니다. preregistration §5-b의 실제 paired A/B 학습은 별도 control/treatment 1,000 epoch
+ 평가(약 2.5–3시간 GPU)가 필요하며, B/C/D가 닫히기 전에는 실행하지 않는다.

### F. fresh PPO — 마지막 단계

physical gate PASS + 실제 sensor contract PASS + observation/reward/source receipt PASS 후에만 실행한다.
순서는 500 epoch smoke → held-out 평가 → 한 축만 바꾼 본학습이다. 실패하면 epoch을 늘리거나 threshold를
완화하지 않고 해당 축을 되돌린다.

현재 software preflight 결과는 physical gate `all_cells_pass=false`와 real sensor contract 부재로
fresh PPO를 명시적으로 `BLOCKED`로 기록했다.

## 금지사항

- 실측 센서가 없는데 Gaussian range noise를 추정해 넣지 않는다.
- physical target gate가 실패한 상태에서 fresh PPO를 시작하지 않는다.
- speed, arena boundary, reward, observation schema를 한 실험에서 동시에 바꾸지 않는다.
- 결과를 본 뒤 gate·seed·speed grid를 바꾸지 않는다.

## 다음 보고 형식

각 단계 종료 시 결과·실패 원인·남은 blocker·다음 단계 권한을 한 번에 갱신한다. “학습을 돌렸다”보다
`어떤 계약이 닫혔고 어떤 숫자가 아직 가정인지`를 우선 보고한다.

## 2026-08-26 closure addendum

Track B의 B 단계는 성공으로 닫힌 것이 아니라 실행 결과로 **종료**됐다. Recovery-v2 lower-1.25는
32/32 integrity를 통과했지만 `FAIL_ROUTE_MECHANISM`이다: 7/32 PASS(모두 route-off), recovery
0/16, 70-bar plan `93.60%`, fallback `47.87%`, 70×0.6 goals/env `0.21875`,
`NO_CONNECTOR` occupancy `63.06%`. 후속 no-anchor probe는 primary `n=1`, observer identity
disagreement `0`으로 유효하지만 `INCONCLUSIVE`다. F의 fresh PPO와 추가 Track B
GPU/retune/1.5/env-count/32-cell rerun은 승인되지 않는다.

Track A는 P2 `STRICT FAIL`, D1 `FAIL`, P3 `BLOCKED`, detection Stage 1
`RANGE_INCONCLUSIVE_AT_THIS_BUDGET`, Stage 2 미승인이다. 다음 실행은 C/D에 해당하는 exact
BOM/calibration, 210 sensor trials, real-log profile/offline replay뿐이며
[`SIM2REAL_3DAY_EXECUTION_PLAN.md`](SIM2REAL_3DAY_EXECUTION_PLAN.md)를 따른다. hardware와 real
log가 모두 없으면 GPU 작업을 하지 않는다.
