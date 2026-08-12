# Genspark AI Slides용 MOTAR 연구 발표 브리프

> **2026-08-13 갱신:** 이 문서는 Gate 1–3의 역사적 발표 수치 원천으로 유지한다. 검증 1–5B를
> 반영해 PPT를 수정할 때의 최상위 계약은
> `docs/CLAUDE_PPT_REVIEW_REQUEST_VERIFICATION5B_2026-08-13.md`이다. 특히 learned-v7 threshold
> 분리, pose isolated-RNG, reachability 좌표 교정, 검증 5A의 engineering-only 판정과 5B 미실행
> 상태는 새 문서를 따른다.

기준일: 2026-08-11 KST
프로젝트: **MOTAR — Moving-Object Tracking And RL for UAV navigation in random obstacle fields**

이 파일 하나를 Genspark AI Slides에 첨부한다. 아래 내용은 발표 제작의 유일한 수치 출처다.

## Genspark 제작 프롬프트

첨부 파일을 유일한 수치 출처로 사용해 한국어 연구발표 PPT를 만들어 주세요.

- 16:9, 본문 15장과 부록 3장 이내, 12–15분 발표.
- 로보틱스·강화학습 연구자와 대학원 심사자가 청중이다.
- Professional Mode에 맞는 밝은 배경, navy/cyan 기본색, 충돌과 위험만 orange/red로 강조한다.
- 각 슬라이드 제목은 주제명이 아니라 결론형 문장으로 쓴다.
- 표를 그대로 붙이지 말고 아래 원자료로 차트를 다시 그린다. 축·단위·seed·ID/OOD를 표시한다.
- 모든 차이는 퍼센트가 아니라 percentage point인 `pp`로 표시한다.
- 슬라이드마다 40–70초 분량의 한국어 발표자 노트를 작성한다.
- 파일에 없는 수치·논문 인용·실험 결과를 검색하거나 추정하지 않는다.
- 220 bars는 학습범위 밖 OOD이므로 회색 음영으로 구분한다.
- v1 24×24 m와 v2 40×40 m, legacy 601-action 결과와 schema-v2 exact-600 결과를 직접 합치지 않는다.
- analytic appearance bootstrap을 GT detector 또는 learned detector라고 부르지 않는다.
- learned detector 결과는 현재 synthetic simulator 범위로 한정하고 실세계 비전 해결로 표현하지 않는다.
- 완성 후 편집 가능한 PPTX 구조와 발표자 노트를 제공한다.

### 금지할 과대주장

- “riskcap 이득은 밀도가 높을수록 유의하게 커진다.”
- “추가 1,000 epoch가 고밀도 충돌 문제를 해결했다.”
- “latency는 중요하지 않다.”
- “learned detector가 실제 환경에서도 완벽하다.”
- “v1보다 v2가 몇 배 개선됐다.”
- “220 bars도 학습 범위다.”
- “frozen PPO가 exact-600/time-limit-bootstrap 의미로 학습됐다.”

## 발표의 한 문장 결론

> 센서 전용 이동표적 추격 정책을 밀집 장애물 환경까지 확장한 결과, sensor-only `riskcap`은 학습범위
> capture를 평균 **+5.82 pp** 개선했고, 표적 속도 비용이 장애물 밀도에 의존한다는 상호작용을 확인했다.
> 새 learned detector는 현재 simulator에서 analytic bootstrap과 **−0.07 pp 차이로 비열등성 PASS**했지만,
> 205 bars에서 남은 crash의 **97.2%는 bar contact**이므로 제어·경로 표현과 sim-to-real이 다음 한계다.

## 연구 문제

GT target 좌표를 받지 않는 quadrotor가 onboard RGB-D camera와 LiDAR만 사용해 무작위 막대 장애물 사이의
이동표적을 추적·요격한다. 핵심 질문은 밀도×속도 상호작용, 고밀도 충돌 원인, latency의 실제 비용,
analytic appearance bootstrap을 learned detector로 대체할 수 있는지다.

## 현재 v2 시스템 계약

| 항목 | 값 |
|---|---|
| arena | 40×40×3 m, full-width `navrl_band` placement |
| 평가 밀도 | 130/160/190/205 bars ID, 220 bars OOD |
| target | mixed CV/waypoint, 기본 U[0.3,1.5] m/s |
| LiDAR/camera | 4×72 rays @12 m, obstacle FOV 240° / RGB-D HFOV 87° @20 m |
| actor/critic | sensor-only 898-D actor, asymmetric 906-D critic |
| policy | 17-token temporal Transformer, frozen ep25000 |
| obstacle proposals | 8, `cluster_sector` selector |
| action | 4-D squashed Gaussian `(x,y,z,yaw)` |
| 수평 한계 | x/y 각각 ±2.5 m/s, 요청 XY norm 최대 3.54 m/s |
| z 의미 | altitude PI가 actuator z를 덮지만 raw z는 다음 `prev_action`에 남음 |
| control layer | sensor-only minimum-intervention `riskcap` |
| 평가 | deterministic/original, exact 600 actions, 약 2,049 episodes/cell |

Actor에는 GT target position/velocity/visibility와 semantic target mask가 들어가지 않는다. GT는 detector label,
training-only critic, reward, evaluation diagnostics에만 허용한다.

시스템 흐름도는 `RGB-D + LiDAR → detection → target track → structured history → 17-token Transformer PPO
→ riskcap → quadrotor`로 그린다. GT는 actor와 분리해 `critic/reward/eval only` 점선으로 표시한다.
`riskcap`은 정지 shield가 아니라 위험 command-corridor에서만 수평 요청을 2.0 m/s로 제한하고 자유 공간에서는
최대 3.54 m/s 요청을 허용하는 최소개입 layer다.

## 핵심 감사와 수정 과정

1. **LiDAR chirality:** ray와 bearing table 좌우 부호가 반대였다. 수정 후 token–bar association
   **13.9%→94.8%**, 당시 held-out curve 평균 **+11.1 pp**. 역사적 진단이며 현재 Gate 1과 합치지 않는다.
2. **Token duplication:** 8 tokens가 약 3개 고유 막대만 반복했다. `cluster_sector`로 unique bars/step
   **3.0→4.6**. `4×72 scan→8 proposals→1 history token` 이중 압축은 여전히 한계다.
3. **PPO transaction:** actor·central critic·normalization·optimizer·AMP를 epoch transaction으로 묶었다.
   강제 reject에서 model **93/93 tensors byte-exact**, optimizer 변화 0개를 확인했다.
4. **Horizon/bootstrap:** 과거 `>600`은 action 601에서 끝났고 rl_games용 `time_outs` key가 없어 value
   bootstrap이 빠졌다. 현재 평가는 고쳤지만 frozen policy 학습 lineage는 legacy 601/no-bootstrap다.
5. **Latency pose-time:** 0.1 s 과거 detection을 현재 pose로 변환해 ego motion이 오차가 됐다. 취득 시각
   pose history를 쓰자 capture **37.82%→78.04%**로 회복됐다.

## 확정 결과 1 — governor와 adaptation 분해

seed53, shared source, deterministic exact-600, 약 2,049 episodes/cell.

| bars | A ep24000/off | B ep24000/riskcap | C ep25000/riskcap | B−A | C−B |
|---:|---:|---:|---:|---:|---:|
| 130 | 83.70% | 87.07% | 89.75% | +3.37 pp | +2.68 pp |
| 160 | 79.17% | 85.26% | 86.34% | +6.09 pp | +1.08 pp |
| 190 | 75.07% | 81.16% | 81.75% | +6.09 pp | +0.59 pp |
| 205 | 70.67% | 78.40% | **80.28%** | **+7.73 pp** | +1.88 pp |
| 220 OOD | 66.37% | 75.55% | 77.06% | +9.18 pp | +1.51 pp |

- riskcap ID pooled: **77.15→82.97%, +5.82 pp**, 95% CI **[+4.60,+7.04]**.
- density에 따른 governor gain interaction은 `p=0.358`, heterogeneity `p=0.587`로 미확정이다.
- adaptation ID pooled: **+1.56 pp**, 95% CI **[+0.43,+2.69]**. 고밀도 충돌 해결로 부르지 않는다.

205 bars outcome:

| arm | capture | crash | timeout |
|---|---:|---:|---:|
| A ep24000/off | 70.67% | 26.50% | 2.83% |
| B ep24000/riskcap | 78.40% | 17.70% | 3.90% |
| C ep25000/riskcap | **80.28%** | **17.37%** | **2.34%** |

A→B는 crash **−8.80 pp**, capture **+7.73 pp**다. mean capture time은 11.83→13.08 s로 **+1.24 s**다.
B→C +1.88 pp는 crash −0.32 pp보다 timeout −1.56 pp가 대부분을 설명한다. 최종 C의 205-bars crash
356건 중 346건, **97.2%가 bar contact**다.

필수 차트: 밀도별 A/B/C line chart, 220 OOD 음영, 205 bars outcome stacked bars.

## 확정 결과 2 — density×target-speed interaction

frozen ep25000+riskcap, seed59/61, exact-600, 16 cells.

| bars | 0.3 m/s | 1.5 m/s | fast−slow capture |
|---:|---:|---:|---:|
| 130 | 88.44% | 87.80% | −0.64 pp |
| 160 | 86.42% | 84.41% | −2.00 pp |
| 190 | 83.83% | 79.90% | −3.93 pp |
| 205 | 81.78% | 75.90% | **−5.87 pp** |

사전등록 model `capture ~ seed + density + fast + density:fast`의 LR test는
**χ²(1)=12.7603, p=0.000354**다. 30 bars당 odds multiplier는 **0.890**, 95% CI **[0.835,0.949]**다.

| bars | fast−slow crash | fast−slow timeout | fast−slow capture |
|---:|---:|---:|---:|
| 130 | +3.40 pp | −2.75 pp | −0.64 pp |
| 160 | +3.64 pp | −1.63 pp | −2.00 pp |
| 190 | +5.49 pp | −1.56 pp | −3.93 pp |
| 205 | +6.73 pp | −0.85 pp | −5.87 pp |

빠른 target은 crash를 늘리지만 저밀도에서는 timeout 감소가 이를 상쇄한다. crash interaction `p=0.614`,
timeout interaction `p=0.00192`이므로 단일 물리 메커니즘은 확정하지 않는다. 확정 문장은 “표적 속도의
capture 비용은 학습범위 안에서 장애물 밀도에 의존한다”다.

## 확정 결과 3 — timestamp-aware latency

이 표는 internally consistent legacy seed47 진단이며 Gate 1 exact-600 arm과 합치지 않는다.

| condition | capture | crash | timeout |
|---|---:|---:|---:|
| clean | 80.54% | 17.17% | 2.29% |
| naive 0.1 s/current-pose | 37.82% | 58.22% | 3.95% |
| corrected 0.1 s/capture-time pose | 78.04% | 19.67% | 2.29% |
| corrected 0.2/0.3/0.5 s | 76.62/72.57/64.76% | 20.40/24.79/32.41% | 2.98/2.64/2.83% |

correction은 잘못 모델링한 손실의 **94.2%**를 회수한다. 0.1 s residual은 **−2.50 pp**, 0.5 s는
**−15.77 pp**다. 정확한 sensor timestamp와 capture-time pose history 조건을 붙인다.

## 확정 결과 4 — learned detector offline + navigation gate

Offline detector:

- train/validation/test = **8,192/2,048/4,096 frames**, seeds 71/73/79.
- full bearing/elevation, 2–20 m, absent/occlusion/small target 포함.
- `balanced_bce`, validation threshold **0.55**, preregistered **14/14 PASS**.
- artifact SHA `8da32d6f21bfbd3bdd5ec5de9ef9cb09e8deb4bd5ce511630e19afee33f26f10`.

Navigation A/B는 frozen ep25000+riskcap, 205 bars, exact-600, unused seeds83/89, 4,100 episodes/arm이다.
사전등록 non-inferiority margin은 −2.0 pp다.

| seed | analytic capture/crash/timeout | learned capture/crash/timeout | capture delta |
|---:|---:|---:|---:|
| 83 | 80.39/16.54/3.07% | 79.86/16.92/3.22% | −0.53 pp |
| 89 | 80.59/17.07/2.34% | 80.97/16.11/2.93% | +0.38 pp |

- pooled analytic: **80.49%** (3300/4100), learned: **80.41%** (3297/4100).
- learned−analytic: **−0.07 pp**, 95% CI **[−1.79,+1.64] pp**.
- lower bound −1.79 pp > margin −2.0 pp이므로 **NON-INFERIORITY PASS**.
- pooled crash analytic/learned 16.80/16.51%, timeout 2.71/3.07%는 secondary descriptive다.

해석은 현재 synthetic simulator appearance에 한정한다. target은 fixed pure-red, background/bars는 neutral이라
pixel class가 쉽게 분리된다. 실제 조명·texture·motion blur·noise·calibration 증거가 아니다.

## 음성 결과와 현재 한계

- complete-stop governors: crash 감소 대신 timeout 16.59–23.57%로 기각.
- 추가 1,000 PPO epochs: ID pooled +1.56 pp, collision ceiling 유지.
- LiDAR association off: two-seed +3.25 pp지만 채택 gate +4 pp 미달.
- H4 seen/age: single-seed +2.36 pp, CI가 0을 포함해 exploratory.
- 현재 핵심 한계는 205 bars crash의 **97.2% bar contact**, 이중 token 압축, contact 시 mean executed
  command 약 2.03 m/s, legacy training semantics, 단순 synthetic RGB appearance다.

## 다음 계획

1. Gate 1–3 결과와 SHA/receipt를 publication artifact로 동결하고 같은 PPO의 추가 연장은 하지 않는다.
2. 다음 fresh run은 exact-600 + `time_outs` bootstrap으로 별도 lineage를 만든다.
3. corridor/risk-aware proposal, independent obstacle tokens, 8→12 capacity 또는 learned slots를 한 축씩 ablation한다.
4. predictive shield를 contact-time clearance·stopping margin으로 사전등록 검증한다.
5. texture/lighting/motion blur/depth noise/calibration/clock skew randomization과 real-log replay를 만든다.

## 권장 15장 구성

1. MOTAR: 센서만으로 밀집 장애물 속 이동표적을 요격한다.
2. 밀도·속도·가림이 결합되면 실패 원인이 바뀐다.
3. Actor에서 GT target state를 차단했다.
4. Camera–LiDAR track을 temporal Transformer로 연결했다.
5. v2와 exact-600 계약으로 결과를 다시 측정했다.
6. 좌우 반전과 token 중복이 초기 한계를 만들었다.
7. PPO update를 원자 transaction으로 바꿨다.
8. Riskcap은 위험 구간에서만 속도를 제한한다.
9. Riskcap은 ID capture를 평균 +5.82 pp 높였다.
10. 205 bars 개선은 추가 학습보다 충돌 억제가 만들었다.
11. 표적 속도 비용은 장애물 밀도에 의존한다.
12. 0.1 s latency 대참사는 pose-time 오류였다.
13. Learned detector는 simulator navigation에서 비열등했다.
14. 현재 ceiling은 episode 시간이 아니라 bar contact다.
15. 다음 단계는 fresh semantics와 sim-to-real이다.

부록에는 Gate 1 전체표, Gate 2 outcome표, Gate 3 receipt/SHA와 legacy-vs-schema 사용 규칙을 둔다.

## 내부 근거 파일명

- `results/navrl_v2_governor_adaptation_abc_seed53_schema2/summary.md`
- `results/navrl_v2_speed_density_interaction_seed59_61_schema2/summary.md`
- `results/navrl_v2_latency_budget/summary.md`
- `results/navrl_detector_offline_gate_v2/summary.md`
- `results/navrl_v2_detector_navigation_ab_seed83_89_schema2/summary.md`
- `docs/codex_review_2026-08-10.md`
- `WORKLOG.md`

정확한 서지정보가 없는 외부 논문 인용은 임의 생성하지 않는다. NavRL/NavRL++는 발표자가 참고문헌 정보를
추가하기 전까지 method inspiration으로만 언급한다.
