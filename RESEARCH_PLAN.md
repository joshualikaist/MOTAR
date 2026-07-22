# MOTAR 연구 계획서

업데이트: 2026-07-22
연구 제목: **장애물 환경에서 원시 카메라–LiDAR 관측으로 이동 표적을 탐지·추적·접근하는 UAV**

## 1. 연구 질문

| RQ | 질문 |
|---|---|
| RQ1 | 장애물 밀도와 표적 속도가 원시 센서 기반 표적 탐지 및 접근 성능에 어떻게 상호작용하는가? |
| RQ2 | 가림 지속시간이 표적 위치추정과 재탐지, 최종 capture에 어떤 영향을 주는가? |
| RQ3 | NavRL++식 temporal Transformer와 perception-failure fine-tuning이 표적 가림과 제어 진동을 얼마나 줄이는가? |
| RQ4 | 위치 불확실도를 인지하는 정책이 가림 중 충돌과 잘못된 추격을 줄이는가? |

## 2. 기대 기여

1. NavRL++의 structured temporal policy를 직접 관측한 camera–LiDAR target track으로 확장하는
   17-token Transformer architecture.
2. perception error가 navigation failure로 전파되는 과정을 density × target speed × occlusion 축에서 분석.
3. camera-only, LiDAR-only, tracking/no-tracking, CNN, LSTM, Transformer, PF/no-PF의 통제된 비교.
4. RTX 3070 8GB에서 재현 가능한 simulator/data/training/evaluation pipeline.

## 3. 절대 원칙: actor에 정답을 주지 않는다

- `target_position`, GT bearing/range/velocity, target semantic id/mask는 actor 관측 금지.
- 시뮬레이터 GT는 렌더링, detector supervision, reward, done, critic, metric에만 사용.
- target은 camera RGB-D와 LiDAR 표면 return에 실제로 나타나야 한다.
- actor는 learned perception이 만든 structured obstacle/target history만 사용한다.
- GT-state policy는 upper-bound baseline이며 제안 방법으로 표현하지 않는다.

자세한 tensor 경계와 모델은 [`PERCEPTION_TRANSFORMER_PLAN.md`](PERCEPTION_TRANSFORMER_PLAN.md)를 따른다.

## 4. 현재까지 확보한 기반

- 24×24×3 m random bar arena와 36×4 Warp LiDAR.
- learned yaw를 포함한 정지표적 항법 baseline과 obstacle-density curriculum.
- scripted moving target, range-rate reward, capture/timeout/crash metrics.
- camera target occlusion raycast와 camera obstacle depth prototype.
- 256 environments에서 prototype sensor pipeline 약 8,180 env-steps/s 확인.

단, analytic target sphere, semantic LiDAR id 50, semantic camera target mask를 actor에 제공하는 현재
vision 코드는 **센서 프로토타입**이지 최종 연구 방법이 아니다.

## 5. 새 단계별 로드맵

### Phase 0–2 — 기존 항법 기반 및 밀도 실험

- 상태: 완료/진행 결과를 historical baseline으로 유지.
- 용도: obstacle avoidance backbone, density curriculum, GT upper-bound control 성능 제공.
- 이전 체크포인트는 새 perception actor와 입력 차원이 달라 직접 비교·재개하지 않는다.

### Phase 3A — 정보 누출 제거와 실제 센서 생성

- target semantic channel을 actor에서 제거한다.
- target visual/geometry asset을 camera와 LiDAR가 직접 렌더링하게 한다.
- RGB-D/LiDAR sequence와 label-only GT pose/bbox/mask/visibility를 기록한다.
- layout/trajectory 단위 train/validation/test split을 고정한다.

완료 기준: GT를 바꿔도 raw sensor가 동일한 blind 조건에서는 actor 입력도 동일하며, visible target은 두
센서의 raw return을 실제로 변화시킨다.

### Phase 3B — Learned detector baseline

- RGB-D는 obstacle/target proposal과 identity/confidence를 직접 검출한다.
- LiDAR는 clustering 뒤 learned cluster classifier로 obstacle/target proposal을 직접 검출한다.
- calibration association + Kalman tracker가 body-frame position/velocity/confidence/covariance를 출력한다.
- visible target localization, false positive, range별 recall을 먼저 검증한다.

완료 기준: 10 m visible target median position error ≤0.30 m.

### Phase 3C — NavRL++-Target temporal policy

- NavRL++ 기준 `[CLS]+static+dynamic-history+robot-history`에 target-history 5 tokens를 추가한다.
- 2초/0.5초 이력, dim 64, 4 heads, 4 layers, FFN 128을 1차 설정으로 고정한다.
- camera/LiDAR detection drop, occlusion, latency, noise를 넣는 perturbation-aware fine-tuning.
- history={0,1,2,4}s, camera/LiDAR/both, no-tracker/no-PF ablation.

완료 기준: 1초 full occlusion 뒤 track survival/reacquisition ≥80%, single-step CNN 대비 control effort와
robustness 개선, LSTM 대비 accuracy–smoothness–latency Pareto 우위.

### Phase 3D — Perception-in-the-loop navigation

- detector freeze → PPO 학습 → 필요 시 마지막 fusion block만 공동 fine-tuning.
- confidence가 낮으면 안전 탐색, 높으면 pursuit/interception.
- GT upper bound와 perception policy gap을 perception/control error로 분해한다.

### Phase 4 — 핵심 실험

- density: bars {25,50,75,110,150}.
- target speed: {0,0.5,1.0,1.5,2.0} m/s; 2.0은 평가 전용.
- occlusion duration: {0,0.25,0.5,1.0,2.0}s 또는 관측된 연속 가림 bin.
- trajectory: CV/waypoint/circle, held-out layout과 appearance.
- 조건당 ≥1,000 episodes, 학습 seed ≥3.

### Phase 5 — 논문화와 실기 준비

- perception: precision/recall, position/velocity RMSE, NLL calibration, reacquisition time.
- navigation: capture/crash/timeout/time-to-capture/minimum clearance.
- system: latency/FPS/VRAM/env-steps/s.
- 실제 센서 로그가 확보되면 perception-only zero-shot와 calibration fine-tuning을 추가한다.

## 6. 모델 비교

| 모델 | Raw sensors | Temporal | Cross-modal attention | 역할 |
|---|---:|---:|---:|---|
| GT state | 아니오 | — | — | upper bound only |
| camera CNN | camera | 아니오 | 아니오 | detector baseline |
| LiDAR temporal CNN | LiDAR | 예 | 아니오 | geometry/motion baseline |
| fused detector + tracker | 둘 다 | 예 | 아니오 | structured perception front-end |
| single-step CNN policy | 둘 다 | 아니오 | 아니오 | temporal ablation |
| CNN+LSTM policy | 둘 다 | 예 | 아니오 | recurrent baseline |
| NavRL++-Target | 둘 다 | 예, 2초 | 예 | proposed |

## 7. 리스크

| 리스크 | 대응 |
|---|---|
| sparse LiDAR로 target/obstacle 구분 불가 | 시간차 motion feature, target geometry, 72×6 ablation; camera와 fusion |
| Transformer가 작은 데이터에서 과적합 | structured 17 tokens, NavRL++ dim 64, detector pretraining, strong split |
| 8GB OOM | detector offline pretraining, structured 17 tokens, gradient accumulation, env 128 fallback |
| detector 오류가 PPO를 붕괴 | perception freeze, confidence input, oracle/noise curriculum |
| semantic label 누출 | 별도 label dict/dataloader, actor observation schema test, code review gate |
| sim-to-real gap | texture/light/noise/dropout randomization, calibration perturbation, real-log validation |

## 8. 당장 할 일

1. 현재 semantic target actor channel을 제거하고 sensor/label API를 분리한다.
2. physical/visual target asset과 RGB-D capture 경로를 결정한다.
3. sequence recorder와 dataset schema를 만든다.
4. camera-only와 LiDAR-only detector를 각각 성공시킨 뒤 fusion/tracker를 고정한다.
5. 그 structured history에 NavRL++-Target Transformer와 perturbation-aware fine-tuning을 적용한다.
