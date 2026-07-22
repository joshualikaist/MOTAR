# Phase 3 실행 계획 — 원시 센서 기반 표적 탐지·추적·접근

업데이트: 2026-07-22
상세 설계: [`PERCEPTION_TRANSFORMER_PLAN.md`](PERCEPTION_TRANSFORMER_PLAN.md)

## 핵심 변경 — NavRL++-Target

이전 Phase 3은 task가 가진 `target_position`을 관측 또는 analytic detector로 변환해 정책에 제공했다.
이 경로는 더 이상 논문의 제안 방법으로 사용하지 않는다. Phase 3의 핵심은 드론이 camera와 LiDAR의
원시 관측으로 표적을 직접 탐지하고, 장애물 가림 동안 NavRL++식 시계열 정책으로 위치와 운동을
추론하는 것이다. Transformer는 주 모델이며 LSTM은 기여 검증용 baseline이다.

NavRL++ 원문에서 Transformer는 raw RGB/point detector가 아니다. perception module이 static map과
dynamic track을 먼저 만들고, 2초 이력을 12 tokens로 통합해 관측 열화와 제어 진동을 줄인다. 본 Phase는
그 구조에 learned target detector/tracker와 target-history 5 tokens를 추가한 17-token 정책을 구현한다.

## 연구 가설

- H1: camera와 LiDAR의 결합은 어느 한 센서만 사용할 때보다 표적 위치 RMSE와 capture를 개선한다.
- H2: temporal memory는 완전 가림 뒤 재탐지 시간과 track survival을 개선한다.
- H3: NavRL++식 Transformer+perception-failure fine-tuning은 single-step CNN보다 가림 견고성과 제어
  평활도가 높고, LSTM보다 accuracy–smoothness–latency Pareto 우위를 보인다.
- H4: uncertainty-aware policy는 마지막 추정 위치로 돌진하는 정책보다 충돌률이 낮다.
- H5: Transformer의 이점은 저밀도·항상-visible 조건보다 고밀도·긴 가림 조건에서 커진다.

## 고정 정보 경계

| 데이터 | Perception 입력 | Actor 직접 입력 | Label/Reward/Critic/Eval |
|---|---:|---:|---:|
| RGB-D camera | 예 | 아니오 | — |
| LiDAR range/points | 예 | 아니오 | — |
| structured obstacle/target history | 출력 | **예** | — |
| proprioception | tracker 보조 | **예** | — |
| target semantic mask/id | **아니오** | **아니오** | label/eval만 |
| GT target position/velocity | **아니오** | **아니오** | label/reward/critic/eval |
| GT visibility/occlusion | **아니오** | **아니오** | label/eval만 |

## 실행 순서

1. 정보 누출 테스트와 sensor/label tensor 분리.
2. 실제 target geometry가 camera와 LiDAR에 나타나는 데이터 생성 환경 구축.
3. RGB-D obstacle/target detector와 LiDAR obstacle/target cluster classifier 학습.
4. fusion + data association + Kalman tracker로 structured state 생성.
5. NavRL++ 기준 2초/0.5초 이력의 17-token Transformer 학습.
6. detector/tracker freeze 상태에서 PPO 학습.
7. `p_drop=0.3` 중심의 perception-failure/latency/noise fine-tuning.
8. density × speed × occlusion 평가와 CNN/LSTM/Transformer ablation.

## 1차 모델 크기

- Camera: RGB-D 160×96 detector → obstacle/target 3D proposals.
- LiDAR: 36×4 또는 point clusters → obstacle/target proposals; 72×6은 해상도 ablation.
- Tracker: 10 Hz update, policy history는 2초 동안 0.5초 간격 5 samples.
- Tokens: `[CLS]` 1 + static 1 + dynamic history 5 + robot history 5 + target history 5 = 17.
- Transformer: NavRL++ 기준 dim 64, 4 heads, 4 layers, FFN 128, dropout 0.1.
- Output: velocity action과 target probability/position/velocity/uncertainty auxiliary heads.

## 4주 계획

| 주차 | 산출물 | 통과 기준 |
|---|---|---|
| W1 | 누출 방화벽 + target asset + sensor dataset | blind-GT equality; 두 센서 raw target return 확인 |
| W2 | 양 센서 detector + fusion/tracker | sensor별 recall 보고; fused 10 m median error ≤0.30 m |
| W3 | NavRL++-Target Transformer + PF curriculum | 1초 동시 miss 뒤 track survival/reacquisition ≥80% |
| W4 | PPO 연결 + ablation | GT/CNN/LSTM/Transformer와 no-PF 비교표, density×speed heatmap |

## 중단 조건

- 어느 한 센서 detector도 visible target을 못 찾으면 Transformer를 키우지 말고 sensor/asset/data부터 수정한다.
- Transformer 평가는 success만 보지 않고 NavRL++처럼 control effort와 perturbation robustness를 함께 본다.
- 8GB에서 OOM이면 raw image를 policy에 넣지 않고 detector output을 detach/cache하고 env를 256→128로 낮춘다.

## 논문 핵심 주장 후보

> Extending NavRL++ temporal reasoning with directly observed camera–LiDAR target tracks and
> perception-failure-aware fine-tuning enables a UAV to pursue a temporarily occluded target without
> privileged target-state input while maintaining collision avoidance and smooth control.
