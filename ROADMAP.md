# MOTAR 구현 로드맵

업데이트: 2026-07-22
우선순위: **원시 센서 기반 표적 탐지 → 시계열 융합 → perception-in-the-loop navigation**

## 0. 방향 전환

이전 로드맵은 이동하는 `target_position`을 정책 관측에 직접 넣거나 semantic detector로 변환하는
경로를 Phase 3의 중심으로 두었다. 이 방식은 제어 upper bound로만 남긴다. 앞으로의 제안 방법은
camera와 LiDAR 원시 관측에서 target state를 학습해 추정한다.

권위 문서:

- 연구 전체: [`RESEARCH_PLAN.md`](RESEARCH_PLAN.md)
- 모델·데이터·loss·ablation: [`PERCEPTION_TRANSFORMER_PLAN.md`](PERCEPTION_TRANSFORMER_PLAN.md)
- Phase 3 실행표: [`PHASE3_PLAN.md`](PHASE3_PLAN.md)

## 1. 완료 기반

| 단계 | 산출물 | 상태 |
|---|---|---|
| B0 | Aerial Gym/Isaac Gym/Warp/rl_games 환경 | 완료 |
| B1 | random bar arena, 36×4 LiDAR, capture task | 완료 |
| B2 | learned yaw obstacle navigation | 완료 |
| B3 | density sweep와 curriculum infrastructure | 완료 |
| B4 | scripted moving target와 interception reward | 완료 |
| B5 | camera/LiDAR occlusion·throughput prototype | 완료, 단 oracle semantic prototype |

## 2. 새 Critical Path

### P0 — Information firewall

**상태: 구현·자동 테스트 완료.**

- actor에서 target semantic mask/id, GT detector vector, target position/velocity 제거.
- `sensor_obs`와 `supervision_labels` storage/API 분리.
- actor/critic observation schema와 blind-target equality test 추가.
- **완료 게이트:** actor 호출 경로에서 `target_position` 참조 0개.

### P1 — Physical target and raw sensing

**상태: analytic geometry raw-return backend + RGB-D appearance frame 구현 완료. 실제 moving-mesh/refit
benchmark와 appearance 다양화는 미완료.**

- 실제 target mesh/appearance asset을 arena에 배치하고 scripted trajectory로 이동.
- camera RGB-D에는 외형/깊이, LiDAR에는 표면 return이 나타나도록 렌더링.
- moving geometry 갱신의 Warp refit 비용을 benchmark하고, 필요하면 target 전용 analytic intersection은
  raw range 생성에만 사용한다. semantic identity는 actor에 제공하지 않는다.
- **완료 게이트:** target 존재/부재가 raw camera와 raw LiDAR를 변화시키며 label tensor 없이 식별 학습 가능.

### P2 — Dataset, dual-sensor detection, and tracking

**상태: appearance bootstrap, camera–LiDAR association, Kalman covariance prototype 구현 완료.
dataset recorder·learned checkpoint·held-out metric은 미완료.**

- sequence recorder: RGB-D, LiDAR, odometry, timestamp, label-only GT pose/bbox/mask/visibility.
- split: unseen layout + unseen trajectory + unseen appearance.
- RGB-D obstacle/target detector, LiDAR clustering + learned obstacle/target classifier.
- calibration association, data association, Kalman filter로 structured obstacle/target tracks 생성.
- **완료 게이트:** sensor별 target recall 보고; fused 10 m median relative-position error ≤0.30 m.

### P3 — NavRL++-Target temporal Transformer

**상태: 17-token network와 PPO config 구현, 1-epoch smoke test 완료. 본 학습/ablation은 미실행.**

- 17 tokens: `[CLS]` 1 + static 1 + dynamic history 5 + robot history 5 + target history 5.
- NavRL++ 기준 2초/0.5초 history, dim 64, 4 heads, 4 layers, FFN 128, dropout 0.1.
- outputs: velocity action + target existence/position/velocity/uncertainty auxiliary heads.
- `p_drop=0.3`에서 시작하는 camera/LiDAR perception-failure fine-tuning과 latency/noise curriculum.
- **완료 게이트:** 1초 full miss 뒤 survival/reacquisition ≥80%; CNN보다 smooth/robust하고 LSTM 대비
  Pareto 이점 확인.

### P4 — Navigation integration

- detector/tracker frozen PPO → confidence-aware search/pursuit policy.
- 안정화 후 마지막 Transformer block만 낮은 LR로 joint fine-tuning.
- **완료 게이트:** unseen layout에서 perception policy가 GT upper bound와 비교 가능한 capture를 보이며,
  failure를 perception/control로 분해할 수 있음.

### P5 — Core experimental matrix

- obstacle density 5 × target speed 5 × occlusion bins.
- camera-only/LiDAR-only/both, CNN/LSTM/Transformer, history/uncertainty ablation.
- ≥3 training seeds, ≥1,000 evaluation episodes/cell, 95% CI.
- **완료 게이트:** 논문 대표 heatmap과 ablation table 생성.

### P6 — Sim-to-real and paper

- latency/FPS/VRAM, calibration perturbation, sensor dropout.
- real log가 있으면 detector zero-shot/fine-tune; 없으면 limitation을 명시.
- RA-L/ICRA/IROS 원고 작성.

## 3. 구현 단위

| 패키지/파일 | 예정 변경 |
|---|---|
| `navrl_task.py` | raw sensor/label 경계, perception output을 actor에 연결 |
| `navrl_detector.py` | oracle detector에서 raw render/learned-model adapter로 역할 변경 |
| `navrl_lidar_config.py` | semantic ID는 renderer 내부에만 유지; actor는 raw nearest range만 사용 |
| `navrl_task_config.py` | detector/tracker, 2초 history, PF, Transformer 설정 추가 |
| `motar_perception_dataset.py` | 신규 dual-sensor sequence recorder/dataset |
| `navrl_perception.py` | RGB-D appearance head + LiDAR association + Kalman tracker + structured history |
| `navrl_transformer_network.py` | NavRL++-style 17-token temporal actor–critic |
| `tests/test_navrl_perception.py` | GT-free API, RGB-D detection, LiDAR continuation, covariance 테스트 |

## 4. 실험 순서와 예상 기간

| 기간 | 작업 | 의사결정 |
|---|---|---|
| D1–D3 | firewall, asset/render, recorder | raw sensor가 충분한가? |
| D4–D7 | camera/LiDAR detector + association/tracker | 각 센서가 target을 직접 찾는가? |
| W2 | NavRL++ tokenization + CNN/LSTM baseline | 2초 structured history가 충분한가? |
| W3 | Transformer + PF/occlusion curriculum | smoothness와 robustness가 함께 좋아지는가? |
| W4 | PPO integration | perception error가 제어에 어떻게 전파되는가? |
| W5–W6 | multi-seed matrix, paper figures | 핵심 claim 유지 여부 |

## 5. 체크포인트 규칙

- old GT/semantic vision checkpoint는 새 raw-perception 모델과 호환되지 않는다.
- navigation-only backbone은 transfer 후보지만 observation layer는 새로 초기화한다.
- perception, policy, optimizer, dataset/config hash를 한 checkpoint manifest에 기록한다.
- `gen_ppo.pth`만 보관하지 말고 detector validation-best와 navigation capture-best를 별도로 보관한다.

## 6. 현재 다음 행동

P0와 sensor-to-track/Transformer prototype은 통합됐다. 지금 다음 행동은 RGB-D/LiDAR sequence recorder와
held-out split을 만들고 bootstrap detector를 학습 checkpoint로 교체하여 G1/G2를 수치로 통과시키는 것이다.
그 전에는 1-epoch smoke test를 성능 결과로 해석하거나 대규모 PPO 학습을 시작하지 않는다.
