# NavRL++ 기반 MOTAR 표적 인지·시계열 정책 계획

업데이트: 2026-07-22
상태: **P0 구현 완료 · P1/P2 prototype 통합 및 1-epoch smoke test 완료 · 본 학습/성능 검증 전**

## 1. 연구 문제와 정보 경계

드론은 외부에서 받은 표적 좌표를 따라가면 안 된다. 카메라와 LiDAR로 장애물과 표적을 직접 관측하고,
표적이 잠시 가려지면 과거 관측을 이용해 위치와 운동을 추정해야 한다.

> 센서 입력: RGB-D camera + LiDAR range/point cloud + proprioception
>
> 인지 출력: 정적 장애물 표현, 동적 장애물 track, 표적 track과 불확실도
>
> 정책 입력: 인지가 만든 짧은 이력만 사용하며 GT 표적 상태는 절대 사용하지 않음

시뮬레이터가 렌더링을 위해 world pose를 사용하는 것은 정상이다. 그러나 target semantic mask/id,
정답 bearing/range, `target_position`, GT visibility는 actor 입력에 들어가면 안 된다. 이 값들은 detector
supervision, reward, 종료 판정, critic, 평가 metric에만 사용한다.

## 2. NavRL++에서 실제로 가져와야 할 것

[NavRL++ (arXiv:2605.15559)](https://arxiv.org/abs/2605.15559)은 Transformer를 raw image detector로
사용하지 않는다. 먼저 perception module이 RGB-D/LiDAR를 관측해 구조화된 장애물 상태를 만들고, 그 짧은
이력을 Transformer policy가 통합한다.

### 2.1 논문의 정확한 구성

- Static obstacle: voxel local map에서 36 horizontal × 4 vertical ray distance를 만들고 CNN으로 encoding.
- Dynamic obstacle: 최대 5개의 `[position, velocity, radius]` 상태를 detector, data association, Kalman
  filter로 생성.
- History: 2초 구간을 0.5초 간격으로 표본화한 5개 시점.
- Tokens: `[CLS]` 1 + static obstacle 1 + dynamic obstacle history 5 + robot-state history 5 = 총 12개.
- Transformer: embedding 64, 4 heads, 4 encoder layers, FFN 128, dropout 0.1.
- Heads: `[CLS]`를 actor/critic MLP에 전달하며 전체 policy는 약 0.31M parameters.
- Perturbation-aware fine-tuning: sensor noise, latency, control mismatch와 함께 dynamic-obstacle detection을
  확률 0.3으로 drop하여 false negative를 학습 중 재현.

### 2.2 무엇이 실제로 좋아졌는가

NavRL++의 Transformer가 입증한 직접적인 장점은 **표적 detection AP 향상**이 아니라 **짧은 관측 이력에
의한 시간 추론과 제어 평활화**다. 논문의 intra-simulator ablation에서 improved-training CNN 정책
`NavRL-IT`의 전체 success/control effort는 92.85%/0.093 m/s²이고, Transformer를 넣은
`NavRL-IT-T`는 90.54%/0.043 m/s²이다. 즉 Transformer 단독은 success를 자동으로 올린 것이 아니라
control effort를 절반 이하로 줄였다. Transformer와 perturbation-aware fine-tuning을 함께 적용한 최종
`NavRL++`가 94.08%/0.048 m/s²로 가장 좋은 종합 결과를 냈다.

따라서 본 연구의 1차 구조는 Transformer를 선택 사항으로 두지 않고 **NavRL++식 temporal policy를
주 모델로 채택**한다. CNN/LSTM은 주 모델 후보가 아니라 기여를 증명하기 위한 baseline이다. 동시에
Transformer만 넣으면 표적을 직접 찾게 된다는 잘못된 가정도 하지 않는다. 표적 detector/tracker와
perception-failure fine-tuning이 같이 있어야 한다.

## 3. 우리 문제로의 확장: NavRL++-Target

### 3.1 두 센서가 장애물과 표적을 모두 관측한다

| 센서 | 장애물 관측 | 표적 관측 |
|---|---|---|
| RGB-D camera | depth/U-depth 또는 learned proposal로 3D obstacle box와 local occupancy 갱신 | RGB detector가 identity/2D box/confidence를 추정하고 depth로 3D 위치 복원 |
| LiDAR | point clustering과 voxel mapping으로 3D obstacle box와 static map 갱신 | cluster-level learned target classifier가 target candidate/confidence/centroid를 추정 |
| Fusion | camera/LiDAR obstacle box를 calibration과 uncertainty로 결합 | camera box에 투영되는 LiDAR cluster를 association하고 두 confidence와 3D 위치를 결합 |

LiDAR range만으로 임의 외형의 표적 identity를 자동으로 알 수는 없다. 따라서 LiDAR target branch는
semantic id를 입력받는 것이 아니라 point cluster의 geometry/intensity/motion으로 학습한다. camera는
외형 의미를, LiDAR는 거리·형상·운동을 제공한다. 물리적으로 두 센서의 시야가 모두 완전히 막힌 구간에는
새 측정이 없으므로 Transformer는 마지막 track의 위치·속도·공분산을 이용해 예측하고 uncertainty를
증가시켜야 한다.

### 3.2 Perception front-end

1. RGB-D detector가 obstacle/target proposal과 3D centroid를 만든다.
2. LiDAR clustering + learned point-cluster head가 obstacle/target proposal을 만든다.
3. camera–LiDAR association이 중복 proposal을 결합하되 한 센서만 검출한 관측도 보존한다.
4. data association + Kalman filter가 dynamic obstacle track과 target track을 갱신한다.
5. static obstacle는 voxel map과 ego-centric ray distance로 표현한다.

이 단계에서 GT box/mask/pose는 학습 label일 뿐 추론 입력이 아니다. NavRL++처럼 raw modality 차이를
정책까지 끌고 가지 않고 구조화된 상태로 정렬하므로, 8GB GPU와 sim-to-real 조건에도 더 적합하다.

### 3.3 NavRL++-Target token 설계

기본 sequence는 17 tokens로 고정한다.

- `[CLS]`: 1 token.
- Static obstacle: 현재 local ray-distance map 1 token.
- Dynamic obstacle history: 5 tokens, 각 시점에 최대 N개 obstacle의 position/velocity/radius/confidence.
- Robot history: 5 tokens, 각 시점의 velocity, yaw rate, altitude, previous action.
- Target history: 5 tokens, 각 시점의
  `[existence, relative position, relative velocity, size, camera confidence, LiDAR confidence,
  covariance, measurement age]`.

NavRL++에 맞춰 1차 설정은 history 2초/0.5초, dim 64, 4 heads, 4 layers, FFN 128, dropout 0.1로 둔다.
0.1초 관측을 모두 token으로 늘리지 않고 tracker는 10 Hz로 갱신하되 policy history는 2 Hz로 sampling한다.
`[CLS]`에서 target state, uncertainty, obstacle latent와 velocity action을 예측한다. 원시 camera/LiDAR patch
token을 직접 attention하는 기존 아이디어는 1차 구조가 아니라 후속 ablation으로 내린다.

### 3.4 정책과 안전 동작

- Actor 입력: 위 17-token sequence 또는 `[CLS]` latent만 허용.
- Actor 금지: GT target position/velocity/visibility와 target semantic channel.
- Critic privileged state는 actor storage와 물리적으로 분리한다.
- target confidence가 낮거나 measurement age가 길면 pursuit 속도를 낮추고, 안전 회피와 search yaw를 수행.
- target도 collision geometry에는 포함해 추격 중 표적/장애물과의 안전거리를 유지.

## 4. 학습 순서

### P0 — 정보 누출 방화벽

- actor observation에서 semantic/analytic target vector와 GT 표적 상태 제거.
- blind 조건에서 GT 표적 상태만 바꿔도 actor 입력이 변하지 않는 자동 테스트 추가.
- `sensor_obs`, `perception_labels`, `critic_privileged_obs`를 별도 API/storage로 분리.

### P1 — 카메라·LiDAR detector와 tracker

- 실제 target mesh/appearance가 RGB-D와 LiDAR point return에 모두 나타나게 한다.
- RGB-D obstacle/target detector, LiDAR obstacle/target cluster classifier를 각각 학습한다.
- fusion, data association, Kalman filter로 structured state와 confidence/covariance를 출력한다.
- split은 frame random이 아니라 arena layout, trajectory, target appearance 단위로 나눈다.

### P2 — NavRL++-style temporal policy pretraining

- frozen perception output 또는 label에 현실적인 noise를 넣은 structured state로 17-token policy를 학습.
- single-step CNN, CNN+LSTM, NavRL++-Target Transformer를 동일 관측/학습 조건에서 비교.
- 먼저 imitation/auxiliary loss로 target existence/position/velocity/uncertainty를 안정화한 뒤 PPO를 연결.

### P3 — Perturbation-aware fine-tuning

- clean pretraining 뒤 camera-only miss, LiDAR-only miss, simultaneous miss를 따로 randomize.
- NavRL++ 기준 `p_drop=0.3`을 시작점으로 `{0.1, 0.3, 0.5}` ablation.
- sensor noise, timestamp jitter, observation/action latency, calibration error, control response mismatch 추가.
- partial/full occlusion 길이와 obstacle density를 curriculum으로 증가.

### P4 — Perception-in-the-loop PPO

- detector/tracker를 먼저 freeze하고 PPO를 학습한다.
- 안정화 후 token projection과 마지막 Transformer block만 낮은 learning rate로 fine-tuning.
- GT-state policy는 성능 상한선일 뿐 제안 방법이나 배포 정책으로 보고하지 않는다.

### P5 — 일반화와 sim-to-real 준비

- unseen layout, target appearance/shape, speed, lighting, sensor dropout을 평가.
- 실제 RGB-D/LiDAR log가 생기면 detector → tracker → policy 순으로 오류를 분해해 zero-shot 평가.

## 5. 필수 baseline과 ablation

| 비교 | 목적 |
|---|---|
| GT-state policy | 제어 성능 상한선; 제안법 아님 |
| camera-only detector/policy | appearance와 RGB-D 기여도 |
| LiDAR-only detector/policy | geometry/motion만으로 가능한 수준 |
| single-step structured CNN | 시간 이력 기여도 |
| structured CNN+LSTM | Transformer temporal reasoning 기여도 |
| NavRL++-Target Transformer | 제안 모델 |
| detector without tracker | tracking/Kalman 기여도 |
| no PF / target-drop PF | perception-failure fine-tuning 기여도 |
| raw-token cross-attention | 구조화 상태 대비 추가 이득/비용을 보는 후속 ablation |

주 지표:

- Perception: camera/LiDAR별 precision/recall, fused relative-position/velocity RMSE, ID switches,
  NLL calibration, measurement age별 track error.
- Occlusion: 가림 길이별 track survival, prediction error, reacquisition time.
- Navigation: capture, collision, timeout, control effort, time-to-capture, minimum clearance.
- 시스템: detector/tracker/policy별 latency, FPS, VRAM, training throughput.

## 6. 구현 게이트

- G0: actor 경로에 GT/semantic target channel 참조 0개.
- G1: camera-only와 LiDAR-only가 각각 visible target에 대해 chance보다 유의하게 높은 recall을 보임.
- G2: fused visible-target median position error ≤0.30 m at 10 m.
- G3: 1초 simultaneous sensor miss 뒤 track survival/reacquisition ≥80%.
- G4: Transformer가 single-step CNN보다 control effort와 occlusion robustness를 개선하고, LSTM 대비
  accuracy–smoothness–latency Pareto 우위를 하나 이상 보임.
- G5: held-out layout에서 GT upper bound와 capture gap을 perception/tracking/policy error로 분해.

## 7. 현재 구현 상태와 다음 gate

구현된 새 경로는 `NAVRL_VISION=1 NAVRL_PERCEPTION=1`로 켠다.

- `navrl_detector.py`: renderer-private label로 RGB-D 센서 frame만 만들며 mask/semantic은 actor에 반환하지 않음.
- `navrl_perception.py`: RGB-D appearance detector, camera–LiDAR range association, constant-velocity Kalman
  tracker, covariance/confidence/age, static map과 장애물 proposal, 2초 history를 생성.
- `navrl_transformer_network.py`: 계획대로 17 token, dim 64, 4 heads, 4 layers, FFN 128을 구현.
- `ppo_navrl_perception_transformer.yaml`: perception-in-the-loop PPO와 train-time privileged critic을 분리.
- 자동 검증: perception API에 GT/semantic 인자가 없음을 검사하고, RGB-D detection, LiDAR continuation,
  occlusion 중 covariance 증가를 단위 테스트함. 64 env/8 bars/1 epoch end-to-end smoke test도 통과.

아직 완료되지 않은 것은 **학습된 detector checkpoint의 실제 precision/recall과 새 Transformer의 navigation
성능**이다. 현재 appearance head는 red-target bootstrap initialization이며 교체 가능한 checkpoint interface를
갖는다. 따라서 현 단계 결과를 최종 learned perception 성능으로 보고하면 안 된다. 다음 gate는 detector
dataset/held-out split을 만들고 G1/G2를 수치로 통과한 뒤 perception-in-the-loop PPO 본 학습을 시작하는 것이다.

## 8. 고밀도와 저속에 대한 geometry audit

`tools/analyze_navrl_density_feasibility.py`는 실제 bar URDF 폭, 0.28 m 기체 footprint, 현재 random-rejection
spacing 완화를 재현한다. 50-layout probe에서 side clearance 0.2 m를 요구해도 130 bars는 100%, 150 bars는
92%가 spawn-to-goal 연결 경로를 가졌다. 즉 현재 고밀도 task는 대부분 **물리적으로 불가능해서** 실패하는
것이 아니다. 다만 130/150 bars에서는 모든 layout이 spacing 완화에 들어가고, 150 bars의 8%는 0.2 m
안전 마진 경로가 사라졌다.

표적 속도 0 m/s인 기존 held-out baseline도 고밀도에서 crash가 컸으므로 표적을 느리게 하는 것만으로는
해결되지 않는다. pursuer 자체를 느리게 하면 정지거리(가속 한계 2 m/s², latency 0.1 s 가정)가 2 m/s의
1.20 m에서 1 m/s의 0.35 m로 줄어 충돌은 감소할 가능성이 크지만, 30초 episode에서 24 m를 통과해야 해
timeout이 증가한다. 새 policy 학습 후 `eval_navrl_speed_density_grid.sh`로 target speed와 pursuer limit을
분리해 측정한다.

## 9. 권장 curriculum과 3-D 검증 앱

거리 6000 epoch 뒤 밀도 6000 epoch를 무조건 수행하거나, 두 난이도를 처음부터 함께 선형 증가시키지 않는다.
현재 권장안은 `train_navrl_perception_staged.sh`의 다음 구조다.

- 0–4000 epoch: 25 bars로 고정하고 goal distance를 7→16 m로 확장한다.
- 4000 epoch 이후: 거리는 최종 범위로 유지하고, capture≥0.65/4096 episodes일 때만 막대를 5개씩
  25→110으로 승급한다.
- density 단계 reset의 25%는 5–10 m goal을 재생해 가까운 거리의 회피·재탐지 능력 망각을 막는다.
- 이 clean/static-target run을 통과한 뒤에만 perception dropout/noise, 마지막으로 target speed를 추가한다.

이는 “두 축을 영원히 완전히 분리”하는 방식이 아니라, 첫 기술을 얻은 뒤 다음 축을 competence-gated로
추가하고 이전 분포 일부를 계속 replay하는 staged curriculum이다. 총 10000 epoch는 상한이며, density 승급이
멈추면 epoch를 더 쓰는 대신 detector error/collision/timeout 원인을 먼저 분해한다.

`launch_navrl_3d.sh`는 실제 Isaac Gym/PhysX/Warp sensor와 같은 task tensor를 사용하는 독립 실행형 3-D 앱이다.
막대 수, 표적 속도, 드론 속도를 실행 중 바꿀 수 있고 policy/manual control, reset, LiDAR ray overlay를 제공한다.
빨간 target wireframe은 사람용 GT 디버그 표시이며 perception/actor input과 물리적으로 분리된다.

## 10. 보조 설계 근거

- [NavRL++ (2026)](https://arxiv.org/abs/2605.15559): structured perception, 2초 history,
  Transformer policy, perception-failure fine-tuning의 직접 기준.
- [NavRL code](https://github.com/Zhefan-Xu/NavRL): 기존 NavRL training/deployment 구현 기준.
- [GAFusion, CVPR 2024](https://openaccess.thecvf.com/content/CVPR2024/html/Li_GAFusion_Adaptive_Fusing_LiDAR_and_Camera_with_Multiple_Guidance_for_CVPR_2024_paper.html): LiDAR–camera adaptive fusion.
- [ORTrack, CVPR 2025](https://openaccess.thecvf.com/content/CVPR2025/html/Wu_Learning_Occlusion-Robust_Vision_Transformers_for_Real-Time_UAV_Tracking_CVPR_2025_paper.html): UAV occlusion-robust tracking.
- [TransFusion, CVPR 2022](https://openaccess.thecvf.com/content/CVPR2022/html/Bai_TransFusion_Robust_LiDAR-Camera_Fusion_for_3D_Object_Detection_With_Transformers_CVPR_2022_paper.html): object-query 기반 camera–LiDAR fusion 비교안.
