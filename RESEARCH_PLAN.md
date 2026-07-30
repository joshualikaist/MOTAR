# RESEARCH PLAN — 센서 전용 UAV 요격: 밀도 × 표적속도

이 문서가 연구 charter다(가설·설계·계획). **현재 상태와 최신 수치는 여기 적지 않는다** —
진행 기록은 `WORKLOG.md`(맨 아래가 최신), 라이브 지표는 `docs/status/`를 본다.
실무(머신 셋업·GPU·이관)는 `OPERATIONS.md`, 진단 도구와 측정된 음성 결과는 `CRASH_TUNING_LOG.md`.

## 1. 연구 질문과 기여

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

## 2. 정보 방화벽 — 무엇이 actor에 들어갈 수 있는가

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

## 3. 사전등록 가설

## 연구 가설

- H1: camera와 LiDAR의 결합은 어느 한 센서만 사용할 때보다 표적 위치 RMSE와 capture를 개선한다.
- H2: temporal memory는 완전 가림 뒤 재탐지 시간과 track survival을 개선한다.
- H3: NavRL++식 Transformer+perception-failure fine-tuning은 single-step CNN보다 가림 견고성과 제어
  평활도가 높고, LSTM보다 accuracy–smoothness–latency Pareto 우위를 보인다.
- H4: uncertainty-aware policy는 마지막 추정 위치로 돌진하는 정책보다 충돌률이 낮다.
- H5: Transformer의 이점은 저밀도·항상-visible 조건보다 고밀도·긴 가림 조건에서 커진다.

논문 주장 한 문장:

## 논문 핵심 주장 후보

> Extending NavRL++ temporal reasoning with directly observed camera–LiDAR target tracks and
> perception-failure-aware fine-tuning enables a UAV to pursue a temporarily occluded target without
> privileged target-state input while maintaining collision avoidance and smooth control.

---

## 4. 방법 — NavRL++식 인지 + Transformer 정책

### 4.1 원논문 ablation 근거

### 2.2 무엇이 실제로 좋아졌는가

NavRL++의 Transformer가 입증한 직접적인 장점은 **표적 detection AP 향상**이 아니라 **짧은 관측 이력에
의한 시간 추론과 제어 평활화**다. 논문의 intra-simulator ablation에서 improved-training CNN 정책
`NavRL-IT`의 전체 success/control effort는 92.85%/0.093 m/s²이고, Transformer를 넣은
`NavRL-IT-T`는 90.54%/0.043 m/s²이다. 즉 Transformer 단독은 success를 자동으로 올린 것이 아니라
control effort를 절반 이하로 줄였다. Transformer와 perturbation-aware fine-tuning을 함께 적용한 최종
`NavRL++`가 94.08%/0.048 m/s²로 가장 좋은 종합 결과를 냈다.

### 4.2 17-token 사양

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


### 4.3 토큰 표현의 이중 압축 문제 (열린 설계 논점)

- 여기서 “8 obstacle tokens”는 Transformer에 들어가는 독립 토큰 8개가 아니다. 구현은
  각 시점의 8×12 obstacle proposal을 이어 붙여 MLP로 64차원 **단일 history token**으로
  압축한다. 따라서 실제 병목은 `4×72 scan → 8 surface proposals`의 중복 선택과
  `8 proposals → 1 temporal token`의 이중 압축이다.
- 원 NavRL++는 static geometry를 ray-distance CNN token으로 유지하고, 최대 5개 object
  slot은 position/velocity/radius가 추적되는 dynamic obstacle에 사용한다. 현재 arena의
  정적 막대를 velocity=0인 obstacle-history slot에 넣는 것은 원 설계와 다른 확장이며,
  suppression 폭만 조절하기 전에 이 정적/동적 역할 혼합을 ablation해야 한다.

후속 방향:

4. hard selector가 부족하면 72개 bearing feature를 8개 latent slot으로 압축하는
   Set Transformer/Slot Attention 후보를 시험한다. PPO 보상만으로 slot collapse를
   맡기지 않고, simulator GT는 actor 입력이 아닌 training-only Hungarian auxiliary
   target으로만 사용한다. 85개 중 보이는 모든 막대를 8 slot에 매칭할 수 없으므로
   TTC/goal corridor로 정의한 top-risk subset만 matching한다.
5. 독립적인 8 obstacle token을 Transformer에 직접 넣는 ablation과 8→12 capacity 증가는
   selection이 개선된 뒤 수행한다. 둘 다 observation/network shape가 바뀌어 fresh training이
   필요하므로 첫 실험으로 쓰지 않는다.

---

## 5. 실행 순서 (단일 번호 체계 P0–P6)

`ROADMAP`/`PHASE3_PLAN`/`PERCEPTION_TRANSFORMER_PLAN`에 세 가지 다른 번호가 붙어 있던 것을
여기로 단일화한다. 각 단계의 실제 완료 여부와 수치는 `WORKLOG.md`를 본다.

| 단계 | 내용 |
|---|---|
| P0 | GT→actor 정보 방화벽 + structured observation 스키마 |
| P1 | RGB-D/LiDAR 검출기 + held-out 검증 |
| P2 | association + 칼만 추적기 + 공분산 |
| P3 | 17-token temporal Transformer actor 연결 |
| P4 | 속도 상향 + 이동표적 요격 + PPO 안정화 |
| P5 | 충돌 저감 — 고도 제어 → look-ahead → 장애물 표현 |
| P6 | 밀도 커리큘럼 → **밀도 × 표적속도 지도** (논문 핵심 그림) |
| P7 | sim-to-real: onboard latency, 센서 노이즈, 캘리브레이션 perturbation |

### 5.1 완료 마일스톤 (B0–B5)

## 1. 완료 기반

| 단계 | 산출물 | 상태 |
|---|---|---|
| B0 | Aerial Gym/Isaac Gym/Warp/rl_games 환경 | 완료 |
| B1 | random bar arena, 36×4 LiDAR, capture task | 완료 |
| B2 | learned yaw obstacle navigation | 완료 |
| B3 | density sweep와 curriculum infrastructure | 완료 |
| B4 | scripted moving target와 interception reward | 완료 |
| B5 | camera/LiDAR occlusion·throughput prototype | 완료, 단 oracle semantic prototype |


### 5.2 게이트 G0–G5

## 6. 구현 게이트

- G0: actor 경로에 GT/semantic target channel 참조 0개.
- G1: camera-only와 LiDAR-only가 각각 visible target에 대해 chance보다 유의하게 높은 recall을 보임.
- G2: fused visible-target median position error ≤0.30 m at 10 m.
- G3: 1초 simultaneous sensor miss 뒤 track survival/reacquisition ≥80%.
- G4: Transformer가 single-step CNN보다 control effort와 occlusion robustness를 개선하고, LSTM 대비
  accuracy–smoothness–latency Pareto 우위를 하나 이상 보임.
- G5: held-out layout에서 GT upper bound와 capture gap을 perception/tracking/policy error로 분해.


### 5.3 중단 조건

## 중단 조건

- 어느 한 센서 detector도 visible target을 못 찾으면 Transformer를 키우지 말고 sensor/asset/data부터 수정한다.
- Transformer 평가는 success만 보지 않고 NavRL++처럼 control effort와 perturbation robustness를 함께 본다.
- 8GB에서 OOM이면 raw image를 policy에 넣지 않고 detector output을 detach/cache하고 env를 256→128로 낮춘다.
- 고정 density에서 KL 또는 latent mean이 지속 상승하면 더 학습하지 말고 last-known-good checkpoint로
  rollback한 뒤 update safety를 수정한다.

---

## 6. 실험 설계

### 6.1 밀도 실현가능성 기하 감사

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

### 6.2 단계적 커리큘럼 레시피

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

### 6.3 평가 그리드

### Phase 4 — 핵심 실험

- density: bars {25,50,75,110,150}.
- target speed: {0,0.5,1.0,1.5,2.0} m/s; 2.0은 평가 전용.
- occlusion duration: {0,0.25,0.5,1.0,2.0}s 또는 관측된 연속 가림 bin.
- trajectory: CV/waypoint/circle, held-out layout과 appearance.
- 조건당 ≥1,000 episodes, 학습 seed ≥3.

### 6.4 모델 비교

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

### 6.5 리스크와 완화

## 7. 리스크

| 리스크 | 대응 |
|---|---|
| sparse LiDAR로 target/obstacle 구분 불가 | 시간차 motion feature, target geometry, 72×6 ablation; camera와 fusion |
| Transformer가 작은 데이터에서 과적합 | structured 17 tokens, NavRL++ dim 64, detector pretraining, strong split |
| 8GB OOM | detector offline pretraining, structured 17 tokens, gradient accumulation, env 128 fallback |
| detector 오류 또는 PPO update가 정책을 붕괴 | perception freeze, confidence input, density-aware collapse guard, KL/latent fail-stop와 last-known-good rollback |
| semantic label 누출 | 별도 label dict/dataloader, actor observation schema test, code review gate |
| sim-to-real gap | texture/light/noise/dropout randomization, calibration perturbation, real-log validation |

---

## 7. 구현 단위와 체크포인트 규약

## 4. 구현 단위

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


## 6. 체크포인트 규칙

- old GT/semantic vision checkpoint는 새 raw-perception 모델과 호환되지 않는다.
- navigation-only backbone은 transfer 후보지만 observation layer는 새로 초기화한다.
- perception, policy, optimizer, dataset/config hash를 한 checkpoint manifest에 기록한다.
- `gen_ppo.pth`만 보관하지 말고 detector validation-best와 navigation capture-best를 별도로 보관한다.

---

## 8. 참고문헌

## 10. 보조 설계 근거

- [NavRL++ (2026)](https://arxiv.org/abs/2605.15559): structured perception, 2초 history,
  Transformer policy, perception-failure fine-tuning의 직접 기준.
- [NavRL code](https://github.com/Zhefan-Xu/NavRL): 기존 NavRL training/deployment 구현 기준.
- [GAFusion, CVPR 2024](https://openaccess.thecvf.com/content/CVPR2024/html/Li_GAFusion_Adaptive_Fusing_LiDAR_and_Camera_with_Multiple_Guidance_for_CVPR_2024_paper.html): LiDAR–camera adaptive fusion.
- [ORTrack, CVPR 2025](https://openaccess.thecvf.com/content/CVPR2025/html/Wu_Learning_Occlusion-Robust_Vision_Transformers_for_Real-Time_UAV_Tracking_CVPR_2025_paper.html): UAV occlusion-robust tracking.
- [TransFusion, CVPR 2022](https://openaccess.thecvf.com/content/CVPR2022/html/Bai_TransFusion_Robust_LiDAR-Camera_Fusion_for_3D_Object_Detection_With_Transformers_CVPR_2022_paper.html): object-query 기반 camera–LiDAR fusion 비교안.
