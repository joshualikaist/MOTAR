# 연구 계획서

**무작위 장애물 환경에서 이동 표적 접근을 위한 강화학습 기반 UAV 항법 및 동적 물체 탐지 성능 분석**
(Performance Analysis of RL-Based UAV Target Approach and Moving Object Detection in Random Obstacle Environments under Varying Obstacle Density and Target Speed)

작성일: 2026-07-09 | 환경 검증 완료 상태에서 작성됨

---

## 1. 연구 질문과 기여

| # | 연구 질문 | 대응 실험 |
|---|----------|----------|
| RQ1 | 장애물 밀도가 증가할 때 RL 항법 정책의 표적 접근 성능은 어떻게 변하는가? | 밀도 스윕 (Phase 2–3) |
| RQ2 | 표적 이동 속도가 증가할 때 접근 성능은 어떻게 변하는가? 정지 표적으로 학습한 정책의 zero-shot 성능 vs 이동 표적으로 재학습한 정책의 차이는? | 속도 스윕 (Phase 4) |
| RQ3 | 밀도·표적속도 변화가 동적 물체 탐지 성능(정밀도/재현율, 상태추정 오차, 지연)에 어떤 영향을 주며, 탐지 품질 열화가 항법 성능에 어떻게 전파되는가? | 탐지 분석 (Phase 5) |

**기대 기여**
1. 밀도 × 표적속도 2차원 **성능 지도(performance map)** — 성공률/충돌률/접근시간 히트맵 (논문의 핵심 그림)
2. GT 관측 vs 탐지 기반 관측의 성능 격차 정량화 (perception-in-the-loop 분석)
3. 실패 모드 분류 및 커리큘럼·관측 설계가 미치는 영향 분석

---

## 2. 시스템 현황 (2026-07-09 검증 완료)

| 항목 | 상태 |
|------|------|
| OS / GPU | Ubuntu 20.04.6 / RTX 3070 8GB (driver 570.133.07) |
| Isaac Gym Preview 4 | `~/isaacgym` 설치됨 (1.0rc4) |
| Aerial Gym 2.0.0 | `src/aerial_gym_simulator` editable 설치 (ntnu-arl, 커스텀 수정 있음) |
| conda env `aerialgym` | Python 3.8.20, torch 2.4.1+cu121, rl-games 1.6.5, warp-lang 1.0.0 |
| 스모크 테스트 | `navigation_task` (장애물 환경 + warp depth camera + VAE) headless 4-env 실행 **통과** |
| NavRL 코드 | `reference/NavRL/` 클론됨 (참조용) |
| 기존 자산 | `shooting_moving_target_task` + `moving_intercept_env` — 이동 표적 요격 과제 (장애물 없음, 6m 큐브). **표적 이동 로직을 Phase 4에서 재활용** |

**오늘 수정한 것**: `~/.local`(AirSim이 `pip install --user`로 설치)의 numpy 1.24.4가 conda 환경의 numpy 1.23.0을 가려 warp 센서 경로(`urdfpy→networkx`)가 `np.int` 오류로 죽는 문제 → `aerialgym` 환경 활성화 시 `PYTHONNOUSERSITE=1`을 자동 설정하는 conda activate.d 스크립트 추가. **이 스크립트를 지우면 안 됨** (AirSim 환경은 영향 없음).

주의: Isaac Gym import 순서 — 항상 `aerial_gym`(또는 `isaacgym`)을 `torch`보다 먼저 import.

---

## 3. NavRL 팩트 시트 (논문 + 코드에서 추출)

**중요한 사실**: NavRL의 학습 환경은 Aerial Gym이 아니라 **Isaac Sim 2023.1.0 + OmniDrones** 기반이다. 8GB VRAM에서 Isaac Sim 학습은 비현실적이므로, 본 연구는 NavRL의 **환경 사양을 Aerial Gym 2.0에 재구현**한다 (올바른 선택). 완전 동일 재현이 아닌 "사양 재현 + 차이점 명시 기록" 방식으로 진행.

`reference/NavRL/isaac-training/training/cfg/*.yaml` 및 `scripts/env.py`에서 추출한 실제 수치:

### 환경
| 항목 | 값 |
|------|-----|
| 맵 크기 | 40 × 40 × 4.5 m (`map_range=[20,20,4.5]`, 논문 그림은 50×50 표기) |
| 정적 장애물 | 350개 (기둥형, 높이 {1.0, 1.5, 2.0, 4.0, 6.0}m 확률 {0.1, 0.15, 0.20, 0.55}) |
| 동적 장애물 | 80개 (커리큘럼: 60→120, 성공률 80% 초과 시 +20) |
| 동적 장애물 종류 | 2D형(높이 5m 기둥, 우회만 가능) + 3D형(높이 1m, 위로 통과 가능), 직육면체/원기둥 |
| 동적 장애물 속도 | [0.5, 1.5] m/s, 스폰 주변 local_range [5, 5, 4.5]m 내 왕복 |
| 에피소드 | max 2200 스텝, sim dt 0.016s, 드론 스폰 z=2.0m |

### 관측 (state)
| 구성 | 차원 | 내용 |
|------|------|------|
| S_int | 7 | 목표방향 단위벡터(3) + 목표거리(1) + 속도(3), **goal coordinate frame** (원점=시작점, x축=시작→목표 방향) |
| S_dyn | 5 × M | 가장 가까운 동적 장애물 N_d=5개: 상대위치 단위벡터+거리+상대속도+크기(폭,높이). 미달 시 zero-padding |
| S_stat | 36 × 4 | 레이캐스트 거리 행렬: 수평 36방향(10° 간격) × 수직 4빔(VFOV −10°~+20°), 최대거리 4.0m |

특징 추출: static/dynamic 각각 3-layer CNN → 임베딩 128/64 → S_int와 concat → 2-layer MLP (actor/critic).

### 행동
- 3D 속도 명령, **Beta 분포** 정책 (출력 [0,1] → V = v_lim·(2·V̂−1)), v_lim = 2.0 m/s
- Beta 분포는 유계 행동공간에서 Gaussian보다 수렴 빠름 (Chou et al. 2017)

### 보상 (env.py 실제 구현)
```
r = r_vel + 1.0(alive) + 1.0·r_ss + 1.0·r_ds − 0.1·r_smooth − 8.0·r_height
```
- `r_vel` = 목표방향 단위벡터 · 현재속도 (목표를 향한 속도 성분)
- `r_ss` = mean(log(클램프된 레이 여유거리)) — 정적 장애물과의 평균 log 거리
- `r_ds` = mean(log(동적 장애물까지 거리)) — 가까운 N_d개
- `r_smooth` = ‖V(t) − V(t−1)‖
- `r_height` = 시작/목표 고도 범위 밖으로 벗어난 정도의 제곱

### PPO 하이퍼파라미터
| 항목 | 값 |
|------|-----|
| lr (actor/critic/extractor) | 5e-4 (ADAM) |
| clip ratio | 0.1 |
| γ | 0.99 |
| rollout horizon | 32 frames, 4 epochs, 16 minibatches |
| entropy coef | 1e-3 |
| 병렬 로봇 수 | 1024 (RTX 4090에서 약 10시간 학습) |

### 배치(deployment) 단계 — Phase 5 참고
- 정적: depth → 점유 복셀맵 → 레이캐스트 / 동적: **U-depth 검출기 + DBSCAN 검출기 앙상블**(상호 일치 확인) + 경량 YOLO로 동적/정적 분류 + 칼만 필터(등가속 모델) 추적
- 탐지 코드가 `reference/NavRL/ros1/onboard_detector`, `ros2/onboard_detector`에 포함됨
- Safety shield: Velocity Obstacle 기반 QP 투영 (배치 시에만) — ablation 소재

---

## 4. 단계별 로드맵

원칙: **한 번에 하나의 축만 업그레이드**한다. 각 Phase는 "이전 Phase의 검증된 코드 + 한 가지 새 요소".

### Phase 0 — 준비 (~3일)
- [x] 스택 검증, numpy 충돌 수정, 스모크 테스트, NavRL 클론
- [ ] `src` 저장소에 research 브랜치 생성 (`git checkout -b research/navrl-env`), 기존 미커밋 수정사항 커밋
- [ ] wandb(권장) 또는 tensorboard 로깅 규약 결정, 시드 고정 유틸 확인
- [ ] NavRL `env.py` 정독 (레이캐스트 구현, 동적 장애물 이동 로직, 종료 조건) — 위 팩트 시트 보완

### Phase 1 — 정적 환경 + 정지 표적 ✅ 완료 (2026-07-15, yaw 제어로 captured 0.95)
목표: NavRL-static을 Aerial Gym에서 재현. 파이프라인 전체(환경→학습→평가→그림)를 한 번 관통. **달성.**
- `navrl_bars_env.py` (env_config): **24×24×3m** 아레나 + 정적 기둥(bars) **48개 균등 배치**(x[3,22]×y[0,24], ~11개/100m²), **2D 비행 @ 고도 1m**. **스폰: 드론 x≈0(왼쪽 가장자리) → 목표 x=k(먼쪽) = 매 에피소드 막대밭 전체 관통**(옛 방사형/18m 설계 폐기). 커리큘럼: k_max 7→24m 램프 후 full-scale hold, 이어 k_min 5→20m. 드론 충돌 = **0.28m 박스**(팔/프롭 포함, navrl 전용 URDF).
- `navrl_task.py` (task): S_int(**12**, vehicle-frame heading 포함) + LiDAR 36×4(range 4m) 관측, goal프레임 속도명령 **+ 학습형 yaw-rate(option b)** 행동, **LiDAR CNN 특징추출기**(`navrl_network.py`).
- 보상: NavRL 정적 브랜치를 **요격용으로 각색**(NavRL과 의도적 차이) — 캡처 종료 +30, alive를 −0.05 시간비용으로, **B1 안전항 재베이스라인**(개방공간=0, loiter 수입 제거), **PBRS progress**(거리 shaping), collision −20 가드. crash 저감용 safety weight 1.5 + clearance 페널티(옵션).
- 정책: rl_games PPO (`ppo_navrl_cnn.yaml`), Gaussian+clamp(Beta 미지원, 필요 시 NavRL 순수 torch PPO 포팅 가능).
- 평가: 성공률(**포획 반경 0.5m 진입**, NavRL env.py 기준)/충돌률/타임아웃/최근접(**crash 제외** 평균+최소)/커리큘럼. 요약기·TB navrl화 완료.
- **완료 기준 M1(성공률 ≥ 90%) → ✅ 달성**: 새 스킴(cross-field 24m·균등막대·0.28m 박스충돌) 진행 = run `1904` 정직 baseline **0.65** → crash 레버 Run D `2207` **0.66**(감속만, 리워드로 안 줄어드는 **기하 바닥** 발견) → **yaw 제어(option b)로 captured 0.954 / crash 0.046**(run `ppo_260715_0251`, k_min=20 최심에서도 0.95). **NavRL 0.81·M1 0.90 모두 상회.** 잔여 crash 원인 = 요-고정 ±30° corner-clip(대각 footprint 0.40m)이었고 yaw 권한(action 3→4)으로 해결. 대조군 브랜치 `ablation/yaw-off`(논문 attribution용). 상세: `WORKLOG.md`.

### Phase 2 — 정적 밀도 스윕 (1주) → RQ1 전반부 ← **현재 여기**
- 밀도 정의: **개수/100m²** (NavRL 기준 350개/1600m² ≈ **22개/100m²** — 이전 계획의 "2.2"는 10배 오타)
- 밀도 5레벨: {5, 10, 15, 22, 30}개/100m² (NavRL 밀도 22를 중심으로). 참고: 현재 Phase 1 아레나(48개/576m²)는 ≈8.3개/100m²로 희소한 쪽. **8GB에서 고밀도(22–30)는 기둥 수가 많아져(576m²면 127–173개) num_envs를 256→128 이하로 낮춰야 함.**
- 커리큘럼 학습 vs 고정 밀도 학습 비교 (NavRL Table I 재현 성격)
- 일반화 매트릭스: 학습 밀도 A → 평가 밀도 B (교차 평가)
- **완료 기준 M2**: 밀도–성능 곡선 + 커리큘럼 효과 그림
- **진행(2026-07-15)**: 밀도 배관 완료(`NAVRL_MAX_BARS`/`NAVRL_NUM_BARS`, `keep_in_env=False`로 런타임 활성수
  제어; 커밋 `7e1b6a8`+`0aed4be`). 실제 sweep = 막대수 {25, 50, 75, 110, 150}개 = {4.3, 8.7, 13, 19, 26}개/100m².
  **VRAM 스크리닝: 150막대+256env ≈ 6.1GB → num_envs 256 그대로 가능**(위 "128 이하로 낮춰야" 우려는 해소됨).
  첫 데이터포인트: **150막대(≈26/100m², NavRL 밀도) → captured 0.85**(run `ppo_260715_1552`, NavRL 0.81 상회).
  밀도-성능(RQ1): 48막대 0.95 → 150막대 0.85. 나머지 4레벨 seed 1 진행 중.

> **빌드 순서 (결정): P2 밀도 → P3 이동표적 → P4 동적장애물 → P5 3D → P6 탐지 → P7 논문.**
> 논문 핵심(밀도×속도 성능지도)은 **P2+P3만으로 완성** → critical path. 기능 간 하드 선후행 없음(모두 P1 정적 정책만 필요). **VRAM은 밀도(P2)만 메시를 늘림** — 이동표적/동적장애물/3D는 전부 VRAM 중립. 3D는 **마지막·격리**(탐색비용↑, 하드설정과 겹치지 말 것). ※ 기존 Phase 3(동적)↔4(표적)를 **스왑**했음.

### Phase 3 — 이동 표적(스크립트) (1~2주) → RQ2 · 연구의 고유 기여 · **논문 대표 그림** ← 밀도 다음 critical path
*(목표=task가 이동시키는 좌표, 거리 캡처 → 액터/메시 0, VRAM 중립. 이동 로직은 `shooting_moving_target_task` 재활용.)*
- `navrl_task.step()`에서 `_advance_target()` 먼저 호출, **rl_dt=0.1s**(=10×sim dt)로 적분, 매 스텝 goal을 막대 밖으로 push-out. 패턴: 등속직선(반사)/원형/랜덤웨이포인트/(스트레치)회피.
- 표적 속도 스윕 {0,0.5,1.0,1.5,2.0} m/s. **v_lim=2.0 근처는 순수추격 불가(degenerate) → 학습은 ≤1.5 캡, 2.0은 평가전용 셀로 보고.**
- **goal frame은 에피소드당 고정**(reset 시 start→초기표적). S_int(방향/거리/속도)는 매 스텝 현재 target_position로 재계산(이미 그럼). ← 옛 "매 스텝 frame 갱신 필요"는 **오해**(방위 소실). 갱신은 ablation만.
- 리워드: 속도항을 **range-rate** `((v−v_target)·dir)` 로 1줄 변경(속도 0이면 현재와 동일). 캡처: 순간 dist<0.5 + **세그먼트 판정(터널링 방지)**, t_hold 없음.
- 관측 152 유지(**zero-shot 가능**)/옵션 표적속도 추가 시 155(재학습). 평가 (a)**zero-shot**(P1/P2 정책 그대로) vs (b)**재학습**.
- **완료 M3**: zero-shot 속도-성공률 곡선 + 재학습 갭 + **밀도(P2)×표적속도 히트맵**(여기서 확보).

### Phase 4 — 동적(이동) 장애물, 2D (1~2주) → RQ1 후반부
*(NavRL 충실: LiDAR는 정적만, 동적은 GT-state S_dyn로만 → **메시 0·VRAM 중립**. P3 이동로직 (N,M) 일반화.)*
- 순찰 동적 장애물을 **순수 per-env GPU 텐서**(액터/메시 없음)로. NavRL `env.py` 이식: 스폰/이동(local_range 왕복, ~2s마다 속도 U[0.5,1.5] 재샘플)/**해석적 충돌**(접촉 아님).
- **S_dyn**: N_d=5 최근접, 장애물당 10차원[상대위치단위(3)+2D거리+z거리+상대속도(3)+폭·높이 카테고리], zero-pad → **관측 152→202**. 리워드 `+ r_ds`(재베이스라인 log거리, loiter income 제거). 종료 `|= 해석적 동적충돌`.
- 네트워크(`navrl_network.py`): 동적 브랜치(50→128→64) 추가, obs 202/152 분기(구 체크포인트 호환). 개수 커리큘럼 60→120(P1/P3 체크포인트 재개, S_int/LiDAR 가중치 전이).
- **완료 M4**: full(60→120, 0.5–1.5 m/s) 성공률 ≥ ~65%, 동적충돌 ≤10%. Ablation: S_dyn/r_ds 유무, GT vs 노이즈.

### Phase 5 — 3D / z축 비행 (신규·격리, ~1주) → capability check
*(마지막. z만 풀면 다 넘어가 trivial → **높이 이질 장애물** 필수. VRAM 중립(높은 박스=같은 삼각형 수), 비용=탐색.)*
- vz 언락(`lock_altitude=False`), 목표 z 랜덤 0.5~2.5m(`_goal_z_range` 램프). **height 리워드(가중치 8, 현재 무효)가 자동 활성** → 3D 튜닝 손잡이.
- **혼합 높이 막대**: 짧은(1.2m, 위로 통과) + 높은(4m=천장, 우회만) — 신규 asset/`navrl_bars3d_env`. LiDAR 36×4 유지(옵션 VFOV 확대).
- 커리큘럼: 목표 z밴드 [1,1]→[0.5,2.5](수평 램프 완료 후). **밀도는 낮춰서** P2~4 하드설정과 겹치지 말 것.
- **완료 M5**: z-lock≈2D(무회귀), z-free가 높/낮은 목표에서 z-lock을 확실히 이김, overfly 비율↑.

### Phase 6 — 동적 물체 탐지 통합·분석 (2~3주) → RQ3
관측을 GT에서 "탐지 기반"으로 교체했을 때의 성능을 분석. (Phase 4의 GT S_dyn가 baseline — 그 위에 열화/탐지기를 얹음.)
- **트랙 A (권장, 통제된 분석)**: GT에 **열화 모델** 주입 — 거리/차폐 의존 미탐지율, 위치·속도 노이즈, 지연(latency), 트랙 드롭. 파라미터를 밀도·속도의 함수로 캘리브레이션 → 탐지 열화의 영향을 인과적으로 분석 가능. 8GB에서 대규모 병렬 유지 가능
- **트랙 B (충실도)**: warp depth camera(예: 270×480) 장착, `onboard_detector`의 U-depth+DBSCAN+칼만 파이프라인을 torch로 포팅해 시뮬레이션 루프에서 실행. num_envs 64–256으로 감소 (평가만 하면 충분)
- 탐지 지표: Precision/Recall(거리 임계 매칭), 위치/속도 RMSE, 트랙 연속성(ID switch), 탐지 지연 — 밀도 × 표적속도 조건별
- 연결 분석: 탐지 지표 열화 ↔ 항법 성공률 하락의 상관/회귀
- **완료 기준 M5**: 탐지 성능 매트릭스 + "탐지→항법" 전파 분석 그림

### Phase 7 — 분석·논문화 (2~3주)
- 실패 모드 분류 (충돌 시점의 상대 기하 클러스터링: 정적 충돌/동적 충돌/추격 실패/타임아웃)
- Ablation: 관측 요소 제거(S_dyn 없음 등), 커리큘럼 유무, (옵션) VO safety shield 포팅 후 유무 비교
- 통계 규약: 학습 시드 ≥ 3–5개, 조건당 평가 에피소드 ≥ 500 (병렬이라 저렴), 95% CI 표기
- 타깃: RA-L / ICRA / IROS (NavRL·NavRL++와의 차별점: 체계적 밀도×속도 성능 분석 + 탐지-항법 결합 분석)

---

## 5. 실험 설계 요약

**독립변수**
| 변수 | 레벨 |
|------|------|
| 장애물 밀도 | 5레벨 ({5, 10, 15, 22, 30}개/100m², 정적) + 동적 개수 3레벨 |
| 표적 속도 | {0, 0.5, 1.0, 1.5, 2.0} m/s |
| 관측 종류 | GT / 열화모델 / (옵션) 실제 탐지기 |
| 정책 | zero-shot / 재학습 |

주 실험은 밀도×속도 grid (5×5), 나머지는 ablation으로 제한해 조합 폭발 방지.

**종속변수(지표)**: 성공률, 충돌률, 타임아웃률, 접근시간, SPL(경로효율), 최소 이격거리, 제어 smoothness(Δv 적분) / 탐지: P/R, RMSE, ID switch, latency

**통제변수**: v_lim=2.0 m/s, 맵 크기, 에피소드 길이, 평가 시드셋(고정), 표적 크기

---

## 6. RTX 3070 8GB 운영 가이드

- **레이캐스트(warp LiDAR) 관측만 사용 시**: 512–1024 envs 예상 가능. 512부터 시작해 `nvidia-smi`로 확인하며 증가. NavRL도 카메라 없이 레이캐스트로 학습함 — 8GB에서 승산이 있는 이유
- **depth camera 렌더 포함 시**: 64–256 envs (Phase 5 트랙 B에서만 필요; 평가 전용이므로 문제 없음)
- 학습은 반드시 headless. 데스크탑(Xorg+gnome)이 ~630MB 점유 중 — 부족하면 학습 시 로그아웃/tty 사용으로 회수
- 학습 시간 추정: NavRL이 4090에서 10시간(1024 robots) → 3070은 동일 조건 3–5배. **환경 축소판(20×20m) + 레이 관측**이면 하룻밤(8–15시간)/run 수준으로 현실적. 본 실험 돌입 전 소규모로 하이퍼 확정할 것
- 병렬 평가는 학습보다 훨씬 가벼움 → 조건당 500 에피소드도 수 분 단위

---

## 7. 리스크와 대응

| 리스크 | 대응 |
|--------|------|
| NavRL 재구현 디테일 차이로 성능 미달 | 팩트 시트 기준 구현 + NavRL 자체 PPO 코드 포팅(순수 torch) 옵션. "동일 사양, 시뮬레이터 차이 명시" 서술 전략 |
| 8GB VRAM 부족 | 맵 축소(20×20), num_envs 감소, 레이 해상도 유지(36×4는 작음). 카메라는 평가 전용 |
| 고밀도+고속 표적에서 학습 불안정 | 커리큘럼 순서: 밀도 → 동적 장애물 → 표적 속도. 보상 가중치는 NavRL 값에서 시작 |
| 탐지 파이프라인 포팅 부담 | 트랙 A(열화 모델)를 주 결과로, 트랙 B는 검증 샘플로 축소 가능 |
| Isaac Gym EOL/드라이버 이슈 | 현 스택 동작 확인됨 — **버전 업그레이드 금지** (torch/numpy/driver 고정) |
| rl_games에 Beta 분포 없음 | 1차 Gaussian+clamp로 진행, 수렴 문제 시 NavRL PPO 포팅 |

---

## 8. 자료 목록

**논문 (Downloads에 이미 있음)**: NavRL (RA-L 2025), NavRL++, Aerial Gym Simulator, MAVRL, 기타 추격-회피/요격 논문들
**추가로 받을 것**: Xu et al., "Onboard dynamic-object detection and tracking for autonomous robot navigation with RGB-D camera" (RA-L 2024) — 탐지 파이프라인 원 논문 / Chou et al. 2017 (Beta policy) / Fiorini & Shiller 1998 (VO)
**코드**: `reference/NavRL/` (학습 env + onboard_detector 포함), `src/aerial_gym_simulator/` (docs: ntnu-arl.github.io/aerial_gym_simulator)

---

## 9. 이번 주 액션 아이템

1. `src` 저장소 정리: 기존 수정사항 커밋 → `research/navrl-env` 브랜치 생성
2. NavRL `env.py` 정독 (특히 `_compute_lidar_scan`, 동적 장애물 왕복 로직, 종료 조건)
3. Aerial Gym warp LiDAR 설정으로 36×4 레이 행렬 뽑아보기 (센서 config 실험)
4. `navrl_env.py` 초안: 20×20m + 정적 기둥 50개 랜덤 배치
5. `navrl_task.py` 초안: S_int+S_stat 관측 + 보상식 → 무학습 랜덤 정책으로 스텝 확인
6. rl_games 러너 연결 후 첫 학습 (수렴 여부 무관, 파이프라인 검증 목적)
