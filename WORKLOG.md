# WORKLOG — MOTAR 연구 작업 기록

무엇을, 왜 바꿨는지의 시간순 기록. 최신이 아래쪽. (연구 로드맵은 워크스페이스 루트의
`RESEARCH_PLAN.md`, 실행 방법은 `README.md`의 Getting Started 참고)

**기록 규칙:** 코드·설정·실험 조건 또는 결과를 변경한 작업은 같은 작업 안에서 이 파일도 함께
갱신한다.

---

## 2026-07-09 — 저장소 정리 & NavRL LiDAR 재현

### 저장소 셋업
- `research/navrl-env` 브랜치 생성, origin = github.com/joshualikaist/MOTAR.
- 연구에 불필요한 파일 삭제 (docs/, sim2real/, dce 예제, CI, supplementary ~170MB) — 커밋 `7847994`.
- `main`은 업스트림(ntnu-arl) + MOTAR README 병합 상태로 유지, 연구 커밋은 전부 `research/navrl-env`에.

### NavRL식 LiDAR (커밋 `b4f72df`)
- **신규** `config/sensor_config/lidar_config/navrl_lidar_config.py` — NavRL 사양 재현:
  수평 36빔(10° 간격, hfov −170°~180°로 끝점 중복 회피), 수직 4빔(−10°~+20°), range 4 m.
- **신규** `config/robot_config/navrl_quad_config.py` — 카메라 끄고 위 LiDAR만 단 쿼드(`navrl_quad`).
- `base_sensor_config.py` + `sensors/warp/warp_sensor.py` — `yaw_only_attach` opt-in 플래그 추가:
  센서가 롤·피치는 무시하고 yaw만 따라가도록 (NavRL과 동일). 기본값 off라 다른 로봇에 영향 없음.

### navrl_task 골격 (커밋 `11b836f`, `3fcf2db`)
- **신규** `task/navrl_task/navrl_task.py` — 관측 152차원 = S_int(8: goal프레임 방향/거리/속도)
  + LiDAR 36×4 평탄화(144). 행동 = goal프레임 3D 속도(→ lee_velocity_control). 보상 = NavRL
  정적 브랜치(vel + alive + static_safety − smooth − height, 충돌 페널티).
- **신규** `config/task_config/navrl_task_config.py`, task_registry에 `navrl_task` 등록.
- rl_games 연결: `rl_training/rl_games/runner.py`에 등록 + `ppo_navrl.yaml` 신규(MLP [256,256,128]).
- **구현 gotcha**: `env.step()`은 센서를 렌더하지 않음 — task가 `post_reward_calculation_step()`을
  호출해야 LiDAR가 실제 raycast됨. 빠뜨리면 전 레이가 max range로 나옴.

---

## 2026-07-09 밤 ~ 07-10 새벽 — 첫 학습과 정직한 진단

- 512 env 헤드리스로 1500 epoch 학습 (VRAM 6.8/8 GB — 512가 상한).
- 평가 지표 내장(커밋 `345cc38`): `ever_reached`, `mean_closest_approach` 등.
- **결과: 목표 도달률 0%.** 원인 진단 — 목표가 첫 에피소드부터 맵 끝(20~38 m)에 뽑히는데
  에피소드는 150스텝(≈최대 15 m 비행)이라 물리적으로 도달 불가.

### 목표 거리 커리큘럼 (커밋 `0a1f8ed`)
- `navrl_task.py` + config에 커리큘럼 추가: 목표를 스폰 주변 2.5~5 m에서 시작, 도달률 60%
  넘으면 최대 거리 +1.5 m씩 확장 (최대 18 m).
- 재학습 결과 ever_reached 0% → 20~36%로 개선됐으나 **~30%에서 정체** (2 m 근처까지 가서
  마지막 1~2 m을 파고들지 못함) — 충돌 회피가 병목.

### README Getting Started (커밋 `e0cca2b`)
- 환경 셋업~학습~재생~평가 전 과정 가이드 + `examples/navrl_task_example.py`(뷰어 예제) 추가.

---

## 2026-07-10 — 뷰어 segfault 수정 & 전용 막대 환경 구축

### segfault(코어 덤프) 원인과 해결
- `navrl_task_example.py` 실행 시 `PxgCudaDeviceMemoryAllocator fail ... Result = 2` 후 segfault.
- 원인: **GPU 메모리 부족** — 백그라운드 학습이 5.7 GB를 물고 있어 뷰어(PhysX)가 할당 실패.
- 해결: 학습 프로세스 정지로 VRAM 확보. **교훈: 8 GB에서는 학습과 뷰어 동시 실행 불가.**

### 전용 "빈 공간 + 막대" 환경 `navrl_bars_env` 신설
그동안 task가 스톡 `env_with_obstacles`(6면 벽 + 패널 + 잡동사니 방)를 쓰고 있었음 —
"빈 공간에 막대만" 통제 환경이 아니었음. 이를 폐기하고 전용 환경 구축:

- **신규** `config/env_config/navrl_bars_env.py` — 빈 10×10×3 m 아레나(벽·패널 없음),
  막대 16개만. `env_manager/__init__.py`에 `navrl_bars_env` 등록,
  `navrl_task_config.py`의 `env_name`을 이것으로 교체.
- **숨은 버그 수정**: 스톡 env는 z가 0 중심(−2.5~2.5)이라 드론 스폰 상당수가 task 바닥
  기준(0.1 m) **아래에서 시작 → 즉시 추락 판정**되고 있었음. 새 환경은 지면 기준
  z(0~3 m)라 스폰 전부 유효. (기존 학습의 높은 충돌률 55~78%를 부풀린 원인일 가능성)

### 막대 사양 확정 (사용자 지시 반영)
- **가변 크기 막대**: 가로·세로 각각 0.4~0.8 m 랜덤, **높이 2 m 고정**, 바닥에 서 있음(0~2 m).
  - **신규** `tools/generate_bar_assets.py` — 막대 URDF 풀(40개) 생성 스크립트 (시드 고정, 재현 가능).
  - **신규** `resources/models/environment_assets/bars/bar_000~039.urdf`.
  - `config/asset_config/env_object_config.py`에 `bar_asset_params` 추가 — bars/ 풀에서
    에피소드마다 랜덤 16개 선택.
- **겹침 방지 + 간격 보장**: `env_manager/asset_manager.py`에 **지터드 격자(jittered grid)**
  배치 추가 (`_enforce_min_xy_spacing`, env config의 `min_obstacle_xy_spacing`로 opt-in —
  기본 off라 다른 환경 영향 없음).
  - 필드를 4×4 = 16칸으로 나눠 칸마다 막대 1개 + 칸 안에서 랜덤 지터
    → **중심간 ≥ 1.3 m 수학적으로 보장** = 최대 크기(0.8 m) 막대끼리도 **실제 틈 ≥ 0.5 m**.
  - 처음엔 rejection sampling(균등 랜덤)으로 구현했으나 16개 @ 1.3 m 간격은 점유율 40%로
    수렴 실패(측정: 최소 0.747 m). 균등 방식의 적정 개수는 12개(점유율 30%)임을 시뮬레이션으로
    확인 → 16개 유지를 위해 격자로 전환 (사용자 승인).
- **2D 비행 @ 1 m 고도**: 드론은 1 m 상공에서 XY로만 추적 (향후 도망 표적도 XY만).
  - `navrl_quad_config.py` — 스폰 z를 1 m 고정(z-ratio 1/3), 초기 수직속도 0.
  - `navrl_task.py` — ① 수직 속도 명령 강제 0 (`transform_action_to_command`),
    ② 목표를 z = `flight_altitude`(1 m)에 배치 (기존 고도 지터 제거).
  - `navrl_task_config.py` — `flight_altitude = 1.0` 추가.

### 검증 (64 env 헤드리스)
| 항목 | 결과 |
|------|------|
| 막대 수 | 16개/env |
| 막대 중심간 최소 거리 | 1.333 m (≥ 1.3 요구 **PASS**, 겹침 0건) |
| 막대 위치 | z중심 1.0 m — 바닥 0~2 m에 서 있음 |
| 드론 고도 (40스텝 전속 비행 후) | 0.83~1.13 m — 1 m 유지 확인 |
| 목표 고도 | 전부 정확히 1.0 m |
- 탑다운 배치도: 워크스페이스 루트 `navrl_bars_env_layout.png`.

### README 확장
- Phase 1 아레나(막대 16개, 격자 배치, 2D @ 1 m) 설명 추가.
- **TensorBoard 모니터링 가이드** 추가: rl_games가 `runs/<run>/summaries/`에 자동 기록,
  `tensorboard --logdir runs`로 열람. 봐야 할 지표(rewards/step, episode_lengths,
  losses/entropy, info/kl 등)와 콘솔 `NavRL progress` 지표(ever_reached, crash,
  mean_closest_approach 등) 해석 표 정리.

### bars 환경 첫 본 학습 (사용자 실행, `ppo_260710_1608_navrl`)
- 512 env × 1500 epoch. mean reward 28→338, 에피소드 길이 29→144/150 (충돌 급감).
- 사전 스모크(2.5분, 119 epoch)에서 이미 ever_reached 71%, 커리큘럼 5→14 m 연속 확장 —
  구 환경에서 30% 정체였던 것과 대비. 환경 교체(벽 제거+스폰 버그 해소+2D)가 결정적.

### 학습 대시보드 개편 (콘솔 + TensorBoard)
예전 요격 과제 때 만든 콘솔 박스가 "Intercept" 제목·intercept 지표 하드코딩이라 교체:
- **신규** `task/navrl_task/train_dashboard.py` — 에피소드 종료 결과를 epoch 단위로 집계.
- `navrl_task.py` — 매 스텝 종료 에피소드를 집계기로 전달(`_record_epoch_dashboard`).
- `early_stop_a2c_agent.py` — epoch마다 navrl 요약 소비: 박스에 `goal reached / success @
  timeout / crash / timeout / closest to goal / curriculum max` 표시 + TB `navrl/*` 스칼라 기록.
- `aerial_tensorboard.py` — 네임스페이스 키(`navrl/...`) 그대로 스칼라로 통과.
- `pretty_train_stats.py` — 제목을 `AERIAL_RUN_TITLE` env var 기반으로(하드코딩 제거),
  "(design ~0–1000)" 문구는 `AERIAL_REWARD_DESIGN_MAX` 선언 시에만 표시.
- `runner.py` — task별로 위 env var 자동 설정(+`AERIAL_TASK_CONFIG_MODULE` → ep length /150 표기).
- `run_header.py` — `navrl_task: "NavRL Navigation · PPO"` 제목 등록.
- tensorboard 뷰어 설치(2.13.0, numpy 1.23.0 핀 무사). 실행: `tensorboard --logdir runs`.
- **주의**: 16:08 학습 run은 이 수정 이전 시작이라 옛 배너로 표시됨. 다음 run부터 적용.

### play 모드 즉시종료 버그 수정 (근본 원인)
`--play` 실행 시 창이 뜨자마자 조용히 종료되는 문제:
- **원인**: `navrl_task.reset()`이 목표만 재배치하고 `sim_env.reset()`(로봇 리스폰)을 안 불러,
  play 첫 리셋 후 드론들이 **빌드 포즈(원점, 막대와 겹침)** 그대로 → 첫 스텝에 전원 충돌
  (-10) → rl_games가 목표 게임 수 즉시 충족으로 판단 → 1스텝 만에 평가 종료.
- 학습에서는 이 전원 충돌이 곧바로 강제 리셋을 유발해 자가 복구됐기 때문에 미발견.
- **수정**: `reset()`에서 `sim_env.reset()`(리스폰) → 목표 배치 → LiDAR 렌더 순으로 변경.
  검증: 리셋 직후 z=1.0 m, 접촉력 0 N, 중간 리셋도 클린.
- **평가 확인**: 512게임 headless play — **평균 보상 299.3, 평균 134.4/150스텝 생존**
  (학습 지표와 일치). 뷰어 play도 정상 동작.
- 참고: play 콘솔이 조용한 것은 `AERIAL_RL_QUIET_STARTUP=1`(기본) 필터 때문 —
  `AERIAL_RL_QUIET_STARTUP=0`으로 끄면 `av reward` 등 rl_games 출력이 보임.

### 뷰어 관찰 결과 + LiDAR CNN 특징추출기 (사용자 방향 확정)
- 뷰어 관찰: 드론이 목표로는 가지만 **막대에 충돌하는 케이스가 여전함** → 학습량 부족 +
  평탄화 MLP의 공간정보 소실이 원인 후보. 사용자 결정: 학습 6000 epoch로 상향 + CNN 진행.
- **신규** `rl_training/rl_games/navrl_network.py` — NavRL ppo.py의 특징추출기를 rl_games
  커스텀 네트워크("navrl_cnn")로 이식: 36×4 스캔 → Conv(1→4→16→16, 커널 5×3, 수평 ×2
  다운샘플) → 288→128 임베딩(LayerNorm) → S_int(8)와 concat → MLP [256,256] → Gaussian
  헤드. (NavRL과의 차이: Beta 대신 Gaussian(logstd), actor/critic 트렁크 공유 — 기록됨)
- **신규** `ppo_navrl_cnn.yaml` (CNN, 권장) / 기존 `ppo_navrl.yaml`(MLP 베이스라인)은 비교용
  유지. 둘 다 `max_epochs: 6000`(≈2h @ 1.1s/epoch)으로 상향. runner.py에 네트워크 등록.
- **스모크 (125 epoch, 2.5분)**: CNN이 MLP 동일 시점 대비 우세 — goal reached 71.8%,
  success@timeout 62.6%(MLP ~52%), **crash 37.4%(MLP ~46%)**, 커리큘럼 12.5 m 도달.
  새 대시보드 박스·TB `navrl/*` 스칼라 모두 정상 동작 확인.

### 성공 반경 1.0 → 0.5 m 수정 (사용자 지적, NavRL 원본 확인)
- 사용자 지적: 요격 연구인데 1 m는 "근처 통과"지 "포획"이 아님 — 0.5 m가 맞지 않냐.
- NavRL env.py:583 확인: `reach_goal = (distance < 0.5)` — **원본도 0.5 m**. 기존 1.0 m는
  연구계획서 초안의 플레이스홀더("예: 1 m")가 그대로 들어간 것으로, 원본 사양과 달랐음.
- `success_radius = 0.5`로 수정 (goal reached / success@timeout / 커리큘럼 게이트 모두 이 값).
- **주의**: 이전 run들(1608 등)의 reach 지표는 1.0 m 기준이라 새 run과 직접 비교 불가.
  0.5 m 기준이 더 어려워 커리큘럼 확장도 느려질 수 있음 — 정상.

### 다음 단계
1. `ppo_navrl_cnn.yaml`로 6000 epoch 본 학습 (0.5 m 기준) → MLP 베이스라인과 비교.
2. 부족하면 보상 재균형(속도 vs 안전 가중), 커리큘럼 최종거리(18 m) 완주 확인.

---

## 2026-07-13 — loiter(배회) 진단 → 리워드 재설계 + 아레나 대형화

### 캡처종료 첫 본학습 결과: 정책이 "포획" 대신 "배회"를 학습 (실패)
- run `ppo_260713_1950_navrl` (캡처종료 + CNN + alive −0.05 + capture 30, 6000 epoch 완주).
- TB `navrl/*` 실측: **captured_rate가 ep46에 0.796까지 갔다가 붕괴 → 최종 0.067**,
  timeout 0.838, crash 0.095(94%→9.5%로 회피는 잘 배움), mean_closest_approach ~1.36 m.
- **근본 원인(산술)**: 리워드의 전역 최적해가 "포획"이 아니라 "목표 근처 배회".
  안전항 `mean(log(lidar_dist))`이 개방공간에서 **+log(4)≈+1.386/step 고정 수입** →
  `V_loiter ≈ 1.336·Σγ^k ≈ 104` ≫ `V_capture = 30`(캡처 시 에피소드 종료로 미래보상 포기).
  alive는 이미 −0.05(비용), capture도 이미 30으로 올렸지만 **안전항 수입이 진짜 범인**이라 미달.
- NavRL 원본엔 이 문제 없음: 먼(24–48 m) 목표로의 flyby 항법이고 reach는 통계일 뿐
  (종료·보너스 없음). 정밀 캡처 요구는 우리 요격 각색이 만든 새 문제.
- **최종 요약기 버그**: 종료 시 콘솔/`run_summary.json`이 옛 intercept 요약기라 "요격 0건 /
  target dist — / reward 낮음"처럼 **엉뚱·빈 필드**를 찍음. 실제 신호는 TB `navrl/*`에만 있음.
  (navrl용 요약기로 교체 필요 — 아직 미수정.)

### 리서치 워크플로우 → 방안 A~F
- 5-에이전트 병렬 조사(PBRS 이론 · 드론 goal-reaching 리워드 · 함정 · 기존 aerial_gym 요격 메커니즘).
- 기존 `shooting_moving_target_task`가 **동일 "stop-short" 버그**를 finish-funnel + 마일스톤 +
  closing bonus + (dense 클램프/터미널 정규화)로 **97.5% 성공**시킨 이력 확인
  (`lms_rl_trial/shooting_fixed_target/INTERCEPT_TUNING_REPORT.md`, 패치 #1~#10 교훈).
- 핵심 교훈: **A·D(포텐셜형)는 최적성 보존이라 단독으론 loiter 최적해를 못 옮김** →
  반드시 "최적해를 옮기는 비-포텐셜 변경(B)" + "학습가능하게 하는 조밀신호(A/C)"를 짝으로.

### 적용(활성) — B1 + B3 + A
- **B1** 안전항 재베이스라인: `navrl_task.py:add_static_safety_reward`에서
  `r_ss = (log(dist_m) − log(range)).mean()` → 개방공간 수입 +1.386을 **0으로**. 상수 시프트라
  **회피 그래디언트 불변**, loiter 수입만 제거(`V_loiter` 104 → ≈ −4.6). `import math` 추가.
- **B3** `collision_penalty −10 → −20` (config) — 자살 가드(무캡처 −12.5보다 크래시가 더 나쁘게).
- **A** PBRS progress: `compute_state_reward_and_terminations`에
  `reward += progress_weight·(prev_dist − γ·dist)`, 종료(capture/crash) 시 Φ=0(Grześ 2017),
  timeout은 truncation이라 그래디언트 유지. `progress_weight=1.0`, `progress_gamma=0.99`(=PPO γ),
  `self.prev_dist` 버퍼(__init__/reset_idx 시딩).

### 주석처리(DISABLED — 학습 후 이상하면 해제)
- **C1 finish-funnel**: `navrl_task.py` 리워드 블록 + `navrl_task_config.py`의
  `funnel_coef/outer/width`. 0.5–1.0 m 셸에서 "계속 안쪽으로 이동" 보상(brake-short 방지).
- **F 세그먼트 캡처**: `navrl_task.py` 4곳(prev_pos 버퍼/reset/세그먼트 판정/갱신). 2 m/s·0.1 s에서
  0.5 m 구를 스텝 사이로 터널링하는 fly-through를 잡음. 켜려면 4곳 모두 해제.

### 환경 대형화 (커리큘럼 18 m를 실제로 도달가능하게)
- **아레나 10×10 → 24×24×3 m** (`navrl_bars_env.py`, bounds [0,−12,0]~[24,12,3]). 전엔 18 m 목표가
  아레나 밖이라 ~10 m로 clamp됐음(대시보드 curriculum_max 18 표시가 실제와 불일치).
- **막대 16 → 48개** (`bar_asset_params.num_assets`), ~8/100 m²(NavRL ~22/100 m²보다 희소 — VRAM.
  참고: RESEARCH_PLAN의 "2.2/100 m²"는 10배 오타, 실제 NavRL은 ~22). spacing 1.3 → 1.8.
- **에피소드 150 → 250 스텝** (18 m 목표: 직선 ~90, 위빙 ~150+ 스텝이라 타임아웃이 실패요인 안 되게).
  (→ 이후 `episode_len_steps=300`으로 상향(24 m cross-field 대각 goal 대응) — config가 현재 최신값.)

### 새 학습 시작
- run `ppo_260713_2210_navrl` (256 env, 48 장애물 확인, fresh weights, 6000 epoch) 시작됨.
- **VRAM**: 옛 512 env×16막대 ≈ 6.8 GB. 48막대는 메시 3배 → `--num_envs 256`부터, OOM이면 128.
  에피소드 250이라 epoch도 ~1.7배 느려짐.
- 판정: `captured_rate ↑`(→0.9) · `crash/timeout ↓` · `mean_closest_approach → 0.5 m`.
  여전히 마지막 1 m서 멈추면 C1 해제, 캡처 누락 의심되면 F 해제.

### 남은 정리(TODO)
- 최종 요약기(`run_summary.json`/콘솔)를 navrl 지표(captured/crash/timeout/closest/curriculum)로 교체.
  (→ 이후 완료: `train_run_recorder.py`에 `is_navrl` 분기 + `_build_navrl_hints` + navrl 요약 박스/density bars 필드.)
- 위 리워드/환경 변경은 전부 **미커밋 working tree** — `git diff` 검토 후 커밋. (→ 이후 커밋됨 `be701eb`)

---

## 2026-07-14 ~ 07-15 — Phase 1 완성(cross-field + yaw 제어) → Phase 2 밀도 착수

Phase 1을 정직한 난이도로 재설계하고, 잔여 충돌의 진짜 원인(기하)을 찾아 **yaw 제어로 해결(captured 0.95)**,
이어서 Phase 2 밀도 스윕에 착수한 이틀. 상세 결과는 아래, 실험 로그는 `CRASH_TUNING_LOG.md`.

### 새 스폰 스킴 + 0.28m 박스 충돌 (커밋 `71aa606`)
- **크로스필드 스킴**: 드론 x≈0(왼쪽 가장자리) → 목표 x=k(먼쪽) → 매 에피소드 **48개 균등 막대밭 전체 관통**.
  옛 스킴은 목표가 드론 쪽에 몰려(관통 8%뿐) 성능이 부풀려졌음(captured 86%).
- **드론 충돌 = 0.28m 박스**(팔/프롭 포함, `quad_navrl_collide.urdf`, navrl 전용). 옛 0.05m 구 대비 단면적 ~10배.
- 커리큘럼: epoch비례 k_max 7→24m, 이후 k_min 5→20m(램프 2000→5000 epoch). checkpoint에 num_task_steps 저장.
- **run `ppo_260714_1904`: captured 0.65 / crash 0.35 / timeout 0 = 정직한 Phase-1 baseline.**

### crash 레버 실험 → "기하 바닥" 발견 (`CRASH_TUNING_LOG.md` Run B/C/D)
- 멀티에이전트 워크플로우로 speed-gated clearance 페널티 설계(cw=6). run `2207`: **captured 0.66 / crash 0.32.**
- **판정: 감속은 시켰으나(mean_ep_len 88→116) crash 거의 안 줄임.** 큰 감속에도 crash 불변 ⇒ **원인은
  속도/리워드가 아니라 기하(corner-clip)**. 리워드로 crash 줄이기 소진 → cw=0 복원(`44e86a2`), k_min 스케줄
  2000→5000(`8b97909`).

### ★ yaw 제어 (option b) → Phase 1 해결 (커밋 `4459281`, run `ppo_260715_0251`)
- **원인 확정**: 요가 스폰 ±30°로 **고정**돼 이동방향과 어긋나면 0.28m 박스가 **대각 0.40m**로 갭을 쓸어 corner-clip.
  NavRL 원본은 모터레벨로 yaw를 학습(=option b) + 목표 바라보고 스폰.
- **구현(멀티에이전트 3설계→적대검증→종합)**: action 3→4(`action[:,3]`=yaw-rate), **관측 heading 추가**
  (S_int 8→12: goal_bearing_body+vel_body_xy, vehicle-frame — 없으면 학습 불가), NavRL 전용 컨트롤러
  `lee_velocity_control_navrl`(yaw clamp π/3→2.5), speed-gated crab 페널티(0.3)+yaw-rate² 댐핑(0.02).
  속도액션은 goal-frame 유지(요와 디커플→nav 안정). 런타임 스모크 검증 후 커밋.
- **결과: captured 0.954 / crash 0.046 / timeout 0** (k_min=20 최심에서도 0.950). **crash 0.32→0.046(7×↓),
  NavRL 0.81·M1 0.90 상회. Phase 1 사실상 해결.** 기하 가설 완전 입증.
- **yaw-off ablation 브랜치** `ablation/yaw-off`(@`8b97909`) 준비 — 0251에서 yaw만 뺀 대조군(논문 attribution용).

### Phase 2 밀도 배관 (커밋 `7e1b6a8` + 수정 `0aed4be`)
- 환경변수: `NAVRL_MAX_BARS`(빌드천장, 기본 150) / `NAVRL_NUM_BARS`(활성수) / `NAVRL_DENSITY_CURRICULUM`.
  **`keep_in_env=False`로 런타임 활성 막대수 제어**(`navigation_task`의 검증된 패턴, `num_obstacles_in_env`를
  리셋마다 읽음), spacing 1.8→1.5로 150개 수용, x밴드 0.09→0.13. checkpoint에 `n_bars_active`.
- 수정(`0aed4be`): 밀도 커리큘럼 워밍업 누적기 리셋 버그 + max-bars 조용한 0 경고 추가(커리큘럼 경로 한정).
- **VRAM 스크리닝**: 150 bars + 256 env ≈ **6.1GB** (8GB OK, num_envs 안 낮춰도 됨).
- **run `ppo_260715_1552` (150 bars ≈ 26/100m²): captured 0.85 / crash 0.15 / timeout 0.** NavRL 밀도에서도
  NavRL 0.81 상회. **밀도-성능(RQ1): 48막대 0.95 → 150막대 0.85.** 나머지 {25,50,75,110} seed 1 진행 예정.

### 인프라 / 문서
- 실행 래퍼 `train_navrl.sh` / `play_navrl.sh`(짧은 실행, VRAM는 `NUM_ENVS`), 2번째머신 원샷
  `bootstrap_second_machine.sh` + `SETUP_SECOND_MACHINE.md`(urdfpy 직접 clone·rsync 회수).
- `RESEARCH_PLAN.md`·`ROADMAP.md`를 repo 안으로 이동(전엔 repo 밖이라 clone에 안 왔음). 콘솔 박스·progress
  로그에 run 이름 표시. 옛 run 24개 요약 후 삭제(`RUNS_ARCHIVE_SUMMARY.md`). upstream 원격 제거.

### ★ 막대 배치 방식 전환: 지터드 격자 → NavRL식 랜덤 rejection (커밋 `54d7cfe` + `10708e2`, 20:26~20:35)
- **변경**: `asset_manager.py`에 `_random_rejection_xy_spacing` 추가 + env config에 `obstacle_placement_mode`
  스위치. `navrl_bars_env.py`가 `mode="random"`으로 전환(밴드 균등 스캐터 + 최소간격 rejection, 포화 시
  128회 실패마다 간격 ×0.8 완화 = NavRL "do not stall when saturated" 트릭). 후보를 32개씩 배치 샘플링해
  파이썬 per-candidate 루프 회피(GPU 벡터화).
- **이유**: 격자는 min-spacing을 결정론적으로 보장하지만 **눈에 보이는 행/열 아티팩트**가 있어 밀도 실험의
  일반성을 해침. NavRL 원본이 랜덤 스캐터라 사양 충실도도 ↑. **opt-in**(env config에서 `mode` 지정, 기본
  `grid`)이라 타 환경 무영향. 07-10에 "rejection→격자로 전환(사용자 승인)"했던 결정을 **뒤집은 것**:
  당시 실패 원인(16개@1.3m 균등)은 완화 로직 + GPU 배치로 해소됨.
- **⚠️ 스윕 교란 주의**: run `1552`(150막대, **격자**, captured 0.85)·`1922`(75막대, 격자, 중단)는 **옛 격자**
  결과라 새 랜덤 배치 run과 **직접 비교 불가**. 배치 방식이 밀도-성능 곡선(RQ1)의 교란변수가 됨.
  → run `2038`(150막대, **랜덤**, seed 1) = 새 배치 기준의 150막대 재-baseline. **나머지 밀도{25,50,75,110}도
  전부 랜덤 배치로 재실행해야 한 곡선 안에서 비교 가능** (격자 데이터포인트는 폐기).

### 2026-07-15 문서 정합성 리뷰 결과
- **밀도 분모 통일 완료**: 실제 배치 밴드 x-ratio[0.13,0.96]×y[0,1] = **약 478 m²** 기준으로
  `RESEARCH_PLAN.md`를 갱신. 48→10.0, 75→15.7, 150→31.4 /100m².
- **VRAM 안내 수정 완료**: 150막대+256env≈6.1GB 실측에 따라 고밀도에서도 256 env 유지 가능하다고 명시.
- **episode 길이 정정 완료**: 실제 config 최신값 `episode_len_steps=300`을 07-13 기록에 후속 표기.
- (코드 소소) `navrl_task._clamp_active_bars`/`_set_active_bars` int-cast 중복, `train_run_recorder`가 navrl run에도
  옛 intercept 컬럼(near_miss/surface_gap 등)을 빈칸으로 계속 기록 — 동작엔 무해, 정리 여지.
- 문서 전반 옛 스킴 흔적(10×10·16막대·150스텝 등) → 현재 스킴으로 동기화.

---

## 2026-07-16 ~ 07-17 — 밀도 스윕 완성 → Phase 3(이동 표적) 착수

### 밀도 스윕 결과 (랜덤 배치, seed 1, 각 6000 epoch)
| 막대수 | 밀도(/100m², 밴드478) | captured(last-500) | crash |
|---|---|---|---|
| 25 | 5.2 | 0.972 | 0.026 |
| 50 | 10.5 | 0.970 | 0.030 |
| 75 | 15.7 | 0.942 | 0.058 |
| 110 (NavRL 앵커) | 23.0 | 0.926 | 0.074 |
| 150 | 31.4 | 0.656 | 0.344 |
- **NavRL 앵커까지 평탄 → 110→150 절벽(−27pt).** 기하 분석: 밴드 478m²의 RSA 재밍 한계 ≈148,
  적응완화 시작 ≈115-120 → 절벽은 배치 기하의 한계와 일치(선형 아님 = 논문 핵심 스토리).
- ⚠️ 25/50은 4GB 노트북에서 **minibatch 정합(512→4096) 이전** 학습 = 업데이트 16배 confound.
  정성 결론 유효, 정량 인용 주의. GPU4GB 프리셋 정합 후 재실행 예약.
- 잔여: 150 seed2(진행), 120막대(절벽 경계), 25/50 재실행.

### Phase 3 착수 — 문헌조사·계획(`PHASE3_PLAN.md`, 커밋 `460366a`) 후 구현
- **사전 전면 감사**(6렌즈 멀티에이전트): 실버그 4(timeout 스텝이 다음 에피소드 스폰스캔으로 safety
  보상 받는 leakage, play의 `--seed` 증발, play_navrl.sh GPU4GB 미적용, set_env_state가 checkpoint
  밀도로 NAVRL_NUM_BARS를 덮어씀) + 죽은코드 ~12(C1 funnel, `class eval`, `full_scale_hold_epochs`,
  clearance 경로, `terminate_on_capture=False`/`return_state_before_reset` 분기, `_REACHED`,
  `quat_rotate`/`torch` import 등) + 문서 드리프트 ~15 — **전부 수정/삭제**.
- **이동 표적 v1 구현** (기본 OFF = Phase 1/2와 byte-호환):
  - `_advance_target()`: 가상점(액터 0, LiDAR 불가시), **rl_dt=0.1s**(물리 dt×10 — `obs_dict["dt"]`
    직접 쓰면 1/10 속도 나는 함정을 회피), 패턴 cv(벽반사)/waypoint/circle(평가전용)/mixed,
    막대 clearance push-out + 벽 클램프, **realized velocity**(반사·push-out 포함 실제 변위/dt)를
    리워드에 공급.
  - **range-rate 리워드**: `(v_drone − v_target)·dir` — v_t=0이면 기존 항과 IEEE-정확히 동일.
  - **PBRS re-anchor**: `‖prev_pos−target_new‖ − γ‖pos−target_new‖` — 표적 이동분을 드론 상벌에서
    제거(naive형은 도주 표적에서 정지 드론에 −0.94/step 오벌점). prev_dist 버퍼 폐지.
  - **상대프레임 세그먼트 캡처**: prev_rel→rel 선분 vs 원점 0.5m — 상대속도 4m/s 터널링 방지,
    v=0에서 point 테스트의 strict superset(수학 유닛테스트 `scratchpad/p3_math_unit.py` 전항 PASS).
  - 속도 커리큘럼 0→`NAVRL_TARGET_SPEED_FINAL`(epoch비례, num_task_steps 기반=checkpoint 생존),
    평가 노브 `NAVRL_TARGET_SPEED`/`NAVRL_TARGET_PATTERN`/`NAVRL_SEED`. 대시보드/TB에
    `navrl/target_speed_{max,mean}_m_s` 추가.
- goal 배치: 10회 rejection 실패 시 최근접 막대에서 방사 snap-out(고밀도 안전망).

## 2026-07-17 — 비전 피벗: 센서 전용 요격 (Stage 0.5 + Stage 1)

**방향 전환(사용자 결정)**: actor가 표적의 GT 상대위치를 관측에서 받지 않는다. LiDAR+비전(카메라)
정보만으로 표적을 찾아 요격. 이것이 논문의 최종 문제 설정이 됨 (기존 GT-주입 경로는 baseline).

- **Stage 0.5 — 해석적 표적 주입** (`38a4f22`): 이동 표적을 LiDAR 커널 내 ray-sphere로 주입
  (`NAVRL_VISION=1`, semantic id 50). 메시+refit 방식은 refit 루프(Python, env당 wp.Mesh.refit)가
  81ms@N128로 학습 불가(8.5 steps/s) → 해석적 주입은 오버헤드 ~0 (25.8 vs 기준 25.0 steps/s).
  검증: `tools/test_navrl_p3_stage0.py` ALL PASS. GPU 이전 가이드 `GPU_SCALING_GUIDE.md` 추가.
- **Stage 1 — 센서 전용 actor + 비대칭 critic**:
  - actor 관측 305 = ego 9(기체속도/요레이트/직전행동/고도) + 검출기 8 + LiDAR 2채널 288(range+표적마스크).
    GT 무누출: blind 상태에서 표적 위치를 바꿔도 관측 완전 동일(프로브 C 검증).
  - 검출기(`navrl_detector.py`) = 전방 카메라 모델(HFOV 87°/20m): FOV+거리+**막대 가림(warp LOS 레이)**
    통과 시에만 방위/거리 노출 + last-seen 추적기(온보드 필터 표준). Stage 2에서 실픽셀로 교체 가능.
  - 행동 = goal프레임 → **기체(vehicle)프레임** (표적 미지 → goal프레임 정의 불가).
  - 비대칭 critic: `states` 313 = actor관측 + GT 8, rl_games `central_value_config` 네이티브
    (runner.py {obs,states} 래퍼 + state_space). 배포 시 critic 폐기 → actor는 센서만.
  - 보상: 기존 유지(GT는 환경 심판) + 가시성 보너스 0.02 + 장외(OOB) 종료.
  - 네트워크 `navrl_vision`(2ch CNN) + 옵션 LSTM(dones-masked, `ppo_navrl_vision_lstm.yaml`).
  - 실행: `NAVRL_VISION=1 ./train_navrl.sh` (LSTM: `NAVRL_LSTM=1` 추가).
- **검증**: stage1 프로브 ALL PASS(가시/후방/원거리/가림/무누출/추적기/행동프레임/states);
  vision off 스모크 ALL PASS(156 무손상); sanity 학습 40ep(비대칭 critic 저장 확인, 입력 313,
  ep_len 19→32 성장) + LSTM 10ep(가중치 저장 확인). sanity run 2개(1522/1525)는 삭제 가능한 테스트.
- **8GB 판단**: 의미론 LiDAR+검출기 경로는 VRAM 추가 ~0. raw 카메라 CNN은 Stage 2(옵션)로 격리.

## 2026-07-18 ~ 07-19 — 센서 전용 밀도 커리큘럼: 첫 결과 + 진단 + 순차 전환

**평가(play) 경로 수정 (커밋 eaad795)**: 비전 체크포인트가 play_navrl.sh에서 안 돌던 2버그
(156-dim cnn yaml 오선택 / {obs,states} dict를 player가 못 먹음) 수정. 이제 held-out 평가 됨.
- **110막대·정적 from-scratch (baseline)**: captured **47.3%** / crash 52% / timeout 0.7% (n=2050).
  timeout≈0 → 표적은 찾음, 실패=충돌. 병목=장애물 회피(밀도).

**4GB 경로 (커밋 3f50639)**: `ppo_navrl_vision_4gb.yaml`(N=128, mb 2048) + train/play 스크립트가
`NAVRL_VISION=1 GPU4GB=1`에 vision_4gb 선택. **1650 Ti("joshualisky")에서 실기 검증됨**.

**밀도 커리큘럼 (커밋 c5bd707 knob) — 첫 run `1841`**: `NAVRL_DENSITY_CURRICULUM=1 WARMUP=1000
THRESHOLD=0.6`. 밀도 knob들 env화(기본 threshold 0.8/warmup 2500는 GT용이라 센서엔 부적합).
- **밀도 계단 25→130 성공적 상승**: 승급별 안정 캡처 25:93% 55:79% 85:61% 100:59% **115:62%** 130:47%(정체).
- ⚠️ **그런데 최종 체크포인트 110 held-out = 40.8%** (crash 50%, **timeout 8.8%↑**) → **baseline 47%보다 낮음**.
  학습 중 62%(115막대)는 목표가 중간거리였을 때 값. 최종 정책은 130막대+깊은목표(20m) 과특화 → 소심해져
  먼 목표 timeout. **두 커리큘럼(거리·밀도)이 끝에서 충돌**. rl_games가 후반 ckpt만 보관해 좋은 중간정책 유실.
- 정정: "깊은목표=시야밖" 진단은 일부 오류(baseline은 깊은목표 timeout 0.7%로 도달함). 진짜 원인=과특화.

**1650 Ti seed-2 (`vision50_seed2`)**: 50막대 peak **92.5%** (seed-1 93.2%와 일치 → 앵커 재현성 확인).
후반 캡처 하락(70%)은 goal 커리큘럼이 목표를 20m로 밀어서(밀도 아님).

**거리 커리큘럼 knob env화 (커밋 24d7036)**: `NAVRL_K_FINAL`/`NAVRL_K_MIN_FINAL`/`NAVRL_K_WARMUP`
(기본 24/20/3000). 센서 전용은 목표를 검출기 시야(20m) 안에 묶을 수 있게.

**다음 계획 — 충돌 회피 두 해법을 두 머신에서 병렬:**
- **3070 = 순차(sequenced)**: 거리 먼저(6000ep, 25막대 고정) → 밀도 나중(6000→12000, 목표 고정).
  단일 명령으로: `NAVRL_DENSITY_CURRICULUM=1 NAVRL_DENSITY_WARMUP=6000 NAVRL_DENSITY_THRESHOLD=0.6
  ./train_navrl.sh --seed 1 --max_epochs 12000` (warmup 6000이 밀도를 6000판까지 막음 → 자동 2단계).
- **1650 Ti = 목표제한(goal-capped)**: `NAVRL_K_FINAL=16 NAVRL_K_MIN_FINAL=10` (목표≤16m 시야내) +
  밀도 커리큘럼 60캡 + GPU4GB. 6000ep. 동시 램프지만 목표가 얕아 충돌 완화.
- 승급마다 체크포인트 스냅샷 필요(중간정책 유실 방지). 밀도별 평가로 곡선 완성이 목표.

## 2026-07-20 — 정정: 밀도 커리큘럼은 성공 (체크포인트 선택 오류였음)

**중대한 정정**: 앞선 "밀도 커리큘럼 최종 110=40.8% < baseline" 결론은 **틀렸다**. 원인 = 평가에
`gen_ppo.pth`(best-reward)를 썼는데, 밀도 커리큘럼에선 **보상이 저밀도(25막대)에서 최고**라
`gen_ppo.pth = 저밀도 정책`이었음. 고밀도 평가엔 `last_gen_ppo_ep_XXXX`(마지막, 고밀도 특화)를 써야 함.

**올바른 110막대 held-out (마지막 체크포인트):**
- from-scratch (고정 110): 47.3%
- 순차 Phase B (거리먼저→밀도): 56.2% (+9pt)
- 동시 1841 (거리·밀도 동시): **60.6%** (+13pt, crash 37% timeout 2.2%)

**결론:**
1. **밀도 커리큘럼은 확실히 통한다** (47→60%). 이전 "충돌로 실패" 서사는 평가 아티팩트였음.
2. **순차가 동시를 못 이김** (56 vs 60) → 더 단순한 동시 방식으로 충분. 2단계 불필요.
3. **마지막(고밀도) 체크포인트가 낮은 밀도에도 잘 일반화** → 하나의 강건한 정책을 밀도별 평가하면
   그게 곧 논문 밀도-성능 곡선. (130 학습 정책이 110에서 60%.)

**warm-start 인프라 (커밋 e7f8ca3)**: `--max_epochs` override + critic `_orig_mod` 자동정규화
(warm-start 시 torch.compile 접두사 문제). Phase B가 이걸로 정상 resume·완주(5648→12000).

**다음**: 1841 마지막 ckpt를 25/50/75/110/130/150에서 평가 → 센서전용 밀도곡선. 1650 Ti는 평가 전담
(학습 N=128 페널티로 학습결과는 3070과 못 섞음; 평가는 N무관이라 OK).
주의: **밀도 커리큘럼 run 평가는 항상 last_ ckpt 사용** (gen_ppo.pth=저밀도 best 함정).

---

## 2026-07-22 — 충돌률 급증 진단 + 3D 연구현황 대시보드 + research-status 스킬

`ppo_260719_1000`(12000ep 순차) 종료 확인: **peak 97.1%(ep 3764) → final 44.9% / crash 54.1%(막대 115·목표 24 m)**.
사용자 우려("갑자기 충돌률↑ = 이상")를 3-에이전트 병렬 조사(run분석·커리큘럼감사·운영리뷰)로 정밀 진단.

### 진단 — "급락"은 절반 아티팩트, 절반 진짜 기하 바닥
- **아티팩트**: peak→final −52pt는 밀도·거리 커리큘럼이 끝에서 동시 최대(115막대+24 m)로 가서 최종 ckpt가
  최난이도 셀에 과특화된 것(정책 붕괴 아님). 07-20 정정대로 **`last_gen_ppo_*`** held-out하면 110막대 60.6% 정상.
  훈련중 final-epoch 지표/`gen_ppo.pth`(저밀도 best)로 밀도 비교 금지.
- **진짜 기하 바닥**: 센서 held-out 충돌 110→130 **+13.5pt**, 130→150 **+19.9pt**(로깅버그 아님). 원인 =
  배치 최소간격 **1.5 m가 128실패마다 ×0.8 완화**(→1.2→0.96 m), ~115–120막대에서 완화 시작. 2단계(0.96 m)면
  평균막대(0.6 m) 기준 **틈 0.36 m < 드론 대각 0.40 m** → 통과불가 → 충돌. GT는 120막대까지 평탄(1.8%)·150만 절벽
  = 같은 RSA 재밍 한계(~148/478 m²). **회피(충돌)가 병목이지 탐지 아님**(timeout은 저밀도서 오히려↑).

### 커리큘럼 감사(코드 확인) → 순차 밀도 레시피 확정
- 밀도 커리큘럼 기본값: `STEP=+15`·`THRESHOLD=0.8`·`WARMUP=2500`·`CHECK_EPS=2048`, capture-gated 승급(25→150).
  거리: `K_FINAL=24`/`K_MIN_FINAL=20`/`K_WARMUP=3000`, num_task_steps/32 램프. 표적속도: `SPEED_FINAL` 기본 0.
- **확정 권장(완만+충돌안전)**: `NAVRL_DENSITY_STEP=5 THRESHOLD=0.55 START=25 FINAL=110`(완화절벽 ~115 직전 정지)
  + **거리 얕게 캡 `K_FINAL=16 K_MIN_FINAL=10 K_WARMUP=8000`**(두 커리큘럼 끝-충돌 제거) + 승급마다 `last_gen` 스냅샷.
  단계별 명시 스테이징(NUM_BARS 고정 25→…→110, --checkpoint resume)은 ckpt 통제 최상 대안.

### 인프라 — 3D 웹 대시보드 + 재사용 스킬
- **`docs/status/`** (index.html+app.js+status.json): three.js 3D 아레나(막대/표적속도 슬라이더·LiDAR 토글) +
  최신현황·밀도곡선·run타임라인·진단·다음계획·로드맵. `status.json` 구동(데이터 자동반영), 오프라인/3D실패 시
  데이터패널은 유지되게 graceful degrade. **research 브랜치 `/docs`에서 GitHub Pages 게시**(gh-pages 브랜치 불필요).
  커밋 `8d25c89` push 완료 → Pages 활성화(Settings→Pages, branch=research/navrl-env, folder=/docs) 후
  `https://joshualikaist.github.io/MOTAR/status/` 라이브.
- **`.cursor/skills/research-status/`**: 이 워크플로우(지표수집→서브에이전트 분석→종합→대시보드→게시→WORKLOG)를
  순차 실행하는 스킬. `collect_status.py`(runs+CSV→status.json, 토큰절약 핵심), `publish_dashboard.sh`(docs만 커밋·push).

### 코드 리뷰(서브에이전트) + 수정
정직한 결함 리뷰 결과 **Critical 버그 없음** — 정적 LiDAR(P1–2) 태스크·리뷰에서 의심됐던 부분(swept-segment 캡처가
순간 dist<0.5의 상위집합, 정적표적 byte-동일, 밀도 accumulator/warmup/clamp/전파, 커리큘럼 min<max 불변식,
종료 aliasing, obs 차원 분할, PBRS Φ=0)은 전부 정상 확인. **수정한 3건**:
- **[Suspicious] `NAVRL_NUM_BARS` 무시 버그**: 밀도 커리큘럼 flag가 켜져 있으면 `_initial_active_bars`가
  `n_start`(25)를 반환해 explicit `NAVRL_NUM_BARS`를 조용히 무시 → 밀도별 eval/resume이 25막대로 잘못 돌던 문제.
  `_initial_active_bars`가 explicit `NAVRL_NUM_BARS`를 최우선하도록 수정(set_env_state의 규칙과 일치). 검증 완료.
- **[Minor] env-var 파싱 무음 삼킴**: `_env_int/_env_float`가 잘못된 값(`NAVRL_K_FINAL=abc`, `1.5m`)을 조용히
  기본값으로 → stderr 경고 추가.
- **[Minor] `_env_bool`이 `1.0` 같은 값을 False로**: 숫자형 허용(`float(s)!=0`)으로 수정(`NAVRL_VISION=1.0` 함정).
**미수정(플래그만)**: ① vision 모드 mid-episode 리셋 첫 프레임에서 LiDAR 표적마스크가 옛 목표를 가리키는
1프레임 불일치(리워드 무관, 수정은 step 순서 재배치라 별도 스모크 필요) ② PBRS re-anchor는 표적 이동 시
엄밀 PBRS 아님(정적=정확, 문서화된 의도적 트레이드오프) ③ episode 301스텝 off-by-one(재현성 영향 우려로 미변경).

---

## 2026-07-22 — 연구 핵심 재정의: oracle semantic → raw-sensor learned perception

사용자 결정으로 Phase 3의 기준을 다시 고정했다. 드론은 `target_position`, GT bearing/range 또는
semantic target mask/id를 받아 추격하면 안 된다. camera RGB-D와 LiDAR의 원시 시계열 관측에서 표적의
상대 위치·속도·가림·불확실도를 직접 추정해야 한다.

### 기존 구현의 판정

- analytic LiDAR target sphere, semantic id 50, analytic camera target mask/depth는 센서 geometry,
  occlusion, 처리량을 확인한 **prototype/upper-bound**다.
- camera obstacle depth + LiDAR range/semantic fusion이 256 env에서 약 8,180 env-steps/s로 동작한 것은
  engineering baseline으로 보존한다.
- 그러나 semantic target identity를 actor에 넣는 checkpoint는 논문의 최종 perception checkpoint가 아니다.

### NavRL++ 원문 확인 후 계획 교정

- NavRL++의 Transformer는 raw sensor detector가 아니라 perception이 만든 structured obstacle/state history를
  처리하는 temporal actor–critic backbone이다. 원문 설정은 12 tokens, 2초/0.5초 history, dim 64,
  4 heads, 4 layers, FFN 128, 약 0.31M parameters다.
- 본 연구는 이를 `[CLS]+static+dynamic history+robot history+target history`의 17 tokens로 확장한다.
  Transformer를 주 모델로 확정하고 CNN/LSTM은 ablation baseline으로 둔다.
- RGB-D와 LiDAR가 각각 obstacle/target proposal을 직접 만들고 association/Kalman tracking으로 structured
  state를 생성한다. semantic target id와 GT pose는 actor에 제공하지 않는다.
- NavRL++ ablation상 Transformer 단독의 명확한 이득은 success 자동 향상이 아니라 control effort 감소이며,
  최종 성능은 perception-failure-aware fine-tuning과 결합할 때 개선됐다. 따라서 target detection drop,
  latency, noise, calibration perturbation fine-tuning을 필수 단계로 추가했다.
- 구현 순서: **information firewall → dual-sensor detector → fusion/tracker → 17-token Transformer →
  perturbation-aware fine-tuning → PPO**.

---

## 2026-07-23 — Perception+Transformer 초기 학습 run 정리 (중단 사유 기록)

`NAVRL_PERCEPTION=1` + `ppo_navrl_perception_transformer.yaml` 첫 본학습 시도들. 아래 run·로그는
학습 가치 없거나 붕괴된 것으로 판단해 **삭제함** (`runs/` + `train_session_logs/`). **유지:**
진행 중인 `ppo_260723_1241_navrl` (12:41 시작, tuned env).

| run / log | ep | peak cap | 종료 이유 |
|---|---|---|---|
| `ppo_260722_1928` / `train_260722_1928` | 615 | 3.8%@443 | **수동 중단.** perception Transformer 첫 시도(128env, 밀도 커리큘럼 on, 25막대). 최종 crash **100%**·cap 0% — from-scratch로는 학습 신호 없음. |
| `train_260722_2044` | 0 | — | **즉시 실패.** conda 미활성화로 `ModuleNotFoundError: yaml`. run 폴더 없음. |
| `ppo_260722_2045` / `train_260722_2045` | 256 | 6.1%@81 | **수동 중단.** **48막대**로 시작(설정 실수로 추정). crash 97%·cap 3% — perception cold-start에 밀도 과다. |
| `ppo_260722_2058` / `train_260722_2058` | 4074 | 4.8%@91 | **포기.** 거리 커리큘럼 기본 `K_FINAL=24`가 ep~3000에 k_max=24 m까지 올라갔으나 cap은 ep1000 이후 **0% 고착**, crash **100%**. perception from-scratch에 원거리 커리큘럼이 너무 공격적. |
| `train_260722_2342` | +124 | — | 2058 resume 시도(`last_gen ep3950`). ep3950→4074만 추가 후 중단 — resume도 회복 불가 확인. |
| `ppo_260722_2349` / `train_260722_2349` | 63 | 21.6%@54 | **조기 중단.** fresh restart, 초반 cap 13~22%로 1928/2058보다 나았으나 레시피 재튜닝 위해 중단. |
| `ppo_260723_0011` / `train_260723_0011` | 1051 | **42.9%@314** | **수동 중단 (NaN 붕괴).** ep314까지 유망(cap 43%, crash 36%) → ep~348부터 PPO gradient **NaN** → 100% timeout·reward NaN. ckpt unusable. |
| `ppo_260723_0225` / `train_260723_0225` | 1111 | 6.7%@79 | **수동 중단 (NaN).** 0011과 동일 패턴 — 초반 미약, 후반 NaN/timeout 붕괴. |
| `ppo_260723_1138` / `train_260723_1138` | 1563 | **48.3%@755** | **수동 중단 (정책 퇴화).** ep755 peak 48% → ep1563 cap 14%/crash 86%로 퇴화. `K_COMPETENCE`·`LIDAR_RANGE=8`·`K_FINAL=16` 등으로 **1241 run 재시작**. |

**교훈:** (1) perception cold-start는 25막대·얕은 목표(≤16 m) 고정부터. (2) default `K_FINAL=24` 거리 커리큘럼은 Transformer+detector 경로에 부적합.
(3) NaN은 ep300~500대에서 발생 — lr/grad clip/mean_std 안정화 점검 필요. (4) 1138→1241 전환 env:
`NAVRL_CRASH_DIAG=1 NAVRL_LIDAR_RANGE=8 NAVRL_K_COMPETENCE=1 NAVRL_K_FINAL=16 NAVRL_FOV_CURRICULUM_EPOCHS=1000000 NAVRL_NUM_BARS=25`.

---

## 2026-07-23 (저녁) — Perception+Transformer 2차 run 정리

1241 tuned run 이후 same-day 추가 시도들. **첫 perception 성공 run = `ppo_260723_1509`** (peak cap **98.6%@ep3343**,
`checkpoints_saved/peak_cap986_r165_ep3343*.pth` 보존). **현재 학습 중 = `ppo_260723_2210`** (peak ckpt에서 resume).

| run / log | ep | peak cap | 종료 이유 |
|---|---|---|---|
| `ppo_260723_1241` / `train_260723_1241` | 705 | 68.3%@136 | **수동 중단.** k_max **7 m에 고착**(거리 커리큘럼 미승급). peak 68%→final 46%. 1330/1509 레시피로 교체. |
| `ppo_260723_1311` / `train_260723_1311` | 584 | 20.0%@83 | **수동 중단 (NaN).** 1241과 중복 fresh start. ep~93부터 NaN → 100% timeout. |
| `ppo_260723_1330` / `train_260723_1330`, `train_260723_1418` | 1263 | **97.0%@488** | **수동 중단.** k_max=16 레시피 첫 성공(peak 97%). ep1263 final 72% — 1509 fresh restart로 교체. 1418=1330 resume 로그. |
| `ppo_260723_1424` / `train_260723_1424` | 607 | 17.3%@115 | **수동 중단 (NaN).** 1330/1509와 병렬 fresh start. ep~122부터 NaN. |
| `ppo_260723_1509` / `train_260723_1509` | **10000** | **98.6%@3343** | **완주(max_epochs).** perception+Transformer **첫 본학습 성공**. ep5000 cap 83% 유지 → ep~7000 NaN/timeout 붕괴(loiter farming). **평가·resume은 peak ckpt 사용** (`gen_ppo`=후반 loiter 함정). |
| `train_260723_1535`, `train_260723_1559` | — | — | 1509 resume 시도 로그(`gen_ppo`, `last_gen ep1000`). gen_ppo=저밀도/loiter ckpt라 무의미 — 세션 로그만 삭제. |
| `ppo_260723_2105` / `train_260723_2105` | 4267 | 96.2%@3792 | **수동 중단 (NaN).** 1509 `gen_ppo_rlnorm`에서 resume(잘못된 ckpt). ep3792 잠깐 96% → NaN 붕괴. peak ckpt resume으로 2210 전환. |

**유지:** `ppo_260723_1509_navrl`(1.5 GB, peak ckpt 원본), `ppo_260723_2210_navrl`(진행 중, peak ckpt resume),
`checkpoints_saved/peak_cap986_r165_ep3343*.pth`.

**교훈:** (1) 거리 커리큘럼 k_max=16 승급이 핵심(1241은 k=7 고착). (2) 1509 완주 후 **gen_ppo로 resume 금지** — peak cap ckpt만.
(3) NaN은 여전히 ep100~700대 random hit; peak ckpt resume(2210)은 ep3790 cap 82%로 정상.

---

## 2026-07-23~24 — 센서 전용 이동표적 요격 검증 + 급사(sudden NaN) 근본원인 수정 + 정직한 baseline

`checkpoints_saved/peak_cap986_r165_ep3343.pth`(98.6% 학습피크)를 기반으로 (1) 속도를 줄이지 않고
올리는 방향으로 이동표적 요격을 검증하고, (2) 반복적으로 학습을 죽이던 급사(sudden NaN)의 진짜
원인을 특정·수정하고, (3) 파라미터 튜닝의 기준이 될 정직한 baseline을 측정했다.

### 속도 상향 + 이동표적 요격 (검증 완료)
`NAVRL_YAW_RATE_MAX=3.0`(task+`lee_controller_config_navrl` 동시 상향, 2.0m/s에서 이미 위빙이
2.4/2.5rad/s를 요구 → 요레이트가 진짜 기동 한계, 추력 아님, T/W≈3.3) + `NAVRL_MAX_VELOCITY=2.5` +
`NAVRL_TARGET_SPEED_FINAL` 램프(0→1.5m/s)로 재학습. capture는 표적속도 전 구간에서 ~0.8 유지 —
드론이 표적보다 충분히 빠르면 속도 자체는 병목이 아님을 확인.

### ★ 급사 NaN의 근본원인 (수정: `navrl_transformer_network.py:149`, 커밋 `bb0faa7`)
건강하게 capture 0.8~0.9로 돌던 run이 ~epoch 5000대에서 a_loss가 **단 한 스텝**에 NaN → hover 붕괴하는
패턴이 반복(entropy_coef 0.005·0.003 둘 다). NaN 직전 c_loss/kl/explained_variance는 전부 정상이었고
**ppo/entropy만 ~16(σ≈13)**으로 튀어 있었다 — 점진적 과학습이 아니라 값 자체의 발산이었다.
기전: `fixed_sigma=True` + `entropy_coef>0`에서 log-std에 **상한이 없어**, 난이도가 오르며 정책손실의
하방압은 약해지고 엔트로피 보너스는 계속 σ를 밀어올려 σ가 1→13까지 표류 → PPO log-prob/gradient가
오버플로. **수정**: `forward()`에서 `log_std = (mu*0.0 + self.sigma).clamp(-5.0, 0.4)`(σ≤1.49,
entropy≤~7.3)로 σ=13 도달 자체를 봉쇄. **검증**: 98.6% 피크에서 warm-start해 옛 사망지점 epoch 5178을
건강하게 통과(entropy 평탄 ~4.4, capture 0.86~0.91, NaN 없음).

### eval 크래시 = Isaac Gym 진단 메시지(우리 버그 아님) + eval은 128 envs
`*** Can't create empty tensor`는 Isaac Gym 빌드타임 진단이 train에서는 조용 래퍼로 숨겨지고 play에서만
노출되는 것뿐, 무해함. 512-env eval이 실제로 죽는 이유는 **VRAM OOM**(512개 perception 카메라 @ 8GB).
128 envs(=학습값)로 하면 정상 동작. 512는 사용 금지.

### 정직한 baseline (측정 잣대) — `results/baseline_speed_axis_peak986.csv`
98.6% 피크 정책, 25막대·목표 13-16m·드론 2.5m/s·deterministic·2049 episode/셀:

| target speed | capture | crash | timeout |
|---|---|---|---|
| 0.0 | 0.741 | 0.251 | 0.008 |
| 0.5 | 0.762 | 0.238 | 0.000 |
| 0.75 | 0.755 | 0.245 | 0.001 |
| 1.0 | 0.744 | 0.256 | 0.000 |
| 1.25 | 0.724 | 0.275 | 0.001 |
| 1.5 | 0.686 | 0.314 | 0.000 |

**capture가 표적속도에 거의 평탄(0.74 근처, timeout≈0) — 병목은 표적속도가 아니라 막대충돌(~25%)이다.**
"25막대인데 충돌은 용납 못한다"는 문제의식이 데이터로 확증됨. 학습곡선 피크 0.986 vs 이 고정평가
0.74가 정직한 train↔eval 갭.

### 다음: 충돌 저감 튜닝, 파라미터 1개씩 (각각 이 baseline 0.74/0.25 대비 측정)
보상 재조정은 과거 3회 연속 무효였던 레버라 제외(충돌은 기하 문제로 판단). 순위:
1. **고도 z-position 피드백** — 인에이블러, 가장 짧음. look-ahead 확장 시 바닥 sag 재발을 막는 전제조건.
2. LiDAR look-ahead 8→10/12m (①이 검증된 뒤에만).
3. 10m 카메라 depth를 actor에 전달 (전방향only, 바닥 무관).
4. 장애물 토큰 5→8 또는 Transformer dim 64→128 (`MAX_OBSTACLES`, 저위험이나 obs shape 변경 → fresh 재학습).

각 실험당 재학습 ~30-40분 + 128-env eval 필요, 무비용 충돌개선 레버는 없음. 상세 = `CRASH_TUNING_LOG.md`
Session 2 섹션. 커밋: `bb0faa7`(코드), `ccb979b`(문서+baseline CSV).

---

## 2026-07-24 — 후보 ① 고도 PI 제어 검증 완료: 확실한 개선

`lee_velocity_control.compute_acceleration`가 `setpoint_position`을 항상 현재 위치로 넘겨 위치 피드백항이
0인 순수 속도추종 컨트롤러임을 확인 → task 레벨 고도 hold를 P전용(`4*z_err`)에서 **PI**로 변경(Ki=1.0,
anti-windup, 에피소드마다 리셋, `navrl_task.py` 미커밋). 근거: 위빙 중 자세추종 지연이 만드는 정상상태
sag는 P만으로는 안 지워짐.

98.6% 피크에서 warm-start해 ~450 epoch 안정성 확인(`below` 0~5.6% 유지, NaN 없음) 후, baseline과 동일
128-env 고정평가(2049 episode/셀)로 검증:

| target | baseline capture | PI capture | baseline crash | PI crash |
|---|---|---|---|---|
| 0.0 | 0.741 | **0.804** | 0.251 | **0.191** |
| 0.5 | 0.762 | **0.822** | 0.238 | **0.178** |
| 0.75 | 0.755 | **0.813** | 0.245 | **0.187** |
| 1.0 | 0.744 | **0.808** | 0.256 | **0.192** |
| 1.25 | 0.724 | **0.786** | 0.275 | **0.214** |
| 1.5 | 0.686 | **0.778** | 0.314 | **0.222** |
| **평균** | **0.735** | **0.802** | **0.263** | **0.197** |

**전 구간 일관 개선: capture 평균 +6.7pt, crash −6.6pt** (표적 1.5m/s에서 최대 +9.2pt). `below`(바닥충돌)는
전 구간 1.6~2.3%로 사실상 해소. **새 발견**: crash 중 `oob`(아레나 이탈, 주로 N벽)가 34~44%로 부각 —
바닥충돌에 가려있던 문제. 결과 = `results/altitude_pi_speed_axis.csv`, 상세 분석 = `CRASH_TUNING_LOG.md`
"Candidate #1 RESULT" 섹션.

**다음 후보(순서)**: ② LiDAR look-ahead 8→10/12m(이제 잠금 해제) ③ N벽 oob drift 원인 조사(새로 부각된
2차 원인) ④ 카메라 depth→actor ⑤ 장애물 토큰/네트워크 확대.

---

## 2026-07-24 (이어서) — General-spawn G0 파인튜닝 검증 (랜덤 스폰 일반화)

사용자가 후보 ②(LiDAR) 대신 `train_navrl_general_8m_finetune.sh`로 **랜덤 드론/타깃 스폰 일반화**
파인튜닝을 실행(`NAVRL_GENERAL_TRAIN=1`, PI 체크포인트 ppo_260724_0110에서 +~1200 epoch →
`ppo_260724_0209_navrl`, LiDAR 8m 유지, OOB_MARGIN=1.0, K_MIN_FINAL=10). 학습은 건강히 완주
(peak captured 93.7%@ep4256, last 83.8%, NaN 없음).

⚠️ **평가 함정(기록)**: 이 정책을 baseline recipe(고정 스폰)로 평가하면 capture 10%·oob 90%(W벽)로
완전히 망가진 것처럼 나옴 — 정책이 랜덤 스폰을 학습했는데 고정 스폰으로 재면 분포 불일치. **general
정책은 반드시 `NAVRL_GENERAL_TRAIN=1`로 평가**해야 함. (체크포인트 env_state에는 n_bars=25,
k=13-16만 저장되고 GENERAL_TRAIN 플래그는 안 저장되므로 평가 시 수동 지정 필수.)

정상 조건(general-spawn, 128 env, 2049 ep/셀) 6-speed 평가 = `results/general_8m_speed_axis.csv`:

| target | capture | crash | timeout |
|---|---|---|---|
| 0.0 | 0.803 | 0.120 | 0.077 |
| 0.5 | 0.873 | 0.124 | 0.002 |
| 0.75 | 0.837 | 0.160 | 0.003 |
| 1.0 | 0.850 | 0.146 | 0.004 |
| 1.25 | 0.837 | 0.161 | 0.002 |
| 1.5 | 0.819 | 0.180 | 0.001 |
| **평균** | **0.836** | **0.149** | **0.015** |

랜덤 스폰 일반화에도 capture 0.836/crash 0.149로 견고(oob는 margin 1.0 덕에 4~8%로 억제). **단,
스폰 분포가 달라 baseline(0.735)·PI-only(0.802)와 직접 비교 불가** — 이건 general-spawn 체제의 새
measuring stick. `below`가 crash의 10~18%로 다시 보이는데, 이 역시 다른 분포에서의 값이라 PI-only
평가(1.6~2.3%)와 직접 비교하면 안 됨.

---

## 2026-07-24 (이어서) — tilt 추력보상 + `below` 근본원인 규명 → "실패의 보존" 발견

**질문("틸트할지 어떻게 미리 아나?")에 대한 답**: 예측 아님. 추력 계산하는 그 줄에서 현재 기울기(측정)와
원하는 힘벡터(방금 계산)를 둘 다 알고, 침하는 그 둘 내적의 결정론적 오차. `velocity_control.py`에
PX4식 고도우선 추력 `T = f_z / clamp(b3_z, 0.5)` 추가(cfg gate, NavRL 전용, `NAVRL_TILT_COMP` 기본 ON).

**below 계측 추가**: crashdiag의 below에 사망 step + 사망 틸트각. general-spawn에서 below=스폰 1.5초 후·
30° 뱅크 = 랜덤스폰 초기 급선회(적분 아직 0) 시점으로 확정.

**제로샷 A/B**(0209, speed1.0): below 1.96%→1.22%(−38%), capture 무손상, 잔존 사망 틸트 30°→36°(겨냥한
죽음만 제거된 지문). tilt-comp ON으로 파인튜닝(0209→`ppo_260724_1052`, +~1180ep, peak cap 96.9%).

**★ tilt-comp 파인튜닝 최종 평가 (6-speed general, 절대율 평균) = results/general_8m_tiltcomp_speed_axis.csv:**

| 지표 | 파인튜닝 전(0209) | 후(1052) | Δ |
|---|---|---|---|
| capture | 0.837 | **0.837** | **0 (완전 평탄 — 사용자 예측 적중)** |
| below(바닥) | 1.82% | **0.50%** | −1.31pp (설계대로 고쳐짐) |
| oob(벽) | 0.89% | **2.38%** | +1.49pp (실패가 여기로 이동) |
| bar_contact | ~12.2% | ~13.1% | 그대로(지배적) |

**핵심 = 실패의 보존**: tilt-comp가 바닥침하를 제거하자 정책이 더 과감히 뱅크(파인튜닝으로 학습) →
그 과감함+측면 추력누출이 드론을 아레나 밖으로 밀어냄. below와 oob가 ~1:1로 맞바꿔지고 capture는
불변. **below(0.5%)·oob(2.4%) 둘 다 작음. bar_contact(~13%)가 둘 합친 것의 4~5배 = 진짜 병목.**
지금까지 고도작업(PI, tilt-comp)은 전부 **작은 2차 실패**를 다듬은 것 → capture가 안 움직인 이유.

**tilt-comp/PI는 유지**(올바른 제어 + 고밀도 일반화 기반). oob는 candidate ③로 별도 추적. **capture를
0.84에서 올릴 유일한 레버 = bar_contact = candidate ② LiDAR look-ahead 8→12m** (고도 단단해져 이제
바닥충돌 재발 없이 확장 가능 — 이게 고도를 먼저 한 이유). 커밋 `c01e552`(tilt-comp+계측), `b28eac1`(G0 지원).

---

## 2026-07-24 (이어서) — candidate ② LiDAR 12m: capture 최고치이나 **가설은 기각**

12m look-ahead 학습(1052→`ppo_260724_1230`). **ep6145에 peak(capture 95.2%, reward 111) 찍고 ~ep6700에
붕괴** — crashdiag oob가 0.10→0.60으로 전 방향(W/E/S/N 균등) 폭발 = 드론이 방향감각 상실, 최종 capture
0.558. `gen_ppo.pth`는 붕괴 이전 peak라 **정책 자체는 무사**. 붕괴 추정 트리거: LiDAR range를 8→12로 바꾸면
스캔 정규화(`scan/range`)가 바뀌는데 warm-start한 체크포인트의 `running_mean_std`는 8m 통계 → 학습 중
입력 스케일이 밑에서 드리프트 → 정책이 스캔 오독 → PPO 온폴리시 나선. (미증명)

**peak 6-speed 평가**(= `results/general_12m_lookahead_speed_axis.csv`, 평가도 반드시 `LIDAR_RANGE=12`):

| 지표(절대율 평균) | 8m tilt-comp | **12m peak** | Δ |
|---|---|---|---|
| capture | 0.837 | **0.856** | **+1.8pp (역대 최고)** |
| crash | 0.160 | 0.139 | |
| oob | 2.4% | **1.2%** | −1.2pp ← **이득은 전부 여기서** |
| **bar_contact** | 13.1% | **12.6%** | **−0.5pp (노이즈, 겨냥한 목표인데 안 움직임)** |

**★ 가설 기각**: look-ahead는 bar_contact의 레버가 아니다. 8m = 2.5m/s에서 3.2초 경고로 단일 회피엔 이미
충분했고, 더 멀리 본 효과는 "아레나 밖으로 덜 샘"(oob)뿐. **이로써 bar_contact(~13%)에 대한 독립적 음성
결과 3연속**: 고도 PI / tilt-comp / look-ahead 전부 불변. bar_contact(mean_x~12.4m = 막대밭 한복판)는
고도 권한에도 감지 거리에도 면역이며, 남은 capture 갭의 전부다.

**다음: 추측 금지, 직접 진단.** 남은 가설 2개(측정으로 구분 가능):
- **H1 토큰 용량**: `MAX_OBSTACLES=5`(navrl_perception.py) — 밀집 구간엔 반경 내 막대가 5개 초과라
  6번째부터 정책 입력에서 잘림 → 안 보이는 걸 피할 수 없음.
- **H2 인지/추적 품질**: 부딪힌 막대의 KF 추적 위치 오차 → 유령을 피하다 실물에 스침.

**진단 계측**(재학습 불필요): bar_contact 순간에 (a) 반경 내 막대 개수(혼잡도) (b) 실제 부딪힌 막대가
정책에 들어간 토큰 안에 있었는지 (c) 그 막대의 추적-실제 위치 오차. H1이면 혼잡도↑ & 부딪힌 막대가
토큰 밖, H2면 혼잡도↓ & 토큰 안에 있으나 오차 큼. (below/tilt를 정확히 짚었던 것과 같은 measure-first 방식)

---

## 2026-07-24 (이어서) — **코드 불일치 정리 + bar_contact 진단 완료: H1·H2 둘 다 확인**

### 서브에이전트 감사로 잡은 불일치 (전부 수정)
- **평가 스크립트가 스케일 결정 env를 안 박음**(`eval_navrl_density_sweep.sh`, `..._speed_density_grid.sh`):
  `NAVRL_LIDAR_RANGE`(=스캔 정규화 divisor!), `MAX_VELOCITY`, `YAW_RATE_MAX` 미지정 → 8/12m 정책이 4m
  기본값으로 평가돼 **밀도곡선이 조용히 틀림**. + density_sweep은 `NAVRL_PERCEPTION=1`이 없어 아예 CNN
  yaml을 골라 현재 정책엔 사용 불가였음. → 전부 pin + 에코.
- **`max_velocity`가 고도 PI 권한·anti-windup까지 결정하던 결함(내 코드)** → `alt_hold_vmax`
  (`NAVRL_ALT_HOLD_VMAX`, 기본 2.5)로 분리. 안 그러면 속도 스윕이 "느린 추격자가 덜 충돌"과 "느린
  추격자는 고도 유지 불가"를 뒤섞음.
- **체크포인트 설정 드리프트 무경고** → `get/set_env_state`에 `cfg_lidar_max_range/max_velocity/
  yaw_rate_max/max_obstacles` 기록 + 불일치 시 경고(복원은 안 함, 의도적 override 허용). 오늘 두 번 헤맨 그 문제.
- **`navrl_task_config`의 `max_obstacles=5`·`history_steps=5`는 아무도 안 읽는 죽은 필드**(실제 상수는
  `navrl_perception.py`) → 삭제 + 경위 주석. 대신 `NAVRL_MAX_OBSTACLES`로 스윕 가능하게.
- 런처 드리프트 정리 → 두 스크립트가 이제 **`LIDAR_RANGE`만 다름**(검증됨). crashdiag 계측 자체는 감사 결과 **정상**.

### ★ bar_contact 진단 결과 (`NAVRL_BAR_PROBE=1`, 12m peak, n=266 충돌)
```
bars_in_range=15.8  occupied_bins=19.5/36  hit_in_tokens=0.647
token_err=0.57m     token_rank=0.9         (capacity=5)
```
- **H1(토큰 용량) 확인**: 반경 내 막대 **15.8개 vs 용량 5개**. **충돌의 35%가 정책 입력에 아예 없던 막대와의
  충돌.** 보여주지 않은 걸 피할 수는 없음.
- **H2(추적 품질) 확인, 단 원인은 추적기가 아님**: 토큰에 있던 65%도 **위치오차 0.57m**(드론 박스 0.28m,
  막대 반경 0.2~0.4m → 통로 폭과 맞먹음). 원인 = **각도 양자화**(수평 36빔=10°/bin → 5m에서 반빔 오차 ~0.44m).
  토큰 위치가 range/angle 기하로 만들어지므로 측방 정확도의 상한이 빔 개수에 묶여 있음.
- **숨은 천장 발견**: 토큰 하나 뽑을 때마다 ±2 bin(=50°)을 지움 → 360/50≈7.2 → **MAX_OBSTACLES를 8로
  올려도 7개 이상은 구조적으로 안 채워짐.** 용량만 올리면 헛돎.

### 수정은 3개 묶음 (전부 관측 차원 변경 → **fresh 재학습 필수**)
1. `MAX_OBSTACLES` 5→8  2. 억제폭 ±2→±1 bin  3. 수평빔 36→72
**캠페인 통틀어 처음으로 추측이 아닌 측정에 근거한 bar_contact 개입** — 고도 PI·tilt-comp·12m look-ahead가
전부 bar_contact를 ~13%로 남긴 이유는 셋 다 **장애물 표현(representation)** 을 건드리지 않았기 때문.

---

## 2026-07-27 — `general_repr` 반복 조기 종료 원인 규명 및 PPO 안전장치

### 증상과 원인

`train_navrl_general_repr.sh`로 시작한 fresh representation 학습 중 다음 두 run이 조기에 끝났다.

- `ppo_260727_0048_navrl`: epoch 62, `early_stop_nan`
- `ppo_260727_0147_navrl`: epoch 43, `early_stop_nan`

비교 run(`0054`, `0058`, `0106`)은 각각 설정된 60/120/120 epoch까지 정상 실행되었고, 실패 로그에는
OOM·CUDA 오류·외부 process kill이 없었다. 조기 종료 로직이 학습을 임의로 끊은 것이 아니라 이미 망가진
학습 상태를 mean reward의 NaN으로 뒤늦게 감지한 것이었다.

근본원인은 `ppo_navrl_perception_transformer.yaml`의 조합이었다.

```yaml
learning_rate: 1e-4
lr_schedule: adaptive
```

현재 `rl_games`의 legacy adaptive scheduler는 KL이 낮으면 minibatch마다 LR을 1.5배 올리고 상한을
`1e-2`로 둔다. 이 batch 구성에서는 첫 epoch에만 `1e-4 → 1.70859e-3`으로 올라갔고 이후 `1e-2`에
도달했다. 실패 run에서 PPO loss가 epoch 56/36부터 NaN이 된 뒤 mean reward가 epoch 62/43에 NaN이
되어 조기 종료되었다. 실패 체크포인트의 optimizer에도 LR `0.01`이 저장되어 있어 그대로 resume하면
설정의 `1e-4`와 무관하게 위험한 LR을 다시 사용하게 되는 문제도 확인했다.

### 수정

- `ppo_navrl_perception_transformer.yaml`
  - actor LR은 `1e-4` 유지.
  - `lr_schedule: adaptive`를 `lr_schedule: None`으로 변경해 IdentityScheduler로 고정.
- `training_safety.py` 신규
  - PPO metric과 모델 파라미터의 첫 NaN/Inf 위치를 찾는 검사 추가.
  - 체크포인트 restore 후 optimizer 모든 param group의 LR을 현재 설정값으로 덮어쓰는 helper 추가.
- `early_stop_a2c_agent.py`
  - restore 시 과거 체크포인트의 optimizer LR을 버리고 현재 config LR로 초기화.
  - 매 epoch 직후 actor/value/bounds loss, entropy, KL 및 모델 파라미터의 NaN/Inf를 검사.
  - non-finite 상태는 `nonfinite_ppo` 실패로 즉시 예외 종료.
  - NaN reward 또는 non-finite PPO 상태에서는 손상된 periodic/best/last checkpoint를 저장하지 않음.
  - NaN 실패에는 `.aerial_training_finished`를 쓰지 않고 non-zero exit를 유지하여 정상 완료로
    오인하거나 손상 체크포인트를 자동 재사용하지 않게 함.
  - 정상 reward collapse/plateau, max epoch/frame 종료의 기존 체크포인트 동작은 유지.
- `run_header.py`, `train_run_recorder.py`
  - 실행 헤더에 NaN/Inf fail-fast, plateau, collapse 조건을 실제 설정대로 표시.
  - NaN 종료를 정상 조기 종료가 아닌 실패로 기록.
- `tests/test_training_safety.py` 신규
  - 고정 LR 설정, metric/parameter NaN 감지, stale checkpoint LR 교정을 검증.

### 실제 실행 경로 확인

사용 중인 명령은 다음과 같다.

```bash
cd aerial_gym/rl_training/rl_games
./train_navrl_general_repr.sh
```

이 wrapper가 `NAVRL_VISION=1`, `NAVRL_PERCEPTION=1`을 설정한 뒤 `train_navrl.sh`를 호출하므로 실제로
수정한 `ppo_navrl_perception_transformer.yaml`을 선택한다. 관측 차원이 574→898로 바뀐 실험이라
wrapper 설계대로 checkpoint warm-start 없이 fresh run으로 실행된다. 따라서 실행 명령은 변경하지 않는다.

### 검증

- `tests/test_training_safety.py`: 4/4 통과.
- 기존 `tests/test_navrl_perception.py`: 4/4 통과.
- 수정 Python 파일 `py_compile` 통과.
- `git diff --check` 통과.
- 사용자 소유의 untracked `checkpoints_saved/`는 변경하지 않음.

전체 Isaac Gym 학습 smoke run은 현재 셸 환경에 `ninja` 실행 파일이 없어 extension import 단계에서
실행하지 못했다. 실제 다음 `general_repr` fresh run에서 시작 헤더의 actor LR `0.0001`,
schedule `None`과 장시간 NaN 재발 여부를 확인한다.

---

## 2026-07-27 — tilt 제한 + 추력보상 부호 안전화, 그리고 내 NaN 진단의 정정

병렬 세션이 같은 날 NaN의 근본원인을 **adaptive LR 스케줄러**(1e-4 → 1e-2 상승)로 규명해 위에
기록했다. 이 항목은 **그와 별개로 진행된 제어 쪽 작업**과, 그로 인해 **내 초기 결론이 부분적으로
틀렸음**을 남긴다.

### 정정: "tilt 수정으로 NaN이 해결됐다"는 근거가 약했다

`0048`(epoch 62 NaN)을 보고 나는 원인을 내가 c01e552에서 넣은 tilt 보상의 부호 버그로 지목하고,
tilt 제한을 넣은 뒤 fresh 120 epoch에서 NaN이 없자 해결로 판단했다. **각 조건 n=1이라 통계적으로
빈약했고, 실제로 `0147`이 tilt 제한이 있는 상태에서도 epoch 43에 NaN으로 죽었다**(위 항목).
→ **NaN의 주원인은 LR 스케줄러이고, 내 tilt 가설은 기각에 가깝다.** 다만 아래 부호 버그 자체는
실재하는 결함이라 수정은 유지한다(NaN의 원인이 아니었을 뿐, 뒤집힌 상태에서 지면으로 가속하는 것은
그 자체로 잘못).

### 유지하는 수정 (커밋 `6d391e0`)

1. **부호 안전화**: `T = f_z / b3_z.clamp(min=0.5)`는 크기만 막고 부호를 안 막아, 뒤집힌 상태
   (`b3_z<0`)에서 `+0.5`를 반환해 **아래를 향한 몸통축에 양의 추력**을 걸었다. `b3_z ≤ 0.5`면 표준
   Lee 투영으로 폴백하도록 변경.
2. **목표 기울기 제한 45°** (`NAVRL_MAX_TILT_DEG`, PX4 `MPC_TILTMAX_AIR` 방식). 기존 Lee는 목표
   body-z를 `힘/|힘|`로 그대로 뽑아 수평 명령이 크면 90°를 넘기는 자세를 요구했다. 고도 우선 형태
   (수직 유지·수평 축소)라 양력이 아니라 횡가속을 희생 — 45°에서도 9.8 m/s², 2.5 m/s에서 선회반경
   0.64 m.

**학습된 정책 무손상 확인**(2048 ep, general spawn, target 1.0): 제한 없음 capture **0.868** vs
45° **0.864**. 즉 잘 나는 정책은 이 제한에 닿지 않는다.

### 기각된 가설 2건 (다시 시도하지 말 것)

- **"tilt 보상이 바닥추락(below)의 원인"** → 기각. A/B 결과 보상 **OFF일 때 below가 85~87%로 더
  높았고** ON은 73~76%였다. 보상은 오히려 바닥추락을 줄이고 있었다.
- **"각속도 감쇠 부족이 자세 오버슈트의 원인"** → 기각. `K_angvel` 0.2 / 0.45 / 0.8 스윕에서
  IID 랜덤 행동 시 최대 기울기가 **127~134°로 변하지 않았다.** 측정 이득이 없어 변경을 되돌렸고
  커밋하지 않았다.

### 부수 측정 (fresh 정책의 고도)

직접 프로브(32~64 env, 고정 명령): 정지·최대전진 모두 z=1.00 m 유지, 기울기 3~7°로 **고도 제어
자체는 정상**. 반면 **매 스텝 IID 랜덤 행동**(=갓 시작한 PPO)에서는 최저 고도 0.10 m(사망선),
최대 기울기 127°. 0.5초 유지 시엔 0.70 m/67°로 안전. fresh 초반 `below` 80%대는 제어 결함이 아니라
**행동이 매 스텝 뒤집히는 데서 오는 것**으로 보인다. 과거 성공한 fresh run도 capture 0.008에서
시작했으므로(이번 `0048`도 epoch 5에 0.007) 초반 고 crash 자체는 이상 신호가 아닐 수 있다 —
학습되며 내려가는지가 관건이며 아직 미확인.

### 작업 규칙 변경

`.claude/skills/navrl/SKILL.md`와 `CLAUDE.md`에 **WORKLOG 규칙**을 명문화했다: 학습·평가·코드변경·
진단(기각된 가설 포함) 각각에 대해 이 파일 갱신을 필수로 하고, diff 검토 요청 **전에** 작성하며,
세션 예산이 부족해도 마지막까지 자르지 않는다.

---

## 2026-07-27 — (A) representation run 평가 완료 + (B) 토큰 선택 FOV 설계

### (A) `ppo_260727_0225` 평가 — **첫 fresh 완주**, bar_contact를 처음으로 움직임

fresh 8000/8000 epoch 완주(NaN 없음). 이전 fresh 시도는 epoch 43~62에서 NaN 사망 — 병렬 세션이
고친 `lr_schedule: None`이 효과를 본 것으로 보인다. peak reward 107.8 @ ep7247, peak capture 96.0%.

6-speed held-out 평가(128 env, 2049 ep/셀, general spawn, 학습과 동일한 72빔/8토큰 조건).
결과 = `results/general_repr_speed_axis.csv`. 절대율(전체 에피소드 대비) 평균, 직전 최고(12m peak,
5토큰/36빔) 대비:

| 지표 | 12m peak | **0225 (8토큰/72빔)** | Δ |
|---|---|---|---|
| capture | 0.851 | **0.861** | +1.0pp |
| **bar_contact** | 12.7% | **9.0%** | **−3.7pp (−29%)** |
| below | 0.2% | 3.5% | +3.3pp |

**캠페인 통틀어 bar_contact를 움직인 첫 개입.** 고도 PI·tilt-comp·12m look-ahead는 셋 다 ~13%로
남겼었다.

### ★ 그러나 예측한 메커니즘 2개 모두 기각 (다시 시도하지 말 것)

- **H1 "토큰 용량이 병목"** → **기각.** 용량을 5→8로 늘렸는데 `hit_in_tokens`가 0.647 → **0.40~0.53
  으로 오히려 하락.** `token_rank≈2.9`라 늘린 슬롯을 쓰고는 있다. 반경 내 막대가 ~16개라 8개로도
  절반이 안 잡힌다.
- **H2 "token 위치오차 = 각도 양자화"** → **기각.** 빔을 36→72로 늘려 bin을 10°→5°로 절반으로
  줄였는데 `token_err`가 0.57m → **0.72~1.13m로 증가.**
- ⇒ **bar_contact 개선은 우리가 겨냥한 토큰 메커니즘이 아니라 다른 경로**(가장 유력: 더 촘촘해진
  static scan 자체)에서 왔다. 개선은 진짜지만 이유는 예측과 달랐다.

### (B) 설계 — 토큰 선택 FOV (`NAVRL_OBSTACLE_FOV_DEG`, 기본 360 = 기존 동작)

기하 계산이 측정과 일치: 360°에서 8토큰 = 반경 내 16개 중 **50% 커버** → 측정된 `hit_in_tokens`
0.40~0.53과 정확히 맞는다. **뒤쪽 막대에 슬롯을 쓰느라 앞쪽이 굶는 구조.**

| 선택 FOV | 반경 내 막대 | 8토큰 커버율 |
|---|---|---|
| 360°(현재) | 16.0 | 50% |
| 240° | 10.7 | 75% |
| 180° | 8.0 | 100% |

구현: `navrl_perception._fuse_static_and_extract_obstacles`에서 **선택용 복사본에만** 섹터 밖
bearing을 blank 처리. `static_state`는 전 360°를 그대로 유지하므로 전방위 인지는 잃지 않는다.
**관측 차원 898 불변 → 0225에서 warm-start 가능**(fresh 재학습 불필요).

**제로샷 사전확인**(재학습 없이 FOV만 바꿔 0225 평가, target 1.0): 360° cap 0.869/hit 0.418,
240° cap 0.857/hit 0.479, 180° cap 0.862/hit 0.402. `hit_in_tokens`가 기대(0.75)에 훨씬 못 미친다 —
정책이 360° 토큰 배치에 맞춰 학습됐기 때문일 수 있어 **재학습해야 판정 가능**. 이것이 (B)를 하는 이유.

**판정 기준**: `hit_in_tokens` 0.42 → 0.65+ **그리고** `bar_contact` < 9%면 성공. 실패 시 240→180
재시도, 그래도 안 되면 `NAVRL_OBSTACLE_FOV_DEG` 제거로 롤백하고 (C) 밀도로 진행.

### 다음 순서 (사용자 확정): A → B → C
- (A) 완료
- (B) 위 warm-start 명령으로 ~2250 epoch(≈1.5h) 추가 학습 — 사용자가 직접 실행
- (C) 밀도 커리큘럼 = 논문 본선(밀도×속도 지도). bar_contact를 먼저 낮춰야 고밀도에서 폭발하지 않음
  (과거 150막대 절벽 −27pt 기록).

---

## 2026-07-27 — (B) FOV-240 결과 감사 + bar probe v2 + (C) 진입 결정

### (B) `ppo_260727_0930` 평가 결과

`ppo_260727_0225`의 360° token-selection 정책에서 warm-start해 epoch 7248→9500을 추가 학습했다.
best `gen_ppo.pth`는 epoch 8571, optimizer LR은 고정된 `1e-4`. 6-speed general-spawn 평가 결과는
`results/general_repr_fov240_speed_axis.csv`에 보존했다.

| target | capture (360 baseline→240) | bar_contact 절대율 | below 절대율 |
|---:|---:|---:|---:|
| 0.0 | 0.874→0.894 | 7.4%→6.2% | 3.0%→2.9% |
| 0.5 | 0.879→0.906 | 7.9%→6.2% | 3.8%→2.7% |
| 0.75 | 0.870→0.894 | 8.6%→6.8% | 3.3%→2.9% |
| 1.0 | 0.869→0.893 | 8.9%→7.8% | 3.3%→2.4% |
| 1.25 | 0.859→0.889 | 10.1%→7.9% | 3.0%→2.6% |
| 1.5 | 0.824→0.859 | 11.9%→10.0% | 4.0%→3.2% |
| **평균** | **0.862→0.889 (+2.7pp)** | **9.1%→7.5%** | **3.4%→2.8%** |

캠페인 궤적은 capture `0.851→0.861→0.889`, bar_contact `12.7%→9.1%→7.5%`.
240° 정책을 360° token FOV로 평가하면 target 1.0 capture가 `0.893→0.878`로 내려가므로 반드시
학습과 동일한 240°로 평가해야 한다.

### 관측성 결함 수정

`0930`은 FOV 기록 패치 전에 시작되어 log/checkpoint에 FOV가 없었다. 240°/360° paired eval로 조건을
역추론해야 했으므로 다음을 추가했다.

- 시작 로그: `tokens`, `token_fov`, `suppress`, `scan V×H`, `lidar_range`를 한 줄로 출력.
- checkpoint `env_state`: `cfg_token_fov_deg`, `cfg_obstacle_suppress_deg`,
  `cfg_lidar_hbeams/vbeams`까지 저장.
- resume/eval 시 현재 값과 다르면 policy input mismatch 경고.
- FOV `(0, 360]`, 양수 beam/token 수, 음수가 아닌 suppression을 시작 시 검증.
- density/speed 평가 스크립트의 기본 contract를 최신 정책의
  `8 tokens / 240° / ±10° / 4×72 / 12m / general spawn / 128 env`로 교정.
- density eval의 `... | grep ... || true`가 `play_navrl.sh` 실패까지 성공으로 숨기던 문제를 제거해,
  `set -o pipefail`이 평가 오류를 정상적으로 중단시키게 함.

### ★ 기존 `hit_in_tokens`/`token_err` 해석은 무효 — bar probe v2

기존 probe에는 세 결함이 있었다.

1. 240° 밖 후방 충돌도 `hit_in_tokens`의 전체 분모에 포함하고 이를 240° 내부의 기하학적 75%
   커버 예상치와 비교했다.
2. 같은 방위의 토큰을 거리와 무관하게 충돌 막대에 매칭해, 밀집 구간에서 앞/뒤의 다른 막대를
   잘못 붙였다.
3. token 위치는 LiDAR **표면점**인데 GT **중심점**과의 거리를 position error라고 불렀다. 막대
   반경 자체와 측정 오차가 섞인 값이었다.

v2는 token ray가 GT bar bounding circle을 실제로 통과하고 표면 range도 가능한 경우에만
range+bearing으로 연관한다. 또한 다음을 별도로 출력한다.

- `hit_fov`: 충돌 막대가 token-selection FOV 안에 있었던 비율.
- `hit_token_given_fov`: 선택 가능했던 충돌만 분모로 한 실제 token representation 비율.
- `unique/duplicate`: 토큰 슬롯이 서로 다른 GT 막대를 담는지, 같은 막대를 중복하는지.
- `center_offset`, `cross_track`, `radial_gap`: 표면-중심 거리, 횡방향 ray 오차, 중심까지 남은
  방사 거리. 더 이상 이것을 하나의 `token_err`로 부르지 않는다.

순수 geometry helper `bar_probe.py`와 동일 방위·다른 거리 오매칭 회귀 테스트를 추가했다.

### 실제 GPU 검증 (FOV 240°, target 1.0, 32 env, 2048 episodes)

```
capture=0.883  crash=0.116  timeout=0.001
bar_contact=0.688 of crashes  below=0.257 of crashes
barprobe v2:
  n=163  bars_range=15.7  bars_fov=10.9
  hit_fov=0.773  hit_token=0.442  hit_token_given_fov=0.556
  valid tokens=8.0  associated=2.3  unique=0.8  duplicate=1.4
  center_offset=0.36m  cross_track=0.20m  radial_gap=0.25m
```

`token_err 1.2~1.3m`는 v2에서 재현되지 않았다. 주된 원인은 실제 센서 오차가 아니라
bearing-only 오매칭과 표면/중심 의미 혼동이었다. global `hit_token=0.442`가 낮은 이유 중 일부도
후방 FOV 밖 충돌을 포함한 분모였고, 조건부 값은 `0.556`이다.

### 결정: suppression ±5° 재학습은 보류하고 (C)로 진행

±10°인 현재 상태에서도 contact 순간 평균 `duplicate=1.4` 슬롯이 이미 같은 GT 막대에 중복 사용된다.
suppression을 ±5°로 줄이면 같은 넓은 막대가 더 많은 슬롯을 소비할 가능성이 커서, “240° 중 더 넓게
덮는다”는 단순 계산과 반대 효과가 날 수 있다. 그 가설의 근거였던 기존 `hit_in_tokens/token_err`도
측정 결함이 확인됐다.

반면 실제 목표 지표는 capture `0.889`, bar_contact `7.5%`로 명확히 개선됐다. 따라서 한 번 더
25-bar 조건을 미세 조정하지 않고 (C) density curriculum으로 진입한다.

전용 `train_navrl_general_repr_density.sh`를 추가했다. 기존
`train_navrl_vision_seq_density.sh`는 898-D representation/FOV를 고정하지 않아 이 checkpoint에
사용하면 안 된다. 새 launcher는 240°/72빔/8토큰/12m를 고정하고 `NAVRL_NUM_BARS`를 unset하여
25→110, +5 bars, capture threshold 0.55 curriculum이 실제로 승급되게 한다.

### 검증

- `tests/test_navrl_bar_probe.py`: 4/4 통과.
- `tests/test_navrl_perception.py`: 4/4 통과.
- `tests/test_training_safety.py`: 4/4 통과.
- 수정 Python `py_compile`, launcher `bash -n`, `git diff --check` 통과.
- 실제 Isaac Gym 2048-episode CUDA 평가 완료, probe v2 출력 확인.

---

## 2026-07-27 — (B) FOV 240° 결과 = 역대 최고 + 대시보드 갱신·3D 튐 수정

### (B) 토큰 선택 FOV 240° — 성공, capture 0.889

`ppo_260727_0930`(0225에서 warm-start, +2253 epoch, peak reward 112.5 @ ep8571). 6-speed held-out
(128 env, 2049 ep/셀, general spawn) = `results/general_repr_speed_axis.csv` 대비:

| target | capture | bar_contact(절대) | below(절대) |
|---|---|---|---|
| 0.0 | 0.874 → **0.894** | 7.4% → 6.2% | 3.0% → 2.9% |
| 0.5 | 0.879 → **0.906** | 7.9% → 6.2% | 3.8% → 2.7% |
| 0.75 | 0.870 → **0.894** | 8.6% → 6.8% | 3.3% → 2.9% |
| 1.0 | 0.869 → **0.893** | 8.9% → 7.8% | 3.3% → 2.4% |
| 1.25 | 0.859 → **0.889** | 10.1% → 7.9% | 3.0% → 2.6% |
| 1.5 | 0.824 → **0.859** | 11.9% → 10.0% | 4.0% → 3.2% |
| **평균** | 0.862 → **0.889 (+2.7pp)** | 9.1% → **7.5%** | 3.4% → 2.8% |

**캠페인 궤적: 0.735 → 0.802 → 0.837 → 0.851 → 0.861 → 0.889**, bar_contact 12.7 → 9.1 → 7.5%.

### ★ 내 구현 결함: FOV가 아무 흔적을 안 남겨 사후 판별 불가였음 (수정함)

학습이 끝난 뒤 **FOV가 실제로 적용됐는지 로그·체크포인트 어디서도 확인할 수 없었다.** 240°/360° 두
조건으로 평가해 **선호가 뒤집히는 것**(0225는 360°가 우세, 0930은 240°가 우세)으로 역추론해야 했다.
수정: ① 시작 시 `NavRL obstacle representation | tokens=.. token_fov=.. suppress=.. scan=..` 출력
② `env_state`에 `cfg_token_fov_deg` 기록 + 불일치 경고. **이 정책은 반드시 240°로 평가해야 한다**
(360°로 평가하면 0.893→0.878로 오독).

### 남은 갭 (다음에 볼 것)

- `hit_in_tokens` 0.42 → 0.50 수준으로만 상승(예측 0.75). 원인 추정: **억제폭 ±10°** 때문에 8토큰이
  최소 20° 간격을 요구해 240° 중 실제로는 ~160°만 덮는다 → 후보 B′ = 억제폭 ±5°(warm-start 가능, 30분).
- `token_err`는 계속 악화 방향(0.75~1.30m). bar_contact는 줄어드는데 이 지표만 나빠지는 패턴이 반복 →
  **지표 자체의 측정 방식을 의심**(밀집 구간에서 ±15° 매칭이 엉뚱한 막대를 잡을 수 있음). 병렬 세션이
  probe v2(range+bearing GT 연관)로 재작성 중.

### 대시보드 갱신 (`docs/status/`, 커밋 `8fa5752`)

내용: 헤드라인 KPI(0.889 / 7.5% / 2.8% / 0%), 캠페인 궤적 표, **기각된 가설 섹션 신설**(look-ahead·
토큰용량·각도양자화 3건), 학습 안정성 3종(σ 폭주·adaptive LR 폭주·기울기 제한 부재), 로드맵을
P6 밀도 커리큘럼 진행 중으로 갱신. 3D 씬 스펙도 현재 정책(LiDAR 72×4 @12m, 기본 25막대)으로 맞춤.

**3D가 툭툭 튀던 원인 4가지 (사용자 지적)**:
1. **`clearBars()`가 하드 투영** — 매 프레임 독립적으로 0.25씩 최대 6회 밀어내(최대 1.5 유닛) 위치가
   `tParam`의 연속함수가 아님. 막대를 스칠 때 보정이 갑자기 켜지거나 방향이 뒤집힘. **주원인.**
2. **헤딩을 그 튀는 위치의 프레임 차분으로 계산** → 위치가 튀면 기체 전체가 홱 돌아 시각적으로 증폭.
3. **프레임 기반 적분**(`tParam += 상수/frame`) → 주사율 의존 + 드롭 프레임마다 건너뜀.
4. **궤적을 5프레임마다만 샘플링** → 꼬리가 각지고 한 마디씩 늘어남.

수정: 투영을 지수 스무딩으로 추종(dt 기반), 헤딩은 필터링된 속도벡터에서 최단각 회전, `performance.now()`
델타 적분(탭 복귀 클램프), 궤적은 이동거리(0.12m) 기준 샘플링.

---

## 2026-07-27 — Stage C 밀도 커리큘럼 중단 진단: collapse guard 오발

### 결론

`ppo_260727_1204_navrl`은 NaN/OOM/프로세스 강제 종료나 실제 정책 붕괴로 끊긴 것이 아니다.
epoch 8906에서 `early_stop_collapse`가 정상적인 밀도 증가에 따른 reward 하락을 붕괴로 오인해
정상 종료 마커를 남기고 학습을 끝냈다.

```
Early stop: reward collapse from peak — peak 108.3 → now 38.7
for 100 epochs (epoch 8906)
```

### 근거

- 커리큘럼은 25→30→35→40→45→50→55→60 bars까지 7회 연속 정상 승급했다.
- 각 승급 capture는 `0.856, 0.815, 0.766, 0.729, 0.682, 0.635, 0.587`로 모두
  승급 문턱 0.55를 통과했다.
- reward 평균은 난도가 오르면서 약 `101.0(25 bars) → 45.4(55 bars)`로 연속 하락했다.
  갑작스러운 수치 발산이 아니라 커리큘럼의 의도된 난도 상승과 일치한다.
- 가드는 전체 run의 저밀도 peak 108.3을 계속 기준으로 삼았다. 설정
  `drop_from_peak=0.35`는 reward가 `70.4` 미만인 epoch를 누적하고, density 변경 때 peak나
  counter를 재설정하지 않아 100 epoch 뒤 종료했다.
- 종료 직전 55 bars capture는 약 0.587, 60 bars의 짧은 4 epoch 구간도 약 0.590이었다.
  커리큘럼이 정체되거나 성능이 0으로 무너진 상태가 아니었다.
- 로그에 NaN, OOM, CUDA 오류, traceback, SIGKILL 흔적이 없고
  `.aerial_training_finished`에 `epoch=8906`이 기록됐다.

### 재개 가능성 및 조치

`nn/last_gen_ppo_ep_8906_rew__38.70596_.pth`의 actor, asymmetric critic, 두 optimizer state를
직접 검사했다. 모든 tensor가 finite이고 actor/critic LR도 모두 `1e-4`여서 이 체크포인트는
60 bars부터 재개 가능한 정상 상태다. 저밀도 best-reward인 `gen_ppo.pth`로 재개하면 안 된다.

다만 같은 설정으로 바로 재실행하지 않는다. Stage C launcher에서는 NaN/Inf fail-fast는 유지하되
고정 밀도용 global `early_stop_collapse`를 비활성화하는 것이 권장 조치다.
`drop_from_peak=0.80` 완화는 이번 오발은 피할 수 있지만 밀도가 더 오르면 다시 오발할 수 있어
근본 해결이 아니다. 더 정교한 대안은 density 승급마다 peak/counter를 재설정하고 동일 밀도
구간 안에서만 붕괴를 판정하는 것이다.

### 조치 완료

- `runner.py`에 run 단위 옵션 `--disable_collapse_early_stop`을 추가했다. 이 옵션은
  `early_stop_collapse.enable=False`만 적용하므로 일반/고정밀도 학습의 YAML 기본 가드는 유지된다.
- Stage C 전용 `train_navrl_general_repr_density.sh`가 위 옵션을 항상 전달하도록 고쳤다.
  시작 헤더에는 `reward-collapse guard=off; NaN/Inf fail-fast=on`이 명시된다.
- Stage C 런처의 기본 체크포인트를 25-bars best인 `ppo_260727_0930/gen_ppo.pth`에서
  `ppo_260727_1204/last_gen_ppo_ep_8906_rew__38.70596_.pth`로 변경했다. 체크포인트
  `env_state.n_bars_active=60`이 복원되므로 별도 인자 없이 60 bars부터 새 run으로 분기 재개한다.
- mean reward의 NaN/Inf 검사를 reward-collapse 함수 밖으로 분리했다. 따라서 collapse 가드를 꺼도
  NaN/Inf reward와 PPO loss/parameter non-finite는 즉시 실패하며 손상 체크포인트를 저장하지 않는다.

검증: 전체 단위 테스트 13/13, 수정 Python `py_compile`, 두 launcher `bash -n`,
`git diff --check`, 실제 runner argparse의 새 옵션과 config override/header 출력 확인 완료.
학습은 자동으로 시작하지 않았다.

재개 명령:

```bash
cd /home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/aerial_gym/rl_training/rl_games
./train_navrl_general_repr_density.sh
```

---

## 2026-07-27 — Stage C 1차 완주(65막대) + 밀도 곡선 실측 + 대시보드 갱신

### Stage C run `ppo_260727_1309` — max_epochs 도달 (8907 → 15000)

느린 승급 설정(`CHECK_EPS=16384`, `THRESHOLD=0.70`)으로 재개. **승급 1회: 60 → 65막대**
(epoch 13208, capture 0.705). 최종 65막대 capture 68.5%, crash 31.5%, reward 48.6(peak 64.9).

**설정 변경이 결정적이었다** — 직전 `1204` run(문턱 0.55 / 창 4096)은 **45 epoch마다** 승급해 7단계를
몰아쳤고 capture가 0.856 → 0.587로 **회복 없이** 내려앉아 reward-collapse 가드에 걸려 종료됐다.
문턱 0.70 / 창 16384로 단계당 정착 시간을 ~4배 늘리자 **승급 후 회복 패턴이 돌아왔다**:

| 65막대 구간 | capture | crash |
|---|---|---|
| 초반 | 0.668 | 0.328 |
| 중반 | 0.680 | 0.315 |
| 후반 | 0.685 | 0.312 |

**"실력이 아니라 시간에 따라 난이도를 올리면 무너진다"는 이 프로젝트에서 두 번째 확인**(첫 번째는
거리 커리큘럼의 epoch 비례 램프 폭주). 커리큘럼 게이트는 문턱과 **판정 창 길이**를 함께 봐야 한다.

추세(회귀): 60막대 구간 crash −0.044/1000ep → 65막대 −0.013/1000ep. 개선은 계속되나 **밀도가 오를수록
단계당 소요가 늘어난다.** 최근 400ep 평균 capture 0.688 → 다음 승급까지 약 800 epoch 예상.

### ★ 밀도 곡선 실측 = `results/general_repr_density_curve.csv`

`last_gen_ppo_ep_15000`(65막대까지 학습), target 1.0 m/s, 2049 ep/셀, deterministic:

| 막대 | 밀도/100㎡ | capture | crash | bar_contact 비중 | 비고 |
|---|---|---|---|---|---|
| 25 | 5.2 | 0.896 | 0.102 | 65% | 학습됨 |
| 50 | 10.5 | 0.796 | 0.202 | 85% | 학습됨 |
| 65 | 13.6 | 0.679 | 0.316 | 88% | **학습 상한** |
| 75 | 15.7 | 0.598 | 0.399 | 92% | 범위 밖 |
| 110 | 23.0 | 0.280 | 0.711 | 94% | 범위 밖 (NavRL 앵커) |
| 130 | 27.2 | 0.150 | 0.847 | 95% | 범위 밖 |
| 150 | 31.4 | 0.079 | 0.920 | 95% | 범위 밖 |

**해석 주의**: 75막대 이상은 **학습하지 않은 밀도**다. 이 하락을 "방법의 한계"로 읽으면 안 되고
**일반화 측정**으로 읽어야 한다(과거 GT-LiDAR 시절 밀도 커리큘럼으로 110막대를 학습했을 때는 0.93였다).
또한 밀도가 오를수록 **실패가 사실상 전부 막대충돌**이 된다(65% → 95%). 고밀도 상한은 곧 충돌 문제다.

### 대시보드 갱신 (`docs/status/`, 커밋 `9495309`, `4c4184c`)

- 히어로 KPI를 밀도 축 중심으로 교체(25/50/65막대 포획 + 학습 도달 밀도)
- **밀도 곡선을 실측값으로 교체**하고, 학습 범위(≤65막대)를 음영+점선으로 표시해 그 오른쪽이
  일반화 구간임을 그래프에서 바로 보이게 함. 표에도 `학습범위` 열 추가.
- Stage C 섹션 신설(승급 속도가 결정적이었다는 내용), 다음 계획을 "커리큘럼 연장 65→110"으로 갱신
- 로드맵 P6를 "밀도 커리큘럼 25→65막대 (진행 중, 목표 110)"로

### 다음 판단 (사용자 확인 대기)

**연장 권장** — 65막대 구간이 아직 포화하지 않았고(crash 기울기 음수 유지), 승급 문턱까지 ~800 epoch.
다만 110막대까지는 단계당 소요 증가를 감안하면 **수 시간~10시간** 규모다. 명령:

```bash
CKPT=runs/ppo_260727_1309_navrl/nn/last_gen_ppo_ep_15000_rew_48.580364.pth \
MAX_EPOCHS=30000 NAVRL_DENSITY_CHECK_EPS=16384 NAVRL_DENSITY_THRESHOLD=0.70 \
  ./train_navrl_general_repr_density.sh
```

---

## 2026-07-27 — 밤샘 Stage C 명령 감사 + 재개 런처 안전성 보강

### 실행 상태와 Claude 안내 판정

사용자가 입력한 명령은 실패하지 않았다. 실제 실행은 `ppo_260727_2324_navrl`로 분기됐고,
감사 중 epoch 15200 이상, 65 bars, GPU 약 5.85 GiB로 정상 진행했다. 실행 환경에서도
`DENSITY_THRESHOLD=0.70`, `DENSITY_CHECK_EPS=16384`, 240°/8-token/4×72/12m contract가
모두 확인됐으며 collapse guard off + NaN/Inf fail-fast on도 정확히 적용됐다.

Claude가 맞게 확인한 항목:

- `ppo_260727_1309/...ep_15000_rew_48.580364.pth`는 실제 존재하고 epoch 15000,
  `env_state.n_bars_active=65`, FOV 240°, tokens 8인 finite 체크포인트다.
- `MAX_EPOCHS=45000`은 30000 epoch 추가이고 현재 약 2.6초/epoch 기준 약 21.7시간이다.
- YAML `save_frequency=50`이므로 periodic checkpoint는 50 epoch마다 저장된다.
- 0.70/16384는 직전 run에서 60→65 승급을 capture 0.705로 통과한 보수적인 설정이다.

수정이 필요했던 항목:

- `nohup ... > /dev/null 2>&1`은 launcher의 “확인할 3줄”까지 버리므로, 직후 터미널에서
  그 줄을 확인하라는 안내와 모순된다. 출력 파일을 남기고 `tail -f`해야 한다.
- task 생성 시 출력되는 25 bars는 **체크포인트 복원 전 초기값**이다. 실제 복원 후 dashboard는
  65 bars였지만 복원 완료 로그가 없어 잘못 시작한 것처럼 보였다.
- “밤새 중간에 끝나지 않는다/110까지 간다”는 보장은 아니다. 45000까지의 계산 예산만 보장하며
  threshold를 넘지 못하면 65 bars에서 held가 반복될 수 있고 OOM/NaN/외부 종료도 가능하다.
- density 판정창의 `_density_succ_agg/_density_fin_agg`가 checkpoint에 없었다.
  16384-episode 창 도중 재개하면 누적 증거가 0으로 리셋돼 승급이 불필요하게 늦어질 수 있었다.
- 분 단위 `train_YYMMDD_HHMM.log` 이름은 같은 분의 빠른 재시도가 앞 로그를 덮어썼다.
  실제로 2323 zero-epoch 폴더와 2324 정상 run이 생겼지만 로그는 하나만 남아 원인 추적이 어려웠다.

### 구현한 수정

- Stage C 기본값을 검증된 `threshold=0.70`, `check_eps=16384`, `max_epochs=45000`으로 갱신.
- `CKPT` 미지정 시 가장 최근의 non-`_rlnorm` `last_gen_ppo_ep_*.pth`를 자동 선택.
- 실행 전 checkpoint를 CPU에서 읽어 epoch < max_epochs, 0≤bars≤final, actor/critic/optimizer
  finite 여부와 FOV/token/LiDAR contract를 검증하는 `navrl_checkpoint_preflight.py` 추가.
- 이미 NavRL train process가 있으면 중복 실행을 거부하고, 동시에 시작되는 race는 `flock`으로 차단.
  `PREFLIGHT_ONLY=1`로 학습 없이 전체 사전검증 가능.
- checkpoint에 density window 누적값과 final/step/threshold/check_eps provenance를 저장·복원.
  복원 시 `bars=25->65`, distance window와 density-window 진행량을 명시적으로 출력.
- 세션 로그 이름을 초+launcher PID까지 포함하도록 변경해 빠른 재시도 시 덮어쓰지 않음.
- `density curriculum held`를 억제되는 INFO에서 WARNING으로 올려, gate가 아직 미평가인지
  평가 후 문턱 미달인지 로그에서 구분 가능하게 함.

검증: 실제 epoch-15000 및 최신 epoch-15200 checkpoint preflight 통과, duplicate-process guard
실제 PID 거부 확인, 전체 단위 테스트 17/17, Python `py_compile`, launcher `bash -n`,
`git diff --check` 통과.

주의: 이미 실행 중인 2324 프로세스는 수정 전에 모듈을 로드했으므로 그대로 계속 학습한다.
지금 재시작하면 거의 찬 16384-episode 창을 버리므로 중단하지 않았다. 새 checkpoint의 density-window
영속화는 다음 재개로 새 코드를 로드한 뒤부터 적용된다.

현재 run의 첫 판정창은 epoch 15208에 16457 episodes, capture 0.6925로 0.70에 조금 못 미쳐
**65 bars held**였다. 기존 코드가 held를 INFO로 숨겨 promotion 로그가 없었던 것이다.
이후 새 창의 초기 2147 episodes는 capture 0.6968로 문턱 근처이며 학습은 계속 정상 진행 중이다.

현재 run 모니터링:

```bash
tail -f train_session_logs/train_260727_2323.log
```

현재 run 종료 후 자동 최신 checkpoint로 다시 밤샘 실행:

```bash
nohup ./train_navrl_general_repr_density.sh \
  > train_session_logs/night_density.out 2>&1 &
tail -f train_session_logs/night_density.out
```

### 2026-07-28 01:47 — 중복 실행 문의 확인

사용자가 동일 launcher를 한 번 더 실행했으나 duplicate guard가 기존 train PID `1652517`을 찾아
의도대로 exit 3으로 거부했다. 실패가 아니라 GPU에 두 학습이 겹치는 것을 막은 정상 동작이다.
기존 `ppo_260727_2324_navrl`은 epoch 18096/45000, 70 bars, VRAM 약 5.85 GiB로 정상 진행 중이다.
65→70 승급은 16388 episodes에서 capture 0.706으로 통과했다. 현재는 새 실행 명령이 아니라
`tail -f train_session_logs/train_260727_2323.log`로 기존 run을 모니터링한다.

### 2026-07-28 02:03 — 학습 건강도·진척도 정량 판정

`ppo_260727_2324_navrl`은 epoch 18432까지 프로세스/GPU/수치 면에서는 정상이다. step time은
최근 100 epoch 평균 2.70초, VRAM 약 5.85 GiB이고 NaN/OOM/traceback이 없다. 최신 epoch-18400
checkpoint의 actor/critic/두 optimizer는 모두 finite이며 LR은 양쪽 모두 `1e-4`다.

그러나 **목표 진척은 70 bars에서 정체**다. 70 bars 진입(epoch 15618) 뒤 218,475 episodes를
학습했고 16,384-episode 승급 판정창을 13번 완료했지만 capture 범위는 `0.663~0.699`로
threshold 0.70을 한 번도 통과하지 못했다. 전체 70-bars capture는 0.6818이고 최근
16,394 episodes는 0.6762, 최근 2,054 episodes는 0.6957이다. 간헐적 반등은 있으나 지속적인
상승 추세는 아니다. reward는 초반 창의 약 39~41에서 최근 약 42~43으로 소폭 개선됐다.

실패 원인은 고도보다 막대충돌로 수렴한다. 최근 crash 중 bar_contact 비중은 평균 약 91%로
상승했고 below 비중은 약 6.8%로 낮아졌다. `hit_token_given_fov`는 최근 32개 probe 평균 약
0.56, 최신 0.587로 붕괴하지 않아 센서/토큰 표현의 갑작스러운 고장 징후는 없다.

판정: **학습 엔진은 건강하지만 커리큘럼은 잘 진행되지 않는다.** 목표가 안정적인 70-bars 정책이면
계속 학습 가능하지만, 110 bars 도달 목적이라면 현재 threshold 0.70을 유지한 채 epoch 45000까지
방치하는 것은 계산 낭비 가능성이 높다. 실행은 사용자 지시 없이 중단하거나 설정 변경하지 않았다.

### 2026-07-28 02:22 — 70-bars 0.70 gate 정체 원인 감사

사용자 요청에 따라 threshold를 낮추거나 코드를 바꾸기 전에 원인을 분리했다. 실행 중인
`ppo_260727_2324_navrl`은 감사 시점 epoch 18551, 70 bars에서 계속 정상 실행 중이며 설정이나
프로세스는 건드리지 않았다.

#### 확정된 사실

- 70 bars 진입(epoch 15618) 후 2,934 epoch의 epoch-level 평균은 capture `0.6830`,
  crash `0.3117`, timeout `0.0053`이다. 최근 100 epoch는 capture `0.6960`까지 반등했지만,
  최근 400/1000 epoch는 각각 `0.6846/0.6899`라 지속적인 0.70 돌파로 보기는 어렵다.
- 앞서 집계한 16,384-finished-episode 창 13개가 모두 `0.663~0.699`였다. 진짜 성공률이
  약 0.683이면 한 창의 이항 표준오차는 약 0.0036이므로 0.70은 약 4.7σ 위다. 따라서 긴 창이
  우연히 나쁘게 뽑힌 문제가 아니라 현재 **stochastic training policy**가 약 68%에 머문 것이다.
- 70-bars 2,927개 정렬 epoch에서 capture와 평균 target speed의 상관은 `0.0136`, goal-distance
  max와의 상관은 `0.0185`다. 두 변수의 사분위별 capture도 모두 `0.6817~0.6847`이다.
  특정 고속 표적이나 먼 목표가 평균을 끌어내린다는 가설은 현재 로그에서 기각된다.
- 실패의 약 91%는 bar contact이고 below는 약 7%, timeout은 약 0.5%다. 충돌 x는 평균
  `12~13 m`, episode step은 약 `38~41`이라 스폰/goal/timeout 가장자리 문제가 아니라
  막대밭 중앙 위빙 중 충돌이다.

#### 가장 먼저 검증할 원인: stochastic gate와 큰 횡행동 노이즈

밀도 gate는 별도의 deterministic 평가가 아니라 PPO rollout의 **샘플 행동**으로 끝난 episode를
그대로 센다. epoch-18550 체크포인트의 정책 log-std/std는 행동 `[x,y,z,yaw]` 순서로:

```
raw log-std = [-0.992, 0.132, 0.400, -2.531]
effective std = [0.371, 1.141, 1.492, 0.080]
```

`y`는 기체 좌우 속도 행동이고 환경에서 `clamp(action,-1,1) * 2.5 m/s`로 변환된다. std 1.141이면
평균 행동이 0이어도 정규분포 샘플의 약 38%가 ±1 바깥이라 매 step 좌우 명령이 포화된다
(평균이 0이 아니면 포화율은 더 커질 수 있다). 이 std는 65-bars 시작 체크포인트의 `1.063`에서
현재 `1.141`로 증가했다. 반면 x/yaw std는 `0.444→0.371`, `0.103→0.080`으로 감소했다.

z std가 상한 1.492에 붙은 것은 task가 z 행동을 즉시 PI altitude command로 덮어써 실제 기동에는
영향이 없지만, 사용하지 않는 차원에도 entropy bonus가 계속 걸린다는 별도 설계 결함이다. 수치
안정성은 괜찮다(entropy 최근 약 2.69, explained variance 약 0.69, LR 1e-4). 다만 좌우 노이즈는
실제 막대 회피를 매 step 교란한다. 따라서 “평균 정책은 70%가 가능한데 탐색 rollout으로 재는
gate만 못 넘는다”가 현재 가장 값싼 설명이다. 아직 동일 체크포인트의 deterministic/stochastic
paired eval이 없으므로 **유력 가설이지 확정 원인은 아니다**.

#### 두 번째 원인 후보: 밀집 기하를 지나치게 압축하는 표현

- 70-bars bar-contact 순간 12m 안에는 평균 `43~44`개 막대, 240° token FOV에는 `29~31`개,
  장애물이 찬 scan bearing은 약 `54/72`개다.
- 8개 token은 모두 valid지만 range+bearing 기하로 GT bar에 연관되는 것은 평균 `2.5~3.0`개,
  서로 다른 막대는 `1.1~1.3`개뿐이고 나머지는 같은 근접 막대 표면 중복 또는 비연관 표면이다.
  충돌 막대가 240° 안에 있었을 때도 token으로 표현된 비율은 최근 대략 `0.47~0.63`,
  장기 평균 약 `0.56`이다.
- 선택 로직은 물체/경로 위험도를 구분하지 않고 nearest surface bearing을 순서대로 고른 뒤
  ±10°를 억제한다. 가까운 넓은 막대가 여러 slot을 소비하고 진행 경로상의 다른 막대가 누락될 수
  있다. 또한 8개 token 전체를 시간 step당 하나의 64-D token으로 projection하므로 slot별
  attention이나 안정적인 object identity가 없다.
- 360° 4×72 static scan은 남아 있지만 CNN이 이것 전체를 하나의 64-D token으로 압축한다. 따라서
  “충돌 막대 token 누락 = 완전 실명”은 아니나, 조밀한 free-space topology를 policy가 쓰기 좋은
  형태로 유지한다고도 보장할 수 없다.

bar probe는 **충돌 순간** 측정이라 token 누락이 충돌의 선행 원인임을 단독으로 증명하지는 못한다.
그러나 25→65→70으로 밀도가 올라갈수록 실패가 bar contact로 수렴하고, 과거 GT-LiDAR policy는
110 bars에서 약 0.93을 달성했으므로 70 bars가 환경의 절대 기하 한계라는 설명은 기각된다.
현재 sensor-only 표현/정책/탐색의 병목이다.

#### 후순위·미측정 가설

- 보상: safety reward가 전 ray의 mean-log-distance라 국소 충돌 위험 신호가 희석될 수 있으나,
  과거 clearance reward 세 실험이 모두 null이었으므로 첫 수정 후보는 아니다.
- 제어: 2.5 m/s에서 yaw 3.0 rad/s가 조밀 위빙에 부족하거나 action saturation이 있을 수 있으나
  현재 action/yaw saturation telemetry가 없어 단정할 수 없다.
- target occlusion: tracker age/visibility가 actor에는 들어가지만 bar-contact outcome별 visibility를
  로그하지 않아 아직 분리하지 못했다. target speed/goal distance 무상관만으로 occlusion까지
  기각할 수는 없다.

#### 다음 진단 순서

1. 현재 체크포인트를 70 bars, 동일 seed/distribution에서 `deterministic=True`와 stochastic 두 조건,
   최소 4096 episodes씩 paired 평가한다. deterministic만 0.70을 넘으면 gate/탐색 노이즈가 주원인이다.
2. 둘 다 0.70 아래면 충돌 전 1~2초 구간의 `hit-bar represented`, target visibility/track age,
   action clamp율, yaw clamp율을 success/bar-contact별로 기록한다.
3. 그 결과 뒤에만 수정한다. 1번이 맞으면 커리큘럼 gate를 deterministic probe로 분리하거나
   lateral std/entropy 구조를 고친다. 2번이 맞으면 단순 token 수 증가보다 object/segment 또는
   TTC·진행경로 위험 기반 token과 slot별 tokenization을 우선한다.

### 2026-07-28 02:40 — NavRL/NavRL++ 원문 밀도 대조 및 원인 가설 정정

사용자가 “NavRL/NavRL++에 비해 70막대가 훨씬 많은 것은 아니지 않느냐”고 지적해 원 논문,
공개 NavRL 코드, 현재 환경 코드를 같은 면적 기준으로 다시 감사했다. 결론은 **사용자 지적이 맞다.**
70막대를 고밀도 자체의 한계로 해석하거나, 정적 막대 수를 8 obstacle token 용량과 직접 비교한
앞선 설명은 잘못됐다.

#### 원문·공개 구현에서 확인한 환경

- NavRL 공개 학습 config: `350 static + 80 dynamic`.
- NavRL 공개 코드의 실제 obstacle 영역: `map_range=[20,20,4.5]`, 즉 내부 `40×40 m`.
  static 폭은 `0.4~1.1 m`, dynamic 폭은 `0.25~1.0 m`, 충돌 robot radius는 `0.3 m`.
- NavRL 논문은 전체 환경을 `50×50 m`로 표기하고 static 350개를 고정한 채 dynamic을
  `60→80→100→120`으로 승급한다. 코드상 장애물 자체는 중앙 40×40m에 배치되고 바깥은 border다.
- NavRL++ Table I:
  - S1: `300 static + 60 dynamic`
  - S2: `350 + 80`
  - S3: `400 + 100`
  - S4: `400 + 120`
  - S5: `400 + 140`
- NavRL++ 평가는 명시적으로 `40×40 m` 안에 static `300/350/400`, dynamic 환경은 여기에
  `60/80/100`을 추가한다. high-complexity 결과는 static SR `99.84%`, dynamic SR `83.96%`다.
- NavRL++ 정적 표현은 `4×36` 360° ray-distance map을 CNN으로 하나의 64-D static token으로
  만든다. 최대 5개 object state와 그 history는 **dynamic obstacle 전용**이다.

#### 면적당 밀도 비교

우리 막대 배치 밴드는 `(0.96-0.13)×24×24 = 478.08 m²`이고, 70 bars는
`14.64 bars/100m²`다. NavRL++의 40×40m 기준은:

| 조건 | static/100m² | static+dynamic/100m² | 우리 밴드 환산 개수 |
|---|---:|---:|---:|
| 우리 70 bars | 14.64 | 14.64 | 70 |
| NavRL++ S1 | 18.75 | 22.50 | static 90 / total 108 |
| S2 | 21.88 | 26.88 | static 105 / total 128 |
| S3 | 25.00 | 31.25 | static 120 / total 149 |
| S4 | 25.00 | 32.50 | static 120 / total 155 |
| S5 | 25.00 | 33.75 | static 120 / total 161 |

4m 반경의 균일밀도 기대 장애물 수도 우리는 약 `7.36`, NavRL++은 S1 total `11.31`,
S2 `13.51`, S5 `16.96`이다. 현재 probe의 “12m 안 43~44개”는 NavRL++의 4m local range와
다른 반경을 사용한 수치라 직접 비교하면 안 된다.

우리 bar 폭/깊이는 실측 평균 `0.603 m`(범위 `0.403~0.790 m`), NavRL 공개 static은
`0.4~1.1 m`라 우리 막대가 특별히 큰 것도 아니다. 우리 70-bar 배치 Monte-Carlo 100 layouts는
drone 반폭 0.14m에 추가 side clearance `0.2 m`를 요구해도 path-exists `100/100`, placement
relaxation `0/100`, 최종 center spacing `1.5 m`였다. 따라서 70 bars는 물리적 통로 한계도 아니다.

#### 철회·수정하는 가설

- **철회:** “70 bars라 8 token이 감당하지 못해 68% ceiling이 생긴다.”
- 이유: 우리 막대는 static obstacle이고, NavRL++도 수백 개 static obstacle을 object token 5개가
  아니라 ray map 하나로 처리한다. 우리 actor에도 더 촘촘한 `4×72` full 360° static scan이 별도로
  있으며 8 proposal token은 보조 경로다.
- bar-contact 순간 `unique token bars≈1.2`, `hit_token_given_fov≈0.56`은 우리 auxiliary proposal
  selector가 static surface를 비효율적으로 중복 선택한다는 증거일 수는 있으나, policy가 충돌 막대를
  전혀 관측하지 못했다는 증거가 아니다. full static scan에 남아 있기 때문이다.
- NavRL++도 static scan 전체를 하나의 64-D token으로 압축하므로 “64-D static token 자체가
  70 bars를 못 담는다” 역시 비교 근거가 약하다.

#### 비교 후 더 유력해진 차이

1. **행동 분포/gate 불일치:** NavRL/NavRL++은 bounded Beta action을 사용하며 논문도 Gaussian의
   boundary bias를 피하는 이유를 명시한다. 우리는 Gaussian을 샘플한 뒤 clamp한다. 현재 lateral
   std `1.141`이면 최소 약 38%가 매 step 경계에 잘려 ±2.5m/s 횡명령으로 포화된다. stochastic
   rollout으로 승급을 판정하는 현재 구조와 직접 충돌한다.
2. **문제 설정 차이:** NavRL++은 정확히 알려진 static navigation goal을 goal-aligned frame으로
   관측한다. 우리는 target GT를 actor에서 제거했고, 움직이며 막대에 가려지는 target을 RGB-D/LiDAR
   tracker로 찾아야 하고, random yaw의 vehicle frame에서 yaw까지 학습한다. target visibility/track
   age를 bar-contact outcome별로 아직 측정하지 않았다.
3. **기동 자유도:** 우리 막대는 모두 0~2m이고 drone은 z=1m에 고정돼 2-D로 통과해야 한다.
   NavRL/NavRL++ UAV는 3-D velocity를 사용하고 높이가 다른 장애물 일부를 상하로 회피할 수 있다.
   다만 우리 70-bar layout은 0.2m 추가 여유를 둬도 전부 연결되므로 이것만으로 68%는 설명되지 않는다.
4. **제어 주기:** 우리는 10Hz, NavRL++ 기본은 50Hz다. 하지만 NavRL++ 자체 ablation에서 10Hz
   overall SR `93.20%` vs 50Hz `94.08%`로 차이가 0.88pp여서 주원인일 가능성은 낮다.

정정된 우선순위는 `(1) deterministic/stochastic paired eval → (2) lateral clamp telemetry →
(3) target visibility/track-age conditioned crash 분석`이다. static token 수 증가는 이 세 검증 뒤로
내린다. 실행 중 학습에는 어떤 변경도 하지 않았다.

### 2026-07-28 02:48 — 행동 포화 가설의 의미 명확화

`lateral std=1.141` 문제는 최대 속도 제한을 더 풀어야 한다는 뜻이 아니다. 현재 Gaussian sample을
`[-1,1]`로 clamp한 뒤 2.5m/s를 곱하므로, 큰 분산이 매 step 좌우 최대속도 명령을 자주 만든다는 뜻이다.
상한을 높이면 같은 포화 sample이 더 큰 횡속도가 되어 충돌을 악화시킬 수 있고, clamp를 없애면
unbounded Gaussian 명령이 controller로 들어가므로 둘 다 올바른 수정이 아니다.

검증 후 가능한 수정 방향은 bounded Beta/tanh-squashed policy, 실제 사용하는 action 차원별 적절한
std 제한 또는 entropy 조정, 사용하지 않는 z action 제거, stochastic rollout과 deterministic
curriculum gate 분리다. 우선순위는 여전히 동일 체크포인트의 deterministic/stochastic paired eval이다.

### 2026-07-28 02:36 — 현재 작업 스냅샷 커밋 정리

사용자 요청에 따라 지금까지의 추적 파일 변경을 하나의 Git 스냅샷으로 정리했다.

- 연구 현황 대시보드의 compact panel 레이아웃 변경(`docs/status/index.html`,
  `docs/status/style.css`)을 포함했다.
- 대시보드가 외부 CDN 연결 없이 3D arena를 열 수 있도록 Three.js r128과
  OrbitControls 로컬 사본(`docs/status/vendor/`)을 포함했다. Three.js 파일에는
  MIT SPDX 라이선스 헤더가 보존돼 있다.
- 실행 중인 학습에서 계속 생성되는 `train_session_logs/`(약 81 MB)와
  `checkpoints_saved/`(약 15 MB)는 소스 변경이 아닌 런타임 산출물이므로 Git에는 넣지 않았다.
- 커밋 전 `git diff --check`, JavaScript DOM 참조 감사, headless Chrome 로딩 검사를 수행한다.

### 2026-07-28 02:38 — PPO action noise 대안 감사

사용자 요청에 따라 bounded Beta에 한정하지 않고 현재 `rl_games` 구현, 최신 체크포인트,
bounded-action 정책 및 시간상관 탐색 관련 원 논문을 대조했다. 실행 중인 학습에는 변경을 가하지 않았다.

#### 현재 정책에서 확인한 사실

- `continuous_a2c_logstd`가 대각 Gaussian에서 매 step 독립 표본을 뽑고, 환경
  `transform_action_to_command()`가 뒤에서 `[-1,1]`로 hard clamp한다.
- 최신 `last_gen_ppo_ep_19100_rew_23.645699.pth`의 축별 `(log_std, std)`는
  forward `(-1.007, 0.365)`, lateral `(0.146, 1.157)`, unused-z `(0.400, 1.492)`,
  yaw `(-2.568, 0.077)`이다.
- 평균이 0인 가장 유리한 경우에도 횡축 표본의 `38.7%`가 `|a|>1`이고, 따라서 실제 clamp율은
  최소 38.7%다. `NAVRL_MAX_VELOCITY=2.5`이므로 이 질량은 매번 `±2.5 m/s` 횡명령에 모인다.
- z action은 altitude PI가 덮어써 물리적으로 사용되지 않는데 entropy bonus는 이 축에도 걸린다.
  실제로 z std가 상한에 붙어 있어 총 entropy 지표도 정책에 쓰이는 세 축보다 크게 보인다.
- `bounds_loss_coef`는 rl_games 코드상 `|mu|>1.1`인 평균만 벌점화한다. sigma와 경계 밖
  표본 질량은 줄이지 않으므로 이 값을 키우는 것은 현재 문제의 직접 해법이 아니다.
- 학습은 감사 시점에 75 bars까지 승급했고 최근 2050 episode의 capture는 `0.674`,
  crash는 `0.322`였다. 따라서 70 bars gate가 영구 정체였다는 가정은 이미 기각됐다.

#### 대안 우선순위

1. **현 Gaussian + 단계별 std/entropy annealing**: 초기 `entropy_coef=0.003`은 hover 탈출까지
   유지하고, competence를 얻은 뒤 횡축 std 목표를 우선 `0.5`로 낮춘다. 평균 0 기준 clamp 질량은
   `38.7% → 4.55%`가 된다. 고정 entropy가 다시 std를 밀어 올리지 않도록 entropy coefficient도
   함께 낮추거나 성능 기반 schedule로 바꿔야 한다. 가장 작고 해석 가능한 변경이다.
2. **tanh-squashed Gaussian PPO**: 설치된 rl_games에 이미 `continuous_a2c_tanh`와 Jacobian을
   반영한 log-prob 구현이 있다. Beta 없이도 action support가 `(-1,1)`로 제한되고 hard-clamp
   pile-up이 사라진다. 다만 현재 player의 deterministic 경로는 raw `mu`를 사용하므로
   `tanh(mu)` mode를 사용하도록 먼저 고쳐야 하며, 기존 checkpoint의 log-std가 `exp`에서
   `softplus`로 재해석되는 점도 warm-start A/B에서 통제해야 한다.
3. **gSDE/시간상관 탐색**: marginal std를 단순히 줄이기보다 noise를 여러 control step 동안
   유지해 10 Hz의 좌우 독립 jitter를 부드러운 탐색 궤적으로 바꾼다. 로봇에서 성능 손실 없이
   smoothness를 조절할 수 있다는 CoRL 결과가 있으나 현재 rl_games PPO에는 바로 연결된 구현이 없어
   1·2보다 작업량이 크다.
4. **CAPG 또는 truncated Gaussian**: 현 hard clamp를 유지하면서 tail probability에 맞는
   likelihood/gradient를 써 clipping mismatch를 고칠 수 있다. CAPG는 unbiased·저분산 estimator지만
   표본의 경계 집중 자체를 없애지는 않으므로 std schedule과 병행해야 한다. truncated Gaussian은
   직접 bounded지만 PPO ratio, entropy, CDF 정규화까지 새로 구현해야 해 tanh보다 우선순위가 낮다.
5. **action EMA/action-rate penalty**: 고주파 떨림 완화에는 유효할 수 있지만 raw Gaussian과 실제
   실행 action의 차이를 또 만들 수 있다. 현재 보상에도 achieved-velocity smooth penalty가 이미 있어,
   먼저 raw action clamp율과 `Δaction` telemetry로 고주파 jitter가 실제 crash 원인인지 확인한다.

권장 실험은 현재 run을 건드리지 않고 동일 checkpoint에서 `(A) 기존 stochastic`,
`(B) deterministic`, `(C) lateral std=0.5`를 paired seed로 평가한 뒤, C가 capture/crash를
개선하면 entropy/std annealing을 먼저 적용하고 tanh-Gaussian을 별도 branch로 비교하는 순서다.

참고 원문: PPO (Schulman et al., 2017), CAPG (Fujita & Maeda, 2018), SAC Appendix C의
tanh change-of-variables (Haarnoja et al., 2018), gSDE (Raffin et al., CoRL 2022),
Beta policy (Chou et al., ICML 2017).

### 2026-07-28 17:36 — 실행 중 density run의 시간 대비 학습효율 감사

사용자가 현재 승급 학습을 계속 오래 돌리는 것이 실제로 도움이 되는지 물어
`ppo_260727_2324_navrl`의 전체 로그(15001~37194 epoch), 체크포인트 sigma, promotion gate,
crashdiag/barprobe를 읽기 전용으로 집계했다. 프로세스는 중단하거나 변경하지 않았다.

#### 현재 상태

- PID `1652517`, 약 18.2시간 실행, GPU 5.85 GiB, 점검 당시 75 bars / 약 37194 epoch.
- 45000까지 약 7800 epoch가 남아 현재 속도 2.6 s/epoch 기준 추가 약 5.6시간이다.
- 실제 promotion은 두 번뿐이다.
  - 65→70: epoch 15617, 실행 0.49시간, gate capture `0.706/16388 eps`
  - 70→75: epoch 18787, 실행 2.96시간, gate capture `0.704/16386 eps`
- 이후 75 bars에서 18407 epoch, 약 147.2만 episode, 약 15.2시간을 보냈지만
  80 bars로 한 번도 승급하지 못했다.

#### 밀도별 집계

| bars | epoch | episodes | 전체 capture | 처음 500 epoch | 마지막 500 epoch |
|---:|---:|---:|---:|---:|---:|
| 65 | 617 | 49,128 | 0.696 | 0.692 | 0.699 |
| 70 | 3,170 | 245,723 | 0.684 | 0.670 | 0.692 |
| 75 | 18,407 | 1,471,998 | 0.634 | 0.653 | 0.587 |

65와 70에서는 완만한 개선과 승급이 있었으므로 처음 약 3시간은 유효했다. 반면 75는
처음 500 epoch보다 마지막 500 epoch capture가 `6.6pp` 낮고 crash가
`34.3%→41.1%`로 증가했다. 최근 2048-episode window도 `capture=0.539~0.565`다.
1000-epoch bin은 중간에 0.65 수준으로 회복한 적은 있으나 0.70 gate를 지속해서 넘은 적이 없다.

75 bars에서 gate window는 `16384 eps`이고 총 147만 episode를 보았으므로 약 89회의
실패 판정을 받은 셈이다. 로그에 `held`가 안 보이는 이유는 run 시작(23:24) 12분 뒤
`cbefbe3`에서 INFO→WARNING으로 가시성을 고쳤기 때문이다. 실행 중 Python은 시작 당시 코드를
메모리에 유지하므로 hold는 수행됐지만 출력만 억제됐다. 즉 승급 로직이 완전히 멈춘 것이 아니라
정책이 문턱을 못 넘은 것이다.

#### 시간이 성능을 악화시킨 정황

- epoch 19050→37150 동안 lateral std가 `1.160→1.492`로 증가해 상한에 붙었다.
  평균 0일 때조차 횡 action hard-clamp 최소 확률이 `38.7%→50.3%`가 된다.
- 같은 기간 forward std는 `0.367→0.192`, yaw는 `0.077→0.062`로 내려가므로
  “모든 행동이 무작정 시끄러운 것”이 아니라 횡축만 선택적으로 포화됐다.
- bar-contact의 crash 내 비중은 초반 20 window `91.4%`, 최근 20 window `92.0%`로 비슷하지만,
  crash 자체가 증가해 절대 bar-contact는 대략 `31.3%→37.8%`로 악화됐다.
- 반대로 `hit_token_given_fov`는 `0.559→0.638`로 좋아졌다. 관측 token hit가 개선됐는데
  포획이 하락했으므로 이번 장기 정체를 representation 용량 부족으로 설명하기 어렵고,
  action 분포/제어 쪽 증거가 더 강하다.

#### 판정 및 다음 조치

**현재 설정 그대로 45000까지 추가 5.6시간을 쓰는 것은 권장하지 않는다.**
75 bars에서 이미 충분한 표본을 훨씬 넘겼고 최근 성능·횡축 sigma가 모두 나빠지고 있다.

중단 후 평가 후보는 75-bar 로그 근방 성능이 좋았던:

- `last_gen_ppo_ep_19050_rew_31.79068.pth` 부근 — 100-epoch 근방 capture `0.691`
- epoch 27450 부근 — `0.681`
- epoch 30450 부근 — `0.672`
- 최신 checkpoint — 퇴행 대조군

이다. 동일 seed로 deterministic/stochastic paired 평가해 19050 계열이 실제 held-out에서도
우세한지 확인한 뒤, 그 checkpoint에서 lateral std=0.5 + entropy annealing branch를 시작하는
것이 다음 합리적 실험이다. `gen_ppo.pth`는 낮은 밀도의 best-reward일 수 있으므로 후보에서 제외한다.

### 2026-07-28 17:46 — 실행 유지 상태에서 전체 로드맵·핵심 병목 재판정

사용자 요청에 따라 실행 중인 `ppo_260727_2324_navrl`은 종료·신호 전송·설정 변경 없이
그대로 유지했다. 점검 시 PID `1652517`, epoch `37401/45000`, 75 bars였고 최근 epoch별
capture는 대체로 45~58%였다.

#### 현재 run의 결론

- 65→70→75까지 약 3시간은 유효한 curriculum 학습이었다.
- 75 bars에서는 약 147만 episode와 약 89개의 16,384-episode gate window를 소비했지만
  80 bars로 승급하지 못했다. 처음 500 epoch capture `0.653`에서 최근 500 epoch
  `0.587`로 하락했고 crash는 `0.343→0.411`로 증가했다.
- 같은 기간 obstacle token의 `hit_token_given_fov`는 `0.559→0.638`로 개선됐는데 절대
  bar-contact는 약 `0.313→0.378`로 증가했다. 따라서 이번 정체의 1차 원인은 8-token
  표현 용량이나 기하학적 불가능성보다 action/optimization 쪽이다.
- 가장 강한 기전은 고정 entropy `0.003`, state-independent unbounded Gaussian,
  후단 hard clamp의 조합이다. 75-bar 초반→최근 lateral std가 `1.160→1.492`로 상한에
  붙어 평균 0에서도 `|a_y|>1`인 표본의 최소 비율이 `38.7%→50.3%`가 됐다. 이 표본은
  실제로 `±2.5 m/s` 횡속도 명령에 쌓인다.
- 다만 sigma와 crash의 상관관계만으로 인과가 완전히 확정된 것은 아니다. 같은 checkpoint의
  deterministic/stochastic paired evaluation과 raw clamp-rate/`Δaction` 계측이 필요하다.

#### capture 비율 개선 우선순위

1. 현재 run은 끝까지 진단 궤적으로 보존하되, 다음 학습의 warm-start 후보는 최신이 아니라
   75-bar 초반의 `last_gen_ppo_ep_19050...pth`와 중간 후보 27450/30450을 held-out 평가해 고른다.
2. 같은 seed·75 bars·4096 episode 이상으로 각 후보의 deterministic/stochastic 성능을 나눠
   평균 정책 퇴행과 sampling noise를 분리한다.
3. noise가 주원인이면 기존 Gaussian의 초기 탐색은 유지하고 competence 이후
   `entropy_coef 0.003→0.0003(또는 0)`과 lateral std 목표/상한 `0.5`를 함께 anneal하는
   최소 변경 branch를 먼저 시험한다. 평균 0 기준 clamp 질량은 약 `50.3%→4.55%`로 줄어든다.
4. 별도 A/B로 tanh-squashed Gaussian을 시험한다. 설치된 `continuous_a2c_tanh`는 사용할 수
   있지만 deterministic player의 `tanh(mu)` 처리와 기존 checkpoint std 재해석을 먼저 고쳐야 한다.
5. curriculum gate는 noisy stochastic train ratio만 보지 말고 고정 seed deterministic
   competence probe를 병행한다. 반대로 threshold를 단순히 0.70→0.60/0.65로 내리는 것은
   퇴행 중인 정책을 다음 밀도로 넘기므로 해결책이 아니다.
6. 속도 상한 확대, reward 재설계, token 억제폭 변경은 현재 증거상 우선순위가 낮다.
   특히 속도 상한 확대는 포화 action의 실제 속도만 키워 충돌을 악화시킬 수 있다.

#### 전체 로드맵의 실제 위치

- 정보 firewall, raw RGB-D/LiDAR 렌더링, tracking/fusion, 17-token Transformer,
  navigation 통합은 구현되어 있어 **엔지니어링 파이프라인은 약 65~75%**로 본다.
- 현재 density curriculum은 목표 25→110 bars 중 75까지 도달했으므로 bar interval 기준
  `(75-25)/(110-25)=58.8%`지만, 75→80 performance gate에 막혀 있다.
- 논문 수준의 실증은 **약 35~45%**다. 25 bars speed-axis와 단일 seed의 부분 density curve는
  있으나, density×speed×occlusion 전체 matrix, 3개 이상 seed, CNN/LSTM/Transformer 및
  camera/LiDAR/fusion ablation, occlusion 생존·재획득 검증이 남아 있다.
- 전체 프로젝트에서 다음 큰 병목은 detector다. 현재 appearance segmenter는 학습된 detector
  증거가 아니라 red-color bootstrap 초기화이며, sequence dataset·held-out detector metric·
  detector checkpoint가 확인되지 않았다. navigation capture를 개선해도 이 부분 없이
  learned-perception 주장을 완성할 수 없다.
- 따라서 대시보드의 P0~P5 “done” 표시는 기능 통합 관점에서는 이해되지만 과학적 검증 완료로
  읽으면 과대평가다. 문서와 대시보드의 완료 기준을 분리해 갱신할 필요가 있다.

현재 run이 GPU를 사용하는 동안에는 평가를 겹쳐 실행하지 않는다. 종료 후 paired evaluation으로
최선 checkpoint와 원인을 확정하고, 그 결과에 따라 std/entropy annealing branch를 우선 진행한다.

### 2026-07-28 18:24 — bounded-action A/B 구현 및 main 실험 시작

사용자가 기존 density run을 더 연장하는 대신, 평균 action이 0이어도 큰 Gaussian sigma 때문에
횡축 표본이 범위 밖으로 나가 task clamp 뒤 `±2.5 m/s`에 합쳐지는 문제를 직접 고치도록 방향을
변경했다. 이 요청은 앞선 “현재 run 유지” 요청을 대체하는 것으로 해석했다. 기존 PID `1652517`은
정확한 command line을 재확인한 뒤 `SIGINT`로 정상 종료했고, 종료 전 마지막 주기 checkpoint
`last_gen_ppo_ep_37850_rew_31.752436.pth`가 존재함을 확인했다.

#### 원인과 설계 결정

- 기존 정책은 unbounded Normal sample을 PPO action으로 기록하고 task에서 뒤늦게 hard clamp했다.
  최근 lateral sigma `1.492`이면 mean=0인 가장 유리한 경우에도 `P(|a_y|>1)=50.3%`다. 서로
  다른 절반가량의 표본이 같은 `±2.5 m/s` 명령으로 합쳐지고, PPO likelihood는 실제 실행 action과
  다른 확률변수를 최적화했다.
- 단순 sigma clamp만으로는 support/likelihood 불일치를 없애지 못하고, Beta policy는 기존
  Gaussian checkpoint의 actor head를 직접 warm-start하기 어렵다. CAPG는 clipped policy의
  gradient variance는 줄이지만 boundary sample 자체를 없애지 않는다. gSDE는 시간적으로 부드러운
  탐색의 후속 후보지만 action support 문제의 직접 해법은 아니다.
- main은 likelihood-correct **tanh-squashed Gaussian**, 1650 Ti sub는 **scale-adjusted truncated
  Gaussian**으로 정했다. 둘 다 실행 action과 PPO log-probability가 같은 bounded distribution을
  가리킨다.
- 공정한 A/B를 위해 둘 다 epoch `19050`, 75 bars, seed 1, 128 envs, 3000 추가 epoch,
  base std `[0.35, 0.35, 0.05, 0.08]`, entropy 0, actor Adam moment reset을 사용한다.
  density promotion은 끄고 75 bars로 고정했다.
- 첫 main smoke에서 tanh만 붙였을 때 기존 clipped-policy가 물려준 큰 lateral mean 때문에
  `edge99_y=98.75%`가 됐다. 따라서 checkpoint 표현/value는 유지하되 inherited lateral mean만
  `0.4×`로 재보정했다. 이는 속도 상한을 낮추는 것이 아니며, PPO가 필요하면 mean을 다시 키울 수
  있다.

#### 구현

- `navrl_action_models.py`
  - `NavRLSquashedGaussianModel`: tanh transform, stable Jacobian correction, bounded stochastic/
    deterministic action, transformed entropy.
  - `NavRLTruncatedGaussianModel`: `[-1,1]` inverse-CDF sampling, normalized exact log-probability와
    entropy, boundary 부근 scale-adjustment(`d_min=0.01`).
  - 기존 checkpoint의 `state_dict` key/shape를 그대로 보존하고 dead legacy sigma parameter는
    ABI 용도로만 유지한다.
- `navrl_players.py`: rl_games 1.6.5 deterministic player가 pre-tanh `mus`를 실행하던 문제를
  막고 model의 bounded `deterministic_actions`를 사용한다.
- `runner.py`: action model/player 등록, `NAVRL_ACTION_POLICY` 선택, checkpoint env-state의
  distribution/std/mu-scale/d_min 자동 복원, run tag 분리.
- `early_stop_a2c_agent.py`: action-distribution branch restore 시 actor optimizer moment만
  초기화하고 central critic state는 유지한다. clamp 전 `raw_oob`, edge95/99, mean action,
  mean mu/sigma, temporal `delta_y`를 TensorBoard에 기록한다.
- `navrl_task.py`: task 입력 OOB와 실제 edge98/mean_abs/delta/sign-flip을 별도 기록하고,
  checkpoint에 action contract를 저장하며 평가 설정 불일치를 경고한다.
- `train_navrl_general_repr_density.sh`: `NAVRL_FIXED_BARS`를 추가해 action A/B에서 curriculum을
  비활성화하고 동일한 75-bar task를 강제한다.
- 실행 파일:
  - main: `train_navrl_action_squashed_main.sh`
  - 1650 Ti: `train_navrl_action_truncated_1650ti.sh` (`GPU4GB=1`, `base_sim_4gb`)
- CPU 단위시험 `test_navrl_action_models.py` 7개를 추가했다. bounded/finite sample, legacy
  state keys, theoretical Gaussian tail, lateral warm-start scale, tanh deterministic action,
  PyTorch `TransformedDistribution`과 log-prob 일치, truncated PDF 적분값 1을 검증한다.

#### 검증 및 현재 main 결과

- 단위시험: `Ran 7 tests ... OK`.
- `bash -n`, `py_compile`, `git diff --check` 통과.
- 두 launcher 모두 checkpoint preflight-only 통과했고 실행 권한을 확인했다.
- old checkpoint를 실제 main model에 strict restore한 뒤 PPO update가 진행되고 있으므로
  checkpoint compatibility는 smoke 수준을 넘어 runtime으로도 확인됐다.
- 현재 main:
  - PID `3089290`
  - run `ppo_260728_1817_navrl_action-squashed-main-s1`
  - log `train_session_logs/action_squashed_main_260728_181755.log`
  - 점검 시 epoch `19162/22050`, 프로세스 정상, GPU 약 `5.72 GiB`.
- epoch 19051~19162 누적 capture `5481/7895 = 69.42%`, crash `30.09%`,
  최근 50 epoch capture `2494/3561 = 70.04%`. 즉 bounded 전환 직후 competence가 붕괴하지
  않았다.
- 세 개의 독립 task diagnostic window에서 lateral `task_input_oob_y=0.0000`; 실제
  `|command_y|>=0.98*2.5` 비율은 `3.71% → 5.20% → 6.71%`였다. 기존 sigma만으로 계산되는
  mean=0 hard-clamp 하한 `50.3%`와 비교하면 `±2.5` 몰림은 약 한 자릿수 비율로 줄었다.
- 다만 `mean_abs_y=0.899→0.916`, `edge95_y`도 상승 중이다. 이는 random OOB tail은 제거됐지만
  actor mean이 강한 횡이동을 다시 선택한다는 뜻이다. main을 중단하지 않고 끝까지 관측하되,
  최종 판정은 capture뿐 아니라 edge98_y, bar-contact, deterministic/stochastic paired 평가를
  함께 사용한다. edge98_y가 다시 크게 오르면 다음 개입은 속도 상한 변경이 아니라 lateral
  mean-margin regularization 또는 더 작은 warm-start scale이다.

1650 Ti sub는 같은 checkpoint 파일을 복사한 뒤 별도 머신에서 launcher만 실행한다. 128 env를
먼저 사용해 main과 PPO batch를 맞추고, 실제 4 GiB OOM이 확인될 때만 `NUM_ENVS=64`로 낮춘다
(`32*64=2048`이라 configured minibatch와 정확히 일치한다).

### 2026-07-28 18:43 — TensorBoard 세션 정리 및 날짜별 run 색인

사용자 요청에 따라 `rl_games/runs/`를 전수 감사했다. 정리 전에는 top-level run directory
38개, TensorBoard 표시 세션 36개, event file 43개, 전체 약 14 GiB였다. TensorBoard
`EventAccumulator`로 모든 event protobuf와 scalar를 읽었고 손상된 event file은 **0개**였다.
40-byte event들은 손상이 아니라 header-only 초기화 artifact였다.

#### 정리 기준과 방식

- 엄격한 short-run 기준은 **완료 epoch ≤120**으로 잡았다. 일반 본학습 6000 epoch의 2% 이하이고,
  실제 분포에서도 120 다음이 335 epoch라 자연스러운 간격이 있다.
- short-run 외에는 후속 정식 run으로 대체된 `0220`, 원인이 WORKLOG와 후속 `1309`에 완전히
  남은 collapse-guard 오발 `1204`, 수정된 `1817`로 대체된 unscaled action smoke `1813`만
  정리했다.
- 연구 provenance를 잃지 않도록 영구 삭제하지 않고 TensorBoard logdir 밖의 복구용 경로
  `/home/fair/workspaces/aerial_gym_ws/tensorboard_archive/2026-07-28_pruned/`로 옮겼다.
  이동량은 약 298 MiB다. 한 run을 복구하려면 해당 directory를 `rl_games/runs/`로 되돌리고
  TensorBoard를 reload하면 된다.
- 현재 학습 `ppo_260728_1817...`과 그 source checkpoint가 있는 `ppo_260727_2324...`는
  process command line까지 확인하고 보호했다.
- `ppo_260719_1000`의 event 6개와 `ppo_260723_1509`의 event 3개는 중복 세션이 아니라 같은
  run의 resume shard이므로 임의로 쪼개 삭제하지 않았다.

#### 실험 날짜별 TensorBoard 색인

| 실험 날짜 | TensorBoard에 보존한 핵심 세션 | 이번에 아카이브한 세션 |
|---|---|---|
| 07-14~15 | `1904` 정직한 baseline, `2207` clearance negative ablation, `0251` yaw 해결, `2038` random-150 seed1 | 없음. `1552` grid-150과 `1922` interrupted grid-75는 event가 없어 원래부터 TB 비표시 |
| 07-16~17 | `density_25`, `density_50`, `0329`=75, `1223`=110, `density_120`, `0032`=150 seed2 — 밀도곡선 원자료 | 없음 |
| 07-18 | `0259` fixed-110 vision baseline, `1841` 동시 밀도 curriculum, `vision50_seed2` | `0220` — 50 bars에서 833 epoch 중단, 정식 후속 run으로 대체 |
| 07-19 | `1000` 12000-epoch 순차 curriculum, `vision_goalcap_seed1` | 없음 |
| 07-23 | `1509` perception+Transformer 첫 성공/후반 붕괴, `2210` peak-resume continuation | 없음 |
| 07-24 | `0110` altitude-PI source, `0209` general representation, `1052` tilt compensation, `1230` 12 m look-ahead | `0108` — 완료 epoch/checkpoint 0 |
| 07-27 | `0225` first stable fresh representation, `0930` FOV-240, `1309` Stage-C 65 bars, `2324` Stage-C 75 bars 장기 진단/source checkpoint | `0048`(62ep NaN), `0054`(60ep), `0058`(120ep), `0106`(120ep), `0147`(43ep NaN), `1204`(335ep collapse-guard 오발), `2323`(NavRL epoch/checkpoint 0) |
| 07-28 | `1817` corrected squashed-Gaussian main — 정리 시 PID `3089290`으로 계속 학습 중 | `1813` — 69ep unscaled-mean negative smoke; raw `edge99_y=98.75%` 증거는 아카이브에 보존 |

정리 후 filesystem run directory는 38→28개, TensorBoard API의 표시 세션은 **36→26개**로
즉시 줄었다. TensorBoard data server가 reload를 정상 반영해 서비스 재시작은 필요 없었다.
정리 직후에도 main PID `3089290`은 정상 실행 중이며 active event/checkpoint 생성에 영향이 없었다.

### 2026-07-28 18:49 — squashed-Gaussian main 500-epoch 종료

TensorBoard 정리 직후까지 정상이던 `ppo_260728_1817_navrl_action-squashed-main-s1`은
epoch `19550`까지 정확히 500 epoch를 완료한 뒤, 다음 PPO update(epoch 19551)의
`ppo/a_loss[7]`에서 non-finite가 검출되어 fail-fast 종료됐다. optimizer output은 버려졌고,
업데이트 전 마지막 finite checkpoint
`last_gen_ppo_ep_19550_rew_31.084248.pth`가 저장되어 있다.

- 마지막 epoch: capture `70.7%`, crash `29.3%`, timeout `0%`, reward `31.08`.
- run peak: capture `83.3%@19106`, reward `48.89@19417`.
- task 입력 OOB는 끝까지 0이었지만, 마지막 세 diagnostic window의 lateral
  `exec_edge98_y`가 `22.15%→26.05%→29.91%`, `mean_abs_y`가 `0.951→0.955→0.958`로
  상승했다. 즉 tanh가 범위 밖 sample/clamp mismatch는 제거했으나 actor mean이 다시 경계로
  이동해 `±2.5 m/s` 근처 사용률이 장기적으로 재상승했다.
- 이번 결과는 `mu_scale=0.4`가 warm-start 순간의 포화는 풀어도 지속적인 제약은 아니라는 증거다.
  non-finite의 직접 원인은 아직 `a_loss[7]`까지만 확정됐으며, 경계 접근에 따른 transformed
  likelihood/PPO ratio 수치 문제인지 별도 재현·tensor audit이 필요하다.

따라서 이 run을 그대로 재개하지 않는다. 다음 main 수정은 lateral latent-mean margin/regularization과
log-probability 수치 안정성을 함께 다룬 뒤, 동일 epoch-19050 checkpoint에서 짧은 재현 실험으로
검증해야 한다.

### 2026-07-28 18:56 — GTX 1650 Ti truncated-Gaussian 전송 방식 정리

sub 실험의 체크포인트는 launcher에 적힌 기본 상대경로와 동일한 위치에 둘 필요가 없다.
`CKPT=/absolute/path/to/last_gen_ppo_ep_19050_rew_31.79068.pth`로 전달하면 runner가 해당
파일을 읽고 필요한 central-value key normalization 사본을 옆에 생성한다. 코드 커밋
`7a10948`은 이미 원격 `research/navrl-env`에 포함되어 있어, 다른 컴퓨터가 같은 저장소를
사용하면 `git fetch/pull`만으로 설치할 수 있다.

네트워크 없이 옮기는 경우를 위해
`/home/fair/workspaces/aerial_gym_ws/transfers/navrl_truncated_1650ti/`에 다음을 준비했다.

- `0001-navrl-add-bounded-action-policies-for-squashed-trunc.patch`
- `last_gen_ppo_ep_19050_rew_31.79068.pth` (8.5 MiB)
- `README_KO.txt`

원본 체크포인트 SHA-256은
`b3d67792f65b71fa3939630d2b182e1b28155564a285b3feaa12db651bc68277`이다. 1650 Ti에서는
main과 batch 조건을 맞추기 위해 우선 `NUM_ENVS=128`을 사용하고, 실제 CUDA OOM이 발생할
때만 launcher가 지원하는 `NUM_ENVS=64`로 낮춘다.

### 2026-07-28 19:04 — upstream `main` 선별 병합

사용자 요청에 따라 `main..research/navrl-env` 전체를 감사했다. 저장소 정책대로 `main`은
upstream Aerial Gym을 보존하고 연구 코드는 연구 브랜치에 두는 구조이며, 연구 브랜치 전체
병합에는 upstream 문서·sim2real·DCE 예제 등 100개 파일 삭제가 포함된다. 따라서 전체 병합은
하지 않고 별도 clean worktree에서 독립적이고 회귀 위험이 낮은 ignore 규칙만 선별 적용했다.

- `b609eee`: 로컬 experiment workspace와 생성된 example media ignore.
- `64ba00a`: `train_session_logs/`, `checkpoints_saved/` ignore. 연구 브랜치 원본은
  `39d472f`; main에는 연구 전용 `*.log` 규칙을 끌어오지 않고 두 디렉터리만 적용했다.
- `main`의 `origin/main` 대비 최종 diff는 `.gitignore` 한 파일, 12줄 추가뿐이며
  `git diff --check`를 통과했다. 원본 upstream 파일 삭제·수정은 0건이다.

다음은 의도적으로 main에서 제외했다.

- `7a10948` bounded-action A/B: squashed main은 epoch 19551에서 non-finite 종료됐고,
  truncated sub는 아직 별도 GPU 검증 전이므로 연구 브랜치에 유지.
- `e3ba49e`, `542895e` 및 기타 WORKLOG/status/result: 연구 provenance 전용.
- NavRL task, representation, density curriculum, controller opt-in 변경: 연구 스택에 대한
  의존성이 크므로 부분 cherry-pick 시 불완전한 기능이 된다.
- `7847994`의 upstream 자료 삭제: main 보존 목적과 정면으로 충돌.

병합은 로컬에서만 수행했으며 자동 push하지 않았다.

### 2026-07-28 19:35 — `main`·연구 브랜치 원격 동기화

다른 컴퓨터와 Git 자료구조를 일치시키려는 사용자 의도에 따라 두 브랜치를 push했다. push 전
`git fetch --prune`에서 `origin/main`에 새 커밋 `317505b`(사용하지 않는 MkDocs gh-pages
workflow 삭제)가 있음을 발견했다. 이를 덮어쓰지 않고 로컬 main과 일반 merge하여 다음 구조로
보존했다.

- `79f0679`: `origin/main`의 `317505b`와 로컬 ignore 커밋 `b609eee`, `64ba00a`의 merge.
- main의 원격 대비 실질 diff는 `.gitignore` 12줄 추가뿐이며, 원격 workflow 삭제도 유지.
- `6fcef96`: research branch의 transfer, TensorBoard 정리, main 선별 병합 기록까지의 tip.

첫 동기화 시도에서 별도 worktree 생성과 rebase를 같은 shell command에 넣어 현재 디렉터리가
자동으로 새 worktree로 바뀐다고 잘못 판단해, push 전 로컬 research ref가 일시적으로 재작성됐다.
원래 tip `6fcef96`과 재작성 tip의 tree diff가 0임을 확인하고 compare-and-swap `git update-ref`로
즉시 `6fcef96`을 복구했다. 원격 push나 파일 손실은 없었다. 이후 rebase를 사용하지 않고 실제
main worktree에서 `origin/main`을 merge했다.

두 브랜치는 한쪽만 갱신되는 상황을 막기 위해 다음 atomic push로 함께 반영했다.

```text
317505b..79f0679  main -> main
7a10948..6fcef96  research/navrl-env -> research/navrl-env
```

체크포인트, `runs/`, `train_session_logs/`, `checkpoints_saved/`와 workspace의 transfer archive는
의도적으로 Git에 포함하지 않는다. 다른 컴퓨터에서는 Git 코드 동기화 후 ep19050 체크포인트만
별도로 복사해야 truncated-Gaussian 실험의 데이터까지 완전히 일치한다.

### 2026-07-28 19:49 — RTX 3070 병렬 실험: squashed-v2 안정화

1650 Ti의 truncated-Gaussian 실험이 진행되는 동안 RTX 3070을 유휴 상태로 두지 않기 위해,
실패한 squashed run을 그대로 반복하지 않고 측정된 실패 원인만 다루는 v2 pilot을 추가했다.
시작점, seed, 75 bars, action std와 lateral warm-start scale은 모두 ep19050 A/B와 동일하다.

#### 변경

- `ppo_update_safety.py`: PPO가 ordinary ratio clip을 적용하기 전에
  `exp(old_neglogp-new_neglogp)`에서 overflow하는 것을 막는 opt-in log-ratio clamp.
- `early_stop_a2c_agent.py`: single-GPU에서 analytic KL이 문턱을 넘은 후속 minibatch를
  optimizer step 없이 건너뛰는 gate와 `ppo/kl_skipped_minibatches` 기록.
- lateral latent mean의 `|mu_y| > margin` 초과분에 대한 soft squared penalty. 속도 상한을
  낮추는 hard cap이 아니며, noisy action은 여전히 2.5 m/s 경계까지 사용할 수 있다.
- `runner.py`: actor learning-rate 환경변수 override. asymmetric critic LR은 기존 1e-4 유지.
- `train_navrl_action_squashed_v2_main.sh`: fixed 75 bars, seed 1, std
  `0.35,0.35,0.05,0.08`, log-ratio `±10`, KL stop `0.04`, lateral margin
  `1.25@0.01`, 500-epoch pilot.
- 새 safety knob와 actor LR을 checkpoint `env_state`에 기록해 provenance를 남긴다.

#### 실제 restore/update smoke

ep19050 checkpoint를 실제 Isaac Gym 128-env run으로 restore하고 각각 1 epoch optimizer update를
수행했다. 단위 테스트만 통과시키고 장기 학습에 넘기지 않았다.

| actor LR | epoch 19051 PPO KL | actor loss | raw OOB y | edge99 y | 판정 |
|---:|---:|---:|---:|---:|---|
| `3e-5` | `0.18701` | `0.02327` | `0` | `0.00537` | 너무 큼, 기각 |
| `5e-6` | `0.005215` | `-0.007895` | `0` | `0.00537` | target 0.016 아래, 채택 |

따라서 처음 추정한 `3e-5`를 그대로 장기 실행하지 않고, 실측으로 검증된 `5e-6`을 launcher
기본값으로 확정했다. 첫 epoch의 capture 15/32=46.9%는 종료 episode가 32개뿐이라 성능
판정에는 사용하지 않는다. 검증은 action-model/safety unit test 9개, Python compile,
shell syntax, checkpoint preflight, 실제 두 번의 restore+optimizer smoke를 통과했다.

RTX 3070에서는 우선 500 epoch만 실행한다. 약 25~35분 뒤 KL, capture, `edge95_y/edge99_y`,
`mean_mu_abs_y`, KL-skip 수를 판정하고, 정상일 때만 같은 run의 last checkpoint에서
ep22050까지 연장한다. 이렇게 하면 1650 Ti가 5~7시간 도는 동안 main GPU도 독립적인 원인
검증을 수행하면서, 잘못된 설정에 수 시간을 쓰는 위험은 제한한다.

### 2026-07-28 20:08 — 타겟 운동: 학습은 random mixed, 사이트만 단축 표현

사용자가 status 사이트에서 TARGET이 한 축으로만 왕복하는 것을 발견해 실제 학습 프로세스와
코드를 대조했다. 실행 중인 `action-squashed-v2-main-s1`의 process environment와 로그는
`NAVRL_TARGET_PATTERN=mixed`, `speed_final=1.5`, fixed speed override 없음이며, target speed
평균도 epoch별 약 0.7~0.86 m/s로 기록되고 있다. 학습을 중단할 문제는 아니다.

실제 `mixed`는 reset episode마다 cv/waypoint를 50:50으로 선택한다. cv는 방향각을
`U[0, 2π)`로 뽑아 벽에서 반사하고, waypoint는 arena XY에서 uniform waypoint를 뽑아 도달 시
재샘플한다. 속도도 episode마다 `U[0, 1.5]`다. 즉 매 simulation step마다 방향이 튀는 random
walk는 아니지만, episode/waypoint 단위로 2D 방향과 궤적이 무작위화된다.

반면 `docs/status/arena.js`는 `tx=goalX`로 x를 고정하고 `ty=sin(...)`만 갱신하는 장식용
1축 왕복 애니메이션이다. status JSON의 `mixed` 설명과 실제 task를 시각적으로 재현하지 않아
오해를 만든다. 사이트 표현 결함이며 현재 학습 데이터/환경에는 영향을 주지 않는다.

### 2026-07-28 20:21 — status Arena 전면 parity 감사 및 수정

사용자 요청에 따라 3D Arena를 실제 실행 중인 general-training recipe와 대조했다. 학습
프로세스 PID 3290841은 중단하거나 수정하지 않았으며, 감사 중에도
`action-squashed-v2-main-s1`이 75 bars, `mixed`, target speed
`U(0, 1.5)` 조건으로 계속 진행되는 것을 확인했다.

#### 확인된 표현 결함

- 타깃은 실제 2D `mixed`가 아니라 x 고정+y 사인파였다.
- 실제 `NAVRL_GENERAL_TRAIN=1`은 pursuer/target 시작점을 arena 전체에서 무작위화하지만
  사이트는 왼쪽→오른쪽 고정 cross-field episode였다.
- speed 슬라이더 값은 물리 속도가 아니라 임의 애니메이션 주파수에 사용되고 있었다.
- `detector 240°` 표기는 camera detector 87°와 obstacle-token selection FOV 240°를
  혼동했다.
- LiDAR 4개 층은 실제 elevation `-10°..+20°`가 아니라 수평 원 4개였고, ray 충돌은
  실제 box 막대를 원으로 근사했다.
- Arena 초기 bars=25가 status의 current/latest run bars=75와 자동 동기화되지 않았다.
- 3D pursuer가 실제 PPO telemetry인 것처럼 오해될 여지가 있었지만 브라우저 status에는
  policy state/action trajectory가 전달되지 않는다.

#### 수정

- `arena_motion.js`를 추가해 학습 코드와 같은 general spawn, target 거리 4–16 m,
  episode speed `U(0, max)`, CV/waypoint 50:50, CV 양축 반사, waypoint 0.5 m 도달
  재샘플, wall clamp, 모든 위반 bar의 composite 1.0 m push-out을 독립 구현했다.
- episode마다 bar layout, pursuer/target spawn, CV heading/waypoint를 다시 샘플링한다.
  target-speed UI는 `target max m/s`로 바꾸고 실제 학습 ceiling 1.5를 기본값으로 했다.
- HUD에 실제 선택된 `mixed → cv|waypoint`와 episode sampled speed를 표시한다.
- LiDAR는 72×4, 12 m, elevation `-10/0/10/20°`와 3D axis-aligned box ray intersection을
  사용한다. camera 표기는 87° @20 m, token selection은 별도 240°로 분리했다.
- status에서 current/latest run bars를 렌더러 성공 여부와 무관하게 slider/HUD에 먼저
  적용한다. 캐시 버전을 갱신하고 motion module을 Arena보다 먼저 로드한다.
- 실제 PPO trajectory가 없는 한 pursuer는 설명용 steering임을 Arena 아래에 명시해,
  training-distribution replay와 live telemetry를 구분했다.

`tests/test_status_arena_motion.js`는 400 episode에서 두 pattern과 양축 heading이 모두
샘플되는지, 4–16 m spawn 거리, 속도 상한, 양축 wall reflection, physical speed 적분,
bar clearance를 검사한다. 또한 launcher/task/env/LiDAR config와 사이트 상수·표기가
어긋나면 실패하는 source-contract 검사를 포함한다. Node parity test, JS syntax,
`git diff --check`를 통과했고, headless Chrome software WebGL에서 75 bars,
`mixed → cv`, sampled speed, 3D target motion과 HUD가 렌더링되는 것을 확인했다.

별도 감사 결과 `docs/status/status.json`은 정적 snapshot이라 현재 19:58 run이 아니라
완료된 18:17 run을 `LAST`로 보여 준다. 페이지가 이를 `LIVE`라고 표시하지는 않아 데이터
거짓 표시는 아니지만, Git commit 사이 실시간 PPO telemetry는 제공하지 않는다. 이번 수정은
그 한계를 숨기지 않고 Arena에도 명시했다.

### 2026-07-28 20:31 — squashed-v2 500-epoch pilot 종료 판정

RTX 3070의 `ppo_260728_1958_navrl_action-squashed-v2-main-s1`은 epoch
19051→19550의 계획된 500 epoch를 모두 수행하고 `max_epochs`로 정상 종료했다. GPU는
비었고 `.aerial_training_finished`는 `epoch=19550`을 기록한다. canonical final checkpoint
`last_gen_ppo_ep_19550_rew_33.75813.pth`를 CPU로 실제 load해 epoch, model/optimizer,
asymmetric critic, `env_state`를 확인했다. SHA-256은
`a0a3aa65f2378580b480f09e43fe6f1fd5bd8ec5bd77db78a846fc8c47fda9ea`다.

종료 summary 뒤의 `*** Can't create empty tensor` 한 줄은 이미 final checkpoint, summary
JSON, CSV, finished marker가 모두 기록된 뒤 shutdown에서 출력됐다. 종료 원인이 아니며
checkpoint load에도 문제가 없다. 다만 cleanup noise로 별도 추적할 수 있다.

같은 source checkpoint, seed, 75 bars와 정확히 같은 500-epoch 구간인 squashed-v1과
TensorBoard scalar를 비교했다.

| metric | v1 tail 50 | v2 tail 50 | 변화 |
|---|---:|---:|---:|
| capture | 67.50% | 71.54% | +4.04pp |
| crash | 31.72% | 28.07% | -3.65pp |
| reward | 32.82 | 35.58 | +2.76 |
| lateral edge95 | 75.36% | 29.26% | -46.10pp |
| lateral edge99 | 8.32% | 0.358% | -7.96pp |
| mean `|mu_y|` | 2.096 | 1.623 | -22.6% |
| mean action `|Δy|` | 0.0330 | 0.0703 | +113% |

전체 500 epoch 평균 capture는 v1 69.85%, v2 70.09%로 거의 같지만, v1은 후반으로
갈수록 포획이 67%대로 하락하고 경계 집중이 커진 반면 v2는 마지막 100 epoch capture
70.67%, crash 28.94%를 유지했다. v1은 최종 직후 `nonfinite_ppo`, v2는 정상 종료했다.
따라서 원래 가설인 “횡축 표본을 ±2.5 명령 근처에 몰리지 않게 한다”는 pilot 수준에서
성공했고, 후반 성능도 악화시키지 않았다.

단, v2 내부 100-epoch chunk의 edge95가 24.99→25.41→26.23→27.24→29.17%로
천천히 다시 증가하고 `|mu_y|`도 1.570→1.622로 증가했다. 완전히 수렴했다고 보기는
이르다. `ppo/kl_skipped_minibatches=0`이라 KL gate는 한 번도 개입하지 않았고,
표시된 KL의 작은 음수(~-0.0016)는 `rl_games.policy_kl`의 epsilon bias로 사실상 0이다.
이번 개선은 주로 5e-6 LR, lateral latent margin, finite log-ratio의 조합에서 왔으며
각 요소의 단독 기여는 아직 분리되지 않았다.

다음 순서는 장기 연장이 아니라 deterministic A/B 평가다.

1. source ep19050, squashed-v1 ep19550, squashed-v2 ep19550을 동일한 75 bars,
   target speed 0/0.5/1.0/1.5, `mixed`, pursuer 2.5 조건에서 비교한다.
2. 먼저 1000 games/cell quick screen, 승자와 source만 2500 games/cell로 확정한다.
3. 1650 Ti truncated-Gaussian 결과가 오면 같은 평가표에 세 번째 후보로 넣는다.
4. v2가 source 대비 capture/crash를 유지하면서 edge 지표 우위를 보일 때만
   ep20550까지 1000 epoch 연장한다. 연장 시 v2 checkpoint의 Adam state를 유지하도록
   `NAVRL_RESET_ACTOR_OPTIMIZER=0`을 반드시 지정한다.
5. 연장 중 edge95 tail이 40% 또는 edge99가 1%를 넘으면 장기화하지 않고 margin/LR
   ablation으로 돌아간다.

### 2026-07-28 21:10 — action A/B 평가 복구 및 squashed-v2 독립 검증

ep19050 source와 squashed-v2 ep19550에 대해 75 bars, `mixed`, pursuer 2.5 m/s,
target 0/0.5/1.0/1.5 m/s, 1000 games/cell quick screen을 실행했다. 최초 실행은 각
체크포인트의 네 셀을 실제로 끝냈지만 capture/crash 결과를 전혀 남기지 않아 성능 비교에
사용할 수 없었다.

원인은 `*** Can't create empty tensor`가 아니다. 이 문구는 종료 시 empty DOF tensor를
wrap하는 Isaac Gym 경고이며 프로세스 실패 원인이 아니었다. 실제 원인은 다음 두 설정의
조합이다.

- vector player의 기본 reward/length 통계는 `runner.py`에서 `print_stats=False`였다.
- task의 NavRL outcome 통계는 고정 2048-episode 주기에만 출력됐는데 평가 셀은 1000
  episodes였다.

따라서 계산은 정상 종료됐지만 outcome이 메모리에서 버려졌다. 이를 막기 위해
`NAVRL_BULK_EVAL=1` 모드를 추가했다. 이 모드에서는 `PLAY_GAMES_NUM`을 결과 집계 주기로
사용하고, player가 끝나기 전에 capture/crash/timeout, closest approach, action boundary
지표, crash 원인을 셀별 JSON으로 atomic 저장한다. 평가 launcher는 headless/num-env/python
경로를 고정하고, 셀별 로그와 CSV를 만들며, JSON이 없으면 exit 3으로 실패한다. 체크포인트
상대 경로는 launcher 호출 위치를 기준으로 해석하고, 실제 학습 분포와 맞도록 target
pattern 기본값을 `mixed`로 바꿨다. quiet 출력 필터에도 machine-readable
`NAVRL_BULK_EVAL_RESULT`를 허용했다.

32 envs × 64 episodes 실제 checkpoint smoke에서 JSON/CSV 저장과 fail guard를 확인한 뒤,
동일 seed 42로 기존 두 체크포인트의 8개 셀을 다시 실행했다. vector batch 종료 때문에 실제
표본 수는 셀별 1000–1005이며 JSON의 `actual_episodes`를 분모로 사용했다.

| target m/s | source capture | v2 capture | source crash | v2 crash |
|---:|---:|---:|---:|---:|
| 0.0 | 69.96% | 74.80% | 28.74% | 23.90% |
| 0.5 | 71.03% | 74.10% | 28.67% | 25.50% |
| 1.0 | 67.93% | 71.46% | 31.87% | 28.34% |
| 1.5 | 62.20% | 66.87% | 37.70% | 32.84% |
| 전체 가중 | 67.78% | 71.80% | 31.74% | 27.65% |

전체 차이는 capture `+4.02pp`(근사 95% CI `+2.01..+6.03pp`), crash
`-4.09pp`(95% CI `-6.09..-2.09pp`)다. bar contact는 28.17%→25.83%,
below는 1.90%→0.82%로 감소했다. 즉 v2의 학습 로그 개선은 독립 rollout에서도 재현됐고
source보다 명확히 낫다.

그러나 원래 action 문제는 부분 해결이다. source의 deterministic lateral action은 네 셀
평균 `edge98_y=99.99%`, 평균 `|a_y|=0.99994`로 사실상 항상 ±2.5 m/s 명령이었다. v2는
`edge98_y=0%`, raw OOB=0%로 exact boundary mass를 제거했지만 평균 `|a_y|=0.9217`
(약 2.30 m/s 명령), sign flip≈0, 평균 `|Δa_y|=0.0119`다. 따라서 “경계에 정확히 붙는
현상”은 해결했지만 한쪽의 큰 지속 횡명령이라는 구조적 bias는 남았다. v2를 장기 연장해
같은 형태를 더 굳히지 않는다.

다음 의사결정은 1650 Ti truncated-Gaussian 결과를 같은 corrected evaluator로 비교하는
것이다. 그것이 capture/crash를 유지하면서 평균 `|a_y|`도 의미 있게 낮추면 truncated
정책을 선택한다. 그렇지 않으면 main에서는 squashed-v2를 기준으로 더 강한 latent-mean
centering/margin ablation을 300–500 epoch만 수행하고, 동일 4-cell screen을 통과할 때만
장기 학습으로 확장한다.

검증: Python compile, shell syntax, `git diff --check`, training-safety 5 tests,
checkpoint-preflight 4 tests, 실제 Isaac Gym 64-episode export smoke, corrected
8-cell/약 8011-episode A/B 평가를 통과했다. 원시 JSON/CSV/log는
`train_session_logs/eval_results/action_ab_{base,v2}_260728_corrected/`에 보관한다.

### 2026-07-28 22:22 — lateral +y 고착 원인 분리와 v3 ablation 기각

1650 Ti truncated-Gaussian 실험을 기다리는 동안 RTX 3070에서 squashed-v2의 남은
`|a_y|≈0.92`가 고밀도 회피에 필요한 행동인지 정책 편향인지 분리했다. 장기학습을 먼저
돌리지 않고 vector eval에 signed y, positive/negative, high80, edge95/98/99와
front clear/blocked, target centered/off-center/visible 조건부 평균 `|a_y|`를 추가했다.

25/50/75/110 bars × target 0.5/1.5 m/s, 500 games/cell, 총 4007 episodes의
squashed-v2 프로파일 결과는 다음과 같다.

| bars | target | capture | signed y | positive y | high80 y | clear `|y|` | blocked `|y|` |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 25 | 0.5 | 91.60% | 0.933 | 100% | 99.99% | 0.934 | 0.931 |
| 25 | 1.5 | 87.03% | 0.932 | 100% | 99.98% | 0.934 | 0.931 |
| 50 | 0.5 | 88.25% | 0.929 | 100% | 99.94% | 0.931 | 0.928 |
| 50 | 1.5 | 81.27% | 0.929 | 100% | 99.93% | 0.931 | 0.928 |
| 75 | 0.5 | 73.40% | 0.921 | 100% | 99.65% | 0.926 | 0.921 |
| 75 | 1.5 | 65.47% | 0.922 | 100% | 99.65% | 0.926 | 0.922 |
| 110 | 0.5 | 38.32% | 0.906 | 100% | 98.64% | 0.921 | 0.905 |
| 110 | 1.5 | 32.60% | 0.907 | 100% | 98.82% | 0.924 | 0.906 |

가장 낮은 25 bars, 전방 clear, 타깃 centered에서도 `|a_y|≈0.93`이고 모든 셀에서
positive y=100%였다. 따라서 큰 횡명령은 고밀도 장애물 회피 요구가 아니라 policy가 한
passing side에 고착된 전역 편향이다.

#### Ablation A: signed minibatch-mean penalty — 기각

`mean(mu_y)^2`는 balanced `[-m,+m]` 회피에는 0이고 한쪽 batch에만 비용을 주므로 일반
action L1/L2보다 적합하다고 판단했다. v2 Adam state, LR 5e-6, margin 1.25@0.01을
유지했다.

- coef 0.002, 약 63 epochs: `mu_y` 1.635→1.604. 너무 느려 중단.
- coef 0.01, 50 epochs: `mu_y` 1.635→1.593, positive y tail 99.98%,
  tail capture 67.55%. 방향 분화 없이 모든 y를 조금 줄였고 성능도 v2보다 낮아 기각.

#### Ablation B: 좌우 reflection equivariance — 기각

898-D actor observation의 좌우 반사를 구현했다. LiDAR 방위 순서를 반전하고 obstacle
position/velocity y, robot velocity y/yaw-rate/previous y+yaw action, target tracked
position/velocity y를 부호 반전한다. action latent는 y와 yaw 부호를 반전하며,
`||mu(mirror(obs))-mirror(mu(obs))||²` auxiliary loss를 쓴다. mirror를 두 번 적용하면
observation/action이 bit-exact 원복되는 unit test를 포함했다.

- coef 0.01은 23 epochs에서도 positive y=100%라 약해서 중단.
- coef 0.1은 25 epochs에서 `mu_y` 1.635→1.493, edge95 31.4%→19.9%,
  tail capture 71.3%를 유지해 총 100 epochs까지 제한 확장했다.
- 100 epochs에서 `mu_y=1.243`, sampled signed y=0.810, edge95=7.1%로 magnitude는
  줄었지만 positive y=99.89%, negative y=0.01%로 방향 고착은 그대로였다.
  training tail capture는 62.78%, crash 36.26%로 악화됐다.

최종 ep19650을 75 bars, mixed, target 0/0.5/1.0/1.5, 500 games/cell로 독립 평가했다.

| metric | squashed-v2 ep19550 | reflect-v3 ep19650 | 판정 |
|---|---:|---:|---|
| capture | 71.80% | 60.64% | -11.16pp |
| crash | 27.65% | 39.36% | +11.71pp |
| mean `|a_y|` | 0.922 | 0.844 | 감소 |
| positive y | 100% | 99.999% | 고착 유지 |
| edge95 y | 약 7.5% | 0.61% | 감소 |
| edge98 y | 0% | 0% | 유지 |

결론은 명확하다. 현재 network/rollout에서 batch centering과 counterfactual reflection
auxiliary는 좌우 상태 의존성을 만들지 못하고 기존의 성공적인 +y passing strategy
크기만 줄여 capture를 훼손한다. 두 방식 모두 main 후보에서 기각한다. 재현 launcher는
`*_ablation.sh`로 이름을 바꾸고 `ALLOW_REJECTED_ABLATION=1` 없이는 실행을 거부하도록
가드했다.

현재 main winner는 계속 squashed-v2 ep19550이다. 다음 정책 결정은 1650 Ti의
truncated-Gaussian 결과다. 그것도 한쪽 고착이면 다음 설계는 auxiliary penalty가 아니라
환경 rollout 자체에서 paired mirror trajectories를 생성하거나 lateral head에 구조적
reflection-equivariance를 넣는 새 architecture가 필요하다.

검증: action-model CPU tests 13개, Python compile, shell syntax, diff check,
checkpoint preflight, 실제 optimizer smoke, 조건부 4007-episode profile,
최종 2002-episode deterministic screen. 원시 결과는
`train_session_logs/eval_results/{v2_action_context_260728,v3_reflect_c100_p100_260728}/`에
보관한다.

---

## 2026-07-28 — 1650 Ti truncated 환경 준비 완료 (학습 미시작)

수신 파일: `~/navrl_truncated_1650ti_transfer.tar.gz` (18:56, RustDesk).

배치:
- 코드: 이미 `7a10948` (`git merge-base --is-ancestor` OK). patch 재적용 불필요.
- 체크포인트 복사:
  `aerial_gym/rl_training/rl_games/runs/ppo_260727_2324_navrl/nn/last_gen_ppo_ep_19050_rew_31.79068.pth`
- SHA-256 일치: `b3d67792f65b71fa3939630d2b182e1b28155564a285b3feaa12db651bc68277`
- `test_navrl_action_models.py` 7/7 OK. GPU idle (~0.45 GiB / 4 GiB).
- launcher `train_navrl_action_truncated_1650ti.sh` 실행 가능 상태 (`READY_TO_LAUNCH`).

**학습은 아직 시작하지 않음.** 사용자 지시 시:
```bash
conda activate aerialgym
cd aerial_gym/rl_training/rl_games
nohup ./train_navrl_action_truncated_1650ti.sh \
  > train_session_logs/action_truncated_1650ti.out 2>&1 &
```
OOM 시에만 `NUM_ENVS=64`.

---

## 2026-07-28 — truncated 1650 mid-run 평가 (학습 유지, held-out 미실행)

학습 중 `ppo_260728_1929_navrl_action-truncated-1650ti-s1` 중간 점검. 4GB에서 학습이
~2.9 GiB를 점유 중이라 **held-out `play`는 GPU 충돌** — 학습을 죽이지 않고
`epoch_metrics.csv` + 로그로 평가. (SIGSTOP은 VRAM을 안 비움.)

### 상태
- epoch **19500 / 22050** (+450/3000 = 15%), fixed 75 bars, step ~6–8s, ETA ~5h.
- ckpt (나중 held-out용): `nn/last_gen_ppo_ep_19500_rew_34.33362.pth` (`gen_ppo.pth` 금지).

### 학습 중 capture/crash (PPO stochastic — held-out와 동일시 금지)

| window | capture | crash | mean reward |
|---|---|---|---|
| first 50 (warm) | 0.675 | 0.319 | 31.7 |
| since start (19051–19500) | 0.712 | 0.281 | 35.4 |
| last 200 | 0.736 | 0.257 | 38.2 |
| last 50 | 0.724 | 0.267 | 36.4 |

### 판정
건강: collapse 없음, warm 대비 capture↑ / crash↓. 마지막 200ep 소폭 출렁임은 노이즈 수준.
A/B 최종 판정은 22050 완주 + held-out 속도축 평가 필요. held-out를 지금 돌리려면 학습
일시 중단(프로세스 종료→평가→같은 last_gen에서 resume) 승인 필요.

---

## 2026-07-29 — 1650 Ti truncated 완주 및 held-out 평가

`ppo_260728_1929_navrl_action-truncated-1650ti-s1`이 epoch `22050/22050`에
`max_epochs`로 정상 종료됐다. 19050 checkpoint에서 시작해 fixed 75 bars, seed 1,
128 envs로 정확히 3000 epoch를 추가했으며 NaN/OOM/reward collapse는 없었다.

### 학습 rollout 요약

| window | capture | crash | timeout | mean reward |
|---|---:|---:|---:|---:|
| first 50 | 67.5% | 31.9% | 0.6% | 31.69 |
| 전체 3000 | 74.8% | 24.4% | 0.8% | 38.35 |
| last 500 | 76.6% | 22.7% | 0.7% | 41.06 |
| last 200 | 74.8% | 24.6% | 0.6% | 40.27 |
| last 50 | 75.5% | 24.0% | 0.5% | 40.76 |

- 최고 100-epoch rolling capture는 `79.5%`(window end epoch 21533), 최고 200-epoch
  rolling capture는 `79.1%`(end 21610)였다.
- 단일 epoch peak reward는 `55.48 @ 21559`; 단일 epoch capture peak는
  `93.0% @ 21121`이다. 단일 epoch 값은 표본 수가 작으므로 checkpoint 선택 근거로 쓰지 않는다.
- value 학습은 안정적이었다: explained variance first50 `0.694`, last200 `0.709`;
  PPO/c-value loss와 모든 기록값은 finite였다.

bounded-action contract는 끝까지 유지됐다. `policy_action/raw_oob_y=0`이라 실행 action과
PPO likelihood의 support 불일치는 제거됐다. 다만 actor mean이 다시 lateral boundary를
선택하면서 `edge99_y`가 first50 `4.0%`에서 last200 `22.2%`,
`mean_abs_y`가 `0.863→0.952`, `mean_mu_abs_y`가 `0.927→0.978`로 상승했다.
동시에 adjusted sigma는 `0.132→0.047`, `delta_y`는 `0.105→0.044`로 감소했다.
즉 무작위 큰 횡샘플 문제는 해결됐지만, 정책 평균 자체의 boundary-seeking은 남아 있다.

### 동일 조건 deterministic held-out

조건: 75 bars, target `cv 1.0 m/s`, pursuer limit `2.5 m/s`, seed 42,
128 envs, 2049 episodes(시작 checkpoint만 종료 집계 2051), 12 m/4x72 LiDAR,
8 tokens, FOV 240°, tilt compensation on.

| checkpoint | capture | crash | timeout | closest, no crash |
|---|---:|---:|---:|---:|
| start `19050` legacy | 67.7% | 31.7% | 0.6% | 0.45 m |
| truncated `21550` | **75.9%** | **22.9%** | 1.1% | 0.50 m |
| truncated `22050` final | 72.1% | 27.2% | 0.6% | 0.45 m |

- `21550`은 start 대비 capture `+8.2%p`, crash `-8.8%p`다. 독립 binomial
  근사 95% CI로 capture 차이는 약 `+5.5~+11.0%p`라 단순 평가 노이즈보다 크다.
- final `22050`도 start 대비 capture `+4.4%p`지만, `21550`보다 `-3.8%p`
  (근사 95% CI 약 `-6.5~-1.1%p`)라 후반 500 epoch에서 실제 held-out 퇴행이 있었다.
- 따라서 이 branch의 대표 checkpoint는 **final이 아니라 `21550`**이다. `gen_ppo.pth`는
  선택 기준이 불투명하므로 사용하지 않는다.
- 이 평가는 deterministic 한 축과 training stochastic rollout을 확인한 것이다.
  완전한 최종 A/B 판정은 3070의 squashed branch에 동일 seed/조건을 적용하고,
  필요하면 player deterministic=False 평가를 별도로 추가해야 한다.

### 3070 전달 항목

추천 checkpoint:

`aerial_gym/rl_training/rl_games/runs/ppo_260728_1929_navrl_action-truncated-1650ti-s1/nn/last_gen_ppo_ep_21550_rew_44.51707.pth`

SHA-256:

`60cea22b0345bb2922376b12d27dfc3d7852012b4b50b80aaada8404321d0992`

최종 checkpoint `22050` SHA-256:

`284b5b3008d251deed278ac5d24b916b1093c97103ae7d3a0b6fe0b1a39ea9c9`

checkpoint와 eval log는 `.gitignore` 대상이므로 이 커밋에는 결론과 식별자만 들어간다.
3070에는 이 커밋을 pull/cherry-pick한 뒤 추천 `.pth`를 별도 전송하고 SHA를 확인한다.
3070 squashed checkpoint도 위 held-out 조건으로 평가해 `capture/crash/timeout`을 직접 비교한다.

---

## 2026-07-29 — ★ +y 고착의 유력 원인 발견: 관측의 좌우(키랄리티) 결함 2건 (코드로 검증됨)

75-에이전트 전면 감사가 지목한 perception HIGH 결함 2건을 세션 한도로 검증이 끊긴 부분까지
**직접 코드로 재검증**했다. 둘 다 실재하며, Codex가 "unknown 전역 편향"으로 규정한 +y 고착
(positive y=100%, clear/blocked 무차별)의 **관측-레벨 원인 후보**다.

### 결함 1 — LiDAR 빈 방위각 테이블이 센서와 거울상 (perception 전 기간 존재)

- 센서: `warp_lidar.py:67-69` — `azimuth = hfov_max − Δ·j/(W−1)`, 즉 **빈 인덱스↑ = 방위각↓**
  (j=0 → +180°, j=71 → −175°).
- perception: `navrl_perception.py:234` — `_lidar_angles = linspace(−175°, +180°)`, **증가** 가정.
- 관계식: **가정 = 5° − 실제** = x축 거울 반사 + 5° 회전. 검증: 검출기의 픽셀↔방위는 렌더
  (`navrl_detector.py:135`)와 측정(`:307`)이 서로 일관된 올바른 규약 → **표적 채널은 정상,
  장애물 토큰 채널만 반전**.
- 결과: ① 오른쪽 막대가 왼쪽 토큰으로 발행(pos=[r·cosα, r·sinα], x≈보존·y≈부호반전)
  ② 카메라 융합(`:313-317`)이 실제 방위 α의 관측을 **거울 빈(5°−α)** 에 min-fuse → 전방
  장애물이 반대편에 고스트로 복제 ③ 표적-리턴 억제/재연관(`:296-305`, `:371-378`)도 거울 빈을
  조작. 정적 스캔 자체는 고정 순열이라 학습 가능(그래서 성능이 나왔음) — 문제는 **채널 간 모순**.

### 결함 2 — 카메라 far-plane 10 m가 12 m LiDAR에 팬텀 벽으로 융합

`camera_obstacle_max_range=10.0` 고정(`navrl_task_config.py:150`), no-hit 채움값이 그대로
`min(scan, 10.0)` 융합(`navrl_perception.py:311,317`) → `NAVRL_LIDAR_RANGE=12`에서 **빈 전방이
항상 10 m 벽으로 보임**(static 전방 상한 0.833, 토큰 유효성 10<11.94 통과 → 팬텀 토큰이 정면에
3~4슬롯 점유). LIDAR_RANGE≤10이던 과거엔 휴면, **12 m 피벗 이후 활성**.

### 증상과의 정합 (사전 관찰들이 전부 설명됨)

- **clear |a_y| ≈ blocked |a_y|** (0.934≈0.931, Codex 조건부 프로파일): 팬텀 벽 때문에 정책
  눈에는 전방이 **한 번도 clear로 보인 적이 없다**. 조건 무차별이 당연한 귀결.
- **positive y=100% 고착**: 좌우 정보가 채널 간 모순(토큰 y 반전 vs 표적/로봇 y 정상) + 전방
  고스트 대칭화 → 좌우 신호의 기대가치 소멸 → 항상-회피(팬텀 벽) × 한쪽 고정(tie-break) 이
  가장 안정한 국소최적.
- **hit_in_tokens 0.40~0.55 미스터리**: 토큰이 거울 위치라 GT 연관이 우연 일치 수준.
- **reflection equivariance 실패의 구조적 이유**: Codex의 mirror 연산자는 잘못된 각도 테이블
  위에 세워져 진짜 세계-거울이 아니고(반전+5° skew, 채널별 불일치), "mirror 2회=항등" 단위테스트
  는 임의의 involution이 통과하므로 물리 정합성을 검증하지 못했다. **오염된 관측은 대칭화로
  고칠 수 없다** — 출력(actor)을 강제하기 전에 입력(센서)을 고쳐야 한다.

### 사전등록 예측 (수정 후 25 bars 300–500 epoch fresh pilot에서 판정)

- P1: positive-y 고착 소멸(좌/우 사용률 모두 >10%)
- P2: barprobe `hit_token_given_fov` 0.556 → 0.8+
- P3: 전방 clear vs blocked의 |a_y| 차이가 유의미해짐
- P4: 25 bars capture ≥ 기존(0.90) — 단 1차 판정 기준은 P1-P3

예측이 맞으면 equivariant actor는 불필요. 틀리면 그때 깨끗한 관측 위에 구조적 equivariance를
얹는다(그 시점엔 잘 정의됨). **기존 전 결과는 "거울 토큰+팬텀 벽 관측 하의 성능"이므로 수정 후
재베이스라인 필수.** 아직 코드 수정은 하지 않음 — 사용자 승인 대기.

---

## 2026-07-29 — 키랄리티 결함 2건 수정 완료 (물리 프로브로 사전·사후 검증)

### 물리 판정 (tools/probe_lidar_bearing.py — 신규, 상시 회귀 가드)

실제 시뮬레이터에서 raw 스캔 1,126개 리턴을 두 각도 테이블로 역투영해 GT 막대와 대조:

| 테이블 | on-bar 일치율 | 평균 오차 |
|---|---:|---:|
| increasing (옛 perception 가정) | **13.9%** | 2.54 m |
| decreasing (warp 센서 실제) | **94.8%** | 0.47 m (≈표면-중심) |

수정 후 perception 라이브 테이블 = 94.8% 동일. **거울 버그 실재가 물리적으로 확정·해소됨.**

### 적용된 수정

1. `navrl_perception.lidar_bin_bearings()` 신설 — bin→방위 단일 진실 공급원(감소 규약, warp_lidar.py:67
   과 일치). `_lidar_angles`가 이를 사용 → 토큰 기하/카메라 융합/표적 억제/LiDAR 재연관 6개 사용처 일괄 교정.
2. `camera_no_return_to_lidar_range()` — 카메라 far-plane(10 m) 열을 no-return(=lidar_max)으로 리맵 후
   min-융합. **12 m LiDAR에서 전방 팬텀 벽 제거.** 구 8 m 레시피에서는 동작 불변(no-op) 확인.
3. `navrl_task.py` 뷰어 오버레이(36빔+증가 규약으로 이중 낡음)와 action-context front mask를 공유
   테이블로 교정.
4. `tests/test_navrl_perception.py`의 픽스처가 **옛 거울 규약을 박제**하고 있었음(정면 표적 리턴을
   bin 17에 주입 — 물리 규약에선 bin 18). "깨진 구현에서 통과하던 테스트"를 물리 규약으로 수정.
5. `tests/test_lidar_bearing_convention.py` 신설 — warp 공식을 독립 재계산해 테이블과 대조(어느 쪽이
   바뀌어도 시끄럽게 실패), far-plane 리맵 케이스(12 m 활성/8 m no-op) 고정. 36/72빔 모두 통과.

검증: 신규 테스트 2세트 + 기존 perception/bar_probe/training_safety 전부 OK, 3-epoch 학습 스모크
(obs 898) 정상, 물리 프로브 사후 94.8%.

### 함의 (재확인)

- 기존 전 정책은 "거울 토큰 + (12 m에선) 팬텀 벽" 관측으로 학습된 것. **수정 후 관측 의미가 바뀌므로
  warm-start 무효, fresh 재학습 필수.** 기존 결과 수치는 수정 전 조건의 기록으로만 유효.
- Codex의 reflection equivariance 실험(기각됨)은 잘못된 각도 테이블 위의 mirror 연산자를 사용했으므로,
  향후 재시도 시 반드시 `lidar_bin_bearings()` 기반으로 재구축해야 함.
- 다음: 사전등록 예측 P1–P4 판정용 25 bars fresh pilot (WORKLOG 2026-07-29 앞 항목 참조).

---

## 2026-07-29 — status 사이트에 논문식 Architecture 블록 다이어그램 추가

`docs/status/index.html`에 `#panel-arch` 섹션(테마 연동 인라인 SVG, 1250×620) + 상단 탭 "Design".
내용: **온보드(배포) 경로 실선** — 환경 → LiDAR 72×4@12m / RGB-D 87°(표적 20m·장애물 depth 40×24@10m)
→ 스캔 융합·토큰화(far-plane 리맵, 240° 선택, ±10° 억제, 물리 방위 테이블) + 표적 분할→3D 측정→칼만
추적(track 16-D) → 히스토리 0.5s×5 → **관측 898-D** → Transformer(17tok d64, σ clamp) → **행동 4-D**
→ 제어기(tilt 45°·tilt-comp·고도 PI·yaw 3.0) → **모터 추력 4** → 루프백. **학습 전용 레인 점선** —
GT 상태 → 보상·종료(0.5m swept)·특권 critic(898+8=906) → PPO → 가중치 업데이트. 두 레인 사이에
**정보 방화벽**(빨간 점선)과 "GT는 학습 신호로만, actor 관측 진입 금지" 명시. 모든 화살표에 텐서
치수 표기. 헤드리스 Chrome 스크린샷으로 배치·겹침 검수 후 게시.

---

## 2026-07-29 20:39 — 표적 속도 실현 결함 감사 및 대칭 local steering 적용

학습 전 사용자가 status 사이트의 표적이 둔하고 막대에서 잘 빠져나오지 못한다고 지적했다.
로그에는 `pattern=mixed`, `speed_fixed`/`speed_final`, `rl_dt=0.100s`만 남아 있어 명령 속도는
확인할 수 있었지만 **실제 변위 속도는 기록되지 않았다**. 중단된 Claude 작업에는 CV 표적이
막대 clearance push-out과 매 스텝 싸운다는 진단, 막대 접촉 시 진행 벡터를 반사하는 미커밋
수정, `tools/probe_target_motion.py` 초안이 남아 있었다.

probe를 drone 전체 step과 분리해 `_advance_target()`만 300회 호출하도록 고쳤다. 이렇게 해야
zero-action drone crash/reset에 따른 표적 teleport와 pattern 재샘플이 계측에 섞이지 않는다.
수정 전 `HEAD`를 별도 임시 worktree에서 동일 seed로 다시 측정한 결과:

| bars | pattern | 명령 | 실현 평균 | 명령 대비 | stall(<20%) | clearance<1m |
|---:|---|---:|---:|---:|---:|---:|
| 75 | CV | 1.50 | 0.78 m/s | 52% | 41.4% | 24.5% |
| 75 | waypoint | 1.50 | 1.40 m/s | 93% | 3.5% | 2.5% |
| 110 | CV | 1.50 | 0.38 m/s | 26% | 71.6% | 32.8% |
| 110 | waypoint | 1.50 | 1.29 m/s | 86% | 4.9% | 6.3% |

원인은 CV가 벽에서만 heading을 반사하고 막대에서는 위치만 밀어낸 것이었다. 막대를 향한 원래
heading이 유지되어 다음 스텝에 다시 진입했고, 고밀도에서는 nominal 1.5 m/s 표적이 사실상
주차됐다. waypoint도 막대에 닿은 뒤 waypoint를 재샘플했지만 현재 스텝 변위는 이미 손실됐다.
추가로 target spawn은 drone용 0.65 m clearance를 재사용해 이동 계약의 1.0 m보다 가까이
시작할 수 있었고, 일부 clearance 경로가 build-time 전체 slot을 참조해 활성 밀도 계약이
불명확했다.

### 적용

1. simulator-independent `target_motion.steer_target_step()`을 추가했다. 직접 heading이
   clear하면 그대로 쓰고, 막히면 `0, ±30, ±60, ±90, ±120, 180°` 후보 중 **가장 작은
   대칭 회전**으로 full-speed endpoint를 선택한다.
2. 정확히 대칭인 좌/우 후보는 에피소드마다 50:50으로 뽑은 `_tm_avoid_sign`으로 결정한다.
   고정된 한쪽 tie-break가 새 lateral chirality를 만들지 않게 했다.
3. CV는 선택된 회피 heading을 유지하고 waypoint는 waypoint 목표를 유지한 채 local steering으로
   막대를 돌아 나간다. 기존 composite push와 CV 반사는 후보가 전부 막힌 경우의 safety fallback으로
   남겼다.
4. spawn target clearance를 0.65→설정값 1.0 m로 맞추고, 이동/생성 모두
   `:n_bars_active`만 사용한다.
5. checkpoint `env_state`에 `cfg_target_motion_model=symmetric_local_steer_v1`을 기록한다.
   이전 checkpoint로 moving-target fine-tune/eval하면 legacy stall 환경과 계약이 달라졌다는
   경고를 출력한다.
6. `docs/status/arena_motion.js`에도 같은 후보각·대칭 tie-break·fallback 반사를 적용했다.
   사이트의 speed slider는 학습 분포와 같이 **최댓값**이며 episode speed는 `U[0,max]`라 평균은
   max의 절반이다. HUD의 sampled speed가 실제 해당 episode 속도다.
7. train dashboard/TensorBoard에 명령 평균과 별도로
   `navrl/target_speed_realized_mean_m_s`를 추가했다. 앞으로는 probe 없이도 로그에서
   명령-실현 속도 괴리를 즉시 확인할 수 있다.

### 사후 물리 probe

| bars | CV | waypoint | stall | overspeed | clearance<1m |
|---:|---:|---:|---:|---:|---:|
| 25 | 1.50 m/s (100%) | 1.50 (100%) | 0% | 0% | 0% |
| 75 | 1.50 (100%) | 1.50 (100%) | 0% | 0% | 0% |
| 110 | 1.50 (100%) | 1.50 (100%) | 0% | 0% | 0% |
| 130 | 1.50 (100%) | 1.50 (100%) | 0% | 0% | 0% |

사이트/interactive 기본값과 같은 110 bars·0.75 m/s도 두 패턴 모두 실현 0.75 m/s,
stall/과속/clearance 위반 0%를 별도로 확인했다.

150 bars는 알려진 기하 절벽(약 148 bars)을 넘는다. CV는 100%를 유지했지만 waypoint는 1.48
m/s, stall 0.9%, clearance 위반 7.2%였다. 이 밀도에서는 1.0 m exclusion disc가 arena를
사실상 덮으므로, 속도와 1.0 m clearance를 동시에 강제하는 것은 코드가 아니라 기하적으로
불가능하다. Stage C의 110 bars와 절벽 아래 130 bars까지는 계약을 정확히 만족한다.

검증: target-motion CPU tests 3개, train-dashboard realized-speed contract,
status Node parity, Python compile/diff check,
Phase-3 실제 Isaac Gym smoke(static identity + CV/waypoint/circle + curriculum) 전부 통과,
25/75/110/130/150 bars × 300-step 물리 probe. 이 변경은 moving-target 환경 자체를
바꾸므로 기존 moving-target 성능표와 직접 비교하지 않고 fresh 재학습·재평가해야 한다.

---

## 2026-07-29 — 센서/표적 수정 후 25-bars fresh pilot 실행 계약

기존 `train_navrl_general_repr.sh`는 `NAVRL_OBSTACLE_FOV_DEG`를 고정하지 않아 현재 평가 계약인
240°가 아니라 기본 360°로 조용히 실행될 수 있고, 기본 8000 epoch라 P1–P4 사전등록 검증보다
범위가 컸다. 전용 `train_navrl_sensorfix_fresh_pilot.sh`를 추가해 아래 조건을 고정했다.

- **fresh only**: `CKPT`, `--checkpoint`, `--resume_in_place`, `--branch_run` 거부. 수정 전
  거울 bearing/팬텀 벽 정책을 섞지 않는다.
- **환경**: 25 bars 고정, density promotion off, 8 tokens, 240° FOV, ±10° suppression,
  4×72 LiDAR, 12 m, mixed CV/waypoint, 목표 속도 ceiling 0→1.5 m/s(3000 epoch curriculum).
- **인과 분리**: baseline `legacy` actor를 사용하고 bounded-action 실험의 환경변수를 제거한다.
  action/crash/barprobe 진단은 켠다.
- **안전**: 중복 NavRL process/launcher lock을 검사하고 reward-collapse guard는 끄되
  NaN/Inf fail-fast는 유지한다.
- 기본 500 epoch, 128 env, seed 1. 이 결과로 P1–P4를 판정한 뒤에만 full fresh run의
  액션 정책과 density curriculum을 결정한다.

---

## 2026-07-29 — sensor-fix fresh pilot 완료 및 5-speed held-out 평가

`ppo_260729_2104_navrl_sensorfix-fresh-pilot-s1`이 계획한 epoch `500/500`에
`max_epochs`로 정상 종료했다. NaN/Inf, reward collapse, OOM은 없었다. 마지막의
`*** Can't create empty tensor`는 checkpoint·summary·finished marker가 기록된 뒤 Isaac Gym
shutdown에서 나오는 기존 cleanup 경고이며 종료 원인이 아니다. 평가 checkpoint는
`last_gen_ppo_ep_500_rew_38.911366.pth`이다.

### 학습 추세

| epoch window | capture 평균 | crash 평균 | reward 평균 |
|---|---:|---:|---:|
| 1–100 | 5.3% | 94.7% | -37.51 |
| 101–200 | 26.6% | 73.4% | -0.61 |
| 201–300 | 44.6% | 55.4% | 18.07 |
| 301–400 | 55.8% | 44.2% | 29.26 |
| 401–500 | 57.1% | 42.9% | 35.21 |
| 451–500 | 55.9% | 44.1% | 36.52 |

마지막 epoch capture는 59.7%, run 단일-epoch peak는 71.6%@429였다. PPO 자체는 안정적이다:
epoch 500 KL `0.00522`, explained variance `0.581`, LR `1e-4`, 모든 scalar finite.

표적 운동 수정도 실제 학습에서 검증됐다. epoch 100/200/300/400/500의 command 평균과
realized 평균이 소수점 출력 정밀도까지 각각 동일했고, epoch 500은 command=realized
`0.12676 m/s`(ceiling `0.24998`)였다. 따라서 현재 성능 저하를 표적 stall로 설명할 수 없다.

### 25-bars deterministic held-out (seed 42, 1000 games/cell)

조건: mixed CV/waypoint, pursuer limit 2.5 m/s, 240°/8-token/4×72/12 m,
실제 episode 합계 5,010개.

| target m/s | capture | crash | bar contact (절대) | below (절대) |
|---:|---:|---:|---:|---:|
| 0.0 | 64.9% | 35.1% | 20.5% | 13.7% |
| 0.25 | **67.2%** | **32.8%** | 18.7% | 13.5% |
| 0.5 | 65.3% | 34.7% | 23.1% | 11.2% |
| 1.0 | 61.0% | 39.0% | 23.2% | 14.6% |
| 1.5 | 55.2% | 44.8% | 28.1% | 14.9% |
| 가중 평균 | **62.7%** | **37.3%** | **22.7%** | **13.6%** |

raw 결과는
`train_session_logs/eval_results/speed_density_ppo_260729_2104_navrl_sensorfix-fresh-pilot-s1_260729_221344/`
에 보관한다.

### 사전등록 P1–P4 판정

1. **P1 부분 통과** — literal 기준인 좌/우 사용률 >10%는 만족했다. 하지만 고정 평가에서
   positive-y `31.3–37.0%`, negative-y `62.4–68.0%`로, 과거 +y 고착이 사라진 대신 약한
   -y 편향이 자랐다. “키랄리티 완전 소멸”로 해석하면 실패다.
2. **P2 강하게 통과** — 마지막 barprobe `hit_token_given_fov=0.981`로 목표 0.8+를 넘었다.
   sensor-bearing/far-plane 수정은 실제 simulation 관측에서도 효과가 확인됐다.
3. **P3 실패** — held-out 전 속도에서 front-clear와 front-blocked의 mean `|a_y|`가
   `0.959–0.969`로 사실상 동일했다(차이 <0.004). 정책이 장애물 문맥에 따라 횡축 크기를
   조절하지 않는다.
4. **P4 실패** — 최고 held-out capture 67.2%로 0.90에 미달한다. 수정 전 15k-epoch 정책과
   수정 후 500-epoch fresh 정책은 직접 동등 비교가 아니지만, pilot의 go 조건은 충족하지 못했다.

### 핵심 원인과 다음 결정

센서는 병목에서 빠졌고 새 병목은 **legacy Gaussian의 action saturation**이다. TensorBoard에서
lateral `edge99_y`가 epoch 100 `59.9%`→500 `90.3%`, raw mean `|a_y|`가
`1.44`→`2.71`, mean `|mu_y|`가 `1.22`→`2.68`로 악화됐다. 반면 sigma는
`1.00`→`0.91`에 불과해, 단순 exploration noise보다 actor mean 자체가 clamp 밖으로 달아난다.
held-out에서도 executed `edge98_y=91.7–93.7%`, clear/blocked 행동이 동일했다.

따라서 이 legacy run을 장기 연장하거나 density curriculum으로 넘기지 않는다. 이전
bounded-policy 결과는 깨진 센서 checkpoint에서 얻었으므로 성능 수치는 재사용하지 않되,
검증된 likelihood-correct 구현과 PPO safety 장치는 재사용할 수 있다. 다음 실험은 깨끗한
관측에서 **fresh bounded actor pilot**로 action support와 context response를 먼저 통과시킨 뒤
장기·밀도 학습으로 확장한다.

---

## 2026-07-29 — corrected-sensor fresh squashed pilot 실행기 확정

위 판정에 따라 `train_navrl_corrected_squashed_fresh_pilot.sh`를 추가했다. 기존
sensor-fix launcher의 환경 계약(25 bars, 240°/8-token/4×72/12 m, mixed moving target,
fresh-only)은 그대로 사용하고 action 쪽만 다음처럼 바꾼다.

- likelihood-correct tanh-squashed Gaussian, fixed std `0.35,0.35,0.05,0.08`
- lateral network-mean scale `0.4`, entropy coefficient `0`
- latent lateral margin `1.25@0.01`
- PPO log-ratio finite clamp `±10`, KL update-stop `0.04`
- fresh 학습률 `1e-4`; warm-start 분포 변경에서만 필요했던 `5e-6`은 사용하지 않음
- checkpoint/resume flag 거부, action raw-OOB/edge/mu/sigma 진단 유지

실제 Isaac Gym 128-env × 3-epoch optimizer smoke를 fresh weights로 수행했다. PPO actor/critic
loss와 모든 parameter가 finite였고, KL은 `-0.00068,-0.00234,-0.00234`로 epsilon bias 범위,
KL-skip 0이었다. lateral raw-OOB/edge95/edge99는 3 epoch 모두 0%, mean `|a_y|≈0.26`,
mean `|mu_y|=0.0004→0.0075`, positive/negative 사용률은 각각 약 37–39%/38–40%였다.
즉 시작부터 좌우 대칭이고 경계에 몰리지 않으며 fresh `1e-4` update도 안정적이다.

단위시험: action model 13개, training safety 5개 전부 통과. smoke run은 정식 TensorBoard
세션과 혼동되지 않도록 workspace 밖 임시 보관 후, 정식 500-epoch pilot만 실행 대상으로
남긴다. 500 epoch 뒤 go 조건은 raw-OOB=0, `edge99_y<5%`, 양 방향 >10%,
clear/blocked `|a_y|` 분리, capture가 계속 상승하는 것이다.

### 2026-07-29 22:27 — 정식 pilot 시작 직후 symbolic-shape 경고 확인

정식 run `ppo_260729_2225_navrl_corrected-squashed-fresh-pilot-s1` 시작 시
`torch/fx/experimental/symbolic_shapes.py`의 `not in var_ranges, defaulting to unknown
range` 경고가 여러 번 출력됐다. 이는 `torch.compile`이 첫 graph를 만들 때 일부 symbolic
dimension의 정적 범위를 증명하지 못해 unknown으로 처리한다는 compile-time 진단이다.
실제 PID `1160972`는 GPU 약 5.85 GiB를 사용하며 경고 이후 epoch 23까지 정상 진행했고,
Traceback/non-finite/OOM/illegal-memory 오류는 0건이었다. 학습 중단 사유가 아니므로 무시한다.

---

## 2026-07-29 22:32 — research status 사이트 최신화

`docs/status` snapshot을 현재 연구 상태와 live bounded pilot로 갱신했다.

- 새 **Research update** 패널: 물리 LiDAR match 94.8%, token hit|FOV 98.1%, target-motion
  25–130 bars 실현속도 100%/stall 0%, legacy 5-speed held-out 5,010 episodes 가중 capture
  62.7%와 edge98-y 91.7–93.7%를 한 화면에 배치했다.
- 현재 `ppo_260729_2225_navrl_corrected-squashed-fresh-pilot-s1`을 epoch 132 snapshot으로
  표시한다. tail50 capture 70.5%, crash 29.3%, raw OOB-y 0%, edge99-y 0%,
  +y/−y 43.9%/45.8%, KL 0.00471이다.
- Architecture 정책 블록을 낡은 learned-σ legacy 설명에서 실제
  tanh-squashed bounded action, fixed σ `[.35,.35,.05,.08]`, KL gate/latent margin/finite
  ratio 계약으로 교정했다.
- Now/phase를 density curriculum active에서 **P6A bounded action contract active**,
  P6B density 25→110 pending으로 수정했다. 장기·밀도 학습 gate도 raw OOB=0,
  edge99-y<5%, 좌우 양방향>10%, clear/blocked context 분리로 명시했다.
- `status.json`과 HTML fallback snapshot을 같은 데이터로 동기화하고 cache version을
  `20260729b`로 갱신했다. status parity test가 update schema와 bounded contract drift를
  감시하도록 확장했다.
- Chrome 실렌더 검수에서 live 카드가 직전 legacy run의 peak→final gap을 현재 run에 섞어
  보여주는 기존 결함을 발견했다. live일 때는 현재 epoch/500을 표시하도록 바꾸고, density
  promotion 설명도 fixed 25-bar bounded pilot 설명으로 교정했다.

## 2026-07-29 22:47 — 고밀도 Arena pursuer 정지 병목 수정

사용자가 status Arena에서 막대 수를 높였을 때 `PURSUER`가 중간에 정지하는 현상을
발견했다. 이 화면의 pursuer는 PPO checkpoint를 브라우저에서 실행하는 것이 아니라
`docs/status/arena.js`의 설명용 steering이므로, 이 현상만으로 실제 정책의 고밀도 성능을
판정할 수 없다. 실제 정식 run
`ppo_260729_2225_navrl_corrected-squashed-fresh-pilot-s1`도 현재 25 bars 고정 P6A
pilot이며, P6B 25→110 density curriculum은 아직 시작하지 않았다.

사이트 정지의 직접 원인은 목표 인력과 근거리 막대 반발력을 단순 합산하던 artificial
potential field였다. 막대가 조밀하거나 좌우 대칭이면 벡터가 상쇄되어 local minimum에
빠질 수 있었다. 이를 다음처럼 수정했다.

- pursuer 반경 0.25 m와 각 막대의 실제 폭을 포함한 swept-path clearance를 사용한다.
- 목표 방향 기준 `0, ±15, ±30, ±45, ±60, ±90, ±120, 180°` 후보를 0.9초
  look-ahead로 평가한다.
- 동일 점수에서 episode별 좌/우 회피 부호와 이전 heading을 유지해 프레임마다 회피 방향이
  뒤집히는 것을 막는다.
- 브라우저 장면은 계속 `illustrative pursuer`임을 명시하며, 실제 PPO 결과로 오인하지 않는다.

회귀 검증은 기존 대칭 막대 함정에서 5초 안에 목표 1 m 이내로 탈출하는지 검사한다. 추가
20 episode/density 동적 probe에서 25/110/130/150 bars 모두 capture `20/20`,
stationary step `0.00%`, 평균 이동속도 `2.50 m/s`였다. 150 bars의 look-ahead
blocked 표시는 `0.95%`였지만 정지는 없었다. 기존 target-motion/status parity test와
`git diff --check`도 통과했다.

## 2026-07-29 22:49 — corrected bounded 500-epoch pilot 완료 판정

`ppo_260729_2225_navrl_corrected-squashed-fresh-pilot-s1`이 epoch 500에서
`max_epochs`로 정상 종료됐다. traceback/non-finite/OOM 없이
`last_gen_ppo_ep_500_rew_102.53619.pth`와 summary/CSV가 저장됐고 GPU도 반환됐다.

학습 추세는 명확한 개선이다.

| epoch block | capture | crash | timeout | mean reward |
|---|---:|---:|---:|---:|
| 1–100 | 46.68% | 43.51% | 9.81% | -19.84 |
| 101–200 | 72.17% | 27.18% | 0.65% | 49.58 |
| 201–300 | 80.25% | 19.07% | 0.68% | 79.64 |
| 301–400 | 90.37% | 9.07% | 0.56% | 96.99 |
| 401–500 | 91.32% | 8.28% | 0.39% | 99.78 |

마지막 50 epoch는 capture `92.68%`, crash `7.07%`, timeout `0.25%`이고 마지막
epoch는 `92.2/7.8/0.0%`다. 사전등록 action gate도 training telemetry 기준으로
raw OOB 전 축 `0%`, lateral edge98 `0.23%`, positive/negative y
`36.5%/60.1%`라 support/boundary/two-sided 조건을 통과했다. legacy의 lateral
edge98 `91.7–93.7%` 문제는 제거됐다. clear/blocked `|a_y|`는
`0.689/0.662`로 legacy `0.934/0.931`보다 분리는 생겼지만 차이가 `0.027`로 작아
held-out에서 재확인한다.

중요한 제한은 이 pilot의 target-speed curriculum이 3000 epoch 기준이라는 점이다.
500 epoch 종료 시 speed ceiling은 약 `0.25 m/s`이고 실제 평균은 `0.12–0.13 m/s`였다.
따라서 training capture 92%를 1.5 m/s 성능이나 고밀도 성능으로 해석하지 않는다.

다음 gate는 density training이 아니라 **25 bars fixed, mixed target, held-out
0/0.25/0.5/1.0/1.5 m/s × 1000 games**다. final `last_*` checkpoint를 사용해 기존
legacy 5-speed 5,010-episode 결과와 직접 비교한다. 이 평가에서 full-speed 성능과
action context를 확인한 뒤에만 3000-epoch speed continuation 또는 P6B
25→110 density curriculum을 선택한다.

## 2026-07-29 23:00 — 5-speed held-out 통과 및 P6B 자동 전환

사용자 부재 중 후속 작업을 이어가기 위해 epoch-500 final checkpoint를 25 bars,
mixed target, pursuer 2.5 m/s, seed 42에서 `0/0.25/0.5/1.0/1.5 m/s × 1000 games`
평가했다. 128-env를 단순 `nohup`으로 분리한 시도는 이 도구 실행 환경이 자식
프로세스를 호출 종료 시 정리해 결과 JSON 없이 끝났다. checkpoint 이상으로 오판하지
않도록 16-env 10-game과 64-env 100-game smoke를 먼저 통과시킨 뒤, 관리형 64-env
세션으로 정식 5,004 episode를 완료했다.

| target m/s | bounded capture | legacy capture | Δ | crash | timeout |
|---:|---:|---:|---:|---:|---:|
| 0.00 | 96.50% | 64.87% | +31.63pp | 3.10% | 0.40% |
| 0.25 | 96.30% | 67.20% | +29.10pp | 3.70% | 0.00% |
| 0.50 | 95.60% | 65.27% | +30.33pp | 4.40% | 0.00% |
| 1.00 | 93.50% | 61.03% | +32.47pp | 6.50% | 0.00% |
| 1.50 | 79.92% | 55.20% | +24.72pp | 19.08% | 1.00% |

episode 가중 capture는 `4622/5004 = 92.37%`다. 모든 셀의 action raw OOB는 전 축
0%, lateral edge99는 0%였다. 속도가 높아질수록 capture는 내려가지만, 500-epoch
checkpoint가 실제로 학습한 speed ceiling 약 0.25 m/s를 크게 벗어난 1.5 m/s에서도
legacy보다 24.7pp 높다. 따라서 별도 25-bars speed-only continuation은 생략하고
P6B density curriculum으로 간다. speed curriculum은 checkpoint의 task step에서
계속 이어지지만 full-speed held-out competence가 이미 확인되어 과거의 검증되지 않은
two-axis 전환과는 다르다.

고밀도 실제 기체 정지를 놓치지 않기 위해 `NavRL motiondiag`를 추가했다. 목표가 1 m
이상 남은 step에서 실제 수평속도 평균, 명령속도 평균, 실제속도 `<0.2*vmax` 비율,
그중 명령도 `>=0.2*vmax`인 `commanded_stall` 비율을 집계한다. 이는 status 사이트의
illustrative pursuer가 아니라 Isaac Gym `robot_linvel`을 직접 측정한다.

재현 launcher `train_navrl_corrected_squashed_density.sh`는 final epoch-500 checkpoint,
bounded action/safety 계약, 25→110 bars(step 5), promotion gate 0.70/16,384 eps,
warmup 1000, total max epoch 8000을 고정한다. Python compile, diff check, training-safety
5개, checkpoint-preflight 4개, target-motion 13개 단위시험을 통과했다. action-model
stochastic tail test는 첫 실행에서 허용치 0.100% 대비 0.102%가 나왔으나, 코드 변경 없이
연속 3회(각 13개) 통과해 표본 경계 변동으로 판정했다. launcher preflight는 epoch 500,
bars 25, task_steps 16000과 bounded action 계약을 정확히 복원했다.

P6B run `ppo_260729_2303_navrl_corrected-squashed-density25to110-s1`을 관리형 백그라운드
세션으로 시작했다. runner PID는 `1226712`, GPU 사용량은 약 5.85 GiB다. epoch
501→535까지 정상 진행했고 첫 재개 구간 capture는 대체로 91–99%, raw OOB 전 축 0%,
lateral edge98 0.16%였다. 첫 2,049-episode 실제 기체 baseline은 평균 수평속도
`2.079 m/s`, 평균 명령속도 `2.607 m/s`, low-speed `6.33%`, commanded-stall
`6.29%`다. 이 수치는 25 bars의 reset/가속 transient를 포함한 기준선이며, 이후 밀도별
절대값과 이 기준선 대비 증가량을 함께 본다.

---

## 2026-07-30 — ★ 키랄리티 수정 사후 검증: P2 압도적 적중, "73.9%" 정정, 억제폭 병목 발견

### 사전등록 예측 판정 (2026-07-29 키랄리티 수정에 대해)

`ppo_260729_2303_navrl_corrected-squashed-density25to110-s1`(85막대까지 12회 승급) 로그로 판정.

- **P2 (hit_token_given_fov 0.556→0.8+) — 압도적 적중**: **0.968~0.975**. 토큰 위치도
  center_offset 0.28m·cross_track **0.18m**로 막대 표면 기하와 정확히 일치. 거울 버그가
  실재했고 해소됐다는 직접 증거.
- **P1 (좌/우 모두 >10%) — 간신히 통과**: pos_y 100%→**12.2%**, neg_y 0%→**83.6%**,
  sign_flip 0→**9.6%**. 절대 고착은 깨졌으나 이제 반대쪽으로 쏠림. 기준(>10%)은 만족.
- **P3 (clear/blocked 행동 분화) — 최초 "실패" 판정은 측정 결함이었음.** 아래 참조.

### 밀도 곡선 재측정 — 수정 전 대비 공통 6점 평균 **+11.1pp**

125-agent 감사 워크플로 이전에 직접 평가(`checkpoints_saved` 아님, 학습 중이던 85막대
정책, target 1.0 m/s, 2049 ep/셀, deterministic):

| bars | 수정 전 capture | 수정 후 | Δ |
|---|---|---|---|
| 25 | 0.896 | **0.978** | +8.2pp |
| 50 | 0.796 | **0.935** | +13.9pp |
| 65 | 0.679 | **0.854** | +17.5pp |
| 85 | (미측정) | **0.689** | — 학습 상한 |
| 110 | 0.280 | **0.412** | +13.2pp |
| 130 | 0.150 | **0.225** | +7.5pp |
| 150 | 0.079 | **0.144** | +6.5pp |

25막대 0.978은 이 프로젝트 역대 최고. `below`(고도이탈)는 전 밀도에서 0.1~6.7%로 사실상
소멸 — 고도 문제는 완전히 종결. 고밀도 실패는 96%가 `bar_contact`.

### 25-agent 감사가 잡은 정정 사항 (전부 재검증 완료)

1. **"85막대 73.9%"는 마지막 1 epoch(17/23, sd 5.8pp)이었다. 정정: 85막대 plateau =
   3042 epoch 평균 **0.676 ± 0.001**.** 비교 대상이던 "65막대 68.5%"도 동일하게 단일
   epoch 값이었고, 그 run의 65막대 plateau도 **0.678** — 통계적으로 동일.
   → **올바른 결론**: 동일 게이트(threshold 0.70, check_eps 16384)에서 키랄리티 수정이
   **같은 실력을 65→85막대(밀도 +31%, 13.6→17.8/100㎡)로 유지**시켰다. capture가 오른 게
   아니라 같은 capture로 더 어려운 조건을 버틴 것 — 해석은 다르지만 결론은 동일하게 긍정적.
2. **P3 "실패" 재해석**: `goal_centered` 판정이 `|lateral_sine|<=sin(15°)`만 보고
   `goal_vehicle_x`의 부호(전방/후방)를 안 봤다 — **전후 이중 원뿔**. 표적이 뒤에 있을 때
   큰 `|a_y|`는 정상 행동이라, `clear_centered` 버킷의 절반가량이 "표적이 뒤에 있는데
   전방이 뚫린" 무관 상황과 섞여 있었다. **P3는 "실패"가 아니라 "판정 불가"로 하향.**
   측정 코드 수정(`goal_vehicle[:,0]>0` 추가) 후 재판정 필요, 아직 미수정.
3. **85막대는 게이트에 막혀 있었다(내 최초 결론과 정반대).** `density curriculum held`가
   85막대에서 **12연속**, 판정창 capture 0.652~0.688로 문턱 0.70을 한 번도 못 넘음.
   → 처음 제안했던 "그대로 24000 epoch까지" 방침은 **틀렸음**: 이 추세로는 게이트를
   못 넘고 12시간을 태울 가능성이 높았음. 학습을 중단하고 원인 진단으로 전환.
4. **`ppo_update_safety.py:91`의 mirror 연산자가 아직 옛 증가 규약 기준** —
   `(H-2-i)%H`는 수정된 감소 규약에서 10°(HBEAMS=72) 비뚤어진 거울. 현재 run은
   `NAVRL_REFLECTION_COEF` unset이라 off-path이지만, reflection ablation을 재시도하면
   **정책에 키랄 왜곡을 주입**한다. 단위테스트(`test_navrl_action_models.py:243`)도 동일하게
   틀린 식을 기대값으로 써서 못 잡는다. `navrl_perception.lidar_bin_bearings()` 기준으로
   재구축 필요 — 미수정, reflection 계열 재시도 전 필수 선결 작업으로 기록.
5. 학습 로그에 action-model 계약(정책 종류·std·LR) 한 줄이 quiet 출력 필터에 걸려
   사라짐 — 어떤 행동 모델로 학습했는지 로그만으로 증명 불가. `quiet_rl_io._allow_print`에
   `[aerial RL]` 화이트리스트 필요 (LOW, 미수정).

### 85막대 게이트 차단의 원인 진단 — 억제폭 병목 (측정 기반, 추측 아님)

barprobe(85막대, 학습 로그 최신): **FOV 내 막대 35개, 토큰 8개 전부 사용, 그중 unique
3.0 / duplicate 2.7** — 실제로 표현되는 서로 다른 막대는 ~3개뿐(커버율 9%). 원인: 억제폭
±10°가 근거리 막대(폭 0.8m, 거리 2m → 각폭 ±11°)보다 좁아 **한 막대가 토큰 2개를 먹는다.**
문턱을 낮추지 않고 이 병목을 겨냥하는 것이 다음 수. 억제폭 ±10°→±15°는 (a) 관측 차원
불변(898) → warm-start 가능, (b) 최대 토큰 상한 360/30=12 > 8이라 부작용 없음, (c) 240°
FOV 좁히기 때와 동일 계열의 변경(그때 +2.7pp 성공 전례).

**결정 (사용자 승인 완료)**: 문턱은 유지, `NAVRL_OBSTACLE_SUPPRESS_DEG=15`로 85막대
체크포인트에서 이어서 학습 재개. 판정 기준: barprobe `unique`가 3.0→4.5+로 오르면 표현
수정이 유효, 이후 자연 승급을 기다림. ~3000 epoch 내 안 오르거나 올라도 capture가 안
따라오면 정보가 병목이 아니었다는 뜻이므로 중단 후 재판단(용량 확장 fresh 또는 85막대를
상한으로 확정).

---

## 2026-07-30 — 85막대 재개 런 발산 및 억제폭 실험 미적용 확인

실행 중인 `ppo_260730_1154_navrl_corrected-squashed-density25to110-s1`을 감사했다. 이
런은 `ppo_260730_1104.../last_gen_ppo_ep_8350_rew_22.905773.pth`에서 재개되어 epoch
11100을 넘겼지만, 최근 500 epoch 평균은 capture **1.0%**, crash **85.8%**, reward
**-104.3**으로 학습 가치가 없는 명백한 발산 상태다. 50-epoch 평균 capture가 20% 아래로
내려간 최초 창은 epoch 10314~10363이며, epoch 10400의 50-epoch 평균은 capture 0.5%,
crash 99.5%였다.

TensorBoard에서 직접 원인을 확인했다. `ppo/kl`은 epoch 10276에 설정 한계 0.04를 처음
넘었고 이후 0.30, 0.48, 최대 2.56 수준까지 폭증했다. 동시에 latent policy mean의
절대값이 x축 4.8→2025, z축 0.32→464, yaw축 0.57→15.3으로 폭발해 실제 action이
x/z/yaw 전부 경계에 고정됐다(`exec_edge98=1.0`). 최근 crash의 약 90%는 bar contact다.
이는 정지 병목이 아니다(actual speed 1.61m/s, commanded-stall 3.5%).

현재 KL 가드는 임계치를 넘은 minibatch를 skip할 뿐 이미 수행된 이전 update를 rollback하지
않는다. 발산 뒤에는 매 epoch 7~8개 minibatch가 skip되지만 손상된 정책은 복구되지 않는다.
또 density launcher는 정상적인 승급 reward 하락을 허용하려고 일반 reward-collapse guard를
완전히 비활성화한다. 두 안전장치 사이의 공백 때문에 실제 발산을 자동 종료하지 못했다.
후속 수정은 (1) 고정 density 안에서만 비교하는 curriculum-aware collapse guard,
(2) KL/latent 폭증 시 last-known-good epoch checkpoint rollback 또는 즉시 fail-stop이어야
한다.

더 중요한 실행 계약 오류도 확인했다. 사용한 명령의
`NAVRL_OBSTACLE_SUPPRESS_DEG=15`는 실제 적용되지 않았다.
`train_navrl_general_repr_density.sh`가 이를 무조건 `10`으로 다시 export하며, 시작 로그도
`suppress=+-10deg`를 명시한다. 따라서 이 런은 제안했던 ±15° 중복 토큰 억제 실험이 아니라
기존 ±10° 정책의 장기 연장이다. `CKPT=$(ls -t runs/*/nn/last_gen_ppo_ep_*.pth | head -1)`도
이제는 발산 런의 최신 checkpoint를 고를 수 있으므로 재사용 금지. 수정 전 재시작 기준점은
명시적 pre-collapse checkpoint `ppo_260730_1104.../last_gen_ppo_ep_8350_rew_22.905773.pth`로
고정한다.

### 후속 수정 — ±15° 실험 및 사용자 가시 로그

사용자가 학습은 직접 실행하기로 해 자동 실행하지 않았다. `general_repr_density`가 외부의
`NAVRL_OBSTACLE_SUPPRESS_DEG`를 덮어쓰지 않도록 기본값 방식으로 수정했다. 체크포인트
preflight는 원래 `cfg_obstacle_suppress_deg=10`과 요청값 15의 차이를 차단했으므로, 모든
다른 관측 계약 검사는 유지하면서 `cfg_obstacle_suppress_deg` 하나만 명시적으로 허용하는
`NAVRL_ALLOW_SUPPRESS_WARMSTART=1` opt-in을 추가했다. opt-in 없이 15를 주면 여전히
실패하고, opt-in 시에는 `INTENTIONAL contract override | ... checkpoint=10 requested=15`가
출력되는 것을 확인했다.

사용자 실수를 줄이기 위해 재현 런처
`train_navrl_corrected_squashed_density_suppress15.sh`를 추가했다. 안전 기준점 epoch 8350,
`MAX_EPOCHS=12000`, suppress ±15°, 별도 run tag/log 이름을 기본값으로 고정하며 학습을
시작하지 않는 preflight에서 epoch 8350·bars 85·tokens 8·FOV 240°·suppress ±15°를
확인했다.

학습 로그는 `train_navrl.sh`가 원래 터미널과 파일에 동시에 쓰는 `tee` 구조였지만, 과거
`nohup ... > /dev/null` 명령 때문에 화면이 사라지고 실제 파일명이 매번 달랐다. 이제 매
실행마다 `train_session_logs/current_training.log`가 실제 세션 로그를 가리키며,
`watch_navrl_training.sh`가 이를 `tail -F`한다. 포그라운드 실행에서는 같은 PPO dashboard가
현재 터미널에 바로 보이고, 다른 터미널에서는 watcher로 볼 수 있다. watcher 종료의
`Ctrl-C`는 tail만 종료하며 학습에는 영향을 주지 않는다.

---

## 2026-07-30 — 연구 문서·status 사이트를 suppress ±15° live gate로 통합

7월 29일 bounded 25-bars pilot에 머물러 있던 정적 사이트와 7월 22일 연구 계획을 현재
증거에 맞춰 전면 동기화했다.

- `RESEARCH_PLAN.md`, `ROADMAP.md`, `PHASE3_PLAN.md`,
  `PERCEPTION_TRANSFORMER_PLAN.md`, `README.md`에 corrected 85-bars plateau
  `0.676±0.001`, 이전 65-bars `0.678`, 같은 competence에서 밀도 `+31%`라는 정정된
  결론을 반영했다.
- ±10° 장기 재개 run `ppo_260730_1154...`는 epoch 10276 이후 KL/latent가 발산해
  tail500 capture/crash `1.0%/86.0%`가 된 PPO safety failure이며, ±15° 결과가 아니므로
  성능 비교에서 제외한다고 모든 계획 문서에 명시했다.
- 현재 critical path를 suppress ±15°의 세 gate로 고정했다: unique `≥4.5/8`, capture
  `≥0.70/16,384 episodes`, KL `≤0.04`. 통과 후 held-out density sweep, 실패 시
  threshold 완화가 아니라 token selection 재설계다. 장기 재시도 전 curriculum-aware
  collapse guard와 KL/latent fail-stop 또는 rollback을 선수정한다.
- `results/corrected_chirality_density_curve.csv`를 추가해 corrected policy의 held-out
  25/50/65/85/110/130/150-bars capture
  `0.978/0.935/0.854/0.689/0.412/0.225/0.144`를 재현 가능한 사이트 데이터로 남겼다.
- `tools/update_status_snapshot.py`가 run CSV를 다시 집계하고 `docs/status/status.json`과
  HTML inline fallback을 하나의 객체로 동시에 갱신하도록 했다. 검증 snapshot은 44 runs,
  live `ppo_260730_1419_navrl_corrected-squashed-density-suppress15-s1`, epoch 8639,
  tail50 capture/crash/timeout `69.1%/30.4%/0.5%`, bars 85였다. 첫 suppress15 barprobe는
  unique `3.5/8`, duplicate `1.8`로 기준 `3.0/~3.0`보다 방향은 맞지만 아직 gate 미달이다.
- status UI의 Research update, evidence ledger, decision gate, phase strip, architecture
  suppression 설명, Now 패널을 모두 이 live experiment로 교체했다. density 기본 그래프도
  corrected curve를 우선 선택한다.

검증: status JSON↔HTML fallback 완전 일치, Chrome 1440×1800 실제 렌더에서 Live/Research
update/evidence ledger 확인, status arena-motion parity 통과, checkpoint preflight 6개,
training-safety 5개, target-motion 테스트 통과, Python compile 및 `git diff --check` 통과.
사이트 snapshot 생성 중 실제 학습 프로세스는 중단하거나 변경하지 않았다.

---

## 2026-07-30 — density curriculum 설계 감사

사용자 요청에 따라 실행 중인 suppress ±15° 학습을 중단하지 않고, 커리큘럼 구현·체크포인트
상태·실제 승급 로그를 대조했다. 결론은 **밀도를 competence gate로 5개씩 올리는 큰 방향은
맞지만, 현재 gate는 학습 스케줄러로는 쓸 수 있어도 표현 ablation과 최종 성능 판정에는
그대로 쓰면 안 된다**는 것이다.

### 확인된 현재 계약

- 실행 중인 `ppo_260730_1419_navrl_corrected-squashed-density-suppress15-s1`은 85막대,
  목표 거리 상한 16 m, 표적 속도 상한 1.5 m/s, density threshold 0.70,
  `check_eps=16384`, step 5로 정상 실행 중이다.
- 거리 competence 상태는 시작 체크포인트부터 `k_min=10`, `k_max=16`으로 포화되어
  현재 density와 경쟁하지 않는다. 표적 속도 램프도 global epoch 3000에 끝나 현재는
  `speed ~ U[0,1.5]`로 고정되어 있다.
- 반면 25→65막대 초기 승급 때는 표적 속도 상한이 약 0.61→1.47 m/s로 동시에 증가했다.
  따라서 초기 승급 곡선의 하락을 density 효과 하나로 해석할 수는 없다. 65막대 이후에는
  속도 상한이 1.5 m/s로 포화되어 density 단일축 비교가 가능하다.
- `NAVRL_GENERAL_TRAIN=1`에서는 드론과 표적을 arena 전역에서 독립 표본화하고 실제 목표
  거리는 `[4 m, k_max]`로 뽑는다. 이 경로는 `k_min_cur=10`과
  `NAVRL_DENSITY_EASY_GOAL_MIX`를 사용하지 않는다. 즉 로그의 저장된 `[10,16]` 계약과 달리
  현재 gate는 실제로 거리 약 4~16 m, 속도 0~1.5 m/s, cv/waypoint 50:50이 섞인
  aggregate capture다.

### 체크포인트 재개의 판정창 오염

epoch 8350 체크포인트에는 suppress ±10°에서 수집한 density window가
`9823/15616`으로 저장되어 있었다. suppress ±15° run이 이 값을 그대로 복원해 첫 gate는
새 표현 에피소드가 768회만 추가된 즉시 0.630으로 판정됐다. 첫 판정 자료의 95.3%가 이전
표현에서 온 것이므로 ±15° 효과 판정으로 사용할 수 없다. 그 다음 창 16,385회는 전부 새
표현이며 0.664로 hold되어 유효하다. 감사가 끝날 때 세 번째 판정도 16,384회, 0.689로
hold되어 suppress ±15°의 clean window는 현재 `0.664, 0.689` 두 개다. 표현·reward·motion
contract가 바뀌면 curriculum position은 복원하되 진행 중인 competence accumulator와
adaptation warmup은 새로 시작해야 한다.

### 0.70 / 16,384 gate 해석

85막대 장기 plateau 0.676에서 이항 표준오차는 약 0.00366, 근사 95% 구간은
`[0.669, 0.683]`이다. 실제 성능이 0.676인 정책이 raw capture 0.70을 우연히 넘을 확률은
약 `2.6e-11`이다. 따라서 0.70은 noisy해서 막는 값이 아니라 **실제 +2.4%p 개선을 요구하는
엄격한 목표**다. 목표가 “85막대에서 최소 70%”이면 유지하는 게 맞고, 단순히 다음 난도를
경험시키는 훈련 스케줄러라면 너무 경직되어 있다. 또한 16k aggregate는 평균을 정밀하게
측정하지만 속도·거리·패턴별 실패를 숨기며, 계속 업데이트되는 정책의 과거 에피소드를
섞으므로 held-out 평가가 아니다.

### 권장 재설계

1. **표현 ablation과 density curriculum 분리**: suppress 변경은 85막대 고정,
   accumulator reset, 최소 adaptation hold 후 동일한 held-out grid로 비교한다.
2. **승급 gate 층화**: 전체 평균 `>=0.70`만 보지 말고 target speed
   `0/0.5/1.0/1.5`, motion `cv/waypoint`, 거리 구간별 capture를 함께 기록한다.
   weighted mean과 hard-bin floor를 동시에 사용해야 쉬운 짧은/느린 에피소드가 어려운
   조건의 실패를 가리지 않는다.
3. **승급 후 cooldown과 회귀 방지**: step 5는 85막대 기준 약 5.9% 증가라 적절하다.
   승급 뒤 일정 epoch 동안 재승급을 금지하고, 현재/이전 density를 일부 재표본화해
   catastrophic forgetting을 막는다.
4. **두 종류의 안전장치 분리**: density 증가로 reward가 내려가는 것은 허용하되, 같은
   density에서 capture 급락·KL/latent 폭증은 last-known-good 저장 또는 fail-stop으로
   막는다.
5. **최종 논문 판정은 별도 평가**: curriculum gate는 teacher 신호일 뿐 성능 근거가
   아니다. 고정 checkpoint를 3 seeds와 고정 density×speed×pattern grid로 평가한다.

문헌 방향도 대조했다. Self-Paced Deep RL은 정책 능력에 맞춰 task distribution 자체를
목표 분포 쪽으로 이동시키고, ALP-GMM과 Prioritized Level Replay는 단일 고정 성공문턱보다
학습 진전이 큰 task/level을 다시 표본화한다. 이 프로젝트에는 당장 복잡한 GMM teacher보다
먼저 **층화 gate + 이전 density replay + contract 변경 시 accumulator reset**을 넣는 것이
비용 대비 효과와 인과 해석 모두에서 우선이다.

---

## 2026-07-30 — 오염된 suppress15 런 종료 및 curriculum 계약 수정

사용자 승인 후 `ppo_260730_1419_navrl_corrected-squashed-density-suppress15-s1`을
SIGINT로 종료했다. 프로세스 그룹 전체가 내려간 것을 확인했고 마지막 정상 주기
checkpoint `last_gen_ppo_ep_8900_rew_22.966688.pth`를 보존했다. 이 run의 clean density
window는 `0.664`, `0.689`, unique는 `3.4~3.5/8`로 사전 기준 0.70/4.5에 미달했다.

다음 계약 오류와 안전 공백을 수정했다.

- `train_navrl_corrected_squashed_density_suppress15.sh`는 이제 85 bars 고정,
  `NAVRL_RESET_DENSITY_WINDOW=1`, 250-epoch resume adaptation을 강제한다. density
  monitor는 켜 두되 final=current라 promotion은 불가능하다.
- checkpoint의 표현/action/motion/task-distribution/promotion 설정이 달라지거나 새
  stratified counter provenance가 없으면 aggregate gate를 자동 폐기한다. 동일 계약
  재개만 aggregate와 speed/distance/pattern counter를 함께 복원한다.
- general-spawn 실제 목표거리 계약을 `NAVRL_GENERAL_GOAL_DIST_MIN/MAX=4/16m`로
  checkpoint와 시작 로그에 명시했다. legacy `k_min=10`을 실제 radial minimum처럼
  해석하지 않는다.
- 각 16,384-episode 창에 speed quartile 4개, initial-distance quartile 4개,
  cv/waypoint/circle pattern capture를 저장·출력한다. broad-slice floor enforcement는
  fixed baseline을 얻기 전까지 diagnostic-only다.
- density stage preflight는 `k_max>=16`, `task_steps>=96,000`을 요구한다. 따라서 거리
  competence나 3000-epoch target-speed ramp가 진행 중인 checkpoint에서 density를
  동시에 올리는 run을 시작할 수 없다.
- reward collapse guard를 density에서 끄더라도, 동일 bar count의 50-epoch rolling
  capture가 같은 density peak보다 25%p 내려간 상태가 25 epoch 지속되면 fail-stop한다.
  density가 바뀌면 기준을 초기화하므로 정상 promotion 하락은 오발하지 않는다.
- `current_training.log` 출력이 교체 전 symlink target을 표시하던 관측 오류도 수정했다.

검증은 Python 전체 test discovery 31개(새 density-capture guard 4개와 density-stage
preflight 2개 포함), action-model 13개, shell syntax, checkpoint preflight와
`git diff --check`를 통과했다. 실제 epoch 8350
checkpoint를 1 epoch만 resume한 smoke에서 `discarded=9823/15616`,
`gate_not_before_step=275200`, bars `85->85`, same-density guard 활성화를 확인했다.
smoke run/log/TensorBoard 이벤트는 휴지통으로 이동해 연구 run 목록을 오염시키지 않았고
`current_training.log`는 종료한 실제 suppress15 로그로 복원했다.

이미 두 개의 fully clean ±15° window와 안정 구간 unique가 기준 미달이므로 동일 설정을
장시간 다시 돌리는 것은 권장하지 않는다. fixed-85 suppress15 launcher는 계약 회귀와 짧은
재현용으로 남기고, 다음 GPU 예산은 angular/cluster-balanced token selection 후보에 쓴다.
검증 중 100k Monte-Carlo action tail 테스트가 경계에서 1-count 변동으로 간헐 실패하던
기존 flake도 확인해 RNG seed를 고정했으며, action 구현이나 판정 한계는 변경하지 않았다.

---

## 2026-07-30 — obstacle token 관리 문헌 조사와 현재 구조 재판정

사용자 요청에 따라 현재 obstacle representation 코드를 다시 읽고, NavRL++ 원문과
point-cloud/set-prediction/object-centric tokenization 연구를 대조했다.

### 코드에서 확정한 이중 압축

- `navrl_perception.py`는 4×72 scan을 vertical-min 72개 range로 만든 뒤, 가장 가까운
  bearing을 반복 선택하고 고정 각도 주변을 지우는 greedy suppression으로 8개 surface
  proposal을 만든다. proposal은 실제 막대 중심이나 instance가 아니라 LiDAR 표면점이다.
- `navrl_transformer_network.py`는 이 8개를 Transformer의 독립 토큰으로 넣지 않는다.
  각 history step의 `8×12=96` 값을 이어 붙여 MLP 하나로 64차원 한 토큰에 투영한다.
  따라서 5-step obstacle history가 Transformer token 5개를 차지한다.
- 즉 현재 “unique 3/8”은 첫 압축의 낭비를 보여주며, 그 뒤에도 여러 proposal 사이의
  관계가 한 MLP 벡터로 다시 압축된다. `MAX_OBSTACLES`를 늘리면 Transformer sequence가
  길어지는 것이 아니라 같은 MLP 입력만 넓어지고 checkpoint shape가 깨진다.

### 원 NavRL++와의 차이

[NavRL++](https://arxiv.org/html/2605.15559v1)은 static obstacle을 4×36 ego-centric
ray-distance array로 유지해 CNN 한 토큰으로 만들고, object slot 최대 5개는
position/velocity/radius가 있는 **dynamic obstacle** history에만 쓴다. 현재 구현은
정적인 arena bar 표면점을 velocity=0으로 채워 이 dynamic-history 자리에 넣었다.
원 논문에 없는 확장이므로, suppression 폭만의 문제가 아니라 static geometry를 object
history로 중복 표현한 설계 자체가 검증 대상이다.

### 유사 연구에서 쓰는 관리 방식

- [PointNet++](https://proceedings.neurips.cc/paper_files/paper/2017/hash/d8bf84be3800d12f74d8b05e9b89836f-Abstract.html)와
  [Point-BERT](https://openaccess.thecvf.com/content/CVPR2022/html/Yu_Point-BERT_Pre-Training_3D_Point_Cloud_Transformers_With_Masked_Point_Modeling_CVPR_2022_paper.html)는
  가까운 점만 순서대로 고르지 않고 공간적으로 퍼진 center를 sampling한 뒤 주변 point를
  local patch로 묶는다. 이 계열의 핵심은 대표점 수와 무관하게 공간 coverage를 먼저
  확보하는 것이다.
- [PointPillars](https://openaccess.thecvf.com/content_CVPR_2019/html/Lang_PointPillars_Fast_Encoders_for_Object_Detection_From_Point_Clouds_CVPR_2019_paper.html)는
  point를 고정 spatial pillar에 모아 pseudo-image로 되돌린다. 객체 수가 token budget보다
  많을 때도 특정 근거리 객체가 모든 slot을 점유하지 않으며, 현재 full polar scan CNN을
  유지해야 한다는 근거에 가깝다.
- [Set Transformer](https://proceedings.mlr.press/v97/lee19d.html)와
  [Perceiver](https://proceedings.mlr.press/v139/jaegle21a.html)는 많은 input을 소수의
  learned inducing/latent vector가 cross-attention으로 읽어 fixed bottleneck으로 만든다.
  hard top-k를 없앨 수 있지만, latent가 각각 물리적 객체 하나라는 보장은 없다.
- [Slot Attention](https://papers.nips.cc/paper/2020/hash/8511df98c02ab60aea1b2356c013bc0f-Abstract.html)은
  fixed slots이 입력 feature를 두고 반복적으로 경쟁하게 해 object-like set을 만든다.
  현재처럼 한 막대가 여러 slot을 먹는 문제와 가장 직접적으로 닮았지만, reconstruction이나
  supervised property loss 없이 PPO reward만 주면 slot specialization을 보장하기 어렵다.
- [DETR](https://arxiv.org/abs/2005.12872)은 learned object query와 Hungarian
  one-to-one matching loss로 duplicate prediction을 억제한다. 이 프로젝트에서도
  simulator bar identity를 actor 입력에 넣지 않고 training-only auxiliary label로 쓰는
  방법은 가능하다. 단, FOV 안 약 35개를 8 slot으로 모두 match할 수 없으므로 TTC와
  goal-corridor 기준 top-risk subset 정의가 먼저 필요하다.
- navigation 쪽의 [Omni-Perception](https://proceedings.mlr.press/v305/wang25b.html)은
  raw LiDAR를 proximal/distal risk로 나눠 서로 다른 sampling/temporal 처리를 하고,
  [SAGA](https://arxiv.org/abs/2605.02301)는 장애물 객체 대신 후보 motion anchor를
  geometry-aware token으로 만든다. 전자는 8 slot을 `near-risk + spatial coverage` quota로
  나누는 근거이고, 후자는 객체를 모두 열거할 수 없는 초고밀도 환경에서 action-corridor
  token으로 문제를 바꾸는 장기 후보이다.

### 결정한 실험 순서

1. 학습 없이 동일 scan에 current suppression, contiguous range clustering,
   risk-weighted angular/farthest selection을 적용해 unique/struck-bar/angular/
   goal-corridor coverage를 비교한다.
2. 첫 PPO 후보는 **cluster-balanced 8-slot**이다. 연속 beam을 surface cluster로 묶어
   cluster당 한 proposal만 만들고, 일부 slot은 TTC/closest-approach 위험, 나머지는
   아직 덮이지 않은 방위에 배정한다. 기존 12차원/8-slot 폭을 유지해 선택 규칙만 검증한다.
3. hard selector가 부족할 때 `72 bearings → 8 learned slots`와 training-only Hungarian
   auxiliary loss를 추가한다. 이때만 obstacle slots을 Transformer의 독립 token으로 넣는
   ablation을 병행한다.
4. 8→12 capacity 증가는 중복 선택을 고친 뒤 수행한다. 지금 늘리면 같은 막대 표면점을
   더 많이 담을 가능성이 높고 fresh policy가 필요해 인과 비교도 나빠진다.

이번 조사는 문헌·코드 감사와 계획 갱신만 수행했으며 학습 프로세스나 정책 코드는 변경하지 않았다.

---

## 2026-07-30 — token 재설계 전 학습 실행 여부 확인

사용자가 다음 학습 명령을 요청해 프로세스, checkpoint, launcher와 perception 구현을 다시
확인했다. 현재 NavRL 학습 프로세스는 없고 GPU 학습은 시작되지 않은 상태다. 최신 정상
baseline은 `ppo_260730_1104.../last_gen_ppo_ep_8350...`이며, 이후 suppress ±15° run의
epoch 8900 checkpoint는 clean window `0.664/0.689`, unique `3.4~3.5/8`로 실패 판정된
ablation 결과다.

문헌 조사에서 정한 `cluster-balanced` selector는 아직 `navrl_perception.py`에 구현되지
않았다. 현재 이용 가능한 `train_navrl_corrected_squashed_density_suppress15.sh`를 실행하면
새 token 설계가 아니라 실패한 fixed-85 ±15° 설정을 반복한다. 따라서 새 selector를 구현하고
학습 없는 offline coverage gate를 통과하기 전에는 장기 PPO 명령을 제공하거나 실행하지
않는 것으로 결정했다.

---

## 2026-07-30 — cluster-sector obstacle selector 구현

사용자 승인 후 기존 898-D observation과 8×12 obstacle proposal 폭을 유지하는
`cluster_sector` 선택기를 구현했다.

### 선택 로직

- 4×72 LiDAR의 vertical-min 72 range를 body-frame 2-D endpoint로 변환한다.
- 인접한 유효 endpoint 거리가 기본 0.45 m 이하이면 같은 surface cluster로 연결한다.
  range 차이만 쓰지 않아 가까운 원통 표면에서 생기는 정상적인 깊이 변화에 덜 민감하다.
- 240° token FOV를 기본 8 sector로 나누고 각 non-empty sector의 최근접 cluster 하나를
  먼저 예약한다. 이미 선택한 cluster는 다른 sector에서 다시 선택할 수 없다.
- 빈 sector 수만큼 아직 선택되지 않은 최근접 cluster를 보충한다. 최종 proposal은 기존
  flattened MLP가 warm-start하기 쉽도록 거리순으로 재정렬한다.
- 이전 `greedy_suppress`는 기본 모드로 그대로 남겨 회귀와 A/B가 가능하다. 두 selector는
  모두 `[8,12]`를 내므로 actor observation은 898-D로 동일하다.

### 계약·launcher·평가 가드

- 환경변수 `NAVRL_OBSTACLE_SELECTOR`, `NAVRL_OBSTACLE_CLUSTER_GAP_M`,
  `NAVRL_OBSTACLE_SECTORS`를 추가하고 시작 로그 및 checkpoint `env_state`에 저장한다.
- preflight는 legacy checkpoint의 selector를 `greedy_suppress`로 해석한다. 새 전용
  launcher만 selector/gap/sector의 same-shape override를 명시적으로 허용하며, 다른
  representation 계약은 계속 엄격하게 검사한다.
- `train_navrl_corrected_squashed_density_cluster_sector.sh`는 정상 epoch-8350 bounded
  checkpoint, 85 bars 고정, density accumulator reset, 250-epoch adaptation hold,
  gap 0.45 m, 8 sectors를 고정한다.
- `eval_navrl_cluster_sector_density_sweep.sh`를 추가하고 공통 density sweep도 selector
  계약을 출력하도록 수정했다. `play_navrl.sh`는 절대 Python 경로 사용 시 해당 conda
  `bin`을 PATH에 넣어 Isaac Gym의 `ninja` 탐색 실패를 막는다.

### 검증

- cluster 중복 제거, sector coverage, empty-sector fallback 단위 테스트 3개를 추가했다.
- 전체 Python discovery `37/37`, action-model `13/13`, Python compile, shell syntax,
  `git diff --check`가 통과했다.
- 실제 epoch-8350 checkpoint preflight는 legacy greedy→cluster-sector와 과거 checkpoint에
  없던 gap/sector provenance만 의도적 override로 기록했고, legacy greedy preflight도
  회귀 없이 통과했다.
- CPU perception observe와 RTX 3070 CUDA `[128,72]→[128,8]` selector smoke가 통과했다.
- 64 environments로 실제 PPO를 1 epoch 재개해 epoch 8351까지 forward/backward와 종료가
  정상임을 확인했다. 17 episodes의 capture 58.8%는 적응 전 극소표본이므로 성능 근거로
  사용하지 않는다. smoke run은 `/tmp/motar_cluster_smoke.zh4AlZ`로 이동했고
  `current_training.log`는 마지막 실제 suppress15 로그로 복원했다.
- bulk-play는 simulator/selector 초기화까지 성공했으나 기존 player가 episode 집계 전에
  `Can't create empty tensor`로 종료해 실제 barprobe offline 수치는 얻지 못했다. 따라서
  첫 clean training window에서 unique/duplicate와 capture를 함께 판정한다.

실행 명령:

```bash
cd /home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/aerial_gym/rl_training/rl_games
./train_navrl_corrected_squashed_density_cluster_sector.sh
```

정상 시작 로그는 `selector=cluster_sector`, `cluster_gap=0.45m`, `sectors=8`,
`fixed bars=85`, 세 개의 `INTENTIONAL contract override`, density evidence reset을
모두 포함해야 한다. 다른 터미널에서는 `./watch_navrl_training.sh`로 같은 로그를 본다.

---

## 2026-07-30 — 최근 UAV 인지 논문 5편의 저비용 적용성 감사

사용자가 제공한 최근 한 달 논문 목록을 원문과 대조하고, 현재 MOTAR/NavRL++-Target
구현에서 학습 방향을 바꾸지 않고 가져올 수 있는 부분을 검토했다. 정책·환경·launcher는
변경하지 않았다.

### 이미 구현되어 있어 새 구조가 필요 없는 부분

- fixed-wing tracking 논문의 `detector → target state estimator` 구조는 현재 RGB-D/LiDAR
  측정 뒤 3-D constant-velocity Kalman tracker로 이미 구현되어 있다. actor target
  feature도 상대 위치/속도, 위치·속도 covariance, camera/LiDAR confidence, track age를
  포함한다. 측정값이 이미 3-D이므로 비선형 image measurement를 직접 처리하는 해당
  논문의 UKF로 바꿀 근거는 없다.
- CosFly-VLA의 핵심인 5-frame history, 이전 action/state, visibility-aware memory도
  현재 5-step robot/target history와 5 s tracker memory에 대부분 대응한다. 검출이
  끊겨도 KF prediction은 유지되고 camera/LiDAR confidence는 0, track age/covariance는
  증가한다. 다만 CosFly-VLA 성능은 평가 때 이전 target state history를 GT로 제공한
  조건이므로 sensor-only MOTAR와 직접 수치 비교하지 않는다.
- CMRTrack이 주장하는 motion reliability에 해당하는 최소 정보는 이미 covariance,
  confidence, track age로 actor에 전달된다. CMRTrack 자체는 infrared image tracker이며
  ViT-B와 별도 detector dataset을 요구하므로 현재 PPO에 통째로 넣지 않는다.

### 방향을 유지하는 후보 순서

1. **설정·평가만으로 시작:** Phase 3C에서 기존
   `NAVRL_PERCEPTION_PERTURB`, `NAVRL_DETECTION_DROPOUT`,
   `NAVRL_RGB_NOISE_STD`, `NAVRL_DEPTH_NOISE_STD`를 사용해 가림/검출누락을 만들고,
   visible/occluded duration별 track survival, reacquisition time, capture/crash를
   층화한다. 현재 clean detector/tracker gate 전에는 perturbation을 켜지 않는다.
2. **작은 same-shape tracker 개선:** learned detector가 준비되면 Kalman normalized
   innovation 및 camera–LiDAR disagreement를 먼저 diagnostic으로 기록한다. false
   correction과 강하게 연관되면 Mahalanobis innovation gate 또는 innovation-derived
   reliability로 측정 update를 조절한다. observation 차원을 늘리지 않고 기존
   confidence/covariance 의미를 강화할 수 있다.
3. **검증 후에만 auxiliary risk:** CPA future-risk는 현재 PPO·asymmetric critic과
   구조적으로 잘 맞지만 원 논문의 주효과는 동적 장애물이다. 현재 arena 막대는 정적이라
   즉시 risk head를 추가하면 LiDAR urgency와 reward를 중복할 수 있다. 우선 GT
   relative velocity로 72-bearing CPA risk diagnostic을 계산해 collision 선행 예측력과
   정지 병목 상관을 확인하고, 이득이 있을 때만 기존 temporal encoder에 training-only
   auxiliary loss를 추가한다.

### 보류

- CosFly-VLA의 0.8B VLA, language prompt, 8-step waypoint chunk와 CoT 학습은 모델·데이터·
  action contract를 크게 바꾸며, 해당 논문도 predicted-state feedback 검증을 미래 과제로
  남겼으므로 채택하지 않는다.
- fixed-wing NMPC/CBF/BPNG는 기체·카메라·terminal-impact 목적이 다르다. quadrotor
  capture PPO를 대체하지 않는다.
- ASUMOT은 event camera 전용이고 코드·데이터가 “공개 예정” 상태다. sensor swap은
  현재 범위 밖이며, motion-consistency 원리는 향후 innovation reliability에만 반영한다.

결론적으로 현재 실행 순서는 바꾸지 않는다. 먼저 fixed-85 `cluster_sector` gate로
navigation backbone을 고정하고, Phase 3B detector 검증 뒤 occlusion/dropout 평가와
innovation reliability를 붙인다. CPA auxiliary head는 진단 로그가 효과를 입증할 때만
세 번째 후보로 진행한다.

---

## 2026-07-30 — cluster-sector 현황 사이트 반영 및 TensorBoard 2차 정리

사용자 요청에 따라 연구현황 사이트를 현재 실행 중인 fixed-85 cluster-sector 실험으로
갱신하고, checkpoint와 metrics CSV를 보존하면서 TensorBoard의 오래된 세션을 별도
archive로 이동했다.

### 사이트에 반영한 현재 결과

- 작업 시작 시 활성 run은
  `ppo_260730_1549_navrl_corrected-squashed-density-cluster-sector-s1`이었다. TensorBoard
  정리 뒤에도 계속 epoch를 기록했으나 이후 terminal error나 정상 종료 summary 없이
  epoch 10157에서 중단됐다. 마지막 주기 checkpoint 10150은 보존됐다.
- clean 16,384-episode competence window 네 개는 capture
  `0.742 / 0.745 / 0.722 / 0.714`로 모두 0.70 기준을 넘었다.
- barprobe의 unique associated bars는 기존 greedy/suppress 계열 `3.0~3.5/8`에서
  cluster-sector 약 `4.4/8`로 상승했고 duplicate는 `1.9`에서 약 `0.1`로 감소했다.
  이는 중복 surface가 slot을 소모하던 병목을 실제로 제거했다는 증거다.
- unique `4.5/8` 기준은 window에 따라 `4.2~4.6`으로 경계이므로 완전 통과로 과장하지
  않고 사이트에 `near threshold`로 표시했다.
- 네 개의 큰 clean window가 이미 같은 결론을 내므로 남은 epoch만 채우기 위해 재시작하지
  않고, checkpoint 10150의 density×target-speed held-out 평가로 넘어간다. 재현되면
  navigation backbone을 고정하고 Phase 3B learned detector로 돌아간다.
  후속 perception 후보는 occlusion/dropout 평가, Kalman innovation reliability 순이며
  CPA auxiliary risk는 diagnostic-first로 유지했다.

`tools/update_status_snapshot.py`, `docs/status/status.json`, HTML inline fallback과
cluster-sector 표시를 동기화했다. dashboard JSON과 fallback의 완전 일치, 활성 run/selector
계약, 브라우저 motion test를 검증했다.

### TensorBoard 정리

- 라이브 TensorBoard에서 2026-07-14~24의 historical 세션 21개와 2026-07-28의
  1~75 epoch action diagnostic 9개, 합계 **30개 summaries**를 제거했다.
- 여러 resume shard를 포함해 event file **37개**를
  `/home/fair/workspaces/aerial_gym_ws/tensorboard_archive/2026-07-30_historical_events/`
  아래로 이동했다. 삭제하지 않았으며 archive README에 전체 목록과 복구 방법을 기록했다.
- `runs/`의 checkpoint, `aerial_run/epoch_metrics.csv`, run summary 등은 전혀 이동하지
  않았다. 따라서 학습 재개 경로와 사이트의 45-run 연구 원자료는 유지된다.
- TensorBoard API에서 표시 세션이 **43→13개**로 즉시 줄어든 것을 확인했다.
  라이브에는 07-27 representation/density 계보, 500-epoch bounded-action 비교,
  07-29~30 corrected 계보와 현재 cluster-sector만 남겼다.
- summaries 이동 직후 TensorBoard API를 검사할 때까지 활성 runner와 event/checkpoint
  기록은 정상 계속됐다. 이후 epoch 10157에서 별도 종료 기록 없이 프로세스가 사라졌으며,
  archive 작업은 활성 run의 `summaries`나 다른 파일을 이동하지 않았다.

---

## 2026-07-30 — research 전체를 main에 합치기 전 감사

사용자가 GitHub의 `research/navrl-env`를 `main`에 합쳐도 되는지 질문해 실제 branch graph,
merge-base, 전체 tree diff와 merge-tree를 read-only로 점검했다. merge는 실행하지 않았다.

- `main...research/navrl-env`은 main-only 4개, research-only 138개 commit으로 갈라져 있어
  fast-forward가 아니다. merge-tree상 직접 text conflict는 `.gitignore` 1건이다.
- main-only 4개는 실제로 두 경로뿐이다. `317505b`는
  `.github/workflows/publish_site.yml`을 삭제했고, `b609eee`와 `64ba00a`는 `.gitignore`에
  local experiment/generated media와 session/checkpoint ignore를 추가했다. `79f0679`는 이
  두 갈래를 합친 merge commit이라 독립 기능 변경이 아니다. research tree에도 workflow
  삭제와 모든 ignore 규칙이 이미 내용상 반영돼 있고 `*.log` ignore만 하나 더 있으므로,
  main-only 네 commit 자체가 통합 위험인 것은 아니다.
- 그러나 전체 변경은 287 files, 약 `+28k/-85k`이며 NavRL/MOTAR 추가뿐 아니라 upstream
  DCE navigation example, sim2real code/weights, MkDocs 문서·이미지와 pretrained artifact의
  대량 삭제를 포함한다. 현재 “main은 upstream 기반, research는 MOTAR 연구”라는 branch
  역할을 유지한다면 전체 merge는 범위가 맞지 않는다.
- 더 중요하게 현재 working tree에는 cluster-sector selector, curriculum/PPO safety,
  preflight, launcher와 tests를 포함한 25개 미커밋 항목이 있다. 지금 merge하면 정작 최신
  핵심 구현은 main에 포함되지 않고, 오래된 research tip만 합쳐진다.
- 따라서 현 시점 결정은 **전체 merge 보류**다. 최신 same-shape selector와 safety 변경을
  먼저 하나의 검증된 research commit으로 만들고 clean worktree에서 main 통합 후보를
  선별한다.
- GitHub에서 최신 프로젝트를 기본으로 보이게 하는 목적이라면 upstream 보존용 main을
  합치기보다 default branch를 `research/navrl-env`로 바꾸는 방법이 더 안전하다. 반대로
  main 자체를 MOTAR 제품 branch로 전환하려는 경우에는 upstream 보존 branch를 먼저 만든
  뒤, 대량 삭제를 의도적으로 수락하는 별도 통합 작업으로 진행해야 한다.

---

## 2026-07-30 — density × target-speed 맵 (논문 헤드라인 그림) + 85→110 확장 학습 재개 + 사이트 Map 탭

체크포인트 `runs/ppo_260730_1549_navrl_corrected-squashed-density-cluster-sector-s1/nn/
last_gen_ppo_ep_10150_rew_36.50442.pth` (cluster_sector selector, epoch 10150)로 밀도(25/50/
65/85/110/130/150 bars) × 표적속도(0.0/0.5/1.0/1.5 m/s) 28셀 held-out 평가를 실행했다.
128 envs, 셀당 2049 episode, deterministic, general(랜덤) spawn, mixed target pattern.
결과: `results/density_speed_map_cluster_sector.csv`.

### 핵심 발견 — 두 축이 완전히 비대칭

| bars | density/100m² | capture @0.0 | @0.5 | @1.0 | @1.5 |
|---|---|---|---|---|---|
| 25  | 5.2  | 96.2% | 97.0% | 96.5% | 94.9% |
| 50  | 10.5 | 91.8% | 93.0% | 92.2% | 90.4% |
| 65  | 13.6 | 88.1% | 87.9% | 86.1% | 83.7% |
| 85  | 17.8 | 73.6% | 75.3% | 71.8% | 67.1% |
| **110** | **23.0** | 49.7% | 50.6% | 48.4% | 43.7% |
| 130 | 27.2 | 31.8% | 29.6% | 28.1% | 25.9% |
| 150 | 31.4 | 19.4% | 19.2% | 19.0% | 15.9% |

(85 bars = 학습된 최대 밀도, 굵게 표시한 110행부터 generalisation. 전체 표+crash 원인
breakdown은 CSV 참고.)

전체 그리드에서 밀도를 5.2→31.4 bars/100m²로 올리면 capture가 **78 pp** 떨어지는 반면,
표적속도를 0→1.5 m/s로 올리면 **4.2 pp**만 떨어진다. pursuer v_max=2.5 m/s가 표적보다
충분히 빨라서, 이 레짐에서 표적속도는 사실상 난이도 축이 아니고 밀도가 거의 전부다 —
이게 논문 헤드라인 그림이 된다. crash 원인은 밀도가 오를수록 거의 전부
`crash_bar_contact_share`(85 bars에서 90%+, 150 bars에서 94%+)로 수렴 — OOB/추락사는
부수적이고 장애물 접촉이 지배적 실패 모드임을 재확인.

### 85→110 밀도 커리큘럼 확장 재개

체크포인트 10150에서 재개하려던 중 `train_navrl_corrected_squashed_density_cluster_sector.sh`
(Codex가 이전에 작성한 launcher)에 다음이 하드코딩되어 있음을 발견:

```
export NAVRL_CONTROLLED_ABLATION=1
export NAVRL_FIXED_BARS="${NAVRL_FIXED_BARS:-85}"
```

`NAVRL_CONTROLLED_ABLATION=1`이 `_update_curriculum`의 밀도 승급 자체를 막아 85 bars에서
영구 고정시킨다 — 위 held-out 평가로 85 bars plateau가 이미 확인됐으므로 이 ablation은
목적을 다했고, 다음 단계(85→110+)로 넘어가려면 반드시 해제해야 한다. 아래처럼 override해서
재시작:

```
NAVRL_CONTROLLED_ABLATION=0 NAVRL_FIXED_BARS= \
  ./train_navrl.sh --checkpoint runs/ppo_260730_1549_navrl_corrected-squashed-density-cluster-sector-s1/nn/last_gen_ppo_ep_10150_rew_36.50442.pth \
  --branch_run --disable_collapse_early_stop --max_epochs 26000 --seed 1
```

20:15 KST 시작, run `ppo_260730_2015_navrl_cluster-sector-density85to110-s1`, 로그
`train_session_logs/cluster_sector_85to110_260730_201534.log`. 20:30 기준 epoch 10429/26000,
capture 75-80%대에서 85 bars 유지 중(아직 Layer-1 window 16384 episode 채우는 중, 승급
전). 예상 소요 11-15시간. `NAVRL_DENSITY_STEP=5` 그대로이므로 다음 승급 목표는 90 bars.

### 사이트 반영

- `docs/status/status.json`에 `density_speed_map`(28행 + notes + trained_max_bars=85) 키 추가.
- `docs/status/app.js`에 `renderHeatmap(s)` 신규 — 28셀을 red→tan→teal 그라디언트로 렌더링,
  85 bars(학습 범위) 아래는 옅은 배경 음영 + 점선으로 generalisation과 구분, 실측값 기반
  헤드라인 문장("Density dominates: ... 78 pp ... 4.2 pp ...")을 데이터에서 직접 계산해 표시.
- `docs/status/index.html`에 `Map` 탭 버튼 + `#pane-map` 추가, `style.css`에 `.hm-v`/`.hm-s`
  셀 텍스트 스타일 추가.
- headless Chrome 스크린샷으로 28셀 전부(25→150 bars × 4속도) 렌더 검증 완료 — 값, 색상,
  구분선, 캡션 텍스트 모두 CSV와 일치 확인.

---

## 2026-07-30 (밤) — 85→110 학습이 사실 85에 얼어있었음: launcher override-무시 버그 수정 후 재시작

### 발견 (버그)

run `ppo_260730_2015`(20:15 시작)가 85→110 확장 학습인 줄 알았으나, **승급 심사가 2회 모두
통과 기준을 넘기고도 "held"** 로 찍힘:

```
NavRL density curriculum held | bars=85 capture=0.759 over 16385 eps   (epoch 10658)
NavRL density curriculum held | bars=85 capture=0.767 over 16384 eps   (epoch 10921)
```

`/proc/<pid>/environ` 실측: `NAVRL_DENSITY_FINAL=85`, `NAVRL_FIXED_BARS=85`,
`NAVRL_CONTROLLED_ABLATION=1` — 승급 조건 `n_bars_active < final_bars`(85<85=False)가 영구
False. 원인은 `train_navrl_corrected_squashed_density_cluster_sector.sh:23-24`:

```bash
export NAVRL_CONTROLLED_ABLATION=1                 # 가드 없음 — override 무조건 덮어씀
export NAVRL_FIXED_BARS="${NAVRL_FIXED_BARS:-85}"  # ':-'는 빈 문자열도 85로 치환
```

launch 시 `NAVRL_CONTROLLED_ABLATION=0 NAVRL_FIXED_BARS=` override를 줬으나 둘 다 무력화됨.
결과: epoch 10151→11200(~1,050 epoch, 2.5시간)이 85 고정 ablation 재확인에 소모됨(손실은
아님 — capture 0.759/0.767로 85-bar 역량 재확증 데이터가 됐고 checkpoint도 개선 지속).

### 승급 주기 실측 (질문 답변용)

- 승급은 epoch가 아니라 **완료 에피소드 16,384개**(`NAVRL_DENSITY_CHECK_EPS`) 단위 심사.
- 실측 완료 에피소드 63.9개/epoch → **심사 1회 ≈ 263 epoch ≈ 14분**(step 2.7-2.9s).
- 재개 직후 첫 심사만 `NAVRL_DENSITY_RESUME_WARMUP=250` 때문에 ~507 epoch.
- 심사 시 capture ≥ 0.70 (+ strata gate, 현재 diagnostic-only) 그리고 bars < final이면 +5.

### 수정 및 재시작

- launcher를 `NAVRL_CONTROLLED_ABLATION="${NAVRL_CONTROLLED_ABLATION:-1}"` + FIXED_BARS는
  ablation=1일 때만 설정하도록 수정(기본 동작=기존 ablation 그대로, `=0` 전달 시 커리큘럼).
  echo도 모드 표시(`mode=density-curriculum->110` vs `mode=fixed-85bars-ablation`)로 변경.
- 기존 run kill 후 `last_gen_ppo_ep_11200_rew_23.425339.pth`에서 재시작 (21:11 KST):
  run `cluster-sector-density85to110-v2-s1`, 로그
  `train_session_logs/cluster_sector_85to110_v2_260730_211103.log`, MAX_EPOCHS=26000.
- **재시작 후 프로세스 env 검증 완료**: `CONTROLLED_ABLATION=0`, FIXED_BARS/NUM_BARS 없음,
  `DENSITY_FINAL=110`, restore 로그 `bars=25->85` (체크포인트 밀도 정상 복원), epoch 11201부터
  85 bars로 진행 중. 첫 승급 심사 예상 ≈ epoch 11710 (~25분 후), 통과 시 85→90.

---

## 부록 A — 삭제된 run 아카이브 (2026-07-14 정리 시점)

`runs/` 폴더가 삭제되어 재구성 불가한 26개 run의 최종 지표. 원본:
`aerial_gym/rl_training/rl_games/RUNS_ARCHIVE_SUMMARY.md` (이 부록으로 통합 후 삭제).

> 로드하지 않는다. 현재 observation/model schema는 각 run manifest로 구분한다.

`runs/`의 옛 run들을 삭제하기 전에 핵심 정보를 남김. 현재 학습 `ppo_260714_1904_navrl`은 유지.
지표: cap=captured_rate, crash=crash_rate, to=timeout_rate (최종 epoch). reward는 스케일이 리워드 설계에 따라 달라 절대비교 불가.

| run | type | ep | exit | last/peak reward | navrl 최종 | 무엇 |
|---|---|---|---|---|---|---|
| ppo_260530_1059 | intercept(옛과제) |  |  | / |  |  |
| ppo_260530_1112 | intercept(옛과제) |  |  | / |  |  |
| ppo_260530_1419 | intercept(옛과제) |  |  | / |  |  |
| ppo_260530_1604 | intercept(옛과제) |  |  | / |  |  |
| ppo_260530_1921 | intercept(옛과제) | 1261 | interrupted | 2408.2/2906.1 |  |  |
| ppo_260530_2005 | intercept(옛과제) | 6500 | interrupted | 922.4/1537.9 |  |  |
| ppo_260531_1031 | intercept(옛과제) | 15000 | max_epochs | 617.9/841.8 |  |  |
| ppo_260601_1152 | intercept(옛과제) | 931 | interrupted | 825.7/1149.7 |  |  |
| ppo_260601_1228 | intercept(옛과제) | 4652 | interrupted | 808.6/1076.9 |  |  |
| ppo_260601_1718 | intercept(옛과제) | 8787 | interrupted | 277.7/455.3 |  |  |
| ppo_260709_2146_navrl | navrl | 1500 | max_epochs | 142.5/170.3 |  | 첫 navrl 학습(1500ep, 옛 env). 목표도달 0%·충돌 다수 — 인프라만 완성. |
| ppo_260710_1230_navrl | navrl | 273 | interrupted | 126.5/159.8 |  | 초기 bars env 스모크/설정. |
| ppo_260710_1559_navrl | navrl |  |  | / |  | 초기 bars env 실험. |
| ppo_260710_1608_navrl | navrl | 1500 | max_epochs | 339.2/349.9 |  | bars env 첫 본학습(512env, MLP). |
| ppo_260710_1709_navrl | navrl |  |  | / | cap 0.00 / crash 0.37 / to 0.00 | bars env 실험. |
| ppo_260710_1738_navrl | navrl | 62 | interrupted | 229.7/236.4 | cap 0.00 / crash 0.42 / to 0.06 | bars env 실험. |
| ppo_260710_1745_navrl | navrl | 6000 | max_epochs | 365.2/375.9 | cap 0.00 / crash 0.02 / to 0.82 | bars env 실험. |
| ppo_260710_2106_navrl | navrl | 6000 | max_epochs | 296.9/327.5 | cap 0.01 / crash 0.12 / to 0.87 | 6000ep CNN 네비게이션 run(캡처종료 前, '목표찍고 배회'). |
| ppo_260713_1950_navrl | navrl | 6000 | max_epochs | 327.3/338.1 | cap 0.07 / crash 0.10 / to 0.84 | 캡처종료 첫 run — LOITER 실패(captured 6.7%, timeout 84%). 안전항 수입이 배회를 보상. |
| ppo_260713_2210_navrl | navrl | 6000 | max_epochs | 88.7/100.0 | cap 0.86 / crash 0.14 / to 0.00 | 리워드 재설계(B1 재베이스+PBRS progress+B3) 성공 — captured 86%(peak 94%), timeout 0%. loiter 해결. |
| ppo_260714_0153_navrl | navrl | 5049 | interrupted | 80.8/98.0 | cap 0.86 / crash 0.14 / to 0.00 | crash튜닝 A: safety_weight 1.5 — crash 13.7%(무효, safety만으론 안됨). |
| ppo_260714_0346_navrl | navrl | 6000 | max_epochs | 89.5/100.1 | cap 0.85 / crash 0.15 / to 0.00 | crash튜닝 B: clearance 거리 페널티 1.5 — crash 13.9%(무효). |
| ppo_260714_0555_navrl | navrl | 6000 | max_epochs | 85.6/100.2 | cap 0.87 / crash 0.13 / to 0.00 | crash튜닝 C: clearance speed-gated 1.5 — crash 13.8%(무효). 결론: 옛 스폰이 막대 무관(8%만 관통). |
| ppo_260714_1853_navrl | navrl | 32 | interrupted | 13.2/15.1 | cap 0.03 / crash 0.97 / to 0.00 | 짧게 중단된 run(설정 확인). |

---

## 2026-07-30 — navigation backbone 동결 게이트 (Codex 기록, 문서 통합 시 계획서에서 이관)

문서 통합(14→6개) 과정에서 `PERCEPTION_TRANSFORMER_PLAN`/`ROADMAP`/`PHASE3_PLAN`에 있던
2026-07-30 상태 기록을 여기로 옮긴다. 계획서는 charter만 담고 상태는 WORKLOG가 canonical.

**전제**: Phase 3 detector/occlusion 효과를 해석하려면 장애물 표현과 PPO update 자체의 실패를
먼저 분리해야 한다. 그래서 detector 본 검증 전에 navigation-only backbone의 고밀도 표현을 동결한다.

| 항목 | 근거 | 판정 |
|---|---|---|
| 센서 기하 | corrected bearing, token hit·FOV 약 97–98% | 통과 |
| bounded action | raw OOB 0%, corrected 500-epoch pilot + speed gate 통과 | 통과 |
| 밀도 확장 | 65 bars `0.678` → 85 bars `0.676±0.001` | 같은 실력으로 밀도 +31% |
| token coverage | 85 bars에서 unique 약 3/8, duplicate 약 2.7–3.0 | **병목** |
| ±10° 장기 재개 | epoch 10276 이후 KL/latent 폭발, tail500 capture 1.0% | **실패 · 결과 제외** |
| ±15° 1차 ablation | clean gate 0.664/0.689, unique 3.4–3.5, epoch 8900 중단 | 기준 미달 |

**token selection 후보의 통과 기준 (넷 다 만족해야 Phase 3의 고정 backbone이 된다)**
- barprobe `unique` 3.0 → **4.5+**, duplicate 감소
- 85 bars promotion window capture **≥0.70 / 16,384 episodes**
- PPO **KL ≤0.04**, latent mean/edge saturation 발산 없음
- 정상 checkpoint의 held-out density sweep 재현

**재시험 계약**: 85 bars 고정, gate accumulator 초기화, 250-epoch adaptation hold, density
promotion cap, speed/distance/pattern 층화 진단, same-density capture collapse fail-stop.
첫 ±15° 시도는 판정창에 이전 ±10° 에피소드가 섞이는 계약 오류가 있었다.

**결과 표에서 제외할 run**: `ppo_260730_1154...`의 tail500 capture 1.0%는 ±10° 장기 재개 중
PPO 발산 사례이며 ±15°의 결과가 아니다. safety ablation / engineering failure로만 기록한다.

**롤백 규칙**: 고정 density에서 KL 또는 latent mean이 지속 상승하면 더 학습하지 말고
last-known-good checkpoint로 rollback한 뒤 update safety를 수정한다.

---

## 2026-07-30 — 저장소 정리: runs/ 14.8GB 회수 + 문서 14→6개 통합

### 디스크 정리 (파괴적 작업 — 4단 검증 후 실행)

`runs/` **16.1GB → 1.27GB (14.8GB 회수)**. 50 epoch마다 저장되던 중간 체크포인트를 run별
최신 2개 + `gen_ppo*.pth`만 남기고 삭제. 근거: 2026-07-29 키랄리티 수정으로 **그 이전 체크포인트는
warm-start가 원천 무효**이고, 성능 수치는 `results/*.csv`와 WORKLOG에 이미 보존돼 있다.

삭제 전 4단 검증 전부 통과: ① 문서/런처가 참조하는 체크포인트 8개가 삭제목록에 없음 ② 실행 중인
run과 그 소스 체크포인트 제외 ③ `gen_ppo.pth` 0건 포함 ④ 모든 run이 최소 1개 체크포인트 유지.
6MB 초과 세션 로그는 gzip(진행 중 로그 제외). 학습 프로세스 무중단 확인.

### 문서 통합 (14 → 6개)

| 남긴 파일 | 통합된 원본 |
|---|---|
| `README.md` | (상태 블록을 실측값으로 재작성) |
| `RESEARCH_PLAN.md` | + `PERCEPTION_TRANSFORMER_PLAN` + `ROADMAP` + `PHASE3_PLAN` |
| `OPERATIONS.md` (신규) | `SETUP_SECOND_MACHINE` + `GPU_SCALING_GUIDE` + `TRANSFER_RESULTS_GUIDE` |
| `CRASH_TUNING_LOG.md` | + `rl_games/CRASH_TUNING_LOG.md`(B/C/D 음성결과를 부록으로) |
| `WORKLOG.md` | + `RUNS_ARCHIVE_SUMMARY`(부록 A) |
| `CLAUDE.md`, `SKILL.md` | 병합 안 함(에이전트 지시문) — 링크·관측차원만 수정 |

**세 가지 상충하던 위상 번호**(ROADMAP P0–P6 / PHASE3 "Phase 3" / PERCEPTION P0–P5)를
`RESEARCH_PLAN`의 **단일 P0–P7**로 통일. CLAUDE.md의 "관측 1265차원" 낡은 기술을
**898 actor / 906 critic + cluster_sector 240°**로 수정하고 폐기된 계보(156→305→1265)를 명시.

보존 확인: 업스트림 Aerial Gym 라이선스·인용 섹션 그대로, **코드 주석 3곳이 참조하는
`CRASH_TUNING_LOG.md` 경로 불변**(옮기면 주석이 깨지므로), 삭제 문서를 가리키던 링크 전부 재지정.

### ⚠ 이 작업 중 내가 낸 실수 (기록)

`RESEARCH_PLAN.md`를 통합본으로 덮어쓸 때 **Codex의 미커밋 상태 블록을 삭제했다.** 그 실질
내용(85 bars `0.676±0.001`, unique 약 3/8 병목, ±15° 1차 미달, ±10° 장기재개 PPO 발산,
token-selection 통과 기준 4개)은 같은 시각 `PERCEPTION`/`ROADMAP`/`PHASE3`의 미커밋 편집에도
동일하게 있었고, 그것을 삭제 전에 위 "navigation backbone 동결 게이트" 항목으로 이관해 보존했다.
교훈: **다른 세션의 미커밋 변경이 있는 파일은 덮어쓰기 전에 `git status`로 확인**한다.

Codex의 미커밋 코드 변경 21개 파일은 손대지 않았다(스테이징 제외).

---

## 2026-07-30 — GitHub 기본 브랜치 전환 시도

사용자 선택에 따라 기존 `main`을 삭제하거나 병합하지 않고, GitHub 저장소
`joshualikaist/MOTAR`의 기본 브랜치만 `research/navrl-env`로 바꾸는 방법을 선택했다.

- 전환 전 확인: 원격 `main`과 `research/navrl-env`가 모두 존재하고 로컬 추적 브랜치와 일치한다.
- 1차 미실행 사유: 이 환경에는 `gh` CLI, `GH_TOKEN`/`GITHUB_TOKEN`, GitHub API credential이
  없었고, 비공개 상태의 저장소는 GitHub 플러그인에서도 404로 조회가 거절됐다.
- 공개 전환 후 재시도: GitHub 플러그인으로 `visibility=public`, 사용자 권한 `admin=true`,
  `default_branch=main`을 확인했다. 다만 현재 플러그인이 저장소 기본 브랜치 변경 API를
  제공하지 않아 설정 변경은 실행하지 못했다.
- 최종 검증: `origin`의 HEAD는 여전히 `main`이다. 원격 브랜치 삭제·병합·강제 푸시는 수행하지 않았다.
- 남은 작업: GitHub 연결 후 기본 브랜치를 `research/navrl-env`로 변경하고,
  `git remote show origin`의 `HEAD branch`가 해당 브랜치인지 재확인한다.

---

## 2026-07-30 (밤) — 아레나 시뮬레이션 "표적 발작·둘 다 정지" 진단 및 수정 (14-에이전트 감사, 70만+ 프레임 실측)

사용자 보고: "pursuer는 똑똑한데 target은 혼자 멋대로 돌아다니고, 가끔 둘 다 가만히 있다.
막대 토큰 병목 때문인 것 같다." → 5렌즈 발견 + 8건 적대적 검증 워크플로우로 조사.

### 사용자 귀속(토큰 병목) 판정: 기각

브라우저 아레나에는 장애물 토큰 개념이 없다. `MAX_OBSTACLES=8`은 `navrl_perception.py:24`의
RL 관측 텐서 크기일 뿐이고, `arena.js`/`arena_motion.js`에는 `MAX_OBSTACLES`/`token`/
`cluster_sector` 문자열이 0회 등장 — 모션 모델은 매 프레임 전체 bars 배열을 직접 순회한다.
토큰 병목은 실제 env에서 **bar contact**(충돌)를 유발하는 문제이지 정지를 만들지 않는다.

### 근본 원인 (실측 확정)

1. **표적 발작 = `steerTargetStep`의 무기억 argmax** (`arena_motion.js`): 직전 비행 방향을
   전혀 참조하지 않고 순간 목표 방위만으로 10개 후보를 재선택. waypoint 모드(에피소드의 50%)는
   조향 결과를 어디에도 저장하지 않아(`cvVelocity` 기록은 cv 전용), 막대에 막히면 ±120~180°
   탈출 후보로 점프→다음 프레임 복귀하는 **주기-2 진동**. 실측(40에피소드×20s): 85막대에서
   렌더 yaw 평균 55.1°/스텝, 29.4%가 90° 초과 반전; 110막대에서 75.8°/41.0%. pursuer가
   "똑똑해 보이는" 것은 `steerPursuerStep`에만 연속성 항(`-turn*0.35`)+저역 필터가 있어서다.
   **이것은 포팅 버그가 아니라 Python `target_motion.py` 조향 법칙의 충실한 재현** —
   trainer의 표적도 동일하게 진동한다(아래 후속 작업).
2. **"둘 다 정지" = 같은 결함의 다른 얼굴 + 기하**: 표적은 전속력으로 움직이지만(경로길이/
   명령속도=1.000) 제자리 진동이라 순수송이 붕괴 — 150막대에서 1초 순수송 효율 0.338, 벽시계의
   58.1%가 순정지. 추가로 110막대↑에서는 표적 여유공간(1.0m clearance) 자체가 단절됨(최대
   연결 성분: 85막대 96.4% → 110막대 45.4% → 150막대 12.5%) — 이건 코드가 아니라 기하 한계.
   기본 설정(85막대)에서는 둘-다-정지 0.00% — 사용자가 본 정지는 슬라이더를 높였을 때.
3. **부가 버그**: `advanceTarget`이 `speed≤1e-6`이면 `episode.age += dt` 전에 조기 return →
   속도 슬라이더 0에서 30초 워치독 무력화(120초에 에피소드 3개 vs 수정 후 16개).

### 적용한 수정 (`docs/status/arena_motion.js`)

- **FIX 1**: `episode.age += dt`를 조기 return 앞으로 호이스팅 — 워치독은 벽시계 기준.
- **FIX 2**: `steerTargetStep`에 opt-in 9번째 인자 `prevHeading` + **90° 연속성 창**
  (lexicographic: 창 내 clear > 창 밖 clear > blocked; veto 아님 — 창에 clear가 없으면 큰
  탈출 턴 허용). `createEpisode`에 `heading` 상태 추가, `advanceTarget`이 모든 패턴에서
  기록, cv 반사 후 재동기화. 스칼라 페널티(k=0.35/1.0/2.0)와 하드 ±120° 필터 변형은 감사에서
  실측으로 기각됨(전자는 반전 잔존, 후자는 push-out 0.23→79회/에피소드).
- **FIX 4**: 파일 헤더에 일시적 divergence 명시(trainer는 아직 진동함) — Python 반영 전까지
  대시보드 정직성 유지.

### 수정 후 실측 (독립 하네스 CURRENT/FIXED 컬럼 수렴 확인)

| 지표 | 수정 전 | 수정 후 |
|---|---|---|
| 렌더 yaw 변화/스텝 (85막대, dt=0.1) | 55.1° | **16.5°** |
| 90° 초과 반전 (85막대) | 29.4% | **5.1%** |
| yaw 변화/스텝 (110막대) | 75.8° | **19.0°** |
| 90° 초과 반전 (110막대) | 41.0% | **4.9%** |
| age 워치독 (speed=0, 10초) | 0.000 (죽음) | **10.000** |

`tests/test_status_arena_motion.js`에 회귀 테스트 T1-T5 추가(수정 전 실패 확인된 T1/T2 포함;
T3는 감사 제안 기하가 전 후보 차단이라 판별 불가 벡터 → 0.88-1.12m 환대 기하로 재설계).
전체 스위트 green. SwiftShader headless Chrome으로 장면 구동 확인.

### 기각된 가설 (재조사 방지)

- `sampleWaypoint` 전영역 균등 샘플: Python `_sample_waypoints`와 올바른 parity. clearance
  필터 추가해도 stall 0.3pp 변화 — 원인 아님.
- cv write-back(`cvVelocity` 갱신): 버그가 아니라 안정자(cv 모드 97.8%+ 프레임에서 직진 유지).
- `steerPursuerStep` no-candidate 경로: 36,250 프로브+70만 프레임에서 0회 발화 — 실질 사문.
- 기존 테스트 `moved>5.5` 무력 주장: 다음 줄 net-arrival 단언이 정지·2-cycle 모두 잡음(변이 검증).

### 후속 작업 (staged)

- **FIX 3 (Python parity)**: 같은 연속성 창을 `target_motion.py`+`navrl_task.py`에 이식.
  **현재 진행 중인 밀도 커리큘럼 run이 끝나고 채점된 뒤에만** 적용 — 진동하지 않는 표적은
  실제로 더 어려워 capture가 떨어지며, 이는 정책 퇴행이 아니라 과제 난이도 보정(task-version
  bump). 적용 시 waypoint/mixed 고밀도 셀 재측정 필요, circle 패턴(held-out eval)은 제외.
- `tools/probe_target_motion.py`는 per-step 속도만 봐서 이 결함에 맹목(반전은 norm 보존) —
  1초 순수송/명령속도 비율과 평균 방향 변화 지표를 추가한 뒤에야 증거로 사용 가능.

**(추가) synth2 전체 animate() 루프 재현 최종 검증** — 감사 목표치 전 항목 통과:
85막대(기본값): 렌더 turn 37.7°→**3.6°**(목표 ≤6°), >90° 프레임 20.6%→**0.6%**(≤2%),
둘-다-정지 0.00% 유지, 에피소드 회전율 57→57 동일(포획 난이도 무변).
150막대(슬라이더 최대): 1초 순수송 효율 0.338→**0.592**(≥0.55), 표적 순정지 58.1%→**19.2%**
(≤25%), 둘-다-정지 35.0%→**11.2%**(≤15%, 최장 연속 1.1s→0.6s), 에피소드 13→19개/240s.
속도 0 극단: 120초당 에피소드 3→16개, age-타임아웃 리셋 5회 발화(FIX 1 시그니처 확인).
잔존 정지는 전부 110막대↑ 기하 단절 구간(코드로 해결 불가, 감사 FIX 5 참조).
