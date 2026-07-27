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
