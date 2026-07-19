# WORKLOG — MOTAR 연구 작업 기록

무엇을, 왜 바꿨는지의 시간순 기록. 최신이 아래쪽. (연구 로드맵은 워크스페이스 루트의
`RESEARCH_PLAN.md`, 실행 방법은 `README.md`의 Getting Started 참고)

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
