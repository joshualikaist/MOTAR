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

### 다음 단계
1. `ppo_navrl_cnn.yaml`로 6000 epoch 본 학습 → MLP 베이스라인과 도달률/충돌률 비교.
2. 부족하면 보상 재균형(속도 vs 안전 가중), 커리큘럼 최종거리(18 m) 완주 확인.
