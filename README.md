# MOTAR

MOTAR는 **막대가 빽빽한 공간에서 움직이는 표적을 쫓는 드론**을 연구합니다. 드론은 카메라와
LiDAR로 표적·장애물을 보고, 시뮬레이터 ego-state로 자신의 자세와 속도를 압니다. 목표는 멋진 한
장면을 만드는 것보다 “얼마나 빽빽해지면, 왜 실패하는가?”를 재현 가능한 숫자로 답하는 것입니다.

쉽게 말하면 세 문제가 한꺼번에 걸립니다.

- 빨리 쫓으면 잡기 쉽지만 제동거리가 길어져 막대에 부딪힙니다.
- 장애물이 늘면 8개 obstacle token에 모든 막대를 담을 수 없습니다.
- 학습 로그가 좋아 보여도 다른 seed·고정 조건에서 다시 평가하면 결과가 달라질 수 있습니다.

그래서 이 저장소에서는 최고 성공률 한 줄보다 **어떤 환경·기체·checkpoint·seed로 돌렸는지**, 그리고
실패가 capture/crash/timeout 중 무엇이었는지를 함께 보존합니다. 처음 보는 분은 아래 “현재 결론”과
“5분 안에 확인하기”만 읽고, 실험을 직접 돌릴 때 `OPERATIONS.md`로 넘어가면 됩니다.

> 현재 기준일: **2026-08-20**
>
> **문서 6개:** [VERIFICATION.md](VERIFICATION.md)(검증 gate·다음 실험) · [RESEARCH_PLAN.md](RESEARCH_PLAN.md)(charter) · [WORKLOG.md](WORKLOG.md)(기록) · [OPERATIONS.md](OPERATIONS.md)(명령) · [CRASH_TUNING_LOG.md](CRASH_TUNING_LOG.md)(과거 진단, archival) · [docs/status/](docs/status/)(대시보드)

## 현재 결론

| 항목 | 현재 판정 | 여기서 말할 수 없는 것 |
|---|---|---|
| corrected-v2 episode 의미론 | **공학 스모크 통과**. fresh PPO 1,000 epoch에서 정확히 600 action 종료, rl_games `time_outs` 전달, finite PPO/KL, rollback 0, raw action OOB 0을 확인했습니다. | 이 run은 70 bars의 on-policy 학습 기록입니다. held-out 성능, 밀도 승급 능력, 알고리즘 우월성을 증명하지 않습니다. |
| learned detector 연결 | frozen navigation policy에서 learned-v2가 analytic bootstrap 대비 비열등하다는 결과를 두 평가 seed에서 재현했습니다. | 더 최신인 learned-v7을 그대로 꽂으면 nominal 성능이 떨어집니다. threshold만 바꿔 해결되는 문제가 아니며, detector 출력 분포에 맞춘 별도 학습이 필요합니다. |
| 고밀도 기하 | 수정된 좌표계의 정적 2-D 검사에서 333/333 장면에 경로가 존재했습니다. | 회전·제동·표적 이동·600-step 제한을 포함한 동적 도달 가능성은 아닙니다. “길이 있으니 정책 문제”라고 단정할 수 없습니다. |
| `navrl_ref5in_quad` 후보 기체 | CPU 저장소 계약 **26/26**, canonical same-controller simulator gate **21/21**, P1c fresh 900-epoch engineering gate를 통과했습니다. 장거리 D1은 q3/CV capture를 D0 대비 +15.19pp 개선했습니다. | held-out P2는 timeout 상한을 넘어 **strict FAIL**이고, D1도 q3/CV timeout 15.98%로 사전등록한 12%를 넘어 **FAIL**입니다. P3 장기학습은 차단했습니다. 실기 비행, CAD, endurance, 열·전원 여유도 미검증입니다. |
| 과거 navigation 결과 | legacy evaluator 안에서는 비교 가능한 동결 기록으로 보존합니다. | old 601-action 결과를 corrected exact-600 결과와 합치거나, legacy 기체 결과를 ref5in 성능으로 부를 수 없습니다. |

gate 표와 진단 요약은 **[VERIFICATION.md](VERIFICATION.md)** 에 통합했습니다. camera-range A/B는
완료된 원인 진단이며, 28 m camera를 채택했다는 뜻은 아닙니다.
canonical 결과 링크: [P0](results/navrl_ref_platform_verification/summary.md) ·
[P1c](results/navrl_ref5in_smoke_seed197/p1c/summary.md) ·
[P2](results/navrl_ref5in_p2_seed313/summary.md) ·
[D0](results/navrl_ref5in_outcome_diagnostic_v2_seed317/summary.md) ·
[D1](results/navrl_ref5in_d1_eval_seed331/summary.md) ·
[first-acquisition seed359](results/navrl_ref5in_cv_first_acquisition_seed359/summary.md) ·
[camera-range seed367](results/navrl_ref5in_camera_range_control_seed367/summary.md).

## 시스템을 짧게 보면

현재 corrected-v2 baseline은 다음 흐름을 사용합니다.

```text
camera target track ─┐
                     ├─ structured observation ─ Transformer actor ─ bounded velocity/yaw command
72×4 LiDAR @ 12 m ───┘        8 cluster-sector obstacle tokens, token FOV 240°

training only: ground-truth state ─ asymmetric critic + reward
```

- actor에는 ground-truth 표적 위치나 semantic mask를 직접 넣지 않습니다.
- navigation/control을 분리해서 볼 때는 analytic detector를 기준선으로 씁니다.
- learned detector는 오프라인 gate와 navigation A/B를 따로 통과해야 합니다.
- action은 squashed Gaussian으로 유한 범위 안에 두며, 평가 때 deterministic/stochastic 모드를 반드시 기록합니다.
- 현재 corrected-v2 공학 baseline은 governor를 끈 조건입니다. 과거 `riskcap` 결과는 별도 legacy 계보입니다.

## 이름이 비슷한 결과를 섞지 않는 법

MOTAR에는 **과제 계약**과 **기체 계약**이라는 서로 다른 두 축이 있습니다.

### 과제·평가 계약

| 계보 | 핵심 차이 | 용도 |
|---|---|---|
| historical v1 | 24×24 m arena와 과거 observation/배치 계약 | 초기 아이디어와 실패 양상 참고용 |
| archived v2 | 40×40×3 m arena지만 과거 종료 조건은 600 설정에서 action 601까지 실행됐고 `time_outs` bootstrap이 없었습니다. | 같은 evaluator로 만든 동결 결과끼리만 비교 |
| **corrected-v2** | 40×40×3 m, exact 600 actions, `time_outs`, checkpoint/평가 receipt와 source provenance guard | 앞으로의 재현·비교 기준 |

### 기체 계약

| robot name | 의미 | 주의점 |
|---|---|---|
| `navrl_quad` | 기존 checkpoint와 논리 연결을 보존하는 legacy simulation 기체 | 0.28 m 충돌 proxy, ±0.13 m motor 좌표, 0.25 kg stock dynamics 등 mixed-scale 개발 파라미터가 exact BOM/CAD의 단일 실제 플랫폼에 추적되지 않습니다. 어떤 실제 기체에도 대응할 수 없다고 증명한 것은 아닙니다. |
| `navrl_ref5in_quad` | 1.20 kg, 220 mm motor diagonal, 0.28 m XY 충돌 proxy를 가정한 opt-in 5-inch hardware-informed simulation candidate | 저장소 정합성·canonical open-arena gate와 P1c engineering gate는 통과했지만, held-out P2는 timeout 상한을 넘었습니다. 장기 P3는 미실행이며 legacy curve에 수치를 이어 붙이지 않습니다. |

따라서 “v2”라고 적혀 있어도 종료 의미론이 다르면 같은 실험이 아니고, observation shape가 같아 checkpoint가 로드되더라도 robot lineage가 다르면 같은 정책으로 평가하면 안 됩니다. 현재 evaluator는 checkpoint에 저장된 arena·sensor·robot provenance를 확인하고 불일치를 fail-closed하도록 설계되어 있습니다.

## 5분 안에 확인하기

### 1. 설치

Isaac Gym Preview 4와 NVIDIA GPU가 먼저 필요합니다. 저장소는 공개 clone이 가능하지만 Isaac Gym은 NVIDIA 계정으로 직접 받아야 합니다.

```bash
mkdir -p ~/workspaces/aerial_gym_ws/src
cd ~/workspaces/aerial_gym_ws/src
git clone https://github.com/joshualikaist/MOTAR.git aerial_gym_simulator
cd aerial_gym_simulator
./bootstrap_second_machine.sh
conda activate aerialgym
```

Isaac Gym 경로, Python 3.8, 4 GB GPU 설정과 `PYTHONNOUSERSITE` 문제는 [OPERATIONS.md](OPERATIONS.md)를 먼저 확인하세요.

### 2. 코드 계약 검사

GPU 학습 전에 CPU에서 빠르게 실패를 잡습니다.

```bash
cd ~/workspaces/aerial_gym_ws/src/aerial_gym_simulator
conda activate aerialgym
export PYTHONNOUSERSITE=1

python tests/test_navrl_v5a_semantics_smoke.py
python tests/test_navrl_ref5in_platform.py

cd aerial_gym/rl_training/rl_games
REF5IN_PREFLIGHT_ONLY=1 ./train_navrl_v2_ref5in_smoke_c.sh
```

마지막 명령은 GPU 학습을 시작하지 않고 ref5in launcher가 고정한 fresh/no-checkpoint 계약만 검사합니다.

### 3. 3-D로 환경 보기

저장소 루트에서 수동 조작으로 실제 Isaac Gym 환경을 볼 수 있습니다.

```bash
cd ~/workspaces/aerial_gym_ws/src/aerial_gym_simulator

./launch_navrl_3d.sh --manual

```

현재 public 3-D policy path는 구형 574D Transformer 전용이라 corrected-v2 898D/ref5in checkpoint를
거부합니다. robot·sensor·token·action metadata를 import 전에 복원하는 fail-closed playback이 회귀검사를
통과하기 전에는 현행 checkpoint를 이 UI로 재생하지 마세요. formal 평가는 아래 evaluator를 사용합니다.

### 4. corrected-v2 held-out 평가

끝 밀도 정책은 반드시 `last_gen_ppo_ep_*.pth`로 평가합니다.

```bash
cd ~/workspaces/aerial_gym_ws/src/aerial_gym_simulator/aerial_gym/rl_training/rl_games

CKPT=/absolute/path/to/last_gen_ppo_ep_XXXX_rew_YY.pth
NAVRL_V2_ACTION_MODE=deterministic \
NAVRL_V2_DENSITIES="130 160 190 205 220" \
./eval_navrl_v2_density_sweep.sh "$CKPT" 2049
```

evaluator는 checkpoint의 selector, detector, arena, robot 및 source receipt를 확인합니다. 강제 옵션으로 provenance 오류를 덮은 결과는 정식 비교표에 넣지 않습니다.

### 5. ref5in canonical gate와 학습 스모크

먼저 open-arena 명령 추종을 재측정합니다.

```bash
cd ~/workspaces/aerial_gym_ws/src/aerial_gym_simulator
conda activate aerialgym
export PYTHONNOUSERSITE=1

python tools/verify_navrl_ref_platform.py \
  --num-envs 16 \
  --output results/navrl_ref_platform_verification/flight_envelope.json
```

P1c는 이미 완료됐습니다. 아래 명령은 결과 재현이나 새 기체 파라미터를 바꾼 뒤 다시 gate를
확인할 때만 사용합니다. **현재 다음 단계로 P3를 실행하면 안 됩니다.**

```bash
cd ~/workspaces/aerial_gym_ws/src/aerial_gym_simulator
git status --short -- aerial_gym resources/robots tools/create_navrl_source_bundle.py
# 아무것도 출력되지 않아야 함. results/와 문서 초안은 실행 바이트가 아니므로 별도 표시됩니다.

cd aerial_gym/rl_training/rl_games
./train_navrl_v2_ref5in_smoke_c.sh
```

이 launcher는 seed 197, 900 epochs, LR `1.5e-5`, `navrl_ref5in_quad`, governor off와 corrected-v2 의미론을 고정합니다. CLI 인자와 `CKPT` resume를 일부러 거부합니다. P1a/P1b를 이어 돌리지 않고 매번 fresh weights로 시작합니다. 결과가 좋아도 이것은 **학습 가능성 engineering gate**일 뿐, legacy보다 낫다는 성능 주장이 아닙니다.

완료된 P2 proof는 다음 명령으로 byte-level 무결성을 다시 확인할 수 있습니다.

```bash
cd ~/workspaces/aerial_gym_ws/src/aerial_gym_simulator
/home/fair/miniconda3/envs/aerialgym/bin/python tools/attest_navrl_ref5in_p2.py verify
```

일반적인 `train_navrl.sh`는 여러 역사적 기본값을 허용하므로 현재 기준 실험의 시작 명령으로 추천하지 않습니다. 새 실험은 목적에 맞는 고정 launcher와 사전등록 문서를 먼저 추가합니다.

## checkpoint는 저장소에 없습니다

`runs/`와 `.pth` snapshot은 크기와 provenance 문제 때문에 `.gitignore` 대상입니다. README와 결과 문서에 보이는 `runs/ppo_.../nn/...pth`는 **그 실험을 수행한 로컬 경로**이지, clone 후 자동으로 생기는 다운로드 파일이 아닙니다.

다른 머신으로 옮길 때는 최소한 다음을 함께 보관하세요.

- `last_gen_ppo_ep_*.pth`
- run의 `aerial_run/`과 `summaries/`
- evaluation JSON/CSV와 receipt
- source manifest 또는 source bundle
- checkpoint SHA-256

```bash
sha256sum /absolute/path/to/last_gen_ppo_ep_XXXX_rew_YY.pth
```

run 폴더 이관과 TensorBoard 병합 절차는 [OPERATIONS.md](OPERATIONS.md)에 있습니다.

## 결과를 읽는 규칙

1. **끝 정책은 `last_gen_ppo_ep_*.pth`입니다.** `gen_ppo.pth`는 reward 최고점을 저장하는데, density curriculum에서는 쉬운 저밀도 시점이 선택될 수 있어 끝 밀도 정책을 대표하지 않습니다.
2. **exact-600과 old 601-action을 섞지 않습니다.** 종료 한 step 차이뿐 아니라 timeout value bootstrap 의미도 다릅니다.
3. **학습 로그의 capture는 성능표가 아닙니다.** curriculum과 stochastic exploration이 섞인 on-policy 진단값입니다. 주장은 동결 checkpoint의 held-out fixed-condition 평가에서 만듭니다.
4. **episode 수와 training seed 수는 다른 표본입니다.** 한 정책을 2,049 episodes 평가해 얻은 좁은 CI가 한 training seed의 우연을 제거하지 않습니다. full result를 주장하려면 최소 2개, 가급적 3개 training seed를 같은 예산으로 반복합니다.
5. **deterministic와 stochastic을 표시합니다.** 전자는 배포 평균 action, 후자는 PPO gate와 탐색 잡음의 영향을 봅니다. 둘을 한 열에 합치지 않습니다.
6. **결과에는 계약을 붙입니다.** checkpoint SHA, source SHA/dirty 여부, training/eval seed, episodes, bars, target speed, detector와 threshold, action mode, robot, episode semantics를 함께 기록합니다.
7. **한 셀의 최고 epoch를 고르지 않습니다.** 사전등록한 checkpoint와 seed를 사용하고 capture/crash/timeout을 pooled count와 95% CI로 보고합니다.

## 현재 근거에서 실제로 말할 수 있는 것

- learned-v2 detector navigation A/B 재현에서는 analytic이 3,271/4,098, learned-v2가 3,272/4,100 captures였습니다. 차이는 **-0.0145 percentage point**, 95% CI는 **[-1.752, +1.723] pp**로 사전등록한 -2 pp 비열등 margin을 통과했습니다. 원자료는 [replication summary](results/navrl_v2_detector_navigation_ab_replication_seed97_101_schema2/summary.md)에 있습니다.
- corrected-v2 seed-197 스모크는 exact-600, `time_outs`, finite PPO와 checkpoint provenance 경로가 실제 학습 중 작동한다는 것을 보였습니다. 그러나 epoch 1,000이 density warmup 경계와 같아 **밀도 승급은 시험하지 못했습니다**.
- corrected reachability audit의 333/333은 정적 경로 존재 증거입니다. 고밀도 정지나 충돌이 representation, control, dynamics 중 어디서 생기는지는 pre-contact trajectory를 맞춘 추가 평가가 필요합니다.
- isolated pose-noise audit에서는 한 environment seed에서 위치 1/3/10 cm와 yaw 0.5°의 손실을 검출하지 못했고, yaw 2°와 5°에서는 손실을 검출했습니다. 이는 step-wise iid Gaussian simulation sensitivity이며 실제 센서 허용오차 규격이 아닙니다.
- learned-v7은 선택한 appearance stress envelope에서 일부 장점이 있었지만 nominal analytic-trained actor와는 분포가 맞지 않았습니다. 그러므로 “detector가 좋아졌으니 기존 PPO에 교체”하거나 appearance randomization까지 한 번에 넣는 실험은 하지 않습니다.

과거 v1 density map과 v2 riskcap 실험은 실패 가설을 만드는 데 유용하지만 corrected-v2의 최종 성능표는 아닙니다. 숫자가 필요하면 메인 README의 요약보다 해당 `results/**/summary.md`의 계약·receipt·한계를 함께 읽으세요.

## 다음 실행 순서

1. **완료:** ref5in CPU 정합성 26/26과 canonical same-controller simulator gate 21/21을 고정했습니다.
2. **P1a 완료·FAIL:** fresh 500 epoch에서 outcome은 gate를 넘었지만 epoch 432 behavior-KL rollback 1회와 거리 27 m 종료 때문에 전체 gate를 통과하지 못했습니다.
3. **P1b 완료·FAIL:** LR `1.5e-5`에서 KL/rollback/OOB/outcome은 전부 통과했지만, 750 epoch도 마지막 2,048-episode 거리 증거창을 채우지 못해 max-state 27 m에서 끝났습니다. 실제 general-spawn 범위는 `[6,27] m`였습니다.
4. **P1c PASS:** budget만 900 epoch로 늘린 fresh run이 모든 engineering gate를 통과했습니다.
5. **P2 strict FAIL:** held-out seed 313에서 capture/crash는 통과했지만 timeout 5.56%가 상한 5%를 넘었습니다. legacy anchor와 P3는 실행하지 않았습니다.
6. **진단 완료:** 장거리에서 timeout과 crash가 함께 증가했고, q3 timeout은 CV에서 waypoint보다 14.20pp 높았습니다. 표적 속도 alone 가설은 지지되지 않았습니다.
7. **D1 완료·FAIL:** `[22.5,28] m` exposure로 모든 outcome은 개선됐지만 q3/CV timeout `15.98% > 12%`라 사전 gate를 통과하지 못했습니다. P2 FAIL은 유지합니다.
8. **heading 진단 완료:** away−toward timeout +23.32pp로 radial-heading 채널을 확인했고 tangent 좌우 outcome 차이는 기준 미달이었습니다.
9. **near-open 완료:** 1 bar에서도 away−toward timeout `+54.32pp`여서 dense obstacle occlusion 필요성은 기각했습니다.
10. **outcome telemetry 완료:** seed 353에서 away capture−timeout visibility는 `+14.43pp`로 20pp screen에 미달했습니다. wall-reflection 표는 생존 선택편향 때문에 인과 판정에 쓰지 않습니다.
11. **최초취득 계측 완료:** seed 359에서 outcome별 never-acquired rate는 away timeout `87.52%`, away capture `0.00%`, toward capture `0.00%`였습니다. 사전 30pp screen을 `+87.52pp`로 통과했고, 두 capture cohort가 모두 정확히 0%라는 점이 임계보다 강한 신호입니다 — 이 조건에서 최초 취득이 capture의 필요조건처럼 동작합니다. 독립 재계산에서 fused와 camera 최초취득이 6개 cohort 전부 일치해, 사실상 camera range 발견입니다. 다만 outcome은 궤적의 결과이므로 연관이지 인과가 아닙니다.
12. **camera-range 진단 완료:** seed 367에서 target camera range `20→28 m` 한 값만 바꾸자 timeout이 `55.80→18.16%`로 줄었습니다. 초기 미관측의 인과 기여는 지지하지만, 사용자가 원하는 과제는 장거리 미관측 상태에서의 active search이므로 28 m camera는 positive control이지 자동 채택안이 아닙니다. 다음은 OOB exit를 acquired/never-acquired로 나눈 동결-policy 계측이며 P3는 계속 차단합니다.
13. perception 연구는 corrected-v2 analytic baseline → learned detector arm → appearance-randomized arm 순으로 한 축씩 추가합니다.

현재 가장 큰 미해결 문제는 “더 오래 학습하면 되는가”가 아니라 **고밀도에서 pre-contact obstacle 정보, 제동 여유, detector 출력 분포, 기체 동역학 중 어느 축이 먼저 한계가 되는가**입니다.

## 저장소 지도

| 경로 | 내용 |
|---|---|
| `aerial_gym/task/navrl_task/` | observation, reward, termination, curriculum, telemetry |
| `aerial_gym/task/navrl_task/navrl_perception.py` | analytic/learned target front-end와 LiDAR association |
| `aerial_gym/config/task_config/navrl_task_config.py` | task와 curriculum 기본 계약 |
| `aerial_gym/config/env_config/navrl_bars_env.py` | v1/v2 arena와 bar 배치 |
| `aerial_gym/config/robot_config/` | `navrl_quad`, `navrl_ref5in_quad` 동역학·allocator 설정 |
| `resources/robots/quad/` | URDF와 충돌/관성 기하 |
| `aerial_gym/rl_training/rl_games/` | PPO config, network, 고정 train/eval launcher |
| `tests/` | 의미론·계보·수학·launcher 회귀검사 |
| `tools/` | 데이터셋, receipt, geometry/platform 검증 도구 |
| `results/` | 원자료와 조건별 `summary.md`; 성능 숫자의 근거 |
| `docs/` | 독립 검수, handoff, 기준 기체 제안, 발표 자료 |
| [WORKLOG.md](WORKLOG.md) | 날짜순 변경·실험 기록 |
| [RESEARCH_PLAN.md](RESEARCH_PLAN.md) | 가설, gate, 장기 순서와 중단 조건 |
| [OPERATIONS.md](OPERATIONS.md) | 설치, GPU별 실행, 결과 이관, 문제 해결 |

## 기반 프로젝트와 크레딧

MOTAR는 [Aerial Gym Simulator](https://github.com/ntnu-arl/aerial_gym_simulator)의 연구 fork입니다. 병렬 Isaac Gym 환경, multirotor/controller 구조와 sensor rendering 기반을 사용합니다. 장애물장과 LiDAR navigation 구성은 [NavRL](https://github.com/Zhefan-Xu/NavRL)을 참고해 Aerial Gym 안에서 다시 구현했습니다. PPO 실행은 [rl_games](https://github.com/Denys88/rl_games)를 사용합니다.

MOTAR 결과를 인용할 때는 이 저장소의 실험 계약과 함께 기반 프로젝트의 citation도 확인해 주세요.

```bibtex
@article{kulkarni2025aerial,
  author  = {Kulkarni, Mihir and Rehberg, Welf and Alexis, Kostas},
  title   = {Aerial Gym Simulator: A Framework for Highly Parallelized Simulation of Aerial Robots},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2025},
  volume  = {10},
  number  = {4},
  pages   = {4093--4100},
  doi     = {10.1109/LRA.2025.3548507}
}
```

라이선스는 [BSD-3-Clause](LICENSE)입니다. 이 저장소의 결과나 문구가 Aerial Gym 또는 NavRL 원 저자들의 검증을 대신하지 않습니다.
