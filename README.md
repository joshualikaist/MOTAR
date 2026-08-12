# MOTAR

**MOTAR는 카메라·LiDAR로 표적과 장애물을 보고, 시뮬레이터 ego-state로 자세와 속도를 아는 쿼드로터가 밀집 장애물 사이의 움직이는 표적을 어디까지 추적·요격할 수 있는지 측정하는 Aerial Gym 기반 연구 저장소입니다.**

이 저장소의 목표는 “드론 요격이 해결됐다”는 데모를 만드는 것이 아닙니다. 표적을 빨리 따라갈수록 충돌 위험이 커지고, 장애물이 많아질수록 제한된 obstacle token에 정보가 빠지는 문제를 같은 조건에서 반복 측정하는 것이 먼저입니다. 그래서 성공률 하나보다 **환경 계약, checkpoint 계보, 실패 구성(capture/crash/timeout), seed와 신뢰구간**을 함께 남깁니다.

> 현재 기준일: **2026-08-13**
>
> 자세한 변경 이력은 [WORKLOG.md](WORKLOG.md), 다음 실험의 순서와 중단 조건은 [RESEARCH_PLAN.md](RESEARCH_PLAN.md), 설치·운영 방법은 [OPERATIONS.md](OPERATIONS.md)에 있습니다.

## 지금 어디까지 됐나

| 항목 | 현재 판정 | 여기서 말할 수 없는 것 |
|---|---|---|
| corrected-v2 episode 의미론 | **공학 스모크 통과**. fresh PPO 1,000 epoch에서 정확히 600 action 종료, rl_games `time_outs` 전달, finite PPO/KL, rollback 0, raw action OOB 0을 확인했습니다. | 이 run은 70 bars의 on-policy 학습 기록입니다. held-out 성능, 밀도 승급 능력, 알고리즘 우월성을 증명하지 않습니다. |
| learned detector 연결 | frozen navigation policy에서 learned-v2가 analytic bootstrap 대비 비열등하다는 결과를 두 평가 seed에서 재현했습니다. | 더 최신인 learned-v7을 그대로 꽂으면 nominal 성능이 떨어집니다. threshold만 바꿔 해결되는 문제가 아니며, detector 출력 분포에 맞춘 별도 학습이 필요합니다. |
| 고밀도 기하 | 수정된 좌표계의 정적 2-D 검사에서 333/333 장면에 경로가 존재했습니다. | 회전·제동·표적 이동·600-step 제한을 포함한 동적 도달 가능성은 아닙니다. “길이 있으니 정책 문제”라고 단정할 수 없습니다. |
| `navrl_ref5in_quad` 후보 기체 | CPU 저장소 계약 **26/26**, canonical same-controller simulator gate **21/21**을 통과했습니다. P1b fresh 750 epoch의 last-100 outcome은 69.55/28.12/2.33%였고 KL/rollback/OOB/source gate도 통과했습니다. | P1b는 거리 `[20,27] m`에서 끝나 전체 FAIL입니다. 예산만 900으로 늘린 P1c를 통과하기 전에는 held-out/장기학습으로 가지 않습니다. 실기 비행, CAD, endurance, 열·전원 여유도 미검증입니다. |
| 과거 navigation 결과 | legacy evaluator 안에서는 비교 가능한 동결 기록으로 보존합니다. | old 601-action 결과를 corrected exact-600 결과와 합치거나, legacy 기체 결과를 ref5in 성능으로 부를 수 없습니다. |

현재 판단의 근거는 [독립 검수 보고서](docs/codex_review_2026-08-12.md), [플랫폼 P0](results/navrl_ref_platform_verification/summary.md), [P1a](results/navrl_ref5in_smoke_seed197/summary.md), [P1b](results/navrl_ref5in_smoke_seed197/p1b/summary.md)에 있습니다. 위 표의 “통과”는 각 문서에 적힌 좁은 gate만 뜻합니다.

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
| `navrl_quad` | 기존 checkpoint와 논리 연결을 보존하는 legacy simulation 기체 | 0.28 m 충돌 proxy, ±0.13 m motor 좌표, 0.25 kg stock dynamics가 한 실제 기체를 일관되게 나타내지는 않습니다. |
| `navrl_ref5in_quad` | 1.20 kg, 220 mm motor diagonal, 0.28 m XY 충돌 proxy를 가정한 opt-in 5-inch hardware-informed simulation candidate | 저장소 내부 정합성과 canonical open-arena simulator gate, on-policy engineering outcome만 관측했습니다. held-out·장기 과제 성능은 아직 없으며 legacy curve에 수치를 이어 붙이지 않습니다. |

따라서 “v2”라고 적혀 있어도 종료 의미론이 다르면 같은 실험이 아니고, observation shape가 같아 checkpoint가 로드되더라도 robot lineage가 다르면 같은 정책으로 평가하면 안 됩니다. 현재 evaluator는 checkpoint에 저장된 arena·sensor·robot provenance를 확인하고 불일치를 fail-closed하도록 설계되어 있습니다.

## 빠르게 확인하기

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

저장소 루트에서 수동 조작 또는 checkpoint 재생을 시작합니다.

```bash
cd ~/workspaces/aerial_gym_ws/src/aerial_gym_simulator

./launch_navrl_3d.sh --manual

# 정책을 재생할 때
CKPT=/absolute/path/to/last_gen_ppo_ep_XXXX_rew_YY.pth
./launch_navrl_3d.sh --checkpoint "$CKPT"
```

checkpoint가 없다면 `--manual`만 사용할 수 있습니다. 아래 설명처럼 `.pth` 파일은 Git에 들어 있지 않습니다.

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

그다음 **실행에 관여하는 source가 commit된 상태**에서 현재 사전등록된 P1c 스모크를 실행합니다.

```bash
cd ~/workspaces/aerial_gym_ws/src/aerial_gym_simulator
git status --short -- aerial_gym resources/robots tools/create_navrl_source_bundle.py
# 아무것도 출력되지 않아야 함. results/와 문서 초안은 실행 바이트가 아니므로 별도 표시됩니다.

cd aerial_gym/rl_training/rl_games
./train_navrl_v2_ref5in_smoke_c.sh
```

이 launcher는 seed 197, 900 epochs, LR `1.5e-5`, `navrl_ref5in_quad`, governor off와 corrected-v2 의미론을 고정합니다. CLI 인자와 `CKPT` resume를 일부러 거부합니다. P1a/P1b를 이어 돌리지 않고 매번 fresh weights로 시작합니다. 결과가 좋아도 이것은 **학습 가능성 engineering gate**일 뿐, legacy보다 낫다는 성능 주장이 아닙니다.

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
3. **P1b 완료·FAIL:** LR `1.5e-5`에서 KL/rollback/OOB/outcome은 전부 통과했지만, 750 epoch도 마지막 2,048-episode 거리 증거창을 채우지 못해 `[20,27] m`에서 끝났습니다.
4. **현재 단계:** task·seed·LR은 그대로 두고 budget만 900 epoch로 늘린 fresh P1c를 실행합니다. P1c가 통과해야만 같은 corrected-v2 계약의 held-out 평가를 수행합니다.
5. held-out gate까지 통과하면 먼저 full-budget seed 1개를 완주하고, 그 결과가 사전등록 기준을 만족할 때만 나머지 training seed를 추가합니다. 한 seed는 데모로만 취급합니다.
6. legacy와 ref5in을 비교할 때는 architecture·curriculum·예산·평가 seed를 맞추고 둘 다 fresh lineage로 만듭니다. 기존 legacy checkpoint를 ref5in에 그대로 재생하지 않습니다.
7. perception 연구는 corrected-v2 analytic baseline → learned detector arm → appearance-randomized arm 순으로 한 축씩 추가합니다.

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
