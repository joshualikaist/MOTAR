# MOTAR 운영 가이드

이 문서는 “어떤 명령을 복사해야 하는가”와 “결과를 어떻게 잃지 않는가”만 다룹니다. 연구 가설과
검증 gate와 다음 실험은 [`VERIFICATION.md`](VERIFICATION.md), charter는 [`RESEARCH_PLAN.md`](RESEARCH_PLAN.md),
최신 요약은 [`README.md`](README.md), 날짜별 기록은 [`WORKLOG.md`](WORKLOG.md)를 보세요.

> 기준일: 2026-09-01
>
> 현재 실행 authority는 새 PPO run이 아닙니다. Track A는 exact BOM·calibration·210개 sensor
> trial·real-log replay만
> [`docs/SIM2REAL_3DAY_EXECUTION_PLAN.md`](docs/SIM2REAL_3DAY_EXECUTION_PLAN.md)에 따라 진행합니다.
> 실제 hardware나 real log가 없으면 GPU 작업을 시작하지 않습니다. Track B recovery-v2는
> `FAIL_ROUTE_MECHANISM` 뒤 no-anchor probe까지 `INCONCLUSIVE`로 종료됐으며 추가
> GPU/PPO/retune/rerun authority가 없습니다. corrected non-overlap r2는
> `FAIL_ROUTE_MECHANISM`이고 새 PPO는 0 epoch입니다. canonical 1.5 v3 receipt는 NO-GO입니다.
> 현재 software GPU 후보는 별도 사전등록된 **lower-contract v3**(`baseline_1p25`)입니다.

GPU 명령을 찾기 전에 아래 동결 receipt를 먼저 검사합니다. 이 명령은 학습이나 평가를 실행하지
않고, Track A/B 원자료 SHA와 `Stage 2=false`, `long training=false`를 fail-closed로 확인합니다.

```bash
cd /home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator
/home/fair/miniconda3/envs/aerialgym/bin/python tools/check_research_authority.py --json
```

기계 판독 계약은 [`docs/research_authority_2026-08-26.json`](docs/research_authority_2026-08-26.json)이며,
허용된 다음 작업은 hardware BOM/calibration/210 trials/real-log offline replay뿐입니다.

현재 software MECHANISM_GATE(`global_astar_braking_v3`)는 CPU 계약만 허용합니다. 아래는 구현
검사 명령이며, GPU/PPO/simulator 평가를 실행하지 않습니다.

```bash
cd /home/fair/workspaces/aerial_gym_ws/.codex_worktrees/braking_route_v3
export PYTHONNOUSERSITE=1
python -m unittest discover -s tests -p 'test_navrl_braking_route_v3*.py'
python -m unittest discover -s tests -p 'test_navrl_target_route_planner.py'
python -m unittest discover -s tests -p 'test_navrl_target_motion.py'
python -m unittest discover -s tests -p 'test_navrl_two_envelope_recovery.py'
git diff --check
```

canonical 1.5 v3 단계 1은 2026-09-01에 NO-GO로 재현됐습니다(1.5 warmup mean 1.442577 m/s).
현재 GPU 순서는 **lower-contract v3**입니다. conda `aerialgym` Python, `PYTHONNOUSERSITE=1`,
**커밋된 clean tree**, `NAVRL_TARGET_BRAKING_CONTRACT_VARIANT=baseline_1p25`. 셀 어댑터는
tracked `tools/run_navrl_braking_route_v3_cell.py`입니다. 2026-08-26 lower receipt는 재사용하지
않습니다.

**단계 L1 — baseline_1p25 braking receipt (GPU).** 현재 커밋 바이트에 바인딩된 새 receipt가
필요합니다. 속도는 0.6/0.9/1.2/1.25입니다.

```bash
cd /home/fair/workspaces/aerial_gym_ws/.codex_worktrees/braking_route_v3
export PYTHONNOUSERSITE=1
export NAVRL_TARGET_BRAKING_CONTRACT_VARIANT=baseline_1p25
NAVRL_BRAKING_PYTHON=/home/fair/miniconda3/envs/aerialgym/bin/python \
NAVRL_NINJA=/home/fair/miniconda3/envs/aerialgym/bin/ninja \
/home/fair/miniconda3/envs/aerialgym/bin/python \
  tools/run_navrl_physical_target_braking_v2_fresh.py \
  --output results/navrl_physical_target_braking_lower1p25_seed827_2026-09-01
sha256sum results/navrl_physical_target_braking_lower1p25_seed827_2026-09-01/receipt.json
```

**단계 L2 — training source bundle (CPU, receipt 이후, 저장소 밖).**

```bash
cd /home/fair/workspaces/aerial_gym_ws/.codex_worktrees/braking_route_v3
/home/fair/miniconda3/envs/aerialgym/bin/python tools/create_navrl_source_bundle.py create \
  --require-clean \
  --output /home/fair/workspaces/aerial_gym_ws/navrl_v3_receipts/training_source_lower1p25_2026-09-01
export MOTAR_V3_TRAINING_SOURCE_MANIFEST=/home/fair/workspaces/aerial_gym_ws/navrl_v3_receipts/training_source_lower1p25_2026-09-01/source_manifest.json
export MOTAR_V3_TRAINING_SOURCE_MANIFEST_SHA256=<create 출력의 manifest_sha256>
```

**단계 L3 — development pilot (GPU, 8 cells, seed 829, 70 bars, 1.25 상한).**

```bash
cd /home/fair/workspaces/aerial_gym_ws/.codex_worktrees/braking_route_v3
export PYTHONNOUSERSITE=1
export NAVRL_TARGET_BRAKING_CONTRACT_VARIANT=baseline_1p25
NAVRL_V3_OUTPUT_ROOT=/home/fair/workspaces/aerial_gym_ws/navrl_v3_runs/pilot_lower1p25_seed829_2026-09-01 \
NAVRL_V3_CELL_RUNNER=$PWD/tools/run_navrl_braking_route_v3_cell.py \
NAVRL_V3_BRAKING_RECEIPT=$PWD/results/navrl_physical_target_braking_lower1p25_seed827_2026-09-01/receipt.json \
NAVRL_V3_BRAKING_RECEIPT_SHA256=<단계 L1 sha256sum 값> \
bash tools/run_navrl_braking_route_v3_pilot.sh
```

confirmatory는 lower pilot 8/8 PASS일 때만, seed 839, bars 70/115/160/205. 어느 단계도
0.05 warmup 게이트나 PID를 바꾸지 않습니다. confirmatory PASS가 여는 것은 별도 사전등록할
500-epoch PPO smoke뿐이며, 장기학습 authority는 만들지 않습니다.

## 1. 처음 설치할 때

필수 조건은 Linux, NVIDIA GPU, Miniconda, Isaac Gym Preview 4입니다. Isaac Gym은 NVIDIA 계정으로
직접 내려받아야 하며 기본 경로는 `~/isaacgym`입니다.

```bash
mkdir -p ~/workspaces/aerial_gym_ws/src
cd ~/workspaces/aerial_gym_ws/src
git clone https://github.com/joshualikaist/MOTAR.git aerial_gym_simulator
cd aerial_gym_simulator

./bootstrap_second_machine.sh
conda deactivate
conda activate aerialgym
```

Isaac Gym 위치가 다르면 한 번만 경로를 넘깁니다.

```bash
ISAACGYM_PATH=/absolute/path/to/isaacgym ./bootstrap_second_machine.sh
```

설치 후 가장 먼저 CPU 계약 검사를 실행하세요.

```bash
cd ~/workspaces/aerial_gym_ws/src/aerial_gym_simulator
export PYTHONNOUSERSITE=1
python -m unittest discover -s tests -p 'test_navrl*.py'
```

`~/.local`의 NumPy가 conda 환경을 덮으면 Warp가 깨질 수 있습니다. bootstrap이 아래 설정을 넣지만,
수동 설치라면 직접 실행하고 conda 환경을 다시 활성화하세요.

```bash
conda env config vars set PYTHONNOUSERSITE=1 -n aerialgym
conda deactivate && conda activate aerialgym
```

## 2. 두 GPU의 역할

| 머신 | canonical 역할 | 주의점 |
|---|---|---|
| RTX 3070 8 GB | main profile 학습, 128 env, formal 평가 | 다른 NavRL 학습을 동시에 시작하지 않음 |
| GTX 1650 Ti 4 GB | `GPU4GB=1` held-out 평가와 가벼운 진단 | main 학습 seed와 같은 결과표에 합치지 않음 |

현재 corrected-v2 main 계약은 128 env입니다. 과거 문서의 256 env나 generic 64-env 학습 명령은 현행
ref5in 계보의 시작 명령이 아닙니다. 4 GB profile은 PhysX buffer를 줄이므로 결과 receipt에 profile을
반드시 남깁니다.

RTX 50 계열처럼 Isaac Gym Preview 4가 지원하지 않는 새 CUDA architecture는 구매·이전 전에 실제
호환성을 따로 확인하세요. 이 저장소의 현재 검증 머신은 RTX 3070입니다.

## 3. 지금 실행할 명령 찾기

현재 명령 authority는 `VERIFICATION.md`와 sim-to-real 72시간 계약이 정합니다. 아래 P0–P3 도식과
학습 명령은 계보 재현을 위한 역사적 운영 참고이며, 현재 GPU 실행 허가가 아닙니다.

```text
P0 repository/simulator gate
  └─ P1 fresh learning smoke
       └─ P2 held-out 70-bar decision cell
            └─ P3 full-budget seed 211
```

2026-08-13 현재 P1c의 preflight와 실제 실행은 다음과 같습니다. 이 문장은 P1c가 끝난 뒤 역사 명령이
되므로, 다시 실행하기 전 README와 WORKLOG의 최신 판정을 확인하세요.

```bash
cd ~/workspaces/aerial_gym_ws/src/aerial_gym_simulator/aerial_gym/rl_training/rl_games

REF5IN_PREFLIGHT_ONLY=1 ./train_navrl_v2_ref5in_smoke_c.sh
./train_navrl_v2_ref5in_smoke_c.sh
```

고정 launcher는 CLI 인자와 `CKPT` resume를 거부하고, runtime source가 commit되지 않았으면 실제
학습을 시작하지 않습니다. 실패한 smoke를 임의로 이어 돌리는 대신 새 corrective 계약을 먼저
RESEARCH_PLAN에 기록합니다.

## 4. 학습 상태 보기

고정 launcher는 터미널에 로그를 보여 주고, 같은 내용을 `train_session_logs/`에도 보존합니다.

```bash
cd ~/workspaces/aerial_gym_ws/src/aerial_gym_simulator/aerial_gym/rl_training/rl_games

./watch_navrl_training.sh
tail -f train_session_logs/current_ref5in_smoke_c.log
```

실행 중인 프로세스와 GPU는 다음처럼 확인합니다.

```bash
pgrep -af 'runner.py .*--task navrl_task .*--train'
nvidia-smi
```

TensorBoard는 run마다 따로 서버를 띄우지 말고 상위 `runs` 하나만 엽니다.

```bash
tensorboard \
  --logdir ~/workspaces/aerial_gym_ws/src/aerial_gym_simulator/aerial_gym/rl_training/rl_games/runs \
  --port 6006
```

중간 run이 여러 줄로 보이는 것은 각 run이 독립 event file을 갖기 때문입니다. 하나의 새 학습을 이전
run과 시각적으로 이어 붙이는 것은 provenance를 숨기므로 하지 않습니다. 같은 run을 resume할 때만
전용 continuation launcher가 명시적으로 branch lineage를 기록합니다.

## 5. 정상 종료를 확인하는 법

끝났다는 판단은 콘솔 마지막 한 줄이 아니라 아래 세 가지를 함께 봅니다.

```bash
RUN=/absolute/path/to/runs/ppo_YYMMDD_HHMM_name

test -f "$RUN/.aerial_training_finished"
python -m json.tool "$RUN/aerial_run/run_summary.json" | tail -40
ls -lh "$RUN"/nn/last_gen_ppo_ep_*.pth | tail
```

- `.aerial_training_finished`가 있어야 정상 terminal path입니다.
- `run_summary.json`의 `exit_reason`과 `last_epoch`를 확인합니다.
- curriculum 끝 정책은 `gen_ppo.pth`가 아니라 terminal `last_gen_ppo_ep_*.pth`입니다.
- formal gate는 전용 analyzer/attestation 결과를 사용합니다. 마지막 epoch의 capture 한 값만 보지
  않습니다.

강제 중단이 필요하면 먼저 PID를 확인하고 해당 프로세스에 `SIGINT`를 한 번 보냅니다. 무관한 Python
전체를 kill하지 마세요.

```bash
pgrep -af 'runner.py .*--task navrl_task .*--train'
kill -INT <확인한_PID>
```

## 6. 평가 규칙

범용 corrected-v2 density evaluator의 기본 사용법은 다음과 같습니다.

```bash
cd ~/workspaces/aerial_gym_ws/src/aerial_gym_simulator/aerial_gym/rl_training/rl_games

CKPT=/absolute/path/to/last_gen_ppo_ep_XXXX_rew_YY.pth
NAVRL_SEED=313 \
NAVRL_V2_ACTION_MODE=deterministic \
NAVRL_V2_DENSITIES="70" \
./eval_navrl_v2_density_sweep.sh "$CKPT" 2049
```

단, 연구 gate에는 README/RESEARCH_PLAN에 고정된 seed·밀도·조건과 깨끗한 wrapper를 사용합니다.
shell에 남은 `NAVRL_*`, `GPU4GB`, governor, detector, pose/appearance 변수가 조건을 바꿀 수 있기
때문입니다. evaluator는 다음을 fail-closed로 확인합니다.

- checkpoint의 arena, sensor, token selector, action contract;
- robot name과 contract-v1 config/URDF SHA;
- learned detector가 필요하면 detector SHA;
- 요청 seed, action mode, density, full goal/FOV 조건;
- checkpoint와 runtime source snapshot receipt.

`2049`는 requested completed episodes의 최솟값입니다. 128개 vector environment가 동시에 진행되므로
실제 완료 수는 더 클 수 있고, 비율의 분모는 JSON의 `actual_episodes`입니다.

## 7. checkpoint와 결과를 다른 컴퓨터로 옮기기

Git에는 `runs/`와 `.pth`가 들어가지 않습니다. 최소 이 묶음을 옮기세요.

```text
runs/<run>/nn/last_gen_ppo_ep_*.pth
runs/<run>/aerial_run/
runs/<run>/summaries/
train_session_logs/<matching log>
train_source_receipts/<matching receipt>/
results/<matching evaluation>/
```

먼저 checkpoint hash를 적습니다.

```bash
sha256sum /absolute/path/to/last_gen_ppo_ep_XXXX_rew_YY.pth
```

같은 네트워크라면 `rsync`가 가장 단순합니다.

```bash
rsync -avh --progress \
  /absolute/path/to/runs/ppo_YYMMDD_HHMM_name/ \
  user@other-host:/absolute/path/to/MOTAR/aerial_gym/rl_training/rl_games/runs/ppo_YYMMDD_HHMM_name/
```

파일 하나만 전달해야 하면 checkpoint를 복사하되, 최소한 SHA·run 이름·source manifest를 같이
전달하세요. shape가 같아도 robot 또는 observation 의미론이 다를 수 있습니다.

## 8. 디스크 정리

삭제 전에 `run_summary.json`, terminal checkpoint SHA, results summary가 WORKLOG에 기록됐는지
확인합니다. smoke run을 정리할 때도 canonical 결과의 근거 파일은 남깁니다.

- 보존: 논문 표에 쓰인 terminal checkpoint, `aerial_run`, `summaries`, source/eval receipt.
- archive 가능: 실패한 짧은 smoke의 TensorBoard event와 중간 checkpoint.
- 삭제 금지: 사용 중인 run, 최신 terminal checkpoint, 결과가 아직 문서화되지 않은 evaluation.

TensorBoard 화면만 단순하게 만들고 싶으면 old `summaries/`를 저장소 밖 archive로 이동한 뒤
WORKLOG에 원래 run과 이동 위치를 남깁니다. 기록 없이 여러 run을 합치거나 event file을 편집하지
않습니다.

## 9. 자주 만나는 메시지

### duplicate NavRL training

```text
refusing duplicate NavRL training; active PID(s): ...
```

오류가 아니라 중복 학습 방지입니다. `pgrep`와 `nvidia-smi`로 기존 run을 확인하세요. 의도적으로 두
학습을 같은 GPU에서 돌리는 것은 canonical 계약이 아닙니다.

### `libtinfo.so.6: no version information available`

conda의 bash/terminfo 경고인 경우가 많습니다. 그 뒤 launcher가 계속되고 epoch가 증가하면 학습 실패
원인은 아닙니다. 반드시 뒤의 실제 exit message를 봅니다.

### `torch/fx/experimental/symbolic_shapes ... unknown range`

PyTorch compile의 경고일 수 있습니다. traceback, NaN/Inf fail-stop, process 종료가 없고 epoch가
증가하면 단독으로 실패 판정하지 않습니다.

### GPU OOM

- 3070 main: 다른 GPU 프로세스를 먼저 정리합니다. canonical 128 env를 임의로 낮추지 않습니다.
- 1650 Ti: formal evaluator에서 `GPU4GB=1`을 사용합니다.
- `NUM_ENVS`만 바꾸면 PPO minibatch와 physics profile이 동시에 달라질 수 있으므로 generic 수정은
  금지합니다.

### terminal을 닫자 학습이 끝남

사용자가 직접 장기 run을 띄울 때는 고정 launcher가 제공하는 로그 경로를 확인하고 `nohup` 또는
`tmux`를 사용합니다. 다만 동일 launcher를 두 번 실행하지 마세요.

```bash
nohup ./<audited-fixed-launcher>.sh > train_session_logs/manual_nohup.out 2>&1 &
echo $!
```

### evaluator provenance mismatch

강제 옵션으로 덮기 전에 checkpoint와 현재 code/robot/detector가 진짜 같은 계보인지 확인합니다.
formal result에는 force로 만든 셀을 넣지 않습니다.

## 10. 하지 않는 것

- `gen_ppo.pth`를 density curriculum의 끝 정책으로 평가하지 않음.
- archived 601-action 결과와 corrected exact-600 결과를 합치지 않음.
- legacy `navrl_quad` checkpoint를 `navrl_ref5in_quad` 성능으로 부르지 않음.
- training log capture를 held-out 성공률로 부르지 않음.
- 한 training seed의 좁은 episode CI를 multi-seed 재현성으로 부르지 않음.
- 결과를 본 뒤 gate를 낮추거나 실패 run을 몰래 이어 돌리지 않음.

과거 recovery/riskcap/1650Ti 실험의 세부 명령은 Git history와 WORKLOG에 남아 있습니다. 현재 계보를
시작할 때는 이 문서와 README의 고정 launcher만 사용하세요.

## 2026-08-25 routed physical gate 운영 기록

attempt 1은 conda Python의 inherited `PATH`에 matching `ninja`가 없어 0/32
`VOID_EXECUTION`이었다. attempt 2는 별도 output directory에서 32/32 JSON integrity PASS를
기록했지만 `FAIL_ROUTE_MECHANISM`과 `BLOCKED_PHYSICAL_TRAINING`이다. 따라서 이 결과를 PPO
학습 명령이나 hardware validation 명령으로 재사용하지 않는다.

당시 후속 진단 전에 확인하도록 기록한 항목:

- [`attempt 2 summary`](results/navrl_physical_target_routed_gate_seed827_attempt2/summary.md)의
  route-on/off 32-cell 표와 receipt SHA를 확인한다.
- route-on의 `unsafe_start` soft-envelope recovery deadlock을 safe-prefix/full-horizon 계측으로
  분리한다. local invalidation 0.125–0.635%와 fallback 32.6–85.0%를 같은 지표로 합치지 않는다.
- 70-bar 4-speed pool의 plan success `.1454668471`, fallback `.359296875`가 각각 `.99`, `.01`
  gate를 못 넘고, 별도 70×0.6 cell의 goal/env `.25`가 `.5` gate를 못 넘는 것을 확인한다.
  same-goal은 0이다.
- evaluator와 preregistered threshold를 수정하거나 300-bar arena-wide connectivity를 주장하지
  않는다. physical PPO는 새 preregistered lineage에서 동일 32-cell gate가 닫힌 뒤에만 검토한다.

## 2026-08-26 Track B 종료 상태

Recovery-v2 lower-1.25는 32/32 integrity를 통과했지만 7/32만 PASS(모두 route-off), recovery
0/16으로 `FAIL_ROUTE_MECHANISM`이다. 70-bar pool은 plan success `93.60%`, fallback `47.87%`,
70×0.6 goal completions/env `0.21875`, recovery `NO_CONNECTOR` occupancy `63.06%`다.

후속 no-anchor observer probe는 primary `n=1`, identity disagreement `0`으로 유효하게 검증됐지만
최소 `n=20`을 못 채워 `INCONCLUSIVE`다. 이 결과는 32-cell FAIL을 바꾸지 않는다. Track B에서
gain 2.5, `0.45 m`, PPO, 1.5 m/s, env 수, 32-cell rerun 또는 추가 GPU probe를 위한 운영 명령은
없다. 다음 실행은 Track A의 hardware/real-log 조건이 충족될 때만 72시간 계약에서 찾는다.
