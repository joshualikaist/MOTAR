# OPERATIONS — 머신 셋업 · GPU 스케일링 · 결과 이관

두 대의 학습/평가 머신을 세우고, GPU 등급별 설정을 고르고, run 결과를 옮기는 실무 문서.
연구 내용은 `RESEARCH_PLAN.md`, 진행 기록은 `WORKLOG.md`(맨 아래가 최신), 진단 도구 레퍼런스는
`CRASH_TUNING_LOG.md`를 본다.

하드웨어 구성: **RTX 3070 8GB = 학습 공장**, **GTX 1650 Ti 4GB = 평가 공장**.
1650 Ti는 N=128로 학습 시 3070(N=256) 대비 약 10–15pt 약하므로 **학습 수치를 섞지 말 것**.
평가는 N에 둔감하므로 1650 Ti로 돌려도 된다.

## 1. GPU 선택과 호환성

## 0. ⚠️ 사기 전에 반드시 확인 — 호환성 (제일 중요)

이 프로젝트는 **Isaac Gym Preview 4** 위에서 돕니다. 이건 **개발 종료(EOL)** 라 CUDA 11.x + 구버전
PyTorch에 고정돼 있습니다. 그래서:

| GPU 세대 | arch | Isaac Gym Preview 4 | 판단 |
|---|---|---|---|
| RTX 30 (Ampere, 3070/3090) | sm_86 | ✅ 잘 됨 | 안전 |
| RTX 40 (Ada, 4070~4090) | sm_89 | ✅ 잘 됨 | 안전 |
| A/L 시리즈 (A5000/A6000/A100/L40S) | Ampere/Ada | ✅ 잘 됨 | 안전 (클라우드) |
| **RTX 50 (Blackwell, 5070~5090)** | **sm_120** | **❌ 거의 안 됨** | **지금은 사지 말 것** |

- **Blackwell(50번대)은 CUDA 12.8+** 를 요구 → 구 CUDA로 빌드된 PyTorch엔 커널이 없어
  `no kernel image available` 로 **실행 자체가 안 될 가능성**이 큼.
- 50번대를 쓰려면 **Isaac Gym Preview 4 → Isaac Lab/Isaac Sim** 으로 스택을 이전해야 함(별도 큰 작업).
  Aerial Gym이 그쪽을 지원하기 전까지는 30/40번대·A/L 시리즈에 머무를 것.
- **결론: 로컬은 24GB Ampere/Ada(중고 3090 추천), 클라우드는 A5000/4090/L40S.**

## 2. GPU 등급별 프리셋

## 2. 조절 손잡이 (knobs) — 파일·환경변수

| knob | 위치 | 역할 | VRAM/속도 영향 |
|---|---|---|---|
| `NUM_ENVS` | `train_navrl.sh` (기본 256) | 병렬 환경 수 | ↑ = VRAM·처리량 ↑ |
| `minibatch_size` | `ppo_navrl_cnn*.yaml` (4096) | PPO 미니배치 | **제약: horizon(32)×NUM_ENVS ≥ minibatch → NUM_ENVS ≥ 128** |
| `horizon_length` | `ppo_navrl_cnn*.yaml` (32) | 롤아웃 길이 | ↑ = VRAM(버퍼) ↑ |
| 카메라 해상도 | `RESEARCH_PLAN.md` 초기값 160×96 RGB-D | offline detector | ↑ = detector/dataset 메모리 ↑↑ |
| policy history | 2초, 0.5초 간격 5 samples | NavRL++-Target | ↑ = structured token·activation 메모리 ↑ |
| `NAVRL_MAX_BARS`/`NAVRL_NUM_BARS` | 환경변수 | 장애물 밀도(빌드/활성) | 미미 |
| `NAVRL_VISION` | 환경변수 (1=켬) | 현재 analytic semantic **프로토타입** | 최종 learned detector가 아님 |
| GPU4GB 프리셋 | `base_sim_4gb_config.py` + `*_4gb.yaml` | PhysX 버퍼 축소 | 4GB(1650Ti)용 |
| 네트워크 크기 | 예정 perception/policy modules | detector 크기; Transformer dim 64·4 layers | ↑ = VRAM·속도 ↓ |

> **미니배치 함정**: `NUM_ENVS`를 줄이면 `minibatch_size`도 같이 줄여야 함
> (`horizon×NUM_ENVS ≥ minibatch`). 안 그러면 rl_games가 시작 못 함. 예: NUM_ENVS=64면 minibatch ≤ 2048.

---

## 3. GPU 등급별 권장 세팅 (복붙용)

### (A) 현재: RTX 3070 8GB (메인)
```bash
# 센서 프로토타입과 향후 detector/tracker + structured Transformer 개발.
NUM_ENVS=256           # minibatch 4096 충족
# PPO rollout에는 raw image가 아니라 structured track tokens/[CLS] latent를 저장한다.
NAVRL_VISION=1 NAVRL_NUM_BARS=110 ./train_navrl.sh --seed 1
```
- 현재 semantic prototype은 256 env에서 검증됐다. 최종 방식은 offline detector pretraining과 structured
  17-token rollout을 사용한다. raw RGB-D sequence를 PPO observation buffer에 그대로 복제하지 않는다.

### (B) 보조: GTX 1650 Ti 4GB
```bash
# (1) 기존 navigation 및 semantic prototype baseline.
NAVRL_VISION=1 GPU4GB=1 NAVRL_NUM_BARS=50 ./train_navrl.sh --seed 1
#   OOM 나면 env 수를 낮춘다 (minibatch 2048 이 32*64=2048 도 나눔):
NAVRL_VISION=1 GPU4GB=1 NUM_ENVS=64 NAVRL_NUM_BARS=50 ./train_navrl.sh --seed 1
# (2) 평가 스윕 / 시각화
NUM_ENVS=128 HEADLESS=True PLAY_GAMES_NUM=3000 ./play_navrl.sh <checkpoint>   # 4gb 프리셋 자동
```
- 4GB 머신은 dataset 생성, detector baseline, 평가 전담으로 둔다. Transformer+PPO 본학습은
  8GB 이상에서 수행한다. semantic prototype 성능을 최종 perception 결과로 보고하지 않는다.

### (C) 업그레이드 로컬: RTX 3090 / 4090 (24GB) ★추천
```bash
NUM_ENVS=512           # minibatch 8192로 올려도 됨 (yaml 수정)
# 큰 detector backbone, 더 긴 history, raw-token ablation 가능
NAVRL_VISION=1 NAVRL_NUM_BARS=110 ./train_navrl.sh --seed 1
```
- VRAM 걱정 사라짐. §2의 절약 knob 전부 원상복구 가능. 속도도 3070 대비 2~3배.

### (D) 클라우드: A5000(24GB)/L40S(48GB)/A100(40·80GB)
```bash
# 최종 히트맵 대량 스윕 몰아치기. 여러 run 병렬 or 큰 NUM_ENVS.
NUM_ENVS=1024          # minibatch 16384 (yaml), 48GB+에서
```
- 시세 대략 $0.3~1.5/시간. 밀도×속도 히트맵(10~20 run)을 **$100~300**에 며칠 내 완료.
- 설치: CUDA 11.x 이미지 + Isaac Gym Preview 4 + 이 리포. (§4)

---

## 3. VRAM 예산과 8GB 생존 규칙

## 5. VRAM 대략 예산 (감 잡기용)

| 구성 | 대략 VRAM (N=256) |
|---|---|
| 물리 + warp 메시 + LiDAR | ~2.5 GB |
| analytic semantic 센서 prototype | 현재 256 env에서 동작; 최종 모델 아님 |
| 160×96 RGB-D + LiDAR detector offline batch | 구현 후 실측 필요 |
| 17-token Transformer(dim 64, 4 layers) | NavRL++ 기준 경량 구조; 구현 후 실측 필요 |
| raw RGB-D sequence를 PPO buffer에 직접 저장 | 금지에 가까움; OOM 위험 가장 큼 |

**8GB 생존 규칙**: detector는 offline sequence batch로 먼저 학습하고, PPO에는 structured obstacle/target
track과 `[CLS]` latent만 저장한다. end-to-end fine-tuning은 작은 env batch, gradient accumulation으로
마지막에만 수행한다. semantic target id/mask로 메모리를 아끼는 것은 연구적으로 허용되지 않는다.

---

## 4. 두 번째 머신 셋업

## 2. Isaac Gym Preview 4 다운로드 (수동 — 유일한 관문)

NVIDIA 계정 로그인이 필요해 자동화가 안 된다.

1. https://developer.nvidia.com/isaac-gym 에서 **Isaac Gym Preview 4** 다운로드.
2. 압축 해제 후 홈에 배치:

```bash
tar -xf IsaacGym_Preview_4_Package.tar.gz -C ~/
ls ~/isaacgym/python   # 이 경로가 존재해야 함
```

## ⚡ 빠른 시작 (부트스트랩 — §3~6을 한 번에)

위 §1(conda)·§2(Isaac Gym)만 끝나 있으면, 아래 두 줄로 나머지(코드 clone·환경·설치·스모크테스트)가
전부 자동으로 된다:

```bash
git clone -b research/navrl-env git@github.com:joshualikaist/MOTAR.git aerial_gym_simulator
cd aerial_gym_simulator && ./bootstrap_second_machine.sh
```

- urdfpy(원본 `mmatl/urdfpy`)를 알아서 clone·설치하고 `PYTHONNOUSERSITE`까지 설정.
- Isaac Gym 경로가 다르면 `ISAACGYM_PATH=/path ./bootstrap_second_machine.sh`.
- NavRL 참고 repo도 받으려면 `CLONE_NAVRL=1 ./bootstrap_second_machine.sh`.

아래 §3~6은 이 스크립트가 하는 일의 **수동 버전**이다(문제 생기면 단계별로 참고).

### 4.1 conda/pip 정확한 버전

## 4. conda 환경 + 패키지 설치

```bash
conda create -n aerialgym python=3.8 -y
conda activate aerialgym

# (a) Isaac Gym
cd ~/isaacgym/python && pip install -e .

# (b) MOTAR 본체 + 의존성
cd ~/workspaces/aerial_gym_ws/src/aerial_gym_simulator
pip install -e .
pip install rl-games==1.6.5 warp-lang==1.0.0

# (c) urdfpy 포크
pip install -e ~/workspaces/aerial_gym_ws/src/urdfpy
```

## 5. 두 가지 gotcha (README 명시)

```bash
# (1) ~/.local numpy가 conda numpy를 덮어써서 warp 센서가 깨지는 문제 방지
conda env config vars set PYTHONNOUSERSITE=1 -n aerialgym
conda deactivate && conda activate aerialgym   # 적용
```

2. 커스텀 스크립트에선 `torch`보다 `isaacgym`(또는 `aerial_gym`)을 **먼저** import.
   (repo 코드엔 이미 반영돼 있음 — 직접 스크립트 짤 때만 주의.)

### 4.2 4GB 머신의 PhysX 함정 (가장 자주 걸리는 곳)

> **왜 `NUM_ENVS` 만으로는 안 되나 (4 GB 함정):** NUM_ENVS 를 낮춰도 두 가지가 남아 학습이 안 뜬다 —
> ① Isaac Gym 의 PhysX **GPU 버퍼**(`base_sim` 의 `max_gpu_contact_pairs=2**24`, `buffer_multiplier=10`)는
> env 수와 무관하게 8000-env 용으로 고정 할당돼 4 GB 를 넘겨 `GymPhysX.cpp: out of memory` 로 죽는다.
> ② rl_games 는 `minibatch_size ≤ horizon_length(32)×num_envs` 를 요구하는데 `runner.py` 가 자동 축소를 안 해
> 기본 `minibatch_size=4096` 은 num_envs≥128 에서만 유효하다. **`GPU4GB=1` 이 한 번에 처리한다**:
> PhysX 버퍼 축소(`base_sim_4gb`, `AERIAL_GYM_SIM_NAME` 로 선택) + `ppo_navrl_cnn_4gb.yaml` — 단, PPO 하이퍼파라미터
> (minibatch 4096 포함)는 **8GB yaml 과 동일하게 유지**(밀도 스윕 비교가능성). NUM_ENVS 기본 256, **반드시 ≥128**.

### 4.3 셋업 요약표

## 요약 표

| 항목 | 내용 |
|------|------|
| Isaac Gym 다운로드 | NVIDIA 계정 필요, 자동화 불가 (2번) |
| Python | **3.8** 고정 (Isaac Gym Preview 4 요구) |
| 최신 코드 | 이미 클론돼 있으면 실행 전 `git pull` (3번) |
| 학습 실행 | `GPU4GB=1 ./train_navrl.sh` (7번) |
| 최종 perception/policy | dual-sensor detector/tracker + structured 17-token Transformer; GT/semantic actor 입력 금지 |
| VRAM 4 GB | `GPU4GB=1` 프리셋(PhysX 버퍼↓, minibatch 4096 유지, NUM_ENVS 기본 256 — 반드시 ≥128) |
| urdfpy | git 미포함 → 메인 머신에서 복사 (3-b) |
| 결과 회수 | `runs/`는 git 제외 → **rsync**로 메인에 회수 (9번) |

### 4.4 스모크 테스트

5. **스모크로 검증** (학습 전 필수):
   ```bash
   PYTHONNOUSERSITE=1 python tools/test_navrl_p3_smoke.py          # 정적/주입 경로 (156 관측)
   NAVRL_VISION=1 python tools/test_navrl_p3_stage0.py             # semantic 센서 prototype 회귀
   python tools/test_navrl_p3_math.py                              # 리워드 수학
   ```

---

## 5. run 결과 이관

경로 규약: 이 저장소는 `~/workspaces/aerial_gym_ws/src/aerial_gym_simulator`에 둔다
(구 문서의 `~/MOTAR` 표기는 폐기).

### 5.1 run 폴더 구조

## 1. 옮길 대상 — run 폴더 구조 이해

한 번 학습하면 이런 폴더가 하나 생긴다:
```
~/MOTAR/aerial_gym/rl_training/rl_games/runs/ppo_YYMMDD_HHMM_navrl/
├── summaries/                     ← TensorBoard 곡선(tfevents). 비교엔 이것만 있으면 됨. 수 MB
│   └── events.out.tfevents...
├── aerial_run/
│   ├── epoch_metrics.csv          ← epoch별 지표 원본(밀도 n_bars_active, reward 등). 작음
│   └── run_summary.json           ← 최종 요약(reward, 완주 여부)
└── nn/                            ← 정책 체크포인트(.pth). 재생/이어학습용. 수백 MB (무거움)
    └── gen_ppo.pth, last_gen_ppo_ep_*.pth ...
```

향후 NavRL++-Target run에는 아래도 함께 보관한다:

```text
├── perception/
│   ├── detector_best.pth
│   ├── config.yaml
│   └── dataset_manifest.json
└── run_manifest.json              # git commit + observation schema + model type
```

`YYMMDD_HHMM` 은 학습 **시작 시각**이다. 실행할 때 콘솔에 `run folder : ppo_XXXX_navrl` 로 찍히고,
`train_session_logs/train_XXXX.log` 로그 파일과 짝을 이룬다.

### 5.2 "이게 어떤 run이었나" 식별 레시피

## 2. (전송 전) 이 run 이 어떤 설정이었는지 확인하기

여러 run 중 뭘 옮길지 헷갈릴 때. 보조머신에서:
```bash
cd ~/MOTAR/aerial_gym/rl_training/rl_games

# (a) 밀도(활성 bar 수) + 빌드 bar 수 + 커리큘럼 여부   ← XXXX 는 실제 시각으로
grep -oE "active_bars=[0-9]+ max_bars=[0-9]+ curriculum=[A-Za-z]+" train_session_logs/train_XXXX.log | head -1
# (b) env 개수
grep "num envs" train_session_logs/train_XXXX.log | head -1
# (c) 완주했는지 + 최종/최고 reward
grep -oE '"(last_epoch|last_mean_reward|peak_mean_reward|exit_reason)": [^,]+' \
    runs/ppo_XXXX_navrl/aerial_run/run_summary.json
```
- `active_bars=` 가 그 run 의 **밀도**다 (예: `active_bars=50` → 밀도 50).
- `max_bars=` 는 **빌드된** bar 수. 깨끗한 밀도 비교를 하려면 여러 run 이 **같은 env 수 + 같은 코드**
  로 돌았는지 꼭 확인. (`max_bars=num_bars` 로 맞추면 "빌드=활성" 이라 배치 밀도가 일관됨.)


### 5.3 rsync 이관 (권장 경로)

## 9. 학습 결과를 메인 머신으로 회수 (`runs/` 는 git 제외 → rsync)

`runs/`·`nn/`·`*.pth`·`*.log`는 `.gitignore` 대상이라 **git으로는 안 넘어온다**. 학습이 끝나면
**rsync/scp로 직접** 메인(3070) 머신으로 가져와 평가·재생한다. (TensorBoard 스칼라와 `.pth`는
머신 독립이라 그대로 열린다.)

**메인 머신에서** 실행해 보조 머신의 결과를 당겨온다 (`ppo_XXXX_navrl`는 실제 run 이름으로 교체):

```bash
RG=aerial_gym/rl_training/rl_games
BASE=~/workspaces/aerial_gym_ws/src/aerial_gym_simulator
REMOTE=<계정>@<보조머신IP>:~/workspaces/aerial_gym_ws/src/aerial_gym_simulator

# (a) 평가·TensorBoard 만 볼 거면 summaries/ 만 — ~10 MB, 빠름
rsync -avz -e ssh $REMOTE/$RG/runs/ppo_XXXX_navrl/summaries/ \
                       $BASE/$RG/runs/ppo_XXXX_navrl/summaries/

# (b) 재생(play)까지 하려면 nn/ 체크포인트 포함해 run 폴더 통째로 — ~200 MB
rsync -avz -e ssh $REMOTE/$RG/runs/ppo_XXXX_navrl/ \
                       $BASE/$RG/runs/ppo_XXXX_navrl/
```

가져온 뒤 메인에서:

```bash
cd $BASE/$RG
tensorboard --logdir runs                                   # 곡선 비교
./play_navrl.sh runs/ppo_XXXX_navrl/nn/gen_ppo.pth          # 정책 재생
```

### 5.4 RustDesk 파일 전송 (rsync가 안 될 때)

## 방법 A — RustDesk 파일 전송  (집컴이 다른 네트워크일 때, 기본)

RustDesk 는 **파일 전송** 기능이 있어서 같은 와이파이가 아니어도 두 컴 사이에 파일을 옮길 수 있다.

### A-1. 보조머신에서 — 한 파일로 압축

**곡선만 (가벼움, 비교용 · 권장):**
```bash
cd ~/MOTAR/aerial_gym/rl_training/rl_games/runs

# staging 폴더에 "density_25 / density_50" 같이 알아볼 이름으로 담고 압축
#  (TensorBoard 범례가 폴더명으로 뜨므로 이름을 깔끔히 하면 좋다)
S=/tmp/tb_stage; rm -rf $S
mkdir -p $S/density_25 $S/density_50
cp -r ppo_260716_0357_navrl/summaries ppo_260716_0357_navrl/aerial_run $S/density_25/   # ← 밀도25 run
cp -r ppo_260716_1250_navrl/summaries ppo_260716_1250_navrl/aerial_run $S/density_50/   # ← 밀도50 run
tar czf ~/transfer_curves.tar.gz -C $S density_25 density_50
ls -lh ~/transfer_curves.tar.gz     # 보통 수 MB
```
> run 이름(`ppo_260716_0357_navrl` 등)·라벨(`density_25`)은 그때그때 실제 값으로 바꿔 쓰면 된다.

**정책 재생까지 필요하면 (무거움) — 폴더 통째로:**
```bash
cd ~/MOTAR/aerial_gym/rl_training/rl_games/runs
tar czf ~/transfer_full.tar.gz ppo_260716_0357_navrl ppo_260716_1250_navrl   # nn/ 포함, 수백 MB
```

### A-2. RustDesk 로 그 파일 보내기
1. RustDesk 메인 화면에서 3070 ID 를 넣는데, **화면 공유 대신 "파일 전송(Transfer File)"** 을 고른다.
   (연결 카드의 `⋮`/폴더 아이콘, 또는 접속 후 상단 툴바의 파일 전송 버튼.)
2. 두 쪽 파일 목록이 나온다. **이 컴(보조머신) 쪽에서 홈 폴더의 `transfer_curves.tar.gz`** 를 고른다
   (`/home/joshuali/transfer_curves.tar.gz`).
3. **3070 쪽 홈 폴더(내 폴더)** 로 전송한다. (어디 뒀는지 기억해두면 아래가 편하지만, 몰라도 됨.)

### A-3. 3070 에서 압축 풀기 (경로 자동 탐색 — 그대로 복붙)
3070 에서 터미널을 열고 통째로 붙여넣기:
```bash
TB=$(find ~ -name 'transfer_curves.tar.gz' 2>/dev/null | head -1)
RUNS=$(find ~ -type d -path '*aerial_gym/rl_training/rl_games/runs' 2>/dev/null | head -1)
tar xzf "$TB" -C "$RUNS"
echo "완료 → $RUNS 에 넣었습니다:"; ls "$RUNS" | grep -i density
```
→ 3070 의 `runs/` 폴더를 알아서 찾아 `density_25`, `density_50` 를 풀어 넣는다. (경로 몰라도 됨.)

### A-4. 같은 터미널에서 TensorBoard 켜기
```bash
tensorboard --logdir "$RUNS"
```
→ 브라우저 `http://localhost:6006` 에서 3070 에 원래 있던 것들 + 방금 넣은 게 **한 그래프에 겹쳐** 뜬다.


---

## 6. v2 actor-collapse 안전 복구

붕괴한 `sched-s1`은 ep10800 이후를 재사용하지 않는다. 아래 전용 런처는 감사된 ep9500 파일의
SHA-256, 40×40 v2 아레나·센서·토큰·행동정책 계약을 확인하고, 130막대에서 정확히 100 epoch만
smoke한다. training seed=1, task/YAML/env 수/simulator/selector/TTC/detector/FOV/tilt,
density/LR/KL/rollback/margin/collapse guard/tag/log까지 고정하며, 이전 shell의
`ALLOW_CONCURRENT`, 4GB preset, network override, viewer/evaluator flag도 제거한다. 이 복구
계보에서는 `SEED`를 바꿀 수 없다. held-out 평가는 별도 seed=42로 강제된다.

```bash
cd /home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/aerial_gym/rl_training/rl_games

# 선택: 학습 없이 파일·계약만 확인
NAVRL_PREFLIGHT_ONLY=1 ./train_navrl_v2_recover_safe.sh

# 실제 ep9500 → ep9600 고정-130 recovery smoke (로그가 터미널에 보임)
./train_navrl_v2_recover_safe.sh
```

정상 종료 후 반드시 `last_gen_ppo_ep_9600_*.pth`의 **정확한 파일명**으로 held-out 평가한다.

```bash
NAVRL_V2_DENSITIES=130 ./eval_navrl_v2_density_sweep.sh \
  runs/ppo_<recovery-smoke-run>/nn/last_gen_ppo_ep_9600_<reward>.pth 2049
```

이 명령은 단순 JSON뿐 아니라 smoke run의 정확한 100개 TensorBoard epoch를 함께 감사한다.
capture≥0.65, crash≤0.35, timeout≤0.10, outcome count/rate 일치, post-update KL≤0.04,
학습·평가 task-input OOB=0, **100 epoch 전체에서 rollback event/streak/누적값=0**을 모두 통과해야 한다.
평가기는 원본 checkpoint의 별도 CoW snapshot을 실제로 재생하고, 시작·종료 checkpoint SHA,
64자 nonce, cell log, evaluator script와 결과 JSON을 `130bars.receipt.json`에 묶는다. 평가 seed=42,
TTC/detector/FOV/noise/max-tilt 계약까지 함께 재검산한 뒤에만
`.navrl_v2_recovery_eval_pass.json`을 run root에 원자적으로 만든다. 결과 JSON만 수기로 만들거나
평가 뒤 같은 경로의 checkpoint를 교체하면 snapshot/receipt 검증에서 거부된다. recovery
checkpoint에서는 `NAVRL_V2_FORCE=1`로 provenance를 우회할 수 없다.

복구용 held-out는 메인 물리 계약(`base_sim`, dt=0.01, 128 env)을 강제한다. 호출 전 shell에
`AERIAL_GYM_SIM_NAME`, `NUM_ENVS`가 남아 있어도 덮어쓰며 `GPU4GB=1`은 복구 증명서 생성에서
거부한다. 목표 거리는 checkpoint의 저장된 curriculum clock과 무관하게 6–28 m 전체를 사용하고
FOV도 최종 분포로 고정한다. 결과 JSON에는 launcher 라벨이 아니라 simulator에서 읽은 config
class, physics dt/substeps, RL step당 physics step 수와 RL step dt, 실제 목표 거리와 FOV 포화 여부가
기록되어야 한다. 4GB profile도 상속된 실제 dt=0.01을 기록한다.

평가까지 건강한 경우에만 같은 최종 checkpoint로 curriculum을 재개한다. 런처가 smoke lineage,
정확한 100 epoch, 정상종료 marker, held-out PASS 증명서와 checkpoint hash를 다시 확인하므로
평가하지 않은 checkpoint, 중간 checkpoint, ep10800은 모두 거부된다. 증명서가 가리키는 held-out
JSON의 실제 SHA-256만 보는 것이 아니라 JSON 내용과 100-epoch TensorBoard scalar를 원본에서 다시
계산하고 canonical 증명서 전체와 일치시키므로, 수기로 만든 PASS나 plausible 숫자만으로는 열리지
않는다. smoke anchor도 `epoch=9600`, `frame=39,321,600`, `num_task_steps=307,200`,
`k_min/k_max=20/28`을 정확히 만족해야 한다.

상태 사이트는 GitHub Pages의 **정적 snapshot**이므로, 사이트의 PASS/재개 가능 표시는
`generated_at` 시각에 로컬 증거를 검증했다는 뜻이다. 그 뒤 checkpoint·평가 JSON·TensorBoard가
바뀌었는지를 브라우저가 실시간으로 알 수는 없다. 따라서 사이트 문구만 보고 일반 런처를 직접
실행하지 말고, 아래 안전 복구 런처를 사용한다. 이 런처가 시작 직전에 같은 증거를 다시 검증하며,
정확한 `epoch=9600` 정상종료 marker가 없거나 달라졌으면 재개를 거부한다.

```bash
RECOVERY_MODE=curriculum \
CKPT=runs/ppo_<recovery-smoke-run>/nn/last_gen_ppo_ep_9600_<reward>.pth \
./train_navrl_v2_recover_safe.sh
```

커리큘럼이 중간에 끊기면 smoke부터 반복하지 않는다. 안전 커리큘럼이 저장한 최신
`last_gen_ppo_ep_*.pth`만 명시해 density evidence를 그대로 이어간다. checkpoint 안에는 PASS
증명서 원문(base64)+SHA가 함께 저장되므로 별도 파일을 옮기지 않아도 lineage를 검증할 수 있으며,
rollback 또는 scheduler가 낮춘 `current_action_learning_rate`도 다시 올리지 않고 그대로 복원한다.
continue checkpoint는 smoke anchor 기준 `task_step += 32/epoch`, `frame += 4096/epoch` 관계와
130→145→…→300 밀도 schedule도 만족해야 한다. 불일치는 자동 보정하지 않고 즉시 중단한다.
연속 PPO rollback 횟수도 checkpoint에 저장·복원한다. patience에 도달하면 이미 소비한 rollout의
frame을 반영하고, 복원된 model/Adam/RMS/scaler와 낮춘 LR을
`last_gen_ppo_ep_*_rew_rollback_livelock.pth`로 먼저 저장한 뒤 fail-stop하므로 재시작으로 streak나
LR backoff가 사라지지 않는다.

```bash
RECOVERY_MODE=continue \
CKPT=runs/ppo_<recovery-curriculum-run>/nn/last_gen_ppo_ep_<latest>_<reward>.pth \
MAX_EPOCHS=30000 \
./train_navrl_v2_recover_safe.sh
```

## 7. v2 종료 후 증거 동결·분석 절차

`ppo_260802_0020_navrl_v2-recover-curriculum-continue-s1`은 2026-08-02 사용자 결정으로 SIGINT 안전
중단되어 ep24010까지 기록되었다. `.aerial_training_finished`가 없는 것은 정상 완주가 아니라 의도적
중단이기 때문이다. 마지막 분석 대상은 50-epoch durable checkpoint인
`last_gen_ppo_ep_24000_rew_44.73549.pth`이며 SHA-256은
`82f7978b42d9d9e95adcc638a40ae85fb3736fd897ae39cb8aa8333be39cf23f`다.

canonical 계보는 다음 세 조각이다.

| 구간 | run | 포함 epoch |
|---|---|---:|
| recovery smoke | `ppo_260801_1150_navrl_v2-recover-smoke-130bars-s1` | 9501–9600 |
| curriculum | `ppo_260801_1235_navrl_v2-recover-curriculum-s1` | 9601–20700 |
| continuation | `ppo_260802_0020_navrl_v2-recover-curriculum-continue-s1` | 20701–24010 |

첫 curriculum run에 남은 20701–20746은 ep20700 checkpoint에서 되돌려 다시 학습된 구간이므로 최종
통계에서 제외한다. TensorBoard 시각화에는 원자료 보존을 위해 남아 있을 수 있지만 분석기는 위 규칙으로
deduplicate해야 한다.

동결 artifact와 최종 분석 재검증:

```bash
cd /home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/aerial_gym/rl_training/rl_games

# 1) runner가 실제로 끝났는지와 의도적 interrupted summary 확인
ps -eo pid,etime,args | rg 'runner.py.*navrl_task.*--train'
jq '{exit_reason,last_epoch,finalized_at}' \
  runs/ppo_260802_0020_navrl_v2-recover-curriculum-continue-s1/aerial_run/run_summary.json

# 2) 평가 대상은 동결된 마지막 durable checkpoint. gen_ppo.pth 금지
sha256sum runs/ppo_260802_0020_navrl_v2-recover-curriculum-continue-s1/nn/last_gen_ppo_ep_24000_rew_44.73549.pth

# 3) postmortem과 dashboard를 재생성
cd /home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator
/home/fair/miniconda3/envs/aerialgym/bin/python tools/analyze_navrl_v2_postrun.py
/home/fair/miniconda3/envs/aerialgym/bin/python tools/summarize_navrl_v2_limit_audit.py
/home/fair/miniconda3/envs/aerialgym/bin/python tools/update_status_snapshot.py
```

핵심 판정은 `results/navrl_v2_ep24000_limit_audit.{md,json}`이다. 기존 curriculum은 재개하지 않는다.
mirror/conjugate와 두 번째 seed 검증은 frozen ep24000의 **평가만** 수행해 완료했으며 결과는
`results/navrl_v2_ep24000_causal_1to3/summary.{md,json}`이다. 재현 명령은
`eval_navrl_v2_ep24000_causal_1to3.sh`; 기존 결과를 overwrite하지 않으므로 재실행 시
`NAVRL_CAUSAL_RESULT_ROOT`를 새 경로로 지정한다. 고정속도 평가와 1650 Ti smoke까지 끝난 뒤
fixed-205 A/B를 승인할 경우에만 아래
launcher를 사용한다. 두 팔은 같은 profile 안에서 비교한다.

고정속도 인과검사 4번은 아래 명령 하나로 실행한다. 동결 ep24000 정책을 205 bars에서
0.3/0.9/1.5 m/s로 각각 2,049회 평가하고
`results/navrl_v2_ep24000_fixed_speed/summary.{md,json}`까지 만든다. optimizer와 checkpoint는
변경하지 않는다.

```bash
cd /home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/aerial_gym/rl_training/rl_games
./eval_navrl_v2_ep24000_fixed_speed.sh
```

기존 결과 디렉터리가 있으면 덮어쓰지 않고 중단한다. 재실행이 필요할 때만 새 출력 경로를 명시한다.

```bash
NAVRL_FIXED_SPEED_RESULT_ROOT=../../../results/navrl_v2_ep24000_fixed_speed_retry \
  ./eval_navrl_v2_ep24000_fixed_speed.sh
```

고정속도 평가 다음의 망각 검사는 아래 명령 하나로 실행한다. 205막대 승급 직전 ep19100과 최종
ep24000을 uniform 및 fixed 1.5 m/s에서 각각 비교하는 평가-only 2×2 실행이다.

```bash
cd /home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/aerial_gym/rl_training/rl_games
./eval_navrl_v2_ep19100_vs_ep24000_forgetting.sh
```

결과와 자동 판정은
`results/navrl_v2_ep19100_vs_ep24000_forgetting/summary.{md,json}`에 저장된다. 기존 결과는 덮어쓰지
않으며 optimizer/RMS/checkpoint를 변경하지 않는다.

망각검사는 uniform +4.65 pp, fixed-1.5 +3.25 pp로 두 조건 모두 improvement였고, 1650 Ti TTC gate도
capture +9.86 pp/crash -8.06 pp로 PASS했다. main baseline은 2026-08-03에 아래 run으로 이미 정상
완료됐다. 다시 실행하지 않는다.

```bash
cd /home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/aerial_gym/rl_training/rl_games
run=runs/ppo_260803_1819_navrl_v2-ep24000-205bars-main-baseline-s1
jq '{exit_reason,last_epoch,epochs_logged}' "$run/aerial_run/run_summary.json"
sha256sum "$run/nn/last_gen_ppo_ep_25000_rew_29.188496.pth"
```

기대값은 `max_epochs`, epoch 25000/1,000개이며 checkpoint SHA-256은
`169ddcddb83c9d74df5c79252274660bc9c52e32d7d5144d325698e32b1d9b08`이다. 다음에는 baseline
ep25000의 held-out 205-bar 결과부터 고정한다.

```bash
cd /home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/aerial_gym/rl_training/rl_games

NAVRL_V2_DENSITIES=205 \
NAVRL_SEED=42 \
NAVRL_V2_ACTION_MODE=deterministic \
NAVRL_EVAL_REFLECTION_MODE=original \
NAVRL_V2_RESULT_DIR=../../../results/navrl_v2_ep24000_ttc_main_baseline \
./eval_navrl_v2_density_sweep.sh \
  runs/ppo_260803_1819_navrl_v2-ep24000-205bars-main-baseline-s1/nn/last_gen_ppo_ep_25000_rew_29.188496.pth \
  2049
```

`results/navrl_v2_ep24000_ttc_main_baseline/205bars.json`이 생성되고 evaluator가 ≥2,049 episodes,
seed42, deterministic/original, U[0.3,1.5], selector=`cluster_sector`를 승인한 뒤에만 TTC를 실행한다.
이 평가는 완료됐으며 capture/crash/timeout **69.50/28.99/1.51%**다. 따라서 primary TTC 통과선은
capture **≥71.50%**, crash **≤26.99%**다. ep24000 원본 72.44/25.07보다 악화됐으므로 원본을 직접
교체하려면 TTC가 **72.44% 이상/25.07% 이하**도 만족해야 한다. launcher가 baseline result/receipt/
snapshot/log SHA와 이 조건을 실행 직전에 재검증한다.

```bash
# RTX 3070: 1,000 epoch × 32 × 128 = 4,096,000 samples
ARM=ttc PROFILE=main ./train_navrl_v2_ep24000_ttc_ab.sh
```

TTC final ep25000도 같은 evaluator와 `NAVRL_V2_RESULT_DIR=../../../results/navrl_v2_ep24000_ttc_main_ttc`
조건으로 평가한다. 채택 gate는 main baseline 대비 held-out capture `+2.0 pp 이상`이면서 crash `-2.0 pp
이하`다. 둘 중 하나만 통과하면 기각한다. density, reward, action sigma, speed limit, LR은 이 A/B에서
바꾸지 않는다. 4GB arm은 70-bar 선행 gate가 이미 끝났으므로 main 결과의 추가 확인이 필요할 때만
2,000×32×64로 사용한다.

이 평가도 완료됐다. TTC는 2,051 episodes에서 capture/crash/timeout **70.21/29.50/0.29%**로 baseline
대비 **+0.71/+0.51/-1.22 pp**였다. primary와 canonical replacement floor 모두 FAIL이므로 같은 TTC
mode를 재실행하거나 연장하지 않는다. selector 하나를 바꿨지만 구현상 effective candidate FOV도
240°→360°로 바뀌므로 이 결과는 current representation bundle의 음성 결과이며 pure ranking ablation은
아니다.

## 7.10 최종 riskcap artifact와 운영 규칙 (2026-08-05)

현재 source의 에피소드 horizon은 정확히 600 actions=60 s다. 다만 아래 frozen 결과를 만들 당시에는
종료 비교가 `>600`여서 실제 timeout은 action 601이었고, rl_games의 `time_outs` bootstrap signal도
누락됐다. 실제 bar contact는 평균 7.4--9.2 s에 발생했고 timeout은 0.29--4.00%여서 이 한 step 차이가
“horizon 연장이 해결책이 아니다”라는 진단을 뒤집지는 않지만, legacy 결과와 새 schema-v2 결과를 같은
arm으로 섞어서는 안 된다. corrected 5-cell screen에서 완전정지 clearance/TTC는
timeout 16.59/23.57%로 기각됐다. 채택된 `riskcap`은 command corridor가 3 m 이내일 때만 XY norm을
2.0 m/s로 제한하고 5 m에서 3.535 m/s로 완전히 해제하며, 방향을 바꾸거나 정지를 강제하지 않는다.

현재 navigation/control candidate는 아래 일반 checkpoint와 **riskcap 실행 설정의 결합**이다. checkpoint만
governor off로 실행하면 최종 결과를 재현하지 못한다.

```bash
cd /home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/aerial_gym/rl_training/rl_games
ckpt=runs/ppo_260805_0413_navrl_v2-speedgov-ep24000-205bars-main-riskcap-s1/nn/last_gen_ppo_ep_25000_rew_39.742134.pth
sha256sum "$ckpt"
```

기대 SHA-256은
`f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40`이다. run은 epoch
24001--25000, 4.096M samples를 `max_epochs`로 정상 완료했고 `.aerial_training_finished`가 존재해야 한다.
기존 평가 폴더는 legacy 601-action 증거이므로 덮어쓰거나 그 안에서 재실행하지 않는다. 원자료 계약과
요약만 다시 검증하려면 다음을 사용한다.

```bash
cd /home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator
/home/fair/miniconda3/envs/aerialgym/bin/python \
  tools/summarize_navrl_v2_riskcap_postadapt.py \
  results/navrl_v2_riskcap_postadapt \
  --trained-sha f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40
```

이 명령은 9개 cell의 checkpoint/snapshot/result/log/receipt SHA, nonce, sensor-only provenance,
seed45/46, 205 bars, deterministic/original, 속도 조건을 모두 재검증한다. 기대 판정은
`winner_kind=trained`, `generalization_pass=true`다. 새 평가가 꼭 필요하면 기존 결과를 보존한 채 launcher의
`RESULT_ROOT`를 새 디렉터리로 바꾼 복사본을 사용한다.

다음 학습은 riskcap을 더 연장하는 run이 아니다. 먼저 analytic detector와 learned detector의 동일
checkpoint 평가 계약을 만들고 detection dropout/지연/거리오차를 한 축씩 screen한다. gate를 통과할
때에만 detector fine-tuning 또는 temporal fusion 학습을 시작한다. riskcap의 2.0/3/5 m 값은 final
seed45/46를 본 뒤 사후 조정하지 않는다.

다음 publication 평가는 `docs/NEXT_WEEK_HANDOFF_2026-08-10.md`의 Gate 1부터 시작한다. ep24000/off,
ep24000/riskcap, ep25000/riskcap 전 arm을 같은 미사용 seed, 정확한 600-action 종료, 동일 source manifest로
새 결과 디렉터리에 측정한다. 과거 요약기는 archive 무결성 확인용이지 새 계약의 대체물이 아니다.

15셀을 한 명령으로 순차 실행하는 canonical launcher는 다음과 같다. seed 53과 2,049 episodes/cell은
launcher 내부에 고정돼 있고, 각 cell은 하나의 shared source bundle SHA를 검증한다.

```bash
cd /home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/aerial_gym/rl_training/rl_games
PREFLIGHT=1 ./eval_navrl_v2_governor_adaptation_abc.sh
./eval_navrl_v2_governor_adaptation_abc.sh
```

평가 중에는 다른 학습/평가를 같은 GPU에서 실행하지 않는다. 진행 로그는 아래로 본다.

```bash
tail -f ../../../results/navrl_v2_governor_adaptation_abc_seed53_schema2/campaign.log
```

## 8. 트러블슈팅

## 자주 겪는 문제 (트러블슈팅)

- **TensorBoard 에 곡선이 안 뜬다** → 그 run 폴더 안에 `summaries/events.out.tfevents...` 파일이 있는지 확인.
  없으면 학습이 몇 epoch 못 돌고 죽은 것(로그 확인).
- **범례가 `ppo_260716_0357_navrl` 처럼 지저분** → 폴더명을 `density_25` 등으로 `mv` 해서 바꾸면 됨.
- **밀도 비교인데 곡선이 안 맞는 느낌** → run 들이 **같은 코드 버전 + 같은 env 수 + 같은 배치방식(랜덤 vs 격자)**
  인지 확인. (구버전 run 은 bar 가 "jittered 7×7 격자" 였고, 현재는 랜덤 산포다. 섞으면 밀도만의 비교가 아님.)
- **학습 자체가 `CUDA Error 804` / `nvidia-smi: Driver/library version mismatch` 로 안 돈다**
  (전송과는 무관하지만 자주 겪음) → 백그라운드 드라이버 자동업데이트 후 커널 모듈이 안 맞는 것. **재부팅**하면 해결.
  재부팅 후 `nvidia-smi` 가 정상 표를 보이면 OK.
- **checkpoint shape mismatch** → 156/305/1265 등 옛 observation schema와 새 structured 17-token schema를
  섞은 것이다. manifest를 확인하고, detector·policy 양쪽의 정확히 같은 schema끼리만 로드한다.

**관측 차원 이력** (체크포인트 shape 불일치 진단용): 156(GT LiDAR) → 305(해석적 semantic,
폐기) → 1265(vision CNN, 폐기) → **898 actor / 906 critic (현재 sensor-only)**.
셋이 섞이면 로드가 실패하니 체크포인트와 실행 설정의 표현 계약을 반드시 맞춘다.
