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

## 6. 트러블슈팅

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
