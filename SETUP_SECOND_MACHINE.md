# 보조 학습 머신 셋업 가이드 (Ubuntu 22.04 + 저사양 GPU)

메인 개발 머신(RTX 3070 8 GB) 외에 **두 번째 컴퓨터에서 학습만 돌리기 위한** 셋업 가이드다.
GPU A/B 비교나 밤샘 학습을 GPU 하나에 욱여넣지 않고 두 대로 나눠 돌리기 위한 용도.

작성 기준 예시 하드웨어: **GTX 1650 Ti (4 GB VRAM), Ubuntu 22.04**. VRAM이 작으므로
`num_envs`를 낮추고 반드시 헤드리스로 돌린다. VRAM이 더 큰 다른 GPU라면 `num_envs`만 올리면 된다.

> 메인 머신의 정식 설치 안내는 `README.md`의 *Getting Started (MOTAR)* 참고. 이 문서는
> 그걸 "두 번째 머신에 git으로 클론해서 학습만" 하는 상황에 맞춰 재정리한 것이다.

---

## 0. 사전 확인

```bash
nvidia-smi          # 드라이버 버전 + VRAM 확인 (드라이버 >= 470, 권장 525+)
lsb_release -a      # Ubuntu 22.04 확인
```

- `nvidia-smi`가 안 뜨면 드라이버부터: `sudo ubuntu-drivers autoinstall` 후 재부팅.
- Isaac Gym Preview 4는 **리눅스 전용 / Python 3.8** 요구.

## 1. Miniconda (없으면)

```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh
# 터미널 새로 열기
```

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

---

## 3. 코드 가져오기 (MOTAR repo + urdfpy)

MOTAR 본체는 git으로 받고, `urdfpy` 포크는 repo에 포함돼 있지 않으므로 **메인 머신에서 복사**한다.

```bash
mkdir -p ~/workspaces/aerial_gym_ws/src && cd ~/workspaces/aerial_gym_ws/src

# (a) MOTAR 본체 — research 브랜치
git clone -b research/navrl-env git@github.com:joshualikaist/MOTAR.git aerial_gym_simulator

# (b) urdfpy — 원본 mmatl/urdfpy 그대로라 직접 clone (수정본 아님 → scp 불필요)
git clone https://github.com/mmatl/urdfpy.git
```

> **이미 클론돼 있으면** 매 실행 전 `git pull` 로 최신 코드를 받는다(메인에서 push한 것 반영).
> 처음부터 학습할 거면 `runs/`, `nn/`(체크포인트)는 옮길 필요 없다.

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

## 6. 임포트 & 빌드 스모크 테스트

```bash
conda activate aerialgym
python -c "import aerial_gym; from aerial_gym.registry.task_registry import task_registry; import torch; print('ok')"
```

`ok`가 나오면 성공.

## 7. 학습 실행 (짧은 래퍼 — 4 GB VRAM 기준)

```bash
conda activate aerialgym
cd ~/workspaces/aerial_gym_ws/src/aerial_gym_simulator/aerial_gym/rl_training/rl_games

# 4 GB VRAM 은 GPU4GB=1 프리셋으로 한 줄 (헤드리스·warp·PYTHONNOUSERSITE·로그 전부 내장):
GPU4GB=1 ./train_navrl.sh
```

> **왜 `NUM_ENVS` 만으로는 안 되나 (4 GB 함정):** NUM_ENVS 를 낮춰도 두 가지가 남아 학습이 안 뜬다 —
> ① Isaac Gym 의 PhysX **GPU 버퍼**(`base_sim` 의 `max_gpu_contact_pairs=2**24`, `buffer_multiplier=10`)는
> env 수와 무관하게 8000-env 용으로 고정 할당돼 4 GB 를 넘겨 `GymPhysX.cpp: out of memory` 로 죽는다.
> ② rl_games 는 `minibatch_size ≤ horizon_length(32)×num_envs` 를 요구하는데 `runner.py` 가 자동 축소를 안 해
> 기본 `minibatch_size=4096` 은 num_envs≥128 에서만 유효하다. **`GPU4GB=1` 이 셋을 한 번에 처리한다**:
> PhysX 버퍼 축소(`base_sim_4gb`, `AERIAL_GYM_SIM_NAME` 로 선택) + `ppo_navrl_cnn_4gb.yaml`(minibatch 512) + `NUM_ENVS=32`.

- 로그는 `train_session_logs/`에, 체크포인트는 이 폴더의 `runs/ppo_XXXX_navrl/`에 자동 저장.
- 이어하기: `GPU4GB=1 ./train_navrl.sh --checkpoint runs/ppo_XXXX_navrl/nn/gen_ppo.pth`
- 활성화된 conda의 `python`을 씀. 다른 파이썬이면 `PYTHON=/path/to/python GPU4GB=1 ./train_navrl.sh`.
- **다른 터미널에서 VRAM 감시**: `watch -n 2 nvidia-smi`
  - 여유가 있으면 `GPU4GB=1 NUM_ENVS=48 ./train_navrl.sh` 처럼 `NUM_ENVS` 를 16 의 배수로 올린다(minibatch 512 가 나눠떨어지게).
  - `out of memory` / `PxgCudaDeviceMemoryAllocator fail` 이 뜨면 `NUM_ENVS` 를 더 낮춘다(예: 16).
- 8 GB 메인 머신은 `GPU4GB` 없이 기존대로 `NUM_ENVS=256 ./train_navrl.sh` (base_sim + minibatch 4096).
- 저사양 GPU는 메인보다 느리다 — 같은 epoch 수라도 오래 걸리니 밤샘 학습용으로.

## 8. (선택) A/B 리워드 비교

- **메인(3070)**: 새 리워드 (`alive_weight = -0.05`, `capture_bonus = 30.0`).
- **보조(1650 Ti)**: 기존 리워드로 대조군 — `aerial_gym/config/task_config/navrl_task_config.py`에서
  `alive_weight`를 `1.0`, `capture_bonus`를 `10.0`으로 되돌린 뒤 학습.

두 머신을 나눠 쓰면 GPU 하나에 두 학습을 욱여넣지 않고 OOM 없이 동시에 비교할 수 있다.

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

---

## 요약 표

| 항목 | 내용 |
|------|------|
| Isaac Gym 다운로드 | NVIDIA 계정 필요, 자동화 불가 (2번) |
| Python | **3.8** 고정 (Isaac Gym Preview 4 요구) |
| 최신 코드 | 이미 클론돼 있으면 실행 전 `git pull` (3번) |
| 학습 실행 | `GPU4GB=1 ./train_navrl.sh` (7번) |
| VRAM 4 GB | `GPU4GB=1` 프리셋(PhysX 버퍼↓ + minibatch↓ + NUM_ENVS=32), `headless` 내장 |
| urdfpy | git 미포함 → 메인 머신에서 복사 (3-b) |
| 결과 회수 | `runs/`는 git 제외 → **rsync**로 메인에 회수 (9번) |
