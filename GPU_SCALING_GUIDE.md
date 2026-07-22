# GPU 스케일링 / 이전 가이드 (환경 바꿀 때 참고)

작성: 2026-07-17, 연구 방향 업데이트: 2026-07-22. 대상: MOTAR dual-sensor detector/tracker +
NavRL++-Target temporal policy 파이프라인.
목적: **더 좋은 GPU를 구하거나 클라우드로 옮길 때 무엇을 어떻게 바꿔야 하는지**를 한 곳에 정리.

---

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

---

## 1. 이 프로젝트에서 GPU가 좌우하는 것 두 가지

1. **VRAM** — 얼마나 큰 실험이 "메모리에 들어가느냐" (binding constraint).
2. **연산 속도** — run 하나가 몇 시간 걸리느냐.

바꿀 수 있는 손잡이(knob)는 아래 §2. GPU 등급별 권장 세팅은 §3.

---

## 2. 조절 손잡이 (knobs) — 파일·환경변수

| knob | 위치 | 역할 | VRAM/속도 영향 |
|---|---|---|---|
| `NUM_ENVS` | `train_navrl.sh` (기본 256) | 병렬 환경 수 | ↑ = VRAM·처리량 ↑ |
| `minibatch_size` | `ppo_navrl_cnn*.yaml` (4096) | PPO 미니배치 | **제약: horizon(32)×NUM_ENVS ≥ minibatch → NUM_ENVS ≥ 128** |
| `horizon_length` | `ppo_navrl_cnn*.yaml` (32) | 롤아웃 길이 | ↑ = VRAM(버퍼) ↑ |
| 카메라 해상도 | `PERCEPTION_TRANSFORMER_PLAN.md` 초기값 160×96 RGB-D | offline detector | ↑ = detector/dataset 메모리 ↑↑ |
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

## 4. 클라우드/새 머신 세팅 체크리스트

새 (Ampere/Ada) 머신에서 이 리포를 돌리려면:

1. **드라이버/CUDA**: NVIDIA 드라이버 + CUDA 11.6~11.8 (Isaac Gym Preview 4 호환). Blackwell 아님 확인.
2. **conda 환경**: `aerialgym` (Python 3.8, PyTorch cu117 계열). `PYTHONNOUSERSITE=1` 필수
   (AirSim user-site numpy 가림 방지 — activate.d 스크립트).
3. **Isaac Gym Preview 4** 설치 (NVIDIA에서 별도 다운로드, `pip install -e isaacgym/python`).
4. **이 리포** `pip install -e .` (aerial_gym).
5. **스모크로 검증** (학습 전 필수):
   ```bash
   PYTHONNOUSERSITE=1 python tools/test_navrl_p3_smoke.py          # 정적/주입 경로 (156 관측)
   NAVRL_VISION=1 python tools/test_navrl_p3_stage0.py             # semantic 센서 prototype 회귀
   python tools/test_navrl_p3_math.py                              # 리워드 수학
   ```
6. **VRAM 헤드룸 확인**: 첫 학습 몇 스텝에서 `nvidia-smi`로 최대 사용량 보고 NUM_ENVS 조정.

---

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

## 6. 한 줄 요약

- **지금(3070 8GB)**: dual detector와 NavRL++-style 17-token policy 개발 가능.
- **하나 산다면**: 중고 **RTX 3090 24GB** (Isaac Gym 100% 호환, 가성비 최고).
- **급하면**: 최종 스윕만 클라우드 **A5000/4090/L40S** 대여($100대).
- **절대 금물(지금)**: RTX 50번대(Blackwell) — 시뮬레이터가 안 돌아갈 위험.
- **바꿀 때 손잡이**: `NUM_ENVS`(+minibatch 정합), 카메라 해상도, 네트워크 폭. §3 복붙.
