# 학습 결과 옮기기 — 보조머신(4GB) ↔ 집 3070 [상세 가이드]

> 경로 주의: 이 가이드는 보조 머신의 repo 를 `~/MOTAR` 로 표기한다. `SETUP_SECOND_MACHINE.md` §3 대로
> `~/workspaces/aerial_gym_ws/src/aerial_gym_simulator` 에 클론했다면 경로를 그에 맞게 바꿔 읽는다.
보조 학습 머신(GTX 1650, 4GB)에서 돌린 학습 결과를 집 메인 머신(RTX 3070)으로 모아
TensorBoard로 같이 비교하거나 정책을 재생하기 위한 전송 방법 정리다. 코드를 몰라도
그대로 복붙하면 되게 적어둔다.

> **왜 git 으로는 안 되나?** `runs/`, `nn/`, `*.pth`, `*.tfevents*` 는 `.gitignore` 대상이라
> `git push`/`pull` 로는 **절대 안 넘어간다**. 그래서 아래처럼 직접 옮겨야 한다.

---

## 0. 요약 (TL;DR)

| 상황 | 방법 |
|------|------|
| 집컴이 **다른 네트워크**(사설 IP) — 평소 RustDesk로 접속 | **방법 A: RustDesk 파일 전송** (기본) |
| 두 컴이 **같은 와이파이** 거나, 집컴이 **공개 주소**(도메인) | **방법 B: rsync** (제일 편함) |
| 네트워크가 아예 안 될 때 | **방법 C: USB / 클라우드** |

- **TensorBoard 곡선 비교만** 하려면 → `summaries/` 폴더만 옮기면 됨 (**수 MB, 가벼움**).
- **정책 재생·이어학습**까지 하려면 → run 폴더 **통째로**(= `nn/` 체크포인트 포함, **수백 MB**).

---

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

`YYMMDD_HHMM` 은 학습 **시작 시각**이다. 실행할 때 콘솔에 `run folder : ppo_XXXX_navrl` 로 찍히고,
`train_session_logs/train_XXXX.log` 로그 파일과 짝을 이룬다.

---

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

---

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

## 방법 B — rsync  (같은 와이파이 이거나, 집컴이 공개 주소일 때 · 제일 편함)

한 줄이면 되고 압축·드래그가 없다. **단, 두 컴이 서로 네트워크로 닿아야 한다:**
- 같은 와이파이/랜 → 상대의 **사설 IP**(예: `192.168.0.x`) 로.
- 상대가 **공개 주소**(예: `이름.kaist.ac.kr`) 거나 포트포워딩/VPN 이 돼 있으면 → 어디서든.

**보조머신 정보**(rsync 에 쓸 값): 사용자 `joshuali`, IP 확인은 `hostname -I`.

**3070 에서 실행(= 당겨오기, pull):**
```bash
cd ~/<3070의 MOTAR 경로>/aerial_gym/rl_training/rl_games
SRC=joshuali@<보조머신_IP>:~/MOTAR/aerial_gym/rl_training/rl_games/runs

# 곡선만(가볍게): --exclude 'nn/'
rsync -avz --exclude 'nn/' -e ssh $SRC/ppo_260716_0357_navrl runs/   # 밀도25
rsync -avz --exclude 'nn/' -e ssh $SRC/ppo_260716_1250_navrl runs/   # 밀도50
tensorboard --logdir runs
```
**보조머신에서 실행(= 밀어넣기, push)** 하려면 방향만 반대로:
```bash
DST=joshuali@<3070_주소>:~/<3070경로>/aerial_gym/rl_training/rl_games/runs
rsync -avz --exclude 'nn/' -e ssh ~/MOTAR/aerial_gym/rl_training/rl_games/runs/ppo_260716_0357_navrl $DST/
```
> 범례 깔끔하게: 옮긴 뒤 `mv runs/ppo_260716_0357_navrl runs/density_25` 처럼 폴더명만 바꾸면 된다.

---

## 방법 C — USB / 클라우드 (백업 수단)

- **USB**: 위 A-1 로 만든 `.tar.gz` 를 USB 에 복사 → 3070 에 꽂아 A-3 로 풀기.
- **클라우드**: 곡선 파일은 수 MB 라 Google Drive/카톡/메일로도 충분. 3070 에서 내려받아 A-3 로 풀기.

---

## 자주 겪는 문제 (트러블슈팅)

- **TensorBoard 에 곡선이 안 뜬다** → 그 run 폴더 안에 `summaries/events.out.tfevents...` 파일이 있는지 확인.
  없으면 학습이 몇 epoch 못 돌고 죽은 것(로그 확인).
- **범례가 `ppo_260716_0357_navrl` 처럼 지저분** → 폴더명을 `density_25` 등으로 `mv` 해서 바꾸면 됨.
- **밀도 비교인데 곡선이 안 맞는 느낌** → run 들이 **같은 코드 버전 + 같은 env 수 + 같은 배치방식(랜덤 vs 격자)**
  인지 확인. (구버전 run 은 bar 가 "jittered 7×7 격자" 였고, 현재는 랜덤 산포다. 섞으면 밀도만의 비교가 아님.)
- **학습 자체가 `CUDA Error 804` / `nvidia-smi: Driver/library version mismatch` 로 안 돈다**
  (전송과는 무관하지만 자주 겪음) → 백그라운드 드라이버 자동업데이트 후 커널 모듈이 안 맞는 것. **재부팅**하면 해결.
  재부팅 후 `nvidia-smi` 가 정상 표를 보이면 OK.

---

## 관련 문서
- `SETUP_SECOND_MACHINE.md` — 보조머신 셋업 + §9 결과 회수(rsync).
- 4GB 학습 실행: `GPU4GB=1 [NAVRL_MAX_BARS=N NAVRL_NUM_BARS=N] NUM_ENVS=512 ./train_navrl.sh`
  (`aerial_gym/rl_training/rl_games/` 에서).
