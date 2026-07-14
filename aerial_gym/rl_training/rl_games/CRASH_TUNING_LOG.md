# Crash-reduction tuning log — Phase 1 (2026-07-14)

목표: crash ~14%(먼 18m 목표로 가는 transit 중 막대 충돌)를 captured 손실 없이 줄이기.
고정 변수: safety_static_weight 1.5, 아레나 24×24, 기둥 48, 커리큘럼 18m, episode 250, num_envs 256, 6000 epoch.
지표는 마지막 500 epoch 평균(안정 구간). 기준 = captured↑ / crash↓ / timeout 0 유지.

## Baseline — `ppo_260714_0153` (clearance OFF)
- captured **0.863**, crash **0.137**, timeout 0.0
- 참고: `ppo_260713_2210`(safety 1.0)은 captured 0.86 / crash 0.14 → **safety 가중치 1.0→1.5는 crash에 무효**.

## Run B — `ppo_260714_0346` (clearance 거리 모드, weight 1.5, margin 0.6, speed-gate OFF)
- captured **0.861**, crash **0.139**, timeout 0.0, closest(no-crash) 0.406, best 0.222
- **판정: crash 저감 실패** (0.139 vs baseline 0.137, 사실상 동일).
- 원인: 페널티가 속도 유인 대비 약함 — 막대 0.3m 앞에서 `1.5×(0.6−0.3)=0.45/step` < 속도 리워드 +2. 스쳐도 순이득이라 회피 유인 부족.

## Run C — `ppo_260714_0555` (clearance speed-gated, weight 1.5, margin 0.6, speed-gate ON)
- captured **0.861**, crash **0.138**, timeout 0.002, closest(no-crash) 0.408, best 0.239
- **판정: crash 저감 실패** (0.138 vs baseline 0.137). 속도-게이트로 페널티가 2배(0.9/step) 세졌지만 여전히 속도 리워드 +2보다 작아 무효.

## 결론 (세 run 종합)

| run | 변경 | captured | crash | timeout |
|---|---|---|---|---|
| 2210 | safety 1.0 | 0.860 | 0.140 | 0.0 |
| 0153 (baseline) | safety 1.5 | 0.863 | 0.137 | 0.0 |
| B (0346) | + clearance 거리 1.5 | 0.861 | 0.139 | 0.0 |
| C (0555) | + clearance speed-gated 1.5 | 0.861 | 0.138 | 0.002 |

**모두 동일 (captured ~0.86 / crash ~0.14).** "장애물 근접 페널티"는 가중치 1.5로는 어떤 형태(soft log / hard 거리 / hard 속도게이트)든 crash를 못 움직임 — **페널티가 속도 유인(+2/step)보다 약한 게 근본 원인.**

**중요 관찰**: captured가 초기(커리큘럼 ~5m 근거리)엔 0.97(crash 3%), full 18m에선 0.86(crash 14%). → **crash는 먼 목표로 가는 긴 transit(48기둥 통과)에 집중**. 거리×밀도가 만드는 hard tail일 수 있음.

**다음 후보** (블라인드 리워드 튜닝은 3run으로 소진 — 이제 LOOK):
1. **뷰어로 14% 실패 재확인** (수분) — "돌진 충돌"인지 "좁은 틈 갇힘"인지. 처방이 갈림.
2. 돌진이면 → clearance_weight 4~5 + margin 1.0 (훨씬 강하게) 한 판.
3. 갇힘/hard-geometry면 → max_velocity↓(반응시간) 또는 LiDAR range↑, 또는 수용.
4. **86% 수용** — NavRL hybrid 80.96% 상회, Phase 1 검증 목적 달성. Phase 2로.

clearance는 무효로 판명 → config를 baseline(off)로 복원함.

---

## ⚠️ 위 3run(0153/0346/0555)은 옛 스킴 결과 — 새 스킴에선 전제가 달라짐 (2026-07-14)

위 실험들은 **옛 스폰 스킴**(목표가 드론 근처에 몰려 막대밭 관통이 8%뿐) + **0.05m 구 충돌체**에서 나온
결과다. 그래서 "clearance 무효"의 진짜 원인은 두 가지가 섞여 있었다: (a) 막대가 애초에 거의 무관했고,
(b) 페널티(cw 1.5)가 속도 유인(+2/step)보다 약했다. **(a)는 새 스킴(매 에피소드 48기둥 관통 + 0.28m 박스,
run 1904)에서 사라졌지만 (b)는 구조적 사실로 남는다.** 따라서 clearance를 재시도하되, (b)를 정면으로
깨는 강도로 건다.

## Run D (예정) — `ppo_2607??_????` : 새 스킴 + speed-gated clearance **cw=6.0, margin=0.5**

- **베이스라인 = 1904** (새 스킴, 축정렬 48기둥, 0.28m 박스): captured 0.65 / crash 0.35 / timeout 0.
- **바꾼 것(1레버, config 3값, 코드변경 없음)**: `clearance_speed_gated False→True`,
  `clearance_weight 0.0→6.0`, `clearance_margin 0.6→0.5`. 그 외(safety 1.5, 커리큘럼, episode 300,
  256 envs, 6000 epoch) 전부 동일 → 결과가 이 레버에 깔끔히 귀속.
- **설계 근거(멀티에이전트 워크플로우: 진단 3렌즈 → 설계 4 → 반박검증 3/4 생존 → 종합)**:
  - **실패모드 = "순항속도로 막대 스치기"(charging/shave-at-cruise-speed)** — 3개 진단 렌즈(리워드수학·
    기하·지각제어)가 모두 dominant로 지목. 드론은 막대를 **보고도** 스친다(closest_nocrash 0.43m가 증거):
    스쳐도 매 step 순이득(+1.2/step)이라 감속·중앙정렬 유인이 없음.
  - **왜 cw=6인가**: d=0.30m 스침·v=2에서 페널티 `6*relu(0.5-0.30)*2 = 2.40/step > 속도리워드 2.0`.
    즉 유효 속도계수 `(1 - 6*relu(0.5-d))`가 d<0.333m에서 **음수** → "빠르게 막대로" 가 처벌됨(단순 상쇄가
    아니라 부호반전). 실패한 cw=1.5의 약 4배(2.40 vs 0.75).
  - **왜 margin=0.5인가**: 최악 갭(0.8m 축정렬 기둥 2개 @1.8m) 자유폭 ~1.0m → 중앙통과 min_dist ~0.5m →
    `relu(0.5-0.5)=0` → **정상 통과는 비용 0(1904와 동일)**. 벗어난/스치는 통과(d<0.5)만 과세.
  - **왜 speed-gated인가**: 페널티가 |v|에 비례 → 좁은 갭도 **감속하면 통과 가능**(회피·정지 아님).
    거리모드였다면 좁은 갭에서 무조건 감점 → detour/timeout 위험.
- **예상**: crash 0.35 → **~0.15-0.22**(점추정 0.18), captured ~0.60-0.70 유지(정상통과 경제성 불변),
  timeout ~0.01-0.03(300 cap 훨씬 아래). **판정 = crash↓ AND captured 유지(둘 다)**.
- **LOOK-first 생략 근거**: 3렌즈가 실패모드에 수렴 + d≤0.30 위험구간에서 부호반전이 robust. 다만 기하
  렌즈는 corner-clip(축정렬 기하 바닥)도 co-dominant로 봄 → crash가 0.18까지 안 내려가고 ~0.25에서
  멎으면 그 잔여분은 reward가 아닌 기하/지각 한계 신호(다음 레버: 속도governor 또는 LiDAR 해상도↑).
- **폴백**: captured가 5pt↑ 하락 → margin 0.5→0.45(cw는 5.0 밑으로 내리지 말 것). timeout>0.05 →
  거리모드 1/d 배리어(cw=2.0, margin=0.45, speed-gate OFF)로 전환.
