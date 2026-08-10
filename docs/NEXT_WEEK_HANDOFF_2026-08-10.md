# Next-week handoff — 2026-08-10

## 현재 동결 상태

- PPO 재학습 금지. 남은 navigation 작업은 frozen-policy evaluation이다.
- 현재 학습·평가 프로세스 없음. RTX 3070은 유휴 상태다.
- current frozen candidate:
  - `aerial_gym/rl_training/rl_games/runs/ppo_260805_0413_navrl_v2-speedgov-ep24000-205bars-main-riskcap-s1/nn/last_gen_ppo_ep_25000_rew_39.742134.pth`
  - SHA-256 `f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40`
- source policy for governor/adaptation decomposition:
  - `aerial_gym/rl_training/rl_games/runs/ppo_260802_0020_navrl_v2-recover-curriculum-continue-s1/nn/last_gen_ppo_ep_24000_rew_44.73549.pth`
  - SHA-256 `82f7978b42d9d9e95adcc638a40ae85fb3736fd897ae39cb8aa8333be39cf23f`
- current detector artifact is diagnostic-only, not a final detector:
  - `artifacts/navrl_target_detector_v1.pth`
  - SHA-256 `15cb90e90615e071c938379acc6c445d3c56f73303e9b6ce915ca7fd695cf492`

## 이번 주 종료 조건

1. `docs/codex_review_2026-08-10.md`의 검수 결과를 Claude가 원 발표 문서와 dashboard generator에 반영한다.
2. 다음 표현을 현재 결론에서 내린다.
   - `riskcap gain grows with density`
   - 학습범위에서 speed×density interaction이 확정됐다는 문장
   - v1/v2 endpoint slope `2.98 vs 2.02` 비교
   - H4 `69% decomposition succeeded` 확정 표현
3. dashboard의 post-fix legacy 500-epoch speed pilot은 archive에는 남기되 current headline fallback에서 뺀다.
4. 수정 뒤 status snapshot을 다시 생성하고 Codex가 숫자·라벨·평가 계약을 재검수한다.
5. audit 문서와 수정된 문서가 함께 있는 한 커밋으로 저장·push한다.

이번 주에는 새 GPU 평가, PPO 재학습, dropout 추가 A/B를 하지 않는다.

## 다음 주 재개 순서

### Gate 0 — 종료·bootstrap·평가 provenance 고정 (구현 완료, 테스트 후 동결)

최종 감사에서 두 학습 의미 오류가 발견됐다.

- 환경은 `infos["timeouts"]`만 내보냈지만 rl_games의 `value_bootstrap=True`는 정확히
  `infos["time_outs"]`를 요구했다. 따라서 동결 checkpoint 학습 중 time-limit truncation은 critic에서
  true terminal처럼 처리됐다.
- `sim_steps > 600` 때문에 설정상 600-step episode가 실제로는 action 601에서 끝났다. 저장된 과거
  결과의 timeout outcome step이 전부 601인 것으로 재확인했다.

현재 source schema v2는 action 600에서 `>=` 종료하고 두 key를 함께 내보낸다. 평가기는 timeout이
존재하면 outcome-step mean/p10/p50/p90가 모두 정확히 600인지 검증한다. 또한 checkpoint snapshot,
top-level evaluator, 모든 runtime source snapshot, git dirty state, Python/pip environment manifest를
receipt에 묶는다. source 원본이 셀 사이에 바뀌어도 거부한다.

**동결 정책 자체는 과거 의미로 학습됐으므로 소급 수정되지 않는다.** 기존 성능은 탐색/정책 비교
출발점으로 보존하지만 schema-v2 결과와 한 표에서 직접 결합하지 않는다. 이 source로 PPO continuation도
하지 않는다. 다음 PPO가 필요해지면 4-D legacy action schema를 유지할지 별도 결정한다. z output은
altitude PI에 덮여 직접 actuator authority는 없지만 raw 값이 다음 `prev_action` 관측에 남으므로 dead
dimension은 아니다. 3-D actor 실험은 이 간접 메모리 채널을 무엇으로 대체할지도 함께 설계해야 한다.

### Gate 1 — governor와 adaptation 밀도별 분리

과거 C arm도 601-action/source-provenance semantics라 재사용하지 않는다. **미사용 seed 하나**에서
다음 15 cells를 같은 schema-v2 source manifest로 모두 평가한다.

- A: ep24000 / governor off × bars 130, 160, 190, 205, 220
- B: ep24000 / riskcap × bars 130, 160, 190, 205, 220
- C: ep25000 / riskcap × bars 130, 160, 190, 205, 220

비교는 `B−A = governor sequential contribution`, `C−B = adaptation sequential contribution`으로 제한한다.
220은 OOD로 별도 표시한다. ep25000/off는 interaction까지 물을 때만 추가한다. 예상 GPU 시간은
기존 5-cell sweep 기준 약 1.5–2.5시간이며 첫 1셀 wall-clock으로 다시 산정한다.

15셀 전용 launcher가 구현됐다. seed 53, 2,049 requested episodes/cell, deterministic/original,
exact-600 schema-v2를 내부에서 고정하며 한 shared source bundle을 모든 cell이 재사용한다. GPU 동시 실행이
아니라 A 5셀 → B 5셀 → C 5셀 순차 실행이고, 완료된 cell은 재실행 시 skip한다.

```bash
cd /home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/aerial_gym/rl_training/rl_games
./eval_navrl_v2_governor_adaptation_abc.sh
```

통합 로그는 `results/navrl_v2_governor_adaptation_abc_seed53_schema2/campaign.log`, 최종 표와 원자료 계약은
같은 폴더의 `summary.md`와 `summary.json`에 생성된다. partial cell directory는 자동 삭제/덮어쓰기하지 않고
명시적으로 중단한다.

### Gate 2 — in-distribution speed interaction 재검정

primary grid는 0.3/1.5 m/s × 130/160/190/205 bars × 미사용 seed 2개 = 16 cells다.
primary statistic은 bars≤205의 aggregate-binomial density×speed interaction 하나로 사전등록한다.
220막대는 primary test에서 제외하고 OOD 보조자료로만 둔다. 예상 GPU 시간은 약 1.5–2시간이다.

interaction이 재현되지 않으면 논문 결론은 “학습 격자에서 density main effect가 speed main effect보다
크다”로 종료한다. 재현되면 그때 trajectory mediation을 별도 평가한다.

### Gate 3 — learned detector offline gate

PPO는 고정한다. 먼저 detector 데이터/평가만 고친다.

- full camera bearing와 2–20 m range stratification
- target-absent, full/partial occlusion, small-pixel cases
- train/validation/test seed 분리
- positive pixel 0.054% 불균형을 반영한 weighted BCE 또는 focal/Dice 후보
- pixel PR/IoU만이 아니라 frame detection recall, false positive rate, bearing/range error, range-bin recall
- validation에서 threshold를 고정한 뒤 test는 한 번만 확인

offline gate를 통과한 artifact만 frozen ep25000+riskcap policy에 넣어 analytic-vs-learned navigation
평가를 한다. 이는 PPO 재학습이 아니라 detector 지도학습 + frozen-policy evaluation이다.

## 중단한 가지

- H2/H3/H4/dropout 내부 채널 추가 분해: 종료. H2는 작은 효과가 재현됐지만 채택 gate 4 pp 미달이다.
- latency P0/P1/P2 추가 탐색: 종료. P3 결과는 timestamp/pose-history 전제를 붙여 사용한다.
- 새 PPO curriculum/action/representation 실험: detector와 평가 confound가 정리될 때까지 금지한다.

## 재개 첫 확인 파일

1. `docs/codex_review_2026-08-10.md`
2. `docs/review_brief_2026-08-10.md`
3. `docs/midterm_summary_2026-08.md`
4. `WORKLOG.md`의 2026-08-10 Codex 독립 검수 항목
5. 이 handoff 문서
