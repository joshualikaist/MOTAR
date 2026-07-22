# MOTAR / NavRL — 작업 규칙 (매 세션 로드)

## 작업 방식 — 스킬 먼저, 무거운 조사는 서브에이전트 (토큰 절약)

substantive한 요청(학습·평가·분석·감사·구현)을 받으면:

1. **스킬 먼저 탐색·호출한다.** 연구 루프(train → eval → record)는 `/navrl` 스킬로 시작한다
   (`.claude/skills/navrl/`). 대시보드 갱신은 저장소 `research-status` 스킬.
2. **무거운 조사는 서브에이전트(Agent 도구)에 위임한다** — 파일 다수 읽기, `runs/**/epoch_metrics.csv`
   파싱, `nn/*.pth` 로드, 긴 학습 로그 스캔. 메인 루프는 짧은 구조화 보고만 회수한다.
   독립 조사는 **한 메시지에 여러 Agent**로 병렬 실행. 서브에이전트 `*.output` 트랜스크립트는
   **절대 셸로 읽지 말 것**(컨텍스트 폭발). 서브에이전트가 세션/주간 한도로 죽으면 read-only 직접
   조사로 폴백하거나 리셋 후 재개.
3. **메인 컨텍스트에 `runs/**/*.csv`·`nn/*.pth`·긴 로그를 직접 로드하지 않는다.**

## 프로젝트 현재 상태 (재도출 금지 — 여기 요약)

- **주제**: 센서 전용 UAV 요격. actor 관측에 GT 표적(semantic id/mask, bearing/range, `target_position`,
  GT visibility) **절대 금지** — detector supervision·reward·종료판정·critic·평가 metric에만 사용.
- **현재 방향(2026-07-22 피벗)**: NavRL++식 **학습형 인지(RGB-D 카메라 + LiDAR detector/tracker) +
  Transformer 시계열 정책**. 계획서 = `PERCEPTION_TRANSFORMER_PLAN.md`. 해석적 semantic id 방식(관측
  305차원)은 이제 **프로토타입/baseline**. 현재 vision 관측 = 1265차원(카메라 depth 포함).
- **canonical 현재 상태**: `WORKLOG.md`(맨 아래) + `docs/status/` 라이브 대시보드(status.json 구동).
  `RESEARCH_PLAN.md`는 charter(가끔 갱신). ROADMAP/PHASE3_PLAN은 위상 번호가 엇갈리니 WORKLOG 기준.
- **하드웨어**: RTX 3070 8GB(학습 공장) + GTX 1650 Ti 4GB(평가 공장 — 학습은 N=128이라 ~10-15pt 약함,
  결과 섞지 말 것). RTX 50번대(Blackwell)는 Isaac Gym Preview4와 비호환 — 사지 말 것.

## 핵심 함정 (재학습 금지)

- **밀도 커리큘럼 run 평가는 `last_gen_ppo_ep_XXXX.pth`로.** `gen_ppo.pth`(best-reward)는 저밀도 정책이라
  고밀도 평가 시 ~15%로 오독됨.
- warm-start: `--checkpoint <p> --max_epochs <N>` (runner가 critic `_orig_mod` 자동 정규화 + max_epochs override).
- 커리큘럼 knob은 전부 env-var: 밀도 `NAVRL_DENSITY_*`, 거리 `NAVRL_K_*`. 센서 전용은 `NAVRL_DENSITY_THRESHOLD=0.6`.

## 커밋 규칙
- **커밋/푸시 전 사용자가 diff 검토·승인.** 자율 커밋 지양. 브랜치 `research/navrl-env`, 원격 joshualikaist/MOTAR.
- `runs/`(4GB+)·`nn/`·tfevents·로그는 gitignore. 결과는 `results/<sweep>.csv`로 소량만 추적.
- 여러 클로드/커서 세션이 동시에 작업할 수 있음 — 미커밋 변경이 남의 WIP일 수 있으니 함부로 커밋하지 말 것.
