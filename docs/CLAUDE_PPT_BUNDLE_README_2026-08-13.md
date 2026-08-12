# Claude PPT 검수 번들 안내

작성일: 2026-08-13

## Claude에 전달하는 방법

1. 이 번들의 `01_CLAUDE_PPT_REVIEW_REQUEST.md`를 먼저 읽게 한다.
2. 나머지 파일은 주장과 수치의 근거 자료로 사용하게 한다.
3. Claude 대화에 현재 PPT/PPTX가 없다면 사용자가 가진 PPT 원본을 이 ZIP과 별도로 첨부한다.
4. ref5in P1a/P1b는 strict FAIL이고 P1c가 진행 중이다. held-out P2와 full 5B `[미실행]` 자리를
   P1의 on-policy 수치로 채우거나 PASS로 추정하면 안 된다.
5. 기준 플랫폼은 실제 5B 결과의 `robot_name`, URDF SHA, config SHA를 확인한 뒤에만 확정한다.

## 파일 구성 (총 11개)

| 번호 | 파일 | 용도 |
|---:|---|---|
| 01 | `01_CLAUDE_PPT_REVIEW_REQUEST.md` | Claude에 그대로 전달할 최종 검수·보완 요청서 |
| 02 | `02_CODEX_FINAL_REVIEW.md` | 검증 1~5A의 위험 주장, 정정 사항, 판정 근거 |
| 03 | `03_V5A_SUMMARY.md` | 검증 5A 사람이 읽는 결과 요약 |
| 04 | `04_V5A_SUMMARY.json` | 검증 5A 원시 요약 수치 |
| 05 | `05_EXISTING_PPT_BRIEF.md` | 기존 PPT 구성과 역사적 수치; 최신 문서와 충돌하면 01~04 우선 |
| 06 | `06_DEVELOPMENT_DIRECTIONS.md` | 후속 연구 로드맵과 단계별 gate |
| 07 | `07_REFERENCE_PLATFORM_PROPOSAL.md` | `navrl_ref5in_quad` 설계와 한계 |
| 08 | `08_REFERENCE_PLATFORM_VERIFICATION.md` | 후보 기체 CPU 정합성·canonical same-controller simulator gate 검증 요약 |
| 09 | `09_REFERENCE_PLATFORM_FLIGHT_ENVELOPE.json` | schema-2 canonical simulator gate 원자료 |
| 10 | `10_WORKLOG.md` | 전체 연구 이력과 변경 근거 |
| 11 | `11_BUNDLE_README.md` | 이 안내문 |

## 우선순위와 금지사항

- 사실 우선순위: `01` > `02` > `03`/`04` > `06`~`09` > `05` > 과거 WORKLOG 서술.
- `05_EXISTING_PPT_BRIEF.md`의 과거 주장이 최신 정정과 충돌하면 반드시 교체한다.
- 5A smoke는 장기 학습 성능 증명이 아니다.
- ref5in은 5인치급 부품 자료를 참고한 **hardware-informed simulation candidate**이며, 제작 가능한
  reference platform으로 검증된 기체가 아니다.
- CPU repository consistency는 **26/26 PASS**, **canonical same-controller simulator gate는
  21/21 PASS**이고 legacy/ref5in 각각 **16/16 env survival**을 확인했다. P1a/P1b는 on-policy
  learning outcome을 관측했지만 전체 gate는 FAIL했다. held-out capture·장기 재현성·buildability를
  증명하지 않는다.
- legacy와 ref5in 결과를 하나의 연속 학습 곡선처럼 연결하지 않는다.
- 5B가 끝나기 전에는 결과 숫자를 창작하지 않고 조건부 슬라이드/placeholder로 유지한다.

## 권장 첫 메시지

> 첨부 ZIP의 `01_CLAUDE_PPT_REVIEW_REQUEST.md`를 최우선 지시문으로 읽고, 나머지 파일로 모든 숫자와 주장을 교차검증해 주세요. P1a/P1b FAIL과 P1c/P2/P3 상태를 구분하고, 검증 5B 미실행 값은 절대 추정하지 말고 placeholder로 남겨 주세요. 기존 PPT가 첨부되어 있다면 수정안과 슬라이드별 교체 지시를 만들고, 없다면 17장 본문 구조의 새 PPT 원고를 작성해 주세요.
