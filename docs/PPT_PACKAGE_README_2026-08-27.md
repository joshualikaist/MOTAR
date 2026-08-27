# MOTAR PPT 전달 패키지 사용 설명서

이 파일을 Claude에게 먼저 전달하고, 같은 ZIP의 나머지 파일을 근거 자료로 사용하게 한다. 이전 PPT의
문구나 숫자는 권위 있는 자료가 아니다. 아래 계보표와 원자료 경로가 충돌하면 계보표와 원자료를 따른다.

## Claude에게 그대로 보낼 요청

> 이 ZIP으로 교수님 발표용 PPT를 다시 작성해 주세요. `CLAUDE_PPT_MASTER_BRIEF_2026-08-26.md`를
> 슬라이드 구성의 최상위 명세로 사용하고, 이 파일의 lineage/density 표를 숫자 계약으로 사용하세요.
> 이전 PPT에서 복사한 숫자, 파일에 없는 수치, 추정한 DOI·저자·실험 결과를 만들지 마세요. 모든 그래프와
> 표에는 historical / corrected-v2 / NOT RUN 중 하나를 명시하고, speaker note에 원자료 경로를 적으세요.
> 본 발표의 결론은 simulation-only입니다. 실제 기체는 미조립, real sensor log 0, flight 0입니다.

## 반드시 지켜야 할 계보 구분

| 계보 | 환경/기하 | 결과 사용 규칙 |
|---|---|---|
| Historical v1 | 24×24×3 m, 478 m² | superseded; 주 결과 곡선에 사용 금지 |
| Historical v2 (`navrl_band`) | 40×40×3 m, 1,600 m², overlap-permitting | 기존 capture/crash/timeout은 이 계보로만 표시 |
| Corrected fresh v2 | 40×40×3 m, `navrl_ref5in_v2_quad`, 0.283 m proxy, footprint-aware non-overlap, surface clearance 0.45 m, merge fallback 0 | airframe/engineering gate만 PASS; PPO 성능은 **NOT RUN** |

새 계보의 실행 계약은 `70→205 bars`(step 15, level당 minimum dwell 1,000 epochs)이다.
2026-08-27 기하 감사에서 body+tracking 팽창 기준 205는 PASS, 250까지 연결, **300은 단절
FAIL**이다. 따라서 300을 “연결 OOD”로 쓰지 말고 disconnected-stress로만 표시한다.
“300 bars에서 학습했다”라고 쓰지 않는다. 과거 v2의 `70→300` 표기는 historical launcher
기록으로 남기되 새 계보와 연결하지 않는다.

## 밀도 숫자 계약

MOTAR arena는 1,600 m²이므로 `bars / 16 = bars per 100 m²`이다. 권장 표/평가 셀은
`64, 70, 100, 130, 160, 190, 205, 220, 250, 300 bars`이다.

| bars | 개수/100 m² | 평균 footprint 점유율* |
|---:|---:|---:|
| 64 | 4.00 | 1.461% |
| 70 | 4.375 | 1.598% |
| 100 | 6.25 | 2.283% |
| 130 | 8.125 | 2.968% |
| 160 | 10.00 | 3.653% |
| 190 | 11.875 | 4.339% |
| 205 | 12.8125 | 4.681% |
| 220 | 13.75 | 5.023% |
| 250 | 15.625 | 5.708% |
| 300 | 18.75 | 6.850% |

\* finite obstacle pool의 평균 단면적 0.365313 m²를 곱한 nominal 값. 실제 배치 overlap은 corrected
lineage에서 금지된다. YOPOv2 공식 simulation navigation reference는 평균 간격 5 m/4 m, 즉
4/6.25 trees per 100 m²(각각 64/100 bars에 해당하는 count 비교)다. YOPOv2 원문은 이 비교에서
tree diameter 분포를 명시하지 않으므로 면적 점유율을 YOPOv2와 직접 비교하지 않는다.

## 발표에 넣을 사실과 넣지 않을 사실

- 넣을 것: sensor-only actor, 898-D structured observation, 17 tokens/5 history samples, 100 Hz
  physics·10 Hz policy, fixed flight-control stack, reward 구성, held-out historical failure analysis,
  corrected-v2 contract, route/physical gate 실패와 다음 측정 계획.
- 반드시 “historical”로 표시할 것: 기존 density×speed map, recovery-v2, 333/333 endpoint oracle.
- “NOT RUN”으로 표시할 것: corrected non-overlap v2의 PPO 학습/성능, 실제 하드웨어 검증.
- 쓰지 말 것: Transformer가 RNN보다 보편적으로 우월하다는 주장, corrected 환경에서 이미 얻었다는
  capture rate, YOPOv2의 확인되지 않은 나무 직경·면적 수치, sim-to-real 성능 주장.

## 패키지 파일

- `CLAUDE_PPT_MASTER_BRIEF_2026-08-26.md`: 18장 본편 + appendix 구성과 speaker-note 계약
- `MOTAR_SYSTEM_SPEC_2026-08-24.md`: 시스템·기하·동역학·보상·실험 명세
- `PAPER_WRITING_BRIEF_2026-08-24.md`: 논문 작성용 근거와 claim boundary
- `motar-system-overview.svg`, `motar-control-stack.svg`: 시스템/제어 구조도
- `platform.json`, `summary.json`들: 현재 플랫폼 gate와 historical raw evidence
- `README.md`, `VERIFICATION.md`: 재현 명령과 최종 판정

발표 생성 전 `sha256sum -c MANIFEST.sha256`를 실행한다. 그래프를 새로 만들 경우에도 원자료 JSON의
필드명과 계보를 유지하고, 빈 값은 추정으로 채우지 말고 `NOT AVAILABLE`로 둔다.
