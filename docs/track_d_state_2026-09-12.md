# Track D — 시뮬레이터 외형: 단계별 상태 (2026-09-12)

한 곳에서 관리한다. 각 줄은 **무엇이 측정됐고 무엇이 아직 아닌지**만 말한다.

| | 단계 | 상태 | 근거 |
|---|---|---|---|
| D1 | 자산 로더 (URDF) | `COMPLETE` | `results/renderer_urdf_smoke_2026-09-11/` |
| D2 | 독립 렌더러 | `COMPLETE` | `results/renderer_r3_seed173_*`, CPU 설치 검증 `results/renderer_cpu_install_2026-09-12/` |
| D3 | 배경 장면 엔지니어링 | `TECHNICAL_PASS` | `results/renderer_background_v1_clean_2026-09-11/`, `..._2026-09-12/` |
| D4 | R4/R4b 통계 기준 | `FAIL` (보존) | `docs/renderer_r3_r5_results_2026-09-11.md` |
| D5 | 시뮬레이터 visual probe | `PARTIAL_EVIDENCE` | `results/target_appearance_in_sim_2026-09-12/` |
| D6 | 동적 메시 타당성 | `INCONCLUSIVE` (정확성 통과, 비용 1.53배) | `results/dynamic_mesh_raycast_feasibility_2026-09-12/` |
| D7 | 통합 렌더 비용 | `INTEGRATED_RENDER_COST_UNMEASURED` | — |
| D8 | 검출기 통합 | `NOT_STARTED` | — |
| D9 | 색 지름길 재측정 | `NOT_RUN` | — |

## D5가 확정한 구조적 경계

`navrl_physical_target_params`는 `include_in_warp = False`다. 표적 메시는 정적 Warp 장면에
들어가지 않는다. 따라서 검출기가 URDF visual을 보지 않는 것은 **자산 버그가 아니라 렌더러 구조의
결과**다. 현재 증거와 일관된 서술은 이것뿐이다.

```
URDF visual 변경  →  Isaac 자산 외형 변경
그러나
표적 마스크  →  해석적 구/상자 프록시  →  URDF visual geometry는 저장된 마스크에 영향을 주지 않는다
```

구조는 의도적으로 보인다. `정적 장애물 메시` + `동적 표적 해석적 프리미티브`로 분리해 매 스텝
BVH 재적합을 피한다.

## D7 — 통합 비용은 아직 측정되지 않았다

**기존 숫자에서 보간하지 않는다.** 3.09 ms, 9.14 ms, R5의 음영 비용은 서로 다른 fixture와 단계의
측정이다. D6이 잰 것은 **독립 프로토타입에서의** 질의 비용(결정 셀 1.53배, +0.245 ms)이며,
그것은 통합 비용이 아니다. 정확한 표현은 이것이다:

> 추가 pixel-wise mesh query가 필요하므로 비용 증가는 예상되지만, **통합 비용은 현재 미측정이다.**

## 동결 체크포인트에 대해 할 수 있는 것과 없는 것

이전 판에 "관측이 바뀌므로 동결된 체크포인트 계보와 비교 불가"라고 적었는데 **너무 강했다.**

**가능하다.** 동결 체크포인트를 그대로 써서 `기존 인지` 대 `새 인지 처리`를 비교하는
**민감도/섭동 평가**는 할 수 있다. 그때 답하는 질문은 "기존 정책이 관측 분포 변화에 얼마나
민감한가"이며, 그것은 별도 treatment로 보고한다.

**할 수 없다.** 새 인지 결과를 기존 nominal 계보와 같은 관측 계약인 것처럼 합치는 것.
그리고 "새 인지에 적응한 정책의 효과"를 주장하려면 **별도 training 계보와 사전등록**이 필요하다.

```
frozen evaluation  = 가능, 단 별도 treatment
retraining         = adaptation 주장을 하려면 별도 계보 필요
```

## GEOMETRY_CONTRACT_DECISION_PENDING

접촉은 URDF 상자(0.283)를, 검출기는 하드코딩된 반치수(0.28)를 쓴다. 실재하는 불일치이지만
**타당성 실험의 선행조건은 아니었고** D6 프로토타입은 기존 기하 의미를 보존했다.
**본 검출기 통합 전에만** 결정하면 된다. 선택지 예시는 아래이며, **아직 고르지 않는다.**

```
A. 센서 visual geometry = 0.283, 접촉 = 기존 0.28 유지
B. 통합된 단일 기하 계약
C. visual / sensor / collision 기하를 명시적으로 분리
```

## 본 통합 전에 필요한 선행조건

1. D7 — 실제 검출기 경로에서의 통합 비용 측정 (D6의 독립 수치로 대체하지 않는다)
2. `GEOMETRY_CONTRACT_DECISION_PENDING` 해소
3. D6이 `INCONCLUSIVE`이므로, 통합을 정당화하려면 비용 근거를 D7에서 다시 세울 것
4. 관측이 바뀌는 변경이므로 사전등록과, adaptation을 주장한다면 별도 training 계보
