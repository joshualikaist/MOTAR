# ref5in 최종 감사와 다음 순서 (2026-08-13)

## 한 줄 결론

`navrl_ref5in_quad`는 내부 정합성이 검증된 **hardware-informed simulation candidate**이고 PPO 학습도
가능하지만, 실기 기준 기체는 아니다. P2는 timeout 상한을 넘겨 실패했고, 후속 진단은 남은 병목을
“장거리 공통 충돌 + 장거리 CV-specific non-capture”로 좁혔다.

## Claude 초안에서 바로잡은 주장

1. `arm_motor_N`은 모터가 아니라 팔 시각 geometry의 중점이다. legacy motor 좌표는 ±0.13 m다.
2. legacy가 어떤 실기에도 대응할 수 없다고 증명하지 않았다. 정확한 표현은 mixed-scale 개발 수치가
   한 BOM/CAD에 추적되지 않는다는 것이다.
3. 377/221 rad/s²는 최대 각가속도가 아니라 hover-thrust roll authority 계산이다.
4. ref5in 1.20 kg, 합성 관성, 9.60 N/motor, 40 ms, 0.12 m height는 실측값이 아니다.
5. 0.28 m XY는 220 mm/5-inch prop-tip AABB보다 약 0.91% 작고, 0.12 m height는 45°에서 projected
   support를 약 2.83 cm 늘린다. intervention은 질량만 바꾼 것이 아니다.

## 완료한 검증

| 단계 | 결과 | 해석 범위 |
|---|---|---|
| P0 repository contract | 26/26 PASS | URDF/config/allocation/inertia 내부 정합성 |
| P0 canonical simulator | 21/21 PASS | 같은 controller에서 open-arena envelope |
| P1c fresh 900 epoch | PASS | 학습 가능성·안전·source provenance |
| P2 seed313, 2,049 eps | STRICT FAIL | 68.28/26.16/5.56%; timeout 114 > 102 |
| D0 seed317, 8,194 eps | descriptive complete | 장거리·pattern별 failure composition |

P1c는 성능 주장이 아니고 P2는 한 training seed/한 evaluation seed/70 bars뿐이다. 실기 성능은 어느
단계에서도 측정하지 않았다.

## D0 숫자가 말하는 것

- 거리 6–11.5 m: capture/crash/timeout `78.05/21.89/0.06%`.
- 거리 22.5–28 m: `55.94/29.09/14.97%`.
- 최장 거리 CV: `48.84/29.00/22.16%`.
- 최장 거리 waypoint: `62.87/29.17/7.96%`.
- 속도 0.3–0.6 → 1.2–1.5 m/s: timeout `8.49→3.10%`.
- 전체 crash: contact 1,737(81.59%), OOB 392(18.41%), height 0.

따라서 표적 속도가 높아서 못 잡는다는 설명은 맞지 않는다. 최장 거리에서 crash는 CV/waypoint 모두
약 29%지만, timeout만 CV에서 크게 늘어난다. episode를 늘리면 timeout 일부가 capture가 아니라 crash로
바뀔 수 있으므로 horizon 연장은 해결책이 아니다.

## 코드 감사 중 잡은 운영 결함

- dashboard generator가 TensorBoard archive 이동 뒤 과거 run 32개를 삭제해 보일 수 있던 문제를
  고쳐 checked-in/legacy/local run을 합친다. 현재 77 runs를 보존한다.
- P2 verifier가 개발이 계속된 뒤 과거 source snapshot까지 current tree와 같아야 한다고 요구하던
  문제를 고쳤다. 새 run은 current-source fail-closed, 과거 proof는 immutable snapshot으로 검증한다.
- held-out strata가 checkpoint에 남은 density training counters를 재사용하던 구조를 분리했다.
- 모든 strata에 capture/crash/timeout과 crash cause를 저장하고 global 합계 불일치 시 export를 막는다.
- 첫 diagnostic speed bin `[0,1.5]` 오류를 검출해 해당 speed 해석을 VOID 처리하고, 실제
  `[0.3,1.5]` support로 deterministic parity replay를 완료했다.
- 빈 circle cell의 Wilson CI를 non-standard JSON `NaN`이 아니라 `null`로 저장한다.

## 남은 순서

1. **D1 adaptation probe:** P1c warm-start, 70 bars, 명시적 `[22.5,28] m`, mixed,
   reward/policy/governor 불변,
   추가 1,000 epoch. full P3나 publication run이 아니다.
2. **D1 held-out gate:** 사전등록한 새 eval seed 331에서 최소 8,193 requested episodes를 사용한다.
   q3 CV timeout ≤12%, 전체 crash ≤27%, q3 crash ≤30%, PPO rollback/OOB/NaN 0을 모두 요구하고,
   하나라도 실패하면 연장하지 않는다.
3. **실패 시 frozen diagnostic:** CV initial heading을 toward/tangent/away로 고정해 path-length와 tracker/
   pursuit failure를 분리한다. PPO는 돌리지 않는다.
4. **D1 통과 시에만 P3 재승인:** fresh seed211 70→205 curriculum을 다시 사전등록한다. seed223/227은
   첫 seed와 held-out density curve를 본 뒤 추가한다.
5. **hardware gate는 별도:** exact BOM/CAD/CG, thrust stand, inertia/actuator identification, power/thermal/
   endurance, camera/LiDAR FOV·latency, 실제 hover/step/collision-envelope 검증이 필요하다.

이 순서는 한 run에서 airframe, reward, horizon, representation, governor를 동시에 바꾸지 않는다.
