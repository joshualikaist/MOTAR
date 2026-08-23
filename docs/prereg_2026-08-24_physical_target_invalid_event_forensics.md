# 사전등록 — physical-target invalid OBB event forensics

작성일: 2026-08-24. 결과를 본 뒤 조건·gate·밀도를 바꾸지 않는다.

## 목적

속도 포락선에서 205 bars의 0.6/0.9 m/s가 tracking/contact/planner 조건은 통과했지만
strict `invalid_state_fraction = 0` 때문에 탈락했다. 이 진단은 그 샘플을 수정하거나 무시하지
않고, **OBB가 어느 축에서 얼마만큼 arena 경계를 넘었는지**와 속도·명령·step을 원자료로 남긴다.
이는 planner가 경로를 거부했다는 주장이 아니라, physical-target의 arena-validity gate가
어떤 상태에서 실패했는지를 확인하는 계측이다.

## 고정 계약

| 항목 | 값 |
|---|---|
| seed | 509 |
| bars | 205 |
| target pattern | mixed |
| speed arms | 0.6, 0.9, 1.2, 1.5 m/s |
| envs / steps | 32 / 300 (warmup 20, 측정 280) |
| detector/vision | off (기존 physical speed gate와 동일) |
| 판정 | 새 pass/fail gate 없음; 기존 invalid-state 정의를 그대로 사용 |

각 arm은 별도 Isaac Gym subprocess에서 실행한다. `invalid & ~contact` 샘플만 event로 기록하며,
contact로 이미 귀속된 샘플을 OBB 원인으로 중복 집계하지 않는다. 기록 필드는 position, target
velocity, controller command, OBB support half-extents, arena bounds, 축별 남은 margin, step,
env id, finite/contact 상태다.

## 해석 규칙

- 이벤트가 0이면 “해결됨”이 아니라 해당 고정 표본에서 재현되지 않았다는 뜻이다.
- 이벤트가 있으면 축별 음수 margin과 reset 직전 상태를 먼저 확인한다.
- 원인이 코드/geometry bug로 확인되기 전에는 wall reserve, target speed, invalid gate를 바꾸지 않는다.
- 이 결과만으로 PPO 재학습, target-speed 완화, physical lineage 해제를 하지 않는다.

## engineering post-fix rerun (원 사전등록 보존)

원 사전등록의 fixed contract와 strict gate는 그대로 둔 채, forensic에서 확인된 center-only
wall reserve 결함에 한정해 두 가지 코드 수정을 적용했다.

1. planner center bounds에 현재 target OBB의 world-axis XY support를 더해 actor 전체가 arena
   안에 남도록 했다.
2. planner가 첫 안전 step을 찾지 못할 때 이전의 least-bad outward command 대신 zero planar
   command를 제출해 동역학적 controller가 감속하도록 했다. 위치 teleport, clamp, gate 완화는
   없다.

수정 후 동일 seed/grid 재실행 결과는 별도 보존했다:
`results/navrl_physical_target_speed_envelope_post_wall_brake_seed509/summary.json`.
invalid forensic은 0.6/0.9/1.2 m/s에서 0건, 1.5 m/s에서 1건(유한 OBB의 x-margin
약 −0.00006 m)으로 감소했지만, strict `invalid_state_fraction = 0`은 여전히 실패한다.
따라서 이 engineering rerun은 gate 통과나 PPO 허가가 아니다.

실행:

```bash
PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python \
  tools/diagnose_navrl_physical_target_invalid_events.py
```
