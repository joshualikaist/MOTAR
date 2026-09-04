# 사전등록 — 접촉 기하 포렌식: 거버너 회랑의 어디가 비어 있는가

작성 2026-09-04. **초안 — 사용자 승인 전 실행 금지.** 새 학습 없음. 기존 체크포인트의
접촉 순간 기하만 분해한다.

## 0. 질문 (RQ)

stopcap 스크린(seed 49)에서 기구는 **설계대로 작동했다**: 접촉 직전 실행 속도 2.024 →
0.302 m/s, 접촉 시 정지여유 −0.026 → **+0.395 m(양수)**. 그런데 crash는 15.95 → 21.31 %로
**올랐다**. 즉 접촉 순간 거버너 자신의 안전 모델은 "정지 가능"이라고 말한다.

→ **회랑(명령 방향 주위 반폭 0.45 m 직선)이 실제로 부딪히는 장애물을 담고 있지 않다.**

어느 사각지대인가? 사전에 넷을 후보로 세웠다: 측면(반폭 밖) · 후방(명령 반대편) ·
미지 공간(no-return을 자유로 취급) · 수직(수평 명령만 조정).

## 1. 측정 대상

`crash_cause = contact`인 프레임에서, **부딪힌 막대**(접촉 순간 차량 프레임 최근접 GT 막대)에
대해:

| 양 | 정의 |
|---|---|
| `delta` | 막대 방위 − **명령 방위** (거버너가 회랑을 그린 그 방향) |
| `forward` | `dist * cos(delta)` |
| `lateral` | `|dist * sin(delta)|` |
| `ray_returned` | 그 막대 방위 bin의 LiDAR가 유효 반환을 냈는가 (`< max_range*0.995`) |
| `elev_span` | 막대 상·하단의 고도각 범위가 LiDAR 수직 FOV(+20°/−10°)와 겹치는가 |

GT 막대 위치를 쓰지만 **평가 전용이며 actor·critic·보상·종료에 도달하지 않는다**
(`_record_bar_contact_probe`와 동일한 근거).

## 2. 분류 범주 — **결과 이전 동결**, 우선순위 순

1. `VERTICAL_OUT` — 막대의 고도각 범위가 LiDAR 수직 FOV와 전혀 겹치지 않음.
   (막대는 높이 2.0 m이고 드론은 ~1 m를 나므로 **거의 0일 것으로 예측**한다. 0이 아니면
   수직 사각지대가 실재한다는 뜻이다.)
2. `BEHIND` — `forward <= 0`. 명령 방향의 반대편 물체와 부딪혔다.
3. `LATERAL` — `forward > 0` 이고 `lateral > 0.45 m`. 회랑 반폭 밖.
4. `NO_RETURN` — 기하적으로 회랑 안인데 그 방위의 LiDAR ray가 무효(미지 공간을 자유로 취급).
5. `IN_CORRIDOR` — 위 어디에도 해당하지 않음. **거버너가 봤는데도 부딪혔다** = 진짜 종방향 실패.

이 다섯은 상호배타적이고 전수를 덮는다. 사후에 범주를 추가하거나 우선순위를 바꾸지 않는다.

## 3. 조건

| 항목 | 값 |
|---|---|
| 체크포인트 | ref5in ep25000 + riskcap 적응본 (stopcap 스크린과 동일) |
| arm | `off` · `riskcap` 2개 |
| bars | 205 (stopcap 스크린과 동일 조건) |
| episodes/arm | 2,049 |
| seed | **491** (미사용 확인) |
| 기기 | RTX 3070 |

두 arm을 두는 이유: 거버너가 켜졌을 때 회랑 사각지대의 **구성이 달라지는지** 본다. 거버너가
회랑 안 위협만 줄인다면 `IN_CORRIDOR` 비율은 riskcap에서 낮아지고 나머지가 상대적으로 커져야
한다.

## 4. 판정 규칙 (결과 이전 동결)

이것은 **기술적 분해이지 go/no-go 게이트가 아니다.** 판정 대신 사전에 예측을 적고 대조한다.

- **예측 1**: `IN_CORRIDOR`가 riskcap arm 접촉의 **과반 미만**이다. (과반이면 stopcap 보론의
  "회랑이 위협을 담고 있지 않다"는 진단이 틀린 것이다.)
- **예측 2**: `LATERAL`이 단일 최대 범주다.
- **예측 3**: `VERTICAL_OUT` ≈ 0 (막대가 높아서). 0이 아니면 예상 밖 발견이다.
- **예측 4**: `off` arm 대비 riskcap arm에서 `IN_CORRIDOR` 비중이 **감소**한다.

예측이 틀리는 것 자체가 결과다. 어느 쪽이든 다음 안전필터 설계의 입력이 된다.

## 5. 기계 무결성

- **M1**: `NAVRL_CONTACT_GEOMETRY` 미설정 시 코드 경로가 실행되지 않아야 하고, 기존 평가 수치가
  변하지 않아야 한다(같은 seed·체크포인트로 대조 arm이 기존 결과를 재현하는지 확인).
- **M2**: 다섯 범주의 합 = 총 접촉 수. 불일치 시 VOID.
- 분류에 쓰인 GT는 actor 경로에 도달하지 않는다(diff로 확인 가능해야 함).

## 5b. 노출시간(exposure) 부지표 — 측정 전 추가 (2026-09-04)

문헌 조사 결과를 반영해 **부지표 하나를 측정 전에 추가**한다. §2의 분류 범주·우선순위와 §4의
예측 1~4는 **일절 바꾸지 않는다.**

배경: stopcap 보론에서 세운 "노출시간" 기제 — clearance가 낮을 때 정확히 감속하므로 통과해야 할
틈에서 가장 오래 머문다 — 를 문헌에서 확인했다. Zhang et al.(arXiv:2602.08653v2, 2026, ZJU
FAST Lab)이 이를 **"constraint-induced reachability phenomenon"** 이라 부르며 말로 기술한다:

> "the vehicle ... remains longer in the vicinity of non-convex obstacle structures, increasing
> its exposure to local trapping configurations"

**그러나 그들의 실패 양태는 stagnation(정체)이고 collision이 아니다.** 조사에서 이 기제를
**충돌률에 귀속시킨 문헌은 발견되지 않았다.** 우리 stopcap 결과(정지여유 +0.395 m 양수인데도
crash 15.95 → 21.31 %)가 바로 그 귀속이므로, 이 실험에서 정량화한다.

**부지표 (결과 이전 동결)**: 에피소드당 **위험 노출 스텝 수** — 회랑 clearance가
`slow_distance_m`(3.0 m) 미만인 스텝의 개수. arm별 평균과 분포를 기록한다.

- **예측 5**: stopcap이 아니라 riskcap/off 비교에서도, 접촉으로 끝난 에피소드가 그렇지 않은
  에피소드보다 노출 스텝이 많다.
- 이 부지표는 **게이트가 아니다.** 분류 결과의 해석 보조이며, 단독으로 채택·기각 결정을 내리지
  않는다. 인과 주장(노출이 충돌을 *일으킨다*)은 이 관측 자료만으로는 하지 않는다 — 상관과 기제
  정합성까지만 보고한다.

## 5b-2. 경쟁 가설 — stopcap 부진의 두 번째 설명 (측정 전 추가)

문헌 조사 Q6에서 **분리되지 않은 경쟁 가설**이 드러났다. 측정 전에 명시한다.

항공 분야의 두 정본 속도 법칙은 **제동(braking) 법칙이 아니라 회피(sidestep) 법칙**이다:

- Falanga, Kim, Scaramuzza, *RA-L* 2019: `v̄ = s / (τ + 2√(r/ū₂))`
- Loquercio et al., *Science Robotics* 6(59), 2021 (보충 Eq. 11):
  `v_max = s / (t_s + t_p + t_rot + √(2 r_obs / (sin φ · c_max)))`,
  s=6 m, t_s=66 ms, c_max=35.3 m/s², r_obs=0.95 m, φ=65.5°, t_rot=125.2 ms → 13.5 m/s

둘 다 상한을 **"제때 옆으로 비킬 수 있는가"**로 정하며 정지거리를 쓰지 않는다. 우리 `stopcap`은
지상로봇 계열(DWA/SSM/PX4)의 제동 법칙을 **비킬 수 있는 비행체**에 적용한 것이다.

→ **가설 B**: stopcap이 부진한 이유는 노출시간(가설 A)이 아니라 **법칙 계열이 틀렸기** 때문일 수
있다. 제동 법칙은 측면 위협에 원리적으로 무력하다.

**이 실험은 두 가설을 완전히 분리하지 못한다.** 다만 §2 분류가 부분적 증거를 준다:

- `LATERAL`·`BEHIND`가 지배적이면 → **가설 B 지지**(제동으로 풀 수 없는 위협이 다수)
- `IN_CORRIDOR`가 지배적이면 → 두 가설 모두 부적합하고 종방향 실패가 진짜 원인
- 노출 부지표(§5b)가 접촉 에피소드에서 유의하게 크면 → **가설 A 지지**

**두 가설을 분리하는 결정적 실험은 이것이 아니라 sidestep 법칙을 세 번째 arm으로 넣는 것**이며,
그것은 별도 사전등록으로 다룬다. 본 실험에서는 **어느 가설도 확정하지 않는다.**

## 5c. 문헌상 위치 (측정과 무관, 서술용)

- **니치 확인**: Falanga/Loquercio의 v_max는 센서·플랫폼 파라미터로 계산한 **전역 오프라인 예산**
  이며 런타임에 측정된 clearance로 매 스텝 적응하지 않는다. 즉 **맵 없는 · 스텝별 · clearance
  조건부 속도 상한을 학습 정책에 얹는 것**은 항공 문헌에서 실제로 희박하다.
- 우리 필터의 계열은 **trajectory scaling / path-consistent safety**
  (Zanchettin & Rocco, IROS 2013: 스칼라 δ∈[0,1]로 경로 보존 속도 조정). 차이는 **δ에 0이 아닌
  하한을 둔 것**이다.
- "상한 아래 요청은 손대지 않음"은 Hsu/Hu/Fisac(*Annual Review*, 2024)의 **Perfect Safety
  Filter** 조건 2와 동일하다. 같은 문헌이 정지 상태 안전집합의 무용함을 명시해 우리 floor를
  정당화한다.
- **정정(2026-09-04, 2차 조사)**: 초안은 "0이 아닌 하한을 일부러 두는 사례가 없다"고 적었다.
  **틀렸다.** ROS 2 Nav2의 Regulated Pure Pursuit(Macenski et al., *Autonomous Robots* 47, 2023)는
  `regulated_linear_scaling_min_speed = 0.25 m/s`를 기본값으로 둔다. 우리 floor는 유일하지 않다.
  같은 논문이 **선형 램프를 지수·이차보다 의도적으로 선택**한 근거도 제공한다("exponential and
  quadratic ... far too significantly penalized proximity to objects").
- **더 가까운 재발견**: `nav2_collision_monitor`의 approach 동작이
  `safe_vel = velocity * (TTC / time_before_collision)`이며 소스 주석이 "Apply the same ratio to
  all components to **preserve curvature**"라고 명시한다. 맵 없음 · raw scan · 크기만 조정 ·
  곡률 보존 — 우리 설계와 사실상 동일하고 **ROS 2에 이미 출하돼 있다**. 다만 **출판된 평가가
  없다.**
- **최근접 항공 이웃**: Zhao, Wu, Chen, Gao, "Learning Speed Adaptation for Flight in Clutter,"
  *IEEE RA-L* 2024 (arXiv:2403.04586). RL 외부루프가 스칼라 속도 제약 `v†`를 EGO-Planner에
  부과하므로 실질적 magnitude-only다. 차이: **맵 기반**(점유격자)이고 성능 수치가 그림에만 있다.
  **반드시 인용·비교해야 한다.**
- **니치 진술(2차 조사 결론)**: 쿼드로터에서 ① 맵 없이 raw depth/LiDAR clearance의 닫힌 형식
  함수로 `v_max`를 정하고 ② 명령 방향을 보존하며 ③ 동일 명목속도에서 with/without ablation을
  낸 결과는 **없다**. 가까운 넷(Zhao RA-L'24 · Ryll ICRA'19 · Falanga RA-L'19 · DWA'97)이 각각
  하나씩 빠진다.
- **CBF/shield/"provably safe"로 부르지 않는다.** forward invariance도 recursive feasibility도
  없다. 가진 것은 liveness by construction 뿐이다.
- DWA 원논문(Fox et al., 1997) §2가 방향·속도 분리를 "무한한 힘을 낼 수 있을 때만 정당하다"고
  공격한다. 우리 답변은 "방향을 정하는 것은 clutter를 이미 관측한 학습 정책이지 기하학적 heading
  planner가 아니다"이며, 논문에 명시해야 한다.

## 6. 명시적 범위 밖

재학습 없음 · 거버너 법칙 변경 없음 · 회랑 반폭/여유 튜닝 없음 · out-of-bounds crash는 회랑
문제가 아니므로 분해 대상 아님(별도 집계만).
