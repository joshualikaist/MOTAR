# What a Camera-Only Pursuit Policy Loses Without Its LiDAR, and How Far It Is From the Vehicle

논문 초안 — 2026-11-23. 범위: 카메라 전용(LiDAR 없음) 정책과 그 실기 이전(sim-to-real) 준비.
자료: [`SIMULATION_EXPERIMENT_MAP.md`](SIMULATION_EXPERIMENT_MAP.md) §4.1·4.2·4.4·5, 가족 `camera-only`·`gain-randomization`·
`load-invariance`·`sitl-transfer`의 기록(`docs/evidence/camonly_2026-11-04/`, `gain_randomization_2026-11-04/`,
`sitl_2026-11-05/`), 사전등록 `docs/plans/prereg_camonly_*.md`·`prereg_sitl_*.md`, 결정 D-2026-11-04-3·11-06-1·11-08-1·
11-13-1·11-15-1·11-18-1·11-18-2. 이 초안의 숫자는 전부 그 기록에서 그대로 옮겼고 다시 계산하지 않았다. 라벨은 기록의 영문
그대로 괄호 안에 둔다.

> **주장의 경계**: 전부 시뮬레이션이다. 실기 비행 0회, 실제 카메라 녹화 0건, 실기체 추력 측정 0건. "카메라만으로 실제
> 드론이 장애물 속에서 표적에 접근한다"는 문장은 쓰지 않는다. 쓰는 문장은 **"고정 정책 F에서 LiDAR를 떼면 무엇이
> 얼마나 사라지는지 측정했고, 카메라만으로 이어 학습한 정책이 그중 얼마를 되찾고 얼마를 못 찾는지 쟀으며, 실기로 가는
> 길에서 어떤 항목이 아직 검증되지 않았는지 명시했다"**이다. 외부 시스템과의 수치 비교는 0건(Class A = 0)이다.

---

## 1. 이 논문이 하는 말

밀집 막대 장애물 속에서 움직이는 표적에 접근하는 학습 정책(Policy F, RGB-D 카메라 + 72×4 LiDAR, 898-D 관측)을 바탕으로,
**실제 기체에 LiDAR가 없다**는 조건(D-2026-11-06-1, D-2026-11-13-1)에서 다음 다섯 가지를 측정했다.

1. **LiDAR를 떼면 충돌이 4.46 %에서 44.40 %로 는다(+39.95 pp [+33.95, +45.52]).** 그 거의 전부가 카메라 시야
   **밖** LiDAR의 몫이다(시야 밖만 제거 +39.25 pp [+33.65, +44.71]); 시야 **안** LiDAR만 떼면 −0.69 pp
   [−1.07, −0.33]로 2 pp 띠 안이다. 즉 LiDAR는 카메라를 "보정"하는 센서가 아니라 **옆을 보는** 센서였다(C2,
   `PASS_WITH_LIMITS`). 추적 재실행에서 카메라 전용 F의 마지막 접촉 막대는 72.9 %가 시야 밖이었고(F 전체 센서 48.8 %,
   차이 +24.0 pp [19.3, 28.7]) 이 읽기는 `BLIND_SIDE_SUPPORTED`다(G3).
2. **카메라만으로 3,000 epoch 이어 학습한 정책(CAM-K1)은 그 손실의 대부분을 되찾지만 전부는 아니다.** 같은 66° 카메라의
   카메라 전용 F(충돌 53.89 %) 대비 −37.67 pp [−38.45, −36.78](시드 1) / −36.74 pp(시드 2)로 `MATERIAL_RECOVERY`,
   그러나 LiDAR를 가진 F(4.46 %) 대비 +11.77 pp [+11.26, +12.30] / +12.69 pp로 `MATERIAL_LOSS`다(CAM-K1-E).
3. **실기체(1.1 kg, T/W 4.0 추정)와 OAK-D 깊이 모델에 맞춘 추가 학습(CAM-K2 v2)은 충돌을 더 낮추지만 "적응"으로는
   읽히지 않는다.** 처리 조건에서 CAM-K1 대비 −3.80 [−4.63, −2.93] / −4.74 [−5.26, −4.24] pp(`SMALL_ADAPTATION`),
   정상 조건에서 손실 없음(`NO_MATERIAL_LOSS`), 그러나 상호작용 게이트는 두 시드가 어긋나(`SEEDS_DISAGREE`)
   `adaptation_supported = false`다. 근거리 깊이 무효 화소를 표면으로 되살리는 규칙(v3)은 학습 진단에서는 효과가
   보이나(`DIAG_OK`) 평가에서는 2 pp 기준 아래다(−1.5015 / −1.1475 pp, `NO_DETECTED_CHANGE`).
4. **카메라 전용 정책의 충돌 위치는 정의에 민감하고, 한 스텝(100 ms) 지연은 해롭다.** 접촉 막대의 발자국 기준으로는
   67.7 %가 시야 안, 최근접점 기준으로는 42.5 %가 완전 사각(`DEFINITION_SENSITIVE`); 지연 1 스텝은 두 시드 모두
   +4.95 [4.30, 5.60] / +3.67 [3.12, 4.17] pp(`WORSE`); 속도 상한 2.5·2.0 m/s는 충돌을 낮추지 않는다(CAM-T1).
   F 자체도 0.4 s 행동 유지에는 `HARMFUL`(+10.47 pp), 100 ms 지연에는 `TOLERANT`(+1.51 pp, CI 0 제외)다(G7).
5. **고정 정책 F는 PX4 v1.14.3 + Gazebo SITL 안에서 자기 센서 모델로는 같은 장면에서 같은 결과를 낸다**(70 막대 122/122
   포착, Δ충돌 −0.82 pp, `FEASIBLE_IN_SITL`). 그러나 **Gazebo 자체 센서로 보게 하면** 39 장면 중 포착 20·충돌 17로
   무너지고(S4, 원인 미확인), 야외 카메라 전용 프로파일은 "실제 근거리 장애물을 놓칠 수 있다"는 기준에서 독립 검토
   세 번 모두 `FAIL`이어서 **야외 비행 GO는 없다**(D-2026-11-15-1).

이 다섯 결과의 공통 단위는 하나의 격자(70/115/160/205 막대 × 평가 시드 4501–4503 × 셀당 2,048 할당 에피소드)와
하나의 학습 시드 쌍(1, 2)이다. 모든 수치는 그 범위 안의 것이다.

## 2. 시스템과 과제

- **시뮬레이터·과제.** Isaac Gym Preview 4 위의 Aerial Gym. 40 × 40 × 3 m 경기장, 기체 높이를 가로지르는 축정렬 막대
  70–205개(2026-08-27 비중첩 기하), 60 s 에피소드, 제어 스텝 0.1 s. 표적은 장애물을 아는 이동 표적 `E2_obstacle_aware`,
  초기 배치는 KROC2027 A3. **포착** = 한 스텝 안의 쓸림 최근접 거리 < 0.5 m(물리적 포획이 아니고 무충돌도 아니다);
  **충돌** = 물리 접촉(막대·바닥·경계 이탈). 타임아웃은 0.004–0.012 %다(`docs/EVIDENCE.md`,
  `docs/evidence/camonly_2026-11-04/README.md`).
- **학습 기체.** navrl_quad, 0.25 kg, 모터당 최대 2.0 N(T/W 3.26), τ 0.04 s, Lee 기하 제어기, 틸트 상한 45°
  (`docs/plans/prereg_camonly_k2_train_v2_2026-11-08.md`).
- **Policy F.** ep25000 + riskcap(SHA `f7022139…`), 2026-08-05 동결. 관측 898-D = 융합 정적 스캔 288 + 장애물 토큰 480
  (±120°를 30° 8섹터) + 표적 이력 80(카메라 Kalman 갱신, LiDAR 보조 보정) + 기체 이력 50. 행동은 기체 좌표 속도
  (축당 2.5 m/s, 대각 최대 3.54 m/s)와 요레이트(3.0 rad/s). 표적 검출기는 학습된 모델이 아니라 **빨간 구에 대한 색
  규칙** sigmoid(3R − 2G − 2B − 0.9)다(`docs/plans/camonly_study_plan_v1.md`, `docs/plans/sitl_validation_plan_v1.md`).
- **센서와 융합.** 카메라 87° × 58°, 장애물 깊이는 40 × 24를 10 m까지 렌더해 160 × 90으로 쌍선형 업샘플(이 업샘플이
  유령 장애물을 만든다, EQ-1D); LiDAR 72 × 4, 5° 간격, 12 m. 시야 안(|방위| ≤ 43.5°, 72 빈 중 17개)에서는 LiDAR 거리와
  해당 열 카메라 거리의 **최소값**을 쓴다(min-fusion). 따라서 LiDAR는 카메라의 유령을 지우지 못하고, 시야 안에서
  LiDAR의 값은 카메라가 **놓친** 것뿐이다(`docs/plans/camonly_study_plan_v1.md`).
- **속도 거버너(riskcap).** 회랑 반폭 0.45 m의 원시 LiDAR 여유가 3 m 이하면 2.0 m/s로 고정, 5 m 이상에서 자유; 방향은
  보존한다. LiDAR가 없는 조건에서는 카메라 융합 스캔의 여유를 시야 안에서만 쓰고 시야 밖은 여유 0(상한 2.0 m/s)으로
  둔다(규칙 K2).

## 3. 실험 설계

### 3.1 LiDAR 제거 — 제로샷 C2 (`docs/plans/prereg_camonly_c1c2_v1.md` + 개정 1)

고정 F를 바꾸지 않고 런타임 래퍼(`navrl_sensor_ablation.py`, 기본 OFF, 연구 전용 진입점으로만 선택; D-2026-11-04-3)로
LiDAR 빈을 "12 m, 무반환"으로 바꾼다.

| 조건 | 시야 안 LiDAR | 시야 밖 LiDAR | 거버너 | OOD |
| --- | --- | --- | --- | --- |
| A `full` (GR-0 S00 셀 재사용) | 있음 | 있음 | 원시 LiDAR | 아니오 |
| B_in `no_lidar_in_fov` | 없음 | 있음 | 같은 센서 | 아니오 |
| B_out `no_lidar_out_fov` | 있음 | 없음 | 같은 센서, K2 | 예 |
| C `no_lidar` | 없음 | 없음 | 카메라만, K2 | 예 |
| C_gL `no_lidar_obs_only` | 없음 | 없음 | 원시 LiDAR | 예 |
| B_in_gL `no_lidar_in_fov_obs_only` | 없음 | 있음 | 원시 LiDAR | 아니오 |

대비 8개(total = C − A, in_fov, out_fov, interaction, obs_in, gov_in, gov_all, obs_all), 띠는 포착 3 pp·충돌 2 pp
(GR-0의 띠), 블록 12개(밀도 × 시드)에 대한 백분위 부트스트랩 10,000회 + t 구간. C1 동일성 게이트: `full`이 GR-0 셀과
바이트 동일, `no_lidar`는 달라야 한다(`C1_IDENTITY_PASS`).

### 3.2 사각 접촉 추적 — G3 (`docs/plans/prereg_camonly_c2trace_v1.md`)

A·B_out·C를 시드 4501 네 밀도에서 추적 기록기와 함께 재실행(바이트 동일성 게이트). 충돌 에피소드의 마지막 1 s 창에서
접촉 막대가 시야 밖이었던 비율 out_last를 재고, Δ = out_last(C) − out_last(A) ≥ 15 pp, CI 하한 > 0, out_last(C) ≥ 60 %면
`BLIND_SIDE_SUPPORTED`.

### 3.3 카메라 전용 연속 학습 — CAM-K1 (`docs/plans/prereg_camonly_k1_train_v1_2026-11-06.md`, 평가 `…k1_eval_v1_2026-11-08.md`)

| 항목 | F | CAM-K1 |
| --- | --- | --- |
| 시작 | — | F ep25000 |
| epoch | 24001..25000 | **25001..28000** (3,000) |
| 센서 | 카메라 + LiDAR | **카메라만** (`NAVRL_SENSOR_ABLATION=no_lidar`) |
| 카메라 FOV | 87° × 58° | **66° × 54°** (OAK-D Pro 자동초점 RGB; 같은 160 × 90 화소, 내부 파라미터만 변경) |
| `yaw_align_weight` | 0.3 | **0.6** |
| `visibility_bonus` | 0.02 | **0.03** (스텝당 시간 비용 −0.05보다 작게) |
| 학습 시드 | — | 1, 2 |

205 막대 고정, 커리큘럼 없음, LR 5e-6, 128 환경(GR-1 `ctrl` 계약). 보상의 정적 안전 항 `r_ss`는 시뮬레이터 LiDAR를
계속 읽는다(보상 전용 특권 정보; 관측에는 들어가지 않는다).

평가 CAM-K1-E의 팔: A = F(카메라 + LiDAR, 87°), C87 = F `no_lidar` 87°(C2 셀), **F66** = F `no_lidar` 66°(새 12셀),
K1s1/K1s2. 게이트 R(K1 − F66, 충돌): `MATERIAL_RECOVERY` = 두 구간 상한 < −10 pp. 게이트 P(K1 − A):
`NO_MATERIAL_LOSS` = 상한 < +3 pp, `MATERIAL_LOSS` = 하한 > +3 pp. 층화 부트스트랩 10,000회 + t(df 8).

### 3.4 실기체 맞춤 학습 — CAM-K2 v2 (`docs/plans/prereg_camonly_k2_train_v2_2026-11-08.md`, 평가 `…k2_eval_v1_2026-11-08.md`)

실기체 사실(D-2026-11-08-1): 1.1 kg(4S 배터리 포함), 5인치 프로펠러, Velox 2550KV, **추력 데이터 없음 → T/W 4.0은
INFERRED**(벤치 자료로 3.94 추정). v1(T/W 2.25, 0.5 kg 가정)은 실행 전 철회(`WITHDRAWN_BEFORE_RUN`).

| 처리 | 값 |
| --- | --- |
| 모터 한계 | navrl_quad의 모터당 한계를 T/W 4.0으로 **올림**: 4.0 × 0.25 × 9.81 / 4 = 2.4525 N |
| 모터 무작위화 | 전달 추력 1 ± 0.15(환경별) × 1 ± 0.05(모터별), τ ∈ [0.02, 0.04] s; 유효 T/W 3.4–4.6 |
| 깊이 모델 `oakd` | OAK-D Pro 800P 확장 시차·서브픽셀, f·B 66.1875 px·m, σ_d 0.15 px(INFERRED) → σ_z = z²·0.15/66.1875; MinZ 0.4 m, MaxZ 20 m, iid 구멍 p 0.02; 무효 = 무반환 |
| 학습 | CAM-K1 각 시드에서 1,000 epoch(28001..29000) |

"올림 무효" 규칙: 기록된 모터 명령 중 2 N 초과가 1 % 미만이면 그 시드의 결과를 T/W 변경에 귀속하지 않는다.
회전 동역학은 맞지 않는다(1.1 kg 5인치 프레임 추정치의 1.1–2.6배 롤 능력, INFERRED).

평가 CAM-K2-E: 처리 ON에서 K2 − K1(게이트 A, `SMALL_ADAPTATION` = 상한 < 0이고 평균 ≤ −2 pp), 정상 조건에서
K2_nom − K1(게이트 N), 상호작용(게이트 I, 6 블록, df 4). "적응"은 A와 I(`INTERACTION_NEGATIVE`)가 함께 성립할 때만.
알려진 틈: MinZ 0.4 m + "무효 → 무반환" 규약은 정면 막대의 마지막 약 0.26 m를 지운다.

### 3.5 깊이 유효성 규칙 — CAM-K2 v3와 E3 (`…k2_train_v3_2026-11-08.md`, `…k2_eval_v3_2026-11-10.md`)

`blob_near`: 깊이 모델의 무효 마스크에서 5 × 5 창 무효 밀도 ≥ 0.5인 화소를 **MinZ(0.4 m) 표면**으로 되살린다(고립 무효
화소는 무반환 유지, 나머지는 비트 그대로). v2 레시피로 재학습, 진단 `hit_token` ≥ 0.5면 `DIAG_OK`. 평가 E3: V3 − V2
(게이트 D, 2 pp 바닥), R = V2+규칙 − V2, T = V3 − V2+규칙, D = R + T.

### 3.6 충돌 위치와 포락선 — CAM-T1 (`camonly/t1`, `docs/plans/prereg_camonly_t1_2026-11-12.md`)

CAM-K2 v3를 E3의 V3 조건 그대로 24 셀 추적 재실행. 접촉 막대(수평 표면 거리 최소, ≤ 0.6 m면 VALID)가 마지막 1 s 창에서
기하학적 FOV(반각 33°, 수직 27°, r ≤ 10 m; 다른 막대에 의한 가림은 NOT_MEASURED) 안이었는지로 `IN_VIEW` / `LEFT_FOV` /
`BLIND`로 분류. 두 프로파일(발자국 기준, 최근접점 기준)이 같은 CLEAR 우세 범주를 줄 때만 방향을 명명, 아니면
`DEFINITION_SENSITIVE`. 포락선 36 셀: cap25(자유 2.5 / 고정 2.0 m/s), cap20(상시 2.0 m/s), d1(한 스텝 전 행동 실행).

### 3.7 지원 실험 — G7 지연·유지, GR-0 이득 민감도

G7(`docs/evidence/sitl_2026-11-05/g7/`): 고정 F, `full`, 시드 4501, 70/205 막대, k2d0(0.2 s 유지)·k4d0(0.4 s)·k1d1(100 ms
지연); `TOLERANT` = Δ ≤ +2 pp & CI 상한 ≤ +4, `HARMFUL` = Δ ≥ +5 & CI 하한 > 0.
GR-0(`gainrand/dev`, `docs/evidence/gain_randomization_2026-11-04/`): 고정 F, Lee 제어기 이득을 에피소드마다 공칭 ×
[0.8, 1.2](S20) / [0.6, 1.4](S40)로 무작위화, 36 셀.

### 3.8 SITL과 ROS 2 런타임 (`docs/plans/sitl_validation_plan_v1.md`, `prereg_sitl_*`, `ros2/motar_camera_policy/README.md`)

- **스택.** Gazebo Classic 11 + PX4 v1.14.3 SITL, lockstep, 브리지가 sim 시간 100 ms마다 행동. navrl_quad를 PX4 기체
  10090으로 설치(0.25 kg, 하향 거리계가 높이 기준), 비교용 iris(1.5 kg, T/W 약 1.9).
- **S3b(공식).** **센서는 MOTAR 자체 커널이 시뮬레이터 참값 포즈에서 렌더**(LiDAR Warp CPU, 깊이 40 × 24 → 160 × 90),
  지각은 PX4 추정 포즈로, F + riskcap이 10 Hz 기체 좌표 속도 설정값을 offboard로 보내고 고도 1.0 m는 PX4가 유지.
  표적은 Isaac 영행동 셀의 60 s 궤적을 재생(추격자에 반응하지 않음). 블록 A 70 막대 122 장면(1차, 게이트 "SITL 충돌 ≤
  Isaac 충돌 + 5 pp"), B 205 막대 30 장면(보고), C 반복 10 장면. 충돌 = MOTAR 기하 규칙(0.14 m 발자국), Gazebo 접촉
  교차 확인은 미구현. S3a 관측 동등성은 `NOT_EQUIVALENT`(TF32)인 채로 사용자가 S3b 진입을 결정했다.
- **S3c.** iris 팔과 delay1 팔(명령만 한 스텝 늦게), 각 40 장면, 보고 전용.
- **S4.** Gazebo 자체 gpu_ray LiDAR·RGB·깊이 카메라(기체 고정, 조명·지면 있음, 무잡음)로 F가 본다. `ground` 팔은
  0.10 m 아래 반환 제거(40 장면), `raw` 팔은 그대로(20 장면). 카메라 재현율 = 그림자 렌더러 표적 마스크 ≥ 2 px인
  스텝 중 색 규칙이 표적을 본 비율(마운트 기하와 섞임).
- **S4F-1.** S4 `ground` 팔에서 표적 외관(조명 반사 vs 자발광) × 지면 렌더(있음/없음) 2 × 2, 셀당 40 에피소드, Holm.
- **S5(안전, 정책 없음).** PX4 failsafe 10 시나리오(offboard 손실→LOITER, 지오펜스 수평/수직, 조종기 개입, RC 손실→LAND,
  데이터링크 손실→LAND, 킬 스위치, 강제 시동 해제 등)를 SITL에서 확인; 실기 판별은 NOT_TESTED.
- **ROS 2 카메라 전용 런타임.** `camera_policy`(OAK-D 정렬 RGB-D → 898-D 관측 → CAM-K2 v3 → `/motar/policy/velocity_enu`)와
  별도 프로세스 `camera_px4_gateway`(uXRCE-DDS, 0.1 s 주기, NED XY 속도 + 현재 Z 유지). **소프트웨어는 시동·모드 전환·
  이륙 명령을 내지 않는다**; 조종사가 POSITION 호버를 만든 뒤 OFFBOARD를 고른다. 지오펜스는 방향을 보존하며 속도를
  s ∈ [0, 1]로 줄이고 상자 밖이면 영속도 고정. 깊이 의미: +Inf = 무반환, −Inf/NaN/0 = 센서 무효, 조밀 무효 블롭만
  근접 표면(`blob_near`). 야외 프로파일(opt-in): 하늘(깊이 0)을 지평선 위·하늘색·화면 상단 연결·검증된 지면과 만나지
  않는 광선일 때만 해제, 가상 천장 3.0 m, 거부 프레임은 영속도 유지 제안, 유지 상한 3.0 s(PROVISIONAL). 모든 임계값
  PROVISIONAL.
- **실기체(D-2026-11-18-1/2).** 250급, 약 10분 비행, Jetson Orin NX(ROS 1, Docker로 ROS 2 가능), PX4 ~1.14(v1.14.3으로
  설정 가능), GNSS 없음 → **같은 OAK-D를 쓰는 온보드 VIO**(한 호스트 연결만 허용되므로 비행 중 정책은 VIO 쪽 드라이버의
  스트림을 받아야 함), 숲 현장, 표적은 **다른 드론**(빨간 색 규칙은 표식 없는 드론을 검출하지 못한다).

## 4. 결과

### 4.1 LiDAR가 기여하는 것 (C2, G3, 기제)

| 조건 | 충돌 % | 대비 | 충돌 Δ pp [95 %] | 라벨 |
| --- | --- | --- | --- | --- |
| A `full` | 4.46 | — | — | — |
| C `no_lidar` | 44.40 | total = C − A | **+39.95 [+33.95, +45.52]** | HIGHER (OOD) |
| B_out `no_lidar_out_fov` | 43.71 | out_fov = B_out − A | **+39.25 [+33.65, +44.71]** | HIGHER (OOD) |
| B_in `no_lidar_in_fov` | 3.76 | in_fov = B_in − A | **−0.69 [−1.07, −0.33]** | SMALL_LOWER |
| — | — | gov_all = C − C_gL | +5.05 [+4.15, +5.83] | HIGHER |
| — | — | obs_in, gov_in, interaction | — | NEGLIGIBLE / SMALL(취약) |

출처 `docs/evidence/camonly_2026-11-04/c1c2/c2_analysis.json`(README 경유). 60 셀 COMPLETE, 호스트 결함 0, 바인딩 18/18.
독립 검토 r1(Claude Opus 5.5) `PASS_WITH_LIMITS`: MAJOR M-1 = 저자의 CPU 센서 검사가 C2 셀 2개와 동시에 돌았다(이탈로
공개; 이후 T-LOAD가 `LOAD_INVARIANT`를 읽어 재실행은 선택 사항). 밀도별 포착 손실은 70 막대 −24.54 pp에서 205 막대
−51.45 pp까지(기술적).

**시야 밖 접촉(G3, `c2trace/c2trace_analysis.json`).** 마지막 접촉 막대가 시야 밖인 비율 out_last: A 48.8 %, B_out 72.1 %,
C 72.9 %; C − A **+24.0 pp [19.3, 28.7]**; 1 s 내내 시야에 없던 비율 33.7 → 56.7 %; C의 접촉 막대는 93.4 %가 왼쪽.
`NONINTERFERENCE_PASS`, `BLIND_SIDE_SUPPORTED`, 검토 `PASS_WITH_LIMITS`. 한 시드(4501)·OOD 조건이며 "consistent with"다.

**기제(탐색적, 검토 없음, `mechanism/`).** 전체 센서 F는 이동 프레임의 51.8 %에서 ±43.5° 시야 밖 방향으로 난다(앞-왼쪽
대각); 임박 막대의 52.0 %가 시야 밖; 횡방향 반전은 A 2.07 % → C 0.15 %(스텝 기준). A에서 포착한 첫 에피소드 1,536개 중
592개(38.5 %)가 C에서 충돌하고, C의 충돌 시각 중앙값은 3.2 s, 2 s 안 충돌이 12.24 %(A 1.40 %). 접촉 방위는 C에서
NOT_MEASURED이므로 측정된 원인이 아니라 가설이다.

**센서 수준(탐색적, `sensor/sensor_check_01.json`).** F 자신의 포즈 1,200 프레임에서 카메라 융합값 vs 정확 수평 광선:
0–2 m 유령 0.2 % / 누락 9.1 %, 4–6 m 7.3 % / 36.4 %, 8–10 m 17.8 % / 42.1 %, 10–12 m "카메라 밖" 68.4 %; 열-방위 오프셋
최대 3.41°.

### 4.2 카메라 전용 연속 학습 (CAM-K1, CAM-K1-E)

학습(`camk1/training_curve.json`): 롤아웃 포착 s1 8.45 → 26.29 %, s2 7.79 → 25.55 %; 충돌 s1 91.55 → 73.70 %(탐색 잡음
하의 롤아웃이지 평가가 아니다). 두 시드 `TRAINED`, 검토 `PASS_WITH_LIMITS`.

| 팔 | 충돌 %(풀) | 대비 | Δ pp [95 %] s1 / s2 | 게이트 |
| --- | --- | --- | --- | --- |
| A: F, 카메라 + LiDAR, 87° | 4.46 | — | — | — |
| C87: F `no_lidar` 87° | 44.40 | F66 − C87 | +9.49 (보고) | 87° → 66° 축소만의 값 |
| F66: F `no_lidar` 66° | 53.89 | F66 − A | +49.43 (보고) | — |
| K1s1 / K1s2 | 16.22 / 17.15 | R = K1 − F66 | **−37.67 [−38.45, −36.78] / −36.74** | `MATERIAL_RECOVERY` ×2 |
| | | P = K1 − A | **+11.77 [+11.26, +12.30] / +12.69** | `MATERIAL_LOSS` ×2 |

출처 `docs/evidence/camonly_2026-11-04/camk1e/analysis.json`. `IDENTITY_PASS`, COMPLETE, 대조 OK, 결과 검토 r1(Opus 5.5)
`PASS_WITH_LIMITS`(MAJOR 0). 한계: K1 − F66은 레시피 전체(센서 + FOV + 보상 가중치 + 3,000 epoch)의 효과이고 분해되지
않았다; 두 시드는 −0.92 pp 차이로 풀링하지 않는다; 구간은 학습 시드 변동을 포함하지 않는다.

### 4.3 실기체 맞춤 (CAM-K2 v2, CAM-K2-E, v3, E3)

CAM-K2 v2 학습: 2 N 초과 명령 5.95 / 5.25 %(올림 무효 규칙 1 % 통과), 롤아웃 포착 24.78 → 28.43 / 25.47 → 28.61 %;
두 시드 `TRAINED`(`camk2/README.md`).

| 게이트 | 대비(블록) | 충돌 % | Δ pp [95 %] s1 / s2 | 라벨 |
| --- | --- | --- | --- | --- |
| A | K2 − K1, 처리 ON (12) | K1 18.64 / 19.58 → K2 14.84 / 14.84 | **−3.80 [−4.63, −2.93] / −4.74 [−5.26, −4.24]** | `SMALL_ADAPTATION` ×2 |
| N | K2_nom − K1(정상) (6) | K2_nom 13.83 / 13.52 | −2.99 [−4.14, −1.85] / −4.18 [−4.46, −3.95] | `NO_MATERIAL_LOSS` ×2 |
| I | 상호작용 (6) | — | −0.15 / −0.69 | `SEEDS_DISAGREE` → `adaptation_supported` **false** |

처리가 K1에 지우는 비용은 +2.42 / +2.44 pp. 출처 `camk2e/analysis.json`; `IDENTITY_PASS`, COMPLETE 60/60, 결과 검토 r1
(Opus 5.5) `PASS`. 읽기: K2가 낮은 것은 처리 유무와 무관하게 비슷하므로 **1,000 epoch 추가 학습과 처리 적응이 분리되지
않았다.** K2s1과 K2s2의 풀 충돌이 같은 것(3,647 / 3,647)은 우연이며 블록별로는 다르다.

v3(`blob_near`): 학습 진단 hit_token 0.6958 / 0.6898(v2 0.1668 / 0.1916, CAM-K1 0.6768 / 0.7222), `DIAG_OK`; 규칙이
전체 화소의 2.15 / 2.04 %(무효 화소의 69.2 / 68.0 %)를 근접 표면으로 바꿨다.

| 대비 | 충돌 % s1 / s2 | Δ pp [95 %] s1 / s2 | 라벨 |
| --- | --- | --- | --- |
| V2 (기준, E의 K2 셀) | 14.8397 / 14.8397 | — | — |
| D = V3 − V2 | V3 13.3382 / 13.6922 | **−1.5015 [−2.0264, −0.9766] / −1.1475 [−1.7619, −0.5493]** | `NO_DETECTED_CHANGE` ×2 |
| R = V2+규칙 − V2 | V2R 13.0208 / 12.1989 | s2 −2.6408 | `SEEDS_DISAGREE` |
| T = V3 − V2R | — | s2 +1.4933 | `SEEDS_DISAGREE` |

출처 `camk2e/analysis_v3e.json`, `RESULTS_v3e_2026-11-10.md`; 결과 검토 r1(Codex `gpt-6.1-sol`) `PASS`. 구간은 0을
제외하지만 사전등록한 2 pp 바닥 아래이므로 라벨은 `NO_DETECTED_CHANGE`다. T가 두 시드 모두 양수여서 재학습의 이득은
성립하지 않는다. v3는 ROS 2 런타임에 바인딩된 정책이다.

### 4.4 충돌 위치·지연·이득 민감도 (CAM-T1, G7, GR-0)

CAM-T1 Stage A(`camt1/stage_a_analysis.json`, `camonly/t1`): 발자국 프로파일 풀 BLIND 10.8 / LEFT_FOV 21.5 / IN_VIEW
67.7 %; 최근접점 프로파일 BLIND 42.5 / IN_VIEW 30.8 % → `DEFINITION_SENSITIVE`. VALID 충돌에서 접촉 시 카메라는 모두
MinZ 0.4 m 안(중앙값 0.22 m)이었으므로 **IN_VIEW ≠ 깊이에 보임**이다. 시드 간 왼쪽 접촉 52.3 vs 71.3 %.

| Stage B 팔 | 충돌 Δ pp [95 %] s1 / s2 | 라벨 |
| --- | --- | --- |
| d1 (100 ms 지연) | **+4.95 [4.30, 5.60] / +3.67 [3.12, 4.17]** | `WORSE` ×2 |
| cap20 (상시 2.0 m/s) | +1.74 [0.93, 2.70] / +2.41 [1.63, 3.26] | `SEEDS_DISAGREE` (NO_DETECTED_CHANGE / WORSE) |
| cap25 | — | `SEEDS_DISAGREE` (NO_DETECTED_CHANGE / NEGLIGIBLE) |

기준 충돌 15.97 / 17.37 %, d1 20.91 / 21.04 %. `NONINTERFERENCE_PASS` 24/24, 결과 검토 `ACCEPT_WITH_LIMITS`(P0/P1 0).
속도 상한은 충돌을 낮추지 않는다(E3 V3 셀의 96–98 % 에피소드가 2.5 m/s 초과를 명령). 재학습은 승인되지 않았다.

G7(고정 F, `sitl_2026-11-05/g7/analysis.json`): k2d0 +2.001953125 pp [0.977, 3.027] `INCONCLUSIVE`(에피소드 하나 차이);
k4d0 **+10.4736328125 pp [9.229, 11.743] `HARMFUL`**(205 막대 충돌 26.27 % vs F 8.84 %); k1d1 +1.513671875 pp
[0.537, 2.515] `TOLERANT`(CI는 0을 제외; "PX4/실비행 지연 보장이 아니다"). 한 시드, 두 밀도.

GR-0(고정 F, `gain_randomization_2026-11-04/README.md`): S00 충돌 4.46 %, S20 5.07 %(Δ +0.61 pp [+0.25, +0.98],
`GAIN_ROBUST`), S40 8.15 %(Δ **+3.69 pp [+3.03, +4.37]**, `GAIN_SENSITIVE`); S40 증가의 약 절반은 41–64° 틸트의 이른 바닥
충돌. 이득 무작위화 학습(GR-1 `GR1_SUPPORTED`, P1 −2.21 pp)은 두 번째 학습 시드에서 재현되지 않았다(GR-1b P1 +0.444 pp
[+0.000, +0.875], `NOT_CONFIRMED`), 그래서 채택하지 않았다.

### 4.5 SITL (`docs/evidence/sitl_2026-11-05/`)

| 단위 | 설정 | 결과 | 라벨 |
| --- | --- | --- | --- |
| S3b formal v1 | 고정 F, MOTAR 자체 센서 렌더, PX4 v1.14.3 | 70 막대: SITL 122/122 포착 vs Isaac 121, Δ충돌 −0.82 pp(여유 +5 pp); 205 막대 27 포착/3 충돌 vs 26/4; 반복 10/10; VALID 162 | **`FEASIBLE_IN_SITL`**, 검토 `PASS_WITH_LIMITS` |
| S3c | iris 기체 / 명령 1 스텝 지연 | iris 40/40 포착; delay1 37 포착 / 3 충돌(+7.5 pp, 95 % CI 1.6–20.4 %) | 보고 전용 |
| S4 | Gazebo 자체 센서 | `ground` 39 VALID 중 포착 20 / 충돌 17 / 타임아웃 2(Isaac·S3b는 39 포착); `raw` 19 중 5 / 11 / 3; 카메라 재현율 0.634, 0.479 | 보고 전용, 원인 미확인 |
| S4F-1 | 외관 × 지면 2 × 2 | 포착 55 / 60 / 67.5 / 75 %(n 40); 지면 +13.75 pp(Holm p 0.069), 외관 +6.25 pp(p 0.44) | `REPORTED`, 1차 대비 미달 |
| S4R-1 | 센서 반복성 | 12 부팅 유효, 최대 스텝 0.128 s; 같은 장면 쌍이 7단계 모두 k = 0에서 갈림 | `REPORTED_DESCRIPTIVE` |
| S5 safety v2 | PX4 failsafe 10 시나리오, 정책 없음 | 10/10 통과 | `PASS_SITL`, 실기 판별 NOT_TESTED |

S3a 관측 동등성은 `NOT_EQUIVALENT`(TF32가 정책 잔차를 설명, `TF32_EXPLAINS_POLICY_RESIDUAL`)인 채다. S4의 손실은 색
규칙이 Gazebo 조명 아래 표적 화소 26개 중 중앙값 5개만 통과시키는 것, 지면 필터 여유 0.05 m 등과 "consistent with"이지만
S4F-1은 어느 요인도 Holm 후 1차 대비에 이르지 못했다. SITL 데이터 유틸리티 감사: ULog 462개 중 460 완전, 매니페스트
0 누락 / 0 불일치; S4 vs S4F-1 장면 일치 κ ≈ 0.

**런타임·야외 프로파일.** ROS 2 카메라 전용 런타임은 CPU 소스 검토만 `PASS_WITH_LIMITS`(ROS 그래프, DDS, PX4, 카메라,
기체 NOT_TESTED). 야외 프로파일 독립 검토 r3(Opus 5.5): 기준 1 LIMITED_VALIDITY, **기준 2(실제 근거리 장애물을 놓치지
않음) FAIL**, 3 PASS, 4·5 LIMITED_VALIDITY, 6·7 PASS. 실패 이유: 보호는 시야 안 유효 실루엣 모서리를 필요로 하는데 시야보다
넓은 밝은 면·상단을 채우는 면·다른 거리의 물체 틈으로 보이는 면(18/24)은 해제되어 정책이 열린 들판처럼 명령한다
(dv 2.0–2.6 m/s). 단일 프레임의 깊이 0 화소에는 거리가 없으므로 합성 규칙은 유령 벽과 누락을 맞바꿀 뿐이고, 임계값은
실제 OAK-D 녹화로만 정할 수 있다(D-2026-11-15-1). **야외 GO 없음.**

## 5. 논의 — 심투리얼로 가는 길에서 알게 된 것

1. **LiDAR의 값은 "보정"이 아니라 "옆"이다.** min-fusion 구조상 시야 안 LiDAR는 카메라 누락만 메우는데 그 몫이 −0.69 pp
   뿐이고, 시야 밖이 +39.25 pp다. 따라서 카메라 전용 설계의 문제는 깊이 품질이 아니라 **시야 밖 운동**이다. F는 시야
   밖으로 비행하는 습관(이동 프레임의 51.8 %)을 LiDAR 덕에 가질 수 있었고, 카메라 전용 정책은 그 습관을 버려야 한다.
   CAM-K1이 되찾은 −37 pp는 보상 가중치(요 정렬 2배, 가시성 보너스)와 3,000 epoch의 합이며 분해되지 않았다.
2. **남은 +12 pp는 지금 레시피로는 안 닫힌다.** K2 v2의 −3.80 / −4.74 pp는 처리 ON/OFF에서 거의 같아 적응이 아니라
   추가 학습의 효과로 읽히고, v3 규칙은 2 pp 아래, 속도 상한은 무효, 지연은 해롭다. CAM-T1이 보인 대로 접촉 시 카메라는
   이미 MinZ 안이어서 "보이는데도 부딪힌" 것이 아니라 **마지막 0.4 m를 못 보는 상태로 들어간다.** 다음 손잡이는 관측
   (시야 밖 기억, 이력 길이)이나 시야 유지 행동이지 깊이 후처리가 아니라는 것이 이 결과들의 범위 안 읽기다.
3. **동역학은 SITL에서 전달되지만 지각은 아니다.** 같은 센서 렌더를 주면 PX4 안에서도 같은 결과(S3b)가 나오고, 명령 지연
   1 스텝의 비용(+7.5 pp, S3c; +1.51 pp G7; +4.95 / +3.67 pp CAM-T1)은 정책·기체에 따라 다르지만 방향은 같다. 그러나
   Gazebo 자체 센서는 포착을 반 토막 내고(S4) 원인은 아직 색 규칙·지면·마운트 중 어느 것으로도 귀속되지 않았다. 실기에서
   같은 정책은 **색 규칙 검출기로는 표적 드론을 보지 못한다**(D-2026-11-18-2)는 점이 지각 쪽의 가장 큰 미해결 항목이다.
4. **야외는 하늘 문제가 아니라 "깊이 0에는 거리가 없다"는 문제다.** 세 번의 합성 수리가 모두 근거리 누락 기준에서 실패한
   이유는 규칙의 완성도가 아니라 정보 부족이다. 그래서 다음 단계는 코드가 아니라 **실제 OAK-D 녹화**(책상 시험 → 현장
   최소 세트 22회)다.

## 6. 한계 — 명시

- **전부 시뮬레이션.** 실기 0회, 실제 카메라 프레임 0장, 추력 측정 0건. T/W 4.0, σ_d 0.15 px, 회전 관성은 INFERRED.
- **한 격자, 한 평가 시드 집합(4501–4503), 학습 시드 2개.** 구간은 평가 블록 부트스트랩이며 학습 시드 변동을 포함하지
  않는다. 두 시드가 어긋나면 `SEEDS_DISAGREE`로 두고 풀링하지 않았다.
- **OOD 조건.** C2·G3의 카메라 전용 F는 학습 분포 밖이다. "LiDAR가 필요/불필요하다"는 표현은 쓰지 않는다.
- **기제는 "consistent with"까지.** 사각 접촉(G3), 시야 밖 운동, 색 규칙 통과 화소, 지면 필터 여유 — 모두 측정된 원인이
  아니다. C2 충돌의 접촉 방위는 NOT_MEASURED, CAM-T1의 가림은 NOT_MEASURED.
- **표적 검출기는 빨간 구 색 규칙이다.** 표식 없는 실제 드론은 검출되지 않는다. 이 논문의 모든 포착은 그 규칙 위의
  것이다.
- **깊이 모델의 근거리 틈(MinZ 0.4 m)**의 충돌 효과는 NOT_TESTED이고, v3 규칙의 실제 하늘·무질감·그림자·반사에 대한
  타당성도 NOT_TESTED다.
- **SITL의 충돌은 기하 규칙**(0.14 m 발자국)이지 Gazebo 접촉이 아니며, S3a 관측 동등성은 `NOT_EQUIVALENT`인 채다.
  S5는 실기 판별 NOT_TESTED. ROS 2 런타임은 ROS 그래프·DDS·PX4·카메라·기체 전부 NOT_TESTED.
- **환경 자체의 판정은 `LIMITED_VALIDITY`**(EQ-1D: 160 × 90 깊이 업샘플의 유령, LiDAR 음성 7/7 FAIL). 보정 계보
  S-L1A–D는 opt-in이고 이 연구에는 켜지 않았다.
- **동시 CPU 작업 이탈.** C2 셀 2개와 GR-0 셀 23개가 저자의 CPU 작업과 겹쳤다. T-LOAD(15/15 바이트 동일,
  `LOAD_INVARIANT`)로 결과 파일은 불변이라 읽었지만 겹친 셀의 바이트 쌍둥이는 일부만 있다.
- **외부 비교 0건(Class A = 0).** 이 논문은 어떤 공개 시스템보다 낫다고 말하지 않는다.

## 7. 남은 일

0. **실제 OAK-D 녹화** — 책상 시험(절차서 4번) → 숲 현장 최소 세트 22회(11 장면 × 프로젝터 2). 깊이 좌표 모델
   (`undistorted_pinhole` vs `distorted_rgb`)은 첫 평지 녹화로 정하고, 하늘·지면 임계값은 그 데이터로 확정한다
   (D-2026-11-15-1). 녹화기는 실제 장치에서 0회 실행이다.
1. **표적 드론 검출기** — 색 규칙을 대체할 학습 검출기(YOLOv5s NPS + Det-Fly 계보는 있으나 정책과 연결되지 않음)와
   그 검출기 오차의 정책 비용 측정(P10 방식).
2. **시야 밖 기억 / 시야 유지** — CAM-T1 `DEFINITION_SENSITIVE`와 G3 사각 접촉이 가리키는 손잡이. 재학습은 미승인.
3. **VIO 공유 카메라 통합** — 비행 중 정책은 VIO 쪽 드라이버 스트림을 받아야 한다; 비행 어댑터의 1014 × 760 정렬 깊이와
   10 ms 동기 임계값 결함(Beads motar-783.25)은 아직 안 고쳤다.
4. **S4 손실의 원인** — 조명 하 색 규칙(26 px 중 5 px)·지면·마운트 분리; S3a 동등성 복구(TF32).
5. **실비행** — S3b·S5 통과와 사용자 결정 뒤에만. 이 논문의 모든 수치는 시뮬레이션이다.

## 관련 연구 메모 (`docs/relation_to_published_systems_2026-09-16.md`, `external_baseline_selection_2026-09-19.md`)

가장 가까운 공개 구현은 NavRL(Xu et al., RA-L 10(4), 2025; PPO + 속도장애물 방패, 2.0 m/s 배포 상한)이나 표적을
추적하지 않는다. 표적 추적·계획 쪽은 Elastic Tracker(ICRA 2022; 가시성·가림 목적함수, 선택된 1차 외부 기준선이나 재현
`UPSTREAM_REPRODUCTION_FAIL`, 환경 차단), Fast-Tracker, YOPO / YOPOv2-Tracker(코드 미공개), OPEN(특권 상태 입력, 다중 에이전트).
카메라 전용 RL 추적으로는 DAT / GC-VAT "Open-World Drone Active Tracking with Goal-Centered Rewards"(NeurIPS 2025)가
있으나 장애물 밀집 접근을 다루지 않는다. 어느 것과도 수치 비교하지 않는다(Class A = 0); MOTAR의 조합(측정된 지각 오차 주입, 랜덤 장애물장 PPO 추격, 센서 제거 대조, SITL 이전)은
"연구 범위의 차이이지 우월성의 증거가 아니다."

## 인용해야 할 것

Xu et al. 2025 (NavRL, RA-L) · Ji et al. 2022 (Elastic Tracker, ICRA) · Lu et al. 2024 (YOPO, RA-L) ·
Sun et al. 2025 (DAT / GC-VAT, NeurIPS) · Fox/Burgard/Thrun 1997 (DWA; 아크 필터는 DWA가 아님을 말할 때) ·
Luxonis OAK-D Pro / DepthAI 2.29 문서 (깊이 모델 매개변수, Sync 노드) · PX4 v1.14.3 failsafe 문서 (S5 시나리오) ·
Lakens 2017 (비유의 ≠ 동등성; `NO_DETECTED_CHANGE` 해석).

## 출처 표

| 절 | 단위 | 기록 (브랜치) |
| --- | --- | --- |
| 4.1 | C0, C1, C2 | `docs/evidence/camonly_2026-11-04/{c0,c1c2}/`, 커밋 `86202c5`, `5522be0`, `f821058` (`camonly/dev`) |
| 4.1 | G3, 기제, 센서 | `…/c2trace/`, `…/mechanism/`, `…/sensor/`, 커밋 `05a630e`, `de1a892`, `a7d368b` |
| 4.2 | CAM-K1, CAM-K1-E | `…/camk1/`, `…/camk1e/`, 커밋 `814c28a`, `ab2eeb5` |
| 4.3 | CAM-K2 v1/v2/v3, E, E3 | `…/camk2/`, `…/camk2e/`, 커밋 `aa0e691`, `04f341c`, `d2a018b`, `326e307`, `504eb4e` |
| 4.4 | CAM-T1 | `docs/evidence/camonly_2026-11-04/camt1/RESULTS_2026-11-12.md`, 커밋 `4da43ab` (`camonly/t1`) |
| 4.4 | G7 | `docs/evidence/sitl_2026-11-05/g7/`, 커밋 `b58b389` |
| 4.4 | GR-0, GR-1, GR-1b | `docs/evidence/gain_randomization_2026-11-04/`, 커밋 `2e45ca0`, `6b3ac27`, `e507ff3` (`gainrand/dev`) |
| 4.5 | S3b, S3c, S4, S4F-1, S4R-1, S5 | `docs/evidence/sitl_2026-11-05/{s3b_formal,s3c_s4,s4f1,s4r1,s5_safety_v2}/` |
| 4.5 | 런타임, 야외, 녹화기 | `docs/evidence/sitl_2026-11-05/camera_runtime/`, 커밋 `fc8aa4d`, `ee0161e`, `5d9610e` |
| 6 | T-LOAD | `docs/evidence/tload_2026-11-05/`, 커밋 `26a4a57` |
