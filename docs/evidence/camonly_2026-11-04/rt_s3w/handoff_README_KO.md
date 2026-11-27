# 학습 측 → Jetson 측: S3-W 정책 전달 v0 (2026-11-28, RT-S3W)

표기: [기록] 학습·평가 기록에서 확인 / [미확인] / [예정]

## 0. 이 묶음으로 할 수 있는 것과 없는 것
- **할 수 있는 것:** 프로펠러를 뗀 탁상 시험, 손에 든 그림자 모드(제안 명령만 기록하고 PX4에는 보내지 않음), 체크포인트 로딩과 Orin NX 스텝 시간 측정.
- **아직 하면 안 되는 것: 정책이 기체를 조종하는 비행.** 사용자 승인이 없다.
  - 시뮬레이션 평가(CAM-S3-E, 205개 막대)에서 충돌 25.4 %(목표 20 % 미달), 포획 72.6 % [기록].
  - 야외 녹화 분석의 지면 오검출·하늘 가짜 장애물 문제가 아직 열려 있다.
  - 런타임 변경(RT-S3W)은 독립 리뷰 전이다 [예정].

## 1. 파일
| 파일 | 내용 | SHA-256 |
|---|---|---|
| `s3w_ep18000.pth` | CAM-S3 S3-W, 18,000 에포크 마지막 체크포인트(`last_gen`) | `2626a4b24eb29b11c469904ac1a4e06de0349a04e9c83164b23a1e6308fa98ee` |
| `s3w_ep18000_obs_norm.npz` | 관측 정규화 running_mean, running_var(898), count, epsilon 1e-5, clip 5.0 (`tools/camera_policy_export_obs_norm.py`, CAM-S2a 묶음과 같은 형식) | `08dc318c29704002912580c7b86f272288897b8bdd47b172c17520c8bd15ce5e` |
| `s3w_training_receipt.json` | 학습 환경 전체 설정과 소스 SHA | `3ae1448904391b1ca5b037ac3990b6fc02836a0fcf1b9db3627280bfb0574c05` |
| `SHA256SUMS` | 위 세 파일의 해시 | — |

## 2. S2b-FT와 달라진 것 [기록]
| 항목 | S2b-FT | S3-W |
|---|---|---|
| 요레이트 최대 | 60 °/s (1.0472 rad/s) | **90 °/s (1.5708 rad/s)** — PX4 MC_YAWRATE_MAX(200 °/s)가 아니라 런타임 명령 스케일이 막는다 |
| 깊이 → 거리 13칸 | 5° 칸마다 열 하나의 최솟값 | **구역 규칙 k=24**: 칸 안 모든 열·모든 행에서 24번째로 가까운 거리 |
| 깊이 결손 | 학습 중 무작위 2 % 구멍 | 학습 중 실측 OAK-D 결손 마스크(**학습 전용, 기체에서는 끔** — 실제 센서가 결손을 준다) |
| 그 밖 | 312×234, 66.7°×52.6°, 고도 1.5 m, 지연 1–2 스텝, 시야 밖 1 m/s 제한 | 같음 |

## 3. Jetson에서 계약 만들기
학습 측 커밋(RT-S3W, camonly/s2b)과 같은 소스 트리에서:
```
python tools/camera_policy_contract_cams2.py --output s3w_contract.json \
  --receipt s3w_training_receipt.json --checkpoint s3w_ep18000.pth --obs-norm s3w_ep18000_obs_norm.npz \
  --previous-action-delay-steps 1   # 실측 지연(0.1 s 단위, 1 또는 2)
```
출력에 `"variant": "CAM-S3W"`, `"cams3_sector_k": 24`, `"yaw_rate_max_rps": 1.5708`이 보여야 한다. 런타임은 시작할 때 구역 규칙 설치를 확인하고(로그 `[cams3] install_sector`), 스텝마다 `cams3_sector_ranges_m`(칸별 거리)을 결과에 낸다. 학습 소스 바이트가 receipt와 다르면 계약 생성이 거부된다.

## 4. 그림자 시험에서 기록할 것
- 스텝 시간(캡처 → 제안). 학습 측 호스트 CPU 1스레드 중앙값 약 46 ms [기록, Orin 아님]; Orin NX 값은 미측정 [미확인].
- `cams3_sector_ranges_m`와 실제 장애물 거리 비교(나무·기둥 앞에서).
- `yaw_rate_rps`가 ±1.5708을 넘지 않는지.
