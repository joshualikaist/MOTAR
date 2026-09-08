# P6 GRU + P7 Temporal Transformer 실행 계약

작성: 2026-09-08

상태: **PRE-REGISTERED — TEMPORAL RESULTS UNSEEN**

## 범위와 데이터 격리

P6와 P7은 P4의 동일한 frozen detector와 Top-K=5 candidate record만 사용한다. detector를 재학습하거나
candidate를 모델별로 다시 만들지 않는다. NPS train clip으로 weight를 학습하고 NPS validation clip으로
epoch와 association arm을 선택한다. 이 단계에서는 NPS test candidate/report를 읽지 않는다.

NPS 원 annotation의 target track ID가 YOLO 변환에서 보존되지 않았고 한 frame에 UAV가 여러 개일 수 있다.
따라서 supervision은 **현재 frame에서 어떤 annotation과든 IoU≥0.3인 candidate**이며, 가장 높은 IoU
candidate를 정답으로 쓴다. 그런 candidate가 없으면 `NO_LOCK`이 정답이다. 이것은 temporal UAV-candidate
selection baseline이지 designated-target ReID 또는 ID-switch 평가가 아니다.

## 공통 입력과 학습 조건

후보 feature는 image 크기로 정규화한 `[u,v,w,h]`, confidence, parameter-free P4 appearance 64D의
69차원이다. 한 clip 안에서 최근 sampled frame만 시간순으로 사용하고 clip 경계를 넘지 않는다. 원 source
frame 간격은 capture timestamp로 계산해 frame representation에 포함한다. 미래 frame은 사용하지 않는다.

- optimizer: AdamW, learning rate `1e-3`, weight decay `1e-4`
- batch: 256, seed: 17, 최대 40 epochs, gradient norm clip: 1.0
- dropout: 0.1, early stopping patience: 8
- checkpoint 선택: validation cross-entropy 최소; exact tie면 더 이른 epoch
- detector candidate floor/top-K와 appearance encoder는 P4 v1 그대로 유지

## P6 GRU

- history: 최근 8 sampled frames
- candidate embedding: 128D
- confidence-weighted frame pooling 후 timestamp gap을 결합
- single-layer GRU hidden 128
- 현재 frame의 5 candidate와 `NO_LOCK`을 6-way 분류

## P7 Temporal Transformer

- history: 최근 16 sampled frames
- model dimension 128, 4 heads, 2 encoder layers, feed-forward 256
- confidence-weighted frame token과 timestamp gap, learned temporal position을 사용
- 현재 frame의 5 candidate와 `NO_LOCK`을 6-way 분류

두 모델은 같은 candidate encoder/scoring head 의미를 갖지만 weight를 공유하지 않는다. Transformer는
현재 frame까지의 window만 입력받으므로 미래 정보가 없다.

## validation 비교와 완료 조건

각 모델은 선택 checkpoint로 validation 전체를 한 번 출력하고 prediction/report/receipt SHA-256을 남긴다.
P5 report의 CNN-only와 KF도 같은 candidate/manifest hash인지 검증한 뒤 아래 utility로 네 arm을 비교한다.

```text
selection_utility = (hit_frames - proxy_false_lock_frames) / frames_with_ground_truth
```

utility 최대 arm을 선택한다. exact tie 순서는 `CNN-only → KF → GRU → Transformer`로, 더 단순한 arm을
우선한다. frame hit, no-lock, proxy false lock, center error, loss/reacquisition, association latency를 모두
함께 보고한다. 실제 FTLR/ID switch로 이름을 바꾸지 않는다.

P6/P7 완료 조건은 코드·단위 테스트, train candidate receipt, 두 모델 checkpoint/report/receipt,
validation-only selection receipt가 모두 존재하고 hash 검증되는 것이다. test 평가는 별도 승인 단계다.
