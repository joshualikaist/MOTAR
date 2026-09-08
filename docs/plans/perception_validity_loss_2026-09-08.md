# Candidate validity 학습 비교 계약

Validation 진단 후 설계한 후속 개발 실험이다. test는 사용하지 않는다.
기존 corrected P7c v2와 architecture, 후보, motion v2 cache, seed 17,
batch 256, AdamW, 최대 40 epoch, patience 8을 동일하게 고정한다.
기존 one-best CE 대신 IoU>=0.3인 모든 후보를 positive로 하는 masked BCE를 사용한다.
현재 후보 중 최대 validity logit이 0 이상이면 그 후보를 고르고, 아니면 NO_LOCK이다.
학습하지 않는 no-lock head 대신 고정 logit 0을 사용한다. 동점은 candidate 우선이다.
epoch는 validation BCE 최소, arm은 기존 utility 최대; 동점은 기존 P7c를 유지한다.
출력 probability는 independent sigmoid이며 합이 1인 분포가 아니다.
기존 report cross_entropy 필드는 BCE arm에서는 binary CE이며 loss_kind로 구분한다.
기존과 같은 7 validation clips를 재사용하므로 confirmatory 성능 주장은 하지 않는다.
