# Perception candidate interface v1

상태: **P4 INTERFACE PREP — producer/encoder 미구현**

기계 판독 정본: `docs/specs/motar_perception_candidates_v1.schema.json`

한 레코드는 detector가 한 frame에서 낸 최대 5개 후보를 나타낸다. 후보 벡터는
`[u_px, v_px, width_px, height_px, confidence, appearance_64d]`이고 confidence 내림차순이다.
`rank`는 0부터 연속해야 하며 detector candidate일 뿐 track ID가 아니다.

- 좌표 원점은 이미지 왼쪽 위, `u`는 오른쪽, `v`는 아래이며 `u/v`는 box 중심이다.
- `capture_timestamp_ns`는 센서 frame의 monotonic capture clock이다.
- `inference_completed_timestamp_ns`는 같은 monotonic clock의 출력 완료 시각이며 capture보다 빠를 수 없다.
- 검출이 없거나 frame이 missing이면 후보를 꾸며내지 않고 `candidates=[]`를 쓴다. frame 레코드 자체는 남긴다.
- appearance는 64차원 L2 unit vector다. crop 규칙·encoder checkpoint가 정해지기 전에는 임의 zero vector를
  쓰지 않는다. encoder 이름과 SHA-256이 없는 레코드는 v1 유효 레코드가 아니다.
- candidate box는 image bounds 안에 있어야 하고, 동일 confidence에서는 detector가 낸 안정 순서를 유지한다.

JSON Schema가 표현하지 못하는 다음 semantic 조건은 producer validator에서 검사한다.

1. `capture_timestamp_ns <= inference_completed_timestamp_ns`
2. candidate rank가 `0..K-1`로 연속이고 confidence가 비증가
3. box 네 경계가 image bounds 내부
4. appearance의 모든 값이 finite이고 L2 norm이 허용 오차 내 1
5. 한 sequence 안에서 frame index와 capture timestamp가 각각 단조 증가

Det-Fly의 파일명 숫자는 순서를 시사하지만 배포 metadata에는 FPS와 capture timestamp가 없다. 따라서 P3
raw prediction을 v1 연속영상 레코드로 위조 변환하지 않는다. 실제 timestamp가 보존된 NPS 원본 또는 별도
연속 영상에서 producer와 KF/GRU/Transformer 비교를 수행한다.
