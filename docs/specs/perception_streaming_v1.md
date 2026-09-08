# Streaming perception v1

`tools/perception_streaming.py`의 공개 객체:

- `FrozenDetector(weights, expected_sha256, yolov5, device)` — 기존 P4와 동일한
  tile640/overlap128, batch8, confidence floor .001, NMS .45, FP32, Top-5.
- `StreamingSelector(checkpoint, expected_sha256, device)` — P7c v2의 T16 buffer와 logits.
- `PerceptionPipeline(detector, selector).process(frame_bgr, sequence, timestamp_ns)` —
  RGB 배열부터 후보·motion·최종 rank 및 단계별 시간 반환. rank=None이면 NO_LOCK.

호출은 단일 스트림에서 시간순으로 직렬화해야 한다. source sequence 변경 또는 explicit
`selector.reset()`으로 이력을 초기화한다. 동일 sequence의 비단조 timestamp와 해상도 변경은
오류다. frame은 uint8 BGR이며 현재 frozen detector는 두 변이 모두 640 px 이상이어야 한다.
GT annotation은 API 입력에 없으며 평가 도구만 이를 읽는다. 결과 rank는 해당 호출의
`candidates` 배열을 가리키며 물리적인 target identity/track ID를 의미하지 않는다.

기본 grayscale은 `cv2.cvtColor(BGR, COLOR_BGR2GRAY)`다. 과거 motion v2는 JPEG를
grayscale로 직접 decode했으므로 byte parity 평가에는 optional `gray_frame` 인자로
동일 JPEG의 grayscale decode를 전달한다. JPEG grayscale decode와 BGR→gray의
반올림 차이가 있을 수 있다. 실제 카메라 입력의 정확한 parity는 이번 검증 범위 밖이다.

`verify_perception_streaming.py --mode replay`는 frozen candidate+motion을 streaming
buffer에 공급하고 전체 offline window와 logits 및 선택 rank를 비교한다.
`--mode rgb`는 validation 이미지 전체를 다시 검출하고 motion을 계산한다.
RGB 실행 환경은 `/home/fair/workspaces/aerial_gym_ws/datasets/detenv/bin/python`이다.
일반 aerialgym 환경은 detector의 pandas 의존성이 없어 RGB 실행에 사용하지 않는다.

실행 예 (저장소 루트):

```bash
/home/fair/workspaces/aerial_gym_ws/datasets/detenv/bin/python tools/verify_perception_streaming.py \
  --data-root /home/fair/workspaces/aerial_gym_ws/detector_runs/results/nps_detfly_joint_final \
  --run /home/fair/workspaces/aerial_gym_ws/detector_runs/runs/perception_temporal/candidate_motion_transformer_v2 \
  --mode rgb \
  --weights /home/fair/workspaces/aerial_gym_ws/detector_runs/runs/nps_detfly_joint/yolov5s_ms_b8_e30_s0/weights/best.pt \
  --yolov5 /home/fair/workspaces/aerial_gym_ws/datasets/yolov5 \
  --output /tmp/motar_stream_rgb_new.json
```

출력 경로는 미존재 경로여야 한다. 이미지 디코딩은 모델 처리 밖에 별도 측정하고
with_decode에 합산한다. FPS는 1000/mean(ms)이며 카메라 전송·ROS·PPO·비행 제어 비용은
포함하지 않는다. 이 버전은 직렬 처리 API이며 frame drop/queue/backpressure 정책을
실장한 카메라 서비스가 아니다.

P7e crop verifier는 별도 실험 branch로 보존했다. 64D learned feature를 histogram에
대입하려면 temporal selector도 그 feature로 재학습해야 하며 현재 checkpoint와 호환되지 않는다.

## 검증 상태

P7e utility 0.52352는 P7c v2 0.68554보다 낮아 채택하지 않았다.
현재 파이프라인의 기본 선택기는 P7c v2다. 캐시 replay는 validation 2,296프레임의
logits와 rank가 모두 일치했다. RGB 재검출 실행은 과거 캐시 대비 rank 5개가 달라
엄격한 rank parity gate가 FAIL이며 CLI는 report를 보존한 뒤 exit 1을 반환한다.
기존 P4 생성기를 현재 환경에서 재실행한 첫 프레임도 과거 cache와 달랐으며, 그 값은
새 detector와 일치했다. 실행환경/수치 재현성 원인은 아직 확정하지 않았다.
이는 카메라 배포 승인이나 완전한 offline/online parity PASS가 아니다.
상세 측정은 [결과](../../results/perception_p7e_streaming_2026-09-08/README.md)를 본다.
