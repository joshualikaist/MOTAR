# P4 candidate producer + P5 Kalman association 실행 계약

작성: 2026-09-08  
상태: **IMPLEMENTED — frozen detector 실행 대기**

P4는 NPS 원본 clip의 container FPS와 원 frame 번호로 capture timestamp를 복원하고, 각 frame에서 native
640/128 sliding-window detector 결과를 image-level NMS한 뒤 confidence 순 Top-K=5를 기록한다. 후보마다
box·confidence와 재현 가능한 parameter-free RGB-grid/histogram 64D L2 descriptor를 출력한다. 낮은 score도
후속 비교에서 같은 입력을 쓰도록 producer floor는 `0.001`이다.

P5 상태는 `[u,v,log(w),log(h),du,dv,dlog(w),dlog(h)]` constant-velocity KF다. 4D innovation의
chi-square p=0.99 gate, appearance cosine cost, confidence penalty를 결합해 최대 5×5 exact assignment를
수행한다. 새 track threshold 0.25, update threshold 0.05, 2 hit confirm, 최대 coasting 0.5초를 고정한다.
기계 판독값은 `configs/perception_kf_v1.json`이 정본이다.

최종 detector hash를 동결한 뒤 NPS validation에서 pipeline 무결성을 확인하고, 같은 설정으로 NPS test를
한 번 평가한다. CNN-only는 confidence≥0.25인 top-1, KF arm은 같은 P4 후보의 confirmed track을 쓴다.
IoU≥0.3 frame hit, no-lock, non-overlap proxy false lock, center error, loss/reacquisition, detector P/R,
producer/KF latency를 보고한다.

NPS YOLO 변환 당시 원 annotation의 track ID가 보존되지 않았고 일부 frame에는 여러 UAV box가 있다.
따라서 실제 FTLR와 ID switch는 측정 불가능하며 proxy를 해당 지표로 이름 바꾸지 않는다. 카메라 intrinsic도
없으므로 degree bearing error는 보류한다.
