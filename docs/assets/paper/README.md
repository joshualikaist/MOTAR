# 논문형 블록 다이어그램

결과 요약 카드가 아니라 모듈의 입출력과 처리 흐름을 보여주는 그림입니다.
영문 라벨·흰 배경·단순 박스와 화살표로 구성했습니다. SVG는 벡터 원본, PNG는 3840×2160입니다.

## Fig. 1. Streaming perception pipeline

RGB 입력에서 검출과 광류를 병렬 계산한 뒤 후보·motion feature를 결합하여 시간 이력과 Transformer 선택기에 전달합니다.
출력은 프레임 안의 후보 rank 또는 NO_LOCK입니다. 물리적 track ID나 거리·자세 추정 출력을 뜻하지 않습니다.
GT는 평가용이며 추론 입력에 포함되지 않습니다. 카메라→PPO 실기 통합은 그리지 않았습니다.

근거: [Streaming perception v1](../../specs/perception_streaming_v1.md).

## Fig. 2. Direction-preserving speed governor

LiDAR 처리 경로와 정책 명령 경로를 분리했습니다. 정책 명령의 방향은 회랑 선택에, 원래 명령은 크기 조정에 전달됩니다.
선택된 clearance로 얻은 cap은 수평 명령의 크기만 제한합니다. 방향 재계획이나 수직 제어 블록은 아닙니다.
이 그림은 기존 **직선 회랑 기준선**입니다. 후속 arc variant를 같은 구조로 묶지 않았습니다.

근거: [기존 상세 그림](../motar-safety-filter.svg) 및 README의 safety-filter 설명.

재생성: `python tools/render_paper_blocks.py` (Chrome, websocket-client 필요).
새 실험이나 알고리즘 변경 없이 기존 문서만 재도식화했습니다.
