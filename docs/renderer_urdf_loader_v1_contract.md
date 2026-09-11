# URDF 자산 로딩 v1 — 구현 계약 (2026-09-11)

외형 개편 계획의 **V1 선행 작업**이다. 독립 렌더러 프로토타입(R1–R4b, background v1)에는 자산
로더가 없다. 장면을 코드에서 프리미티브로 만들어 왔고, R3이 예상보다 빨랐던 이유도 그것이다.
V1(표적을 요격기와 같은 기체로 통일)을 **검증**하려면 URDF를 읽어 삼각형으로 바꾸는 경로가 먼저 있어야 한다.

이것은 실험이 아니라 구현이다. 가설도 임계도 없다. 따라서 사전등록이 아니라 계약이다.

## 무엇을 만드는가

URDF의 **visual** geometry를 기존 `MeshScene`(정점·삼각형·face별 material·face별 instance)으로
바꾼다. collision geometry는 읽지 않는다. 카메라가 보는 것은 visual이다.

## 지원 범위 — 저장소 실측에 근거한다

저장소의 URDF에서 geometry 태그를 세면 box 1107, cylinder 118, sphere 14, **mesh 3**이다.
mesh를 쓰는 셋(BlueROV, morphy)은 외형 개편 경로에 없다.

- **지원**: `<box>`, `<sphere>`, `<cylinder>`, fixed joint, 링크당 복수 `<visual>`,
  visual의 `<origin xyz rpy>`, `<material name>`의 중첩 `<color rgba>`, robot 수준 material의 이름 참조.
- **거부(fail closed)**: `<mesh>`, fixed가 아닌 joint, 없는 링크를 가리키는 joint, 순환,
  루트 복수, 모르는 geometry, 비유한 수, 음수/영 치수. 전부 예외로 멈춘다.

mesh 미지원은 **한계이지 결함이 아니다.** 필요해지면 계약을 갱신하고 STL부터 추가한다.

## 고정하는 규약

- `rpy`는 고정축 roll-pitch-yaw다: `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`.
- 링크의 월드 변환은 루트에서 joint origin을 연쇄한 것이다. joint는 전부 fixed이므로 관절값이 없다.
- **instance = 링크** (문서 순서). 렌더러의 instance_id와 같은 뜻이다.
- **material = URDF material 이름** (첫 등장 순서). 이름 없는 visual은 선언된 대체 material
  `__urdf_default__`(0.5, 0.5, 0.5, 1.0)를 쓰고, 그렇게 처리된 visual 수를 receipt에 남긴다.
- URDF 좌표: 미터. 원기둥은 +Z 축, 원점 중심, 길이 L. 구는 원점 중심. 상자는 원점 중심.
- 삼각형 winding은 **바깥쪽**이다: `cross(p1-p0, p2-p0)`가 형상 바깥을 향한다.

## 색에 대한 명시적 미결 사항

URDF의 `rgba`를 **변환 없이 그대로** base_color로 쓴다. URDF 색은 표시용(sRGB에 가까운) 값인데
이 렌더러의 Lambertian은 base_color를 linear RGB로 다룬다. **즉 색공간 변환을 하지 않는다는 선택을
선언한 것이지, 두 값이 같다고 주장하는 것이 아니다.** 알파는 무시하고 원본 rgba를 receipt에 남긴다.
변환이 필요해지면 선언된 단계로 나중에 추가한다.

## 곡면 분할

구와 원기둥은 분할 수가 필요하다. 모듈 상수로 고정하고 출력에 기록한다:
`SPHERE_SEGMENTS=16`(경도), `SPHERE_RINGS=8`(위도), `CYLINDER_SEGMENTS=16`.
분할 수를 바꾸면 삼각형 수와 렌더 비용이 바뀌므로 임의로 조정하지 않는다.

## 검증

- CPU 단위 테스트: 파싱, 변환 연쇄, rpy 규약, 분할, **모든 삼각형 법선이 바깥**,
  **닫힌 표면**(모든 모서리가 정확히 삼각형 2개에 공유), 결정성(두 번 읽으면 바이트 동일),
  거부 경로 전부.
- **부피 수렴 검사**: 분할한 구·원기둥의 부피가 해석값에 선언된 오차 안에서 수렴한다.
  이것이 분할이 실제로 그 형상을 만들었다는 가장 강한 증거다.
- GPU 기술 smoke: 실제 요격기 URDF를 읽어 렌더하고, 링크와 material이 화면에 보이는지 확인한다.

## 이 작업이 말하지 않는 것

렌더링 품질, shortcut 감소, 학습 성능에 대해 아무 주장도 하지 않는다. V1의 자산 교체 자체도
아직 하지 않는다. 이것은 그것을 **검증할 수 있게 하는** 도구다.
