# Vision 담당 TODO

담당 범위: xsting_vision, bringup의 vision 설정·카메라 launch, GetTarget 계약.
목표: 팔 담당자가 즉시 사용할 수 있는 신선하고 좌표계가 명확한 물체 목표를 제공.

## 준비된 것

- [x] 빌드 가능한 xsting_vision_core와 target_check 검증 함수.
- [x] Q5 프로파일: best_effort / volatile / KEEP_LAST 1.
- [x] 오래된 관측·미변환 좌표·낮은 신뢰도·NaN·잘못된 회전 테스트.

## P0 — 세 담당자와 먼저 합의

- [ ] 카메라 모델·드라이버·RGB/깊이 정합·CameraInfo 토픽 확정.
- [ ] Hardware 담당과 camera_optical→arm_base TF 및 시간 동기화 합의.
- [ ] RL 담당과 물체 ID 유지 방식, 목표 자세/접근 방향, 허용 관측 나이·신뢰도 합의.
- [ ] GetTarget의 target_version·실패 이유·요청 max_age 적용 규칙 확정.

완료 조건: 실제 녹화 샘플 1개와 토픽·좌표·단위 문서를 다른 두 담당자가 읽고 재생 가능.

## P1 — 최소 기능 구현

- [ ] object_detector를 C++ 컴포넌트로 구현하고 카메라/전처리와 동일 프로세스에서 구성 가능한지 확인.
- [ ] 동기화 큐와 추론 대기 큐를 분리. 대기 프레임은 최신 1개만 유지.
- [ ] Detection3DArray에 고유 object_id, 실제 관측 stamp, frame_id, class_id, confidence, 자세 기록.
- [ ] 검출 시각의 TF로 좌표 변환. TF 부재 시 최신 TF로 대체하지 않고 재관측.
- [ ] target_check를 캐시 저장/조회 경로에 연결하고 /objects/get_target 서비스 구현.
- [ ] xsting_bringup/launch/vision.launch.py 및 config/vision.yaml 추가.

완료 조건: rosbag 재생 → 검출 → 변환 → 목표 조회까지 연결. 오래된 캐시 조회는 success=false.

## P2 — 테스트 추가

- [ ] RGB·깊이 시각 불일치와 깊이 결측 샘플 테스트.
- [ ] TF 지연·부재·시뮬레이션 시각 역행 시 캐시 폐기 테스트.
- [ ] 검출 물체 교체/추적 ID 변경 시 이전 목표 반환 금지 테스트.
- [ ] ROS 실행 테스트로 실제 발행 QoS·서비스 실패 응답 확인.
- [ ] 입력 폭주 중 메모리/대기 개수가 무한 증가하지 않는지 측정.

기존 실행: `colcon test --packages-select xsting_vision --return-code-on-test-failure`.

## P3 — Jetson 최적화와 전달

- [ ] 해상도·빈도·모델 크기별 프레임 나이, p50/p95/p99 지연, CPU/GPU 메모리 기록.
- [ ] 프로세스 내부 전달의 실제 복사 감소 확인; 외부 기록 구독 추가 시 다시 측정.
- [ ] RL 추론 동시 실행 시 지연·열·전력 비교. 주행 충돌 감시 경로는 계속 활성화.
- [ ] 최종 샘플 bag, 실행 명령, 보정 파일, 측정표를 docs에 전달.

수락 조건: 합의한 관측 나이 안에 목표 제공, 잘못된 목표 거절, RL 담당의 통합 집기 시험 통과.
