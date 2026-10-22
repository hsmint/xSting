# RL 담당 TODO

담당 범위: training, xsting_robot의 팔 모델·동작·작업/보관 관리, 모델/팔/작업 설정.
목표: 시뮬레이션에서 학습한 팔 정책을 Jetson에서 검증된 후보 목표로 실행.
작업 관리와 보관 통합은 담당 공백을 없애기 위해 RL 담당에 함께 배정한 초기안입니다.

## 준비된 것

- [x] ArmInput/ArmTarget, RunTask/MoveArm, 슬롯·모델 서비스 타입.
- [x] command_check: 작업/세대/관측/모델·시간·관절 순서·증분·위치 제한 검사.
- [x] 추론 대기 입력 최신 1개 슬롯과 동시 접근 테스트.
- [x] 모델 manifest·정규화·차원·파일 SHA-256 검증 도구.
- [x] 취소·늦은 결과·오래된 상태·NaN·관절 한계 및 Vision→팔 검증 테스트.

## P0 — 학습과 실기 계약 확정

- [ ] Hardware 담당에게 관절 순서·단위·URDF·한계·실제 갱신 지연을 받기.
- [ ] Vision 담당과 목표 자세·시각·object_id 및 재관측 조건 확정.
- [ ] 관측 벡터 순서/차원, 정규화, action_scale, 출력 형식 문서화.
- [ ] 기본 실행 방식인 구간 완료 후 재추론과 학습 시간 규칙을 일치시킴.
- [ ] 파지 성공·접촉·슬롯 점유·배치 성공의 실제 증거 정의.

완료 조건: 동일 시험 입력에 대한 학습 환경/내보낸 모델/Jetson 출력 비교 규격 확보.

## P1 — 학습·평가·배포

- [ ] 시뮬레이터 버전·학습 환경 구현, 관측 노이즈/지연·질량·마찰 변화 적용.
- [ ] 집기·보관·회수·배치 동작의 보상과 실패/종료 조건 구현.
- [ ] 학습과 분리한 물체·위치·시드로 평가하고 성공률/실패 원인 기록.
- [ ] ONNX 내보내기와 manifest.json/evaluation.json 생성.
- [ ] validate_model.py 통과 후 실제 ONNX 추론·수치 일치 테스트 추가.
- [ ] Jetson FP32/FP16·CPU/GPU 비교; 정확도 저하와 지연을 함께 확인.

## P2 — 추론·팔 실행 연결

- [ ] arm_model 별도 프로세스: ROS 작업자 1개 + 추론 작업자 1개, 배치 1·사전 할당·워밍업.
- [ ] QoS helper와 최신 슬롯을 실제 /arm/input, /arm/target 콜백에 연결.
- [ ] 활성 작업 중 모델 교체 거부, 실패 시 이전 모델 유지, 유휴 상태에서만 교체.
- [ ] arm_executor의 비동기 목표/취소/결과와 입력 구성 구현.
- [ ] command_check를 최종 명령 경로에 연결하고 충돌·속도·가속도·궤적 생성 검사를 추가.
- [ ] 취소 확인과 최종 전송을 같은 소유자에서 직렬화해 검사 후 취소 경쟁 방지.
- [ ] FollowJointTrajectory/그리퍼 액션 연결. 명령 완료와 실제 파지 성공을 구분.

## P3 — 작업·보관 통합

- [ ] task_manager의 주행→정지→재관측→팔 동작→검증 순서 구현.
- [ ] storage_manager의 예약·버전·중복 커밋 방지·UNKNOWN 복구 구현.
- [ ] 주행/집어서 보관/집어서 배치/보관 물체 꺼내 배치 4개 시나리오 구성.
- [ ] tasks.yaml, arm.yaml, slots.yaml, model.yaml 작성 후 bringup에 연결.
- [ ] 물체 관측 지연·취소·모델 오류·제어기 실패·통신 단절에서 완료/실패 결과 검증.

기존 실행: `colcon test --packages-select xsting_robot --return-code-on-test-failure`,
`python3 -m unittest discover -s training/tests -v`.

최종 전달: 모델+명세+평가 결과, 실행 명령, 입력/출력 비교 샘플, Jetson 지연표, 4개 시나리오 결과.
수락 조건: 검증되지 않은 명령이 제어기로 전달되지 않고 취소 후 늦은 추론 결과가 폐기됨.
