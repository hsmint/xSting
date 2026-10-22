# 공통 계약과 담당

구현 전에 세 담당자가 아래 초안을 함께 확정합니다. 숫자는 개발용 초기값이며 실측 성능 기준이 아닙니다.

| 영역 | 주 담당 제안 | 다른 담당자에게 전달할 것 |
|---|---|---|
| 영상·물체 목표 | Vision | object_id, 관측 시각, arm_base 기준 자세, 신뢰도 |
| 관절·통신·URDF·주행 장치 | Hardware | 관절 순서·단위·한계, 실제 상태, 부팅/연결 상태, TF |
| 팔 정책·작업 순서·보관 상태 | RL | 모델 명세, 후보 목표, 작업/관측 ID, 평가 결과 |
| 전체 launch | Hardware 기본 실행 + 각자 노드 연결 | 카메라·주행·제어기 순서, RL 작업 수락 조건 |
| 인터페이스 변경 | 해당 기능 담당 작성, 나머지 2명 검토 | 타입 변경과 생산자/소비자 테스트를 같은 변경에 포함 |

RL 담당에게 작업 관리·보관·통합 액션 연결을 함께 배정한 초기안입니다. 팀에서 별도 통합 담당을 정하면 해당 TODO만 이관합니다.

## ROS 연결

| 이름 | 타입 | 생산자 → 소비자 | 규칙 |
|---|---|---|---|
| /objects/detections | vision_msgs/msg/Detection3DArray | Vision → 물체 캐시/팔 입력 | Q5: best_effort, volatile, depth 1; ID·stamp·frame 유지 |
| /objects/get_target | xsting_interfaces/srv/GetTarget | 작업 → Vision | 기본 ROS 서비스 QoS; 오래된 캐시는 실패 응답 |
| /arm/input | xsting_interfaces/msg/ArmInput | 팔 동작 → 팔 모델 | Q2: reliable, volatile, depth 1 |
| /arm/target | xsting_interfaces/msg/ArmTarget | 팔 모델 → 팔 동작 | Q2; 검사 전 후보값, 직접 모터에 전달 금지 |
| /arm/run | xsting_interfaces/action/MoveArm | 작업 → 팔 동작 | 비동기 목표·취소·결과 |
| /task/run | xsting_interfaces/action/RunTask | PC → 작업 | NAVIGATE/PICK_STORE/PICK_PLACE/STORE_PLACE |
| /storage/reserve_slot | xsting_interfaces/srv/ReserveSlot | 작업 → 보관 관리 | task_id·expected_version 기반 예약 |
| /storage/commit_transfer | xsting_interfaces/srv/CommitTransfer | 작업 → 보관 관리 | 예약 ID·버전·실제 증거 확인, 중복 커밋 방지 |
| /arm/load_model | xsting_interfaces/srv/LoadModel | 배포 도구 → 팔 모델 | 활성 목표가 없을 때만 검증 후 교체 |
| /joint_states | sensor_msgs/msg/JointState | Hardware → 모델/TF | 실측 시각·관절 이름; 관절별 발행 주체 1개 |
| /cmd_vel | 배포판에 맞는 Twist 또는 TwistStamped | Nav2 → Hardware | 최종 충돌 검사 후, 타입 합의 및 명령 만료 확인 |

동작의 상대 토픽 이름과 네임스페이스를 노드에서 지원해야 합니다. 위 절대 이름은 단일 로봇 논리 명칭입니다.
ROS 메시지 시간은 같은 ROS clock 영역에서 비교합니다. 호스트 연결 만료는 단조 시계를 사용합니다.
시뮬레이션 시간 재설정 시 작업·캐시·대기 입력을 폐기합니다. LinkGuard의 boot_id/sequence는 신규 호스트 계약이며 기존 펌웨어가 이미 제공한다고 가정하지 않습니다.

## 팔 명령 계약

- 모델은 관절별 상대 위치 증분(rad)을 반환합니다. 실기 실행은 구간 완료 후 다음 추론 방식입니다.
- 모델 관절 순서·정규화·출력 스케일을 manifest로 고정합니다. 현재 실제 관절 이름/제한은 미확정입니다.
- 후보 출력은 task_id, generation, input_id, model_version을 현재 요청과 대조합니다.
- 현재 관절 상태·입력 시각도 확인합니다. 출력 시각만 최신이라고 오래된 입력을 허용하지 않습니다.
- guard 통과 뒤에도 충돌·속도·가속도 제한, 궤적 변환, 전송 직전 취소 검사는 별도로 구현합니다.
- 최신 슬롯은 대기 1개만 유지하지만 현재 실행 중인 추론을 취소하지 않습니다.

## 설정과 API 상태

contracts.yaml의 수치와 C++ 검사 함수에 넘길 설정은 아직 노드로 연결되지 않았습니다.
인터페이스의 오류 코드 표·timeout 상한·문자열/배열 크기 상한·슬롯 상태 enum은 구현 전에 확정합니다.
자동 테스트에서 사용하는 j1/j2와 ±1 rad는 시험용 가상 관절이며 실기 설정으로 쓰지 않습니다.
