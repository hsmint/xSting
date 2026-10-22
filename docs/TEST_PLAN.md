# 테스트 구성과 다음 단계

자동 테스트는 현재 구현한 검증 모듈과 생성된 ROS 타입을 실제로 실행합니다.
통신 서버·실제 모델·모터가 없는 상태를 성공한 로봇 동작으로 가장하지 않습니다.

| 테스트 파일 | 확인하는 실패/정상 조건 | 담당 |
|---|---|---|
| xsting_interfaces/test/test_wire_contracts.py | 생성 타입 직렬화, 작업/세대/입력 ID·만료·관절 데이터 보존 | 공통 |
| xsting_vision/test/test_vision_contracts.cpp | 신선도 경계·미래 시각·좌표계·신뢰도·NaN·회전·Q5 | Vision |
| xsting_hardware/test/test_hardware_contracts.cpp | 무응답·재생 순번·재부팅·단조 시각·만료·명시적 재활성화 | Hardware |
| xsting_robot/test/test_robot_contracts.cpp | 취소·세대·관측·모델·시간·관절·한계·최신 슬롯·Q2 | RL |
| xsting_robot/test/test_perception_to_arm.cpp | 유효 인식 뒤 취소된 팔 후보 차단, 인식 노후화 | Vision + RL |
| xsting_bringup/test/test_bringup_contracts.py | 개발 기본 설정·미구현 모드 거절·QoS/큐 계약 | 공통 |
| training/tests/test_manifest.py | 모델 명세·해시·차원·정규화·실행 방식 | RL |

C++ 테스트는 GoogleTest, ROS 타입/설정 시험은 pytest, 학습 도구는 Python unittest를 사용합니다.
현재 실행 테스트는 DDS 네트워크나 카메라/USB를 사용하지 않습니다. QoS 설정 값 검증은 실제 전달·손실 시험과 구분합니다.

## 다음 구현에서 추가할 ROS 통합 시험

| ID | 입력/장애 주입 | 기대 결과 | 담당 |
|---|---|---|---|
| V-ROS-01 | 샘플 bag의 RGB/깊이/TF 재생 | 검출·조회 서버에서 올바른 arm_base 목표 반환 | Vision |
| V-ROS-02 | TF 누락·오래된 캐시·시간 역행 | GetTarget 실패, 잘못된 목표 발행 없음 | Vision |
| H-IO-01 | fake transport로 부분 프레임/CRC 오류 | 무효 프레임이 피드백 시각을 연장하지 않음 | Hardware |
| H-IO-02 | USB 단절/MCU 재부팅 | 실제 정의된 정지/유지 동작, 명시적 복구 전 재실행 금지 | Hardware |
| R-ROS-01 | 목표 시작 후 취소, 늦은 모델 응답 주입 | 제어기로 추가 목표 전달 0회 | RL |
| R-ROS-02 | 모델 교체 중 실패/활성 목표 | 교체 거부 또는 이전 모델 유지 | RL |
| R-STORE-01 | 동시 슬롯 예약/중복 커밋/실패 | 단일 소유권·버전 일치·불명확 상태 복구 | RL |
| ALL-01 | 4개 작업 시나리오 | 정지 후 조작·파지 증거·최종 배치 검증 | 세 담당 |

이 표의 미구현 시험을 skip/xfail로 자동 통과 목록에 넣지 않습니다. 실제 어댑터가 생기면 실행 가능한 launch_testing/가짜 장치 시험으로 추가합니다.

## 성능 측정

동일한 데이터·모델·열/전력 조건에서 변경 전후를 비교합니다. 목표 수치는 첫 장치 측정 후 확정합니다.
- Vision: 관측 나이, 프레임 드롭, p50/p95/p99 지연, GPU 메모리, 외부 구독 시 복사 비용.
- Hardware: read/update/write 및 버스 왕복 지연, 주기 초과 횟수, 단절 감지→실제 정지/유지 시간.
- RL: 입력→후보 지연, 취소→마지막 명령, 모델 로드/워밍업 시간, 작업 성공률.
- 통합: Vision+RL 동시 부하에서 제어 주기·최악 지연·CPU/GPU/RAM·열 스로틀링.

기능 테스트 통과는 실시간 마감이나 로봇 성공률을 증명하지 않습니다.
