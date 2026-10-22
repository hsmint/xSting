# xSting 개발 시작 구조

주행·집기·보관·배치를 위한 ROS 2 프로젝트입니다. 직접 관리하는 ROS 패키지는 5개입니다.
현재 제공 범위는 **빌드 가능한 패키지, 통신 타입, 재사용할 검증 모듈, 자동 테스트, 담당자별 개발 목록**입니다.
카메라 검출기·학습된 팔 모델·Nav2·OpenCR 호스트 드라이버·전체 작업 실행 노드는 아직 연결하지 않았습니다.

```text
xSting/
├── firmware/             # 기존 OpenCR 코드
├── xsting_interfaces/    # 메시지 3개·액션 2개·서비스 4개
├── xsting_bringup/       # 실행 점검·설정·로봇 모델·지도
├── xsting_robot/         # 팔 명령 검사·최신 입력 슬롯; 작업/팔 노드 구현 위치
├── xsting_vision/        # 인식 결과 검사; 영상 컴포넌트 구현 위치
├── xsting_hardware/      # 호스트 통신 상태 검사; 장치 드라이버 구현 위치
├── training/             # 모델 배포 검증·학습 환경 구현 위치
├── docs/                 # 담당별 TODO·시험 계획·인터페이스 계약
└── test.sh               # 빌드와 전체 자동 테스트
```

기존 `firmware/mk1_arm` 소스는 수정하지 않았습니다. 개발 위치는 이 README가 있는 `xSting/`이며 패키지는 firmware와 같은 깊이입니다.
프로젝트 표기는 xSting, ROS 패키지는 소문자 xsting_를 사용합니다.

## 시작하기

검증 환경은 ROS 2 Lyrical, C++17, Python 3입니다. 실제 Jetson 배포판은 별도로 확정해야 합니다.
먼저 설치된 ROS 환경을 불러옵니다. 아래 스크립트는 패키지를 설치하거나 펌웨어를 업로드하지 않습니다.

```bash
source /opt/ros/lyrical/setup.bash
bash test.sh
```

직접 빌드·시험하려면:

```bash
colcon build --base-paths xsting_interfaces xsting_bringup xsting_robot xsting_vision xsting_hardware \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select xsting_interfaces xsting_bringup xsting_robot xsting_vision xsting_hardware \
  --return-code-on-test-failure
colcon test-result --verbose
python3 -m unittest discover -s training/tests -v
ros2 launch xsting_bringup robot.launch.py mode:=check
```

`mode:=check`는 개발 구조 확인 메시지만 출력합니다. 실기·시뮬레이션 모드는 아직 지원하지 않으며 명시적으로 오류를 반환합니다.
기본 라이브러리와 테스트는 GPU·카메라·직렬 포트·로봇 없이 실행됩니다.
패키지 메타데이터의 maintainer와 license는 프로젝트에서 확정할 값으로 교체하세요.

## 현재 작성된 코드

| 위치 | 실제 동작 | 아직 제공하지 않는 것 |
|---|---|---|
| xsting_interfaces | ROS 타입 생성·직렬화 | 서비스/액션 서버 |
| xsting_vision | 관측 시각·좌표계·ID·신뢰도·유한 좌표·회전 검사 | 카메라/TF 변환·검출 모델·물체 추적 |
| xsting_robot | 후보 명령의 작업/세대/관측/모델 ID·만료·관절 순서·증분·위치 제한 검사 | 충돌/속도/가속도 검사·궤적 생성·실제 추론·액션 실행 |
| xsting_robot | 대기 입력 1개, 작업자에 넘긴 입력의 불변 소유권 | ROS 구독 콜백/모델 작업자 연결 |
| xsting_hardware | 피드백 순번·부팅 ID·단조 시각·만료·명시적 재활성화 검사 | 프레임 파서·직렬 통신·모터 정지·펌웨어 워치독 연결 |
| training | 모델 명세·차원·정규화·SHA-256 검사 | ONNX 실행·성공률·실제 모델 학습 |

헤더와 CMake export를 통해 검증 코드를 실제 노드에서 재사용할 수 있습니다.
반환된 빈 문자열은 해당 검사 통과만 뜻하며 로봇 전체 동작의 안전성을 보장하지 않습니다.
`reject_reason()` 이후 취소가 발생할 수 있으므로 실제 명령 전송 직전의 상태 확인과 전송은 하나의 소유자에서 직렬화해야 합니다.

## 담당별 작업

- [Vision 담당 TODO](docs/TODO_VISION.md)
- [Hardware 담당 TODO](docs/TODO_HARDWARE.md)
- [RL 담당 TODO](docs/TODO_RL.md)
- [통신 계약과 공통 담당](docs/CONTRACTS.md)
- [자동 테스트와 다음 통합 시험](docs/TEST_PLAN.md)
- [이번 검증 결과](docs/VALIDATION.md)
- [전체 설계와 다이어그램](../ros-robot-architecture.html)

패키지 수는 코드 관리 기준입니다. 작업/팔 추론/영상/주행/장치 제어의 실행 경계는 유지합니다.
