# 검증 결과 — 2026-10-22

환경: ROS 2 Lyrical, C++17/Release, Python 3.14.4. 실제 Jetson이나 OpenCR에서 실행한 결과는 아닙니다.

| 항목 | 결과 |
|---|---|
| 5개 ROS 패키지 colcon build | 통과 |
| Hardware C++ | 10개 통과 |
| Vision C++ | 13개 통과 |
| Robot C++ | 22개 통과 |
| Vision→Robot 검증 연결 | 1개 통과 |
| 생성 ROS 타입 직렬화 | 18개 통과 |
| Bringup 설정·모드 | 6개 통과 |
| 모델 명세·해시 Python unittest | 5개 통과 |
| robot.launch.py mode:=check | 정상 종료 |
| test.sh 셸 문법 | 통과 |

**개별 자동 테스트 총 75개 통과, 실패/건너뜀 없음.**
colcon test-result는 ROS 개별 테스트 70개와 CTest 실행 묶음 6개를 함께 세어 76으로 출력합니다.
Python unittest의 내부 subTest는 별도 테스트 개수로 더하지 않았습니다.

빌드 산출물은 소스 폴더 밖 `/tmp/xsting-scaffold-build`, 설치는 `/tmp/xsting-scaffold-install`, 로그는 `/tmp/xsting-scaffold-log`에서 검증했습니다.

```bash
source /opt/ros/lyrical/setup.bash
colcon --log-base /tmp/xsting-scaffold-log build \
  --base-paths xsting_interfaces xsting_bringup xsting_robot xsting_vision xsting_hardware \
  --build-base /tmp/xsting-scaffold-build --install-base /tmp/xsting-scaffold-install \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source /tmp/xsting-scaffold-install/setup.bash
colcon --log-base /tmp/xsting-scaffold-log test \
  --build-base /tmp/xsting-scaffold-build --install-base /tmp/xsting-scaffold-install \
  --packages-select xsting_interfaces xsting_bringup xsting_robot xsting_vision xsting_hardware \
  --return-code-on-test-failure
colcon test-result --test-result-base /tmp/xsting-scaffold-build
python3 -m unittest discover -s training/tests -v
ROS_LOG_DIR=/tmp/xsting-launch-log ros2 launch xsting_bringup robot.launch.py mode:=check
```

실행 위치는 xSting 프로젝트 최상위입니다. 실제 검증 시 작업 디렉터리 차이에 따라 base-paths에 xSting/ 접두사를 사용했습니다.
빌드 과정에서 Lyrical에 없는 구형 CMake 의존성 매크로를 표준 imported target 연결로 교체했습니다.
Bringup 테스트 로그는 홈 디렉터리 대신 빌드 디렉터리에 저장하도록 설정했습니다.

## 검증하지 않은 범위

DDS 네트워크 손실·QoS 실제 전달, 카메라/TF 재생, 실제 모델 추론, Nav2 작업 실행,
직렬 버스·펌웨어 컴파일/업로드·모터 정지, Jetson 지연·전력·성공률은 아직 검증하지 않았습니다.
다음 시험과 담당은 TEST_PLAN.md 및 TODO_VISION/HARDWARE/RL.md에 기록했습니다.
