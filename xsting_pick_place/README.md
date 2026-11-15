# xsting_pick_place

`xsting_description` 및 `xsting_moveit_config` 기반 **Isaac ROS cuMotion + MoveIt 2** 상자 위 퍽 적재/하역 패키지입니다.

## 주요 기능

- **상자 적재 (`load`)**: `205 x 205 x 70 mm` 상자 위 랜덤 배치된 퍽 2개(`Ø30 x 60` 원형 / `30 x 30 x 60` 사각형)를 인식해 12슬롯 팔레트(`1..12`)에 적재합니다.
- **누운 사각 퍽 세우기**: 누워 있는 사각 퍽은 파지 후 손목을 세워 상자 위 빈 공간에 올리고 그리퍼를 살짝 풀어(`grip_loosen`) 수직 안착시킨 뒤 재파지하여 적재합니다.
- **팔레트 하역 (`unload`)**: 지정 슬롯의 퍽을 상자 위 기본 2구 배치 또는 `mission.pdf` 기준 `3x3` 주사위 눈(`1..6`) 좌표에 수직 배치합니다.
- **플래너 자동 전환**: `isaac_ros_cumotion` 설치 시 GPU 기반 cuMotion을 기본 파이프라인으로 사용하며, 미설치 또는 계획 실패 시 `ompl`로 자동 전환합니다.

## 의존성 및 빌드

```bash
# 1. Isaac ROS cuMotion 런타임·빌드 의존성 설치
sudo apt update && sudo apt install -y \
  ros-lyrical-cumotion-vendor \
  ros-lyrical-isaac-ros-cumotion-interfaces \
  libassimp-dev

# 2. 워크스페이스 빌드 (Ubuntu 26.04 libassimp6 / MoveIt 2.15.2 호환 cuMotion 오버레이 포함)
cd /home/pa03/Documents/FINAL/mk2_ws
source /opt/ros/lyrical/setup.bash
colcon build --packages-select \
  isaac_ros_cumotion \
  isaac_ros_cumotion_moveit \
  xsting_interfaces \
  xsting_description \
  xsting_moveit_config \
  xsting_pick_place \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

## 실행 방법

```bash
# 1. Mock 하드웨어 단독 테스트
ros2 launch xsting_pick_place pick_place.launch.py hardware:=mock pipeline:=cumotion

# 2. Isaac Sim 연동 (터미널 1에서 run_isaac_sim.sh 실행 -> Play 클릭 후 터미널 2에서 아래 실행)
ros2 run xsting_pick_place run_isaac_sim.sh
ros2 launch xsting_pick_place pick_place.launch.py hardware:=isaac pipeline:=cumotion

# 3. 실기 연동 (xsting_bringup 실행 상태)
ros2 launch xsting_pick_place pick_place.launch.py hardware:=real
```

## 명령 전송 예시 (`send_mission.py`)

```bash
# 데모 적재 (세워진 원형 퍽 1개 + 누운 사각 퍽 1개 세우기 후 슬롯 1, 2 적재)
ros2 run xsting_pick_place send_mission.py load --demo --slots 1 2

# 실시간 카메라(/detections_3d) 기반 상자 위 퍽 2개 적재
ros2 run xsting_pick_place send_mission.py load --slots 1 2

# 슬롯 1, 2 퍽을 상자 위 주사위 눈 2 위치에 하역
ros2 run xsting_pick_place send_mission.py unload --slots 1 2 --eye 2

# 홈 자세 복귀
ros2 run xsting_pick_place send_mission.py home
```

## 실측 현장 보정 (`calib_pick_place.py`)

다리 자세별 지면 높이(`ground_z`), 카메라-팔 정합 오프셋(`pick_off_*`), 팔 처짐 계수(`droop_k`), 슬롯 오프셋(`slot_off_*`), 누운 사각 퍽 기립 이완 폭(`grip_loosen`)을 현장에서 즉시 보정하고 `pick_place.yaml`에 영구 저장합니다.

```bash
# 대화형 보정 콘솔 실행
ros2 run xsting_pick_place calib_pick_place.py

# 주요 대화형 명령 예시
calib> zero                 # 물리 홈 자세에서 팔 6관절 영점 오차 확인
calib> vision 0.02          # /detections_3d 검출 퍽 +20mm 상공 프로브 이동
calib> jog 0.002 -0.003 -0.020  # 패드 중심 미세 조그 이동(m)
calib> apply pick           # 누적 조그를 pick_off_x/y/z에 반영 및 실시간 동기화
calib> grip loosen          # 누운 사각 퍽 기립용 살짝 풀기 폭 구동 확인
calib> set grip_loosen 0.0076
calib> slot 1 0.01          # 바스켓 1번 슬롯 프로브 이동
calib> save                 # src 및 install의 pick_place.yaml에 영구 저장
```

