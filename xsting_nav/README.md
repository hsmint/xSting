# xsting_nav

XSting 6족 로봇 강화학습 전역 주행(41D PPO MLP + 10초 보행자 대기·다익스트라 동적 우회 + 100Hz 해석적 3자유도 보행기) 및 **상자 검출 후 사전 접근 지점 5~10cm 이내 도달 시 2cm 정밀 접근(`/approach_box`) 자동 전환** ROS 2 C++ 패키지입니다.

## 주행 → 정밀 접근 파이프라인

1. **전역 주행 (`NAVIGATING`)**
   - `/nav/run` (`xsting_interfaces/action/RunTask`, `operation: "NAVIGATE"`, `place_id: "HOME" | "S1"~"S6" | "L1"~"L3"`) 또는 `/nav/goal_id` 요청을 수락합니다.
   - `nvBlox` ESDF + 360° `COIN-D4` LiDAR(`/scan_filtered`) + 전방 초음파(`/sonar_broadcaster/range`)를 융합하여 주행하며, 복도 내 동적 장애물(사람) 조우 시 제자리 10초 대기 후 다익스트라 우회 경로를 재탐색합니다.
2. **상자 검출 및 사전 접근 목표 보정 (`5~10cm` 판정)**
   - `/detections_3d` (`xsting_vision::DepthPcaNode`, `paper_box`) 검출 시 실제 상자 위치를 기준으로 D435 스테레오 사각지대(12cm) 밖인 **범퍼 전방 약 20cm 사전 접근 지점**을 보정합니다.
   - 상자가 검출된 상태에서 사전 접근 지점까지 **`pre_approach_tol_m` (기본 `0.08m` = 8cm, `0.05~0.10m` 범위)** 및 목표 방위각 이내에 도달하면 정지 후 기립 자세를 안정화합니다.
   - 목표 지점 5~10cm 이내에 도달했으나 상자가 아직 시야에 잡히지 않은 경우 제자리 미세 좌우 탐색(`SEARCHING_BOX`)을 수행하여 상자를 찾은 즉시 전환합니다.
3. **정밀 접근 전환 (`CALLING_APPROACH` → `APPROACH_ALIGNED`)**
   - `/leg_controller/commands` 및 `/arm_controller/commands` 제어권을 완전히 양보한 뒤 `xsting_approach`의 `/approach_box` 액션을 직접 호출합니다.
   - `xsting_approach`가 상자 전면 2cm 정렬 및 2초 정지 유지를 완료하면 `/nav/goal_reached = true` 발행과 함께 `/nav/run` 액션을 최종 완료(`ErrorCode.OK`)합니다. (`HOME` 복귀 목표는 상자 정밀 접근 없이 출발 구역 도착 후 즉시 완료됩니다.)

## 빌드 및 단위 테스트

```bash
cd /home/pa03/Documents/FINAL/mk2_ws
source /opt/ros/lyrical/setup.bash
colcon build --packages-select xsting_interfaces xsting_approach xsting_nav xsting_mission --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select xsting_nav --event-handlers console_direct+
```

## 실행 방법

### 1. 주행 노드 + 정밀 접근 노드 동시 실행

```bash
source /opt/ros/lyrical/setup.bash
source /home/pa03/Documents/FINAL/mk2_ws/install/setup.bash

# xsting_nav + xsting_approach 함께 기동
ros2 launch xsting_nav nav.launch.py with_approach:=true
```

### 2. 미션 스택(`xsting_mission`)과 통합 구동

```bash
# 터미널 1: 하드웨어 + 비전/SLAM 기동 후 주행 + 정밀 접근 실행
ros2 launch xsting_nav nav.launch.py with_approach:=true

# 터미널 2: 미션 브리지 + 판단 노드 실행 (nav_mode:=action)
ros2 launch xsting_mission mission.launch.py nav_mode:=action
```

### 3. 단독 액션/토픽 테스트 및 실기체 보정

```bash
# S1 상자 주행 + 5~10cm 도달 후 /approach_box 정밀 접근 연속 실행
ros2 action send_goal /nav/run xsting_interfaces/action/RunTask \
  "{operation: 'NAVIGATE', place_id: 'S1', timeout: {sec: 120}}" --feedback

# 실기체 IMU / 18축 다리 영점 / HOME 원점 / 카펫 슬립 보정 마법사
ros2 run xsting_nav calib_nav_wizard.py
```
