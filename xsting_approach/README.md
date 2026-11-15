# xsting_approach

6족 보행 로봇 **xSting**이 전면 RealSense D435 깊이 점군 PCA와 전면 HC-SR04 초음파 센서를 융합해 상자 전면 **2 cm (`standoff = 0.2845 m`)** 지점에 수직 정렬 후 정지하는 계층형 강화학습(50Hz Approach Policy + 200Hz WalkNet) ROS 2 C++ 패키지입니다.

## 주요 연동 구성

- **하드웨어 (`xsting_hardware`)**: `/joint_states`, `/imu_broadcaster/imu`, `/sonar_broadcaster/range`, `/leg_controller/commands`, `/arm_controller/commands`와 직접 연동됩니다.
- **비전·SLAM (`xsting_bringup` / `xsting_vision`)**: `/camera/depth/image_emitter_on`, `/camera/color/detections`(`class 4 = paper_box`, `0..3 = puck`), 스테레오 오프셋(`color_offset_x = -0.015 m`) 및 `/visual_slam/tracking/odometry`를 사용합니다.
- **매니퓰레이터 충돌 방지 (`xsting_pick_place`)**: 보행 중에만 팔 홈 자세를 유지하고, 정지 또는 2 cm 도달 즉시 `/arm_controller/commands` 발행을 중단해 `pick_place_node`에 팔 제어권을 넘깁니다.
- **Foxglove 실시간 실측 보정 검증**: `/approach/markers`(바닥 평면, 범퍼선, 녹색 기준 상자, 주황 PCA 실측 상자·법선, 초음파 빔·타격점, IMU 중력 화살표, mm/deg HUD), `/approach/points`(바닥/벽면/퍽 색상 점군), `/approach/depth_preview`(깊이+ROI 프리뷰), `/approach/metrics`(실시간 오차 Plot)를 발행합니다.

## 빌드

```bash
cd /home/pa03/Documents/FINAL/mk2_ws
colcon build --packages-select xsting_interfaces xsting_approach --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Foxglove에서 실측 보정 확인 및 저장

```bash
# 1. 로봇 하드웨어 및 전면 카메라·YOLO 브링업 (Foxglove 브리지 포트 8765 활성화)
ros2 launch xsting_bringup xsting.launch.py foxglove:=true

# 2. 실측 보정 라이브 노드 실행 (기본 전방 20cm 기준 상자, 브리지 미실행 시 foxglove:=true 추가)
ros2 launch xsting_approach calib.launch.py box_gap_m:=0.200
```

1. **Foxglove Studio**에서 `ws://<로봇_IP>:8765`에 연결한 뒤 `src/xsting_approach/config/xsting_foxglove_calib.json` 레이아웃을 불러옵니다.
2. **3D 패널**에서 녹색 기준 상자(20 cm)와 현재 주황색 PCA 측정 상자, 하늘색 바닥 점군, 청록색 초음파 타격점, 자홍색 IMU 중력 벡터 및 상단 수치 HUD를 실시간으로 확인합니다.
3. 로봇을 평지에 정지시킨 뒤 하단 **`/calib/imu` Call** 버튼을 눌러 IMU 수평·자이로 바이어스를 저장합니다.
4. 차체 전면 범퍼(`X = 0.162 m`) 앞 정확히 `20.0 cm`에 상자를 놓은 뒤 **`/calib/cam` Call** 버튼을 누르면 바닥 평면(`base_z_m`, `cam_rpy_deg`), 상자 PCA(`cam_pos_m`), 초음파 영점(`range_bias_m`)이 동시에 계산·저장되며 화면의 주황색 상자와 초음파 구가 녹색 기준 상자에 즉시 정렬됩니다.

## 정밀 접근 실행 및 액션 호출

```bash
# 정밀 접근 노드 실행 (모터 무출력 점검 시 dry_run:=true, 단독 시각화 시 foxglove:=true)
ros2 launch xsting_approach approach.launch.py

# 상위 미션 액션 호출 (/approach_box)
ros2 action send_goal /approach_box xsting_interfaces/action/ApproachBox "{timeout_sec: 35.0, dry_run: false}" --feedback

# 실행 중인 노드에 보정값 즉시 재반영
ros2 service call /approach/reload_calib std_srvs/srv/Trigger "{}"
```
