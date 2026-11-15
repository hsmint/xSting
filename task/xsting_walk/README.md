# xsting_walk

Isaac Lab(`skrl` PPO)에서 학습된 6족 로봇 **xSting**의 카펫 전방향 보행·회전 정책(`74D` 관측 + $C^1$ Hermite 스윙 CPG + `18D` RL 잔차 제어)을 **`mk2_ws` 프로젝트 구조에 맞춰 순수 C++20 ROS 2 패키지**로 통합한 텔레옵 및 실측 보정 패키지입니다.

## 주요 구성 및 연동

- **하드웨어 연동 (`xsting_hardware`)**: `/joint_states`, `/imu_broadcaster/imu` 구독 및 200Hz `/leg_controller/commands` (`leg1..6_yaw/hip/knee`), 보행 중 팔 고정 `/arm_controller/commands` 발행 (정지 시 즉시 팔 제어권을 해제해 `xsting_pick_place`와 충돌하지 않음).
- **오도메트리 자동 폴백 (`xsting_bringup`)**: `/visual_slam/tracking/odometry` 실행 시 cuVSLAM 선속도를 사용하고, 카메라/SLAM 없이 `xsting_hardware`만 단독 실행할 때는 **6족 스탠스 발끝 기구학 오도메트리**로 자동 전환되어 단독 텔레옵이 가능합니다.
- **멀티 입력 텔레옵**:
  - `teleop_key_node`: 전후·좌우·4대각선·제자리 회전·좌우 커브·속도 단수(`20%~100%`)·`Hold/Latch` 전환을 지원하는 순수 C++ 터미널 키보드 텔레옵
  - `/cmd_vel` (`Twist`), `/cmd_vel_stamped` (`TwistStamped`), `/joy` (게임패드), `/walk/mode` (`String`: `forward`, `left`, `diag_fl`, `turn_left`, `curve_left` 등 `play.py` 13개 모드 호환)
- **실측 보정 도구 (`calib_walk_wizard.sh`)**:
  1. 관절 영점(`offset_rad`), 개별 관절 회전 방향(`+6°` 왕복 점검 및 부호 반전), 보행 표준 기립 자세 검증 (`calib_joints_node`)
  2. 평지 IMU 장착 회전(`mount_quat`), 자이로 바이어스, 가속도계 중력(`+9.81 m/s²`) 바이어스 보정 (`calib_imu_node`)
  3. 전진(`vx`)·횡이동(`vy`)·제자리 회전(`wz`) 정량 시험 주행 후 **줄자·각도기 실측값(또는 cuVSLAM/IMU 자동 계측값)** 으로 슬립 배율(`cmd_vel_scale`) 및 직진 편향(`drift_vy_per_vx`) 자동 산출 (`calib_gait_node`)
  4. 카펫 파일 두께에 따른 스윙 높이(`swing_clearance_m`) 조정 및 무중단 실시간 반영 (`/walk/reload_calib`)
- **원본 Isaac Lab 번들 보존**: 학습 환경·스크립트·로그는 `isaaclab/` (`COLCON_IGNORE` 적용, `xsting_description` 공용 USD 자동 참조)에 보존되어 있으며, 재학습 시 `scripts/export_walk_policy.py`로 `models/walk_policy_74d.bin`을 재생성할 수 있습니다.

## 빌드 및 테스트

```bash
cd /home/pa03/Documents/FINAL/mk2_ws
source /opt/ros/lyrical/setup.bash
colcon build --packages-select xsting_walk --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select xsting_walk --event-handlers console_direct+
source install/setup.bash
```

## 텔레옵 실행 방법

```bash
# 1. 하드웨어 실행 (비전/SLAM 함께 사용 시 xsting_bringup 추가 실행 가능)
ros2 launch xsting_hardware hardware.launch.py

# 2. C++ 전방향 보행 제어 노드 실행 (무출력 테스트 시 dry_run:=true, 게임패드 드라이버 동시 실행 시 joy_driver:=true)
ros2 launch xsting_walk walk_teleop.launch.py

# 3. 새 터미널에서 C++ 키보드 텔레옵 실행
ros2 run xsting_walk teleop_key_node
```

### 명명 모드 / CLI 제어 예시

```bash
# play.py 동일 프리셋 모드 (stop, forward, backward, left, right, diag_fl, diag_fr, diag_rl, diag_rr, turn_left, turn_right, curve_left, curve_right)
ros2 topic pub --once /walk/mode std_msgs/msg/String "{data: 'forward'}"
ros2 service call /walk/stop std_srvs/srv/Trigger "{}"
ros2 service call /walk/reload_calib std_srvs/srv/Trigger "{}"
```

## 실측 보정 실행

```bash
# 하드웨어 및 walk_node가 켜진 상태에서 대화형 실측 보정 마법사 실행
ros2 run xsting_walk calib_walk_wizard.sh src/xsting_walk/config/calibration.yaml
```

## Foxglove 실측 보정 모니터링 및 즉시 보정

1. **Foxglove 브리지와 함께 실행**
   - `xsting_bringup` 미실행(단독 하드웨어 보정) 시:
     ```bash
     ros2 launch xsting_walk walk_teleop.launch.py foxglove:=true
     ```
     (`robot_state_publisher`, 정적 `odom -> base` TF, `foxglove_bridge:8765`가 자동 실행됩니다.)
   - `xsting_bringup` 실행 시: `ros2 launch xsting_bringup xsting.launch.py foxglove:=true` 후 `ros2 launch xsting_walk walk_teleop.launch.py`만 실행하면 됩니다.

2. **Foxglove Studio 연결 및 레이아웃 불러오기**
   - 연결 주소: `ws://<로봇IP>:8765` (로컬 실행 시 `ws://localhost:8765`)
   - 레이아웃 Import:
     - 보행 보정 전용: `src/xsting_walk/config/xsting_foxglove_walk_calib.json`
     - 통합(SLAM + YOLO + 보행 보정 탭 3·4 포함): `src/xsting_bringup/config/xsting_foxglove.json`

3. **Foxglove 제공 시각화 토픽 및 즉시 수정 명령**
   - `/walk/vis` (`MarkerArray`): 목표 속도(청록), 보정 명령 속도(주황), 실측 속도(초록) 화살표, 보정 중력 방향(하늘색), 6족 발끝 FK 구(지지=초록 / 스윙=노랑 / 목표=보라), 상단 3D 요약 배너
   - `/walk/calib_vis` & `/walk/calib_path`: 시험 주행 시 **10cm 단위 가상 줄자 눈금**, 목표 도달 지점, 실제 주행 궤적, 좌우 편차(빨간 화살표), 추천 스케일 배율 3D 보드
   - `/walk/calib_info` & `/walk/calib_trial` (`DiagnosticStatus`): 현재 적용된 전체 보정값 및 실시간 시험 주행/IMU/관절 계측 현황
   - `/walk/imu_calibrated` (`Imu`), `/walk/cmd_user`, `/walk/cmd_calibrated`, `/walk/vel_measured` (`TwistStamped`): 보정 전·후 IMU 및 목표·보정·실측 속도 실시간 Plot 비교
   - `/walk/calib_cmd` (`String`, Foxglove Publish 패널 지원):
     - `reload` (보정 파일 재로드), `clear_vis` (궤적 마커 초기화)
     - `vx_scale=1.08`, `vy_scale=1.05`, `wz_scale=1.03`, `drift_vy=-0.02`, `swing_clearance=0.040`
     - `apply_vx:0.20:3.0:0.56:0.01` (`속도:시간:줄자실측거리:좌우쏠림` 입력 시 자동 배율·편향 산출 및 저장)
