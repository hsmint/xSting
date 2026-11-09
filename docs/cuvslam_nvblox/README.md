# xSting cuVSLAM + nvblox 핸드오프 문서

이 문서 하나로 다른 세션(사람 또는 Claude)이 **현재 구성을 이해하고, 다시 실행하고, 문제를 해결**할 수 있게 정리했다.
작성: 2026-10-29. 이 날짜 이후 바뀐 내용은 git log와 대조할 것.

- 실행 도구: [`tools/`](tools/) (Jetson에는 `~/calibration/`에 같은 파일이 복사돼 있음)
- 보정 결과·설정: [`calibration/`](calibration/)
- 문제가 생기면 먼저 [7. 트러블슈팅](#7-트러블슈팅)과 [8. 기준값](#8-정상일-때의-기준값)을 볼 것

---

## 1. 시스템 개요

| 항목 | 내용 |
|---|---|
| 로봇 | xSting mk1: TurtleBot3 Waffle Pi 차체, 바퀴 4개(스키드 스티어), 팔(rm-x52, 판 아래에 매달려 접힘), 맨 윗판 앞쪽 브래킷에 **RealSense D435**(IMU 없음, D435i 아님) |
| 바퀴 OpenCR | Dynamixel ID **201**, 2층판 앞쪽, 펌웨어 `firmware/mk1_motor`. **cuVSLAM이 쓰는 IMU(MPU9250)가 여기 있음** |
| 팔 OpenCR | ID **200**, 윗판 (`firmware/mk1_arm`) |
| 로봇 컴퓨터 | Jetson Orin Nano, `pa03@jon.local` (IP 예: 10.2.12.61, DHCP라 바뀔 수 있음), JetPack R39.2, ROS 2 **Lyrical**, Isaac ROS **5.0** (bare metal) |
| 분석 PC | Ubuntu 24.04 x86, ROS 2 Lyrical, RTX 노트북. Kalibr 계산·RViz용. 로봇 하드웨어는 연결 안 됨 |
| 카메라 사용 | cuVSLAM은 **IR 좌우 영상**(infra1/2, **1280×720@30**, 11-02부터. 그전엔 640×480)만 사용. 컬러는 안 씀. nvblox는 **깊이 영상** 사용 |
| IR 프로젝터 | **꺼서 사용** (점 무늬가 cuVSLAM·Kalibr를 방해). 아래 [함정 1](#7-트러블슈팅) 참고 |

### 1-1. 무엇을 보정했고 어디에 들어가 있나

| cuVSLAM이 쓰는 값 | 출처 | 들어가는 곳 |
|---|---|---|
| 카메라 내부값 | **D435 공장값** (Kalibr 재투영 0.09px로 확인) | 드라이버의 `camera_info`가 자동 제공 |
| 가속도계 오차(스케일·바이어스) | 6면 보정 v2 | Jetson `~/calibration/opencr_accel_v2.yaml` → `apply_calib_node` |
| 자이로 바이어스 | 시작할 때 정지 측정 (8000샘플 ≈ 45초) | `apply_calib_node` 실행 시마다 |
| 카메라–IMU 위치·각도 | Kalibr (motion8/9: 각도 평균, 위치 motion9) | URDF `d435_joint` |
| 카메라–IMU 시간차 | Kalibr: **+17.1 ms** (IR 1280×720. 640×480은 +12.4 ms) | `imu_time_shift.py` (IMU 시각 −17.1 ms) |
| IMU 노이즈 | **임시값** (데이터시트×10, Allan 미실시) | cuVSLAM 런치 파라미터 |

---

## 2. 데이터 흐름과 TF

```
[OpenCR ID201] --USB--> ros2_control (200 Hz) --imu_sensor_broadcaster--> /imu  (~165-200 Hz)
                                                                            |
                                                   apply_calib_node (opencr_accel_v2 + 자이로 바이어스)
                                                                            v
                                                              /opencr/imu_corrected
                                                                            |
                                                   imu_time_shift.py (stamp - 17.1 ms)
                                                                            v
[D435] infra1/2 (30 Hz) ---------------------------> cuVSLAM <------- /opencr/imu_synced
[D435] depth (30 Hz) ------> nvblox <---- TF (odom -> ... -> camera_depth_optical_frame)
```

nvblox까지 쓸 때(`wheel_odom_tf:=false`, cuVSLAM `tf:=true`)의 TF 트리:

```
map --(cuVSLAM)--> odom --(cuVSLAM)--> base_footprint --(URDF)--> base_link
   base_link -> opencr_link -> imu_link                         (URDF, CAD 위치)
   base_link -> turret_link -> front_camera_link -> camera_link (URDF, d435_joint = Kalibr 결과)
   camera_link -> camera_infra1/2_frame -> *_optical_frame, camera_depth_frame ... (RealSense 드라이버)
```

- 평소(nvblox 없이)에는 `diff_drive_controller`가 `odom -> base_footprint`를 내고 cuVSLAM은 TF를 안 낸다.
  **한 프레임에 부모가 둘이면 안 되므로 둘 중 하나만** 내야 한다.
- URDF의 `camera_link`는 **RealSense용**이다. 그리퍼 카메라는 `gripper_camera_link`로 이름을 바꿨다.
  드라이버는 반드시 `camera_namespace:=camera camera_name:=camera`로 띄울 것(프레임·토픽 이름이 이걸 전제로 함).

---

## 3. 파일 지도

### 3-1. 저장소 (`~/git/xSting`, 브랜치 `control`)
| 파일 | 내용 |
|---|---|
| `firmware/mk1_motor/include/xSting.hpp`, `src/xSting.cpp` | IMU 제어 테이블 갱신 20 ms → **5 ms** (`INTERVAL_MS_TO_UPDATE_IMU`). `xSting.cpp`는 **CRLF** 파일 |
| `control/xsting_description/config/controllers.yaml` | `update_rate: 200` |
| `control/xsting_base_hardware/src/xsting_base_system.cpp` | `write()`: 하트비트·바퀴 명령을 각각 50 Hz로, **서로 다른 주기에** 전송 (200 Hz 주기 안에 OpenCR 통신이 몰리지 않게) |
| `control/xsting_description/urdf/mk1_xsting.urdf.xacro` | xSting_51 CAD 형상 + `ros2_control` 블록. `d435_joint`, `imu_link`(opencr_link 아래), `gripper_camera_*` |
| `control/xsting_description/launch/xsting_control.launch.py` | `wheel_odom_tf` 인자 (2026-10-29 기준 **미커밋**) |
| `control/xsting_description/config/diff_drive_no_odom_tf.yaml` | `wheel_odom_tf:=false`일 때 덧붙이는 설정 (**미커밋**) |
| `docs/cuvslam_nvblox/` | 이 문서, 도구, 보정 결과 |

### 3-2. 도구 (`docs/cuvslam_nvblox/tools/`, Jetson `~/calibration/`)
| 파일 | 어디서 | 용도 |
|---|---|---|
| `cuvslam_stereo.launch.py` | Jetson / PC | cuVSLAM. 인자 `imu`, `tf`, `viz` ([4장](#4-실행-방법)), bag 재생은 PC에서 `use_sim_time:=true` |
| `nvblox_d435.launch.py` | Jetson / PC | nvblox (깊이 → 3D 메시, 2D ESDF). bag 재생은 PC에서 `use_sim_time:=true` |
| `record_nvblox_bag.sh` | Jetson | PC 재생용 센서 원본 녹화(4층 IR·IMU, 깊이·컬러, `/tf`). 3층 D435도 켜져 있으면 같이. 필요한 토픽이 없거나 위치 TF가 겹치면 녹화 안 함 |
| `imu_time_shift.py` | Jetson | `/opencr/imu_corrected` → `/opencr/imu_synced`, 시각 −17.1 ms (640×480이면 `-p shift_ms:=12.4`) |
| `compute_d435_joint.py` | PC | Kalibr 결과(`*-camchain-imucam.yaml`) → URDF `d435_joint` 값 (6장 8번) |
| `cuvslam_monitor.py` | Jetson | cuVSLAM 위치·추적 상태를 한 줄로 출력 (영상 전송 없음) |
| `ir_preview.py` | Jetson | IR 영상을 320×240, 5 fps로 줄여 재발행 (Wi-Fi로 볼 때) |
| `cuvslam.rviz` | PC | 카메라 모드 + `viz:=true`용 (특징점 지도, 영상 겹쳐보기) |
| `cuvslam_nvblox.rviz` | PC | nvblox 메시·ESDF·경로 (PC에 `ros-lyrical-nvblox-rviz-plugin` 필요, 설치됨) |
| `board_preview.py` | PC | 체커보드 검출 미리보기 (**Wi-Fi 대역을 거의 다 씀, 쓰고 나면 꼭 끌 것**) |
| `bag_quality_check.py` | 아무데나 | 녹화 bag의 IMU·영상 주기, 보드 검출률, 구간별 움직임 점검 |
| `opencr_imu_only.py` | Jetson | 로봇에서 뗀 OpenCR의 IMU만 `/imu`로 발행 (모터 없이 ros2_control이 안 뜰 때) |
| `imu_axis_check.py` | Jetson | 6면 가속도 보정 때 어느 축이 위인지·기울기·정지 여부 표시 |
| `accel_calib_verify.py` | Jetson | 6면에서 보정 후 중력 크기 비교 |

### 3-3. 장비별 위치
| 위치 | 내용 |
|---|---|
| Jetson `~/Documents/xSting/control` | 로봇 워크스페이스. git은 **030a434 + 로컬 수정** 상태이고, 위 커밋들의 파일을 **scp로 복사해 빌드**한 상태 (pull 안 함). 정리하려면 `git stash` 후 pull |
| Jetson `~/imu_calibration_ws` | `imu_calib` (mzahana) |
| Jetson `~/calibration/` | 도구, `opencr_accel_v2.yaml`, 녹화 bag (`d435_opencr_motion*`) |
| PC `~/calibration/` | bag 사본, Kalibr 결과 (`d435_opencr_motion6_trim`, `..7_trim`이 최종) |
| PC `~/calibration_tools/kalibr_ws` | Kalibr ROS 2 (roboticsmick/kalibr_ros2 @7c33893 + [패치](calibration/kalibr_ros2_patches.diff)) |
| PC `~/xsting_ws/src/xsting` | 저장소 `control/`로의 심볼릭 링크 |

---

## 4. 실행 방법

모든 Jetson 명령은 `ssh pa03@jon.local` 터미널에서. 각 명령 앞에 `source /opt/ros/lyrical/setup.bash`.

### 공통 터미널
```bash
# A: 로봇 base (IMU 발행). nvblox/TF 모드면 끝에 wheel_odom_tf:=false 추가
source ~/Documents/xSting/control/install/setup.bash && ros2 launch xsting_description xsting_control.launch.py use_mock_base:=false use_mock_hardware:=true
# B: 가속도 보정 적용 + 자이로 바이어스. 시작 후 약 45초 로봇을 건드리지 말 것
source ~/imu_calibration_ws/install/setup.bash && ros2 run imu_calib apply_calib_node --ros-args -r raw:=/imu -r corrected:=/opencr/imu_corrected -p calib_file:=$HOME/calibration/opencr_accel_v2.yaml -p calibrate_gyros:=true -p gyro_calib_samples:=8000
# C: 4층(맨 윗판) D435 (시리얼 261322071425, USB 2-1.3), IR 1280x720 (시간차 17.1 ms가 이 모드 기준). cuVSLAM만이면 이대로,
#    nvblox면 enable_depth:=true depth_module.depth_profile:=1280x720x30, 색을 입히려면 enable_color:=true rgb_camera.color_profile:=1920x1080x30
#    (D435 최고 모드: IR·깊이 1280x720, 컬러 1920x1080, 모두 30 Hz. 다 켜면 카메라 1대에 약 300 MB/s)
ros2 launch realsense2_camera rs_launch.py camera_namespace:=camera camera_name:=camera serial_no:="'_261322071425'" enable_infra1:=true enable_infra2:=true enable_color:=false enable_depth:=false enable_gyro:=false enable_accel:=false depth_module.infra_profile:=1280x720x30
# C 다음에 반드시: 프로젝터 끄기 (launch 인자로는 안 꺼짐)
ros2 param set /camera/camera depth_module.emitter_enabled 0
# C2: 3층(아래) D435 (시리얼 261322072019, USB 2-1.4, URDF lower_camera_link). 아래는 IR·깊이·컬러 모두 최고 모드.
#     두 대를 같이 쓰면 serial_no로 각자 카메라를 지정해야 함(앞의 '_'는 숫자로 읽히지 않게). 시리얼은 rs-enumerate-devices -s.
#     드라이버의 rs_multi_camera_launch.py는 두 카메라 사이에 가짜 TF를 내서 URDF와 충돌하므로 쓰지 않음.
#     두 대 모두 최고 모드면 약 600 MB/s: Orin Nano CPU·디스크에 부담, 프레임이 빠지면 컬러를 1280x720x30으로
ros2 launch realsense2_camera rs_launch.py camera_namespace:=camera camera_name:=lower_camera serial_no:="'_261322072019'" enable_infra1:=true enable_infra2:=true enable_color:=true enable_depth:=true depth_module.infra_profile:=1280x720x30 depth_module.depth_profile:=1280x720x30 rgb_camera.color_profile:=1920x1080x30 enable_gyro:=false enable_accel:=false
# C2 다음: 3층 프로젝터도 끄기 (점 무늬가 앞 카메라 IR에 보이면 cuVSLAM을 방해. 3층 깊이는 무늬 없는 면에서 나빠짐)
ros2 param set /camera/lower_camera depth_module.emitter_enabled 0
# D: IMU 시각 보정
python3 ~/calibration/imu_time_shift.py
```

### 모드별
| 모드 | 필요한 터미널 | cuVSLAM 명령 | 결과 보는 법 |
|---|---|---|---|
| **카메라만** (`tracking_mode 0`) | C | `ros2 launch ~/calibration/cuvslam_stereo.launch.py` | `python3 ~/calibration/cuvslam_monitor.py` (좌표는 카메라 축, 28° 기울어짐) |
| 카메라만 + RViz | C (**A는 끔**) | `... viz:=true` | PC: `cuvslam.rviz`, 영상 겹쳐보기는 Jetson에서 `ir_preview.py` |
| **카메라 + IMU** (`tracking_mode 1`) | A, B, C, D | `... imu:=true` | 모니터 (좌표는 `base_footprint`: x 앞, z 위) |
| 카메라 + IMU + TF (nvblox·Nav2용) | A(`wheel_odom_tf:=false`), B, C(깊이 켬), D | `... imu:=true tf:=true` | + nvblox |
| **nvblox** | 위 + | `ros2 launch ~/calibration/nvblox_d435.launch.py` (색 입힌 메시: `color:=true`, C에 컬러 켬) | PC: `cuvslam_nvblox.rviz` |

PC에서 RViz (PC `~/.bashrc`는 도메인 42·LOCALHOST라 아래처럼 덮어써야 Jetson이 보임):
```bash
source /opt/ros/lyrical/setup.bash && ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET rviz2 -d docs/cuvslam_nvblox/tools/cuvslam_nvblox.rviz
```

### bag으로 PC에서 cuVSLAM + nvblox 돌리기 (Wi-Fi 없이)
Jetson은 센서 원본만 녹화하고, cuVSLAM과 nvblox는 PC에서 돌린다 (Jetson 부담이 적고, 같은 bag으로 설정을 바꿔 여러 번 재생 가능).
1. Jetson: A(`use_mock_base:=false use_mock_hardware:=true wheel_odom_tf:=false`), B, C(IR·깊이 켬, 색을 원하면 컬러 켬),
   D, 필요하면 C2(3층). **cuVSLAM은 띄우지 않음.** `bash ~/calibration/record_nvblox_bag.sh <이름>` → 한 바퀴 → Ctrl+C
   - 녹화: 4층 IR 좌우·IMU(`/opencr/imu_synced`, cuVSLAM 입력), 4층·3층 깊이(+컬러, 3층 IR), `/tf`(바퀴·팔 관절), `/tf_static`
   - 스크립트가 막는 경우: 필요한 토픽이 실제로 발행되지 않을 때, Jetson에서 cuVSLAM이 돌 때, 바퀴 odom TF가 켜져 있을 때
     (bag의 `/tf`에 `odom → base_footprint`가 있으면 PC의 cuVSLAM과 겹침)
   - 처음엔 10–20초만 찍고 `ros2 bag info`로 영상 토픽이 초당 약 30개인지 확인. 두 대 최고 모드는 USB가 못 따라감(11-02 실측)
   - Jetson에서 cuVSLAM까지 돌려 그 결과로만 nvblox를 볼 거면 `--with-slam` (cuVSLAM `imu:=true tf:=true`가 떠 있어야 함)
2. `~/bags/<이름>_<날짜>`를 PC로 복사 (USB·유선 권장)
3. PC: `sudo apt install ros-lyrical-isaac-ros-cuvslam ros-lyrical-isaac-ros-nvblox ros-lyrical-nvblox-examples-bringup` (처음 한 번)
4. PC (저장소 루트에서, 순서대로 터미널 3개):
   ```bash
   ros2 launch docs/cuvslam_nvblox/tools/cuvslam_stereo.launch.py imu:=true tf:=true use_sim_time:=true
   ros2 launch docs/cuvslam_nvblox/tools/nvblox_d435.launch.py use_sim_time:=true color:=true   # 3층도 녹화했으면 lower:=true
   ros2 bag play <bag> --clock
   ```
   RViz는 PC 안에서 `cuvslam_nvblox.rviz` (도메인 덮어쓰기 불필요). 로봇 base launch는 PC에서 띄우지 않음 (`/tf_static`·`/tf`가 bag에 있음)
- cuVSLAM·nvblox 인자(`max_distance`, `voxel_size`, `color`, `lower`)는 처리 방법만 정하므로, 같은 bag을 값만 바꿔 여러 번 재생해도 됨
- `--with-slam`으로 녹화한 bag은 위치가 bag에 있으므로 PC에서 cuVSLAM을 띄우지 않고 nvblox만

### nvblox 설정 요점 (`nvblox_d435.launch.py`)
- 기본값: `nvblox_examples_bringup/config/nvblox/nvblox_base.yaml`(복셀 5 cm, static TSDF, ESDF 2D)
- 덮어쓴 값: `global_frame: odom`, 복셀 3 cm(`voxel_size:=0.05`로 바꿀 수 있음), 컬러 기본 끔(`color:=true`면 5 Hz로 메시에 색만 입힘, 지도·ESDF에는 영향 없음), 깊이 15 Hz 통합,
  메시 계산·전송 0.2 Hz(`update_mesh_rate_hz`, `publish_layer_rate_hz`) + 전송 상한 10 Mbps (2 Hz·3 cm에서 메시가 약 10 MB/s로 Wi-Fi를 막았음), ESDF 2 Hz(10 Hz에서 ESDF 점군만 약 3 MB/s), 최대 거리 3 m(`max_distance:=`, D435 오차가 거리 제곱에 비례해 4–5 m면 10 cm 이상),
  ESDF 슬라이스 높이 0.05–0.6 m(바닥 제외, 로봇 높이까지), 입력 QoS `SENSOR_DATA`
- 출력: `/nvblox_node/mesh`, `/nvblox_node/static_esdf_pointcloud`, `/nvblox_node/static_occupancy_grid`, `/nvblox_node/static_map_slice`(Nav2용)

---

## 5. 보정 결과와 근거

### 5-1. 카메라–IMU (Kalibr, 최종: 2026-11-02, 맨 윗판 55.5 cm, IR 1280×720)
| | motion8 | motion9 | URDF에 반영 |
|---|---|---|---|
| IMU 기준 카메라 위치 (앞/좌/위) | 9.3 / 1.4 / 42.5 cm | 11.8 / 2.2 / 47.1 cm | **motion9** |
| 카메라 방향 | 27.4° 아래 | 27.4° 아래 | 두 결과 평균 (서로 0.20°) |
| 시간차 `timeshift_cam_imu` | +18.7 ms | +15.5 ms | **+17.1 ms** (평균) |
| 재투영 / 자이로 / 가속도 잔차 | 0.16 px / 0.0093 rad/s / 0.083 m/s² | 0.20 px / 0.0109 / 0.090 | |

위치 차 5.3 cm, 시간 차 3.3 ms로 기준(2 cm·2 ms)을 넘음. 각도는 일치. motion9를 택한 이유: 회전이 두 배(최대 1.9 rad/s),
기둥 10 cm 높이기 전 37.0 cm + 10 cm = 47 cm 예상과 일치, CAD로 옮긴 높이와 1 cm 이내. motion8은 4.6 cm 낮게 튐.
잔차가 motion6/7의 약 2.5배: 높아진 로봇(+팔)이 흔들 때 휘청거려 강체 가정이 깨진 것으로 추정. **나중에 회전 위주로
부드럽게(급정지 없이) 다시 찍어 위치·시간차 확정할 것.** 기둥 재조립 뒤 각도가 motion6/7보다 약 1° 달라짐.
camchain은 `calibration/camchain_1280x720.yaml`(bag의 `camera_info` P값). 7번 Kalibr 명령의 `--cams`를 이것으로.
URDF `d435_joint`: `xyz="0.04204 0.02198 -0.00886" rpy="-0.00503 -0.04540 -0.01867"` (`tools/compute_d435_joint.py`).

이전 결과 (기둥 높이기 전, 맨 윗판 45.5 cm, IR 640×480, 지금은 쓰지 않음):

| | motion6 | motion7 | 평균(당시 URDF에 반영) |
|---|---|---|---|
| IMU 기준 카메라 위치 (앞/좌/위) | 9.1 / 1.4 / 36.6 cm | 8.8 / 1.1 / 37.4 cm | 8.95 / 1.27 / 37.0 cm |
| 카메라 방향 | 27.9° 아래, 좌우 −0.7° | 27.8° 아래, 좌우 −0.5° | |
| 시간차 `timeshift_cam_imu` | +12.8 ms | +12.0 ms | **+12.4 ms** |
| 재투영 / 자이로 / 가속도 잔차 | 0.087 px / 0.0036 rad/s / 0.034 m/s² | 0.091 px / 0.0046 / 0.041 | |

두 결과 차이: 회전 0.25°, 위치 0.9 cm, 시간 0.7 ms → 신뢰 가능.
당시 URDF `d435_joint`(`front_camera_link -> camera_link`): `xyz="0.01846 0.01268 -0.02405" rpy="-0.01654 -0.03745 -0.00868"`.
계산 방법: `T(base->camera_link) = T(base->imu, CAD) · T(imu->infra1_optical, Kalibr) · T(optical->camera_link)`를
`front_camera_link` 기준으로 표현. **IMU(CAD 위치)와 카메라의 상대 관계가 Kalibr와 같아지도록** 넣은 값이라,
CAD의 D435 위치(브래킷 높이가 실물과 다름)와는 다르다.

이전 실패 결과(쓰지 말 것): motion4(시간차 11 ms, 높이 42.9 cm), motion5(50 ms, 29 cm) — IMU가 50 Hz이고
"읽은 시각"이 붙어서 실행마다 시간차가 0–40 ms 흔들렸다. 펌웨어·ros2_control 수정 후 해결.

### 5-2. 가속도계 (`opencr_accel_v2.yaml`, imu_calib 모델 `corrected = SM·raw − bias`)
- OpenCR를 떼어 상자에 붙이고 6면 측정 (2026-10-28). 축 교차 항 최대 0.0055 (v1은 0.053, 손으로 들고 해서 틀어짐)
- 로봇에 달린 정지 상태에서 보정 후 |a| = 9.94 (+1.3%). 온도 영향으로 보이며 cuVSLAM/Kalibr가 바이어스로 흡수
- 6면 전체 검증(`accel_calib_verify.py`)은 OpenCR 재장착 때문에 못 했음

### 5-3. 측정으로 확인한 것
- 바닥 → 맨 윗판 윗면 45.5 cm (URDF 45.9 cm), 바닥 → OpenCR 약 6 cm (CAD 6.1 cm): URDF 기둥·판 높이 OK
- 11-02: 기둥을 높여 맨 윗판 윗면 55.5 cm (v62 URDF 55.9 cm, 2·3판 사이 기둥 45 cm)
- CAD의 D435 위치는 실물보다 낮게 그려져 있음 → `d435_joint`는 CAD가 아니라 Kalibr 기준

---

## 6. 보정 다시 하기

**언제**: OpenCR나 D435를 떼었다 달았을 때, 브래킷·기둥을 바꿨을 때, 펌웨어/ros2_control의 IMU 경로를 바꿨을 때.

1. Jetson에서 공통 터미널 A, B, C(깊이 끔) 실행, 프로젝터 끄기. `ros2 topic hz /opencr/imu_corrected` ≈ 165–200 Hz 확인
2. 체커보드(10×7칸 = **내부 코너 9×6**, 25 mm)를 **벽·박스에 고정**. 손에 들면 안 됨
3. 녹화 (**`--topics` 필수**, 이름 바꿔서 두 번):
   ```bash
   ros2 bag record --storage mcap -o ~/calibration/d435_opencr_motionN --topics /camera/camera/infra1/image_rect_raw /camera/camera/infra1/camera_info /camera/camera/infra2/image_rect_raw /camera/camera/infra2/camera_info /opencr/imu_corrected /tf_static
   ```
   로봇 전체를 들고 0.3–0.5 m 거리에서, 보드가 화면 여러 위치에 오게, 3축 회전·이동을 섞어 60–90초. 처음·끝 3초 정지
4. 점검: `python3 bag_quality_check.py <bag_dir> out.png` (보드 검출률 80%+, IMU·영상 구간 일치 확인)
5. PC로 복사 (`rsync -e "ssh -o HostKeyAlias=jon.local" ...`), 체크섬 확인
6. **bag 앞의 묵은 영상 제거** ([함정 9](#7-트러블슈팅)): `ros2 bag convert`로 시작 시각 + 0.5–3 s 이후만 남김
   ```yaml
   # trim.yaml
   output_bags:
     - uri: /home/sik/calibration/d435_opencr_motionN_trim
       storage_id: mcap
       all_topics: true
       start_time_ns: <metadata.yaml의 starting_time + 3000000000>
   ```
   `ros2 bag convert -i d435_opencr_motionN -o trim.yaml` 후 `bag_quality_check.py`로 `overlap`이 영상·IMU 같은지 확인
7. Kalibr (PC):
   ```bash
   source /opt/ros/lyrical/setup.bash && source ~/calibration_tools/kalibr_ws/install/setup.bash
   systemd-run --user --scope -p MemoryMax=20G ros2 run kalibr_imu_ros2 kalibr_calibrate_imu_camera \
     --bag <trim>/xxx.mcap --cams ~/calibration/camchain_1280x720.yaml --imu ~/calibration/imu_provisional_200.yaml \
     --target ~/calibration/checkerboard_25mm.yaml --timeoffset-padding 0.1 --dont-show-report --num-threads 1
   ```
   결과: bag 옆 `*-results-imucam.txt`, `*-camchain-imucam.yaml`. 두 번 결과가 회전 1°·위치 2 cm·시간 2 ms 안에서 맞는지 확인
8. 평균을 `d435_joint`에 반영 (5-1의 계산식), 시간차가 바뀌면 `imu_time_shift.py`의 `shift_ms`와 cuVSLAM 런치 주석 갱신

가속도계 6면 보정: OpenCR를 떼서 반듯한 상자에 붙임 → `opencr_imu_only.py`(모터 없이 `/imu`) + `imu_axis_check.py`(축·기울기 확인) +
`ros2 run imu_calib do_calib_node --ros-args -r imu:=/imu -p measurements:=1000 -p reference_acceleration:=9.80665 -p output_file:=...`
(순서 X+ X− Y+ Y− Z+ Z−, 양수 = 그 축이 위). 검증은 `accel_calib_verify.py <old.yaml> <new.yaml>`.

Kalibr 빌드 (PC, 이미 됨): `MAKEFLAGS=-j4` + `systemd-run --user --scope -p MemoryMax=20G colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DPython3_EXECUTABLE=/usr/bin/python3`.
병렬 제한 없이 빌드하면 24코어 동시 컴파일로 PC가 멈춘다. `kalibr2_ros`(카메라 내부 보정 C++ 도구)는 Lyrical의 C++20 헤더 문제로 빌드 안 됨 — 필요 없음.

---

## 7. 트러블슈팅

| # | 증상 | 원인 | 해결 |
|---|---|---|---|
| 1 | 영상·Kalibr 영상에 **흰 점 무늬** | `rs_launch.py`(4.57)가 `depth_module.emitter_enabled:=0` 인자를 **조용히 무시** | 카메라 뜬 뒤 `ros2 param set /camera/camera depth_module.emitter_enabled 0`. 카메라 재시작마다 다시 |
| 2 | `ros2 bag record ... /topic`이 `unrecognized arguments` | Lyrical은 토픽 앞에 `--topics` 필요 | `--topics` 추가 |
| 3 | bag 파일 **0바이트**, 녹화는 도는 것 같음 | 발행 노드(A·B)가 꺼져 있음. 녹화기는 에러 없이 기다림 | 녹화 전 `ros2 topic hz`로 확인 |
| 4 | `Failed to open auto at 1000000 baud` 후 `Segmentation fault` | OpenCR가 그 컴퓨터에 없음 (PC에서 실행 등). segfault는 초기화 실패 뒤 ros2_control이 따라 죽는 것 | Jetson에서 실행, `/dev/serial/by-id/*OpenCR*` 확인 |
| 5 | `Wheel motors not connected` 로 멈춤 | 바퀴 하드웨어 플러그인은 모터가 있어야 시작 | 모터 전원·케이블 연결. 떼어 낸 OpenCR는 `opencr_imu_only.py` |
| 6 | `Overrun detected ... 200 Hz` 경고 (약 1초에 1번) | Jetson–OpenCR USB 왕복이 가끔 4–6 ms | 정상 범위. 잦아지면 `/imu` 주기 확인 (165 Hz 미만이면 조사) |
| 7 | Kalibr 결과가 실행마다 크게 다름 (시간차 수십 ms, 위치 10 cm+) | IMU가 50 Hz + "읽은 시각" 스탬프 | 펌웨어 5 ms·200 Hz 수정이 적용됐는지 확인 (`/imu` ≈ 165–200 Hz) |
| 8 | Kalibr 결과에 위치 ≈ 0, IMU 잔차 ≈ 1e-10 | 스플라인 매듭이 IMU보다 촘촘해 과적합 (50 Hz IMU일 때) | IMU 175 Hz+면 기본값. 50 Hz면 `--pose-knots-per-second 20 --bias-knots-per-second 5` (패치로 추가한 옵션) |
| 9 | Kalibr가 IMU를 몇 초만 읽음 (`truncated ... messages`) / 결과 엉망 | RealSense가 **묵은 영상 1장**을 bag 맨 앞에 넣음(transient_local) + `--bag-from-to`가 토픽마다 자기 첫 시각 기준 | `--bag-from-to` 쓰지 말고 `ros2 bag convert`로 앞부분을 잘라냄 (6장 6번) |
| 10 | Kalibr 코너 검출이 몇 장뿐 | 멀티프로세스 추출 버그 | `--num-threads 1` |
| 11 | Kalibr가 결과 저장 후 안 끝남 | 남은 스레드 | 패치로 `os._exit(0)` 추가됨. 예전 빌드면 결과 파일 확인 후 kill |
| 12 | `--recover-covariance`로 `BSplineMotionError ... unsupported` | Kalibr 한계 | 옵션 빼기 |
| 13 | 복사·RViz가 갑자기 극도로 느림, 핑 수백 ms·유실 | **영상을 Wi-Fi로 받는 노드**(`board_preview.py`, 원본 이미지 구독 RViz)가 대역을 다 씀 | 그런 노드를 끄기. 영상은 `ir_preview.py`(5 fps 축소본)만 |
| 14 | `jon.local` 이름 해석 실패 | mDNS 불안정 | `getent hosts jon.local`이나 공유기에서 IP 확인 후 IP로 접속 |
| 15 | IP로 ssh 시 `REMOTE HOST IDENTIFICATION HAS CHANGED` | 같은 IP를 예전에 다른 기기가 썼음 | 받은 키가 `jon.local` 저장 키와 같은지 대조 후 `ssh -o HostKeyAlias=jon.local pa03@<IP>`. **대조 없이 무시하지 말 것** |
| 16 | PC에서 Jetson 토픽이 안 보이거나 늦게 보임 | 도메인/탐색 범위 차이 + 느린 Wi-Fi | `ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`. 짧은 CLI 명령은 발견 전에 끝날 수 있음 (RViz처럼 오래 켜면 붙음) |
| 17 | RViz 영상 겹쳐보기·점이 안 뜸 | cuVSLAM이 꺼져 있음 / `viz:=true`가 아님 / 로봇 base와 TF 충돌 | Jetson에서 `pgrep -af component_container`, `viz:=true` 확인, `viz` 모드에선 A를 끔 |
| 18 | `camera_link`에 부모가 둘, TF 경고 | cuVSLAM `viz:=true`/`tf:=true`인데 로봇 URDF나 바퀴 odom TF도 켜짐 | `viz`는 A 끔. `tf:=true`는 A를 `wheel_odom_tf:=false`로 |
| 19 | D435 프레임이 그리퍼에 붙어 보임 | 예전 URDF(그리퍼 카메라가 `camera_link`) | 새 URDF(커밋 e4d1058) 사용 확인: `xacro ... | grep d435_joint` |
| 20 | cuVSLAM IMU 모드인데 지도가 기울어짐 | `imu_link` TF나 `d435_joint` 틀림 / IMU 토픽 미수신 | `ros2 run tf2_ros tf2_echo camera_infra1_optical_frame imu_link` (IMU가 카메라 아래 약 47 cm), `/opencr/imu_synced` 주기 확인 |
| 21 | 정지 상태에서 IMU 모드 위치가 계속 흐름 | 자이로 바이어스 측정 중 로봇이 움직였음 / IMU 노이즈 임시값 | B를 다시 띄우고 45초 정지. 계속되면 Allan 측정(미실시) |
| 22 | nvblox 지도에 흰 벽이 비거나 구멍 | 프로젝터 끔 → 무늬 없는 면은 스테레오 깊이 실패 | NVIDIA 방식: 프로젝터 on/off 번갈아 + `realsense_splitter` (켠 프레임→nvblox, 끈 프레임→cuVSLAM), 카메라 60 fps 권장 (미구현) |
| 23 | nvblox ESDF가 온통 장애물 | 슬라이스 높이에 바닥이 포함 | `static_mapper.esdf_slice_min_height`가 0.05 이상인지, `odom` z=0이 바닥인지(IMU 모드) 확인 |
| 24 | RViz에 nvblox 메시가 일부만 | 메시는 바뀐 블록만 전송 | 로봇을 조금 움직이면 갱신. RViz를 nvblox보다 먼저 켜면 확실 |
| 25 | PC mock 실행 시 `imu_sensor_broadcaster` 로더 없음 | PC에 `ros-lyrical-imu-sensor-broadcaster` 미설치 | 무시하거나 설치 (Jetson에는 있음) |
| 26 | Jetson에서 `git pull` 충돌 | 로컬 수정 + 파일 복사 상태 | `git stash` 후 pull, 확인 뒤 stash 삭제 |
| 27 | 펌웨어 수정 후 `xSting.cpp` diff가 파일 전체 | CRLF 파일을 LF로 저장 | 바이너리 안전하게 편집(`\r\n` 유지) |

### 펌웨어 올리기 (PC)
Arduino IDE AppImage 안의 `arduino-cli` 사용 (OpenCR 보드 패키지 1.5.3 설치됨). 스케치 폴더 이름이 `.ino`와 같아야 해서 복사 후 컴파일:
```bash
CLI=<AppImage 압축 해제 경로>/resources/app/lib/backend/resources/arduino-cli
mkdir -p /tmp/fw/xSting && cp -r firmware/mk1_motor/{xSting.ino,src,include} /tmp/fw/xSting/
$CLI --config-file ~/.arduinoIDE/arduino-cli.yaml compile --fqbn OpenCR:OpenCR:OpenCR --output-dir /tmp/fw/build /tmp/fw/xSting
$CLI --config-file ~/.arduinoIDE/arduino-cli.yaml upload -p /dev/ttyACM0 --fqbn OpenCR:OpenCR:OpenCR --input-dir /tmp/fw/build /tmp/fw/xSting
```
올리기 전에 Dynamixel ping으로 **ID 201(바퀴, model 0x5000)** 인지 확인할 것 (팔 보드는 200).

---

## 8. 정상일 때의 기준값

| 항목 | 정상 값 (2026-10-29 측정) |
|---|---|
| `/imu`, `/opencr/imu_corrected` | 165–200 Hz (`/opencr/imu_synced`는 150–180) |
| `/camera/camera/infra1/image_rect_raw`, depth | 30 Hz |
| 정지 상태 보정 후 중력 | 9.94 m/s² 근처, 자이로 평균 0 |
| cuVSLAM 출력 | 25–30 Hz, `vo_state` 1, 처리 2.5 ms(카메라만) / 5.8 ms(IMU) |
| 정지 10초 드리프트 | 카메라만 0 mm, IMU 약 6 mm |
| IMU 모드 자세 | 평평한 바닥에서 롤·피치 ±0.5° 이내 |
| 직선 시험 | 60 cm 이동에 옆 흐름 2.5 cm, 1.7 m 왕복(턱 포함) 오차 약 3 cm (카메라만) |
| TF `camera_infra1_optical_frame → imu_link` | 이동 약 (0.017, 0.473, 0.113) m — 카메라 아래 47 cm (11-02 이전 37 cm) |
| nvblox | 메시 약 6 Hz, ESDF·점유 지도 약 10 Hz, Jetson CPU 약 30%, RAM 2.9/7.3 GB |

---

## 9. 남은 일 / 미검증

- **`wheel_odom_tf` 런치 변경 미커밋** (`xsting_control.launch.py`, `diff_drive_no_odom_tf.yaml`)
- Jetson 저장소를 pull로 정리하지 않음 (파일 복사 상태)
- IMU 노이즈: Allan 분석 안 함 (정지 1시간 기록 필요, 50→200 Hz 바뀐 뒤 기준). 지금은 데이터시트×10 임시값
- 가속도 v2의 6면 전체 검증 안 함 (Z+만: +0.6%, 로봇 장착 후 +1.3%)
- 실제 주행(teleop)에서 200 Hz 루프·바퀴 명령 분리의 장시간 동작 확인 안 함
- 프로젝터 끈 상태의 nvblox 품질(흰 벽) 미평가. 필요하면 프로젝터 번갈아 켜기 + splitter
- odom 구조 결정: 지금은 nvblox 시험용으로 cuVSLAM이 odom을 냄. 바퀴 odom과 융합(robot_localization)할지 미정
- 팔 관련: joint3–5 한계는 기존 ±90° 유지(새 CAD는 더 넓음), 그리퍼 joint7 범위·0점이 새 CAD와 다름 → 팔 담당자 확인 필요
- 퍽 집기: 팔이 D435 시야를 가림. 손목 카메라로 마지막 정렬 / 퍽 위치를 지도 좌표로 기억하는 방안 논의만 함

---

## 10. 결정 이력 요약

| 날짜 | 결정 | 이유 |
|---|---|---|
| 10-27 | Kalibr ROS 2는 PC에서, 병렬 제한·메모리 상한으로 빌드 | Jetson이 아닌 x86 PC가 분석용, 무제한 병렬 빌드로 PC가 멈췄음 |
| 10-28 | 카메라 내부값은 공장값 | 손으로 한 GUI 보정은 칸 크기(0.108 입력)·코너 수 오류, 프로젝터 켜짐. 공장값은 Kalibr 재투영 0.1 px 이하 |
| 10-28 | URDF를 xSting_51로 교체, 제어 관련 한계는 기존 유지 | 실물과 같은 최신 CAD. 한계값은 CAD 쪽이 placeholder |
| 10-28 | 그리퍼 카메라 프레임 이름 변경 | RealSense 드라이버의 `camera_link`와 충돌 |
| 10-29 | IMU 경로 50 Hz → 200 Hz (펌웨어 5 ms, ros2_control 200 Hz, 쓰기 분리) | 50 Hz 폴링·읽은 시각 스탬프로 시간차가 실행마다 0–40 ms 흔들려 Kalibr 결과가 불안정 |
| 10-29 | `d435_joint` = Kalibr(motion6/7 평균) | 두 번 결과가 0.25°·0.9 cm·0.7 ms로 일치 |
| 10-29 | cuVSLAM은 IMU 모드 권장 | 지도가 중력 기준 수평, 출력이 `base_footprint` 기준. 이 환경에선 카메라만으로도 `lost` 0 |
| 10-29 | nvblox는 프로젝터 끈 채로 먼저 시험 | 설정이 단순. 흰 벽 품질 보고 splitter 방식 결정 |
| 11-02 | URDF를 xSting_v62로 교체(기둥 +10 cm, 2번째 D435), joint3~5 방향 반전 | 실물 맨 윗판 55.5 cm와 일치. 모터 방향이 URDF 축과 반대 |
| 11-02 | cuVSLAM IR 1280×720, Kalibr 재보정(motion8/9), 시간차 17.1 ms | 기둥 재조립. 두 결과 위치·시간이 어긋나 각도 평균 + motion9 위치, 나중에 재녹화 |
