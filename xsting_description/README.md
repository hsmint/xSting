# xsting_description

## Isaac Sim 학습용 USD

**30×30×60 mm 사각 / Ø30×60 mm 원형 퍽 수납 학습**은
`usd/xsting_storage_training.usda`를 사용하세요. 바스킷을 원본 메시 기반 SDF로 분리한
[정밀 충돌 프리셋 및 실측 검증 결과](usd/storage/README.md)가 추가되었습니다.
이 프리셋에는 층판 4개의 박스 충돌, 전면 RealSense 본체 충돌, 전면·좌우 RGB/깊이 카메라,
팔 RGB 카메라, 초음파 방식 raycast 거리 센서가 포함됩니다. 센서 실행 방법도 위 문서에 있습니다.

cuMotion + MoveIt 팔 조작용은 `usd/xsting_manipulation.usda`와
[`xsting_moveit_config`](../xsting_moveit_config/README.md)를 사용합니다.
이 프리셋은 베이스 고정, 다리 서기 자세 잠금, 팔 L-pose 시작, 위치 drive를 적용합니다.
기존 `xsting_training.usda`의 floating-base / 토크 제어 설정은 그대로 유지됩니다.

`usd/xsting_training.usda`를 Isaac Sim에서 열거나 학습 환경의 로봇 prim으로 reference합니다.
메시 데이터는 같은 폴더의 `usd/xsting_meshes.usdc`에 있으므로 두 파일을 함께 옮겨야 합니다.

- PhysX용 floating-base articulation: `/xsting/base`. 월드 고정 조인트 없음.
- 원본 URDF의 링크 질량·관성·관절 축·가동 범위 유지. 총 질량 약 6.4893 kg.
- 다리 18 + 팔 5 + 손가락 2 = 25개 가동 관절. `finger_r`는 `finger_l`을 따르므로 독립 action은 24개입니다.
- 충돌은 26개 링크의 convex decomposition, self-collision은 꺼져 있습니다. 팔레트의 작은 홈 등은 근사 충돌이므로 정밀 조작 학습 전 충돌 형상을 확인하세요.
- 메시를 instanceable reference로 구성해 여러 환경에서 공유합니다. 센서와 팔레트의 고정 프레임은 보존하지만 카메라·IMU·초음파 센서의 실제 시뮬레이션은 별도 구성해야 합니다.
- 파일의 기하 자세는 URDF 영점입니다. `usd/training_config.json`에 기본 서기 자세(rad/m), 권장 시작 베이스 위치, action 관절 이름 순서를 저장했습니다. 시뮬레이터 관절 인덱스는 이름으로 매핑하세요.
- 기본 drive stiffness/damping은 모두 0인 토크 제어용입니다. URDF damping은 이 프리셋에서 사용하지 않습니다. 위치 제어 학습은 학습 환경의 actuator/PD 이득을 설정해야 하며, 직접 토크를 적용할 때는 URDF effort 한계(회전 3.8 N·m, 손가락 10 N)를 제어기에서도 제한하세요. 오른손가락에 독립 actuator를 추가하지 마세요.
- 지면·조명·PhysicsScene은 학습 환경에서 생성합니다. 이 파일은 로봇 asset입니다.

재생성 (현재 설치된 Isaac Lab Python 환경 기준, GPU 불필요):

```bash
/opt/IsaacLab/.venv/bin/python xsting_description/scripts/build_training_usd.py
```

필요 패키지: `urdf-usd-converter`, `usd-core`, `numpy`, `PyYAML`.
변환기는 원본 URDF를 수정하지 않으며 생성된 USD를 검사하고 `usd/validation.json`을 저장합니다.
현재 검증은 OpenUSD 로딩, reference 재배치, 연결된 관절 트리, 질량, 양의 관성,
관절 프레임 일치, 가동 범위, mimic 및 충돌 메시 수까지 통과했습니다.
기본 변환 검증과 별도로 storage 프리셋은 NVIDIA GPU / Isaac Sim에서 퍽 낙하 및 센서 검사를
수행합니다. 검사 범위와 결과는 위 정밀 충돌 프리셋 문서를 확인하세요. 학습 안정성 자체는 별도 검증 대상입니다.

Mimic 설정은 [NVIDIA PhysX 스키마](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.2/dev_guide/schemas/physxschema.html)의
`position + gearing * referencePosition + offset = 0` 규칙에 따라 gearing=-1, offset=0을 사용합니다.

xSting 헥사포드 + 전방 x53 매니퓰레이터 + 12칸 중공 퍽 팔레트 로봇 설명 패키지 (`robot: xsting`, 단위 `m·kg·rad`, 총 질량 약 `6.49 kg`).

## 주요 파일 안내
| 용도 | 파일 |
|---|---|
| **ROS 2 / RViz / ros2_control / 비전·SLAM TF** | `urdf/xsting.urdf` |
| **관절 순서 · 영점 · 시작 자세 · 가동 범위 · 모터 ID** | `config/joint_order.yaml` |
| **RViz 모델 및 관절 슬라이더 확인** | `ros2 launch xsting_description display.launch.py` (`pose:=stand` 또는 `pose:=zero`) |

## 관절 및 링크 명명 규칙
- **패키지 / 루트 이름**: `xsting_description` (`package://xsting_description/meshes/...`), 루트 로봇 이름 `xsting` (`base`, 원점 = 하단 추가 와플판 바닥 중앙, 1층 와플판 바닥은 `z = +0.0367 m`)
- **다리 관절 (18개)**: `leg1_yaw`, `leg1_hip`, `leg1_knee` … `leg6_yaw`, `leg6_hip`, `leg6_knee`
  - 다리 번호: `1` 왼쪽 앞, `2` 왼쪽 가운데, `3` 왼쪽 뒤, `4` 오른쪽 앞, `5` 오른쪽 가운데, `6` 오른쪽 뒤
  - OpenCR 보드: 앞다리 보드(`ID 200`: `leg1`, `leg2`, `leg4`), 뒷다리 보드(`ID 201`: `leg3`, `leg5`, `leg6`)
- **다리 프레임 (18개)**: `leg1_yaw`, `leg1_hip`, `leg1_knee` … `leg6_knee`
- **팔 및 그리퍼 관절 (7개, 팔 OpenCR 보드 `ID 202`)**:
  - `arm_yaw` (`±180°`), `arm_shoulder` (`-95° ~ +110°`), `arm_elbow` (`-115° ~ +90°`), `arm_wrist` (`-105° ~ +125°`), `arm_roll` (`±180°`)
  - `finger_l` (`-1.86 ~ 25 mm`), `finger_r` (`finger_l` mimic)
- **팔 및 그리퍼 프레임**: `arm_1` ~ `arm_4`, `gripper`, `finger_l`, `finger_r`
- **센서 및 팔레트 프레임 (`xsting_vision` / `cuvslam` / `nvblox` / `xsting_hardware` 호환)**:
  - IMU: `base` → `imu` (`imu_joint`, `(0.08014, 0, 0.0579169) m`)
  - 전면 RealSense D435: `base` → `camera` (`(0.157793, 0.0175, 0.107498) m`, `rpy = (0, 0, 0)`) → `camera_infra1_frame` → `{camera_depth_frame, camera_infra2_frame, camera_color_frame}`
  - 좌·우 측면 RealSense D435 (`side_cams:=true`):
    - 좌측 (`+Y`): `base` → `left_camera` (`(-0.0175, 0.1198, 0.123722) m`, `yaw = +90°`) → `left_camera_depth_frame` → `left_camera_color_frame`
    - 우측 (`-Y`): `base` → `right_camera` (`(0.0175, -0.1198, 0.123722) m`, `yaw = -90°`) → `right_camera_depth_frame` → `right_camera_color_frame`
  - 전면 초음파 센서 (HC-SR04): `base` → `ultrasonic` (`ultrasonic_joint`, `(0.12718, 0, 0.0245219) m`, `+X` 빔 방향, `0.02 ~ 4.0 m`, `15°`)
  - 1층 중앙 천장형(상하 반전) 2D LiDAR (COIN-D4 / LDS-03): `base` → `lidar` (`lidar_joint`, `(0, 0, 0.0245) m`, `rpy = (π, 0, 0)`, `+X` 정면 · `-Z` 하향, `0.10 ~ 12.0 m`, `10 Hz`)
  - 팔 카메라 (Arducam): `gripper` → `arducam` → `arducam_frame`
  - 12칸 퍽 팔레트: `pallet_inner_1`~`4` (`z = 0.162722 m`), `pallet_middle_1`~`4` (`z = 0.172722 m`), `pallet_outer_1`~`4` (`z = 0.184722 m`)

## 영점(`0°`) 및 기본 서기 자세
1. **모든 관절 `0°` 영점 자세 (`pose:=zero`)**:
   - 다리: 좌우 수평 직선 전개
   - 팔: **OpenManipulator-X 표준 `ㄱ`자(L-pose) 영점 (모터 틱 2048)** — `arm_shoulder`(`arm_2` 기둥)가 `+Z` 수직 직립(`arm_elbow` 축은 `24 mm` 전방 오프셋), `arm_elbow`(`arm_3`·`arm_4`·`gripper`)가 `+X` 전방 수평, 손가락 닫힘(`0`)
2. **기본 서기 / 보행 시작 자세 (`pose:=stand`, 기본값)**:
   - 다리: `hip = +30° (+0.5236 rad)`, `knee = -110° (-1.9199 rad)` (접지 시 하단 와플판 `base` 바닥 높이 약 `0.021 m`)
   - 팔: `arm_yaw = 0°`, `arm_shoulder = -74.27° (-1.2963 rad)`, `arm_elbow = +74.27° (+1.2963 rad)`, `arm_wrist = 0°`, `arm_roll = 0°`, `finger_l = 0`
