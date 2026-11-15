# xSting cuMotion + MoveIt

`xsting_description`에서 생성한 **정지 상태의 팔 조작용** 구성입니다.
보행·베이스 이동은 계획하지 않습니다. 다리는 stand 자세로 고정되고, 팔은 충돌 구 모델에서도
간섭이 없는 영점 L-pose에서 시작합니다. 기존 학습용 asset과 원본 URDF는 변경하지 않습니다.

| 파일 | 역할 |
|---|---|
| `../xsting_description/usd/xsting_manipulation.usda` | 학습 USD를 reference하는 고정 베이스 + 위치 구동 overlay |
| `config/xsting_manipulation.urdf` | 다리 자세를 fixed joint에 반영한 계획용 URDF |
| `config/xsting.srdf` | arm / gripper 그룹, home / zero / open / closed 자세, 충돌 제외 쌍 |
| `config/xsting.xrdf` | cuMotion XRDF 1.0, 5축 cspace, 충돌 구와 자기충돌 설정 |
| `config/ros2_controllers.yaml` | 팔과 손가락의 FollowJointTrajectory 컨트롤러 |
| `launch/planning.launch.py` | MoveIt + OMPL/cuMotion + mock/Isaac/외부 컨트롤러 선택 |
| `scripts/setup_isaac_bridge.py` | Isaac Sim 6의 관절 상태·명령·시계 OmniGraph 생성 |

`arm`은 `arm_yaw, arm_shoulder, arm_elbow, arm_wrist, arm_roll` 순서입니다.
`finger_l`은 별도의 `gripper` 그룹에서 제어하고 `finger_r`는 mimic입니다.
TCP `tool0`는 gripper 기준 `(0.0995, 0, 0) m`인 손가락 기준점의 중앙입니다.
실제 물체 접촉점에 대한 캘리브레이션 값은 아닙니다.

## 실행

작업공간에서 빌드합니다.

```bash
cd /home/pa03/Documents/FINAL/mk2_ws
source /opt/ros/lyrical/setup.bash
colcon build --packages-select xsting_description xsting_moveit_config
source install/setup.bash

# GPU나 실기체 없이 MoveIt + 모의 ros2_control
ros2 launch xsting_moveit_config planning.launch.py hardware:=mock pipeline:=ompl
```

MoveIt, KDL, OMPL, ros2_control, JointTrajectoryController, RViz가 필요합니다.
cuMotion은 선택 의존성으로, `isaac_ros_cumotion`과 `isaac_ros_cumotion_moveit`을 별도로 설치해야 합니다.
이 launch의 cuMotion 경로는 `nvidia::isaac_ros::cumotion::CumotionPlanner` C++ 컴포넌트와
`urdf_file_path`, `xrdf_file_path` 파라미터를 제공하는 버전을 대상으로 합니다.
예전 Python `cumotion_planner_node`용 launch와는 API가 다릅니다.
설치된 MoveIt 플러그인의 planning YAML을 읽으므로 플러그인과 서버는 같은 Isaac ROS 릴리스를 사용하세요.
검토한 upstream commit: `ee8f0831b9e0104a79c6dac09cb72824c6b52129`.
현재 PC에서 ROS Lyrical의 MoveIt 구성은 검사했지만 cuMotion 설치·Lyrical 호환성은 검증하지 못했습니다.

### Isaac Sim 연결

1. Isaac Sim 6에서 `xsting_manipulation.usda`를 엽니다. 같은 `usd` 폴더의
   `xsting_training.usda`, `xsting_meshes.usdc`도 필요합니다.
2. `isaacsim.ros2.bridge` 확장을 켠 뒤 Script Editor에서 `scripts/setup_isaac_bridge.py`를 실행합니다.
   다른 경로로 reference했다면 스크립트의 `ROBOT` 경로를 맞춥니다.
3. Play를 누릅니다. 같은 ROS_DOMAIN_ID를 사용하고, ROS 쪽에는
   `topic_based_ros2_control/TopicBasedSystem` 플러그인이 설치되어 있어야 합니다.
4. 아래 launch를 실행합니다.

```bash
ros2 launch xsting_moveit_config planning.launch.py \
  hardware:=isaac pipeline:=cumotion use_sim_time:=true
```

cuMotion이 없는 환경에서는 `pipeline:=ompl`을 사용합니다.
연결은 `/isaac_joint_states` → TopicBasedSystem → `/joint_states` → MoveIt,
MoveIt 궤적 → JointTrajectoryController → `/isaac_joint_commands` → Isaac 위치 구동입니다.
`/clock`은 Isaac이 발행합니다. 같은 로봇을 위한 `/joint_states`, `/robot_description` 발행자를 중복 실행하지 마세요.
`xsting_manipulation.usda` 자체에는 ROS 그래프가 없으며, 브리지 스크립트 실행 후 필요하면 별도 scene으로 저장합니다.

### 외부 컨트롤러 연결

```bash
ros2 launch xsting_moveit_config planning.launch.py hardware:=external pipeline:=ompl
```

이 모드에서는 controller_manager를 시작하지 않습니다. 외부에서 `/joint_states` 및
`/arm_trajectory_controller/follow_joint_trajectory`,
`/gripper_trajectory_controller/follow_joint_trajectory` 액션을 제공해야 합니다.
기존 `xsting_hardware`의 `arm_controller`는 ForwardCommandController이므로
이 액션 인터페이스와 직접 호환되지 않습니다. 제공한 trajectory 컨트롤러로 해당 position
인터페이스를 독점 제어하도록 연결해야 합니다. 이 변경은 기존 실기체 launch에 자동 적용하지 않았습니다.
실기체의 다리 자세가 고정 계획 모델과 같아야 합니다.

## 계획과 충돌 모델

팔은 5축이라 임의의 위치+자세 6자유도 목표를 모두 만족시킬 수 없습니다.
MoveIt KDL은 `position_only_ik: true`로 설정했습니다. 이는 cuMotion의 목표 제약을 자동으로 바꾸지 않습니다.
cuMotion은 먼저 도달 가능한 **관절 목표**로 확인하고, 자세 목표를 사용할 때는 5축으로 가능한 자세를 지정하세요.
그리퍼 계획은 RViz에서 `ompl` 파이프라인을 선택합니다. cuMotion cspace는 팔 5축만 포함합니다.

XRDF는 원본 collision STL 삼각형을 덮도록 자동 생성한 4,637개 구를 사용합니다.
어깨 주변만 세밀하게 나눴으며 모든 삼각형의 세 꼭짓점이 대응 구 안에 들어감을 확인했습니다.
그리퍼 구에는 전체 개폐 스트로크를 덮는 27 mm 여유를 추가했으므로 정밀 파지 공간은 보수적으로 막힐 수 있습니다.
인접 링크·서로 고정된 다리/몸체·손가락 조립부만 자기충돌에서 제외했습니다.
원본의 접힌 보행용 팔 자세는 메시 검사에서는 통과하지만 보수적인 구 모델과는 충돌하여 초기 자세로 쓰지 않습니다.
속도/가속도/jerk와 USD PD 이득은 초기 시뮬레이션 설정이며 모터 동특성에 맞춘 튜닝값은 아닙니다.

환경 장애물은 MoveIt PlanningScene으로 별도 제공해야 합니다. Octomap 센서, nvblox ESDF,
물체 부착 및 실제 카메라 스트림은 이 구성에 포함되지 않습니다. Isaac Sim의 self-collision은
꺼져 있고, 궤적 계획의 self-collision 검사는 SRDF/XRDF가 담당합니다.

## 재생성 및 검증

소스 작업공간 루트에서 실행합니다. 생성기는 Isaac Lab Python의 OpenUSD, trimesh, scipy,
numpy, PyYAML을 사용합니다.

```bash
/opt/IsaacLab/.venv/bin/python xsting_moveit_config/scripts/generate_assets.py
/opt/IsaacLab/.venv/bin/python xsting_moveit_config/scripts/validate_assets.py
```

검증 결과는 `config/generation_report.json`, `config/validation.json`에 기록합니다.
URDF/SRDF/XRDF 관절 일치, USD와 URDF의 초기 순기구학, 잠긴 다리,
단일 고정 articulation, 초기 구 자기충돌 없음, 컨트롤러 관절 매핑을 검사했습니다.
ROS Lyrical에서 두 패키지 빌드, MoveIt RobotModel/FCL의 5축·관절 범위·초기 메시 자기충돌 검사,
mock 하드웨어와 세 컨트롤러 활성화도 확인했습니다.
GPU가 없어 cuMotion 경로 생성과 Isaac PhysX/ROS 브리지 실행은 미검증입니다.
전체 MoveIt 계획·실행 액션 왕복도 이번 환경에서는 완료하지 못했습니다.

참고: [NVIDIA custom robot 구성](https://nvidia-isaac-ros.github.io/v/release-4.6/concepts/manipulation/cumotion_moveit/tutorial_custom_manipulator.html),
[현재 cuMotion MoveIt 예제](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion/blob/ee8f0831b9e0104a79c6dac09cb72824c6b52129/isaac_ros_cumotion_examples/launch/ur_isaac_sim.launch.py),
[MoveIt 위치 중심 IK](https://moveit.picknik.ai/main/doc/examples/kinematics_configuration/kinematics_configuration_tutorial.html).
