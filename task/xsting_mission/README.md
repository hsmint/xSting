# xsting_mission

`mk2_ws` 주사위 기억 로봇 임무 통합 패키지입니다. 노트북에서 실행되는 판단기(`llm_commander`), 명령 서버(`robot_command_server`), 그리고 하위 로봇 패키지(`xsting_vision`, `xsting_nav`, `xsting_approach`, `xsting_pick_place`, `xsting_walk`)와 연결되는 브릿지(`mission_bridge`)로 구성됩니다.

## 전체 연결 구조 (노트북 ↔ 로봇)

```
[노트북]
  llm_commander
    │ RobotCommand (/robot_command)
    ▼
  robot_command_server (backend:=ros)
    │ /task/run, /task/read_mission_input, /task/load_box, /task/unload_pucks, /task/observe_location
    ▼
  mission_bridge
    ├── GOTO (NAVIGATE)       ─► 외부 주행 액션(/nav/run 또는 수동 확인/스텁) ─► /approach_box (상자 앞 2cm 정밀 접근 연속 실행)
    ├── READ_MISSION_INPUT    ─► /read_sign_barcode (xsting_vision::SignBarcodeNode)
    ├── LOAD                  ─► /pick_place CMD_LOAD_BOX (슬롯 0~11 ↔ 1~12 변환, 색상·모양 기반 임시 ID 0~11 자동 배정)
    ├── UNLOAD                ─► /pick_place CMD_UNLOAD_BOX (지도 +Y 기준 3x3 cell → 로봇 base 좌표 자동 회전 + 2초 안정 확인)
    └── VERIFY_LOCATION       ─► /pick_place CMD_OBSERVE + /detections_3d 신규 프레임 실측 9칸 판정
```

## 빌드 (`mk2_ws`)

`xsting_interfaces`에 미션용 메시지·액션과 로봇용 액션이 통합되어 있습니다.

```bash
cd /home/pa03/Documents/FINAL/mk2_ws
source /opt/ros/lyrical/setup.bash
colcon build --packages-select xsting_interfaces xsting_vision xsting_approach xsting_nav xsting_pick_place xsting_mission --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

## 실행 방법 (노트북)

### 1. 주행 학습 완료 전 수동 이동 확인 모드 (`nav_mode:=manual`, 기본값)

```bash
# 터미널 1: 미션 브릿지 + 로봇 명령 서버 실행 (상자 도착 확인 시 /approach_box 자동 연계)
ros2 launch xsting_mission mission.launch.py backend:=ros nav_mode:=manual

# 터미널 2: LLM 판단기(또는 고정 스크립트 decider:=script) 실행
ros2 run xsting_mission llm_commander --ros-args -p guard:=true

# 터미널 3: GOTO 대기 중 텔레옵 이동을 마친 뒤 도착 트리거 호출
ros2 service call /mission_bridge/nav_arrive std_srvs/srv/Trigger "{}"
# (장애물 막힘 상황 테스트 시)
ros2 service call /mission_bridge/nav_fail std_srvs/srv/Trigger "{}"
```

### 2. RL Navigation(`xsting_nav`) 액션 연동 모드 (`nav_mode:=action`)

`xsting_nav` 노드가 `/nav/run` 액션을 처리하며, 상자 검출 및 사전 접근 지점 5~10cm 이내 도달 시 내부에서 `/approach_box` 정밀 접근까지 완료한 뒤 결과를 반환합니다.

```bash
# 로봇(또는 통합 터미널): xsting_nav + xsting_approach 기동
ros2 launch xsting_nav nav.launch.py with_approach:=true

# 미션 브릿지 + 판단기 기동
ros2 launch xsting_mission mission.launch.py backend:=ros nav_mode:=action nav_action:=/nav/run commander:=true
```

### 3. 가짜 경기장 시뮬레이션 모드 (`backend:=fake`)

```bash
ros2 launch xsting_mission mission.launch.py backend:=fake commander:=true decider:=script
```

- 가짜 경기장 화면: `http://localhost:8765`
- 판 기록 위치: `~/xsting_logs/run_<시각>/` (`mission.jsonl`, `report.md`)
- 적재 슬롯·임시 ID 수동 초기화: `ros2 service call /mission_bridge/reset_state std_srvs/srv/Trigger "{}"`
