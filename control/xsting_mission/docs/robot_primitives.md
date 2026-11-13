# ros_backend → 로봇 팀 명령 형식

> 상태: 초안 (2026-11-13). 로봇 팀(주행·인식·팔)과 회의해 확정한다. 확정 전에는 이름·필드가 바뀔 수 있다.
> 노트북 → 로봇 서버 규약은 `xsting_mission/core/contract.py`, 에러 코드 뜻은 `docs/error_codes.md`가 원본이다.

## 1. 층과 책임

```
노트북 llm_commander
   │  RobotCommand 액션 (행동 5개, JSON)          ← contract.py
   ▼
젯슨 robot_command_server + ros_backend.py        ← 우리 (이 문서의 "보내는 쪽")
   │  행동 단위 액션 (ROS 타입)                    ← 이 문서
   ▼
젯슨 로봇 팀 노드: 주행(강화학습), 입력 읽기, 싣기, 놓기, 관측
```

행동 하나를 로봇 팀 액션 하나로 보낸다. 로봇 팀은 액션을 받으면 인식부터 동작까지 끝까지 해낸다.

| | ros_backend (우리) | 로봇 팀 |
|---|---|---|
| 하는 일 | 액션을 보내고, 결과를 `contract.py` 형식으로 바꾸고, 에러면 다시 시도한다 | 인식(바코드·신호·퍽·태그), 주행, 집기·놓기, 적재함 관리 |
| 아는 것 | 미션 규칙(신호 규칙), 다시 시도 규칙, 에러 코드 변환, 지금 장소 | 로봇 몸과 인식. 주사위 cell 번호 → 놓을 위치 계산 |
| 예 | 신호가 규칙에 안 맞으면 다시 읽기, 인식 실패면 다시 관측 | 박스 위 퍽을 보고 모두 집어 빈 슬롯에 싣기, cell에 놓고 2초 안정 확인 |

## 2. 공통 규칙

### 이름

| 대상 | 형식 | 비고 |
|---|---|---|
| 장소 ID | `HOME`, `S1`~`S6`, `L1`~`L3` (문자열) | 노트북과 같다. 좌표는 `config/arena.yaml`의 `places.<ID>.target` |
| 퍽 id | 정수 0~11 | 미션지의 퍽 ID (placement.csv). **퍽 AprilTag 번호로 읽는다**. 못 읽으면 -1 |
| 적재함 슬롯 | 정수 0~11 | 번호 배치(4×3 중 어디가 0번)는 로봇 팀이 정해 알려 준다 |
| 하차 cell | 정수 0~8 | 위쪽 줄 0 1 2 / 가운데 3 4 5 / 아래쪽 6 7 8, 지도 +Y가 위쪽 (모든 하차 박스 같은 방향), 간격 60mm |

### 좌표계 (TF frame)

| frame | 원점·축 | 누가 발행 |
|---|---|---|
| `map` | 경기장 원점(왼쪽 아래 기둥 중심), +Y 지도 위쪽 | 위치 추정(cuVSLAM). 경기장 원점과 맞춰야 한다 |
| `base_link` | 로봇 차체 중심 | 하드웨어 |

- ros_backend는 `map → base_link`로 GOTO 뒤 `robot.pose`를 채우고 HOME 출발 구역 안인지 판정한다.
- 놓을 위치는 로봇 팀이 cell 번호로 계산한다. 하차 박스 태그 기준으로 맞추기를 권한다. 지도 좌표로 놓으면 위치 추정 오차가 그대로 놓는 위치 오차가 되어 10mm 기준을 맞추기 어렵다.

### 단위

- ROS 메시지 안에서는 ROS 표준 단위(m, rad, 초)를 쓴다.
- 노트북 규약(`contract.py`)은 mm, deg다. 변환은 ros_backend가 한다.

### 결과·에러

- 모든 결과에는 `bool success`, `uint16 error_code`, `string message`가 있다.
- `error_code`는 4절의 `ErrorCode` 상수를 쓴다. ros_backend가 같은 이름의 문자열 코드로 바꿔 노트북에 보낸다.
- 표에 없는 숫자는 규약이 어긋난 것이라 `INTERNAL_ERROR`로 바꾸고, 원래 숫자를 `message`에 남긴다.
- 실패해도 이미 한 것(실은 퍽, 놓은 항목)은 결과에 싣는다.

### 지금 장소

- LOAD·UNLOAD·VERIFY는 노트북 인자에 장소가 없다. ros_backend가 마지막 GOTO 결과로 지금 장소(`here`)를 기억해 액션 목표에 넣는다 (출발은 HOME).
- GOTO가 출발한 뒤 실패하면 `UNKNOWN`이 되고, 그때 READ·LOAD·UNLOAD·VERIFY는 `WRONG_LOCATION`으로 거절한다.

### 취소·시간

- 노트북이 행동을 취소하면 ros_backend는 진행 중인 로봇 팀 액션에 취소를 보낸다. 로봇 팀 액션은 취소를 받으면 안전한 상태로 멈추고 `CANCELED`로 끝낸다.
- 목표의 `timeout`은 ros_backend가 기다리는 시간보다 2초 짧다. 로봇 팀은 그 안에 끝내거나 실패 이유와 함께 돌려준다.
- 모든 액션의 feedback에는 `string phase`(진행 상황 문장)가 있다. ros_backend가 노트북 로그로 흘린다.

### 다시 시도 (ros_backend)

같은 행동을 처음 포함 최대 3번 보낸다. 중간 실패는 feedback으로만 남기고(노트북 로그), 마지막 결과를 노트북에 보낸다.
결과에 `attempts`(시도 횟수)를 더한다.

| 행동 | 다시 시도하는 경우 | 이유 |
|---|---|---|
| READ_MISSION_INPUT | `PERCEPTION_FAILED` (못 찾음, 못 읽음, 신호가 `rules.signal_errors`에 걸림) | 신호는 반복되므로 다음 주기를 보면 된다 |
| LOAD | `PERCEPTION_FAILED`이고 아무것도 못 실었을 때 | 실은 퍽이 있으면 상태가 바뀌었으니 판단에 맡긴다 |
| VERIFY_LOCATION | `PERCEPTION_FAILED` | 관측만 하므로 다시 봐도 안전하다 |
| GOTO | 하지 않음 | 장애물이면 미룰지 다른 곳 먼저 갈지 판단이 정한다 |
| UNLOAD | 하지 않음 | 놓기는 되돌릴 수 없다. 남은 퍽을 다시 놓을지는 판단이 정한다 |

`TIMEOUT`, `CANCELED`, `INTERNAL_ERROR`는 로봇 상태를 알 수 없어 다시 시도하지 않는다.

## 3. 행동별 흐름

| 노트북 행동 | 로봇 팀 액션 (이름) | 한 번 한도 | 전체 한도 (노트북 대기) |
|---|---|---|---|
| GOTO | `RunTask` NAVIGATE (`/task/run`) | 170초 | 170초 (180) |
| READ_MISSION_INPUT | `ReadMissionInput` (`/task/read_mission_input`) | 60초 | 590초 (600) |
| LOAD | `LoadBox` (`/task/load_box`) | 150초 | 170초 (180) |
| UNLOAD | `UnloadPucks` (`/task/unload_pucks`) | 400초 | 400초 (420) |
| VERIFY_LOCATION | `ObserveLocation` (`/task/observe_location`) | 30초 | 110초 (120) |

### GOTO `{t}`

1. `RunTask(operation=NAVIGATE, place_id=t, destination=places.<t>.target in map)`. HOME이면 `destination`은 `start_pose`.
   접근 자리(어디에 어느 방향으로 설지)는 주행 쪽이 정한다. 박스 앞에서는 팔이 먼 줄 퍽까지 닿고 카메라가 태그를 보는 자리여야 한다.
2. 성공하면 TF로 `robot.pose`를 채운다. HOME이면 3.2초 기다린 뒤 차체 footprint가 출발 구역 안인지 판정해 `in_zone`, `stopped_s`를 채운다.

```json
{"success": true, "robot": {"here": "S1", "pose": {"x": 1482, "y": 567, "yaw": 92.1}}}
{"success": true, "robot": {"here": "HOME", "pose": {...}}, "in_zone": true, "stopped_s": 3.2}
{"success": false, "error": "OBSTACLE_BLOCKED", "robot": {"here": "UNKNOWN", "pose": {...}}}
```

### READ_MISSION_INPUT `{}`

1. `ReadMissionInput()` → `barcode`, `colors[12]`, `end_seen`. 신호등·바코드를 찾는 동작(한 바퀴 탐색)도 로봇 팀이 한다.
2. ros_backend가 `rules.signal_errors(colors, end_seen)`로 검사한다. 걸리면 `PERCEPTION_FAILED`로 보고 다시 시도한다.
3. 3번 다 실패하면 `cause`(`SIGNAL_INCOMPLETE` 못 읽은 칸 / `SIGNAL_INVALID` 규칙 위반)와 `signal_errors`를 싣는다.

```json
{"success": true, "barcode": "P2-02", "signal": ["G","Y","R", ...12개], "end_seen": true,
 "attempts": 2, "signal_reread": {"reads": 2, "complete": true}}
```

### LOAD `{}`

1. `LoadBox(place_id=here)`. 로봇 팀이 박스 위 퍽을 인식하고 모두 집어 빈 슬롯에 싣는다.
2. 결과의 `pucks`(슬롯, 태그 번호, 색, 모양, 신뢰도)와 `box_remaining`을 규약 형식으로 바꾼다. 태그를 못 읽은 퍽은 `id: null`.

```json
{"success": true, "box_remaining": 0,
 "pucks": [{"slot": 0, "puck": {"id": 7, "color": "R", "shape": "cylinder", "conf": 0.93}}, ...]}
{"success": false, "error": "GRASP_FAILED", "box_remaining": 1, "pucks": [{"slot": 0, "puck": {...}}]}
```

### UNLOAD `{items: [{puck_id, cell}, ...]}`

1. `UnloadPucks(location_id=here, items)`. 로봇 팀이 적재함에서 퍽을 찾아 items 순서대로 cell에 놓는다 (각 2초 안정 확인).
2. 실패하면 그 항목에서 멈추고, 시도한 항목까지 `items`(ok, 오차 또는 에러)와 `stopped_at`을 돌려준다.

```json
{"success": true, "stopped_at": null,
 "items": [{"puck_id": 7, "cell": 0, "ok": true, "offset_mm": [1.5, -2.0]}, ...]}
{"success": false, "error": "PLACE_FAILED", "stopped_at": 1,
 "items": [{"puck_id": 7, "cell": 0, "ok": true, "offset_mm": [...]},
           {"puck_id": 4, "cell": 2, "ok": false, "error": "PLACE_FAILED"}]}
```

### VERIFY_LOCATION `{items: [{puck_id, cell}, ...]}`

1. `ObserveLocation(location_id=here)` → 9칸 관측 (상태, 그 칸 퍽의 태그 번호, 오차)
2. ros_backend가 items와 비교한다: `ok`(그 cell에 그 퍽), `seen_puck_id`(실제로 본 퍽), `extra`(items에 없는데 퍽이 있는 cell).
   배치가 틀린 것은 실패가 아니라 성공 + `all_ok: false`다.

```json
{"success": true, "all_ok": true, "extra": [],
 "items": [{"puck_id": 7, "cell": 0, "ok": true, "seen_puck_id": 7, "offset_mm": [1.2, -0.8]}],
 "cells": [{"cell": 0, "state": "occupied", "puck_id": 7}, ... 9칸]}
```

## 4. 인터페이스 초안 (`xsting_interfaces`)

### `RunTask.action` (origin/vision 것에 `place_id` 추가, ros_backend는 NAVIGATE만 쓴다)

```
string task_id
string operation                             # NAVIGATE / PICK_STORE / PICK_PLACE / STORE_PLACE
string place_id                              # 추가: NAVIGATE 목적지 장소
string object_id
string slot_id
geometry_msgs/PoseStamped destination        # NAVIGATE: 대상 중심
builtin_interfaces/Duration timeout
---
bool success
uint16 error_code
string message
---
string phase
float32 progress
```

### `ReadMissionInput.action`

```
builtin_interfaces/Duration timeout
---
bool success
uint16 error_code
string message
string barcode                               # "P2-01" ~ "P2-04"
string[] colors                              # 12개, "R" | "Y" | "G" | "?" (못 읽은 칸)
bool end_seen                                # 12번째 뒤 3초 소등을 봤나
---
string phase                                 # 예: "blink 3/12 R"
```

### `LoadBox.action`

```
string place_id                              # S1 ~ S6
builtin_interfaces/Duration timeout
---
bool success
uint16 error_code                            # 실패해도 실은 퍽은 pucks에 싣는다
string message
xsting_interfaces/LoadedPuck[] pucks
int32 box_remaining                          # 박스에 남은 퍽 수, 모르면 -1
---
string phase
```

`LoadedPuck.msg`
```
uint8 slot                                   # 0~11
int32 tag_id                                 # 퍽 AprilTag 번호 = 퍽 id, 못 읽으면 -1
string color                                 # "R" | "G" | "" (모름)
string shape                                 # "cylinder" | "box" | "" (모름)
float32 confidence                           # 0~1
```

### `UnloadPucks.action`

```
string location_id                           # L1 ~ L3
xsting_interfaces/PlaceItem[] items          # {int32 puck_id, uint8 cell}
builtin_interfaces/Duration timeout
---
bool success
uint16 error_code
string message
xsting_interfaces/PlacedItem[] items         # 시도한 항목까지 (실패 항목 포함)
int32 stopped_at                             # 멈춘 항목 번호(0부터), 끝까지 했으면 -1
---
string phase
```

`PlacedItem.msg`
```
int32 puck_id
uint8 cell
bool ok
uint16 error_code                            # 실패 항목의 ErrorCode
geometry_msgs/Point offset                   # 놓은 위치 오차 (cell 중심 기준, m)
```

### `ObserveLocation.action`

```
string location_id                           # L1 ~ L3
builtin_interfaces/Duration timeout
---
bool success
uint16 error_code
string message
xsting_interfaces/CellObservation[] cells    # 9칸, cell 0~8 순서
---
string phase
```

`CellObservation.msg`
```
uint8 cell
string state                                 # "empty" | "occupied" | "unknown"
int32 tag_id                                 # 그 칸 퍽의 AprilTag 번호, 모르면 -1
geometry_msgs/Point offset                   # cell 중심에서 퍽 중심까지 (m)
```

### `ErrorCode.msg` (상수만)

```
uint16 OK=0
# 공통 1xx
uint16 ROBOT_FAULT=100
uint16 INTERNAL_ERROR=101
uint16 CANCELED=102
uint16 TIMEOUT=103
uint16 BAD_REQUEST=104
# 주행 2xx
uint16 OBSTACLE_BLOCKED=200
uint16 NO_PROGRESS=201
uint16 NO_PATH=202
uint16 NAV_FAILED=203
uint16 LOCALIZATION_LOST=204
# 인식 3xx
uint16 TARGET_NOT_FOUND=300
uint16 PERCEPTION_FAILED=301
# 팔·적재함 4xx (팔이 닿지 않으면 GRASP_FAILED·PLACE_FAILED로 보낸다)
uint16 GRASP_FAILED=400
uint16 PLACE_FAILED=401
uint16 BOX_EMPTY=402
uint16 STORAGE_FULL=403
uint16 PUCK_NOT_IN_STORAGE=404
uint16 CELL_OCCUPIED=405
uint16 code
```

## 5. 에러 코드 대응

로봇 팀 `ErrorCode`를 노트북 코드로 바꾸는 표다. 노트북 쪽 코드 뜻은 `docs/error_codes.md`를 따른다.

| 로봇 팀 `ErrorCode` | 노트북 `error` | 비고 |
|---|---|---|
| `OBSTACLE_BLOCKED`, `NO_PROGRESS`, `NO_PATH`, `NAV_FAILED`, `LOCALIZATION_LOST` | 같은 이름 | GOTO |
| `TARGET_NOT_FOUND`, `PERCEPTION_FAILED` | `PERCEPTION_FAILED` | 다시 시도 대상 (2절) |
| `GRASP_FAILED`, `BOX_EMPTY`, `STORAGE_FULL` | 같은 이름 | LOAD |
| `PLACE_FAILED`, `PUCK_NOT_IN_STORAGE`, `CELL_OCCUPIED` | 같은 이름 | UNLOAD |
| `ROBOT_FAULT`, `INTERNAL_ERROR`, `CANCELED`, `TIMEOUT` | 같은 이름 | |
| `BAD_REQUEST`, 표에 없는 숫자 | `INTERNAL_ERROR` | 우리 쪽 요청이나 규약이 틀린 것. 원래 숫자를 `message`에 남긴다 |

## 6. 미정 사항 (회의 안건)

1. 액션 이름과 필드 (`/task/read_mission_input` 등), `RunTask`를 주행에 계속 쓸지 전용 액션을 둘지
2. 퍽 AprilTag 번호 = 퍽 ID인지 (미션지 placement.csv 확인), 퍽 색을 태그로 아는지 카메라로 판정하는지
3. cell 번호 → 놓을 위치 계산 (태그 면에서 격자 중심까지 실측 값, 태그 기준으로 맞추는지)
4. 출발 구역(`start_zone`) 실제 크기와 HOME 판정 방법 (지금 `arena.yaml` 값은 추정)
5. 액션별 `timeout` 값 (실물에서 걸린 시간을 재서 정한다)
6. 슬롯 번호 배치 (4×3 중 어디가 0번인지)
7. 로봇 팀 안에서 하는 다시 시도(집기 재시도 등)와 ros_backend의 다시 시도(2절)가 겹치지 않게 횟수 정하기
