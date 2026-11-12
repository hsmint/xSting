# ros_backend → 로봇 팀 명령 형식

> 상태: 초안 (2026-11-12). 로봇 팀(주행·인식·팔)과 회의해 확정한다. 확정 전에는 이름·필드가 바뀔 수 있다.
> 노트북 → 로봇 서버 규약은 `xsting_mission/core/contract.py`, 에러 코드 뜻은 `docs/error_codes.md`가 원본이다.

## 1. 층과 책임

```
노트북 llm_commander
   │  RobotCommand 액션 (행동 5개, JSON)          ← contract.py
   ▼
젯슨 robot_command_server + ros_backend.py        ← 우리 (이 문서의 "보내는 쪽")
   │  기본 동작 액션·서비스 (ROS 타입)              ← 이 문서
   ▼
젯슨 로봇 팀 노드: 작업 관리자(RunTask), 주행(강화학습), 인식, 팔, 보관 관리자
```

| | ros_backend (우리) | 로봇 팀 |
|---|---|---|
| 하는 일 | 행동 하나를 기본 동작 여러 개로 나눠 부르고, 결과를 모아 `contract.py` 형식으로 돌려준다 | 기본 동작 하나를 끝까지 해내고 결과를 돌려준다 |
| 아는 것 | 미션 규칙(신호 규칙, 주사위 cell, 행동 순서), 다시 시도 횟수, 에러 코드 변환 | 로봇 몸(주행, 카메라, 팔, 적재함). 미션 규칙은 몰라도 된다 |
| 예 | 신호를 3번까지 다시 읽기, cell 번호를 위치로 바꾸기, 퍽 id 붙이기 | 장소로 가서 서기, 퍽 집어 슬롯에 넣기, 정해진 위치에 놓고 2초 안정 확인 |

## 2. 공통 규칙

### 이름

| 대상 | 형식 | 비고 |
|---|---|---|
| 장소 ID | `HOME`, `S1`~`S6`, `L1`~`L3` (문자열) | 노트북과 같다. 좌표는 `config/arena.yaml`의 `places.<ID>.target` |
| 퍽 id | 정수 0~11 | 미션지의 퍽 ID (placement.csv). **퍽 AprilTag 번호로 읽는다**. 못 읽으면 -1 |
| 인식 물체 id | `object_id` 문자열 | 인식 노드의 추적용 id. 퍽 id와 다르다. 대응은 ros_backend가 관리 |
| 적재함 슬롯 | `slot_id` 문자열 `"0"`~`"11"` | 노트북에는 같은 번호의 정수로 보낸다. 번호 배치(4×3 중 어디가 0번)는 로봇 팀이 정해 알려 준다 |
| 하차 cell | 정수 0~8 | 위쪽 줄 0 1 2 / 가운데 3 4 5 / 아래쪽 6 7 8, 지도 +Y가 위쪽 (모든 하차 박스 같은 방향) |
| 행동 이름 | 노트북: `GOTO` `READ_MISSION_INPUT` `LOAD` `UNLOAD` `VERIFY_LOCATION` | 로봇 팀 작업 이름은 아래 4절 |

### 좌표계 (TF frame)

| frame | 원점·축 | 누가 발행 |
|---|---|---|
| `map` | 경기장 원점(왼쪽 아래 기둥 중심), +Y 지도 위쪽 | 위치 추정(cuVSLAM). 경기장 원점과 맞춰야 한다 |
| `base_link` | 로봇 차체 중심 | 하드웨어 |
| `l1_grid`, `l2_grid`, `l3_grid` | 하차 박스 3×3 격자 중심, 축은 `map`과 같은 방향 | 인식 (장소 태그 12/13/14를 보고 계산) |

- `lN_grid`로 놓을 위치를 주면, 로봇의 지도상 위치 오차(cuVSLAM 오차)가 놓는 위치에 섞이지 않는다. 10mm 기준을 맞추기 위한 핵심이다.
- cell `c`의 위치(`lN_grid` 기준, m): `x = (c % 3 - 1) × 0.060`, `y = (1 - c // 3) × 0.060`, `z = 0`.

### 단위

- ROS 메시지 안에서는 ROS 표준 단위(m, rad, 초)를 쓴다.
- 노트북 규약(`contract.py`)은 mm, deg다. 변환은 ros_backend가 한다.

### 결과·에러

- 모든 결과에는 `bool success`, `uint16 error_code`, `string message`가 있다.
- `error_code`는 5절의 `ErrorCode` 상수를 쓴다. ros_backend가 같은 이름의 문자열 코드로 바꿔 노트북에 보낸다.
- 표에 없는 숫자는 `ROBOT_FAULT`로 바꾸고, 원래 숫자를 `message`에 남긴다.

### 취소·시간

- 노트북이 행동을 취소하면 ros_backend는 진행 중인 하위 액션에 취소를 보낸다. 로봇 팀 액션은 취소를 받으면 안전한 상태로 멈추고 `CANCELED`로 끝낸다.
- 기본 동작마다 `timeout`을 goal에 넣는다. 로봇 팀은 그 안에 끝내거나 실패로 돌려준다.

## 3. 쓰는 인터페이스

| 이름 | 타입 | 상태 | 쓰는 행동 |
|---|---|---|---|
| `/task/run` | `xsting_interfaces/action/RunTask` | 있음 (origin/vision, 필드 추가 제안) | GOTO, LOAD, UNLOAD |
| `/storage/reserve_slot` | `xsting_interfaces/srv/ReserveSlot` | 있음 (origin/vision) | LOAD |
| `/storage/get_storage` | `xsting_interfaces/srv/GetStorage` | **새로 제안** | LOAD, UNLOAD |
| `/perception/read_barcode` | `xsting_interfaces/srv/ReadBarcode` | **새로 제안** | READ_MISSION_INPUT |
| `/perception/watch_signal` | `xsting_interfaces/action/WatchSignal` | **새로 제안** | READ_MISSION_INPUT |
| `/perception/detect_pucks` | `xsting_interfaces/srv/DetectPucks` | **새로 제안** | LOAD |
| `/perception/observe_cells` | `xsting_interfaces/srv/ObserveCells` | **새로 제안** | VERIFY_LOCATION |
| TF `map → base_link` | tf2 | 위치 추정 | GOTO (`robot.pose`, HOME 구역 판정) |

## 4. 행동별 흐름

### GOTO `{t}`

1. `RunTask(operation=NAVIGATE, place_id=t, destination=places.<t>.target in map)`
   - 접근 자리(어디에 어느 방향으로 설지)는 주행 쪽이 정한다. 박스 앞에서는 팔이 먼 줄 퍽까지 닿고 카메라가 태그를 보는 자리여야 한다.
   - HOME이면 `destination`은 `arena.yaml`의 `start_pose`다.
2. 성공하면 TF `map → base_link`로 `robot.pose`를 채운다.
3. HOME이면 3.2초 기다린 뒤, 차체 footprint 네 꼭짓점이 출발 구역(`start_zone`) 안에 있는지 TF로 계산해 `in_zone`과 `stopped_s`를 채운다.

노트북에 보내는 결과:
```json
{"success": true, "robot": {"here": "S1", "pose": {"x": 1482, "y": 567, "yaw": 92.1}}}
{"success": true, "robot": {"here": "HOME", "pose": {...}}, "in_zone": true, "stopped_s": 3.2}
{"success": false, "error": "OBSTACLE_BLOCKED", "robot": {"here": "UNKNOWN", "pose": {...}}}
```
실패하면 `here`는 `UNKNOWN`으로 둔다. 가드가 틀린 위치를 믿지 않게 하기 위해서다.

### READ_MISSION_INPUT `{}`

1. `ReadBarcode()` → `code`
2. `WatchSignal()` → `colors[12]`, `end_seen`
3. `rules.signal_errors(colors, end_seen)`가 오류를 내면 2로 돌아간다. 처음을 포함해 최대 3번까지. 중간 오류는 feedback으로만 보낸다.
4. 3번 다 오류면 `PERCEPTION_FAILED`를 보낸다. `cause`는 `?`가 남았으면 `SIGNAL_INCOMPLETE`, 아니면 `SIGNAL_INVALID`이고, `signal_errors`도 싣는다.
5. 신호등이나 바코드가 화면에 없으면(`TARGET_NOT_FOUND`) 그 자리에서 한 바퀴 돌며 찾는다. 이 탐색을 누가 할지는 7절 미정 사항이다. 그래도 못 찾으면 `cause: LIGHT_NOT_FOUND`.

```json
{"success": true, "barcode": "P2-02", "signal": ["G","Y","R", ...12개], "end_seen": true,
 "signal_reread": {"reads": 2, "complete": true}}
```

### LOAD `{}`

1. `DetectPucks(place_id=here)` → 박스 위 퍽 목록 (`object_id`, `tag_id`, 색, 모양, 신뢰도)
2. 퍽마다:
   1. `ReserveSlot()` → 빈 `slot_id`
   2. `RunTask(operation=PICK_STORE, object_id, slot_id)`
   3. 실패하면 거기서 멈춘다.
3. 퍽 id는 `tag_id`다. 태그를 못 읽었으면 `null`로 보낸다. 노트북 상태에는 `unknown`으로 남는다.
4. `box_remaining`은 1에서 본 개수에서 실은 개수를 뺀 값이다.

```json
{"success": true, "box_remaining": 0,
 "pucks": [{"slot": 0, "puck": {"id": 7, "color": "R", "shape": "cylinder", "conf": 0.93}},
           {"slot": 1, "puck": {"id": 2, "color": "G", "shape": "box", "conf": 0.88}}]}
{"success": false, "error": "GRASP_FAILED", "box_remaining": 1, "pucks": [{"slot": 0, "puck": {...}}]}
```

### UNLOAD `{items: [{puck_id, cell}, ...]}`

1. `GetStorage()`로 퍽 id마다 들어 있는 `slot_id`를 찾는다. 없으면 그 항목에서 `PUCK_NOT_IN_STORAGE`로 멈춘다.
2. items 순서대로:
   1. `RunTask(operation=STORE_PLACE, slot_id, destination={frame: lN_grid, cell 위치})`
   2. 로봇 팀은 놓은 뒤 2초 안정을 확인하고, 놓은 위치 오차(`offset`)를 돌려준다.
   3. 실패하면 거기서 멈춘다. `stopped_at`은 그 항목 번호(0부터)다.

```json
{"success": true, "stopped_at": null,
 "items": [{"puck_id": 7, "cell": 0, "ok": true, "offset_mm": [1.5, -2.0]}, ...]}
{"success": false, "error": "PLACE_FAILED", "stopped_at": 2,
 "items": [{"puck_id": 7, "cell": 0, "ok": true}, {"puck_id": 4, "cell": 2, "ok": true},
           {"puck_id": 9, "cell": 4, "ok": false, "error": "PLACE_FAILED"}]}
```

### VERIFY_LOCATION `{items: [{puck_id, cell}, ...]}`

1. `ObserveCells(location_id=here)` → 9칸 관측 (상태, 그 칸 퍽의 `tag_id`, 오차)
2. ros_backend가 items와 비교한다.
   - `ok`: 그 cell에 그 퍽이 있다.
   - `seen_puck_id`: 실제로 본 퍽 id
   - `extra`: items에 없는데 퍽이 있는 cell

```json
{"success": true, "all_ok": true, "extra": [],
 "items": [{"puck_id": 7, "cell": 0, "ok": true, "seen_puck_id": 7, "offset_mm": [1.2, -0.8]}],
 "cells": [{"cell": 0, "state": "occupied", "puck_id": 7}, ... 9칸]}
```

## 5. 인터페이스 초안

### `RunTask.action` (origin/vision 것에 `place_id` 추가)

```
# NAVIGATE / PICK_STORE / PICK_PLACE / STORE_PLACE
string task_id
string operation
string place_id                              # 추가: NAVIGATE 목적지 장소 (HOME, S1~S6, L1~L3)
string object_id                             # PICK_STORE
string slot_id                               # PICK_STORE, STORE_PLACE
geometry_msgs/PoseStamped destination        # NAVIGATE: 대상 중심, STORE_PLACE: lN_grid 기준 cell 위치
builtin_interfaces/Duration timeout
---
bool success
uint16 error_code                            # ErrorCode 상수
string message
geometry_msgs/Point place_offset             # 추가: STORE_PLACE 놓은 위치 오차 (destination frame, m)
---
string phase
float32 progress
```

### `ErrorCode.msg` (새로 제안, 상수만)

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
# 팔·적재함 4xx
uint16 GRASP_FAILED=400
uint16 PLACE_FAILED=401
uint16 BOX_EMPTY=402
uint16 STORAGE_FULL=403
uint16 PUCK_NOT_IN_STORAGE=404
uint16 CELL_OCCUPIED=405
uint16 UNREACHABLE=406
uint16 code
```

### `ReadBarcode.srv`

```
builtin_interfaces/Duration timeout
---
bool success
uint16 error_code
string message
string code                                  # "P2-01" ~ "P2-04"
float32 confidence
```

### `WatchSignal.action`

```
builtin_interfaces/Duration timeout          # 한 주기 + 여유 (약 40초)
---
bool success
uint16 error_code
string message
string[] colors                              # 12개, "R" | "Y" | "G" | "?" (못 읽은 칸)
bool end_seen                                # 12번째 뒤 3초 소등을 봤나
---
uint8 index                                  # 지금 본 점멸 번호 (1~12)
string color
```

### `DetectPucks.srv`

```
string place_id                              # S1 ~ S6
---
bool success
uint16 error_code
string message
xsting_interfaces/PuckObservation[] pucks
```

`PuckObservation.msg`
```
string object_id
int32 tag_id                                 # 퍽 AprilTag 번호, 못 읽으면 -1
string color                                 # "R" | "G" | "" (모름)
string shape                                 # "cylinder" | "box" | "" (모름)
float32 confidence                           # 0~1
geometry_msgs/PoseStamped pose
```

### `ObserveCells.srv`

```
string location_id                           # L1 ~ L3
---
bool success
uint16 error_code
string message
xsting_interfaces/CellObservation[] cells    # 9칸, cell 0~8 순서
```

`CellObservation.msg`
```
uint8 cell
string state                                 # "empty" | "occupied" | "unknown"
int32 tag_id                                 # 그 칸 퍽의 AprilTag 번호, 모르면 -1
geometry_msgs/Point offset                   # cell 중심에서 퍽 중심까지 (lN_grid, m)
```

### `GetStorage.srv`

```
---
bool success
string message
xsting_interfaces/StorageSlot[] slots        # 12칸
```

`StorageSlot.msg`
```
string slot_id
string state                                 # "EMPTY" | "RESERVED" | "OCCUPIED" | "UNKNOWN"
string object_id
int32 tag_id                                 # 실린 퍽 번호, 모르면 -1
```

## 6. 에러 코드 대응

로봇 팀 `ErrorCode`를 노트북 코드로 바꾸는 표다. 노트북 쪽 코드 뜻은 `docs/error_codes.md`를 따른다.

| 로봇 팀 `ErrorCode` | 노트북 `error` | 비고 |
|---|---|---|
| `OBSTACLE_BLOCKED`, `NO_PROGRESS`, `NO_PATH`, `NAV_FAILED`, `LOCALIZATION_LOST` | 같은 이름 | GOTO |
| `TARGET_NOT_FOUND`, `PERCEPTION_FAILED` | `PERCEPTION_FAILED` | `cause`로 구분 (`LIGHT_NOT_FOUND` 등) |
| `GRASP_FAILED`, `BOX_EMPTY`, `STORAGE_FULL` | 같은 이름 | LOAD |
| `PLACE_FAILED`, `PUCK_NOT_IN_STORAGE`, `CELL_OCCUPIED` | 같은 이름 | UNLOAD |
| `UNREACHABLE` | LOAD면 `GRASP_FAILED`, UNLOAD면 `PLACE_FAILED` | `cause: UNREACHABLE`. 팔이 닿지 않음 |
| `ROBOT_FAULT`, `INTERNAL_ERROR`, `CANCELED` | 같은 이름 | |
| `TIMEOUT` (하위 동작) | 그 행동의 실패 코드 | `cause: TIMEOUT` |
| `BAD_REQUEST`, 표에 없는 숫자 | `INTERNAL_ERROR` | 우리 쪽 요청이 틀린 것이므로 코드 문제다. 원래 숫자를 `message`에 남긴다 |

## 7. 미정 사항 (회의 안건)

1. `RunTask`를 이 형식으로 쓸지, 작업 관리자를 우리가 맡을지 (ros_backend와 작업 관리자는 하는 일이 겹친다)
2. 슬롯 예약 순서: ros_backend가 `ReserveSlot`을 부르나, 작업 관리자가 `PICK_STORE` 안에서 하나
3. 퍽 AprilTag 번호 = 퍽 ID인지 (미션지 placement.csv 확인), 퍽 색을 태그로 아는지 카메라로 판정하는지
4. 신호등·박스를 못 찾을 때 한 바퀴 도는 탐색을 누가 하나 (인식 서비스 안 / ros_backend가 회전 요청)
5. `lN_grid` frame을 인식 쪽이 발행할 수 있는지, 태그 면에서 격자 중심까지의 실측 값
6. 출발 구역(`start_zone`) 실제 크기와 HOME 판정 방법 (지금 `arena.yaml` 값은 추정)
7. 기본 동작별 `timeout` 값 (실물에서 걸린 시간을 재서 정한다)
8. 슬롯 번호 배치 (4×3 중 어디가 0번인지)
