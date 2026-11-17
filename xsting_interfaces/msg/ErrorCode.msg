# 로봇 팀 기본 동작의 에러 코드. 이름은 노트북 에러 코드(docs/error_codes.md)와 같게 한다
# 초안: docs/robot_primitives.md (xsting_mission)
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
