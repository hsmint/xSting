"""LLM에 보여 줄 도구 정의. 이름과 enum은 고정 프롬프트(mission.md)와 똑같아야 한다.

- ROBOT_TOOLS: 로봇 행동. 중계 노드가 로봇에 보낸다. 모두 plan을 함께 받는다.
- COMPARE_ROUTES: 계산 도구. 로봇에 보내지 않고 중계 노드가 노트북에서 경로 길이를 계산해 바로 돌려준다.
순서와 내용을 바꾸지 않아야 프롬프트 캐시가 유지된다.
"""
from .rules import LOCATIONS, NUM_CELLS, NUM_PUCKS, PLACES


# 모든 도구에 함께 적는 계획. 중계 노드가 상태의 memory.plan으로 다음 판단에 돌려준다 (로봇에는 보내지 않는다)
_PLAN = {
    "type": "object",
    "description": "미션 끝까지의 계획. 프롬프트 '계획' 절 참고.",
    "properties": {
        "assignment": {"type": "string", "description": "결과만 한 줄 형식으로: 'L1=R×5: 0,1,2,3,9 / L2=G×5: 미정 / L3=Y×2(cylinder): 미정'. 따져 본 과정은 쓰지 않는다"},
        "route": {"type": "array", "items": {"type": "string"},
                  "description": "지금 부르는 행동 다음부터 HOME 복귀까지 남은 행동 순서"},
        "reason": {"type": "string", "description": "결론만 짧게 (200자 이내): 고른 순서의 이동 거리 합계, 비교한 다른 순서의 합계, 확정된 그룹"},
        "change": {"type": "string", "description": "memory.plan에서 바꾼 점과 이유. 그대로 이행하면 빈 문자열"},
    },
    "required": ["assignment", "route", "reason", "change"],
    "additionalProperties": False,
}


def _tool(name, description, properties=None):
    # OpenAI Responses API 함수 도구 형식. strict면 모든 속성이 required, additionalProperties는 false여야 한다.
    properties = {**(properties or {}), "plan": _PLAN}
    return {
        "type": "function",
        "name": name,
        "description": description,
        "strict": True,
        "parameters": {
            "type": "object",
            "properties": properties,
            "required": list(properties),
            "additionalProperties": False,
        },
    }


_ITEMS = {
    "type": "array",
    "items": {
        "type": "object",
        "properties": {
            "puck_id": {"type": "integer", "enum": list(range(NUM_PUCKS))},
            "cell": {"type": "integer", "enum": list(range(NUM_CELLS))},
        },
        "required": ["puck_id", "cell"],
        "additionalProperties": False,
    },
}

ROBOT_TOOLS = [
    _tool("READ_MISSION_INPUT",
          f"위치 조건: here=HOME. 바코드와 신호등 12회 점멸을 관측한다. 신호는 반복되므로 놓치면 다음 주기를 본다."),
    _tool("GOTO",
          "위치 조건: 없음. 대상 t 둘레의 갈 수 있는 자리로 이동해 대상을 바라보고 정지한다. "
          "경로는 로봇이 정하고, 도착한 자리는 robot.pose로 알려 준다. "
          "t=HOME이면 차체 전체가 출발 구역 안에서 3초 정지한다.",
          {"t": {"type": "string", "enum": list(PLACES)}}),
    _tool("LOAD",
          "위치 조건: here=S1~S6. 지금 있는 시작 박스의 퍽을 한 번에 모두(보통 2개) 집어 적재함 빈 slot에 싣는다. "
          "결과로 실은 퍽마다 id·색·모양을 알려 준다."),
    _tool("UNLOAD",
          f"위치 조건: here={'/'.join(LOCATIONS)}. 적재함의 퍽을 지금 있는 하차 박스의 cell에 items 순서대로 놓는다. "
          "그 하차 박스에 놓을 퍽 전부(눈 개수만큼)를 한 번에 보낸다. 나눠 놓으면 거절된다. "
          "중간에 실패하면 그 항목에서 멈추고 항목별 결과를 돌려준다.",
          {"items": _ITEMS}),
    _tool("VERIFY_LOCATION",
          f"위치 조건: here={'/'.join(LOCATIONS)}. 그 하차 박스의 UNLOAD가 끝난 뒤 부른다. "
          "items대로 놓였는지와 목록에 없는 cell에 퍽이 있는지를 새로 관측해 돌려준다.",
          {"items": _ITEMS}),
]

TOOL_NAMES = [t["name"] for t in ROBOT_TOOLS]  # 로봇 행동 이름 (기록·화면 명령 검사에 쓴다)

CALC_TOOL = "COMPARE_ROUTES"
MAX_ROUTES = 6  # 한 번에 비교할 수 있는 후보 순서 수
COMPARE_ROUTES = {
    "type": "function",
    "name": CALC_TOOL,
    "description": "계산 도구 (로봇은 움직이지 않는다). 지금 로봇 위치(robot.pose)에서 출발해 후보 순서마다 "
                   "장소를 차례로 도는 실제 경로 길이(벽·박스를 피한 A*, mm)를 돌려준다. "
                   f"후보는 {MAX_ROUTES}개까지. 판단 한 번에 3번까지 부를 수 있고, 그다음에는 로봇 행동을 하나 고른다.",
    "strict": True,
    "parameters": {
        "type": "object",
        "properties": {
            "routes": {"type": "array", "description": "후보 순서 목록. 각 후보는 들를 장소 ID를 순서대로 (예: [\"S2\", \"S6\", \"L3\", \"HOME\"])",
                       "items": {"type": "array", "items": {"type": "string", "enum": list(PLACES)}}},
        },
        "required": ["routes"],
        "additionalProperties": False,
    },
}

TOOLS = ROBOT_TOOLS + [COMPARE_ROUTES]  # LLM에 보여 주는 전체
