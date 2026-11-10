"""LLM에 보여 줄 도구(행동) 정의. 이름과 enum은 고정 프롬프트(mission.md)와 똑같아야 한다.

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

TOOLS = [
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

TOOL_NAMES = [t["name"] for t in TOOLS]
