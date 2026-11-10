"""LLM에 보여 줄 도구(행동) 정의. 이름과 enum은 고정 프롬프트(mission.md)와 똑같아야 한다.

순서와 내용을 바꾸지 않아야 프롬프트 캐시가 유지된다.
"""
from .rules import LOCATIONS, NUM_CELLS, NUM_PUCKS, PLACES


def _tool(name, description, properties=None):
    # OpenAI Responses API 함수 도구 형식. strict면 모든 속성이 required, additionalProperties는 false여야 한다.
    properties = properties or {}
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
          "위치 조건: 없음. 대상 t의 접근 위치로 이동해 대상을 바라보고 정지한다. "
          "t=HOME이면 차체 전체가 출발 구역 안에서 3초 정지한다.",
          {"t": {"type": "string", "enum": list(PLACES)}}),
    _tool("LOAD",
          "위치 조건: here=S1~S6. 지금 있는 시작 박스의 퍽 하나를 집어 적재함 빈 slot에 싣는다. "
          "어느 퍽을 집을지는 로봇이 정하고, 결과로 퍽 id·색·모양을 알려 준다."),
    _tool("UNLOAD",
          f"위치 조건: here={'/'.join(LOCATIONS)}. 적재함의 퍽을 지금 있는 하차 박스의 cell에 items 순서대로 놓는다. "
          "중간에 실패하면 그 항목에서 멈추고 항목별 결과를 돌려준다.",
          {"items": _ITEMS}),
    _tool("VERIFY_LOCATION",
          f"위치 조건: here={'/'.join(LOCATIONS)}. 그 하차 박스의 UNLOAD가 끝난 뒤 부른다. "
          "items대로 놓였는지와 목록에 없는 cell에 퍽이 있는지를 새로 관측해 돌려준다.",
          {"items": _ITEMS}),
]

TOOL_NAMES = [t["name"] for t in TOOLS]
