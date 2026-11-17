"""LLM 계산 도구 COMPARE_ROUTES: 정의와 A* 경로 길이 계산 (로봇에는 안 보냄)."""
from ..core.rules import PLACES

CALC_TOOL = "COMPARE_ROUTES"
MAX_ROUTES = 6  # 한 번에 비교할 수 있는 후보 순서 수
COMPARE_ROUTES = {
    "type": "function",
    "name": CALC_TOOL,
    "description": (
        "계산 도구 (로봇은 움직이지 않는다). 지금 로봇 위치"
        "(robot.pose)에서 출발해 후보 순서마다 "
        "장소를 차례로 도는 실제 경로 길이(벽·박스를 피한 A*, mm)를 돌려준다. "
        f"후보는 {MAX_ROUTES}개까지. 판단 한 번에 3번까지 부를 수 있고, "
        "그다음에는 로봇 행동을 하나 고른다."),
    "strict": True,
    "parameters": {
        "type": "object",
        "properties": {
            "routes": {
                "type": "array",
                "description": "후보 순서 목록. 각 후보는 들를 장소 ID를 "
                               "순서대로 (예: [\"S2\", \"S6\", \"L3\", "
                               "\"HOME\"])",
                "items": {"type": "array",
                          "items": {"type": "string", "enum": list(PLACES)}},
            },
        },
        "required": ["routes"],
        "additionalProperties": False,
    },
}


def compare_routes(arena_map, args, state):
    """보낸 상태의 로봇 위치에서 후보 순서별 경로 길이."""
    routes = [list(route) for route in (args.get("routes") or [])]
    routes = routes[:MAX_ROUTES]
    pose, here = state["robot"]["pose"], state["robot"]["here"]
    if pose:
        start = (pose["x"], pose["y"])
    else:  # 첫 판단 등 pose를 아직 모르면 지금 장소의 등록 위치에서
        start = arena_map.targets.get(here, arena_map.targets["HOME"])
    result = {"from": {"x": round(start[0]), "y": round(start[1])},
              "routes": arena_map.route_lengths(start, routes),
              "note": "길이는 mm. 장소는 대상 둘레(330~520mm)의 가장 가까운 "
                      "자리까지. None은 갈 수 없는 구간."}
    if len(args.get("routes") or []) > MAX_ROUTES:
        result["note"] += f" 후보는 {MAX_ROUTES}개까지만 계산했다."
    return result
