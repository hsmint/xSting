"""고정 행동 테스트(llm_commander decider:=script)용 할당 계획.

AI 판단(decider:=llm)에서는 쓰지 않는다. 할당은 AI가 한다.
신호 횟수·바코드로 L1~L3의 그룹과 눈 개수를 정하고, 실은 퍽의 색·모양으로 어느 퍽을 어느 cell에 놓을지 정한다.
"""
from . import rules


class PlanError(Exception):
    pass


def group_order(state):
    """{L1: 색, L2: 색, L3: 색}. 횟수를 바코드 방향으로 정렬하고, 같으면 R → Y → G 순서를 유지한다."""
    w = state["world"]
    if not (w["barcode"] and w["barcode"]["valid"] and w["signal"] and w["signal"]["valid"]):
        raise PlanError("바코드·신호를 아직 읽지 않음")
    counts = w["signal"]["counts"]
    # sorted는 같은 값의 순서를 유지한다 (reverse=True여도)
    colors = sorted(rules.COLORS, key=lambda c: counts[c], reverse=w["barcode"]["order"] == "desc")
    return dict(zip(rules.LOCATIONS, colors))


def assign(state):
    """{L1: [{puck_id, cell}, ...], L2: ..., L3: ...}. 퍽 12개의 색·모양을 모두 알아야 한다."""
    groups = group_order(state)
    counts = state["world"]["signal"]["counts"]
    pucks = state["world"]["pucks"]
    if any(p["color"] is None or p["shape"] is None for p in pucks):
        raise PlanError("아직 색·모양을 모르는 퍽이 있음 (모두 LOAD한 뒤에 계획한다)")
    red = [p for p in pucks if p["color"] == "R"]
    green = [p for p in pucks if p["color"] == "G"]
    spare_r, spare_g = len(red) - counts["R"], len(green) - counts["G"]
    if spare_r < 0 or spare_g < 0:
        raise PlanError(f"퍽 색이 모자람: 빨강 {len(red)}/{counts['R']}, 초록 {len(green)}/{counts['G']}")
    # R·G 그룹에 쓰고 남는 퍽이 Y 그룹이 되고, Y는 한 모양이어야 한다
    for shape in rules.SHAPES:
        y_red = [p for p in red if p["shape"] == shape][:spare_r]
        y_green = [p for p in green if p["shape"] == shape][:spare_g]
        if len(y_red) == spare_r and len(y_green) == spare_g:
            break
    else:
        raise PlanError("Y 그룹을 한 모양으로 만들 수 없음")
    y_ids = {p["id"] for p in y_red + y_green}
    members = {"R": [p["id"] for p in red if p["id"] not in y_ids],
               "G": [p["id"] for p in green if p["id"] not in y_ids],
               "Y": sorted(y_ids)}
    plan = {}
    for loc, color in groups.items():
        cells = rules.DICE_CELLS[counts[color]]
        plan[loc] = [{"puck_id": i, "cell": c} for i, c in zip(members[color], cells)]
    return plan
