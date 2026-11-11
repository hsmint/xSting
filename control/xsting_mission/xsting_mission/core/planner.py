"""고정 행동 테스트(decider:=script)용 할당 계획. AI 판단에서는 쓰지 않는다."""
from . import rules


class PlanError(Exception):
    pass


def group_order(state):
    """{L1: 색, L2: 색, L3: 색}. 횟수를 바코드 방향으로 정렬, 같으면 R→Y→G 순."""
    world = state["world"]
    if not (world["barcode"] and world["barcode"]["valid"]
            and world["signal"] and world["signal"]["valid"]):
        raise PlanError("바코드·신호를 아직 읽지 않음")
    counts = world["signal"]["counts"]
    # sorted는 reverse=True여도 같은 값의 순서를 유지한다
    colors = sorted(rules.COLORS, key=lambda color: counts[color],
                    reverse=world["barcode"]["order"] == "desc")
    return dict(zip(rules.LOCATIONS, colors))


def assign(state):
    """{L1: [{puck_id, cell}, ...], ...}. 퍽 12개의 색·모양을 모두 알아야 한다."""
    groups = group_order(state)
    counts = state["world"]["signal"]["counts"]
    pucks = state["world"]["pucks"]
    if any(puck["color"] is None or puck["shape"] is None for puck in pucks):
        raise PlanError("아직 색·모양을 모르는 퍽이 있음 (모두 LOAD한 뒤에 계획한다)")

    red = [puck for puck in pucks if puck["color"] == "R"]
    green = [puck for puck in pucks if puck["color"] == "G"]
    spare_red, spare_green = len(red) - counts["R"], len(green) - counts["G"]
    if spare_red < 0 or spare_green < 0:
        raise PlanError(f"퍽 색이 모자람: 빨강 {len(red)}/{counts['R']}, "
                        f"초록 {len(green)}/{counts['G']}")

    # R·G 그룹에 쓰고 남는 퍽이 Y 그룹이 되고, Y는 한 모양이어야 한다
    for shape in rules.SHAPES:
        yellow_red = [puck for puck in red
                      if puck["shape"] == shape][:spare_red]
        yellow_green = [puck for puck in green
                        if puck["shape"] == shape][:spare_green]
        if (len(yellow_red) == spare_red
                and len(yellow_green) == spare_green):
            break
    else:
        raise PlanError("Y 그룹을 한 모양으로 만들 수 없음")
    yellow_ids = {puck["id"] for puck in yellow_red + yellow_green}
    members = {
        "R": [puck["id"] for puck in red if puck["id"] not in yellow_ids],
        "G": [puck["id"] for puck in green if puck["id"] not in yellow_ids],
        "Y": sorted(yellow_ids),
    }

    plan = {}
    for location, color in groups.items():
        cells = rules.DICE_CELLS[counts[color]]
        plan[location] = [{"puck_id": puck_id, "cell": cell}
                          for puck_id, cell in zip(members[color], cells)]
    return plan
