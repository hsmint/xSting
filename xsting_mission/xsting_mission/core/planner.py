"""고정 행동 테스트(decider:=script)용 할당 계획 및 allocation.json 산출물 생성기."""
import copy

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


def build_allocation_record(state):
    """제출 필수 산출물 allocation.json 생성용 구조화 레코드를 만든다."""
    world = state["world"]
    pucks = copy.deepcopy(world["pucks"])
    puck_by_id = {p["id"]: p for p in pucks}

    try:
        order_map = group_order(state)
    except PlanError:
        order_map = None

    try:
        planned_locs = assign(state)
        planned_groups = {}
        if order_map is not None:
            for loc, color in order_map.items():
                planned_groups[color] = [
                    item["puck_id"] for item in planned_locs.get(loc, [])]
        y_ids = planned_groups.get("Y", [])
        y_shapes = {puck_by_id[pid]["shape"] for pid in y_ids if pid in puck_by_id}
        planned = {
            "valid": True,
            "y_shape": next(iter(y_shapes)) if len(y_shapes) == 1 else None,
            "groups": planned_groups,
            "locations": planned_locs,
            "error": None,
        }
    except PlanError as err:
        planned = {
            "valid": False,
            "y_shape": None,
            "groups": None,
            "locations": None,
            "error": str(err),
        }

    placed_locs = {}
    placed_ids = []
    for loc in rules.LOCATIONS:
        box = world["locations"][loc]
        items = [
            {"puck_id": cell["puck_id"], "cell": cell["cell"]}
            for cell in box["cells"]
            if cell.get("state") == "occupied"
        ]
        placed_locs[loc] = {
            "placed": items,
            "verified": copy.deepcopy(box["verified"]),
        }
        for item in items:
            if item.get("puck_id") is not None:
                placed_ids.append(int(item["puck_id"]))

    seen = set()
    duplicate_ids = []
    for pid in placed_ids:
        if pid in seen and pid not in duplicate_ids:
            duplicate_ids.append(pid)
        seen.add(pid)
    duplicate_ids.sort()
    missing_ids = [pid for pid in range(rules.NUM_PUCKS) if pid not in seen]

    y_shape_unified = None
    group_counts_match = None
    if order_map is not None and world["signal"] and world["signal"]["valid"]:
        counts = world["signal"]["counts"]
        counts_ok = True
        for loc, color in order_map.items():
            loc_items = placed_locs[loc]["placed"]
            expected_n = counts[color]
            expected_cells = set(rules.DICE_CELLS[expected_n])
            actual_cells = {item["cell"] for item in loc_items}
            if len(loc_items) != expected_n or actual_cells != expected_cells:
                counts_ok = False
            if color == "R" and any(
                    puck_by_id.get(item["puck_id"], {}).get("color") != "R"
                    for item in loc_items):
                counts_ok = False
            if color == "G" and any(
                    puck_by_id.get(item["puck_id"], {}).get("color") != "G"
                    for item in loc_items):
                counts_ok = False
            if color == "Y" and loc_items:
                shapes = {
                    puck_by_id.get(item["puck_id"], {}).get("shape")
                    for item in loc_items
                }
                y_shape_unified = (len(shapes) == 1 and None not in shapes)
        group_counts_match = counts_ok

    complete = bool(
        len(seen) == rules.NUM_PUCKS
        and not duplicate_ids
        and not missing_ids
        and y_shape_unified is True
        and group_counts_match is True
    )

    return {
        "stamp": (state.get("meta") or {}).get("stamp"),
        "phase": state["phase"],
        "barcode": copy.deepcopy(world["barcode"]),
        "signal": copy.deepcopy(world["signal"]),
        "group_order": order_map,
        "pucks": pucks,
        "planned": planned,
        "active_plan": copy.deepcopy((state.get("memory") or {}).get("plan")),
        "placed": {
            "locations": placed_locs,
            "consistency": {
                "placed_puck_ids": placed_ids,
                "unique_count": len(seen),
                "duplicate_ids": duplicate_ids,
                "missing_ids": missing_ids,
                "y_shape_unified": y_shape_unified,
                "group_counts_match": group_counts_match,
                "complete": complete,
            },
        },
    }
