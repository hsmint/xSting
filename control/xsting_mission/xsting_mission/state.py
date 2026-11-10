"""MissionState: 중계 노드(llm_commander)가 들고 있는 상태.

LLM에는 매 판단마다 고정 프롬프트 + 이 상태 하나만 보낸다(대화 기록 없음).
값은 Jetson(로봇)이 돌려준 행동 결과를 그대로 기록한 것이다. 중계 노드는 할당·순서를 계산하지 않는다.
"""
import json
from datetime import datetime, timezone

from . import rules

PHASES = ("READ_MISSION_INPUT", "LOAD", "UNLOAD", "VERIFY_LOCATION", "GOTO", "DONE")
ROBOT_FIELDS = ("status", "error", "here", "pose", "localization", "arm_stowed", "gripper")

# 행동별 위치 조건 (robot.here가 이 중 하나여야 한다). GOTO는 어디서든 가능하다.
LOCATION_CONDITION = {
    "READ_MISSION_INPUT": (rules.HOME,),
    "LOAD": rules.START_BOXES,
    "UNLOAD": rules.LOCATIONS,
    "VERIFY_LOCATION": rules.LOCATIONS,
}


class ToolError(Exception):
    """가드가 로봇에 보내기 전에 거절한 명령. 메시지는 LLM에 그대로 전달된다."""


def empty_slot(i):
    return {"slot": i, "state": "empty", "puck_id": None, "color": None, "shape": None}


def empty_cell(i):
    return {"cell": i, "state": "empty", "puck_id": None}


def new_state():
    return {
        "meta": {"seq": 0, "stamp": None},
        "phase": "READ_MISSION_INPUT",
        "robot": {
            "status": "OK",  # OK | BUSY | WARN | FAULT
            "error": None,  # {"code", "message"}
            "here": rules.HOME,  # HOME | S1~S6 | L1~L3 | MOVING | UNKNOWN
            "pose": None,  # {"x", "y" (mm), "yaw" (deg)}. 로봇이 결과마다 알려 준다
            "localization": "ok",  # ok | degraded | lost
            "arm_stowed": True,
            "gripper": "empty",  # empty | holding
            "home_stopped": False,  # HOME 구역 안에서 3초 정지 완료 (다른 곳으로 GOTO하면 false)
            "storage": [empty_slot(i) for i in range(rules.STORAGE_SLOTS)],
        },
        "world": {
            "barcode": None,  # {"code", "order", "valid"}
            "signal": None,  # {"sequence", "counts", "valid", "errors"}
            "boxes": {box: {"status": "UNVISITED", "loaded": []} for box in rules.START_BOXES},
            "pucks": [
                {"id": i, "color": None, "shape": None, "conf": None, "origin": None,
                 "where": "unknown", "slot": None, "placed": None}
                for i in range(rules.NUM_PUCKS)
            ],
            "locations": {
                loc: {"cells": [empty_cell(c) for c in range(rules.NUM_CELLS)], "verified": None}
                for loc in rules.LOCATIONS
            },
        },
        "last_action": None,
        # LLM이 직전 판단에 적은 계획 {turn, assignment, route, reason}. 대화 기록이 없으므로 계획은 여기로 이어진다
        "memory": {"plan": None},
    }


def stamp_now(t=None):
    """UTC 시각 문자열. t(유닉스 시각)를 주면 그 시각으로 (지난 판 상태 복원용)."""
    dt = datetime.now(timezone.utc) if t is None else datetime.fromtimestamp(t, timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%SZ")


def check_location(state, name):
    """가드: 위치 조건을 만족하지 않으면 로봇에 보내기 전에 거절한다."""
    allowed = LOCATION_CONDITION.get(name)
    here = state["robot"]["here"]
    if allowed and here not in allowed:
        raise ToolError(f"LOCATION_CONDITION: {name}은(는) {'/'.join(allowed)}에서만 가능, 지금 here={here}")


def check_unload(state, args):
    """UNLOAD는 하차 박스 하나에 놓을 퍽을 한 번에 모두 놓아야 한다 (나눠 놓지 않는다).

    이미 놓은 퍽 수 + 이번 items 수 = 그 하차 박스의 눈 개수여야 한다. 처음에는 전부를 한 번에,
    도중에 실패해 멈췄으면 남은 퍽 전부를 한 번에 놓는 경우만 통과한다.
    """
    from .planner import PlanError, group_order  # planner가 rules만 쓰므로 순환하지 않는다
    loc = state["robot"]["here"]
    if loc not in rules.LOCATIONS:
        return  # 위치 조건은 check_location(가드)이 본다
    try:
        color = group_order(state)[loc]  # 바코드·신호가 없으면 여기서 PlanError
        need = state["world"]["signal"]["counts"][color]
    except PlanError as e:
        raise ToolError(f"PARTIAL_UNLOAD: {e}. 바코드·신호를 읽은 뒤 UNLOAD한다")
    cells_now = state["world"]["locations"][loc]["cells"]
    placed = sum(c["state"] == "occupied" for c in cells_now)
    items = args.get("items", [])
    # cell은 그 눈 개수의 cell 목록에서만, 겹치지 않게, 이미 놓은 칸이 아닌 곳에 놓는다
    allowed = rules.DICE_CELLS.get(need, [])
    cells = [it.get("cell") for it in items]
    used = [c["cell"] for c in cells_now if c["state"] == "occupied"]
    if any(c not in allowed for c in cells) or len(set(cells)) != len(cells) or any(c in used for c in cells):
        raise ToolError(f"WRONG_CELLS: {loc}는 눈 {need}개라 cell {allowed}에 하나씩 놓는다 "
                        f"(받은 cell {cells}" + (f", 이미 놓은 cell {used}" if used else "") + ")")
    if placed + len(items) != need:
        raise ToolError(f"PARTIAL_UNLOAD: {loc}에는 퍽 {need}개를 한 번에 놓아야 한다 "
                        f"(이미 놓은 것 {placed}개 + 이번 items {len(args.get('items', []))}개). "
                        f"그 하차 박스에 놓을 퍽이 모두 적재함에 있을 때 UNLOAD한다")


def apply_result(state, name, args, result, t=None):
    """Jetson이 돌려준 행동 결과를 상태에 기록한다. t는 결과를 받은 시각(지난 판 복원할 때만 준다)."""
    ok = bool(result.get("success"))
    robot, world = state["robot"], state["world"]

    # 로봇 상태는 매 결과에 함께 온다
    for key in ROBOT_FIELDS:
        if key in result.get("robot", {}):
            robot[key] = result["robot"][key]

    if name == "READ_MISSION_INPUT" and ok:
        code = result.get("barcode")
        order = rules.BARCODE_ORDER.get(code)
        world["barcode"] = {"code": code, "order": order, "valid": order is not None}
        seq = list(result.get("signal", []))
        errors = rules.signal_errors(seq, result.get("end_seen", False))
        world["signal"] = {"sequence": seq, "counts": rules.count_signal(seq),
                           "valid": not errors, "errors": errors}

    elif name == "GOTO":
        robot["home_stopped"] = (ok and args["t"] == rules.HOME and result.get("in_zone", False)
                                 and result.get("stopped_s", 0) >= 3.0)
        if ok and args["t"] in world["boxes"] and world["boxes"][args["t"]]["status"] == "UNVISITED":
            world["boxes"][args["t"]]["status"] = "VISITED"

    elif name == "LOAD":
        box = world["boxes"].get(robot["here"])
        if ok:
            # 한 번에 그 박스의 퍽을 모두 싣는다. 예전 기록은 {"slot", "puck"} 하나짜리 형식이다
            loaded = result["pucks"] if "pucks" in result else [{"slot": result["slot"], "puck": result["puck"]}]
            for item in loaded:
                puck, slot = item["puck"], item["slot"]
                robot["storage"][slot] = {"slot": slot, "state": "occupied" if puck.get("id") is not None else "unknown",
                                          "puck_id": puck.get("id"), "color": puck.get("color"),
                                          "shape": puck.get("shape")}
                if puck.get("id") is not None:
                    world["pucks"][puck["id"]].update(color=puck.get("color"), shape=puck.get("shape"),
                                                      conf=puck.get("conf"), origin=robot["here"],
                                                      where="storage", slot=slot)
                    if box is not None:
                        box["loaded"].append(puck["id"])
        if box is not None and result.get("box_remaining") == 0:
            box["status"] = "CLEARED"

    elif name == "UNLOAD":
        loc = robot["here"]
        for item in result.get("items", []):
            if not item.get("ok"):
                continue
            puck = world["pucks"][item["puck_id"]]
            if puck["slot"] is not None:
                robot["storage"][puck["slot"]] = empty_slot(puck["slot"])
            puck.update(where="placed", slot=None,
                        placed={"t": loc, "cell": item["cell"], "offset_mm": item.get("offset_mm")})
            world["locations"][loc]["cells"][item["cell"]] = {"cell": item["cell"], "state": "occupied",
                                                              "puck_id": item["puck_id"]}
        if loc in world["locations"] and any(i.get("ok") for i in result.get("items", [])):
            world["locations"][loc]["verified"] = None  # 새로 놓았으니 다시 검증해야 한다

    elif name == "VERIFY_LOCATION" and ok:
        world["locations"][robot["here"]]["verified"] = {
            "items": result.get("items", []), "extra": result.get("extra", []),
            "cells": result.get("cells", []), "stamp": stamp_now(t)}

    last = state["last_action"]
    same = last is not None and last["name"] == name and last["args"] == args
    retries = 0 if ok else (last["retries"] + 1 if same and not last["success"] else 1)
    state["last_action"] = {"name": name, "args": args, "success": ok,
                            "error": result.get("error"), "retries": retries, "result": result}
    state["phase"] = derive_phase(state)


def derive_phase(state):
    """남은 일 중 가장 앞 단계. 행동을 제한하지 않는다(LOAD 중에도 UNLOAD 가능)."""
    w, robot = state["world"], state["robot"]
    if not (w["barcode"] and w["barcode"]["valid"] and w["signal"] and w["signal"]["valid"]):
        return "READ_MISSION_INPUT"
    if any(b["status"] != "CLEARED" for b in w["boxes"].values()):
        return "LOAD"
    if any(s["state"] != "empty" for s in robot["storage"]):
        return "UNLOAD"
    if any(loc["verified"] is None and any(c["state"] == "occupied" for c in loc["cells"])
           for loc in w["locations"].values()):
        return "VERIFY_LOCATION"
    if not robot["home_stopped"]:
        return "GOTO"
    return "DONE"


def to_message(state):
    """LLM에 보낼 상태 문자열. sort_keys로 매번 같은 순서를 유지한다."""
    return json.dumps(state, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
