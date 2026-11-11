"""중계 노드가 들고 있는 미션 상태(로봇이 돌려준 결과 기록)와 명령 가드."""
import json
from datetime import datetime, timezone

from . import rules
from .planner import PlanError, group_order

ROBOT_FIELDS = ("status", "error", "here", "pose", "localization",
                "arm_stowed", "gripper")

# 행동별로 robot.here가 있어야 하는 곳. GOTO는 어디서든 가능하다
LOCATION_CONDITION = {
    "READ_MISSION_INPUT": (rules.HOME,),
    "LOAD": rules.START_BOXES,
    "UNLOAD": rules.LOCATIONS,
    "VERIFY_LOCATION": rules.LOCATIONS,
}


class ToolError(Exception):
    """가드가 로봇에 보내기 전에 거절한 명령. 메시지는 LLM에 그대로 간다."""


# ---------- 상태 만들기 ----------

def _empty_slot(slot):
    return {"slot": slot, "state": "empty", "puck_id": None, "color": None,
            "shape": None}


def _empty_cell(cell):
    return {"cell": cell, "state": "empty", "puck_id": None}


def new_state():
    """빈 미션 상태. 키 순서는 LLM에 가는 순서라 바꾸지 않는다."""
    return {
        "meta": {"seq": 0, "stamp": None},
        "phase": "READ_MISSION_INPUT",
        "robot": {
            "status": "OK",  # OK | BUSY | WARN | FAULT
            "error": None,  # {"code", "message"}
            "here": rules.HOME,  # HOME | S1~S6 | L1~L3 | MOVING | UNKNOWN
            "pose": None,  # {"x", "y" (mm), "yaw" (deg)}
            "localization": "ok",  # ok | degraded | lost
            "arm_stowed": True,
            "gripper": "empty",  # empty | holding
            "home_stopped": False,  # HOME 3초 정지 완료 (다른 곳 GOTO면 false)
            "storage": [_empty_slot(slot)
                        for slot in range(rules.STORAGE_SLOTS)],
        },
        "world": {
            "barcode": None,  # {"code", "order", "valid"}
            "signal": None,  # {"sequence", "counts", "valid", "errors"}
            "boxes": {box: {"status": "UNVISITED", "loaded": []}
                      for box in rules.START_BOXES},
            "pucks": [
                {"id": puck_id, "color": None, "shape": None, "conf": None,
                 "origin": None, "where": "unknown", "slot": None,
                 "placed": None}
                for puck_id in range(rules.NUM_PUCKS)
            ],
            "locations": {
                location: {"cells": [_empty_cell(cell)
                                     for cell in range(rules.NUM_CELLS)],
                           "verified": None}
                for location in rules.LOCATIONS
            },
        },
        "last_action": None,
        # LLM이 직전에 적은 계획. 대화 기록이 없어 계획은 여기로 이어진다
        "memory": {"plan": None},
    }


def stamp_now(t=None):
    """UTC 시각 문자열. t(유닉스 시각)를 주면 그 시각으로(지난 판 복원용)."""
    moment = (datetime.now(timezone.utc) if t is None
              else datetime.fromtimestamp(t, timezone.utc))
    return moment.strftime("%Y-%m-%dT%H:%M:%SZ")


# ---------- 가드 ----------

def check_location(state, name):
    """위치 조건을 만족하지 않으면 로봇에 보내기 전에 거절한다."""
    allowed = LOCATION_CONDITION.get(name)
    here = state["robot"]["here"]
    if allowed and here not in allowed:
        raise ToolError(f"LOCATION_CONDITION: {name}은(는) "
                        f"{'/'.join(allowed)}에서만 가능, 지금 here={here}")


def check_unload(state, args):
    """UNLOAD는 그 하차 박스에 남은 퍽을 맞는 cell에 한 번에 모두 놓아야 한다."""
    location = state["robot"]["here"]
    if location not in rules.LOCATIONS:
        return  # 위치 조건은 check_location이 본다

    try:
        color = group_order(state)[location]
        need = state["world"]["signal"]["counts"][color]
    except PlanError as error:
        raise ToolError(f"PARTIAL_UNLOAD: {error}. "
                        "바코드·신호를 읽은 뒤 UNLOAD한다")

    cells_now = state["world"]["locations"][location]["cells"]
    placed = sum(cell["state"] == "occupied" for cell in cells_now)
    items = args.get("items", [])

    # 눈 개수의 cell 목록 안에서, 겹치지 않게, 비어 있는 칸에만 놓는다
    allowed = rules.DICE_CELLS.get(need, [])
    cells = [item.get("cell") for item in items]
    used = [cell["cell"] for cell in cells_now if cell["state"] == "occupied"]
    if (any(cell not in allowed for cell in cells)
            or len(set(cells)) != len(cells)
            or any(cell in used for cell in cells)):
        raise ToolError(
            f"WRONG_CELLS: {location}는 눈 {need}개라 cell {allowed}에 하나씩 "
            f"놓는다 (받은 cell {cells}"
            + (f", 이미 놓은 cell {used}" if used else "") + ")")

    if placed + len(items) != need:
        raise ToolError(
            f"PARTIAL_UNLOAD: {location}에는 퍽 {need}개를 한 번에 놓아야 한다 "
            f"(이미 놓은 것 {placed}개 + 이번 items {len(items)}개). "
            "그 하차 박스에 놓을 퍽이 모두 적재함에 있을 때 UNLOAD한다")


# ---------- 결과 반영 ----------

def apply_result(state, name, args, result, t=None):
    """로봇이 돌려준 행동 결과를 상태에 기록한다. t는 지난 판 복원 때의 수신 시각."""
    ok = bool(result.get("success"))
    robot, world = state["robot"], state["world"]

    for key in ROBOT_FIELDS:
        if key in result.get("robot", {}):
            robot[key] = result["robot"][key]

    if name == "READ_MISSION_INPUT" and ok:
        code = result.get("barcode")
        order = rules.BARCODE_ORDER.get(code)
        world["barcode"] = {"code": code, "order": order,
                            "valid": order is not None}
        sequence = list(result.get("signal", []))
        errors = rules.signal_errors(sequence, result.get("end_seen", False))
        world["signal"] = {"sequence": sequence,
                           "counts": rules.count_signal(sequence),
                           "valid": not errors, "errors": errors}

    elif name == "GOTO":
        robot["home_stopped"] = (ok and args["t"] == rules.HOME
                                 and result.get("in_zone", False)
                                 and result.get("stopped_s", 0) >= 3.0)
        if (ok and args["t"] in world["boxes"]
                and world["boxes"][args["t"]]["status"] == "UNVISITED"):
            world["boxes"][args["t"]]["status"] = "VISITED"

    elif name == "LOAD":
        _apply_load(robot, world, result, ok)

    elif name == "UNLOAD":
        _apply_unload(robot, world, result)

    elif name == "VERIFY_LOCATION" and ok:
        world["locations"][robot["here"]]["verified"] = {
            "items": result.get("items", []), "extra": result.get("extra", []),
            "cells": result.get("cells", []), "stamp": stamp_now(t)}

    last = state["last_action"]
    same = (last is not None and last["name"] == name
            and last["args"] == args)
    if ok:
        retries = 0
    else:
        retries = last["retries"] + 1 if same and not last["success"] else 1
    state["last_action"] = {"name": name, "args": args, "success": ok,
                            "error": result.get("error"), "retries": retries,
                            "result": result}
    state["phase"] = derive_phase(state)


def _apply_load(robot, world, result, ok):
    """LOAD 결과 반영. 실패여도 실은 퍽은 기록한다(집기 실패여도 실었을 수 있다)."""
    box = world["boxes"].get(robot["here"])
    # 예전 기록은 성공일 때만 {"slot", "puck"} 하나짜리 형식이다
    loaded = result.get("pucks") or (
        [{"slot": result["slot"], "puck": result["puck"]}]
        if ok and "puck" in result else [])
    for item in loaded:
        puck, slot = item["puck"], item["slot"]
        puck_id = puck.get("id")
        robot["storage"][slot] = {
            "slot": slot,
            "state": "occupied" if puck_id is not None else "unknown",
            "puck_id": puck_id, "color": puck.get("color"),
            "shape": puck.get("shape")}
        if puck_id is not None:
            world["pucks"][puck_id].update(
                color=puck.get("color"), shape=puck.get("shape"),
                conf=puck.get("conf"), origin=robot["here"],
                where="storage", slot=slot)
            if box is not None:
                box["loaded"].append(puck_id)
    if box is not None and result.get("box_remaining") == 0:
        box["status"] = "CLEARED"


def _apply_unload(robot, world, result):
    """UNLOAD 결과 반영. 성공한 항목만 적재함에서 하차 박스 cell로 옮긴다."""
    location = robot["here"]
    for item in result.get("items", []):
        if not item.get("ok"):
            continue
        puck = world["pucks"][item["puck_id"]]
        if puck["slot"] is not None:
            robot["storage"][puck["slot"]] = _empty_slot(puck["slot"])
        puck.update(where="placed", slot=None,
                    placed={"t": location, "cell": item["cell"],
                            "offset_mm": item.get("offset_mm")})
        world["locations"][location]["cells"][item["cell"]] = {
            "cell": item["cell"], "state": "occupied",
            "puck_id": item["puck_id"]}
    if (location in world["locations"]
            and any(item.get("ok") for item in result.get("items", []))):
        world["locations"][location]["verified"] = None  # 다시 검증해야 한다


# ---------- 단계·메시지 ----------

def derive_phase(state):
    """남은 일 중 가장 앞 단계. 행동을 제한하지 않는다(LOAD 중에도 UNLOAD 가능)."""
    world, robot = state["world"], state["robot"]
    if not (world["barcode"] and world["barcode"]["valid"]
            and world["signal"] and world["signal"]["valid"]):
        return "READ_MISSION_INPUT"
    if any(box["status"] != "CLEARED" for box in world["boxes"].values()):
        return "LOAD"
    if any(slot["state"] != "empty" for slot in robot["storage"]):
        return "UNLOAD"
    if any(location["verified"] is None
           and any(cell["state"] == "occupied" for cell in location["cells"])
           for location in world["locations"].values()):
        return "VERIFY_LOCATION"
    if not robot["home_stopped"]:
        return "GOTO"
    return "DONE"


def to_message(state):
    """LLM에 보낼 상태 문자열. sort_keys로 매번 같은 순서를 유지한다."""
    return json.dumps(state, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"))
