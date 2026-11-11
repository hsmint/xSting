"""로봇 명령(RobotCommand 액션)의 인자·결과 계약. 노트북(llm_commander)과 로봇(robot_command_server) 사이의 약속이다.

로봇 쪽 구현(가짜 경기장, 실물 _ros_xxx, 다른 팀 노드)은 이 형식대로 답해야 한다. ROS 없이 import된다.
  check_args(name, args)            → 틀린 곳 목록 (맞으면 [])
  check_result(name, args, result)  → 틀린 곳 목록 (맞으면 [])
목록에 없는 필드를 더 넣는 것은 괜찮다(나중에 필드를 늘릴 수 있게). 필수 필드가 빠지거나 값이 틀리면 안 된다.

공통 결과
  {"success": bool,                         필수
   "error": str,                            실패면 필수 (예: "NO_PATH", "GRASP_FAILED")
   "message": str,                          선택: 사람이 읽을 설명
   "robot": {                               선택이지만 되도록 매번 싣는다 (MissionState.robot에 그대로 들어감)
     "here": "HOME"|"S1"~"S6"|"L1"~"L3"|"MOVING"|"UNKNOWN",
     "pose": {"x": mm, "y": mm, "yaw": deg} 또는 null,   경기장 좌표 (원점 = 왼쪽 아래 기둥 중심, +Y 지도 위쪽)
     "status": "OK"|"BUSY"|"WARN"|"FAULT", "localization": "ok"|"degraded"|"lost",
     "arm_stowed": bool, "gripper": "empty"|"holding", "error": null 또는 {...}}}

행동별 (성공일 때 필요한 것)
  READ_MISSION_INPUT  인자 {}
      {"success": true, "barcode": "P2-01", "signal": ["R", "G", ...12개], "end_seen": true}
      signal 항목은 "R"|"Y"|"G", 못 읽은 칸은 "?" (개수·색 검사는 중계 노드가 한다)
  GOTO  인자 {"t": 장소 ID}
      {"success": true, "robot": {"here": t, "pose": {...}}}       robot.here는 t여야 한다
      t=HOME이면 추가로 "in_zone": bool, "stopped_s": 숫자 (출발 구역 안에서 정지한 시간)
      선택: "dist_to_target_mm"
  LOAD  인자 {}   (지금 있는 시작 박스의 퍽을 한 번에 모두 싣는다)
      {"success": true, "box_remaining": 0,
       "pucks": [{"slot": 0~11, "puck": {"id": 0~11|null, "color": "R"|"G"|null, "shape": "cylinder"|"box"|null,
                                          "conf": 0~1 (선택)}}, ...]}       slot·id는 겹치지 않는다
      실패해도 box_remaining을 알면 싣는다
  UNLOAD  인자 {"items": [{"puck_id": 0~11, "cell": 0~8}, ...]}
      {"success": true, "stopped_at": null,
       "items": [{"puck_id", "cell", "ok": true, "offset_mm": [x, y] (선택), "error": str (실패 항목)}, ...]}
      items는 성공·실패 모두 필수(실패면 거기까지 한 항목). 항목은 인자의 items에 있는 것이어야 한다.
      성공이면 인자의 모든 항목이 ok=true
  VERIFY_LOCATION  인자 {"items": [{"puck_id", "cell"}, ...]}
      {"success": true, "all_ok": bool (선택),
       "items": [{"puck_id", "cell", "ok": bool, "offset_mm": [x, y]|null (선택), "seen_puck_id": int|null (선택)}],
       "extra": [목록에 없는데 퍽이 있는 cell 번호], "cells": [9칸 관측 {"cell", "state": "empty"|"occupied"|"unknown",
                                                              "puck_id": int|null, ...}]}
      success는 "관측을 했다"는 뜻이다. 배치가 맞는지는 items[].ok와 extra로 알린다
"""
from numbers import Number

from .rules import NUM_CELLS, NUM_PUCKS, PLACES, PUCK_COLORS, SHAPES

COMMANDS = ("READ_MISSION_INPUT", "GOTO", "LOAD", "UNLOAD", "VERIFY_LOCATION")
HERE_VALUES = PLACES + ("MOVING", "UNKNOWN")
SIGNAL_VALUES = ("R", "Y", "G", "?")
STORAGE_SLOTS = 12


def _is_int(v):
    return isinstance(v, int) and not isinstance(v, bool)


def _is_num(v):
    return isinstance(v, Number) and not isinstance(v, bool)


def _check_items_arg(items, errs, where="args.items"):
    if not isinstance(items, list) or not items:
        errs.append(f"{where}는 1개 이상인 리스트")
        return
    for k, it in enumerate(items):
        if not isinstance(it, dict) or set(it) != {"puck_id", "cell"}:
            errs.append(f"{where}[{k}]는 {{puck_id, cell}}만 가진 객체")
        elif not (_is_int(it["puck_id"]) and 0 <= it["puck_id"] < NUM_PUCKS):
            errs.append(f"{where}[{k}].puck_id는 0~{NUM_PUCKS - 1}")
        elif not (_is_int(it["cell"]) and 0 <= it["cell"] < NUM_CELLS):
            errs.append(f"{where}[{k}].cell은 0~{NUM_CELLS - 1}")


def check_args(name, args):
    """행동 인자가 계약에 맞는지. 틀린 곳 목록 (맞으면 [])."""
    if name not in COMMANDS:
        return [f"알 수 없는 행동: {name} (가능: {', '.join(COMMANDS)})"]
    if not isinstance(args, dict):
        return ["args는 객체여야 함"]
    errs = []
    if name == "GOTO":
        if set(args) != {"t"} or args["t"] not in PLACES:
            errs.append(f"GOTO 인자는 {{t: {' | '.join(PLACES)}}}")
    elif name in ("UNLOAD", "VERIFY_LOCATION"):
        if set(args) != {"items"}:
            errs.append(f"{name} 인자는 {{items: [...]}}")
        else:
            _check_items_arg(args["items"], errs)
    elif args:
        errs.append(f"{name}은(는) 인자가 없음")
    return errs


def _check_robot(robot, errs):
    if not isinstance(robot, dict):
        errs.append("robot은 객체")
        return
    if "here" in robot and robot["here"] not in HERE_VALUES:
        errs.append(f"robot.here는 {' | '.join(HERE_VALUES)} (받은 값 {robot['here']!r})")
    pose = robot.get("pose")
    if pose is not None and not (isinstance(pose, dict) and all(_is_num(pose.get(k)) for k in ("x", "y", "yaw"))):
        errs.append("robot.pose는 null 또는 {x, y, yaw} 숫자")
    for key, allowed in (("status", ("OK", "BUSY", "WARN", "FAULT")), ("localization", ("ok", "degraded", "lost")),
                         ("gripper", ("empty", "holding"))):
        if key in robot and robot[key] not in allowed:
            errs.append(f"robot.{key}는 {' | '.join(allowed)}")
    if "arm_stowed" in robot and not isinstance(robot["arm_stowed"], bool):
        errs.append("robot.arm_stowed는 bool")


def _check_offset(it, where, errs):
    off = it.get("offset_mm")
    if off is not None and not (isinstance(off, list) and len(off) == 2 and all(_is_num(v) for v in off)):
        errs.append(f"{where}.offset_mm는 null 또는 [x, y] 숫자")


def _check_result_items(name, args, items, errs, need_ok_all):
    """UNLOAD·VERIFY_LOCATION 결과 items: 인자에 있는 항목이고, ok가 bool."""
    if not isinstance(items, list):
        errs.append("items는 리스트")
        return
    asked = {(i["puck_id"], i["cell"]) for i in args.get("items", []) if isinstance(i, dict)}
    for k, it in enumerate(items):
        where = f"items[{k}]"
        if not isinstance(it, dict):
            errs.append(f"{where}는 객체")
            continue
        if (it.get("puck_id"), it.get("cell")) not in asked:
            errs.append(f"{where}의 (puck_id, cell)=({it.get('puck_id')}, {it.get('cell')})가 인자의 items에 없음")
        if not isinstance(it.get("ok"), bool):
            errs.append(f"{where}.ok는 bool")
        _check_offset(it, where, errs)
    if need_ok_all:
        if len(items) != len(asked) or not all(isinstance(it, dict) and it.get("ok") is True for it in items):
            errs.append(f"성공이면 인자의 items {len(asked)}개가 모두 ok=true여야 함")


def check_result(name, args, result):
    """로봇이 돌려준 결과가 계약에 맞는지. 틀린 곳 목록 (맞으면 []). 인자는 맞다고 보고 결과만 본다."""
    if not isinstance(result, dict):
        return ["결과는 객체여야 함"]
    errs = []
    ok = result.get("success")
    if not isinstance(ok, bool):
        return ["success는 bool (필수)"]
    if not ok and not (isinstance(result.get("error"), str) and result["error"]):
        errs.append("실패면 error 문자열이 필요")
    if "robot" in result:
        _check_robot(result["robot"], errs)

    if name == "READ_MISSION_INPUT" and ok:
        if not (isinstance(result.get("barcode"), str) and result["barcode"]):
            errs.append("barcode는 문자열 (필수)")
        sig = result.get("signal")
        if not (isinstance(sig, list) and all(c in SIGNAL_VALUES for c in sig)):
            errs.append(f"signal은 {'|'.join(SIGNAL_VALUES)} 문자열 리스트 (필수)")
        if not isinstance(result.get("end_seen"), bool):
            errs.append("end_seen은 bool (필수)")

    elif name == "GOTO" and ok:
        here = (result.get("robot") or {}).get("here")
        if here != args.get("t"):
            errs.append(f"성공이면 robot.here가 목적지 {args.get('t')}여야 함 (받은 값 {here!r})")
        if args.get("t") == "HOME":
            if not isinstance(result.get("in_zone"), bool):
                errs.append("HOME이면 in_zone은 bool (필수)")
            if not _is_num(result.get("stopped_s")):
                errs.append("HOME이면 stopped_s는 숫자 (필수)")

    elif name == "LOAD":
        rem = result.get("box_remaining")
        if rem is not None and not (_is_int(rem) and rem >= 0):
            errs.append("box_remaining은 0 이상 정수")
        if ok:
            if rem is None:
                errs.append("성공이면 box_remaining이 필요")
            pucks = result.get("pucks")
            if not (isinstance(pucks, list) and pucks):
                errs.append("성공이면 pucks는 1개 이상인 리스트")
                pucks = []
            slots, ids = [], []
            for k, item in enumerate(pucks):
                where = f"pucks[{k}]"
                if not (isinstance(item, dict) and isinstance(item.get("puck"), dict)):
                    errs.append(f"{where}는 {{slot, puck: {{id, color, shape}}}}")
                    continue
                slot, puck = item.get("slot"), item["puck"]
                if not (_is_int(slot) and 0 <= slot < STORAGE_SLOTS):
                    errs.append(f"{where}.slot은 0~{STORAGE_SLOTS - 1}")
                slots.append(slot)
                pid = puck.get("id")
                if pid is not None and not (_is_int(pid) and 0 <= pid < NUM_PUCKS):
                    errs.append(f"{where}.puck.id는 0~{NUM_PUCKS - 1} 또는 null")
                if pid is not None:
                    ids.append(pid)
                if puck.get("color") not in PUCK_COLORS + (None,):
                    errs.append(f"{where}.puck.color는 {'|'.join(PUCK_COLORS)} 또는 null")
                if puck.get("shape") not in SHAPES + (None,):
                    errs.append(f"{where}.puck.shape는 {'|'.join(SHAPES)} 또는 null")
                conf = puck.get("conf")
                if conf is not None and not (_is_num(conf) and 0 <= conf <= 1):
                    errs.append(f"{where}.puck.conf는 0~1")
            if len(set(slots)) != len(slots):
                errs.append("pucks의 slot이 겹침")
            if len(set(ids)) != len(ids):
                errs.append("pucks의 puck.id가 겹침")

    elif name == "UNLOAD":
        if "items" not in result:
            errs.append("items가 필요 (실패해도 거기까지 한 항목)")
        else:
            _check_result_items(name, args, result["items"], errs, need_ok_all=ok)
        stopped = result.get("stopped_at")
        if stopped is not None and not _is_int(stopped):
            errs.append("stopped_at은 정수 또는 null")

    elif name == "VERIFY_LOCATION" and ok:
        if "items" not in result:
            errs.append("성공이면 items가 필요")
        else:
            _check_result_items(name, args, result["items"], errs, need_ok_all=False)
            for k, it in enumerate(result["items"] if isinstance(result["items"], list) else []):
                seen = it.get("seen_puck_id") if isinstance(it, dict) else None
                if seen is not None and not _is_int(seen):
                    errs.append(f"items[{k}].seen_puck_id는 정수 또는 null")
        extra = result.get("extra")
        if not (isinstance(extra, list) and all(_is_int(c) and 0 <= c < NUM_CELLS for c in extra)):
            errs.append(f"성공이면 extra는 cell 번호(0~{NUM_CELLS - 1}) 리스트 (없으면 [])")
        cells = result.get("cells")
        if not (isinstance(cells, list) and len(cells) == NUM_CELLS and all(
                isinstance(c, dict) and c.get("cell") == k and c.get("state") in ("empty", "occupied", "unknown")
                for k, c in enumerate(cells))):
            errs.append(f"성공이면 cells는 0~{NUM_CELLS - 1}번 순서의 {NUM_CELLS}칸 관측 "
                        "{cell, state: empty|occupied|unknown, puck_id}")
    return errs
