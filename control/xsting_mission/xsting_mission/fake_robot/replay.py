"""지난 판(run) 다시 실행: 기록 폴더에서 문제·판단 순서·실패를 복원한다."""
import copy
import json
import os
import shutil
from pathlib import Path

from ..core.contract import COMMANDS
from ..core.rules import NUM_PUCKS, START_BOXES
from ..core.state import apply_result, new_state

# 로봇에 보내지 않은 판단 (가드 거절, 알 수 없는 도구). 다시 실행할 때도 보내지 않는다
NOT_SENT = ("REJECTED", "UNKNOWN_TOOL")


# ---------- 판 목록·관리 ----------
def list_runs(log_dir):
    runs = []
    for run_dir in sorted(Path(log_dir).glob("run_*"), reverse=True):
        if not ((run_dir / "decisions.json").exists()
                or (run_dir / "mission.jsonl").exists()):
            continue
        name_path = run_dir / "name.txt"
        name = (name_path.read_text(encoding="utf-8").strip()
                if name_path.exists() else "")
        info = {"run_id": run_dir.name[4:], "name": name, "dir": str(run_dir),
                "has_problem": (run_dir / "problem.json").exists(),
                "full": (run_dir / "decisions.json").exists(),
                "phase": None, "halt": None, "turns": None, "model": None,
                "deciders": {}, "legacy": False}
        try:
            final = json.loads(
                (run_dir / "final_state.json").read_text(encoding="utf-8"))
            info.update(phase=final["state"]["phase"], halt=final.get("halt"))
        except (OSError, ValueError, KeyError):
            pass
        try:
            records = _read_records(run_dir, info["run_id"])
            counts = {}
            for record in records:
                counts[record["decider"]] = counts.get(record["decider"], 0) + 1
            model = next((record.get("model") for record in records
                          if record.get("model")), None)
            info.update(turns=len(records), deciders=counts,
                        legacy=_is_legacy(records), model=model)
        except (OSError, ValueError, KeyError):
            pass
        runs.append(info)
    return runs


def _run_dir(log_dir, run_id):
    """run_id로 판 폴더를 찾는다. 경로를 벗어나는 id는 거절한다."""
    run_dir = Path(log_dir) / f"run_{run_id}"
    if (not run_id or os.path.sep in run_id or ".." in run_id
            or not run_dir.is_dir()):
        raise FileNotFoundError(run_id)
    return run_dir


def set_run_name(log_dir, run_id, name):
    """판 이름을 name.txt에 저장한다. 빈 이름이면 지운다."""
    path = _run_dir(log_dir, run_id) / "name.txt"
    name = " ".join(str(name).split())[:60]
    if name:
        path.write_text(name, encoding="utf-8")
    elif path.exists():
        path.unlink()
    return name


def delete_run(log_dir, run_id):
    """판 폴더를 통째로 지운다 (되돌릴 수 없음)."""
    shutil.rmtree(_run_dir(log_dir, run_id))


# ---------- 판단 기록 읽기 ----------
def _outcome(data):
    """mission.jsonl의 행동 결과를 decisions.json의 outcome 형식으로."""
    return {"success": bool(data["result"].get("success")),
            "error": data["result"].get("error"),
            "result": data["result"], "phase_after": data.get("phase")}


def _records_from_jsonl(run_dir, run_id):
    """decisions.json이 없는 예전 판: llm/tool 줄은 turn별로 합치고 user 줄은 따로 넣는다."""
    order, by_turn = [], {}
    for line in (run_dir / "mission.jsonl").read_text(
            encoding="utf-8").splitlines():
        entry = json.loads(line)
        data = entry["data"] if entry["kind"] in ("user", "tool") else None
        if entry["kind"] == "user":
            order.append({"run_id": run_id, "turn": entry["turn"],
                          "t": entry["t"], "decider": "user",
                          "name": data["name"], "args": data["args"],
                          "outcome": _outcome(data)})
            continue
        if entry["kind"] not in ("llm", "tool"):
            continue

        if entry["turn"] not in by_turn:
            by_turn[entry["turn"]] = {"run_id": run_id, "turn": entry["turn"],
                                      "t": entry["t"]}
            order.append(by_turn[entry["turn"]])
        record = by_turn[entry["turn"]]
        if entry["kind"] == "llm":
            record.update({key: entry["data"].get(key) for key in (
                "reasoning", "text", "usage", "raw_output", "latency_s",
                "name", "arguments", "status")})
        else:
            record.update(name=data["name"], args=data["args"],
                          outcome=_outcome(data))
    return [record for record in order if record.get("name")]


def _infer_decider(record):
    """decider가 없는 예전 기록: LLM 응답 기록이 있으면 ai, 없으면 rule."""
    if record.get("decider"):
        return record["decider"]
    llm_fields = ("usage", "raw_output", "latency_s", "status", "reasoning")
    return ("ai" if any(record.get(key) is not None for key in llm_fields)
            else "rule")


def _is_legacy(records):
    """예전 행동 이름(read_barcode, pick 등)을 쓴 판인지 (다시 실행은 못 하고 보기만)."""
    return any(record.get("name") and record["name"] not in COMMANDS
               for record in records)


def _read_records(run_dir, run_id):
    if (run_dir / "decisions.json").exists():
        records = json.loads(
            (run_dir / "decisions.json").read_text(encoding="utf-8"))
    else:
        records = _records_from_jsonl(run_dir, run_id)
    try:
        meta = json.loads((run_dir / "meta.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        meta = {}

    # model이 없는 예전 판은 meta.json(중계 노드 시작 때 남김)의 모델로 채운다
    for record in records:
        record["decider"] = _infer_decider(record)
        if record["decider"] == "ai" and not record.get("model"):
            record.update(model=meta.get("model"),
                          effort=record.get("effort") or meta.get("effort"))
    return records


# ---------- 다시 실행 복원 ----------
def _problem_from_final_state(run_dir):
    """problem.json이 없는 예전 판: final_state.json의 바코드·신호·LOAD 순서로 문제를 만든다."""
    world = json.loads((run_dir / "final_state.json").read_text(
        encoding="utf-8"))["state"]["world"]
    pucks = {puck["id"]: puck for puck in world["pucks"]}
    unknown = [i for i in range(NUM_PUCKS) if pucks[i]["color"] is None]
    boxes = {}
    for box in START_BOXES:
        slots = [{"id": i, "color": pucks[i]["color"],
                  "shape": pucks[i]["shape"]}
                 for i in world["boxes"][box]["loaded"]]
        while len(slots) < 2 and unknown:  # 못 실은 퍽은 알 수 없어 임시값
            slots.append({"id": unknown.pop(0), "color": "G", "shape": "box"})
        boxes[box] = slots
    return {
        "barcode": (world["barcode"] or {}).get("code") or "P2-01",
        "signal": ("".join((world["signal"] or {}).get("sequence") or [])
                   or "RGRRGRGGYGYG"),
        "boxes": boxes,
        "reconstructed": True,
    }


def _faults(records):
    """그 판에서 났던 실패를 같은 순서로 다시 내기 위한 faults 목록 (fake_world 형식)."""
    # 주입 실패(injected)는 그대로, 인식 탐색은 그때 찾았는지대로 확률 100/0%로,
    # 그 외는 LOAD 집기 실패(GRASP_FAILED)만 되살린다
    faults, here = [], "HOME"
    for record in records:
        if record.get("input_state"):
            here = record["input_state"]["robot"]["here"]
        outcome = record.get("outcome") or {}
        result = outcome.get("result") or {}
        # BAD_RESULT가 됐으면 주입 정보와 탐색 기록은 원래 결과(raw_result) 안에 있다
        raw = result.get("raw_result") or {}
        injected = result.get("injected") or raw.get("injected")
        search = result.get("perception_search") or raw.get("perception_search")
        name = record.get("name")
        if search and injected and injected.get("code") != "PERCEPTION_FAILED":
            # 인식 탐색과 다른 실패가 한 행동에서 함께 났다: 한 실패로 합친다
            option = 100 if search.get("found") else 0
            faults.append({**injected, "perception_option": option,
                           "times": 1})
        elif search:
            faults.append({"action": name, "code": "PERCEPTION_FAILED",
                           "target": (None if name == "READ_MISSION_INPUT"
                                      else here),
                           "option": 100 if search.get("found") else 0,
                           "times": 1})
        elif injected:
            faults.append({**injected, "times": 1})
        elif name == "LOAD" and outcome.get("error") == "GRASP_FAILED":
            faults.append({"action": "LOAD", "code": "GRASP_FAILED",
                           "target": here, "option": 0, "times": 1})
        if name == "GOTO" and outcome.get("success"):
            here = record["args"]["t"]
    return faults


def _rebuild_states(records):
    """보낸 상태가 없는 예전 판: 기록된 결과를 apply_result로 다시 적용해 판단마다 상태를 채운다."""
    state = new_state()
    for record in records:
        if not record.get("input_state"):
            snapshot = copy.deepcopy(state)
            snapshot["meta"] = {"seq": record["turn"], "stamp": None}
            record["input_state"] = snapshot
            record["input_state_rebuilt"] = True
        if record.get("args") is None and record.get("arguments"):
            try:
                record["args"] = json.loads(record["arguments"])
            except ValueError:
                pass
        outcome = record.get("outcome")
        if outcome and record.get("name") and "result" in outcome:
            # 결과를 받은 시각 ≈ 다음 판단 기록 시각. verified.stamp를 그때 값으로 맞춘다
            next_t = next((other["t"] for other in records
                           if other["turn"] > record["turn"]
                           and other.get("t")), None)
            apply_result(state, record["name"], record.get("args") or {},
                         outcome["result"], t=next_t or record.get("t"))
    return records


def load_run(log_dir, run_id):
    run_dir = _run_dir(log_dir, run_id)
    records = _read_records(run_dir, run_id)
    if _is_legacy(records):  # 예전 상태 형식이라 지금 계산으로 복원하면 틀린다
        return {"legacy": True}, records

    if (run_dir / "problem.json").exists():
        problem = json.loads(
            (run_dir / "problem.json").read_text(encoding="utf-8"))
    else:
        problem = _problem_from_final_state(run_dir)
    records = _rebuild_states(records)
    problem["faults"] = _faults(records)
    # 인식 실패는 위 faults로 재현하므로 다시 실행 중에는 무작위 인식 실패를 끈다
    problem["perception"] = {"fail_rate": 0, "search_success_rate": 0}
    return problem, records
