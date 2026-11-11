"""지난 판(run) 다시 실행: 기록 폴더에서 문제와 판단 순서를 읽는다.

  문제   problem.json이 있으면 그대로, 없으면 final_state.json에서 복원한다
         (바코드·신호, 시작 박스별로 LOAD한 순서와 퍽 색·모양). 끝까지 못 실은 퍽은 알 수 없어서 임시값으로 채운다.
  판단   decisions.json이 있으면 그대로(보낸 상태·생각 요약·결과 포함), 없으면 mission.jsonl에서 복원한다.
  실패   그때 났던 실패(웹·시나리오에서 주입한 실패, 예전 기록의 LOAD 집기 실패)는 같은 시점에 다시 나도록 faults에 넣는다.
  상태   보낸 상태(input_state)가 없는 예전 판은 기록된 로봇 결과를 중계 노드와 같은 계산(apply_result)으로
         처음부터 다시 적용해 판단마다 보낸 상태를 복원한다.
  누가   판단마다 decider: "ai"(ChatGPT) | "rule"(예전 mode:=rule 시험) | "user"(웹 화면에서 직접 보낸 명령).
         예전 기록에 없으면 LLM 응답 기록이 있으면 ai, 없으면 rule로 본다.
  모델   판단 기록에 model이 없으면(예전 판) meta.json(중계 노드가 시작할 때 남김)의 모델로 채운다.
  예전   예전 행동 이름(read_barcode, pick 등)을 쓴 규칙 판은 지금 로봇에 다시 실행할 수 없어 보기만 된다.
"""
import copy
import json
import os
import shutil
from pathlib import Path

from .rules import NUM_PUCKS, START_BOXES
from .state import apply_result, new_state
from .tools import TOOL_NAMES

# 로봇에 보내지 않은 판단 (가드 거절, 알 수 없는 도구). 다시 실행할 때도 보내지 않는다
NOT_SENT = ("REJECTED", "UNKNOWN_TOOL")


def list_runs(log_dir):
    runs = []
    for d in sorted(Path(log_dir).glob("run_*"), reverse=True):
        if not ((d / "decisions.json").exists() or (d / "mission.jsonl").exists()):
            continue
        name = (d / "name.txt").read_text(encoding="utf-8").strip() if (d / "name.txt").exists() else ""
        info = {"run_id": d.name[4:], "name": name, "dir": str(d), "has_problem": (d / "problem.json").exists(),
                "full": (d / "decisions.json").exists(), "phase": None, "halt": None, "turns": None, "model": None,
                "deciders": {}, "legacy": False}
        try:
            fs = json.loads((d / "final_state.json").read_text(encoding="utf-8"))
            info.update(phase=fs["state"]["phase"], halt=fs.get("halt"))
        except (OSError, ValueError, KeyError):
            pass
        try:
            recs = _read_records(d, info["run_id"])
            counts = {}
            for r in recs:
                counts[r["decider"]] = counts.get(r["decider"], 0) + 1
            info.update(turns=len(recs), deciders=counts, legacy=_is_legacy(recs),
                        model=next((r.get("model") for r in recs if r.get("model")), None))
        except (OSError, ValueError, KeyError):
            pass
        runs.append(info)
    return runs


def _records_from_jsonl(run_dir, run_id):
    """decisions.json이 없는 예전 판: mission.jsonl의 llm/tool 줄은 turn별로 합치고, user 줄은 따로 순서대로 넣는다."""
    order, by_turn = [], {}
    for line in (run_dir / "mission.jsonl").read_text(encoding="utf-8").splitlines():
        e = json.loads(line)
        if e["kind"] == "user":
            d = e["data"]
            order.append({"run_id": run_id, "turn": e["turn"], "t": e["t"], "decider": "user",
                          "name": d["name"], "args": d["args"],
                          "outcome": {"success": bool(d["result"].get("success")), "error": d["result"].get("error"),
                                      "result": d["result"], "phase_after": d.get("phase")}})
            continue
        if e["kind"] not in ("llm", "tool"):
            continue
        if e["turn"] not in by_turn:
            by_turn[e["turn"]] = {"run_id": run_id, "turn": e["turn"], "t": e["t"]}
            order.append(by_turn[e["turn"]])
        rec = by_turn[e["turn"]]
        if e["kind"] == "llm":
            rec.update({k: e["data"].get(k) for k in ("reasoning", "text", "usage", "raw_output", "latency_s",
                                                      "name", "arguments", "status")})
        else:
            d = e["data"]
            rec.update(name=d["name"], args=d["args"],
                       outcome={"success": bool(d["result"].get("success")), "error": d["result"].get("error"),
                                "result": d["result"], "phase_after": d.get("phase")})
    return [r for r in order if r.get("name")]


def _infer_decider(r):
    if r.get("decider"):
        return r["decider"]
    llm_fields = ("usage", "raw_output", "latency_s", "status", "reasoning")
    return "ai" if any(r.get(k) is not None for k in llm_fields) else "rule"


def _is_legacy(records):
    """예전 행동 이름(read_barcode, pick 등)을 쓴 판인지."""
    return any(r.get("name") and r["name"] not in TOOL_NAMES for r in records)


def _read_records(run_dir, run_id):
    if (run_dir / "decisions.json").exists():
        records = json.loads((run_dir / "decisions.json").read_text(encoding="utf-8"))
    else:
        records = _records_from_jsonl(run_dir, run_id)
    try:
        meta = json.loads((run_dir / "meta.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        meta = {}
    for r in records:
        r["decider"] = _infer_decider(r)
        if r["decider"] == "ai" and not r.get("model"):
            r.update(model=meta.get("model"), effort=r.get("effort") or meta.get("effort"))
    return records


def _problem_from_final_state(run_dir, records):
    fs = json.loads((run_dir / "final_state.json").read_text(encoding="utf-8"))["state"]
    w = fs["world"]
    pucks = {p["id"]: p for p in w["pucks"]}
    unknown = [i for i in range(NUM_PUCKS) if pucks[i]["color"] is None]
    boxes = {}
    for box in START_BOXES:
        slots = [{"id": i, "color": pucks[i]["color"], "shape": pucks[i]["shape"]} for i in w["boxes"][box]["loaded"]]
        while len(slots) < 2 and unknown:  # 못 실은 퍽: 알 수 없으므로 임시값
            slots.append({"id": unknown.pop(0), "color": "G", "shape": "box"})
        boxes[box] = slots
    return {
        "barcode": (w["barcode"] or {}).get("code") or "P2-01",
        "signal": "".join((w["signal"] or {}).get("sequence") or []) or "RGRRGRGGYGYG",
        "boxes": boxes,
        "reconstructed": True,
    }


def _faults(records):
    """그 판에서 났던 실패를 같은 순서로 다시 내기 위한 faults 목록 (가짜 경기장의 실패 주입 형식).

    - 결과에 injected가 있으면 그대로 쓴다 (웹·시나리오에서 주입한 실패).
    - 인식 실패 뒤 한 바퀴 탐색(perception_search)이 있었으면, 무작위였든 주입이었든 그때 찾았는지대로
      "처음 실패 + 찾을 확률 100% 또는 0%"로 다시 건다. 다시 실행 때는 무작위 인식 실패를 끄므로 결과가 같아진다.
    - 그 전의 기록은 LOAD 집기 실패(GRASP_FAILED)만 그 박스의 실패로 되살린다.
    """
    faults, here = [], "HOME"
    for r in records:
        if r.get("input_state"):
            here = r["input_state"]["robot"]["here"]
        o = r.get("outcome") or {}
        result = o.get("result") or {}
        # 규약 위반으로 BAD_RESULT가 됐으면 주입 정보와 탐색 기록은 원래 결과(raw_result) 안에 있다
        raw = result.get("raw_result") or {}
        injected = result.get("injected") or raw.get("injected")
        search = result.get("perception_search") or raw.get("perception_search")
        name = r.get("name")
        if search and injected and injected.get("code") != "PERCEPTION_FAILED":
            # 같은 행동에서 인식 탐색과 다른 실패(예: 집기 실패)가 함께 났다: 한 실패로 합쳐 같은 행동에 걸리게 한다
            faults.append({**injected, "perception_option": 100 if search.get("found") else 0, "times": 1})
        elif search:
            faults.append({"action": name, "code": "PERCEPTION_FAILED",
                           "target": None if name == "READ_MISSION_INPUT" else here,
                           "option": 100 if search.get("found") else 0, "times": 1})
        elif injected:
            faults.append({**injected, "times": 1})
        elif name == "LOAD" and o.get("error") == "GRASP_FAILED":
            faults.append({"action": "LOAD", "code": "GRASP_FAILED", "target": here, "option": 0, "times": 1})
        if name == "GOTO" and o.get("success"):
            here = r["args"]["t"]
    return faults


def _rebuild_states(records):
    st = new_state()
    for r in records:
        if not r.get("input_state"):
            snap = copy.deepcopy(st)
            snap["meta"] = {"seq": r["turn"], "stamp": None}
            r["input_state"], r["input_state_rebuilt"] = snap, True
        if r.get("args") is None and r.get("arguments"):
            try:
                r["args"] = json.loads(r["arguments"])
            except ValueError:
                pass
        o = r.get("outcome")
        if o and r.get("name") and "result" in o:
            # 결과를 받은 시각 ≈ 다음 판단 기록 시각. 검증 시각(verified.stamp)을 그때 값으로 맞춘다
            nxt = next((x["t"] for x in records if x["turn"] > r["turn"] and x.get("t")), None)
            apply_result(st, r["name"], r.get("args") or {}, o["result"], t=nxt or r.get("t"))
    return records


def _run_dir(log_dir, run_id):
    """run_id로 판 폴더를 찾는다. 경로를 벗어나는 id는 거절한다."""
    run_dir = Path(log_dir) / f"run_{run_id}"
    if not run_id or os.path.sep in run_id or ".." in run_id or not run_dir.is_dir():
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


def load_run(log_dir, run_id):
    run_dir = _run_dir(log_dir, run_id)
    records = _read_records(run_dir, run_id)
    legacy = _is_legacy(records)
    if legacy:  # 예전 상태 형식이라 지금 계산으로 복원하면 틀린 값이 나온다. 보기만 한다
        return {"legacy": True}, records
    if (run_dir / "problem.json").exists():
        problem = json.loads((run_dir / "problem.json").read_text(encoding="utf-8"))
    else:
        problem = _problem_from_final_state(run_dir, records)
    records = _rebuild_states(records)
    problem["faults"] = _faults(records)
    # 인식 실패는 위 faults로 그대로 재현하므로, 다시 실행 중에는 무작위 인식 실패를 끈다
    problem["perception"] = {"fail_rate": 0, "search_success_rate": 0}
    return problem, records
