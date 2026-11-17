"""이동 중 미리 판단: 행동이 성공했다고 가정한 다음 판단을 백그라운드에서 받는다."""
import copy
import json
import threading
import time

from ..core.state import apply_result, stamp_now


class Prefetcher:
    """미리 판단 하나(box)를 받아 두었다가 가정이 맞으면 내준다."""

    def __init__(self, llm_decide, publish_decision, log):
        self.llm_decide = llm_decide  # LlmDecider.llm_decide
        self.publish_decision = publish_decision  # 노드의 판단 카드 발행
        self.log = log  # MissionLog.log
        self.box = None  # 미리 받는 중이거나 받은 다음 판단

    def start_prefetch(self, state, turn, name, args):
        """행동이 성공했다고 가정한 상태로 다음 판단을 백그라운드에서 미리 받는다."""
        # 결과가 판단 재료인 READ_MISSION_INPUT, LOAD는 미리 판단하지 않는다
        self.box = None
        here = state["robot"]["here"]
        if name == "GOTO" and args.get("t") != "HOME":
            # 도착 좌표는 모르므로 pose는 비운다. 마지막 HOME 복귀는 하지 않는다
            assumed = {"success": True,
                       "robot": {"here": args["t"], "pose": None}}
        elif name == "UNLOAD":  # 보낸 items가 모두 놓였다
            assumed = {"success": True, "stopped_at": None,
                       "robot": {"here": here},
                       "items": [{**item, "ok": True, "offset_mm": [0.0, 0.0]}
                                 for item in args["items"]]}
        elif (name == "VERIFY_LOCATION"
              and here in state["world"]["locations"]):
            # 보낸 items가 모두 일치하고 남는 퍽이 없다
            pucks = state["world"]["pucks"]
            cells = []
            for cell in state["world"]["locations"][here]["cells"]:
                puck_id = cell["puck_id"]
                puck = pucks[puck_id] if puck_id is not None else None
                cells.append({
                    "cell": cell["cell"], "state": cell["state"],
                    "puck_id": puck_id,
                    "color": puck["color"] if puck is not None else None,
                    "shape": puck["shape"] if puck is not None else None,
                    "offset_mm": None})
            assumed = {"success": True, "all_ok": True, "extra": [],
                       "cells": cells, "robot": {"here": here},
                       "items": [{**item, "ok": True,
                                  "seen_puck_id": item["puck_id"]}
                                 for item in args["items"]]}
        else:
            return
        predicted = copy.deepcopy(state)
        apply_result(predicted, name, args, assumed)
        predicted["meta"] = {**state["meta"], "seq": turn + 1,
                             "stamp": stamp_now()}
        if name == "GOTO":
            assume = f"{name} {args['t']}"
        else:
            assume = f"{name} {here} ({len(args['items'])}개)"
        box = {"turn": turn + 1, "name": name, "args": args,
               "assume": f"{assume} 성공 가정", "done": threading.Event(),
               "lock": threading.Lock(), "discarded": False, "result": None,
               "error": None}

        def work():
            try:
                box["result"] = self.llm_decide(predicted, quiet=True)
                answer_name, answer_args, _, record = box["result"]
                answer = (f"{answer_name} "
                          f"{json.dumps(answer_args, ensure_ascii=False)}")
                with box["lock"]:  # 이미 버렸으면 "받음" 카드를 띄우지 않는다
                    if not box["discarded"]:
                        self._prefetch_card(box, "answered", answer=answer,
                                            latency_s=record["latency_s"])
            # 미리 판단이 안 되면 결과가 나온 뒤 평소처럼 판단한다
            except Exception as error:
                box["error"] = repr(error)
            finally:
                box["done"].set()

        self.box = box
        self._prefetch_card(box, "requested")
        threading.Thread(target=work, daemon=True).start()
        self.log("prefetch", {"start": True, "turn": box["turn"],
                              "assume": box["assume"]})

    def _prefetch_card(self, box, status, **extra):
        """웹 LLM 판단 탭에 미리 판단 진행 카드(요청함 → 받음 → 교체/버림)를 띄운다."""
        label = {"requested": "미리 판단 요청함",
                 "answered": "미리 판단 받음 (로봇 결과 확인 중)",
                 "discarded": "미리 판단 버림"}[status]
        note = f"{label}: {box['assume']}"
        if extra.get("answer"):
            note += f" → {extra['answer']}"
        if extra.get("reason"):
            note += f" · {extra['reason']}"
        self.publish_decision({
            "turn": box["turn"], "decider": "ai", "name": None,
            "text": [note],
            "prefetch": {"status": status, "assume": box["assume"],
                         **extra}})

    def _discard_prefetch(self, box, reason):
        with box["lock"]:
            box["discarded"] = True
            self._prefetch_card(box, "discarded", reason=reason)
        self.log("prefetch", {"used": False, "reason": reason,
                              "turn": box["turn"]})

    @staticmethod
    def _prefetch_matches(state, turn, box):
        """실제 결과가 미리 판단의 가정과 다르면 그 이유, 같으면 None."""
        last = state["last_action"]
        if (box["turn"] != turn or not last
                or last["name"] != box["name"]
                or last["args"] != box["args"]):
            return "다른 행동의 결과"
        if not last["success"]:
            return f"실패 ({last['error']})"
        result = last["result"]
        if (box["name"] == "GOTO"
                and state["robot"]["here"] != box["args"]["t"]):
            return "다른 곳에 도착"
        if (box["name"] in ("UNLOAD", "VERIFY_LOCATION")
                and not all(item.get("ok")
                            for item in result.get("items", []))):
            return "일부 항목이 성공·일치하지 않음"
        if (box["name"] == "UNLOAD"
                and len(result.get("items", [])) != len(box["args"]["items"])):
            return "일부만 놓음"
        if box["name"] == "VERIFY_LOCATION" and result.get("extra"):
            return "목록에 없는 cell에 퍽이 있음"
        return None

    def take_prefetch(self, state, turn):
        """쓸 수 있는 미리 판단 (행동, 인자, 계획, 기록). 가정과 실제가 다르면 버린다."""
        box, self.box = self.box, None
        if box is None:
            return None
        mismatch = self._prefetch_matches(state, turn, box)
        if mismatch:
            self._discard_prefetch(box, f"가정과 다름: {mismatch}")
            return None
        wait_started = time.monotonic()
        box["done"].wait()  # 아직 답을 받는 중이면 남은 시간만 기다린다
        waited = round(time.monotonic() - wait_started, 2)
        if box["result"] is None:
            self._discard_prefetch(box, f"미리 판단 실패: {box['error']}")
            return None
        name, args, plan, record = box["result"]
        self.log("prefetch", {"used": True, "turn": box["turn"],
                              "waited_s": waited})
        return name, args, plan, {**record, "prefetched": True,
                                  "prefetch_wait_s": waited}
