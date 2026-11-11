"""중계 노드. MissionState를 들고 ChatGPT의 판단을 받아 로봇(Jetson)에 명령하고, 결과를 상태에 기록한다.

ChatGPT(OpenAI Responses API)가 [고정 프롬프트 + 현재 상태]를 보고 도구(행동) 하나를 고른다(대화 기록 없음).
중계 노드는 할당·순서를 계산하지 않는다. API 오류·거절·도구 미호출이면 미션을 멈춘다.

decider:=script면 AI 없이 config/fixed_actions.yaml의 행동을 순서대로 보낸다 (주행·적재 시험용).

기록: log_dir/run_<시각>/ 에
  mission.jsonl     판단·명령·진행 상황 (한 줄에 하나)
  decisions.json    판단 전체 기록: 보낸 상태, 생각 요약, 응답 원문, 토큰, 로봇 결과 (매 판단마다 갱신)
  report.md         사람이 읽기 쉬운 요약 (판이 끝날 때)
  problem.json      가짜 로봇이 낸 문제(퍽·신호·바코드). 웹 화면에서 이 판을 똑같이 다시 실행할 때 쓴다
  signal.json, verification.json, final_state.json
"""
import copy
import json
import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from xsting_interfaces.action import RobotCommand

from . import planner
from .state import ToolError, apply_result, check_location, check_unload, new_state, stamp_now, to_message
from .arena_map import ArenaMap
from .contract import check_result
from .tools import CALC_TOOL, MAX_ROUTES, ROBOT_TOOLS, TOOL_NAMES, TOOLS

# 행동별 결과 대기 시간(초). 신호는 반복되므로 놓치면 다음 주기까지 기다릴 수 있다
ACTION_TIMEOUT_S = {"READ_MISSION_INPUT": 600.0, "GOTO": 180.0, "UNLOAD": 300.0}
DEFAULT_TIMEOUT_S = 120.0
# OpenAI 안전 필터 오탐은 같은 입력이라도 다시 보내면 통과하는 경우가 많다. 이만큼 더 보내 본다
FLAGGED_RETRIES = 3
MAX_CALC_CALLS = 3  # 판단 한 번에 계산 도구(COMPARE_ROUTES)를 부를 수 있는 횟수. 넘으면 로봇 행동만 보여 준다


class LlmUnavailable(Exception):
    """이번 판단에 LLM을 쓸 수 없음. 미션을 멈춘다."""


def _pending_prefetch_index(records, record):
    """record로 바꿔야 할 "미리 판단 요청함/받음" 카드의 위치 (같은 turn). 없으면 None. 사람이 보낸 명령은 바꾸지 않는다."""
    if record.get("decider") == "user":
        return None
    for k in range(len(records) - 1, -1, -1):
        d = records[k]
        if d.get("turn") == record.get("turn") and (d.get("prefetch") or {}).get("status") in ("requested", "answered"):
            return k
    return None


def _input_item(item):
    """응답 항목을 다음 요청의 입력 항목으로 (없는 값은 뺀다)."""
    return item.model_dump(exclude_none=True)


def _log_item(item):
    """기록용 응답 항목. 암호화된 생각(encrypted_content)은 길고 읽을 수 없으니 뺀다."""
    data = item.model_dump()
    data.pop("encrypted_content", None)
    return data


def _sum_usage(usages):
    """여러 응답의 토큰 사용량을 더한다 (계산 도구로 여러 번 요청한 판단)."""
    if not usages:
        return None

    def add(a, b):
        if isinstance(a, dict):
            return {k: add(a.get(k), b.get(k)) if isinstance(b, dict) else a.get(k) for k in a}
        return (a or 0) + (b or 0) if isinstance(a, (int, float)) or isinstance(b, (int, float)) else a
    total = usages[0]
    for u in usages[1:]:
        total = add(total, u)
    return total


def _is_flagged(e):
    """OpenAI 안전 필터가 입력(프롬프트)을 걸러낸 400 오류인지."""
    msg = str(e).lower()
    return "flagged" in msg or "usage policy" in msg


class LlmCommander(Node):
    def __init__(self):
        super().__init__("llm_commander")
        share = get_package_share_directory("xsting_mission")
        # 기본값은 환경 변수(~/xsting_env.sh)에서, 실행할 때 -p model:=... 을 주면 그것이 우선한다.
        # 모델: gpt-6-luna(테스트) < gpt-6.1-terra < gpt-6.1-sol < gpt-6-astra(가장 정확)
        self.declare_parameter("model", os.environ.get("XSTING_LLM_MODEL", "gpt-6.1-sol"))
        # reasoning effort: low가 빠르다. gpt-6-astra, gpt-6.1-sol은 none을 지원하지 않는다
        self.declare_parameter("effort", os.environ.get("XSTING_LLM_EFFORT", "low"))
        # 생각 요약(reasoning summary): auto | detailed | concise | none. 원문 생각은 OpenAI가 공개하지 않는다
        self.declare_parameter("reasoning_summary", "auto")
        self.declare_parameter("prompt_file", os.path.join(share, "prompts", "mission.md"))
        self.declare_parameter("arena_file", os.path.join(share, "config", "arena.yaml"))  # 경로 길이 계산용 지도
        self.declare_parameter("log_dir", os.path.expanduser("~/xsting_logs"))
        self.declare_parameter("max_turns", 200)
        # 로봇이 GOTO·UNLOAD·VERIFY_LOCATION을 하는 동안 "성공했다"고 가정한 다음 판단을 미리 받아 둔다
        # (결과가 가정과 같으면 바로 쓴다)
        self.declare_parameter("prefetch", True)
        self.declare_parameter("run_name", "")  # 이 판의 이름 (웹 지난 판 목록에 보임, 나중에 웹에서 바꿀 수 있음)
        # 가드: 위치 조건을 만족하지 않는 명령을 로봇에 보내기 전에 거절한다.
        # 기본은 꺼 둔다(LLM 단독 판단 시험). 실물 로봇에서는 true를 권장한다.
        self.declare_parameter("guard", False)
        # 판단 주체: llm(ChatGPT) | script(고정 행동 목록, AI를 쓰지 않는다)
        self.declare_parameter("decider", "llm")
        self.declare_parameter("script_file", os.path.join(share, "config", "fixed_actions.yaml"))

        self.model = self.get_parameter("model").value
        self.effort = self.get_parameter("effort").value
        self.max_turns = self.get_parameter("max_turns").value
        self.guard = self.get_parameter("guard").value
        self.prefetch_enabled = self.get_parameter("prefetch").value
        self.prefetch = None  # 미리 받는 중이거나 받은 다음 판단
        self._log_lock = threading.Lock()  # 미리 판단하는 스레드도 기록을 쓴다
        self._records_lock = threading.Lock()
        self.decider = self.get_parameter("decider").value
        if self.decider not in ("llm", "script"):
            raise ValueError(f"decider는 llm 또는 script: {self.decider}")
        summary = self.get_parameter("reasoning_summary").value
        self.reasoning_summary = None if summary == "none" else summary

        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(self.get_parameter("log_dir").value, f"run_{stamp}")
        os.makedirs(self.run_dir, exist_ok=True)
        self._log = open(os.path.join(self.run_dir, "mission.jsonl"), "a", encoding="utf-8")
        run_name = " ".join(self.get_parameter("run_name").value.split())[:60]
        if run_name:
            with open(os.path.join(self.run_dir, "name.txt"), "w", encoding="utf-8") as f:
                f.write(run_name)

        self.robot = ActionClient(self, RobotCommand, "robot_command")
        self.state = new_state()
        self.turn = 0
        self.last_decision = None
        self.records = []  # decisions.json
        self.halt = None  # 미션을 멈춘 이유
        self.plan_history = []  # 계획을 처음 세우거나 바꾼 판단만 [{turn, plan}] (화면 표시용)
        # 가짜 로봇 웹 화면이 판단 내용을 보여 줄 수 있게 상태를 발행한다 (늦게 켠 쪽도 마지막 값을 받음)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, "/mission/state", qos)
        # 판단 하나하나의 전체 기록 (보낸 상태, 생각 요약, 응답 원문, 토큰). 화면의 "LLM 판단" 탭에서 본다
        self.run_id = stamp
        self.decision_pub = self.create_publisher(
            String, "/mission/decision", QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # 웹 화면의 일시정지·재개·미션 정지
        self.paused = False
        self.stop_requested = False
        self.create_subscription(String, "/mission/control", self.on_control, 10)
        # 가짜 로봇이 낸 문제를 판마다 남긴다 (실물 로봇에는 없는 토픽이라 그때는 저장되지 않음)
        self.create_subscription(String, "/fake/problem", lambda m: self.save("problem.json", json.loads(m.data)),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # 사람이 웹 화면에서 직접 보낸 명령: 상태에 반영하고 판단 기록에 "사용자"로 남긴다
        self.create_subscription(String, "/mission/user_command", self.on_user_command, 10)

        if self.decider == "script":
            self.load_script(self.get_parameter("script_file").value)
            self.model, self.effort = "script", None
        else:
            import openai  # 이 노드에서만 쓰므로 여기서 불러온다
            self.openai = openai
            self.client = openai.OpenAI(timeout=60.0, max_retries=2)  # 키: OPENAI_API_KEY
            with open(self.get_parameter("prompt_file").value, encoding="utf-8") as f:
                # 경기 중 바꾸지 않는다. 도구 + 고정 프롬프트가 매번 같아서 앞부분이 자동으로 캐시된다.
                self.instructions = f.read()
            with open(self.get_parameter("arena_file").value, encoding="utf-8") as f:
                self.arena_map = ArenaMap(yaml.safe_load(f))  # COMPARE_ROUTES 계산 (가짜 경기장과 같은 지도·코스트맵)
        self.save("meta.json", {"decider": self.decider, "model": self.model, "effort": self.effort,
                                "guard": self.guard, "reasoning_summary": self.reasoning_summary})
        self.get_logger().info(f"llm_commander decider={self.decider}, model={self.model}, effort={self.effort}, "
                               f"guard={'on' if self.guard else 'off'}, logs in {self.run_dir}")

    def load_script(self, path):
        """고정 행동 목록을 [(이름, 인자)]로 읽는다. 형식은 config/fixed_actions.yaml 주석 참고."""
        with open(path, encoding="utf-8") as f:
            script = yaml.safe_load(f)
        self.script, self.script_pos = [], 0
        self.script_retries = script.get("retries", 0)
        self.script_stop_on_fail = script.get("stop_on_fail", False)
        for item in script["actions"]:
            if isinstance(item, str):
                name, args = item, {}
            else:
                (name, value), = item.items()
                if value == "auto":  # UNLOAD·VERIFY_LOCATION: 그 자리에서 할당 계획(planner)으로 채운다
                    args = {"items": "auto"}
                else:
                    args = {"t": value} if isinstance(value, str) else {"items": value} if isinstance(value, list) else value
            if name not in TOOL_NAMES:
                raise ValueError(f"{path}: 모르는 행동 {name}")
            self.script.append((name, args))
        self.script_fails = 0  # 지금 행동의 연속 실패 수

    # ---------- 기록 ----------
    def log(self, kind, data):
        line = json.dumps({"t": time.time(), "turn": self.turn, "kind": kind, "data": data},
                          ensure_ascii=False, default=str)
        with self._log_lock:
            self._log.write(line + "\n")
            self._log.flush()

    def save(self, filename, data):
        with open(os.path.join(self.run_dir, filename), "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

    def publish_decision(self, record):
        record = {"run_id": self.run_id, "turn": self.turn, "t": time.time(), "model": self.model,
                  "effort": self.effort, **record}
        with self._records_lock:  # 미리 판단 스레드도 카드를 발행한다
            self.decision_pub.publish(String(data=json.dumps(record, ensure_ascii=False, default=str)))
            # 파일에도 같은 내용을 남긴다. 결과(outcome)는 같은 turn의 판단에 합친다
            if "outcome" in record and record.get("decider") != "user":
                for d in reversed(self.records):
                    if d["turn"] == record["turn"] and d.get("decider") != "user":
                        d["outcome"] = record["outcome"]
                        break
            else:
                k = _pending_prefetch_index(self.records, record)
                if k is None:
                    self.records.append(record)
                else:  # "미리 판단 요청함/받음" 카드를 실제 판단(또는 버림)으로 바꾼다
                    self.records[k] = record
            self.save("decisions.json", self.records)

    def write_report(self):
        """판 전체를 사람이 읽기 쉬운 마크다운으로 남긴다."""
        recs, st = self.records, self.state
        def total(*keys):  # 예: total("input_tokens_details", "cached_tokens")
            n = 0
            for d in recs:
                v = d.get("usage") or {}
                for k in keys:
                    v = (v or {}).get(k)
                n += v or 0
            return n

        lat = [d["latency_s"] for d in recs if d.get("latency_s")]
        w = st["world"]
        lines = [
            f"# 미션 실행 기록 {self.run_id}", "",
            f"- 모델: {self.model} (effort {self.effort}), 가드: {'on' if self.guard else 'off'}",
            f"- 결과: phase={st['phase']}" + (f", 중단: {self.halt}" if self.halt else ""),
            f"- 판단 {len(recs)}회, 응답 평균 {sum(lat) / max(len(lat), 1):.1f}초 / 최대 {max(lat, default=0):.1f}초",
            f"- 토큰: 입력 {total('input_tokens')} (캐시 {total('input_tokens_details', 'cached_tokens')}), "
            f"출력 {total('output_tokens')} (생각 {total('output_tokens_details', 'reasoning_tokens')})",
        ]
        if w["barcode"] and w["signal"]:
            c = w["signal"]["counts"]
            lines.append(f"- 입력: 바코드 {w['barcode']['code']} ({w['barcode']['order']}), "
                         f"신호 {''.join(w['signal']['sequence'])} (R{c['R']} Y{c['Y']} G{c['G']})")
        lines += ["", "## 하차 박스 검증", ""]
        for loc, v in w["locations"].items():
            ver = v["verified"]
            placed = [f"{c['puck_id']}→c{c['cell']}" for c in v["cells"] if c["state"] == "occupied"]
            ok = "검증 안 함" if ver is None else ("통과" if all(i.get("ok") for i in ver["items"]) and not ver["extra"]
                                                 else f"불일치 (extra {ver['extra']})")
            lines.append(f"- {loc}: {' '.join(placed) or '비어 있음'} — {ok}")
        lines += ["", "## 판단 순서", "",
                  "| # | 누가 | 행동 | 인자 | 결과 | 응답(초) | 생각 요약 / 응답 문장 |", "|---|---|---|---|---|---|---|"]
        for d in recs:
            o = d.get("outcome") or {}
            res = d.get("error") or ("✓" if o.get("success") else f"✗ {o.get('error')}" if o else "-")
            note = " / ".join((d.get("reasoning") or []) + (d.get("text") or [])).replace("\n", " ").replace("|", "\\|")
            who = {"ai": "AI", "user": "사용자", "rule": "규칙"}.get(d.get("decider"), "-")
            lines.append(f"| {d['turn']} | {who} | {d.get('name') or '-'} | `{json.dumps(d.get('args', {}), ensure_ascii=False)}` "
                         f"| {res} | {d.get('latency_s', '-')} | {note[:300]} |")
        with open(os.path.join(self.run_dir, "report.md"), "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")

    def publish_state(self):
        msg = String()
        msg.data = json.dumps({"model": self.model, "turn": self.turn, "state": self.state,
                               "decision": self.last_decision, "halt": self.halt,
                               "plan_history": self.plan_history},
                              ensure_ascii=False, default=str)
        self.state_pub.publish(msg)

    def on_control(self, msg):
        if msg.data == "pause":
            self.paused = True
        elif msg.data == "resume":
            self.paused = False
        elif msg.data == "stop":
            self.stop_requested, self.paused = True, False
        self.get_logger().info(f"control: {msg.data}")

    def on_user_command(self, msg):
        cmd = json.loads(msg.data)
        name, args, result = cmd["name"], cmd["args"], cmd["result"]
        apply_result(self.state, name, args, result)  # 로봇이 실제로 움직였으니 중계 노드 상태도 맞춘다
        self.log("user", {"name": name, "args": args, "result": result, "phase": self.state["phase"]})
        self.get_logger().info(f"[{self.turn}] user {name} {json.dumps(args, ensure_ascii=False)} -> "
                               f"{'OK' if result.get('success') else result.get('error')} (phase={self.state['phase']})")
        self.publish_decision({"decider": "user", "model": None, "effort": None, "name": name, "args": args,
                               "outcome": {"success": bool(result.get("success")), "error": result.get("error"),
                                           "result": result, "phase_after": self.state["phase"]}})
        self.publish_state()

    def wait_while_paused(self):
        """일시정지면 재개나 정지가 올 때까지 기다린다. 정지가 요청되면 False."""
        rclpy.spin_once(self, timeout_sec=0)
        while self.paused and not self.stop_requested and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        return not self.stop_requested

    # ---------- 로봇 통신 ----------
    def call_robot(self, name, args):
        if not self.robot.wait_for_server(timeout_sec=5.0):
            return {"success": False, "error": "ROBOT_UNREACHABLE"}

        goal = RobotCommand.Goal(name=name, args_json=json.dumps(args))
        send_future = self.robot.send_goal_async(
            goal, feedback_callback=lambda fb: self.log("feedback", {"name": name, "phase": fb.feedback.phase}))
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return {"success": False, "error": "GOAL_REJECTED"}

        result_future = handle.get_result_async()
        limit, waited = ACTION_TIMEOUT_S.get(name, DEFAULT_TIMEOUT_S), 0.0
        while not result_future.done():
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.5)
            if not self.paused:  # 일시정지 중인 시간은 대기 시간에 넣지 않는다
                waited += 0.5
            if waited > limit:
                handle.cancel_goal_async()
                return {"success": False, "error": "TIMEOUT"}
        return json.loads(result_future.result().result.result_json)

    # ---------- 판단 (ChatGPT) ----------
    def request(self, sent_state, extra=(), tools=TOOLS):
        """extra: 이번 판단 안에서 앞서 받은 응답 항목과 계산 도구 결과 (계산 도구를 쓴 뒤 다시 요청할 때)."""
        reasoning = {"effort": self.effort}
        if self.reasoning_summary:
            reasoning["summary"] = self.reasoning_summary
        text = "현재 MissionState입니다. 규칙과 상태를 보고 다음 행동 하나를 도구로 호출하세요.\n" + to_message(sent_state)
        return self.client.responses.create(
            model=self.model,
            instructions=self.instructions,
            input=[{"role": "user", "content": text}, *extra],
            tools=list(tools),
            tool_choice="auto",
            parallel_tool_calls=False,  # 한 번에 도구 하나
            reasoning=reasoning,
            max_output_tokens=16000,
            prompt_cache_key="xsting_mission",
            store=False,  # 대화를 서버에 남기지 않는다. 매번 상태만 보내므로 필요 없다
            # store=False에서 계산 도구 결과를 주고 이어서 판단하려면 앞 응답의 생각(암호화)을 돌려줘야 한다
            include=["reasoning.encrypted_content"],
        )

    def compare_routes(self, args, state):
        """COMPARE_ROUTES: 그 판단에 보낸 상태의 로봇 위치에서 후보 순서마다 경로 길이를 계산한다 (로봇에는 보내지 않는다)."""
        routes = [list(r) for r in (args.get("routes") or [])][:MAX_ROUTES]
        pose, here = state["robot"]["pose"], state["robot"]["here"]
        if pose:
            start = (pose["x"], pose["y"])
        else:  # 첫 판단 등 pose를 아직 모르면 지금 장소의 등록 위치에서
            start = self.arena_map.targets.get(here, self.arena_map.targets["HOME"])
        result = {"from": {"x": round(start[0]), "y": round(start[1])},
                  "routes": self.arena_map.route_lengths(start, routes),
                  "note": "길이는 mm. 장소는 대상 둘레(330~520mm)의 가장 가까운 자리까지. None은 갈 수 없는 구간."}
        if len(args.get("routes") or []) > MAX_ROUTES:
            result["note"] += f" 후보는 {MAX_ROUTES}개까지만 계산했다."
        return result

    def decide_script(self):
        """고정 행동 목록에서 다음 행동. 직전 행동이 실패했으면 retries번까지 같은 행동을 다시 보낸다."""
        last = self.state["last_action"]
        if self.script_pos > 0 and last is not None and not last["success"] and self.script_fails < self.script_retries:
            self.script_fails += 1
            name, args = self.script[self.script_pos - 1]
            note = f"고정 행동 {self.script_pos}/{len(self.script)} 다시 시도 ({self.script_fails}/{self.script_retries})"
        else:
            if self.script_pos > 0 and last is not None and not last["success"] and self.script_stop_on_fail:
                raise LlmUnavailable(f"SCRIPT_FAILED {last['name']} {last['error']}")
            if self.script_pos >= len(self.script):
                raise LlmUnavailable("SCRIPT_END")
            name, args = self.script[self.script_pos]
            self.script_pos += 1
            self.script_fails = 0
            note = f"고정 행동 {self.script_pos}/{len(self.script)}"
        if args.get("items") == "auto":
            args = {"items": self.planned_items(name)}
            note += " (할당 계획으로 items 채움)"
        # 고정 행동 목록의 남은 순서를 계획으로 보여 준다 (화면 표시용)
        rest = [f"{n} {a.get('t', '')}".strip() if "items" not in a else f"{n} {a['items'] if a['items'] == 'auto' else '…'}"
                for n, a in self.script[self.script_pos:]]
        plan = {"assignment": "UNLOAD·VERIFY_LOCATION auto는 할당 계획(planner.py)으로 채움", "route": rest,
                "reason": "고정 행동 목록 (config/fixed_actions.yaml)", "change": ""}
        sent_state = copy.deepcopy(self.state)
        self.set_plan(plan)
        self.publish_decision({"decider": "rule", "input_state": sent_state, "name": name,
                               "args": args, "text": [note], "plan": plan})
        self.last_decision = {"model": self.model, "name": name, "args": args, "text": note, "reasoning": "",
                              "latency_s": 0, "plan": plan}
        return name, args

    def _request_with_retries(self, sent_state, extra, tools, t0, quiet=False):
        """요청 한 번. 생각 요약이 막혔거나 안전 필터 오탐이면 다시 보낸다. 쓸 수 없으면 LlmUnavailable.

        quiet: 미리 판단(이동 중)일 때는 실패해도 판단 기록을 남기지 않는다 (버리고 도착 뒤에 새로 판단한다).
        """
        openai = self.openai
        try:
            flagged_retries = FLAGGED_RETRIES
            request_state = sent_state
            while True:
                try:
                    return self.request(request_state, extra, tools)
                except openai.BadRequestError as e:
                    # 생각 요약은 계정(조직 인증)이나 모델에 따라 막혀 있을 수 있다. 그때는 요약 없이 다시 요청한다
                    if self.reasoning_summary and "summar" in str(e).lower():
                        self.get_logger().warning(f"reasoning summary unavailable, retrying without it: {e}")
                        self.reasoning_summary = None
                    # OpenAI 안전 필터가 입력을 잘못 걸러내는 경우가 있다 (오탐). 지난 계획의 근거 문장(reason)이
                    # 걸린 것일 수 있으니, 그 문장을 빼고(할당·순서는 남김) 다시 보낸다
                    elif _is_flagged(e) and flagged_retries > 0:
                        self.get_logger().warning(f"prompt flagged, retrying without plan text "
                                                  f"({FLAGGED_RETRIES - flagged_retries + 1}/{FLAGGED_RETRIES}): {e}")
                        self.log("flagged", {"error": repr(e)})
                        flagged_retries -= 1
                        request_state = copy.deepcopy(sent_state)
                        plan = request_state["memory"]["plan"]
                        if plan:
                            plan.pop("reason", None)
                    else:
                        raise
        except openai.BadRequestError as e:
            if _is_flagged(e):  # 몇 번을 다시 보내도 걸렸다. 노드를 죽이지 않고 기록을 남긴 뒤 멈춘다
                if not quiet:
                    self.publish_decision({"decider": "ai", "input_state": sent_state,
                                           "error": f"PROMPT_FLAGGED {e!r}", "latency_s": round(time.monotonic() - t0, 2)})
                raise LlmUnavailable(f"PROMPT_FLAGGED {e!r}")
            raise  # 요청 형식 오류는 코드나 설정 문제라 숨기지 않는다
        except (openai.AuthenticationError, openai.NotFoundError):
            raise  # 키·모델 이름 오류는 설정 문제라 숨기지 않는다
        except (openai.RateLimitError, openai.APIConnectionError, openai.APIStatusError) as e:
            if not quiet:
                self.publish_decision({"decider": "ai", "input_state": sent_state, "error": f"API_ERROR {e!r}",
                                       "latency_s": round(time.monotonic() - t0, 2)})
            raise LlmUnavailable(f"API_ERROR {e!r}")

    def set_plan(self, plan):
        """LLM이 적은 계획을 다음 판단에 돌려줄 memory.plan에 저장한다. 처음 세우거나 바뀐 계획은 이력에 남긴다."""
        if not plan:
            return
        prev = self.state["memory"]["plan"]
        self.state["memory"]["plan"] = {"turn": self.turn, "assignment": plan.get("assignment"),
                                        "route": plan.get("route"), "reason": plan.get("reason")}
        if prev is None or plan.get("change"):
            self.plan_history.append({"turn": self.turn, "plan": plan})

    def planned_items(self, name):
        """지금 있는 하차 박스에 놓을(UNLOAD) / 확인할(VERIFY_LOCATION) items를 할당 계획에서 꺼낸다."""
        loc = self.state["robot"]["here"]
        try:
            items = planner.assign(self.state).get(loc)
        except planner.PlanError as e:
            raise LlmUnavailable(f"PLAN_FAILED {e}")
        if items is None:
            raise LlmUnavailable(f"PLAN_FAILED {name} auto는 하차 박스에서만 (지금 here={loc})")
        if name == "UNLOAD":  # 이미 놓은 퍽은 빼고, 적재함에 있는 것만
            items = [i for i in items if self.state["world"]["pucks"][i["puck_id"]]["where"] == "storage"]
        return items

    def decide(self):
        self.state["meta"] = {"seq": self.turn, "stamp": stamp_now()}
        if self.decider == "script":
            return self.decide_script()
        decided = self._take_prefetch()  # 이동하는 동안 미리 받아 둔 판단 (가정이 맞았을 때만)
        if decided is None:
            sent_state = copy.deepcopy(self.state)  # 이번 판단에 보낸 상태 그대로 기록한다
            decided = self._llm_decide(sent_state)
        name, args, plan, record = decided
        self.set_plan(plan)
        self.publish_decision({**record, "decider": "ai", "args": args, "plan": plan})
        self.last_decision = {"model": self.model, "name": name, "args": args, "plan": plan,
                              "calcs": record.get("calcs"), "prefetched": record.get("prefetched", False),
                              "text": " ".join(record.get("text") or []),
                              "reasoning": " ".join(record.get("reasoning") or []), "latency_s": record["latency_s"]}
        return name, args

    def _llm_decide(self, sent_state, quiet=False):
        """sent_state를 보내 LLM의 다음 행동을 받는다. 상태·계획은 바꾸지 않는다 (확정은 decide가 한다).

        반환: (행동 이름, 인자, 계획, 판단 기록). 쓸 수 없으면 LlmUnavailable (quiet가 아니면 기록을 남긴다).
        """
        t0 = time.monotonic()
        extra, calcs, responses = [], [], []
        while True:
            # 계산 도구를 MAX_CALC_CALLS번 쓰면 로봇 행동만 보여 줘서 하나를 고르게 한다
            tools = TOOLS if len(calcs) < MAX_CALC_CALLS else ROBOT_TOOLS
            response = self._request_with_retries(sent_state, extra, tools, t0, quiet)
            responses.append(response)
            call = next((item for item in response.output if item.type == "function_call"), None)
            if (call is None or call.name != CALC_TOOL or response.status != "completed"
                    or len(calcs) >= MAX_CALC_CALLS):
                break
            try:
                calc_args = json.loads(call.arguments)
            except json.JSONDecodeError:
                calc_args = {}
            result = self.compare_routes(calc_args, sent_state)
            calcs.append({"routes": calc_args.get("routes"), "result": result})
            self.log("calc", calcs[-1])
            # 앞 응답(생각·호출)과 계산 결과를 붙여 같은 판단을 이어서 요청한다
            extra += [_input_item(item) for item in response.output]
            extra.append({"type": "function_call_output", "call_id": call.call_id,
                          "output": json.dumps(result, ensure_ascii=False)})

        latency = time.monotonic() - t0
        texts, refusals, reasoning = [], [], []
        for resp in responses:
            for item in resp.output:
                if item.type == "reasoning":
                    reasoning += [part.text for part in (item.summary or []) if getattr(part, "text", None)]
                elif item.type == "message":
                    for part in item.content:
                        if part.type == "output_text":
                            texts.append(part.text)
                        elif part.type == "refusal":
                            refusals.append(part.refusal)
        usage = _sum_usage([resp.usage.model_dump() for resp in responses if resp.usage])
        record = {"input_state": sent_state, "latency_s": round(latency, 2), "status": response.status,
                  "reasoning": reasoning, "text": texts, "refusal": refusals, "usage": usage,
                  "raw_output": [_log_item(item) for resp in responses for item in resp.output],
                  "calcs": calcs, "rounds": len(responses),
                  "name": call.name if call else None, "arguments": call.arguments if call else None}
        self.log("llm", {k: v for k, v in record.items() if k != "input_state"})

        error = None
        if response.status != "completed":  # incomplete: max_output_tokens 초과 등
            error = f"STATUS_{response.status}"
        elif refusals:
            error = f"REFUSAL {refusals}"
        elif call is None:
            error = f"NO_TOOL_CALL {texts}"
        elif call.name == CALC_TOOL:  # 계산 도구만 계속 부르고 로봇 행동을 고르지 않음
            error = "TOO_MANY_CALC_CALLS"
        else:
            try:
                args = json.loads(call.arguments)
            except json.JSONDecodeError:
                error = f"BAD_ARGUMENTS {call.arguments!r}"
        if error:
            if not quiet:
                self.publish_decision({**record, "decider": "ai", "error": error})
            raise LlmUnavailable(error)

        plan = args.pop("plan", None)  # 계획은 상태에 남기고, 로봇에는 행동 인자만 보낸다
        return call.name, args, plan, record

    # ---------- 이동 중 미리 판단 ----------
    def start_prefetch(self, name, args):
        """로봇에 행동을 보낼 때, 그 행동이 성공했다고 가정한 상태로 다음 판단을 백그라운드에서 미리 받는다.

        - GOTO: 그 장소에 도착 (도착 좌표는 모르므로 pose는 비운다). 마지막 HOME 복귀는 하지 않는다.
        - UNLOAD: 보낸 items가 모두 놓였다.
        - VERIFY_LOCATION: 보낸 items가 모두 일치하고 남는 퍽이 없다.
        결과가 나오면 decide가 가정이 맞았는지 보고 쓴다(_take_prefetch). 결과 내용이 판단 재료인 행동
        (READ_MISSION_INPUT, LOAD)은 하지 않는다.
        """
        self.prefetch = None
        if self.decider != "llm" or not self.prefetch_enabled:
            return
        here = self.state["robot"]["here"]
        if name == "GOTO" and args.get("t") != "HOME":
            assumed = {"success": True, "robot": {"here": args["t"], "pose": None}}
        elif name == "UNLOAD":
            assumed = {"success": True, "stopped_at": None, "robot": {"here": here},
                       "items": [{**it, "ok": True, "offset_mm": [0.0, 0.0]} for it in args["items"]]}
        elif name == "VERIFY_LOCATION" and here in self.state["world"]["locations"]:
            pucks = self.state["world"]["pucks"]
            cells = [{"cell": c["cell"], "state": c["state"], "puck_id": c["puck_id"],
                      "color": pucks[c["puck_id"]]["color"] if c["puck_id"] is not None else None,
                      "shape": pucks[c["puck_id"]]["shape"] if c["puck_id"] is not None else None, "offset_mm": None}
                     for c in self.state["world"]["locations"][here]["cells"]]
            assumed = {"success": True, "all_ok": True, "extra": [], "cells": cells, "robot": {"here": here},
                       "items": [{**it, "ok": True, "seen_puck_id": it["puck_id"]} for it in args["items"]]}
        else:
            return
        predicted = copy.deepcopy(self.state)
        apply_result(predicted, name, args, assumed)
        predicted["meta"] = {"seq": self.turn + 1, "stamp": stamp_now()}
        assume = f"{name} {args['t']}" if name == "GOTO" else f"{name} {here} ({len(args['items'])}개)"
        box = {"turn": self.turn + 1, "name": name, "args": args, "assume": f"{assume} 성공 가정",
               "done": threading.Event(), "lock": threading.Lock(), "discarded": False, "result": None, "error": None}

        def work():
            try:
                box["result"] = self._llm_decide(predicted, quiet=True)
                n, a, _, rec = box["result"]
                with box["lock"]:  # 이미 버렸으면 "받음" 카드를 띄우지 않는다
                    if not box["discarded"]:
                        self._prefetch_card(box, "answered", answer=f"{n} {json.dumps(a, ensure_ascii=False)}",
                                            latency_s=rec["latency_s"])
            except Exception as e:  # 미리 판단이 안 되면 결과가 나온 뒤 평소처럼 판단한다
                box["error"] = repr(e)
            finally:
                box["done"].set()

        self.prefetch = box
        self._prefetch_card(box, "requested")
        threading.Thread(target=work, daemon=True).start()
        self.log("prefetch", {"start": True, "turn": box["turn"], "assume": box["assume"]})

    def _prefetch_card(self, box, status, **extra):
        """웹 LLM 판단 탭에 미리 판단 진행 상황 카드를 띄운다 (요청함 → 받음 → 실제 판단으로 교체 / 버림)."""
        label = {"requested": "미리 판단 요청함", "answered": "미리 판단 받음 (로봇 결과 확인 중)",
                 "discarded": "미리 판단 버림"}[status]
        note = f"{label}: {box['assume']}" + (f" → {extra['answer']}" if extra.get("answer") else "") \
            + (f" · {extra['reason']}" if extra.get("reason") else "")
        self.publish_decision({"turn": box["turn"], "decider": "ai", "name": None, "text": [note],
                               "prefetch": {"status": status, "assume": box["assume"], **extra}})

    def _discard_prefetch(self, box, reason):
        with box["lock"]:
            box["discarded"] = True
            self._prefetch_card(box, "discarded", reason=reason)
        self.log("prefetch", {"used": False, "reason": reason, "turn": box["turn"]})

    def _prefetch_matches(self, box):
        """실제 결과가 미리 판단에서 가정한 결과와 같은지. 다르면 그 이유, 같으면 None."""
        last = self.state["last_action"]
        if box["turn"] != self.turn or not last or last["name"] != box["name"] or last["args"] != box["args"]:
            return "다른 행동의 결과"
        if not last["success"]:
            return f"실패 ({last['error']})"
        result = last["result"]
        if box["name"] == "GOTO" and self.state["robot"]["here"] != box["args"]["t"]:
            return "다른 곳에 도착"
        if box["name"] in ("UNLOAD", "VERIFY_LOCATION") and not all(i.get("ok") for i in result.get("items", [])):
            return "일부 항목이 성공·일치하지 않음"
        if box["name"] == "UNLOAD" and len(result.get("items", [])) != len(box["args"]["items"]):
            return "일부만 놓음"
        if box["name"] == "VERIFY_LOCATION" and result.get("extra"):
            return "목록에 없는 cell에 퍽이 있음"
        return None

    def _take_prefetch(self):
        """미리 받은 판단을 쓸 수 있으면 (행동, 인자, 계획, 기록). 가정한 결과와 실제가 다르면 버린다."""
        box, self.prefetch = self.prefetch, None
        if box is None:
            return None
        mismatch = self._prefetch_matches(box)
        if mismatch:
            self._discard_prefetch(box, f"가정과 다름: {mismatch}")
            return None
        t = time.monotonic()
        box["done"].wait()  # 아직 답을 받는 중이면 남은 시간만 기다린다
        waited = round(time.monotonic() - t, 2)
        if box["result"] is None:
            self._discard_prefetch(box, f"미리 판단 실패: {box['error']}")
            return None
        name, args, plan, record = box["result"]
        self.log("prefetch", {"used": True, "turn": box["turn"], "waited_s": waited})
        return name, args, plan, {**record, "prefetched": True, "prefetch_wait_s": waited}

    # ---------- 실행 ----------
    def run_tool(self, name, args):
        state = self.state
        if name not in TOOL_NAMES:
            result = {"success": False, "error": "UNKNOWN_TOOL"}
        else:
            try:
                if self.guard:
                    check_location(state, name)
                if name == "UNLOAD":  # 나눠 놓기는 가드와 관계없이 항상 막는다
                    check_unload(state, args)
            except ToolError as e:
                result = {"success": False, "error": "REJECTED", "message": str(e)}
            else:
                self.start_prefetch(name, args)  # 로봇이 움직이는 동안 다음 판단을 미리 받아 둔다
                result = self.call_robot(name, args)
                # 로봇 서버가 다른 구현이어도 약속과 다른 결과로 상태를 망치지 않게 한 번 더 본다 (contract.py)
                result_errors = check_result(name, args, result)
                if result_errors:
                    self.get_logger().error(f"{name} result breaks contract.py: {result_errors}")
                    result = {"success": False, "error": "BAD_RESULT", "contract_errors": result_errors,
                              "raw_result": result}

        apply_result(state, name, args, result)
        self.publish_decision({"outcome": {"success": bool(result.get("success")), "error": result.get("error"),
                                           "result": result, "phase_after": state["phase"]}})
        self.log("tool", {"name": name, "args": args, "result": result, "phase": state["phase"]})
        self.get_logger().info(f"[{self.turn}] {name} {json.dumps(args, ensure_ascii=False)} -> "
                               f"{'OK' if result.get('success') else result.get('error')} (phase={state['phase']})")

        w = state["world"]
        if name == "READ_MISSION_INPUT" and result.get("success"):
            self.save("signal.json", {"barcode": w["barcode"], "signal": w["signal"]})
        if name == "VERIFY_LOCATION" and result.get("success"):
            self.save("verification.json", {loc: v["verified"] for loc, v in w["locations"].items()})

    def run(self):
        try:
            self.loop()
        except KeyboardInterrupt:
            self.halt = "CTRL_C"
            raise
        except Exception as e:  # 예상 못 한 오류(키·모델 설정 오류 포함)도 기록은 남긴다
            self.halt = f"ERROR {e!r}"
            raise
        finally:
            # 파일을 먼저 저장한다 (Ctrl+C로 ROS가 먼저 내려가도 기록은 남게)
            self.save("final_state.json", {"halt": self.halt, "state": self.state})
            self.write_report()
            try:
                self.publish_state()
            except Exception:
                pass
            self.get_logger().info(f"mission finished: phase={self.state['phase']}, halt={self.halt}, "
                                   f"logs in {self.run_dir}")

    def loop(self):
        while rclpy.ok() and self.state["phase"] != "DONE":
            self.turn += 1
            if self.turn > self.max_turns:
                self.halt = "MAX_TURNS"
                break
            self.publish_state()
            if not self.wait_while_paused():
                self.halt = "STOPPED_BY_USER"
                break
            try:
                name, args = self.decide()
            except LlmUnavailable as e:
                self.halt = str(e)
                if self.halt == "SCRIPT_END":
                    self.get_logger().info("fixed actions finished")
                else:
                    self.get_logger().error(f"LLM unavailable, mission halted: {e}")
                self.log("halt", {"reason": self.halt})
                break
            self.publish_state()
            self.run_tool(name, args)
            if self.stop_requested:
                self.halt = "STOPPED_BY_USER"
                break


def main():
    rclpy.init()
    node = LlmCommander()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
