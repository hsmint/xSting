"""중계 노드. MissionState를 들고 ChatGPT의 판단을 받아 로봇(Jetson)에 명령하고, 결과를 상태에 기록한다.

ChatGPT(OpenAI Responses API)가 [고정 프롬프트 + 현재 상태]를 보고 도구(행동) 하나를 고른다(대화 기록 없음).
중계 노드는 할당·순서를 계산하지 않는다. API 오류·거절·도구 미호출이면 미션을 멈춘다.

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
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from xsting_interfaces.action import RobotCommand

from .state import ToolError, apply_result, check_location, new_state, stamp_now, to_message
from .tools import TOOL_NAMES, TOOLS

# 행동별 결과 대기 시간(초). 신호는 반복되므로 놓치면 다음 주기까지 기다릴 수 있다
ACTION_TIMEOUT_S = {"READ_MISSION_INPUT": 600.0, "GOTO": 180.0, "UNLOAD": 300.0}
DEFAULT_TIMEOUT_S = 120.0


class LlmUnavailable(Exception):
    """이번 판단에 LLM을 쓸 수 없음. 미션을 멈춘다."""


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
        self.declare_parameter("log_dir", os.path.expanduser("~/xsting_logs"))
        self.declare_parameter("max_turns", 200)
        # 가드: 위치 조건을 만족하지 않는 명령을 로봇에 보내기 전에 거절한다.
        # 기본은 꺼 둔다(LLM 단독 판단 시험). 실물 로봇에서는 true를 권장한다.
        self.declare_parameter("guard", False)

        self.model = self.get_parameter("model").value
        self.effort = self.get_parameter("effort").value
        self.max_turns = self.get_parameter("max_turns").value
        self.guard = self.get_parameter("guard").value
        summary = self.get_parameter("reasoning_summary").value
        self.reasoning_summary = None if summary == "none" else summary

        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(self.get_parameter("log_dir").value, f"run_{stamp}")
        os.makedirs(self.run_dir, exist_ok=True)
        self._log = open(os.path.join(self.run_dir, "mission.jsonl"), "a", encoding="utf-8")

        self.robot = ActionClient(self, RobotCommand, "robot_command")
        self.state = new_state()
        self.turn = 0
        self.last_decision = None
        self.records = []  # decisions.json
        self.halt = None  # 미션을 멈춘 이유
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

        import openai  # 이 노드에서만 쓰므로 여기서 불러온다
        self.openai = openai
        self.client = openai.OpenAI(timeout=60.0, max_retries=2)  # 키: OPENAI_API_KEY
        with open(self.get_parameter("prompt_file").value, encoding="utf-8") as f:
            # 경기 중 바꾸지 않는다. 도구 + 고정 프롬프트가 매번 같아서 앞부분이 자동으로 캐시된다.
            self.instructions = f.read()
        self.save("meta.json", {"model": self.model, "effort": self.effort, "guard": self.guard,
                                "reasoning_summary": self.reasoning_summary})
        self.get_logger().info(f"llm_commander model={self.model}, effort={self.effort}, "
                               f"guard={'on' if self.guard else 'off'}, logs in {self.run_dir}")

    # ---------- 기록 ----------
    def log(self, kind, data):
        self._log.write(json.dumps({"t": time.time(), "turn": self.turn, "kind": kind,
                                    "data": data}, ensure_ascii=False, default=str) + "\n")
        self._log.flush()

    def save(self, filename, data):
        with open(os.path.join(self.run_dir, filename), "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

    def publish_decision(self, record):
        record = {"run_id": self.run_id, "turn": self.turn, "t": time.time(), "model": self.model,
                  "effort": self.effort, **record}
        self.decision_pub.publish(String(data=json.dumps(record, ensure_ascii=False, default=str)))
        # 파일에도 같은 내용을 남긴다. 결과(outcome)는 같은 turn의 판단에 합친다
        if "outcome" in record and record.get("decider") != "user":
            for d in reversed(self.records):
                if d["turn"] == record["turn"] and d.get("decider") != "user":
                    d["outcome"] = record["outcome"]
                    break
        else:
            self.records.append(record)
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
                               "decision": self.last_decision, "halt": self.halt},
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
    def request(self, sent_state):
        reasoning = {"effort": self.effort}
        if self.reasoning_summary:
            reasoning["summary"] = self.reasoning_summary
        return self.client.responses.create(
            model=self.model,
            instructions=self.instructions,
            input="현재 MissionState입니다. 규칙과 상태를 보고 다음 행동 하나를 도구로 호출하세요.\n"
                  + to_message(sent_state),
            tools=TOOLS,
            tool_choice="auto",
            parallel_tool_calls=False,  # 한 번에 행동 하나
            reasoning=reasoning,
            max_output_tokens=16000,
            prompt_cache_key="xsting_mission",
            store=False,  # 대화를 서버에 남기지 않는다. 매번 상태만 보내므로 필요 없다
        )

    def decide(self):
        openai = self.openai
        self.state["meta"] = {"seq": self.turn, "stamp": stamp_now()}
        sent_state = copy.deepcopy(self.state)  # 이번 판단에 보낸 상태 그대로 기록한다
        t0 = time.monotonic()
        try:
            try:
                response = self.request(sent_state)
            except openai.BadRequestError as e:
                # 생각 요약은 계정(조직 인증)이나 모델에 따라 막혀 있을 수 있다. 그때는 요약 없이 다시 요청한다
                if not self.reasoning_summary or "summar" not in str(e).lower():
                    raise
                self.get_logger().warning(f"reasoning summary unavailable, retrying without it: {e}")
                self.reasoning_summary = None
                response = self.request(sent_state)
        except (openai.BadRequestError, openai.AuthenticationError, openai.NotFoundError):
            raise  # 요청 형식·키·모델 이름 오류는 코드나 설정 문제라 숨기지 않는다
        except (openai.RateLimitError, openai.APIConnectionError, openai.APIStatusError) as e:
            self.publish_decision({"decider": "ai", "input_state": sent_state, "error": f"API_ERROR {e!r}",
                                   "latency_s": round(time.monotonic() - t0, 2)})
            raise LlmUnavailable(f"API_ERROR {e!r}")

        latency = time.monotonic() - t0
        texts, refusals, reasoning, call = [], [], [], None
        for item in response.output:
            if item.type == "function_call" and call is None:
                call = item
            elif item.type == "reasoning":
                reasoning += [part.text for part in (item.summary or []) if getattr(part, "text", None)]
            elif item.type == "message":
                for part in item.content:
                    if part.type == "output_text":
                        texts.append(part.text)
                    elif part.type == "refusal":
                        refusals.append(part.refusal)
        usage = response.usage.model_dump() if response.usage else None
        record = {"input_state": sent_state, "latency_s": round(latency, 2), "status": response.status,
                  "reasoning": reasoning, "text": texts, "refusal": refusals, "usage": usage,
                  "raw_output": [item.model_dump() for item in response.output],
                  "name": call.name if call else None, "arguments": call.arguments if call else None}
        self.log("llm", {k: v for k, v in record.items() if k != "input_state"})

        error = None
        if response.status != "completed":  # incomplete: max_output_tokens 초과 등
            error = f"STATUS_{response.status}"
        elif refusals:
            error = f"REFUSAL {refusals}"
        elif call is None:
            error = f"NO_TOOL_CALL {texts}"
        else:
            try:
                args = json.loads(call.arguments)
            except json.JSONDecodeError:
                error = f"BAD_ARGUMENTS {call.arguments!r}"
        if error:
            self.publish_decision({**record, "decider": "ai", "error": error})
            raise LlmUnavailable(error)

        self.publish_decision({**record, "decider": "ai", "args": args})
        self.last_decision = {"model": self.model, "name": call.name, "args": args,
                              "text": " ".join(texts), "reasoning": " ".join(reasoning),
                              "latency_s": round(latency, 2)}
        return call.name, args

    # ---------- 실행 ----------
    def run_tool(self, name, args):
        state = self.state
        if name not in TOOL_NAMES:
            result = {"success": False, "error": "UNKNOWN_TOOL"}
        else:
            try:
                if self.guard:
                    check_location(state, name)
            except ToolError as e:
                result = {"success": False, "error": "REJECTED", "message": str(e)}
            else:
                result = self.call_robot(name, args)

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
