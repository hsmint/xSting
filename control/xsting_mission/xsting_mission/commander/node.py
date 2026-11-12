"""중계 노드: LLM(또는 고정 행동 목록)의 판단으로 로봇에 명령하고 판 기록을 남긴다."""
import copy
import json
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from xsting_interfaces.action import RobotCommand

from ..core.contract import check_result
from ..core.state import (ToolError, apply_result, check_location,
                          check_repeat, check_revisit, check_time,
                          check_unload, new_state, stamp_now)
from .llm_decider import LlmDecider, LlmUnavailable
from .mission_log import MissionLog
from .prefetch import Prefetcher
from .script_decider import ScriptDecider
from .tools import TOOL_NAMES

# 행동별 결과 대기 시간(초). 신호는 반복되므로 놓치면 다음 주기까지 기다릴 수 있다
# 로봇 쪽 하위 동작 한도의 합보다 길어야 한다 (robot/ros_backend.py 위쪽 상수)
ACTION_TIMEOUT_S = {"READ_MISSION_INPUT": 600.0, "GOTO": 180.0,
                    "LOAD": 180.0, "UNLOAD": 420.0}
DEFAULT_TIMEOUT_S = 120.0
# 샘플링 값의 허용 범위 (OpenAI 기준)
SAMPLING_RANGES = {"temperature": (0.0, 2.0), "top_p": (0.0, 1.0)}
# effort none을 지원하지 않는 모델 (OpenAI 최신 모델 안내 기준)
NO_NONE_EFFORT_MODELS = ("gpt-6-astra", "gpt-6.1-sol")


def _sampling_options(model, effort, temperature, top_p):
    """보낼 샘플링 값 {이름: 값}. 음수는 빼고, 쓸 수 없는 조합이면 시작할 때 멈춘다."""
    if effort == "none" and model in NO_NONE_EFFORT_MODELS:
        raise ValueError(f"{model}은 effort none을 지원하지 않음")
    options = {name: value
               for name, value in (("temperature", temperature),
                                   ("top_p", top_p))
               if value >= 0}
    for name, value in options.items():
        low, high = SAMPLING_RANGES[name]
        if not low <= value <= high:
            raise ValueError(f"{name}는 {low}~{high}: {value}")
    if options and effort != "none":
        raise ValueError(
            f"temperature·top_p는 effort가 none일 때만 쓸 수 있음 "
            f"(지금 effort={effort})")
    return options


class LlmCommander(Node):
    """판단(LLM·고정 목록)과 로봇 실행을 번갈아 하는 미션 루프."""

    def __init__(self):
        super().__init__("llm_commander")
        share = get_package_share_directory("xsting_mission")

        # 기본값은 환경 변수(~/xsting_env.sh), -p model:=... 을 주면 그것이 우선
        # 모델: gpt-6-luna(테스트) < gpt-6.1-terra < gpt-6.1-sol < gpt-6-astra
        self.declare_parameter(
            "model", os.environ.get("XSTING_LLM_MODEL", "gpt-6.1-sol"))
        # low가 빠르다. gpt-6-astra, gpt-6.1-sol은 none을 지원하지 않는다
        self.declare_parameter(
            "effort", os.environ.get("XSTING_LLM_EFFORT", "low"))
        # 생각 요약: auto | detailed | concise | none (원문 생각은 공개되지 않음)
        self.declare_parameter("reasoning_summary", "auto")
        # 샘플링 (음수면 보내지 않음). OpenAI는 effort가 none일 때만 받는다
        self.declare_parameter("temperature", -1.0)
        self.declare_parameter("top_p", -1.0)
        self.declare_parameter(
            "prompt_file", os.path.join(share, "prompts", "mission.md"))
        # COMPARE_ROUTES 경로 길이 계산용 지도
        self.declare_parameter(
            "arena_file", os.path.join(share, "config", "arena.yaml"))
        self.declare_parameter(
            "log_dir", os.path.expanduser("~/xsting_logs"))
        self.declare_parameter("max_turns", 200)
        # 미션 시간 한도(초, 배터리 사용 시간). 지나면 HOME 복귀만 보낸다. 0이면 끔
        self.declare_parameter(
            "mission_time_s", 1800.0,
            ParameterDescriptor(dynamic_typing=True))  # 600처럼 정수로 줘도 된다
        # 같은 행동(같은 인자)의 연속 실패 상한. 넘으면 더 보내지 않는다. 0이면 끔
        self.declare_parameter("repeat_limit", 3)
        # GOTO·UNLOAD·VERIFY_LOCATION 중에 성공을 가정한 다음 판단을 미리 받아 둔다
        self.declare_parameter("prefetch", True)
        # 웹 지난 판 목록에 보이는 이 판의 이름 (웹에서 바꿀 수 있음)
        self.declare_parameter("run_name", "")
        # 위치 조건이 안 맞는 명령을 미리 거절. 실물 로봇에서는 true를 권장한다
        self.declare_parameter("guard", False)
        # 판단 주체: llm(ChatGPT) | script(고정 행동 목록, AI를 쓰지 않는다)
        self.declare_parameter("decider", "llm")
        self.declare_parameter(
            "script_file",
            os.path.join(share, "config", "fixed_actions.yaml"))

        self.model = self.get_parameter("model").value
        self.effort = self.get_parameter("effort").value
        self.max_turns = self.get_parameter("max_turns").value
        self.mission_time_s = float(
            self.get_parameter("mission_time_s").value or 0)
        self.repeat_limit = self.get_parameter("repeat_limit").value
        self.guard = self.get_parameter("guard").value
        self.decider = self.get_parameter("decider").value
        if self.decider not in ("llm", "script"):
            raise ValueError(f"decider는 llm 또는 script: {self.decider}")
        summary = self.get_parameter("reasoning_summary").value
        # effort가 none이면 생각이 없으니 생각 요약도 받지 않는다
        reasoning_summary = (None if "none" in (summary, self.effort)
                             else summary)
        sampling = _sampling_options(
            self.model, self.effort, self.get_parameter("temperature").value,
            self.get_parameter("top_p").value)

        # 기록 폴더 run_<시각>. 기록 줄의 turn은 쓰는 순간의 값
        self.mission_log = MissionLog(
            self.get_parameter("log_dir").value,
            self.get_parameter("run_name").value, lambda: self.turn)
        self.log, self.save = self.mission_log.log, self.mission_log.save
        self.robot = ActionClient(self, RobotCommand, "robot_command")
        self.state = new_state()
        self.turn = 0
        self.last_decision = None
        self.halt = None  # 미션을 멈춘 이유
        # 계획을 처음 세우거나 바꾼 판단만 [{turn, plan}] (화면 표시용)
        self.plan_history = []

        # 웹 화면용 상태 발행 (늦게 켠 쪽도 마지막 값을 받음)
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(
            String, "/mission/state", latched)
        # 판단 하나하나의 전체 기록. 화면의 "LLM 판단" 탭에서 본다
        self.decision_pub = self.create_publisher(
            String, "/mission/decision",
            QoSProfile(depth=100,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # 미션 시계: 첫 판단부터 재고, 웹 일시정지 시간은 뺀다
        self.mission_started = None
        self.paused_total_s = 0.0
        self.pause_started = None
        # 웹 화면의 일시정지·재개·미션 정지
        self.paused = False
        self.stop_requested = False
        self.create_subscription(
            String, "/mission/control", self.on_control, 10)
        # 가짜 로봇이 낸 문제 (실물 로봇에는 없는 토픽이라 그때는 저장 안 됨)
        self.create_subscription(
            String, "/fake/problem",
            lambda message: self.save("problem.json",
                                      json.loads(message.data)),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # 사람이 웹에서 직접 보낸 명령: 상태에 반영하고 "사용자" 판단으로 남긴다
        self.create_subscription(
            String, "/mission/user_command", self.on_user_command, 10)

        # 판단 주체: 둘 중 하나만 만든다. 미리 판단은 LLM일 때만
        self.script = self.llm = self.prefetcher = None
        if self.decider == "script":
            self.script = ScriptDecider(
                self.get_parameter("script_file").value)
            self.model, self.effort, sampling = "script", None, {}
        else:
            self.llm = LlmDecider(
                self.model, self.effort, reasoning_summary, sampling,
                self.get_parameter("prompt_file").value,
                self.get_parameter("arena_file").value,
                self.get_logger(), self.log, self.publish_decision)
            if self.get_parameter("prefetch").value:
                self.prefetcher = Prefetcher(
                    self.llm.llm_decide, self.publish_decision, self.log)
        self.save("meta.json", {
            "decider": self.decider, "model": self.model,
            "effort": self.effort, "guard": self.guard,
            "mission_time_s": self.mission_time_s,
            "repeat_limit": self.repeat_limit,
            "reasoning_summary": reasoning_summary,
            "temperature": sampling.get("temperature"),
            "top_p": sampling.get("top_p")})
        self.get_logger().info(
            f"llm_commander decider={self.decider}, model={self.model}, "
            f"effort={self.effort}, sampling={sampling or 'default'}, "
            f"guard={'on' if self.guard else 'off'}, "
            f"logs in {self.mission_log.run_dir}")

    # ---------- 발행 ----------
    def publish_decision(self, record):
        """판단 카드를 발행하고 decisions.json에 합친다 (미리 판단 스레드도 부른다)."""
        record = {"run_id": self.mission_log.run_id, "turn": self.turn,
                  "t": time.time(), "model": self.model,
                  "effort": self.effort, **record}
        with self.mission_log.records_lock:
            self.decision_pub.publish(String(data=json.dumps(
                record, ensure_ascii=False, default=str)))
            self.mission_log.add_decision(record)

    def publish_state(self):
        message = String()
        message.data = json.dumps(
            {"model": self.model, "turn": self.turn, "state": self.state,
             "decision": self.last_decision, "halt": self.halt,
             "plan_history": self.plan_history},
            ensure_ascii=False, default=str)
        self.state_pub.publish(message)

    # ---------- 웹 화면 제어 ----------
    def on_control(self, message):
        if message.data == "pause":
            self.paused = True
            if self.pause_started is None:
                self.pause_started = time.monotonic()
        elif message.data in ("resume", "stop"):
            self.paused = False
            if self.pause_started is not None:
                self.paused_total_s += time.monotonic() - self.pause_started
                self.pause_started = None
            if message.data == "stop":
                self.stop_requested = True
        self.get_logger().info(f"control: {message.data}")

    def on_user_command(self, message):
        command = json.loads(message.data)
        name, args = command["name"], command["args"]
        result = command["result"]
        # 로봇이 실제로 움직였으니 중계 노드 상태도 맞춘다
        apply_result(self.state, name, args, result)
        self.log("user", {"name": name, "args": args, "result": result,
                          "phase": self.state["phase"]})
        status = "OK" if result.get("success") else result.get("error")
        self.get_logger().info(
            f"[{self.turn}] user {name} "
            f"{json.dumps(args, ensure_ascii=False)} -> "
            f"{status} (phase={self.state['phase']})")
        self.publish_decision({
            "decider": "user", "model": None, "effort": None, "name": name,
            "args": args,
            "outcome": {"success": bool(result.get("success")),
                        "error": result.get("error"), "result": result,
                        "phase_after": self.state["phase"]}})
        self.publish_state()

    def wait_while_paused(self):
        """일시정지면 재개나 정지가 올 때까지 기다린다. 정지가 요청되면 False."""
        rclpy.spin_once(self, timeout_sec=0)
        while self.paused and not self.stop_requested and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        return not self.stop_requested

    def time_left(self):
        """미션 남은 시간(초). 제한을 껐거나 아직 시작 전이면 None."""
        if not self.mission_time_s or self.mission_started is None:
            return None
        now = time.monotonic()
        paused = self.paused_total_s + (
            now - self.pause_started if self.pause_started is not None else 0)
        return self.mission_time_s - (now - self.mission_started - paused)

    # ---------- 로봇 통신 ----------
    def call_robot(self, name, args):
        if not self.robot.wait_for_server(timeout_sec=5.0):
            return {"success": False, "error": "ROBOT_UNREACHABLE"}

        goal = RobotCommand.Goal(name=name, args_json=json.dumps(args))
        send_future = self.robot.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self.log(
                "feedback", {"name": name, "phase": feedback.feedback.phase}))
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {"success": False, "error": "GOAL_REJECTED"}

        result_future = goal_handle.get_result_async()
        limit, waited = ACTION_TIMEOUT_S.get(name, DEFAULT_TIMEOUT_S), 0.0
        while not result_future.done():
            rclpy.spin_until_future_complete(
                self, result_future, timeout_sec=0.5)
            if not self.paused:  # 일시정지 중인 시간은 대기 시간에 넣지 않는다
                waited += 0.5
            if waited > limit:
                goal_handle.cancel_goal_async()
                return {"success": False, "error": "TIMEOUT"}
        return json.loads(result_future.result().result.result_json)

    # ---------- 판단 ----------
    def decide(self):
        if self.mission_started is None:
            self.mission_started = time.monotonic()
        left = self.time_left()
        self.state["meta"] = {
            "seq": self.turn, "stamp": stamp_now(),
            "time_left_s": None if left is None else round(left)}
        if self.script is not None:
            return self.decide_script()
        # 이동하는 동안 미리 받아 둔 판단 (가정이 맞았을 때만)
        decided = None
        if self.prefetcher is not None:
            decided = self.prefetcher.take_prefetch(self.state, self.turn)
        if decided is None:
            # 이번 판단에 보낸 상태 그대로 기록한다
            sent_state = copy.deepcopy(self.state)
            decided = self.llm.llm_decide(sent_state)
        name, args, plan, record = decided
        self.set_plan(plan)
        self.publish_decision({**record, "decider": "ai", "args": args,
                               "plan": plan})
        self.last_decision = {
            "model": self.model, "name": name, "args": args, "plan": plan,
            "calcs": record.get("calcs"),
            "prefetched": record.get("prefetched", False),
            "text": " ".join(record.get("text") or []),
            "reasoning": " ".join(record.get("reasoning") or []),
            "latency_s": record["latency_s"]}
        return name, args

    def decide_script(self):
        """고정 행동 목록의 다음 행동을 계획·판단 카드와 함께 낸다."""
        name, args, plan, note = self.script.decide_script(self.state)
        sent_state = copy.deepcopy(self.state)
        self.set_plan(plan)
        self.publish_decision({"decider": "rule", "input_state": sent_state,
                               "name": name, "args": args, "text": [note],
                               "plan": plan})
        self.last_decision = {"model": self.model, "name": name, "args": args,
                              "text": note, "reasoning": "", "latency_s": 0,
                              "plan": plan}
        return name, args

    def set_plan(self, plan):
        """계획을 다음 판단에 돌려줄 memory.plan에 두고, 새·바뀐 계획은 이력에 남긴다."""
        if not plan:
            return
        previous = self.state["memory"]["plan"]
        self.state["memory"]["plan"] = {
            "turn": self.turn, "assignment": plan.get("assignment"),
            "route": plan.get("route"), "reason": plan.get("reason")}
        if previous is None or plan.get("change"):
            self.plan_history.append({"turn": self.turn, "plan": plan})

    # ---------- 실행 ----------
    def run_tool(self, name, args):
        state = self.state
        if name not in TOOL_NAMES:
            result = {"success": False, "error": "UNKNOWN_TOOL"}
        else:
            try:
                # 시간·반복 한도, 재방문, 나눠 놓기는 guard와 관계없이 막는다
                # (시간은 mission_time_s:=0, 반복은 repeat_limit:=0으로 끈다)
                left = self.time_left()
                check_time(name, args, left)
                check_repeat(state, name, args, left, self.repeat_limit)
                check_revisit(state, name, args)
                if self.guard:
                    check_location(state, name)
                if name == "UNLOAD":
                    check_unload(state, args)
            except ToolError as error:
                result = {"success": False, "error": "REJECTED",
                          "message": str(error)}
            else:
                # 로봇이 움직이는 동안 다음 판단을 미리 받아 둔다
                if self.prefetcher is not None:
                    self.prefetcher.start_prefetch(
                        state, self.turn, name, args)
                result = self.call_robot(name, args)
                # 로봇 서버가 약속(contract.py)과 다른 결과를 내도 상태를 망치지 않게 본다
                result_errors = check_result(name, args, result)
                if result_errors:
                    self.get_logger().error(
                        f"{name} result breaks contract.py: {result_errors}")
                    result = {"success": False, "error": "BAD_RESULT",
                              "contract_errors": result_errors,
                              "raw_result": result}

        apply_result(state, name, args, result)
        self.publish_decision({"outcome": {
            "success": bool(result.get("success")),
            "error": result.get("error"), "result": result,
            "phase_after": state["phase"]}})
        self.log("tool", {"name": name, "args": args, "result": result,
                          "phase": state["phase"]})
        status = "OK" if result.get("success") else result.get("error")
        self.get_logger().info(
            f"[{self.turn}] {name} {json.dumps(args, ensure_ascii=False)} -> "
            f"{status} (phase={state['phase']})")

        world = state["world"]
        if name == "READ_MISSION_INPUT" and result.get("success"):
            self.save("signal.json", {"barcode": world["barcode"],
                                      "signal": world["signal"]})
        if name == "VERIFY_LOCATION" and result.get("success"):
            self.save("verification.json",
                      {location: box["verified"]
                       for location, box in world["locations"].items()})

    def run(self):
        try:
            self.loop()
        except KeyboardInterrupt:
            self.halt = "CTRL_C"
            raise
        except Exception as error:  # 예상 못 한 오류(설정 오류 포함)도 기록은 남긴다
            self.halt = f"ERROR {error!r}"
            raise
        finally:
            # Ctrl+C로 ROS가 먼저 내려가도 기록이 남게 파일을 먼저 저장한다
            self.save("final_state.json",
                      {"halt": self.halt, "state": self.state})
            self.mission_log.write_report(self.state, self.model, self.effort,
                                          self.guard, self.halt)
            try:
                self.publish_state()
            except Exception:
                pass
            self.get_logger().info(
                f"mission finished: phase={self.state['phase']}, "
                f"halt={self.halt}, logs in {self.mission_log.run_dir}")

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
            except LlmUnavailable as error:
                self.halt = str(error)
                if self.halt == "SCRIPT_END":
                    self.get_logger().info("fixed actions finished")
                else:
                    self.get_logger().error(
                        f"LLM unavailable, mission halted: {error}")
                self.log("halt", {"reason": self.halt})
                break

            self.publish_state()
            self.run_tool(name, args)
            if self.stop_requested:
                self.halt = "STOPPED_BY_USER"
                break
            # 시간이 지나 HOME에 돌아왔으면 남은 일이 있어도 끝낸다
            left = self.time_left()
            if (left is not None and left <= 0
                    and self.state["robot"]["home_stopped"]):
                self.halt = "TIME_UP"
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
