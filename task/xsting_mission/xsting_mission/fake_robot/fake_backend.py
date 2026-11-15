"""backend:=fake 가짜 로봇: 가짜 경기장으로 명령을 흉내 내고 웹 화면·지난 판 다시 실행을 제공한다."""
import json
import os
import threading
import time
import uuid

import yaml
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from unique_identifier_msgs.msg import UUID

from xsting_interfaces.action import RobotCommand

from ..core.contract import COMMANDS, Canceled
from . import replay
from .fake_world import FakeWorld


def _latched_qos(depth):
    """늦게 붙은 구독자도 마지막 메시지를 받는 QoS."""
    return QoSProfile(depth=depth,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class FakeBackend:
    def __init__(self, node, arena, share):
        self.node = node
        node.declare_parameter(
            "scenario_file",
            os.path.join(share, "config", "fake_scenario.yaml"))
        # 가짜 동작 배속 (1.0 = 실제 속도)
        node.declare_parameter("fake_time_scale", 3.0)
        node.declare_parameter("fake_speed_mps", 0.3)
        # 웹 화면 포트, 0이면 끔
        node.declare_parameter("view_port", 8765)
        # 다시 실행할 판을 찾는 곳
        node.declare_parameter(
            "replay_log_dir", os.path.expanduser("~/xsting_logs"))

        scenario_file = node.get_parameter("scenario_file").value
        with open(scenario_file, encoding="utf-8") as file:
            scenario = yaml.safe_load(file)
        self.world = FakeWorld(
            scenario, arena,
            time_scale=node.get_parameter("fake_time_scale").value,
            speed_mps=node.get_parameter("fake_speed_mps").value)
        self.handlers = {
            name: (lambda args, goal_handle, command=name:
                   self._run(command, args, goal_handle))
            for name in COMMANDS}

        # 노트북 판단기의 MissionState와 판단 내용을 화면에 함께 보여 준다
        node.create_subscription(String, "/mission/state",
                                 self.on_mission_state, _latched_qos(1))
        node.create_subscription(String, "/mission/decision",
                                 self.on_mission_decision, _latched_qos(100))
        # 중계 노드가 판마다 problem.json으로 남기게 한다 (다시 실행용)
        self.problem_pub = node.create_publisher(
            String, "/fake/problem", _latched_qos(1))
        self.publish_problem()
        self.replay_log_dir = node.get_parameter("replay_log_dir").value
        self._replay_stop = False
        self._replay_thread = None
        # goal id → "web" | "replay" (나머지는 중계 노드)
        self.goal_sources = {}

        self.view = None
        port = node.get_parameter("view_port").value
        if port:
            self._setup_view(share, port)

    def _setup_view(self, share, port):
        """웹 화면과 화면이 쓰는 action 클라이언트·토픽을 준비한다."""
        node = self.node
        # 화면의 "명령 보내기"도 중계 노드와 똑같이 ROS action으로 보낸다
        self.web_client = ActionClient(
            node, RobotCommand, "robot_command",
            callback_group=node.callback_group)
        # 일시정지·미션 정지를 중계 노드에도 알린다
        self.control_pub = node.create_publisher(
            String, "/mission/control", 10)
        # 화면에서 보낸 명령·결과를 중계 노드가 "사용자"로 기록하게 알린다
        self.user_cmd_pub = node.create_publisher(
            String, "/mission/user_command", 10)
        from .sim_view import ViewServer
        self.view = ViewServer(
            self.world, os.path.join(share, "web"), port, node.get_logger(),
            send_command=self.send_from_web, is_busy=self.is_busy,
            control=self.web_control, reset=self.reset_world,
            list_runs=lambda: replay.list_runs(self.replay_log_dir),
            start_replay=self.start_replay, view_run=self.view_run,
            name_run=lambda run_id, name: replay.set_run_name(
                self.replay_log_dir, run_id, name),
            delete_run=self.delete_run)

    def stop(self):
        if self.view:
            self.view.stop()

    # ---------- 명령 실행 ----------
    def _run(self, name, args, goal_handle):
        world = self.world

        def feedback(phase):
            world.set_phase(phase)
            self.node.feedback(goal_handle, phase)

        source = self.goal_sources.get(bytes(goal_handle.goal_id.uuid),
                                       "relay")
        world.begin(name, args, source)
        outcome = {"success": False, "error": "CANCELED"}
        try:
            outcome = world.run(name, args,
                                lambda: goal_handle.is_cancel_requested,
                                feedback)
            return outcome
        finally:
            world.end(name, args, outcome)

    # ---------- 중계 노드 토픽 ----------
    def on_mission_state(self, message):
        with self.world.lock:
            self.world.commander = json.loads(message.data)

    def on_mission_decision(self, message):
        record = json.loads(message.data)
        # 화면 명령은 add_user_card가 이미 남겼으니 중계 노드가 되돌려 보낸 것은 버린다
        if record.get("decider") != "user":
            self.world.add_decision(record)

    def publish_problem(self):
        self.problem_pub.publish(String(
            data=json.dumps(self.world.problem(), ensure_ascii=False)))

    # ---------- 화면 제어 ----------
    def web_control(self, command):
        """화면의 일시정지/재개/미션 정지를 가짜 로봇과 중계 노드에 함께 적용한다."""
        world = self.world
        with world.lock:
            if command == "pause":
                world.paused = True
            elif command == "resume":
                world.paused = False
            elif command == "stop":
                world.paused = False
                # 진행 중인 명령을 취소한다
                world.abort = world.command is not None
                self._replay_stop = True
        self.control_pub.publish(String(data=command))
        self.node.get_logger().info(f"web control: {command}")

    def is_busy(self):
        return self.node.is_busy() or (self._replay_thread is not None
                                       and self._replay_thread.is_alive())

    def reset_world(self, problem=None):
        self.world.reset(problem)
        with self.world.lock:  # 새 문제이니 지난 판단과 명령 결과도 화면에서 지운다
            self.world.decisions = []
            self.world.web_last = None
            self.world.replay = None
        self.publish_problem()

    # ---------- 화면에서 보낸 명령 ----------
    def send_goal(self, name, args, source):
        """중계 노드와 같은 ROS action 경로로 명령을 보내고 결과를 기다린다."""
        goal_id = uuid.uuid4().bytes
        # 실행 콜백이 누가 보낸 명령인지 알 수 있게 먼저 등록
        self.goal_sources[goal_id] = source
        try:
            if not self.web_client.wait_for_server(timeout_sec=2.0):
                return {"success": False, "error": "SERVER_UNAVAILABLE"}
            goal = RobotCommand.Goal(name=name, args_json=json.dumps(args))
            future = self.web_client.send_goal_async(
                goal, goal_uuid=UUID(uuid=list(goal_id)))
            while not future.done():
                time.sleep(0.02)
            handle = future.result()
            if handle is None or not handle.accepted:
                return {"success": False, "error": "GOAL_REJECTED",
                        "message": "로봇이 다른 명령을 실행 중"}
            result_future = handle.get_result_async()
            while not result_future.done():
                time.sleep(0.05)
            return json.loads(result_future.result().result.result_json)
        finally:
            self.goal_sources.pop(goal_id, None)

    def send_from_web(self, name, args):
        """화면이 멈추지 않게 별도 스레드에서 명령을 보낸다."""
        def worker():
            result = self.send_goal(name, args, "web")
            with self.world.lock:
                self.world.web_last = {"t": time.time(), "name": name,
                                       "args": args, "result": result}
            # 실제로 실행된 명령만 남긴다
            if result.get("error") in ("GOAL_REJECTED", "SERVER_UNAVAILABLE"):
                return
            self.add_user_card(name, args, result)
            self.user_cmd_pub.publish(String(data=json.dumps(
                {"t": time.time(), "name": name, "args": args,
                 "result": result}, ensure_ascii=False)))

        with self.world.lock:
            self.world.web_last = {"t": time.time(), "name": name,
                                   "args": args, "result": None}
        threading.Thread(target=worker, daemon=True).start()

    def add_user_card(self, name, args, result):
        """화면 명령을 판단 탭에 "사용자" 카드로 남긴다 (중계 노드 판에 잇거나 "web"으로)."""
        world = self.world
        with world.lock:
            last = world.decisions[-1] if world.decisions else None
            # 지난 판 보기·다시 실행 화면이면 비우고 "web"으로 새로 쌓는다
            if last is not None and not str(last["run_id"]).startswith(
                    ("log:", "replay:")):
                run_id, turn = last["run_id"], last["turn"]
            else:
                run_id, turn = "web", 0
            if run_id == "web":
                turn = sum(decision["run_id"] == "web"
                           for decision in world.decisions) + 1
            phase = (((world.commander or {}).get("state") or {}).get("phase")
                     if run_id != "web" else None)
        world.add_decision({
            "run_id": run_id, "turn": turn, "t": time.time(),
            "decider": "user", "model": None, "name": name, "args": args,
            "outcome": {"success": bool(result.get("success")),
                        "error": result.get("error"), "result": result,
                        "phase_after": phase}})

    # ---------- 지난 판 보기·지우기·다시 실행 ----------
    def view_run(self, run_id):
        """다시 실행하지 않고 그 판의 판단 기록 전체를 판단 탭에 펼친다."""
        _, records = replay.load_run(self.replay_log_dir, run_id)
        with self.world.lock:
            self.world.decisions = [{**record, "run_id": f"log:{run_id}"}
                                    for record in records]

    def delete_run(self, run_id):
        """지난 판 기록을 지운다 (진행 중인 판과 다시 실행 중인 판은 안 됨)."""
        world = self.world
        with world.lock:
            live = world.decisions[0]["run_id"] if world.decisions else None
            running = (world.commander is not None
                       and not world.commander.get("halt")
                       and world.commander["state"]["phase"] != "DONE")
            replaying = (world.replay is not None
                         and not world.replay.get("done"))
        if ((live == run_id and running)
                or (replaying and world.replay["run_id"] == run_id)):
            raise ValueError("지금 진행 중인 판이라 지울 수 없음")
        replay.delete_run(self.replay_log_dir, run_id)
        with world.lock:  # 지운 판을 보고 있었으면 화면에서도 비운다
            if (world.decisions
                    and world.decisions[0]["run_id"] == f"log:{run_id}"):
                world.decisions = []
        self.node.get_logger().info(f"deleted run {run_id}")

    def start_replay(self, run_id):
        problem, records = replay.load_run(self.replay_log_dir, run_id)
        if problem.get("legacy"):
            raise ValueError(
                "예전 행동 이름(read_barcode 등)으로 돌린 규칙 판이라 "
                "지금 로봇에 다시 실행할 수 없음 (보기만 가능)")
        self.reset_world(problem)
        world = self.world
        with world.lock:
            world.decisions = []
            world.replay = {
                "run_id": run_id, "turn": 0, "total": len(records),
                "done": False,
                "reconstructed": bool(problem.get("reconstructed")),
                "mismatch": 0}
        self._replay_stop = False
        self._replay_thread = threading.Thread(
            target=self._replay, args=(run_id, records), daemon=True)
        self._replay_thread.start()
        self.node.get_logger().info(
            f"replay {run_id}: {len(records)} decisions")

    def _replay(self, run_id, records):
        world, replay_run_id = self.world, f"replay:{run_id}"
        try:
            for record in records:
                # LLM이 생각하던 시간만큼 기다린다 (배속·일시정지 적용)
                world.sleep(record.get("latency_s") or 0,
                            lambda: self._replay_stop)
                card = {key: value for key, value in record.items()
                        if key != "outcome"}
                world.add_decision({**card, "run_id": replay_run_id})
                with world.lock:
                    world.replay["turn"] = record["turn"]

                # 로봇에 보내지 않았던 판단은 기록된 결과만 다시 보여 준다
                recorded = record.get("outcome")
                if (recorded is None or record.get("error")
                        or recorded.get("error") in replay.NOT_SENT):
                    if recorded:
                        world.add_decision({"run_id": replay_run_id,
                                            "turn": record["turn"],
                                            "outcome": recorded})
                    continue

                result = self.send_goal(record["name"],
                                        record.get("args", {}), "replay")
                same = (bool(result.get("success")) == recorded["success"]
                        and result.get("error") == recorded.get("error"))
                if not same:
                    with world.lock:
                        world.replay["mismatch"] += 1
                world.add_decision({
                    "run_id": replay_run_id, "turn": record["turn"],
                    "outcome": {
                        "success": bool(result.get("success")),
                        "error": result.get("error"), "result": result,
                        "phase_after": recorded.get("phase_after"),
                        "recorded": None if same else recorded}})
                if self._replay_stop:
                    break
        except Canceled:
            pass
        finally:
            with world.lock:
                world.replay["done"] = True
            self.node.get_logger().info(f"replay {run_id} finished")
