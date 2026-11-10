"""로봇 쪽 action 서버. 중계 노드의 명령(READ_MISSION_INPUT, GOTO, LOAD, UNLOAD, VERIFY_LOCATION)을 받아 실행하고 결과를 돌려준다.

backend:=fake  가짜 경기장(fake_world)으로 응답한다. 시뮬레이터나 로봇 없이 노트북 판단 루프를 시험할 때 쓴다.
               브라우저에서 http://localhost:8765 를 열면 지도 위에서 로봇이 움직이는 모습이 보인다(view_port).
backend:=ros   ROS 2 인터페이스로 실제 동작한다. 시뮬레이터와 실물 로봇이 같은 코드를 쓴다
               (시뮬레이터면 use_sim_time:=true). 지금은 GOTO만 구현되어 있다.

모든 결과에는 MissionState.robot에 들어갈 로봇 상태("robot": {here, status, ...})를 함께 싣는다.
"""
import json
import math
import os
import threading
import time
import uuid

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from unique_identifier_msgs.msg import UUID

from xsting_interfaces.action import RobotCommand

from . import replay
from .fake_world import Canceled, FakeWorld

COMMANDS = ("READ_MISSION_INPUT", "GOTO", "LOAD", "UNLOAD", "VERIFY_LOCATION")


class RobotCommandServer(Node):
    def __init__(self):
        super().__init__("robot_command_server")
        share = get_package_share_directory("xsting_mission")
        self.declare_parameter("backend", "fake")
        self.declare_parameter("scenario_file", os.path.join(share, "config", "fake_scenario.yaml"))
        self.declare_parameter("arena_file", os.path.join(share, "config", "arena.yaml"))  # 장소·벽·장애물·로봇 크기
        self.declare_parameter("fake_time_scale", 3.0)  # 가짜 동작 배속 (1.0 = 실제 속도)
        self.declare_parameter("fake_speed_mps", 0.3)  # 가짜 로봇 주행 속도
        self.declare_parameter("view_port", 8765)  # 웹 화면 포트, 0이면 끔
        self.declare_parameter("replay_log_dir", os.path.expanduser("~/xsting_logs"))  # 다시 실행할 판을 찾는 곳
        self.declare_parameter("nav_timeout_s", 120.0)

        self.backend = self.get_parameter("backend").value
        self.nav_timeout_s = self.get_parameter("nav_timeout_s").value
        with open(self.get_parameter("arena_file").value, encoding="utf-8") as f:
            arena = yaml.safe_load(f)
        self.frame_id, self.places = arena["frame_id"], arena["places"]

        self.cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._busy = False
        self._nav_handle = None

        self.view = None
        if self.backend == "fake":
            with open(self.get_parameter("scenario_file").value, encoding="utf-8") as f:
                scenario = yaml.safe_load(f)
            self.world = FakeWorld(scenario, arena,
                                   time_scale=self.get_parameter("fake_time_scale").value,
                                   speed_mps=self.get_parameter("fake_speed_mps").value)
            # 노트북 판단기의 MissionState와 판단 내용을 화면에 함께 보여 준다
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, "/mission/state", self.on_mission_state, qos)
            self.create_subscription(String, "/mission/decision", self.on_mission_decision,
                                     QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            # 지금 문제를 알려 중계 노드가 판마다 problem.json으로 남기게 한다 (다시 실행용)
            self.problem_pub = self.create_publisher(
                String, "/fake/problem", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.publish_problem()
            self.replay_log_dir = self.get_parameter("replay_log_dir").value
            self._replay_stop = False
            self._replay_thread = None
            port = self.get_parameter("view_port").value
            if port:
                # 화면의 "명령 보내기"도 중계 노드와 똑같이 ROS action으로 보낸다
                self.web_client = ActionClient(self, RobotCommand, "robot_command", callback_group=self.cb)
                self.goal_sources = {}  # goal id → "web" | "replay" (나머지는 중계 노드)
                # 일시정지·미션 정지를 중계 노드에도 알린다
                self.control_pub = self.create_publisher(String, "/mission/control", 10)
                # 화면에서 직접 보낸 명령과 결과를 중계 노드에 알려 상태·판단 기록에 "사용자"로 남기게 한다
                self.user_cmd_pub = self.create_publisher(String, "/mission/user_command", 10)
                from .sim_view import ViewServer
                self.view = ViewServer(self.world, os.path.join(share, "web"), port, self.get_logger(),
                                       send_command=self.send_from_web, is_busy=self.is_busy,
                                       control=self.web_control, reset=self.reset_world,
                                       list_runs=lambda: replay.list_runs(self.replay_log_dir),
                                       start_replay=self.start_replay, view_run=self.view_run,
                                       name_run=lambda rid, name: replay.set_run_name(self.replay_log_dir, rid, name),
                                       delete_run=self.delete_run)
        elif self.backend == "ros":
            from nav2_msgs.action import NavigateToPose
            self.NavigateToPose = NavigateToPose
            self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=self.cb)
        else:
            raise ValueError(f"unknown backend: {self.backend}")

        # backend에 따라 가짜 경기장 또는 _ros_xxx를 연결한다
        if self.backend == "fake":
            self.handlers = {name: (lambda args, gh, n=name: self._fake(n, args, gh)) for name in COMMANDS}
        else:
            self.handlers = {name: getattr(self, f"_ros_{name}", self._not_implemented) for name in COMMANDS}
        self.server = ActionServer(
            self, RobotCommand, "robot_command",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=self.cb,
        )
        self.get_logger().info(f"robot_command_server ready (backend={self.backend})")

    # ---------- action 서버 공통 ----------
    def on_goal(self, goal_request):
        # 동작 중에 들어온 명령은 거절한다
        with self._lock:
            if self._busy:
                self.get_logger().warning(f"busy, reject {goal_request.name}")
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        name = goal_handle.request.name
        try:
            args = json.loads(goal_handle.request.args_json or "{}")
            self.get_logger().info(f"command: {name} {args}")
            handler = self.handlers.get(name)
            if handler is None:
                out = {"success": False, "error": "UNKNOWN_COMMAND"}
            else:
                out = handler(args, goal_handle)
        except Canceled:
            out = {"success": False, "error": "CANCELED"}
        except Exception as e:  # 예외가 나도 노트북에는 실패 결과를 돌려준다
            self.get_logger().error(f"{name} failed: {e!r}")
            out = {"success": False, "error": "EXCEPTION", "message": repr(e)}
        finally:
            with self._lock:
                self._busy = False

        result = RobotCommand.Result()
        result.success = bool(out.get("success"))
        result.result_json = json.dumps(out, ensure_ascii=False)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self.get_logger().info(f"result: {name} {result.result_json}")
        return result

    def feedback(self, goal_handle, phase):
        fb = RobotCommand.Feedback()
        fb.phase = phase
        goal_handle.publish_feedback(fb)

    def wait(self, goal_handle, seconds):
        """취소를 확인하면서 기다린다. 시뮬레이터의 sim time을 따르도록 노드 시계를 쓴다."""
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.get_clock().now().nanoseconds < end:
            if goal_handle.is_cancel_requested:
                raise Canceled()
            time.sleep(0.02)

    def _not_implemented(self, args, goal_handle):
        return {"success": False, "error": "NOT_IMPLEMENTED"}

    # ---------- backend:=fake ----------
    def on_mission_state(self, msg):
        with self.world.lock:
            self.world.commander = json.loads(msg.data)

    def on_mission_decision(self, msg):
        record = json.loads(msg.data)
        # 화면에서 보낸 명령은 이 서버가 직접 카드로 남긴다(add_user_card). 중계 노드가 되돌려 보낸 것은 겹치니 버린다
        if record.get("decider") != "user":
            self.world.add_decision(record)

    def web_control(self, cmd):
        """화면의 일시정지/재개/미션 정지. 가짜 로봇과 중계 노드에 함께 적용한다."""
        world = self.world
        with world.lock:
            if cmd == "pause":
                world.paused = True
            elif cmd == "resume":
                world.paused = False
            elif cmd == "stop":
                world.paused = False
                world.abort = world.command is not None  # 진행 중인 명령을 취소한다
                self._replay_stop = True
        self.control_pub.publish(String(data=cmd))
        self.get_logger().info(f"web control: {cmd}")

    def is_busy(self):
        return self._busy or (self._replay_thread is not None and self._replay_thread.is_alive())

    def publish_problem(self):
        self.problem_pub.publish(String(data=json.dumps(self.world.problem(), ensure_ascii=False)))

    def reset_world(self, problem=None):
        self.world.reset(problem)
        with self.world.lock:  # 새 문제이니 지난 판단과 명령 결과도 화면에서 지운다
            self.world.decisions = []
            self.world.web_last = None
            self.world.replay = None
        self.publish_problem()

    def send_goal(self, name, args, source):
        """중계 노드와 같은 ROS action 경로로 명령을 보내고 결과를 기다린다 (화면·다시 실행용)."""
        goal_id = uuid.uuid4().bytes
        self.goal_sources[goal_id] = source  # 실행 콜백이 누가 보낸 명령인지 알 수 있게 먼저 등록
        try:
            if not self.web_client.wait_for_server(timeout_sec=2.0):
                return {"success": False, "error": "SERVER_UNAVAILABLE"}
            goal = RobotCommand.Goal(name=name, args_json=json.dumps(args))
            future = self.web_client.send_goal_async(goal, goal_uuid=UUID(uuid=list(goal_id)))
            while not future.done():
                time.sleep(0.02)
            handle = future.result()
            if handle is None or not handle.accepted:
                return {"success": False, "error": "GOAL_REJECTED", "message": "로봇이 다른 명령을 실행 중"}
            rf = handle.get_result_async()
            while not rf.done():
                time.sleep(0.05)
            return json.loads(rf.result().result.result_json)
        finally:
            self.goal_sources.pop(goal_id, None)

    def send_from_web(self, name, args):
        """화면에서 보낸 명령. 결과를 기다리는 동안 화면이 멈추지 않게 별도 스레드에서 보낸다."""
        def worker():
            result = self.send_goal(name, args, "web")
            with self.world.lock:
                self.world.web_last = {"t": time.time(), "name": name, "args": args, "result": result}
            if result.get("error") in ("GOAL_REJECTED", "SERVER_UNAVAILABLE"):  # 실제로 실행된 명령만 남긴다
                return
            self.add_user_card(name, args, result)
            self.user_cmd_pub.publish(String(data=json.dumps(
                {"t": time.time(), "name": name, "args": args, "result": result}, ensure_ascii=False)))

        with self.world.lock:
            self.world.web_last = {"t": time.time(), "name": name, "args": args, "result": None}
        threading.Thread(target=worker, daemon=True).start()

    def add_user_card(self, name, args, result):
        """화면에서 보낸 명령을 LLM 판단 탭에 "사용자" 카드로 남긴다. 중계 노드가 꺼져 있어도 쌓인다.

        지금 보이는 기록이 중계 노드의 판이면 거기에 이어 붙이고, 지난 판 보기·다시 실행 화면이면 비우고 "web"으로 새로 쌓는다.
        """
        world = self.world
        with world.lock:
            last = world.decisions[-1] if world.decisions else None
            if last is not None and not str(last["run_id"]).startswith(("log:", "replay:")):
                run_id, turn = last["run_id"], last["turn"]
            else:
                run_id, turn = "web", 0
            if run_id == "web":
                turn = sum(d["run_id"] == "web" for d in world.decisions) + 1
            phase = ((world.commander or {}).get("state") or {}).get("phase") if run_id != "web" else None
        world.add_decision({"run_id": run_id, "turn": turn, "t": time.time(), "decider": "user",
                            "model": None, "name": name, "args": args,
                            "outcome": {"success": bool(result.get("success")), "error": result.get("error"),
                                        "result": result, "phase_after": phase}})

    # ---------- 지난 판 다시 실행 ----------
    def view_run(self, run_id):
        """다시 실행하지 않고 그 판의 판단 기록 전체를 LLM 판단 탭에 펼친다."""
        _, records = replay.load_run(self.replay_log_dir, run_id)
        with self.world.lock:
            self.world.decisions = [{**r, "run_id": f"log:{run_id}"} for r in records]


    def delete_run(self, run_id):
        """지난 판 기록을 지운다. 지금 진행 중인 판(중계 노드가 쓰는 중)과 다시 실행 중인 판은 지우지 않는다."""
        world = self.world
        with world.lock:
            live = world.decisions[0]["run_id"] if world.decisions else None
            running = world.commander is not None and not world.commander.get("halt") \
                and world.commander["state"]["phase"] != "DONE"
            replaying = world.replay is not None and not world.replay.get("done")
        if (live == run_id and running) or (replaying and world.replay["run_id"] == run_id):
            raise ValueError("지금 진행 중인 판이라 지울 수 없음")
        replay.delete_run(self.replay_log_dir, run_id)
        with world.lock:  # 지운 판을 보고 있었으면 화면에서도 비운다
            if world.decisions and world.decisions[0]["run_id"] == f"log:{run_id}":
                world.decisions = []
        self.get_logger().info(f"deleted run {run_id}")

    def start_replay(self, run_id):
        problem, records = replay.load_run(self.replay_log_dir, run_id)
        if problem.get("legacy"):
            raise ValueError("예전 행동 이름(read_barcode 등)으로 돌린 규칙 판이라 지금 로봇에 다시 실행할 수 없음 (보기만 가능)")
        self.reset_world(problem)
        world = self.world
        with world.lock:
            world.decisions = []
            world.replay = {"run_id": run_id, "turn": 0, "total": len(records), "done": False,
                            "reconstructed": bool(problem.get("reconstructed")), "mismatch": 0}
        self._replay_stop = False
        self._replay_thread = threading.Thread(target=self._replay, args=(run_id, records), daemon=True)
        self._replay_thread.start()
        self.get_logger().info(f"replay {run_id}: {len(records)} decisions")

    def _replay(self, run_id, records):
        world, rid = self.world, f"replay:{run_id}"
        stopped = lambda: self._replay_stop
        try:
            for rec in records:
                # ChatGPT가 생각하던 시간만큼 기다린다 (배속·일시정지 적용)
                world.sleep(rec.get("latency_s") or 0, stopped)
                card = {k: v for k, v in rec.items() if k != "outcome"}
                world.add_decision({**card, "run_id": rid})
                with world.lock:
                    world.replay["turn"] = rec["turn"]
                old = rec.get("outcome")
                if old is None or rec.get("error") or old.get("error") in replay.NOT_SENT:
                    if old:
                        world.add_decision({"run_id": rid, "turn": rec["turn"], "outcome": old})
                    continue
                result = self.send_goal(rec["name"], rec.get("args", {}), "replay")
                same = bool(result.get("success")) == old["success"] and result.get("error") == old.get("error")
                if not same:
                    with world.lock:
                        world.replay["mismatch"] += 1
                world.add_decision({"run_id": rid, "turn": rec["turn"], "outcome": {
                    "success": bool(result.get("success")), "error": result.get("error"), "result": result,
                    "phase_after": old.get("phase_after"), "recorded": None if same else old}})
                if self._replay_stop:
                    break
        except Canceled:
            pass
        finally:
            with world.lock:
                world.replay["done"] = True
            self.get_logger().info(f"replay {run_id} finished")

    def _fake(self, name, args, gh):
        world = self.world

        def feedback(phase):
            world.set_phase(phase)
            self.feedback(gh, phase)

        source = getattr(self, "goal_sources", {}).get(bytes(gh.goal_id.uuid), "relay")
        world.begin(name, args, source)
        out = {"success": False, "error": "CANCELED"}
        try:
            out = world.run(name, args, lambda: gh.is_cancel_requested, feedback)
            return out
        finally:
            world.end(name, args, out)

    # ---------- backend:=ros (시뮬레이터 / 실물) ----------
    def _nav_goal(self, place):
        approach = self.places.get(place, {}).get("approach")
        if approach is None:
            return None
        x_mm, y_mm, yaw_deg = approach
        goal = self.NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x_mm / 1000.0
        goal.pose.pose.position.y = y_mm / 1000.0
        yaw = math.radians(yaw_deg)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        return goal

    def _wait_future(self, future, timeout_s, goal_handle=None):
        # 콜백 안이라 spin_until_future_complete를 못 쓴다. 다른 스레드가 future를 채운다.
        start = time.monotonic()
        while not future.done():
            if goal_handle is not None and goal_handle.is_cancel_requested:
                raise Canceled()
            if time.monotonic() - start > timeout_s:
                return False
            time.sleep(0.02)
        return True

    def _ros_GOTO(self, args, gh):
        place = args["t"]
        goal = self._nav_goal(place)
        if goal is None:
            return {"success": False, "error": "APPROACH_NOT_SET", "message": f"arena.yaml places.{place}.approach"}
        if not self.nav.wait_for_server(timeout_sec=5.0):
            return {"success": False, "error": "NAV_UNAVAILABLE"}

        self.feedback(gh, "planning")
        send_future = self.nav.send_goal_async(
            goal, feedback_callback=lambda fb: self.feedback(
                gh, f"moving {fb.feedback.distance_remaining:.2f} m"))
        if not self._wait_future(send_future, 5.0, gh) or not send_future.result().accepted:
            return {"success": False, "error": "NAV_REJECTED"}
        self._nav_handle = send_future.result()

        result_future = self._nav_handle.get_result_async()
        try:
            done = self._wait_future(result_future, self.nav_timeout_s, gh)
        except Canceled:
            self._nav_handle.cancel_goal_async()
            raise
        finally:
            handle, self._nav_handle = self._nav_handle, None
        if not done:
            handle.cancel_goal_async()
            return {"success": False, "error": "NAV_TIMEOUT", "robot": {"here": "UNKNOWN"}}
        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            return {"success": False, "error": "NAV_FAILED", "status": status, "robot": {"here": "UNKNOWN"}}
        # TODO: 도착한 자리를 TF(map → base_link)로 읽어 robot.pose({x, y (mm), yaw (deg)})로 넣기 (LLM이 다음 경로 판단에 쓴다)
        out = {"success": True, "robot": {"here": place}}
        if place == "HOME":
            self.feedback(gh, "holding_3s")
            self.wait(gh, 3.2)
            # TODO: 차체 전체가 출발 구역 안에 있는지 TF와 footprint로 확인
            out.update(in_zone=True, stopped_s=3.2)
        return out

    # TODO(시뮬레이터 / 실물): 아래는 아직 NOT_IMPLEMENTED
    #   _ros_READ_MISSION_INPUT  앞 D435 컬러 영상 → Code128 디코딩 + 점멸 12회·3초 소등 감지 (신호는 반복됨)
    #   _ros_LOAD                박스의 퍽마다 AprilTag(id·색) + 모양 판정 → 팔 집기 → 적재함 빈 slot에 넣기 (한 번에 모두)
    #                            결과: {"pucks": [{"slot", "puck": {"id","color","shape","conf"}}, ...], "box_remaining"}
    #   _ros_UNLOAD              items 순서대로 적재함에서 꺼내 하차 박스 태그 기준 cell에 놓기 → 2초 안정 확인
    #                            실패하면 그 항목에서 멈추고 {"items": [...], "stopped_at"} 반환
    #   _ros_VERIFY_LOCATION     9칸 새 관측 → {"items": [...ok], "extra": [...], "cells": [...]}

def main():
    rclpy.init()
    node = RobotCommandServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.view:
            node.view.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
