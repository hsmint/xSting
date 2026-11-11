"""로봇 명령 action 서버: 명령을 받아 백엔드(fake 또는 ros)로 실행하고 결과를 돌려준다."""
import json
import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from xsting_interfaces.action import RobotCommand

from ..core.contract import Canceled, check_args, check_result


class RobotCommandServer(Node):
    def __init__(self):
        super().__init__("robot_command_server")
        share = get_package_share_directory("xsting_mission")
        self.declare_parameter("backend", "fake")
        # 장소·벽·장애물·로봇 크기
        self.declare_parameter(
            "arena_file", os.path.join(share, "config", "arena.yaml"))

        self.backend_name = self.get_parameter("backend").value
        arena_file = self.get_parameter("arena_file").value
        with open(arena_file, encoding="utf-8") as file:
            arena = yaml.safe_load(file)

        self.callback_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._busy = False

        # 실제 로봇에서는 가짜 로봇 코드를 불러오지 않는다
        if self.backend_name == "fake":
            from ..fake_robot.fake_backend import FakeBackend
            self.backend = FakeBackend(self, arena, share)
        elif self.backend_name == "ros":
            from .ros_backend import RosBackend
            self.backend = RosBackend(self, arena)
        else:
            raise ValueError(f"unknown backend: {self.backend_name}")

        self.server = ActionServer(
            self, RobotCommand, "robot_command",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=self.callback_group,
        )
        self.get_logger().info(
            f"robot_command_server ready (backend={self.backend_name})")

    # ---------- action 서버 ----------
    def on_goal(self, goal_request):
        """동작 중에 들어온 명령은 거절한다."""
        with self._lock:
            if self._busy:
                self.get_logger().warning(f"busy, reject {goal_request.name}")
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def is_busy(self):
        return self._busy

    def execute(self, goal_handle):
        name = goal_handle.request.name
        try:
            args = json.loads(goal_handle.request.args_json or "{}")
            self.get_logger().info(f"command: {name} {args}")
            handler = self.backend.handlers.get(name)
            arg_errors = check_args(name, args)
            if handler is None:
                outcome = {"success": False, "error": "UNKNOWN_COMMAND"}
            elif arg_errors:
                outcome = {"success": False, "error": "BAD_ARGS",
                           "message": "; ".join(arg_errors)}
            else:
                outcome = handler(args, goal_handle)
                # 약속과 다른 결과는 실패로 바꿔 노트북이 잘못된 값을 쓰지 않게 한다
                result_errors = check_result(name, args, outcome)
                if result_errors:
                    self.get_logger().error(
                        f"{name} result breaks contract.py: {result_errors}")
                    outcome = {"success": False, "error": "BAD_RESULT",
                               "contract_errors": result_errors,
                               "raw_result": outcome}
        except Canceled:
            outcome = {"success": False, "error": "CANCELED"}
        except Exception as error:  # 예외가 나도 노트북에는 실패 결과를 돌려준다
            self.get_logger().error(f"{name} failed: {error!r}")
            outcome = {"success": False, "error": "EXCEPTION",
                       "message": repr(error)}
        finally:
            with self._lock:
                self._busy = False

        result = RobotCommand.Result()
        result.success = bool(outcome.get("success"))
        result.result_json = json.dumps(outcome, ensure_ascii=False)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self.get_logger().info(f"result: {name} {result.result_json}")
        return result

    # ---------- 백엔드가 쓰는 도우미 ----------
    def feedback(self, goal_handle, phase):
        feedback_msg = RobotCommand.Feedback()
        feedback_msg.phase = phase
        goal_handle.publish_feedback(feedback_msg)

    def wait(self, goal_handle, seconds):
        """취소를 확인하며 노드 시계(sim time 따름)로 기다린다."""
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.get_clock().now().nanoseconds < end:
            if goal_handle.is_cancel_requested:
                raise Canceled()
            time.sleep(0.02)


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
        node.backend.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
