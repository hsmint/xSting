"""backend:=ros 실제 로봇(또는 시뮬레이터) 동작: Nav2 주행, 나머지 행동은 아직 구현 전."""
import math
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from ..core.contract import COMMANDS, Canceled


class RosBackend:
    def __init__(self, node, arena):
        self.node = node
        node.declare_parameter("nav_timeout_s", 120.0)
        self.nav_timeout_s = node.get_parameter("nav_timeout_s").value
        self.frame_id, self.places = arena["frame_id"], arena["places"]
        self._nav_handle = None
        self.nav = ActionClient(node, NavigateToPose, "navigate_to_pose",
                                callback_group=node.callback_group)
        # 아직 없는 행동(_ros_LOAD 등)은 NOT_IMPLEMENTED로 답한다
        self.handlers = {
            name: getattr(self, f"_ros_{name}", self._not_implemented)
            for name in COMMANDS}

    def stop(self):
        pass

    def _not_implemented(self, args, goal_handle):
        return {"success": False, "error": "NOT_IMPLEMENTED"}

    # ---------- 주행 (Nav2) ----------
    def _nav_goal(self, place):
        approach = self.places.get(place, {}).get("approach")
        if approach is None:
            return None
        x_mm, y_mm, yaw_deg = approach
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x_mm / 1000.0
        goal.pose.pose.position.y = y_mm / 1000.0
        yaw = math.radians(yaw_deg)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        return goal

    def _wait_future(self, future, timeout_s, goal_handle=None):
        """콜백 안이라 spin_until_future_complete 대신 다른 스레드가 채우길 기다린다."""
        start = time.monotonic()
        while not future.done():
            if goal_handle is not None and goal_handle.is_cancel_requested:
                raise Canceled()
            if time.monotonic() - start > timeout_s:
                return False
            time.sleep(0.02)
        return True

    def _ros_GOTO(self, args, goal_handle):
        node, place = self.node, args["t"]
        goal = self._nav_goal(place)
        if goal is None:
            return {"success": False, "error": "APPROACH_NOT_SET",
                    "message": f"arena.yaml places.{place}.approach"}
        if not self.nav.wait_for_server(timeout_sec=5.0):
            return {"success": False, "error": "NAV_UNAVAILABLE"}

        node.feedback(goal_handle, "planning")
        send_future = self.nav.send_goal_async(
            goal, feedback_callback=lambda nav_feedback: node.feedback(
                goal_handle,
                f"moving {nav_feedback.feedback.distance_remaining:.2f} m"))
        if (not self._wait_future(send_future, 5.0, goal_handle)
                or not send_future.result().accepted):
            return {"success": False, "error": "NAV_REJECTED"}
        self._nav_handle = send_future.result()

        result_future = self._nav_handle.get_result_async()
        try:
            done = self._wait_future(result_future, self.nav_timeout_s,
                                     goal_handle)
        except Canceled:
            self._nav_handle.cancel_goal_async()
            raise
        finally:
            handle, self._nav_handle = self._nav_handle, None
        if not done:
            handle.cancel_goal_async()
            return {"success": False, "error": "NAV_TIMEOUT",
                    "robot": {"here": "UNKNOWN"}}
        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            return {"success": False, "error": "NAV_FAILED", "status": status,
                    "robot": {"here": "UNKNOWN"}}

        # TODO: 도착 자리를 TF(map → base_link)로 읽어 robot.pose로 넣기
        result = {"success": True, "robot": {"here": place}}
        if place == "HOME":
            node.feedback(goal_handle, "holding_3s")
            node.wait(goal_handle, 3.2)
            # TODO: 차체 전체가 출발 구역 안에 있는지 TF와 footprint로 확인
            result.update(in_zone=True, stopped_s=3.2)
        return result

    # ---------- 아직 구현 전 ----------
    # TODO: _ros_READ_MISSION_INPUT  D435 영상 → Code128 + 신호 점멸 감지
    #       (못 찾으면 한 바퀴 돌며 찾기, rules.signal_errors에 걸리면 다음 주기
    #        다시 읽기, 처음 포함 3번 실패면 PERCEPTION_FAILED. contract.py 참고)
    # TODO: _ros_LOAD  AprilTag·모양 판정 → 팔로 집어 적재함 빈 slot에
    # TODO: _ros_UNLOAD  items 순서대로 하차 cell에 놓고 2초 안정 확인
    # TODO: _ros_VERIFY_LOCATION  9칸 새 관측 → items·extra·cells
