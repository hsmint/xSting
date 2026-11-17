"""backend:=ros 실제 로봇 동작: 행동 하나를 로봇 팀 액션 하나로 보내고, 결과를 규약 형식으로 바꾸고, 에러면 다시 시도한다.

인식부터 동작까지는 로봇 팀 액션이 한다. 형식은 docs/robot_primitives.md (초안, 로봇 팀과 확정 전).
"""
import math
import time

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from xsting_interfaces.action import (LoadBox, ObserveLocation,
                                      ReadMissionInput, RunTask, UnloadPucks)
from xsting_interfaces.msg import ErrorCode, PlaceItem

from ..core.contract import COMMANDS, Canceled
from ..core.rules import LOCATIONS, NUM_CELLS, START_BOXES, signal_errors

# 하위 서버가 목표를 받아 줄 때까지 기다리는 시간(초)
SERVER_WAIT_S = 2.0
# 한 번 시도의 한도(초)와, 다시 시도까지 합한 전체 한도(초).
# 전체 한도는 노트북 대기(node.py ACTION_TIMEOUT_S)보다 짧게 해 먼저 끝내고 결과를 보낸다
ATTEMPT_S = {"GOTO": 170.0, "READ_MISSION_INPUT": 170.0, "LOAD": 150.0,
             "UNLOAD": 400.0, "VERIFY_LOCATION": 45.0}
BUDGET_S = {"GOTO": 170.0, "READ_MISSION_INPUT": 590.0, "LOAD": 170.0,
            "UNLOAD": 410.0, "VERIFY_LOCATION": 110.0}
# 같은 행동을 다시 시도하는 최대 횟수 (처음 포함)
MAX_TRIES = 3
# 남은 시간이 이보다 짧으면 다시 시도하지 않는다
MIN_ATTEMPT_S = 10.0
# HOME 도착 뒤 정지 시간 (출발 구역 3초 정지 + 여유)
HOME_HOLD_S = 3.2

# 로봇 팀 ErrorCode 숫자 → 노트북 에러 코드. 이름이 같으면 그대로 쓴다
_SAME_NAMES = ("ROBOT_FAULT", "INTERNAL_ERROR", "CANCELED", "TIMEOUT",
               "OBSTACLE_BLOCKED", "NO_PROGRESS", "NO_PATH", "NAV_FAILED",
               "LOCALIZATION_LOST", "PERCEPTION_FAILED", "GRASP_FAILED",
               "PLACE_FAILED", "BOX_EMPTY", "STORAGE_FULL",
               "PUCK_NOT_IN_STORAGE", "CELL_OCCUPIED")
ERROR_NAMES = {getattr(ErrorCode, name): name for name in _SAME_NAMES}
ERROR_NAMES[ErrorCode.TARGET_NOT_FOUND] = "PERCEPTION_FAILED"
ERROR_NAMES[ErrorCode.BAD_REQUEST] = "INTERNAL_ERROR"  # 우리 요청이 틀림


def error_name(code):
    """표에 없는 숫자는 규약이 어긋난 것이라 INTERNAL_ERROR로 본다."""
    return ERROR_NAMES.get(code, "INTERNAL_ERROR")


def _pose_stamped(frame_id, x_mm, y_mm, yaw_deg):
    """경기장 좌표(mm, deg) → PoseStamped(m, 쿼터니언)."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x, pose.pose.position.y = x_mm / 1000, y_mm / 1000
    pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
    pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
    return pose


def _goal_timeout(timeout_s):
    """로봇 쪽 한도는 우리 대기보다 2초 짧게 (로봇이 먼저 실패 이유를 돌려주도록)."""
    return Duration(sec=max(1, int(timeout_s - 2)))


def _mm(point):
    """geometry_msgs/Point(m) → [x, y] mm."""
    return [round(point.x * 1000, 1), round(point.y * 1000, 1)]


def _outcome(response, label):
    """하위 결과 → (결과, 에러 코드 또는 None, 설명)."""
    if response.success:
        return response, None, ""
    code = getattr(response, "error_code", None)
    return (response, error_name(code),
            f"{label}: [{code}] {getattr(response, 'message', '')}")


class RosBackend:
    def __init__(self, node, arena):
        self.node = node
        self.arena = arena
        self.frame_id = arena["frame_id"]
        self.base_frame = (
            node.get_parameter("base_frame").value
            if node.has_parameter("base_frame") else "base")
        self.fallback_pose = bool(
            node.get_parameter("fallback_pose_when_no_tf").value
            if node.has_parameter("fallback_pose_when_no_tf") else False)
        # 지금 장소. LOAD·UNLOAD·VERIFY는 인자에 장소가 없어 마지막 GOTO 결과로 안다 (출발은 HOME)
        self.here = "HOME"
        # 로봇 팀 액션 (같은 콜백 그룹이어야 콜백 안에서 기다려도 멈추지 않는다)
        group = node.callback_group
        self.navigate = ActionClient(node, RunTask, "/task/run",
                                     callback_group=group)
        self.read_input = ActionClient(
            node, ReadMissionInput, "/task/read_mission_input",
            callback_group=group)
        self.load_box = ActionClient(node, LoadBox, "/task/load_box",
                                     callback_group=group)
        self.unload_pucks = ActionClient(node, UnloadPucks, "/task/unload_pucks",
                                         callback_group=group)
        self.observe = ActionClient(node, ObserveLocation,
                                    "/task/observe_location",
                                    callback_group=group)
        # 지금 위치 (map → base_link)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, node)
        self.handlers = {
            name: getattr(self, f"_ros_{name}", self._not_implemented)
            for name in COMMANDS}

    def stop(self):
        pass

    def _not_implemented(self, args, goal_handle):
        return {"success": False, "error": "NOT_IMPLEMENTED"}

    # ---------- 하위 호출 ----------
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

    def _call_action(self, client, goal, timeout_s, goal_handle, label):
        """하위 액션 하나를 보내고 기다린다 → (결과, 에러 코드 또는 None, 설명)."""
        # 목표 보내기 (하위 진행 상황은 노트북 feedback으로 흘려 로그에 남긴다)
        send = client.send_goal_async(goal, feedback_callback=lambda message: (
            self.node.feedback(goal_handle, f"{label} "
                               f"{getattr(message.feedback, 'phase', '')}")))
        if (not self._wait_future(send, SERVER_WAIT_S)
                or not send.result().accepted):
            return None, "INTERNAL_ERROR", f"{label}: 서버 없음 또는 거절"
        # 결과 기다리기 (노트북 취소·시간 초과면 하위 액션도 취소해 로봇을 멈춘다)
        child = send.result()
        result = child.get_result_async()
        try:
            done = self._wait_future(result, timeout_s, goal_handle)
        except Canceled:
            child.cancel_goal_async()
            raise
        if not done:
            child.cancel_goal_async()
            return None, "TIMEOUT", f"{label}: {timeout_s:.0f}초 초과"
        # 결과를 노트북 에러 코드로 (설명에 하위 동작 이름과 원래 번호를 남긴다)
        return _outcome(result.result().result, label)

    def _with_retry(self, name, once, should_retry, goal_handle):
        """once(한도, 이름표)를 should_retry가 참인 동안 최대 MAX_TRIES번 부른다.

        중간 실패는 feedback으로만 남기고(노트북 로그), 마지막 결과를 돌려준다.
        """
        deadline = time.monotonic() + BUDGET_S[name]
        for attempt in range(1, MAX_TRIES + 1):
            timeout_s = min(ATTEMPT_S[name], deadline - time.monotonic())
            label = f"{name.lower()} {attempt}/{MAX_TRIES}"
            result = once(timeout_s, goal_handle, label)
            left = deadline - time.monotonic()
            if (not should_retry(result) or attempt == MAX_TRIES
                    or left < MIN_ATTEMPT_S):
                break
            self.node.feedback(goal_handle, f"{label} 실패 "
                                            f"({result.get('error')}), 다시 시도")
        if attempt > 1:
            result["attempts"] = attempt
        return result

    # ---------- 위치 ----------
    def _pose(self):
        """지금 로봇 자세 {x, y (mm), yaw (deg)}. TF가 없으면 None."""
        alt = "base_link" if self.base_frame == "base" else "base"
        transform = None
        for child in (self.base_frame, alt):
            try:
                transform = self.tf.lookup_transform(
                    self.frame_id, child, Time()).transform
                break
            except Exception:
                continue
        if transform is None:
            return None
        rotation = transform.rotation
        yaw = math.atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                         1 - 2 * (rotation.y ** 2 + rotation.z ** 2))
        return {"x": round(transform.translation.x * 1000),
                "y": round(transform.translation.y * 1000),
                "yaw": round(math.degrees(yaw), 1)}

    def _fallback_place_pose(self, place):
        """SLAM 없이 수동·스텁 시험할 때 쓸 추정 도착 자세."""
        if place == "HOME":
            x, y, yaw = self.arena["start_pose"]
            return {"x": round(x), "y": round(y), "yaw": round(yaw, 1)}
        tx, ty = self.arena["places"][place]["target"]
        return {"x": round(tx - 285), "y": round(ty), "yaw": 0.0}

    def _in_zone(self, pose):
        """차체 footprint 네 꼭짓점이 모두 출발 구역(HOME 중심 정사각형) 안에 있는지."""
        if pose is None:
            return False
        center_x, center_y = self.arena["places"]["HOME"]["target"]
        half = self.arena["start_zone"] / 2
        yaw = math.radians(pose["yaw"])
        for corner_x, corner_y in self.arena["robot"]["footprint"]:
            x = pose["x"] + corner_x * math.cos(yaw) - corner_y * math.sin(yaw)
            y = pose["y"] + corner_x * math.sin(yaw) + corner_y * math.cos(yaw)
            if abs(x - center_x) > half or abs(y - center_y) > half:
                return False
        return True

    def _wrong_place(self, allowed, name, extra=None):
        """지금 장소에서 할 수 없는 행동이면 실패 결과, 아니면 None."""
        if self.here in allowed:
            return None
        return {"success": False, "error": "WRONG_LOCATION",
                "message": f"{name}은(는) {'/'.join(allowed)}에서만, "
                           f"지금 {self.here}", **(extra or {})}

    # ---------- 행동 ----------
    def _ros_GOTO(self, args, goal_handle):
        place = args["t"]
        # 목적지: HOME은 출발 자세, 나머지는 대상 중심 (접근 자리는 주행 쪽이 정한다)
        if place == "HOME":
            x, y, yaw = self.arena["start_pose"]
        else:
            (x, y), yaw = self.arena["places"][place]["target"], 0.0
        timeout_s = ATTEMPT_S["GOTO"]
        goal = RunTask.Goal(
            operation="NAVIGATE", place_id=place,
            destination=_pose_stamped(self.frame_id, x, y, yaw),
            timeout=_goal_timeout(timeout_s))
        # 주행 실패(장애물 등)는 다시 갈지 미룰지 판단이 정하므로 다시 시도하지 않는다
        result, error, message = self._call_action(
            self.navigate, goal, timeout_s, goal_handle, f"goto {place}")
        if error:
            # 접수도 안 됐으면 그대로 있고, 출발했으면 어디서 멈췄는지 모른다
            robot = {"pose": self._pose()}
            if not (result is None and error == "INTERNAL_ERROR"):
                self.here = robot["here"] = "UNKNOWN"
            return {"success": False, "error": error, "message": message,
                    "robot": robot}
        self.here = place
        if place != "HOME":
            pose = self._pose()
            if pose is None and self.fallback_pose:
                pose = self._fallback_place_pose(place)
            if pose is None:
                self.here = "UNKNOWN"
                return {"success": False, "error": "LOCALIZATION_LOST",
                        "message": f"TF({self.frame_id}->{self.base_frame}) 없음",
                        "robot": {"here": "UNKNOWN", "pose": None}}
            return {"success": True, "robot": {"here": place, "pose": pose}}
        # HOME: 출발 구역 안에서 3초 이상 멈춘다 (취소되면 Canceled로 올라간다)
        self.node.feedback(goal_handle, "holding_3s")
        self.node.wait(goal_handle, HOME_HOLD_S)
        pose = self._pose()
        if pose is None and self.fallback_pose:
            pose = self._fallback_place_pose("HOME")
        if pose is None:
            self.here = "UNKNOWN"
            return {"success": False, "error": "LOCALIZATION_LOST",
                    "message": f"TF({self.frame_id}->{self.base_frame}) 없음",
                    "robot": {"here": "UNKNOWN", "pose": None}}
        return {"success": True, "robot": {"here": "HOME", "pose": pose},
                "in_zone": self._in_zone(pose), "stopped_s": HOME_HOLD_S}

    def _ros_READ_MISSION_INPUT(self, args, goal_handle):
        wrong = self._wrong_place(("HOME",), "READ_MISSION_INPUT")
        if wrong:
            return wrong

        def once(timeout_s, handle, label):
            goal = ReadMissionInput.Goal(timeout=_goal_timeout(timeout_s))
            result, error, message = self._call_action(
                self.read_input, goal, timeout_s, handle, label)
            if error:
                return {"success": False, "error": error, "message": message}
            # 읽은 신호가 규칙(12개, 끝 소등, 색마다 1~6개)에 맞는지 본다
            colors = list(result.colors)
            errors = signal_errors(colors, result.end_seen)
            read = {"success": not errors, "barcode": result.barcode,
                    "signal": colors, "end_seen": result.end_seen}
            if errors:
                read.update(
                    error="PERCEPTION_FAILED", signal_errors=errors,
                    cause="SIGNAL_INCOMPLETE" if "?" in colors else "SIGNAL_INVALID",
                    message=f"{label}: 신호가 규칙에 안 맞음: {', '.join(errors)}")
            return read

        # 못 찾음·못 읽음·규칙 위반은 신호가 반복되니 다시 읽는다
        result = self._with_retry(
            "READ_MISSION_INPUT", once,
            lambda read: read.get("error") == "PERCEPTION_FAILED", goal_handle)
        if result.get("attempts"):
            result["signal_reread"] = {"reads": result["attempts"],
                                       "complete": result["success"]}
        return result

    def _ros_LOAD(self, args, goal_handle):
        wrong = self._wrong_place(START_BOXES, "LOAD")
        if wrong:
            return wrong

        def once(timeout_s, handle, label):
            goal = LoadBox.Goal(place_id=self.here,
                                timeout=_goal_timeout(timeout_s))
            result, error, message = self._call_action(
                self.load_box, goal, timeout_s, handle, label)
            # 실패해도 실은 퍽은 보고한다 (퍽 id = AprilTag 번호, 못 읽으면 null)
            pucks = [{"slot": puck.slot, "puck": {
                "id": puck.tag_id if puck.tag_id >= 0 else None,
                "color": puck.color or None, "shape": puck.shape or None,
                "conf": round(puck.confidence, 2)}}
                for puck in (result.pucks if result else [])]
            load = {"success": not error, "pucks": pucks}
            if result is not None and result.box_remaining >= 0:
                load["box_remaining"] = result.box_remaining
            if error:
                load.update(error=error, message=message)
            return load

        # 아무것도 못 싣고 인식만 실패했을 때만 다시 시도한다 (실은 게 있으면 판단에 맡긴다)
        return self._with_retry(
            "LOAD", once,
            lambda load: load.get("error") == "PERCEPTION_FAILED"
            and not load["pucks"], goal_handle)

    def _ros_UNLOAD(self, args, goal_handle):
        location = self.here
        wrong = self._wrong_place(LOCATIONS, "UNLOAD",
                                  {"items": [], "stopped_at": 0})
        if wrong:
            return wrong
        timeout_s = ATTEMPT_S["UNLOAD"]
        goal = UnloadPucks.Goal(
            location_id=location, timeout=_goal_timeout(timeout_s),
            items=[PlaceItem(puck_id=item["puck_id"], cell=item["cell"])
                   for item in args["items"]])
        # 놓기는 되돌릴 수 없어서 다시 시도하지 않는다 (남은 퍽을 다시 놓을지는 판단이 정한다)
        result, error, message = self._call_action(
            self.unload_pucks, goal, timeout_s, goal_handle, f"unload {location}")
        if result is None:  # 서버 없음·시간 초과: 어디까지 놓았는지 모른다
            return {"success": False, "error": error, "message": message,
                    "items": [], "stopped_at": 0}
        items = [{"puck_id": placed.puck_id, "cell": placed.cell, "ok": placed.ok,
                  **({"offset_mm": _mm(placed.offset)} if placed.ok
                     else {"error": error_name(placed.error_code)})}
                 for placed in result.items]
        unload = {"success": not error, "items": items,
                  "stopped_at": None if result.stopped_at < 0 else result.stopped_at}
        if error:
            unload.update(error=error, message=message)
        return unload

    def _ros_VERIFY_LOCATION(self, args, goal_handle):
        location = self.here
        wrong = self._wrong_place(LOCATIONS, "VERIFY_LOCATION")
        if wrong:
            return wrong

        def once(timeout_s, handle, label):
            goal = ObserveLocation.Goal(location_id=location,
                                        timeout=_goal_timeout(timeout_s))
            result, error, message = self._call_action(
                self.observe, goal, timeout_s, handle, label)
            if error:
                return {"success": False, "error": error, "message": message}
            return self._compare(args["items"], result.cells)

        # 관측 실패는 다시 본다 (배치가 틀린 것은 실패가 아니라 성공 + all_ok=false)
        return self._with_retry(
            "VERIFY_LOCATION", once,
            lambda verify: verify.get("error") == "PERCEPTION_FAILED",
            goal_handle)

    @staticmethod
    def _compare(asked_items, observed_cells):
        """9칸 관측을 보낸 items와 비교: 그 cell에 그 퍽이 있나, 목록에 없는 cell에 퍽이 있나."""
        seen = {observed.cell: observed for observed in observed_cells}
        cells = []
        for cell in range(NUM_CELLS):
            observed = seen.get(cell)
            occupied = observed is not None and observed.state == "occupied"
            cells.append({
                "cell": cell,
                "state": observed.state if observed else "unknown",
                "puck_id": observed.tag_id
                if observed and observed.tag_id >= 0 else None,
                "offset_mm": _mm(observed.offset) if occupied else None})
        asked = {item["cell"] for item in asked_items}
        items = [{**item, "ok": cells[item["cell"]]["puck_id"] == item["puck_id"],
                  "seen_puck_id": cells[item["cell"]]["puck_id"],
                  "offset_mm": cells[item["cell"]]["offset_mm"]}
                 for item in asked_items]
        extra = [cell["cell"] for cell in cells
                 if cell["state"] == "occupied" and cell["cell"] not in asked]
        return {"success": True, "all_ok": all(item["ok"] for item in items)
                and not extra, "items": items, "extra": extra, "cells": cells}
