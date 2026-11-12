"""backend:=ros 실제 로봇(또는 시뮬레이터) 동작: 로봇 팀의 액션·서비스를 불러 행동 하나를 조립한다.

로봇 팀 인터페이스와 흐름은 docs/robot_primitives.md (초안, 로봇 팀과 확정 전).
"""
import math
import time

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from xsting_interfaces.action import RunTask, WatchSignal
from xsting_interfaces.msg import ErrorCode
from xsting_interfaces.srv import (DetectPucks, GetStorage, ObserveCells,
                                  ReadBarcode)

from ..core.contract import COMMANDS, Canceled
from ..core.rules import LOCATIONS, NUM_CELLS, START_BOXES, signal_errors

# 하위 서버를 찾거나 목표를 받아 줄 때까지 기다리는 시간(초)
SERVER_WAIT_S = 2.0
# 하위 동작 한도(초). 합이 노트북 대기(node.py ACTION_TIMEOUT_S)보다 짧아야 한다
GOTO_TIMEOUT_S = 170.0     # 노트북 GOTO 180
BARCODE_TIMEOUT_S = 5.0
SIGNAL_TIMEOUT_S = 40.0    # 한 주기 약 27초 + 여유. READ 전체 ≈ 5 + 3 × 40 < 노트북 600
DETECT_TIMEOUT_S = 5.0
STORAGE_TIMEOUT_S = 2.0
PICK_TIMEOUT_S = 60.0      # 퍽 2개 ≈ 120 + 조회 < 노트북 LOAD 180
PLACE_TIMEOUT_S = 60.0     # 퍽 6개 ≈ 360 + 조회 < 노트북 UNLOAD 420
OBSERVE_TIMEOUT_S = 5.0
# HOME 도착 뒤 정지 시간 (출발 구역 3초 정지 + 여유)
HOME_HOLD_S = 3.2
# 신호 한 주기를 다시 읽는 최대 횟수 (처음 포함, 가짜 경기장과 같다)
SIGNAL_READ_LIMIT = 3
# 하차 박스 cell 간격(mm). cell 0 1 2 / 3 4 5 / 6 7 8, 지도 +Y가 위쪽
CELL_PITCH_MM = 60

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


def _seconds(value):
    return Duration(sec=int(value))


def _mm(point):
    """geometry_msgs/Point(m) → [x, y] mm."""
    return [round(point.x * 1000, 1), round(point.y * 1000, 1)]


def _outcome(response, label):
    """하위 결과·응답 → (응답, 에러 코드 또는 None, 설명). error_code·message가 없는 타입도 받는다."""
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
        # 지금 장소. LOAD·UNLOAD는 인자에 장소가 없어 마지막 GOTO 결과로 안다 (출발은 HOME)
        self.here = "HOME"
        # 로봇 팀 클라이언트 (같은 콜백 그룹이어야 콜백 안에서 기다려도 멈추지 않는다)
        group = node.callback_group
        self.task = ActionClient(node, RunTask, "/task/run",
                                 callback_group=group)
        self.watch_signal = ActionClient(
            node, WatchSignal, "/perception/watch_signal",
            callback_group=group)
        self.read_barcode = node.create_client(
            ReadBarcode, "/perception/read_barcode", callback_group=group)
        self.detect_pucks = node.create_client(
            DetectPucks, "/perception/detect_pucks", callback_group=group)
        self.observe_cells = node.create_client(
            ObserveCells, "/perception/observe_cells", callback_group=group)
        self.get_storage = node.create_client(
            GetStorage, "/storage/get_storage", callback_group=group)
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

    def _call_service(self, client, request, timeout_s, goal_handle, label):
        """서비스 하나를 부르고 기다린다 → (응답, 에러 코드 또는 None, 설명)."""
        # 서버 확인 (서비스는 접수 단계가 없어, 확인 안 하면 timeout_s 내내 기다린다)
        if not client.wait_for_service(timeout_sec=SERVER_WAIT_S):
            return None, "INTERNAL_ERROR", f"{label}: 서버 없음"
        # 응답 기다리기 (서비스는 취소가 없다. 짧은 조회에만 쓰므로 기다리기만 멈춘다)
        future = client.call_async(request)
        if not self._wait_future(future, timeout_s, goal_handle):
            return None, "TIMEOUT", f"{label}: {timeout_s:.0f}초 안에 응답 없음"
        # 결과를 노트북 에러 코드로 (error_code가 없는 응답은 INTERNAL_ERROR)
        return _outcome(future.result(), label)

    # ---------- 위치 ----------
    def _pose(self):
        """지금 로봇 자세 {x, y (mm), yaw (deg)}. TF가 없으면 None."""
        try:
            transform = self.tf.lookup_transform(
                self.frame_id, "base_link", Time()).transform
        except Exception:  # TF가 아직 없거나 끊김
            return None
        rotation = transform.rotation
        yaw = math.atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                         1 - 2 * (rotation.y ** 2 + rotation.z ** 2))
        return {"x": round(transform.translation.x * 1000),
                "y": round(transform.translation.y * 1000),
                "yaw": round(math.degrees(yaw), 1)}

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

    def _cell_pose(self, location, cell):
        """하차 박스 cell 중심 (lN_grid 기준). 차체 위치 오차가 섞이지 않게 태그 기준으로 준다."""
        x = (cell % 3 - 1) * CELL_PITCH_MM
        y = (1 - cell // 3) * CELL_PITCH_MM
        return _pose_stamped(f"{location.lower()}_grid", x, y, 0.0)

    # ---------- 인식 ----------
    def _read_barcode(self, goal_handle):
        """바코드 한 번 읽기 → (코드 문자열 또는 None, 에러 코드 또는 None, 설명)."""
        request = ReadBarcode.Request(timeout=_seconds(BARCODE_TIMEOUT_S))
        # 인식 쪽 한도보다 1초 더 기다린다 (그쪽이 먼저 실패를 돌려주도록)
        response, error, message = self._call_service(
            self.read_barcode, request, BARCODE_TIMEOUT_S + 1.0, goal_handle,
            "read barcode")
        if error:
            return None, error, message
        return response.code, None, ""

    def _watch_signal(self, read, goal_handle):
        """신호 한 주기 보기 → (색 목록, 끝 소등 봤나, 에러 코드 또는 None, 설명)."""
        goal = WatchSignal.Goal(timeout=_seconds(SIGNAL_TIMEOUT_S))
        result, error, message = self._call_action(
            self.watch_signal, goal, SIGNAL_TIMEOUT_S + 2.0, goal_handle,
            f"read {read}/{SIGNAL_READ_LIMIT}")
        if error:
            return None, None, error, message
        return list(result.colors), result.end_seen, None, ""

    def _storage(self, goal_handle):
        """적재함 12칸 → (칸 목록, 에러 코드 또는 None, 설명)."""
        response, error, message = self._call_service(
            self.get_storage, GetStorage.Request(), STORAGE_TIMEOUT_S,
            goal_handle, "get storage")
        return (response.slots if response else None), error, message

    # ---------- 행동 ----------
    def _ros_GOTO(self, args, goal_handle):
        place = args["t"]
        # 목적지: HOME은 출발 자세, 나머지는 대상 중심 (접근 자리는 주행 쪽이 정한다)
        if place == "HOME":
            x, y, yaw = self.arena["start_pose"]
        else:
            (x, y), yaw = self.arena["places"][place]["target"], 0.0
        goal = RunTask.Goal(
            operation="NAVIGATE", place_id=place,
            destination=_pose_stamped(self.frame_id, x, y, yaw),
            timeout=_seconds(GOTO_TIMEOUT_S))
        result, error, message = self._call_action(
            self.task, goal, GOTO_TIMEOUT_S, goal_handle, f"goto {place}")
        if error:
            # 접수도 안 됐으면 그대로 있고, 출발했으면 어디서 멈췄는지 모른다
            robot = {"pose": self._pose()}
            if not (result is None and error == "INTERNAL_ERROR"):
                self.here = robot["here"] = "UNKNOWN"
            return {"success": False, "error": error, "message": message,
                    "robot": robot}
        self.here = place
        if place != "HOME":
            return {"success": True, "robot": {"here": place, "pose": self._pose()}}
        # HOME: 출발 구역 안에서 3초 이상 멈춘다 (취소되면 Canceled로 올라간다)
        self.node.feedback(goal_handle, "holding_3s")
        self.node.wait(goal_handle, HOME_HOLD_S)
        pose = self._pose()
        return {"success": True, "robot": {"here": "HOME", "pose": pose},
                "in_zone": self._in_zone(pose), "stopped_s": HOME_HOLD_S}

    def _ros_READ_MISSION_INPUT(self, args, goal_handle):
        wrong = self._wrong_place(("HOME",), "READ_MISSION_INPUT")
        if wrong:
            return wrong
        # 바코드 (못 찾을 때 한 바퀴 돌며 찾기는 아직: docs/robot_primitives.md 7절 4번)
        code, error, message = self._read_barcode(goal_handle)
        if error:
            return {"success": False, "error": error, "message": message}
        # 신호: 규칙(rules.signal_errors)에 안 맞으면 다음 주기를 다시 읽는다 (중간 오류는 로그만)
        for read in range(1, SIGNAL_READ_LIMIT + 1):
            colors, end_seen, error, message = self._watch_signal(read, goal_handle)
            if error:
                return {"success": False, "error": error, "message": message}
            errors = signal_errors(colors, end_seen)
            if not errors:
                break
            self.node.feedback(goal_handle, f"read {read}/{SIGNAL_READ_LIMIT} "
                                            f"invalid: {', '.join(errors)}")
        result = {"success": not errors, "barcode": code, "signal": colors,
                  "end_seen": end_seen}
        if read > 1:
            result["signal_reread"] = {"reads": read, "complete": not errors}
        if errors:
            # 못 읽은 칸이 남았으면 INCOMPLETE, 다 읽었지만 규칙에 안 맞으면 INVALID
            result.update(
                error="PERCEPTION_FAILED", signal_errors=errors,
                cause="SIGNAL_INCOMPLETE" if "?" in colors else "SIGNAL_INVALID",
                message=f"신호를 {read}번 읽었지만 규칙에 안 맞음: {', '.join(errors)}")
        return result

    def _ros_LOAD(self, args, goal_handle):
        wrong = self._wrong_place(START_BOXES, "LOAD")
        if wrong:
            return wrong
        # 박스 위 퍽 보기
        response, error, message = self._call_service(
            self.detect_pucks, DetectPucks.Request(place_id=self.here),
            DETECT_TIMEOUT_S, goal_handle, f"detect {self.here}")
        if error:
            return {"success": False, "error": error, "message": message}
        pucks = list(response.pucks)
        if not pucks:
            return {"success": False, "error": "BOX_EMPTY", "box_remaining": 0,
                    "pucks": [], "message": f"{self.here}에 퍽이 보이지 않음"}
        # 빈 슬롯 (예약·확정은 작업 관리자가 PICK_STORE 안에서 한다)
        slots, error, message = self._storage(goal_handle)
        if error:
            return {"success": False, "error": error, "message": message}
        empty = [slot.slot_id for slot in slots if slot.state == "EMPTY"]
        # 퍽마다 집어 싣는다. 실패하면 거기까지 실은 것만 보고한다
        loaded = []
        for index, puck in enumerate(pucks):
            if index >= len(empty):
                error, message = "STORAGE_FULL", "적재함에 빈 슬롯이 없음"
                break
            goal = RunTask.Goal(operation="PICK_STORE", object_id=puck.object_id,
                                slot_id=empty[index], timeout=_seconds(PICK_TIMEOUT_S))
            _, error, message = self._call_action(
                self.task, goal, PICK_TIMEOUT_S, goal_handle,
                f"pick {index + 1}/{len(pucks)}")
            if error:
                break
            loaded.append({"slot": int(empty[index]), "puck": {
                "id": puck.tag_id if puck.tag_id >= 0 else None,  # 퍽 AprilTag = 퍽 id
                "color": puck.color or None, "shape": puck.shape or None,
                "conf": round(puck.confidence, 2)}})
        result = {"success": not error, "box_remaining": len(pucks) - len(loaded),
                  "pucks": loaded}
        if error:
            result.update(error=error, message=message)
        return result

    def _ros_UNLOAD(self, args, goal_handle):
        location = self.here
        wrong = self._wrong_place(LOCATIONS, "UNLOAD",
                                  {"items": [], "stopped_at": 0})
        if wrong:
            return wrong
        # 퍽 id → 들어 있는 슬롯
        slots, error, message = self._storage(goal_handle)
        if error:
            return {"success": False, "error": error, "message": message,
                    "items": [], "stopped_at": 0}
        slot_of = {slot.tag_id: slot.slot_id for slot in slots
                   if slot.state == "OCCUPIED" and slot.tag_id >= 0}
        # items 순서대로 놓는다. 실패하면 그 항목에서 멈추고 어디까지 했는지 보고한다
        done = []
        for index, item in enumerate(args["items"]):
            slot = slot_of.get(item["puck_id"])
            if slot is None:
                result = None
                error = "PUCK_NOT_IN_STORAGE"
                message = f"퍽 {item['puck_id']}이(가) 적재함에 없음"
            else:
                goal = RunTask.Goal(
                    operation="STORE_PLACE", slot_id=slot,
                    destination=self._cell_pose(location, item["cell"]),
                    timeout=_seconds(PLACE_TIMEOUT_S))
                result, error, message = self._call_action(
                    self.task, goal, PLACE_TIMEOUT_S, goal_handle,
                    f"place {index + 1}/{len(args['items'])}")
            if error:
                done.append({**item, "ok": False, "error": error})
                return {"success": False, "error": error, "message": message,
                        "items": done, "stopped_at": index}
            done.append({**item, "ok": True, "offset_mm": _mm(result.place_offset)})
        return {"success": True, "stopped_at": None, "items": done}

    def _ros_VERIFY_LOCATION(self, args, goal_handle):
        location = self.here
        wrong = self._wrong_place(LOCATIONS, "VERIFY_LOCATION")
        if wrong:
            return wrong
        response, error, message = self._call_service(
            self.observe_cells, ObserveCells.Request(location_id=location),
            OBSERVE_TIMEOUT_S, goal_handle, f"observe {location}")
        if error:
            return {"success": False, "error": error, "message": message}
        # 9칸 관측 (빠진 칸은 unknown)
        seen = {observed.cell: observed for observed in response.cells}
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
        # 보낸 items와 비교: 그 cell에 그 퍽이 있나, 목록에 없는 cell에 퍽이 있나
        asked = {item["cell"] for item in args["items"]}
        items = [{**item, "ok": cells[item["cell"]]["puck_id"] == item["puck_id"],
                  "seen_puck_id": cells[item["cell"]]["puck_id"],
                  "offset_mm": cells[item["cell"]]["offset_mm"]}
                 for item in args["items"]]
        extra = [cell["cell"] for cell in cells
                 if cell["state"] == "occupied" and cell["cell"] not in asked]
        return {"success": True, "all_ok": all(item["ok"] for item in items)
                and not extra, "items": items, "extra": extra, "cells": cells}
