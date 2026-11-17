"""노트북 미션 브릿지 노드: ros_backend의 /task/* 액션을 로봇 노드 액션·토픽으로 중계한다.

연결 구조:
  llm_commander -> robot_command_server (ros_backend) -> mission_bridge (노트북)
    - /task/run (NAVIGATE)        -> 외부 주행 액션(또는 수동 확인/스텁) -> /approach_box (연속 실행)
    - /task/read_mission_input    -> /read_sign_barcode (xsting_vision)
    - /task/load_box              -> /pick_place CMD_LOAD_BOX (xsting_pick_place) + 슬롯/임시 ID 매핑
    - /task/unload_pucks          -> /pick_place CMD_UNLOAD_BOX (xsting_pick_place) + 지도 방향 3x3 변환
    - /task/observe_location      -> /pick_place CMD_OBSERVE + /detections_3d 9칸 실측 관측
"""
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3DArray

from xsting_interfaces.action import (ApproachBox, LoadBox, ObserveLocation,
                                      PickPlace, ReadMissionInput,
                                      ReadSignBarcode, RunTask, UnloadPucks)
from xsting_interfaces.msg import CellObservation, ErrorCode, LoadedPuck, PlacedItem

from ..core.rules import LOCATIONS, NUM_CELLS, NUM_PUCKS, START_BOXES, STORAGE_SLOTS

BOX_PLACES = START_BOXES + LOCATIONS
CELL_PITCH_M = 0.060
CELL_MATCH_RADIUS_M = 0.035
SERVER_WAIT_S = 2.5


class _BridgeCanceled(Exception):
    """상위 액션 취소 요청이 들어왔을 때 내부 단계를 중단한다."""


def _duration_sec(dur, default_s):
    """builtin_interfaces/Duration -> 초(float)."""
    sec = float(dur.sec) + float(dur.nanosec) * 1e-9
    return sec if sec > 0.0 else default_s


def _is_box_class(cls_id):
    """종이 상자(paper_box / class 4) 검출인지 판별한다."""
    text = (cls_id or "").strip().lower()
    return text in ("4", "box", "paper_box") or "paper_box" in text


def _parse_puck_class(cls_id):
    """xsting_vision 퍽 클래스 문자열(red_square 등)을 (color, shape)로 변환한다."""
    text = (cls_id or "").strip().lower()
    if _is_box_class(text):
        return "", ""
    color = "R" if "red" in text else ("G" if "green" in text else "")
    if "cylinder" in text:
        shape = "cylinder"
    elif "square" in text or "box" in text:
        shape = "box"
    else:
        shape = ""
    return color, shape


def _cell_map_offset(cell):
    """주사위 격자 cell(0~8)의 박스 중심 기준 지도(+X 오른쪽, +Y 위쪽) 오프셋(m)."""
    row, col = divmod(int(cell), 3)
    return (col - 1) * CELL_PITCH_M, (1 - row) * CELL_PITCH_M


def _map_to_base_xy(map_dx, map_dy, robot_yaw_rad):
    """지도 기준 박스 오프셋(m)을 로봇 base 기준(앞 +X, 왼쪽 +Y)으로 회전한다."""
    cos_y = math.cos(robot_yaw_rad)
    sin_y = math.sin(robot_yaw_rad)
    return (cos_y * map_dx + sin_y * map_dy,
            -sin_y * map_dx + cos_y * map_dy)


def _base_to_map_xy(base_dx, base_dy, robot_yaw_rad):
    """로봇 base 기준 박스 오프셋(m)을 지도 기준(+X 오른쪽, +Y 위쪽)으로 역회전한다."""
    cos_y = math.cos(robot_yaw_rad)
    sin_y = math.sin(robot_yaw_rad)
    return (cos_y * base_dx - sin_y * base_dy,
            sin_y * base_dx + cos_y * base_dy)


class MissionBridge(Node):
    """상위 미션 액션(/task/*)과 하위 로봇 노드 액션·센서를 잇는 브릿지 노드."""

    def __init__(self):
        super().__init__("mission_bridge")

        # 주행·정밀접근 연동 파라미터
        self.declare_parameter("nav_mode", "manual")  # action | manual | stub
        self.declare_parameter("nav_action", "/nav/run")
        self.declare_parameter("approach_action", "/approach_box")
        self.declare_parameter("approach_after_nav", True)
        self.declare_parameter("approach_in_manual", True)
        self.declare_parameter("require_approach_in_manual", False)
        self.declare_parameter("approach_in_stub", False)
        self.declare_parameter("approach_timeout_s", 35.0)
        self.declare_parameter("approach_dry_run", False)
        self.declare_parameter("walk_stop_service", "/walk/stop")

        # 인식·팔 연동 파라미터
        self.declare_parameter("sign_action", "/read_sign_barcode")
        self.declare_parameter("pick_action", "/pick_place")
        self.declare_parameter("det3d_topic", "/detections_3d")
        self.declare_parameter("puck_tags_topic", "/puck_tags")
        self.declare_parameter("pucks_per_box", 2)
        self.declare_parameter("default_puck_conf", 0.90)
        self.declare_parameter("default_box_x", 0.285)
        self.declare_parameter("default_box_y", 0.0)
        self.declare_parameter("unload_hold_s", 2.0)
        self.declare_parameter("observe_wait_s", 3.0)
        self.declare_parameter("use_arm_observe_pose", True)
        self.declare_parameter("allow_direct_slot_fallback", False)
        self.declare_parameter("reset_on_read_input", True)

        # 좌표계 파라미터
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("snap_cardinal_yaw", True)

        self._group = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()
        self._det_lock = threading.Lock()
        self._nav_lock = threading.Lock()

        # 적재함 슬롯(0~11) ↔ 임시/태그 퍽 ID(0~11) 및 하차 기록 상태
        self._slot_to_puck = {}
        self._puck_to_slot = {}
        self._placed_by_loc = {loc: {} for loc in LOCATIONS}
        self._pending_tag_ids = []

        # 최신 3D 검출 캐시
        self._latest_det = None
        self._det_seq = 0
        self._det_mono = 0.0

        # 수동 주행 확인 대기 상태
        self._manual_waiting = False
        self._manual_place = ""
        self._manual_outcome = None
        self._manual_event = threading.Event()

        # TF 리스너
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

        # 하위 액션·서비스 클라이언트 및 센서 구독
        nav_act = self.get_parameter("nav_action").value
        app_act = self.get_parameter("approach_action").value
        sign_act = self.get_parameter("sign_action").value
        pick_act = self.get_parameter("pick_action").value
        walk_stop = self.get_parameter("walk_stop_service").value
        det_topic = self.get_parameter("det3d_topic").value
        tag_topic = self.get_parameter("puck_tags_topic").value

        self._nav_cli = ActionClient(
            self, RunTask, nav_act, callback_group=self._group)
        self._approach_cli = ActionClient(
            self, ApproachBox, app_act, callback_group=self._group)
        self._sign_cli = ActionClient(
            self, ReadSignBarcode, sign_act, callback_group=self._group)
        self._pick_cli = ActionClient(
            self, PickPlace, pick_act, callback_group=self._group)
        self._walk_stop_cli = (
            self.create_client(Trigger, walk_stop, callback_group=self._group)
            if walk_stop else None)

        self._det_sub = self.create_subscription(
            Detection3DArray, det_topic, self._on_det3d,
            qos_profile_sensor_data, callback_group=self._group)
        if tag_topic:
            self._tag_sub = self.create_subscription(
                Int32MultiArray, tag_topic, self._on_puck_tags, 10,
                callback_group=self._group)

        # 노트북 수동 주행 제어 및 상태 초기화 서비스
        self._arrive_srv = self.create_service(
            Trigger, "/mission_bridge/nav_arrive", self._on_nav_arrive,
            callback_group=self._group)
        self._fail_srv = self.create_service(
            Trigger, "/mission_bridge/nav_fail", self._on_nav_fail,
            callback_group=self._group)
        self._reset_srv = self.create_service(
            Trigger, "/mission_bridge/reset_state", self._on_reset_state,
            callback_group=self._group)

        # 상위 ros_backend가 호출하는 5개 미션 기본 동작 액션 서버
        self._servers = [
            ActionServer(
                self, RunTask, "/task/run",
                execute_callback=self._exec_run_task,
                goal_callback=self._accept_goal,
                cancel_callback=self._accept_cancel,
                callback_group=self._group),
            ActionServer(
                self, ReadMissionInput, "/task/read_mission_input",
                execute_callback=self._exec_read_input,
                goal_callback=self._accept_goal,
                cancel_callback=self._accept_cancel,
                callback_group=self._group),
            ActionServer(
                self, LoadBox, "/task/load_box",
                execute_callback=self._exec_load_box,
                goal_callback=self._accept_goal,
                cancel_callback=self._accept_cancel,
                callback_group=self._group),
            ActionServer(
                self, UnloadPucks, "/task/unload_pucks",
                execute_callback=self._exec_unload_pucks,
                goal_callback=self._accept_goal,
                cancel_callback=self._accept_cancel,
                callback_group=self._group),
            ActionServer(
                self, ObserveLocation, "/task/observe_location",
                execute_callback=self._exec_observe_location,
                goal_callback=self._accept_goal,
                cancel_callback=self._accept_cancel,
                callback_group=self._group),
        ]

        self.get_logger().info(
            f"mission_bridge ready (nav_mode={self._param('nav_mode')}, "
            f"nav_action={nav_act}, approach={app_act}, pick={pick_act})")

    # ---------- 공통 도우미 ----------
    def _param(self, name):
        return self.get_parameter(name).value

    def _accept_goal(self, _goal_request):
        return GoalResponse.ACCEPT

    def _accept_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _on_det3d(self, msg):
        with self._det_lock:
            self._latest_det = msg
            self._det_seq += 1
            self._det_mono = time.monotonic()

    def _on_puck_tags(self, msg):
        """추후 AprilTag 인식 노드가 발행하는 태그 번호 큐 연동용 콜백."""
        with self._state_lock:
            self._pending_tag_ids = [
                int(tag) for tag in msg.data if 0 <= int(tag) < NUM_PUCKS]

    def _on_nav_arrive(self, _req, res):
        with self._nav_lock:
            if not self._manual_waiting:
                res.success = False
                res.message = "현재 대기 중인 수동 주행 목표가 없습니다"
                return res
            place = self._manual_place
            self._manual_outcome = "arrive"
            self._manual_event.set()
        res.success = True
        res.message = f"{place} 수동 도착 확인 완료"
        return res

    def _on_nav_fail(self, _req, res):
        with self._nav_lock:
            if not self._manual_waiting:
                res.success = False
                res.message = "현재 대기 중인 수동 주행 목표가 없습니다"
                return res
            place = self._manual_place
            self._manual_outcome = "fail"
            self._manual_event.set()
        res.success = True
        res.message = f"{place} 주행 실패(장애물) 처리 완료"
        return res

    def _on_reset_state(self, _req, res):
        with self._state_lock:
            self._slot_to_puck.clear()
            self._puck_to_slot.clear()
            self._placed_by_loc = {loc: {} for loc in LOCATIONS}
            self._pending_tag_ids.clear()
        res.success = True
        res.message = "적재함 슬롯 및 임시 퍽 ID 상태를 초기화했습니다"
        return res

    def _finish(self, goal_handle, result):
        if goal_handle.is_cancel_requested:
            result.success = False
            result.error_code = ErrorCode.CANCELED
            if not result.message:
                result.message = "취소됨"
            goal_handle.canceled()
        elif result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _wait_future(self, future, timeout_s, goal_handle=None):
        start = time.monotonic()
        while not future.done():
            if goal_handle is not None and goal_handle.is_cancel_requested:
                raise _BridgeCanceled()
            if timeout_s is not None and (time.monotonic() - start) > timeout_s:
                return False
            time.sleep(0.02)
        return True

    def _sleep_cancelable(self, seconds, goal_handle):
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                raise _BridgeCanceled()
            time.sleep(0.02)

    def _call_child_action(self, client, goal, timeout_s, goal_handle,
                           on_feedback=None, wait_server_s=SERVER_WAIT_S):
        """하위 액션 호출 -> (status, result_msg, err_code, err_text)."""
        if not client.wait_for_server(timeout_sec=wait_server_s):
            return None, None, ErrorCode.INTERNAL_ERROR, "하위 액션 서버 없음"

        send_fut = client.send_goal_async(goal, feedback_callback=on_feedback)
        if not self._wait_future(send_fut, SERVER_WAIT_S, goal_handle):
            return None, None, ErrorCode.TIMEOUT, "하위 액션 목표 전송 시간 초과"

        child_handle = send_fut.result()
        if child_handle is None or not child_handle.accepted:
            return None, None, ErrorCode.INTERNAL_ERROR, "하위 액션 목표 거절됨"

        res_fut = child_handle.get_result_async()
        try:
            done = self._wait_future(res_fut, timeout_s, goal_handle)
        except _BridgeCanceled:
            child_handle.cancel_goal_async()
            raise
        if not done:
            child_handle.cancel_goal_async()
            return None, None, ErrorCode.TIMEOUT, f"{timeout_s:.1f}초 시간 초과"

        wrapped = res_fut.result()
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            return wrapped.status, wrapped.result, ErrorCode.CANCELED, "하위 액션 취소됨"
        return wrapped.status, wrapped.result, ErrorCode.OK, ""

    def _stop_walk_best_effort(self):
        """수동 보행 정지 후 정밀 접근·팔 동작으로 넘어가도록 /walk/stop을 호출한다."""
        if self._walk_stop_cli is None:
            return
        if not self._walk_stop_cli.wait_for_service(timeout_sec=0.2):
            return
        fut = self._walk_stop_cli.call_async(Trigger.Request())
        self._wait_future(fut, 0.5)

    def _robot_yaw_rad(self):
        """TF(map -> base/base_link)에서 현재 로봇 yaw(rad)를 구한다. 없으면 0.0."""
        map_frame = self._param("map_frame")
        base_frame = self._param("base_frame")
        alt_frame = "base_link" if base_frame == "base" else "base"
        tf_msg = None
        for child in (base_frame, alt_frame):
            try:
                tf_msg = self._tf.lookup_transform(
                    map_frame, child, Time()).transform
                break
            except Exception:
                continue
        if tf_msg is None:
            return 0.0
        q = tf_msg.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self._param("snap_cardinal_yaw"):
            half_pi = 0.5 * math.pi
            yaw = round(yaw / half_pi) * half_pi
        return yaw

    def _allocate_puck_id(self):
        """AprilTag 큐가 있으면 우선 사용하고, 없으면 미사용 임시 ID(0~11)를 순서대로 부여한다."""
        used = set(self._puck_to_slot.keys())
        for loc_cells in self._placed_by_loc.values():
            used.update(info["puck_id"] for info in loc_cells.values())

        while self._pending_tag_ids:
            cand = self._pending_tag_ids.pop(0)
            if 0 <= cand < NUM_PUCKS and cand not in used:
                return cand

        for pid in range(NUM_PUCKS):
            if pid not in used:
                return pid
        return -1

    def _extract_box_and_pucks(self, det_msg):
        """Detection3DArray(base 기준)에서 작업 박스와 상판 위 퍽 목록을 추출한다."""
        box_x = float(self._param("default_box_x"))
        box_y = float(self._param("default_box_y"))
        pucks = []
        if det_msg is None:
            return box_x, box_y, pucks

        for det in det_msg.detections:
            if not det.results:
                continue
            cls_id = det.results[0].hypothesis.class_id
            pt = det.bbox.center.position
            if _is_box_class(cls_id):
                if 0.16 < pt.x < 0.48 and abs(pt.y) < 0.18:
                    box_x, box_y = float(pt.x), float(pt.y)

        for det in det_msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            if _is_box_class(hyp.class_id):
                continue
            color, shape = _parse_puck_class(hyp.class_id)
            if not color and not shape:
                continue
            pt = det.bbox.center.position
            if abs(pt.x - box_x) <= 0.135 and abs(pt.y - box_y) <= 0.135:
                pucks.append({
                    "x": float(pt.x),
                    "y": float(pt.y),
                    "z": float(pt.z),
                    "color": color,
                    "shape": shape,
                    "conf": float(hyp.score),
                })
        return box_x, box_y, pucks

    def _wait_fresh_det3d(self, wait_s, goal_handle):
        """관측 시점 이후 새로 들어온 /detections_3d 메시지를 기다린다."""
        with self._det_lock:
            base_seq = self._det_seq
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                raise _BridgeCanceled()
            with self._det_lock:
                if self._det_seq > base_seq and self._latest_det is not None:
                    return self._latest_det
            time.sleep(0.03)
        with self._det_lock:
            if self._latest_det is not None and (time.monotonic() - self._det_mono) <= 1.5:
                return self._latest_det
        return None

    # ---------- 1. GOTO: 주행 + ApproachBox 연속 실행 ----------
    def _pub_run_fb(self, goal_handle, phase, progress=0.0):
        fb = RunTask.Feedback()
        fb.phase = phase
        fb.progress = float(progress)
        goal_handle.publish_feedback(fb)

    def _should_chain_approach(self, place_id, nav_mode):
        if place_id not in BOX_PLACES:
            return False
        if nav_mode == "action":
            return bool(self._param("approach_after_nav"))
        if nav_mode == "manual":
            return bool(self._param("approach_in_manual"))
        if nav_mode == "stub":
            return bool(self._param("approach_in_stub"))
        return False

    def _run_coarse_nav(self, req, nav_mode, timeout_s, goal_handle):
        """1단계: 목적지 근처까지 주행 (외부 RL 주행 액션 / 수동 확인 / 스텁)."""
        place = req.place_id
        if nav_mode == "stub":
            self._pub_run_fb(goal_handle, f"stub_nav arrived {place}", 0.5)
            return True, ErrorCode.OK, f"{place} 스텁 이동 완료"

        if nav_mode == "manual":
            with self._nav_lock:
                self._manual_waiting = True
                self._manual_place = place
                self._manual_outcome = None
                self._manual_event.clear()
            msg = (f"manual_nav {place}: 이동 후 "
                   f"/mission_bridge/nav_arrive 호출 대기")
            self.get_logger().info(msg)
            self._pub_run_fb(goal_handle, msg, 0.2)
            start = time.monotonic()
            try:
                while not self._manual_event.is_set():
                    if goal_handle.is_cancel_requested:
                        raise _BridgeCanceled()
                    if (time.monotonic() - start) > timeout_s:
                        return False, ErrorCode.TIMEOUT, f"{place} 수동 주행 대기 시간 초과"
                    time.sleep(0.05)
                outcome = self._manual_outcome
            finally:
                with self._nav_lock:
                    self._manual_waiting = False
                    self._manual_place = ""
            self._stop_walk_best_effort()
            if outcome == "arrive":
                return True, ErrorCode.OK, f"{place} 수동 이동 완료"
            return False, ErrorCode.OBSTACLE_BLOCKED, f"{place} 수동 주행 실패 처리됨"

        if nav_mode == "action":
            nav_goal = RunTask.Goal()
            nav_goal.task_id = req.task_id
            nav_goal.operation = "NAVIGATE"
            nav_goal.place_id = place
            nav_goal.destination = req.destination
            nav_goal.timeout = Duration(sec=max(1, int(timeout_s)))

            def on_nav_fb(fb_msg):
                fb = fb_msg.feedback
                self._pub_run_fb(
                    goal_handle, f"nav {fb.phase}", 0.5 * float(fb.progress))

            _, nav_res, err_code, err_txt = self._call_child_action(
                self._nav_cli, nav_goal, timeout_s, goal_handle,
                on_feedback=on_nav_fb)
            if err_code != ErrorCode.OK:
                self._stop_walk_best_effort()
                return False, err_code, f"주행 액션 실패: {err_txt}"
            if not nav_res.success:
                code = nav_res.error_code or ErrorCode.NAV_FAILED
                return False, code, nav_res.message or "주행 실패"
            return True, ErrorCode.OK, nav_res.message or f"{place} 주행 완료"

        return False, ErrorCode.BAD_REQUEST, f"알 수 없는 nav_mode: {nav_mode}"

    def _run_chained_approach(self, place, nav_mode, timeout_s, goal_handle):
        """2단계: 주행 완료 직후 상자 앞 2cm 정밀 접근(/approach_box)을 이어서 실행한다."""
        if not self._approach_cli.wait_for_server(timeout_sec=1.5):
            if nav_mode == "manual" and not self._param("require_approach_in_manual"):
                self.get_logger().warning(
                    f"{place}: /approach_box 서버가 없어 수동 위치를 그대로 사용합니다")
                return True, ErrorCode.OK, "approach_box 생략(수동 모드)"
            return False, ErrorCode.INTERNAL_ERROR, "/approach_box 액션 서버 없음"

        app_timeout = min(float(self._param("approach_timeout_s")),
                          max(5.0, timeout_s))
        app_goal = ApproachBox.Goal()
        app_goal.timeout_sec = float(app_timeout)
        app_goal.dry_run = bool(self._param("approach_dry_run"))
        self._pub_run_fb(goal_handle, f"approach_box start ({place})", 0.6)

        last_visible = [False]

        def on_app_fb(fb_msg):
            fb = fb_msg.feedback
            if fb.visible:
                last_visible[0] = True
            self._pub_run_fb(
                goal_handle,
                f"approach dist={fb.dist_m:.3f}m yaw={fb.yaw_deg:.1f}deg "
                f"hold={fb.hold_sec:.1f}s",
                0.8)

        _, app_res, err_code, err_txt = self._call_child_action(
            self._approach_cli, app_goal, app_timeout + 3.0, goal_handle,
            on_feedback=on_app_fb)
        if err_code != ErrorCode.OK:
            return False, err_code, f"정밀 접근 실패: {err_txt}"
        if not app_res.success:
            fail_code = (ErrorCode.NAV_FAILED if last_visible[0]
                         else ErrorCode.TARGET_NOT_FOUND)
            return (False, fail_code,
                    f"{app_res.message} (dist={app_res.dist_m:.3f}m, "
                    f"yaw={app_res.yaw_deg:.1f}deg)")
        return (True, ErrorCode.OK,
                f"{app_res.message} (dist={app_res.dist_m:.3f}m, "
                f"yaw={app_res.yaw_deg:.1f}deg)")

    def _exec_run_task(self, goal_handle):
        req = goal_handle.request
        res = RunTask.Result()
        if req.operation and req.operation != "NAVIGATE":
            res.success = False
            res.error_code = ErrorCode.BAD_REQUEST
            res.message = f"지원하지 않는 RunTask operation: {req.operation}"
            return self._finish(goal_handle, res)

        total_s = _duration_sec(req.timeout, 168.0)
        deadline = time.monotonic() + total_s
        nav_mode = str(self._param("nav_mode")).strip().lower()
        chain_app = self._should_chain_approach(req.place_id, nav_mode)
        reserve_s = (min(float(self._param("approach_timeout_s")), total_s * 0.35)
                     if chain_app and nav_mode == "action" else 0.0)

        try:
            ok, code, msg = self._run_coarse_nav(
                req, nav_mode, max(5.0, total_s - reserve_s), goal_handle)
            if not ok:
                res.success = False
                res.error_code = code
                res.message = msg
                return self._finish(goal_handle, res)

            if chain_app:
                rem_s = max(5.0, deadline - time.monotonic())
                ok, code, app_msg = self._run_chained_approach(
                    req.place_id, nav_mode, rem_s, goal_handle)
                if not ok:
                    res.success = False
                    res.error_code = code
                    res.message = app_msg
                    return self._finish(goal_handle, res)
                msg = f"{msg} -> {app_msg}"

            self._pub_run_fb(goal_handle, f"arrived {req.place_id}", 1.0)
            res.success = True
            res.error_code = ErrorCode.OK
            res.message = msg
            return self._finish(goal_handle, res)
        except _BridgeCanceled:
            self._stop_walk_best_effort()
            return self._finish(goal_handle, res)

    # ---------- 2. READ_MISSION_INPUT: 바코드 + 신호등 12회 점멸 ----------
    def _exec_read_input(self, goal_handle):
        req = goal_handle.request
        res = ReadMissionInput.Result()
        timeout_s = _duration_sec(req.timeout, 165.0)

        if self._param("reset_on_read_input"):
            with self._state_lock:
                self._slot_to_puck.clear()
                self._puck_to_slot.clear()
                self._placed_by_loc = {loc: {} for loc in LOCATIONS}

        sign_goal = ReadSignBarcode.Goal()
        sign_goal.timeout_sec = float(timeout_s)

        def on_sign_fb(fb_msg):
            fb = fb_msg.feedback
            out = ReadMissionInput.Feedback()
            out.phase = (f"blink {len(fb.sequence)}/12 seq={fb.sequence} "
                         f"barcode={fb.barcode or '?'}")
            goal_handle.publish_feedback(out)

        try:
            _, sign_res, err_code, err_txt = self._call_child_action(
                self._sign_cli, sign_goal, timeout_s + 3.0, goal_handle,
                on_feedback=on_sign_fb)
            if err_code != ErrorCode.OK:
                res.success = False
                res.error_code = (ErrorCode.PERCEPTION_FAILED
                                  if err_code == ErrorCode.TIMEOUT else err_code)
                res.message = f"read_sign_barcode 실패: {err_txt}"
                return self._finish(goal_handle, res)

            seq = list(sign_res.sequence or "")
            res.barcode = sign_res.barcode or ""
            res.colors = seq
            res.end_seen = bool(sign_res.success and len(seq) == 12)
            if not sign_res.success:
                res.success = False
                res.error_code = ErrorCode.PERCEPTION_FAILED
                res.message = sign_res.error_msg or "신호등·바코드 인식 실패"
                return self._finish(goal_handle, res)

            res.success = True
            res.error_code = ErrorCode.OK
            res.message = "신호등·바코드 인식 완료"
            return self._finish(goal_handle, res)
        except _BridgeCanceled:
            return self._finish(goal_handle, res)

    # ---------- 3. LOAD: 상자 퍽 적재 + 임시 ID/슬롯 매핑 ----------
    def _exec_load_box(self, goal_handle):
        req = goal_handle.request
        res = LoadBox.Result()
        res.box_remaining = -1
        timeout_s = _duration_sec(req.timeout, 148.0)

        with self._state_lock:
            free_slots = [s for s in range(STORAGE_SLOTS)
                          if s not in self._slot_to_puck]
            occupied_1based = [s + 1 for s in sorted(self._slot_to_puck)]

        if not free_slots:
            res.success = False
            res.error_code = ErrorCode.STORAGE_FULL
            res.message = "적재함 빈 슬롯 없음"
            return self._finish(goal_handle, res)

        max_load = min(int(self._param("pucks_per_box")), len(free_slots))
        target_1based = [s + 1 for s in free_slots[:max_load]]

        pick_goal = PickPlace.Goal()
        pick_goal.command = PickPlace.Goal.CMD_LOAD_BOX
        pick_goal.max_count = max_load
        pick_goal.slots = target_1based
        pick_goal.occupied_slots = occupied_1based

        def on_pick_fb(fb_msg):
            fb = fb_msg.feedback
            out = LoadBox.Feedback()
            out.phase = f"{fb.phase} ({fb.current_idx}/{fb.total_count})"
            goal_handle.publish_feedback(out)

        try:
            _, pick_res, err_code, err_txt = self._call_child_action(
                self._pick_cli, pick_goal, timeout_s, goal_handle,
                on_feedback=on_pick_fb)
            if err_code != ErrorCode.OK or pick_res is None:
                res.success = False
                res.error_code = err_code
                res.message = f"pick_place 적재 호출 실패: {err_txt}"
                return self._finish(goal_handle, res)

            default_conf = float(self._param("default_puck_conf"))
            obs_confs = list(getattr(pick_res, "observed_puck_conf", []) or [])
            loaded_msgs = []
            with self._state_lock:
                for idx, slot_1 in enumerate(pick_res.used_slots):
                    slot_0 = int(slot_1) - 1
                    cls_name = (pick_res.detected_classes[idx]
                                if idx < len(pick_res.detected_classes) else "")
                    color, shape = _parse_puck_class(cls_name)
                    conf = (float(obs_confs[idx])
                            if idx < len(obs_confs)
                            and 0.0 <= float(obs_confs[idx]) <= 1.0
                            else default_conf)
                    puck_id = self._allocate_puck_id()
                    if 0 <= slot_0 < STORAGE_SLOTS:
                        info = {"puck_id": puck_id, "color": color,
                                "shape": shape, "conf": conf,
                                "place_id": req.place_id}
                        self._slot_to_puck[slot_0] = info
                        if puck_id >= 0:
                            self._puck_to_slot[puck_id] = slot_0
                    msg_item = LoadedPuck()
                    msg_item.slot = max(0, min(STORAGE_SLOTS - 1, slot_0))
                    msg_item.tag_id = int(puck_id)
                    msg_item.color = color
                    msg_item.shape = shape
                    msg_item.confidence = float(conf)
                    loaded_msgs.append(msg_item)

            res.pucks = loaded_msgs

            # 적재 후 상자 위에 남은 퍽 수 확인
            det_after = self._wait_fresh_det3d(0.6, goal_handle)
            if det_after is not None:
                _, _, rem_pucks = self._extract_box_and_pucks(det_after)
                res.box_remaining = len(rem_pucks)
            elif pick_res.success and len(loaded_msgs) > 0:
                res.box_remaining = 0

            if not pick_res.success or not loaded_msgs:
                res.success = False
                if not loaded_msgs and "찾지 못" in (pick_res.message or ""):
                    res.error_code = ErrorCode.PERCEPTION_FAILED
                else:
                    res.error_code = ErrorCode.GRASP_FAILED
                res.message = pick_res.message or "상자 퍽 적재 실패"
                return self._finish(goal_handle, res)

            if res.box_remaining < 0:
                res.box_remaining = 0
            res.success = True
            res.error_code = ErrorCode.OK
            res.message = pick_res.message or f"{len(loaded_msgs)}개 적재 완료"
            return self._finish(goal_handle, res)
        except _BridgeCanceled:
            return self._finish(goal_handle, res)

    # ---------- 4. UNLOAD: 적재함 슬롯 -> 지도 기준 3x3 주사위 격자 배치 ----------
    def _exec_unload_pucks(self, goal_handle):
        req = goal_handle.request
        res = UnloadPucks.Result()
        res.stopped_at = -1
        timeout_s = _duration_sec(req.timeout, 395.0)
        items = list(req.items)

        if not items:
            res.success = False
            res.error_code = ErrorCode.BAD_REQUEST
            res.message = "하역할 items가 비어 있음"
            res.stopped_at = 0
            return self._finish(goal_handle, res)

        # 요청된 puck_id -> 적재함 슬롯(1~12) 조회
        allow_fallback = bool(self._param("allow_direct_slot_fallback"))
        slots_1based = []
        puck_infos = []
        with self._state_lock:
            occupied_1based = [s + 1 for s in sorted(self._slot_to_puck)]
            for idx, item in enumerate(items):
                pid = int(item.puck_id)
                slot_0 = self._puck_to_slot.get(pid)
                if slot_0 is None and allow_fallback and 0 <= pid < STORAGE_SLOTS:
                    slot_0 = pid
                if slot_0 is None:
                    failed = PlacedItem()
                    failed.puck_id = pid
                    failed.cell = int(item.cell)
                    failed.ok = False
                    failed.error_code = ErrorCode.PUCK_NOT_IN_STORAGE
                    res.items = [failed]
                    res.stopped_at = idx
                    res.success = False
                    res.error_code = ErrorCode.PUCK_NOT_IN_STORAGE
                    res.message = f"puck_id={pid}가 적재함에 없음"
                    return self._finish(goal_handle, res)
                slots_1based.append(slot_0 + 1)
                puck_infos.append((slot_0, self._slot_to_puck.get(slot_0, {})))

        # 지도 기준(+Y 위쪽) 3x3 cell 좌표를 현재 로봇 방향(base 기준)으로 변환
        yaw_rad = self._robot_yaw_rad()
        place_xy = []
        for item in items:
            mdx, mdy = _cell_map_offset(item.cell)
            bdx, bdy = _map_to_base_xy(mdx, mdy, yaw_rad)
            place_xy.extend([bdx, bdy])

        pick_goal = PickPlace.Goal()
        pick_goal.command = PickPlace.Goal.CMD_UNLOAD_BOX
        pick_goal.max_count = len(items)
        pick_goal.slots = slots_1based
        pick_goal.occupied_slots = occupied_1based
        pick_goal.place_xy = place_xy

        def on_pick_fb(fb_msg):
            fb = fb_msg.feedback
            out = UnloadPucks.Feedback()
            out.phase = f"{fb.phase} ({fb.current_idx}/{fb.total_count})"
            goal_handle.publish_feedback(out)

        try:
            _, pick_res, err_code, err_txt = self._call_child_action(
                self._pick_cli, pick_goal, timeout_s, goal_handle,
                on_feedback=on_pick_fb)
            if err_code != ErrorCode.OK or pick_res is None:
                res.success = False
                res.error_code = err_code
                res.message = f"pick_place 하역 호출 실패: {err_txt}"
                res.stopped_at = 0
                return self._finish(goal_handle, res)

            handled = max(0, min(len(items), int(pick_res.handled_count)))

            # 2초 안정 대기 후 가능하면 최신 3D 센서로 놓인 위치 오차 측정
            if handled > 0:
                hold_s = float(self._param("unload_hold_s"))
                if hold_s > 0.0:
                    fb = UnloadPucks.Feedback()
                    fb.phase = f"holding_{hold_s:.1f}s"
                    goal_handle.publish_feedback(fb)
                    self._sleep_cancelable(hold_s, goal_handle)

            measured_offsets = self._measure_cell_offsets(yaw_rad, goal_handle)
            placed_results = []
            with self._state_lock:
                loc_map = self._placed_by_loc.setdefault(req.location_id, {})
                for idx in range(handled):
                    item = items[idx]
                    slot_0, info = puck_infos[idx]
                    pid = int(item.puck_id)
                    cell = int(item.cell)
                    self._slot_to_puck.pop(slot_0, None)
                    self._puck_to_slot.pop(pid, None)
                    loc_map[cell] = {
                        "puck_id": pid,
                        "color": info.get("color", ""),
                        "shape": info.get("shape", ""),
                    }
                    placed = PlacedItem()
                    placed.puck_id = pid
                    placed.cell = cell
                    placed.ok = True
                    placed.error_code = ErrorCode.OK
                    off_xy = measured_offsets.get(cell, (0.0, 0.0))
                    placed.offset = Point(x=off_xy[0], y=off_xy[1], z=0.0)
                    placed_results.append(placed)

            if not pick_res.success or handled < len(items):
                fail_idx = min(handled, len(items) - 1)
                fail_item = items[fail_idx]
                if handled < len(items):
                    failed = PlacedItem()
                    failed.puck_id = int(fail_item.puck_id)
                    failed.cell = int(fail_item.cell)
                    failed.ok = False
                    failed.error_code = ErrorCode.PLACE_FAILED
                    placed_results.append(failed)
                res.items = placed_results
                res.stopped_at = fail_idx
                res.success = False
                res.error_code = ErrorCode.PLACE_FAILED
                res.message = pick_res.message or "상자 퍽 하역 실패"
                return self._finish(goal_handle, res)

            res.items = placed_results
            res.stopped_at = -1
            res.success = True
            res.error_code = ErrorCode.OK
            res.message = pick_res.message or f"{handled}개 하역 완료"
            return self._finish(goal_handle, res)
        except _BridgeCanceled:
            return self._finish(goal_handle, res)

    def _measure_cell_offsets(self, yaw_rad, goal_handle):
        """최신 /detections_3d가 있으면 각 cell에 놓인 퍽의 실측 오차(m)를 계산한다."""
        det_msg = self._wait_fresh_det3d(0.5, goal_handle)
        if det_msg is None:
            return {}
        box_x, box_y, pucks = self._extract_box_and_pucks(det_msg)
        offsets = {}
        for puck in pucks:
            mdx, mdy = _base_to_map_xy(
                puck["x"] - box_x, puck["y"] - box_y, yaw_rad)
            best_cell, best_dist, best_off = -1, CELL_MATCH_RADIUS_M, (0.0, 0.0)
            for cell in range(NUM_CELLS):
                cx, cy = _cell_map_offset(cell)
                dist = math.hypot(mdx - cx, mdy - cy)
                if dist < best_dist:
                    best_cell = cell
                    best_dist = dist
                    best_off = (mdx - cx, mdy - cy)
            if best_cell >= 0:
                offsets[best_cell] = best_off
        return offsets

    # ---------- 5. VERIFY_LOCATION: 팔 끝 ArduCam 탑뷰 우선 + 3D 센서 폴백 9칸 재검증 ----------
    def _move_arm_cmd(self, command, timeout_s, goal_handle,
                      occupied_1based=None):
        """/pick_place 서버가 켜져 있을 때 관측/홈 명령을 실행하고 Result를 돌려준다."""
        if not self._pick_cli.wait_for_server(timeout_sec=0.4):
            return None
        goal = PickPlace.Goal()
        goal.command = command
        if occupied_1based:
            goal.occupied_slots = list(occupied_1based)

        def on_obs_fb(fb_msg):
            fb = fb_msg.feedback
            if fb.phase:
                out = ObserveLocation.Feedback()
                out.phase = fb.phase
                goal_handle.publish_feedback(out)

        _, pick_res, err_code, _ = self._call_child_action(
            self._pick_cli, goal, timeout_s, goal_handle,
            on_feedback=on_obs_fb, wait_server_s=0.4)
        return pick_res if err_code == ErrorCode.OK else None

    @staticmethod
    def _extract_arducam_observation(pick_res, default_box_x, default_box_y):
        """PickPlace CMD_OBSERVE의 ArduCam 탑뷰 결과에서 (box_x, box_y, pucks)를 꺼낸다."""
        if pick_res is None:
            return None
        if getattr(pick_res, "observe_source", "") != "arducam":
            return None
        box_xy = list(getattr(pick_res, "observed_box_xy", []) or [])
        box_x = float(box_xy[0]) if len(box_xy) >= 2 else float(default_box_x)
        box_y = float(box_xy[1]) if len(box_xy) >= 2 else float(default_box_y)
        xyz = list(getattr(pick_res, "observed_puck_xyz", []) or [])
        classes = list(getattr(pick_res, "detected_classes", []) or [])
        confs = list(getattr(pick_res, "observed_puck_conf", []) or [])
        n_pucks = min(len(classes), len(xyz) // 3)
        pucks = []
        for idx in range(n_pucks):
            color, shape = _parse_puck_class(classes[idx])
            if not color or not shape:
                continue
            px = float(xyz[3 * idx])
            py = float(xyz[3 * idx + 1])
            pz = float(xyz[3 * idx + 2])
            conf = float(confs[idx]) if idx < len(confs) else 0.90
            pucks.append({
                "x": px, "y": py, "z": pz,
                "color": color, "shape": shape, "score": conf,
            })
        return box_x, box_y, pucks

    @classmethod
    def _match_pucks_to_cells(cls, box_x, box_y, pucks, yaw_rad, expected_loc):
        """검출된 base 기준 퍽 목록을 지도 기준(+Y 위쪽) 3x3 cell(0~8)로 매칭한다."""
        cell_hits = {}
        for puck in pucks:
            mdx, mdy = _base_to_map_xy(
                puck["x"] - box_x, puck["y"] - box_y, yaw_rad)
            best_cell, best_dist = -1, CELL_MATCH_RADIUS_M
            best_off = (0.0, 0.0)
            for cell in range(NUM_CELLS):
                cx, cy = _cell_map_offset(cell)
                dist = math.hypot(mdx - cx, mdy - cy)
                if dist < best_dist:
                    best_cell = cell
                    best_dist = dist
                    best_off = (mdx - cx, mdy - cy)
            if best_cell >= 0 and (best_cell not in cell_hits
                                   or best_dist < cell_hits[best_cell]["dist"]):
                cell_hits[best_cell] = {
                    "dist": best_dist,
                    "offset": best_off,
                    "color": puck["color"],
                    "shape": puck["shape"],
                }

        cells_out = []
        used_ids = set()
        for cell in range(NUM_CELLS):
            obs = CellObservation()
            obs.cell = cell
            hit = cell_hits.get(cell)
            if hit is None:
                obs.state = "empty"
                obs.tag_id = -1
                obs.offset = Point(x=0.0, y=0.0, z=0.0)
            else:
                obs.state = "occupied"
                obs.offset = Point(
                    x=hit["offset"][0], y=hit["offset"][1], z=0.0)
                obs.tag_id = cls._match_observed_puck_id(
                    cell, hit, expected_loc, used_ids)
                if obs.tag_id >= 0:
                    used_ids.add(obs.tag_id)
            cells_out.append(obs)
        return cells_out, cell_hits

    def _exec_observe_location(self, goal_handle):
        req = goal_handle.request
        res = ObserveLocation.Result()
        timeout_s = _duration_sec(req.timeout, 30.0)
        use_obs_pose = bool(self._param("use_arm_observe_pose"))
        wait_s = min(float(self._param("observe_wait_s")), max(1.0, timeout_s - 2.0))

        with self._state_lock:
            occupied_1based = [s + 1 for s in sorted(self._slot_to_puck)]

        try:
            arducam_obs = None
            obs_source = "det3d"
            if use_obs_pose:
                fb = ObserveLocation.Feedback()
                fb.phase = "moving_arm_observe"
                goal_handle.publish_feedback(fb)
                pick_res = self._move_arm_cmd(
                    PickPlace.Goal.CMD_OBSERVE, min(18.0, timeout_s * 0.65),
                    goal_handle, occupied_1based=occupied_1based)
                arducam_obs = self._extract_arducam_observation(
                    pick_res,
                    float(self._param("default_box_x")),
                    float(self._param("default_box_y")))

            if arducam_obs is not None:
                box_x, box_y, pucks = arducam_obs
                obs_source = "arducam"
            else:
                fb = ObserveLocation.Feedback()
                fb.phase = "waiting_detections_3d"
                goal_handle.publish_feedback(fb)
                det_msg = self._wait_fresh_det3d(wait_s, goal_handle)

                if use_obs_pose:
                    self._move_arm_cmd(
                        PickPlace.Goal.CMD_HOME, min(12.0, timeout_s * 0.4),
                        goal_handle, occupied_1based=occupied_1based)

                if det_msg is None:
                    res.success = False
                    res.error_code = ErrorCode.PERCEPTION_FAILED
                    res.message = (
                        "ArduCam 탑뷰 비활성 및 /detections_3d 신규 관측 프레임 수신 실패")
                    return self._finish(goal_handle, res)

                box_x, box_y, pucks = self._extract_box_and_pucks(det_msg)

            yaw_rad = self._robot_yaw_rad()
            with self._state_lock:
                expected_loc = dict(
                    self._placed_by_loc.get(req.location_id, {}))

            cells_out, cell_hits = self._match_pucks_to_cells(
                box_x, box_y, pucks, yaw_rad, expected_loc)

            res.cells = cells_out
            res.success = True
            res.error_code = ErrorCode.OK
            res.message = (
                f"{req.location_id} {len(cell_hits)}개 퍽 실측 관측 완료 "
                f"(source={obs_source})")
            return self._finish(goal_handle, res)
        except _BridgeCanceled:
            return self._finish(goal_handle, res)

    @staticmethod
    def _match_observed_puck_id(cell, hit, expected_loc, used_ids):
        """실측 검출된 퍽의 색상·모양이 기록된 배치 정보와 일치할 때 임시 ID를 매칭한다."""
        cand = expected_loc.get(cell)
        if cand is not None and cand["puck_id"] not in used_ids:
            color_ok = (not cand["color"] or not hit["color"]
                        or cand["color"] == hit["color"])
            shape_ok = (not cand["shape"] or not hit["shape"]
                        or cand["shape"] == hit["shape"])
            if color_ok and shape_ok:
                return int(cand["puck_id"])

        for info in expected_loc.values():
            pid = int(info["puck_id"])
            if pid in used_ids:
                continue
            if (info["color"] == hit["color"]
                    and info["shape"] == hit["shape"]):
                return pid
        return -1


def main():
    rclpy.init()
    node = MissionBridge()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
