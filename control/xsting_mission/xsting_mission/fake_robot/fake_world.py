"""backend:=fake용 가짜 경기장 (좌표 mm, yaw deg, 웹은 snapshot·static_map으로 읽음)."""
import base64
import copy
import math
import random
import threading
import time

import numpy as np

from ..core.contract import Canceled
from ..core.rules import LOCATIONS, STORAGE_SLOTS
from ..map.arena_map import APPROACH_MM, PLAN_CELL_MM, ArenaMap

BARCODES = ("P2-01", "P2-02", "P2-03", "P2-04")
PUCK_OFFSET_MM = (-40, 40)  # 시작 박스 위 퍽 두 개의 X 오프셋 (화면 표시용)

# 가짜 로봇이 흉내 낼 수 있는 실패 {행동: {코드: 옵션 설명 또는 None}}
# SIGNAL_INCOMPLETE는 주입 전용: 그 횟수만큼 신호를 덜 읽고, 한도 이상이면 실패
FAULT_CODES = {
    "READ_MISSION_INPUT": {"PERCEPTION_FAILED": "탐색 후 찾을 확률 %",
                           "SIGNAL_INCOMPLETE": "덜 읽을 횟수"},
    "GOTO": {"OBSTACLE_BLOCKED": "멈출 지점 %", "NO_PROGRESS": "멈출 지점 %",
             "NAV_FAILED": "멈출 지점 %", "LOCALIZATION_LOST": "멈출 지점 %",
             "NO_PATH": None},
    "LOAD": {"GRASP_FAILED": "실을 개수",
             "PERCEPTION_FAILED": "탐색 후 찾을 확률 %"},
    "UNLOAD": {"PLACE_FAILED": "실패할 항목 번호"},
    "VERIFY_LOCATION": {"PERCEPTION_FAILED": "탐색 후 찾을 확률 %"},
}
# 어느 행동에든 걸 수 있는 실패
COMMON_FAULTS = {"ROBOT_FAULT": None, "INTERNAL_ERROR": None,
                 "BAD_RESULT": None}
FAULT_DEFAULT_OPTION = {"멈출 지점 %": 50, "실을 개수": 0, "실패할 항목 번호": 2,
                        "탐색 후 찾을 확률 %": 0, "덜 읽을 횟수": 1}
SIGNAL_READ_LIMIT = 3  # 신호를 다 못 읽으면 다시 읽는 최대 횟수 (처음 포함)
SIGNAL_MISSED_RANGE = (1, 3)  # 덜 읽을 때 놓치는 점멸 개수
# 무작위 인식 실패: fail_rate %로 실패하면 한 바퀴 돌아 search_success_rate %로 찾는다
PERCEPTION_DEFAULT = {"fail_rate": 0, "search_success_rate": 70}
OBSTACLE_TTL_S = 5.0  # 임시 장애물이 길을 막는 경기장 시간
OBSTACLE_SIZE_MM = 250
STOP_PERCENT_RANGE = (1, 90)  # GOTO 실패 주입의 멈출 지점 (경로 길이의 %)


class FakeWorld(ArenaMap):

    # ---------- 초기화 ----------
    def __init__(self, scenario, arena, time_scale=1.0, speed_mps=0.3):
        """arena: config/arena.yaml을 읽은 dict."""
        places = arena["places"]
        self.lock = threading.Lock()
        self.time_scale = time_scale
        self.speed_mmps = speed_mps * 1000
        self.scenario = scenario
        self.commander = None  # 중계 노드가 발행한 MissionState
        self.web_last = None  # 화면에서 보낸 마지막 명령과 결과
        self.decisions = []  # 중계 노드의 LLM 판단 기록
        self.decisions_version = 0  # 카드를 교체해도 화면이 다시 불러오게 올린다
        self.replay = None  # 다시 실행 중이면 {"run_id", "turn", "total"}
        self.paused = False
        self.abort = False  # 진행 중인 명령을 취소한다
        self._fault = None  # 지금 실행 중인 행동에 걸린 실패

        super().__init__(arena)
        self._base_blocked = self.blocked.copy()  # 임시 장애물 없을 때의 막힌 칸
        self.temp_obstacles = []  # [{x, y, size_mm, expires}]
        self._sim_t, self._sim_last = 0.0, time.monotonic()
        self._clock_lock = threading.Lock()
        self.home = tuple(places["HOME"]["approach"])
        # approach가 없는 장소는 GOTO마다 대상 둘레에서 무작위로 정한다
        self.approaches = {name: tuple(place["approach"])
                           for name, place in places.items()
                           if place.get("approach")}
        self._reach = self._reachable()
        self.reset()

    def reset(self, problem=None):
        """시나리오 처음 상태로 되돌린다 (problem을 주면 지난 판 문제를 그대로)."""
        scenario = self.scenario
        if problem is not None:
            scenario = {
                **scenario, "barcode": problem["barcode"],
                "signal": problem["signal"], "boxes": problem["boxes"],
                "fail_once": problem.get("fail_once", []),
                "faults": problem.get("faults", []),
                "perception": problem.get("perception",
                                          scenario.get("perception"))}
        pick = random.Random()  # 신호·바코드는 초기화마다 달라지게 시드 없이
        with self.lock:
            self.rng = random.Random(scenario.get("seed", 0))
            # 인식 확률을 바꿔도 다른 무작위 결과가 그대로이도록 따로 뽑는다
            self.perception_rng = random.Random(scenario.get("seed", 0) + 1)
            self.barcode = (pick.choice(BARCODES)
                            if scenario["barcode"] == "random"
                            else scenario["barcode"])

            # 웹에서 넣은 실패도 여기 쌓이고, 초기화하면 시나리오 것으로 돌아간다
            self.faults, self._fault_seq = [], 0
            # 예전 형식 "LOAD:S3" = 그 박스의 첫 LOAD를 집기 실패
            for key in scenario.get("fail_once", []):
                action, target = key.split(":")
                self._add_fault_locked({"action": action,
                                        "code": "GRASP_FAILED",
                                        "target": target})
            for fault in scenario.get("faults", []) or []:
                self._add_fault_locked(fault)
            self.localization = "ok"
            self.temp_obstacles = []
            self.blocked = self._base_blocked.copy()
            self.perception = {**PERCEPTION_DEFAULT,
                               **(scenario.get("perception") or {})}

            # 출제 규칙대로 풀 수 있는 조합이 하나 이상인 문제만 낸다
            while True:
                if scenario["boxes"] == "random":
                    self.boxes = self._random_boxes(pick)
                else:
                    self.boxes = {
                        box: [dict(puck) if puck else None for puck in slots]
                        for box, slots in scenario["boxes"].items()}
                options = self._solvable_counts()
                if scenario["signal"] != "random":
                    self.signal = list(scenario["signal"])
                    break
                if options:
                    red, yellow, green = pick.choice(options)
                    self.signal = ["R"] * red + ["Y"] * yellow + ["G"] * green
                    pick.shuffle(self.signal)
                    break
                if scenario["boxes"] != "random":
                    raise ValueError("fake_scenario의 퍽 구성으로 풀 수 있는 신호가 없음")

            self.storage = {}  # slot -> 퍽
            self.placed = {location: [None] * 9 for location in LOCATIONS}
            self.verified = set()
            self.paused = False
            self.abort = False
            self.initial_pucks = {
                box: [dict(puck) for puck in slots if puck]
                for box, slots in self.boxes.items()}
            self.here = "HOME"
            self.pose = {"x": float(self.home[0]), "y": float(self.home[1]),
                         "yaw": float(self.home[2])}

            # 화면 표시용
            self.light = None
            self.seen_signal = []
            self.barcode_read = False
            self.command = None
            self.history = []
            self.path = []

    @staticmethod
    def _random_boxes(pick):
        """퍽 12개의 색·모양을 무작위로 정해 시작 박스 6곳에 2개씩 나눈다."""
        ids = list(range(12))
        pick.shuffle(ids)
        pucks = [{"id": puck_id, "color": pick.choice("RG"),
                  "shape": pick.choice(("cylinder", "box"))}
                 for puck_id in ids]
        return {f"S{k + 1}": pucks[2 * k:2 * k + 2] for k in range(6)}

    def _solvable_counts(self):
        """지금 퍽 구성으로 풀 수 있는 (R, Y, G) 신호 횟수 조합 목록."""
        pucks = [puck for slots in self.boxes.values()
                 for puck in slots if puck]
        red = [puck for puck in pucks if puck["color"] == "R"]
        green = [puck for puck in pucks if puck["color"] == "G"]
        options = []
        for red_count in range(1, 7):
            for yellow_count in range(1, 7):
                green_count = 12 - red_count - yellow_count
                if (not 1 <= green_count <= 6 or red_count > len(red)
                        or green_count > len(green)):
                    continue
                # 남는 퍽이 Y가 되고, Y는 한 모양으로 통일할 수 있어야 한다
                red_left = len(red) - red_count
                green_left = len(green) - green_count
                if any(sum(puck["shape"] == shape for puck in red) >= red_left
                       and sum(puck["shape"] == shape for puck in green)
                       >= green_left
                       for shape in ("cylinder", "box")):
                    options.append((red_count, yellow_count, green_count))
        return options

    # ---------- 판단 기록·설정 ----------
    def add_decision(self, record):
        """중계 노드의 판단 기록을 넣는다 (같은 turn 결과는 합치고 새 실행이면 비운다)."""
        with self.lock:
            self.decisions_version += 1
            last_run_id = (self.decisions[-1]["run_id"] if self.decisions
                           else None)
            # 화면 명령("web") 뒤에 중계 노드가 시작하면 이어 붙인다
            if last_run_id is not None and last_run_id != record["run_id"] \
                    and not (last_run_id == "web"
                             and not str(record["run_id"]).startswith(
                                 ("log:", "replay:"))):
                self.decisions = []
            if "outcome" in record and record.get("decider") != "user":
                for decision in reversed(self.decisions):
                    if (decision["turn"] == record["turn"]
                            and decision.get("decider") != "user"):
                        decision["outcome"] = record["outcome"]
                        break
                return
            index = next(
                (index for index in range(len(self.decisions) - 1, -1, -1)
                 if record.get("decider") != "user"
                 and self.decisions[index].get("turn") == record.get("turn")
                 and (self.decisions[index].get("prefetch") or {}).get(
                     "status") in ("requested", "answered")),
                None)
            if index is None:
                self.decisions.append(record)
                del self.decisions[:-100]
            else:  # "미리 판단" 카드를 실제 판단(또는 버림)으로 바꾼다
                self.decisions[index] = record

    def decision_list(self):
        with self.lock:
            return copy.deepcopy(self.decisions)

    def problem(self):
        """지금 문제 (판마다 problem.json으로 저장된다)."""
        with self.lock:
            return {"barcode": self.barcode, "signal": "".join(self.signal),
                    "boxes": self.initial_pucks}

    def set_time_scale(self, value):
        with self.lock:
            self.time_scale = max(0.25, min(50.0, float(value)))
            return self.time_scale

    def set_perception(self, fail_rate, search_success_rate):
        with self.lock:
            self.perception = {
                "fail_rate": max(0, min(100, float(fail_rate))),
                "search_success_rate":
                    max(0, min(100, float(search_success_rate)))}
            return dict(self.perception)

    # ---------- 화면용 ----------
    def static_map(self):
        height, width = self.map_occ.shape
        return {
            "width": width, "height": height, "res_mm": self.map_res_mm,
            "origin_mm": list(self.map_origin_mm),
            "occupied_b64": base64.b64encode(
                self.map_occ.astype(np.uint8).tobytes()).decode(),
            "targets": {name: list(xy) for name, xy in self.targets.items()},
            "approaches": {name: list(pose)
                           for name, pose in self.approaches.items()},
            # 로봇 중심이 갈 수 없는 칸 (화면에 옅게 칠한다)
            "blocked_b64": base64.b64encode(
                self.blocked.astype(np.uint8).tobytes()).decode(),
            "plan_w": self.grid_w, "plan_h": self.grid_h,
            "plan_cell_mm": PLAN_CELL_MM,
            "virtual_walls": [
                {"id": wall_id, "from": wall["from"], "to": wall["to"]}
                for wall_id, wall in
                (self.arena.get("virtual_walls") or {}).items()],
            "wall_mm": self.arena["wall_thickness"],
            "box_mm": self.arena["box_size"],
            "obstacles": [{"name": obstacle["name"],
                           "center": obstacle["center"],
                           "size": obstacle["size"]}
                          for obstacle in self.arena.get("obstacles") or []],
            "start_zone_mm": self.arena["start_zone"],
            "light_mm": self.arena["signal_light"],
            "barcode_mm": self.arena["barcode"],
            "puck_offset_mm": list(PUCK_OFFSET_MM),
            "footprint_mm": self.arena["robot"]["footprint"],
            # 웹 "실패 주입" 카드에서 고를 수 있는 실패
            "fault_codes": {action: {**codes, **COMMON_FAULTS}
                            for action, codes in FAULT_CODES.items()},
            "fault_default_option": FAULT_DEFAULT_OPTION,
        }

    def snapshot(self):
        with self.lock:
            self._expire_obstacles()
            now = self.sim_now()
            decisions_key = (
                f"{self.decisions[0]['run_id'] if self.decisions else ''}:"
                f"{len(self.decisions)}:"
                f"{sum('outcome' in decision for decision in self.decisions)}"
                f":{self.decisions_version}")
            return copy.deepcopy({
                "temp_obstacles": [
                    {**self._obstacle_info(obstacle),
                     "left_s": round(obstacle["expires"] - now, 1)}
                    for obstacle in self.temp_obstacles],
                "pose": self.pose, "path": self.path, "light": self.light,
                "seen_signal": self.seen_signal,
                "barcode": self.barcode if self.barcode_read else None,
                "boxes": self.boxes,
                "storage": {str(slot): puck
                            for slot, puck in self.storage.items()},
                "placed": self.placed,
                "verified": sorted(self.verified), "here": self.here,
                "command": self.command, "history": self.history[-30:],
                "commander": self.commander, "time_scale": self.time_scale,
                "web_last": self.web_last,
                "answer": {"barcode": self.barcode,
                           "signal": "".join(self.signal),
                           "pucks": self.initial_pucks},
                "paused": self.paused, "replay": self.replay,
                "faults": self.faults, "perception": self.perception,
                "decisions_key": decisions_key,
            })

    def begin(self, name, args, source):
        with self.lock:
            self.abort = False
            self.command = {"name": name, "args": args, "phase": "",
                            "started": time.time(), "source": source}

    def set_phase(self, phase):
        with self.lock:
            if self.command:
                self.command["phase"] = phase

    def end(self, name, args, result):
        with self.lock:
            started = self.command["started"] if self.command else time.time()
            source = self.command["source"] if self.command else "?"
            self.history.append({
                "t": time.time(), "started": started,
                "duration_s": round(time.time() - started, 2),
                "source": source, "name": name, "args": args,
                "success": bool(result.get("success")),
                "error": result.get("error"), "result": result})
            self.command = None

    # ---------- 시간과 취소 ----------
    def sim_now(self):
        """경기장 시간(초). 배속을 곱하고 일시정지 동안은 흐르지 않는다."""
        with self._clock_lock:
            now = time.monotonic()
            if not self.paused:
                self._sim_t += (now - self._sim_last) * self.time_scale
            self._sim_last = now
            return self._sim_t

    def sleep(self, seconds, is_canceled):
        """경기장 시간으로 기다린다 (배속 변경이 진행 중인 대기에도 반영된다)."""
        left = seconds
        while left > 0:
            if is_canceled() or self.abort:
                raise Canceled()
            time.sleep(0.02)
            self.sim_now()
            if not self.paused:
                left -= 0.02 * self.time_scale

    # ---------- 임시 장애물 ----------
    def _expire_obstacles(self):
        """시간이 지난 임시 장애물을 지운다 (잠금을 잡은 상태에서)."""
        now = self.sim_now()
        alive = [obstacle for obstacle in self.temp_obstacles
                 if obstacle["expires"] > now]
        if len(alive) != len(self.temp_obstacles):
            self.temp_obstacles = alive
            self._apply_temp_obstacles()

    def _apply_temp_obstacles(self):
        """막힌 칸 = 원래 막힌 칸 + 임시 장애물 둘레(코스트맵 패딩만큼)."""
        blocked = self._base_blocked.copy()
        if self.temp_obstacles:
            origin_x, origin_y = self.map_origin_mm
            centers_x = origin_x + (np.arange(self.grid_w) + 0.5) * PLAN_CELL_MM
            centers_y = origin_y + (np.arange(self.grid_h) + 0.5) * PLAN_CELL_MM
            grid_x, grid_y = np.meshgrid(centers_x, centers_y)
            pad = self.arena["costmap"]["padding"]
            for obstacle in self.temp_obstacles:
                width, height = obstacle["size_mm"]
                dx = np.maximum(np.abs(grid_x - obstacle["x"]) - width / 2, 0)
                dy = np.maximum(np.abs(grid_y - obstacle["y"]) - height / 2, 0)
                blocked |= np.hypot(dx, dy) <= pad
        self.blocked = blocked

    @staticmethod
    def _obstacle_info(obstacle):
        """결과에 싣는 장애물 정보 (실물은 사라질 시각을 모르므로 싣지 않는다)."""
        return {"x": obstacle["x"], "y": obstacle["y"],
                "size_mm": list(obstacle["size_mm"])}

    # ---------- 주행 ----------
    def _reachable(self):
        """HOME에서 갈 수 있는 칸 (도착 자리가 갈 수 없는 곳에 생기지 않게)."""
        start = self._cell(*self.home[:2])
        seen, stack = {start}, [start]
        while stack:
            i, j = stack.pop()
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbor = (i + di, j + dj)
                if neighbor not in seen and self._free(*neighbor):
                    seen.add(neighbor)
                    stack.append(neighbor)
        return seen

    def _arrival(self, name):
        """GOTO 도착 자리: approach가 있으면 그 자리, 없으면 대상 둘레 무작위."""
        if name in self.approaches:
            return self.approaches[name]
        target_x, target_y = self.targets[name]
        for _ in range(500):
            distance = self.rng.uniform(*APPROACH_MM)
            angle = self.rng.uniform(0, 2 * math.pi)
            x = round(target_x + distance * math.cos(angle))
            y = round(target_y + distance * math.sin(angle))
            if self._cell(x, y) in self._reach:
                yaw = math.degrees(math.atan2(target_y - y, target_x - x))
                return (x, y, round(yaw, 1))
        return None

    def _turn_to(self, yaw, is_canceled):
        while True:
            with self.lock:
                diff = (yaw - self.pose["yaw"] + 180) % 360 - 180
                step = 180 * 0.03 * self.time_scale  # 초당 180도
                if abs(diff) <= step:
                    self.pose["yaw"] = yaw
                    return
                self.pose["yaw"] += math.copysign(step, diff)
            self.sleep(0.03, is_canceled)

    def drive_to(self, place, is_canceled, feedback):
        fault = self._fault
        if not (fault and fault["action"] == "GOTO"):
            fault = None
        if fault and fault["code"] == "NO_PATH":
            return {"success": False, "error": "NO_PATH",
                    "message": "주입한 실패: 경로를 찾지 못함"}
        with self.lock:
            self._expire_obstacles()

        # 도착 자리가 임시 장애물 근처에 뽑히면 다른 자리로
        for _ in range(5):
            arrival = self._arrival(place)
            path = (None if arrival is None else
                    self.plan(arrival, (self.pose["x"], self.pose["y"])))
            if path is not None:
                break
        if path is None:
            if self.temp_obstacles:  # 임시 장애물 때문에 돌아갈 길이 없다
                nearest = min(
                    self.temp_obstacles,
                    key=lambda obstacle: math.hypot(
                        obstacle["x"] - self.pose["x"],
                        obstacle["y"] - self.pose["y"]))
                return {"success": False, "error": "OBSTACLE_BLOCKED",
                        "remaining_mm": None,
                        "message": "길 위의 장애물 때문에 돌아갈 길이 없음",
                        "obstacle": self._obstacle_info(nearest),
                        "blocked_segment": {"from": self.here, "to": place}}
            return {"success": False, "error": "NO_PATH"}

        # 실패 주입: 멈출 거리. 장애물 놓을 자리가 경로 안에 남도록 끝쪽은 줄인다
        arrival_x, arrival_y, arrival_yaw = arrival
        total = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
        ahead = OBSTACLE_SIZE_MM / 2 + self.arena["costmap"]["padding"] + 150
        stop_at = None
        if fault and fault["option"] is not None:
            percent = min(max(fault["option"], STOP_PERCENT_RANGE[0]),
                          STOP_PERCENT_RANGE[1])
            stop_at = min(total * percent / 100, max(0.0, total - ahead))

        start_here, done = self.here, 0.0
        with self.lock:
            self.path = [list(point) for point in path]
        for (x0, y0), (x1, y1) in zip(path, path[1:]):
            segment = math.hypot(x1 - x0, y1 - y0)
            if segment < 1:
                continue
            self._turn_to(math.degrees(math.atan2(y1 - y0, x1 - x0)),
                          is_canceled)
            moved = 0.0
            while moved < segment:
                moved = min(segment,
                            moved + self.speed_mmps * 0.03 * self.time_scale)
                if stop_at is not None and done + moved >= stop_at:
                    moved = stop_at - done
                    with self.lock:
                        self.pose["x"] = x0 + (x1 - x0) * moved / segment
                        self.pose["y"] = y0 + (y1 - y0) * moved / segment
                    return self._stop_midway(
                        fault, place, start_here, total - stop_at,
                        self._point_on_path(path, stop_at + ahead))
                with self.lock:
                    self.pose["x"] = x0 + (x1 - x0) * moved / segment
                    self.pose["y"] = y0 + (y1 - y0) * moved / segment
                self.sleep(0.03, is_canceled)
            done += segment
            left_m = math.hypot(arrival_x - x1, arrival_y - y1) / 1000
            feedback(f"moving {left_m:.2f} m left")

        self._turn_to(arrival_yaw, is_canceled)
        with self.lock:
            self.path = []
            self.here = place
        target_x, target_y = self.targets[place]
        return {"success": True,
                "dist_to_target_mm": round(math.hypot(arrival_x - target_x,
                                                      arrival_y - target_y))}

    @staticmethod
    def _point_on_path(path, dist):
        """경로를 따라 dist(mm) 간 지점 (끝을 넘으면 끝점)."""
        for a, b in zip(path, path[1:]):
            segment = math.dist(a, b)
            if dist <= segment and segment > 0:
                return (a[0] + (b[0] - a[0]) * dist / segment,
                        a[1] + (b[1] - a[1]) * dist / segment)
            dist -= segment
        return tuple(path[-1])

    def _stop_midway(self, fault, place, start_here, remaining, obstacle_xy):
        """GOTO 실패 주입으로 중간에 멈춘 결과 (장애물이면 로봇 앞에 놓는다)."""
        code = fault["code"]
        with self.lock:
            self.path = []
            self.here = "UNKNOWN"
            if code == "LOCALIZATION_LOST":
                self.localization = "lost"
        result = {
            "success": False, "error": code, "remaining_mm": round(remaining),
            "message": {
                "OBSTACLE_BLOCKED":
                    "주입한 실패: 길 위의 장애물 때문에 돌아갈 길이 없음",
                "NO_PROGRESS": "주입한 실패: 장애물은 안 보이는데 앞으로 가지 못함",
                "NAV_FAILED": "주입한 실패: 주행이 중단됨",
                "LOCALIZATION_LOST": "주입한 실패: 위치 추정을 잃고 복구하지 못함",
            }[code]}
        if code == "OBSTACLE_BLOCKED":
            # 장애물 둘레보다 멀리 놓아야 로봇이 막힌 칸에 갇히지 않는다
            obstacle = {"x": round(obstacle_xy[0]), "y": round(obstacle_xy[1]),
                        "size_mm": [OBSTACLE_SIZE_MM, OBSTACLE_SIZE_MM],
                        "expires": self.sim_now() + OBSTACLE_TTL_S}
            with self.lock:
                self.temp_obstacles.append(obstacle)
                self._apply_temp_obstacles()
            result["obstacle"] = self._obstacle_info(obstacle)
            result["blocked_segment"] = {"from": start_here, "to": place}
        return result

    # ---------- 실패 주입 ----------
    def _add_fault_locked(self, spec):
        """실패 하나를 목록에 넣는다 (잠금을 잡은 상태에서, 틀린 값이면 ValueError)."""
        action, code = spec.get("action"), spec.get("code")
        codes = ({**FAULT_CODES.get(action, {}), **COMMON_FAULTS}
                 if action in FAULT_CODES else {})
        if code not in codes:
            raise ValueError(f"{action}에서 낼 수 없는 실패: {code}")
        label = codes[code]
        option = spec.get("option")
        option = (FAULT_DEFAULT_OPTION.get(label) if option in (None, "")
                  else int(option))
        if label == "멈출 지점 %":
            option = min(max(option, STOP_PERCENT_RANGE[0]),
                         STOP_PERCENT_RANGE[1])
        self._fault_seq += 1
        fault = {"id": self._fault_seq, "action": action, "code": code,
                 "target": spec.get("target") or None,
                 "option": option if label else None, "option_label": label,
                 "times": max(1, int(spec.get("times", 1)))}
        # 다시 실행: 이 행동의 인식도 처음에 실패하고 이 확률로 찾는다
        if spec.get("perception_option") is not None:
            fault["perception_option"] = int(spec["perception_option"])
        self.faults.append(fault)
        return fault

    def add_fault(self, spec):
        with self.lock:
            return dict(self._add_fault_locked(spec))

    def remove_fault(self, fault_id):
        with self.lock:
            self.faults = [fault for fault in self.faults
                           if fault["id"] != fault_id]

    def _take_fault(self, name, target):
        """그 행동·대상에 걸린 실패를 하나 꺼낸다 (남은 횟수를 줄인다)."""
        for fault in self.faults:
            if fault["action"] == name and fault["target"] in (None, target):
                fault["times"] -= 1
                if fault["times"] <= 0:
                    self.faults.remove(fault)
                return dict(fault)
        return None

    @staticmethod
    def _break_result(name, result):
        """결과를 규약(contract.py)에 어긋나게 바꿔 BAD_RESULT 검사를 시험한다."""
        drop = {"READ_MISSION_INPUT": "barcode", "LOAD": "box_remaining",
                "UNLOAD": "items", "VERIFY_LOCATION": "cells"}.get(name)
        if drop:
            result.pop(drop, None)
        elif name == "GOTO":
            result["robot"]["here"] = "SOMEWHERE"

    # ---------- 행동 실행 ----------
    def robot_status(self):
        """매 결과에 함께 싣는 로봇 상태 (MissionState.robot)."""
        pose = {"x": round(self.pose["x"]), "y": round(self.pose["y"]),
                "yaw": round(self.pose["yaw"], 1)}
        lost = self.localization == "lost"
        return {"status": "OK", "error": None, "here": self.here,
                "pose": None if lost else pose,
                "localization": self.localization, "arm_stowed": True,
                "gripper": "empty"}

    def run(self, name, args, is_canceled, feedback):
        handler = getattr(self, "_" + name.lower(), None)
        if handler is None:
            return {"success": False, "error": "UNKNOWN_COMMAND",
                    "robot": self.robot_status()}
        if name == "GOTO":
            target = args.get("t")
        else:
            target = None if name == "READ_MISSION_INPUT" else self.here
        with self.lock:
            # 잃었던 위치 추정은 다음 행동 전에 사람이 되돌렸다고 본다
            self.localization = "ok"
            fault = self._take_fault(name, target)

        self._fault = fault
        try:
            if fault and fault["code"] in ("ROBOT_FAULT", "INTERNAL_ERROR"):
                reason = ("모터 과전류" if fault["code"] == "ROBOT_FAULT"
                          else "예상 못 한 예외")
                result = {"success": False, "error": fault["code"],
                          "message": "주입한 실패: " + reason}
            else:
                result = handler(args, is_canceled, feedback)
        except Canceled:
            with self.lock:
                self.path = []
            raise
        finally:
            self._fault = None

        result["robot"] = self.robot_status()
        if fault and fault["code"] == "ROBOT_FAULT":
            result["robot"]["status"] = "FAULT"
        if fault and fault["code"] == "BAD_RESULT":
            self._break_result(name, result)
        if fault:
            result["injected"] = {key: fault[key] for key in
                                  ("action", "code", "target", "option")}
        return result

    def _perceive(self, what, is_canceled, feedback):
        """인식을 해 보고 실패하면 한 바퀴 돌며 찾는다 -> (찾음, 탐색 기록 또는 None)."""
        # PERCEPTION_FAILED 주입이나 다시 실행의 perception_option이면 실패를 강제한다
        fault = self._fault
        if not (fault and (fault["code"] == "PERCEPTION_FAILED"
                           or fault.get("perception_option") is not None)):
            fault = None
        rate = self.perception["fail_rate"]
        if not fault and not (
                rate > 0 and self.perception_rng.random() * 100 < rate):
            return True, None

        feedback(f"{what} 인식 실패, 제자리에서 한 바퀴 돌며 찾는 중")
        start = self.pose["yaw"]
        for quarter in range(1, 5):
            self._turn_to((start + 90 * quarter + 180) % 360 - 180,
                          is_canceled)
            self.sleep(0.3, is_canceled)

        if fault and fault.get("perception_option") is not None:
            chance = fault["perception_option"]
        elif fault and fault["option"] is not None:
            chance = fault["option"]
        else:
            chance = self.perception["search_success_rate"]
        found = chance >= 100 or (
            chance > 0 and self.perception_rng.random() * 100 < chance)
        feedback(f"{what} {'찾음' if found else '끝내 못 찾음'}")
        return found, {"spun": True, "found": found,
                       "cause": "injected" if fault else "random"}

    # ---------- 행동별 처리 (run이 "_" + 이름 소문자로 부른다) ----------
    def _read_mission_input(self, args, is_canceled, feedback):
        if self.here != "HOME":
            return {"success": False, "error": "NOT_AT_HOME"}
        feedback("reading_barcode")
        self.sleep(1.0, is_canceled)
        found, search = self._perceive("바코드·신호등", is_canceled, feedback)
        if not found:
            return {"success": False, "error": "PERCEPTION_FAILED",
                    "cause": "LIGHT_NOT_FOUND", "perception_search": search,
                    "message": "한 바퀴 돌며 찾았지만 바코드와 신호등이 보이지 않음"}
        with self.lock:
            self.barcode_read = True

        # 12개를 다 못 읽으면 신호가 반복되므로 다음 주기를 다시 읽는다
        fault = self._fault
        short_reads = (fault["option"]
                       if fault and fault["code"] == "SIGNAL_INCOMPLETE" else 0)
        for read in range(1, SIGNAL_READ_LIMIT + 1):
            seen = self._watch_signal(read, read <= short_reads, is_canceled,
                                      feedback)
            if "?" not in seen:
                break

        complete = "?" not in seen
        result = {"success": complete, "barcode": self.barcode,
                  "signal": seen, "end_seen": True}
        if read > 1:
            result["signal_reread"] = {"reads": read, "complete": complete}
        if search:
            result["perception_search"] = search
        if not complete:
            result.update(error="PERCEPTION_FAILED", cause="SIGNAL_INCOMPLETE",
                          message=f"신호를 {read}번 읽었지만 12개를 다 보지 못함")
        return result

    def _watch_signal(self, read, short, is_canceled, feedback):
        """신호 한 주기(점멸 12번, 3초 소등)를 본다 (short면 몇 칸을 "?"로 놓친다)."""
        missed = set()
        if short:
            missed = set(self.perception_rng.sample(
                range(12), self.perception_rng.randint(*SIGNAL_MISSED_RANGE)))
        with self.lock:
            self.seen_signal = []
        for index, color in enumerate(self.signal):
            seen_color = "?" if index in missed else color
            with self.lock:
                self.light = color
                self.seen_signal.append(seen_color)
            feedback(f"read {read}/{SIGNAL_READ_LIMIT} "
                     f"blink {index + 1}/12 {seen_color}")
            self.sleep(1.0, is_canceled)
            with self.lock:
                self.light = None
            self.sleep(1.0, is_canceled)
        feedback("end_dark_3s")
        self.sleep(3.0, is_canceled)
        with self.lock:
            return list(self.seen_signal)

    def _goto(self, args, is_canceled, feedback):
        result = self.drive_to(args["t"], is_canceled, feedback)
        if result["success"] and args["t"] == "HOME":
            feedback("holding_3s")
            self.sleep(3.0, is_canceled)
            result.update(in_zone=True, stopped_s=3.0)
        return result

    def _load(self, args, is_canceled, feedback):
        """지금 있는 시작 박스의 퍽을 모두 집어 적재함 빈 slot에 싣는다."""
        box = self.here
        if box not in self.boxes:
            return {"success": False, "error": "NOT_AT_START_BOX"}
        with self.lock:
            puck_indexes = [index for index, puck in enumerate(self.boxes[box])
                            if puck is not None]
            free_slots = [slot for slot in range(STORAGE_SLOTS)
                          if slot not in self.storage]
        if not puck_indexes:
            return {"success": False, "error": "BOX_EMPTY", "box_remaining": 0}
        if not free_slots:
            return {"success": False, "error": "STORAGE_FULL"}

        feedback("aligning")
        self.sleep(1.0, is_canceled)
        found, search = self._perceive("박스 위 퍽", is_canceled, feedback)
        if not found:
            return {"success": False, "error": "PERCEPTION_FAILED",
                    "pucks": [], "box_remaining": len(puck_indexes),
                    "perception_search": search,
                    "message": "한 바퀴 돌며 찾았지만 박스 위 퍽을 인식하지 못함"}
        for phase in ("grasping", "lifting"):
            feedback(phase)
            self.sleep(1.0, is_canceled)

        fault = self._fault
        grasp_failed = (fault and fault["action"] == "LOAD"
                        and fault["code"] == "GRASP_FAILED")
        if grasp_failed:  # 앞의 몇 개만 싣고 실패 (옵션 = 실을 개수)
            keep = max(0, min(fault["option"] or 0, len(puck_indexes) - 1))
            puck_indexes = puck_indexes[:keep]
        loaded = []
        with self.lock:
            # 적재함이 모자라면 들어가는 만큼만
            for index, slot in zip(puck_indexes, free_slots):
                puck = self.boxes[box][index]
                self.boxes[box][index] = None
                self.storage[slot] = puck
                confidence = round(self.rng.uniform(0.85, 0.99), 2)
                loaded.append({"slot": slot,
                               "puck": {**puck, "conf": confidence}})
            remaining = sum(puck is not None for puck in self.boxes[box])
        feedback("stowing")
        self.sleep(0.5 * len(loaded), is_canceled)

        if grasp_failed:
            return {"success": False, "error": "GRASP_FAILED",
                    "pucks": loaded, "box_remaining": remaining,
                    "message": f"주입한 실패: 집기 실패 ({len(loaded)}개만 실음)"}
        result = {"success": True, "pucks": loaded,
                  "box_remaining": remaining}
        if search:
            result["perception_search"] = search
        return result

    def _unload(self, args, is_canceled, feedback):
        location = self.here
        if location not in self.placed:
            return {"success": False, "error": "NOT_AT_LOCATION", "items": []}
        total = len(args["items"])
        results = []
        for number, item in enumerate(args["items"], 1):
            puck_id, cell = item["puck_id"], item["cell"]
            with self.lock:
                slot = next((slot for slot, puck in self.storage.items()
                             if puck["id"] == puck_id), None)
                occupied = self.placed[location][cell] is not None
            if slot is None:
                error = "PUCK_NOT_IN_STORAGE"
            else:
                error = "CELL_OCCUPIED" if occupied else None

            # 주입한 실패: 이 퍽은 놓지 못하고 적재함에 남는다
            fault = self._fault
            if (not error and fault and fault["action"] == "UNLOAD"
                    and fault["code"] == "PLACE_FAILED"
                    and number == max(1, min(fault["option"] or 1, total))):
                feedback(f"{number}/{total} placing")
                self.sleep(1.0, is_canceled)
                error = "PLACE_FAILED"
            if error:  # 그 항목에서 멈추고 보고한다
                results.append({"puck_id": puck_id, "cell": cell, "ok": False,
                                "error": error})
                return {"success": False, "error": error, "items": results,
                        "stopped_at": number - 1}

            for phase in ("taking_from_storage", "aligning_to_tag", "placing",
                          "checking_stable_2s"):
                feedback(f"{number}/{total} {phase}")
                self.sleep(1.0 if phase != "checking_stable_2s" else 2.0,
                           is_canceled)
            offset = [round(self.rng.uniform(-4, 4), 1) for _ in range(2)]
            with self.lock:
                self.placed[location][cell] = {**self.storage.pop(slot),
                                               "offset_mm": offset}
                self.verified.discard(location)
            results.append({"puck_id": puck_id, "cell": cell, "ok": True,
                            "offset_mm": offset, "stable_s": 2.0})
        return {"success": True, "items": results, "stopped_at": None}

    def _verify_location(self, args, is_canceled, feedback):
        location = self.here
        if location not in self.placed:
            return {"success": False, "error": "NOT_AT_LOCATION"}
        feedback("observing")
        self.sleep(1.5, is_canceled)
        found, search = self._perceive("하차 박스", is_canceled, feedback)
        if not found:
            return {"success": False, "error": "PERCEPTION_FAILED",
                    "perception_search": search,
                    "message": "한 바퀴 돌며 찾았지만 하차 박스를 관측하지 못함"}
        with self.lock:
            placed = list(self.placed[location])
            self.verified.add(location)

        cells = [
            {"cell": cell, "state": "empty", "puck_id": None, "color": None,
             "shape": None, "offset_mm": None}
            if puck is None else
            {"cell": cell, "state": "occupied", "puck_id": puck["id"],
             "color": puck["color"], "shape": puck["shape"],
             "offset_mm": puck["offset_mm"]}
            for cell, puck in enumerate(placed)]
        items = []
        for item in args["items"]:
            puck = placed[item["cell"]]
            ok = (puck is not None and puck["id"] == item["puck_id"]
                  and math.hypot(*puck["offset_mm"]) <= 10)
            items.append({**item, "ok": ok,
                          "offset_mm": puck["offset_mm"] if puck else None,
                          "seen_puck_id": puck["id"] if puck else None})
        listed = {item["cell"] for item in args["items"]}
        extra = [cell["cell"] for cell in cells
                 if cell["state"] == "occupied" and cell["cell"] not in listed]
        result = {"success": True,
                  "all_ok": all(item["ok"] for item in items) and not extra,
                  "items": items, "extra": extra, "cells": cells}
        if search:
            result["perception_search"] = search
        return result
