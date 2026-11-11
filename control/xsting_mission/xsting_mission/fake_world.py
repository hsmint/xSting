"""backend:=fake용 가짜 경기장.

경기장 정의(config/arena.yaml: 벽, 장애물, 로봇 크기)로 만든 지도 위에서 로봇을 벽을 피하는 경로로 움직이고, 퍽·바구니·장소·신호등 상태를 들고 있다.
웹 화면(sim_view)은 snapshot()과 static_map()으로 이 상태를 읽는다. 좌표는 미션지 기준 mm, yaw는 deg.
"""
import base64
import copy
import math
import random
import threading
import time

import numpy as np

from .arena_map import APPROACH_MM, PLAN_CELL_MM, ArenaMap
BARCODES = ("P2-01", "P2-02", "P2-03", "P2-04")
PUCK_OFFSET_MM = (-40, 40)  # 시작 박스 위 퍽 두 개의 X 오프셋 (화면 표시용)
STORAGE_SLOTS = 12

# 웹·시나리오에서 일부러 낼 수 있는 실패 (docs/error_codes.md의 ③④ 단계 중 가짜 로봇이 흉내 낼 수 있는 것)
# 코드: 옵션 설명 (옵션이 없으면 None)
FAULT_CODES = {
    "READ_MISSION_INPUT": {"PERCEPTION_FAILED": "탐색 후 찾을 확률 %"},
    "GOTO": {"OBSTACLE_BLOCKED": "멈출 지점 %", "NO_PROGRESS": "멈출 지점 %", "NAV_FAILED": "멈출 지점 %",
             "LOCALIZATION_LOST": "멈출 지점 %", "NO_PATH": None},
    "LOAD": {"GRASP_FAILED": "실을 개수", "PERCEPTION_FAILED": "탐색 후 찾을 확률 %"},
    "UNLOAD": {"PLACE_FAILED": "실패할 항목 번호"},
    "VERIFY_LOCATION": {"PERCEPTION_FAILED": "탐색 후 찾을 확률 %"},
}
COMMON_FAULTS = {"ROBOT_FAULT": None, "INTERNAL_ERROR": None, "BAD_RESULT": None}  # 어느 행동이든
FAULT_DEFAULT_OPTION = {"멈출 지점 %": 50, "실을 개수": 0, "실패할 항목 번호": 2, "탐색 후 찾을 확률 %": 0}
# 무작위 인식 실패 (READ·LOAD·VERIFY). 처음 인식이 fail_rate %로 실패하면 제자리에서 한 바퀴 돌며 다시 찾고,
# search_success_rate %로 찾는다. 시나리오의 perception과 웹에서 바꾼다
PERCEPTION_DEFAULT = {"fail_rate": 0, "search_success_rate": 70}
OBSTACLE_TTL_S = 5.0  # OBSTACLE_BLOCKED로 생긴 장애물이 길을 막는 시간 (경기장 시간, 배속이면 더 빨리 사라진다)
OBSTACLE_SIZE_MM = 250
STOP_PERCENT_RANGE = (1, 90)  # GOTO 실패 주입의 "막히는 위치" (경로 길이의 %)


class Canceled(Exception):
    pass


class FakeWorld(ArenaMap):
    def __init__(self, scenario, arena, time_scale=1.0, speed_mps=0.3):
        """arena: config/arena.yaml을 읽은 dict (장소, 벽, 장애물, 로봇 크기)."""
        places = arena["places"]
        self.lock = threading.Lock()
        self.time_scale = time_scale
        self.speed_mmps = speed_mps * 1000
        self.scenario = scenario
        self.commander = None  # 중계 노드가 발행한 MissionState (화면 표시용)
        self.web_last = None  # 화면에서 보낸 마지막 명령과 결과
        self.decisions = []  # 중계 노드의 LLM 판단 기록 (화면 "LLM 판단" 탭)
        self.decisions_version = 0  # 판단 기록이 바뀔 때마다 올린다 (카드를 교체해도 화면이 다시 불러오게)
        self.replay = None  # 다시 실행 중이면 {"run_id", "turn", "total"}
        self.paused = False  # 일시정지: 시간이 흐르지 않는다
        self.abort = False  # 미션 정지: 진행 중인 명령을 취소한다
        self._fault = None  # 지금 실행 중인 행동에 걸린 실패 (실패 주입)

        super().__init__(arena)  # 지도·코스트맵·경로 계산 (arena_map.py)
        self._base_blocked = self.blocked.copy()  # 임시 장애물이 없을 때의 막힌 칸
        self.temp_obstacles = []  # OBSTACLE_BLOCKED로 생긴 임시 장애물 [{x, y, size_mm, expires}]
        self._sim_t, self._sim_last = 0.0, time.monotonic()  # 경기장 시간 (배속·일시정지 반영)
        self._clock_lock = threading.Lock()
        self.home = tuple(places["HOME"]["approach"])
        # 정해 둔 도착 자리 (arena.yaml의 approach). 없는 장소는 GOTO마다 대상 둘레에서 무작위로 정한다
        self.approaches = {name: tuple(p["approach"]) for name, p in places.items() if p.get("approach")}
        self._reach = self._reachable()
        self.reset()

    def reset(self, problem=None):
        """시나리오 처음 상태로 되돌린다 (퍽 배치, 로봇 위치, 기록).

        problem을 주면(지난 판 다시 실행) 그 문제(바코드·신호·퍽·실패 시점)를 그대로 쓴다.
        """
        sc = self.scenario
        if problem is not None:
            sc = {**sc, "barcode": problem["barcode"], "signal": problem["signal"], "boxes": problem["boxes"],
                  "fail_once": problem.get("fail_once", []), "faults": problem.get("faults", []),
                  "perception": problem.get("perception", sc.get("perception"))}
        pick = random.Random()  # 신호·바코드 무작위는 초기화할 때마다 달라지게 시드 없이
        with self.lock:
            self.rng = random.Random(sc.get("seed", 0))
            # 인식 실패는 따로 뽑는다: 확률을 바꿔도 도착 자리·위치 오차 같은 다른 무작위 결과가 달라지지 않게
            self.perception_rng = random.Random(sc.get("seed", 0) + 1)
            self.barcode = pick.choice(BARCODES) if sc["barcode"] == "random" else sc["barcode"]
            # 일부러 낼 실패 목록 (시나리오의 faults·fail_once, 지난 판 다시 실행 때는 그 판에서 났던 실패).
            # 웹에서 넣은 실패도 여기에 쌓인다. 초기화하면 시나리오 것으로 돌아간다
            self.faults, self._fault_seq = [], 0
            for key in sc.get("fail_once", []):  # 예전 형식 "LOAD:S3" = 그 박스의 첫 LOAD를 집기 실패
                action, target = key.split(":")
                self._add_fault_locked({"action": action, "code": "GRASP_FAILED", "target": target})
            for f in sc.get("faults", []) or []:
                self._add_fault_locked(f)
            self.localization = "ok"
            self.temp_obstacles = []
            self.blocked = self._base_blocked.copy()
            self.perception = {**PERCEPTION_DEFAULT, **(sc.get("perception") or {})}
            # 퍽과 신호: 미션지 출제 규칙대로 "풀 수 있는 조합이 하나 이상"인 문제만 낸다
            while True:
                self.boxes = (self._random_boxes(pick) if sc["boxes"] == "random" else
                              {box: [dict(p) if p else None for p in slots] for box, slots in sc["boxes"].items()})
                options = self._solvable_counts()
                if sc["signal"] != "random":
                    self.signal = list(sc["signal"])
                    break
                if options:
                    r, y, g = pick.choice(options)
                    self.signal = ["R"] * r + ["Y"] * y + ["G"] * g
                    pick.shuffle(self.signal)
                    break
                if sc["boxes"] != "random":
                    raise ValueError("fake_scenario의 퍽 구성으로 풀 수 있는 신호가 없음")
            self.storage = {}  # slot -> 퍽
            self.placed = {loc: [None] * 9 for loc in ("L1", "L2", "L3")}
            self.verified = set()
            self.paused = False
            self.abort = False
            self.initial_pucks = {box: [dict(p) for p in slots if p] for box, slots in self.boxes.items()}
            self.here = "HOME"
            self.pose = {"x": float(self.home[0]), "y": float(self.home[1]), "yaw": float(self.home[2])}
            # 화면 표시용 상태
            self.light = None
            self.seen_signal = []
            self.barcode_read = False
            self.command = None
            self.history = []
            self.path = []

    @staticmethod
    def _random_boxes(pick):
        """퍽 12개(id 0~11)의 색·모양을 무작위로 정하고 시작 박스 6곳에 2개씩 나눈다."""
        ids = list(range(12))
        pick.shuffle(ids)
        pucks = [{"id": i, "color": pick.choice("RG"), "shape": pick.choice(("cylinder", "box"))} for i in ids]
        return {f"S{k + 1}": pucks[2 * k:2 * k + 2] for k in range(6)}

    def _solvable_counts(self):
        """지금 퍽 구성으로 풀 수 있는 (R, Y, G) 신호 횟수 조합 목록."""
        pucks = [p for slots in self.boxes.values() for p in slots if p]
        red = [p for p in pucks if p["color"] == "R"]
        green = [p for p in pucks if p["color"] == "G"]
        options = []
        for r in range(1, 7):
            for y in range(1, 7):
                g = 12 - r - y
                if not 1 <= g <= 6 or r > len(red) or g > len(green):
                    continue
                # R·G에 쓰고 남는 퍽이 Y가 되고, Y는 한 모양으로 통일할 수 있어야 한다
                ry, gy = len(red) - r, len(green) - g
                if any(sum(p["shape"] == s for p in red) >= ry and sum(p["shape"] == s for p in green) >= gy
                       for s in ("cylinder", "box")):
                    options.append((r, y, g))
        return options

    def add_decision(self, record):
        """중계 노드가 보낸 판단 기록. 같은 turn의 결과(outcome)는 그 판단에 합친다. 새 실행이면 비운다."""
        with self.lock:
            self.decisions_version += 1
            prev = self.decisions[-1]["run_id"] if self.decisions else None
            # 다른 판의 기록이면 비운다. 단, 화면에서 먼저 보낸 명령("web") 뒤에 중계 노드가 시작하면 이어 붙인다
            if prev is not None and prev != record["run_id"] and not (
                    prev == "web" and not str(record["run_id"]).startswith(("log:", "replay:"))):
                self.decisions = []
            if "outcome" in record and record.get("decider") != "user":
                for d in reversed(self.decisions):
                    if d["turn"] == record["turn"] and d.get("decider") != "user":
                        d["outcome"] = record["outcome"]
                        break
            else:
                k = next((k for k in range(len(self.decisions) - 1, -1, -1)
                          if record.get("decider") != "user" and self.decisions[k].get("turn") == record.get("turn")
                          and (self.decisions[k].get("prefetch") or {}).get("status") in ("requested", "answered")), None)
                if k is None:
                    self.decisions.append(record)
                    del self.decisions[:-100]
                else:  # "미리 판단 요청함/받음" 카드를 실제 판단(또는 버림)으로 바꾼다
                    self.decisions[k] = record

    def decision_list(self):
        with self.lock:
            return copy.deepcopy(self.decisions)

    def problem(self):
        """지금 문제 (다시 실행할 수 있게 판마다 problem.json으로 저장된다)."""
        with self.lock:
            return {"barcode": self.barcode, "signal": "".join(self.signal), "boxes": self.initial_pucks}

    def set_time_scale(self, value):
        with self.lock:
            self.time_scale = max(0.25, min(50.0, float(value)))
            return self.time_scale

    # ---------- 도착 자리 ----------
    def _reachable(self):
        """HOME에서 갈 수 있는 칸 (도착 자리가 갈 수 없는 곳에 생기지 않게)."""
        start = self._cell(*self.home[:2])
        seen, stack = {start}, [start]
        while stack:
            i, j = stack.pop()
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                n = (i + di, j + dj)
                if n not in seen and self._free(*n):
                    seen.add(n)
                    stack.append(n)
        return seen

    def _arrival(self, name):
        """GOTO의 도착 자리. arena.yaml에 approach가 있으면 그 자리, 없으면 대상 둘레의 갈 수 있는 곳 중 무작위.

        실물에서도 정확히 같은 자리에 서지 않으므로, LLM은 도착한 robot.pose를 보고 다음 경로를 판단한다.
        """
        if name in self.approaches:
            return self.approaches[name]
        tx, ty = self.targets[name]
        for _ in range(500):
            d, a = self.rng.uniform(*APPROACH_MM), self.rng.uniform(0, 2 * math.pi)
            x, y = round(tx + d * math.cos(a)), round(ty + d * math.sin(a))
            if self._cell(x, y) in self._reach:
                return (x, y, round(math.degrees(math.atan2(ty - y, tx - x)), 1))
        return None

    # ---------- 시간과 취소 ----------
    def sim_now(self):
        """경기장 시간(초). 배속을 곱하고, 일시정지 동안은 흐르지 않는다."""
        with self._clock_lock:
            now = time.monotonic()
            if not self.paused:
                self._sim_t += (now - self._sim_last) * self.time_scale
            self._sim_last = now
            return self._sim_t

    def _expire_obstacles(self):
        """시간이 지난 임시 장애물을 지우고, 바뀌었으면 막힌 칸을 다시 만든다 (잠금을 잡은 상태에서)."""
        now = self.sim_now()
        alive = [o for o in self.temp_obstacles if o["expires"] > now]
        if len(alive) != len(self.temp_obstacles):
            self.temp_obstacles = alive
            self._apply_temp_obstacles()

    def _apply_temp_obstacles(self):
        """막힌 칸 = 원래 막힌 칸 + 임시 장애물 둘레(코스트맵 패딩만큼)."""
        blocked = self._base_blocked.copy()
        if self.temp_obstacles:
            ox, oy = self.map_origin_mm
            cx = ox + (np.arange(self.grid_w) + 0.5) * PLAN_CELL_MM
            cy = oy + (np.arange(self.grid_h) + 0.5) * PLAN_CELL_MM
            gx, gy = np.meshgrid(cx, cy)
            pad = self.arena["costmap"]["padding"]
            for o in self.temp_obstacles:
                w, h = o["size_mm"]
                dx = np.maximum(np.abs(gx - o["x"]) - w / 2, 0)
                dy = np.maximum(np.abs(gy - o["y"]) - h / 2, 0)
                blocked |= np.hypot(dx, dy) <= pad
        self.blocked = blocked

    def sleep(self, seconds, is_canceled):
        """시뮬레이션 시간으로 seconds만큼 기다린다. 배속을 바꾸면 진행 중인 대기에도 바로 반영된다."""
        left = seconds
        while left > 0:
            if is_canceled() or self.abort:
                raise Canceled()
            time.sleep(0.02)
            self.sim_now()
            if not self.paused:  # 일시정지 중에는 시간이 흐르지 않는다
                left -= 0.02 * self.time_scale

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
        fault = self._fault if self._fault and self._fault["action"] == "GOTO" else None
        if fault and fault["code"] == "NO_PATH":
            return {"success": False, "error": "NO_PATH", "message": "주입한 실패: 경로를 찾지 못함"}
        with self.lock:
            self._expire_obstacles()
        path = None
        for _ in range(5):  # 도착 자리가 임시 장애물 근처에 뽑히면 다른 자리로
            arrival = self._arrival(place)
            path = None if arrival is None else self.plan(arrival, (self.pose["x"], self.pose["y"]))
            if path is not None:
                break
        if path is None:
            if self.temp_obstacles:  # 임시 장애물 때문에 돌아갈 길이 없다
                o = min(self.temp_obstacles, key=lambda o: math.hypot(o["x"] - self.pose["x"], o["y"] - self.pose["y"]))
                return {"success": False, "error": "OBSTACLE_BLOCKED", "remaining_mm": None,
                        "message": "길 위의 장애물 때문에 돌아갈 길이 없음",
                        "obstacle": self._obstacle_info(o), "blocked_segment": {"from": self.here, "to": place}}
            return {"success": False, "error": "NO_PATH"}
        ax, ay, ayaw = arrival
        total = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
        # 실패 주입: 경로의 몇 % 지점에서 멈출지. 장애물을 놓을 자리(멈춘 곳 앞)가 경로 안에 남도록 끝쪽은 줄인다
        ahead = OBSTACLE_SIZE_MM / 2 + self.arena["costmap"]["padding"] + 150
        stop_at = None
        if fault and fault["option"] is not None:
            pct = min(max(fault["option"], STOP_PERCENT_RANGE[0]), STOP_PERCENT_RANGE[1])
            stop_at = min(total * pct / 100, max(0.0, total - ahead))
        start_here, done = self.here, 0.0
        with self.lock:
            self.path = [list(p) for p in path]
        for (x0, y0), (x1, y1) in zip(path, path[1:]):
            seg = math.hypot(x1 - x0, y1 - y0)
            if seg < 1:
                continue
            self._turn_to(math.degrees(math.atan2(y1 - y0, x1 - x0)), is_canceled)
            moved = 0.0
            while moved < seg:
                moved = min(seg, moved + self.speed_mmps * 0.03 * self.time_scale)
                if stop_at is not None and done + moved >= stop_at:  # 주입한 실패: 여기서 멈춘다
                    moved = stop_at - done
                    with self.lock:
                        self.pose["x"] = x0 + (x1 - x0) * moved / seg
                        self.pose["y"] = y0 + (y1 - y0) * moved / seg
                    return self._stop_midway(fault, place, start_here, total - stop_at,
                                             self._point_on_path(path, stop_at + ahead))
                with self.lock:
                    self.pose["x"] = x0 + (x1 - x0) * moved / seg
                    self.pose["y"] = y0 + (y1 - y0) * moved / seg
                self.sleep(0.03, is_canceled)
            done += seg
            feedback(f"moving {math.hypot(ax - x1, ay - y1) / 1000:.2f} m left")
        self._turn_to(ayaw, is_canceled)
        with self.lock:
            self.path = []
            self.here = place
        return {"success": True, "dist_to_target_mm": round(math.hypot(ax - self.targets[place][0], ay - self.targets[place][1]))}

    @staticmethod
    def _point_on_path(path, dist):
        """경로를 따라 dist(mm) 간 지점 (경로 끝을 넘으면 끝점)."""
        for a, b in zip(path, path[1:]):
            seg = math.dist(a, b)
            if dist <= seg and seg > 0:
                return (a[0] + (b[0] - a[0]) * dist / seg, a[1] + (b[1] - a[1]) * dist / seg)
            dist -= seg
        return tuple(path[-1])

    def _stop_midway(self, fault, place, start_here, remaining, obstacle_xy):
        """GOTO 실패 주입: 경로 중간에서 멈춘 결과. 장애물이면 남은 경로 위(로봇 앞)에 장애물을 놓고 알린다."""
        code = fault["code"]
        with self.lock:
            self.path = []
            self.here = "UNKNOWN"
            if code == "LOCALIZATION_LOST":
                self.localization = "lost"
        out = {"success": False, "error": code, "remaining_mm": round(remaining),
               "message": {"OBSTACLE_BLOCKED": "주입한 실패: 길 위의 장애물 때문에 돌아갈 길이 없음",
                           "NO_PROGRESS": "주입한 실패: 장애물은 안 보이는데 앞으로 가지 못함",
                           "NAV_FAILED": "주입한 실패: 주행이 중단됨",
                           "LOCALIZATION_LOST": "주입한 실패: 위치 추정을 잃고 복구하지 못함"}[code]}
        if code == "OBSTACLE_BLOCKED":  # 남은 경로 위에 임시 장애물을 놓는다 (OBSTACLE_TTL_S 동안 길을 막는다)
            # 로봇에서 장애물 둘레(크기 절반 + 코스트맵 패딩)보다 멀리 놓아야 막힌 칸 안에 갇히지 않고 돌아 나올 수 있다
            o = {"x": round(obstacle_xy[0]), "y": round(obstacle_xy[1]), "size_mm": [OBSTACLE_SIZE_MM, OBSTACLE_SIZE_MM],
                 "expires": self.sim_now() + OBSTACLE_TTL_S}
            with self.lock:
                self.temp_obstacles.append(o)
                self._apply_temp_obstacles()
            out["obstacle"] = self._obstacle_info(o)
            out["blocked_segment"] = {"from": start_here, "to": place}
        return out

    @staticmethod
    def _obstacle_info(o):
        """결과에 싣는 장애물 정보. 언제 사라지는지는 싣지 않는다 (실물 로봇은 알 수 없는 값이라 AI가 기대면 안 된다)."""
        return {"x": o["x"], "y": o["y"], "size_mm": list(o["size_mm"])}

    # ---------- 화면용 ----------
    def static_map(self):
        h, w = self.map_occ.shape
        return {
            "width": w, "height": h, "res_mm": self.map_res_mm, "origin_mm": list(self.map_origin_mm),
            "occupied_b64": base64.b64encode(self.map_occ.astype(np.uint8).tobytes()).decode(),
            "targets": {k: list(v) for k, v in self.targets.items()},
            "approaches": {k: list(v) for k, v in self.approaches.items()},
            # 로봇 중심(점)이 갈 수 없는 칸 (벽·박스에서 costmap.padding 안). 화면에 옅게 칠한다
            "blocked_b64": base64.b64encode(self.blocked.astype(np.uint8).tobytes()).decode(),
            "plan_w": self.grid_w, "plan_h": self.grid_h, "plan_cell_mm": PLAN_CELL_MM,
            "virtual_walls": [{"id": k, "from": v["from"], "to": v["to"]}
                              for k, v in (self.arena.get("virtual_walls") or {}).items()],
            "wall_mm": self.arena["wall_thickness"],
            "box_mm": self.arena["box_size"],
            "obstacles": [{"name": o["name"], "center": o["center"], "size": o["size"]}
                          for o in self.arena.get("obstacles") or []],
            "start_zone_mm": self.arena["start_zone"], "light_mm": self.arena["signal_light"],
            "barcode_mm": self.arena["barcode"], "puck_offset_mm": list(PUCK_OFFSET_MM),
            "footprint_mm": self.arena["robot"]["footprint"],
            # 웹 "실패 주입" 카드에서 고를 수 있는 실패 {행동: {코드: 옵션 설명}}
            "fault_codes": {a: {**codes, **COMMON_FAULTS} for a, codes in FAULT_CODES.items()},
            "fault_default_option": FAULT_DEFAULT_OPTION,
        }

    def snapshot(self):
        with self.lock:
            self._expire_obstacles()
            now = self.sim_now()
            return copy.deepcopy({
                "temp_obstacles": [{**self._obstacle_info(o), "left_s": round(o["expires"] - now, 1)}
                                   for o in self.temp_obstacles],
                "pose": self.pose, "path": self.path, "light": self.light, "seen_signal": self.seen_signal,
                "barcode": self.barcode if self.barcode_read else None, "boxes": self.boxes,
                "storage": {str(k): v for k, v in self.storage.items()}, "placed": self.placed,
                "verified": sorted(self.verified), "here": self.here,
                "command": self.command, "history": self.history[-30:],
                "commander": self.commander, "time_scale": self.time_scale, "web_last": self.web_last,
                "answer": {"barcode": self.barcode, "signal": "".join(self.signal), "pucks": self.initial_pucks},
                "paused": self.paused, "replay": self.replay, "faults": self.faults, "perception": self.perception,
                "decisions_key": f"{self.decisions[0]['run_id'] if self.decisions else ''}:{len(self.decisions)}:"
                                 f"{sum('outcome' in d for d in self.decisions)}:{self.decisions_version}",
            })

    def begin(self, name, args, source):
        with self.lock:
            self.abort = False
            self.command = {"name": name, "args": args, "phase": "", "started": time.time(), "source": source}

    def set_phase(self, phase):
        with self.lock:
            if self.command:
                self.command["phase"] = phase

    def end(self, name, args, out):
        with self.lock:
            started = self.command["started"] if self.command else time.time()
            source = self.command["source"] if self.command else "?"
            self.history.append({"t": time.time(), "started": started, "duration_s": round(time.time() - started, 2),
                                 "source": source, "name": name, "args": args,
                                 "success": bool(out.get("success")), "error": out.get("error"), "result": out})
            self.command = None

    # ---------- 행동 ----------
    def robot_status(self):
        """매 결과에 함께 싣는 로봇 상태 (MissionState.robot)."""
        pose = {"x": round(self.pose["x"]), "y": round(self.pose["y"]), "yaw": round(self.pose["yaw"], 1)}
        lost = self.localization == "lost"
        return {"status": "OK", "error": None, "here": self.here, "pose": None if lost else pose,
                "localization": self.localization, "arm_stowed": True, "gripper": "empty"}

    def run(self, name, args, is_canceled, feedback):
        handler = getattr(self, "_" + name.lower(), None)
        if handler is None:
            return {"success": False, "error": "UNKNOWN_COMMAND", "robot": self.robot_status()}
        target = args.get("t") if name == "GOTO" else (None if name == "READ_MISSION_INPUT" else self.here)
        with self.lock:
            self.localization = "ok"  # 위치 추정을 잃었던 것은 다음 행동 전에 사람이 되돌렸다고 본다
            fault = self._take_fault(name, target)
        self._fault = fault
        try:
            if fault and fault["code"] in ("ROBOT_FAULT", "INTERNAL_ERROR"):
                out = {"success": False, "error": fault["code"],
                       "message": "주입한 실패: " + ("모터 과전류" if fault["code"] == "ROBOT_FAULT" else "예상 못 한 예외")}
            else:
                out = handler(args, is_canceled, feedback)
        except Canceled:
            with self.lock:
                self.path = []
            raise
        finally:
            self._fault = None
        out["robot"] = self.robot_status()
        if fault and fault["code"] == "ROBOT_FAULT":
            out["robot"]["status"] = "FAULT"
        if fault and fault["code"] == "BAD_RESULT":  # 동작은 했는데 결과를 규약과 다르게 만든 것처럼
            self._break_result(name, out)
        if fault:
            out["injected"] = {k: fault[k] for k in ("action", "code", "target", "option")}
        return out

    # ---------- 인식 ----------
    def set_perception(self, fail_rate, search_success_rate):
        with self.lock:
            self.perception = {"fail_rate": max(0, min(100, float(fail_rate))),
                               "search_success_rate": max(0, min(100, float(search_success_rate)))}
            return dict(self.perception)

    def _perceive(self, what, is_canceled, feedback):
        """인식을 한 번 해 본다. 처음 인식이 실패하면(무작위 또는 주입) 제자리에서 한 바퀴 돌며 다시 찾는다.

        반환: (찾았는지, 탐색 기록 또는 None). 처음에 찾으면 기록은 None이다.
        난수는 perception_rng에서 뽑는다. 지난 판 다시 실행 때는 기록된 탐색 결과를 확정된 실패로 다시 걸어 재현한다.
        """
        # 인식 실패를 강제하는 경우: PERCEPTION_FAILED 주입, 또는 다시 실행에서 다른 실패와 합쳐진 인식 탐색(perception_option)
        f = self._fault
        fault = f if f and (f["code"] == "PERCEPTION_FAILED" or f.get("perception_option") is not None) else None
        rate = self.perception["fail_rate"]
        if not fault and not (rate > 0 and self.perception_rng.random() * 100 < rate):
            return True, None
        feedback(f"{what} 인식 실패, 제자리에서 한 바퀴 돌며 찾는 중")
        start = self.pose["yaw"]
        for k in range(1, 5):  # 90도씩 네 번 = 한 바퀴
            self._turn_to((start + 90 * k + 180) % 360 - 180, is_canceled)
            self.sleep(0.3, is_canceled)
        if fault and fault.get("perception_option") is not None:
            chance = fault["perception_option"]
        elif fault and fault["option"] is not None:
            chance = fault["option"]
        else:
            chance = self.perception["search_success_rate"]
        found = chance >= 100 or (chance > 0 and self.perception_rng.random() * 100 < chance)
        feedback(f"{what} {'찾음' if found else '끝내 못 찾음'}")
        return found, {"spun": True, "found": found, "cause": "injected" if fault else "random"}

    # ---------- 실패 주입 ----------
    def _add_fault_locked(self, f):
        """실패 하나를 목록에 넣는다 (잠금을 잡은 상태에서). 틀린 값이면 ValueError."""
        action, code = f.get("action"), f.get("code")
        codes = {**FAULT_CODES.get(action, {}), **COMMON_FAULTS} if action in FAULT_CODES else {}
        if code not in codes:
            raise ValueError(f"{action}에서 낼 수 없는 실패: {code}")
        label = codes[code]
        option = f.get("option")
        option = FAULT_DEFAULT_OPTION.get(label) if option in (None, "") else int(option)
        if label == "멈출 지점 %":
            option = min(max(option, STOP_PERCENT_RANGE[0]), STOP_PERCENT_RANGE[1])
        self._fault_seq += 1
        fault = {"id": self._fault_seq, "action": action, "code": code, "target": f.get("target") or None,
                 "option": option if label else None, "option_label": label, "times": max(1, int(f.get("times", 1)))}
        if f.get("perception_option") is not None:  # 다시 실행: 이 행동의 인식도 처음에 실패하고 이 확률로 찾는다
            fault["perception_option"] = int(f["perception_option"])
        self.faults.append(fault)
        return fault

    def add_fault(self, f):
        with self.lock:
            return dict(self._add_fault_locked(f))

    def remove_fault(self, fault_id):
        with self.lock:
            self.faults = [f for f in self.faults if f["id"] != fault_id]

    def _take_fault(self, name, target):
        """그 행동·대상에 걸린 실패를 하나 꺼낸다 (남은 횟수를 하나 줄인다). 없으면 None."""
        for f in self.faults:
            if f["action"] == name and f["target"] in (None, target):
                f["times"] -= 1
                if f["times"] <= 0:
                    self.faults.remove(f)
                return dict(f)
        return None

    @staticmethod
    def _break_result(name, out):
        """규약(contract.py)에 어긋나는 결과로 바꾼다. 로봇 서버·중계 노드의 BAD_RESULT 검사를 시험한다."""
        drop = {"READ_MISSION_INPUT": "barcode", "LOAD": "box_remaining", "UNLOAD": "items",
                "VERIFY_LOCATION": "cells"}.get(name)
        if drop:
            out.pop(drop, None)
        elif name == "GOTO":
            out["robot"]["here"] = "SOMEWHERE"

    def _read_mission_input(self, args, is_canceled, feedback):
        if self.here != "HOME":
            return {"success": False, "error": "NOT_AT_HOME"}
        feedback("reading_barcode")
        self.sleep(1.0, is_canceled)
        found, search = self._perceive("바코드·신호등", is_canceled, feedback)
        if not found:
            return {"success": False, "error": "PERCEPTION_FAILED", "perception_search": search,
                    "message": "한 바퀴 돌며 찾았지만 바코드와 신호등이 보이지 않음"}
        with self.lock:
            self.barcode_read = True
            self.seen_signal = []
        for k, color in enumerate(self.signal, 1):
            with self.lock:
                self.light = color
                self.seen_signal.append(color)
            feedback(f"blink {k}/12 {color}")
            self.sleep(1.0, is_canceled)
            with self.lock:
                self.light = None
            self.sleep(1.0, is_canceled)
        feedback("end_dark_3s")
        self.sleep(3.0, is_canceled)
        out = {"success": True, "barcode": self.barcode, "signal": list(self.signal), "end_seen": True}
        if search:
            out["perception_search"] = search
        return out

    def _goto(self, args, is_canceled, feedback):
        out = self.drive_to(args["t"], is_canceled, feedback)
        if out["success"] and args["t"] == "HOME":
            feedback("holding_3s")
            self.sleep(3.0, is_canceled)
            out.update(in_zone=True, stopped_s=3.0)
        return out

    def _load(self, args, is_canceled, feedback):
        """지금 있는 시작 박스의 퍽을 한 번에 모두 집어 적재함 빈 slot에 싣는다."""
        box = self.here
        if box not in self.boxes:
            return {"success": False, "error": "NOT_AT_START_BOX"}
        with self.lock:
            ks = [i for i, p in enumerate(self.boxes[box]) if p is not None]
            free = [i for i in range(STORAGE_SLOTS) if i not in self.storage]
        if not ks:
            return {"success": False, "error": "BOX_EMPTY", "box_remaining": 0}
        if not free:
            return {"success": False, "error": "STORAGE_FULL"}
        feedback("aligning")
        self.sleep(1.0, is_canceled)
        found, search = self._perceive("박스 위 퍽", is_canceled, feedback)
        if not found:
            return {"success": False, "error": "PERCEPTION_FAILED", "pucks": [], "box_remaining": len(ks),
                    "perception_search": search, "message": "한 바퀴 돌며 찾았지만 박스 위 퍽을 인식하지 못함"}
        for phase in ("grasping", "lifting"):
            feedback(phase)
            self.sleep(1.0, is_canceled)
        fault = self._fault if self._fault and self._fault["action"] == "LOAD" else None
        if fault and fault["code"] == "GRASP_FAILED":  # 앞의 몇 개만 싣고 실패 (옵션 = 실을 개수)
            ks = ks[:max(0, min(fault["option"] or 0, len(ks) - 1))]
        loaded = []
        with self.lock:
            for k, slot in zip(ks, free):  # 적재함이 모자라면 들어가는 만큼만
                puck = self.boxes[box][k]
                self.boxes[box][k] = None
                self.storage[slot] = puck
                loaded.append({"slot": slot, "puck": {**puck, "conf": round(self.rng.uniform(0.85, 0.99), 2)}})
            remaining = sum(p is not None for p in self.boxes[box])
        feedback("stowing")
        self.sleep(0.5 * len(loaded), is_canceled)
        if fault and fault["code"] == "GRASP_FAILED":
            return {"success": False, "error": "GRASP_FAILED", "pucks": loaded, "box_remaining": remaining,
                    "message": f"주입한 실패: 집기 실패 ({len(loaded)}개만 실음)"}
        out = {"success": True, "pucks": loaded, "box_remaining": remaining}
        if search:
            out["perception_search"] = search
        return out

    def _unload(self, args, is_canceled, feedback):
        loc = self.here
        if loc not in self.placed:
            return {"success": False, "error": "NOT_AT_LOCATION", "items": []}
        results = []
        for n, item in enumerate(args["items"], 1):
            puck_id, cell = item["puck_id"], item["cell"]
            with self.lock:
                slot = next((s for s, p in self.storage.items() if p["id"] == puck_id), None)
                occupied = self.placed[loc][cell] is not None
            error = "PUCK_NOT_IN_STORAGE" if slot is None else "CELL_OCCUPIED" if occupied else None
            fault = self._fault if self._fault and self._fault["action"] == "UNLOAD" else None
            if not error and fault and fault["code"] == "PLACE_FAILED" \
                    and n == max(1, min(fault["option"] or 1, len(args["items"]))):
                feedback(f"{n}/{len(args['items'])} placing")
                self.sleep(1.0, is_canceled)
                error = "PLACE_FAILED"  # 주입한 실패: 이 퍽은 놓지 못하고 적재함에 남는다
            if error:  # 그 항목에서 멈추고 보고한다
                results.append({"puck_id": puck_id, "cell": cell, "ok": False, "error": error})
                return {"success": False, "error": error, "items": results, "stopped_at": n - 1}
            for phase in ("taking_from_storage", "aligning_to_tag", "placing", "checking_stable_2s"):
                feedback(f"{n}/{len(args['items'])} {phase}")
                self.sleep(1.0 if phase != "checking_stable_2s" else 2.0, is_canceled)
            offset = [round(self.rng.uniform(-4, 4), 1) for _ in range(2)]
            with self.lock:
                self.placed[loc][cell] = {**self.storage.pop(slot), "offset_mm": offset}
                self.verified.discard(loc)
            results.append({"puck_id": puck_id, "cell": cell, "ok": True, "offset_mm": offset, "stable_s": 2.0})
        return {"success": True, "items": results, "stopped_at": None}

    def _verify_location(self, args, is_canceled, feedback):
        loc = self.here
        if loc not in self.placed:
            return {"success": False, "error": "NOT_AT_LOCATION"}
        feedback("observing")
        self.sleep(1.5, is_canceled)
        found, search = self._perceive("하차 박스", is_canceled, feedback)
        if not found:
            return {"success": False, "error": "PERCEPTION_FAILED", "perception_search": search,
                    "message": "한 바퀴 돌며 찾았지만 하차 박스를 관측하지 못함"}
        with self.lock:
            placed = list(self.placed[loc])
            self.verified.add(loc)
        cells = [{"cell": i, "state": "empty", "puck_id": None, "color": None, "shape": None, "offset_mm": None}
                 if p is None else
                 {"cell": i, "state": "occupied", "puck_id": p["id"], "color": p["color"],
                  "shape": p["shape"], "offset_mm": p["offset_mm"]}
                 for i, p in enumerate(placed)]
        items = []
        for item in args["items"]:
            p = placed[item["cell"]]
            ok = p is not None and p["id"] == item["puck_id"] and math.hypot(*p["offset_mm"]) <= 10
            items.append({**item, "ok": ok, "offset_mm": p["offset_mm"] if p else None,
                          "seen_puck_id": p["id"] if p else None})
        listed = {item["cell"] for item in args["items"]}
        extra = [c["cell"] for c in cells if c["state"] == "occupied" and c["cell"] not in listed]
        out = {"success": True, "all_ok": all(i["ok"] for i in items) and not extra,
               "items": items, "extra": extra, "cells": cells}
        if search:
            out["perception_search"] = search
        return out
