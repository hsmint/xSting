"""backend:=fake용 가짜 경기장.

경기장 정의(config/arena.yaml: 벽, 장애물, 로봇 크기)로 만든 지도 위에서 로봇을 벽을 피하는 경로로 움직이고, 퍽·바구니·장소·신호등 상태를 들고 있다.
웹 화면(sim_view)은 snapshot()과 static_map()으로 이 상태를 읽는다. 좌표는 미션지 기준 mm, yaw는 deg.
"""
import base64
import copy
import heapq
import math
import random
import threading
import time

import numpy as np

MAP_RES_MM = 10  # 벽 지도 해상도
MAP_MARGIN_MM = 100  # 바깥 벽 밖으로 지도를 이만큼 더 그린다
PLAN_CELL_MM = 50  # 코스트맵·경로 계산 격자 (Nav2 costmap resolution 0.05 m와 같다)
APPROACH_MM = (330, 520)  # GOTO 도착 자리: 대상 중심에서 로봇 중심까지 이 범위 안 무작위 거리
BARCODES = ("P2-01", "P2-02", "P2-03", "P2-04")
PUCK_OFFSET_MM = (-40, 40)  # 시작 박스 위 퍽 두 개의 X 오프셋 (화면 표시용)
STORAGE_SLOTS = 12


class Canceled(Exception):
    pass


class FakeWorld:
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
        self.replay = None  # 다시 실행 중이면 {"run_id", "turn", "total"}
        self.paused = False  # 일시정지: 시간이 흐르지 않는다
        self.abort = False  # 미션 정지: 진행 중인 명령을 취소한다

        self.targets = {name: tuple(p["target"]) for name, p in places.items()}
        self.arena = arena
        self._load_map(arena)
        self._build_plan_grid()
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
            sc = {**sc, "barcode": problem["barcode"], "signal": problem["signal"],
                  "boxes": problem["boxes"], "fail_once": problem.get("fail_once", [])}
        pick = random.Random()  # 신호·바코드 무작위는 초기화할 때마다 달라지게 시드 없이
        with self.lock:
            self.rng = random.Random(sc.get("seed", 0))
            self.barcode = pick.choice(BARCODES) if sc["barcode"] == "random" else sc["barcode"]
            self.fail_once = list(sc.get("fail_once", []))  # 같은 키가 여러 번이면 그 횟수만큼 실패
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
                self.decisions.append(record)
                del self.decisions[:-100]

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

    # ---------- 지도와 경로 ----------
    def wall_segments(self):
        """바깥 벽 4개 + 안쪽 벽. [((x1, y1), (x2, y2)), ...]"""
        w, h = self.arena["size"]
        corners = [(0, 0), (w, 0), (w, h), (0, h)]
        segs = [(corners[k], corners[(k + 1) % 4]) for k in range(4)]
        return segs + [(tuple(v["from"]), tuple(v["to"])) for v in (self.arena.get("walls") or {}).values()]

    def virtual_wall_segments(self):
        """가벽 (팀이 정한 진입 금지선). 경로 계산에서는 벽과 똑같이 막지만 화면에는 따로 그린다."""
        return [(tuple(v["from"]), tuple(v["to"])) for v in (self.arena.get("virtual_walls") or {}).values()]

    def _load_map(self, arena):
        """arena.yaml의 벽(map_occ)과 가벽(map_virtual)을 MAP_RES_MM 격자에 두께 wall_thickness로 그린다. 0행 = 아래쪽."""
        res, m = MAP_RES_MM, MAP_MARGIN_MM
        w_mm, h_mm = arena["size"]
        w, h = int((w_mm + 2 * m) // res) + 1, int((h_mm + 2 * m) // res) + 1
        self.map_res_mm = res
        self.map_origin_mm = (-m, -m)
        self.map_occ = self._raster(self.wall_segments(), w, h)
        self.map_virtual = self._raster(self.virtual_wall_segments(), w, h)

    def _raster(self, segments, w, h):
        res, m = self.map_res_mm, -self.map_origin_mm[0]
        xs, ys = -m + (np.arange(w) + 0.5) * res, -m + (np.arange(h) + 0.5) * res
        occ = np.zeros((h, w), dtype=bool)
        half = self.arena["wall_thickness"] / 2
        for (x1, y1), (x2, y2) in segments:
            # 선분 둘레 사각형 안의 칸만 선분까지 거리를 잰다
            i0, i1 = np.searchsorted(xs, min(x1, x2) - half), np.searchsorted(xs, max(x1, x2) + half)
            j0, j1 = np.searchsorted(ys, min(y1, y2) - half), np.searchsorted(ys, max(y1, y2) + half)
            px, py = np.meshgrid(xs[i0:i1], ys[j0:j1])
            dx, dy = x2 - x1, y2 - y1
            t = np.clip(((px - x1) * dx + (py - y1) * dy) / max(dx * dx + dy * dy, 1e-9), 0, 1)
            occ[j0:j1, i0:i1] |= np.hypot(px - (x1 + t * dx), py - (y1 + t * dy)) <= half
        return occ

    def _obstacle_rects(self):
        """벽 외에 피해야 할 것: 시작·하차 박스, arena.yaml의 obstacles. {이름: (중심 x, 중심 y, 가로, 세로)}"""
        box = self.arena["box_size"]
        rects = {b: (*self.targets[b], box, box) for b in self.targets if b[0] in "SL"}
        rects.update({o["name"]: (*o["center"], *o["size"]) for o in self.arena.get("obstacles") or []})
        return rects

    def _build_plan_grid(self):
        """코스트맵: 로봇을 점으로 보고, 벽·박스에서 costmap.padding 안을 못 가는 칸으로 막는다."""
        from scipy import ndimage

        res, (ox, oy) = self.map_res_mm, self.map_origin_mm
        occ = self.map_occ | self.map_virtual  # 가벽도 벽처럼 막는다
        for cx, cy, sx, sy in self._obstacle_rects().values():
            i0, i1 = int((cx - sx / 2 - ox) // res), int((cx + sx / 2 - ox) // res)
            j0, j1 = int((cy - sy / 2 - oy) // res), int((cy + sy / 2 - oy) // res)
            occ[max(j0, 0):j1 + 1, max(i0, 0):i1 + 1] = True
        k = PLAN_CELL_MM / res
        h, w = int(occ.shape[0] // k), int(occ.shape[1] // k)
        self.grid_w, self.grid_h = w, h
        rows = ((np.arange(h) + 0.5) * k).astype(int)
        cols = ((np.arange(w) + 0.5) * k).astype(int)
        # 경로 격자 칸 중심이 장애물에서 padding 안이면 막힌다
        clear_mm = (ndimage.distance_transform_edt(~occ) * res)[np.ix_(rows, cols)]
        self.blocked = clear_mm <= self.arena["costmap"]["padding"]

    def _cell(self, x, y):
        return (int((x - self.map_origin_mm[0]) // PLAN_CELL_MM), int((y - self.map_origin_mm[1]) // PLAN_CELL_MM))

    def _center(self, i, j):
        return (self.map_origin_mm[0] + (i + 0.5) * PLAN_CELL_MM, self.map_origin_mm[1] + (j + 0.5) * PLAN_CELL_MM)

    def _free(self, i, j, blocked=None):
        blocked = self.blocked if blocked is None else blocked
        return 0 <= i < self.grid_w and 0 <= j < self.grid_h and not blocked[j, i]

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

    def _line_free(self, a, b, blocked=None):
        """a → b 직선(mm)이 지나는 칸이 모두 갈 수 있는지.

        직선이 칸 경계를 넘는 지점을 모두 구해, 그 사이 구간마다 어느 칸에 있는지 본다 (모서리를 스치는 칸도 빠지지 않는다).
        """
        blocked = self.blocked if blocked is None else blocked
        (ox, oy), c = self.map_origin_mm, PLAN_CELL_MM
        ts = [0.0, 1.0]
        for p0, p1, o in ((a[0], b[0], ox), (a[1], b[1], oy)):
            if p1 != p0:
                lo, hi = sorted(((p0 - o) / c, (p1 - o) / c))
                ts += [(o + k * c - p0) / (p1 - p0) for k in range(math.ceil(lo), math.floor(hi) + 1)]
        t = np.unique(np.clip(ts, 0, 1))
        mid = (t[:-1] + t[1:]) / 2 if len(t) > 1 else t
        i = ((a[0] + (b[0] - a[0]) * mid - ox) // c).astype(int)
        j = ((a[1] + (b[1] - a[1]) * mid - oy) // c).astype(int)
        if (i < 0).any() or (j < 0).any() or (i >= self.grid_w).any() or (j >= self.grid_h).any():
            return False
        return not blocked[j, i].any()

    def plan(self, goal_xy, start_xy=None):
        """A* (8방향) 후 직선으로 이어지는 점은 건너뛰어 경로를 단순하게 만든다. 기본은 지금 위치에서 출발."""
        start_xy = (self.pose["x"], self.pose["y"]) if start_xy is None else tuple(start_xy)
        blocked = self.blocked
        free = lambda i, j: self._free(i, j, blocked)  # noqa: E731
        start = self._cell(*start_xy)
        goal = self._cell(*goal_xy[:2])
        if not free(*goal):
            return None
        g, came, heap = {start: 0.0}, {}, [(0.0, start)]
        while heap:
            _, cur = heapq.heappop(heap)
            if cur == goal:
                break
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    if di == dj == 0:
                        continue
                    n = (cur[0] + di, cur[1] + dj)
                    if not free(*n) and n != start:
                        continue
                    # 대각선은 옆 두 칸도 비어 있어야 한다 (막힌 칸의 모서리를 자르지 않게)
                    if di and dj and not (free(cur[0] + di, cur[1]) and free(cur[0], cur[1] + dj)):
                        continue
                    ng = g[cur] + math.hypot(di, dj)
                    if ng < g.get(n, 1e18):
                        g[n], came[n] = ng, cur
                        h = max(abs(goal[0] - n[0]), abs(goal[1] - n[1])) + 0.414 * min(abs(goal[0] - n[0]), abs(goal[1] - n[1]))
                        heapq.heappush(heap, (ng + h, n))
        if goal not in g:
            return None
        cells = [goal]
        while cells[-1] != start:
            cells.append(came[cells[-1]])
        cells.reverse()
        # 실제 좌표 점 (처음·끝은 정확한 출발·도착 위치). 직선으로 갈 수 있으면 사이 점은 건너뛴다
        pts = [start_xy] + [self._center(*c) for c in cells[1:-1]] + [tuple(goal_xy[:2])]
        keep = [pts[0]]
        for k in range(2, len(pts)):
            if not self._line_free(keep[-1], pts[k], blocked):
                keep.append(pts[k - 1])
        keep.append(pts[-1])
        return keep

    # ---------- 시간과 취소 ----------
    def sleep(self, seconds, is_canceled):
        """시뮬레이션 시간으로 seconds만큼 기다린다. 배속을 바꾸면 진행 중인 대기에도 바로 반영된다."""
        left = seconds
        while left > 0:
            if is_canceled() or self.abort:
                raise Canceled()
            time.sleep(0.02)
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
        arrival = self._arrival(place)
        path = None if arrival is None else self.plan(arrival)
        if path is None:
            return {"success": False, "error": "NO_PATH"}
        ax, ay, ayaw = arrival
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
                with self.lock:
                    self.pose["x"] = x0 + (x1 - x0) * moved / seg
                    self.pose["y"] = y0 + (y1 - y0) * moved / seg
                self.sleep(0.03, is_canceled)
            feedback(f"moving {math.hypot(ax - x1, ay - y1) / 1000:.2f} m left")
        self._turn_to(ayaw, is_canceled)
        with self.lock:
            self.path = []
            self.here = place
        return {"success": True, "dist_to_target_mm": round(math.hypot(ax - self.targets[place][0], ay - self.targets[place][1]))}

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
        }

    def snapshot(self):
        with self.lock:
            return copy.deepcopy({
                "pose": self.pose, "path": self.path, "light": self.light, "seen_signal": self.seen_signal,
                "barcode": self.barcode if self.barcode_read else None, "boxes": self.boxes,
                "storage": {str(k): v for k, v in self.storage.items()}, "placed": self.placed,
                "verified": sorted(self.verified), "here": self.here,
                "command": self.command, "history": self.history[-30:],
                "commander": self.commander, "time_scale": self.time_scale, "web_last": self.web_last,
                "answer": {"barcode": self.barcode, "signal": "".join(self.signal), "pucks": self.initial_pucks},
                "paused": self.paused, "replay": self.replay,
                "decisions_key": f"{self.decisions[0]['run_id'] if self.decisions else ''}:{len(self.decisions)}:"
                                 f"{sum('outcome' in d for d in self.decisions)}",
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
        return {"status": "OK", "error": None, "here": self.here, "pose": pose, "localization": "ok",
                "arm_stowed": True, "gripper": "empty"}

    def run(self, name, args, is_canceled, feedback):
        handler = getattr(self, "_" + name.lower(), None)
        if handler is None:
            return {"success": False, "error": "UNKNOWN_COMMAND", "robot": self.robot_status()}
        try:
            out = handler(args, is_canceled, feedback)
        except Canceled:
            with self.lock:
                self.path = []
            raise
        out["robot"] = self.robot_status()
        return out

    def _read_mission_input(self, args, is_canceled, feedback):
        if self.here != "HOME":
            return {"success": False, "error": "NOT_AT_HOME"}
        feedback("reading_barcode")
        self.sleep(1.0, is_canceled)
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
        return {"success": True, "barcode": self.barcode, "signal": list(self.signal), "end_seen": True}

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
        for phase in ("aligning", "grasping", "lifting"):
            feedback(phase)
            self.sleep(1.0, is_canceled)
        key = f"LOAD:{box}"
        loaded = []
        with self.lock:
            if key in self.fail_once:  # 시험용 / 다시 실행: 기록된 실패를 같은 시점에 낸다
                self.fail_once.remove(key)
                return {"success": False, "error": "GRASP_FAILED", "pucks": [],
                        "box_remaining": len(ks)}
            for k, slot in zip(ks, free):  # 적재함이 모자라면 들어가는 만큼만
                puck = self.boxes[box][k]
                self.boxes[box][k] = None
                self.storage[slot] = puck
                loaded.append({"slot": slot, "puck": {**puck, "conf": round(self.rng.uniform(0.85, 0.99), 2)}})
            remaining = sum(p is not None for p in self.boxes[box])
        feedback("stowing")
        self.sleep(0.5 * len(loaded), is_canceled)
        return {"success": True, "pucks": loaded, "box_remaining": remaining}

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
        return {"success": True, "all_ok": all(i["ok"] for i in items) and not extra,
                "items": items, "extra": extra, "cells": cells}
