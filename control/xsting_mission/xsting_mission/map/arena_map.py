"""경기장 지도·코스트맵·경로 계산 (fake_world 주행과 COMPARE_ROUTES가 같이 쓴다)."""
import heapq
import math

import numpy as np

MAP_RES_MM = 10  # 벽 지도 해상도
MAP_MARGIN_MM = 100  # 바깥 벽 밖으로 지도를 이만큼 더 그린다
PLAN_CELL_MM = 50  # 코스트맵·경로 격자 (Nav2 costmap resolution 0.05 m와 같다)
APPROACH_MM = (330, 520)  # GOTO 도착 자리: 대상 중심에서 로봇 중심까지 무작위 거리 범위


class ArenaMap:
    """config/arena.yaml dict로 만든 지도. 좌표는 미션지 기준 mm, 로봇은 점으로 본다."""

    def __init__(self, arena):
        self.arena = arena
        self.targets = {name: tuple(place["target"])
                        for name, place in arena["places"].items()}
        self._load_map(arena)
        self._build_plan_grid()
        self._graph = None  # 경로 길이용 격자 그래프 (처음 쓸 때 만든다)
        self._place_dist = {}  # 장소에서 출발한 최단 거리 (장소마다 한 번만 계산)

    # ---------- 지도 ----------
    def wall_segments(self):
        """바깥 벽 4개 + 안쪽 벽. [((x1, y1), (x2, y2)), ...]"""
        width, height = self.arena["size"]
        corners = [(0, 0), (width, 0), (width, height), (0, height)]
        outer = [(corners[k], corners[(k + 1) % 4]) for k in range(4)]
        return outer + _segments(self.arena.get("walls"))

    def virtual_wall_segments(self):
        """가벽 (팀이 정한 진입 금지선). 경로에서는 벽처럼 막고 화면에는 따로 그린다."""
        return _segments(self.arena.get("virtual_walls"))

    def _load_map(self, arena):
        """벽(map_occ)·가벽(map_virtual)을 MAP_RES_MM 격자에 그린다. 0행 = 아래쪽."""
        res, margin = MAP_RES_MM, MAP_MARGIN_MM
        width_mm, height_mm = arena["size"]
        width = int((width_mm + 2 * margin) // res) + 1
        height = int((height_mm + 2 * margin) // res) + 1
        self.map_res_mm = res
        self.map_origin_mm = (-margin, -margin)
        self.map_occ = self._raster(self.wall_segments(), width, height)
        self.map_virtual = self._raster(
            self.virtual_wall_segments(), width, height)

    def _raster(self, segments, width, height):
        res, margin = self.map_res_mm, -self.map_origin_mm[0]
        xs = -margin + (np.arange(width) + 0.5) * res
        ys = -margin + (np.arange(height) + 0.5) * res
        occ = np.zeros((height, width), dtype=bool)
        half = self.arena["wall_thickness"] / 2
        for (x1, y1), (x2, y2) in segments:
            # 선분 둘레 사각형 안의 칸만 선분까지 거리를 잰다
            i0 = np.searchsorted(xs, min(x1, x2) - half)
            i1 = np.searchsorted(xs, max(x1, x2) + half)
            j0 = np.searchsorted(ys, min(y1, y2) - half)
            j1 = np.searchsorted(ys, max(y1, y2) + half)
            px, py = np.meshgrid(xs[i0:i1], ys[j0:j1])
            dx, dy = x2 - x1, y2 - y1
            t = np.clip(((px - x1) * dx + (py - y1) * dy)
                        / max(dx * dx + dy * dy, 1e-9), 0, 1)
            occ[j0:j1, i0:i1] |= (
                np.hypot(px - (x1 + t * dx), py - (y1 + t * dy)) <= half)
        return occ

    def _obstacle_rects(self):
        """벽 외 장애물: 시작·하차 박스와 obstacles. {이름: (cx, cy, 가로, 세로)}"""
        box = self.arena["box_size"]
        rects = {name: (*self.targets[name], box, box)
                 for name in self.targets if name[0] in "SL"}
        rects.update({obstacle["name"]: (*obstacle["center"], *obstacle["size"])
                      for obstacle in self.arena.get("obstacles") or []})
        return rects

    def _build_plan_grid(self):
        """코스트맵: 벽·박스에서 costmap.padding 안을 못 가는 칸으로 막는다."""
        from scipy import ndimage

        res, (ox, oy) = self.map_res_mm, self.map_origin_mm
        occ = self.map_occ | self.map_virtual  # 가벽도 벽처럼 막는다
        for cx, cy, size_x, size_y in self._obstacle_rects().values():
            i0 = int((cx - size_x / 2 - ox) // res)
            i1 = int((cx + size_x / 2 - ox) // res)
            j0 = int((cy - size_y / 2 - oy) // res)
            j1 = int((cy + size_y / 2 - oy) // res)
            occ[max(j0, 0):j1 + 1, max(i0, 0):i1 + 1] = True

        scale = PLAN_CELL_MM / res
        height, width = int(occ.shape[0] // scale), int(occ.shape[1] // scale)
        self.grid_w, self.grid_h = width, height
        rows = ((np.arange(height) + 0.5) * scale).astype(int)
        cols = ((np.arange(width) + 0.5) * scale).astype(int)
        # 경로 격자 칸 중심이 장애물에서 padding 안이면 막힌다
        clear_mm = (ndimage.distance_transform_edt(~occ) * res)[
            np.ix_(rows, cols)]
        self.blocked = clear_mm <= self.arena["costmap"]["padding"]

    def _cell(self, x, y):
        ox, oy = self.map_origin_mm
        return (int((x - ox) // PLAN_CELL_MM), int((y - oy) // PLAN_CELL_MM))

    def _center(self, i, j):
        ox, oy = self.map_origin_mm
        return (ox + (i + 0.5) * PLAN_CELL_MM, oy + (j + 0.5) * PLAN_CELL_MM)

    def _free(self, i, j, blocked=None):
        blocked = self.blocked if blocked is None else blocked
        return (0 <= i < self.grid_w and 0 <= j < self.grid_h
                and not blocked[j, i])

    # ---------- 경로 ----------
    def _line_free(self, a, b, blocked=None):
        """a → b 직선(mm)이 지나는 칸이 모두 비었는지 (모서리를 스치는 칸 포함)."""
        blocked = self.blocked if blocked is None else blocked
        (ox, oy), cell = self.map_origin_mm, PLAN_CELL_MM

        # 직선이 칸 경계를 넘는 지점(t)을 모두 구해 구간마다 어느 칸인지 본다
        ts = [0.0, 1.0]
        for p0, p1, origin in ((a[0], b[0], ox), (a[1], b[1], oy)):
            if p1 != p0:
                lo, hi = sorted(((p0 - origin) / cell, (p1 - origin) / cell))
                ts += [(origin + k * cell - p0) / (p1 - p0)
                       for k in range(math.ceil(lo), math.floor(hi) + 1)]
        t = np.unique(np.clip(ts, 0, 1))
        mid = (t[:-1] + t[1:]) / 2 if len(t) > 1 else t

        i = ((a[0] + (b[0] - a[0]) * mid - ox) // cell).astype(int)
        j = ((a[1] + (b[1] - a[1]) * mid - oy) // cell).astype(int)
        if ((i < 0).any() or (j < 0).any()
                or (i >= self.grid_w).any() or (j >= self.grid_h).any()):
            return False
        return not blocked[j, i].any()

    def plan(self, goal_xy, start_xy):
        """A*(8방향) 경로 [(x, y), ...]를 직선 구간으로 줄여 돌려준다. 못 가면 None."""
        start_xy = tuple(start_xy[:2])
        blocked = self.blocked

        def free(i, j):
            return self._free(i, j, blocked)

        start = self._cell(*start_xy)
        goal = self._cell(*goal_xy[:2])
        if not free(*goal):
            return None

        cost, came_from, heap = {start: 0.0}, {}, [(0.0, start)]
        while heap:
            _, current = heapq.heappop(heap)
            if current == goal:
                break
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    if di == dj == 0:
                        continue
                    neighbor = (current[0] + di, current[1] + dj)
                    if not free(*neighbor) and neighbor != start:
                        continue
                    # 대각선은 옆 두 칸도 비어야 한다 (막힌 칸 모서리를 자르지 않게)
                    if di and dj and not (free(current[0] + di, current[1])
                                          and free(current[0],
                                                   current[1] + dj)):
                        continue
                    new_cost = cost[current] + math.hypot(di, dj)
                    if new_cost < cost.get(neighbor, 1e18):
                        cost[neighbor], came_from[neighbor] = new_cost, current
                        gap_i = abs(goal[0] - neighbor[0])
                        gap_j = abs(goal[1] - neighbor[1])
                        heuristic = (max(gap_i, gap_j)
                                     + 0.414 * min(gap_i, gap_j))
                        heapq.heappush(heap, (new_cost + heuristic, neighbor))
        if goal not in cost:
            return None

        cells = [goal]
        while cells[-1] != start:
            cells.append(came_from[cells[-1]])
        cells.reverse()

        # 처음·끝은 정확한 출발·도착 위치. 직선으로 갈 수 있으면 사이 점은 건너뛴다
        points = ([start_xy] + [self._center(*cell) for cell in cells[1:-1]]
                  + [tuple(goal_xy[:2])])
        kept = [points[0]]
        for k in range(2, len(points)):
            if not self._line_free(kept[-1], points[k], blocked):
                kept.append(points[k - 1])
        kept.append(points[-1])
        return kept

    # ---------- 경로 길이 (COMPARE_ROUTES) ----------
    def _build_graph(self):
        """갈 수 있는 칸을 8방향으로 잇는 그래프 (무게 mm, 대각선은 옆 두 칸이 빌 때만)."""
        from scipy.sparse import coo_matrix

        free = ~self.blocked
        node = -np.ones(free.shape, dtype=int)
        node[free] = np.arange(free.sum())
        height, width = free.shape
        padded = np.pad(free, 1)  # 바깥을 막힌 칸으로 덧대 이웃을 그대로 본다

        def shifted(dj, di):
            """(j+dj, i+di) 칸이 비었는지."""
            return padded[1 + dj:height + 1 + dj, 1 + di:width + 1 + di]

        rows, cols, weights = [], [], []
        for dj, di in ((0, 1), (1, 0), (1, 1), (1, -1)):
            linked = free & shifted(dj, di)
            if dj and di:
                linked &= shifted(dj, 0) & shifted(0, di)
            jj, ii = np.nonzero(linked)
            src, dst = node[jj, ii], node[jj + dj, ii + di]
            rows += [src, dst]
            cols += [dst, src]
            weights += [np.full(len(src),
                                PLAN_CELL_MM * math.hypot(di, dj))] * 2
        count = int(free.sum())
        self._graph = coo_matrix(
            (np.concatenate(weights),
             (np.concatenate(rows), np.concatenate(cols))),
            shape=(count, count)).tocsr()
        self._node = node

    def _nodes_near(self, place):
        """로봇이 서는 칸들: 박스는 대상 둘레 APPROACH_MM 안의 빈 칸, HOME은 그 점의 칸."""
        tx, ty = self.targets[place]
        if place[0] not in "SL":
            i, j = self._cell(tx, ty)
            return [self._node[j, i]] if self._node[j, i] >= 0 else []

        reach = APPROACH_MM[1] / PLAN_CELL_MM + 1
        ci, cj = self._cell(tx, ty)
        nodes = []
        for j in range(max(0, int(cj - reach)),
                       min(self.grid_h, int(cj + reach) + 1)):
            for i in range(max(0, int(ci - reach)),
                           min(self.grid_w, int(ci + reach) + 1)):
                cx, cy = self._center(i, j)
                if (self._node[j, i] >= 0
                        and math.hypot(cx - tx, cy - ty) <= APPROACH_MM[1]):
                    nodes.append(self._node[j, i])
        return nodes

    def _dist_from(self, sources):
        from scipy.sparse.csgraph import dijkstra

        return dijkstra(self._graph, indices=sources, min_only=True)

    def route_lengths(self, start_xy, routes):
        """route마다 {"route", "total_mm", "legs_mm"}. 못 가는 구간·total은 None."""
        # 장소까지는 대상 둘레 가장 가까운 칸까지 잰다 (50mm 격자라 조금 길 수 있다)
        if self._graph is None:
            self._build_graph()
        i, j = self._cell(*start_xy[:2])
        start_node = self._node[j, i] if self._free(i, j) else -1
        start_dist = self._dist_from([start_node]) if start_node >= 0 else None

        results = []
        for route in routes:
            legs, dist = [], start_dist
            for place in route:
                goals = self._nodes_near(place)
                length = (None if dist is None or not goals
                          else float(np.min(dist[goals])))
                legs.append(None if length is None or math.isinf(length)
                            else int(round(length / 10.0)) * 10)
                # 다음 구간은 이 장소 둘레에서 출발
                if place not in self._place_dist:
                    self._place_dist[place] = (
                        self._dist_from(goals) if goals else None)
                dist = self._place_dist[place]
            total = None if None in legs else sum(legs)
            results.append(
                {"route": list(route), "total_mm": total, "legs_mm": legs})
        return results


def _segments(walls):
    """arena.yaml의 벽 dict를 [((x1, y1), (x2, y2)), ...]로."""
    return [(tuple(wall["from"]), tuple(wall["to"]))
            for wall in (walls or {}).values()]
