"""경기장 지도, 코스트맵, 경로 계산 (config/arena.yaml로 만든다).

가짜 경기장(fake_world)의 주행과 중계 노드(llm_commander)의 경로 길이 계산(COMPARE_ROUTES 도구)이 같이 쓴다.
좌표는 미션지 기준 mm. 로봇은 점으로 보고, 벽·박스에서 costmap.padding 안은 갈 수 없는 칸으로 막는다.
"""
import heapq
import math

import numpy as np

MAP_RES_MM = 10  # 벽 지도 해상도
MAP_MARGIN_MM = 100  # 바깥 벽 밖으로 지도를 이만큼 더 그린다
PLAN_CELL_MM = 50  # 코스트맵·경로 계산 격자 (Nav2 costmap resolution 0.05 m와 같다)
APPROACH_MM = (330, 520)  # GOTO 도착 자리: 대상 중심에서 로봇 중심까지 이 범위 안 무작위 거리


class ArenaMap:
    def __init__(self, arena):
        """arena: config/arena.yaml을 읽은 dict (장소, 벽, 장애물, 코스트맵)."""
        self.arena = arena
        self.targets = {name: tuple(p["target"]) for name, p in arena["places"].items()}
        self._load_map(arena)
        self._build_plan_grid()
        self._graph = None  # 경로 길이 계산용 격자 그래프 (처음 쓸 때 만든다)
        self._place_dist = {}  # 장소에서 출발한 최단 거리 (장소마다 한 번만 계산)

    # ---------- 지도 ----------
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

    # ---------- 경로 ----------
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

    def plan(self, goal_xy, start_xy):
        """start_xy에서 goal_xy까지 경로 [(x, y), ...]. 못 가면 None.

        A* (8방향) 후 직선으로 이어지는 점은 건너뛰어 경로를 단순하게 만든다.
        """
        start_xy = tuple(start_xy[:2])
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

    # ---------- 경로 길이 (COMPARE_ROUTES) ----------
    def _build_graph(self):
        """갈 수 있는 칸을 8방향으로 잇는 그래프 (대각선은 옆 두 칸이 비어 있을 때만). 무게는 mm."""
        from scipy.sparse import coo_matrix

        free = ~self.blocked
        idx = -np.ones(free.shape, dtype=int)
        idx[free] = np.arange(free.sum())
        h, w = free.shape
        pad = np.pad(free, 1)  # 바깥은 막힌 칸으로 덧대 이웃을 잘라 내지 않고 본다
        at = lambda dj, di: pad[1 + dj:h + 1 + dj, 1 + di:w + 1 + di]  # noqa: E731  (j+dj, i+di) 칸
        rows, cols, wts = [], [], []
        for dj, di in ((0, 1), (1, 0), (1, 1), (1, -1)):
            ok = free & at(dj, di)
            if dj and di:  # 대각선: 옆 두 칸도 비어 있어야 한다
                ok &= at(dj, 0) & at(0, di)
            jj, ii = np.nonzero(ok)
            src, dst = idx[jj, ii], idx[jj + dj, ii + di]
            rows += [src, dst]
            cols += [dst, src]
            wts += [np.full(len(src), PLAN_CELL_MM * math.hypot(di, dj))] * 2
        n = int(free.sum())
        self._graph = coo_matrix((np.concatenate(wts), (np.concatenate(rows), np.concatenate(cols))),
                                 shape=(n, n)).tocsr()
        self._node = idx

    def _nodes_near(self, place):
        """그 장소에 로봇이 서는 칸들: 박스면 대상 둘레 APPROACH_MM 안의 갈 수 있는 칸, 그 밖(HOME)은 그 점의 칸."""
        tx, ty = self.targets[place]
        if place[0] not in "SL":
            i, j = self._cell(tx, ty)
            return [self._node[j, i]] if self._node[j, i] >= 0 else []
        r = APPROACH_MM[1] / PLAN_CELL_MM + 1
        ci, cj = self._cell(tx, ty)
        out = []
        for j in range(max(0, int(cj - r)), min(self.grid_h, int(cj + r) + 1)):
            for i in range(max(0, int(ci - r)), min(self.grid_w, int(ci + r) + 1)):
                cx, cy = self._center(i, j)
                if self._node[j, i] >= 0 and math.hypot(cx - tx, cy - ty) <= APPROACH_MM[1]:
                    out.append(self._node[j, i])
        return out

    def _dist_from(self, sources):
        from scipy.sparse.csgraph import dijkstra

        return dijkstra(self._graph, indices=sources, min_only=True)

    def route_lengths(self, start_xy, routes):
        """start_xy에서 출발해 각 route(장소 ID 목록)를 차례로 도는 길이.

        반환: [{"route", "total_mm", "legs_mm"}]. 못 가는 구간이 있으면 그 구간과 total_mm이 None.
        장소의 길이는 대상 둘레(APPROACH_MM 안)의 가장 가까운 칸까지로 잰다. 격자(50mm) 경로라 실제보다 조금 길 수 있다.
        """
        if self._graph is None:
            self._build_graph()
        i, j = self._cell(*start_xy[:2])
        start_node = self._node[j, i] if self._free(i, j) else -1
        start_dist = self._dist_from([start_node]) if start_node >= 0 else None
        out = []
        for route in routes:
            legs, dist = [], start_dist
            for place in route:
                goals = self._nodes_near(place)
                d = None if dist is None or not goals else float(np.min(dist[goals]))
                legs.append(None if d is None or math.isinf(d) else int(round(d / 10.0)) * 10)
                if place not in self._place_dist:  # 다음 구간은 이 장소 둘레에서 출발
                    self._place_dist[place] = self._dist_from(goals) if goals else None
                dist = self._place_dist[place]
            total = None if None in legs else sum(legs)
            out.append({"route": list(route), "total_mm": total, "legs_mm": legs})
        return out
