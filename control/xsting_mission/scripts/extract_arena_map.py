"""경기장 USD의 벽(maze)을 위에서 내려다본 격자 지도(Nav2 map 형식: pgm + yaml)로 만든다.

좌표는 미션지 기준: 원점은 왼쪽 아래 기둥 중심, +Y가 지도 위쪽, 단위 m.
USD 라이브러리(pxr)가 있는 파이썬으로 실행한다 (pip install usd-core 또는 Isaac Sim 파이썬).
  python3 extract_arena_map.py <arena/scene.usda> <출력 폴더> [해상도_m=0.01]
"""
import json
import sys
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom

scene, out_dir = sys.argv[1], Path(sys.argv[2])
res = float(sys.argv[3]) if len(sys.argv) > 3 else 0.01
stage = Usd.Stage.Open(scene)

# 경기장 USD 원점 → 미션지 원점 (scene.usda customLayerData의 mapOriginM)
meta = json.loads(stage.GetRootLayer().customLayerData["boxRandomizer"])
ox, oy, _ = meta["mission"]["mapOriginM"]
W, H = 4.464, 6.364  # 경기장 외곽 (미션지)
x0, y0 = -0.03, -0.03  # 외곽 벽 바깥 끝
nx, ny = int(round((W) / res)), int(round((H) / res))
occ = np.zeros((ny, nx), dtype=bool)

xs = x0 + (np.arange(nx) + 0.5) * res
ys = y0 + (np.arange(ny) + 0.5) * res


def fill_triangle(a, b, c):
    minx, maxx = min(a[0], b[0], c[0]), max(a[0], b[0], c[0])
    miny, maxy = min(a[1], b[1], c[1]), max(a[1], b[1], c[1])
    i0, i1 = np.searchsorted(xs, minx), np.searchsorted(xs, maxx)
    j0, j1 = np.searchsorted(ys, miny), np.searchsorted(ys, maxy)
    if i1 <= i0 or j1 <= j0:
        return
    gx, gy = np.meshgrid(xs[i0:i1], ys[j0:j1])
    d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
    if abs(d) < 1e-12:
        return
    l1 = ((b[1] - c[1]) * (gx - c[0]) + (c[0] - b[0]) * (gy - c[1])) / d
    l2 = ((c[1] - a[1]) * (gx - c[0]) + (a[0] - c[0]) * (gy - c[1])) / d
    inside = (l1 >= 0) & (l2 >= 0) & (l1 + l2 <= 1)
    occ[j0:j1, i0:i1] |= inside


def draw_line(a, b):
    # 얇은 벽판은 위에서 본 면적이 격자보다 작아서 모서리 선으로 칠한다
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) / (res / 2)) + 1
    for t in np.linspace(0.0, 1.0, n + 1):
        i = int((a[0] + (b[0] - a[0]) * t - x0) / res)
        j = int((a[1] + (b[1] - a[1]) * t - y0) / res)
        if 0 <= i < nx and 0 <= j < ny:
            occ[j, i] = True


for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/maze")):
    if not prim.IsA(UsdGeom.Mesh) or "collision" in str(prim.GetPath()):
        continue
    mesh = UsdGeom.Mesh(prim)
    m = np.array(mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
    pts = np.array(mesh.GetPointsAttr().Get(), dtype=float)
    pts = np.c_[pts, np.ones(len(pts))] @ m  # USD는 행 벡터 규약
    pts[:, 0] -= ox
    pts[:, 1] -= oy
    counts = mesh.GetFaceVertexCountsAttr().Get()
    idx = mesh.GetFaceVertexIndicesAttr().Get()
    k = 0
    for n in counts:
        face = idx[k:k + n]
        k += n
        if max(pts[i][2] for i in face) < 0.01:
            continue  # 바닥에 붙은 면은 벽이 아니다
        for t in range(1, n - 1):
            fill_triangle(pts[face[0]], pts[face[t]], pts[face[t + 1]])
        for t in range(n):
            draw_line(pts[face[t]], pts[face[(t + 1) % n]])

out_dir.mkdir(parents=True, exist_ok=True)
img = np.where(occ, 0, 254).astype(np.uint8)[::-1]  # pgm은 위쪽 행부터
with open(out_dir / "arena_map.pgm", "wb") as f:
    f.write(f"P5\n{nx} {ny}\n255\n".encode())
    f.write(img.tobytes())
(out_dir / "arena_map.yaml").write_text(
    "# 경기장 벽 지도 (extract_arena_map.py로 생성). 원점은 미션지 원점, 단위 m.\n"
    f"image: arena_map.pgm\nmode: trinary\nresolution: {res}\n"
    f"origin: [{x0}, {y0}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
print(f"{nx}x{ny} cells, occupied {occ.mean() * 100:.1f}%")
