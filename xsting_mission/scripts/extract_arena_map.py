"""경기장 USD 벽을 ROS 점유 격자 지도(pgm + yaml)로: <scene.usda> <출력 폴더> [해상도_m=0.01]."""
import json
import sys
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom

scene, out_dir = sys.argv[1], Path(sys.argv[2])
res = float(sys.argv[3]) if len(sys.argv) > 3 else 0.01
stage = Usd.Stage.Open(scene)

# 좌표는 미션지 기준(왼쪽 아래 기둥 중심 원점, +Y 위쪽, m). USD 원점과의 차이
meta = json.loads(stage.GetRootLayer().customLayerData["boxRandomizer"])
ox, oy, _ = meta["mission"]["mapOriginM"]
ARENA_W, ARENA_H = 4.464, 6.364  # 경기장 외곽 (미션지)
x0, y0 = -0.03, -0.03  # 외곽 벽 바깥 끝
nx, ny = int(round(ARENA_W / res)), int(round(ARENA_H / res))
occ = np.zeros((ny, nx), dtype=bool)

xs = x0 + (np.arange(nx) + 0.5) * res
ys = y0 + (np.arange(ny) + 0.5) * res


# ---------- 격자 칠하기 ----------

def fill_triangle(a, b, c):
    """삼각형 안에 중심이 든 칸을 칠한다(무게중심 좌표)."""
    minx, maxx = min(a[0], b[0], c[0]), max(a[0], b[0], c[0])
    miny, maxy = min(a[1], b[1], c[1]), max(a[1], b[1], c[1])
    i0, i1 = np.searchsorted(xs, minx), np.searchsorted(xs, maxx)
    j0, j1 = np.searchsorted(ys, miny), np.searchsorted(ys, maxy)
    if i1 <= i0 or j1 <= j0:
        return
    gx, gy = np.meshgrid(xs[i0:i1], ys[j0:j1])
    denom = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
    if abs(denom) < 1e-12:
        return
    l1 = ((b[1] - c[1]) * (gx - c[0]) + (c[0] - b[0]) * (gy - c[1])) / denom
    l2 = ((c[1] - a[1]) * (gx - c[0]) + (a[0] - c[0]) * (gy - c[1])) / denom
    inside = (l1 >= 0) & (l2 >= 0) & (l1 + l2 <= 1)
    occ[j0:j1, i0:i1] |= inside


def draw_line(a, b):
    """얇은 벽판은 위에서 본 면적이 격자보다 작아서 모서리 선으로 칠한다."""
    steps = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) / (res / 2)) + 1
    for t in np.linspace(0.0, 1.0, steps + 1):
        i = int((a[0] + (b[0] - a[0]) * t - x0) / res)
        j = int((a[1] + (b[1] - a[1]) * t - y0) / res)
        if 0 <= i < nx and 0 <= j < ny:
            occ[j, i] = True


# ---------- maze 메시 → 격자 ----------

for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/maze")):
    if not prim.IsA(UsdGeom.Mesh) or "collision" in str(prim.GetPath()):
        continue
    mesh = UsdGeom.Mesh(prim)
    transform = np.array(
        mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
    points = np.array(mesh.GetPointsAttr().Get(), dtype=float)
    points = np.c_[points, np.ones(len(points))] @ transform  # USD는 행 벡터
    points[:, 0] -= ox
    points[:, 1] -= oy
    face_sizes = mesh.GetFaceVertexCountsAttr().Get()
    indices = mesh.GetFaceVertexIndicesAttr().Get()
    start = 0
    for size in face_sizes:
        face = indices[start:start + size]
        start += size
        if max(points[index][2] for index in face) < 0.01:
            continue  # 바닥에 붙은 면은 벽이 아니다
        for corner in range(1, size - 1):
            fill_triangle(points[face[0]], points[face[corner]],
                          points[face[corner + 1]])
        for corner in range(size):
            draw_line(points[face[corner]],
                      points[face[(corner + 1) % size]])

# ---------- 저장 ----------

out_dir.mkdir(parents=True, exist_ok=True)
image = np.where(occ, 0, 254).astype(np.uint8)[::-1]  # pgm은 위쪽 행부터
with open(out_dir / "arena_map.pgm", "wb") as pgm_file:
    pgm_file.write(f"P5\n{nx} {ny}\n255\n".encode())
    pgm_file.write(image.tobytes())
(out_dir / "arena_map.yaml").write_text(
    "# 경기장 벽 지도 (extract_arena_map.py로 생성). 원점은 미션지 원점, 단위 m.\n"
    f"image: arena_map.pgm\nmode: trinary\nresolution: {res}\n"
    f"origin: [{x0}, {y0}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\n"
    "free_thresh: 0.25\n")
print(f"{nx}x{ny} cells, occupied {occ.mean() * 100:.1f}%")
