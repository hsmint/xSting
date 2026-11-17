"""미션 맵(Isaac Sim)에서 DOPE(6DoF 자세 추정) 학습용 합성 데이터를 만듦 (NVlabs DOPE / Isaac ROS DOPE 형식).

gen_rg_synth.py (YOLO용)와 같은 맵·카메라·무작위 배치를 쓰고, 라벨만 DOPE 형식으로 저장함.
물체 (에셋 실제 크기, 원점은 바닥면 → 박스 중심으로 옮겨서 씀):
  red_square, green_square, red_cylinder, green_cylinder : 3 x 3 x 6 cm 퍽 (선반 위 무작위 1~3개,
                                                           LIE_P 확률로 눕힘 - 기본: 사각 퍽 35%)
      사각 퍽 AprilTag: 끝면(윗면) 태그는 항상, 옆면 4개는 모두 / 없음 / 일부(1~3개) 를 각각 1/3
      (옆면 태그 유무로 사각/원기둥을 구분하지 않도록)
  paper_box                                              : 20.7 x 20.7 x 7 cm (맵의 시작 상자 6개 + 도크 3개, 위치 고정)

라벨 원리
  - 물체 좌표계(에셋의 x, y, z, z = 위)의 직육면체 꼭짓점 8개 + 중심 1개를 카메라로 투영 → projected_cuboid
  - 꼭짓점 순서 (DOPE / Isaac ROS DopeDecoder 와 같음):
      0 (+x,+y,+z) 1 (-x,+y,+z) 2 (-x,-y,+z) 3 (+x,-y,+z)
      4 (+x,+y,-z) 5 (-x,+y,-z) 6 (-x,-y,-z) 7 (+x,-y,-z)  8 중심
  - 자세(location cm, quaternion_xyzw)는 OpenCV 카메라 좌표 (x 오른쪽, y 아래, z 앞)
  - 대칭 처리: 물체 z축(퍽은 긴 축)으로 돌려도 모양이 같으므로, 꼭짓점 번호가 뒤섞이지 않게 회전을 정함
      사각 퍽 / paper_box: 90도 단위 4가지 중 물체 +x 가 카메라 쪽을 가장 많이 향하는 것
      원기둥 퍽: 물체 +x 가 정확히 카메라 쪽(z축에 수직 방향)을 향하도록   (눕힌 퍽도 같은 규칙)
  - 환경 다양화: 바닥/벽/기둥/단상 색·재질(DR_MATERIAL_P), 조명 색온도·밝기·햇빛 방향, 방해 물체 0~8개
    (빨강·초록 제외, 물체를 가리기도 함), 카메라 높이 ±3 cm / 기울기 ±5도
  - visibility = 카메라를 향한 표면 점 중 실제로 보이는 점의 비율 (깊이 사진과 비교: 가림 + 화면 밖 잘림 반영)
    MIN_VISIBLE 미만은 뺌. bounding_box 는 보이는 점들의 범위

실행 (Isaac Sim 파이썬, 화면 없이):
  ~/pa-course/.venv-isaacsim/bin/python code/gen_dope_synth.py --out ~/xsting_ws/vision/sim_dope/run01 --frames 5000
  ... --lie-square 1.0 --seed 1   # 사각 퍽을 전부 눕힌 데이터 (seed 를 바꿔야 다른 장면)
  --debug 를 붙이면 <out>/debug/ 에 투영한 직육면체를 그린 확인용 사진도 저장
결과:
  <out>/_camera_settings.json, <out>/_object_settings.json, <out>/dope_config.yaml (Isaac ROS 용 치수)
  <out>/{train,val}/000000.jpg + 000000.json ...   (val = VAL_SHELF 선반 주변 장면)
"""
import argparse
import json
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gen_rg_synth import (ASSETS, CAM_H, CAM_PITCH, HFOV, RES, SCENE, SCENE_RANDOM, SHELF_TOP, VAL_SHELF,  # noqa: E402
                          Map, load_map_module, sample_camera, seg_dist)

PUCKS = ["red_square", "green_square", "red_cylinder", "green_cylinder"]
CLASSES = PUCKS + ["paper_box"]
SIZE = {c: (0.03, 0.03, 0.06) for c in PUCKS}          # m, 물체 좌표계 x y z
SIZE["paper_box"] = (0.207, 0.207, 0.07)
SYMMETRY = {"red_square": 4, "green_square": 4, "red_cylinder": 0, "green_cylinder": 0, "paper_box": 4}  # 0 = 연속
MIN_VISIBLE = 0.3
MIN_BOX_PX = 10
# 환경 다양화 (도메인 랜덤화)
DR_MATERIAL_P = 0.7                 # 바닥/벽/기둥/단상 색·재질을 바꾸는 장면 비율 (나머지는 원래 맵 그대로)
DR_TARGETS = ["/World/Environment/GroundPlane", "/World/maze/walls", "/World/maze/posts", "/World/Mission"]
N_DISTRACT = 8                      # 방해 물체 최대 개수 (장면마다 0 ~ N_DISTRACT)
DISTRACT_SIZE = (0.04, 0.25)        # 방해 물체 한 변 크기 범위 (m)
CAM_H_JITTER, CAM_PITCH_JITTER = 0.03, 5.0   # 카메라 높이 ±m, 아래 기울기 ±deg
# 눕힌 퍽: 클래스별 눕힐 확률 (눕히면 긴 축이 수평, 옆면이 선반에 닿음)
LIE_P = {"red_square": 0.35, "green_square": 0.35, "red_cylinder": 0.0, "green_cylinder": 0.0}
SHELF_HALF_TOP = 0.1035                      # 선반(paper_box) 윗면 반폭
# 사각 퍽 AprilTag 빼기: 태그 마스크(puck_box_tagmask.png, 4096 px)의 태그 자리 (x0, x1, y0, y1)
#   0~3 = 옆면 4개, 4~5 = 끝면 2개(세우면 윗면/아랫면, 눕히면 옆을 봄)
TAG_BOXES = [(678, 1355, 2224, 2901), (2718, 3395, 2224, 2901), (678, 1355, 3248, 3925), (2718, 3395, 3248, 3925),
             (1189, 1866, 1201, 1878), (169, 845, 1202, 1878)]
#   끝면(윗면) 태그는 항상 둠. 옆면 4개만: 모두 있음 / 하나도 없음 / 일부(1~3개) 를 각각 1/3
SIDE_ALL_P, SIDE_NONE_P = 1 / 3, 1 / 3
CORNER_SIGNS = [(1, 1, 1), (-1, 1, 1), (-1, -1, 1), (1, -1, 1), (1, 1, -1), (-1, 1, -1), (-1, -1, -1), (1, -1, -1)]


def canonical_axes(cls, ax0, ay0, az, to_cam):
    """대칭 물체의 회전을 하나로 정함: 물체 z축(긴 축/대칭축)은 그대로 두고 그 축으로 돌린 것 중
    사각(4회 대칭)은 90도 단위 4가지 중 +x 가 카메라 쪽을 가장 많이 향하는 것, 원기둥(연속 대칭)은 +x 가
    정확히 카메라 쪽(z축에 수직인 방향)을 향하는 것. 세운 퍽·눕힌 퍽 모두 같은 규칙. 반환: (ax, ay, az)"""
    def dot(a, b):
        return sum(a[i] * b[i] for i in range(3))

    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
    n = SYMMETRY[cls]
    if n == 0:
        k = dot(to_cam, az)
        pr = tuple(to_cam[i] - k * az[i] for i in range(3))
        L = math.sqrt(dot(pr, pr)) or 1.0
        ax = tuple(v / L for v in pr)
    else:
        best = None
        for j in range(n):
            t = 2 * math.pi * j / n
            cand = tuple(math.cos(t) * ax0[i] + math.sin(t) * ay0[i] for i in range(3))
            if best is None or dot(cand, to_cam) > dot(best, to_cam):
                best = cand
        ax = best
    return ax, cross(az, ax), az


def puck_pose(x, y, yaw_deg, lying):
    """퍽 배치 -> (USD translate, USD rotateXYZ deg, 물체 중심, 물체 축 x, y, z (월드)).
    퍽 에셋은 원점이 바닥면 중심, z = 긴 축. 눕히면 rotateXYZ = (90, 0, yaw): 물체 y축이 위, z축이 수평"""
    c, s_ = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    hz = SIZE["red_square"][2] / 2                     # 긴 축 절반 (0.03)
    if not lying:
        ax, ay, az = (c, s_, 0.0), (-s_, c, 0.0), (0.0, 0.0, 1.0)
        center = (x, y, SHELF_TOP + hz)
        return (x, y, SHELF_TOP), (0.0, 0.0, yaw_deg), center, ax, ay, az
    ax, ay, az = (c, s_, 0.0), (0.0, 0.0, 1.0), (s_, -c, 0.0)   # Rz(yaw) * Rx(90)
    center = (x, y, SHELF_TOP + SIZE["red_square"][1] / 2)
    t = tuple(center[i] - hz * az[i] for i in range(3))          # 원점(바닥면 중심) = 중심 - 긴축 * 0.03
    return t, (90.0, 0.0, yaw_deg), center, ax, ay, az


def place_pucks_dope(rng, shelf_xy):
    """선반 하나 위의 퍽 [(클래스, x, y, yaw, 눕힘)] (x, y = 물체 중심). 눕힌 퍽은 길어서 간격을 더 둠"""
    n = rng.choice([1, 2, 2, 2, 3])
    out = []
    for _ in range(200):
        if len(out) == n:
            break
        cls = rng.choice(PUCKS)
        lying = rng.random() < LIE_P[cls]
        r = 0.032 if lying else 0.022                   # 위에서 본 반경 (대략)
        m = SHELF_HALF_TOP - r
        x, y = shelf_xy[0] + rng.uniform(-m, m), shelf_xy[1] + rng.uniform(-m, m)
        if all(math.hypot(x - o[1], y - o[2]) >= r + o[5] + 0.004 for o in out):
            out.append((cls, x, y, rng.uniform(0, 360), lying, r))
    return [o[:5] for o in out]


def surface_samples(cls, n=10):
    """물체 표면 점과 바깥 방향 법선 (물체 좌표계, 중심 원점). 원기둥 퍽은 원기둥, 나머지는 직육면체"""
    import numpy as np
    sx, sy, sz = SIZE[cls]
    t = (np.arange(n) + 0.5) / n - 0.5                # -0.5 .. 0.5 사이 n 개
    a, b = np.meshgrid(t, t)
    a, b = a.ravel(), b.ravel()
    P, N = [], []
    if SYMMETRY[cls] == 0:                            # 원기둥: 옆면 + 윗면 + 아랫면
        r = sx / 2
        th = np.linspace(0, 2 * np.pi, 4 * n, endpoint=False)
        T, Z = np.meshgrid(th, t * sz)
        T, Z = T.ravel(), Z.ravel()
        P.append(np.c_[r * np.cos(T), r * np.sin(T), Z]); N.append(np.c_[np.cos(T), np.sin(T), np.zeros_like(T)])
        rr, tt = np.sqrt((a + 0.5)) * r, (b + 0.5) * 2 * np.pi
        for sgn in (1, -1):
            P.append(np.c_[rr * np.cos(tt), rr * np.sin(tt), np.full_like(rr, sgn * sz / 2)])
            N.append(np.tile([0, 0, sgn], (len(rr), 1)))
    else:                                             # 직육면체: 6면
        for axis, size in enumerate((sx, sy, sz)):
            o = [i for i in range(3) if i != axis]
            s2 = (sx, sy, sz)
            for sgn in (1, -1):
                p = np.zeros((len(a), 3)); nrm = np.zeros((len(a), 3))
                p[:, axis] = sgn * size / 2; p[:, o[0]] = a * s2[o[0]]; p[:, o[1]] = b * s2[o[1]]
                nrm[:, axis] = sgn
                P.append(p); N.append(nrm)
    return np.vstack(P).astype(float), np.vstack(N).astype(float)


SURF = {}


def rot_to_quat_xyzw(R):
    """3x3 회전행렬 (리스트) -> [x, y, z, w]"""
    m00, m01, m02 = R[0]; m10, m11, m12 = R[1]; m20, m21, m22 = R[2]
    tr = m00 + m11 + m22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2; w = 0.25 * s
        x, y, z = (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2; x = 0.25 * s
        w, y, z = (m21 - m12) / s, (m01 + m10) / s, (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2; y = 0.25 * s
        w, x, z = (m02 - m20) / s, (m01 + m10) / s, (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2; z = 0.25 * s
        w, x, y = (m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s
    return [x, y, z, w]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--frames", type=int, default=5000)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--subframes", type=int, default=6, help="장면을 바꾼 뒤 렌더 누적 횟수 (잔상 방지)")
    ap.add_argument("--debug", action="store_true", help="투영한 직육면체를 그린 확인용 사진 저장")
    ap.add_argument("--lie-square", type=float, default=LIE_P["red_square"], help="사각 퍽을 눕힐 확률 (1 = 전부 눕힘)")
    ap.add_argument("--lie-cyl", type=float, default=LIE_P["red_cylinder"], help="원기둥 퍽을 눕힐 확률")
    args = ap.parse_args()
    LIE_P.update({"red_square": args.lie_square, "green_square": args.lie_square,
                  "red_cylinder": args.lie_cyl, "green_cylinder": args.lie_cyl})
    out = args.out.expanduser().resolve()
    if out.exists():
        sys.exit(f"{out} exists: choose a new --out")

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    import cv2
    import numpy as np
    import omni.kit.app
    import omni.replicator.core as rep
    import omni.usd
    from isaacsim.core.utils.semantics import add_labels
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

    for c in CLASSES:
        SURF[c] = surface_samples(c)
    world = Map(load_map_module())
    rng = random.Random(args.seed)
    ka = omni.kit.app.get_app()
    ctx = omni.usd.get_context()
    ctx.open_stage(str(SCENE))
    for _ in range(10):
        ka.update()
    stage = ctx.get_stage()
    stage.GetPrimAtPath("/World/Pucks").SetActive(False)

    # 퍽 풀 (gen_rg_synth.py 와 같음)
    pool = {}
    for k in world.shelves:
        for cls in PUCKS:
            for i in range(3):
                prim = stage.DefinePrim(f"/Gen/s{k}_{cls}_{i}")
                prim.GetPayloads().AddPayload(str(SCENE.parent / ASSETS[cls]))
                xf = UsdGeom.Xformable(prim)
                xf.ClearXformOpOrder()
                t_op, r_op = xf.AddTranslateOp(), xf.AddRotateXYZOp()
                xf.AddScaleOp().Set(Gf.Vec3f(1, 1, 1))
                add_labels(prim, labels=[cls], instance_name="class")
                for q in Usd.PrimRange(prim):
                    if q.HasAPI(UsdPhysics.RigidBodyAPI):
                        UsdPhysics.RigidBodyAPI(q).CreateRigidBodyEnabledAttr().Set(False)
                pool[(k, cls, i)] = (UsdGeom.Imageable(prim), t_op, r_op, str(prim.GetPath()))

    # 옆면 태그 조합 16가지 마스크를 만들어 두고, 사각 퍽마다 장면별로 바꿔 끼움 (원본 텍스처는 그대로)
    tex_dir = SCENE.parent / "textures" / "puck"
    var_dir = tex_dir / "tagmask_var"
    var_dir.mkdir(exist_ok=True)
    base_mask = cv2.imread(str(tex_dir / "puck_box_tagmask.png"), cv2.IMREAD_GRAYSCALE)
    mask_files = {}
    for bits in range(64):                   # bit i = 태그 자리 i 남김 (TAG_BOXES 순서)
        f = var_dir / f"tagmask_faces{bits:02d}.png"
        if not f.exists():
            m = base_mask.copy()
            for i, (x0, x1, y0, y1) in enumerate(TAG_BOXES):
                if not bits >> i & 1:
                    m[y0 - 8:y1 + 9, x0 - 8:x1 + 9] = 0
            cv2.imwrite(str(f), m)
        mask_files[bits] = Sdf.AssetPath(str(f))
    tag_attr = {}
    for (k, cls, i), (_, _, _, path) in pool.items():
        if "square" in cls:
            sh = stage.GetPrimAtPath(path + "/box/Looks/mtlximage4")
            tag_attr[path] = sh.GetAttribute("inputs:file") if sh.IsValid() else None
    print(f"tag mask shaders found: {sum(a is not None for a in tag_attr.values())}/{len(tag_attr)}", flush=True)
    # paper_box: 맵에 있는 시작 상자 + 도크 (위치 고정)
    boxes = []
    for parent in ("/World/StartBoxes", "/World/Docks"):
        for prim in stage.GetPrimAtPath(parent).GetChildren():
            add_labels(prim, labels=["paper_box"], instance_name="class")
            m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            t = m.ExtractTranslation()
            yaw = math.degrees(math.atan2(m[0][1], m[0][0]))   # 물체 x축의 방향 (Gf 는 행 = 축)
            boxes.append((str(prim.GetPath()), (t[0], t[1], t[2]), yaw))
    print(f"paper_box {len(boxes)}: {[b[0] for b in boxes]}", flush=True)

    dome = next(UsdLux.DomeLight(p) for p in stage.Traverse() if p.IsA(UsdLux.DomeLight))
    dome_i = dome.GetIntensityAttr().Get()
    tower = stage.GetPrimAtPath("/World/SignTower").GetVariantSets().GetVariantSet("signal")

    # ---- 환경 다양화 준비 ----
    def make_mat(path):
        mat = UsdShade.Material.Define(stage, path)
        sh = UsdShade.Shader.Define(stage, path + "/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        ins = (sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f),
               sh.CreateInput("roughness", Sdf.ValueTypeNames.Float),
               sh.CreateInput("metallic", Sdf.ValueTypeNames.Float))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return mat, ins

    def rand_color(avoid_red_green=False):
        import colorsys
        while True:
            h = rng.random()
            if not avoid_red_green or (0.06 < h < 0.2 or 0.45 < h < 0.92):   # 빨강·초록 계열 제외 (퍽과 헷갈리지 않게)
                break
        return Gf.Vec3f(*colorsys.hsv_to_rgb(h, rng.uniform(0, 0.8), rng.uniform(0.15, 0.95)))

    def set_mat(ins, avoid=False):
        ins[0].Set(rand_color(avoid)); ins[1].Set(rng.uniform(0.2, 1.0)); ins[2].Set(rng.uniform(0.0, 0.3))

    dr = []   # (prim, 원래 직접 바인딩된 재질 또는 None, 랜덤 재질, 입력)
    for i, t in enumerate(DR_TARGETS):
        prim = stage.GetPrimAtPath(t)
        if not prim.IsValid():
            print(f"skip DR target {t}", flush=True)
            continue
        orig = UsdShade.MaterialBindingAPI(prim).GetDirectBinding().GetMaterial()
        mat, ins = make_mat(f"/Gen/Looks/dr_{i}")
        dr.append((prim, orig if orig and orig.GetPrim().IsValid() else None, mat, ins))
    dome.CreateEnableColorTemperatureAttr(True)
    sun = UsdLux.DistantLight.Define(stage, "/Gen/sun")
    sun.CreateAngleAttr(1.0)
    sun.CreateEnableColorTemperatureAttr(True)
    sun_rot = UsdGeom.Xformable(sun).AddRotateXYZOp()
    distract = []
    for i in range(N_DISTRACT):
        kind = ["Cube", "Sphere", "Cylinder", "Cone"][i % 4]
        g = {"Cube": UsdGeom.Cube, "Sphere": UsdGeom.Sphere, "Cylinder": UsdGeom.Cylinder, "Cone": UsdGeom.Cone}[kind]
        prim = g.Define(stage, f"/Gen/distract_{i}")
        if kind == "Cube":
            prim.CreateSizeAttr(1.0)
        elif kind == "Sphere":
            prim.CreateRadiusAttr(0.5)
        else:
            prim.CreateRadiusAttr(0.5); prim.CreateHeightAttr(1.0); prim.CreateAxisAttr("Z")
        xf = UsdGeom.Xformable(prim)
        ops = (xf.AddTranslateOp(), xf.AddRotateXYZOp(), xf.AddScaleOp())
        mat, ins = make_mat(f"/Gen/Looks/distract_{i}")
        UsdShade.MaterialBindingAPI.Apply(prim.GetPrim()).Bind(mat)
        distract.append((UsdGeom.Imageable(prim), ops, ins))
    avoid_xy = [s for s in world.shelves.values()] + [(b[1][0], b[1][1]) for b in boxes]

    def place_distractors(cx_, cy_, yaw_deg):
        for im, *_ in distract:
            im.MakeInvisible()
        n = rng.randint(0, N_DISTRACT)
        for im, (t_op, r_op, s_op), ins in rng.sample(distract, n):
            for _ in range(30):
                d = rng.uniform(0.25, 2.0)
                a = math.radians(yaw_deg + rng.uniform(-HFOV / 2, HFOV / 2))
                px, py = cx_ + d * math.cos(a), cy_ + d * math.sin(a)
                sx_, sy_, sz_ = (rng.uniform(*DISTRACT_SIZE) for _ in range(3))
                rad = math.hypot(sx_, sy_) / 2
                if not (rad < px < world.w - rad and rad < py < world.h - rad):
                    continue
                if any(seg_dist((px, py), p, q) < rad + 0.01 for p, q in world.walls):
                    continue
                if any(max(abs(px - ax_), abs(py - ay_)) < 0.11 + rad for ax_, ay_ in avoid_xy):
                    continue                     # paper_box 와 겹치지 않게
                t_op.Set(Gf.Vec3d(px, py, sz_ / 2)); r_op.Set(Gf.Vec3f(0, 0, rng.uniform(0, 360)))
                s_op.Set(Gf.Vec3f(sx_, sy_, sz_)); set_mat(ins, avoid=True); im.MakeVisible()
                break

    def randomize_env():
        use = rng.random() < DR_MATERIAL_P
        for prim, orig, mat, ins in dr:
            api = UsdShade.MaterialBindingAPI.Apply(prim)
            if use:
                set_mat(ins)
                api.Bind(mat, UsdShade.Tokens.strongerThanDescendants)
            elif orig is not None:
                api.Bind(orig)
            else:
                api.UnbindDirectBinding()
        dome.GetIntensityAttr().Set(dome_i * rng.uniform(0.5, 1.8))
        dome.CreateColorTemperatureAttr().Set(rng.uniform(3000, 7500))
        sun.CreateIntensityAttr().Set(rng.uniform(0, 3.0) * (dome_i or 1000))
        sun.CreateColorTemperatureAttr().Set(rng.uniform(3000, 7500))
        sun_rot.Set(Gf.Vec3f(rng.uniform(-60, 60), rng.uniform(-60, 60), rng.uniform(0, 360)))
    print(f"DR targets {[str(d[0].GetPath()) for d in dr]} dome {dome_i}", flush=True)

    cam = UsdGeom.Camera.Define(stage, "/Gen/cam")
    focal = 1.88
    h_ap = 2 * focal * math.tan(math.radians(HFOV / 2))
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureAttr(h_ap)
    cam.CreateVerticalApertureAttr(h_ap * RES[1] / RES[0])
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
    cam_op = cam.AddTransformOp()
    W, H = RES
    fx = fy = focal / h_ap * W          # 정사각 픽셀
    cx, cy = W / 2.0, H / 2.0

    rp = rep.create.render_product(str(cam.GetPath()), RES)
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    # 보이는지 판정은 깊이 사진으로 함 (bounding_box_2d_tight 는 보이는 물체를 가끔 빼먹음)
    depth = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    rgb.attach([rp])
    depth.attach([rp])
    for _ in range(20):
        rep.orchestrator.step(rt_subframes=2)

    out.mkdir(parents=True)
    for sp in ("train", "val"):
        (out / sp).mkdir()
    if args.debug:
        (out / "debug").mkdir()
    cam_settings = {"camera_settings": [{"name": "isaac_cam", "horizontal_fov": HFOV,
                    "intrinsic_settings": {"fx": fx, "fy": fy, "cx": cx, "cy": cy, "s": 0},
                    "captured_image_size": {"width": W, "height": H}}]}
    (out / "_camera_settings.json").write_text(json.dumps(cam_settings, indent=2))
    obj_settings = {"exported_object_classes": CLASSES,
                    "exported_objects": [{"class": c, "segmentation_class_id": i + 1,
                                          "cuboid_dimensions": [round(v * 100, 3) for v in SIZE[c]],
                                          "symmetry_about_z": SYMMETRY[c]} for i, c in enumerate(CLASSES)]}
    (out / "_object_settings.json").write_text(json.dumps(obj_settings, indent=2))
    (out / "dope_config.yaml").write_text(
        "# Isaac ROS DOPE: 물체 치수 (cm, 물체 좌표계 x y z)\ndimensions:\n"
        + "".join(f"  {c}: [{', '.join(f'{v * 100:.1f}' for v in SIZE[c])}]\n" for c in CLASSES))

    def project(view, p):
        """월드 점 -> (OpenCV 카메라 좌표, 픽셀). view = 월드->USD 카메라 (Gf, 행벡터)"""
        q = view.Transform(Gf.Vec3d(*p))
        xc, yc, zc = q[0], -q[1], -q[2]          # USD 카메라(-z 앞, y 위) -> OpenCV(z 앞, y 아래)
        return (xc, yc, zc), ((fx * xc / zc + cx, fy * yc / zc + cy) if zc > 1e-6 else None)

    counts = {"train": 0, "val": 0, "objects": {c: 0 for c in CLASSES}, "empty": 0}
    f = 0
    while f < args.frames:
        # 물체 배치: (클래스, prim 경로, 물체 중심 xyz, 물체 축 x, y, z (월드, 대칭 정리 전), 눕힘)
        for xf, *_ in pool.values():
            xf.MakeInvisible()
        objs = []
        for path, xyz, yaw_b in boxes:
            c, s_ = math.cos(math.radians(yaw_b)), math.sin(math.radians(yaw_b))
            objs.append(("paper_box", path, (xyz[0], xyz[1], xyz[2] + SIZE["paper_box"][2] / 2),
                         (c, s_, 0.0), (-s_, c, 0.0), (0.0, 0.0, 1.0), False, None))
        for k, s in world.shelves.items():
            used = {c: 0 for c in PUCKS}
            for cls, px, py, pyaw, lying in place_pucks_dope(rng, s):
                xf, t_op, r_op, path = pool[(k, cls, used[cls])]
                used[cls] += 1
                t, rot, center, ax0, ay0, az0 = puck_pose(px, py, pyaw, lying)
                t_op.Set(Gf.Vec3d(*t))
                r_op.Set(Gf.Vec3f(*rot))
                xf.MakeVisible()
                side_tags = None
                if tag_attr.get(path) is not None:
                    u = rng.random()
                    side = 15 if u < SIDE_ALL_P else 0 if u < SIDE_ALL_P + SIDE_NONE_P else rng.randrange(1, 15)
                    bits = side | 0b110000                       # 끝면 2개 태그는 항상
                    tag_attr[path].Set(mask_files[bits])
                    side_tags = (bin(bits & 15).count("1"), bin(bits >> 4).count("1"))   # (옆면 수, 끝면 수)
                objs.append((cls, path, center, ax0, ay0, az0, lying, side_tags))
        shelf = None if rng.random() < SCENE_RANDOM else rng.choice(list(world.shelves))
        pose = sample_camera(rng, world, shelf)
        if pose is None:
            continue
        x, y, yaw = pose
        pitch = math.radians(CAM_PITCH + rng.uniform(-CAM_PITCH_JITTER, CAM_PITCH_JITTER))
        yr = math.radians(yaw)
        eye = Gf.Vec3d(x, y, CAM_H + rng.uniform(-CAM_H_JITTER, CAM_H_JITTER))
        fwd = Gf.Vec3d(math.cos(yr) * math.cos(pitch), math.sin(yr) * math.cos(pitch), -math.sin(pitch))
        view = Gf.Matrix4d().SetLookAt(eye, eye + fwd, Gf.Vec3d(0, 0, 1))   # 월드 -> 카메라
        cam_op.Set(view.GetInverse())
        randomize_env()
        place_distractors(x, y, yaw)
        tower.SetVariantSelection(rng.choice(["off", "red", "yellow", "green"]))

        rep.orchestrator.step(rt_subframes=args.subframes)
        img = rgb.get_data()[..., :3]
        dep = np.nan_to_num(np.asarray(depth.get_data(), dtype=np.float64).reshape(H, W), nan=1e9, posinf=1e9)
        V = np.array(view)                         # 월드 -> USD 카메라 (행벡터 규약)
        eye_np = np.array([eye[0], eye[1], eye[2]])

        objects = []
        for cls, path, center, ax0, ay0, az0, lying, side_tags in objs:
            sx, sy, sz = SIZE[cls]
            to_cam = tuple(eye_np[i] - center[i] for i in range(3))
            ax, ay, az = canonical_axes(cls, ax0, ay0, az0, to_cam)   # 대칭 정리한 물체 축 (월드)
            pts3, pts2 = [], []
            for gx, gy, gz in CORNER_SIGNS + [(0, 0, 0)]:
                p = tuple(center[i] + gx * sx / 2 * ax[i] + gy * sy / 2 * ay[i] + gz * sz / 2 * az[i] for i in range(3))
                c3, c2 = project(view, p)
                pts3.append(c3); pts2.append(c2)
            if any(p is None for p in pts2):
                continue
            # 보이는 비율: 카메라를 향한 표면 점 중 깊이 사진과 거리가 맞는(= 안 가려지고 화면 안인) 점의 비율
            A = np.array([ax, ay, az])                       # 행 = 물체 축 (월드)
            Pw = np.array(center) + SURF[cls][0] @ A
            Nw = SURF[cls][1] @ A
            facing = np.einsum("ij,ij->i", Nw, eye_np - Pw) > 0
            Pc = np.c_[Pw, np.ones(len(Pw))] @ V
            xc, yc, zc = Pc[:, 0], -Pc[:, 1], -Pc[:, 2]
            ok = facing & (zc > 1e-3)
            u = np.where(ok, fx * xc / np.maximum(zc, 1e-3) + cx, -1)
            v = np.where(ok, fy * yc / np.maximum(zc, 1e-3) + cy, -1)
            inimg = ok & (u >= 0) & (u < W) & (v >= 0) & (v < H)
            ui, vi = np.clip(u.astype(int), 0, W - 1), np.clip(v.astype(int), 0, H - 1)
            visible = inimg & (zc <= dep[vi, ui] + np.maximum(0.006, 0.01 * zc))
            vis = visible.sum() / max(1, facing.sum())
            if vis < MIN_VISIBLE:
                continue
            b2 = [u[visible].min(), v[visible].min(), u[visible].max(), v[visible].max()]
            if b2[2] - b2[0] < MIN_BOX_PX or b2[3] - b2[1] < MIN_BOX_PX:
                continue
            o = pts3[8]
            # 물체 축을 카메라 좌표로: 축 끝점을 투영해서 뺌 -> 회전행렬의 열
            cols = []
            for a in (ax, ay, az):
                c3, _ = project(view, tuple(center[i] + a[i] for i in range(3)))
                cols.append([c3[i] - o[i] for i in range(3)])
            R = [[cols[j][i] for j in range(3)] for i in range(3)]
            objects.append({
                "class": cls, "name": path.rsplit("/", 1)[-1], "visibility": round(float(vis), 4), "lying": lying,
                "tags_side": side_tags[0] if side_tags else None,   # 사각 퍽: 태그 남은 옆면 수 (0~4)
                "tags_end": side_tags[1] if side_tags else None,    # 사각 퍽: 태그 남은 끝면 수 (0~2)
                "location": [round(v * 100, 4) for v in o],                  # cm
                "quaternion_xyzw": [round(v, 6) for v in rot_to_quat_xyzw(R)],
                "projected_cuboid": [[round(u, 2), round(v, 2)] for u, v in pts2],
                "bounding_box": {"top_left": [round(float(b2[0]), 1), round(float(b2[1]), 1)],
                                 "bottom_right": [round(float(b2[2]), 1), round(float(b2[3]), 1)]},
            })
            counts["objects"][cls] += 1

        sp = "val" if shelf == VAL_SHELF else "train"
        name = f"{f:06d}"
        bgr = np.ascontiguousarray(img[..., ::-1])
        cv2.imwrite(str(out / sp / f"{name}.jpg"), bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
        ann = {"camera_data": {"width": W, "height": H,
                               "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy},
                               "location_worldframe": [round(v * 100, 3) for v in eye]},
               "objects": objects}
        (out / sp / f"{name}.json").write_text(json.dumps(ann, indent=1))
        if args.debug and f < 200:
            dbg = bgr.copy()
            edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
            for ob in objects:
                p = [tuple(int(round(v)) for v in q) for q in ob["projected_cuboid"]]
                for i, j in edges:
                    cv2.line(dbg, p[i], p[j], (255, 255, 0) if i < 4 and j < 4 else (0, 200, 255), 1, cv2.LINE_AA)
                for i, q in enumerate(p):
                    cv2.putText(dbg, str(i), q, cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255) if i == 8 else (255, 255, 255), 1)
            cv2.imwrite(str(out / "debug" / f"{sp}_{name}.jpg"), dbg)
        counts[sp] += 1
        counts["empty"] += not objects
        f += 1
        if f % 100 == 0 or f == args.frames:
            print(f"{f}/{args.frames}  train {counts['train']} val {counts['val']} empty {counts['empty']} "
                  f"objects {counts['objects']}", flush=True)
    app.close()


if __name__ == "__main__":
    main()
