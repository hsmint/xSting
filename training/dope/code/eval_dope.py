"""DOPE 가중치를 합성 val 사진으로 평가 (위치 오차 cm, 검출률)

  python eval_dope.py --weights W1.pth [W2.pth ...] --data DIR1 [DIR2 ...] --objects red_square green_square \
                      --dims 3 3 6 [--max 300] [--raw] [--vis OUT_DIR]

- --raw: 학습 폴더에서 바로 나온 가중치(module. 접두어, 우리 꼭짓점 순서)를 변환해서 평가
         (변환된 가중치면 빼기)
- 정답 물체: --objects 클래스 중 visibility >= 0.5
- 맞춤 판정: 예측 중심점이 정답 중심점에서 원본 해상도로 20 px 이내
- 출력: 검출률(recall), 정밀도(precision), 위치 오차(cm, 중앙값/평균/90%), ADD-S(cm, 꼭짓점 기준 대칭 무관 오차)
"""
import argparse, glob, json, os, random, sys, io, contextlib
import numpy as np, cv2, torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "Deep_Object_Pose", "common"))
sys.path.insert(0, HERE)
from models import DopeNetwork
from detector import ObjectDetector
from cuboid import Cuboid3d
from cuboid_pnp_solver import CuboidPNPSolver
from convert_to_dope_order import convert

CFG = lambda: None
CFG.mask_edges = CFG.mask_faces = CFG.vertex = 1
CFG.threshold, CFG.softmax = 0.5, 1000
CFG.thresh_angle, CFG.thresh_map, CFG.sigma, CFG.thresh_points = 0.5, 0.01, 3, 0.1
DOWN_H = 720
PNP = {"auto": None, "sqpnp": cv2.SOLVEPNP_SQPNP, "epnp": cv2.SOLVEPNP_EPNP, "iterative": cv2.SOLVEPNP_ITERATIVE}
MATCH_PX = 20


def quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def corners(loc, q, dims):
    s = np.array([[a, b, c] for c in (1, -1) for a, b in ((1, 1), (-1, 1), (-1, -1), (1, -1))]) * np.array(dims) / 2
    return (quat_to_R(q) @ s.T).T + np.array(loc)


def adds(c1, c2):
    d = np.linalg.norm(c1[:, None] - c2[None], axis=2)
    return d.min(1).mean()


def detect(net, solver, img, cfg, min_pts=5, pnp_flag=None):
    """DOPE 검출. 원본 find_object_poses 는 꼭짓점 9개가 모두 있어야 결과를 내서 작은 퍽을 대부분 버림
    -> 꼭짓점이 min_pts 개 이상(중심 포함)이면 PnP 로 자세를 구함"""
    from detector import transform
    x = transform(img).unsqueeze(0).cuda()
    with torch.no_grad():
        b, af = net(x)
    with contextlib.redirect_stdout(io.StringIO()):
        objs, _ = ObjectDetector.find_objects(b[-1][0], af[-1][0], cfg)
    out = []
    for o in objs:
        pts = list(o[1]) + [(o[0][0] * 8, o[0][1] * 8)]
        if sum(p is not None for p in pts) < min_pts:
            continue
        n = sum(p is not None for p in pts)
        flag = pnp_flag if pnp_flag is not None else (cv2.SOLVEPNP_ITERATIVE if n >= 6 else cv2.SOLVEPNP_SQPNP)
        loc, q, proj = solver.solve_pnp(pts, pnp_algorithm=flag)
        if loc is None:
            continue
        out.append({"location": loc, "quaternion": q, "projected_points": proj,
                    "n_points": sum(p is not None for p in pts), "score": float(o[3])})
    return out


def load(path, raw):
    sd = torch.load(path, map_location="cpu")
    if raw or any(k.startswith("module.") for k in sd):
        sd = convert(sd) if raw else {k[7:]: v for k, v in sd.items()}
    net = DopeNetwork().cuda().eval()
    net.load_state_dict(sd)
    return net


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", nargs="+", required=True)
    ap.add_argument("--data", nargs="+", required=True)
    ap.add_argument("--objects", nargs="+", required=True)
    ap.add_argument("--dims", nargs=3, type=float, required=True)
    ap.add_argument("--max", type=int, default=300, help="폴더마다 평가할 사진 수")
    ap.add_argument("--raw", action="store_true")
    ap.add_argument("--height", type=int, default=720, help="추론 때 사진 높이(px). 원본 720 권장")
    ap.add_argument("--sigma", type=float, default=1, help="후처리 가우시안 sigma (DOPE 기본 3, 작은 퍽은 1 권장)")
    ap.add_argument("--thresh_angle", type=float, default=0.5)
    ap.add_argument("--thresh_points", type=float, default=0.1)
    ap.add_argument("--min_pts", type=int, default=5, help="자세를 구할 최소 점 수(꼭짓점+중심, 4~9)")
    ap.add_argument("--pnp", default="auto", choices=list(PNP), help="auto: 점 6개 이상 ITERATIVE, 미만 SQPNP")
    ap.add_argument("--debug", action="store_true", help="놓친 정답을 하나씩 출력")
    ap.add_argument("--vis", help="예측 그림 저장 폴더 (첫 가중치, 처음 40장)")
    a = ap.parse_args()
    global DOWN_H
    DOWN_H = a.height
    CFG.sigma, CFG.thresh_angle, CFG.thresh_points = a.sigma, a.thresh_angle, a.thresh_points

    files = []
    for d in a.data:
        js = sorted(glob.glob(os.path.join(d, "*.json")))
        random.Random(0).shuffle(js)
        files += js[:a.max]
    solver = CuboidPNPSolver("obj", cuboid3d=Cuboid3d(a.dims))
    rows = []
    for wi, w in enumerate(a.weights):
        net = load(w, a.raw)
        n_gt = n_tp = n_pred = 0
        loc_err, add_err = [], []
        for fi, jf in enumerate(files):
            lab = json.load(open(jf))
            img = cv2.imread(jf[:-5] + ".jpg")[..., ::-1].copy()
            H, W = img.shape[:2]
            sc = DOWN_H / H
            K = np.array([[lab["camera_data"]["intrinsics"][k] for k in r] for r in
                          (("fx", "cx"), ("fy", "cy"))])
            Km = np.array([[K[0, 0] * sc, 0, K[0, 1] * sc], [0, K[1, 0] * sc, K[1, 1] * sc], [0, 0, 1]])
            solver.set_camera_intrinsic_matrix(Km)
            solver.set_dist_coeffs(np.zeros((4, 1)))
            small = cv2.resize(img, (int(W * sc), DOWN_H))
            preds = detect(net, solver, small, CFG, a.min_pts, PNP[a.pnp])
            gts = [o for o in lab["objects"] if o["class"] in a.objects and o["visibility"] >= 0.5]
            n_gt += len(gts); n_pred += len(preds)
            used = set()
            for g in gts:
                gc = np.array(g["projected_cuboid"][8])
                best, bd = None, MATCH_PX
                for pi, p in enumerate(preds):
                    if pi in used or p["projected_points"][8] is None:
                        continue
                    d = np.linalg.norm(np.array(p["projected_points"][8]) / sc - gc)
                    if d < bd:
                        best, bd = pi, d
                if best is None:
                    if a.debug:
                        near = [round(float(np.linalg.norm(np.array(p["projected_points"][8]) / sc - gc))) for p in preds]
                        print(f"  MISS {os.path.basename(jf)} {g['class']} vis {g['visibility']:.2f} z {g['location'][2]:.0f}cm c {gc.round().tolist()} pred dists {near}")
                    continue
                used.add(best); n_tp += 1
                p = preds[best]
                loc_err.append(np.linalg.norm(np.array(p["location"]) - np.array(g["location"])))
                add_err.append(adds(corners(p["location"], p["quaternion"], a.dims),
                                    corners(g["location"], g["quaternion_xyzw"], a.dims)))
            if a.vis and wi == 0 and fi < 40:
                os.makedirs(a.vis, exist_ok=True)
                out = img[..., ::-1].copy()
                E = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
                for p in preds:
                    pts = p["projected_points"]
                    if any(q is None for q in pts):
                        continue
                    pts = [(int(q[0] / sc), int(q[1] / sc)) for q in pts]
                    for i, j in E:
                        cv2.line(out, pts[i], pts[j], (0, 255, 0) if i < 4 and j < 4 else (0, 200, 255), 2)
                    cv2.circle(out, pts[8], 4, (0, 0, 255), -1)
                    cv2.putText(out, f"z={p['location'][2]:.0f}cm", (pts[8][0] + 6, pts[8][1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.imwrite(os.path.join(a.vis, os.path.basename(os.path.dirname(os.path.dirname(jf)))
                                         + "_" + os.path.basename(jf)[:-5] + ".jpg"), out)
        le, ae = np.array(loc_err), np.array(add_err)
        row = (f"{os.path.basename(w):28} recall {n_tp / max(n_gt, 1):.3f} ({n_tp}/{n_gt})  precision {n_tp / max(n_pred, 1):.3f}  "
               f"loc cm med {np.median(le):.2f} mean {le.mean():.2f} p90 {np.percentile(le, 90):.2f}  "
               f"ADD-S cm med {np.median(ae):.2f} <1cm {np.mean(ae < 1):.3f}")
        print(row, flush=True); rows.append(row)
    return rows


if __name__ == "__main__":
    main()
