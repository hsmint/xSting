"""사진 폴더에 DOPE 사각기둥 퍽 모델 실행 -> 결과 그림 + json

  python infer_dope.py --weights ../model/square_puck_dope_e60.pth --images IMG_DIR --out OUT_DIR \
                       --fx 924.28 --fy 924.28 --cx 640 --cy 360

Deep_Object_Pose/common 이 필요함: --dope_common 경로 지정 (기본 ~/xsting_ws/dope/Deep_Object_Pose/common)
"""
import argparse, glob, json, os, sys
import cv2, numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--weights", required=True)
ap.add_argument("--images", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--fx", type=float, required=True); ap.add_argument("--fy", type=float, required=True)
ap.add_argument("--cx", type=float, required=True); ap.add_argument("--cy", type=float, required=True)
ap.add_argument("--min_pts", type=int, default=6)
ap.add_argument("--dope_common", default=os.path.expanduser("~/xsting_ws/dope/Deep_Object_Pose/common"))
a = ap.parse_args()
sys.argv = sys.argv[:1]
sys.path.insert(0, a.dope_common)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_dope as E
from cuboid import Cuboid3d
from cuboid_pnp_solver import CuboidPNPSolver

E.CFG.sigma = 1
net = E.load(a.weights, False)
solver = CuboidPNPSolver("square_puck", cuboid3d=Cuboid3d([3, 3, 6]))
solver.set_camera_intrinsic_matrix(np.array([[a.fx, 0, a.cx], [0, a.fy, a.cy], [0, 0, 1]]))
solver.set_dist_coeffs(np.zeros((4, 1)))
EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
os.makedirs(a.out, exist_ok=True)
for f in sorted(glob.glob(os.path.join(a.images, "*.jpg")) + glob.glob(os.path.join(a.images, "*.png"))):
    bgr = cv2.imread(f)
    preds = E.detect(net, solver, bgr[..., ::-1].copy(), E.CFG, a.min_pts)
    objs = []
    for p in preds:
        pts = [(int(q[0]), int(q[1])) for q in p["projected_points"]]
        for i, j in EDGES:
            cv2.line(bgr, pts[i], pts[j], (0, 255, 0), 2)
        cv2.putText(bgr, f"z={p['location'][2]:.1f}cm", (pts[8][0] + 6, pts[8][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        objs.append({"location_cm": [float(v) for v in p["location"]], "quaternion_xyzw": [float(v) for v in p["quaternion"]],
                     "n_points": p["n_points"], "score": p["score"]})
    name = os.path.splitext(os.path.basename(f))[0]
    cv2.imwrite(os.path.join(a.out, name + ".jpg"), bgr)
    json.dump({"objects": objs}, open(os.path.join(a.out, name + ".json"), "w"), indent=2)
    print(name, len(objs), "objects")
