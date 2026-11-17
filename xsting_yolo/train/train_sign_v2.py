"""신호등 v2 학습 (YOLOv8 nano): 데이터셋 하나를 학습하고 val/test 점수를 요약

  ../.venv/bin/python train_sign_v2.py --data sign_all_v2/data.yaml --name sign_all_v2_yolov8n
  ../.venv/bin/python train_sign_v2.py --data sign_new_v2/data.yaml --name sign_new_v2_yolov8n

결과: runs/detect/<name>/  (weights/best.pt, results.png, confusion_matrix.png ...)
요약: runs/detect/<name>_summary.txt
설정: imgsz 640, batch 16, patience 0(끝까지), seed 0
"""
import argparse
import os

from ultralytics import YOLO

BASE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(BASE, "runs", "detect")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--model", default="yolov8n.pt")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--imgsz", type=int, default=640)
    a = ap.parse_args()
    data = os.path.join(BASE, a.data) if not os.path.isabs(a.data) else a.data

    YOLO(a.model).train(
        data=data,
        epochs=a.epochs,
        imgsz=a.imgsz,
        batch=a.batch,
        patience=0,
        seed=0,
        project=RUNS,
        name=a.name,
        exist_ok=True,
    )

    best = os.path.join(RUNS, a.name, "weights", "best.pt")
    rows = [f"{'split':5} {'mAP50':>6} {'mAP50-95':>8} {'P':>6} {'R':>6}"]
    for sp in ("val", "test"):
        r = YOLO(best).val(
            data=data,
            split=sp,
            imgsz=a.imgsz,
            batch=a.batch,
            plots=False,
            verbose=False,
            project=RUNS,
            name=f"{a.name}_{sp}",
            exist_ok=True,
        )
        rows.append(f"{sp:5} {r.box.map50:6.3f} {r.box.map:8.3f} {r.box.mp:6.3f} {r.box.mr:6.3f}")
        rows += [
            f"   {r.names[int(c)]:14} mAP50 {r.box.ap50[j]:.3f}  mAP50-95 {r.box.ap[j]:.3f}"
            for j, c in enumerate(r.box.ap_class_index)
        ]
    with open(os.path.join(RUNS, f"{a.name}_summary.txt"), "w", encoding="utf-8") as sf:
        sf.write("\n".join(rows) + "\n")
    print("\n".join(rows))


if __name__ == "__main__":
    main()
