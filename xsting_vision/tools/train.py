"""Train the YOLO segmentation model on data/dataset (build_dataset.py) -> models/<name>/.

usage: .venv/bin/python tools/train.py [--model yolo11n-seg.pt | yolo26n-seg.pt] [--epochs 100] [--imgsz 640] [--name pilot]
       [--onnx]   (also export weights/best.onnx for the Jetson; build the TensorRT engine ON the Jetson)
Starts from COCO-pretrained weights (transfer learning). Colour augmentation is kept small so red
and green never swap; flips are left/right only (the camera never sees the objects upside down).
"""
import argparse
from pathlib import Path

from ultralytics import YOLO

PKG = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser(description='train YOLO-seg on the xsting objects')
    ap.add_argument('--model', default='yolo11n-seg.pt', help='pretrained weights (downloaded on first use)')
    ap.add_argument('--data', type=Path, default=PKG / 'data' / 'dataset' / 'data.yaml')
    ap.add_argument('--epochs', type=int, default=100)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--batch', type=int, default=16)
    ap.add_argument('--name', default='pilot')
    ap.add_argument('--onnx', action='store_true')
    a = ap.parse_args()
    weights = Path(a.model)
    if not weights.exists() and weights.parent == Path('.'):   # bare name: download into models/pretrained
        weights = PKG / 'models' / 'pretrained' / weights.name
        weights.parent.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(weights))
    model.train(data=str(a.data), epochs=a.epochs, imgsz=a.imgsz, batch=a.batch,
                project=str(PKG / 'models'), name=a.name, exist_ok=False,
                hsv_h=0.005, hsv_s=0.5, hsv_v=0.4,   # small hue shift: red/green must not swap
                fliplr=0.5, flipud=0.0, degrees=0.0, patience=30, plots=True)
    best = Path(model.trainer.best)
    metrics = YOLO(best).val(data=str(a.data), split='val', plots=False)
    print(f'best: {best}\nmask mAP50 {metrics.seg.map50:.3f}  mAP50-95 {metrics.seg.map:.3f}   '
          f'box mAP50 {metrics.box.map50:.3f}')
    if a.onnx:
        print('onnx:', YOLO(best).export(format='onnx', imgsz=a.imgsz, simplify=True))


if __name__ == '__main__':
    main()
