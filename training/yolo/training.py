"""
sign_tower 4-state detector: COCO pretrained yolo11n.pt fine-tuning.

Classes (same names/order as captures_sign/classes.txt):
    0 sign_tower     (no light)
    1 sign_tower_R   (red)
    2 sign_tower_Y   (yellow)
    3 sign_tower_G   (green)

Input data (either one):

  A) X-AnyLabeling output folder: image + same-name .json in one folder
     captures_sign/color/
       sign_tower1_f00480.jpg, sign_tower1_f00480.json, ...
     -> JSON is converted to YOLO txt automatically
     -> file name "<video>_f<frame>" is used to split train/val per video

  B) Ready-made YOLO dataset with data.yaml
     -> pass --yaml path/to/data.yaml

Usage:
  python training.py --src /home/pa10/xsting_ws/vision/captures_sign/color
  python training.py --yaml ./my_dataset/data.yaml
"""
import argparse
import json
import shutil
from collections import defaultdict
from pathlib import Path

import cv2
import torch
from ultralytics import YOLO

CLASS_NAMES = ["sign_tower", "sign_tower_R", "sign_tower_Y", "sign_tower_G"]
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def json_to_yolo(json_path: Path, img_w: int, img_h: int) -> tuple[list[str], list[str]]:
    """X-AnyLabeling rectangle (pixel corners) -> YOLO lines 'cls cx cy w h' (0~1)."""
    data = json.loads(json_path.read_text())
    lines, unknown = [], []
    for shape in data.get("shapes", []):
        label = shape.get("label")
        if label not in CLASS_NAMES:
            unknown.append(label)
            continue
        xs = [min(max(p[0], 0), img_w) for p in shape["points"]]
        ys = [min(max(p[1], 0), img_h) for p in shape["points"]]
        x1, x2, y1, y2 = min(xs), max(xs), min(ys), max(ys)
        if x2 - x1 < 1 or y2 - y1 < 1:
            continue
        cx, cy = (x1 + x2) / 2 / img_w, (y1 + y2) / 2 / img_h
        w, h = (x2 - x1) / img_w, (y2 - y1) / img_h
        lines.append(f"{CLASS_NAMES.index(label)} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
    return lines, unknown


def video_and_frame(img: Path) -> tuple[str, int]:
    """'sign_tower1_f00480' -> ('sign_tower1', 480). Other names -> (stem, 0)."""
    video, sep, frame = img.stem.rpartition("_f")
    if sep and frame.isdigit():
        return video, int(frame)
    return img.stem, 0


def build_dataset(src: Path, out: Path, val_ratio: float, size: tuple[int, int]) -> Path:
    """Convert src/*.jpg + *.json into YOLO train/val layout, resized to size=(w, h)."""
    imgs = sorted(p for p in src.iterdir() if p.suffix.lower() in IMG_EXTS)
    if not imgs:
        raise RuntimeError(f"no images in {src}")

    # Split per video: the last N% of frames of each video -> val, so that
    # near-identical consecutive frames don't leak between train and val.
    videos = defaultdict(list)
    for img in imgs:
        video, frame = video_and_frame(img)
        videos[video].append((frame, img))
    split_of = {}
    for frames in videos.values():
        frames.sort()
        n_val = max(1, int(len(frames) * val_ratio))
        for i, (_, img) in enumerate(frames):
            split_of[img] = "val" if i >= len(frames) - n_val else "train"

    if out.exists():
        shutil.rmtree(out)
    for split in ("train", "val"):
        (out / split / "images").mkdir(parents=True)
        (out / split / "labels").mkdir(parents=True)

    counts = {s: [0] * len(CLASS_NAMES) for s in ("train", "val")}
    background = {"train": 0, "val": 0}
    no_json, unknown_labels = [], set()

    for img_path in imgs:
        json_path = img_path.with_suffix(".json")
        if not json_path.exists():
            no_json.append(img_path.name)        # not labeled yet -> skip
            continue
        img = cv2.imread(str(img_path))
        if img is None:
            raise RuntimeError(f"cannot read image: {img_path}")

        # Labels are normalized to the original size, so resizing keeps them valid.
        lines, unknown = json_to_yolo(json_path, img.shape[1], img.shape[0])
        unknown_labels.update(unknown)
        if (img.shape[1], img.shape[0]) != size:
            img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)

        split = split_of[img_path]
        cv2.imwrite(str(out / split / "images" / f"{img_path.stem}.jpg"), img)
        (out / split / "labels" / f"{img_path.stem}.txt").write_text("\n".join(lines))
        if lines:
            for line in lines:
                counts[split][int(line.split()[0])] += 1
        else:
            background[split] += 1               # empty label = background image

    print(f"\nvideos: {', '.join(f'{v}({len(f)})' for v, f in sorted(videos.items()))}")
    print(f"{'class':<16}{'train':>7}{'val':>7}")
    for cid, name in enumerate(CLASS_NAMES):
        print(f"{name:<16}{counts['train'][cid]:>7}{counts['val'][cid]:>7}")
    print(f"{'(background)':<16}{background['train']:>7}{background['val']:>7}")
    if no_json:
        print(f"skipped {len(no_json)} images without json (e.g. {no_json[0]})")
    if unknown_labels:
        print(f"WARNING: ignored unknown labels: {sorted(unknown_labels)}")

    yaml_path = out / "data.yaml"
    yaml_path.write_text(
        f"path: {out.resolve()}\n"
        "train: train/images\n"
        "val: val/images\n"
        f"names: {CLASS_NAMES}\n"
    )
    return yaml_path


def main():
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--src", type=Path, help="folder with images + X-AnyLabeling .json")
    src.add_argument("--yaml", type=Path, help="existing YOLO data.yaml")
    ap.add_argument("--out", type=Path, default=Path("sign_tower_dataset"))
    ap.add_argument("--val-ratio", type=float, default=0.2)
    ap.add_argument("--model", default="yolo11n.pt", help="COCO pretrained weights")
    ap.add_argument("--epochs", type=int, default=150)
    ap.add_argument("--resize", type=int, nargs=2, default=[640, 480], metavar=("W", "H"),
                    help="resize dataset images to W H (only with --src)")
    ap.add_argument("--imgsz", type=int, default=640, help="training size (long side)")
    ap.add_argument("--batch", type=int, default=16, help="-1 = auto by GPU memory")
    ap.add_argument("--device", default="0", help="GPU index (0 = first GPU)")
    ap.add_argument("--name", default="sign_tower")
    ap.add_argument("--prepare-only", action="store_true",
                    help="only build the dataset, don't train")
    args = ap.parse_args()

    if args.src:
        data_yaml = build_dataset(args.src, args.out, args.val_ratio, tuple(args.resize))
    else:
        data_yaml = args.yaml
    print(f"\ndata: {data_yaml}")
    if args.prepare_only:
        return

    if not torch.cuda.is_available():
        raise SystemExit("CUDA GPU not available. Check the NVIDIA driver / CUDA build of torch.")
    print(f"GPU: {torch.cuda.get_device_name(int(args.device))}")

    # COCO pretrained backbone is reused; the 80-class head is replaced by 4 classes.
    model = YOLO(args.model)
    model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        name=args.name,
        patience=30,          # early stop if val mAP doesn't improve
        # Color augmentation: the state is defined by color/brightness, keep them intact.
        hsv_h=0.0,            # no hue shift (red <-> yellow confusion)
        hsv_s=0.3,
        hsv_v=0.15,           # small brightness jitter (keeps on/off difference)
        # Geometry augmentation: the dataset already has upright, upside-down and
        # lying (both directions) poses, and flips don't change which color is lit.
        fliplr=0.5,           # 좌우 반전
        flipud=0.5,           # 상하 반전
        degrees=5.0,
        scale=0.5,
        mosaic=1.0,
        close_mosaic=10,
        plots=True,
    )

    # Evaluate best weights on the val set
    best = Path(model.trainer.save_dir) / "weights" / "best.pt"
    metrics = YOLO(best).val(data=str(data_yaml), imgsz=args.imgsz, plots=True)
    print(f"\nbest weights: {best}")
    print(f"mAP50: {metrics.box.map50:.3f}   mAP50-95: {metrics.box.map:.3f}")
    for i, cid in enumerate(metrics.box.ap_class_index):
        p, r, ap50, _ = metrics.box.class_result(i)
        print(f"  {CLASS_NAMES[cid]:<16} P={p:.3f} R={r:.3f} mAP50={ap50:.3f}")
    print(f"confusion matrix: {metrics.save_dir}/confusion_matrix.png")


if __name__ == "__main__":
    main()
