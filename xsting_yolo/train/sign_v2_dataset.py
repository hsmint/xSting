"""신호등 데이터셋 v2: X-AnyLabeling .json -> YOLO 데이터셋

  python sign_v2_dataset.py --mode all --out sign_all_v2
  python sign_v2_dataset.py --mode new --out sign_new_v2
  python sign_v2_dataset.py --src /path/to/captures_sign/color --mode all --out sign_all_v2

- 박스가 없는 사진, .json 이 없는 사진은 넣지 않음
- 클래스: 0 sign_tower, 1 sign_tower_R, 2 sign_tower_Y, 3 sign_tower_G (sign_only_v1 과 같음)
- 분할은 영상 단위 (같은 영상의 비슷한 프레임이 train/test 에 섞이지 않게)
    이전 데이터: val sign_cycle3, sign_tower_lightness2 / test sign_cycle4, sign_tower_lightness3 (sign_only_v1 과 같음)
    신규 데이터: val sign_two_boxes_light2 / test sign_two_boxes_light4, sign_two_boxes_light5
  -> 두 모델 모두 신규 데이터의 같은 val/test 로 비교할 수 있음
- 같은 파일시스템이면 하드링크를 생성하고, 다른 파일시스템이면 복사함
"""
import argparse
import collections
import glob
import json
import os
import shutil

BASE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SRC = os.path.join(BASE, "captures_sign", "color")
CLASSES = ["sign_tower", "sign_tower_R", "sign_tower_Y", "sign_tower_G"]
NEW = "sign_two_boxes"  # 새로 찍은 영상 이름 앞부분
VAL = {"sign_cycle3", "sign_tower_lightness2", "sign_two_boxes_light2"}
TEST = {"sign_cycle4", "sign_tower_lightness3", "sign_two_boxes_light4", "sign_two_boxes_light5"}


def link_or_copy(src: str, dst: str) -> None:
    try:
        os.link(src, dst)
    except OSError:
        shutil.copy2(src, dst)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=DEFAULT_SRC, help="X-AnyLabeling 이미지/JSON 폴더 경로")
    ap.add_argument("--mode", choices=["all", "new"], required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    src_dir = a.src if os.path.isabs(a.src) else os.path.abspath(os.path.join(BASE, a.src))
    out = a.out if os.path.isabs(a.out) else os.path.abspath(os.path.join(BASE, a.out))
    if not os.path.isdir(src_dir):
        raise FileNotFoundError(f"원본 라벨 폴더를 찾을 수 없습니다: {src_dir}")

    shutil.rmtree(out, ignore_errors=True)
    splits = ("train", "val", "test")
    for sp in splits:
        os.makedirs(os.path.join(out, "images", sp))
        os.makedirs(os.path.join(out, "labels", sp))

    n_img = {sp: collections.Counter() for sp in splits}
    n_box = {sp: collections.Counter() for sp in splits}
    skipped = 0
    for jpath in sorted(glob.glob(os.path.join(src_dir, "*.json"))):
        name = os.path.basename(jpath)[:-5]
        vid = name.rsplit("_f", 1)[0]
        if a.mode == "new" and not vid.startswith(NEW):
            continue
        with open(jpath, encoding="utf-8") as jf:
            d = json.load(jf)
        W, H = d["imageWidth"], d["imageHeight"]
        lines = []
        for s in d["shapes"]:
            if s["shape_type"] != "rectangle" or s["label"] not in CLASSES:
                continue
            xs = [p[0] for p in s["points"]]
            ys = [p[1] for p in s["points"]]
            x1, x2 = max(0, min(xs)), min(W, max(xs))
            y1, y2 = max(0, min(ys)), min(H, max(ys))
            if x2 - x1 < 1 or y2 - y1 < 1:
                continue
            c = CLASSES.index(s["label"])
            lines.append(
                f"{c} {(x1 + x2) / 2 / W:.6f} {(y1 + y2) / 2 / H:.6f} "
                f"{(x2 - x1) / W:.6f} {(y2 - y1) / H:.6f}"
            )
        if not lines:
            skipped += 1
            continue
        sp = "val" if vid in VAL else "test" if vid in TEST else "train"
        with open(os.path.join(out, "labels", sp, name + ".txt"), "w", encoding="utf-8") as lf:
            lf.write("\n".join(lines) + "\n")
        link_or_copy(
            os.path.join(src_dir, name + ".jpg"),
            os.path.join(out, "images", sp, name + ".jpg"),
        )
        group = "new" if vid.startswith(NEW) else "old"
        n_img[sp][group] += 1
        for line in lines:
            n_box[sp][CLASSES[int(line.split()[0])]] += 1

    with open(os.path.join(out, "data.yaml"), "w", encoding="utf-8") as f:
        f.write(
            f"train: {out}/images/train\n"
            f"val: {out}/images/val\n"
            f"test: {out}/images/test\n"
            f"nc: {len(CLASSES)}\n"
            f"names: {CLASSES}\n"
        )
    print(f"박스 없는 사진 제외: {skipped}")
    for sp in splits:
        print(sp, dict(n_img[sp]), "boxes", {c: n_box[sp][c] for c in CLASSES})


if __name__ == "__main__":
    main()
