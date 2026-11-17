"""Auto-label D435 capture sessions with colour masks -> images + YOLO-seg polygon labels.

usage: python3 tools/auto_label.py data/raw/<session> [more sessions ...] [--overwrite]
Sessions must be D435 sessions (info.txt 'camera: d435'); the photos are used as they are (no undistortion).
For each session (info.txt 'objects:' from capture.py):
  - objects none                       -> every photo gets an empty label (negative sample)
  - each colour appears in one class   -> every green / red blob gets that colour's class
    (e.g. 'red_prism' alone, or 'green_cylinder,red_prism')
  - both shapes of one colour present  -> outlines are made, the class is a placeholder (the first
    shape of that colour) and the photo is listed in needs_class.txt: pick the class per object
    in review_labels.py (Tab / c / 1-4). build_dataset.py skips photos still in needs_class.txt
Writes data/labeled/<session>/{images,labels,review}/ ; review/ has the photos with the labels
drawn so they can be checked with review_labels.py before build_dataset.py uses them.
Colour ranges: config/object_hsv.yaml (tools/hsv_tune.py).
"""
import argparse
import shutil
from collections import Counter
from pathlib import Path

import cv2

from camera import check_d435_session, describe_d435
from capture import CLASSES
from hsv_tune import color_mask, load

LABELED = Path(__file__).resolve().parents[1] / 'data' / 'labeled'
DRAW = {'green': (0, 255, 0), 'red': (0, 0, 255)}


def read_info(session):
    info = {}
    for line in (session / 'info.txt').read_text(encoding='utf-8').splitlines():
        k, _, v = line.partition(':')
        info[k.strip()] = v.strip()
    return info


def read_objects(session):
    v = read_info(session).get('objects')
    if v is None:
        raise SystemExit(f'{session}/info.txt에 objects 항목이 없음 (capture.py --objects로 찍은 세션이어야 함)')
    return [] if v == 'none' else v.split(',')


def polygons(mask, min_area):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    out = []
    for c in cnts:
        if cv2.contourArea(c) < min_area:
            continue
        c = cv2.approxPolyDP(c, 0.003 * cv2.arcLength(c, True), True)   # keeps the outline within ~1 px
        if len(c) >= 3:
            out.append(c.reshape(-1, 2))
    return out


def label_session(session, cfg, overwrite):
    objects = read_objects(session)
    photos = sorted(session.glob('*.jpg'))
    if not photos:
        print(f'{session.name}: 사진 없음')
        return None
    check_d435_session(session)
    size = cv2.imread(str(photos[0])).shape[1::-1]   # (width, height) of the first photo, for source.txt
    cls_of, ambiguous = {}, set()
    for o in objects:   # first class of each colour; a second shape of that colour makes it ambiguous
        color = o.split('_')[0]
        if color in cls_of:
            ambiguous.add(color)
        else:
            cls_of[color] = CLASSES.index(o)
    out = LABELED / session.name
    if out.exists():
        if not overwrite:
            print(f'{session.name}: 이미 있음 (--overwrite로 다시 만들기)')
            return None
        shutil.rmtree(out)
    for d in ('images', 'labels', 'review'):
        (out / d).mkdir(parents=True)
    counts, pending = Counter(), []
    for p in photos:
        img = cv2.imread(str(p))
        h, w = img.shape[:2]
        lines, view, n = [], img.copy(), 0
        for color, cls in cls_of.items():
            for poly in polygons(color_mask(img, cfg, color), cfg['min_area_px']):
                lines.append(f'{cls} ' + ' '.join(f'{x / w:.5f} {y / h:.5f}' for x, y in poly))
                cv2.polylines(view, [poly.reshape(-1, 1, 2)], True, DRAW[color], 2)
                cv2.putText(view, CLASSES[cls], tuple(int(v) for v in poly.min(0)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, DRAW[color], 2)
                n += 1
        counts[n] += 1
        if n and any(c in ambiguous for c in cls_of):
            pending.append(p.name)
        cv2.imwrite(str(out / 'images' / p.name), img, [cv2.IMWRITE_JPEG_QUALITY, 95])
        (out / 'labels' / f'{p.stem}.txt').write_text('\n'.join(lines) + ('\n' if lines else ''))
        cv2.putText(view, f'{p.name}: {n} objects', (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        cv2.imwrite(str(out / 'review' / p.name), cv2.resize(view, (w // 2, h // 2)))
    if pending:
        (out / 'needs_class.txt').write_text('\n'.join(pending) + '\n')
    (out / 'source.txt').write_text(f'raw: {session}\nobjects: {",".join(objects) or "none"}\n'
                                    f'undistort: {describe_d435(size)}\nhsv: {cfg}\n', encoding='utf-8')
    print(f'{session.name}: {len(photos)}장, 물체 수별 사진 수 {dict(sorted(counts.items()))}  → {out}'
          + (f'\n  클래스 지정 필요 {len(pending)}장 ({"/".join(sorted(ambiguous))}의 모양 구분): review_labels.py' if pending else ''))
    return out


def main():
    ap = argparse.ArgumentParser(description='colour-mask auto labels (YOLO-seg)')
    ap.add_argument('sessions', nargs='+', type=Path)
    ap.add_argument('--overwrite', action='store_true')
    a = ap.parse_args()
    cfg = load()
    for s in a.sessions:
        label_session(s, cfg, a.overwrite)


if __name__ == '__main__':
    main()
