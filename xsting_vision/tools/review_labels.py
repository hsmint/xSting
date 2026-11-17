"""Check auto labels, fix object classes, and reject bad photos.

usage: python3 tools/review_labels.py data/labeled/<session>
keys:  n / space  next (also confirms the classes of this photo)     p  previous
       Tab        select the next object          c  cycle its class among shapes of the same colour
       1-4        set its class directly (1 green_cylinder, 2 green_prism, 3 red_cylinder, 4 red_prism)
       d          delete the selected outline (e.g. a non-target green thing)
       x          reject / un-reject the whole photo          q/Esc  quit (saves)
Labels are rewritten as you go. Photos in needs_class.txt (both shapes of one colour) leave the
list once you move on from them with n. Rejected photos go to rejected.txt.
"""
import argparse
from pathlib import Path

import cv2
import numpy as np

from capture import CLASSES

DRAW = {'green': (0, 255, 0), 'red': (0, 0, 255)}


def read_labels(path):
    out = []
    for line in path.read_text().splitlines():
        v = line.split()
        out.append([int(v[0]), np.array(v[1:], float).reshape(-1, 2)])
    return out


def write_labels(path, labels):
    path.write_text(''.join(f'{c} ' + ' '.join(f'{x:.5f} {y:.5f}' for x, y in poly) + '\n' for c, poly in labels))


def main():
    ap = argparse.ArgumentParser(description='review and fix auto labels')
    ap.add_argument('session', type=Path)
    a = ap.parse_args()
    s = a.session
    photos = sorted((s / 'images').glob('*.jpg'))
    if not photos:
        raise SystemExit(f'{s}/images에 사진이 없음 (auto_label.py 먼저)')
    lists = {n: set((s / f'{n}.txt').read_text().split()) if (s / f'{n}.txt').exists() else set()
             for n in ('rejected', 'needs_class')}
    i, sel, win = 0, 0, f'review - {s.name}'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 720)
    while True:
        p = photos[i]
        lab_path = s / 'labels' / f'{p.stem}.txt'
        labels = read_labels(lab_path)
        sel = min(sel, max(len(labels) - 1, 0))
        img = cv2.imread(str(p))
        h, w = img.shape[:2]
        for k, (c, poly) in enumerate(labels):
            pts = (poly * [w, h]).astype(np.int32).reshape(-1, 1, 2)
            col = DRAW[CLASSES[c].split('_')[0]]
            cv2.polylines(img, [pts], True, (255, 255, 255) if k == sel else col, 4 if k == sel else 2)
            cv2.putText(img, f'{k + 1}:{CLASSES[c]}', tuple(int(v) for v in pts.reshape(-1, 2).min(0) - [0, 8]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
        bad, todo = p.name in lists['rejected'], p.name in lists['needs_class']
        state = 'REJECTED' if bad else ('CHECK CLASSES (Tab/c/1-4, then n)' if todo else 'ok')
        cv2.putText(img, f'{i + 1}/{len(photos)} {p.name}  {state}   objects {len(labels)}   '
                         f'to check {len(lists["needs_class"])}  rejected {len(lists["rejected"])}',
                    (15, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255) if bad or todo else (0, 255, 0), 2)
        if bad:
            cv2.rectangle(img, (0, 0), (w - 1, h - 1), (0, 0, 255), 12)
        cv2.imshow(win, img)
        k = cv2.waitKey(0) & 0xFF
        if k in (ord('q'), 27):
            break
        if k in (ord('n'), ord(' ')):
            lists['needs_class'].discard(p.name)
            i, sel = min(i + 1, len(photos) - 1), 0
        elif k == ord('p'):
            i, sel = max(i - 1, 0), 0
        elif k == 9 and labels:   # Tab
            sel = (sel + 1) % len(labels)
        elif k == ord('c') and labels:
            color = CLASSES[labels[sel][0]].split('_')[0]
            same = [j for j, n in enumerate(CLASSES) if n.startswith(color)]
            labels[sel][0] = same[(same.index(labels[sel][0]) + 1) % len(same)]
            write_labels(lab_path, labels)
        elif ord('1') <= k <= ord('4') and labels:
            labels[sel][0] = k - ord('1')
            write_labels(lab_path, labels)
        elif k == ord('d') and labels:
            del labels[sel]
            write_labels(lab_path, labels)
        elif k == ord('x'):
            lists['rejected'] ^= {p.name}
    for n, v in lists.items():
        (s / f'{n}.txt').write_text('\n'.join(sorted(v)) + ('\n' if v else ''))
    cv2.destroyAllWindows()
    print(f'제외 {len(lists["rejected"])}장, 클래스 확인 남음 {len(lists["needs_class"])}장')


if __name__ == '__main__':
    main()
