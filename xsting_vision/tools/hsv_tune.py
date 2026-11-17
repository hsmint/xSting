"""Tune the green / red HSV ranges used by auto_label.py on live D435 colour frames or D435 photos.

usage: ../.venv/bin/python hsv_tune.py [--images data/raw/<session>]   (default: live D435)
keys:  g / r   edit the green / red range       n / p   next / previous photo (with --images)
       s       save to config/object_hsv.yaml    q/Esc   quit
Red wraps around H = 0: its range is H in [0, h_hi] or [h_lo2, 179].
"""
import argparse
from pathlib import Path

import cv2
import numpy as np

from camera import D435Camera, check_d435_session

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'object_hsv.yaml'
DEFAULT = {   # starting points for saturated PLA; tune on real photos
    'green': dict(h_lo=40, h_hi=85, s_lo=80, v_lo=40),
    'red': dict(h_lo=170, h_hi=10, s_lo=90, v_lo=40),     # red: H >= h_lo or H <= h_hi
    'min_area_px': 300,
}


def load():
    cfg = {k: (dict(v) if isinstance(v, dict) else v) for k, v in DEFAULT.items()}
    if CONFIG.exists():
        fs = cv2.FileStorage(str(CONFIG), cv2.FILE_STORAGE_READ)
        for c in ('green', 'red'):
            for k in cfg[c]:
                n = fs.getNode(f'{c}_{k}')
                if not n.empty():
                    cfg[c][k] = int(n.real())
        n = fs.getNode('min_area_px')
        if not n.empty():
            cfg['min_area_px'] = int(n.real())
        fs.release()
    return cfg


def save(cfg):
    fs = cv2.FileStorage(str(CONFIG), cv2.FILE_STORAGE_WRITE)
    for c in ('green', 'red'):
        for k, v in cfg[c].items():
            fs.write(f'{c}_{k}', int(v))
    fs.write('min_area_px', int(cfg['min_area_px']))
    fs.release()


def color_mask(bgr, cfg, color):
    """binary mask of one colour; blurred and cleaned the same way as auto_label.py"""
    hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)
    c = cfg[color]
    if color == 'red':
        m = cv2.inRange(hsv, (c['h_lo'], c['s_lo'], c['v_lo']), (179, 255, 255)) | \
            cv2.inRange(hsv, (0, c['s_lo'], c['v_lo']), (c['h_hi'], 255, 255))
    else:
        m = cv2.inRange(hsv, (c['h_lo'], c['s_lo'], c['v_lo']), (c['h_hi'], 255, 255))
    k = np.ones((5, 5), np.uint8)
    return cv2.morphologyEx(cv2.morphologyEx(m, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)


def main():
    ap = argparse.ArgumentParser(description='HSV range tuner for the object colours')
    ap.add_argument('--images', type=Path, help='photo folder (RAW D435 photos from capture.py); default live D435')
    a = ap.parse_args()
    photos = sorted(a.images.glob('*.jpg')) if a.images else []
    if photos:
        check_d435_session(a.images)
        cap = None
    else:
        cap = D435Camera()
    cfg, color, idx = load(), 'green', 0
    win = 'hsv tune'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 900)

    cv2.namedWindow('ranges', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('ranges', 420, 220)

    def make_bars():
        for k, mx in (('h_lo', 179), ('h_hi', 179), ('s_lo', 255), ('v_lo', 255)):
            cv2.createTrackbar(k, 'ranges', cfg[color][k], mx, lambda _: None)
        cv2.createTrackbar('min_area', 'ranges', cfg['min_area_px'], 5000, lambda _: None)

    make_bars()
    while True:
        if photos:
            img = cv2.imread(str(photos[idx]))
        else:
            ok, img = cap.read()
            if not ok:
                break
        for k in ('h_lo', 'h_hi', 's_lo', 'v_lo'):
            cfg[color][k] = cv2.getTrackbarPos(k, 'ranges')
        cfg['min_area_px'] = cv2.getTrackbarPos('min_area', 'ranges')
        view = img.copy()
        for c, col in (('green', (0, 255, 0)), ('red', (0, 0, 255))):
            m = color_mask(img, cfg, c)
            cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = [x for x in cnts if cv2.contourArea(x) >= cfg['min_area_px']]
            cv2.drawContours(view, cnts, -1, col, 2)
        m = color_mask(img, cfg, color)
        small = cv2.resize(cv2.cvtColor(m, cv2.COLOR_GRAY2BGR), (320, 180))
        view[10:190, -330:-10] = small
        label = f'editing {color}' + (f'   photo {idx + 1}/{len(photos)} {photos[idx].name}' if photos else '')
        cv2.putText(view, label, (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        cv2.putText(view, 'g/r: colour  n/p: photo  s: save  q: quit', (15, view.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow(win, view)
        key = cv2.waitKey(30) & 0xFF
        if key in (ord('q'), 27):
            break
        if key in (ord('g'), ord('r')):
            color = 'green' if key == ord('g') else 'red'
            make_bars()
        if photos and key == ord('n'):
            idx = min(idx + 1, len(photos) - 1)
        if photos and key == ord('p'):
            idx = max(idx - 1, 0)
        if key == ord('s'):
            save(cfg)
            print(f'저장: {CONFIG}  {cfg}')
    if cap:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
