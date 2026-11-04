"""D435 컬러 카메라로 학습용 사진을 찍어 data/raw/<날짜_시간>_<장면>/ 에 저장한다.

파일은 **원본**으로 저장한다. D435 컬러는 보정(왜곡 펴기) 없이 그대로 쓴다.
카메라와 해상도·내부 파라미터는 info.txt에 남는다.

실행 예 (pyrealsense2가 있는 .venv의 python으로):
    ../.venv/bin/python capture.py --scene p_both --objects green_cylinder,green_prism
    ../.venv/bin/python capture.py --scene p_none --objects none --width 848 --height 480

--objects: 이 장면에 놓은 물체 클래스 (쉼표로 구분). 한 종류만 놓은 장면은 자동 라벨이 클래스를
           장면에서 바로 정한다. 물체가 없는 장면은 --objects none.

키:
    space / s  한 장 저장
    a          자동 촬영 켜기/끄기 (--interval 초마다 저장)
    q / Esc    종료
"""
import argparse
import time
from datetime import datetime
from pathlib import Path

import cv2

from camera import D435Camera, describe_d435

RAW_DIR = Path(__file__).resolve().parents[1] / 'data' / 'raw'
CLASSES = ['green_cylinder', 'green_prism', 'red_cylinder', 'red_prism']


def parse_args():
    p = argparse.ArgumentParser(description='D435 학습 사진 수집')
    p.add_argument('--scene', required=True, help='장면 이름 (예: p_red_prism_desk, mix_basic, no_object)')
    p.add_argument('--objects', required=True, help=f'놓은 물체: {",".join(CLASSES)} 중 쉼표 구분, 또는 none')
    p.add_argument('--width', type=int, default=1280, help='기본 1280 (D435: 640x480, 848x480, 1280x720, 1920x1080 등)')
    p.add_argument('--height', type=int, default=720, help='기본 720')
    p.add_argument('--interval', type=float, default=1.0, help='자동 촬영 간격(초)')
    a = p.parse_args()
    a.objects = [] if a.objects == 'none' else [o.strip() for o in a.objects.split(',')]
    bad = [o for o in a.objects if o not in CLASSES]
    if bad:
        p.error(f'모르는 클래스 {bad}: {CLASSES} 또는 none')
    return a


def make_session_dir(args, cam):
    """data/raw/<날짜_시간>_<장면>/ 폴더를 만들고 촬영 조건을 info.txt에 남긴다."""
    session = RAW_DIR / f'{datetime.now():%Y%m%d_%H%M%S}_{args.scene}'
    session.mkdir(parents=True, exist_ok=False)
    (session / 'info.txt').write_text(
        f'scene: {args.scene}\n'
        f'objects: {",".join(args.objects) or "none"}\n'
        f'started: {datetime.now():%Y-%m-%d %H:%M:%S}\n'
        f'camera: d435\n'
        f'resolution: {cam.size[0]}x{cam.size[1]}, saved RAW (not undistorted)\n'
        f'calibration: {describe_d435(cam.size, cam.K)}\n',
        encoding='utf-8')
    return session


def draw_overlay(view, count, auto, interval, flash):
    if flash:  # 저장 직후 테두리를 잠깐 표시
        cv2.rectangle(view, (0, 0), (view.shape[1] - 1, view.shape[0] - 1), (255, 255, 255), 12)
    status = f"saved {count}   auto {'ON' if auto else 'OFF'} ({interval:g}s)"
    cv2.putText(view, status, (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255) if auto else (0, 255, 0), 2)
    cv2.putText(view, 'space/s: save   a: auto   q: quit', (15, view.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return view


def main():
    args = parse_args()
    cap = D435Camera(args.width, args.height)
    session = make_session_dir(args, cap)
    print(f'저장 폴더: {session}\n카메라: d435, {describe_d435(cap.size, cap.K)}')

    window = f'capture - d435 - {args.scene}'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, *cap.size)

    count, auto, last_auto, flash_until = 0, False, 0.0, 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            print('프레임을 읽지 못함')
            break
        now = time.time()

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        if key == ord('a'):
            auto, last_auto = not auto, now
            print(f"자동 촬영 {'켜짐' if auto else '꺼짐'}")

        if key in (ord(' '), ord('s')) or (auto and now - last_auto >= args.interval):
            count += 1
            path = session / f'{args.scene}_{count:04d}.jpg'
            cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])   # RAW frame
            print(f'저장: {path.name}')
            last_auto, flash_until = now, now + 0.15

        view = frame.copy()
        cv2.imshow(window, draw_overlay(view, count, auto, args.interval, now < flash_until))

    cap.release()
    cv2.destroyAllWindows()
    with open(session / 'info.txt', 'a', encoding='utf-8') as f:
        f.write(f"ended: {datetime.now():%Y-%m-%d %H:%M:%S}\nimages: {count}\n")
    if count == 0:  # 한 장도 안 찍었으면 빈 폴더를 남기지 않음
        (session / 'info.txt').unlink()
        session.rmdir()
        print('저장한 사진이 없어 폴더를 지웠습니다.')
    else:
        print(f'{count}장 저장: {session}')


if __name__ == '__main__':
    main()
