"""RealSense D435 colour camera shared by the capture, HSV and labelling tools.

The D435 colour stream is used as delivered (nearly distortion-free from the factory): no lens
undistortion is applied anywhere, so training images are the RAW frames exactly as captured.
"""
from pathlib import Path

import numpy as np


class D435Camera:
    """RealSense D435 colour stream with the cv2.VideoCapture read()/release() interface.
    Auto exposure and white balance stay on (driver defaults); pyrealsense2 is in the .venv."""

    def __init__(self, width=1280, height=720, fps=30):
        try:
            import pyrealsense2 as rs
        except ImportError:
            raise SystemExit('pyrealsense2 없음: .venv의 python으로 실행 (예: ../.venv/bin/python capture.py ...)')
        if not rs.context().query_devices():
            raise SystemExit('RealSense가 연결되어 있지 않음 (lsusb에 8086:0b07)')
        self.pipe, cfg = rs.pipeline(), rs.config()
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        try:
            prof = self.pipe.start(cfg)
        except RuntimeError as e:
            raise SystemExit(f'D435를 열 수 없음: {e} (다른 프로그램이 쓰고 있는지, 지원 해상도인지 확인)')
        i = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.size = (i.width, i.height)
        self.K = np.array([[i.fx, 0, i.ppx], [0, i.fy, i.ppy], [0, 0, 1]])
        self.serial = prof.get_device().get_info(rs.camera_info.serial_number)

    def read(self):
        f = self.pipe.wait_for_frames().get_color_frame()
        return (True, np.asanyarray(f.get_data()).copy()) if f else (False, None)

    def release(self):
        self.pipe.stop()


def describe_d435(size, K=None):
    """one line for info.txt / source.txt, e.g. 'd435 colour 1280x720, no undistortion, fx .. fy .. cx .. cy ..'
    (the intrinsics part only when the camera matrix K is known)"""
    text = f'd435 colour {size[0]}x{size[1]}, no undistortion'
    if K is not None:
        text += f', fx {K[0, 0]:.1f} fy {K[1, 1]:.1f} cx {K[0, 2]:.1f} cy {K[1, 2]:.1f}'
    return text


def check_d435_session(session):
    """stop unless the capture session's info.txt says 'camera: d435' (the tools only handle D435 photos)"""
    info = Path(session) / 'info.txt'
    camera = None
    if info.exists():
        for line in info.read_text(encoding='utf-8').splitlines():
            key, _, value = line.partition(':')
            if key.strip() == 'camera':
                camera = value.strip()
    if camera != 'd435':
        raise SystemExit(f'{session}: D435로 찍은 세션이 아님 (info.txt camera: {camera or "없음"}). '
                         f'도구들은 D435 사진만 처리함 (capture.py로 다시 찍기)')
