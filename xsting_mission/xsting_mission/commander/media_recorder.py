"""미션 수행 중 이동·작업 전후 카메라 스냅샷(JPG) 및 연속 미션 영상(AVI) 자동 기록기."""
import json
import os
import re
import threading
import time

from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - OpenCV 미설치 환경에서도 스냅샷(Compressed) 동작 보장
    cv2 = None
    np = None


def _safe_token(text):
    cleaned = re.sub(r"[^A-Za-z0-9_-]+", "_", str(text or "none")).strip("_")
    return cleaned or "none"


class MediaRecorder:
    """전면 D435 및 팔 끝 ArduCam 영상을 구독해 전후 이미지와 연속 영상을 기록한다."""

    def __init__(self, node, run_dir, front_topic, arm_topic, video_fps=4.0):
        self._node = node
        self._media_dir = os.path.join(run_dir, "media")
        self._img_dir = os.path.join(self._media_dir, "images")
        os.makedirs(self._img_dir, exist_ok=True)

        self._video_fps = max(1.0, float(video_fps))
        self._min_video_dt = 1.0 / self._video_fps
        self._lock = threading.Lock()
        self._closed = False
        self._latest = {}       # cam_key -> {"jpeg": bytes|None, "raw": Image|None, "t": float}
        self._writers = {}      # cam_key -> {"writer": cv2.VideoWriter, "path": str, "frames": int, "last_t": float, "size": (w, h)}
        self._snapshots = []
        self._subs = []

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for cam_key, topic in (("front", front_topic), ("arm", arm_topic)):
            if not topic:
                continue
            self._subscribe_camera(cam_key, topic.strip(), qos)

    def _subscribe_camera(self, cam_key, topic, qos):
        if topic.endswith("/compressed"):
            self._subs.append(
                self._node.create_subscription(
                    CompressedImage, topic,
                    lambda msg, k=cam_key: self._on_compressed(k, msg), qos))
            raw_topic = topic[:-len("/compressed")]
            if raw_topic:
                self._subs.append(
                    self._node.create_subscription(
                        Image, raw_topic,
                        lambda msg, k=cam_key: self._on_raw_backup(k, msg), qos))
        else:
            self._subs.append(
                self._node.create_subscription(
                    Image, topic,
                    lambda msg, k=cam_key: self._on_raw(k, msg), qos))

    def _on_compressed(self, cam_key, msg):
        now = time.monotonic()
        data = bytes(msg.data)
        if not data:
            return
        with self._lock:
            if self._closed:
                return
            self._latest[cam_key] = {"jpeg": data, "raw": None, "t": now}
            self._maybe_write_video_locked(cam_key, now, jpeg_bytes=data)

    def _on_raw_backup(self, cam_key, msg):
        now = time.monotonic()
        with self._lock:
            if self._closed:
                return
            prev = self._latest.get(cam_key)
            if prev is not None and prev.get("jpeg") and (now - prev["t"]) < 1.0:
                return
            self._latest[cam_key] = {"jpeg": None, "raw": msg, "t": now}
            self._maybe_write_video_locked(cam_key, now, raw_msg=msg)

    def _on_raw(self, cam_key, msg):
        now = time.monotonic()
        with self._lock:
            if self._closed:
                return
            self._latest[cam_key] = {"jpeg": None, "raw": msg, "t": now}
            self._maybe_write_video_locked(cam_key, now, raw_msg=msg)

    @staticmethod
    def _decode_bgr(jpeg_bytes=None, raw_msg=None):
        if cv2 is None or np is None:
            return None
        if jpeg_bytes is not None:
            arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if raw_msg is None or not raw_msg.width or not raw_msg.height:
            return None
        enc = (raw_msg.encoding or "").lower()
        buf = np.frombuffer(raw_msg.data, dtype=np.uint8)
        if enc in ("rgb8", "bgr8"):
            expected = raw_msg.height * raw_msg.step
            if buf.size < expected:
                return None
            img = buf[:expected].reshape((raw_msg.height, raw_msg.step // 3, 3))
            img = img[:, :raw_msg.width, :]
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == "rgb8" else img.copy()
        if enc in ("mono8", "8uc1"):
            expected = raw_msg.height * raw_msg.step
            if buf.size < expected:
                return None
            gray = buf[:expected].reshape((raw_msg.height, raw_msg.step))[:, :raw_msg.width]
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        return None

    def _maybe_write_video_locked(self, cam_key, now, jpeg_bytes=None, raw_msg=None):
        if cv2 is None:
            return
        w_info = self._writers.get(cam_key)
        if w_info is not None and (now - w_info["last_t"]) < self._min_video_dt:
            return
        bgr = self._decode_bgr(jpeg_bytes=jpeg_bytes, raw_msg=raw_msg)
        if bgr is None or bgr.ndim != 3:
            return
        h, w = int(bgr.shape[0]), int(bgr.shape[1])
        if w <= 0 or h <= 0:
            return
        if w_info is None:
            rel_path = os.path.join("media", f"mission_{cam_key}.avi")
            abs_path = os.path.join(os.path.dirname(self._media_dir), rel_path)
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            writer = cv2.VideoWriter(abs_path, fourcc, self._video_fps, (w, h))
            if not writer.isOpened():
                return
            w_info = {
                "writer": writer,
                "rel_path": rel_path,
                "frames": 0,
                "last_t": 0.0,
                "size": (w, h),
            }
            self._writers[cam_key] = w_info
        elif w_info["size"] != (w, h):
            bgr = cv2.resize(bgr, w_info["size"])

        w_info["writer"].write(bgr)
        w_info["frames"] += 1
        w_info["last_t"] = now

    def capture_snapshot(self, turn, stage, action_name, args, robot_state):
        """행동 실행 전(before)·후(after) 최신 프레임을 JPG로 저장하고 경로 목록을 반환한다."""
        target = (args or {}).get("t") or (robot_state or {}).get("here") or "unknown"
        prefix = (f"turn_{int(turn):03d}_{_safe_token(stage)}_"
                  f"{_safe_token(action_name)}_{_safe_token(target)}")
        saved_rel = []
        now_wall = time.time()

        with self._lock:
            if self._closed:
                return saved_rel
            items = list(self._latest.items())

        for cam_key, frame in items:
            rel_path = os.path.join("media", "images", f"{prefix}_{cam_key}.jpg")
            abs_path = os.path.join(os.path.dirname(self._media_dir), rel_path)
            wrote = False
            if frame.get("jpeg"):
                try:
                    with open(abs_path, "wb") as handle:
                        handle.write(frame["jpeg"])
                    wrote = True
                except OSError:
                    wrote = False
            elif frame.get("raw") is not None and cv2 is not None:
                bgr = self._decode_bgr(raw_msg=frame["raw"])
                if bgr is not None:
                    wrote = bool(cv2.imwrite(abs_path, bgr))
            if wrote:
                saved_rel.append(rel_path)

        if saved_rel:
            entry = {
                "turn": int(turn),
                "stage": stage,
                "action": action_name,
                "target": target,
                "t": now_wall,
                "here": (robot_state or {}).get("here"),
                "pose": (robot_state or {}).get("pose"),
                "files": saved_rel,
            }
            with self._lock:
                self._snapshots.append(entry)
                self._write_index_locked()
        return saved_rel

    def _write_index_locked(self):
        videos = {
            cam: {"file": info["rel_path"], "frames": info["frames"],
                  "fps": self._video_fps}
            for cam, info in self._writers.items()
        }
        payload = {
            "videos": videos,
            "snapshots": self._snapshots,
        }
        idx_path = os.path.join(self._media_dir, "media_index.json")
        tmp_path = idx_path + ".tmp"
        try:
            with open(tmp_path, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, ensure_ascii=False, indent=2)
            os.replace(tmp_path, idx_path)
        except OSError:
            pass

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            for info in self._writers.values():
                try:
                    info["writer"].release()
                except Exception:
                    pass
            self._write_index_locked()
