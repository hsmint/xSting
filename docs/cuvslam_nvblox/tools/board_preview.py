#!/usr/bin/env python3
"""Live check before recording: shows both D435 IR images with the 9x6 checkerboard
corners drawn, so you can see whether Kalibr will be able to use each frame.

Green border = board found in that image, red = not found.
Bottom line: detection rate over the last 3 seconds, and the mean image brightness
(dots from the IR projector or a dark scene show up here first).
Press q in the window to quit.
"""
import collections
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

PATTERN = (9, 6)  # inner corners of the 10x7-square board
TOPICS = {
    'left': '/camera/camera/infra1/image_rect_raw',
    'right': '/camera/camera/infra2/image_rect_raw',
}
FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK


class BoardPreview(Node):
    def __init__(self):
        super().__init__('board_preview')
        self.images = {}
        self.history = {name: collections.deque() for name in TOPICS}
        for name, topic in TOPICS.items():
            self.create_subscription(
                Image, topic, lambda msg, n=name: self.on_image(n, msg), qos_profile_sensor_data)
        self.get_logger().info('Waiting for ' + ', '.join(TOPICS.values()))

    def on_image(self, name, msg):
        if msg.encoding not in ('mono8', '8UC1'):
            self.get_logger().warn(f'{name}: unexpected encoding {msg.encoding}', once=True)
            return
        self.images[name] = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width]

    def render(self):
        now = time.monotonic()
        panels = []
        for name in TOPICS:
            img = self.images.get(name)
            if img is None:
                panels.append(np.zeros((480, 640, 3), np.uint8))
                continue
            found, corners = cv2.findChessboardCorners(img, PATTERN, flags=FLAGS)
            view = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(view, PATTERN, corners, found)
            cv2.rectangle(view, (0, 0), (view.shape[1] - 1, view.shape[0] - 1),
                          (0, 200, 0) if found else (0, 0, 255), 6)

            hist = self.history[name]
            hist.append((now, found))
            while hist and now - hist[0][0] > 3.0:
                hist.popleft()
            rate = 100.0 * sum(f for _, f in hist) / len(hist)
            cv2.putText(view, f'{name}: {"FOUND" if found else "no board"}  last 3s {rate:3.0f}%  '
                              f'brightness {img.mean():3.0f}',
                        (10, view.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            panels.append(view)
        cv2.imshow('board preview (q to quit)', np.hstack(panels))
        return (cv2.waitKey(1) & 0xFF) != ord('q')


def main():
    rclpy.init()
    node = BoardPreview()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            if not node.render():
                break
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
