#!/usr/bin/env python3
"""Small, slow copy of the left IR stream for viewing over Wi-Fi (the full stream is ~9 MB/s).

Every Nth frame of /camera/camera/infra1/image_rect_raw is halved to 320x240 and republished on
/camera/camera/infra1_preview/image_raw with a matching camera_info (K and P scaled), in the same
optical frame, so an RViz Camera display can draw cuVSLAM's points and path over the image.
Runs on the robot.

  python3 ir_preview.py            # 5 fps
  python3 ir_preview.py 10         # 10 fps
"""
import sys

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

SRC = '/camera/camera/infra1'
DST = '/camera/camera/infra1_preview'
CAMERA_FPS = 30
SCALE = 0.5


class IrPreview(Node):
    def __init__(self, fps):
        super().__init__('ir_preview')
        self.every = max(1, round(CAMERA_FPS / fps))
        self.count = 0
        self.info = None
        self.img_pub = self.create_publisher(Image, DST + '/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, DST + '/camera_info', qos_profile_sensor_data)
        self.create_subscription(CameraInfo, SRC + '/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, SRC + '/image_rect_raw', self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f'{SRC} -> {DST}, every {self.every}th frame, x{SCALE}')

    def on_info(self, msg):
        self.info = msg

    def on_image(self, msg):
        self.count += 1
        if self.count % self.every or self.info is None or msg.encoding not in ('mono8', '8UC1'):
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
        small = cv2.resize(img, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_AREA)

        out = Image()
        out.header = msg.header
        out.height, out.width = small.shape
        out.encoding = 'mono8'
        out.step = out.width
        out.data = small.tobytes()

        info = CameraInfo()
        info.header = msg.header
        info.width, info.height = out.width, out.height
        info.distortion_model = self.info.distortion_model
        info.d = list(self.info.d)
        info.r = list(self.info.r)
        k = list(self.info.k)
        for i in (0, 2, 4, 5):
            k[i] *= SCALE
        info.k = k
        p = list(self.info.p)
        for i in (0, 2, 3, 5, 6):
            p[i] *= SCALE
        info.p = p

        self.img_pub.publish(out)
        self.info_pub.publish(info)


def main():
    fps = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    rclpy.init()
    node = IrPreview(fps)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
