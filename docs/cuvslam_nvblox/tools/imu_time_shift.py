#!/usr/bin/env python3
"""Republishes the OpenCR IMU with its timestamps moved onto the camera clock.

Kalibr (motion6/7) gives timeshift_cam_imu = +12.4 ms (t_imu = t_cam + shift): the IMU stamp is
12.4 ms later than the moment it measured, as seen by the camera. So corrected = original - shift.
Apply this once only; cuVSLAM has no parameter for it.

  python3 imu_time_shift.py                       # /opencr/imu_corrected -> /opencr/imu_synced, 12.4 ms
  python3 imu_time_shift.py --ros-args -p shift_ms:=12.4
"""
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu


class ImuTimeShift(Node):
    def __init__(self):
        super().__init__('imu_time_shift')
        shift_ms = self.declare_parameter('shift_ms', 12.4).value
        self.shift = Duration(nanoseconds=int(shift_ms * 1e6))
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Imu, '/opencr/imu_synced', qos)
        self.create_subscription(Imu, '/opencr/imu_corrected', self.on_imu, qos)
        self.get_logger().info(f'/opencr/imu_corrected -> /opencr/imu_synced, stamp - {shift_ms} ms')

    def on_imu(self, msg):
        msg.header.stamp = (Time.from_msg(msg.header.stamp) - self.shift).to_msg()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImuTimeShift()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
