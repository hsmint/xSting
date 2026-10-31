#!/usr/bin/env python3
"""Prints cuVSLAM's pose and tracking state on one line, on the robot itself (no images over Wi-Fi).

pos: base frame position in the cuVSLAM odom frame (start = 0). With imu:=false the base frame is camera_link,
whose axes look ~28 deg down, so driving straight moves x and z together; with imu:=true it is base_footprint
(x forward, y left, z up). dist: straight-line distance from the first pose
(use this to compare with a tape measure). yaw in degrees, path: summed trajectory length (includes wobble), vo_state: 1 means tracking (other values = lost / not started),
lost: how many status messages were not 1 since start, track: per-frame tracking time.

  python3 cuvslam_monitor.py
"""
import math

import rclpy
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class Monitor(Node):
    def __init__(self):
        super().__init__('cuvslam_monitor')
        self.pose = None
        self.start = None
        self.last = None
        self.path = 0.0
        self.state = None
        self.lost = 0
        self.track_ms = 0.0
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(VisualSlamStatus, '/visual_slam/status', self.on_status, 10)
        self.create_timer(0.5, self.show)

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if self.last is not None:
            self.path += math.dist((p.x, p.y, p.z), self.last)
        self.last = (p.x, p.y, p.z)
        if self.start is None:
            self.start = self.last
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        self.pose = (p.x, p.y, p.z, yaw)

    def on_status(self, msg):
        self.state = msg.vo_state
        if msg.vo_state != 1:
            self.lost += 1
        self.track_ms = msg.track_execution_time * 1000.0

    def show(self):
        if self.pose is None:
            print(f'\rwaiting for cuVSLAM ... vo_state={self.state}   ', end='', flush=True)
            return
        x, y, z, yaw = self.pose
        dist = math.dist((x, y, z), self.start)
        print(f'\rpos x {x:+7.3f}  y {y:+7.3f}  z {z:+7.3f} m  dist {dist:5.3f} m  yaw {yaw:+7.1f} deg  path {self.path:6.2f} m  '
              f'vo_state {self.state}  lost {self.lost}  track {self.track_ms:4.1f} ms   ', end='', flush=True)


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    print()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
