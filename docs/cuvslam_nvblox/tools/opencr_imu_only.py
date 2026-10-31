#!/usr/bin/env python3
"""Publishes the wheel OpenCR's IMU on /imu without the ros2_control stack.

For accelerometer calibration with the OpenCR off the robot: xsting_base_hardware refuses
to start without the wheel motors, so this node reads the same control table items
(gyro x/y/z at 60, accel x/y/z at 72, float32) and publishes sensor_msgs/Imu at 50 Hz,
matching the firmware's 20 ms IMU update.

  python3 opencr_imu_only.py           # publish /imu
  python3 opencr_imu_only.py --test    # print a few readings and exit
"""
import glob
import struct
import sys

import rclpy
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
from rclpy.node import Node
from sensor_msgs.msg import Imu

OPENCR_ID = 201          # wheel OpenCR; the arm OpenCR is 200
BAUD = 1000000
ADDR_GYRO_X = 60         # 3 gyro floats, then 3 accel floats at 72
READ_LEN = 24
RATE_HZ = 50.0


def open_opencr():
    ports = sorted(glob.glob('/dev/serial/by-id/*OpenCR*')) or ['/dev/ttyACM0']
    port = PortHandler(ports[0])
    if not (port.openPort() and port.setBaudRate(BAUD)):
        sys.exit(f'Could not open {ports[0]} at {BAUD} baud')
    return port, PacketHandler(2.0)


def read_imu(port, packet):
    data, result, error = packet.readTxRx(port, OPENCR_ID, ADDR_GYRO_X, READ_LEN)
    if result != COMM_SUCCESS or error != 0:
        return None
    values = struct.unpack('<6f', bytes(data))
    return values[:3], values[3:]


class OpenCRImu(Node):
    def __init__(self, port, packet):
        super().__init__('opencr_imu_only')
        self.port, self.packet = port, packet
        self.pub = self.create_publisher(Imu, '/imu', 10)
        self.failures = 0
        self.create_timer(1.0 / RATE_HZ, self.tick)
        self.get_logger().info(f'Publishing /imu from OpenCR ID {OPENCR_ID} at {RATE_HZ:.0f} Hz')

    def tick(self):
        reading = read_imu(self.port, self.packet)
        if reading is None:
            self.failures += 1
            if self.failures % 50 == 1:
                self.get_logger().warn(f'IMU read failed ({self.failures} so far)')
            return
        gyro, accel = reading
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation_covariance[0] = -1.0  # orientation not provided
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel
        self.pub.publish(msg)


def main():
    port, packet = open_opencr()
    if '--test' in sys.argv:
        for _ in range(5):
            reading = read_imu(port, packet)
            if reading is None:
                print('read failed')
            else:
                gyro, accel = reading
                norm = sum(a * a for a in accel) ** 0.5
                print('gyro', [round(g, 4) for g in gyro], 'accel', [round(a, 3) for a in accel],
                      f'|a|={norm:.3f}')
        port.closePort()
        return

    rclpy.init()
    node = OpenCRImu(port, packet)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    port.closePort()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
