#!/usr/bin/env python3
"""Shows which IMU axis is pointing up, for the 6-orientation accelerometer calibration.

At rest the accelerometer reads about +9.8 on the axis that points up, which is how
imu_calib's do_calib_node labels the poses (X+ up = ax near +9.8, X- up = ax near -9.8).
Also shows how far the board is tilted from that axis and whether it is still.

  python3 imu_axis_check.py            # reads /imu
  python3 imu_axis_check.py --ros-args -r /imu:=/other_topic
"""
import collections
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu

WINDOW = 25            # samples averaged (0.5 s at 50 Hz)
TILT_OK_DEG = 3.0
STILL_GYRO = 0.02      # rad/s, spread around the mean (raw gyro has a bias)
STILL_ACC_STD = 0.05   # m/s^2

GREEN, YELLOW, RED, RESET = '\033[32m', '\033[33m', '\033[31m', '\033[0m'


class AxisCheck(Node):
    def __init__(self):
        super().__init__('imu_axis_check')
        self.acc = collections.deque(maxlen=WINDOW)
        self.gyro = collections.deque(maxlen=WINDOW)
        self.create_subscription(Imu, '/imu', self.on_imu, 50)
        self.create_timer(0.2, self.show)
        print('do_calib order: X+ -> X- -> Y+ -> Y- -> Z+ -> Z-   (Ctrl+C to quit)\n')

    def on_imu(self, msg):
        a, g = msg.linear_acceleration, msg.angular_velocity
        self.acc.append((a.x, a.y, a.z))
        self.gyro.append((g.x, g.y, g.z))

    def show(self):
        if len(self.acc) < WINDOW:
            print('\rwaiting for /imu ...', end='', flush=True)
            return
        mean = [sum(s[i] for s in self.acc) / WINDOW for i in range(3)]
        norm = math.sqrt(sum(m * m for m in mean))
        axis = max(range(3), key=lambda i: abs(mean[i]))
        label = 'XYZ'[axis] + ('+' if mean[axis] > 0 else '-')
        tilt = math.degrees(math.acos(min(1.0, abs(mean[axis]) / norm)))

        norms = [math.sqrt(sum(v * v for v in s)) for s in self.acc]
        mn = sum(norms) / WINDOW
        acc_std = math.sqrt(sum((n - mn) ** 2 for n in norms) / WINDOW)
        gmean = [sum(s[i] for s in self.gyro) / WINDOW for i in range(3)]
        gyro_spread = max(math.dist(s, gmean) for s in self.gyro)
        still = gyro_spread < STILL_GYRO and acc_std < STILL_ACC_STD

        tilt_color = GREEN if tilt <= TILT_OK_DEG else (YELLOW if tilt <= 8 else RED)
        state = f'{GREEN}still{RESET}' if still else f'{RED}moving{RESET}'
        print(f'\rUP: {tilt_color}{label}{RESET}  tilt {tilt_color}{tilt:4.1f} deg{RESET}  '
              f'[{state}]  ax {mean[0]:+6.2f}  ay {mean[1]:+6.2f}  az {mean[2]:+6.2f}  '
              f'|a| {norm:5.2f}   ', end='', flush=True)


def main():
    rclpy.init()
    node = AxisCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    print()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
