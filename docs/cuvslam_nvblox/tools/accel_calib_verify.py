#!/usr/bin/env python3
"""Checks accelerometer calibration files on all 6 faces.

Turn the board through the 6 faces (any order) and leave it still for ~3 s on each.
Every still pose is recorded under the axis pointing up; once all 6 are in (or on Ctrl+C)
it prints |a| after each calibration, which should be 9.807 on every face.
imu_calib's model: corrected = SM * raw - bias, applied to the raw /imu.

  python3 accel_calib_verify.py ~/calibration/opencr_accel.yaml ~/calibration/opencr_accel_v2.yaml
"""
import collections
import math
import os
import sys

import numpy as np
import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu

G = 9.80665
STILL_SAMPLES = 100     # 2 s at 50 Hz
STILL_GYRO = 0.02       # rad/s spread
STILL_ACC_STD = 0.05    # m/s^2
MAX_TILT_DEG = 10.0
LABELS = ['X+', 'X-', 'Y+', 'Y-', 'Z+', 'Z-']


def load(path):
    with open(os.path.expanduser(path)) as f:
        data = yaml.safe_load(f)
    return np.array(data['SM'], dtype=float).reshape(3, 3), np.array(data['bias'], dtype=float)


class Verify(Node):
    def __init__(self, calibs):
        super().__init__('accel_calib_verify')
        self.calibs = calibs
        self.acc = collections.deque(maxlen=STILL_SAMPLES)
        self.gyro = collections.deque(maxlen=STILL_SAMPLES)
        self.poses = {}
        self.create_subscription(Imu, '/imu', self.on_imu, 50)
        self.create_timer(0.5, self.status)

    def on_imu(self, msg):
        a, g = msg.linear_acceleration, msg.angular_velocity
        self.acc.append((a.x, a.y, a.z))
        self.gyro.append((g.x, g.y, g.z))
        if len(self.acc) < STILL_SAMPLES:
            return
        acc, gyro = np.array(self.acc), np.array(self.gyro)
        if (np.linalg.norm(gyro - gyro.mean(0), axis=1).max() > STILL_GYRO
                or np.linalg.norm(acc, axis=1).std() > STILL_ACC_STD):
            return
        mean = acc.mean(0)
        axis = int(np.argmax(np.abs(mean)))
        tilt = math.degrees(math.acos(min(1.0, abs(mean[axis]) / np.linalg.norm(mean))))
        if tilt <= MAX_TILT_DEG:
            self.poses['XYZ'[axis] + ('+' if mean[axis] > 0 else '-')] = (mean, tilt)

    def status(self):
        done = ' '.join(l if l in self.poses else '__' for l in LABELS)
        print(f'\rrecorded: {done}   (hold each face still ~3 s)   ', end='', flush=True)
        if len(self.poses) == len(LABELS):
            raise SystemExit

    def report(self):
        print('\n')
        names = list(self.calibs)
        print('face  tilt   raw|a|  ' + '  '.join(f'{n:>16s}' for n in names))
        for label in LABELS:
            if label not in self.poses:
                print(f'{label:4s}  (not recorded)')
                continue
            raw, tilt = self.poses[label]
            cells = []
            for name in names:
                sm, bias = self.calibs[name]
                norm = np.linalg.norm(sm @ raw - bias)
                cells.append(f'{norm:6.3f} ({100 * (norm / G - 1):+5.2f}%)')
            print(f'{label:4s}  {tilt:4.1f}  {np.linalg.norm(raw):7.3f}  ' + '  '.join(f'{c:>16s}' for c in cells))
        print(f'\ntarget |a| = {G}. Within about 0.3% on every face is good.')


def main():
    paths = sys.argv[1:] or ['~/calibration/opencr_accel.yaml', '~/calibration/opencr_accel_v2.yaml']
    calibs = {os.path.basename(p).replace('.yaml', ''): load(p) for p in paths}
    rclpy.init()
    node = Verify(calibs)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    node.report()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
