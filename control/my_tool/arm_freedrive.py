#!/usr/bin/env python3
"""Turn the arm torque off, move the joints by hand, and record each joint's range.

Run on the robot with ros2_control stopped (this script talks to the arm OpenCR directly):
  source /opt/ros/lyrical/setup.bash && python3 arm_freedrive.py

Steps:
  1. Hold the arm, press Enter: torque turns off for every arm motor (joints and gripper at once;
     the firmware has no per-joint torque switch). The arm hangs under the deck, so it drops.
  2. Move each joint by hand to its limits. The screen shows the present angle and the min/max seen.
  3. Ctrl+C: prints the ranges in degrees and as URDF <limit> values, and appends them to ~/arm_ranges.txt.
  4. Hold the arm in the pose to keep, press Enter: goals are set to the present pose, then torque on.
     Ctrl+C here instead leaves the torque off.
"""
import glob
import math
import os
import subprocess
import sys
import time

import dynamixel_sdk.port_handler as port_handler_module
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

ARM_ID = 200
BAUD = 1000000

# opencr_control_table.hpp (xsting_manipulation_hardware)
ADDR_CONNECT_MANIP = 16
ADDR_HEARTBEAT = 19
ADDR_TORQUE_JOINTS = 59
ADDR_GOAL_POSITION_JOINT_1 = 60      # joint1..6, 4 bytes each
ADDR_GOAL_POSITION_GRIPPER = 88
ADDR_GOAL_POSITION_WR_JOINT = 92
ADDR_GOAL_POSITION_WR_GRIPPER = 93
ADDR_PRESENT_POSITION_JOINT_1 = 95   # joint1..6 at 95..115, gripper at 123
PRESENT_BLOCK_LENGTH = 123 + 4 - 95

NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
GRIPPER_RAD_TO_METER = -0.015        # opencr_definitions.hpp grippers::RAD_TO_METER


def tick_to_rad(tick):
    # Same as convert_tick_to_radian() in opencr.cpp.
    zero = (4095 + 0) // 2
    if tick > zero:
        return (tick - zero) * math.pi / (4095 - zero)
    if tick < zero:
        return (tick - zero) * -math.pi / (0 - zero)
    return 0.0


def to_signed(value):
    return value - 2**32 if value >= 2**31 else value


class ArmOpenCR:
    def __init__(self):
        self.packet = PacketHandler(2.0)
        self.port = None
        self.heartbeat = 0

    def connect(self):
        # The firmware loop can be slow when a motor stops answering, so allow long replies.
        port_handler_module.LATENCY_TIMER = 100
        for path in sorted(glob.glob("/dev/ttyACM*")):
            port = PortHandler(path)
            if not port.openPort() or not port.setBaudRate(BAUD):
                continue
            for _ in range(3):
                port.clearPort()
                if self.packet.ping(port, ARM_ID)[1] == COMM_SUCCESS:
                    self.port = port
                    return path
            port.closePort()
        return None

    def read(self, address, length):
        for _ in range(3):
            self.port.clearPort()
            data, result, error = self.packet.readTxRx(self.port, ARM_ID, address, length)
            if result == COMM_SUCCESS and not error:
                return data
        return None

    def write(self, address, data):
        for _ in range(3):
            self.port.clearPort()
            result, error = self.packet.writeTxRx(self.port, ARM_ID, address, len(data), data)
            if result == COMM_SUCCESS and not error:
                return True
        return False

    def present_ticks(self):
        data = self.read(ADDR_PRESENT_POSITION_JOINT_1, PRESENT_BLOCK_LENGTH)
        if data is None:
            return None
        offsets = [0, 4, 8, 12, 16, 20, 28]
        return [to_signed(int.from_bytes(bytes(data[o:o + 4]), "little")) for o in offsets]

    def beat(self):
        # The firmware drops goal writes unless the heartbeat keeps changing.
        self.heartbeat = (self.heartbeat + 1) % 256
        self.write(ADDR_HEARTBEAT, [self.heartbeat])

    def torque(self, on):
        return self.write(ADDR_TORQUE_JOINTS, [1 if on else 0])

    def hold_present_pose(self, ticks):
        """Set every goal to `ticks`, so turning torque on does not move toward an old goal."""
        for _ in range(3):
            self.beat()
            time.sleep(0.05)
        joints = b"".join(t.to_bytes(4, "little", signed=True) for t in ticks[:6])
        ok = self.write(ADDR_GOAL_POSITION_JOINT_1, list(joints))
        ok &= self.write(ADDR_GOAL_POSITION_WR_JOINT, [1])
        ok &= self.write(ADDR_GOAL_POSITION_GRIPPER, list(ticks[6].to_bytes(4, "little", signed=True)))
        ok &= self.write(ADDR_GOAL_POSITION_WR_GRIPPER, [1])
        self.beat()
        return ok


def value(name, tick):
    """Joint angle in degrees, or gripper opening in mm."""
    if name == "gripper":
        return f"{tick_to_rad(tick) * GRIPPER_RAD_TO_METER * 1000:+7.2f} mm"
    return f"{math.degrees(tick_to_rad(tick)):+7.1f} deg"


def describe(name, tick):
    return f"tick {tick:5d}  {value(name, tick)}  {tick_to_rad(tick):+.4f} rad"


RESULT_FILE = os.path.expanduser("~/arm_ranges.txt")


def print_ranges(lows, highs):
    lines = [f"=== 측정한 범위 ({time.strftime('%Y-%m-%d %H:%M:%S')}) ==="]
    for i, name in enumerate(NAMES):
        if name == "gripper":
            a = tick_to_rad(lows[i]) * GRIPPER_RAD_TO_METER
            b = tick_to_rad(highs[i]) * GRIPPER_RAD_TO_METER
            lines.append(f"{name:8s} tick {lows[i]:5d} ~ {highs[i]:5d}   "
                         f'<limit lower="{min(a, b):.4f}" upper="{max(a, b):.4f}"/>  (m)')
            continue
        lo, hi = tick_to_rad(lows[i]), tick_to_rad(highs[i])
        lines.append(f"{name:8s} tick {lows[i]:5d} ~ {highs[i]:5d}   "
                     f"{math.degrees(lo):+7.1f} ~ {math.degrees(hi):+7.1f} deg   "
                     f'<limit lower="{lo:.4f}" upper="{hi:.4f}"/>')
    print("\n" + "\n".join(lines))
    # Appended, so earlier runs stay in the file.
    with open(RESULT_FILE, "a") as f:
        f.write("\n".join(lines) + "\n\n")
    print(f"(saved to {RESULT_FILE})")


def main():
    running = subprocess.run(["pgrep", "-f", "ros2_control_node"], capture_output=True).stdout
    if running.strip():
        sys.exit("ros2_control_node is running. Stop the launch first; both would drive the same OpenCR.")

    arm = ArmOpenCR()
    path = arm.connect()
    if path is None:
        sys.exit(f"No OpenCR ID {ARM_ID} answered on /dev/ttyACM*.")
    connected = arm.read(ADDR_CONNECT_MANIP, 1)
    if not connected or connected[0] != 1:
        sys.exit("The arm OpenCR does not see all 7 motors. Check the cables and reset the board.")

    ticks = arm.present_ticks()
    if ticks is None:
        sys.exit("Could not read the present pose.")
    print(f"arm OpenCR on {path}")
    for name, tick in zip(NAMES, ticks):
        print(f"  {name:8s} {describe(name, tick)}")

    print("\n토크를 끄면 7개 모터가 한꺼번에 풀리고, 팔이 자기 무게로 떨어져요.")
    try:
        input("팔을 손으로 받친 다음 Enter를 누르세요 (그만두려면 Ctrl+C) ")
    except (KeyboardInterrupt, EOFError):
        print("\n취소했어요. 토크는 그대로 켜져 있어요.")
        return
    if not arm.torque(False):
        sys.exit("Torque off write failed; torque is unchanged.")

    lows, highs = list(ticks), list(ticks)
    print("\n토크 OFF. 관절을 손으로 끝까지 움직여보세요. 다 했으면 Ctrl+C.\n")
    drawn = False
    try:
        while True:
            ticks = arm.present_ticks()
            if ticks is not None:
                lows = [min(a, b) for a, b in zip(lows, ticks)]
                highs = [max(a, b) for a, b in zip(highs, ticks)]
                if drawn:
                    sys.stdout.write(f"\x1b[{len(NAMES)}A")  # redraw the same lines in place
                for n, t, lo, hi in zip(NAMES, ticks, lows, highs):
                    sys.stdout.write(
                        f"  {n:8s} now {value(n, t)}   min {value(n, lo)}   max {value(n, hi)}\x1b[K\n")
                sys.stdout.flush()
                drawn = True
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    print_ranges(lows, highs)

    print("\n토크를 다시 켜면 그 순간의 자세에서 고정돼요.")
    try:
        input("팔을 유지할 자세로 잡고 Enter를 누르세요 (토크를 끈 채로 두려면 Ctrl+C) ")
    except KeyboardInterrupt:
        print("\n토크 OFF 상태로 종료합니다.")
        return

    ticks = arm.present_ticks()
    if ticks is None or not arm.hold_present_pose(ticks):
        print("Could not set the goals to the present pose; leaving torque off.")
        return
    if arm.torque(True):
        print("토크 ON. 현재 자세로 고정했어요.")
    else:
        print("Torque on write failed; torque is still off.")


if __name__ == "__main__":
    main()
