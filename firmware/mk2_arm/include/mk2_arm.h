#pragma once

#include <stdint.h>

namespace mk2_arm {

#ifndef XSTING_AUTO_HOME
#define XSTING_AUTO_HOME 0
#endif
constexpr bool kAutoHome = (XSTING_AUTO_HOME != 0);
constexpr uint8_t kSlaveId = 202;
constexpr uint8_t kMotorCount = 6;
constexpr uint32_t kMotorBaud = 3000000;
constexpr uint32_t kUpdateMs = 5;

// Motor IDs: 11(J1), 12(J2), 13(J3), 14(J4), 15(roll), 16(gripper).
constexpr uint8_t kMotorIds[kMotorCount] = {11, 12, 13, 14, 15, 16};
constexpr int32_t kHomeTicks[kMotorCount] = {2048, 1363, 2746, 2048, 2048, 2048};
constexpr int16_t kGripperCurrent = 200;

// USB DYNAMIXEL slave control table addresses for 6 arm motors.
enum Addr : uint16_t {
  ADDR_MILLIS = 10,
  ADDR_CONNECT_MOTORS = 16,
  ADDR_DEVICE_STATUS = 18,
  ADDR_HEARTBEAT = 19,
  ADDR_RESET = 25,
  ADDR_BATTERY_V = 42,
  ADDR_SOUND = 50,
  ADDR_TORQUE = 59,
  ADDR_GOAL_POS = 60,
  ADDR_GOAL_POS_WR = 84,
  ADDR_PRESENT_POS = 85,
  ADDR_PRESENT_VEL = 109,
  ADDR_PRESENT_CUR = 133,
  ADDR_GRIPPER_CUR = 145,
};

void Begin();
void Run();

}  // namespace mk2_arm
