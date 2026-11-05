#pragma once

#include <stdint.h>

namespace mk2 {
constexpr uint8_t LEG_COUNT = 3;
constexpr uint8_t JOINTS_PER_LEG = 3;
constexpr uint8_t MOTOR_COUNT = LEG_COUNT * JOINTS_PER_LEG;
// Order: leg 1 joints 1..3, leg 2 joints 1..3, leg 3 joints 1..3.
constexpr uint8_t MOTOR_IDS[MOTOR_COUNT] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
constexpr uint8_t SLAVE_ID = 200;
constexpr uint32_t MOTOR_BAUD = 1000000;
constexpr uint32_t UPDATE_MS = 20;
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 500;
// Same 3-cell battery voltage thresholds as mk1_arm.
constexpr float BATTERY_WARNING_V = 11.0f;
constexpr float BATTERY_CUTOFF_V = 10.5f;
constexpr float BATTERY_FULL_V = 12.3f;

// USB DYNAMIXEL slave registers. Array elements use native DYNAMIXEL units.
// Fixed addresses for 9 motors.
enum Address : uint16_t {
  ADDR_MILLIS = 10,
  ADDR_CONNECT_HOST = 15,
  ADDR_CONNECT_MOTORS = 16,
  ADDR_DEVICE_STATUS = 18,
  ADDR_HEARTBEAT = 19,
  ADDR_USER_LED = 20,
  ADDR_BUTTON = 26,
  ADDR_BUMPER = 28,
  ADDR_BATTERY_VOLTAGE = 42,
  ADDR_BATTERY_PERCENT = 46,
  ADDR_SOUND = 50,
  ADDR_TORQUE = 59,
  ADDR_GOAL_POSITION = 60,
  ADDR_GOAL_POSITION_WR = 96,
  ADDR_PRESENT_POSITION = 97,
  ADDR_PRESENT_VELOCITY = 133,
  ADDR_PRESENT_CURRENT = 169,
  ADDR_PROFILE_ACC = 187,
  ADDR_PROFILE_ACC_WR = 223,
  ADDR_PROFILE_VEL = 224,
  ADDR_PROFILE_VEL_WR = 260,
  ADDR_IMU_RECALIBRATION = 261,
  ADDR_IMU_STATUS = 262,
  ADDR_ANGULAR_VELOCITY_X = 264,
  ADDR_ANGULAR_VELOCITY_Y = 268,
  ADDR_ANGULAR_VELOCITY_Z = 272,
  ADDR_LINEAR_ACC_X = 276,
  ADDR_LINEAR_ACC_Y = 280,
  ADDR_LINEAR_ACC_Z = 284,
  ADDR_ORIENTATION_W = 288,
  ADDR_ORIENTATION_X = 292,
  ADDR_ORIENTATION_Y = 296,
  ADDR_ORIENTATION_Z = 300,
};
}  // namespace mk2

namespace xStingHexapod {
void begin();
void run();
}  // namespace xStingHexapod
