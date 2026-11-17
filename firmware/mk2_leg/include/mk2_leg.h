#pragma once

#include <stdint.h>

namespace mk2_leg {

// Select front (true: ID 200) or rear (false: ID 201) OpenCR board.
#ifndef XSTING_IS_FRONT
#define XSTING_IS_FRONT 1
#endif
#ifndef XSTING_AUTO_STAND
#define XSTING_AUTO_STAND 1
#endif
constexpr bool kIsFront = (XSTING_IS_FRONT != 0);
constexpr bool kAutoStand = (XSTING_AUTO_STAND != 0);
constexpr uint8_t kSlaveId = kIsFront ? 200 : 201;
constexpr uint8_t kMotorCount = 9;
constexpr uint32_t kMotorBaud = 3000000;
constexpr uint32_t kUpdateMs = 5;

// Front HC-SR04 ultrasonic sensor pins (Arduino D2=TRIG, D3=ECHO/EXTI_1) and timing.
constexpr uint8_t kSonarTrigPin = 2;
constexpr uint8_t kSonarEchoPin = 3;
constexpr uint32_t kSonarIntervalUs = 65000;
constexpr uint32_t kSonarTimeoutUs = 30000;

// Motor IDs and boot standby ticks (yaw +-45/0 deg, hip +90 deg up, knee 0 deg straight).
constexpr uint8_t kMotorIds[kMotorCount] = {
    kIsFront ? 1 : 7,   kIsFront ? 2 : 8,   kIsFront ? 3 : 9,
    kIsFront ? 4 : 13,  kIsFront ? 5 : 14,  kIsFront ? 6 : 15,
    kIsFront ? 10 : 16, kIsFront ? 11 : 17, kIsFront ? 12 : 18};
constexpr int32_t kStandTicks[kMotorCount] = {
    kIsFront ? 1536 : 2560, 3072, 2048,
    kIsFront ? 1024 : 3072, 3072, 2048,
    kIsFront ? 1536 : 2560, 3072, 2048};

// USB DYNAMIXEL slave control table addresses.
enum Addr : uint16_t {
  ADDR_MILLIS = 10,
  ADDR_CONNECT_MOTORS = 16,
  ADDR_DEVICE_STATUS = 18,
  ADDR_HEARTBEAT = 19,
  ADDR_RESET = 25,
  ADDR_ULTRASONIC_MM = 30,
  ADDR_BATTERY_V = 42,
  ADDR_SOUND = 50,
  ADDR_TORQUE = 59,
  ADDR_GOAL_POS = 60,
  ADDR_GOAL_POS_WR = 96,
  ADDR_PRESENT_POS = 97,
  ADDR_PRESENT_VEL = 133,
  ADDR_PRESENT_CUR = 169,
  ADDR_IMU_CAL = 261,
  ADDR_IMU_STATUS = 262,
  ADDR_GYRO_X = 264,
  ADDR_GYRO_Y = 268,
  ADDR_GYRO_Z = 272,
  ADDR_ACC_X = 276,
  ADDR_ACC_Y = 280,
  ADDR_ACC_Z = 284,
  ADDR_QUAT_W = 288,
  ADDR_QUAT_X = 292,
  ADDR_QUAT_Y = 296,
  ADDR_QUAT_Z = 300,
};

void Begin();
void Run();

}  // namespace mk2_leg
