#ifndef XSTING_MANIPULATION_HARDWARE__OPENCR_DEFINITIONS_HPP_
#define XSTING_MANIPULATION_HARDWARE__OPENCR_DEFINITIONS_HPP_

#include <cstdint>
#include <array>

namespace xsting_manipulation_hardware
{
namespace opencr
{
constexpr uint8_t OFF = 0;
constexpr uint8_t ON = 1;

// Melody indices handled by xStingSensors::makeMelody() in the firmware.
namespace SOUND
{
constexpr uint8_t OFF = 0;
constexpr uint8_t ON = 1;
constexpr uint8_t LOW_BATTERY = 2;
constexpr uint8_t ERROR = 3;
constexpr uint8_t BUTTON1 = 4;
constexpr uint8_t BUTTON2 = 5;

constexpr uint8_t ASCENDING = ON;
constexpr uint8_t DESCENDING = OFF;
}  // namespace SOUND

struct Battery
{
  double voltage;
  double percentage;
  double design_capacity;
  bool present;
};

namespace joints
{
enum Joint : uint8_t
{
  JOINT1 = 0,
  JOINT2,
  JOINT3,
  JOINT4,
  JOINT5,
  JOINT6,
  // JOINT7
};
constexpr uint8_t NUM = 6;//7;

// Present velocity unit of X-series DYNAMIXEL is 0.229 rpm.
constexpr double RPM_TO_RAD_PER_SEC = 0.229 * 0.104719755;

constexpr int32_t MAX_TICK = 4095;
constexpr int32_t MIN_TICK = 0;

constexpr double MAX_RADIAN = 3.14159265359;
constexpr double MIN_RADIAN = -3.14159265359;

// Home pose. 2048 is the DYNAMIXEL X-series center tick, i.e. 0 rad in the URDF
// Must match the home pose the firmware moves to in xStingCore::begin() (firmware/mk1_arm)
constexpr std::array<int32_t, NUM> HOME_TICK = {2048, 2048, 2048, 2048, 2048, 2048};
}  // namespace joints

namespace grippers
{
constexpr double RAD_TO_METER = -0.015;
constexpr int16_t GOAL_CURRENT = 200;

// Gripper home, Same center tick and firmware counterpart as joints::HOME_TICK
constexpr int32_t HOME_TICK = 2048;
}  // namespace grippers
}  // namespace opencr
}  // namespace xsting_manipulation_hardware

#endif  // XSTING_MANIPULATION_HARDWARE__OPENCR_DEFINITIONS_HPP_
