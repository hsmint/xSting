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

// Home pose: the arm folded under the deck, read from the motors (2048 is 0 rad in the URDF).
// joint4 (+86.7 deg) and joint5 (-105.3 deg) sit at the folded end of their range, so the URDF
// limits on that side must stay at or beyond these ticks or ros2_control clamps commands off home.
constexpr std::array<int32_t, NUM> HOME_TICK = {2022, 1995, 1747, 3035, 850, 2158};
}  // namespace joints

namespace grippers
{
constexpr double RAD_TO_METER = -0.015;
constexpr int16_t GOAL_CURRENT = 200;

// Gripper home, read in the same folded pose as joints::HOME_TICK
constexpr int32_t HOME_TICK = 2022;
}  // namespace grippers
}  // namespace opencr
}  // namespace xsting_manipulation_hardware

#include <limits>

#endif  // XSTING_MANIPULATION_HARDWARE__OPENCR_DEFINITIONS_HPP_
