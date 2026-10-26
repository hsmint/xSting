#ifndef XSTING_BASE_HARDWARE__BASE_DEFINITIONS_HPP_
#define XSTING_BASE_HARDWARE__BASE_DEFINITIONS_HPP_

#include <cmath>
#include <cstdint>

namespace xsting_base_hardware
{
namespace base
{
constexpr uint8_t OFF = 0;
constexpr uint8_t ON = 1;

namespace wheels
{
// Same order as MotorLocation in firmware/mk1_motor/include/xSting_motor_driver.hpp.
// Index i here is the i-th wheel in base_control_table (present_position_left_1 + i * 4).
enum Wheel : uint8_t
{
  LEFT_1 = 0,
  LEFT_2,
  RIGHT_1,
  RIGHT_2,
  MOTOR_NUM_MAX
};
constexpr uint8_t NUM = MOTOR_NUM_MAX;

// wheel_radius in firmware/mk1_motor/src/xSting.cpp, in meters.
constexpr double WHEEL_RADIUS = 0.033;

// wheel_separation in firmware/mk1_motor/src/xSting.cpp.
// The firmware turns cmd_vel into wheel speeds with this value, not the URDF one,
// so the PC must use the same value when it builds the angular command.
constexpr double FIRMWARE_WHEEL_SEPARATION = 0.160;

// Present position is in ticks, 4096 ticks per revolution.
constexpr double TICK_TO_RAD = 2 * M_PI / 4096;

// Present velocity unit of X-series DYNAMIXEL is 0.229 rpm.
constexpr double VELOCITY_UNIT_TO_RAD_PER_SEC = 2 * M_PI / 60 * 0.229;

// The firmware multiplies cmd_vel items by 0.01, so the PC sends value * this.
constexpr int32_t CMD_VEL_SCALE = 100;
}  // namespace wheels
}  // namespace base
}  // namespace xsting_base_hardware

#endif  // XSTING_BASE_HARDWARE__BASE_DEFINITIONS_HPP_
