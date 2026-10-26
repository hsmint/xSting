#ifndef XSTING_BASE_HARDWARE__BASE_CONTROL_TABLE_HPP_
#define XSTING_BASE_HARDWARE__BASE_CONTROL_TABLE_HPP_

#include <cstdint>

namespace xsting_base_hardware
{
// Mirrors ControlTableItemAddr in firmware/mk1_motor/src/xSting.cpp.
struct ControlItem
{
  uint16_t address;
  uint16_t length;
};

const struct ControlTable
{
  ControlItem model_information = {2, 4};

  ControlItem millis = {10, 4};

  ControlItem debug_mode = {14, 1};
  ControlItem connect_ros2 = {15, 1};

  ControlItem device_status = {18, 1};
  ControlItem heartbeat = {19, 1};

  ControlItem user_led_1 = {20, 1};
  ControlItem user_led_2 = {21, 1};
  ControlItem user_led_3 = {22, 1};
  ControlItem user_led_4 = {23, 1};

  ControlItem button_1 = {26, 1};
  ControlItem button_2 = {27, 1};
  ControlItem bumper_1 = {28, 1};
  ControlItem bumper_2 = {29, 1};

  ControlItem battery_voltage = {42, 4};
  ControlItem battery_percentage = {46, 4};

  ControlItem sound = {50, 1};

  ControlItem imu_re_calibration = {59, 1};

  ControlItem imu_angular_velocity_x = {60, 4};
  ControlItem imu_angular_velocity_y = {64, 4};
  ControlItem imu_angular_velocity_z = {68, 4};
  ControlItem imu_linear_acceleration_x = {72, 4};
  ControlItem imu_linear_acceleration_y = {76, 4};
  ControlItem imu_linear_acceleration_z = {80, 4};
  ControlItem imu_magnetic_x = {84, 4};
  ControlItem imu_magnetic_y = {88, 4};
  ControlItem imu_magnetic_z = {92, 4};
  ControlItem imu_orientation_w = {96, 4};
  ControlItem imu_orientation_x = {100, 4};
  ControlItem imu_orientation_y = {104, 4};
  ControlItem imu_orientation_z = {108, 4};

  ControlItem connect_motors = {148, 1};
  ControlItem torque_motors = {149, 1};

  // cmd_vel values are scaled by 100 (firmware multiplies by 0.01).
  ControlItem cmd_velocity_linear_x = {150, 4};
  ControlItem cmd_velocity_linear_y = {154, 4};
  ControlItem cmd_velocity_linear_z = {158, 4};
  ControlItem cmd_velocity_angular_x = {162, 4};
  ControlItem cmd_velocity_angular_y = {166, 4};
  ControlItem cmd_velocity_angular_z = {170, 4};

  ControlItem present_current_left_1 = {182, 4};
  ControlItem present_current_left_2 = {186, 4};
  ControlItem present_current_right_1 = {190, 4};
  ControlItem present_current_right_2 = {194, 4};

  ControlItem present_velocity_left_1 = {198, 4};
  ControlItem present_velocity_left_2 = {202, 4};
  ControlItem present_velocity_right_1 = {206, 4};
  ControlItem present_velocity_right_2 = {210, 4};

  ControlItem present_position_left_1 = {214, 4};
  ControlItem present_position_left_2 = {218, 4};
  ControlItem present_position_right_1 = {222, 4};
  ControlItem present_position_right_2 = {226, 4};

  ControlItem profile_acceleration_left_1 = {230, 4};
  ControlItem profile_acceleration_left_2 = {234, 4};
  ControlItem profile_acceleration_right_1 = {238, 4};
  ControlItem profile_acceleration_right_2 = {242, 4};
} base_control_table;

constexpr uint16_t CONTROL_TABLE_SIZE = 246;
}  // namespace xsting_base_hardware

#endif  // XSTING_BASE_HARDWARE__BASE_CONTROL_TABLE_HPP_
