#ifndef XSTING_MANIPULATION_HARDWARE__OPENCR_CONTROL_TABLE_HPP_
#define XSTING_MANIPULATION_HARDWARE__OPENCR_CONTROL_TABLE_HPP_

#include <cstdint>

namespace robotis
{
namespace xsting_manipulation_hardware
{
// Mirrors ControlTableItemAddr in firmware/mk1_arm/src/xSting.cpp.
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
  ControlItem connect_manipulator = {16, 1};

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

  ControlItem torque_joints = {59, 1};

  ControlItem goal_position_joint_1 = {60, 4};
  ControlItem goal_position_joint_2 = {64, 4};
  ControlItem goal_position_joint_3 = {68, 4};
  ControlItem goal_position_joint_4 = {72, 4};
  ControlItem goal_position_joint_5 = {76, 4};
  ControlItem goal_position_joint_6 = {80, 4};
  // ControlItem goal_position_joint_7 = {84, 4};
  ControlItem goal_position_gripper = {88, 4};
  ControlItem goal_position_write_joints = {92, 1};
  ControlItem goal_position_write_gripper = {93, 1};
  ControlItem goal_position_read = {94, 1};

  ControlItem present_position_joint_1 = {95, 4};
  ControlItem present_position_joint_2 = {99, 4};
  ControlItem present_position_joint_3 = {103, 4};
  ControlItem present_position_joint_4 = {107, 4};
  ControlItem present_position_joint_5 = {111, 4};
  ControlItem present_position_joint_6 = {115, 4};
  // ControlItem present_position_joint_7 = {119, 4};
  ControlItem present_position_gripper = {123, 4};

  ControlItem present_velocity_joint_1 = {127, 4};
  ControlItem present_velocity_joint_2 = {131, 4};
  ControlItem present_velocity_joint_3 = {135, 4};
  ControlItem present_velocity_joint_4 = {139, 4};
  ControlItem present_velocity_joint_5 = {143, 4};
  ControlItem present_velocity_joint_6 = {147, 4};
  // ControlItem present_velocity_joint_7 = {151, 4};
  ControlItem present_velocity_gripper = {155, 4};

  ControlItem present_current_joint_1 = {159, 2};
  ControlItem present_current_joint_2 = {161, 2};
  ControlItem present_current_joint_3 = {163, 2};
  ControlItem present_current_joint_4 = {165, 2};
  ControlItem present_current_joint_5 = {167, 2};
  ControlItem present_current_joint_6 = {169, 2};
  // ControlItem present_current_joint_7 = {171, 2};
  ControlItem present_current_gripper = {173, 2};

  ControlItem profile_acceleration_joint_1 = {175, 4};
  ControlItem profile_acceleration_joint_2 = {179, 4};
  ControlItem profile_acceleration_joint_3 = {183, 4};
  ControlItem profile_acceleration_joint_4 = {187, 4};
  ControlItem profile_acceleration_joint_5 = {191, 4};
  ControlItem profile_acceleration_joint_6 = {195, 4};
  // ControlItem profile_acceleration_joint_7 = {199, 4};
  ControlItem profile_acceleration_gripper = {203, 4};
  ControlItem profile_acceleration_write_joints = {207, 1};
  ControlItem profile_acceleration_write_gripper = {208, 1};
  ControlItem profile_acceleration_read = {209, 1};

  ControlItem profile_velocity_joint_1 = {210, 4};
  ControlItem profile_velocity_joint_2 = {214, 4};
  ControlItem profile_velocity_joint_3 = {218, 4};
  ControlItem profile_velocity_joint_4 = {222, 4};
  ControlItem profile_velocity_joint_5 = {226, 4};
  ControlItem profile_velocity_joint_6 = {230, 4};
  // ControlItem profile_velocity_joint_7 = {234, 4};
  ControlItem profile_velocity_gripper = {238, 4};
  ControlItem profile_velocity_write_joints = {242, 1};
  ControlItem profile_velocity_write_gripper = {243, 1};
  ControlItem profile_velocity_read = {244, 1};

  ControlItem goal_current_joint_1 = {245, 2};
  ControlItem goal_current_joint_2 = {247, 2};
  ControlItem goal_current_joint_3 = {249, 2};
  ControlItem goal_current_joint_4 = {251, 2};
  ControlItem goal_current_joint_5 = {253, 2};
  ControlItem goal_current_joint_6 = {255, 2};
  // ControlItem goal_current_joint_7 = {257, 2};
  ControlItem goal_current_gripper = {259, 2};
  ControlItem goal_current_write_joints = {261, 1};
  ControlItem goal_current_write_gripper = {262, 1};
  ControlItem goal_current_read = {263, 1};
} opencr_control_table;

constexpr uint16_t CONTROL_TABLE_SIZE = 264;
}  // namespace xsting_manipulation_hardware
}  // namespace robotis

#endif  // XSTING_MANIPULATION_HARDWARE__OPENCR_CONTROL_TABLE_HPP_
