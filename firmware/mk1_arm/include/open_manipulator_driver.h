#pragma once

#include <Dynamixel2Arduino.h>

constexpr auto DXL_TORQUE_ENABLE = ControlTableItem::TORQUE_ENABLE;

enum JointMotorLocation{
  JOINT_1 = 0,
  JOINT_2,
  JOINT_3,
  JOINT_4,
  JOINT_5,
  JOINT_6,
  GRIPPER,
  JOINT_MOTOR_NUM_MAX
};

typedef struct
{
  int32_t value[JOINT_MOTOR_NUM_MAX];
} joint_position_info_t;

typedef struct
{
  int32_t value[JOINT_MOTOR_NUM_MAX];
} joint_velocity_info_t;

typedef struct
{
  int16_t value[JOINT_MOTOR_NUM_MAX];
} joint_current_info_t;

typedef struct
{
  uint32_t value[JOINT_MOTOR_NUM_MAX];
} joint_accel_info_t;


class OpenManipulatorDriver
{
 public:
  OpenManipulatorDriver();
  ~OpenManipulatorDriver();
  
  bool init(void);
  void close(void);

  bool is_connected();  

  bool set_torque(bool onoff);
  bool get_torque(void);
  
  bool read_goal_position(joint_position_info_t &position_info);
  bool read_present_position(joint_position_info_t &position_info);
  bool read_present_velocity(joint_velocity_info_t &velocity_info);
  bool read_present_current(joint_current_info_t &current_info);
  bool read_profile_acceleration(joint_accel_info_t &accel_info);  
  bool read_profile_velocity(joint_accel_info_t &accel_info);  
  bool read_goal_current(joint_current_info_t &current_info);
  
  bool write_goal_position_joint(joint_position_info_t &position_info);
  bool write_profile_acceleration_joint(joint_accel_info_t &accel_info);
  bool write_profile_velocity_joint(joint_accel_info_t &accel_info);
  bool write_goal_current_joint(joint_current_info_t &current_info);

  bool write_goal_position_gripper(joint_position_info_t &position_info);
  bool write_profile_acceleration_gripper(joint_accel_info_t &accel_info);
  bool write_profile_velocity_gripper(joint_accel_info_t &accel_info);
  bool write_goal_current_gripper(joint_current_info_t &current_info);

 private:
  
  bool is_ready(void);

  uint8_t motor_id_[JOINT_MOTOR_NUM_MAX];
  bool torque_;
  bool is_init_;
  bool is_connected_;
};