#pragma once

#include <Dynamixel2Arduino.h>

#include "xSting.h"

typedef struct {
  int32_t value[mk2::MOTOR_COUNT];
} joint_position_info_t;
typedef struct {
  int32_t value[mk2::MOTOR_COUNT];
} joint_velocity_info_t;
typedef struct {
  int16_t value[mk2::MOTOR_COUNT];
} joint_current_info_t;
typedef struct {
  uint32_t value[mk2::MOTOR_COUNT];
} joint_accel_info_t;

class xStingMotorDriver {
 public:
  xStingMotorDriver();
  bool init();
  void close();
  bool is_connected();
  bool set_torque(bool onoff);
  bool get_torque() const;

  bool read_present_position(joint_position_info_t& position_info);
  bool read_present_velocity(joint_velocity_info_t& velocity_info);
  bool read_present_current(joint_current_info_t& current_info);
  bool write_goal_position_joint(const joint_position_info_t& position_info);
  bool write_profile_acceleration_joint(const joint_accel_info_t& accel_info);
  bool write_profile_velocity_joint(const joint_accel_info_t& velocity_info);

 private:
  bool is_ready() const;
  bool read_all(uint16_t address, void* values, uint16_t width);
  bool write_all(uint16_t address, const void* values, uint16_t width);

  uint8_t motor_id_[mk2::MOTOR_COUNT];
  bool torque_;
  bool is_init_;
  bool is_connected_;
};
