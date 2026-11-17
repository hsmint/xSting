#pragma once

#include <Dynamixel2Arduino.h>

#define TORQUE_ENABLE ControlTableItem::TORQUE_ENABLE

enum MotorLocation{
  LEFT_1 = 0,
  LEFT_2,
  RIGHT_1,
  RIGHT_2,
  MOTOR_NUM_MAX
};

enum VelocityType{
  LINEAR = 0,
  ANGULAR,
  TYPE_NUM_MAX
};


class xStingMotorDriver
{
 public:
  xStingMotorDriver();
  ~xStingMotorDriver();
  
  bool init(void);
  void close(void);

  bool is_connected();

  bool set_torque(bool onoff);
  bool get_torque();
  
  bool read_present_position(int32_t &left_value_1, int32_t &left_value_2, int32_t &right_value_1, int32_t &right_value_2);
  bool read_present_velocity(int32_t &left_value_1, int32_t &left_value_2, int32_t &right_value_1, int32_t &right_value_2);
  bool read_present_current(int16_t &left_value_1, int16_t &left_value_2, int16_t &right_value_1, int16_t &right_value_2);
  bool read_profile_acceleration(uint32_t &left_value_1, uint32_t &left_value_2, uint32_t &right_value_1, uint32_t &right_value_2);

  bool write_velocity(int32_t left_value_1, int32_t left_value_2, int32_t right_value_1, int32_t right_value_2);
  bool write_profile_acceleration(uint32_t left_value_1, uint32_t left_value_2, uint32_t right_value_1, uint32_t right_value_2);

  bool control_motors(const float wheel_separation, float linear_value, float angular_value);

  Dynamixel2Arduino& getDxl();
  
 private:
  uint8_t wheel_id_[4];
  bool torque_;
};