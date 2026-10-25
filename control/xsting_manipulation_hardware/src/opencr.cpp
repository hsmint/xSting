// Copyright 2022 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

#include "xsting_manipulation_hardware/opencr.hpp"

#include <unistd.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace xsting_manipulation_hardware
{
OpenCR::OpenCR(const uint8_t & id)
{
  dxl_sdk_wrapper_ = std::make_unique<DynamixelSDKWrapper>(id);
}

OpenCR::~OpenCR()
{
  send_heartbeat(1);
  read_all();

  std::array<int32_t, opencr::joints::NUM> tick{};
  for (uint8_t i = 0; i < tick.size(); i++) {
    tick[i] = get_data<int32_t>(
      opencr_control_table.present_position_joint_1.address + i * 4, 4);
  }
  set_joints_variables(opencr_control_table.goal_position_joint_1.address, tick);
  dxl_sdk_wrapper_->write_byte(opencr_control_table.goal_position_write_joints.address, 1);

  // init_gripper(); // 그리퍼를 0 위치로

  sleep(3);

  joints_torque(opencr::OFF);
}

bool OpenCR::open_port(const std::string & usb_port)
{
  return dxl_sdk_wrapper_->open_port(usb_port);
}

bool OpenCR::set_baud_rate(const uint32_t & baud_rate)
{
  return dxl_sdk_wrapper_->set_baud_rate(baud_rate);
}

uint16_t OpenCR::ping()
{
  return dxl_sdk_wrapper_->ping();
}

bool OpenCR::is_connect_manipulator()
{
  return dxl_sdk_wrapper_->read_byte(opencr_control_table.connect_manipulator.address);
}

void OpenCR::play_sound(uint8_t sound) const
{
  dxl_sdk_wrapper_->write_byte(opencr_control_table.sound.address, sound);
}

opencr::Battery OpenCR::get_battery()
{
  opencr::Battery battery;

  battery.design_capacity = 1.8;

  battery.voltage = 0.01 * get_data<int32_t>(
    opencr_control_table.battery_voltage.address,
    opencr_control_table.battery_voltage.length);

  battery.percentage = 0.01 * get_data<int32_t>(
    opencr_control_table.battery_percentage.address,
    opencr_control_table.battery_percentage.length);

  battery.voltage <= 10.5 ? battery.present = false : battery.present = true;

  return battery;
}

void OpenCR::joints_torque(uint8_t onoff) const
{
  dxl_sdk_wrapper_->write_byte(opencr_control_table.torque_joints.address, onoff);
}

bool OpenCR::read_all()
{
  bool comm_result = dxl_sdk_wrapper_->read(0, CONTROL_TABLE_SIZE, &data_buffer_[0]);

  if (comm_result) {
    std::lock_guard<std::mutex> lock(buffer_m_);
    std::copy(data_buffer_, data_buffer_ + CONTROL_TABLE_SIZE, data_);
  }

  return comm_result;
}

inline int32_t convert_radian_to_tick(
  const double & radian,
  const int32_t & max_tick,
  const int32_t & min_tick,
  const double & max_radian,
  const double & min_radian)
{
  int32_t tick = 0;
  int32_t zero_tick = (max_tick + min_tick) / 2;

  if (radian > 0) {
    tick = (radian * (max_tick - zero_tick) / max_radian) + zero_tick;
  } else if (radian < 0) {
    tick = (radian * (min_tick - zero_tick) / min_radian) + zero_tick;
  } else {
    tick = zero_tick;
  }

  return tick;
}

inline double convert_tick_to_radian(
  const int32_t & tick,
  const int32_t & max_tick,
  const int32_t & min_tick,
  const double & max_radian,
  const double & min_radian)
{
  double radian = 0.0;
  int32_t zero_tick = (max_tick + min_tick) / 2;

  if (tick > zero_tick) {
    radian = static_cast<double>(tick - zero_tick) * max_radian /
      static_cast<double>(max_tick - zero_tick);
  } else if (tick < zero_tick) {
    radian = static_cast<double>(tick - zero_tick) * min_radian /
      static_cast<double>(min_tick - zero_tick);
  } else {
    radian = 0.0;
  }

  return radian;
}

std::array<double, opencr::joints::NUM> OpenCR::get_joint_positions()
{
  std::array<double, opencr::joints::NUM> positions = {};

  std::array<int32_t, opencr::joints::NUM> ticks = {
    get_data<int32_t>(
      opencr_control_table.present_position_joint_1.address,
      opencr_control_table.present_position_joint_1.length),
    get_data<int32_t>(
      opencr_control_table.present_position_joint_2.address,
      opencr_control_table.present_position_joint_2.length),
    get_data<int32_t>(
      opencr_control_table.present_position_joint_3.address,
      opencr_control_table.present_position_joint_3.length),
    get_data<int32_t>(
      opencr_control_table.present_position_joint_4.address,
      opencr_control_table.present_position_joint_4.length),
    get_data<int32_t>(
      opencr_control_table.present_position_joint_5.address,
      opencr_control_table.present_position_joint_5.length),
    get_data<int32_t>(
      opencr_control_table.present_position_joint_6.address,
      opencr_control_table.present_position_joint_6.length),
    // get_data<int32_t>(
    //   opencr_control_table.present_position_joint_7.address,
    //   opencr_control_table.present_position_joint_7.length),
  };

  for (uint8_t i = 0; i < ticks.size(); i++) {
    positions[i] = convert_tick_to_radian(
      ticks[i],
      opencr::joints::MAX_TICK,
      opencr::joints::MIN_TICK,
      opencr::joints::MAX_RADIAN,
      opencr::joints::MIN_RADIAN);
  }

  return positions;
}

std::array<double, opencr::joints::NUM> OpenCR::get_joint_velocities()
{
  std::array<double, opencr::joints::NUM> velocities = {};

  std::array<int32_t, opencr::joints::NUM> rpms = {
    get_data<int32_t>(
      opencr_control_table.present_velocity_joint_1.address,
      opencr_control_table.present_velocity_joint_1.length),
    get_data<int32_t>(
      opencr_control_table.present_velocity_joint_2.address,
      opencr_control_table.present_velocity_joint_2.length),
    get_data<int32_t>(
      opencr_control_table.present_velocity_joint_3.address,
      opencr_control_table.present_velocity_joint_3.length),
    get_data<int32_t>(
      opencr_control_table.present_velocity_joint_4.address,
      opencr_control_table.present_velocity_joint_4.length),
    get_data<int32_t>(
      opencr_control_table.present_velocity_joint_5.address,
      opencr_control_table.present_velocity_joint_5.length),
    get_data<int32_t>(
      opencr_control_table.present_velocity_joint_6.address,
      opencr_control_table.present_velocity_joint_6.length),
    // get_data<int32_t>(
    //   opencr_control_table.present_velocity_joint_7.address,
    //   opencr_control_table.present_velocity_joint_7.length),
  };

  for (uint8_t i = 0; i < rpms.size(); i++) {
    velocities[i] = rpms[i] * opencr::joints::RPM_TO_RAD_PER_SEC;
  }

  return velocities;
}

double OpenCR::get_gripper_position()
{
  double radian = 0.0;

  int32_t tick = get_data<int32_t>(
    opencr_control_table.present_position_gripper.address,
    opencr_control_table.present_position_gripper.length);

  radian = convert_tick_to_radian(
    tick,
    opencr::joints::MAX_TICK,
    opencr::joints::MIN_TICK,
    opencr::joints::MAX_RADIAN,
    opencr::joints::MIN_RADIAN);

  return radian * opencr::grippers::RAD_TO_METER;
}

double OpenCR::get_gripper_velocity()
{
  double velocity = 0.0;

  int32_t rpm = get_data<int32_t>(
    opencr_control_table.present_velocity_gripper.address,
    opencr_control_table.present_velocity_gripper.length);

  return velocity = rpm * opencr::joints::RPM_TO_RAD_PER_SEC * opencr::grippers::RAD_TO_METER;
}

bool OpenCR::set_joints_variables(
  const uint16_t & address,
  const std::array<int32_t, opencr::joints::NUM> & variables)
{
  union Data {
    int32_t dword[opencr::joints::NUM]; // 6 * 32 bit
    uint8_t byte[opencr::joints::NUM * 4]; // 6 * 4 * 8 bit
  } data;

  for (uint8_t i = 0; i < variables.size(); i++) {
    data.dword[i] = variables[i];
  }

  uint8_t * p_data = &data.byte[0];
  bool comm_result = dxl_sdk_wrapper_->write(address, opencr::joints::NUM * 4, p_data);

  return comm_result;
}

bool OpenCR::set_joint_positions(const std::vector<double> & radians)
{
  std::array<int32_t, opencr::joints::NUM> tick = {};
  for (uint8_t i = 0; i < radians.size() && i < tick.size(); i++) {
    tick[i] = convert_radian_to_tick(
      radians[i],
      opencr::joints::MAX_TICK,
      opencr::joints::MIN_TICK,
      opencr::joints::MAX_RADIAN,
      opencr::joints::MIN_RADIAN);
  }

  bool comm_result = set_joints_variables(
    opencr_control_table.goal_position_joint_1.address, tick);

  dxl_sdk_wrapper_->write_byte(opencr_control_table.goal_position_write_joints.address, 1);

  return comm_result;
}

bool OpenCR::set_joint_profile_acceleration(const std::array<int32_t, opencr::joints::NUM> & acceleration)
{
  bool comm_result = set_joints_variables(
    opencr_control_table.profile_acceleration_joint_1.address, acceleration);

  dxl_sdk_wrapper_->write_byte(opencr_control_table.profile_acceleration_write_joints.address, 1);

  return comm_result;
}

bool OpenCR::set_joint_profile_velocity(const std::array<int32_t, opencr::joints::NUM> & velocity)
{
  bool comm_result = set_joints_variables(
    opencr_control_table.profile_velocity_joint_1.address, velocity);

  dxl_sdk_wrapper_->write_byte(opencr_control_table.profile_velocity_write_joints.address, 1);

  return comm_result;
}

bool OpenCR::set_gripper_variables(const uint16_t & address, const int32_t & variable)
{
  union Data {
    int32_t dword[1];
    uint8_t byte[1 * 4];
  } data;

  data.dword[0] = variable;

  uint8_t * p_data = &data.byte[0];
  bool comm_result = dxl_sdk_wrapper_->write(address, 4, p_data);

  return comm_result;
}

bool OpenCR::set_gripper_position(const double & meters)
{
  double radian = meters / opencr::grippers::RAD_TO_METER;
  int32_t tick = convert_radian_to_tick(
    radian,
    opencr::joints::MAX_TICK,
    opencr::joints::MIN_TICK,
    opencr::joints::MAX_RADIAN,
    opencr::joints::MIN_RADIAN);

  bool comm_result = set_gripper_variables(
    opencr_control_table.goal_position_gripper.address, tick);

  dxl_sdk_wrapper_->write_byte(opencr_control_table.goal_position_write_gripper.address, 1);

  return comm_result;
}

bool OpenCR::set_gripper_profile_acceleration(const int32_t & acceleration)
{
  bool comm_result = set_gripper_variables(
    opencr_control_table.profile_acceleration_gripper.address, acceleration);

  dxl_sdk_wrapper_->write_byte(opencr_control_table.profile_acceleration_write_gripper.address, 1);

  return comm_result;
}

bool OpenCR::set_gripper_profile_velocity(const int32_t & velocity)
{
  bool comm_result = set_gripper_variables(
    opencr_control_table.profile_velocity_gripper.address, velocity);

  dxl_sdk_wrapper_->write_byte(opencr_control_table.profile_velocity_write_gripper.address, 1);

  return comm_result;
}

bool OpenCR::set_init_pose()
{
  std::vector<double> init_pose(opencr::joints::NUM, 0.0);
  return set_joint_positions(init_pose);
}

bool OpenCR::set_zero_pose()
{
  std::vector<double> zero_pose(opencr::joints::NUM, 0.0);
  return set_joint_positions(zero_pose);
}

bool OpenCR::set_home_pose()
{
  std::vector<double> home_pose(opencr::joints::NUM, 0.0);
  return set_joint_positions(home_pose);
}

bool OpenCR::set_gripper_current()
{
  union Data {
    int16_t word[1];
    uint8_t byte[1 * 2];
  } data;

  data.word[0] = opencr::grippers::GOAL_CURRENT;

  uint8_t * p_data = &data.byte[0];
  bool comm_result = dxl_sdk_wrapper_->write(
    opencr_control_table.goal_current_gripper.address,
    opencr_control_table.goal_current_gripper.length,
    p_data);

  dxl_sdk_wrapper_->write_byte(opencr_control_table.goal_current_write_gripper.address, 1);

  return comm_result;
}

bool OpenCR::open_gripper()
{
  return set_gripper_position(0.01);
}

bool OpenCR::close_gripper()
{
  return set_gripper_position(-0.01);
}

bool OpenCR::init_gripper()
{
  return set_gripper_position(0.0);
}

uint8_t OpenCR::read_byte(const uint16_t & address)
{
  return dxl_sdk_wrapper_->read_byte(address);
}

void OpenCR::write_byte(const uint16_t & address, uint8_t data)
{
  dxl_sdk_wrapper_->write_byte(address, data);
}

void OpenCR::send_heartbeat(const uint8_t & count)
{
  this->write_byte(opencr_control_table.heartbeat.address, count);
}
}  // namespace turtlebot3_manipulation_hardware