#include "xsting_base_hardware/opencr_base.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

namespace xsting_base_hardware
{
OpenCRBase::OpenCRBase(const uint8_t & id)
{
  dxl_sdk_wrapper_ = std::make_unique<xsting_manipulation_hardware::DynamixelSDKWrapper>(id);
}

OpenCRBase::~OpenCRBase()
{
  if (is_open_) {
    stop();
  }
}

bool OpenCRBase::open(const std::string & usb_port, const uint32_t & baud_rate)
{
  if (dxl_sdk_wrapper_->open_port(usb_port) && dxl_sdk_wrapper_->set_baud_rate(baud_rate))
    is_open_ = true;
  return is_open_;
}

uint16_t OpenCRBase::ping()
{
  return dxl_sdk_wrapper_->ping();
}

bool OpenCRBase::is_connect_motors()
{
  return dxl_sdk_wrapper_->read_byte(base_control_table.connect_motors.address);
}

void OpenCRBase::motors_torque(uint8_t onoff)
{
  dxl_sdk_wrapper_->write_byte(base_control_table.torque_motors.address, onoff);
}

void OpenCRBase::play_sound(uint8_t sound)
{
  dxl_sdk_wrapper_->write_byte(base_control_table.sound.address, sound);
}

void OpenCRBase::send_heartbeat(const uint8_t & count)
{
  dxl_sdk_wrapper_->write_byte(base_control_table.heartbeat.address, count);
}

bool OpenCRBase::read_all()
{
  bool comm_result = dxl_sdk_wrapper_->read(0, CONTROL_TABLE_SIZE, &data_buffer_[0]);

  if (comm_result) {
    std::lock_guard<std::mutex> lock(buffer_m_);
    std::copy(data_buffer_, data_buffer_ + CONTROL_TABLE_SIZE, data_);
  }

  return comm_result;
}

OpenCRBase::WheelArray OpenCRBase::get_wheel_positions()
{
  WheelArray positions = {};

  for (uint8_t i = 0; i < base::wheels::NUM; i++) {
    int32_t tick = get_data<int32_t>(base_control_table.present_position_left_1.address + i * 4, 4);
    positions[i] = base::wheels::TICK_TO_RAD * tick;
  }

  return positions;
}

OpenCRBase::WheelArray OpenCRBase::get_wheel_velocities()
{
  WheelArray velocities = {};

  for (uint8_t i = 0; i < base::wheels::NUM; i++) {
    int32_t vel = get_data<int32_t>(base_control_table.present_velocity_left_1.address + i * 4, 4);
    velocities[i] = base::wheels::VELOCITY_UNIT_TO_RAD_PER_SEC * vel;
  }

  return velocities;
}

bool OpenCRBase::write_int32(const uint16_t & address, const int32_t & value)
{
  uint8_t data[4];
  std::memcpy(&data, &value, 4);
  return dxl_sdk_wrapper_->write(address, sizeof(data), data);
}

bool OpenCRBase::set_wheel_velocities(const WheelArray & rad_per_sec)
{
  using base::wheels::LEFT_1;
  using base::wheels::LEFT_2;
  using base::wheels::RIGHT_1;
  using base::wheels::RIGHT_2;

  // Side speeds in m/s, from the average of the two wheels on each side.
  const double left = (rad_per_sec[LEFT_1] + rad_per_sec[LEFT_2]) / 2 * base::wheels::WHEEL_RADIUS;
  const double right = (rad_per_sec[RIGHT_1] + rad_per_sec[RIGHT_2]) / 2 * base::wheels::WHEEL_RADIUS;

  // Inverse of control_motors() in the firmware, so it turns these back into the same side speeds.
  const double linear = (left + right) / 2;
  const double angular = (right - left) / base::wheels::FIRMWARE_WHEEL_SEPARATION;

  // Round rather than truncate, or small speeds below 0.01 become 0.
  const bool ok_linear = write_int32(
    base_control_table.cmd_velocity_linear_x.address, static_cast<int32_t>(std::lround(linear * base::wheels::CMD_VEL_SCALE)));
  const bool ok_angular = write_int32(
    base_control_table.cmd_velocity_angular_z.address, static_cast<int32_t>(std::lround(angular * base::wheels::CMD_VEL_SCALE)));

  return ok_linear && ok_angular;
}

bool OpenCRBase::stop()
{
  return set_wheel_velocities(WheelArray{});
}
}  // namespace xsting_base_hardware
