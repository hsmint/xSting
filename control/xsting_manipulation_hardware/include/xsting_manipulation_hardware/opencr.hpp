#ifndef XSTING_MANIPULATION_HARDWARE__OPENCR_HPP_
#define XSTING_MANIPULATION_HARDWARE__OPENCR_HPP_

#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "xsting_manipulation_hardware/dynamixel_sdk_wrapper.hpp"
#include "xsting_manipulation_hardware/opencr_control_table.hpp"
#include "xsting_manipulation_hardware/opencr_definitions.hpp"

namespace robotis
{
namespace xsting_manipulation_hardware
{
class OpenCR
{
public:
  explicit OpenCR(const uint8_t & id);
  virtual ~OpenCR();

  bool open_port(const std::string & usb_port);
  bool set_baud_rate(const uint32_t & baud_rate);
  uint16_t ping();

  bool is_connect_manipulator();
  void play_sound(uint8_t sound) const;
  opencr::Battery get_battery();
  void joints_torque(uint8_t onoff) const;

  bool read_all();

  std::array<double, opencr::joints::NUM> get_joint_positions();
  std::array<double, opencr::joints::NUM> get_joint_velocities();
  double get_gripper_position();
  double get_gripper_velocity();

  bool set_joints_variables(
    const uint16_t & address,
    const std::array<int32_t, opencr::joints::NUM> & variables);
  bool set_joint_positions(const std::vector<double> & radians);
  bool set_joint_profile_acceleration(
    const std::array<int32_t, opencr::joints::NUM> & acceleration);
  bool set_joint_profile_velocity(const std::array<int32_t, opencr::joints::NUM> & velocity);

  bool set_gripper_variables(const uint16_t & address, const int32_t & variable);
  bool set_gripper_position(const double & meters);
  bool set_gripper_profile_acceleration(const int32_t & acceleration);
  bool set_gripper_profile_velocity(const int32_t & velocity);
  bool set_gripper_current();

  bool set_init_pose();
  bool set_zero_pose();
  bool set_home_pose();

  bool open_gripper();
  bool close_gripper();
  bool init_gripper();

  uint8_t read_byte(const uint16_t & address);
  void write_byte(const uint16_t & address, uint8_t data);
  void send_heartbeat(const uint8_t & count);

private:
  // Reads a value cached by the last read_all().
  template<typename DataByteT>
  DataByteT get_data(const uint16_t & address, const uint16_t & length)
  {
    std::lock_guard<std::mutex> lock(buffer_m_);
    DataByteT data = 0;
    std::memcpy(&data, &data_[address], length);
    return data;
  }

  std::unique_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper_;

  uint8_t data_buffer_[CONTROL_TABLE_SIZE];
  uint8_t data_[CONTROL_TABLE_SIZE];
  std::mutex buffer_m_;
};
}  // namespace xsting_manipulation_hardware
}  // namespace robotis

#endif  // XSTING_MANIPULATION_HARDWARE__OPENCR_HPP_
