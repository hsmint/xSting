#ifndef XSTING_BASE_HARDWARE__OPENCR_BASE_HPP_
#define XSTING_BASE_HARDWARE__OPENCR_BASE_HPP_

#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>

#include "xsting_manipulation_hardware/dynamixel_sdk_wrapper.hpp"
#include "xsting_base_hardware/base_control_table.hpp"
#include "xsting_base_hardware/base_definitions.hpp"

namespace xsting_base_hardware
{
// Talks to the wheel OpenCR (firmware/mk1_motor). Callers use rad and rad/s only;
// addresses and firmware units stay inside this class.
class OpenCRBase
{
public:
  // One value per wheel, in base::wheels::Wheel order.
  using WheelArray = std::array<double, base::wheels::NUM>;

  explicit OpenCRBase(const uint8_t & id);
  virtual ~OpenCRBase();

  // Opens the port and sets the baud rate. Nothing else may talk to the board until this succeeds.
  bool open(const std::string & usb_port, const uint32_t & baud_rate);
  uint16_t ping();

  bool is_connect_motors();
  void motors_torque(uint8_t onoff);
  void play_sound(uint8_t sound);
  void send_heartbeat(const uint8_t & count);

  // Reads the whole control table into the buffer that the getters below use.
  bool read_all();
  WheelArray get_wheel_positions();
  WheelArray get_wheel_velocities();

  // The firmware only takes cmd_vel, so the wheel speeds are turned into linear x and angular z.
  bool set_wheel_velocities(const WheelArray & rad_per_sec);
  bool stop();

private:
  bool write_int32(const uint16_t & address, const int32_t & value);

  // Reads a value cached by the last read_all().
  template<typename DataByteT>
  DataByteT get_data(const uint16_t & address, const uint16_t & length)
  {
    std::lock_guard<std::mutex> lock(buffer_m_);
    DataByteT data = 0;
    std::memcpy(&data, &data_[address], length);
    return data;
  }

  std::unique_ptr<xsting_manipulation_hardware::DynamixelSDKWrapper> dxl_sdk_wrapper_;

  uint8_t data_buffer_[CONTROL_TABLE_SIZE];
  uint8_t data_[CONTROL_TABLE_SIZE];
  std::mutex buffer_m_;

  // The wrapper's packet handler is only set by set_baud_rate(), so a failed open() must
  // keep the destructor from sending anything.
  bool is_open_ = false;
};
}  // namespace xsting_base_hardware

#endif  // XSTING_BASE_HARDWARE__OPENCR_BASE_HPP_
