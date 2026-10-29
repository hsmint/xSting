#ifndef XSTING_BASE_HARDWARE__XSTING_BASE_SYSTEM_HPP_
#define XSTING_BASE_HARDWARE__XSTING_BASE_SYSTEM_HPP_

#include <memory>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "xsting_base_hardware/opencr_base.hpp"

namespace xsting_base_hardware
{
// Expects info_.joints in base::wheels::Wheel order (left 1, left 2, right 1, right 2),
// each with a velocity command interface and position and velocity state interfaces.
class XStingBaseSystemHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  uint8_t id_;
  std::string usb_port_;
  uint32_t baud_rate_;
  uint8_t heartbeat_;
  // Seconds of the last heartbeat / wheel command, see write().
  double last_heartbeat_sec_ = 0.0;
  double last_command_sec_ = 0.0;

  std::unique_ptr<OpenCRBase> opencr_;
};
}  // namespace xsting_base_hardware

#endif  // XSTING_BASE_HARDWARE__XSTING_BASE_SYSTEM_HPP_
