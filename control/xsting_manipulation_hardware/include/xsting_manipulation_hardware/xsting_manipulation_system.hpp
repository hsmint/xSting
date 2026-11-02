#ifndef XSTING_MANIPULATION_HARDWARE__XSTING_MANIPULATION_SYSTEM_HPP_
#define XSTING_MANIPULATION_HARDWARE__XSTING_MANIPULATION_SYSTEM_HPP_

#include <array>
#include <memory>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "xsting_manipulation_hardware/opencr.hpp"

namespace xsting_manipulation_hardware
{
// Expects info_.joints ordered as joint1..joint6 followed by the gripper joint.
class XStingManipulationSystemHardware : public hardware_interface::SystemInterface
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
  static constexpr size_t GRIPPER_INDEX = opencr::joints::NUM;

  uint8_t id_;
  std::string usb_port_;
  uint32_t baud_rate_;
  uint8_t heartbeat_;

  std::array<int32_t, opencr::joints::NUM> joints_acceleration_;
  std::array<int32_t, opencr::joints::NUM> joints_velocity_;
  int32_t gripper_acceleration_;
  int32_t gripper_velocity_;

  // +1 or -1 per arm joint from the URDF <param name="direction">; -1 where the motor turns against the URDF axis.
  std::array<double, opencr::joints::NUM> directions_;

  std::unique_ptr<OpenCR> opencr_;
};
}  // namespace xsting_manipulation_hardware

#endif  // XSTING_MANIPULATION_HARDWARE__XSTING_MANIPULATION_SYSTEM_HPP_
