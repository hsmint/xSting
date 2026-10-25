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

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "xsting_manipulation_hardware/xsting_manipulation_system.hpp"

namespace xsting_manipulation_hardware
{
namespace
{
auto logger = rclcpp::get_logger("xsting_manipulation");

std::string position_of(const std::string & joint)
{
  return joint + "/" + hardware_interface::HW_IF_POSITION;
}

std::string velocity_of(const std::string & joint)
{
  return joint + "/" + hardware_interface::HW_IF_VELOCITY;
}
}  // namespace

hardware_interface::CallbackReturn XStingManipulationSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() < opencr::joints::NUM + 1) {
    RCLCPP_FATAL(
      logger, "Expected %u arm joints and a gripper joint, got %zu joints",
      opencr::joints::NUM, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  id_ = stoi(info_.hardware_parameters["opencr_id"]);
  usb_port_ = info_.hardware_parameters["opencr_usb_port"];
  baud_rate_ = stoi(info_.hardware_parameters["opencr_baud_rate"]);
  heartbeat_ = 0;

  joints_acceleration_.fill(stoi(info_.hardware_parameters["dxl_joints_profile_acceleration"]));
  joints_velocity_.fill(stoi(info_.hardware_parameters["dxl_joints_profile_velocity"]));

  gripper_acceleration_ = stoi(info_.hardware_parameters["dxl_gripper_profile_acceleration"]);
  gripper_velocity_ = stoi(info_.hardware_parameters["dxl_gripper_profile_velocity"]);

  opencr_ = std::make_unique<OpenCR>(id_);
  if (opencr_->open_port(usb_port_)) {
    RCLCPP_INFO(logger, "Succeeded to open port");
  } else {
    RCLCPP_FATAL(logger, "Failed to open port");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (opencr_->set_baud_rate(baud_rate_)) {
    RCLCPP_INFO(logger, "Succeeded to set baudrate");
  } else {
    RCLCPP_FATAL(logger, "Failed to set baudrate");
    return hardware_interface::CallbackReturn::ERROR;
  }

  int32_t model_number = opencr_->ping();
  RCLCPP_INFO(logger, "OpenCR Model Number %d", model_number);

  if (opencr_->is_connect_manipulator()) {
    RCLCPP_INFO(logger, "Connected manipulator");
  } else {
    RCLCPP_FATAL(logger, "Not connected manipulator");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn XStingManipulationSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger, "Ready for start");

  // The firmware ignores goal writes until it sees the heartbeat change.
  opencr_->send_heartbeat(heartbeat_++);

  RCLCPP_INFO(logger, "Joints torque ON");
  opencr_->joints_torque(opencr::ON);

  opencr_->send_heartbeat(heartbeat_++);
  RCLCPP_INFO(logger, "Set profile acceleration and velocity to joints");
  opencr_->set_joint_profile_acceleration(joints_acceleration_);
  opencr_->set_joint_profile_velocity(joints_velocity_);

  RCLCPP_INFO(logger, "Set profile acceleration and velocity to gripper");
  opencr_->set_gripper_profile_acceleration(gripper_acceleration_);
  opencr_->set_gripper_profile_velocity(gripper_velocity_);

  RCLCPP_INFO(logger, "Set goal current value to gripper");
  opencr_->set_gripper_current();

  // Start commands at the present pose so the arm does not jump on activation.
  if (opencr_->read_all()) {
    const auto positions = opencr_->get_joint_positions();
    for (size_t i = 0; i < opencr::joints::NUM; i++) {
      set_command(position_of(info_.joints[i].name), positions[i]);
    }
    set_command(position_of(info_.joints[GRIPPER_INDEX].name), opencr_->get_gripper_position());
  } else {
    RCLCPP_WARN(logger, "Failed to read present pose before activation");
  }

  RCLCPP_INFO(logger, "System starting");
  opencr_->play_sound(opencr::SOUND::ASCENDING);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn XStingManipulationSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger, "Ready for stop");
  opencr_->play_sound(opencr::SOUND::DESCENDING);

  RCLCPP_INFO(logger, "System stopped");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type XStingManipulationSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO_ONCE(logger, "Start to read manipulator states");

  if (opencr_->read_all() == false) {
    RCLCPP_WARN(logger, "Failed to read all control table");
  }

  const auto positions = opencr_->get_joint_positions();
  const auto velocities = opencr_->get_joint_velocities();
  for (size_t i = 0; i < opencr::joints::NUM; i++) {
    set_state(position_of(info_.joints[i].name), positions[i]);
    if (has_state(velocity_of(info_.joints[i].name))) {
      set_state(velocity_of(info_.joints[i].name), velocities[i]);
    }
  }

  // Any joints after the gripper (e.g. a mirrored finger) report the gripper state.
  const double gripper_position = opencr_->get_gripper_position();
  const double gripper_velocity = opencr_->get_gripper_velocity();
  for (size_t i = GRIPPER_INDEX; i < info_.joints.size(); i++) {
    set_state(position_of(info_.joints[i].name), gripper_position);
    if (has_state(velocity_of(info_.joints[i].name))) {
      set_state(velocity_of(info_.joints[i].name), gripper_velocity);
    }
  }

  const auto battery = opencr_->get_battery();
  for (const auto & sensor : info_.sensors) {
    for (const auto & state : sensor.state_interfaces) {
      const std::string name = sensor.name + "/" + state.name;
      if (state.name == "voltage") {
        set_state(name, battery.voltage);
      } else if (state.name == "percentage") {
        set_state(name, battery.percentage);
      } else if (state.name == "design_capacity") {
        set_state(name, battery.design_capacity);
      } else if (state.name == "present") {
        set_state(name, battery.present ? 1.0 : 0.0);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type XStingManipulationSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO_ONCE(logger, "Start to write manipulator commands");
  opencr_->send_heartbeat(heartbeat_++);

  std::vector<double> joint_commands(opencr::joints::NUM);
  for (size_t i = 0; i < opencr::joints::NUM; i++) {
    joint_commands[i] = get_command<double>(position_of(info_.joints[i].name));
    if (std::isnan(joint_commands[i])) {
      return hardware_interface::return_type::OK;
    }
  }

  if (opencr_->set_joint_positions(joint_commands) == false) {
    RCLCPP_ERROR(logger, "Can't control joints");
  }

  const double gripper_command =
    get_command<double>(position_of(info_.joints[GRIPPER_INDEX].name));
  if (!std::isnan(gripper_command) && opencr_->set_gripper_position(gripper_command) == false) {
    RCLCPP_ERROR(logger, "Can't control gripper");
  }

  return hardware_interface::return_type::OK;
}
}  // namespace xsting_manipulation_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  xsting_manipulation_hardware::XStingManipulationSystemHardware,
  hardware_interface::SystemInterface)
