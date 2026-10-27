#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "xsting_base_hardware/xsting_base_system.hpp"

namespace xsting_base_hardware
{
namespace
{
auto logger = rclcpp::get_logger("xsting_base");

std::string position_of(const std::string & joint)
{
  return joint + "/" + hardware_interface::HW_IF_POSITION;
}

std::string velocity_of(const std::string & joint)
{
  return joint + "/" + hardware_interface::HW_IF_VELOCITY;
}

// The arm and wheel OpenCRs share one by-id name, and by-path changes with the USB socket,
// but each firmware answers only its own ID. So try every /dev/ttyACM* and keep the one where
// this ID answers a ping. Returns nullptr, with usb_port untouched, if none answers.
std::unique_ptr<OpenCRBase> find_opencr(
  const uint8_t & id, const uint32_t & baud_rate, std::string & usb_port)
{
  std::vector<std::string> ports;
  for (const auto & entry : std::filesystem::directory_iterator("/dev")) {
    if (entry.path().filename().string().rfind("ttyACM", 0) == 0) {
      ports.push_back(entry.path().string());
    }
  }
  std::sort(ports.begin(), ports.end());

  for (const auto & port : ports) {
    auto opencr = std::make_unique<OpenCRBase>(id);
    if (!opencr->open(port, baud_rate)) {
      RCLCPP_WARN(logger, "Can't open %s. Is the user in the dialout group?", port.c_str());
      continue;
    }
    if (opencr->ping() != 0) {
      usb_port = port;
      return opencr;
    }
    RCLCPP_INFO(logger, "%s is not OpenCR ID %u, skipping", port.c_str(), id);
  }
  return nullptr;
}
}  // namespace

hardware_interface::CallbackReturn XStingBaseSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != base::wheels::NUM) {
    RCLCPP_FATAL(
      logger, "Expected %u wheel joints, got %zu joints",
      base::wheels::NUM, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Check before stoi(), which throws on the empty string a missing parameter gives.
  for (const auto & name : {"opencr_id", "opencr_usb_port", "opencr_baud_rate"}) {
    if (info_.hardware_parameters.count(name) == 0) {
      RCLCPP_FATAL(logger, "Missing hardware parameter '%s'", name);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  id_ = stoi(info_.hardware_parameters["opencr_id"]);
  usb_port_ = info_.hardware_parameters["opencr_usb_port"];
  baud_rate_ = stoi(info_.hardware_parameters["opencr_baud_rate"]);
  heartbeat_ = 0;

  if (usb_port_ == "auto") {
    opencr_ = find_opencr(id_, baud_rate_, usb_port_);
    if (opencr_ == nullptr) {
      RCLCPP_FATAL(
        logger, "No OpenCR ID %u answered on /dev/ttyACM*. Is the wheel board plugged in?", id_);
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger, "Found OpenCR ID %u on %s at %u baud", id_, usb_port_.c_str(), baud_rate_);
  } else {
    opencr_ = std::make_unique<OpenCRBase>(id_);
    if (opencr_->open(usb_port_, baud_rate_)) {
      RCLCPP_INFO(logger, "Opened %s at %u baud", usb_port_.c_str(), baud_rate_);
    } else {
      RCLCPP_FATAL(logger, "Failed to open %s at %u baud", usb_port_.c_str(), baud_rate_);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  const uint16_t model_number = opencr_->ping();
  if (model_number == 0) {
    RCLCPP_FATAL(
      logger, "No reply from OpenCR ID %u on %s. Wrong port, or the arm OpenCR (ID 200)?",
      id_, usb_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger, "Wheel OpenCR model number 0x%04X", model_number);

  if (opencr_->is_connect_motors()) {
    RCLCPP_INFO(logger, "Connected wheels");
  } else {
    RCLCPP_FATAL(logger, "Wheel motors not connected. Check motor power and cables.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn XStingBaseSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger, "Ready for start");

  // The firmware ignores cmd_vel until the heartbeat changes, and it starts at 0,
  // so increment before sending.
  opencr_->send_heartbeat(++heartbeat_);

  opencr_->motors_torque(base::ON);

  // Clear any command left from a previous activation.
  opencr_->stop();
  for (int i=0; i<base::wheels::NUM; i++) {
    set_command(velocity_of(info_.joints[i].name), 0.0);
  }

  RCLCPP_INFO(logger, "System starting");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn XStingBaseSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger, "Ready for stop");

  // Torque stays on: zero velocity with torque on holds the wheels in place.
  opencr_->send_heartbeat(++heartbeat_);
  opencr_->stop();

  RCLCPP_INFO(logger, "System stopped");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type XStingBaseSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (opencr_->read_all() == false)
    RCLCPP_WARN(logger, "Failed to read all control table");

  const auto positions = opencr_->get_wheel_positions();
  const auto velocities = opencr_->get_wheel_velocities();

  for (size_t i = 0; i < base::wheels::NUM; i++) {
    set_state(position_of(info_.joints[i].name), positions[i]);
    if (has_state(velocity_of(info_.joints[i].name))) {
      set_state(velocity_of(info_.joints[i].name), velocities[i]);
    }
  }

  // Interface names match the <sensor name="imu"> block in the URDF.
  const auto imu = opencr_->get_imu();
    for (const auto & sensor : info_.sensors) {
    for (const auto & state : sensor.state_interfaces) {
      const std::string name = sensor.name + "/" + state.name;
      if (state.name == "orientation.w") {
        set_state(name, imu.orientation_w);
      } else if (state.name == "orientation.x") {
        set_state(name, imu.orientation_x);
      }
      else if (state.name == "orientation.y") {
        set_state(name, imu.orientation_y);
      }
      else if (state.name == "orientation.z") {
        set_state(name, imu.orientation_z);
      }
      else if (state.name == "angular_velocity.x") {
        set_state(name, imu.angular_velocity[0]);
      }
      else if (state.name == "angular_velocity.y") {
        set_state(name, imu.angular_velocity[1]);
      }
      else if (state.name == "angular_velocity.z") {
        set_state(name, imu.angular_velocity[2]);
      }
      else if (state.name == "linear_acceleration.x") {
        set_state(name, imu.linear_acceleration[0]);
      }
      else if (state.name == "linear_acceleration.y") {
        set_state(name, imu.linear_acceleration[1]);
      }
      else if (state.name == "linear_acceleration.z") {
        set_state(name, imu.linear_acceleration[2]);
      }
    }
  }
  

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type XStingBaseSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  opencr_->send_heartbeat(++heartbeat_);

  OpenCRBase::WheelArray commands = {};
  for (size_t i = 0; i < base::wheels::NUM; i++) {
    commands[i] = get_command<double>(velocity_of(info_.joints[i].name));
    // NaN means no controller has commanded yet. Send 0 instead of skipping the write,
    // or the firmware keeps running the last speed while the heartbeat stays alive.
    if (std::isnan(commands[i])) {
      commands[i] = 0.0;
    }
  }

  // Returning ERROR would make ros2_control shut the hardware down over one dropped packet.
  if (opencr_->set_wheel_velocities(commands) == false) {
    RCLCPP_ERROR(logger, "Can't control wheels");
  }

  return hardware_interface::return_type::OK;
}
}  // namespace xsting_base_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  xsting_base_hardware::XStingBaseSystemHardware,
  hardware_interface::SystemInterface)
