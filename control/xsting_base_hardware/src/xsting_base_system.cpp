#include <cmath>
#include <memory>
#include <string>

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

  // TODO J1: URDF의 바퀴 관절이 정확히 base::wheels::NUM개가 아니면 FATAL 로그를 찍고 ERROR를 반환하세요.
  //          팔 xsting_manipulation_system.cpp:54-59와 같은 모양이에요 (팔은 "<", 여기는 "!=").
  if (info_.joints.size() != base::wheels::NUM) {
    RCLCPP_FATAL(
      logger, "Expected %u wheel joints, got %zu joints",
      base::wheels::NUM, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 팔은 파라미터가 없으면 stoi("")에서 예외가 나서 죽어요. 여기서는 먼저 있는지 확인해요.
  for (const auto & name : {"opencr_id", "opencr_usb_port", "opencr_baud_rate"}) {
    if (info_.hardware_parameters.count(name) == 0) {
      RCLCPP_FATAL(logger, "Missing hardware parameter '%s'", name);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // TODO J2: 위 세 파라미터를 id_, usb_port_, baud_rate_에 읽어 넣으세요. 팔 61-63번째 줄과 같아요.
  id_ = stoi(info_.hardware_parameters["opencr_id"]);
  usb_port_ = info_.hardware_parameters["opencr_usb_port"];
  baud_rate_ = stoi(info_.hardware_parameters["opencr_baud_rate"]);
  heartbeat_ = 0;

  // TODO J3: opencr_를 만들고(팔 72번째 줄, 클래스 이름만 OpenCRBase) open()을 부르세요.
  //          open()이 실패하면 FATAL 로그 + ERROR 반환, 성공하면 INFO 로그.
  //          팔은 open_port와 set_baud_rate를 따로 불렀지만, OpenCRBase는 open() 하나로 둘 다 해요.
  opencr_ = std::make_unique<OpenCRBase>(id_);
  if (opencr_->open(usb_port_, baud_rate_)) {
    RCLCPP_INFO(logger, "Opened %s at %u baud", usb_port_.c_str(), baud_rate_);
  } else {
    RCLCPP_FATAL(logger, "Failed to open %s at %u baud", usb_port_.c_str(), baud_rate_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // TODO J4: ping() 결과가 0이면 보드가 응답하지 않은 거예요 (포트가 틀렸거나, ID 200에만 답하는 팔 보드).
  //          0이면 FATAL 로그 + ERROR 반환. 아니면 팔 87-88번째 줄처럼 모델 번호를 INFO로 찍으세요.
  const uint16_t model_number = opencr_->ping();
  if (model_number == 0) {
    RCLCPP_FATAL(
      logger, "No reply from OpenCR ID %u on %s. Wrong port, or the arm OpenCR (ID 200)?",
      id_, usb_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger, "Wheel OpenCR model number 0x%04X", model_number);

  // TODO J5: is_connect_motors()가 false면 FATAL + ERROR, true면 INFO. 팔 90-95번째 줄과 같아요.
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

  // TODO K1: heartbeat를 보내세요. 팔은 heartbeat_++ (보낸 뒤 증가)라서 첫 값이 0인데,
  //          펌웨어도 0에서 시작하니까 "바뀌지 않음"으로 보여요. 증가한 뒤 보내는 모양으로 쓰세요.
  opencr_->send_heartbeat(++heartbeat_);

  // TODO K2: 바퀴 토크를 켜세요. 켜고 끄는 값은 base_definitions.hpp의 base::ON / base::OFF
  opencr_->motors_torque(base::ON);

  // TODO K3: 바퀴를 멈추고(opencr_의 함수 하나), 네 관절의 velocity 명령을 0.0으로 초기화하세요.
  //          명령 초기화: 팔 139번째 줄 모양인데 position_of 대신 velocity_of
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

  // TODO L: heartbeat를 한 번 보낸 뒤 바퀴를 멈추세요. 끌 때는 토크를 끄지 않아요
  //         (속도 0 + 토크 ON이 브레이크 역할을 해요).
  opencr_->send_heartbeat(++heartbeat_);
  opencr_->stop();

  RCLCPP_INFO(logger, "System stopped");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type XStingBaseSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO M: 팔 168-179번째 줄과 같은 흐름이에요.
  //         read_all 실패 시 WARN, 그다음 바퀴 위치/속도를 받아 네 관절의 position, velocity 상태에 넣기.
  //         바퀴는 관절 수가 NUM으로 고정이라 has_state 확인은 필요 없어요.
  if (opencr_->read_all() == false)
    RCLCPP_WARN(logger, "Failed to read all control table");

  const auto positions = opencr_->get_wheel_positions();
  const auto velocities = opencr_->get_wheel_velocities();

  for (size_t i = 0; i < base::wheels::NUM; i++) { // 뭐하는코드?
    set_state(position_of(info_.joints[i].name), positions[i]);
    if (has_state(velocity_of(info_.joints[i].name))) {
      set_state(velocity_of(info_.joints[i].name), velocities[i]);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type XStingBaseSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO N1: heartbeat 보내기 (K1과 같은 모양)
  opencr_->send_heartbeat(++heartbeat_);

  OpenCRBase::WheelArray commands = {};
  for (size_t i = 0; i < base::wheels::NUM; i++) {
    // TODO N2: i번째 관절의 velocity 명령을 읽으세요 (팔 218번째 줄 모양, velocity_of).
    //          컨트롤러가 아직 바퀴를 잡지 않았으면 NaN이 와요. 팔은 NaN이면 그냥 return하지만,
    //          바퀴는 NaN을 0.0으로 바꿔서 보내요. 바퀴는 "멈춰"가 가장 안전한 명령이라서요.
    //          NaN 검사: std::isnan(값)
    commands[i] = get_command<double>(velocity_of(info_.joints[i].name));
    if (std::isnan(commands[i])) {
      commands[i] = 0.0;
    }
  }

  // TODO N3: commands를 보드에 보내고, 실패하면 ERROR 로그 (팔 224-226번째 줄 모양)
  if (opencr_->set_wheel_velocities(commands) == false) {
    RCLCPP_ERROR(logger, "Can't control wheels");
  }

  return hardware_interface::return_type::OK;
}
}  // namespace xsting_base_hardware

// TODO P: 팔 xsting_manipulation_system.cpp:238-241을 복사하고 클래스 이름을 이 파일의 것으로 바꾸세요.
//         이게 없으면 빌드는 되지만, 실행할 때 controller_manager가 이 클래스를 못 찾아요.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  xsting_base_hardware::XStingBaseSystemHardware,
  hardware_interface::SystemInterface)
