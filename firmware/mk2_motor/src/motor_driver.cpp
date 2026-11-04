#include "../include/motor_driver.h"

#include <string.h>

using namespace mk2;

const float DXL_PORT_PROTOCOL_VERSION = 2.0;
const int OPENCR_DXL_DIR_PIN = 84;

static Dynamixel2Arduino dxl(Serial3, OPENCR_DXL_DIR_PIN);
static ParamForSyncReadInst_t sync_read_param;
static ParamForSyncWriteInst_t sync_write_param;
static RecvInfoFromStatusInst_t read_result;

xStingMotorDriver::xStingMotorDriver()
    : torque_(false), is_init_(false), is_connected_(false) {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) motor_id_[i] = MOTOR_IDS[i];
}

bool xStingMotorDriver::init() {
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  delay(500);
  dxl.begin(MOTOR_BAUD);
  dxl.setPortProtocolVersion(DXL_PORT_PROTOCOL_VERSION);

  sync_read_param.id_count = MOTOR_COUNT;
  sync_write_param.id_count = MOTOR_COUNT;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    sync_read_param.xel[i].id = motor_id_[i];
    sync_write_param.xel[i].id = motor_id_[i];
  }
  is_init_ = true;
  bool ret = is_connected();
  if (!set_torque(false)) ret = false;
  if (ret) {
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
      if (!dxl.writeControlTableItem(ControlTableItem::OPERATING_MODE,
                                     motor_id_[i], 3))
        ret = false;
    }
  }
  is_init_ = ret;
  return ret;
}

bool xStingMotorDriver::is_connected() {
  if (!is_init_) return false;
  is_connected_ = true;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (!dxl.ping(motor_id_[i])) is_connected_ = false;
  }
  return is_connected_;
}

bool xStingMotorDriver::is_ready() const { return is_init_ && is_connected_; }

bool xStingMotorDriver::set_torque(bool onoff) {
  if (!is_init_ || (onoff && !is_ready())) return false;
  bool ret = true;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (!dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE,
                                   motor_id_[i], onoff))
      ret = false;
  }
  // Undo a partially successful enable request.
  if (onoff && !ret) set_torque(false);
  torque_ = onoff && ret;
  return ret;
}

bool xStingMotorDriver::get_torque() const { return torque_; }

void xStingMotorDriver::close() { set_torque(false); }

bool xStingMotorDriver::read_all(uint16_t address, void* values,
                                 uint16_t width) {
  if (!is_ready()) return false;
  sync_read_param.addr = address;
  sync_read_param.length = width;
  read_result.id_count = 0;
  if (!dxl.syncRead(sync_read_param, read_result, 10) ||
      read_result.id_count != MOTOR_COUNT)
    return false;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    const auto& result = read_result.xel[i];
    if (result.id != motor_id_[i] || result.length != width ||
        result.error != 0)
      return false;
  }
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
    memcpy(static_cast<uint8_t*>(values) + i * width, read_result.xel[i].data,
           width);
  return true;
}

bool xStingMotorDriver::write_all(uint16_t address, const void* values,
                                  uint16_t width) {
  if (!is_ready()) return false;
  sync_write_param.addr = address;
  sync_write_param.length = width;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
    memcpy(sync_write_param.xel[i].data,
           static_cast<const uint8_t*>(values) + i * width, width);
  return dxl.syncWrite(sync_write_param);
}

bool xStingMotorDriver::read_present_position(
    joint_position_info_t& position_info) {
  return read_all(132, position_info.value, 4);
}

bool xStingMotorDriver::read_present_velocity(
    joint_velocity_info_t& velocity_info) {
  return read_all(128, velocity_info.value, 4);
}

bool xStingMotorDriver::read_present_current(
    joint_current_info_t& current_info) {
  return read_all(126, current_info.value, 2);
}

bool xStingMotorDriver::write_goal_position_joint(
    const joint_position_info_t& position_info) {
  return write_all(116, position_info.value, 4);
}

bool xStingMotorDriver::write_profile_acceleration_joint(
    const joint_accel_info_t& accel_info) {
  return write_all(108, accel_info.value, 4);
}

bool xStingMotorDriver::write_profile_velocity_joint(
    const joint_accel_info_t& velocity_info) {
  return write_all(112, velocity_info.value, 4);
}
