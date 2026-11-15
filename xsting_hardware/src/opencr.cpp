#include "xsting_hardware/opencr.hpp"

#include <algorithm>
#include <filesystem>

namespace xsting_hardware {

OpenCrBase::OpenCrBase(uint8_t id, uint16_t table_size)
    : id_(id),
      table_size_(table_size),
      pkt_(dynamixel::PacketHandler::getPacketHandler(2.0f)),
      buf_(table_size, 0) {}

OpenCrBase::~OpenCrBase() { Close(); }

bool OpenCrBase::Open(const std::string& port, uint32_t baud) {
  Close();
  port_.reset(dynamixel::PortHandler::getPortHandler(port.c_str()));
  if (port_->openPort() && port_->setBaudRate(baud)) {
    port_path_ = port;
    return true;
  }
  Close();
  return false;
}

void OpenCrBase::Close() {
  if (port_) {
    port_->closePort();
    port_.reset();
  }
  port_path_.clear();
}

bool OpenCrBase::Connect(const std::string& port, uint32_t baud,
                         const std::vector<std::string>& skip_ports) {
  if (port != "auto") {
    if (Open(port, baud) && Ping()) return true;
    Close();
    return false;
  }

  for (const auto& entry : std::filesystem::directory_iterator("/dev")) {
    if (entry.path().filename().string().rfind("ttyACM", 0) != 0) continue;
    const std::string p = entry.path().string();
    if (std::find(skip_ports.begin(), skip_ports.end(), p) != skip_ports.end()) {
      continue;
    }
    if (Open(p, baud) && Ping()) return true;
    Close();
  }
  return false;
}

bool OpenCrBase::Ping() {
  if (!port_) return false;
  uint16_t model = 0;
  uint8_t err = 0;
  return pkt_->ping(port_.get(), id_, &model, &err) == COMM_SUCCESS && err == 0;
}

// 보드가 즉시 NVIC_SystemReset() 되므로 응답을 기다리지 않고 포트를 바로 닫아 반환한다
void OpenCrBase::Reset() {
  if (!port_) return;
  uint8_t val = 1;
  pkt_->writeTxOnly(port_.get(), id_, ADDR_RESET, 1, &val);
  Close();
}

bool OpenCrBase::SetTorque(bool on) {
  const uint8_t val = on ? 1 : 0;
  return Write(ADDR_TORQUE, 1, &val);
}

bool OpenCrBase::PlaySound(uint8_t idx) { return Write(ADDR_SOUND, 1, &idx); }

bool OpenCrBase::ReadAll() {
  if (!port_) return false;
  uint8_t err = 0;
  return pkt_->readTxRx(port_.get(), id_, 0, table_size_, buf_.data(), &err) ==
             COMM_SUCCESS &&
         err == 0;
}

bool OpenCrBase::Write(uint16_t addr, uint16_t len, const uint8_t* data) {
  if (!port_) return false;
  uint8_t err = 0;
  return pkt_->writeTxRx(port_.get(), id_, addr, len,
                         const_cast<uint8_t*>(data), &err) == COMM_SUCCESS;
}

bool LegBoard::WritePos(const Array& rad) {
  uint8_t data[kLegJoints * 4 + 1];
  for (size_t i = 0; i < kLegJoints; ++i) {
    if (!std::isfinite(rad[i])) return false;
    const int32_t t = std::clamp(
        static_cast<int32_t>(std::lround(rad[i] / kTickToRad)) + kCenterTick, 0,
        4095);
    std::memcpy(&data[i * 4], &t, 4);
  }
  data[kLegJoints * 4] = 1;
  return Write(ADDR_GOAL_POS, sizeof(data), data);
}

LegBoard::Array LegBoard::GetPos() const {
  Array pos{};
  for (size_t i = 0; i < kLegJoints; ++i) {
    pos[i] = (Get<int32_t>(ADDR_LEG_PRESENT_POS + i * 4) - kCenterTick) *
             kTickToRad;
  }
  return pos;
}

LegBoard::Array LegBoard::GetVel() const {
  Array vel{};
  for (size_t i = 0; i < kLegJoints; ++i) {
    vel[i] = Get<int32_t>(ADDR_LEG_PRESENT_VEL + i * 4) * kVelToRad;
  }
  return vel;
}

LegBoard::CurArray LegBoard::GetCur() const {
  CurArray cur{};
  for (size_t i = 0; i < kLegJoints; ++i) {
    cur[i] = Get<int16_t>(ADDR_LEG_PRESENT_CUR + i * 2);
  }
  return cur;
}

Imu LegBoard::GetImu() const {
  Imu imu{};
  for (uint8_t i = 0; i < 4; ++i) {
    imu.quat[i] = Get<float>(ADDR_LEG_QUAT_W + i * 4);
  }
  for (uint8_t i = 0; i < 3; ++i) {
    imu.gyro[i] = Get<float>(ADDR_LEG_GYRO_X + i * 4);
    imu.acc[i] = Get<float>(ADDR_LEG_ACC_X + i * 4);
  }
  return imu;
}

double LegBoard::GetRange() const {
  const uint32_t mm = ultrasonic_mm();
  return mm > 0 ? mm * 0.001 : std::numeric_limits<double>::infinity();
}

bool ArmBoard::WritePos(const Array& cmd) {
  uint8_t data[kArmJoints * 4 + 1];
  for (size_t i = 0; i < kArmJoints; ++i) {
    if (!std::isfinite(cmd[i])) return false;
    const double rad = (i == 5) ? (cmd[i] / kGripperRadToM) : cmd[i];
    const int32_t t = std::clamp(
        static_cast<int32_t>(std::lround(rad / kTickToRad)) + kCenterTick, 0,
        4095);
    std::memcpy(&data[i * 4], &t, 4);
  }
  data[kArmJoints * 4] = 1;
  return Write(ADDR_GOAL_POS, sizeof(data), data);
}

bool ArmBoard::WriteGripperCur(int16_t cur) {
  return Write(ADDR_ARM_GRIPPER_CUR, 2, reinterpret_cast<const uint8_t*>(&cur));
}

ArmBoard::Array ArmBoard::GetPos() const {
  Array pos{};
  for (size_t i = 0; i < kArmJoints; ++i) {
    const double rad =
        (Get<int32_t>(ADDR_ARM_PRESENT_POS + i * 4) - kCenterTick) * kTickToRad;
    pos[i] = (i == 5) ? (rad * kGripperRadToM) : rad;
  }
  return pos;
}

ArmBoard::Array ArmBoard::GetVel() const {
  Array vel{};
  for (size_t i = 0; i < kArmJoints; ++i) {
    const double w = Get<int32_t>(ADDR_ARM_PRESENT_VEL + i * 4) * kVelToRad;
    vel[i] = (i == 5) ? (w * kGripperRadToM) : w;
  }
  return vel;
}

}  // namespace xsting_hardware
