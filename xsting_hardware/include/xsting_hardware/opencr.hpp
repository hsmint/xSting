#ifndef XSTING_HARDWARE_OPENCR_HPP_
#define XSTING_HARDWARE_OPENCR_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace xsting_hardware {

// OpenCR 보드 ID: 앞다리(200), 뒷다리(201), 팔(202)
constexpr uint8_t kFrontId = 200;
constexpr uint8_t kRearId = 201;
constexpr uint8_t kArmId = 202;

constexpr size_t kLegJoints = 9;
constexpr size_t kArmJoints = 6;
constexpr uint16_t kLegTableSize = 304;
constexpr uint16_t kArmTableSize = 147;
constexpr uint32_t kDefaultBaud = 1000000;

constexpr int32_t kCenterTick = 2048;
constexpr double kTickToRad = 2.0 * M_PI / 4096.0;
constexpr double kVelToRad = 2.0 * M_PI / 60.0 * 0.229;
constexpr double kGripperRadToM = 0.015;

// 다리 주행 시작 기립 자세 (모터 좌표계 rad; URDF yaw +-30/0°, hip +15°, knee -90°)
constexpr std::array<double, kLegJoints> kStandFront = {
    -M_PI / 3.0, 15.0 * M_PI / 180.0, -M_PI / 2.0,
    -M_PI / 2.0, 15.0 * M_PI / 180.0, -M_PI / 2.0,
    -M_PI / 3.0, 15.0 * M_PI / 180.0, -M_PI / 2.0};
constexpr std::array<double, kLegJoints> kStandRear = {
    M_PI / 3.0, 15.0 * M_PI / 180.0, -M_PI / 2.0,
    M_PI / 2.0, 15.0 * M_PI / 180.0, -M_PI / 2.0,
    M_PI / 3.0, 15.0 * M_PI / 180.0, -M_PI / 2.0};

constexpr std::array<uint8_t, kLegJoints> kFrontDxlIds = {1, 2,  3,  4, 5,
                                                          6, 10, 11, 12};
constexpr std::array<uint8_t, kLegJoints> kRearDxlIds = {7,  8,  9,  13, 14,
                                                         15, 16, 17, 18};

// 다리 펌웨어 초기 대기 자세 (yaw +-45/90 deg, hip +90 deg 접음, knee 0 deg 폄)
constexpr std::array<double, kLegJoints> kWaitFront = {
    -M_PI / 4.0, M_PI / 2.0, 0.0,
    -M_PI / 2.0, M_PI / 2.0, 0.0,
    -M_PI / 4.0, M_PI / 2.0, 0.0};
constexpr std::array<double, kLegJoints> kWaitRear = {
    M_PI / 4.0, M_PI / 2.0, 0.0,
    M_PI / 2.0, M_PI / 2.0, 0.0,
    M_PI / 4.0, M_PI / 2.0, 0.0};

// 팔 접힌 홈 자세 (모터 좌표계 rad / 그리퍼 m)
constexpr std::array<double, kArmJoints> kArmHome = {
    0.0, -1.0508, 1.0707, 0.0, 0.0, 0.0};

// mk2_leg, mk2_arm 펌웨어 공통 및 전용 컨트롤 테이블 주소
enum Addr : uint16_t {
  ADDR_CONNECT_MOTORS = 16,
  ADDR_DEVICE_STATUS = 18,
  ADDR_HEARTBEAT = 19,
  ADDR_RESET = 25,
  ADDR_BATTERY_V = 42,
  ADDR_SOUND = 50,
  ADDR_TORQUE = 59,
  ADDR_GOAL_POS = 60,

  // 다리 보드 (9축 + IMU + 전면 초음파)
  ADDR_LEG_ULTRASONIC_MM = 30,
  ADDR_LEG_PRESENT_POS = 97,
  ADDR_LEG_PRESENT_VEL = 133,
  ADDR_LEG_PRESENT_CUR = 169,
  ADDR_LEG_IMU_STATUS = 262,
  ADDR_LEG_GYRO_X = 264,
  ADDR_LEG_ACC_X = 276,
  ADDR_LEG_QUAT_W = 288,

  // 팔 보드 (6축 + 그리퍼 전류)
  ADDR_ARM_PRESENT_POS = 85,
  ADDR_ARM_PRESENT_VEL = 109,
  ADDR_ARM_PRESENT_CUR = 133,
  ADDR_ARM_GRIPPER_CUR = 145,
};

struct Imu {
  double quat[4];
  double gyro[3];
  double acc[3];
};

// OpenCR USB DYNAMIXEL Slave 공통 통신 클래스
class OpenCrBase {
 public:
  OpenCrBase(uint8_t id, uint16_t table_size);
  ~OpenCrBase();

  OpenCrBase(const OpenCrBase&) = delete;
  OpenCrBase& operator=(const OpenCrBase&) = delete;

  bool Connect(const std::string& port, uint32_t baud,
               const std::vector<std::string>& skip_ports = {});
  bool Ping();
  void Reset();
  bool SetTorque(bool on);
  bool PlaySound(uint8_t idx);
  bool ReadAll();

  uint8_t id() const { return id_; }
  const std::string& port_path() const { return port_path_; }
  bool motors_ok() const { return Get<uint8_t>(ADDR_CONNECT_MOTORS) != 0; }
  int8_t status() const { return Get<int8_t>(ADDR_DEVICE_STATUS); }
  double bat_volt() const { return Get<uint32_t>(ADDR_BATTERY_V) * 0.01; }

 protected:
  template <typename T>
  T Get(uint16_t addr) const {
    T val{};
    std::memcpy(&val, &buf_[addr], sizeof(T));
    return val;
  }

  bool Open(const std::string& port, uint32_t baud);
  void Close();
  bool Write(uint16_t addr, uint16_t len, const uint8_t* data);

 private:
  uint8_t id_;
  uint16_t table_size_;
  std::string port_path_;
  std::unique_ptr<dynamixel::PortHandler> port_;
  dynamixel::PacketHandler* pkt_ = nullptr;
  std::vector<uint8_t> buf_;
};

// 6족 다리 한쪽(앞다리 ID 200 또는 뒷다리 ID 201) OpenCR 보드
class LegBoard : public OpenCrBase {
 public:
  using Array = std::array<double, kLegJoints>;
  using CurArray = std::array<int16_t, kLegJoints>;

  explicit LegBoard(uint8_t id) : OpenCrBase(id, kLegTableSize) {}

  bool WritePos(const Array& rad);
  Array GetPos() const;
  Array GetVel() const;
  CurArray GetCur() const;
  const std::array<uint8_t, kLegJoints>& DxlIds() const {
    return id() == kFrontId ? kFrontDxlIds : kRearDxlIds;
  }
  bool imu_ready() const { return Get<uint8_t>(ADDR_LEG_IMU_STATUS) == 2; }
  Imu GetImu() const;
  uint32_t ultrasonic_mm() const {
    return Get<uint32_t>(ADDR_LEG_ULTRASONIC_MM);
  }
  double GetRange() const;
  const Array& StandPose() const {
    return id() == kFrontId ? kStandFront : kStandRear;
  }
  const Array& WaitPose() const {
    return id() == kFrontId ? kWaitFront : kWaitRear;
  }
};

// 6자유도 팔 + 그리퍼(ID 202) OpenCR 보드
class ArmBoard : public OpenCrBase {
 public:
  using Array = std::array<double, kArmJoints>;

  ArmBoard() : OpenCrBase(kArmId, kArmTableSize) {}

  bool WritePos(const Array& cmd);
  bool WriteGripperCur(int16_t cur);
  bool SetHomePose() { return WritePos(kArmHome); }
  Array GetPos() const;
  Array GetVel() const;
  int16_t grip_cur() const {
    return Get<int16_t>(ADDR_ARM_PRESENT_CUR + 5 * 2);
  }
};

}  // namespace xsting_hardware

#endif  // XSTING_HARDWARE_OPENCR_HPP_
