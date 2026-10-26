#include "xsting_base_hardware/opencr_base.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

namespace xsting_base_hardware
{
// 예시: 팔 opencr.cpp:28-31과 같아요.
OpenCRBase::OpenCRBase(const uint8_t & id)
{
  dxl_sdk_wrapper_ = std::make_unique<xsting_manipulation_hardware::DynamixelSDKWrapper>(id);
}

OpenCRBase::~OpenCRBase()
{
  if (is_open_) {
    // TODO B: 바퀴를 멈추세요. 이 파일 맨 아래 함수 하나를 부르면 돼요.
    stop();
  }
}

bool OpenCRBase::open(const std::string & usb_port, const uint32_t & baud_rate)
{
  // TODO C: 래퍼의 open_port(usb_port)와 set_baud_rate(baud_rate)가 "둘 다" 성공해야 true.
  //         팔 opencr.cpp:97, 102에서 래퍼를 어떻게 부르는지 보세요.
  if (dxl_sdk_wrapper_->open_port(usb_port) && dxl_sdk_wrapper_->set_baud_rate(baud_rate))
    is_open_ = true;
  return is_open_;
}

// 예시: 팔 opencr.cpp:105-108과 같아요.
uint16_t OpenCRBase::ping()
{
  return dxl_sdk_wrapper_->ping();
}

bool OpenCRBase::is_connect_motors()
{
  // TODO D1: 팔 opencr.cpp:112와 같은 모양. 주소는 base_control_table.connect_motors
  return dxl_sdk_wrapper_->read_byte(base_control_table.connect_motors.address);
}

void OpenCRBase::motors_torque(uint8_t onoff)
{
  // TODO D2: 팔 opencr.cpp:141과 같은 모양. 주소는 base_control_table.torque_motors
  dxl_sdk_wrapper_->write_byte(base_control_table.torque_motors.address, onoff);
}

// 예시: 팔 opencr.cpp:115-118과 같아요.
void OpenCRBase::play_sound(uint8_t sound)
{
  dxl_sdk_wrapper_->write_byte(base_control_table.sound.address, sound);
}

void OpenCRBase::send_heartbeat(const uint8_t & count)
{
  // TODO D3: 팔 opencr.cpp:475와 같은 일. 주소는 base_control_table.heartbeat
  dxl_sdk_wrapper_->write_byte(base_control_table.heartbeat.address, count);
}

bool OpenCRBase::read_all()
{
  // TODO F: 팔 opencr.cpp:146-153 몸통을 그대로 복사해서 이 줄과 바꾸세요.
  //         CONTROL_TABLE_SIZE, data_buffer_, data_, buffer_m_ 이름이 팔과 같아서 고칠 곳이 없어요.
  bool comm_result = dxl_sdk_wrapper_->read(0, CONTROL_TABLE_SIZE, &data_buffer_[0]);

  if (comm_result) {
    std::lock_guard<std::mutex> lock(buffer_m_);
    std::copy(data_buffer_, data_buffer_ + CONTROL_TABLE_SIZE, data_);
  }

  return comm_result;
}

OpenCRBase::WheelArray OpenCRBase::get_wheel_positions()
{
  WheelArray positions = {};

  for (uint8_t i = 0; i < base::wheels::NUM; i++) {
    // TODO E1: i번째 바퀴의 tick을 읽어서 rad로 바꾸세요.
    //          읽기: 팔 opencr.cpp:72-73처럼 get_data<int32_t>(첫 바퀴 주소 + i * 4, 4)
    //          첫 바퀴 주소: base_control_table.present_position_left_1.address
    //          변환: base::wheels::TICK_TO_RAD
    int32_t tick = get_data<int32_t>(base_control_table.present_position_left_1.address + i * 4, 4);
    positions[i] = base::wheels::TICK_TO_RAD * tick;
  }

  return positions;
}

OpenCRBase::WheelArray OpenCRBase::get_wheel_velocities()
{
  WheelArray velocities = {};

  for (uint8_t i = 0; i < base::wheels::NUM; i++) {
    // TODO E2: E1과 같은 모양. 주소는 present_velocity_left_1, 변환은 VELOCITY_UNIT_TO_RAD_PER_SEC
    int32_t vel = get_data<int32_t>(base_control_table.present_velocity_left_1.address + i * 4, 4);
    velocities[i] = base::wheels::VELOCITY_UNIT_TO_RAD_PER_SEC * vel;
  }

  return velocities;
}

bool OpenCRBase::write_int32(const uint16_t & address, const int32_t & value)
{
  uint8_t data[4];
  // TODO G: value의 4바이트를 data 배열로 복사하세요 (std::memcpy).
  //         헤더의 get_data()는 버퍼 -> 변수 방향이고, 여기는 반대 방향이에요.
  std::memcpy(&data, &value, 4);
  return dxl_sdk_wrapper_->write(address, sizeof(data), data);
}

bool OpenCRBase::set_wheel_velocities(const WheelArray & rad_per_sec)
{
  using base::wheels::LEFT_1;
  using base::wheels::LEFT_2;
  using base::wheels::RIGHT_1;
  using base::wheels::RIGHT_2;

  // TODO H1: 왼쪽 두 바퀴 각속도(rad/s)의 평균 x 바퀴 반지름 = 왼쪽 선속도(m/s). 오른쪽도 같은 방식.
  //          꺼내는 법: rad_per_sec[LEFT_1], 반지름: base::wheels::WHEEL_RADIUS
  const double left = (rad_per_sec[LEFT_1] + rad_per_sec[LEFT_2]) / 2 * base::wheels::WHEEL_RADIUS;
  const double right = (rad_per_sec[RIGHT_1] + rad_per_sec[RIGHT_2]) / 2 * base::wheels::WHEEL_RADIUS;

  // TODO H2: 로봇 선속도 = 좌우 평균
  //          로봇 각속도 = (오른쪽 - 왼쪽) / base::wheels::FIRMWARE_WHEEL_SEPARATION
  const double linear = (left + right) / 2;
  const double angular = (right - left) / base::wheels::FIRMWARE_WHEEL_SEPARATION;

  // TODO H3: 값 x base::wheels::CMD_VEL_SCALE 를 std::lround로 반올림하고 static_cast<int32_t>로 바꾸세요.
  //          그냥 (int32_t)로 바꾸면 0.009 같은 값이 버림돼서 0이 돼요.
  const bool ok_linear = write_int32(
    base_control_table.cmd_velocity_linear_x.address, static_cast<int32_t>(std::lround(linear * base::wheels::CMD_VEL_SCALE)));
  const bool ok_angular = write_int32(
    base_control_table.cmd_velocity_angular_z.address, static_cast<int32_t>(std::lround(angular * base::wheels::CMD_VEL_SCALE)));

  return ok_linear && ok_angular;
}

bool OpenCRBase::stop()
{
  // TODO I: 네 바퀴 모두 0인 WheelArray를 set_wheel_velocities에 넘기세요. WheelArray{} 는 전부 0이에요.
  return set_wheel_velocities(WheelArray{});
}
}  // namespace xsting_base_hardware
