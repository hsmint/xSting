#include "../include/motor.h"

#include <math.h>
#include <string.h>

namespace mk2_leg {

static Dynamixel2Arduino dxl(Serial3, 84);
static ParamForSyncReadInst_t sync_read;
static ParamForSyncWriteInst_t sync_write;
static RecvInfoFromStatusInst_t read_res;

// Power on DXL bus (3 Mbps with 1 Mbps fallback) and move to standing pose.
bool Motor::Init() {
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  delay(500);

  dxl.begin(kMotorBaud);
  dxl.setPortProtocolVersion(2.0f);
  if (!dxl.ping(kMotorIds[0])) {
    dxl.begin(1000000);
    dxl.setPortProtocolVersion(2.0f);
  }
  sync_read.id_count = kMotorCount;
  sync_write.id_count = kMotorCount;

  for (uint8_t i = 0; i < kMotorCount; ++i) {
    sync_read.xel[i].id = kMotorIds[i];
    sync_write.xel[i].id = kMotorIds[i];
    if (!dxl.ping(kMotorIds[i])) return false;
    dxl.torqueOff(kMotorIds[i]);
    dxl.writeControlTableItem(ControlTableItem::RETURN_DELAY_TIME, kMotorIds[i], 0);
    dxl.writeControlTableItem(ControlTableItem::OPERATING_MODE, kMotorIds[i], 3);
  }
  return kAutoStand ? Stand() : WriteProf(0, 0);
}

// Enable or disable torque on all 9 leg motors.
bool Motor::SetTorque(bool on) {
  for (uint8_t i = 0; i < kMotorCount; ++i) {
    if (!dxl.writeControlTableItem(ControlTableItem::TORQUE_ENABLE, kMotorIds[i], on))
      return false;
  }
  torque_ = on;
  return true;
}

// Smoothly move to standby pose over 2s S-curve, then hold for 200Hz RL control.
bool Motor::Stand() {
  int32_t start[kMotorCount], cmd[kMotorCount];
  WriteProf(0, 0);
  if (!ReadPos(start) || !SetTorque(true)) return false;

  constexpr uint16_t kSteps = 200;
  for (uint16_t s = 1; s <= kSteps; ++s) {
    const float a = 0.5f * (1.0f - cosf(3.14159265f * s / kSteps));
    for (uint8_t i = 0; i < kMotorCount; ++i) {
      cmd[i] = start[i] + static_cast<int32_t>((kStandTicks[i] - start[i]) * a);
    }
    WritePos(cmd);
    delay(10);
  }
  return WritePos(kStandTicks);
}

// Sync read 9 motors from a control table address.
bool Motor::ReadAll(uint16_t addr, void* out, uint16_t width) {
  sync_read.addr = addr;
  sync_read.length = width;
  if (!dxl.syncRead(sync_read, read_res, 5) || read_res.id_count != kMotorCount)
    return false;

  for (uint8_t i = 0; i < kMotorCount; ++i) {
    memcpy(static_cast<uint8_t*>(out) + i * width, read_res.xel[i].data, width);
  }
  return true;
}

// Sync write 9 motors to a control table address.
bool Motor::WriteAll(uint16_t addr, const void* in, uint16_t width) {
  sync_write.addr = addr;
  sync_write.length = width;
  for (uint8_t i = 0; i < kMotorCount; ++i) {
    memcpy(sync_write.xel[i].data, static_cast<const uint8_t*>(in) + i * width, width);
  }
  return dxl.syncWrite(sync_write);
}

bool Motor::ReadPos(int32_t* pos) { return ReadAll(132, pos, 4); }
bool Motor::ReadVel(int32_t* vel) { return ReadAll(128, vel, 4); }
bool Motor::ReadCur(int16_t* cur) { return ReadAll(126, cur, 2); }
bool Motor::WritePos(const int32_t* pos) { return WriteAll(116, pos, 4); }

// Set uniform profile acceleration and velocity on all 9 motors.
bool Motor::WriteProf(uint32_t acc, uint32_t vel) {
  uint32_t acc_arr[kMotorCount], vel_arr[kMotorCount];
  for (uint8_t i = 0; i < kMotorCount; ++i) {
    acc_arr[i] = acc;
    vel_arr[i] = vel;
  }
  return WriteAll(108, acc_arr, 4) && WriteAll(112, vel_arr, 4);
}

}  // namespace mk2_leg
