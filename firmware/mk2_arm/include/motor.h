#pragma once

#include <Dynamixel2Arduino.h>

#include "mk2_arm.h"

namespace mk2_arm {

class Motor {
 public:
  bool Init();
  bool SetTorque(bool on);
  bool Home();

  bool ReadPos(int32_t* pos);
  bool ReadVel(int32_t* vel);
  bool ReadCur(int16_t* cur);
  bool WritePos(const int32_t* pos);
  bool WriteGripperCur(int16_t cur);
  bool WriteProf(uint32_t acc, uint32_t vel);

  bool torque() const { return torque_; }

 private:
  bool ReadAll(uint16_t addr, void* out, uint16_t width);
  bool WriteAll(uint16_t addr, const void* in, uint16_t width);

  bool torque_ = false;
};

}  // namespace mk2_arm
