#pragma once

#include <Arduino.h>

namespace mk2_arm {

class Sensor {
 public:
  float ReadVolt();
  void PlaySound(uint8_t idx);
  void UpdateSound();

 private:
  uint16_t notes_[8] = {};
  uint8_t durs_[8] = {};
  uint8_t note_idx_ = 8;
  uint32_t note_time_ = 0;
  bool playing_ = false;
};

}  // namespace mk2_arm
