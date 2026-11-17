#pragma once

#include <Arduino.h>
#include <IMU.h>

#include "mk2_leg.h"

namespace mk2_leg {

class Sensor {
 public:
  struct Note {
    uint16_t freq;
    uint16_t ms;
  };

  bool Init();
  void UpdateImu();
  void CalibImu();
  void UpdateSonar();
  float ReadVolt();

  void PlaySound(uint8_t idx);
  void UpdateSound();

  uint8_t imu_state() const { return imu_state_; }
  const float* gyro() const { return gyro_; }
  const float* acc() const { return acc_; }
  const float* quat() const { return quat_; }
  uint32_t sonar_mm() const { return sonar_mm_; }

 private:
  cIMU imu_;
  uint8_t imu_state_ = 0;
  float gyro_[3] = {};
  float acc_[3] = {};
  float quat_[4] = {};
  uint32_t sonar_mm_ = 0;

  const Note* seq_ = nullptr;
  uint8_t seq_len_ = 0;
  uint8_t note_idx_ = 0;
  uint32_t note_time_ = 0;
  bool playing_ = false;
};

}  // namespace mk2_leg
