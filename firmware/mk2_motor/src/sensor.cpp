/*******************************************************************************
 * Copyright 2016 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#include "../include/sensor.h"

xStingSensors::xStingSensors() {}
xStingSensors::~xStingSensors() {}

bool xStingSensors::init() {
  initBumper();
  initLED();
  initIMU();
  return imu_status_ == IMU_READY;
}

void xStingSensors::initIMU() {
  const uint8_t error = imu_.begin();
  // OpenCR 1.5.3 can return IMU_OK even when no IMU was found.
  imu_status_ =
      error == IMU_OK && imu_.bConnected && imu_.SEN.gyro_cali_get_done()
          ? IMU_READY
          : IMU_ERROR;
}

void xStingSensors::updateIMU() {
  if (!imu_.bConnected) {
    imu_status_ = IMU_ERROR;
    return;
  }
  const bool updated = imu_.update() != 0;
  if (imu_status_ == IMU_CALIBRATING) {
    if (updated && imu_.SEN.gyro_cali_get_done())
      imu_status_ = IMU_READY;
    else if (millis() - calibration_started_ >= 5000)
      imu_status_ = IMU_ERROR;
  }
  if (!updated || imu_status_ != IMU_READY) return;

  for (uint8_t i = 0; i < 3; ++i) {
    angular_velocity_[i] = imu_.SEN.gyroADC[i] * imu_.gRes * DEG_TO_RAD;
    linear_acceleration_[i] = imu_.SEN.accADC[i] * imu_.aRes * 9.80665f;
  }
  for (uint8_t i = 0; i < 4; ++i) orientation_[i] = imu_.quat[i];
}

bool xStingSensors::calibrationGyro() {
  if (!imu_.bConnected) return false;
  if (imu_status_ == IMU_CALIBRATING) return true;
  imu_.SEN.gyro_cali_start();
  calibration_started_ = millis();
  imu_status_ = IMU_CALIBRATING;
  return true;
}

float* xStingSensors::getImuAngularVelocity() { return angular_velocity_; }

float* xStingSensors::getImuLinearAcc() { return linear_acceleration_; }

float* xStingSensors::getOrientation() { return orientation_; }

float xStingSensors::checkVoltage() { return getPowerInVoltage(); }

uint8_t xStingSensors::checkPushButton() { return getPushButton(); }

bool xStingSensors::getBumper1State() { return ollo_.read(3, TOUCH_SENSOR); }

bool xStingSensors::getBumper2State() { return ollo_.read(4, TOUCH_SENSOR); }

void xStingSensors::setLED(uint8_t index, bool enabled) {
  const int pins[] = {led_pin_array_.front_left, led_pin_array_.front_right,
                      led_pin_array_.back_left, led_pin_array_.back_right};
  if (index < 4) digitalWrite(pins[index], enabled ? HIGH : LOW);
}

void xStingSensors::makeMelody(uint8_t index) {
  const uint16_t NOTE_C4 = 262;
  const uint16_t NOTE_D4 = 294;
  const uint16_t NOTE_E4 = 330;
  const uint16_t NOTE_F4 = 349;
  const uint16_t NOTE_G4 = 392;
  const uint16_t NOTE_A4 = 440;
  const uint16_t NOTE_B4 = 494;
  const uint16_t NOTE_C5 = 523;
  const uint16_t NOTE_E5 = 659;   // 미
  const uint16_t NOTE_DS5 = 622;  // 레♯
  const uint16_t NOTE_D5 = 587;   // 레
  // const uint16_t NOTE_C6 = 1047;

  const uint8_t OFF = 0;
  const uint8_t ON = 1;
  const uint8_t LOW_BATTERY = 2;
  const uint8_t ERROR = 3;
  const uint8_t BUTTON1 = 4;
  const uint8_t BUTTON2 = 5;

  // clang-format off
  switch (index)
  {
    case ON:
      melody_note_[0] = NOTE_E5;   melody_duration_[0] = 4;
      melody_note_[1] = NOTE_DS5;   melody_duration_[1] = 4;
      melody_note_[2] = NOTE_E5;   melody_duration_[2] = 4;
      melody_note_[3] = NOTE_DS5;   melody_duration_[3] = 4;
      melody_note_[4] = NOTE_E5;   melody_duration_[4] = 4;
      melody_note_[5] = NOTE_B4;   melody_duration_[5] = 4;
      melody_note_[6] = NOTE_D5;   melody_duration_[6] = 4;
      melody_note_[7] = NOTE_C5;   melody_duration_[7] = 4;
     break;

    case OFF:
      melody_note_[0] = NOTE_C5;   melody_duration_[0] = 4;
      melody_note_[1] = NOTE_B4;   melody_duration_[1] = 4;
      melody_note_[2] = NOTE_A4;   melody_duration_[2] = 4;
      melody_note_[3] = NOTE_G4;   melody_duration_[3] = 4;
      melody_note_[4] = NOTE_F4;   melody_duration_[4] = 4;
      melody_note_[5] = NOTE_E4;   melody_duration_[5] = 4;
      melody_note_[6] = NOTE_D4;   melody_duration_[6] = 4;
      melody_note_[7] = NOTE_C4;   melody_duration_[7] = 4;  
     break;

    case LOW_BATTERY:
      melody_note_[0] = 1000;      melody_duration_[0] = 1;
      melody_note_[1] = 1000;      melody_duration_[1] = 1;
      melody_note_[2] = 1000;      melody_duration_[2] = 1;
      melody_note_[3] = 1000;      melody_duration_[3] = 1;
      melody_note_[4] = 0;         melody_duration_[4] = 8;
      melody_note_[5] = 0;         melody_duration_[5] = 8;
      melody_note_[6] = 0;         melody_duration_[6] = 8;
      melody_note_[7] = 0;         melody_duration_[7] = 8;
     break;

    case ERROR:
      melody_note_[0] = 1000;      melody_duration_[0] = 3;
      melody_note_[1] = 500;       melody_duration_[1] = 3;
      melody_note_[2] = 1000;      melody_duration_[2] = 3;
      melody_note_[3] = 500;       melody_duration_[3] = 3;
      melody_note_[4] = 1000;      melody_duration_[4] = 3;
      melody_note_[5] = 500;       melody_duration_[5] = 3;
      melody_note_[6] = 1000;      melody_duration_[6] = 3;
      melody_note_[7] = 500;       melody_duration_[7] = 3;
     break;

    case BUTTON1:
    case BUTTON2:
    default:
      return;
  }
  // clang-format on

  noTone(BDPIN_BUZZER);
  current_note_number_ = 0;
  is_melody_play_complete_ = false;
  note_playing_ = false;
}

void xStingSensors::onMelody() {
  if (is_melody_play_complete_) return;
  // Same duration convention as mk1_arm: 4 = 250 ms, 8 = 125 ms.
  if (melody_duration_[current_note_number_] == 0) {
    noTone(BDPIN_BUZZER);
    if (++current_note_number_ == 8) is_melody_play_complete_ = true;
    note_playing_ = false;
    return;
  }
  const uint32_t duration = 1000 / melody_duration_[current_note_number_];
  if (!note_playing_) {
    const uint16_t frequency = melody_note_[current_note_number_];
    if (frequency != 0) tone(BDPIN_BUZZER, frequency, duration);
    pre_time_ = millis();
    note_playing_ = true;
  } else if (millis() - pre_time_ >= duration * 13 / 10) {
    noTone(BDPIN_BUZZER);
    if (++current_note_number_ == 8) is_melody_play_complete_ = true;
    note_playing_ = false;
  }
}

void xStingSensors::initBumper() {
  ollo_.begin(3, TOUCH_SENSOR);
  ollo_.begin(4, TOUCH_SENSOR);
}

uint8_t xStingSensors::checkPushBumper() {
  return (getBumper1State() ? 2 : 0) | (getBumper2State() ? 1 : 0);
}

void xStingSensors::initLED() {
  led_pin_array_.front_left = BDPIN_GPIO_4;
  led_pin_array_.front_right = BDPIN_GPIO_6;
  led_pin_array_.back_left = BDPIN_GPIO_8;
  led_pin_array_.back_right = BDPIN_GPIO_10;
  const int pins[] = {led_pin_array_.front_left, led_pin_array_.front_right,
                      led_pin_array_.back_left, led_pin_array_.back_right};
  for (int pin : pins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
}

void xStingSensors::stopMelody() {
  noTone(BDPIN_BUZZER);
  is_melody_play_complete_ = true;
  note_playing_ = false;
}
