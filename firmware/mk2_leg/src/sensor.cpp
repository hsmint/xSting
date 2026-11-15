#include "../include/sensor.h"

namespace mk2_leg {

namespace {

enum EchoState : uint8_t {
  ECHO_IDLE = 0,
  ECHO_WAIT_RISE = 1,
  ECHO_WAIT_FALL = 2,
  ECHO_COMPLETE = 3,
};

volatile EchoState echo_state = ECHO_IDLE;
volatile uint32_t echo_start_us = 0;
volatile uint32_t echo_width_us = 0;
volatile uint32_t trig_time_us = 0;

void OnSonarEcho() {
  const uint32_t now = micros();
  if (now - trig_time_us >= kSonarTimeoutUs) return;

  if (echo_state == ECHO_WAIT_RISE && digitalRead(kSonarEchoPin) == HIGH) {
    echo_start_us = now;
    echo_state = ECHO_WAIT_FALL;
  } else if (echo_state == ECHO_WAIT_FALL && digitalRead(kSonarEchoPin) == LOW) {
    echo_width_us = now - echo_start_us;
    echo_state = ECHO_COMPLETE;
  }
}

}  // namespace

// Initialize IMU (and front HC-SR04 ultrasonic sensor on ID 200).
bool Sensor::Init() {
  if (kIsFront) {
    detachInterrupt(digitalPinToInterrupt(kSonarEchoPin));
    digitalWrite(kSonarTrigPin, LOW);
    pinMode(kSonarTrigPin, OUTPUT);
    pinMode(kSonarEchoPin, INPUT_PULLDOWN);
    echo_state = ECHO_IDLE;
    echo_start_us = 0;
    echo_width_us = 0;
    sonar_mm_ = 0;
    trig_time_us = micros();
    attachInterrupt(digitalPinToInterrupt(kSonarEchoPin), OnSonarEcho, CHANGE);
  }

  imu_state_ = (imu_.begin() == IMU_OK && imu_.bConnected) ? 2 : 0;
  return imu_state_ == 2;
}

// Update IMU readings in SI units (rad/s, m/s^2, quaternion).
void Sensor::UpdateImu() {
  if (!imu_.bConnected || !imu_.update()) return;
  if (imu_state_ == 1 && imu_.SEN.gyro_cali_get_done()) imu_state_ = 2;
  if (imu_state_ != 2) return;

  for (uint8_t i = 0; i < 3; ++i) {
    gyro_[i] = imu_.SEN.gyroADC[i] * imu_.gRes * DEG_TO_RAD;
    acc_[i] = imu_.SEN.accADC[i] * imu_.aRes * 9.80665f;
  }
  for (uint8_t i = 0; i < 4; ++i) quat_[i] = imu_.quat[i];
}

// Start gyroscope calibration.
void Sensor::CalibImu() {
  if (!imu_.bConnected) return;
  imu_.SEN.gyro_cali_start();
  imu_state_ = 1;
}

// Non-blocking HC-SR04 ultrasonic trigger and pulse width conversion (~15Hz, 20..4000mm).
void Sensor::UpdateSonar() {
  if (!kIsFront) return;

  noInterrupts();
  const uint32_t now = micros();
  const EchoState state = echo_state;
  const uint32_t width = echo_width_us;
  const uint32_t elapsed = now - trig_time_us;
  const bool timed_out =
      (state == ECHO_WAIT_RISE || state == ECHO_WAIT_FALL) && elapsed >= kSonarTimeoutUs;
  if (state == ECHO_COMPLETE || timed_out) echo_state = ECHO_IDLE;
  const bool idle = (echo_state == ECHO_IDLE);
  interrupts();

  if (state == ECHO_COMPLETE) {
    sonar_mm_ = (width >= 116 && width <= 23200) ? (width * 10 + 29) / 58 : 0;
  } else if (timed_out) {
    sonar_mm_ = 0;
  }

  if (!idle || elapsed < kSonarIntervalUs) return;

  if (digitalRead(kSonarEchoPin) == HIGH) {
    sonar_mm_ = 0;
    return;
  }

  noInterrupts();
  trig_time_us = micros();
  echo_state = ECHO_WAIT_RISE;
  interrupts();

  digitalWrite(kSonarTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(kSonarTrigPin, LOW);
}

float Sensor::ReadVolt() { return getPowerInVoltage(); }

// Start Super Mario Bros. intro (idx 1) or error sound (idx 3).
void Sensor::PlaySound(uint8_t idx) {
  noTone(BDPIN_BUZZER);
  static constexpr Note kMario[] = {
      {659, 125}, {659, 125}, {0, 125}, {659, 125},
      {0, 125},   {523, 125}, {659, 250}, {784, 250},
      {0, 250},   {392, 250},
  };
  static constexpr Note kErr[] = {
      {1000, 250}, {500, 250}, {1000, 250}, {500, 250},
      {1000, 250}, {500, 250}, {1000, 250}, {500, 250},
  };

  if (idx == 1) {
    seq_ = kMario;
    seq_len_ = sizeof(kMario) / sizeof(kMario[0]);
  } else if (idx == 3) {
    seq_ = kErr;
    seq_len_ = sizeof(kErr) / sizeof(kErr[0]);
  } else {
    seq_ = nullptr;
    seq_len_ = 0;
  }
  note_idx_ = 0;
  playing_ = false;
}

// Non-blocking buzzer update loop.
void Sensor::UpdateSound() {
  if (!seq_ || note_idx_ >= seq_len_) return;
  const Note& cur = seq_[note_idx_];

  if (!playing_) {
    if (cur.freq > 0) tone(BDPIN_BUZZER, cur.freq, cur.ms * 9 / 10);
    note_time_ = millis();
    playing_ = true;
  } else if (millis() - note_time_ >= cur.ms) {
    noTone(BDPIN_BUZZER);
    ++note_idx_;
    playing_ = false;
  }
}

}  // namespace mk2_leg
