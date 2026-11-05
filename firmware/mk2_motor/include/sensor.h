#pragma once

#include <IMU.h>
#include <OLLO.h>

typedef struct LED_PIN_ARRAY {
  int front_left;
  int front_right;
  int back_left;
  int back_right;
} LedPinArray;

class xStingSensors {
 public:
  enum ImuStatus : uint8_t {
    IMU_ERROR = 0,
    IMU_CALIBRATING = 1,
    IMU_READY = 2
  };

  xStingSensors();
  ~xStingSensors();
  bool init();

  // IMU
  void initIMU();
  void updateIMU();
  bool calibrationGyro();
  ImuStatus getIMUStatus() const { return imu_status_; }
  float* getImuAngularVelocity();
  float* getImuLinearAcc();
  float* getOrientation();
  // Battery and buttons
  float checkVoltage();
  uint8_t checkPushButton();
  // Bumper
  void initBumper();
  uint8_t checkPushBumper();
  bool getBumper1State();
  bool getBumper2State();
  // LEDs and sound
  void initLED();
  void stopMelody();
  void setLED(uint8_t index, bool enabled);
  void makeMelody(uint8_t index);
  void onMelody();

 private:
  cIMU imu_;
  ImuStatus imu_status_ = IMU_ERROR;
  uint32_t calibration_started_ = 0;
  float angular_velocity_[3] = {};
  float linear_acceleration_[3] = {};
  float orientation_[4] = {};
  OLLO ollo_;
  LedPinArray led_pin_array_;
  uint16_t melody_note_[8] = {};
  uint8_t melody_duration_[8] = {};
  bool is_melody_play_complete_ = true;
  uint8_t current_note_number_ = 0;
  uint32_t pre_time_ = 0;
  bool note_playing_ = false;
};
