#pragma once

#include <Arduino.h>

#define LED_TXD 0
#define LED_RXD 1
#define LED_LOW_BATTERY 2
#define LED_ROS_CONNECT 3
#define LED_WORKING_CHECK 13

#define BATTERY_POWER_OFF 0
#define BATTERY_POWER_STARTUP 1
#define BATTERY_POWER_NORMAL 2
#define BATTERY_POWER_CHECK 3
#define BATTERY_POWER_WARNNING 4

#define WAIT_FOR_BUTTON_PRESS 0
#define WAIT_SECOND 1
#define CHECK_BUTTON_RELEASED 2

enum DEVICE_STATUS {
  STATUS_POWER_OFF = -2,
  STATUS_NOT_CONNECTED_MOTORS = -1,
  STATUS_RUNNING = 0,
  STATUS_LOW_BATTERY = 1
};

class xStingDiagnosis {
 public:
  xStingDiagnosis();
  ~xStingDiagnosis();

  bool init();

  void showLedStatus(bool isConnected);
  void updateRxTxLed(void);

  void setPowerOn(void);
  void setPowerOff(void);

  uint8_t updateVoltageCheck(bool check_setup);

  uint8_t getButtonPress(uint16_t time_to_press);

  bool isPowerOn() const { return !is_power_cut_; }
  bool isLowBattery() const {
    return battery_state_ == BATTERY_POWER_CHECK ||
           battery_state_ == BATTERY_POWER_WARNNING;
  }
  uint8_t getBatteryState() const { return battery_state_; }

 private:
  uint8_t battery_state_ = BATTERY_POWER_OFF;
  bool is_power_cut_ = false;
};