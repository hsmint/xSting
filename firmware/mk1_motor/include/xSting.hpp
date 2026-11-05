#pragma once

#include <stdint.h>
#include "xSting_motor_driver.hpp"
#include "xSting_sensor.hpp"
#include "xSting_controller.hpp"
#include "xSting_diagnosis.hpp"

#define DEBUG_ENABLE 1

#if DEBUG_ENABLE
  #define DEBUG_SERIAL_BEGIN(x) SerialBT2.begin(x)
  #define DEBUG_PRINT(x) SerialBT2.print(x)
  #define DEBUG_PRINTLN(x) SerialBT2.println(x)
#else
  #define DEBUG_SERIAL_BEGIN(x) 
  #define DEBUG_PRINT(x) 
  #define DEBUG_PRINTLN(x) 
#endif

const uint8_t FIRMWARE_VER = 5;
const uint32_t MODEL_INFO = 3;
const uint32_t INTERVAL_MS_TO_CONTROL_MOTOR = 20;
const uint32_t INTERVAL_MS_TO_UPDATE_CONTROL_ITEM = 20;

namespace xStingMotor{
  void begin();
  void run();
}
