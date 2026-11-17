#pragma once 

#include <stdint.h>
#include "sensor.h"
#include "diagnosis.h"
#include "open_manipulator_driver.h"

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
const uint32_t INTERVAL_MS_TO_UPDATE_CONTROL_ITEM = 20;

namespace xStingCore{
  void begin();
  void run();
}