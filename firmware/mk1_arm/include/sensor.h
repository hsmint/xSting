#pragma once

#include "OLLO.h"

typedef struct LED_PIN_ARRAY
{
  int front_left;
  int front_right;
  int back_left;
  int back_right;
}LedPinArray;
 
class xStingSensors
{
 public:
  xStingSensors();
  ~xStingSensors();

  bool init(void);

  // Battery
  float checkVoltage(void);

  // Button
  uint8_t checkPushButton(void);

  // Sound
  void onMelody();
  void makeMelody(uint8_t index);  

  // Bumper
  void initBumper(void);
  bool getBumper1State();
  bool getBumper2State();
  uint8_t checkPushBumper(void);

  void initLED(void);
  
 private:
  OLLO ollo_;

  LedPinArray led_pin_array_;

  bool is_melody_play_complete_;
  uint16_t melody_note_[8];
  uint8_t melody_duration_[8];
};
