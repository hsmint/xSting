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

xStingSensors::xStingSensors()
{}

  xStingSensors::~xStingSensors()
{}

bool xStingSensors::init(void)
{
  initBumper();
  initLED();

  return true;
}

float xStingSensors::checkVoltage(void)
{
  float vol_value;
  
  vol_value = getPowerInVoltage();

  return vol_value;
}

uint8_t xStingSensors::checkPushButton(void)
{
  return getPushButton();
}

void xStingSensors::onMelody()
{
  static uint8_t pre_note_number = 7, current_note_number;
  static uint32_t pre_time;
  static uint32_t note_duration, pause_time_ms_between_notes;

  if(is_melody_play_complete_ == false){
    if(pre_note_number != current_note_number){
      // to calculate the note duration, take one second
      // divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      note_duration = 1000/melody_duration_[current_note_number];    
      // to distinguish the notes, set a minimum time between them.
      // the note's duration + 30% seems to work well:
      pause_time_ms_between_notes = note_duration * 1.30;

      tone(BDPIN_BUZZER, melody_note_[current_note_number], note_duration);
      
      pre_note_number = current_note_number;
      pre_time = millis();
    }else{
      if(millis()-pre_time >= pause_time_ms_between_notes){
        pre_time = millis();
        noTone(BDPIN_BUZZER);
        current_note_number++;

        if(current_note_number == 8){
          is_melody_play_complete_ = true;
          current_note_number = 0;
        }
      }
    }
  }
}

void xStingSensors::makeMelody(uint8_t index)
{
  const uint16_t NOTE_C4 = 262;
  const uint16_t NOTE_D4 = 294;
  const uint16_t NOTE_E4 = 330;
  const uint16_t NOTE_F4 = 349;
  const uint16_t NOTE_G4 = 392;
  const uint16_t NOTE_A4 = 440;
  const uint16_t NOTE_B4 = 494;
  const uint16_t NOTE_C5 = 523;
  const uint16_t NOTE_E5  = 659;  // 미
  const uint16_t NOTE_DS5 = 622;  // 레♯
  const uint16_t NOTE_D5  = 587;  // 레
  //const uint16_t NOTE_C6 = 1047;

  const uint8_t OFF         = 0;
  const uint8_t ON          = 1;
  const uint8_t LOW_BATTERY = 2;
  const uint8_t ERROR       = 3;
  const uint8_t BUTTON1     = 4;
  const uint8_t BUTTON2     = 5;

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

  is_melody_play_complete_ = false;
}

void xStingSensors::initBumper(void)
{
  ollo_.begin(3, TOUCH_SENSOR);
  ollo_.begin(4, TOUCH_SENSOR);
}

uint8_t xStingSensors::checkPushBumper(void)
{
  uint8_t push_state = 0;

  if      (ollo_.read(3, TOUCH_SENSOR) == HIGH) push_state = 2;
  else if (ollo_.read(4, TOUCH_SENSOR) == HIGH) push_state = 1;
  else    push_state = 0;
  
  return push_state;
}

bool xStingSensors::getBumper1State()
{
  return ollo_.read(3, TOUCH_SENSOR); 
}

bool xStingSensors::getBumper2State()
{
  return ollo_.read(4, TOUCH_SENSOR);
}

void xStingSensors::initLED(void)
{
  led_pin_array_.front_left  = BDPIN_GPIO_4;
  led_pin_array_.front_right = BDPIN_GPIO_6;
  led_pin_array_.back_left   = BDPIN_GPIO_8;
  led_pin_array_.back_right  = BDPIN_GPIO_10;
 
  pinMode(led_pin_array_.front_left, OUTPUT);
  pinMode(led_pin_array_.front_right, OUTPUT);
  pinMode(led_pin_array_.back_left, OUTPUT);
  pinMode(led_pin_array_.back_right, OUTPUT);
}
