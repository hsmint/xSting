#include "../include/sensor.h"

namespace mk2_arm {

float Sensor::ReadVolt() { return getPowerInVoltage(); }

// Start MapleStory login theme (idx 1) or error sound (idx 3).
void Sensor::PlaySound(uint8_t idx) {
  static constexpr uint16_t kMapleNotes[8] = {494, 330, 494, 440, 392, 392, 440, 494};
  static constexpr uint8_t kMapleDurs[8] = {4, 2, 8, 8, 2, 4, 4, 2};
  static constexpr uint16_t kErrNotes[8] = {1000, 500, 1000, 500, 1000, 500, 1000, 500};
  static constexpr uint8_t kErrDurs[8] = {3, 3, 3, 3, 3, 3, 3, 3};

  const uint16_t* n = (idx == 1) ? kMapleNotes : kErrNotes;
  const uint8_t* d = (idx == 1) ? kMapleDurs : kErrDurs;
  for (uint8_t i = 0; i < 8; ++i) {
    notes_[i] = n[i];
    durs_[i] = d[i];
  }
  noTone(BDPIN_BUZZER);
  note_idx_ = 0;
  playing_ = false;
}

// Non-blocking buzzer update loop.
void Sensor::UpdateSound() {
  if (note_idx_ >= 8) return;
  const uint32_t dur = 1000 / durs_[note_idx_];

  if (!playing_) {
    tone(BDPIN_BUZZER, notes_[note_idx_], dur);
    note_time_ = millis();
    playing_ = true;
  } else if (millis() - note_time_ >= dur * 13 / 10) {
    noTone(BDPIN_BUZZER);
    ++note_idx_;
    playing_ = false;
  }
}

}  // namespace mk2_arm
