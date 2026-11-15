#include "../include/mk2_arm.h"

#include "../include/motor.h"
#include "../include/sensor.h"

namespace mk2_arm {

struct Table {
  uint32_t millis;
  bool connect_motors;
  int8_t status;
  uint8_t heartbeat;
  bool reset;
  uint32_t bat_v_x100;
  uint8_t sound;
  bool torque;
  int32_t goal_pos[kMotorCount];
  bool goal_pos_wr;
  int32_t pres_pos[kMotorCount];
  int32_t pres_vel[kMotorCount];
  int16_t pres_cur[kMotorCount];
  int16_t gripper_cur;
};

static Motor motor;
static Sensor sensor;
static Table tbl;
static DYNAMIXEL::USBSerialPortHandler port(Serial);
static DYNAMIXEL::Slave slave(port, 0x5000);

static void OnWrite(uint16_t addr, uint8_t& err, void* arg);
static void UpdateState();

// Initialize arm motors, register slave items, and play Maple BGM.
void Begin() {
  tbl.connect_motors = motor.Init();
  tbl.torque = motor.torque();
  tbl.gripper_cur = kGripperCurrent;
  if (!tbl.connect_motors || !motor.ReadPos(tbl.pres_pos)) {
    for (uint8_t i = 0; i < kMotorCount; ++i) tbl.pres_pos[i] = kHomeTicks[i];
  }
  for (uint8_t i = 0; i < kMotorCount; ++i) {
    tbl.goal_pos[i] = kAutoHome ? kHomeTicks[i] : tbl.pres_pos[i];
  }

  slave.begin();
  slave.setPortProtocolVersion(2.0f);
  slave.setFirmwareVersion(1);
  slave.setID(kSlaveId);

  slave.addControlItem(ADDR_MILLIS, tbl.millis);
  slave.addControlItem(ADDR_CONNECT_MOTORS, tbl.connect_motors);
  slave.addControlItem(ADDR_DEVICE_STATUS, tbl.status);
  slave.addControlItem(ADDR_HEARTBEAT, tbl.heartbeat);
  slave.addControlItem(ADDR_RESET, tbl.reset);
  slave.addControlItem(ADDR_BATTERY_V, tbl.bat_v_x100);
  slave.addControlItem(ADDR_SOUND, tbl.sound);
  slave.addControlItem(ADDR_TORQUE, tbl.torque);
  for (uint8_t i = 0; i < kMotorCount; ++i) {
    slave.addControlItem(ADDR_GOAL_POS + 4 * i, tbl.goal_pos[i]);
    slave.addControlItem(ADDR_PRESENT_POS + 4 * i, tbl.pres_pos[i]);
    slave.addControlItem(ADDR_PRESENT_VEL + 4 * i, tbl.pres_vel[i]);
    slave.addControlItem(ADDR_PRESENT_CUR + 2 * i, tbl.pres_cur[i]);
  }
  slave.addControlItem(ADDR_GOAL_POS_WR, tbl.goal_pos_wr);
  slave.addControlItem(ADDR_GRIPPER_CUR, tbl.gripper_cur);
  slave.setWriteCallbackFunc(OnWrite);

  sensor.PlaySound(tbl.connect_motors ? 1 : 3);
}

// Run 200Hz state updates and process host USB packets.
void Run() {
  sensor.UpdateSound();
  UpdateState();
  slave.processPacket();
}

// Update arm motor and filtered battery states every 5ms (200Hz).
static void UpdateState() {
  static uint32_t pre_ms = 0;
  static float v_avg = 12.0f;
  if (millis() - pre_ms < kUpdateMs) return;
  pre_ms = millis();

  tbl.millis = pre_ms;
  const float v = sensor.ReadVolt();
  if (v > 5.0f) v_avg = 0.95f * v_avg + 0.05f * v;
  tbl.bat_v_x100 = static_cast<uint32_t>(v_avg * 100.0f);

  tbl.connect_motors =
      motor.ReadPos(tbl.pres_pos) && motor.ReadVel(tbl.pres_vel) && motor.ReadCur(tbl.pres_cur);

  // Keep torque ON on low battery so the arm never drops; alert and hold on critical voltage.
  if (v > 5.0f && v_avg < 10.7f) {
    tbl.status = -2;
    sensor.PlaySound(3);
  } else if (v > 5.0f && v_avg < 11.1f) {
    tbl.status = 1;
  } else {
    tbl.status = tbl.connect_motors ? 0 : -1;
  }
}

// Handle write commands from host PC (block new motion if battery is critical).
static void OnWrite(uint16_t addr, uint8_t& err, void*) {
  (void)err;
  if (addr == ADDR_RESET && tbl.reset) NVIC_SystemReset();
  if (addr == ADDR_SOUND) sensor.PlaySound(tbl.sound);
  if (addr == ADDR_TORQUE) {
    if (tbl.torque) {
      for (uint8_t i = 0; i < kMotorCount; ++i) tbl.goal_pos[i] = tbl.pres_pos[i];
      motor.WritePos(tbl.pres_pos);
    }
    motor.SetTorque(tbl.torque);
  }
  if (addr == ADDR_GOAL_POS_WR && tbl.goal_pos_wr) {
    if (tbl.status != -2) motor.WritePos(tbl.goal_pos);
    tbl.goal_pos_wr = false;
  }
  if (addr == ADDR_GRIPPER_CUR) motor.WriteGripperCur(tbl.gripper_cur);
}

}  // namespace mk2_arm
