#include "../include/mk2_leg.h"

#include <string.h>

#include "../include/motor.h"
#include "../include/sensor.h"

namespace mk2_leg {

struct Table {
  uint32_t millis;
  bool connect_motors;
  int8_t status;
  uint8_t heartbeat;
  bool reset;
  uint32_t ultrasonic_mm;
  uint32_t bat_v_x100;
  uint8_t sound;
  bool torque;
  int32_t goal_pos[kMotorCount];
  bool goal_pos_wr;
  int32_t pres_pos[kMotorCount];
  int32_t pres_vel[kMotorCount];
  int16_t pres_cur[kMotorCount];
  bool imu_cal;
  uint8_t imu_state;
  float gyro[3];
  float acc[3];
  float quat[4];
};

static Motor motor;
static Sensor sensor;
static Table tbl;
static DYNAMIXEL::USBSerialPortHandler port(Serial);
static DYNAMIXEL::Slave slave(port, 0x5002);

static void OnWrite(uint16_t addr, uint8_t& err, void* arg);
static void UpdateState();

// Initialize sensors, stand up motors, register slave items, and play Maple BGM.
void Begin() {
  sensor.Init();
  tbl.connect_motors = motor.Init();
  tbl.torque = motor.torque();
  for (uint8_t i = 0; i < kMotorCount; ++i) tbl.goal_pos[i] = kStandTicks[i];

  slave.begin();
  slave.setPortProtocolVersion(2.0f);
  slave.setFirmwareVersion(1);
  slave.setID(kSlaveId);

  slave.addControlItem(ADDR_MILLIS, tbl.millis);
  slave.addControlItem(ADDR_CONNECT_MOTORS, tbl.connect_motors);
  slave.addControlItem(ADDR_DEVICE_STATUS, tbl.status);
  slave.addControlItem(ADDR_HEARTBEAT, tbl.heartbeat);
  slave.addControlItem(ADDR_RESET, tbl.reset);
  slave.addControlItem(ADDR_ULTRASONIC_MM, tbl.ultrasonic_mm);
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

  slave.addControlItem(ADDR_IMU_CAL, tbl.imu_cal);
  slave.addControlItem(ADDR_IMU_STATUS, tbl.imu_state);
  slave.addControlItem(ADDR_GYRO_X, tbl.gyro[0]);
  slave.addControlItem(ADDR_GYRO_Y, tbl.gyro[1]);
  slave.addControlItem(ADDR_GYRO_Z, tbl.gyro[2]);
  slave.addControlItem(ADDR_ACC_X, tbl.acc[0]);
  slave.addControlItem(ADDR_ACC_Y, tbl.acc[1]);
  slave.addControlItem(ADDR_ACC_Z, tbl.acc[2]);
  slave.addControlItem(ADDR_QUAT_W, tbl.quat[0]);
  slave.addControlItem(ADDR_QUAT_X, tbl.quat[1]);
  slave.addControlItem(ADDR_QUAT_Y, tbl.quat[2]);
  slave.addControlItem(ADDR_QUAT_Z, tbl.quat[3]);
  slave.setWriteCallbackFunc(OnWrite);

  sensor.PlaySound(tbl.connect_motors ? 1 : 3);
}

// Run 200Hz state updates and process host USB packets.
void Run() {
  sensor.UpdateImu();
  sensor.UpdateSonar();
  sensor.UpdateSound();
  UpdateState();
  slave.processPacket();
}

// Update motor, IMU, and filtered battery states every 5ms (200Hz).
static void UpdateState() {
  static uint32_t pre_ms = 0;
  static float v_avg = 12.0f;
  if (millis() - pre_ms < kUpdateMs) return;
  pre_ms = millis();

  tbl.millis = pre_ms;
  tbl.ultrasonic_mm = sensor.sonar_mm();
  const float v = sensor.ReadVolt();
  if (v > 5.0f) v_avg = 0.95f * v_avg + 0.05f * v;
  tbl.bat_v_x100 = static_cast<uint32_t>(v_avg * 100.0f);

  tbl.connect_motors =
      motor.ReadPos(tbl.pres_pos) && motor.ReadVel(tbl.pres_vel) && motor.ReadCur(tbl.pres_cur);

  // Keep torque ON on low battery to prevent robot collapse; alert and hold pose on critical voltage.
  if (v > 5.0f && v_avg < 10.7f) {
    tbl.status = -2;
    sensor.PlaySound(3);
  } else if (v > 5.0f && v_avg < 11.1f) {
    tbl.status = 1;
  } else {
    tbl.status = tbl.connect_motors ? 0 : -1;
  }

  tbl.imu_state = sensor.imu_state();
  memcpy(tbl.gyro, sensor.gyro(), sizeof(tbl.gyro));
  memcpy(tbl.acc, sensor.acc(), sizeof(tbl.acc));
  memcpy(tbl.quat, sensor.quat(), sizeof(tbl.quat));
}

// Handle write commands from host PC (block new motion if battery is critical).
static void OnWrite(uint16_t addr, uint8_t& err, void*) {
  if (addr == ADDR_ULTRASONIC_MM) {
    tbl.ultrasonic_mm = sensor.sonar_mm();
    err = DXL_ERR_ACCESS;
    return;
  }
  if (addr == ADDR_RESET && tbl.reset) NVIC_SystemReset();
  if (addr == ADDR_SOUND) sensor.PlaySound(tbl.sound);
  if (addr == ADDR_IMU_CAL && tbl.imu_cal) {
    sensor.CalibImu();
    tbl.imu_cal = false;
  }
  if (addr == ADDR_TORQUE) {
    if (tbl.torque) motor.WritePos(tbl.pres_pos);
    motor.SetTorque(tbl.torque);
  }
  if (addr == ADDR_GOAL_POS_WR && tbl.goal_pos_wr) {
    if (tbl.status != -2) motor.WritePos(tbl.goal_pos);
    tbl.goal_pos_wr = false;
  }
}

}  // namespace mk2_leg
