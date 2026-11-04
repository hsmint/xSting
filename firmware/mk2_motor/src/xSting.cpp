#include "../include/xSting.h"

#include <string.h>

#include "../include/diagnosis.h"
#include "../include/motor_driver.h"
#include "../include/sensor.h"

using namespace mk2;

/*******************************************************************************
 * Declaration for motors, sensors, and diagnosis
 ******************************************************************************/
static xStingMotorDriver motor_driver;
static xStingSensors sensors;
static xStingDiagnosis diagnosis;
static bool initialized;
static uint32_t last_heartbeat;

/*******************************************************************************
 * Declaration for DYNAMIXEL Slave Function
 ******************************************************************************/
static DYNAMIXEL::USBSerialPortHandler port_dxl_slave(Serial);
static DYNAMIXEL::Slave dxl_slave(port_dxl_slave, 0x5000);

typedef struct ControlItemVariables {
  uint32_t dev_time_millis;
  bool is_connect_host;
  bool is_connect_motors;
  int8_t device_status;
  uint8_t heart_beat;
  bool user_led[4];
  bool push_button[2];
  bool bumper[2];
  uint32_t bat_voltage_x100;
  uint32_t bat_percent_x100;
  uint8_t buzzer_sound;

  bool joint_torque_enable_state;
  joint_position_info_t joint_goal_position;
  joint_position_info_t joint_present_position;
  joint_velocity_info_t joint_present_velocity;
  joint_current_info_t joint_present_current;
  joint_accel_info_t joint_profile_acc;
  joint_accel_info_t joint_profile_vel;
  bool joint_goal_position_wr_joint;
  bool joint_profile_acc_wr_joint;
  bool joint_profile_vel_wr_joint;

  bool imu_recalibration;
  uint8_t imu_status;
  float angular_vel[3];
  float linear_acc[3];
  float orientation[4];
} ControlItemVariables;

static ControlItemVariables control_items;

static void update_times(uint32_t interval_ms);
static void update_gpios(uint32_t interval_ms);
static void update_battery_status(uint32_t interval_ms);
static void update_imu(uint32_t interval_ms);
static void update_joint_status(uint32_t interval_ms);
static void update_connection_state_with_host();
static void update_device_status();
static bool set_torque(bool onoff);
static bool read_joint_status();
static bool hold_joints_at_present_position();
static void dxl_slave_write_callback_func(uint16_t address, uint8_t& error,
                                          void* arg);

/*******************************************************************************
 * Definition for xStingHexapod 'begin()' function
 ******************************************************************************/
void xStingHexapod::begin() {
  diagnosis.init();
  sensors.init();
  initialized = motor_driver.init();
  if (initialized) hold_joints_at_present_position();

  dxl_slave.begin();
  dxl_slave.setPortProtocolVersion(2.0);
  dxl_slave.setFirmwareVersion(1);
  dxl_slave.setID(SLAVE_ID);
  dxl_slave.addControlItem(ADDR_MILLIS, control_items.dev_time_millis);
  dxl_slave.addControlItem(ADDR_CONNECT_HOST, control_items.is_connect_host);
  dxl_slave.addControlItem(ADDR_CONNECT_MOTORS,
                           control_items.is_connect_motors);
  dxl_slave.addControlItem(ADDR_DEVICE_STATUS, control_items.device_status);
  dxl_slave.addControlItem(ADDR_HEARTBEAT, control_items.heart_beat);
  for (uint8_t i = 0; i < 4; ++i)
    dxl_slave.addControlItem(ADDR_USER_LED + i, control_items.user_led[i]);
  for (uint8_t i = 0; i < 2; ++i) {
    dxl_slave.addControlItem(ADDR_BUTTON + i, control_items.push_button[i]);
    dxl_slave.addControlItem(ADDR_BUMPER + i, control_items.bumper[i]);
  }
  dxl_slave.addControlItem(ADDR_BATTERY_VOLTAGE,
                           control_items.bat_voltage_x100);
  dxl_slave.addControlItem(ADDR_BATTERY_PERCENT,
                           control_items.bat_percent_x100);
  dxl_slave.addControlItem(ADDR_SOUND, control_items.buzzer_sound);
  dxl_slave.addControlItem(ADDR_TORQUE,
                           control_items.joint_torque_enable_state);
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    dxl_slave.addControlItem(ADDR_GOAL_POSITION + 4 * i,
                             control_items.joint_goal_position.value[i]);
    dxl_slave.addControlItem(ADDR_PRESENT_POSITION + 4 * i,
                             control_items.joint_present_position.value[i]);
    dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY + 4 * i,
                             control_items.joint_present_velocity.value[i]);
    dxl_slave.addControlItem(ADDR_PRESENT_CURRENT + 2 * i,
                             control_items.joint_present_current.value[i]);
    dxl_slave.addControlItem(ADDR_PROFILE_ACC + 4 * i,
                             control_items.joint_profile_acc.value[i]);
    dxl_slave.addControlItem(ADDR_PROFILE_VEL + 4 * i,
                             control_items.joint_profile_vel.value[i]);
  }
  dxl_slave.addControlItem(ADDR_GOAL_POSITION_WR,
                           control_items.joint_goal_position_wr_joint);
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_WR,
                           control_items.joint_profile_acc_wr_joint);
  dxl_slave.addControlItem(ADDR_PROFILE_VEL_WR,
                           control_items.joint_profile_vel_wr_joint);
  dxl_slave.addControlItem(ADDR_IMU_RECALIBRATION,
                           control_items.imu_recalibration);
  dxl_slave.addControlItem(ADDR_IMU_STATUS, control_items.imu_status);
  dxl_slave.addControlItem(ADDR_ANGULAR_VELOCITY_X,
                           control_items.angular_vel[0]);
  dxl_slave.addControlItem(ADDR_ANGULAR_VELOCITY_Y,
                           control_items.angular_vel[1]);
  dxl_slave.addControlItem(ADDR_ANGULAR_VELOCITY_Z,
                           control_items.angular_vel[2]);
  dxl_slave.addControlItem(ADDR_LINEAR_ACC_X, control_items.linear_acc[0]);
  dxl_slave.addControlItem(ADDR_LINEAR_ACC_Y, control_items.linear_acc[1]);
  dxl_slave.addControlItem(ADDR_LINEAR_ACC_Z, control_items.linear_acc[2]);
  dxl_slave.addControlItem(ADDR_ORIENTATION_W, control_items.orientation[0]);
  dxl_slave.addControlItem(ADDR_ORIENTATION_X, control_items.orientation[1]);
  dxl_slave.addControlItem(ADDR_ORIENTATION_Y, control_items.orientation[2]);
  dxl_slave.addControlItem(ADDR_ORIENTATION_Z, control_items.orientation[3]);
  dxl_slave.setWriteCallbackFunc(dxl_slave_write_callback_func);
  sensors.makeMelody(initialized && control_items.is_connect_motors ? 1 : 3);
  sensors.updateIMU();
  update_times(0);
  update_gpios(0);
  update_battery_status(0);
  update_imu(0);
  update_device_status();
}

/*******************************************************************************
 * Definition for xStingHexapod 'run()' function
 ******************************************************************************/
void xStingHexapod::run() {
  sensors.updateIMU();
  update_connection_state_with_host();

  diagnosis.showLedStatus(control_items.is_connect_host);
  const uint8_t previous_battery_state = diagnosis.getBatteryState();
  const uint8_t battery_state = diagnosis.updateVoltageCheck(true);
  if (battery_state == BATTERY_POWER_WARNNING) {
    // Diagnosis owns the buzzer while warning; do not overlap a melody.
    if (previous_battery_state != battery_state) sensors.stopMelody();
  } else {
    sensors.onMelody();
  }
  if (!diagnosis.isPowerOn()) {
    initialized = false;
    control_items.is_connect_motors = false;
    control_items.joint_torque_enable_state = false;
  }

  update_times(UPDATE_MS);
  update_gpios(UPDATE_MS);
  update_battery_status(UPDATE_MS);
  update_imu(UPDATE_MS);
  update_joint_status(UPDATE_MS);
  update_device_status();
  dxl_slave.processPacket();
}

/*******************************************************************************
 * Functions for updating control items
 ******************************************************************************/
static void update_times(uint32_t interval_ms) {
  static uint32_t pre_time;
  if (millis() - pre_time < interval_ms) return;
  pre_time = millis();
  control_items.dev_time_millis = millis();
}

static void update_gpios(uint32_t interval_ms) {
  static uint32_t pre_time;
  if (millis() - pre_time < interval_ms) return;
  pre_time = millis();
  control_items.user_led[0] = digitalRead(BDPIN_GPIO_4);
  control_items.user_led[1] = digitalRead(BDPIN_GPIO_6);
  control_items.user_led[2] = digitalRead(BDPIN_GPIO_8);
  control_items.user_led[3] = digitalRead(BDPIN_GPIO_10);
  const uint8_t pressed = sensors.checkPushButton();
  control_items.push_button[0] = pressed & 1;
  control_items.push_button[1] = pressed & 2;
  control_items.bumper[0] = sensors.getBumper1State();
  control_items.bumper[1] = sensors.getBumper2State();
}

static void update_battery_status(uint32_t interval_ms) {
  static uint32_t pre_time;
  if (millis() - pre_time < interval_ms) return;
  pre_time = millis();
  const float voltage = sensors.checkVoltage();
  control_items.bat_voltage_x100 = voltage > 0 ? voltage * 100 : 0;
  const float percent = (voltage - BATTERY_CUTOFF_V) /
                        (BATTERY_FULL_V - BATTERY_CUTOFF_V) * 10000;
  control_items.bat_percent_x100 =
      percent < 0 ? 0 : (percent > 10000 ? 10000 : percent);
}

static void update_imu(uint32_t interval_ms) {
  static uint32_t pre_time;
  if (millis() - pre_time < interval_ms) return;
  pre_time = millis();
  control_items.imu_status = sensors.getIMUStatus();
  memcpy(control_items.angular_vel, sensors.getImuAngularVelocity(),
         sizeof(control_items.angular_vel));
  memcpy(control_items.linear_acc, sensors.getImuLinearAcc(),
         sizeof(control_items.linear_acc));
  memcpy(control_items.orientation, sensors.getOrientation(),
         sizeof(control_items.orientation));
}

static bool set_torque(bool onoff) {
  const bool ret = motor_driver.set_torque(onoff);
  control_items.joint_torque_enable_state = motor_driver.get_torque();
  return ret;
}

static bool read_joint_status() {
  joint_position_info_t position;
  joint_velocity_info_t velocity;
  joint_current_info_t current;
  if (!motor_driver.read_present_position(position) ||
      !motor_driver.read_present_velocity(velocity) ||
      !motor_driver.read_present_current(current))
    return false;
  // Publish only a complete feedback set for all 9 motors.
  control_items.joint_present_position = position;
  control_items.joint_present_velocity = velocity;
  control_items.joint_present_current = current;
  return true;
}

static void update_joint_status(uint32_t interval_ms) {
  static uint32_t pre_time;
  if (millis() - pre_time < interval_ms) return;
  pre_time = millis();
  control_items.is_connect_motors = initialized && read_joint_status();
  if (!control_items.is_connect_motors &&
      control_items.joint_torque_enable_state)
    set_torque(false);
}

static void update_device_status() {
  if (!diagnosis.isPowerOn())
    control_items.device_status = STATUS_POWER_OFF;
  else if (!control_items.is_connect_motors)
    control_items.device_status = STATUS_NOT_CONNECTED_MOTORS;
  else if (diagnosis.isLowBattery())
    control_items.device_status = STATUS_LOW_BATTERY;
  else
    control_items.device_status = STATUS_RUNNING;
}

/*******************************************************************************
 * Callback for commands from the host
 ******************************************************************************/
void dxl_slave_write_callback_func(uint16_t address, uint8_t& error, void*) {
  if (address == ADDR_HEARTBEAT) {
    last_heartbeat = millis();
    control_items.is_connect_host = true;
    return;
  }
  if (address >= ADDR_USER_LED && address < ADDR_USER_LED + 4) {
    const uint8_t index = address - ADDR_USER_LED;
    sensors.setLED(index, control_items.user_led[index]);
    return;
  }
  if (address == ADDR_SOUND) {
    if (diagnosis.getBatteryState() == BATTERY_POWER_WARNNING) {
      error = DXL_ERR_ACCESS;
      return;
    }
    if (control_items.buzzer_sound > 3) {
      error = DXL2_0_ERR_DATA_RANGE;
      return;
    }
    sensors.makeMelody(control_items.buzzer_sound);
    return;
  }
  if (address == ADDR_IMU_RECALIBRATION) {
    if (control_items.imu_recalibration && !sensors.calibrationGyro())
      error = DXL2_0_ERR_RESULT_FAIL;
    control_items.imu_recalibration = false;
    control_items.imu_status = sensors.getIMUStatus();
    return;
  }
  // Array writes only stage values; the separate WR register commits them.
  if ((address >= ADDR_GOAL_POSITION && address < ADDR_GOAL_POSITION_WR) ||
      (address >= ADDR_PROFILE_ACC && address < ADDR_PROFILE_ACC_WR) ||
      (address >= ADDR_PROFILE_VEL && address < ADDR_PROFILE_VEL_WR))
    return;

  if (address == ADDR_TORQUE && !control_items.joint_torque_enable_state) {
    if (!set_torque(false)) error = DXL2_0_ERR_RESULT_FAIL;
    return;
  }
  if (address != ADDR_TORQUE && address != ADDR_GOAL_POSITION_WR &&
      address != ADDR_PROFILE_ACC_WR && address != ADDR_PROFILE_VEL_WR) {
    error = DXL_ERR_ACCESS;
    return;
  }
  if (!initialized || !control_items.is_connect_motors ||
      !control_items.is_connect_host ||
      millis() - last_heartbeat >= HEARTBEAT_TIMEOUT_MS) {
    error = DXL_ERR_ACCESS;
    return;
  }

  bool ok = true;
  switch (address) {
    case ADDR_TORQUE:
      // Seed the goal before enabling torque, avoiding a jump to an old goal.
      ok = hold_joints_at_present_position() && set_torque(true);
      break;
    case ADDR_GOAL_POSITION_WR:
      for (uint8_t i = 0;
           control_items.joint_goal_position_wr_joint && i < MOTOR_COUNT; ++i) {
        if (control_items.joint_goal_position.value[i] < 0 ||
            control_items.joint_goal_position.value[i] > 4095) {
          error = DXL2_0_ERR_DATA_RANGE;
          return;
        }
      }
      if (control_items.joint_goal_position_wr_joint)
        ok = motor_driver.write_goal_position_joint(
            control_items.joint_goal_position);
      control_items.joint_goal_position_wr_joint = false;
      break;
    case ADDR_PROFILE_ACC_WR:
      if (control_items.joint_profile_acc_wr_joint)
        ok = motor_driver.write_profile_acceleration_joint(
            control_items.joint_profile_acc);
      control_items.joint_profile_acc_wr_joint = false;
      break;
    case ADDR_PROFILE_VEL_WR:
      if (control_items.joint_profile_vel_wr_joint)
        ok = motor_driver.write_profile_velocity_joint(
            control_items.joint_profile_vel);
      control_items.joint_profile_vel_wr_joint = false;
      break;
  }
  if (!ok) error = DXL2_0_ERR_RESULT_FAIL;
}

/*******************************************************************************
 * Connection state with the host
 ******************************************************************************/
static void update_connection_state_with_host() {
  if (control_items.is_connect_host &&
      millis() - last_heartbeat >= HEARTBEAT_TIMEOUT_MS) {
    control_items.is_connect_host = false;
    if (control_items.joint_torque_enable_state &&
        !hold_joints_at_present_position())
      set_torque(false);
  }
}

static bool hold_joints_at_present_position() {
  control_items.is_connect_motors = initialized && read_joint_status();
  if (!control_items.is_connect_motors) return false;
  control_items.joint_goal_position = control_items.joint_present_position;
  return motor_driver.write_goal_position_joint(
      control_items.joint_goal_position);
}
