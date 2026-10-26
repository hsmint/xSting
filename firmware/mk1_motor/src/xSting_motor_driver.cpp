#include "../include/xSting_motor_driver.hpp"

// Limit values (XM430-W210-T and XM430-W350-T)
// MAX RPM is 77 when DXL is powered 12.0V
// 77 / 0.229 (RPM) = 336.24454...
const uint16_t LIMIT_X_MAX_VELOCITY = 337; 
// V = r * w = r     *        (RPM             * 0.10472)
//           = 0.033 * (0.229 * Goal_Velocity) * 0.10472
// Goal_Velocity = V * 1263.632956882
const float VELOCITY_CONSTANT_VALUE = 1263.632956882; 

const uint8_t DXL_MOTOR_ID_LEFT_1 = 1;
const uint8_t DXL_MOTOR_ID_LEFT_2 = 2;
const uint8_t DXL_MOTOR_ID_RIGHT_1 = 3;
const uint8_t DXL_MOTOR_ID_RIGHT_2 = 4;
const float DXL_PORT_PROTOCOL_VERSION = 2.0; // Dynamixel protocol version 2.0
const uint32_t DXL_PORT_BAUDRATE = 1000000; // baurd rate of Dynamixel
const int OPENCR_DXL_DIR_PIN = 84; // Arduino pin number of DYNAMIXEL direction pin on OpenCR.

ParamForSyncReadInst_t sync_read_param;
ParamForSyncWriteInst_t sync_write_param;
RecvInfoFromStatusInst_t read_result;
Dynamixel2Arduino dxl(Serial3, OPENCR_DXL_DIR_PIN);



xStingMotorDriver::xStingMotorDriver()
: torque_(false)
{
  wheel_id_[MotorLocation::LEFT_1]  = DXL_MOTOR_ID_LEFT_1;
  wheel_id_[MotorLocation::LEFT_2]  = DXL_MOTOR_ID_LEFT_2;
  wheel_id_[MotorLocation::RIGHT_1] = DXL_MOTOR_ID_RIGHT_1;
  wheel_id_[MotorLocation::RIGHT_2] = DXL_MOTOR_ID_RIGHT_2;
}

xStingMotorDriver::~xStingMotorDriver()
{
  close();
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
}

bool xStingMotorDriver::init(void)
{
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  drv_dxl_init();

  dxl.begin(DXL_PORT_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PORT_PROTOCOL_VERSION);

  sync_write_param.id_count = 4;
  sync_write_param.xel[LEFT_1].id = wheel_id_[MotorLocation::LEFT_1];
  sync_write_param.xel[LEFT_2].id = wheel_id_[MotorLocation::LEFT_2];
  sync_write_param.xel[RIGHT_1].id = wheel_id_[MotorLocation::RIGHT_1];
  sync_write_param.xel[RIGHT_2].id = wheel_id_[MotorLocation::RIGHT_2];

  sync_read_param.addr = 132;
  sync_read_param.length = 4;
  sync_read_param.id_count = 4;
  sync_read_param.xel[LEFT_1].id = wheel_id_[MotorLocation::LEFT_1];
  sync_read_param.xel[LEFT_2].id = wheel_id_[MotorLocation::LEFT_2];
  sync_read_param.xel[RIGHT_1].id = wheel_id_[MotorLocation::RIGHT_1];
  sync_read_param.xel[RIGHT_2].id = wheel_id_[MotorLocation::RIGHT_2];

  // Enable Dynamixel Torque
  set_torque(true);

  return true;
}

bool xStingMotorDriver::is_connected()
{
  return (dxl.ping(DXL_MOTOR_ID_LEFT_1) == true && dxl.ping(DXL_MOTOR_ID_LEFT_2) == true && dxl.ping(DXL_MOTOR_ID_RIGHT_1) == true && dxl.ping(DXL_MOTOR_ID_RIGHT_2) == true);
}

bool xStingMotorDriver::set_torque(bool onoff)
{
  bool ret = false;

  sync_write_param.addr = 64;
  sync_write_param.length = 1;
  sync_write_param.xel[LEFT_1].data[0] = onoff;
  sync_write_param.xel[LEFT_2].data[0] = onoff;
  sync_write_param.xel[RIGHT_1].data[0] = onoff;
  sync_write_param.xel[RIGHT_2].data[0] = onoff;

  if(dxl.syncWrite(sync_write_param) == true){
    ret = true;
    torque_ = onoff;
  }

  return ret;
}

bool xStingMotorDriver::get_torque()
{
  if(dxl.readControlTableItem(TORQUE_ENABLE, wheel_id_[MotorLocation::LEFT_1]) == true
    && dxl.readControlTableItem(TORQUE_ENABLE, wheel_id_[MotorLocation::LEFT_2]) == true
    && dxl.readControlTableItem(TORQUE_ENABLE, wheel_id_[MotorLocation::RIGHT_1]) == true
    && dxl.readControlTableItem(TORQUE_ENABLE, wheel_id_[MotorLocation::RIGHT_2]) == true){
    torque_ = true;
  }else{
    torque_ = false;
  }

  return torque_;
}

void xStingMotorDriver::close(void)
{
  // Disable Dynamixel Torque
  set_torque(false);
}

bool xStingMotorDriver::read_present_position(int32_t &left_value_1, int32_t &left_value_2, int32_t &right_value_1, int32_t &right_value_2)
{
  bool ret = false;

  sync_read_param.addr = 132;
  sync_read_param.length = 4;
  sync_read_param.id_count = 4;
  sync_read_param.xel[LEFT_1].id = wheel_id_[MotorLocation::LEFT_1];
  sync_read_param.xel[LEFT_2].id = wheel_id_[MotorLocation::LEFT_2];
  sync_read_param.xel[RIGHT_1].id = wheel_id_[MotorLocation::RIGHT_1];
  sync_read_param.xel[RIGHT_2].id = wheel_id_[MotorLocation::RIGHT_2];

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&left_value_1, read_result.xel[LEFT_1].data, sizeof(left_value_1));
    memcpy(&left_value_2, read_result.xel[LEFT_2].data, sizeof(left_value_2));
    memcpy(&right_value_1, read_result.xel[RIGHT_1].data, sizeof(right_value_1));
    memcpy(&right_value_2, read_result.xel[RIGHT_2].data, sizeof(right_value_2));
    ret = true;
  }

  return ret;
}

bool xStingMotorDriver::read_present_velocity(int32_t &left_value_1, int32_t &left_value_2, int32_t &right_value_1, int32_t &right_value_2)
{
  bool ret = false;

  sync_read_param.addr = 128;
  sync_read_param.length = 4;
  sync_read_param.id_count = 4;
  sync_read_param.xel[LEFT_1].id = wheel_id_[MotorLocation::LEFT_1];
  sync_read_param.xel[LEFT_2].id = wheel_id_[MotorLocation::LEFT_2];
  sync_read_param.xel[RIGHT_1].id = wheel_id_[MotorLocation::RIGHT_1];
  sync_read_param.xel[RIGHT_2].id = wheel_id_[MotorLocation::RIGHT_2];

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&left_value_1, read_result.xel[LEFT_1].data, sizeof(left_value_1));
    memcpy(&left_value_2, read_result.xel[LEFT_2].data, sizeof(left_value_2));
    memcpy(&right_value_1, read_result.xel[RIGHT_1].data, sizeof(right_value_1));
    memcpy(&right_value_2, read_result.xel[RIGHT_2].data, sizeof(right_value_2));
    ret = true;
  }

  return ret;
}

bool xStingMotorDriver::read_present_current(int16_t &left_value_1, int16_t &left_value_2, int16_t &right_value_1, int16_t &right_value_2)
{
  bool ret = false;

  sync_read_param.addr = 126;
  sync_read_param.length = 2;
  sync_read_param.id_count = 4;
  sync_read_param.xel[LEFT_1].id = wheel_id_[MotorLocation::LEFT_1];
  sync_read_param.xel[LEFT_2].id = wheel_id_[MotorLocation::LEFT_2];
  sync_read_param.xel[RIGHT_1].id = wheel_id_[MotorLocation::RIGHT_1];
  sync_read_param.xel[RIGHT_2].id = wheel_id_[MotorLocation::RIGHT_2];

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&left_value_1, read_result.xel[LEFT_1].data, sizeof(left_value_1));
    memcpy(&left_value_2, read_result.xel[LEFT_2].data, sizeof(left_value_2));
    memcpy(&right_value_1, read_result.xel[RIGHT_1].data, sizeof(right_value_1));
    memcpy(&right_value_2, read_result.xel[RIGHT_2].data, sizeof(right_value_2));
    ret = true;
  }

  return ret;
}

bool xStingMotorDriver::read_profile_acceleration(uint32_t &left_value_1, uint32_t &left_value_2, uint32_t &right_value_1, uint32_t &right_value_2)
{
  bool ret = false;

  sync_read_param.addr = 108;
  sync_read_param.length = 4;
  sync_read_param.id_count = 4;
  sync_read_param.xel[LEFT_1].id = wheel_id_[MotorLocation::LEFT_1];
  sync_read_param.xel[LEFT_2].id = wheel_id_[MotorLocation::LEFT_2];
  sync_read_param.xel[RIGHT_1].id = wheel_id_[MotorLocation::RIGHT_1];
  sync_read_param.xel[RIGHT_2].id = wheel_id_[MotorLocation::RIGHT_2];

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&left_value_1, read_result.xel[LEFT_1].data, sizeof(left_value_1));
    memcpy(&left_value_2, read_result.xel[LEFT_2].data, sizeof(left_value_2));
    memcpy(&right_value_1, read_result.xel[RIGHT_1].data, sizeof(right_value_1));
    memcpy(&right_value_2, read_result.xel[RIGHT_2].data, sizeof(right_value_2));
    ret = true;
  }

  return ret;
}

  bool xStingMotorDriver::write_velocity(int32_t left_value_1, int32_t left_value_2, int32_t right_value_1, int32_t right_value_2)
{
  bool ret = false;

  sync_write_param.addr = 104;
  sync_write_param.length = 4;
  memcpy(sync_write_param.xel[LEFT_1].data, &left_value_1, sync_write_param.length);
  memcpy(sync_write_param.xel[LEFT_2].data, &left_value_2, sync_write_param.length);
  memcpy(sync_write_param.xel[RIGHT_1].data, &right_value_1, sync_write_param.length);
  memcpy(sync_write_param.xel[RIGHT_2].data, &right_value_2, sync_write_param.length);

  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}

bool xStingMotorDriver::write_profile_acceleration(uint32_t left_value_1, uint32_t left_value_2, uint32_t right_value_1, uint32_t right_value_2)
{
  bool ret = false;

  sync_write_param.addr = 108;
  sync_write_param.length = 4;
  memcpy(sync_write_param.xel[LEFT_1].data, &left_value_1, sync_write_param.length);
  memcpy(sync_write_param.xel[LEFT_2].data, &left_value_2, sync_write_param.length);
  memcpy(sync_write_param.xel[RIGHT_1].data, &right_value_1, sync_write_param.length);
  memcpy(sync_write_param.xel[RIGHT_2].data, &right_value_2, sync_write_param.length);

  if(dxl.syncWrite(sync_write_param)){
    ret = true;
  }

  return ret;
}

bool xStingMotorDriver::control_motors(const float wheel_separation, float linear_value, float angular_value)
{
  bool dxl_comm_result = false;
  
  float wheel_velocity[MotorLocation::MOTOR_NUM_MAX];
  float lin_vel = linear_value;
  float ang_vel = angular_value;

  wheel_velocity[LEFT_1]   = lin_vel - (ang_vel * wheel_separation / 2);
  wheel_velocity[LEFT_2]   = lin_vel - (ang_vel * wheel_separation / 2);
  wheel_velocity[RIGHT_1]  = lin_vel + (ang_vel * wheel_separation / 2);
  wheel_velocity[RIGHT_2]  = lin_vel + (ang_vel * wheel_separation / 2);

  wheel_velocity[LEFT_1]  = constrain(wheel_velocity[LEFT_1]  * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  wheel_velocity[LEFT_2]  = constrain(wheel_velocity[LEFT_2]  * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  wheel_velocity[RIGHT_1] = constrain(wheel_velocity[RIGHT_1] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  wheel_velocity[RIGHT_2] = constrain(wheel_velocity[RIGHT_2] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);

  dxl_comm_result = write_velocity((int32_t)wheel_velocity[LEFT_1], (int32_t)wheel_velocity[LEFT_2], (int32_t)wheel_velocity[RIGHT_1], (int32_t)wheel_velocity[RIGHT_2]);
  if (dxl_comm_result == false)
    return false;

  return true;
}
