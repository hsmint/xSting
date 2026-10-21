#include "../include/xSting.h"

/*******************************************************************************
* Declaration for motors
*******************************************************************************/
static OpenManipulatorDriver manipulator_driver;

static bool get_connection_state_with_joints();
static void set_connection_state_with_joints(bool is_connected);

/*******************************************************************************
* Declaration for sensors
*******************************************************************************/
static xStingSensors sensors;

/*******************************************************************************
* Declaration for diagnosis
*******************************************************************************/
static xStingDiagnosis diagnosis;

/*******************************************************************************
* Declaration for DYNAMIXEL Slave Function
*******************************************************************************/
#define SERIAL_DXL_SLAVE Serial
const uint8_t ID_DXL_SLAVE = 200;
const uint16_t MODEL_NUM_DXL_SLAVE = 0x5000;
const float PROTOCOL_VERSION_DXL_SLAVE = 2.0;
const uint32_t HEARTBEAT_TIMEOUT_MS = 500;
const uint32_t MODEL_INFO_XSTING = 3;

static void dxl_slave_write_callback_func(uint16_t addr, uint8_t &dxl_err_code, void* arg);

static bool get_connection_state_with_ros2_node();
static void set_connection_state_with_ros2_node(bool is_connected);
static void update_connection_state_with_ros2_node();
static void hold_joints_at_present_position();

static void update_times(uint32_t interval_ms);
static void update_gpios(uint32_t interval_ms);
static void update_battery_status(uint32_t interval_ms);
static void update_joint_status(uint32_t interval_ms);


DYNAMIXEL::USBSerialPortHandler port_dxl_slave(SERIAL_DXL_SLAVE);
DYNAMIXEL::Slave dxl_slave(port_dxl_slave, MODEL_NUM_DXL_SLAVE);

enum ControlTableItemAddr{
  ADDR_MODEL_INFORM    = 2,
  
  ADDR_MILLIS          = 10,

  ADDR_DEBUG_MODE      = 14,  
  ADDR_CONNECT_ROS2    = 15,
  ADDR_CONNECT_MANIP   = 16,

  ADDR_DEVICE_STATUS   = 18,
  ADDR_HEARTBEAT       = 19,

  ADDR_USER_LED_1      = 20,
  ADDR_USER_LED_2      = 21,
  ADDR_USER_LED_3      = 22,
  ADDR_USER_LED_4      = 23,

  ADDR_BUTTON_1        = 26,
  ADDR_BUTTON_2        = 27,
  ADDR_BUMPER_1        = 28,
  ADDR_BUMPER_2        = 29,

  ADDR_BATTERY_VOLTAGE = 42,
  ADDR_BATTERY_PERCENT = 46,

  ADDR_SOUND           = 50,

  // Joint registers are packed from address 59 using each item's byte width.
  ADDR_TORQUE_JOINT             = 59,

  ADDR_GOAL_POSITION_JOINT_1    = 60,
  ADDR_GOAL_POSITION_JOINT_2    = 64,
  ADDR_GOAL_POSITION_JOINT_3    = 68,
  ADDR_GOAL_POSITION_JOINT_4    = 72,
  ADDR_GOAL_POSITION_JOINT_5    = 76,
  ADDR_GOAL_POSITION_JOINT_6    = 80,
  ADDR_GOAL_POSITION_JOINT_7    = 84,
  ADDR_GOAL_POSITION_GRIPPER    = 88,
  ADDR_GOAL_POSITION_WR_JOINT   = 92,
  ADDR_GOAL_POSITION_WR_GRIPPER = 93,
  ADDR_GOAL_POSITION_RD         = 94,

  ADDR_PRESENT_POSITION_JOINT_1 = 95,
  ADDR_PRESENT_POSITION_JOINT_2 = 99,
  ADDR_PRESENT_POSITION_JOINT_3 = 103,
  ADDR_PRESENT_POSITION_JOINT_4 = 107,
  ADDR_PRESENT_POSITION_JOINT_5 = 111,
  ADDR_PRESENT_POSITION_JOINT_6 = 115,
  ADDR_PRESENT_POSITION_JOINT_7 = 119,
  ADDR_PRESENT_POSITION_GRIPPER = 123,

  ADDR_PRESENT_VELOCITY_JOINT_1 = 127,
  ADDR_PRESENT_VELOCITY_JOINT_2 = 131,
  ADDR_PRESENT_VELOCITY_JOINT_3 = 135,
  ADDR_PRESENT_VELOCITY_JOINT_4 = 139,
  ADDR_PRESENT_VELOCITY_JOINT_5 = 143,
  ADDR_PRESENT_VELOCITY_JOINT_6 = 147,
  ADDR_PRESENT_VELOCITY_JOINT_7 = 151,
  ADDR_PRESENT_VELOCITY_GRIPPER = 155,

  ADDR_PRESENT_CURRENT_JOINT_1  = 159,
  ADDR_PRESENT_CURRENT_JOINT_2  = 161,
  ADDR_PRESENT_CURRENT_JOINT_3  = 163,
  ADDR_PRESENT_CURRENT_JOINT_4  = 165,
  ADDR_PRESENT_CURRENT_JOINT_5  = 167,
  ADDR_PRESENT_CURRENT_JOINT_6  = 169,
  ADDR_PRESENT_CURRENT_JOINT_7  = 171,
  ADDR_PRESENT_CURRENT_GRIPPER  = 173,

  ADDR_PROFILE_ACC_JOINT_1      = 175,
  ADDR_PROFILE_ACC_JOINT_2      = 179,
  ADDR_PROFILE_ACC_JOINT_3      = 183,
  ADDR_PROFILE_ACC_JOINT_4      = 187,
  ADDR_PROFILE_ACC_JOINT_5      = 191,
  ADDR_PROFILE_ACC_JOINT_6      = 195,
  ADDR_PROFILE_ACC_JOINT_7      = 199,
  ADDR_PROFILE_ACC_GRIPPER      = 203,
  ADDR_PROFILE_ACC_WR_JOINT     = 207,
  ADDR_PROFILE_ACC_WR_GRIPPER   = 208,
  ADDR_PROFILE_ACC_RD           = 209,

  ADDR_PROFILE_VEL_JOINT_1      = 210,
  ADDR_PROFILE_VEL_JOINT_2      = 214,
  ADDR_PROFILE_VEL_JOINT_3      = 218,
  ADDR_PROFILE_VEL_JOINT_4      = 222,
  ADDR_PROFILE_VEL_JOINT_5      = 226,
  ADDR_PROFILE_VEL_JOINT_6      = 230,
  ADDR_PROFILE_VEL_JOINT_7      = 234,
  ADDR_PROFILE_VEL_GRIPPER      = 238,
  ADDR_PROFILE_VEL_WR_JOINT     = 242,
  ADDR_PROFILE_VEL_WR_GRIPPER   = 243,
  ADDR_PROFILE_VEL_RD           = 244,

  ADDR_GOAL_CURRENT_JOINT_1     = 245,
  ADDR_GOAL_CURRENT_JOINT_2     = 247,
  ADDR_GOAL_CURRENT_JOINT_3     = 249,
  ADDR_GOAL_CURRENT_JOINT_4     = 251,
  ADDR_GOAL_CURRENT_JOINT_5     = 253,
  ADDR_GOAL_CURRENT_JOINT_6     = 255,
  ADDR_GOAL_CURRENT_JOINT_7     = 257,
  ADDR_GOAL_CURRENT_GRIPPER     = 259,
  ADDR_GOAL_CURRENT_WR_JOINT    = 261,
  ADDR_GOAL_CURRENT_WR_GRIPPER  = 262,
  ADDR_GOAL_CURRENT_RD          = 263,

};

typedef struct ControlItemVariables{
  uint32_t model_inform;

  uint32_t dev_time_millis;

  int8_t device_status;
  uint8_t heart_beat;
  bool debug_mode;
  bool is_connect_ros2_node;
  bool is_connect_manipulator;

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
  joint_current_info_t joint_goal_current;

  bool joint_goal_position_wr_joint;
  bool joint_goal_position_wr_gripper;
  bool joint_goal_position_rd;

  bool joint_profile_acc_wr_joint;
  bool joint_profile_acc_wr_gripper;
  bool joint_profile_acc_rd;

  bool joint_profile_vel_wr_joint;
  bool joint_profile_vel_wr_gripper;
  bool joint_profile_vel_rd;

  bool joint_goal_current_wr_joint;
  bool joint_goal_current_wr_gripper;
  bool joint_goal_current_rd;

}ControlItemVariables;

static ControlItemVariables control_items;


/*******************************************************************************
* Definition for xStingCore 'begin()' function
*******************************************************************************/
void xStingCore::begin()
{
  bool ret; (void)ret;
  DEBUG_SERIAL_BEGIN(57600);
  DEBUG_PRINTLN(" ");
  DEBUG_PRINTLN("Version : V221004R1");
  DEBUG_PRINTLN("Begin Start...");

  ret = diagnosis.init();
  DEBUG_PRINTLN(ret==true?"Diagnosis setup completed.":"Diagnosis setup failed.");
  ret = sensors.init();
  DEBUG_PRINTLN(ret==true?"Sensors setup completed.":"Sensors setup failed.");
  
  ret = manipulator_driver.init();
  DEBUG_PRINTLN(ret==true?"Manipulator driver setup completed.":"Manipulator driver setup failed.");

  DEBUG_PRINT("Dynamixel2Arduino Item Max : ");
  DEBUG_PRINTLN(CONTROL_ITEM_MAX);

  control_items.debug_mode = false;
  control_items.is_connect_ros2_node = false;
  control_items.is_connect_manipulator = false;  

  // Port begin
  dxl_slave.begin();
  // Init DXL Slave function
  dxl_slave.setPortProtocolVersion(PROTOCOL_VERSION_DXL_SLAVE);
  dxl_slave.setFirmwareVersion(FIRMWARE_VER);
  dxl_slave.setID(ID_DXL_SLAVE);

  /* Add control items for Slave */
  // Items for model information of device
  control_items.model_inform = MODEL_INFO_XSTING;
  dxl_slave.addControlItem(ADDR_MODEL_INFORM, control_items.model_inform);
  // Items for Timer of device
  dxl_slave.addControlItem(ADDR_MILLIS, control_items.dev_time_millis);

  // Items to debug mode
  dxl_slave.addControlItem(ADDR_DEBUG_MODE, control_items.debug_mode);
  // Items to connect ros2
  dxl_slave.addControlItem(ADDR_CONNECT_ROS2, control_items.is_connect_ros2_node);
  // Items to connect manipulator
  dxl_slave.addControlItem(ADDR_CONNECT_MANIP, control_items.is_connect_manipulator);

  // Items to inform device status
  dxl_slave.addControlItem(ADDR_DEVICE_STATUS, control_items.device_status);
  // Items to check connection state with node
  dxl_slave.addControlItem(ADDR_HEARTBEAT, control_items.heart_beat);
  // Items for GPIO
  dxl_slave.addControlItem(ADDR_USER_LED_1, control_items.user_led[0]);
  dxl_slave.addControlItem(ADDR_USER_LED_2, control_items.user_led[1]);
  dxl_slave.addControlItem(ADDR_USER_LED_3, control_items.user_led[2]);
  dxl_slave.addControlItem(ADDR_USER_LED_4, control_items.user_led[3]);
  dxl_slave.addControlItem(ADDR_BUTTON_1, control_items.push_button[0]);
  dxl_slave.addControlItem(ADDR_BUTTON_2, control_items.push_button[1]);
  dxl_slave.addControlItem(ADDR_BUMPER_1, control_items.bumper[0]);
  dxl_slave.addControlItem(ADDR_BUMPER_2, control_items.bumper[1]);

  // Items for Battery
  dxl_slave.addControlItem(ADDR_BATTERY_VOLTAGE, control_items.bat_voltage_x100);
  dxl_slave.addControlItem(ADDR_BATTERY_PERCENT, control_items.bat_percent_x100);
  // Items for Buzzer
  dxl_slave.addControlItem(ADDR_SOUND, control_items.buzzer_sound);

  
  control_items.joint_goal_position_wr_joint = false;
  control_items.joint_goal_position_wr_gripper = false;
  control_items.joint_goal_position_rd = false;
  control_items.joint_profile_acc_wr_joint = false;
  control_items.joint_profile_acc_wr_gripper = false;
  control_items.joint_profile_acc_rd = false;
  control_items.joint_profile_vel_wr_joint = false;
  control_items.joint_profile_vel_wr_gripper = false;
  control_items.joint_profile_vel_rd = false;
  control_items.joint_goal_current_wr_joint = false;
  control_items.joint_goal_current_wr_gripper = false;
  control_items.joint_goal_current_rd = false;

    // Items to joint motors
  dxl_slave.addControlItem(ADDR_TORQUE_JOINT, control_items.joint_torque_enable_state);

  dxl_slave.addControlItem(ADDR_GOAL_POSITION_JOINT_1, control_items.joint_goal_position.value[JOINT_1]);
  dxl_slave.addControlItem(ADDR_GOAL_POSITION_JOINT_2, control_items.joint_goal_position.value[JOINT_2]);
  dxl_slave.addControlItem(ADDR_GOAL_POSITION_JOINT_3, control_items.joint_goal_position.value[JOINT_3]);
  dxl_slave.addControlItem(ADDR_GOAL_POSITION_JOINT_4, control_items.joint_goal_position.value[JOINT_4]);
  dxl_slave.addControlItem(ADDR_GOAL_POSITION_JOINT_5, control_items.joint_goal_position.value[JOINT_5]);
  dxl_slave.addControlItem(ADDR_GOAL_POSITION_JOINT_6, control_items.joint_goal_position.value[JOINT_6]);
  dxl_slave.addControlItem(ADDR_GOAL_POSITION_JOINT_7, control_items.joint_goal_position.value[JOINT_7]);
  dxl_slave.addControlItem(ADDR_GOAL_POSITION_GRIPPER, control_items.joint_goal_position.value[GRIPPER]);
  dxl_slave.addControlItem(ADDR_GOAL_POSITION_WR_JOINT, control_items.joint_goal_position_wr_joint);
  dxl_slave.addControlItem(ADDR_GOAL_POSITION_WR_GRIPPER, control_items.joint_goal_position_wr_gripper);
  dxl_slave.addControlItem(ADDR_GOAL_POSITION_RD, control_items.joint_goal_position_rd);

  dxl_slave.addControlItem(ADDR_PRESENT_POSITION_JOINT_1, control_items.joint_present_position.value[JOINT_1]);
  dxl_slave.addControlItem(ADDR_PRESENT_POSITION_JOINT_2, control_items.joint_present_position.value[JOINT_2]);
  dxl_slave.addControlItem(ADDR_PRESENT_POSITION_JOINT_3, control_items.joint_present_position.value[JOINT_3]);
  dxl_slave.addControlItem(ADDR_PRESENT_POSITION_JOINT_4, control_items.joint_present_position.value[JOINT_4]);
  dxl_slave.addControlItem(ADDR_PRESENT_POSITION_JOINT_5, control_items.joint_present_position.value[JOINT_5]);
  dxl_slave.addControlItem(ADDR_PRESENT_POSITION_JOINT_6, control_items.joint_present_position.value[JOINT_6]);
  dxl_slave.addControlItem(ADDR_PRESENT_POSITION_JOINT_7, control_items.joint_present_position.value[JOINT_7]);
  dxl_slave.addControlItem(ADDR_PRESENT_POSITION_GRIPPER, control_items.joint_present_position.value[GRIPPER]);

  dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_JOINT_1, control_items.joint_present_velocity.value[JOINT_1]);
  dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_JOINT_2, control_items.joint_present_velocity.value[JOINT_2]);
  dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_JOINT_3, control_items.joint_present_velocity.value[JOINT_3]);
  dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_JOINT_4, control_items.joint_present_velocity.value[JOINT_4]);
  dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_JOINT_5, control_items.joint_present_velocity.value[JOINT_5]);
  dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_JOINT_6, control_items.joint_present_velocity.value[JOINT_6]);
  dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_JOINT_7, control_items.joint_present_velocity.value[JOINT_7]);
  dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_GRIPPER, control_items.joint_present_velocity.value[GRIPPER]);

  dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_JOINT_1, control_items.joint_present_current.value[JOINT_1]);
  dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_JOINT_2, control_items.joint_present_current.value[JOINT_2]);
  dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_JOINT_3, control_items.joint_present_current.value[JOINT_3]);
  dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_JOINT_4, control_items.joint_present_current.value[JOINT_4]);
  dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_JOINT_5, control_items.joint_present_current.value[JOINT_5]);
  dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_JOINT_6, control_items.joint_present_current.value[JOINT_6]);
  dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_JOINT_7, control_items.joint_present_current.value[JOINT_7]);
  dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_GRIPPER, control_items.joint_present_current.value[GRIPPER]);

  dxl_slave.addControlItem(ADDR_PROFILE_ACC_JOINT_1, control_items.joint_profile_acc.value[JOINT_1]);
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_JOINT_2, control_items.joint_profile_acc.value[JOINT_2]);
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_JOINT_3, control_items.joint_profile_acc.value[JOINT_3]);
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_JOINT_4, control_items.joint_profile_acc.value[JOINT_4]);
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_JOINT_5, control_items.joint_profile_acc.value[JOINT_5]);
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_JOINT_6, control_items.joint_profile_acc.value[JOINT_6]);
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_JOINT_7, control_items.joint_profile_acc.value[JOINT_7]);
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_GRIPPER, control_items.joint_profile_acc.value[GRIPPER]);
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_WR_JOINT, control_items.joint_profile_acc_wr_joint);
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_WR_GRIPPER, control_items.joint_profile_acc_wr_gripper);
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_RD, control_items.joint_profile_acc_rd);

  dxl_slave.addControlItem(ADDR_PROFILE_VEL_JOINT_1, control_items.joint_profile_vel.value[JOINT_1]);
  dxl_slave.addControlItem(ADDR_PROFILE_VEL_JOINT_2, control_items.joint_profile_vel.value[JOINT_2]);
  dxl_slave.addControlItem(ADDR_PROFILE_VEL_JOINT_3, control_items.joint_profile_vel.value[JOINT_3]);
  dxl_slave.addControlItem(ADDR_PROFILE_VEL_JOINT_4, control_items.joint_profile_vel.value[JOINT_4]);
  dxl_slave.addControlItem(ADDR_PROFILE_VEL_JOINT_5, control_items.joint_profile_vel.value[JOINT_5]);
  dxl_slave.addControlItem(ADDR_PROFILE_VEL_JOINT_6, control_items.joint_profile_vel.value[JOINT_6]);
  dxl_slave.addControlItem(ADDR_PROFILE_VEL_JOINT_7, control_items.joint_profile_vel.value[JOINT_7]);
  dxl_slave.addControlItem(ADDR_PROFILE_VEL_GRIPPER, control_items.joint_profile_vel.value[GRIPPER]);
  dxl_slave.addControlItem(ADDR_PROFILE_VEL_WR_JOINT, control_items.joint_profile_vel_wr_joint);
  dxl_slave.addControlItem(ADDR_PROFILE_VEL_WR_GRIPPER, control_items.joint_profile_vel_wr_gripper);
  dxl_slave.addControlItem(ADDR_PROFILE_VEL_RD, control_items.joint_profile_vel_rd);

  dxl_slave.addControlItem(ADDR_GOAL_CURRENT_JOINT_1, control_items.joint_goal_current.value[JOINT_1]);
  dxl_slave.addControlItem(ADDR_GOAL_CURRENT_JOINT_2, control_items.joint_goal_current.value[JOINT_2]);
  dxl_slave.addControlItem(ADDR_GOAL_CURRENT_JOINT_3, control_items.joint_goal_current.value[JOINT_3]);
  dxl_slave.addControlItem(ADDR_GOAL_CURRENT_JOINT_4, control_items.joint_goal_current.value[JOINT_4]);
  dxl_slave.addControlItem(ADDR_GOAL_CURRENT_JOINT_5, control_items.joint_goal_current.value[JOINT_5]);
  dxl_slave.addControlItem(ADDR_GOAL_CURRENT_JOINT_6, control_items.joint_goal_current.value[JOINT_6]);
  dxl_slave.addControlItem(ADDR_GOAL_CURRENT_JOINT_7, control_items.joint_goal_current.value[JOINT_7]);
  dxl_slave.addControlItem(ADDR_GOAL_CURRENT_GRIPPER, control_items.joint_goal_current.value[GRIPPER]);
  dxl_slave.addControlItem(ADDR_GOAL_CURRENT_WR_JOINT, control_items.joint_goal_current_wr_joint);
  dxl_slave.addControlItem(ADDR_GOAL_CURRENT_WR_GRIPPER, control_items.joint_goal_current_wr_gripper);
  dxl_slave.addControlItem(ADDR_GOAL_CURRENT_RD, control_items.joint_goal_current_rd);    

  // Set user callback function for processing write command from master.
  dxl_slave.setWriteCallbackFunc(dxl_slave_write_callback_func);

  if(manipulator_driver.is_connected() == true){
    manipulator_driver.set_torque(true);    
    control_items.is_connect_manipulator = true;
    set_connection_state_with_joints(true);
    DEBUG_PRINTLN("Joint motors are connected");      
  }else{
    control_items.is_connect_manipulator = false;
    set_connection_state_with_joints(false);
      DEBUG_PRINTLN("Can't communicate with the joint!");
      DEBUG_PRINTLN("  Please check the connection to the joint motor and the power supply.");
      DEBUG_PRINTLN();
  } 

  //To indicate that the initialization is complete.
  sensors.makeMelody(1); 

  DEBUG_PRINTLN("Begin End...");
}

/*******************************************************************************
* Definition for xStingCore 'run()' function
*******************************************************************************/
void xStingCore::run()
{
  // Check connection state with ROS2 node
  update_connection_state_with_ros2_node();

  /* For diagnosis */
  // Show LED status
  diagnosis.showLedStatus(get_connection_state_with_ros2_node());
  // Update Voltage
  diagnosis.updateVoltageCheck(true);

  /* For run buzzer */

  // Run buzzer if there is still melody to play.
  sensors.onMelody();

  /* For processing DYNAMIXEL slave function */
  // Update control table of OpenCR to communicate with ROS2 node
  update_times(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_gpios(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_battery_status(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_joint_status(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);

  // Packet processing with ROS2 Node.
  dxl_slave.processPacket();
}

/*******************************************************************************
* Function definition for updating control items in TB3.
*******************************************************************************/
float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void update_times(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    control_items.dev_time_millis = millis();
  } 
}

void update_gpios(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    control_items.user_led[0] = digitalRead(BDPIN_GPIO_4);
    control_items.user_led[1] = digitalRead(BDPIN_GPIO_6);
    control_items.user_led[2] = digitalRead(BDPIN_GPIO_8);
    control_items.user_led[3] = digitalRead(BDPIN_GPIO_10);

    control_items.push_button[0] = digitalRead(BDPIN_PUSH_SW_1);
    control_items.push_button[1] = digitalRead(BDPIN_PUSH_SW_2);

    control_items.bumper[0] = sensors.getBumper1State();
    control_items.bumper[1] = sensors.getBumper2State();
  }  
}

void update_battery_status(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;
  float bat_voltage, bat_percent;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    bat_voltage = sensors.checkVoltage();
    control_items.bat_voltage_x100 = (uint32_t)(bat_voltage*100);

    if(bat_voltage >= 3.5*3){
      bat_percent = map_float(bat_voltage, 3.5*3, 4.1*3, 0.0, 100.0);
      control_items.bat_percent_x100 = (uint32_t)(bat_percent*100);
    }
  }
}

void update_joint_status(uint32_t interval_ms)
{
  static uint32_t pre_time;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    manipulator_driver.read_present_position(control_items.joint_present_position);
    manipulator_driver.read_present_velocity(control_items.joint_present_velocity);
    manipulator_driver.read_present_current(control_items.joint_present_current);

    if(get_connection_state_with_joints() == true){

      control_items.joint_torque_enable_state = manipulator_driver.get_torque();
    }
  }  
}

/*******************************************************************************
* Callback function definition to be used in communication with the ROS2 node.
*******************************************************************************/
static void dxl_slave_write_callback_func(uint16_t item_addr, uint8_t &dxl_err_code, void* arg)
{
  (void)arg;

  switch(item_addr)
  {
    case ADDR_MODEL_INFORM:
      control_items.model_inform = MODEL_INFO_XSTING;
      dxl_err_code = DXL_ERR_ACCESS;
      break;

    case ADDR_DEBUG_MODE:
      if (control_items.debug_mode == true)
        DEBUG_PRINTLN("Debug Mode : Enabled");
      else
        DEBUG_PRINTLN("Debug Mode : Disabled");
      break;

    case ADDR_SOUND:
      sensors.makeMelody(control_items.buzzer_sound);
      break;

    case ADDR_TORQUE_JOINT:
      manipulator_driver.set_torque(control_items.joint_torque_enable_state);
      break;

    // ADDR_GOAL_POSITION
    //
    case ADDR_GOAL_POSITION_WR_JOINT:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_goal_position_wr_joint == true) {
        manipulator_driver.write_goal_position_joint(control_items.joint_goal_position);
      }
      control_items.joint_goal_position_wr_joint = false;
      break;

    case ADDR_GOAL_POSITION_WR_GRIPPER:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_goal_position_wr_gripper == true) {
        manipulator_driver.write_goal_position_gripper(control_items.joint_goal_position);
      }
      control_items.joint_goal_position_wr_gripper = false;
      break;

    case ADDR_GOAL_POSITION_RD:
      if (control_items.joint_goal_position_rd == true) {
        manipulator_driver.read_goal_position(control_items.joint_goal_position);
      }
      control_items.joint_goal_position_rd = false;
      break;

    // ADDR_PROFILE_ACC
    //
    case ADDR_PROFILE_ACC_WR_JOINT:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_profile_acc_wr_joint == true) {
        manipulator_driver.write_profile_acceleration_joint(control_items.joint_profile_acc);
      }
      control_items.joint_profile_acc_wr_joint = false;
      break;      

    case ADDR_PROFILE_ACC_WR_GRIPPER:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_profile_acc_wr_gripper == true) {
        manipulator_driver.write_profile_acceleration_gripper(control_items.joint_profile_acc);
      }
      control_items.joint_profile_acc_wr_gripper = false;
      break;      

    case ADDR_PROFILE_ACC_RD:
      if (control_items.joint_profile_acc_rd == true) {
        manipulator_driver.read_profile_acceleration(control_items.joint_profile_acc);
      }
      control_items.joint_profile_acc_rd = false;
      break;     

    // ADDR_PROFILE_VEL
    //
    case ADDR_PROFILE_VEL_WR_JOINT:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_profile_vel_wr_joint == true) {
        manipulator_driver.write_profile_velocity_joint(control_items.joint_profile_vel);
      }
      control_items.joint_profile_vel_wr_joint = false;
      break;      

    case ADDR_PROFILE_VEL_WR_GRIPPER:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_profile_vel_wr_gripper == true) {
        manipulator_driver.write_profile_velocity_gripper(control_items.joint_profile_vel);
      }
      control_items.joint_profile_vel_wr_gripper = false;
      break;   

    case ADDR_PROFILE_VEL_RD:
      if (control_items.joint_profile_vel_rd == true) {
        manipulator_driver.read_profile_velocity(control_items.joint_profile_vel);
      }
      control_items.joint_profile_vel_rd = false;
      break;      

    // ADDR_GOAL_CURRENT
    //
    case ADDR_GOAL_CURRENT_WR_JOINT:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_goal_current_wr_joint == true) {
        manipulator_driver.write_goal_current_joint(control_items.joint_goal_current);
      }
      control_items.joint_goal_current_wr_joint = false;
      break;      

    case ADDR_GOAL_CURRENT_WR_GRIPPER:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_goal_current_wr_gripper == true) {
        manipulator_driver.write_goal_current_gripper(control_items.joint_goal_current);
      }
      control_items.joint_goal_current_wr_gripper = false;
      break;   

    case ADDR_GOAL_CURRENT_RD:
      if (control_items.joint_goal_current_rd == true) {
        manipulator_driver.read_goal_current(control_items.joint_goal_current);
      }
      control_items.joint_goal_current_rd = false;
      break;        
  }
}


/*******************************************************************************
* Function definition to check the connection status with the ROS2 node.
*******************************************************************************/
static bool connection_state_with_ros2_node = false;

static bool get_connection_state_with_ros2_node()
{
  return connection_state_with_ros2_node;
}

static void set_connection_state_with_ros2_node(bool is_connected)
{
  connection_state_with_ros2_node = is_connected;
}

void update_connection_state_with_ros2_node()
{
  static uint32_t pre_time;
  static uint8_t pre_data;
  static bool pre_state;

  //To wait for IMU Calibration
  if(pre_state != get_connection_state_with_ros2_node()){
    pre_state = get_connection_state_with_ros2_node();
    pre_time = millis();
    return;
  }

  if(pre_data != control_items.heart_beat || control_items.debug_mode == true){
    pre_time = millis();
    pre_data = control_items.heart_beat;
    set_connection_state_with_ros2_node(true);
  }else{
    if(millis()-pre_time >= HEARTBEAT_TIMEOUT_MS){
      pre_time = millis();
      if(get_connection_state_with_ros2_node() == true){
        hold_joints_at_present_position();
      }
      set_connection_state_with_ros2_node(false);
    }
  }

  control_items.is_connect_ros2_node = get_connection_state_with_ros2_node();
}

// Freeze the arm where it is, so a dead host can't leave it moving toward an old goal.
static void hold_joints_at_present_position()
{
  manipulator_driver.write_goal_position_joint(control_items.joint_present_position);
  manipulator_driver.write_goal_position_gripper(control_items.joint_present_position);
}

/*******************************************************************************
* Function definition to check the connection with the motor.
*******************************************************************************/
static bool is_connected_joints = false;

static bool get_connection_state_with_joints()
{
  return is_connected_joints;
}

static void set_connection_state_with_joints(bool is_connected)
{
  is_connected_joints = is_connected;
}