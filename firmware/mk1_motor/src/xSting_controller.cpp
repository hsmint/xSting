#include "../include/xSting_controller.hpp"

xStingController::xStingController()
{
  const_cmd_vel_ = CONST_VEL;
}

xStingController::~xStingController()
{}

bool xStingController::init(float max_lin_vel, float max_ang_vel, uint8_t scale_lin_vel, uint8_t scale_ang_vel)
{
  // 57600bps baudrate for RC100 control
  rc100_.begin(1);  

  max_lin_vel_ = max_lin_vel;
  min_lin_vel_ = (-1)*max_lin_vel;
  max_ang_vel_ = max_ang_vel;
  min_ang_vel_ = (-1)*max_ang_vel;
  scale_lin_vel_ = scale_lin_vel;
  scale_ang_vel_ = scale_ang_vel;

  return true;
}

void xStingController::getRCdata(float *get_cmd_vel)
{
  uint16_t received_data = 0;

  static float lin_x = 0.0, ang_z = 0.0;
  static uint32_t last_rx_time = 0;

  if (rc100_.available())
  {
    received_data = rc100_.readData();
    last_rx_time = millis();

    if (received_data & RC100_BTN_U)
    {
      lin_x += VELOCITY_LINEAR_X * scale_lin_vel_;
    }
    else if (received_data & RC100_BTN_D)
    {
      lin_x -= VELOCITY_LINEAR_X * scale_lin_vel_;
    }
    else if (received_data & RC100_BTN_L)
    {
      ang_z += VELOCITY_ANGULAR_Z * scale_ang_vel_;
    }
    else if (received_data & RC100_BTN_R)
    {
      ang_z -= VELOCITY_ANGULAR_Z * scale_ang_vel_;
    }
    else if (received_data & RC100_BTN_6)
    {
      lin_x = const_cmd_vel_;
      ang_z = 0.0;
    }
    else if (received_data & RC100_BTN_5)
    {
      lin_x = 0.0;
      ang_z = 0.0;
    }

    lin_x = constrain(lin_x, min_lin_vel_, max_lin_vel_);
    ang_z = constrain(ang_z, min_ang_vel_, max_ang_vel_);

    get_cmd_vel[0] = lin_x;
    get_cmd_vel[1] = ang_z;
  }
  else if (millis() - last_rx_time >= RC100_TIMEOUT_MS)
  {
    lin_x = 0.0;
    ang_z = 0.0;
    get_cmd_vel[0] = 0.0;
    get_cmd_vel[1] = 0.0;
  }
}