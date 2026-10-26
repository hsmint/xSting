#pragma once

#include <RC100.h>

#define CONST_VEL 0.2

#define VELOCITY_LINEAR_X                0.01   // m/s
#define VELOCITY_ANGULAR_Z               0.1    // rad/s

#define RC100_TIMEOUT_MS                 1000   // zero RC100 command if no packet for this long

class xStingController
{
 public:
  xStingController();
  ~xStingController();

  bool init(float max_lin_vel, float max_ang_vel, uint8_t scale_lin_vel = 1, uint8_t scale_ang_vel = 1);

  void getRCdata(float *get_cmd_vel);

 private:
  RC100 rc100_;

  double const_cmd_vel_;

  float max_lin_vel_;
  float min_lin_vel_;
  float max_ang_vel_;
  float min_ang_vel_;
  uint8_t scale_lin_vel_;
  uint8_t scale_ang_vel_;
};