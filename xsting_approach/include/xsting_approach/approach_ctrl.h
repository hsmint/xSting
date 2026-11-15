#ifndef XSTING_APPROACH_APPROACH_CTRL_H_
#define XSTING_APPROACH_APPROACH_CTRL_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include "xsting_approach/calib.h"
#include "xsting_approach/mlp.h"

namespace xsting_approach {

// 50Hz 정밀 접근 제어기 실시간 상태 요약
struct CtrlState {
  bool active = false;
  bool stopped = true;
  bool aligned = false;
  bool reached = false;
  float visible = 0.0f;
  float dist_m = 0.0f;
  float yaw_rad = 0.0f;
  float hold_sec = 0.0f;
  float range_m = 4.0f;
  bool range_valid = false;
  Eigen::Vector3f action = Eigen::Vector3f::Zero();
  Eigen::Vector3f cmd_vel = Eigen::Vector3f::Zero();
};

// 계층형 강화학습(50Hz Approach MLP + 200Hz WalkNet) 정밀 접근 제어기
class ApproachCtrl {
 public:
  // 두 신경망 가중치 및 보정 데이터 로드
  bool Init(const std::string& approach_bin,
            const std::string& walk_bin,
            const Calib& calib,
            std::string* err = nullptr);

  // 보정 갱신, 내부 상태 리셋 및 주행 활성화 전환
  void SetCalib(const Calib& calib);
  void Reset();
  void SetActive(bool active);

  // 하드웨어 관절·IMU·초음파·오도메트리 센서 상태 입력
  void UpdateJoints(const std::vector<std::string>& names,
                    const std::vector<double>& pos,
                    const std::vector<double>& vel);
  void UpdateImu(const Eigen::Vector3d& gyro_raw,
                 const Eigen::Quaterniond& quat_raw);
  void UpdateSonar(double raw_range_m, bool valid);
  void UpdateVel(const Eigen::Vector2f& vel_xy, float wz);

  // 50Hz 상위 접근 정책 추론 및 200Hz 하위 다리·팔 목표각 생성
  CtrlState StepPolicy(const Eigen::Matrix<float, 9, 1>& vis_obs, float dt_s);
  void StepWalk(float dt_s,
                std::vector<double>* leg_cmds,
                std::vector<double>* arm_cmds);

  bool active() const { return active_; }
  bool reached() const { return reached_; }
  bool stopped() const { return stopped_; }
  float us_raw_m() const { return us_raw_m_; }
  const Eigen::Vector3f& proj_grav_b() const { return proj_grav_b_; }
  const CtrlState& state() const { return state_; }
  const Calib& calib() const { return calib_; }

 private:
  // ROS 관절명과 WalkNet 내부 18축 순서 매핑 재구성
  void RebuildMap();

  Mlp approach_mlp_;
  Mlp walk_mlp_;
  Calib calib_;

  Eigen::Matrix<float, kNumLegs, 1> q_def_ =
      Eigen::Matrix<float, kNumLegs, 1>::Zero();
  Eigen::Matrix<float, kNumLegs, 1> home_legs_ =
      Eigen::Matrix<float, kNumLegs, 1>::Zero();
  Eigen::Matrix<float, kNumLegs, 1> leg_low_ =
      Eigen::Matrix<float, kNumLegs, 1>::Zero();
  Eigen::Matrix<float, kNumLegs, 1> leg_high_ =
      Eigen::Matrix<float, kNumLegs, 1>::Zero();

  std::unordered_map<std::string, int> ros_to_walk_;
  std::array<int, kNumLegs> out_to_walk_{};

  bool has_joints_ = false;
  Eigen::Matrix<float, kNumLegs, 1> q_sim_ =
      Eigen::Matrix<float, kNumLegs, 1>::Zero();
  Eigen::Matrix<float, kNumLegs, 1> dq_sim_ =
      Eigen::Matrix<float, kNumLegs, 1>::Zero();
  Eigen::Matrix<float, kNumLegs, 1> stance_q_ =
      Eigen::Matrix<float, kNumLegs, 1>::Zero();
  Eigen::Matrix<float, kNumLegs, 1> slew_q_ =
      Eigen::Matrix<float, kNumLegs, 1>::Zero();

  Eigen::Vector3f ang_vel_b_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f proj_grav_b_ = Eigen::Vector3f(0.0f, 0.0f, -1.0f);
  Eigen::Vector2f body_vel_xy_ = Eigen::Vector2f::Zero();
  bool has_vel_ = false;
  bool has_gyro_ = false;

  float us_raw_m_ = 4.0f;
  float us_range_m_ = 4.0f;
  bool us_valid_ = false;

  bool seen_box_ = false;
  Eigen::Vector2f box_xy_ = Eigen::Vector2f::Zero();
  Eigen::Vector2f norm_xy_ = Eigen::Vector2f::UnitX();
  Eigen::Vector2f p1_rel_ = Eigen::Vector2f(0.0f, -0.04f);
  Eigen::Vector2f p2_rel_ = Eigen::Vector2f(0.0f, 0.04f);

  Eigen::Vector3f action_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f cmd_ema_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f cmd_ = Eigen::Vector3f::Zero();
  float phase_ = 0.0f;

  bool active_ = false;
  bool stopped_ = true;
  bool reached_ = false;
  float hold_sec_ = 0.0f;
  CtrlState state_;
};

}  // namespace xsting_approach

#endif  // XSTING_APPROACH_APPROACH_CTRL_H_
