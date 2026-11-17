#ifndef XSTING_WALK_WALK_CTRL_H_
#define XSTING_WALK_WALK_CTRL_H_

#include <Eigen/Dense>
#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include "xsting_walk/calib.h"
#include "xsting_walk/mlp.h"

namespace xsting_walk {

inline constexpr int kObsDim = 74;

// 50Hz 보행 제어 상태 텔레메트리 요약
struct WalkState {
  bool enabled = true;
  bool moving = false;
  bool using_odom_vel = false;
  float phase = 0.0f;
  Eigen::Vector3f user_cmd = Eigen::Vector3f::Zero();
  Eigen::Vector3f eff_cmd = Eigen::Vector3f::Zero();
  Eigen::Vector2f est_vel_xy = Eigen::Vector2f::Zero();
  float imu_wz = 0.0f;
  float body_yaw_rad = 0.0f;
  float heading_err_rad = 0.0f;
};

// xSting 74D CPG + RL 잔차 전방향 보행·회전 C++ 제어기
class WalkCtrl {
 public:
  bool Init(const std::string& model_bin,
            const Calib& calib,
            std::string* err = nullptr);

  void SetCalib(const Calib& calib);
  void Reset();
  void SetEnabled(bool enabled);
  void StopImmediate();

  // 사용자 텔레옵 목표 속도 설정 (vx [m/s], vy [m/s], wz [rad/s])
  void SetTeleopCmd(const Eigen::Vector3f& cmd_vel);

  // play.py 호환 명명 모드("forward", "left", "turn_left", "curve_left" 등) 변환
  static bool NamedModeToCmd(const std::string& mode, Eigen::Vector3f* out_cmd);

  // 센서 피드백 갱신
  void UpdateJoints(const std::vector<std::string>& names,
                    const std::vector<double>& pos,
                    const std::vector<double>& vel);
  void UpdateImu(const Eigen::Vector3d& gyro_raw,
                 const Eigen::Vector3d& acc_raw,
                 bool has_acc,
                 const Eigen::Quaterniond& quat_raw);
  void UpdateOdomVel(const Eigen::Vector2f& vel_xy, bool valid);

  // 50Hz(dt=0.02s) 74D 관측 생성, MLP 추론 및 CPG 목표 관절각 갱신
  WalkState StepPolicy(float dt_s = 0.02f);

  // 200Hz(dt=0.005s) 보간·슬루율 제한 및 하드웨어 명령 벡터 출력
  void StepHardware(float dt_s,
                    std::vector<double>* leg_cmds,
                    std::vector<double>* arm_cmds);

  // 단일 다리 URDF 순기구학(발끝 base 좌표) 및 3x3 자코비안 계산
  static Eigen::Vector3f LegFk(int leg_idx, const Eigen::Vector3f& q_leg);
  static Eigen::Matrix3f LegJacobian(int leg_idx,
                                     const Eigen::Vector3f& q_leg,
                                     float eps = 0.02f);

  bool enabled() const { return enabled_; }
  bool moving() const { return state_.moving; }
  bool has_joints() const { return has_joints_; }
  const Calib& calib() const { return calib_; }
  const WalkState& state() const { return state_; }
  const Eigen::Matrix<float, kObsDim, 1>& last_obs() const { return last_obs_; }
  const std::array<Eigen::Matrix3f, 6>& jac_inv() const { return jac_inv_; }
  const std::array<Eigen::Vector2f, 6>& foot_home_xy() const {
    return foot_home_xy_;
  }
  const Eigen::Vector3f& ang_vel_b() const { return ang_vel_b_; }
  const Eigen::Vector3f& lin_acc_b() const { return lin_acc_b_; }
  const Eigen::Vector3f& proj_grav_b() const { return proj_grav_b_; }
  const Eigen::Matrix<float, kNumLegs, 1>& q_sim() const { return q_sim_; }
  const Eigen::Matrix<float, kNumLegs, 1>& target_legs() const {
    return target_legs_;
  }

 private:
  void RebuildMapAndKinematics();
  void UpdateKinematicVel();

  Mlp mlp_;
  Calib calib_;
  std::unordered_map<std::string, int> ros_to_leg_;
  std::array<int, kNumLegs> out_to_leg_{};

  std::array<Eigen::Matrix3f, 6> jac_inv_{};
  std::array<Eigen::Vector2f, 6> foot_home_xy_{};

  Eigen::Matrix<float, kNumLegs, 1> q_def_ =
      Eigen::Matrix<float, kNumLegs, 1>::Zero();
  Eigen::Matrix<float, kNumLegs, 1> home_legs_ =
      Eigen::Matrix<float, kNumLegs, 1>::Zero();
  Eigen::Matrix<float, kNumLegs, 1> soft_low_ =
      Eigen::Matrix<float, kNumLegs, 1>::Constant(-3.14f);
  Eigen::Matrix<float, kNumLegs, 1> soft_high_ =
      Eigen::Matrix<float, kNumLegs, 1>::Constant(3.14f);
  Eigen::Matrix<float, kNumLegs, 1> hard_low_ =
      Eigen::Matrix<float, kNumLegs, 1>::Constant(-3.14f);
  Eigen::Matrix<float, kNumLegs, 1> hard_high_ =
      Eigen::Matrix<float, kNumLegs, 1>::Constant(3.14f);

  Eigen::Matrix<float, kNumLegs, 1> q_sim_ =
      Eigen::Matrix<float, kNumLegs, 1>::Zero();
  Eigen::Matrix<float, kNumLegs, 1> dq_sim_ =
      Eigen::Matrix<float, kNumLegs, 1>::Zero();
  Eigen::Matrix<float, kNumLegs, 1> target_legs_ =
      Eigen::Matrix<float, kNumLegs, 1>::Zero();
  Eigen::Matrix<float, kNumLegs, 1> interp_start_q_ =
      Eigen::Matrix<float, kNumLegs, 1>::Zero();
  Eigen::Matrix<float, kNumLegs, 1> slew_q_ =
      Eigen::Matrix<float, kNumLegs, 1>::Zero();

  Eigen::Matrix<float, kNumLegs, 1> raw_actions_ =
      Eigen::Matrix<float, kNumLegs, 1>::Zero();
  Eigen::Matrix<float, kNumLegs, 1> prev_raw_actions_ =
      Eigen::Matrix<float, kNumLegs, 1>::Zero();
  Eigen::Matrix<float, kNumLegs, 1> actions_ =
      Eigen::Matrix<float, kNumLegs, 1>::Zero();
  Eigen::Matrix<float, kObsDim, 1> last_obs_ =
      Eigen::Matrix<float, kObsDim, 1>::Zero();

  Eigen::Vector3f ang_vel_b_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f lin_acc_b_ = Eigen::Vector3f(0.0f, 0.0f, 9.81f);
  Eigen::Vector3f proj_grav_b_ = Eigen::Vector3f(0.0f, 0.0f, -1.0f);
  float body_yaw_ = 0.0f;
  bool has_imu_quat_ = false;
  bool has_joints_ = false;

  Eigen::Vector2f odom_vel_xy_ = Eigen::Vector2f::Zero();
  bool odom_valid_ = false;
  Eigen::Vector2f kin_vel_xy_ = Eigen::Vector2f::Zero();

  Eigen::Vector3f user_cmd_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f smoothed_user_cmd_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f cmd_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f prev_cmd_ = Eigen::Vector3f::Zero();

  float phase_ = 0.0f;
  float interp_alpha_ = 1.0f;
  float heading_ref_ = 0.0f;
  Eigen::Vector2f vel_int_ = Eigen::Vector2f::Zero();

  bool enabled_ = true;
  bool was_moving_ = false;
  WalkState state_{};
};

}  // namespace xsting_walk

#endif  // XSTING_WALK_WALK_CTRL_H_
