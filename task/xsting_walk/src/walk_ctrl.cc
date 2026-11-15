#include "xsting_walk/walk_ctrl.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace xsting_walk {

namespace {

constexpr std::array<float, 6> kTripodOffsets = {0.0f, 0.5f, 0.0f,
                                                 0.5f, 0.0f, 0.5f};

// xsting.urdf 기준 다리별 yaw 원점(base 프레임)
constexpr std::array<std::array<float, 3>, 6> kYawOrigins = {{
    {0.11f, 0.10f, 0.0724719f},
    {0.00f, 0.10f, 0.0724719f},
    {-0.11f, 0.10f, 0.0724719f},
    {0.11f, -0.10f, 0.0724719f},
    {0.00f, -0.10f, 0.0724719f},
    {-0.11f, -0.10f, 0.0724719f},
}};

inline float WrapPi(float a) {
  return std::atan2(std::sin(a), std::cos(a));
}

}  // namespace

Eigen::Vector3f WalkCtrl::LegFk(int leg_idx, const Eigen::Vector3f& q_leg) {
  const int leg = std::clamp(leg_idx, 0, 5);
  const bool left = (leg < 3);
  const float side = left ? 1.0f : -1.0f;

  const Eigen::Vector3f p_yaw(kYawOrigins[leg][0], kYawOrigins[leg][1],
                              kYawOrigins[leg][2]);
  const Eigen::Vector3f axis_yaw(0.0f, 0.0f, left ? -1.0f : 1.0f);
  const Eigen::Vector3f axis_hk(left ? 1.0f : -1.0f, 0.0f, 0.0f);

  const Eigen::Vector3f p_hip(0.0f, side * 0.0605f, 0.00025f);
  const Eigen::Vector3f p_knee(0.0f, side * 0.081f, 0.0f);
  const Eigen::Vector3f p_tip(0.0f, side * 0.13275f, -0.02f);

  Eigen::Matrix3f r = Eigen::AngleAxisf(q_leg[0], axis_yaw).toRotationMatrix();
  Eigen::Vector3f p = p_yaw + r * p_hip;
  r = r * Eigen::AngleAxisf(q_leg[1], axis_hk).toRotationMatrix();
  p += r * p_knee;
  r = r * Eigen::AngleAxisf(q_leg[2], axis_hk).toRotationMatrix();
  p += r * p_tip;
  return p;
}

Eigen::Matrix3f WalkCtrl::LegJacobian(int leg_idx,
                                      const Eigen::Vector3f& q_leg,
                                      float eps) {
  const Eigen::Vector3f f0 = LegFk(leg_idx, q_leg);
  Eigen::Matrix3f jac;
  for (int j = 0; j < 3; ++j) {
    Eigen::Vector3f qp = q_leg;
    qp[j] += eps;
    jac.col(j) = (LegFk(leg_idx, qp) - f0) / eps;
  }
  return jac;
}

bool WalkCtrl::NamedModeToCmd(const std::string& mode,
                              Eigen::Vector3f* out_cmd) {
  if (!out_cmd) return false;
  if (mode == "stop") {
    *out_cmd = {0.0f, 0.0f, 0.0f};
  } else if (mode == "forward") {
    *out_cmd = {0.35f, 0.0f, 0.0f};
  } else if (mode == "backward") {
    *out_cmd = {-0.35f, 0.0f, 0.0f};
  } else if (mode == "left") {
    *out_cmd = {0.0f, 0.30f, 0.0f};
  } else if (mode == "right") {
    *out_cmd = {0.0f, -0.30f, 0.0f};
  } else if (mode == "diag_fl") {
    *out_cmd = {0.25f, 0.23f, 0.0f};
  } else if (mode == "diag_fr") {
    *out_cmd = {0.25f, -0.23f, 0.0f};
  } else if (mode == "diag_rl") {
    *out_cmd = {-0.25f, 0.23f, 0.0f};
  } else if (mode == "diag_rr") {
    *out_cmd = {-0.25f, -0.23f, 0.0f};
  } else if (mode == "turn_left") {
    *out_cmd = {0.0f, 0.0f, 1.25f};
  } else if (mode == "turn_right") {
    *out_cmd = {0.0f, 0.0f, -1.25f};
  } else if (mode == "curve_left") {
    *out_cmd = {0.28f, 0.0f, 0.95f};
  } else if (mode == "curve_right") {
    *out_cmd = {0.28f, 0.0f, -0.95f};
  } else {
    return false;
  }
  return true;
}

bool WalkCtrl::Init(const std::string& model_bin,
                    const Calib& calib,
                    std::string* err) {
  if (!mlp_.Load(model_bin, err)) return false;
  if (mlp_.in_dim() != kObsDim || mlp_.out_dim() != kNumLegs) {
    if (err) *err = "Walk MLP 입출력 차원 오류 (74 -> 18 필요)";
    return false;
  }
  if (!mlp_.extras().empty() && mlp_.extras()[0].size() == kNumLegs) {
    q_def_ = mlp_.extras()[0];
  }

  SetCalib(calib);
  Reset();
  return true;
}

void WalkCtrl::SetCalib(const Calib& calib) {
  calib_ = calib;
  const double sf = std::clamp(calib_.gait.soft_limit_factor, 0.50, 1.00);
  for (int i = 0; i < kNumLegs; ++i) {
    home_legs_[i] = static_cast<float>(calib_.legs[i].stand_rad);
    hard_low_[i] = static_cast<float>(calib_.legs[i].lower_rad);
    hard_high_[i] = static_cast<float>(calib_.legs[i].upper_rad);
    soft_low_[i] = static_cast<float>(calib_.legs[i].SoftLower(sf));
    soft_high_[i] = static_cast<float>(calib_.legs[i].SoftUpper(sf));
  }
  if (q_def_.squaredNorm() < 1e-6f) q_def_ = home_legs_;
  RebuildMapAndKinematics();
}

void WalkCtrl::RebuildMapAndKinematics() {
  ros_to_leg_.clear();
  for (int i = 0; i < kNumLegs; ++i) {
    ros_to_leg_[calib_.legs[i].ros_name] = i;
    ros_to_leg_[calib_.legs[i].name] = i;
  }
  for (int out_i = 0; out_i < kNumLegs; ++out_i) {
    int mapped = out_i;
    if (static_cast<size_t>(out_i) < calib_.out_leg_order.size()) {
      const auto it = ros_to_leg_.find(calib_.out_leg_order[out_i]);
      if (it != ros_to_leg_.end()) mapped = it->second;
    }
    out_to_leg_[out_i] = mapped;
  }

  // 기본 기립각 기준 발끝 역자코비안(6x3x3) 및 홈 XY 좌표 산출 (XStingWalkEnv 동일)
  for (int leg = 0; leg < 6; ++leg) {
    const Eigen::Vector3f q0 = home_legs_.segment<3>(3 * leg);
    const Eigen::Vector3f f0 = LegFk(leg, q0);
    const Eigen::Matrix3f jac = LegJacobian(leg, q0, 0.02f);
    jac_inv_[leg] = jac.inverse();
    foot_home_xy_[leg] = f0.head<2>();
  }
}

void WalkCtrl::Reset() {
  if (!has_joints_) {
    q_sim_ = home_legs_;
    dq_sim_.setZero();
  }
  target_legs_ = home_legs_;
  interp_start_q_ = has_joints_ ? q_sim_ : home_legs_;
  slew_q_ = interp_start_q_;
  interp_alpha_ = 1.0f;

  raw_actions_.setZero();
  prev_raw_actions_.setZero();
  actions_.setZero();
  last_obs_.setZero();

  user_cmd_.setZero();
  smoothed_user_cmd_.setZero();
  cmd_.setZero();
  prev_cmd_.setZero();
  kin_vel_xy_.setZero();

  phase_ = 0.0f;
  heading_ref_ = body_yaw_;
  vel_int_.setZero();
  was_moving_ = false;
  state_ = WalkState{};
  state_.enabled = enabled_;
}

void WalkCtrl::SetEnabled(bool enabled) {
  enabled_ = enabled;
  if (!enabled_) StopImmediate();
  state_.enabled = enabled_;
}

void WalkCtrl::StopImmediate() {
  user_cmd_.setZero();
  smoothed_user_cmd_.setZero();
  cmd_.setZero();
  prev_cmd_.setZero();
  raw_actions_.setZero();
  prev_raw_actions_.setZero();
  actions_.setZero();
  phase_ = 0.0f;
  vel_int_.setZero();
  heading_ref_ = body_yaw_;
  was_moving_ = false;
  target_legs_ = home_legs_;
  interp_start_q_ = slew_q_;
  interp_alpha_ = 1.0f;
  state_.moving = false;
  state_.user_cmd.setZero();
  state_.eff_cmd.setZero();
}

void WalkCtrl::SetTeleopCmd(const Eigen::Vector3f& cmd_vel) {
  const Eigen::Vector3f max_v = calib_.gait.max_speed.cast<float>();
  for (int i = 0; i < 3; ++i) {
    const float v = std::isfinite(cmd_vel[i]) ? cmd_vel[i] : 0.0f;
    user_cmd_[i] = std::clamp(v, -max_v[i], max_v[i]);
  }
}

void WalkCtrl::UpdateJoints(const std::vector<std::string>& names,
                            const std::vector<double>& pos,
                            const std::vector<double>& vel) {
  const size_t n = std::min(names.size(), pos.size());
  bool updated = false;
  for (size_t i = 0; i < n; ++i) {
    const auto it = ros_to_leg_.find(names[i]);
    if (it == ros_to_leg_.end()) continue;
    const int idx = it->second;
    q_sim_[idx] = static_cast<float>(calib_.legs[idx].ToSim(pos[i]));
    if (i < vel.size()) {
      dq_sim_[idx] = static_cast<float>(calib_.legs[idx].VelToSim(vel[i]));
    }
    updated = true;
  }
  if (updated && !has_joints_) {
    target_legs_ = home_legs_;
    interp_start_q_ = q_sim_;
    slew_q_ = q_sim_;
    has_joints_ = true;
  }
}

void WalkCtrl::UpdateImu(const Eigen::Vector3d& gyro_raw,
                         const Eigen::Vector3d& acc_raw,
                         bool has_acc,
                         const Eigen::Quaterniond& quat_raw) {
  float q_yaw = body_yaw_;
  bool got_quat_yaw = false;
  calib_.imu.Transform(gyro_raw, acc_raw, has_acc, quat_raw, &ang_vel_b_,
                       &lin_acc_b_, &proj_grav_b_, &q_yaw, &got_quat_yaw);
  if (got_quat_yaw) {
    if (!has_imu_quat_ && !was_moving_) heading_ref_ = q_yaw;
    body_yaw_ = q_yaw;
    has_imu_quat_ = true;
  }
}

void WalkCtrl::UpdateOdomVel(const Eigen::Vector2f& vel_xy, bool valid) {
  if (valid && std::isfinite(vel_xy.x()) && std::isfinite(vel_xy.y())) {
    odom_vel_xy_ = vel_xy;
    odom_valid_ = true;
  } else {
    odom_valid_ = false;
  }
}

void WalkCtrl::UpdateKinematicVel() {
  if (!was_moving_) {
    kin_vel_xy_ *= 0.8f;
    return;
  }
  Eigen::Vector2f sum_v = Eigen::Vector2f::Zero();
  int cnt = 0;
  for (int leg = 0; leg < 6; ++leg) {
    const float lp = std::fmod(phase_ + kTripodOffsets[leg], 1.0f);
    // 확실히 착지 중인 삼각보행 스탠스 다리만 사용해 기구학 차체 속도 추정
    if (lp >= 0.55f && lp <= 0.95f) {
      const Eigen::Vector3f q_l = q_sim_.segment<3>(3 * leg);
      const Eigen::Vector3f dq_l = dq_sim_.segment<3>(3 * leg);
      const Eigen::Vector3f p_f = LegFk(leg, q_l);
      const Eigen::Matrix3f j_f = LegJacobian(leg, q_l, 0.01f);
      const Eigen::Vector3f v_body = -(j_f * dq_l + ang_vel_b_.cross(p_f));
      sum_v += v_body.head<2>();
      ++cnt;
    }
  }
  if (cnt > 0) {
    const Eigen::Vector2f raw_v = sum_v / static_cast<float>(cnt);
    kin_vel_xy_ = 0.75f * kin_vel_xy_ + 0.25f * raw_v;
  }
}

WalkState WalkCtrl::StepPolicy(float dt_s) {
  const Eigen::Vector3f max_v = calib_.gait.max_speed.cast<float>();
  const Eigen::Vector3f acc_lim = calib_.gait.cmd_accel_lim.cast<float>();
  const Eigen::Vector3f scale = calib_.gait.cmd_vel_scale.cast<float>();
  const Eigen::Vector3f trim = calib_.gait.cmd_vel_trim.cast<float>();

  // 1) 사용자 속도 명령 가감속 스무딩
  const Eigen::Vector3f desired = enabled_ ? user_cmd_ : Eigen::Vector3f::Zero();
  for (int i = 0; i < 3; ++i) {
    const float max_dv = std::max(0.01f, acc_lim[i] * dt_s);
    const float dv =
        std::clamp(desired[i] - smoothed_user_cmd_[i], -max_dv, max_dv);
    smoothed_user_cmd_[i] += dv;
    if (std::abs(desired[i]) < 1e-4f && std::abs(smoothed_user_cmd_[i]) < 5e-3f) {
      smoothed_user_cmd_[i] = 0.0f;
    }
  }

  // 2) 실측 슬립 스케일 및 직진 편향(trim / coupling) 반영
  const bool user_moving = (smoothed_user_cmd_.head<2>().norm() > 0.01f) ||
                           (std::abs(smoothed_user_cmd_.z()) > 0.02f);
  if (enabled_ && user_moving) {
    const float ux = smoothed_user_cmd_.x();
    const float uy = smoothed_user_cmd_.y();
    const float uwz = smoothed_user_cmd_.z();
    cmd_.x() = std::clamp(ux * scale.x() + trim.x(), -max_v.x(), max_v.x());
    cmd_.y() = std::clamp(
        uy * scale.y() + trim.y() +
            static_cast<float>(calib_.gait.drift_vy_per_vx) * ux,
        -max_v.y(), max_v.y());
    cmd_.z() = std::clamp(
        uwz * scale.z() + trim.z() +
            static_cast<float>(calib_.gait.drift_wz_per_vx) * ux,
        -max_v.z(), max_v.z());
  } else {
    cmd_.setZero();
  }

  const bool moving_mask =
      enabled_ &&
      ((cmd_.head<2>().norm() > 0.01f) || (std::abs(cmd_.z()) > 0.02f));
  const bool use_odom = odom_valid_ && calib_.gait.use_odom_vel;
  const Eigen::Vector2f v_obs_xy =
      use_odom ? odom_vel_xy_.cwiseProduct(scale.head<2>()) : kin_vel_xy_;
  const Eigen::Vector2f v_real_xy = use_odom ? odom_vel_xy_ : kin_vel_xy_;

  if (!moving_mask) {
    prev_raw_actions_.setZero();
    raw_actions_.setZero();
    actions_.setZero();
    phase_ = 0.0f;
    heading_ref_ = body_yaw_;
    vel_int_.setZero();
    was_moving_ = false;
    prev_cmd_.setZero();
    target_legs_ = home_legs_;
    interp_start_q_ = slew_q_;
    interp_alpha_ = 1.0f;

    state_.enabled = enabled_;
    state_.moving = false;
    state_.using_odom_vel = use_odom;
    state_.phase = 0.0f;
    state_.user_cmd = smoothed_user_cmd_;
    state_.eff_cmd.setZero();
    state_.est_vel_xy = v_real_xy;
    state_.imu_wz = ang_vel_b_.z();
    state_.body_yaw_rad = body_yaw_;
    state_.heading_err_rad = 0.0f;
    return state_;
  }

  // 3) 헤딩 레퍼런스 및 속도 적분 오차(PI 관측) 갱신
  if (!was_moving_) {
    heading_ref_ = body_yaw_;
    vel_int_.setZero();
  } else {
    if ((cmd_ - prev_cmd_).head<2>().norm() > 0.12f) {
      vel_int_ *= 0.25f;
    }
    heading_ref_ = WrapPi(heading_ref_ + cmd_.z() * dt_s);
    const float h_clamped =
        std::clamp(WrapPi(body_yaw_ - heading_ref_), -0.35f, 0.35f);
    heading_ref_ = WrapPi(body_yaw_ - h_clamped);
    vel_int_ = (vel_int_ + (v_obs_xy - cmd_.head<2>()) * dt_s)
                   .cwiseMax(-0.06f)
                   .cwiseMin(0.06f);
  }

  // 4) XStingWalkEnv._get_observations() 동일 74D 관측 벡터 구성
  const float ang = static_cast<float>(2.0 * M_PI) * phase_;
  const float h_err = WrapPi(body_yaw_ - heading_ref_);
  const Eigen::Vector2f vel_err_xy = v_obs_xy - cmd_.head<2>();
  const float yaw_err = ang_vel_b_.z() - cmd_.z();

  last_obs_.segment<3>(0) = ang_vel_b_ * 0.25f;
  last_obs_.segment<3>(3) =
      (lin_acc_b_ - Eigen::Vector3f(0.0f, 0.0f, 9.81f)) * 0.10f;
  last_obs_.segment<3>(6) = proj_grav_b_;
  last_obs_.segment<3>(9) =
      cmd_.cwiseProduct(Eigen::Vector3f(3.0f, 3.0f, 0.8f));
  last_obs_[12] = std::sin(ang);
  last_obs_[13] = std::cos(ang);
  last_obs_.segment<2>(14) = vel_err_xy * 4.0f;
  last_obs_.segment<2>(16) = vel_int_ * 20.0f;
  last_obs_[18] = yaw_err * 1.5f;
  last_obs_[19] = h_err * 4.0f;
  last_obs_.segment<kNumLegs>(20) = q_sim_ - home_legs_;
  last_obs_.segment<kNumLegs>(38) = dq_sim_ * 0.05f;
  last_obs_.segment<kNumLegs>(56) = actions_;

  // 5) 74->256->128->64->18 정책 추론 및 XStingWalkEnv._pre_physics_step() 수행
  const Eigen::VectorXf net_act = mlp_.Forward(last_obs_);
  prev_raw_actions_ = raw_actions_;
  for (int i = 0; i < kNumLegs; ++i) {
    raw_actions_[i] =
        std::clamp(std::isfinite(net_act[i]) ? net_act[i] : 0.0f, -1.0f, 1.0f);
  }
  const float alpha =
      std::clamp(static_cast<float>(calib_.gait.action_ema), 0.05f, 1.0f);
  actions_ = (1.0f - alpha) * actions_ + alpha * raw_actions_;

  const float gait_freq =
      std::max(0.5f, static_cast<float>(calib_.gait.gait_freq_hz));
  phase_ = std::fmod(phase_ + gait_freq * dt_s, 1.0f);

  const float gx = static_cast<float>(calib_.gait.stride_gain_xy.x());
  const float gy = static_cast<float>(calib_.gait.stride_gain_xy.y());
  const float gyaw = static_cast<float>(calib_.gait.stride_gain_yaw);
  const float swing_clear =
      static_cast<float>(calib_.gait.swing_clearance_m);
  const float stance_press =
      static_cast<float>(calib_.gait.stance_press_m);
  const float act_scale = static_cast<float>(calib_.gait.action_scale);

  Eigen::Matrix<float, kNumLegs, 1> cpg_dq;
  for (int leg = 0; leg < 6; ++leg) {
    const float lp = std::fmod(phase_ + kTripodOffsets[leg], 1.0f);
    const float u = 2.0f * lp;
    const float s_swing =
        -1.0f - 2.0f * u + 12.0f * u * u - 8.0f * u * u * u;
    const float s_prof = (lp < 0.5f) ? s_swing : (3.0f - 4.0f * lp);

    const float u_sw = std::clamp(u, 0.0f, 1.0f);
    const float u_st = std::clamp(u - 1.0f, 0.0f, 1.0f);
    const float sin_sw = std::sin(static_cast<float>(M_PI) * u_sw);
    const float sin_st = std::sin(static_cast<float>(M_PI) * u_st);
    const float z_swing = sin_sw * sin_sw;
    const float z_stance = sin_st * sin_st;

    const float f_vx = gx * cmd_.x() - gyaw * cmd_.z() * foot_home_xy_[leg].y();
    const float f_vy = gy * cmd_.y() + gyaw * cmd_.z() * foot_home_xy_[leg].x();
    const float hs_x = f_vx / (4.0f * gait_freq);
    const float hs_y = f_vy / (4.0f * gait_freq);

    const Eigen::Vector3f df(s_prof * hs_x, s_prof * hs_y,
                             swing_clear * z_swing - stance_press * z_stance);
    cpg_dq.segment<3>(3 * leg) = jac_inv_[leg] * df;
  }

  for (int i = 0; i < kNumLegs; ++i) {
    const float q_tgt = home_legs_[i] + cpg_dq[i] + act_scale * actions_[i];
    target_legs_[i] = std::clamp(q_tgt, soft_low_[i], soft_high_[i]);
  }

  interp_start_q_ = slew_q_;
  interp_alpha_ = 0.0f;
  was_moving_ = true;
  prev_cmd_ = cmd_;

  state_.enabled = enabled_;
  state_.moving = true;
  state_.using_odom_vel = use_odom;
  state_.phase = phase_;
  state_.user_cmd = smoothed_user_cmd_;
  state_.eff_cmd = cmd_;
  state_.est_vel_xy = v_real_xy;
  state_.imu_wz = ang_vel_b_.z();
  state_.body_yaw_rad = body_yaw_;
  state_.heading_err_rad = h_err;
  return state_;
}

void WalkCtrl::StepHardware(float dt_s,
                            std::vector<double>* leg_cmds,
                            std::vector<double>* arm_cmds) {
  if (!has_imu_quat_) {
    body_yaw_ = WrapPi(body_yaw_ + ang_vel_b_.z() * dt_s);
  }
  UpdateKinematicVel();

  Eigen::Matrix<float, kNumLegs, 1> des_q;
  if (was_moving_) {
    interp_alpha_ = std::min(1.0f, interp_alpha_ + dt_s / 0.02f);
    des_q = interp_start_q_ + interp_alpha_ * (target_legs_ - interp_start_q_);
  } else {
    des_q = slew_q_ + 0.18f * (home_legs_ - slew_q_);
    if ((des_q - home_legs_).cwiseAbs().maxCoeff() < 1e-3f) {
      des_q = home_legs_;
    }
  }

  const float max_step = static_cast<float>(calib_.gait.max_step_rad);
  for (int i = 0; i < kNumLegs; ++i) {
    const float delta = std::clamp(des_q[i] - slew_q_[i], -max_step, max_step);
    slew_q_[i] = std::clamp(slew_q_[i] + delta, hard_low_[i], hard_high_[i]);
  }

  // 하드웨어 관절 피드백이 없는 드라이런·단독 시험 시 루프백 유지
  if (!has_joints_) {
    const Eigen::Matrix<float, kNumLegs, 1> prev_q = q_sim_;
    q_sim_ = slew_q_;
    if (dt_s > 1e-4f) dq_sim_ = (q_sim_ - prev_q) / dt_s;
  }

  if (leg_cmds) {
    leg_cmds->resize(kNumLegs);
    for (int out_i = 0; out_i < kNumLegs; ++out_i) {
      const int idx = out_to_leg_[out_i];
      (*leg_cmds)[out_i] = calib_.legs[idx].ToHw(slew_q_[idx]);
    }
  }
  if (arm_cmds) {
    arm_cmds->resize(kNumArm);
    for (int i = 0; i < kNumArm; ++i) {
      (*arm_cmds)[i] = calib_.arm[i].ToHw(calib_.arm[i].stand_rad);
    }
  }
}

}  // namespace xsting_walk
