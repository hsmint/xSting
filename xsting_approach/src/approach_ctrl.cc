#include "xsting_approach/approach_ctrl.h"

#include <algorithm>
#include <cmath>

#include "xsting_approach/pca_vis.h"

namespace xsting_approach {

// 상위 접근 MLP(28->3)와 하위 WalkNet MLP(26->18) 초기화
bool ApproachCtrl::Init(const std::string& approach_bin,
                        const std::string& walk_bin,
                        const Calib& calib,
                        std::string* err) {
  if (!approach_mlp_.Load(approach_bin, err)) return false;
  if (approach_mlp_.in_dim() != 28 || approach_mlp_.out_dim() != 3) {
    if (err) *err = "Approach MLP 입출력 차원 오류 (28 -> 3 필요)";
    return false;
  }

  if (!walk_mlp_.Load(walk_bin, err)) return false;
  if (walk_mlp_.in_dim() != 26 || walk_mlp_.out_dim() != kNumLegs) {
    if (err) *err = "WalkNet MLP 입출력 차원 오류 (26 -> 18 필요)";
    return false;
  }
  if (!walk_mlp_.extras().empty() &&
      walk_mlp_.extras()[0].size() == kNumLegs) {
    q_def_ = walk_mlp_.extras()[0];
  }

  SetCalib(calib);
  Reset();
  return true;
}

// 보정 구조체를 반영해 기립 관절각·한계·출력 순서 맵 갱신
void ApproachCtrl::SetCalib(const Calib& calib) {
  calib_ = calib;
  for (int i = 0; i < kNumLegs; ++i) {
    home_legs_[i] = static_cast<float>(calib_.legs[i].stand_rad);
    leg_low_[i] = static_cast<float>(calib_.legs[i].lower_rad);
    leg_high_[i] = static_cast<float>(calib_.legs[i].upper_rad);
  }
  if (q_def_.squaredNorm() < 1e-6f) q_def_ = home_legs_;
  RebuildMap();
}

// ROS 관절 이름과 WalkNet 내부 순서 간 인덱스 매핑 생성
void ApproachCtrl::RebuildMap() {
  ros_to_walk_.clear();
  for (int i = 0; i < kNumLegs; ++i) {
    ros_to_walk_[calib_.legs[i].ros_name] = i;
    ros_to_walk_[calib_.legs[i].name] = i;
  }
  for (int out_i = 0; out_i < kNumLegs; ++out_i) {
    int mapped = out_i;
    if (static_cast<size_t>(out_i) < calib_.out_leg_order.size()) {
      const auto it = ros_to_walk_.find(calib_.out_leg_order[out_i]);
      if (it != ros_to_walk_.end()) mapped = it->second;
    }
    out_to_walk_[out_i] = mapped;
  }
}

// 제어기 내부 필터·위상·도달 상태 초기화
void ApproachCtrl::Reset() {
  if (!has_joints_) {
    q_sim_ = home_legs_;
    dq_sim_.setZero();
  }
  stance_q_ = home_legs_;
  slew_q_ = home_legs_;
  action_.setZero();
  cmd_ema_.setZero();
  cmd_.setZero();
  phase_ = 0.0f;
  us_range_m_ = static_cast<float>(calib_.sonar.max_range_m);
  us_valid_ = false;
  seen_box_ = false;
  box_xy_.setZero();
  norm_xy_ = Eigen::Vector2f::UnitX();
  p1_rel_ = Eigen::Vector2f(0.0f, -0.04f);
  p2_rel_ = Eigen::Vector2f(0.0f, 0.04f);
  stopped_ = true;
  reached_ = false;
  hold_sec_ = 0.0f;
  state_ = CtrlState{};
  state_.active = active_;
}

// 정밀 접근 주행 시작 또는 정지 전환
void ApproachCtrl::SetActive(bool active) {
  if (active && !active_) {
    if (has_joints_) {
      stance_q_ = q_sim_;
      slew_q_ = q_sim_;
    }
    reached_ = false;
    hold_sec_ = 0.0f;
    action_.setZero();
    cmd_ema_.setZero();
    cmd_.setZero();
    stopped_ = true;
  }
  active_ = active;
  if (!active_) {
    stopped_ = true;
    action_.setZero();
    cmd_ema_.setZero();
    cmd_.setZero();
  }
  state_.active = active_;
}

// /joint_states 피드백을 시뮬레이션 다리 좌표계로 변환
void ApproachCtrl::UpdateJoints(const std::vector<std::string>& names,
                                const std::vector<double>& pos,
                                const std::vector<double>& vel) {
  const size_t n = std::min(names.size(), pos.size());
  bool updated = false;
  for (size_t i = 0; i < n; ++i) {
    const auto it = ros_to_walk_.find(names[i]);
    if (it == ros_to_walk_.end()) continue;
    const int idx = it->second;
    q_sim_[idx] = static_cast<float>(calib_.legs[idx].ToSim(pos[i]));
    if (i < vel.size()) {
      dq_sim_[idx] = static_cast<float>(calib_.legs[idx].VelToSim(vel[i]));
    }
    updated = true;
  }
  if (updated && !has_joints_) {
    stance_q_ = q_sim_;
    slew_q_ = q_sim_;
    has_joints_ = true;
  }
}

// IMU 원시 각속도·쿼터니언을 차체 기준으로 갱신
void ApproachCtrl::UpdateImu(const Eigen::Vector3d& gyro_raw,
                             const Eigen::Quaterniond& quat_raw) {
  calib_.imu.Transform(gyro_raw, quat_raw, &ang_vel_b_, &proj_grav_b_);
  has_gyro_ = true;
}

// 전방 HC-SR04 초음파 거리 보정 및 유효 범위 판정
void ApproachCtrl::UpdateSonar(double raw_range_m, bool valid) {
  us_raw_m_ = std::isfinite(raw_range_m) ? static_cast<float>(raw_range_m)
                                         : 4.0f;
  if (!valid || !calib_.sonar.enabled || !std::isfinite(raw_range_m)) {
    us_valid_ = false;
    us_range_m_ = static_cast<float>(calib_.sonar.max_range_m);
    return;
  }
  const double corr = raw_range_m + calib_.sonar.range_bias_m;
  if (corr >= calib_.sonar.min_range_m && corr < calib_.sonar.max_range_m) {
    us_valid_ = true;
    us_range_m_ = static_cast<float>(corr);
  } else {
    us_valid_ = false;
    us_range_m_ = static_cast<float>(calib_.sonar.max_range_m);
  }
}

// cuVSLAM 등 외부 차체 선속도·요레이트 피드백 갱신
void ApproachCtrl::UpdateVel(const Eigen::Vector2f& vel_xy, float wz) {
  body_vel_xy_ = vel_xy;
  has_vel_ = true;
  if (!has_gyro_) ang_vel_b_.z() = wz;
}

// 50Hz 상위 접근 정책 추론, 사각지대 융합 및 2cm 정렬 유지 판정
CtrlState ApproachCtrl::StepPolicy(const Eigen::Matrix<float, 9, 1>& vis_obs,
                                   float dt_s) {
  const float standoff = static_cast<float>(calib_.cam.standoff_m);
  Eigen::Matrix<float, 9, 1> eff_vis = vis_obs;

  // 상자 가시 시 최근 관측 갱신, 사각지대 진입 시 추측항법+초음파 융합
  if (vis_obs[8] > 0.5f) {
    norm_xy_ = vis_obs.segment<2>(2);
    norm_xy_ = norm_xy_.norm() > 1e-5f ? norm_xy_.normalized()
                                       : Eigen::Vector2f::UnitX();
    box_xy_ = vis_obs.segment<2>(0) + standoff * norm_xy_;
    p1_rel_ = vis_obs.segment<2>(4);
    p2_rel_ = vis_obs.segment<2>(6);
    seen_box_ = true;
  } else if (seen_box_) {
    bool blind_updated = false;
    const float cached_dist = (box_xy_ - standoff * norm_xy_).norm();
    if (calib_.gait.blind_dr &&
        cached_dist <= static_cast<float>(calib_.gait.blind_max_m)) {
      Eigen::Vector2f v_xy = Eigen::Vector2f::Zero();
      if (has_vel_) {
        v_xy = body_vel_xy_;
      } else if (!stopped_) {
        v_xy = cmd_ema_.head<2>();
      }
      const float wz =
          has_gyro_ ? ang_vel_b_.z() : (stopped_ ? 0.0f : cmd_ema_.z());
      const float c = std::cos(-wz * dt_s);
      const float s = std::sin(-wz * dt_s);
      Eigen::Matrix2f r_inv;
      r_inv << c, -s, s, c;
      box_xy_ = r_inv * (box_xy_ - v_xy * dt_s);
      norm_xy_ = (r_inv * norm_xy_).normalized();
      blind_updated = true;
    }

    if (calib_.sonar.enabled && calib_.sonar.close_fusion && us_valid_ &&
        us_range_m_ < 0.35f && norm_xy_.x() > 0.85f &&
        std::abs(box_xy_.y()) <= 0.09f) {
      const float wall_x = static_cast<float>(calib_.sonar.us_pos_m.x()) +
                           us_range_m_ * norm_xy_.x();
      box_xy_.x() =
          wall_x + static_cast<float>(calib_.cam.box_half_m) * norm_xy_.x();
      blind_updated = true;
    }

    if (blind_updated) {
      eff_vis = PackVis(box_xy_, norm_xy_, box_xy_ + p1_rel_, box_xy_ + p2_rel_,
                        1.0f, standoff);
    }
  }

  // 28차원 정책 관측 벡터 조립
  const Eigen::Vector2f cam_xy = calib_.cam.cam_pos_m.head<2>().cast<float>();
  const Eigen::Vector4f fine_vis = MakeFineVis(eff_vis, standoff, cam_xy);
  const float bumper_off =
      static_cast<float>(calib_.sonar.BumperOff(calib_.cam.robot_front_x_m));
  const Eigen::Vector3f sonar_obs = MakeSonarObs(
      us_range_m_, us_valid_, bumper_off,
      static_cast<float>(calib_.cam.target_gap_m),
      static_cast<float>(calib_.sonar.min_range_m));
  const Eigen::Vector3f speed_lim = calib_.gait.speed_lim.cast<float>();
  const Eigen::Vector3f speed_scale = calib_.gait.speed_scale.cast<float>();

  Eigen::Vector3f cmd_norm = cmd_ema_.cwiseQuotient(speed_lim);
  for (int i = 0; i < 3; ++i) cmd_norm[i] = std::clamp(cmd_norm[i], -1.0f, 1.0f);

  Eigen::Matrix<float, 28, 1> obs28;
  obs28.segment<3>(0) = ang_vel_b_ * 0.25f;
  obs28.segment<3>(3) = proj_grav_b_;
  obs28.segment<9>(6) = eff_vis;
  obs28.segment<4>(15) = fine_vis;
  obs28.segment<3>(19) = action_;
  obs28.segment<3>(22) = cmd_norm;
  obs28.segment<3>(25) = sonar_obs;

  // 정책 추론, 정지 슈미트 트리거 및 초음파 안전 제동
  if (!active_ || reached_) {
    action_.setZero();
    cmd_ema_.setZero();
    cmd_.setZero();
    stopped_ = true;
  } else {
    const Eigen::VectorXf raw_act = approach_mlp_.Forward(obs28);
    for (int i = 0; i < 3; ++i) {
      action_[i] = std::clamp(std::isfinite(raw_act[i]) ? raw_act[i] : 0.0f,
                              -1.0f, 1.0f);
    }

    const float act_mag = action_.cwiseAbs().maxCoeff();
    const bool enter_stop =
        act_mag < static_cast<float>(calib_.gait.stop_enter);
    const bool stay_stop =
        stopped_ && (act_mag < static_cast<float>(calib_.gait.stop_exit));
    stopped_ = enter_stop || stay_stop;

    Eigen::Vector3f gated;
    const float db = static_cast<float>(calib_.gait.deadband);
    for (int i = 0; i < 3; ++i) {
      gated[i] = (std::abs(action_[i]) < db) ? 0.0f : action_[i];
    }

    cmd_ema_ = 0.5f * cmd_ema_ + 0.5f * gated.cwiseProduct(speed_lim);
    if (stopped_ || gated.cwiseAbs().maxCoeff() == 0.0f) {
      stopped_ = true;
      cmd_.setZero();
      cmd_ema_.setZero();
    } else {
      cmd_ = cmd_ema_.cwiseProduct(speed_scale);
      if (us_valid_ && calib_.sonar.enabled && calib_.sonar.brake_gap_m > 0.0) {
        const float min_us =
            bumper_off + static_cast<float>(calib_.sonar.brake_gap_m);
        if (us_range_m_ <= min_us && cmd_.x() > 0.0f) {
          cmd_.x() = 0.0f;
          cmd_ema_.x() = 0.0f;
          if (cmd_.cwiseAbs().maxCoeff() < 1e-4f) {
            stopped_ = true;
            cmd_.setZero();
            cmd_ema_.setZero();
          }
        }
      }
    }
  }

  // 2cm 위치·각도 허용 오차 및 다리 정지 2초 유지(hold) 판정
  const Eigen::Vector2f err_xy = eff_vis.segment<2>(0);
  const Eigen::Vector2f n_xy = eff_vis.segment<2>(2);
  const float dist_m = err_xy.norm();
  const float yaw_err = std::atan2(n_xy.y(), n_xy.x());
  const float max_dq = dq_sim_.cwiseAbs().maxCoeff();
  const bool aligned = (eff_vis[8] > 0.5f) &&
                       (dist_m < static_cast<float>(calib_.cam.pos_tol_m)) &&
                       (std::abs(yaw_err) <
                        static_cast<float>(calib_.cam.yaw_tol_rad));
  const bool still =
      stopped_ && (max_dq < 0.15f) && (std::abs(ang_vel_b_.z()) < 0.15f);

  if (active_ && !reached_) {
    hold_sec_ = aligned ? (still ? hold_sec_ + dt_s
                                 : std::max(0.0f, hold_sec_ - 0.5f * dt_s))
                        : 0.0f;
    if (hold_sec_ >= static_cast<float>(calib_.cam.hold_time_s)) {
      reached_ = true;
      stopped_ = true;
      action_.setZero();
      cmd_ema_.setZero();
      cmd_.setZero();
    }
  }

  // 상태 요약 갱신
  state_.active = active_;
  state_.stopped = stopped_;
  state_.aligned = aligned;
  state_.reached = reached_;
  state_.visible = vis_obs[8];
  state_.dist_m = dist_m;
  state_.yaw_rad = yaw_err;
  state_.hold_sec = hold_sec_;
  state_.range_m = us_range_m_;
  state_.range_valid = us_valid_;
  state_.action = action_;
  state_.cmd_vel = cmd_;
  return state_;
}

// 200Hz WalkNet 추론 및 슬루율 제한 후 하드웨어 다리·팔 목표각 출력
void ApproachCtrl::StepWalk(float dt_s,
                            std::vector<double>* leg_cmds,
                            std::vector<double>* arm_cmds) {
  if (!stopped_) {
    phase_ = std::fmod(
        phase_ + static_cast<float>(calib_.gait.gait_freq_hz) * dt_s, 1.0f);
    const float ang = static_cast<float>(2.0 * M_PI) * phase_;

    Eigen::Matrix<float, 26, 1> w_obs;
    w_obs.segment<3>(0) = cmd_;
    w_obs.segment<kNumLegs>(3) = q_sim_ - q_def_;
    w_obs.segment<3>(21) = proj_grav_b_;
    w_obs[24] = std::sin(ang);
    w_obs[25] = std::cos(ang);

    const Eigen::VectorXf w_act = walk_mlp_.Forward(w_obs);
    const float scale = static_cast<float>(calib_.gait.walk_scale);
    for (int i = 0; i < kNumLegs; ++i) {
      stance_q_[i] =
          std::clamp(q_def_[i] + scale * w_act[i], leg_low_[i], leg_high_[i]);
    }
  } else {
    phase_ = 0.0f;
    stance_q_ += 0.20f * (home_legs_ - stance_q_);
    if ((stance_q_ - home_legs_).cwiseAbs().maxCoeff() < 1e-3f) {
      stance_q_ = home_legs_;
    }
  }

  // 급격한 관절 점프 방지를 위한 스텝당 변화량 제한
  const float max_step = static_cast<float>(calib_.gait.max_step_rad);
  for (int i = 0; i < kNumLegs; ++i) {
    const float delta =
        std::clamp(stance_q_[i] - slew_q_[i], -max_step, max_step);
    slew_q_[i] = std::clamp(slew_q_[i] + delta, leg_low_[i], leg_high_[i]);
  }
  if (!has_joints_) {
    q_sim_ = slew_q_;
    dq_sim_.setZero();
  }

  // 하드웨어 컨트롤러 순서에 맞춰 다리·팔 목표각 변환
  if (leg_cmds) {
    leg_cmds->resize(kNumLegs);
    for (int out_i = 0; out_i < kNumLegs; ++out_i) {
      const int idx = out_to_walk_[out_i];
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

}  // namespace xsting_approach
