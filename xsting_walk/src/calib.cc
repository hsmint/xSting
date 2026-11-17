#include "xsting_walk/calib.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace xsting_walk {

namespace {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kScaleMin = 0.40;
constexpr double kScaleMax = 2.20;

}  // namespace

const std::array<std::string, kNumLegs>& GetLegNames() {
  static const std::array<std::string, kNumLegs> kNames = {
      "leg1_yaw", "leg1_hip", "leg1_knee", "leg2_yaw", "leg2_hip", "leg2_knee",
      "leg3_yaw", "leg3_hip", "leg3_knee", "leg4_yaw", "leg4_hip", "leg4_knee",
      "leg5_yaw", "leg5_hip", "leg5_knee", "leg6_yaw", "leg6_hip", "leg6_knee",
  };
  return kNames;
}

const std::array<std::string, kNumArm>& GetArmNames() {
  static const std::array<std::string, kNumArm> kNames = {
      "arm_yaw",   "arm_shoulder", "arm_elbow",
      "arm_wrist", "arm_roll",     "finger_l",
  };
  return kNames;
}

double GetBootRad(const std::string& joint_name) {
  if (joint_name == "leg1_yaw" || joint_name == "leg4_yaw") return M_PI / 6.0;
  if (joint_name == "leg3_yaw" || joint_name == "leg6_yaw") return -M_PI / 6.0;
  if (joint_name.find("_hip") != std::string::npos) return 15.0 * kDegToRad;
  if (joint_name.find("_knee") != std::string::npos) return -M_PI / 2.0;
  if (joint_name == "arm_shoulder") return -1.0508;
  if (joint_name == "arm_elbow") return 1.0707;
  return 0.0;
}

void ImuCalib::Transform(const Eigen::Vector3d& gyro_raw,
                         const Eigen::Vector3d& acc_raw,
                         bool has_acc,
                         const Eigen::Quaterniond& quat_raw,
                         Eigen::Vector3f* ang_vel_b,
                         Eigen::Vector3f* lin_acc_b,
                         Eigen::Vector3f* proj_grav_b,
                         float* body_yaw_rad,
                         bool* has_quat_yaw) const {
  const Eigen::Quaterniond q_m = mount_quat.normalized();
  const Eigen::Vector3d w_b = q_m * (gyro_raw - gyro_bias);
  if (ang_vel_b) *ang_vel_b = w_b.cast<float>();

  Eigen::Vector3d grav_b(0.0, 0.0, -1.0);
  const bool valid_q = quat_raw.norm() > 1e-4;
  if (valid_q) {
    const Eigen::Quaterniond q_s = quat_raw.normalized();
    const Eigen::Vector3d grav_s =
        q_s.conjugate() * Eigen::Vector3d(0.0, 0.0, -1.0);
    grav_b = (q_m * grav_s).normalized();

    // q_wb = q_ws * q_bs^{-1}
    const Eigen::Quaterniond q_wb = (q_s * q_m.conjugate()).normalized();
    if (body_yaw_rad) {
      *body_yaw_rad = static_cast<float>(std::atan2(
          2.0 * (q_wb.w() * q_wb.z() + q_wb.x() * q_wb.y()),
          1.0 - 2.0 * (q_wb.y() * q_wb.y() + q_wb.z() * q_wb.z())));
    }
    if (has_quat_yaw) *has_quat_yaw = true;
  } else {
    if (has_acc && acc_raw.norm() > 1.0) {
      grav_b = (-(q_m * (acc_raw - acc_bias))).normalized();
    }
    if (has_quat_yaw) *has_quat_yaw = false;
  }

  if (proj_grav_b) *proj_grav_b = grav_b.cast<float>();

  if (lin_acc_b) {
    if (has_acc && acc_raw.squaredNorm() > 1e-4) {
      *lin_acc_b = (q_m * (acc_raw - acc_bias)).cast<float>();
    } else {
      // 가속도계가 비어 있는 모의 환경에서는 정지 시 중력 반작용(0, 0, +9.81) 유지
      *lin_acc_b = static_cast<float>(-kGravityMps2) * grav_b.cast<float>();
    }
  }
}

Calib Calib::MakeDefault() {
  Calib c;
  const auto& lnames = GetLegNames();
  // xsting_walk 학습 초기 자세: leg[1,4]_yaw=+30deg, leg[2,5]_yaw=0, leg[3,6]_yaw=-30deg,
  // hip=+15deg, knee=-90deg
  const std::array<double, 6> yaw_deg = {30.0, 0.0, -30.0, 30.0, 0.0, -30.0};
  const std::array<double, 6> yaw_low = {-0.436332, -0.436332, -1.5708,
                                         -0.436332, -0.750492, -1.5708};
  const std::array<double, 6> yaw_high = {1.5708, 0.750492, 0.436332,
                                          1.5708, 0.436332, 0.436332};

  for (int leg = 0; leg < 6; ++leg) {
    const int idx = leg * 3;
    c.legs[idx + 0] = {lnames[idx + 0], lnames[idx + 0], 1.0, 0.0,
                       yaw_low[leg], yaw_high[leg], yaw_deg[leg] * kDegToRad};
    c.legs[idx + 1] = {lnames[idx + 1], lnames[idx + 1], 1.0, 0.0,
                       -1.65806, 1.65806, 15.0 * kDegToRad};
    c.legs[idx + 2] = {lnames[idx + 2], lnames[idx + 2], 1.0, 0.0,
                       -2.00713, 1.74533, -90.0 * kDegToRad};
  }

  const auto& anames = GetArmNames();
  const std::array<double, kNumArm> arm_stand = {0.0, -1.2962634, 1.2962634,
                                                 0.0, 0.0,        0.0};
  const std::array<double, kNumArm> arm_low = {-3.14159, -1.658063, -2.007129,
                                               -1.8326,  -3.14159,  -0.00186};
  const std::array<double, kNumArm> arm_high = {3.14159, 1.919862, 1.570796,
                                                2.18166, 3.14159,  0.025};
  for (int i = 0; i < kNumArm; ++i) {
    c.arm[i] = {anames[i], anames[i], 1.0,        0.0,
                arm_low[i], arm_high[i], arm_stand[i]};
  }

  c.out_leg_order.assign(lnames.begin(), lnames.end());
  return c;
}

bool Calib::Load(const std::string& path, std::string* err) {
  try {
    const YAML::Node root = YAML::LoadFile(path);

    if (root["output_leg_order"] && root["output_leg_order"].IsSequence()) {
      std::vector<std::string> order;
      for (const auto& item : root["output_leg_order"]) {
        order.push_back(item.as<std::string>());
      }
      if (order.size() == kNumLegs) out_leg_order = std::move(order);
    }

    auto load_joints = [](const YAML::Node& node, auto& arr) {
      if (!node || !node.IsMap()) return;
      for (auto& j : arr) {
        const YAML::Node jn = node[j.name];
        if (!jn) continue;
        if (jn["ros_name"]) j.ros_name = jn["ros_name"].as<std::string>();
        if (jn["direction"]) j.dir = jn["direction"].as<double>();
        if (jn["offset_rad"]) j.offset_rad = jn["offset_rad"].as<double>();
        if (jn["lower_rad"]) j.lower_rad = jn["lower_rad"].as<double>();
        if (jn["upper_rad"]) j.upper_rad = jn["upper_rad"].as<double>();
        if (jn["stand_rad"]) j.stand_rad = jn["stand_rad"].as<double>();
      }
    };
    load_joints(root["leg_joints"], legs);
    load_joints(root["arm_joints"], arm);

    if (const YAML::Node in = root["imu"]) {
      if (in["mount_quat_xyzw"] && in["mount_quat_xyzw"].size() == 4) {
        imu.mount_quat = Eigen::Quaterniond(
                             in["mount_quat_xyzw"][3].as<double>(),
                             in["mount_quat_xyzw"][0].as<double>(),
                             in["mount_quat_xyzw"][1].as<double>(),
                             in["mount_quat_xyzw"][2].as<double>())
                             .normalized();
      }
      if (in["gyro_bias_rad_s"] && in["gyro_bias_rad_s"].size() == 3) {
        imu.gyro_bias = Eigen::Vector3d(in["gyro_bias_rad_s"][0].as<double>(),
                                        in["gyro_bias_rad_s"][1].as<double>(),
                                        in["gyro_bias_rad_s"][2].as<double>());
      }
      if (in["acc_bias_m_s2"] && in["acc_bias_m_s2"].size() == 3) {
        imu.acc_bias = Eigen::Vector3d(in["acc_bias_m_s2"][0].as<double>(),
                                       in["acc_bias_m_s2"][1].as<double>(),
                                       in["acc_bias_m_s2"][2].as<double>());
      }
    }

    if (const YAML::Node wn = root["walk_gait"]) {
      if (wn["action_scale"]) gait.action_scale = wn["action_scale"].as<double>();
      if (wn["action_ema"]) gait.action_ema = wn["action_ema"].as<double>();
      if (wn["gait_freq_hz"]) gait.gait_freq_hz = wn["gait_freq_hz"].as<double>();
      if (wn["stride_gain_xy"] && wn["stride_gain_xy"].size() == 2) {
        gait.stride_gain_xy = Eigen::Vector2d(
            wn["stride_gain_xy"][0].as<double>(),
            wn["stride_gain_xy"][1].as<double>());
      }
      if (wn["stride_gain_yaw"]) {
        gait.stride_gain_yaw = wn["stride_gain_yaw"].as<double>();
      }
      if (wn["swing_clearance_m"]) {
        gait.swing_clearance_m = wn["swing_clearance_m"].as<double>();
      }
      if (wn["stance_press_m"]) {
        gait.stance_press_m = wn["stance_press_m"].as<double>();
      }
      if (wn["soft_limit_factor"]) {
        gait.soft_limit_factor = wn["soft_limit_factor"].as<double>();
      }
      if (wn["max_speed"] && wn["max_speed"].size() == 3) {
        gait.max_speed = Eigen::Vector3d(wn["max_speed"][0].as<double>(),
                                         wn["max_speed"][1].as<double>(),
                                         wn["max_speed"][2].as<double>());
      }
      if (wn["cmd_vel_scale"] && wn["cmd_vel_scale"].size() == 3) {
        gait.cmd_vel_scale = Eigen::Vector3d(
            wn["cmd_vel_scale"][0].as<double>(),
            wn["cmd_vel_scale"][1].as<double>(),
            wn["cmd_vel_scale"][2].as<double>());
      }
      if (wn["cmd_vel_trim"] && wn["cmd_vel_trim"].size() == 3) {
        gait.cmd_vel_trim = Eigen::Vector3d(
            wn["cmd_vel_trim"][0].as<double>(),
            wn["cmd_vel_trim"][1].as<double>(),
            wn["cmd_vel_trim"][2].as<double>());
      }
      if (wn["drift_vy_per_vx"]) {
        gait.drift_vy_per_vx = wn["drift_vy_per_vx"].as<double>();
      }
      if (wn["drift_wz_per_vx"]) {
        gait.drift_wz_per_vx = wn["drift_wz_per_vx"].as<double>();
      }
      if (wn["cmd_accel_lim"] && wn["cmd_accel_lim"].size() == 3) {
        gait.cmd_accel_lim = Eigen::Vector3d(
            wn["cmd_accel_lim"][0].as<double>(),
            wn["cmd_accel_lim"][1].as<double>(),
            wn["cmd_accel_lim"][2].as<double>());
      }
      if (wn["max_leg_step_rad"]) {
        gait.max_step_rad = wn["max_leg_step_rad"].as<double>();
      }
      if (wn["use_odom_vel"]) gait.use_odom_vel = wn["use_odom_vel"].as<bool>();
      if (wn["dry_run"]) gait.dry_run = wn["dry_run"].as<bool>();
    }
    return true;
  } catch (const std::exception& e) {
    if (err) *err = std::string("YAML 로드 실패: ") + e.what();
    return false;
  }
}

bool Calib::Save(const std::string& path, std::string* err) const {
  std::ofstream ofs(path);
  if (!ofs.is_open()) {
    if (err) *err = "YAML 저장 실패: " + path;
    return false;
  }

  ofs << std::fixed << std::setprecision(6);
  ofs << "# xSting 전방향 보행(xsting_walk) 실측 보정 설정 (q_hw = direction * q_sim + offset_rad)\n";
  ofs << "output_leg_order: [";
  for (size_t i = 0; i < out_leg_order.size(); ++i) {
    ofs << out_leg_order[i] << (i + 1 < out_leg_order.size() ? ", " : "");
  }
  ofs << "]\n\n";

  auto write_joints = [&](const char* title, const auto& arr) {
    ofs << title << ":\n";
    for (const auto& j : arr) {
      ofs << "  " << j.name << ": {ros_name: " << j.ros_name
          << ", direction: " << std::setprecision(1) << j.dir
          << std::setprecision(6) << ", offset_rad: " << j.offset_rad
          << ", lower_rad: " << j.lower_rad << ", upper_rad: " << j.upper_rad
          << ", stand_rad: " << j.stand_rad << "}\n";
    }
  };
  write_joints("leg_joints", legs);
  ofs << "\n";
  write_joints("arm_joints", arm);

  const Eigen::Quaterniond q = imu.mount_quat.normalized();
  ofs << "\nimu:\n"
      << "  mount_quat_xyzw: [" << q.x() << ", " << q.y() << ", " << q.z()
      << ", " << q.w() << "]\n"
      << "  gyro_bias_rad_s: [" << imu.gyro_bias.x() << ", "
      << imu.gyro_bias.y() << ", " << imu.gyro_bias.z() << "]\n"
      << "  acc_bias_m_s2: [" << imu.acc_bias.x() << ", " << imu.acc_bias.y()
      << ", " << imu.acc_bias.z() << "]\n";

  ofs << "\nwalk_gait:\n"
      << "  action_scale: " << gait.action_scale << "\n"
      << "  action_ema: " << gait.action_ema << "\n"
      << "  gait_freq_hz: " << gait.gait_freq_hz << "\n"
      << "  stride_gain_xy: [" << gait.stride_gain_xy.x() << ", "
      << gait.stride_gain_xy.y() << "]\n"
      << "  stride_gain_yaw: " << gait.stride_gain_yaw << "\n"
      << "  swing_clearance_m: " << gait.swing_clearance_m << "\n"
      << "  stance_press_m: " << gait.stance_press_m << "\n"
      << "  soft_limit_factor: " << gait.soft_limit_factor << "\n"
      << "  max_speed: [" << gait.max_speed.x() << ", " << gait.max_speed.y()
      << ", " << gait.max_speed.z() << "]\n"
      << "  cmd_vel_scale: [" << gait.cmd_vel_scale.x() << ", "
      << gait.cmd_vel_scale.y() << ", " << gait.cmd_vel_scale.z() << "]\n"
      << "  cmd_vel_trim: [" << gait.cmd_vel_trim.x() << ", "
      << gait.cmd_vel_trim.y() << ", " << gait.cmd_vel_trim.z() << "]\n"
      << "  drift_vy_per_vx: " << gait.drift_vy_per_vx << "\n"
      << "  drift_wz_per_vx: " << gait.drift_wz_per_vx << "\n"
      << "  cmd_accel_lim: [" << gait.cmd_accel_lim.x() << ", "
      << gait.cmd_accel_lim.y() << ", " << gait.cmd_accel_lim.z() << "]\n"
      << "  max_leg_step_rad: " << gait.max_step_rad << "\n"
      << "  use_odom_vel: " << (gait.use_odom_vel ? "true" : "false") << "\n"
      << "  dry_run: " << (gait.dry_run ? "true" : "false") << "\n";

  return static_cast<bool>(ofs);
}

ImuCalib SolveImu1Pose(const Eigen::Vector3d& gyro_mean,
                       const Eigen::Vector3d& acc_mean,
                       const Eigen::Vector3d& grav_imu,
                       double yaw_deg) {
  ImuCalib out;
  out.gyro_bias = gyro_mean;
  const Eigen::Quaterniond q_yaw(
      Eigen::AngleAxisd(yaw_deg * kDegToRad, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d g_rot = (q_yaw * grav_imu).normalized();
  const Eigen::Quaterniond q_rp = Eigen::Quaterniond::FromTwoVectors(
      g_rot, Eigen::Vector3d(0.0, 0.0, -1.0));
  out.mount_quat = (q_rp * q_yaw).normalized();

  // 수평 정지 시 차체 선가속도(특정력)가 정확히 (0, 0, +9.81)이 되도록 바이어스 산출
  if (acc_mean.norm() > 1.0) {
    out.acc_bias =
        acc_mean - out.mount_quat.conjugate() * Eigen::Vector3d(0.0, 0.0, kGravityMps2);
  }
  return out;
}

ImuCalib SolveImu2Pose(const Eigen::Vector3d& gyro_mean,
                       const Eigen::Vector3d& acc_mean,
                       const Eigen::Vector3d& grav_level,
                       const Eigen::Vector3d& grav_tilt) {
  const Eigen::Vector3d z_b = (-grav_level).normalized();
  const Eigen::Vector3d dg = grav_tilt.normalized() - grav_level.normalized();
  Eigen::Vector3d x_b = dg - dg.dot(z_b) * z_b;
  if (x_b.norm() < 1e-4) {
    return SolveImu1Pose(gyro_mean, acc_mean, grav_level, 0.0);
  }
  x_b.normalize();

  ImuCalib out;
  out.gyro_bias = gyro_mean;
  Eigen::Matrix3d r_mount;
  r_mount.row(0) = x_b.transpose();
  r_mount.row(1) = z_b.cross(x_b).normalized().transpose();
  r_mount.row(2) = z_b.transpose();
  out.mount_quat = Eigen::Quaterniond(r_mount).normalized();
  if (acc_mean.norm() > 1.0) {
    out.acc_bias =
        acc_mean - out.mount_quat.conjugate() * Eigen::Vector3d(0.0, 0.0, kGravityMps2);
  }
  return out;
}

bool UpdateLinearCalib(const std::string& axis,
                       double cmd_speed_mps,
                       double duration_s,
                       double measured_dist_m,
                       double measured_lat_drift_m,
                       double measured_yaw_drift_deg,
                       bool has_lat_drift,
                       bool has_yaw_drift,
                       WalkGaitCalib* gait,
                       std::string* summary) {
  if (!gait || duration_s <= 0.1 || std::abs(cmd_speed_mps) < 1e-3 ||
      std::abs(measured_dist_m) < 0.02) {
    return false;
  }

  const double expected_m = cmd_speed_mps * duration_s;
  // 방향 부호가 반대면 경고성 실패 반환 (배선/관절 방향 오류 가능성)
  if (expected_m * measured_dist_m <= 0.0) return false;

  const double ratio = std::abs(expected_m) / std::abs(measured_dist_m);
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4);

  if (axis == "vx" || axis == "x") {
    const double prev_s = gait->cmd_vel_scale.x();
    const double next_s = std::clamp(prev_s * ratio, kScaleMin, kScaleMax);
    gait->cmd_vel_scale.x() = next_s;
    oss << "vx 스케일: " << prev_s << " -> " << next_s
        << " (목표 " << expected_m << "m / 실측 " << measured_dist_m << "m)";

    if (has_lat_drift && std::isfinite(measured_lat_drift_m)) {
      // +vx 전진 중 +Y(좌측)로 쏠렸다면 음(-)의 vy 보정으로 상쇄
      const double delta_k = -measured_lat_drift_m / expected_m;
      gait->drift_vy_per_vx =
          std::clamp(gait->drift_vy_per_vx + delta_k, -0.35, 0.35);
      oss << ", drift_vy_per_vx=" << gait->drift_vy_per_vx;
    }
    if (has_yaw_drift && std::isfinite(measured_yaw_drift_deg)) {
      const double yaw_rad = measured_yaw_drift_deg * kDegToRad;
      const double delta_kw = -yaw_rad / expected_m;
      gait->drift_wz_per_vx =
          std::clamp(gait->drift_wz_per_vx + delta_kw, -0.80, 0.80);
      oss << ", drift_wz_per_vx=" << gait->drift_wz_per_vx;
    }
  } else if (axis == "vy" || axis == "y") {
    const double prev_s = gait->cmd_vel_scale.y();
    const double next_s = std::clamp(prev_s * ratio, kScaleMin, kScaleMax);
    gait->cmd_vel_scale.y() = next_s;
    oss << "vy 스케일: " << prev_s << " -> " << next_s
        << " (목표 " << expected_m << "m / 실측 " << measured_dist_m << "m)";
  } else {
    return false;
  }

  if (summary) *summary = oss.str();
  return true;
}

bool UpdateTurnCalib(double cmd_wz_rps,
                     double duration_s,
                     double measured_yaw_deg,
                     WalkGaitCalib* gait,
                     std::string* summary) {
  if (!gait || duration_s <= 0.1 || std::abs(cmd_wz_rps) < 1e-3 ||
      std::abs(measured_yaw_deg) < 3.0) {
    return false;
  }

  const double expected_deg = (cmd_wz_rps * duration_s) * (180.0 / M_PI);
  if (expected_deg * measured_yaw_deg <= 0.0) return false;

  const double ratio = std::abs(expected_deg) / std::abs(measured_yaw_deg);
  const double prev_s = gait->cmd_vel_scale.z();
  const double next_s = std::clamp(prev_s * ratio, kScaleMin, kScaleMax);
  gait->cmd_vel_scale.z() = next_s;

  if (summary) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "wz 스케일: " << prev_s << " -> " << next_s << " (목표 "
        << std::setprecision(1) << expected_deg << "deg / 실측 "
        << measured_yaw_deg << "deg)";
    *summary = oss.str();
  }
  return true;
}

}  // namespace xsting_walk
