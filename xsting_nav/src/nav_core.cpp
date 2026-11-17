#include "xsting_nav/nav_core.hpp"
#include "xsting_nav/json_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace xsting_nav
{

namespace
{
constexpr float kGridX[4] = {-0.30f, 0.00f, 0.35f, 0.70f};
constexpr float kGridY[4] = {-0.45f, -0.15f, 0.15f, 0.45f};

double mk2_opencr_tick_to_rad(int32_t tick)
{
  constexpr int32_t zero_tick = 2047;
  const double span = tick >= zero_tick ? (4095 - zero_tick) : (zero_tick - 0);
  return (static_cast<double>(tick) - zero_tick) * Mk2Kinematics::kPi / span;
}

Vec4f quat_from_rpy(float roll, float pitch, float yaw)
{
  const float cr = std::cos(roll * 0.5f);
  const float sr = std::sin(roll * 0.5f);
  const float cp = std::cos(pitch * 0.5f);
  const float sp = std::sin(pitch * 0.5f);
  const float cy = std::cos(yaw * 0.5f);
  const float sy = std::sin(yaw * 0.5f);
  return {
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy
  };
}

Vec4f normalize_quat(const Vec4f & q)
{
  const float n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n < 1e-6f) {
    return {1.0f, 0.0f, 0.0f, 0.0f};
  }
  return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}
}  // namespace

Leg18f Mk2Kinematics::stand_pose()
{
  Leg18f out{};
  for (size_t leg = 0; leg < 6; ++leg) {
    out[3 * leg + 0] = static_cast<float>(kYaw0Deg[leg] * kPi / 180.0);
    out[3 * leg + 1] = kHip0;
    out[3 * leg + 2] = kKnee0;
  }
  return out;
}

Arm6f Mk2Kinematics::arm_home_pose()
{
  return {0.0f, kArmHomeShoulder, kArmHomeElbow, 0.0f, 0.0f, 0.0f};
}

std::array<std::string, 18> Mk2Kinematics::sim_leg_joint_names()
{
  return {
    "leg1_yaw", "leg1_hip", "leg1_knee",
    "leg2_yaw", "leg2_hip", "leg2_knee",
    "leg3_yaw", "leg3_hip", "leg3_knee",
    "leg4_yaw", "leg4_hip", "leg4_knee",
    "leg5_yaw", "leg5_hip", "leg5_knee",
    "leg6_yaw", "leg6_hip", "leg6_knee"
  };
}

std::array<std::string, 18> Mk2Kinematics::mk2_hw_joint_names()
{
  return {
    "joint1", "joint2", "joint3",
    "joint4", "joint5", "joint6",
    "joint7", "joint8", "joint9",
    "joint10", "joint11", "joint12",
    "joint13", "joint14", "joint15",
    "joint16", "joint17", "joint18"
  };
}

std::array<std::string, 6> Mk2Kinematics::arm_joint_names()
{
  return {"arm_yaw", "arm_shoulder", "arm_elbow", "arm_wrist", "arm_roll", "finger_l"};
}

CalibrationConfig CalibrationConfig::make_defaults(const std::string & profile)
{
  CalibrationConfig cfg;
  cfg.hardware_profile = profile;
  cfg.leg_sign.fill(1.0f);
  cfg.leg_offset_rad.fill(0.0f);

  if (profile == "mk2_hexapod") {
    cfg.leg_hw_min_rad.fill(static_cast<float>(-Mk2Kinematics::kPi));
    cfg.leg_hw_max_rad.fill(static_cast<float>(Mk2Kinematics::kPi));
    // OpenCR v4 continuous yaw coordinates for middle legs (motor_id 4 & 13).
    cfg.leg_offset_rad[3] = static_cast<float>(mk2_opencr_tick_to_rad(0));     // -pi
    cfg.leg_hw_min_rad[3] = static_cast<float>(mk2_opencr_tick_to_rad(-1024));
    cfg.leg_hw_max_rad[3] = static_cast<float>(mk2_opencr_tick_to_rad(1024));

    cfg.leg_offset_rad[12] = static_cast<float>(mk2_opencr_tick_to_rad(4095));  // +pi
    cfg.leg_hw_min_rad[12] = static_cast<float>(mk2_opencr_tick_to_rad(3071));
    cfg.leg_hw_max_rad[12] = static_cast<float>(mk2_opencr_tick_to_rad(5119));
    cfg.yolo_cloud_frame = "base_link";
  } else {
    // "xsting" / "leg_arm_split": xsting_hardware uses URDF joint limits directly.
    cfg.leg_hw_min_rad = Mk2Kinematics::kLegLow;
    cfg.leg_hw_max_rad = Mk2Kinematics::kLegHigh;
    cfg.yolo_cloud_frame = "base";
  }

  cfg.arm_sign.fill(1.0f);
  cfg.arm_offset_rad.fill(0.0f);
  return cfg;
}

CalibrationConfig CalibrationConfig::load_yaml(const std::string & yaml_path)
{
  YAML::Node root = YAML::LoadFile(yaml_path);
  std::string profile = "xsting";
  if (root["hardware_profile"]) {
    profile = root["hardware_profile"].as<std::string>();
  }
  CalibrationConfig cfg = make_defaults(profile);
  if (root["last_calibrated_utc"]) {
    cfg.last_calibrated_utc = root["last_calibrated_utc"].as<std::string>();
  }

  if (const auto imu = root["imu"]) {
    if (imu["board_yaw_deg"]) cfg.imu_board_yaw_deg = imu["board_yaw_deg"].as<float>();
    if (imu["gyro_bias_xyz"] && imu["gyro_bias_xyz"].size() == 3) {
      for (size_t i = 0; i < 3; ++i) cfg.imu_gyro_bias_xyz[i] = imu["gyro_bias_xyz"][i].as<float>();
    }
    if (imu["level_quat_wxyz"] && imu["level_quat_wxyz"].size() == 4) {
      for (size_t i = 0; i < 4; ++i) cfg.imu_level_quat_wxyz[i] = imu["level_quat_wxyz"][i].as<float>();
      cfg.imu_level_quat_wxyz = normalize_quat(cfg.imu_level_quat_wxyz);
    }
  }

  if (const auto jn = root["joints"]) {
    if (jn["leg_sign"] && jn["leg_sign"].size() == 18) {
      for (size_t i = 0; i < 18; ++i) cfg.leg_sign[i] = jn["leg_sign"][i].as<float>();
    }
    if (jn["leg_offset_rad"] && jn["leg_offset_rad"].size() == 18) {
      for (size_t i = 0; i < 18; ++i) cfg.leg_offset_rad[i] = jn["leg_offset_rad"][i].as<float>();
    }
    if (jn["leg_hw_min_rad"] && jn["leg_hw_min_rad"].size() == 18) {
      for (size_t i = 0; i < 18; ++i) cfg.leg_hw_min_rad[i] = jn["leg_hw_min_rad"][i].as<float>();
    }
    if (jn["leg_hw_max_rad"] && jn["leg_hw_max_rad"].size() == 18) {
      for (size_t i = 0; i < 18; ++i) cfg.leg_hw_max_rad[i] = jn["leg_hw_max_rad"][i].as<float>();
    }
    if (jn["arm_sign"] && jn["arm_sign"].size() == 6) {
      for (size_t i = 0; i < 6; ++i) cfg.arm_sign[i] = jn["arm_sign"][i].as<float>();
    }
    if (jn["arm_offset_rad"] && jn["arm_offset_rad"].size() == 6) {
      for (size_t i = 0; i < 6; ++i) cfg.arm_offset_rad[i] = jn["arm_offset_rad"][i].as<float>();
    }
    if (jn["max_joint_slew_rad_s"]) cfg.max_joint_slew_rad_s = jn["max_joint_slew_rad_s"].as<float>();
    if (jn["soft_start_duration_s"]) cfg.soft_start_duration_s = jn["soft_start_duration_s"].as<float>();
  }

  if (const auto og = root["odom_gait"]) {
    if (og["init_pose_arena"] && og["init_pose_arena"].size() == 3) {
      for (size_t i = 0; i < 3; ++i) cfg.init_pose_arena[i] = og["init_pose_arena"][i].as<float>();
    }
    if (og["odom_pos_scale"] && og["odom_pos_scale"].size() == 2) {
      cfg.odom_pos_scale[0] = og["odom_pos_scale"][0].as<float>();
      cfg.odom_pos_scale[1] = og["odom_pos_scale"][1].as<float>();
    }
    if (og["odom_yaw_offset_rad"]) cfg.odom_yaw_offset_rad = og["odom_yaw_offset_rad"].as<float>();
    if (og["cmd_vel_scale"] && og["cmd_vel_scale"].size() == 3) {
      for (size_t i = 0; i < 3; ++i) cfg.cmd_vel_scale[i] = og["cmd_vel_scale"][i].as<float>();
    }
    if (og["cmd_vel_trim"] && og["cmd_vel_trim"].size() == 3) {
      for (size_t i = 0; i < 3; ++i) cfg.cmd_vel_trim[i] = og["cmd_vel_trim"][i].as<float>();
    }
  }

  if (const auto pc = root["perception"]) {
    if (pc["esdf_frame_mode"]) cfg.esdf_frame_mode = pc["esdf_frame_mode"].as<std::string>();
    if (pc["esdf_dist_offset_m"]) cfg.esdf_dist_offset_m = pc["esdf_dist_offset_m"].as<float>();
    if (pc["yolo_cloud_frame"]) cfg.yolo_cloud_frame = pc["yolo_cloud_frame"].as<std::string>();
    if (pc["cam_pos_base_xyz"] && pc["cam_pos_base_xyz"].size() == 3) {
      for (size_t i = 0; i < 3; ++i) cfg.cam_pos_base_xyz[i] = pc["cam_pos_base_xyz"][i].as<float>();
    }
    if (pc["cam_rpy_base_rad"] && pc["cam_rpy_base_rad"].size() == 3) {
      for (size_t i = 0; i < 3; ++i) cfg.cam_rpy_base_rad[i] = pc["cam_rpy_base_rad"][i].as<float>();
    }
    if (pc["lidar_pos_base_xyz"] && pc["lidar_pos_base_xyz"].size() == 3) {
      for (size_t i = 0; i < 3; ++i) cfg.lidar_pos_base_xyz[i] = pc["lidar_pos_base_xyz"][i].as<float>();
    }
    if (pc["lidar_roll_rad"]) cfg.lidar_roll_rad = pc["lidar_roll_rad"].as<float>();
    if (pc["lidar_yaw_offset_rad"]) cfg.lidar_yaw_offset_rad = pc["lidar_yaw_offset_rad"].as<float>();
    if (pc["ultrasonic_pos_base_xyz"] && pc["ultrasonic_pos_base_xyz"].size() == 3) {
      for (size_t i = 0; i < 3; ++i) cfg.ultrasonic_pos_base_xyz[i] = pc["ultrasonic_pos_base_xyz"][i].as<float>();
    }
  }

  return cfg;
}

void CalibrationConfig::save_yaml(const std::string & yaml_path) const
{
  std::ofstream ofs(yaml_path);
  if (!ofs.is_open()) {
    throw std::runtime_error("Cannot write calibration YAML to: " + yaml_path);
  }
  ofs << std::fixed << std::setprecision(6);
  ofs << "# XSting Real-Robot Calibration File (auto-loaded by xsting_nav::nav_node)\n";
  ofs << "# Joint order [0..17]: leg1..leg6_{yaw, hip, knee} == joint1..joint18 (motor IDs 1..18)\n";
  ofs << "hardware_profile: \"" << hardware_profile << "\"\n";
  ofs << "last_calibrated_utc: \"" << last_calibrated_utc << "\"\n\n";

  ofs << "imu:\n";
  ofs << "  # Coarse yaw of IMU relative to base (+X forward, +Y left, +Z up)\n";
  ofs << "  board_yaw_deg: " << imu_board_yaw_deg << "\n";
  ofs << "  gyro_bias_xyz: [" << imu_gyro_bias_xyz[0] << ", " << imu_gyro_bias_xyz[1] << ", "
      << imu_gyro_bias_xyz[2] << "]\n";
  ofs << "  level_quat_wxyz: [" << imu_level_quat_wxyz[0] << ", " << imu_level_quat_wxyz[1] << ", "
      << imu_level_quat_wxyz[2] << ", " << imu_level_quat_wxyz[3] << "]\n\n";

  auto write_arr18 = [&ofs](const std::string & name, const Leg18f & arr) {
      ofs << "  " << name << ":\n";
      for (size_t leg = 0; leg < 6; ++leg) {
        ofs << "    - " << arr[3 * leg + 0] << "  # leg" << (leg + 1) << "_yaw (ID " << (3 * leg + 1) << ")\n";
        ofs << "    - " << arr[3 * leg + 1] << "  # leg" << (leg + 1) << "_hip (ID " << (3 * leg + 2) << ")\n";
        ofs << "    - " << arr[3 * leg + 2] << "  # leg" << (leg + 1) << "_knee (ID " << (3 * leg + 3) << ")\n";
      }
    };

  ofs << "joints:\n";
  write_arr18("leg_sign", leg_sign);
  write_arr18("leg_offset_rad", leg_offset_rad);
  write_arr18("leg_hw_min_rad", leg_hw_min_rad);
  write_arr18("leg_hw_max_rad", leg_hw_max_rad);
  ofs << "  arm_sign: [" << arm_sign[0] << ", " << arm_sign[1] << ", " << arm_sign[2] << ", "
      << arm_sign[3] << ", " << arm_sign[4] << ", " << arm_sign[5] << "]\n";
  ofs << "  arm_offset_rad: [" << arm_offset_rad[0] << ", " << arm_offset_rad[1] << ", "
      << arm_offset_rad[2] << ", " << arm_offset_rad[3] << ", " << arm_offset_rad[4] << ", "
      << arm_offset_rad[5] << "]\n";
  ofs << "  max_joint_slew_rad_s: " << max_joint_slew_rad_s << "\n";
  ofs << "  soft_start_duration_s: " << soft_start_duration_s << "\n\n";

  ofs << "odom_gait:\n";
  ofs << "  init_pose_arena: [" << init_pose_arena[0] << ", " << init_pose_arena[1] << ", "
      << init_pose_arena[2] << "]\n";
  ofs << "  odom_pos_scale: [" << odom_pos_scale[0] << ", " << odom_pos_scale[1] << "]\n";
  ofs << "  odom_yaw_offset_rad: " << odom_yaw_offset_rad << "\n";
  ofs << "  cmd_vel_scale: [" << cmd_vel_scale[0] << ", " << cmd_vel_scale[1] << ", "
      << cmd_vel_scale[2] << "]\n";
  ofs << "  cmd_vel_trim: [" << cmd_vel_trim[0] << ", " << cmd_vel_trim[1] << ", "
      << cmd_vel_trim[2] << "]\n\n";

  ofs << "perception:\n";
  ofs << "  esdf_frame_mode: \"" << esdf_frame_mode << "\"\n";
  ofs << "  esdf_dist_offset_m: " << esdf_dist_offset_m << "\n";
  ofs << "  yolo_cloud_frame: \"" << yolo_cloud_frame << "\"\n";
  ofs << "  cam_pos_base_xyz: [" << cam_pos_base_xyz[0] << ", " << cam_pos_base_xyz[1] << ", "
      << cam_pos_base_xyz[2] << "]\n";
  ofs << "  cam_rpy_base_rad: [" << cam_rpy_base_rad[0] << ", " << cam_rpy_base_rad[1] << ", "
      << cam_rpy_base_rad[2] << "]\n";
  ofs << "  lidar_pos_base_xyz: [" << lidar_pos_base_xyz[0] << ", " << lidar_pos_base_xyz[1] << ", "
      << lidar_pos_base_xyz[2] << "]\n";
  ofs << "  lidar_roll_rad: " << lidar_roll_rad << "\n";
  ofs << "  lidar_yaw_offset_rad: " << lidar_yaw_offset_rad << "\n";
  ofs << "  ultrasonic_pos_base_xyz: [" << ultrasonic_pos_base_xyz[0] << ", "
      << ultrasonic_pos_base_xyz[1] << ", " << ultrasonic_pos_base_xyz[2] << "]\n";
}

float wrap_to_pi(float angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Vec3f quat_to_grav(const Vec4f & quat_wxyz)
{
  const float n = std::sqrt(
    quat_wxyz[0] * quat_wxyz[0] + quat_wxyz[1] * quat_wxyz[1] +
    quat_wxyz[2] * quat_wxyz[2] + quat_wxyz[3] * quat_wxyz[3]);
  if (n < 1e-6f) {
    return {0.0f, 0.0f, -1.0f};
  }
  const float w = quat_wxyz[0] / n;
  const float x = quat_wxyz[1] / n;
  const float y = quat_wxyz[2] / n;
  const float z = quat_wxyz[3] / n;
  const float gx = 2.0f * (w * y - x * z);
  const float gy = -2.0f * (w * x + y * z);
  const float gz = -(1.0f - 2.0f * (x * x + y * y));
  return {gx, gy, gz};
}

float quat_to_yaw(float qx, float qy, float qz, float qw)
{
  const float siny_cosp = 2.0f * (qw * qz + qx * qy);
  const float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
  return std::atan2(siny_cosp, cosy_cosp);
}

Vec4f quat_mul(const Vec4f & a, const Vec4f & b)
{
  return {
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]
  };
}

Vec3f quat_rotate(const Vec4f & q_wxyz, const Vec3f & v)
{
  const Vec4f q = normalize_quat(q_wxyz);
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  const float tx = 2.0f * (y * v[2] - z * v[1]);
  const float ty = 2.0f * (z * v[0] - x * v[2]);
  const float tz = 2.0f * (x * v[1] - y * v[0]);
  return {
    v[0] + w * tx + (y * tz - z * ty),
    v[1] + w * ty + (z * tx - x * tz),
    v[2] + w * tz + (x * ty - y * tx)
  };
}

Vec4f solve_gravity_level_quat(const Vec3f & measured_grav_xyz)
{
  const float n = std::sqrt(
    measured_grav_xyz[0] * measured_grav_xyz[0] +
    measured_grav_xyz[1] * measured_grav_xyz[1] +
    measured_grav_xyz[2] * measured_grav_xyz[2]);
  if (n < 1e-6f) {
    return {1.0f, 0.0f, 0.0f, 0.0f};
  }
  const Vec3f u{measured_grav_xyz[0] / n, measured_grav_xyz[1] / n, measured_grav_xyz[2] / n};
  const float dot = -u[2];
  if (dot < -0.9999f) {
    return {0.0f, 1.0f, 0.0f, 0.0f};
  }
  const Vec4f raw{1.0f + dot, -u[1], u[0], 0.0f};
  return normalize_quat(raw);
}

Vec2f pca_axis_2d(const std::vector<Vec2f> & pts)
{
  if (pts.empty()) {
    return {1.0f, 0.0f};
  }
  double mx = 0.0, my = 0.0;
  for (const auto & p : pts) {
    mx += p[0];
    my += p[1];
  }
  mx /= static_cast<double>(pts.size());
  my /= static_cast<double>(pts.size());

  double cxx = 1e-6, cyy = 1e-6, cxy = 0.0;
  for (const auto & p : pts) {
    const double dx = p[0] - mx;
    const double dy = p[1] - my;
    cxx += dx * dx;
    cyy += dy * dy;
    cxy += dx * dy;
  }

  const double half_diff = 0.5 * (cxx - cyy);
  const double rad = std::hypot(half_diff, cxy);
  double vx = 1.0, vy = 0.0;
  if (std::abs(cxy) > 1e-12) {
    if (half_diff >= 0.0) {
      vx = half_diff + rad;
      vy = cxy;
    } else {
      vx = cxy;
      vy = -half_diff + rad;
    }
  } else {
    if (cxx >= cyy) {
      vx = 1.0;
      vy = 0.0;
    } else {
      vx = 0.0;
      vy = 1.0;
    }
  }
  const double norm = std::hypot(vx, vy);
  if (norm > 1e-12) {
    vx /= norm;
    vy /= norm;
  }
  if (vx < 0.0) {
    vx = -vx;
    vy = -vy;
  }
  return {static_cast<float>(vx), static_cast<float>(vy)};
}

Vec6f transform_calibrated_imu(
  const Vec3f & raw_gyro_xyz,
  const Vec4f & raw_quat_wxyz,
  const CalibrationConfig & calib)
{
  const Vec3f unbiased{
    raw_gyro_xyz[0] - calib.imu_gyro_bias_xyz[0],
    raw_gyro_xyz[1] - calib.imu_gyro_bias_xyz[1],
    raw_gyro_xyz[2] - calib.imu_gyro_bias_xyz[2]
  };
  const float yaw_rad = static_cast<float>(calib.imu_board_yaw_deg * Mk2Kinematics::kPi / 180.0);
  const Vec4f q_yaw = quat_from_rpy(0.0f, 0.0f, yaw_rad);
  const Vec4f q_rot = quat_mul(calib.imu_level_quat_wxyz, q_yaw);

  const Vec3f gyro_base = quat_rotate(q_rot, unbiased);
  const Vec3f grav_raw = quat_to_grav(raw_quat_wxyz);
  const Vec3f grav_base = quat_rotate(q_rot, grav_raw);

  return {
    gyro_base[0], gyro_base[1], gyro_base[2],
    grav_base[0], grav_base[1], grav_base[2]
  };
}

std::vector<Vec2f> transform_yolo_points_xy(
  const std::vector<Vec3f> & raw_pts,
  const CalibrationConfig & calib)
{
  std::vector<Vec2f> out;
  out.reserve(raw_pts.size());
  const Vec4f q_cam = quat_from_rpy(
    calib.cam_rpy_base_rad[0], calib.cam_rpy_base_rad[1], calib.cam_rpy_base_rad[2]);

  for (const auto & p : raw_pts) {
    if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) {
      continue;
    }
    if (calib.yolo_cloud_frame == "camera_optical") {
      const Vec3f p_cam{p[2], -p[0], -p[1]};
      const Vec3f p_rot = quat_rotate(q_cam, p_cam);
      out.push_back({
        p_rot[0] + calib.cam_pos_base_xyz[0],
        p_rot[1] + calib.cam_pos_base_xyz[1]
      });
    } else {
      out.push_back({p[0], p[1]});
    }
  }
  return out;
}

std::vector<Vec2f> synthesize_box_front_points(
  const Vec2f & center_base_xy,
  float box_yaw_base_rad,
  float half_extent_m)
{
  // Select the box face whose inward normal n points away from the robot (dot(n, center) maximized).
  float best_dot = -1e9f;
  Vec2f inward_n{1.0f, 0.0f};
  for (int k = 0; k < 4; ++k) {
    const float ang = box_yaw_base_rad + static_cast<float>(k) * static_cast<float>(0.5 * Mk2Kinematics::kPi);
    const Vec2f cand{std::cos(ang), std::sin(ang)};
    const float d = cand[0] * center_base_xy[0] + cand[1] * center_base_xy[1];
    if (d > best_dot) {
      best_dot = d;
      inward_n = cand;
    }
  }
  const Vec2f face_c{
    center_base_xy[0] - half_extent_m * inward_n[0],
    center_base_xy[1] - half_extent_m * inward_n[1]
  };
  const Vec2f tang{-inward_n[1], inward_n[0]};
  constexpr float kSpan[5] = {-0.08f, -0.04f, 0.00f, 0.04f, 0.08f};
  std::vector<Vec2f> pts;
  pts.reserve(5);
  for (int i = 0; i < 5; ++i) {
    const float depth_jitter = (i == 2) ? 0.004f : 0.0f;
    pts.push_back({
      face_c[0] + kSpan[i] * tang[0] + depth_jitter * inward_n[0],
      face_c[1] + kSpan[i] * tang[1] + depth_jitter * inward_n[1]
    });
  }
  return pts;
}

Vec3f calibrated_odom_pose(const Vec3f & raw_odom_pose, const CalibrationConfig & calib)
{
  return {
    raw_odom_pose[0] * calib.odom_pos_scale[0],
    raw_odom_pose[1] * calib.odom_pos_scale[1],
    wrap_to_pi(raw_odom_pose[2] + calib.odom_yaw_offset_rad)
  };
}

Vec3f compose_se2(const Vec3f & a, const Vec3f & b)
{
  const float ca = std::cos(a[2]);
  const float sa = std::sin(a[2]);
  return {
    a[0] + ca * b[0] - sa * b[1],
    a[1] + sa * b[0] + ca * b[1],
    wrap_to_pi(a[2] + b[2])
  };
}

Vec3f solve_map_to_odom_se2(const Vec3f & arena_pose, const Vec3f & eff_odom_pose)
{
  const float syaw = wrap_to_pi(arena_pose[2] - eff_odom_pose[2]);
  const float cs = std::cos(syaw);
  const float ss = std::sin(syaw);
  const float sx = arena_pose[0] - (cs * eff_odom_pose[0] - ss * eff_odom_pose[1]);
  const float sy = arena_pose[1] - (ss * eff_odom_pose[0] + cs * eff_odom_pose[1]);
  return {sx, sy, syaw};
}

Vec3f blend_se2(
  const Vec3f & current_tf,
  const Vec3f & target_tf,
  float pos_alpha,
  float yaw_alpha,
  float max_step_pos_m,
  float max_step_yaw_rad)
{
  const float a_pos = std::clamp(pos_alpha, 0.0f, 1.0f);
  const float a_yaw = std::clamp(yaw_alpha, 0.0f, 1.0f);
  float dx = a_pos * (target_tf[0] - current_tf[0]);
  float dy = a_pos * (target_tf[1] - current_tf[1]);
  const float step_pos = std::hypot(dx, dy);
  if (max_step_pos_m > 0.0f && step_pos > max_step_pos_m && step_pos > 1e-6f) {
    const float sc = max_step_pos_m / step_pos;
    dx *= sc;
    dy *= sc;
  }
  float dyaw = a_yaw * wrap_to_pi(target_tf[2] - current_tf[2]);
  if (max_step_yaw_rad > 0.0f) {
    dyaw = std::clamp(dyaw, -max_step_yaw_rad, max_step_yaw_rad);
  }
  return {
    current_tf[0] + dx,
    current_tf[1] + dy,
    wrap_to_pi(current_tf[2] + dyaw)
  };
}

Vec3f odom_to_arena_pose(const Vec3f & raw_odom_pose, const CalibrationConfig & calib)
{
  return compose_se2(calib.init_pose_arena, calibrated_odom_pose(raw_odom_pose, calib));
}

Vec3f arena_to_odom_pose(const Vec3f & arena_pose, const Vec3f & init_pose)
{
  const float dx = arena_pose[0] - init_pose[0];
  const float dy = arena_pose[1] - init_pose[1];
  const float cs = std::cos(init_pose[2]);
  const float ss = std::sin(init_pose[2]);
  const float ox = cs * dx + ss * dy;
  const float oy = -ss * dx + cs * dy;
  const float oyaw = wrap_to_pi(arena_pose[2] - init_pose[2]);
  return {ox, oy, oyaw};
}

BoxPreApproachRefinement refine_box_pre_approach_goal(
  const Vec3f & nominal_goal,
  const Vec2f & nominal_box_xy,
  const Vec3f & robot_arena_pose,
  const Vec2f & det_box_base_xy,
  float assoc_radius_m,
  float max_shift_m)
{
  BoxPreApproachRefinement out;
  out.refined_goal = nominal_goal;

  if (!std::isfinite(det_box_base_xy[0]) || !std::isfinite(det_box_base_xy[1])) {
    return out;
  }
  const float det_r = std::hypot(det_box_base_xy[0], det_box_base_xy[1]);
  if (det_box_base_xy[0] < 0.14f || det_r > 2.20f) {
    return out;
  }

  const float cm = std::cos(robot_arena_pose[2]);
  const float sm = std::sin(robot_arena_pose[2]);
  const Vec2f obs_box{
    robot_arena_pose[0] + cm * det_box_base_xy[0] - sm * det_box_base_xy[1],
    robot_arena_pose[1] + sm * det_box_base_xy[0] + cm * det_box_base_xy[1]
  };

  float shift_x = obs_box[0] - nominal_box_xy[0];
  float shift_y = obs_box[1] - nominal_box_xy[1];
  const float assoc_d = std::hypot(shift_x, shift_y);
  if (assoc_d > assoc_radius_m) {
    return out;
  }
  if (assoc_d > max_shift_m && assoc_d > 1e-6f) {
    const float sc = max_shift_m / assoc_d;
    shift_x *= sc;
    shift_y *= sc;
  }

  const Vec2f clamped_box{nominal_box_xy[0] + shift_x, nominal_box_xy[1] + shift_y};
  float standoff_m = std::hypot(
    nominal_goal[0] - nominal_box_xy[0],
    nominal_goal[1] - nominal_box_xy[1]);
  if (standoff_m < 0.36f || standoff_m > 0.65f) {
    standoff_m = Mk2Kinematics::kNominalPreApproachStandoffM;
  }

  const float gyaw = nominal_goal[2];
  out.accepted = true;
  out.box_arena_xy = clamped_box;
  out.refined_goal = {
    clamped_box[0] - standoff_m * std::cos(gyaw),
    clamped_box[1] - standoff_m * std::sin(gyaw),
    gyaw
  };
  return out;
}

bool should_trigger_precision_approach(
  const Vec3f & robot_arena_pose,
  const Vec3f & pre_approach_goal,
  bool box_visible,
  float pos_tol_m,
  float yaw_tol_rad)
{
  if (!box_visible) {
    return false;
  }
  const float eff_tol = std::clamp(pos_tol_m, 0.05f, 0.10f);
  const float dist = std::hypot(
    pre_approach_goal[0] - robot_arena_pose[0],
    pre_approach_goal[1] - robot_arena_pose[1]);
  const float yaw_err = std::abs(wrap_to_pi(pre_approach_goal[2] - robot_arena_pose[2]));
  return (dist <= eff_tol) && (yaw_err <= yaw_tol_rad);
}

EsdfSlice EsdfSlice::make_clear_default(int w, int h, float clear_m)
{
  EsdfSlice s;
  s.width = w;
  s.height = h;
  s.origin_x = -0.5f * w * 0.05f;
  s.origin_y = -0.5f * h * 0.05f;
  s.resolution = 0.05f;
  s.unknown_value = 1000.0f;
  s.data.assign(static_cast<size_t>(w * h), clear_m);
  return s;
}

namespace
{
void edt_1d(const std::vector<float> & f, int n, std::vector<float> & d)
{
  constexpr float kInf = 1e12f;
  std::vector<int> v(n, 0);
  std::vector<float> z(n + 1, 0.0f);
  int k = 0;
  v[0] = 0;
  z[0] = -kInf;
  z[1] = kInf;
  for (int q = 1; q < n; ++q) {
    float s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0f * (q - v[k]));
    while (s <= z[k]) {
      --k;
      if (k < 0) {
        k = 0;
        break;
      }
      s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0f * (q - v[k]));
    }
    if (s > z[k]) {
      ++k;
    }
    v[k] = q;
    z[k] = s;
    z[k + 1] = kInf;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < static_cast<float>(q)) {
      ++k;
    }
    const float diff = static_cast<float>(q - v[k]);
    d[q] = diff * diff + f[v[k]];
  }
}
}  // namespace

EsdfSlice EsdfSlice::from_occupancy_grid(
  int w, int h, float origin_x, float origin_y, float res,
  const std::vector<int8_t> & occ_data, int occ_thresh, float max_dist_m)
{
  EsdfSlice s;
  s.width = w;
  s.height = h;
  s.origin_x = origin_x;
  s.origin_y = origin_y;
  s.resolution = res;
  s.unknown_value = 1000.0f;
  s.data.assign(static_cast<size_t>(w * h), max_dist_m);
  if (w <= 0 || h <= 0 || static_cast<int>(occ_data.size()) < w * h) {
    return s;
  }

  constexpr float kInf = 1e10f;
  std::vector<float> grid(static_cast<size_t>(w * h), kInf);
  bool any_occ = false;
  for (int i = 0; i < w * h; ++i) {
    if (occ_data[i] >= occ_thresh) {
      grid[i] = 0.0f;
      any_occ = true;
    }
  }
  if (!any_occ) {
    return s;
  }

  std::vector<float> f_col(h), d_col(h);
  for (int x = 0; x < w; ++x) {
    for (int y = 0; y < h; ++y) f_col[y] = grid[y * w + x];
    edt_1d(f_col, h, d_col);
    for (int y = 0; y < h; ++y) grid[y * w + x] = d_col[y];
  }
  std::vector<float> f_row(w), d_row(w);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) f_row[x] = grid[y * w + x];
    edt_1d(f_row, w, d_row);
    for (int x = 0; x < w; ++x) {
      const float dist_m = std::sqrt(std::max(0.0f, d_row[x])) * res;
      s.data[y * w + x] = std::min(max_dist_m, dist_m);
    }
  }
  return s;
}

Obs41f build_obs_41d(
  const Vec3f & odom_pose,
  const Vec3f & odom_vel,
  const Vec3f & goal,
  const EsdfSlice & esdf_slice,
  const std::vector<Vec2f> & box_pts_body,
  const std::vector<Vec2f> & puck_pts_body,
  const Vec3f & init_pose,
  const Vec6f & imu_state,
  const Vec3f & nav_waypoint,
  float esdf_max,
  const std::string & esdf_frame_mode,
  float esdf_dist_offset_m)
{
  Obs41f obs{};
  const float ox = odom_pose[0];
  const float oy = odom_pose[1];
  const float oyaw = odom_pose[2];
  const float sx = init_pose[0];
  const float sy = init_pose[1];
  const float syaw = init_pose[2];

  // [0..5] 6D IMU feature vector.
  obs[0] = imu_state[0] * 0.25f;
  obs[1] = imu_state[1] * 0.25f;
  obs[2] = imu_state[2] * 0.25f;
  obs[3] = imu_state[3];
  obs[4] = imu_state[4];
  obs[5] = imu_state[5];

  // [6..16] 11D cuVSLAM + Nav2 lookahead waypoint features.
  const float cs = std::cos(syaw);
  const float ss = std::sin(syaw);
  const float mx = sx + cs * ox - ss * oy;
  const float my = sy + ss * ox + cs * oy;
  const float myaw = wrap_to_pi(syaw + oyaw);
  const float cm = std::cos(myaw);
  const float sm = std::sin(myaw);

  const float tx = nav_waypoint[0];
  const float ty = nav_waypoint[1];
  const float tyaw = nav_waypoint[2];
  const float dx = tx - mx;
  const float dy = ty - my;
  const float gx_b = std::clamp((cm * dx + sm * dy) * 0.25f, -2.0f, 2.0f);
  const float gy_b = std::clamp((-sm * dx + cm * dy) * 0.25f, -2.0f, 2.0f);
  const float dyaw = wrap_to_pi(tyaw - myaw);
  (void)goal;

  obs[6] = mx / 4.404f;
  obs[7] = my / 6.304f;
  obs[8] = sm;
  obs[9] = cm;
  obs[10] = gx_b;
  obs[11] = gy_b;
  obs[12] = std::sin(dyaw);
  obs[13] = std::cos(dyaw);
  obs[14] = odom_vel[0] * 2.0f;
  obs[15] = odom_vel[1] * 2.0f;
  obs[16] = odom_vel[2] * 0.25f;

  // [17..32] 16D 4x4 local ESDF grid sampled in indexing="ij" row-major order.
  const float co = std::cos(oyaw);
  const float so = std::sin(oyaw);
  const int w = esdf_slice.width;
  const int h = esdf_slice.height;
  const bool slice_ok = (w > 0 && h > 0 && static_cast<int>(esdf_slice.data.size()) >= w * h &&
                         esdf_slice.resolution > 1e-5f);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      float qx = kGridX[i];
      float qy = kGridY[j];
      if (esdf_frame_mode != "base_link" && esdf_frame_mode != "base") {
        qx = ox + co * kGridX[i] - so * kGridY[j];
        qy = oy + so * kGridX[i] + co * kGridY[j];
      }
      float val = 1.0f;
      if (slice_ok) {
        const int ix = static_cast<int>(std::floor((qx - esdf_slice.origin_x) / esdf_slice.resolution));
        const int iy = static_cast<int>(std::floor((qy - esdf_slice.origin_y) / esdf_slice.resolution));
        if (ix >= 0 && ix < w && iy >= 0 && iy < h) {
          const float raw = esdf_slice.data[iy * w + ix];
          if (std::isfinite(raw) && std::abs(raw - esdf_slice.unknown_value) > 1e-3f) {
            val = (raw + esdf_dist_offset_m) / esdf_max;
          }
        }
      }
      obs[17 + i * 4 + j] = std::clamp(val, -0.2f, 1.0f);
    }
  }

  // [33..40] 8D YOLO-PCA box & puck centroid + principal axis features.
  if (box_pts_body.size() < 3) {
    for (int k = 33; k < 41; ++k) obs[k] = 0.0f;
  } else {
    float bx = 0.0f, by = 0.0f;
    for (const auto & p : box_pts_body) {
      bx += p[0];
      by += p[1];
    }
    bx /= static_cast<float>(box_pts_body.size());
    by /= static_cast<float>(box_pts_body.size());
    const Vec2f b_axis = pca_axis_2d(box_pts_body);

    const auto & p_src = (puck_pts_body.size() >= 3) ? puck_pts_body : box_pts_body;
    float px = 0.0f, py = 0.0f;
    for (const auto & p : p_src) {
      px += p[0];
      py += p[1];
    }
    px /= static_cast<float>(p_src.size());
    py /= static_cast<float>(p_src.size());
    const Vec2f p_axis = pca_axis_2d(p_src);

    obs[33] = 1.0f;
    obs[34] = bx;
    obs[35] = by;
    obs[36] = b_axis[0];
    obs[37] = b_axis[1];
    obs[38] = px;
    obs[39] = py;
    obs[40] = std::abs(p_axis[1]);
  }
  return obs;
}

std::vector<Vec2f> project_lidar_scan_to_base_xy(
  const std::vector<float> & ranges,
  float angle_min,
  float angle_increment,
  const CalibrationConfig & calib,
  const std::string & frame_id,
  bool apply_self_mask,
  float min_range_m,
  float max_range_m)
{
  std::vector<Vec2f> pts;
  pts.reserve(ranges.size());
  const bool in_base = (frame_id == "base" || frame_id == "base_link");
  const float roll_y = in_base ? 1.0f : std::cos(calib.lidar_roll_rad);
  const float cy = in_base ? 1.0f : std::cos(calib.lidar_yaw_offset_rad);
  const float sy = in_base ? 0.0f : std::sin(calib.lidar_yaw_offset_rad);
  const float tx = in_base ? 0.0f : calib.lidar_pos_base_xyz[0];
  const float ty = in_base ? 0.0f : calib.lidar_pos_base_xyz[1];

  for (size_t i = 0; i < ranges.size(); ++i) {
    const float r = ranges[i];
    if (!std::isfinite(r) || r < min_range_m || r > max_range_m) {
      continue;
    }
    const float ang = angle_min + static_cast<float>(i) * angle_increment;
    const float lx = r * std::cos(ang);
    const float ly = r * std::sin(ang) * roll_y;
    const float bx = cy * lx - sy * ly + tx;
    const float by = sy * lx + cy * ly + ty;

    if (apply_self_mask) {
      const float ax = std::abs(bx);
      const float ay = std::abs(by);
      if ((ax < 0.145f && ay < 0.145f) ||
          (ax < 0.180f && ay >= 0.145f && ay < 0.310f))
      {
        continue;
      }
    }
    pts.push_back({bx, by});
  }
  return pts;
}

void fuse_lidar_ultrasonic_into_obs(
  Obs41f & obs,
  const std::vector<Vec2f> & lidar_pts_base,
  float ultrasonic_range_m,
  float esdf_max,
  const Vec3f & ultrasonic_pos_base)
{
  const bool has_lidar = !lidar_pts_base.empty();
  const bool has_sonar =
    std::isfinite(ultrasonic_range_m) && ultrasonic_range_m >= 0.02f && ultrasonic_range_m <= 4.0f;
  if (!has_lidar && !has_sonar) {
    return;
  }

  const float sx = ultrasonic_pos_base[0] + ultrasonic_range_m;
  const float sy = ultrasonic_pos_base[1];
  const float inv_max = 1.0f / std::max(1e-4f, esdf_max);

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      const float gx = kGridX[i];
      const float gy = kGridY[j];
      float best_d = obs[17 + i * 4 + j] * esdf_max;
      if (has_lidar) {
        float d2_min = std::numeric_limits<float>::infinity();
        for (const auto & p : lidar_pts_base) {
          const float dx = gx - p[0];
          const float dy = gy - p[1];
          const float d2 = dx * dx + dy * dy;
          if (d2 < d2_min) {
            d2_min = d2;
          }
        }
        best_d = std::min(best_d, std::sqrt(d2_min));
      }
      if (has_sonar) {
        best_d = std::min(best_d, std::hypot(gx - sx, gy - sy));
      }
      obs[17 + i * 4 + j] = std::clamp(best_d * inv_max, -0.2f, 1.0f);
    }
  }
}

void MlpPolicy::load_json(const std::string & json_path)
{
  const JsonValue root = JsonValue::parse_file(json_path);
  if (root.at("obs_dim").as_int() != 41 || root.at("act_dim").as_int() != 3) {
    throw std::runtime_error("Policy JSON dimensions do not match 41 -> 3");
  }
  clip_obs_ = root.at("clip_observations").as_float();
  eps_ = root.at("scaler_epsilon").as_float();
  clip_act_ = root.at("clip_actions").as_float();

  const auto spd = root.at("speed_limits_mps_radps").as_float_vec();
  if (spd.size() == 3) {
    speed_limits_ = {spd[0], spd[1], spd[2]};
  }

  const auto mean_vec = root.at("scaler").at("running_mean").as_float_vec();
  const auto var_vec = root.at("scaler").at("running_variance").as_float_vec();
  if (mean_vec.size() != 41 || var_vec.size() != 41) {
    throw std::runtime_error("Invalid RunningStandardScaler length in JSON");
  }
  for (size_t i = 0; i < 41; ++i) {
    mean_[i] = mean_vec[i];
    inv_std_[i] = 1.0f / (std::sqrt(std::max(0.0f, var_vec[i])) + eps_);
  }

  const auto & jlayers = root.at("layers");
  if (jlayers.size() != 3) {
    throw std::runtime_error("Expected 3 linear layers in mk2_nav_policy.json");
  }
  for (size_t l = 0; l < 3; ++l) {
    const auto & jl = jlayers[l];
    layers_[l].in_dim = jl.at("in_features").as_int();
    layers_[l].out_dim = jl.at("out_features").as_int();
    layers_[l].bias = jl.at("bias").as_float_vec();
    layers_[l].weight.clear();
    layers_[l].weight.reserve(static_cast<size_t>(layers_[l].out_dim * layers_[l].in_dim));
    const auto & wrows = jl.at("weight");
    for (size_t r = 0; r < wrows.size(); ++r) {
      const auto row = wrows[r].as_float_vec();
      layers_[l].weight.insert(layers_[l].weight.end(), row.begin(), row.end());
    }
  }
  loaded_ = true;
}

Vec3f MlpPolicy::act_raw(const Obs41f & obs) const
{
  if (!loaded_) {
    throw std::runtime_error("MlpPolicy::act_raw called before load_json()");
  }
  auto elu = [](float x) -> float { return x > 0.0f ? x : (std::exp(x) - 1.0f); };

  std::array<float, 41> x{};
  for (size_t i = 0; i < 41; ++i) {
    const float raw = std::isfinite(obs[i]) ? obs[i] : 0.0f;
    x[i] = std::clamp((raw - mean_[i]) * inv_std_[i], -clip_obs_, clip_obs_);
  }

  std::vector<float> h0(static_cast<size_t>(layers_[0].out_dim), 0.0f);
  for (int o = 0; o < layers_[0].out_dim; ++o) {
    float acc = layers_[0].bias[o];
    const float * wrow = &layers_[0].weight[o * layers_[0].in_dim];
    for (int i = 0; i < layers_[0].in_dim; ++i) {
      acc += wrow[i] * x[i];
    }
    h0[o] = elu(acc);
  }

  std::vector<float> h1(static_cast<size_t>(layers_[1].out_dim), 0.0f);
  for (int o = 0; o < layers_[1].out_dim; ++o) {
    float acc = layers_[1].bias[o];
    const float * wrow = &layers_[1].weight[o * layers_[1].in_dim];
    for (int i = 0; i < layers_[1].in_dim; ++i) {
      acc += wrow[i] * h0[i];
    }
    h1[o] = elu(acc);
  }

  Vec3f out{0.0f, 0.0f, 0.0f};
  for (int o = 0; o < 3; ++o) {
    float acc = layers_[2].bias[o];
    const float * wrow = &layers_[2].weight[o * layers_[2].in_dim];
    for (int i = 0; i < layers_[2].in_dim; ++i) {
      acc += wrow[i] * h1[i];
    }
    out[o] = std::clamp(acc, -clip_act_, clip_act_);
  }
  return out;
}

Vec3f MlpPolicy::act_shielded(const Obs41f & obs) const
{
  return SafetyShield::apply(obs, act_raw(obs));
}

Vec3f SafetyShield::apply(const Obs41f & obs, const Vec3f & raw_act)
{
  auto g = [&obs](int i, int j) -> float { return obs[17 + i * 4 + j]; };

  const float box_d = std::hypot(obs[34], obs[35]);
  const bool dock_box = (obs[33] > 0.5f) && (box_d < 0.72f);
  const bool too_close_box = (obs[33] > 0.5f) && (obs[34] > 0.0f) && (box_d < 0.30f);

  float vx = std::clamp(raw_act[0], -1.0f, 1.0f);
  float vy = std::clamp(raw_act[1], -1.0f, 1.0f);
  float wz = std::clamp(raw_act[2], -1.0f, 1.0f);

  const float r_near = std::min(g(0, 1), g(0, 2));
  const float f_near = std::min(g(2, 1), g(2, 2));
  const float w_min1 = std::min(g(1, 0), g(1, 3));
  const float raw_vx = vx;

  const bool close_f = ((!dock_box) && (f_near < 0.045f)) || (f_near < 0.005f) || too_close_box;
  if (close_f) {
    vx = std::min(vx, 0.0f);
  }
  if (r_near < 0.050f) {
    vx = std::max(vx, 0.0f);
  }

  const bool turn_escape = close_f && (!dock_box) && (raw_vx > 0.15f) && (std::abs(wz) < 0.35f);
  if (turn_escape) {
    wz = (g(2, 3) >= g(2, 0)) ? 0.50f : -0.50f;
  }

  const float l_ext = std::min(g(1, 3), g(2, 3));
  const float l_in = std::min(g(1, 2), g(2, 2));
  const float r_ext = std::min(g(1, 0), g(2, 0));
  const float r_in = std::min(g(1, 1), g(2, 1));

  const bool l_hit = (l_ext < -0.150f) || ((l_in < 0.035f) && (w_min1 > 0.04f) && (!dock_box));
  const bool r_hit = (r_ext < -0.150f) || ((r_in < 0.035f) && (w_min1 > 0.04f) && (!dock_box));
  if (l_hit) {
    vy = std::min(vy, 0.0f);
  }
  if (r_hit) {
    vy = std::max(vy, 0.0f);
  }

  const bool allow_back = dock_box || (f_near < 0.22f);
  vx = allow_back ? std::clamp(vx, -0.30f, 1.0f) : std::clamp(vx, 0.0f, 1.0f);
  if (r_near < 0.050f) {
    vx = std::max(vx, 0.0f);
  }
  const float vy_lim = dock_box ? 0.65f : 1.0f;
  vy = std::clamp(vy, -vy_lim, vy_lim);

  return {
    std::clamp(vx, -1.0f, 1.0f),
    std::clamp(vy, -1.0f, 1.0f),
    std::clamp(wz, -1.0f, 1.0f)
  };
}

SafetyShield::DockResult SafetyShield::dock_vel(
  const std::array<float, 5> & box_obs5, float standoff)
{
  DockResult res;
  const float vis = box_obs5[0];
  if (vis < 0.5f) {
    return res;
  }
  const float dx = box_obs5[1];
  const float dy = box_obs5[2];
  const float nx = box_obs5[3];
  const float ny = box_obs5[4];

  const float ex = dx - standoff * nx;
  const float ey = dy - standoff * ny;
  const float eyaw = std::atan2(ny, nx);
  if (std::hypot(ex, ey) < 0.012f && std::abs(eyaw) < 0.04f) {
    res.aligned = true;
    return res;
  }
  res.vel_cmd[0] = std::clamp(0.9f * ex, -0.10f, 0.12f);
  res.vel_cmd[1] = std::clamp(0.9f * ey, -0.08f, 0.08f);
  res.vel_cmd[2] = std::clamp(1.4f * eyaw, -0.60f, 0.60f);
  return res;
}

int DetourPlanner::place_id_to_goal_id(const std::string & place_id)
{
  std::string up;
  up.reserve(place_id.size());
  for (unsigned char c : place_id) {
    if (!std::isspace(c)) {
      up.push_back(static_cast<char>(std::toupper(c)));
    }
  }
  if (up == "HOME" || up == "START" || up == "0") return 0;
  if (up == "S1") return 1;
  if (up == "S2") return 2;
  if (up == "S3") return 3;
  if (up == "S4") return 4;
  if (up == "S5") return 5;
  if (up == "S6") return 6;
  if (up == "L1") return 7;
  if (up == "L2") return 8;
  if (up == "L3") return 9;
  return -1;
}

std::string DetourPlanner::goal_id_to_place_id(int goal_id)
{
  switch (goal_id) {
    case 0: return "HOME";
    case 1: return "S1";
    case 2: return "S2";
    case 3: return "S3";
    case 4: return "S4";
    case 5: return "S5";
    case 6: return "S6";
    case 7: return "L1";
    case 8: return "L2";
    case 9: return "L3";
    default: return "CUSTOM";
  }
}

void DetourPlanner::load_arena_json(const std::string & json_path)
{
  const JsonValue root = JsonValue::parse_file(json_path);
  nx_ = root.at("nx").as_int();
  ny_ = root.at("ny").as_int();
  res_ = root.at("res_m").as_float();
  static_ce_ = root.at("cell_esdf_row_major").as_float_vec();
  if (static_cast<int>(static_ce_.size()) != nx_ * ny_) {
    throw std::runtime_error("Invalid cell_esdf_row_major size in arena JSON");
  }

  goals_.clear();
  const auto & jgoals = root.at("goals");
  for (size_t i = 0; i < jgoals.size(); ++i) {
    const auto g = jgoals[i].as_float_vec();
    if (g.size() == 3) {
      goals_.push_back({g[0], g[1], g[2]});
    }
  }

  boxes_.clear();
  if (root.contains("boxes")) {
    const auto & jboxes = root.at("boxes");
    for (size_t i = 0; i < jboxes.size(); ++i) {
      const auto b = jboxes[i].as_float_vec();
      if (b.size() >= 2) {
        boxes_.push_back({b[0], b[1]});
      }
    }
  }

  if (goals_.size() > 1) {
    active_gid_ = 1;
    active_goal_ = goals_[1];
  } else if (!goals_.empty()) {
    active_gid_ = 0;
    active_goal_ = goals_[0];
  }
  loaded_ = true;
  dist_map_ = solve_grid_dijkstra({active_goal_[0], active_goal_[1]}, {0.734f, 0.950f}, {});
}

void DetourPlanner::set_goal_by_id(int goal_id, const Vec2f & robot_arena_xy)
{
  if (goals_.empty()) return;
  const int clamped = std::clamp(goal_id, 0, static_cast<int>(goals_.size()) - 1);
  active_gid_ = clamped;
  active_goal_ = goals_[clamped];
  reset_episode_blocks(robot_arena_xy);
}

void DetourPlanner::set_custom_goal(const Vec3f & goal_pose, const Vec2f & robot_arena_xy)
{
  active_gid_ = -1;
  active_goal_ = goal_pose;
  reset_episode_blocks(robot_arena_xy);
}

void DetourPlanner::update_goal_pose_preserving_blocks(
  const Vec3f & goal_pose, const Vec2f & robot_arena_xy)
{
  active_goal_ = goal_pose;
  if (loaded_) {
    dist_map_ = solve_grid_dijkstra(
      {active_goal_[0], active_goal_[1]}, robot_arena_xy, blocked_pts_);
  }
}

void DetourPlanner::reset_episode_blocks(const Vec2f & robot_arena_xy)
{
  block_steps_ = 0;
  human_waiting_ = false;
  blocked_pts_.clear();
  if (loaded_) {
    dist_map_ = solve_grid_dijkstra({active_goal_[0], active_goal_[1]}, robot_arena_xy, {});
  }
}

bool DetourPlanner::nominal_box_xy(int goal_id, Vec2f * out_xy) const
{
  if (goal_id < 1 || static_cast<size_t>(goal_id) > boxes_.size()) {
    return false;
  }
  if (out_xy) {
    *out_xy = boxes_[static_cast<size_t>(goal_id - 1)];
  }
  return true;
}

bool DetourPlanner::is_goal_unreachable(float mx, float my) const
{
  if (!loaded_ || dist_map_.empty()) {
    return false;
  }
  const float eucl_d = std::hypot(active_goal_[0] - mx, active_goal_[1] - my);
  if (eucl_d < 0.45f) {
    return false;
  }
  const int ix = std::clamp(static_cast<int>(mx / res_), 0, nx_ - 1);
  const int iy = std::clamp(static_cast<int>(my / res_), 0, ny_ - 1);
  return dist_map_[ix * ny_ + iy] >= 48.0f;
}

float DetourPlanner::static_clearance(float x, float y) const
{
  if (!loaded_) return 1.0f;
  const int ix = std::clamp(static_cast<int>(x / res_), 0, nx_ - 1);
  const int iy = std::clamp(static_cast<int>(y / res_), 0, ny_ - 1);
  return static_ce_[ix * ny_ + iy];
}

float DetourPlanner::static_esdf_bilinear(
  float x, float y, float * grad_x, float * grad_y) const
{
  if (!loaded_ || nx_ < 2 || ny_ < 2 || static_cast<int>(static_ce_.size()) != nx_ * ny_) {
    if (grad_x) *grad_x = 0.0f;
    if (grad_y) *grad_y = 0.0f;
    return 1.0f;
  }

  // Cell centers are ((i + 0.5)*res_, (j + 0.5)*res_). Allow slight boundary extrapolation
  // (up to 1.5 cells) so outer perimeter walls at x~0, x~4.40, y~0, y~6.30 preserve zero-crossings.
  const float fx = std::clamp((x / res_) - 0.5f, -1.5f, static_cast<float>(nx_) + 0.5f);
  const float fy = std::clamp((y / res_) - 0.5f, -1.5f, static_cast<float>(ny_) + 0.5f);
  const int i0 = std::clamp(static_cast<int>(std::floor(fx)), 0, nx_ - 2);
  const int j0 = std::clamp(static_cast<int>(std::floor(fy)), 0, ny_ - 2);
  const float tx = fx - static_cast<float>(i0);
  const float ty = fy - static_cast<float>(j0);

  const float v00 = static_ce_[i0 * ny_ + j0];
  const float v10 = static_ce_[(i0 + 1) * ny_ + j0];
  const float v01 = static_ce_[i0 * ny_ + (j0 + 1)];
  const float v11 = static_ce_[(i0 + 1) * ny_ + (j0 + 1)];

  if (grad_x) {
    *grad_x = ((1.0f - ty) * (v10 - v00) + ty * (v11 - v01)) / res_;
  }
  if (grad_y) {
    *grad_y = ((1.0f - tx) * (v01 - v00) + tx * (v11 - v10)) / res_;
  }
  return (1.0f - tx) * (1.0f - ty) * v00 +
         tx * (1.0f - ty) * v10 +
         (1.0f - tx) * ty * v01 +
         tx * ty * v11;
}

DetourPlanner::ArenaLocalizationResult DetourPlanner::localize_lidar_on_arena(
  const std::vector<Vec2f> & lidar_pts_base,
  const Vec3f & prior_arena_pose,
  const Vec3f & eff_odom_pose,
  bool global_search,
  int min_inliers,
  float max_rmse_m) const
{
  ArenaLocalizationResult out;
  out.arena_pose = prior_arena_pose;
  out.map_to_odom = solve_map_to_odom_se2(prior_arena_pose, eff_odom_pose);
  if (!loaded_ || static_cast<int>(lidar_pts_base.size()) < std::max(8, min_inliers)) {
    return out;
  }

  // Uniformly subsample up to ~180 rays for sub-millisecond real-time execution.
  const size_t stride = std::max<size_t>(1, lidar_pts_base.size() / 180);
  std::vector<Vec2f> pts;
  pts.reserve(lidar_pts_base.size() / stride + 1);
  for (size_t i = 0; i < lidar_pts_base.size(); i += stride) {
    const auto & p = lidar_pts_base[i];
    if (std::isfinite(p[0]) && std::isfinite(p[1])) {
      pts.push_back(p);
    }
  }
  const int eff_min_inliers = std::max(
    8, std::min(min_inliers, static_cast<int>(pts.size() * 2 / 3)));
  if (static_cast<int>(pts.size()) < eff_min_inliers) {
    return out;
  }

  const float max_x = static_cast<float>(nx_) * res_ + 0.08f;
  const float max_y = static_cast<float>(ny_) * res_ + 0.08f;

  Vec3f best_seed = prior_arena_pose;
  if (global_search) {
    const size_t coarse_stride = std::max<size_t>(1, pts.size() / 72);
    float best_cost = 1e12f;
    constexpr float kHuberCoarse = 0.07f;
    for (int ix = -4; ix <= 4; ++ix) {
      const float cx = prior_arena_pose[0] + static_cast<float>(ix) * 0.06f;
      for (int iy = -4; iy <= 4; ++iy) {
        const float cy = prior_arena_pose[1] + static_cast<float>(iy) * 0.06f;
        if (cx < 0.14f || cx > max_x - 0.22f || cy < 0.14f || cy > max_y - 0.22f) {
          continue;
        }
        if (static_clearance(cx, cy) < 0.10f) {
          continue;
        }
        for (int ia = -4; ia <= 4; ++ia) {
          const float cyaw = wrap_to_pi(
            prior_arena_pose[2] + static_cast<float>(ia) * 0.07f);  // +/- ~16 deg
          const float c = std::cos(cyaw);
          const float s = std::sin(cyaw);
          float sum_cost = 0.0f;
          int valid_n = 0;
          for (size_t k = 0; k < pts.size(); k += coarse_stride) {
            const float wx = cx + c * pts[k][0] - s * pts[k][1];
            const float wy = cy + s * pts[k][0] + c * pts[k][1];
            if (wx < -0.08f || wx > max_x || wy < -0.08f || wy > max_y) {
              sum_cost += 0.10f;
              continue;
            }
            const float ad = std::abs(static_esdf_bilinear(wx, wy));
            const float clipped = std::min(ad, 0.20f);
            sum_cost += (clipped <= kHuberCoarse)
              ? (0.5f * clipped * clipped)
              : (kHuberCoarse * (clipped - 0.5f * kHuberCoarse));
            ++valid_n;
          }
          if (valid_n >= 8) {
            const float avg_cost = sum_cost / static_cast<float>(valid_n);
            if (avg_cost < best_cost) {
              best_cost = avg_cost;
              best_seed = {cx, cy, cyaw};
            }
          }
        }
      }
    }
  }

  // Huber-weighted SE(2) Gauss-Newton refinement with gentle prior regularization.
  Vec3f est = best_seed;
  const float prior_w_pos = global_search ? 0.02f : 0.25f;
  const float prior_w_yaw = global_search ? 0.01f : 0.15f;
  constexpr float kHuberDelta = 0.05f;
  const float gate_m = global_search ? 0.26f : 0.18f;

  for (int iter = 0; iter < 18; ++iter) {
    const float c = std::cos(est[2]);
    const float s = std::sin(est[2]);
    double H[3][3] = {
      {static_cast<double>(prior_w_pos + 1e-4f), 0.0, 0.0},
      {0.0, static_cast<double>(prior_w_pos + 1e-4f), 0.0},
      {0.0, 0.0, static_cast<double>(prior_w_yaw + 1e-4f)}
    };
    double b[3] = {
      static_cast<double>(prior_w_pos * (est[0] - best_seed[0])),
      static_cast<double>(prior_w_pos * (est[1] - best_seed[1])),
      static_cast<double>(prior_w_yaw * wrap_to_pi(est[2] - best_seed[2]))
    };

    int used = 0;
    for (const auto & p : pts) {
      const float wx = est[0] + c * p[0] - s * p[1];
      const float wy = est[1] + s * p[0] + c * p[1];
      if (wx < -0.08f || wx > max_x || wy < -0.08f || wy > max_y) {
        continue;
      }
      float gx = 0.0f;
      float gy = 0.0f;
      const float r = static_esdf_bilinear(wx, wy, &gx, &gy);
      const float ar = std::abs(r);
      if (ar > gate_m) {
        continue;
      }
      const double w = (ar <= kHuberDelta) ? 1.0 : static_cast<double>(kHuberDelta / ar);
      const double rx_th = static_cast<double>(-s * p[0] - c * p[1]);
      const double ry_th = static_cast<double>(c * p[0] - s * p[1]);
      const double J[3] = {
        static_cast<double>(gx),
        static_cast<double>(gy),
        static_cast<double>(gx) * rx_th + static_cast<double>(gy) * ry_th
      };
      for (int r_i = 0; r_i < 3; ++r_i) {
        b[r_i] += w * J[r_i] * static_cast<double>(r);
        for (int c_i = 0; c_i < 3; ++c_i) {
          H[r_i][c_i] += w * J[r_i] * J[c_i];
        }
      }
      ++used;
    }
    if (used < eff_min_inliers) {
      break;
    }

    // Solve 3x3 linear system H * delta = -b via Gaussian elimination with partial pivoting.
    double A[3][4];
    for (int r_i = 0; r_i < 3; ++r_i) {
      for (int c_i = 0; c_i < 3; ++c_i) A[r_i][c_i] = H[r_i][c_i];
      A[r_i][3] = -b[r_i];
    }
    bool singular = false;
    for (int col = 0; col < 3; ++col) {
      int piv = col;
      for (int r_i = col + 1; r_i < 3; ++r_i) {
        if (std::abs(A[r_i][col]) > std::abs(A[piv][col])) piv = r_i;
      }
      if (std::abs(A[piv][col]) < 1e-8) {
        singular = true;
        break;
      }
      if (piv != col) {
        for (int k = col; k < 4; ++k) std::swap(A[col][k], A[piv][k]);
      }
      for (int r_i = col + 1; r_i < 3; ++r_i) {
        const double f = A[r_i][col] / A[col][col];
        for (int k = col; k < 4; ++k) A[r_i][k] -= f * A[col][k];
      }
    }
    if (singular) {
      break;
    }
    double delta[3] = {0.0, 0.0, 0.0};
    for (int r_i = 2; r_i >= 0; --r_i) {
      double s_acc = A[r_i][3];
      for (int k = r_i + 1; k < 3; ++k) s_acc -= A[r_i][k] * delta[k];
      delta[r_i] = s_acc / A[r_i][r_i];
    }

    const float dx = std::clamp(static_cast<float>(delta[0]), -0.05f, 0.05f);
    const float dy = std::clamp(static_cast<float>(delta[1]), -0.05f, 0.05f);
    const float dth = std::clamp(static_cast<float>(delta[2]), -0.08f, 0.08f);
    est[0] += dx;
    est[1] += dy;
    est[2] = wrap_to_pi(est[2] + dth);
    if (dx * dx + dy * dy + dth * dth < 1e-8f) {
      break;
    }
  }

  if (static_clearance(est[0], est[1]) < 0.09f) {
    return out;
  }
  const float max_dev_pos = global_search ? 0.40f : 0.16f;
  const float max_dev_yaw = global_search ? 0.45f : 0.22f;
  if (std::hypot(est[0] - prior_arena_pose[0], est[1] - prior_arena_pose[1]) > max_dev_pos ||
      std::abs(wrap_to_pi(est[2] - prior_arena_pose[2])) > max_dev_yaw)
  {
    return out;
  }

  const float c = std::cos(est[2]);
  const float s = std::sin(est[2]);
  double sq_sum = 0.0;
  int inliers = 0;
  for (const auto & p : pts) {
    const float wx = est[0] + c * p[0] - s * p[1];
    const float wy = est[1] + s * p[0] + c * p[1];
    if (wx < -0.08f || wx > max_x || wy < -0.08f || wy > max_y) {
      continue;
    }
    const float r = static_esdf_bilinear(wx, wy);
    if (std::abs(r) <= 0.10f) {
      sq_sum += static_cast<double>(r) * static_cast<double>(r);
      ++inliers;
    }
  }
  if (inliers < eff_min_inliers) {
    return out;
  }
  const float rmse = static_cast<float>(std::sqrt(sq_sum / static_cast<double>(inliers)));
  if (rmse > max_rmse_m) {
    return out;
  }

  out.converged = true;
  out.arena_pose = est;
  out.map_to_odom = solve_map_to_odom_se2(est, eff_odom_pose);
  out.rmse_m = rmse;
  out.inliers = inliers;
  return out;
}

std::vector<float> DetourPlanner::solve_grid_dijkstra(
  const Vec2f & goal_xy,
  const Vec2f & rob_xy,
  const std::vector<Vec2f> & blocked_pts) const
{
  std::vector<float> eff_ce = static_ce_;
  const float rx_m = rob_xy[0];
  const float ry_m = rob_xy[1];

  for (const auto & bp : blocked_pts) {
    const float bx = bp[0];
    const float by = bp[1];
    const float d_rob = std::hypot(rx_m - bx, ry_m - by);
    for (int i = 0; i < nx_; ++i) {
      const float gx = (static_cast<float>(i) + 0.5f) * res_;
      for (int j = 0; j < ny_; ++j) {
        const float gy = (static_cast<float>(j) + 0.5f) * res_;
        const float dh = std::hypot(gx - bx, gy - by);
        float hum_ce = dh - (0.50f - 0.22f);
        if (d_rob <= 0.68f) {
          if (std::hypot(gx - rx_m, gy - ry_m) < 0.30f &&
              dh >= std::max(0.38f, d_rob - 0.06f))
          {
            hum_ce = std::max(hum_ce, 0.32f);
          }
        }
        const size_t idx = static_cast<size_t>(i * ny_ + j);
        eff_ce[idx] = std::min(eff_ce[idx], hum_ce);
      }
    }
  }

  std::vector<float> dist(static_cast<size_t>(nx_ * ny_), 50.0f);
  const int gi = std::clamp(static_cast<int>(goal_xy[0] / res_), 0, nx_ - 1);
  const int gj = std::clamp(static_cast<int>(goal_xy[1] / res_), 0, ny_ - 1);
  if (eff_ce[gi * ny_ + gj] <= 0.22f) {
    return dist;
  }

  using Node = std::tuple<float, int, int>;
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
  dist[gi * ny_ + gj] = 0.0f;
  pq.emplace(0.0f, gi, gj);

  const float diag = std::hypot(res_, res_);
  const std::array<std::tuple<int, int, float>, 8> shifts{{
    {-1, 0, res_}, {1, 0, res_}, {0, -1, res_}, {0, 1, res_},
    {-1, -1, diag}, {-1, 1, diag}, {1, -1, diag}, {1, 1, diag}
  }};

  while (!pq.empty()) {
    const auto [du, i, j] = pq.top();
    pq.pop();
    if (du > dist[i * ny_ + j]) {
      continue;
    }
    const float cu = eff_ce[i * ny_ + j];
    const float pu = 0.25f * std::max(0.0f, 0.45f - cu) + 4.0f * std::max(0.0f, 0.30f - cu);
    for (const auto & [sx, sy, sl] : shifts) {
      const int ni = i + sx;
      const int nj = j + sy;
      if (ni < 0 || ni >= nx_ || nj < 0 || nj >= ny_) {
        continue;
      }
      const float cv = eff_ce[ni * ny_ + nj];
      if (cv <= 0.22f) {
        continue;
      }
      if (sx != 0 && sy != 0) {
        if (!(eff_ce[ni * ny_ + j] > 0.29f && eff_ce[i * ny_ + nj] > 0.29f)) {
          continue;
        }
      }
      const float pv = 0.25f * std::max(0.0f, 0.45f - cv) + 4.0f * std::max(0.0f, 0.30f - cv);
      const float nd = du + sl + 0.5f * (pu + pv);
      if (nd < dist[ni * ny_ + nj]) {
        dist[ni * ny_ + nj] = nd;
        pq.emplace(nd, ni, nj);
      }
    }
  }
  return dist;
}

Vec3f DetourPlanner::raw_waypoint(float mx, float my) const
{
  const float eucl_d = std::hypot(active_goal_[0] - mx, active_goal_[1] - my);
  if (eucl_d < 0.38f || !loaded_ || dist_map_.empty()) {
    const float wyaw = (eucl_d < 0.165f) ? active_goal_[2]
                                         : std::atan2(active_goal_[1] - my, active_goal_[0] - mx);
    return {active_goal_[0], active_goal_[1], wyaw};
  }
  int cx = std::clamp(static_cast<int>(mx / res_), 1, nx_ - 2);
  int cy = std::clamp(static_cast<int>(my / res_), 1, ny_ - 2);
  constexpr int kDirs[8][2] = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1},
    {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
  };
  for (int step = 0; step < 4; ++step) {
    float best_d = dist_map_[cx * ny_ + cy];
    int bx = cx, by = cy;
    for (const auto & d : kDirs) {
      const int nx = std::clamp(cx + d[0], 0, nx_ - 1);
      const int ny = std::clamp(cy + d[1], 0, ny_ - 1);
      const float cand = dist_map_[nx * ny_ + ny];
      if (cand < best_d) {
        best_d = cand;
        bx = nx;
        by = ny;
      }
    }
    cx = bx;
    cy = by;
  }
  const float wx = (static_cast<float>(cx) + 0.5f) * res_;
  const float wy = (static_cast<float>(cy) + 0.5f) * res_;
  const float wyaw = (eucl_d < 0.165f) ? active_goal_[2] : std::atan2(wy - my, wx - mx);
  return {wx, wy, wyaw};
}

Vec3f DetourPlanner::active_waypoint(float mx, float my, float myaw) const
{
  if (human_waiting_) {
    return {mx, my, myaw};
  }
  return raw_waypoint(mx, my);
}

float DetourPlanner::path_distance(float mx, float my) const
{
  const float eucl_d = std::hypot(active_goal_[0] - mx, active_goal_[1] - my);
  if (!loaded_ || dist_map_.empty()) {
    return eucl_d;
  }
  const int ix = std::clamp(static_cast<int>(mx / res_), 0, nx_ - 1);
  const int iy = std::clamp(static_cast<int>(my / res_), 0, ny_ - 1);
  const float gd = dist_map_[ix * ny_ + iy];
  return (eucl_d < 0.40f || gd >= 49.0f) ? eucl_d : gd;
}

void DetourPlanner::update_blocking_replan(
  float mx, float my, float myaw,
  const Obs41f & probe_obs,
  float step_dt,
  float human_wait_timeout_s,
  float goal_tol_m,
  bool reroute_unreachable_goal)
{
  if (!loaded_) return;
  const float eucl_d = std::hypot(active_goal_[0] - mx, active_goal_[1] - my);
  if (eucl_d <= goal_tol_m) {
    block_steps_ = 0;
    human_waiting_ = false;
    return;
  }

  auto g = [&probe_obs](int i, int j) -> float { return probe_obs[17 + i * 4 + j]; };
  const float front_esdf = std::min(std::min(g(2, 1), g(2, 2)), std::min(g(3, 1), g(3, 2)));
  const float fx = mx + 0.62f * std::cos(myaw);
  const float fy = my + 0.62f * std::sin(myaw);
  const Vec3f wp = raw_waypoint(mx, my);
  const bool heading_to_wp = std::cos(std::atan2(wp[1] - my, wp[0] - mx) - myaw) > 0.25f;

  bool already_masked = false;
  for (const auto & bp : blocked_pts_) {
    if (std::hypot(fx - bp[0], fy - bp[1]) < 0.45f) {
      already_masked = true;
      break;
    }
  }

  const bool dyn_person_ahead =
    (front_esdf < 0.28f) &&
    (static_clearance(fx, fy) > 0.32f) &&
    heading_to_wp &&
    (!already_masked);

  if (!dyn_person_ahead) {
    block_steps_ = 0;
    human_waiting_ = false;
    return;
  }

  block_steps_ += 1;
  const int need_steps = std::max(1, static_cast<int>(std::lround(human_wait_timeout_s / step_dt)));
  if (block_steps_ < need_steps) {
    human_waiting_ = true;
    return;
  }

  // Blocked >= 10.0s: mask blocking person and compute corridor detour (zero teleportation).
  blocked_pts_.push_back({fx, fy});
  block_steps_ = 0;
  human_waiting_ = false;
  human_replans_ += 1;

  std::vector<float> new_map = solve_grid_dijkstra(
    {active_goal_[0], active_goal_[1]}, {mx, my}, blocked_pts_);
  const int ri = std::clamp(static_cast<int>(mx / res_), 0, nx_ - 1);
  const int rj = std::clamp(static_cast<int>(my / res_), 0, ny_ - 1);

  if (new_map[ri * ny_ + rj] >= 48.0f && reroute_unreachable_goal && !goals_.empty()) {
    float best_cost = 50.0f;
    int best_gid = -1;
    std::vector<int> candidates;
    for (int id = 1; id < static_cast<int>(goals_.size()); ++id) candidates.push_back(id);
    candidates.push_back(0);

    for (int cand_gid : candidates) {
      const Vec3f & cand_g = goals_[cand_gid];
      if (std::hypot(cand_g[0] - mx, cand_g[1] - my) < 0.75f) {
        continue;
      }
      std::vector<float> cand_map = solve_grid_dijkstra(
        {cand_g[0], cand_g[1]}, {mx, my}, blocked_pts_);
      const float c_val = cand_map[ri * ny_ + rj];
      if (c_val < best_cost && c_val < 48.0f && (cand_gid > 0 || best_gid < 0)) {
        best_cost = c_val;
        best_gid = cand_gid;
        active_gid_ = cand_gid;
        active_goal_ = cand_g;
        new_map = std::move(cand_map);
      }
    }
  }
  dist_map_ = std::move(new_map);
}

LegGaitController::LegGaitController(float freq, float step_h, float max_stride)
: freq_(freq), step_h_(step_h), max_stride_(max_stride), phase_(0.0f),
  q_sim_(Mk2Kinematics::stand_pose())
{
}

Leg18f LegGaitController::solve_leg_ik(const std::array<Vec3f, 6> & foot_body)
{
  const float l1 = Mk2Kinematics::kL1;
  const float l2 = std::hypot(Mk2Kinematics::kTipR, Mk2Kinematics::kTipZ);
  const float alpha = std::atan2(Mk2Kinematics::kTipZ, Mk2Kinematics::kTipR);
  const float d2_max = (l1 + l2 - 0.001f) * (l1 + l2 - 0.001f);

  Leg18f q{};
  for (size_t leg = 0; leg < 6; ++leg) {
    const float dx = foot_body[leg][0] - Mk2Kinematics::kMountX[leg];
    const float dy = Mk2Kinematics::kSide[leg] * (foot_body[leg][1] - Mk2Kinematics::kMountY[leg]);
    const float yaw = std::atan2(dx, std::max(0.02f, dy));
    const float r = std::max(0.01f, std::hypot(dx, dy) - Mk2Kinematics::kHipR);
    const float z = foot_body[leg][2] - Mk2Kinematics::kHipZ;

    const float d2 = std::clamp(r * r + z * z, 0.0009f, d2_max);
    const float d = std::sqrt(d2);
    const float cos_b = std::clamp((d2 + l1 * l1 - l2 * l2) / (2.0f * d * l1), -1.0f, 1.0f);
    const float hip = std::atan2(z, r) + std::acos(cos_b);
    const float phi = std::atan2(z - l1 * std::sin(hip), r - l1 * std::cos(hip));
    const float knee = phi - hip - alpha;

    q[3 * leg + 0] = std::clamp(yaw, Mk2Kinematics::kLegLow[3 * leg + 0], Mk2Kinematics::kLegHigh[3 * leg + 0]);
    q[3 * leg + 1] = std::clamp(hip, Mk2Kinematics::kLegLow[3 * leg + 1], Mk2Kinematics::kLegHigh[3 * leg + 1]);
    q[3 * leg + 2] = std::clamp(knee, Mk2Kinematics::kLegLow[3 * leg + 2], Mk2Kinematics::kLegHigh[3 * leg + 2]);
  }
  return q;
}

Leg18f LegGaitController::compute_gait_joints(
  const Vec3f & vel_body, float phase, float freq, float step_h, float max_stride)
{
  const float l1 = Mk2Kinematics::kL1;
  const float l2 = std::hypot(Mk2Kinematics::kTipR, Mk2Kinematics::kTipZ);
  const float alpha = std::atan2(Mk2Kinematics::kTipZ, Mk2Kinematics::kTipR);
  const float hip0 = Mk2Kinematics::kHip0;
  const float knee0 = Mk2Kinematics::kKnee0;
  const float r0 = Mk2Kinematics::kHipR + l1 * std::cos(hip0) + l2 * std::cos(hip0 + knee0 + alpha);
  const float z0 = Mk2Kinematics::kHipZ + l1 * std::sin(hip0) + l2 * std::sin(hip0 + knee0 + alpha);

  const float vx = vel_body[0];
  const float vy = vel_body[1];
  const float wz = vel_body[2];
  const float lat = std::clamp(std::abs(vy) / 0.18f, 0.0f, 1.0f);
  const float r_eff = r0 + 0.016f * lat;
  const float t_st = 0.5f / freq;
  const float two_pi = static_cast<float>(2.0 * Mk2Kinematics::kPi);
  const float pi = static_cast<float>(Mk2Kinematics::kPi);

  std::array<Vec3f, 6> feet{};
  for (size_t leg = 0; leg < 6; ++leg) {
    const float yaw0 = static_cast<float>(Mk2Kinematics::kYaw0Deg[leg] * Mk2Kinematics::kPi / 180.0);
    const float x0 = Mk2Kinematics::kMountX[leg] + r_eff * std::sin(yaw0);
    const float y0 = Mk2Kinematics::kMountY[leg] + Mk2Kinematics::kSide[leg] * r_eff * std::cos(yaw0);

    const float vg_x = -vx + wz * y0;
    const float vg_y = -vy - wz * x0;

    float hx = 0.5f * t_st * vg_x;
    float hy = 0.5f * t_st * vg_y;
    const float rad = std::max(1e-6f, std::hypot(hx, hy));
    const float scale = std::min(1.0f, max_stride / rad);
    hx *= scale;
    hy *= scale;

    const float tri_off = (leg % 2 == 0) ? 0.0f : pi;
    float phi = std::fmod(phase + tri_off, two_pi);
    if (phi < 0.0f) phi += two_pi;

    const bool in_st = phi < pi;
    const float s_sw = std::clamp((phi - pi) / pi, 0.0f, 1.0f);
    const float u = in_st ? (2.0f * (phi / pi) - 1.0f) : std::cos(pi * s_sw);
    const float amp = std::clamp(std::hypot(vg_x, vg_y) / 0.04f, 0.0f, 1.0f);
    const float lift = in_st ? 0.0f : (step_h * std::sin(pi * s_sw));

    feet[leg][0] = x0 + u * hx * amp;
    feet[leg][1] = y0 + u * hy * amp;
    feet[leg][2] = z0 + lift * amp;
  }
  return solve_leg_ik(feet);
}

Leg18f LegGaitController::reset()
{
  phase_ = 0.0f;
  q_sim_ = Mk2Kinematics::stand_pose();
  return q_sim_;
}

void LegGaitController::sync_from_hardware(const Leg18f & q_hw, const CalibrationConfig & calib)
{
  phase_ = 0.0f;
  q_sim_ = hardware_to_sim_legs(q_hw, calib);
}

Leg18f LegGaitController::step_sim(const Vec3f & vel_body, float dt)
{
  const float vx = vel_body[0];
  const float vy = vel_body[1];
  const float wz = vel_body[2];
  const float spd = std::hypot(vx, vy) + 0.26f * std::abs(wz);
  if (spd < 0.01f) {
    phase_ = 0.0f;
    const Leg18f stand = Mk2Kinematics::stand_pose();
    const float alpha = std::min(1.0f, 12.0f * dt);
    for (size_t i = 0; i < 18; ++i) {
      q_sim_[i] += alpha * (stand[i] - q_sim_[i]);
    }
    return q_sim_;
  }
  const float two_pi = static_cast<float>(2.0 * Mk2Kinematics::kPi);
  const float rate = two_pi * freq_ * std::min(1.25f, std::max(0.35f, spd / 0.25f));
  phase_ = std::fmod(phase_ + rate * dt, two_pi);
  if (phase_ < 0.0f) phase_ += two_pi;
  q_sim_ = compute_gait_joints(vel_body, phase_, freq_, step_h_, max_stride_);
  return q_sim_;
}

Leg18f LegGaitController::step_hardware(
  const Vec3f & cmd_vel_body,
  const CalibrationConfig & calib,
  float dt)
{
  Vec3f scaled_vel{0.0f, 0.0f, 0.0f};
  const float spd = std::hypot(cmd_vel_body[0], cmd_vel_body[1]) + 0.26f * std::abs(cmd_vel_body[2]);
  if (spd >= 0.01f) {
    scaled_vel[0] = cmd_vel_body[0] * calib.cmd_vel_scale[0] + calib.cmd_vel_trim[0];
    scaled_vel[1] = cmd_vel_body[1] * calib.cmd_vel_scale[1] + calib.cmd_vel_trim[1];
    scaled_vel[2] = cmd_vel_body[2] * calib.cmd_vel_scale[2] + calib.cmd_vel_trim[2];
  }
  const Leg18f q_sim = step_sim(scaled_vel, dt);
  return sim_to_hardware_legs(q_sim, calib);
}

Leg18f LegGaitController::sim_to_hardware_legs(const Leg18f & q_sim, const CalibrationConfig & calib)
{
  Leg18f q_hw{};
  constexpr float kMargin = 1e-3f;
  for (size_t i = 0; i < 18; ++i) {
    const float raw = calib.leg_sign[i] * q_sim[i] + calib.leg_offset_rad[i];
    const float lo = calib.leg_hw_min_rad[i] + kMargin;
    const float hi = calib.leg_hw_max_rad[i] - kMargin;
    q_hw[i] = std::clamp(std::isfinite(raw) ? raw : calib.leg_offset_rad[i], lo, hi);
  }
  return q_hw;
}

Leg18f LegGaitController::hardware_to_sim_legs(const Leg18f & q_hw, const CalibrationConfig & calib)
{
  Leg18f q_sim{};
  for (size_t i = 0; i < 18; ++i) {
    const float s = (std::abs(calib.leg_sign[i]) < 0.5f) ? 1.0f : calib.leg_sign[i];
    q_sim[i] = (q_hw[i] - calib.leg_offset_rad[i]) / s;
  }
  return q_sim;
}

Arm6f LegGaitController::sim_to_hardware_arm(const Arm6f & q_arm_sim, const CalibrationConfig & calib)
{
  Arm6f out{};
  for (size_t i = 0; i < 6; ++i) {
    out[i] = calib.arm_sign[i] * q_arm_sim[i] + calib.arm_offset_rad[i];
  }
  return out;
}

}  // namespace xsting_nav
