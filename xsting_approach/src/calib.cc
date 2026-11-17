#include "xsting_approach/calib.h"

#include <Eigen/Eigenvalues>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>

#include "xsting_approach/pca_vis.h"

namespace xsting_approach {

namespace {

// 각도 변환 상수 및 오일러각(deg) 회전 행렬 생성
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

Eigen::Matrix3d MakeRpyMat(const Eigen::Vector3d& rpy_deg) {
  return (Eigen::AngleAxisd(rpy_deg.z() * kDegToRad, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(rpy_deg.y() * kDegToRad, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rpy_deg.x() * kDegToRad, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

}  // namespace

// WalkNet 정책 학습 순서 다리 관절 이름 반환
const std::array<std::string, kNumLegs>& GetWalkLegNames() {
  static const std::array<std::string, kNumLegs> kNames = {
      "leg3_yaw", "leg3_hip", "leg3_knee", "leg2_yaw", "leg2_hip", "leg2_knee",
      "leg1_yaw", "leg1_hip", "leg1_knee", "leg6_yaw", "leg6_hip", "leg6_knee",
      "leg5_yaw", "leg5_hip", "leg5_knee", "leg4_yaw", "leg4_hip", "leg4_knee",
  };
  return kNames;
}

// xsting_hardware leg_controller 명령 순서 다리 관절 이름 반환
const std::array<std::string, kNumLegs>& GetStdLegNames() {
  static const std::array<std::string, kNumLegs> kNames = {
      "leg1_yaw", "leg1_hip", "leg1_knee", "leg2_yaw", "leg2_hip", "leg2_knee",
      "leg3_yaw", "leg3_hip", "leg3_knee", "leg4_yaw", "leg4_hip", "leg4_knee",
      "leg5_yaw", "leg5_hip", "leg5_knee", "leg6_yaw", "leg6_hip", "leg6_knee",
  };
  return kNames;
}

// xsting_hardware arm_controller 명령 순서 팔 관절 이름 반환
const std::array<std::string, kNumArm>& GetArmNames() {
  static const std::array<std::string, kNumArm> kNames = {
      "arm_yaw",   "arm_shoulder", "arm_elbow",
      "arm_wrist", "arm_roll",     "finger_l",
  };
  return kNames;
}

// xsting_hardware 부팅 시 초기 기립 자세(URDF 기준 rad/m) 반환
double GetBootRad(const std::string& joint_name) {
  if (joint_name == "leg1_yaw" || joint_name == "leg4_yaw") return M_PI / 6.0;
  if (joint_name == "leg3_yaw" || joint_name == "leg6_yaw") return -M_PI / 6.0;
  if (joint_name.find("_hip") != std::string::npos) return 15.0 * kDegToRad;
  if (joint_name.find("_knee") != std::string::npos) return -M_PI / 2.0;
  if (joint_name == "arm_shoulder") return -1.0508;
  if (joint_name == "arm_elbow") return 1.0707;
  return 0.0;
}

// leg_controller 출력 순서에 맞춘 WalkNet 서기 자세(base_z=21.3mm) 하드웨어 목표각 반환
std::array<double, kNumLegs> BuildStandLegCmd(const Calib& calib) {
  std::array<double, kNumLegs> cmd{};
  for (int out_i = 0; out_i < kNumLegs; ++out_i) {
    int idx = out_i;
    if (static_cast<size_t>(out_i) < calib.out_leg_order.size()) {
      const auto& target_name = calib.out_leg_order[out_i];
      for (int j = 0; j < kNumLegs; ++j) {
        if (calib.legs[j].ros_name == target_name ||
            calib.legs[j].name == target_name) {
          idx = j;
          break;
        }
      }
    }
    cmd[out_i] = calib.legs[idx].ToHw(calib.legs[idx].stand_rad);
  }
  return cmd;
}

// 원시 IMU 출력을 차체 기준 각속도와 단위 중력 벡터로 변환
void ImuCalib::Transform(const Eigen::Vector3d& gyro_raw,
                         const Eigen::Quaterniond& quat_raw,
                         Eigen::Vector3f* ang_vel_b,
                         Eigen::Vector3f* proj_grav_b) const {
  const Eigen::Quaterniond q_m = mount_quat.normalized();
  const Eigen::Vector3d w_b = q_m * (gyro_raw - gyro_bias);
  Eigen::Quaterniond q_s =
      quat_raw.norm() < 1e-6 ? Eigen::Quaterniond::Identity()
                             : quat_raw.normalized();
  const Eigen::Vector3d grav_s =
      q_s.conjugate() * Eigen::Vector3d(0.0, 0.0, -1.0);
  const Eigen::Vector3d grav_b = (q_m * grav_s).normalized();
  if (ang_vel_b) *ang_vel_b = w_b.cast<float>();
  if (proj_grav_b) *proj_grav_b = grav_b.cast<float>();
}

// 깊이 광학 프레임(+X 우, +Y 하, +Z 전)에서 차체로의 회전 행렬 반환
Eigen::Matrix3f CamCalib::OptToBaseRot() const {
  Eigen::Matrix3d r_opt;
  r_opt << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
  return (MakeRpyMat(cam_rpy_deg) * r_opt).cast<float>();
}

// URDF 및 WalkNet 기본값을 반영한 표준 xSting 보정 구조체 생성
Calib Calib::MakeDefault() {
  Calib c;
  const auto& wnames = GetWalkLegNames();
  const std::array<double, 6> yaw_deg = {-40.0, 0.0, 40.0, -40.0, 0.0, 40.0};
  const std::array<double, 6> yaw_low = {-1.5708, -0.436332, -0.436332,
                                         -1.5708, -0.750492, -0.436332};
  const std::array<double, 6> yaw_high = {0.436332, 0.750492, 1.5708,
                                          0.436332, 0.436332, 1.5708};

  // 다리 18축 기립각 및 관절 한계 초기화
  for (int leg = 0; leg < 6; ++leg) {
    const int idx = leg * 3;
    c.legs[idx + 0] = {wnames[idx + 0], wnames[idx + 0], 1.0, 0.0,
                       yaw_low[leg], yaw_high[leg], yaw_deg[leg] * kDegToRad};
    c.legs[idx + 1] = {wnames[idx + 1], wnames[idx + 1], 1.0, 0.0,
                       -1.65806, 1.65806, 30.0 * kDegToRad};
    c.legs[idx + 2] = {wnames[idx + 2], wnames[idx + 2], 1.0, 0.0,
                       -2.00713, 1.74533, -110.0 * kDegToRad};
  }

  // 팔 6축 보행 중 홈 자세 및 한계 초기화
  const auto& anames = GetArmNames();
  const double sh = 0.10 - 80.0 * kDegToRad;
  const double el = 80.0 * kDegToRad - 0.10;
  const std::array<double, kNumArm> arm_stand = {0.0, sh, el, 0.0, 0.0, 0.0};
  const std::array<double, kNumArm> arm_low = {-3.14159, -1.658063, -2.007129,
                                               -1.8326,  -3.14159,  -0.00186};
  const std::array<double, kNumArm> arm_high = {3.14159, 1.919862, 1.570796,
                                                2.18166, 3.14159,  0.025};
  for (int i = 0; i < kNumArm; ++i) {
    c.arm[i] = {anames[i], anames[i], 1.0,        0.0,
                arm_low[i], arm_high[i], arm_stand[i]};
  }

  const auto& std_legs = GetStdLegNames();
  c.out_leg_order.assign(std_legs.begin(), std_legs.end());
  return c;
}

// YAML 파일에서 전체 실기 보정 설정 로드
bool Calib::Load(const std::string& path, std::string* err) {
  try {
    const YAML::Node root = YAML::LoadFile(path);

    // 출력 다리 관절 순서 로드
    if (root["output_leg_order"] && root["output_leg_order"].IsSequence()) {
      std::vector<std::string> order;
      for (const auto& item : root["output_leg_order"]) {
        order.push_back(item.as<std::string>());
      }
      if (order.size() == kNumLegs) out_leg_order = std::move(order);
    }

    // 다리 및 팔 관절 맵 로드
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

    // IMU 장착 쿼터니언 및 자이로 바이어스 로드
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
    }

    // 카메라 및 박스 접근 파라미터 로드
    if (const YAML::Node cn = root["camera_box"]) {
      if (cn["cam_pos_m"] && cn["cam_pos_m"].size() == 3) {
        cam.cam_pos_m = Eigen::Vector3d(cn["cam_pos_m"][0].as<double>(),
                                        cn["cam_pos_m"][1].as<double>(),
                                        cn["cam_pos_m"][2].as<double>());
      }
      if (cn["cam_rpy_deg"] && cn["cam_rpy_deg"].size() == 3) {
        cam.cam_rpy_deg = Eigen::Vector3d(cn["cam_rpy_deg"][0].as<double>(),
                                          cn["cam_rpy_deg"][1].as<double>(),
                                          cn["cam_rpy_deg"][2].as<double>());
      }
      if (cn["base_z_m"]) cam.base_z_m = cn["base_z_m"].as<double>();
      if (cn["robot_front_x_m"]) {
        cam.robot_front_x_m = cn["robot_front_x_m"].as<double>();
      }
      if (cn["target_gap_m"]) cam.target_gap_m = cn["target_gap_m"].as<double>();
      if (cn["box_half_m"]) cam.box_half_m = cn["box_half_m"].as<double>();
      if (cn["box_height_m"]) cam.box_height_m = cn["box_height_m"].as<double>();
      if (cn["puck_radius_m"]) {
        cam.puck_radius_m = cn["puck_radius_m"].as<double>();
      }
      if (cn["standoff_m"]) cam.standoff_m = cn["standoff_m"].as<double>();
      if (cn["pos_tol_m"]) cam.pos_tol_m = cn["pos_tol_m"].as<double>();
      if (cn["yaw_tol_rad"]) cam.yaw_tol_rad = cn["yaw_tol_rad"].as<double>();
      if (cn["hold_time_s"]) cam.hold_time_s = cn["hold_time_s"].as<double>();
    }

    // 초음파 센서 파라미터 로드
    if (const YAML::Node un = root["ultrasonic"]) {
      if (un["enabled"]) sonar.enabled = un["enabled"].as<bool>();
      if (un["us_pos_m"] && un["us_pos_m"].size() == 3) {
        sonar.us_pos_m = Eigen::Vector3d(un["us_pos_m"][0].as<double>(),
                                         un["us_pos_m"][1].as<double>(),
                                         un["us_pos_m"][2].as<double>());
      }
      if (un["min_range_m"]) sonar.min_range_m = un["min_range_m"].as<double>();
      if (un["max_range_m"]) sonar.max_range_m = un["max_range_m"].as<double>();
      if (un["range_bias_m"]) {
        sonar.range_bias_m = un["range_bias_m"].as<double>();
      }
      if (un["close_range_fusion"]) {
        sonar.close_fusion = un["close_range_fusion"].as<bool>();
      }
      if (un["safety_brake_gap_m"]) {
        sonar.brake_gap_m = un["safety_brake_gap_m"].as<double>();
      }
    }

    // 보행 안전 파라미터 로드
    if (const YAML::Node gn = root["gait_safety"]) {
      if (gn["walk_scale"]) gait.walk_scale = gn["walk_scale"].as<double>();
      if (gn["gait_freq_hz"]) {
        gait.gait_freq_hz = gn["gait_freq_hz"].as<double>();
      }
      if (gn["speed_limits"] && gn["speed_limits"].size() == 3) {
        gait.speed_lim = Eigen::Vector3d(gn["speed_limits"][0].as<double>(),
                                         gn["speed_limits"][1].as<double>(),
                                         gn["speed_limits"][2].as<double>());
      }
      if (gn["speed_scale"] && gn["speed_scale"].size() == 3) {
        gait.speed_scale = Eigen::Vector3d(gn["speed_scale"][0].as<double>(),
                                           gn["speed_scale"][1].as<double>(),
                                           gn["speed_scale"][2].as<double>());
      }
      if (gn["stop_enter_thresh"]) {
        gait.stop_enter = gn["stop_enter_thresh"].as<double>();
      }
      if (gn["stop_exit_thresh"]) {
        gait.stop_exit = gn["stop_exit_thresh"].as<double>();
      }
      if (gn["action_deadband"]) {
        gait.deadband = gn["action_deadband"].as<double>();
      }
      if (gn["max_leg_step_rad"]) {
        gait.max_step_rad = gn["max_leg_step_rad"].as<double>();
      }
      if (gn["blind_zone_dead_reckoning"]) {
        gait.blind_dr = gn["blind_zone_dead_reckoning"].as<bool>();
      }
      if (gn["blind_zone_max_dist_m"]) {
        gait.blind_max_m = gn["blind_zone_max_dist_m"].as<double>();
      }
      if (gn["dry_run"]) gait.dry_run = gn["dry_run"].as<bool>();
    }
    return true;
  } catch (const std::exception& e) {
    if (err) *err = std::string("YAML 로드 실패: ") + e.what();
    return false;
  }
}

// 현재 실기 보정 데이터를 YAML 형식으로 저장
bool Calib::Save(const std::string& path, std::string* err) const {
  std::ofstream ofs(path);
  if (!ofs.is_open()) {
    if (err) *err = "YAML 저장 실패: " + path;
    return false;
  }

  // 헤더 및 출력 다리 순서 기록
  ofs << std::fixed << std::setprecision(6);
  ofs << "# xSting 실기 보정 통합 설정 (q_hw = direction * q_sim + offset_rad)\n";
  ofs << "output_leg_order: [";
  for (size_t i = 0; i < out_leg_order.size(); ++i) {
    ofs << out_leg_order[i] << (i + 1 < out_leg_order.size() ? ", " : "");
  }
  ofs << "]\n\n";

  // 다리 및 팔 관절 항목 기록
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

  // IMU·카메라·초음파·보행 파라미터 기록
  const Eigen::Quaterniond q = imu.mount_quat.normalized();
  ofs << "\nimu:\n"
      << "  mount_quat_xyzw: [" << q.x() << ", " << q.y() << ", " << q.z()
      << ", " << q.w() << "]\n"
      << "  gyro_bias_rad_s: [" << imu.gyro_bias.x() << ", "
      << imu.gyro_bias.y() << ", " << imu.gyro_bias.z() << "]\n";

  ofs << "\ncamera_box:\n"
      << "  cam_pos_m: [" << cam.cam_pos_m.x() << ", " << cam.cam_pos_m.y()
      << ", " << cam.cam_pos_m.z() << "]\n"
      << "  cam_rpy_deg: [" << cam.cam_rpy_deg.x() << ", "
      << cam.cam_rpy_deg.y() << ", " << cam.cam_rpy_deg.z() << "]\n"
      << "  base_z_m: " << cam.base_z_m << "\n"
      << "  robot_front_x_m: " << cam.robot_front_x_m << "\n"
      << "  target_gap_m: " << cam.target_gap_m << "\n"
      << "  box_half_m: " << cam.box_half_m << "\n"
      << "  box_height_m: " << cam.box_height_m << "\n"
      << "  puck_radius_m: " << cam.puck_radius_m << "\n"
      << "  standoff_m: " << cam.standoff_m << "\n"
      << "  pos_tol_m: " << cam.pos_tol_m << "\n"
      << "  yaw_tol_rad: " << cam.yaw_tol_rad << "\n"
      << "  hold_time_s: " << cam.hold_time_s << "\n";

  ofs << "\nultrasonic:\n"
      << "  enabled: " << (sonar.enabled ? "true" : "false") << "\n"
      << "  us_pos_m: [" << sonar.us_pos_m.x() << ", " << sonar.us_pos_m.y()
      << ", " << sonar.us_pos_m.z() << "]\n"
      << "  min_range_m: " << sonar.min_range_m << "\n"
      << "  max_range_m: " << sonar.max_range_m << "\n"
      << "  range_bias_m: " << sonar.range_bias_m << "\n"
      << "  close_range_fusion: " << (sonar.close_fusion ? "true" : "false")
      << "\n"
      << "  safety_brake_gap_m: " << sonar.brake_gap_m << "\n";

  ofs << "\ngait_safety:\n"
      << "  walk_scale: " << gait.walk_scale << "\n"
      << "  gait_freq_hz: " << gait.gait_freq_hz << "\n"
      << "  speed_limits: [" << gait.speed_lim.x() << ", " << gait.speed_lim.y()
      << ", " << gait.speed_lim.z() << "]\n"
      << "  speed_scale: [" << gait.speed_scale.x() << ", "
      << gait.speed_scale.y() << ", " << gait.speed_scale.z() << "]\n"
      << "  stop_enter_thresh: " << gait.stop_enter << "\n"
      << "  stop_exit_thresh: " << gait.stop_exit << "\n"
      << "  action_deadband: " << gait.deadband << "\n"
      << "  max_leg_step_rad: " << gait.max_step_rad << "\n"
      << "  blind_zone_dead_reckoning: " << (gait.blind_dr ? "true" : "false")
      << "\n"
      << "  blind_zone_max_dist_m: " << gait.blind_max_m << "\n"
      << "  dry_run: " << (gait.dry_run ? "true" : "false") << "\n";
  return static_cast<bool>(ofs);
}

// 평지 기립 1자세 측정값으로 IMU Roll/Pitch 장착 회전 및 자이로 영점 산출
ImuCalib SolveImu1Pose(const Eigen::Vector3d& gyro,
                       const Eigen::Vector3d& grav_imu,
                       double yaw_deg) {
  ImuCalib out;
  out.gyro_bias = gyro;
  const Eigen::Quaterniond q_yaw(
      Eigen::AngleAxisd(yaw_deg * kDegToRad, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d g_rot = (q_yaw * grav_imu).normalized();
  const Eigen::Quaterniond q_rp = Eigen::Quaterniond::FromTwoVectors(
      g_rot, Eigen::Vector3d(0.0, 0.0, -1.0));
  out.mount_quat = (q_rp * q_yaw).normalized();
  return out;
}

// 수평 자세와 전방 숙임 2자세로 IMU 3축 장착 회전 행렬 산출
ImuCalib SolveImu2Pose(const Eigen::Vector3d& gyro,
                       const Eigen::Vector3d& grav_level,
                       const Eigen::Vector3d& grav_tilt) {
  ImuCalib out;
  out.gyro_bias = gyro;
  const Eigen::Vector3d z_b = (-grav_level).normalized();
  const Eigen::Vector3d dg = grav_tilt.normalized() - grav_level.normalized();
  Eigen::Vector3d x_b = dg - dg.dot(z_b) * z_b;
  if (x_b.norm() < 1e-4) return SolveImu1Pose(gyro, grav_level, 0.0);
  x_b.normalize();

  Eigen::Matrix3d r_mount;
  r_mount.row(0) = x_b.transpose();
  r_mount.row(1) = z_b.cross(x_b).normalized().transpose();
  r_mount.row(2) = z_b.transpose();
  out.mount_quat = Eigen::Quaterniond(r_mount).normalized();
  return out;
}

// 바닥 평면 점군 PCA로 카메라 Roll/Pitch와 차체 지상고(base_z) 산출
bool SolveFloor(const std::vector<Eigen::Vector3f>& pts_opt,
                CamCalib* cam,
                double* rmse_m) {
  if (!cam || pts_opt.size() < 20) return false;

  Eigen::Matrix3d r_opt;
  r_opt << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
  const Eigen::Matrix3d r_yaw =
      Eigen::AngleAxisd(cam->cam_rpy_deg.z() * kDegToRad,
                        Eigen::Vector3d::UnitZ())
          .toRotationMatrix();

  // 광학 점군을 현재 Yaw 기준 카메라 좌표계로 변환 후 공분산 계산
  Eigen::Vector3d mu = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> pts_cam;
  pts_cam.reserve(pts_opt.size());
  for (const auto& p : pts_opt) {
    const Eigen::Vector3d pc = r_yaw * (r_opt * p.cast<double>());
    pts_cam.push_back(pc);
    mu += pc;
  }
  mu /= static_cast<double>(pts_cam.size());

  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto& pc : pts_cam) {
    const Eigen::Vector3d d = pc - mu;
    cov += d * d.transpose();
  }
  cov /= static_cast<double>(pts_cam.size());

  // 최소 고유벡터(바닥 법선)를 +Z로 정렬해 Roll/Pitch 갱신
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
  if (eig.info() != Eigen::Success) return false;
  Eigen::Vector3d norm = eig.eigenvectors().col(0).normalized();
  if (norm.z() < 0.0) norm = -norm;

  const Eigen::Matrix3d r_tot =
      Eigen::Quaterniond::FromTwoVectors(norm, Eigen::Vector3d::UnitZ())
          .toRotationMatrix() *
      r_yaw;
  const double pitch = std::asin(std::clamp(-r_tot(2, 0), -1.0, 1.0));
  const double roll = std::atan2(r_tot(2, 1), r_tot(2, 2));
  const double yaw = std::atan2(r_tot(1, 0), r_tot(0, 0));
  cam->cam_rpy_deg =
      Eigen::Vector3d(roll * kRadToDeg, pitch * kRadToDeg, yaw * kRadToDeg);

  // 보정된 회전으로 바닥 평균 Z를 구해 base_z_m 및 평면 오차 산출
  const Eigen::Matrix3f r_base = cam->OptToBaseRot();
  double sum_z = 0.0;
  for (const auto& p : pts_opt) {
    sum_z += ((r_base * p).cast<double>() + cam->cam_pos_m).z();
  }
  cam->base_z_m = -sum_z / static_cast<double>(pts_opt.size());

  if (rmse_m) {
    double sq_sum = 0.0;
    for (const auto& p : pts_opt) {
      const double z_gnd =
          ((r_base * p).cast<double>() + cam->cam_pos_m).z() + cam->base_z_m;
      sq_sum += z_gnd * z_gnd;
    }
    *rmse_m = std::sqrt(sq_sum / static_cast<double>(pts_opt.size()));
  }
  return true;
}

// 기준 거리 상자 전면 벽면 점군으로 카메라 Yaw 및 XY 장착 위치 보정
bool SolveBox(const std::vector<Eigen::Vector3f>& pts_opt,
              CamCalib* cam,
              Eigen::Vector2f* box_xy,
              Eigen::Vector2f* norm_xy,
              double gap_m) {
  if (!cam || pts_opt.size() < 12) return false;

  auto get_wall = [&](const CamCalib& c) {
    const Eigen::Matrix3f r = c.OptToBaseRot();
    const Eigen::Vector3f t = c.cam_pos_m.cast<float>();
    std::vector<Eigen::Vector3f> wall;
    wall.reserve(pts_opt.size());
    for (const auto& po : pts_opt) {
      const Eigen::Vector3f pb = r * po + t;
      const double z_gnd = static_cast<double>(pb.z()) + c.base_z_m;
      if (z_gnd > 0.008 && z_gnd < c.box_height_m - 0.008) wall.push_back(pb);
    }
    return wall;
  };

  // 1차 PCA 법선 각도로 카메라 Yaw 오차 제거
  const auto wall1 = get_wall(*cam);
  if (wall1.size() < 8) return false;
  const BoxPca pca1 = FitBoxPca(wall1, static_cast<float>(cam->box_half_m));
  if (!pca1.valid) return false;
  cam->cam_rpy_deg.z() -=
      std::atan2(pca1.norm_xy.y(), pca1.norm_xy.x()) * kRadToDeg;

  // 2차 PCA 중심 좌표로 카메라 X/Y 위치 및 목표 standoff 확정
  const auto wall2 = get_wall(*cam);
  if (wall2.size() < 8) return false;
  const BoxPca pca2 = FitBoxPca(wall2, static_cast<float>(cam->box_half_m));
  if (!pca2.valid) return false;

  const double used_gap = (gap_m >= 0.0) ? gap_m : cam->target_gap_m;
  const double true_x = cam->robot_front_x_m + used_gap + cam->box_half_m;
  cam->cam_pos_m.x() += true_x - static_cast<double>(pca2.box_xy.x());
  cam->cam_pos_m.y() -= static_cast<double>(pca2.box_xy.y());
  cam->standoff_m = cam->robot_front_x_m + cam->target_gap_m + cam->box_half_m;

  const BoxPca pca3 =
      FitBoxPca(get_wall(*cam), static_cast<float>(cam->box_half_m));
  if (box_xy) *box_xy = pca3.box_xy;
  if (norm_xy) *norm_xy = pca3.norm_xy;
  return pca3.valid;
}

// 상하위 10% 절사 평균으로 초음파 거리 바이어스 산출
bool SolveSonar(const std::vector<double>& ranges_m,
                const CamCalib& cam,
                SonarCalib* sonar,
                double gap_m,
                double* corr_m) {
  if (!sonar) return false;
  std::vector<double> valid;
  for (double r : ranges_m) {
    if (std::isfinite(r) && r >= 0.015 && r < 3.5) valid.push_back(r);
  }
  if (valid.size() < 5) return false;

  std::sort(valid.begin(), valid.end());
  const size_t trim = valid.size() / 10;
  double sum = 0.0;
  size_t cnt = 0;
  for (size_t i = trim; i + trim < valid.size(); ++i) {
    sum += valid[i];
    ++cnt;
  }
  if (cnt == 0) return false;

  const double mean_r = sum / static_cast<double>(cnt);
  const double used_gap = (gap_m >= 0.0) ? gap_m : cam.target_gap_m;
  const double exp_r = cam.robot_front_x_m + used_gap - sonar->us_pos_m.x();
  sonar->range_bias_m = exp_r - mean_r;
  if (corr_m) *corr_m = mean_r + sonar->range_bias_m;
  return true;
}

}  // namespace xsting_approach
