#include "xsting_nav/nav_core.hpp"

#include <ament_index_cpp/get_package_share_path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#ifdef XSTING_NAV_HAS_NVBLOX_MSGS
#include <nvblox_msgs/msg/distance_map_slice.hpp>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace xsting_nav
{

namespace
{
std::string utc_timestamp_str()
{
  const std::time_t t = std::time(nullptr);
  std::tm tm_utc{};
  gmtime_r(&t, &tm_utc);
  std::ostringstream oss;
  oss << std::put_time(&tm_utc, "%Y-%m-%dT%H:%M:%SZ");
  return oss.str();
}

std::string resolve_calib_path(const std::string & path_or_rel)
{
  if (std::filesystem::exists(path_or_rel)) {
    return std::filesystem::absolute(path_or_rel).string();
  }
  try {
    const std::filesystem::path share = ament_index_cpp::get_package_share_path("xsting_nav");
    const std::filesystem::path cand = share / path_or_rel;
    if (std::filesystem::exists(cand)) {
      return cand.string();
    }
  } catch (const std::exception &) {
  }
  return path_or_rel;
}

float rad2deg(float r)
{
  return static_cast<float>(r * 180.0 / Mk2Kinematics::kPi);
}

float deg2rad(float d)
{
  return static_cast<float>(d * Mk2Kinematics::kPi / 180.0);
}
}  // namespace

class CalibNode : public rclcpp::Node
{
public:
  CalibNode()
  : Node("calib_node")
  {
    mode_ = declare_parameter<std::string>("mode", "status");
    calib_file_ = resolve_calib_path(
      declare_parameter<std::string>("calib_file", "config/calibration.yaml"));
    const std::string profile_override =
      declare_parameter<std::string>("hardware_profile", "");

    if (std::filesystem::exists(calib_file_)) {
      calib_ = CalibrationConfig::load_yaml(calib_file_);
    } else {
      calib_ = CalibrationConfig::make_defaults(
        profile_override.empty() ? "xsting" : profile_override);
    }
    if (!profile_override.empty() && profile_override != calib_.hardware_profile) {
      calib_ = CalibrationConfig::make_defaults(profile_override);
    }

    const bool is_mk2_hw = (calib_.hardware_profile == "mk2_hexapod");
    const std::string leg_topic = declare_parameter<std::string>(
      "leg_cmd_topic", is_mk2_hw ? "/position_controller/commands" : "/leg_controller/commands");
    const std::string imu_topic = declare_parameter<std::string>(
      "imu_topic", is_mk2_hw ? "/mk2/front/imu" : "/imu_broadcaster/imu");
    const std::string odom_topic = declare_parameter<std::string>(
      "odom_topic", "/visual_slam/tracking/odometry");
    const std::string esdf_topic = declare_parameter<std::string>(
      "esdf_topic", "/nvblox_node/static_map_slice");
    const std::string lidar_topic = declare_parameter<std::string>(
      "lidar_scan_topic", "/scan_filtered");
    const std::string sonar_topic = declare_parameter<std::string>(
      "ultrasonic_topic", "/sonar_broadcaster/range");

    duration_s_ = declare_parameter<double>("duration_s", 3.0);
    board_yaw_deg_ = declare_parameter<double>("board_yaw_deg", calib_.imu_board_yaw_deg);
    ref_pose_ = declare_parameter<std::string>("ref_pose", "zero");
    test_joint_idx_ = declare_parameter<int>("test_joint_index", 0);
    nudge_deg_ = declare_parameter<double>("nudge_deg", 6.0);
    flip_sign_ = declare_parameter<bool>("flip_sign", false);
    target_arena_x_ = declare_parameter<double>("target_arena_x", 0.734);
    target_arena_y_ = declare_parameter<double>("target_arena_y", 0.950);
    target_arena_yaw_deg_ = declare_parameter<double>("target_arena_yaw_deg", 180.0);
    gait_axis_ = declare_parameter<std::string>("gait_axis", "vx");
    gait_test_speed_ = declare_parameter<double>("gait_test_speed", 0.16);
    measured_delta_ = declare_parameter<double>("measured_delta", 0.0);
    known_front_wall_m_ = declare_parameter<double>("known_front_wall_m", 0.0);

    esdf_slice_ = EsdfSlice::make_clear_default(64, 64, 1.0f);

    leg_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(leg_topic, 10);
    const auto qos = rclcpp::SensorDataQoS();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, qos, [this](const sensor_msgs::msg::Imu::SharedPtr m) { on_imu(*m); });
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", qos, [this](const sensor_msgs::msg::JointState::SharedPtr m) { on_joints(*m); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, qos, [this](const nav_msgs::msg::Odometry::SharedPtr m) { on_odom(*m); });
    lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      lidar_topic, qos,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        latest_lidar_pts_ = project_lidar_scan_to_base_xy(
          m->ranges, m->angle_min, m->angle_increment,
          calib_, m->header.frame_id, false,
          std::max(0.12f, m->range_min), std::min(3.5f, m->range_max));
        ++lidar_count_;
      });
    sonar_sub_ = create_subscription<sensor_msgs::msg::Range>(
      sonar_topic, qos,
      [this](const sensor_msgs::msg::Range::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        latest_sonar_range_ = m->range;
        ++sonar_count_;
      });
    occ_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap", rclcpp::QoS(5),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!has_esdf_) {
          esdf_slice_ = EsdfSlice::from_occupancy_grid(
            static_cast<int>(m->info.width), static_cast<int>(m->info.height),
            static_cast<float>(m->info.origin.position.x),
            static_cast<float>(m->info.origin.position.y),
            m->info.resolution, m->data);
          ++esdf_count_;
        }
      });
#ifdef XSTING_NAV_HAS_NVBLOX_MSGS
    auto on_nvblox = [this](const nvblox_msgs::msg::DistanceMapSlice::SharedPtr m) {
        if (m->width == 0 || m->height == 0) return;
        std::lock_guard<std::mutex> lk(mtx_);
        esdf_slice_.width = static_cast<int>(m->width);
        esdf_slice_.height = static_cast<int>(m->height);
        esdf_slice_.origin_x = static_cast<float>(m->origin.x);
        esdf_slice_.origin_y = static_cast<float>(m->origin.y);
        esdf_slice_.resolution = m->resolution;
        esdf_slice_.unknown_value = m->unknown_value;
        esdf_slice_.data = m->data;
        has_esdf_ = true;
        ++esdf_count_;
      };
    nvblox_sub_ = create_subscription<nvblox_msgs::msg::DistanceMapSlice>(
      esdf_topic, qos, on_nvblox);
    if (esdf_topic != "/nvblox_node/combined_dynamic_map_slice") {
      nvblox_dyn_sub_ = create_subscription<nvblox_msgs::msg::DistanceMapSlice>(
        "/nvblox_node/combined_dynamic_map_slice", qos, on_nvblox);
    }
#endif

    start_wall_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { tick(); });

    RCLCPP_INFO(
      get_logger(),
      "[xsting_nav 보정 노드 시작] mode='%s' | profile='%s' | calib_file='%s'",
      mode_.c_str(), calib_.hardware_profile.c_str(), calib_file_.c_str());

    if (mode_ == "flip_joint_sign") {
      apply_immediate_sign_flip();
    }
  }

  bool is_done() const { return done_; }
  int exit_code() const { return exit_code_; }

private:
  void on_imu(const sensor_msgs::msg::Imu & msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    const Vec3f g{
      static_cast<float>(msg.angular_velocity.x),
      static_cast<float>(msg.angular_velocity.y),
      static_cast<float>(msg.angular_velocity.z)
    };
    const Vec4f q{
      static_cast<float>(msg.orientation.w),
      static_cast<float>(msg.orientation.x),
      static_cast<float>(msg.orientation.y),
      static_cast<float>(msg.orientation.z)
    };
    imu_gyros_.push_back(g);
    imu_quats_.push_back(q);
    latest_imu_ = transform_calibrated_imu(g, q, calib_);
  }

  void on_joints(const sensor_msgs::msg::JointState & msg)
  {
    std::unordered_map<std::string, float> jmap;
    const size_t n = std::min(msg.name.size(), msg.position.size());
    for (size_t i = 0; i < n; ++i) {
      jmap[msg.name[i]] = static_cast<float>(msg.position[i]);
    }
    const auto sim_names = Mk2Kinematics::sim_leg_joint_names();
    const auto mk2_names = Mk2Kinematics::mk2_hw_joint_names();
    Leg18f cur{};
    int found = 0;
    for (size_t i = 0; i < 18; ++i) {
      auto it = jmap.find(sim_names[i]);
      if (it == jmap.end()) it = jmap.find(mk2_names[i]);
      if (it != jmap.end() && std::isfinite(it->second)) {
        cur[i] = it->second;
        ++found;
      }
    }
    if (found == 18) {
      std::lock_guard<std::mutex> lk(mtx_);
      joint_samples_.push_back(cur);
      latest_hw_joints_ = cur;
      has_joints_ = true;
    }
  }

  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    const Vec3f pose{
      static_cast<float>(p.x),
      static_cast<float>(p.y),
      quat_to_yaw(
        static_cast<float>(q.x), static_cast<float>(q.y),
        static_cast<float>(q.z), static_cast<float>(q.w))
    };
    odom_samples_.push_back(pose);
    latest_odom_ = pose;
    has_odom_ = true;
  }

  void save_with_backup()
  {
    calib_.last_calibrated_utc = utc_timestamp_str();
    const std::filesystem::path p(calib_file_);
    if (p.has_parent_path()) {
      std::filesystem::create_directories(p.parent_path());
    }
    if (std::filesystem::exists(p)) {
      std::filesystem::copy_file(
        p, p.string() + ".bak", std::filesystem::copy_options::overwrite_existing);
    }
    calib_.save_yaml(calib_file_);
    std::cout << "\n[저장 완료] 보정 파일 업데이트: " << calib_file_ << "\n";
  }

  void apply_immediate_sign_flip()
  {
    if (test_joint_idx_ < 0 || test_joint_idx_ >= 18) {
      RCLCPP_ERROR(get_logger(), "test_joint_index must be in 0..17");
      exit_code_ = 1;
      done_ = true;
      return;
    }
    calib_.leg_sign[test_joint_idx_] *= -1.0f;
    const auto names = Mk2Kinematics::sim_leg_joint_names();
    std::cout << "[부호 반전] 관절 #" << test_joint_idx_ << " (" << names[test_joint_idx_]
              << ") -> leg_sign = " << calib_.leg_sign[test_joint_idx_] << "\n";
    save_with_backup();
    done_ = true;
  }

  void publish_legs(const Leg18f & q_hw)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(18);
    for (size_t i = 0; i < 18; ++i) {
      msg.data[i] = static_cast<double>(std::clamp(
        q_hw[i], calib_.leg_hw_min_rad[i] + 1e-3f, calib_.leg_hw_max_rad[i] - 1e-3f));
    }
    leg_pub_->publish(msg);
  }

  void tick()
  {
    if (done_) return;
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_wall_).count();

    if (mode_ == "status") {
      if (elapsed >= duration_s_) finish_status(elapsed);
    } else if (mode_ == "imu") {
      if (elapsed >= duration_s_) finish_imu_calib();
    } else if (mode_ == "joints_zero") {
      if (elapsed >= duration_s_) finish_joints_zero();
    } else if (mode_ == "joints_nudge") {
      step_joints_nudge(elapsed);
    } else if (mode_ == "stand_test") {
      step_stand_test(elapsed);
    } else if (mode_ == "odom_origin") {
      if (elapsed >= duration_s_) finish_odom_origin();
    } else if (mode_ == "gait_scale") {
      step_gait_scale(elapsed);
    } else if (mode_ == "esdf_check") {
      if (elapsed >= duration_s_) finish_esdf_check();
    } else {
      RCLCPP_ERROR(get_logger(), "알 수 없는 보정 모드: '%s'", mode_.c_str());
      exit_code_ = 1;
      done_ = true;
    }
  }

  void finish_status(double elapsed)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    const auto sim_names = Mk2Kinematics::sim_leg_joint_names();
    const Leg18f stand = Mk2Kinematics::stand_pose();

    std::cout << "\n==================== XSting 주행 센서/모터 보정 상태 점검 ====================\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "하드웨어 프로필 : " << calib_.hardware_profile
              << " (마지막 보정: " << calib_.last_calibrated_utc << ")\n";
    std::cout << "수신 샘플 수    : IMU=" << imu_gyros_.size() << " (" << (imu_gyros_.size() / elapsed) << " Hz)"
              << " | JointState=" << joint_samples_.size()
              << " | Odom=" << odom_samples_.size()
              << " | LiDAR=" << lidar_count_
              << " | Sonar=" << sonar_count_
              << " | ESDF=" << esdf_count_ << "\n";

    if (!imu_gyros_.empty()) {
      const float tilt_deg = rad2deg(std::acos(std::clamp(-latest_imu_[5], -1.0f, 1.0f)));
      std::cout << "[IMU 보정 후] 중력벡터=(" << latest_imu_[3] << ", " << latest_imu_[4] << ", "
                << latest_imu_[5] << ") | 기울기=" << tilt_deg << " deg | 각속도=("
                << latest_imu_[0] << ", " << latest_imu_[1] << ", " << latest_imu_[2] << ") rad/s\n";
    } else {
      std::cout << "[IMU 미수신] /imu_broadcaster/imu 토픽을 확인하세요.\n";
    }

    if (has_joints_) {
      const Leg18f sim_q = LegGaitController::hardware_to_sim_legs(latest_hw_joints_, calib_);
      std::cout << "[18축 다리 관절 상태 (보정 적용 후 시뮬 기준 각도)]\n";
      for (size_t leg = 0; leg < 6; ++leg) {
        std::cout << "  Leg " << (leg + 1) << ": ";
        for (size_t j = 0; j < 3; ++j) {
          const size_t idx = 3 * leg + j;
          std::cout << sim_names[idx] << "=" << std::setw(6) << rad2deg(sim_q[idx]) << "°"
                    << " (목표서기:" << std::setw(6) << rad2deg(stand[idx]) << "°, hw="
                    << std::setw(6) << latest_hw_joints_[idx] << " rad)  ";
        }
        std::cout << "\n";
      }
    } else {
      std::cout << "[JointState 미수신] /joint_states 퍼블리셔를 확인하세요.\n";
    }

    if (has_odom_) {
      const Vec3f arena = odom_to_arena_pose(latest_odom_, calib_);
      std::cout << "[오도메트리] 원시=(" << latest_odom_[0] << ", " << latest_odom_[1] << ", "
                << rad2deg(latest_odom_[2]) << "°) -> 아레나=(" << arena[0] << ", " << arena[1]
                << ", " << rad2deg(arena[2]) << "°)\n";
    } else {
      std::cout << "[오도메트리 미수신] /visual_slam/tracking/odometry 토픽을 확인하세요.\n";
    }
    std::cout << "==============================================================================\n";
    done_ = true;
  }

  void finish_imu_calib()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (imu_gyros_.size() < 10) {
      RCLCPP_ERROR(get_logger(), "IMU 샘플 부족 (%zu개). 로봇 연결 및 IMU 토픽을 확인하세요.", imu_gyros_.size());
      exit_code_ = 1;
      done_ = true;
      return;
    }

    Vec3f gyro_mean{0.0f, 0.0f, 0.0f};
    Vec3f grav_raw_mean{0.0f, 0.0f, 0.0f};
    for (size_t i = 0; i < imu_gyros_.size(); ++i) {
      for (int k = 0; k < 3; ++k) gyro_mean[k] += imu_gyros_[i][k];
      const Vec3f g = quat_to_grav(imu_quats_[i]);
      for (int k = 0; k < 3; ++k) grav_raw_mean[k] += g[k];
    }
    const float inv_n = 1.0f / static_cast<float>(imu_gyros_.size());
    for (int k = 0; k < 3; ++k) {
      gyro_mean[k] *= inv_n;
      grav_raw_mean[k] *= inv_n;
    }

    float max_gyro_dev = 0.0f;
    for (const auto & g : imu_gyros_) {
      const float dev = std::hypot(
        g[0] - gyro_mean[0], std::hypot(g[1] - gyro_mean[1], g[2] - gyro_mean[2]));
      max_gyro_dev = std::max(max_gyro_dev, dev);
    }
    if (max_gyro_dev > 0.15f) {
      RCLCPP_WARN(
        get_logger(),
        "보정 중 흔들림 감지 (max_gyro_dev=%.3f rad/s). 정지 상태에서 재실행을 권장합니다.", max_gyro_dev);
    }

    calib_.imu_board_yaw_deg = static_cast<float>(board_yaw_deg_);
    calib_.imu_gyro_bias_xyz = gyro_mean;
    const float yaw_rad = deg2rad(calib_.imu_board_yaw_deg);
    const Vec4f q_yaw{std::cos(yaw_rad * 0.5f), 0.0f, 0.0f, std::sin(yaw_rad * 0.5f)};
    const Vec3f grav_after_yaw = quat_rotate(q_yaw, grav_raw_mean);
    calib_.imu_level_quat_wxyz = solve_gravity_level_quat(grav_after_yaw);

    const Vec6f verify = transform_calibrated_imu(gyro_mean, imu_quats_.back(), calib_);
    std::cout << "\n[IMU 수평 & 자이로 바이어스 보정 완료] (샘플: " << imu_gyros_.size() << "개)\n"
              << "  - 자이로 바이어스 (rad/s): [" << gyro_mean[0] << ", " << gyro_mean[1] << ", " << gyro_mean[2] << "]\n"
              << "  - 수평 정렬 쿼터니언(wxyz): [" << calib_.imu_level_quat_wxyz[0] << ", "
              << calib_.imu_level_quat_wxyz[1] << ", " << calib_.imu_level_quat_wxyz[2] << ", "
              << calib_.imu_level_quat_wxyz[3] << "]\n"
              << "  - 검증 중력벡터 (목표 0,0,-1): [" << verify[3] << ", " << verify[4] << ", " << verify[5] << "]\n";
    save_with_backup();
    done_ = true;
  }

  void finish_joints_zero()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (joint_samples_.size() < 5) {
      RCLCPP_ERROR(get_logger(), "/joint_states 샘플 부족 (%zu개). 하드웨어 노드를 확인하세요.", joint_samples_.size());
      exit_code_ = 1;
      done_ = true;
      return;
    }

    Leg18f mean_hw{};
    for (const auto & s : joint_samples_) {
      for (size_t i = 0; i < 18; ++i) mean_hw[i] += s[i];
    }
    for (size_t i = 0; i < 18; ++i) {
      mean_hw[i] /= static_cast<float>(joint_samples_.size());
    }

    Leg18f q_ref{};
    q_ref.fill(0.0f);
    if (ref_pose_ == "stand") {
      q_ref = Mk2Kinematics::stand_pose();
    }

    const auto names = Mk2Kinematics::sim_leg_joint_names();
    std::cout << "\n[18축 다리 영점 오프셋 보정 결과] (기준 자세: " << ref_pose_
              << ", 샘플: " << joint_samples_.size() << "개)\n";
    for (size_t i = 0; i < 18; ++i) {
      const float old_off = calib_.leg_offset_rad[i];
      calib_.leg_offset_rad[i] = mean_hw[i] - calib_.leg_sign[i] * q_ref[i];
      std::cout << "  [" << std::setw(2) << i << "] " << std::setw(10) << names[i]
                << " | 측정 hw=" << std::setw(8) << mean_hw[i] << " rad"
                << " | 이전 offset=" << std::setw(8) << old_off
                << " -> 새 offset=" << std::setw(8) << calib_.leg_offset_rad[i] << " rad\n";
    }
    save_with_backup();
    done_ = true;
  }

  void step_joints_nudge(double elapsed)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!has_joints_) {
      if (elapsed > 3.0) {
        RCLCPP_ERROR(get_logger(), "/joint_states 수신 대기 시간 초과.");
        exit_code_ = 1;
        done_ = true;
      }
      return;
    }
    if (test_joint_idx_ < 0 || test_joint_idx_ >= 18) {
      RCLCPP_ERROR(get_logger(), "test_joint_index(%d) 범위 오류 (0..17).", test_joint_idx_);
      exit_code_ = 1;
      done_ = true;
      return;
    }
    if (!captured_start_pose_) {
      start_hw_pose_ = latest_hw_joints_;
      nudge_t0_ = elapsed;
      captured_start_pose_ = true;
      const auto names = Mk2Kinematics::sim_leg_joint_names();
      const int jtype = test_joint_idx_ % 3;
      const std::string expected =
        (jtype == 0) ? "발끝이 로봇 앞쪽(+X)으로 회전" :
        (jtype == 1) ? "허벅지가 위(+Z)로 들림" : "발끝(정강이)이 위(+Z)로 들림";
      std::cout << "\n[관절 방향 검증] #" << test_joint_idx_ << " (" << names[test_joint_idx_]
                << ") 에 +" << nudge_deg_ << "° 미세 구동 중...\n"
                << "  -> 정상 방향 기준: [" << expected << "]\n"
                << "  -> 반대로 움직였다면: mode:=flip_joint_sign test_joint_index:="
                << test_joint_idx_ << " 실행!\n";
    }

    const double t = elapsed - nudge_t0_;
    const float delta_sim = deg2rad(static_cast<float>(nudge_deg_));
    float env = 0.0f;
    if (t < 1.2) {
      env = static_cast<float>(0.5 * (1.0 - std::cos(Mk2Kinematics::kPi * (t / 1.2))));
    } else if (t < 2.2) {
      env = 1.0f;
    } else if (t < 3.4) {
      env = static_cast<float>(0.5 * (1.0 + std::cos(Mk2Kinematics::kPi * ((t - 2.2) / 1.2))));
    } else {
      publish_legs(start_hw_pose_);
      if (flip_sign_) {
        calib_.leg_sign[test_joint_idx_] *= -1.0f;
        save_with_backup();
      }
      std::cout << "[관절 방향 검증 완료] 원위치 복귀 완료.\n";
      done_ = true;
      return;
    }

    Leg18f cmd = start_hw_pose_;
    cmd[test_joint_idx_] += calib_.leg_sign[test_joint_idx_] * delta_sim * env;
    publish_legs(cmd);
  }

  void step_stand_test(double elapsed)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!has_joints_) {
      if (elapsed > 3.0) {
        RCLCPP_ERROR(get_logger(), "/joint_states 미수신으로 서기 보정 테스트를 중단합니다.");
        exit_code_ = 1;
        done_ = true;
      }
      return;
    }
    if (!captured_start_pose_) {
      start_hw_pose_ = latest_hw_joints_;
      nudge_t0_ = elapsed;
      captured_start_pose_ = true;
      std::cout << "\n[표준 서기 자세(STAND_POSE) 테스트] 2.5초간 부드럽게 기립합니다...\n";
    }
    const double t = elapsed - nudge_t0_;
    const Leg18f stand_hw = LegGaitController::sim_to_hardware_legs(Mk2Kinematics::stand_pose(), calib_);
    const float u = static_cast<float>(std::clamp(t / 2.5, 0.0, 1.0));
    const float s = 0.5f * (1.0f - std::cos(static_cast<float>(Mk2Kinematics::kPi) * u));
    Leg18f cmd{};
    for (size_t i = 0; i < 18; ++i) {
      cmd[i] = start_hw_pose_[i] + s * (stand_hw[i] - start_hw_pose_[i]);
    }
    publish_legs(cmd);

    if (t >= 2.5 + duration_s_) {
      std::cout << "[서기 테스트 완료] 목표 자세 유지 종료.\n";
      done_ = true;
    }
  }

  void finish_odom_origin()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (odom_samples_.size() < 5) {
      RCLCPP_ERROR(get_logger(), "cuVSLAM 오도메트리 샘플 부족 (%zu개).", odom_samples_.size());
      exit_code_ = 1;
      done_ = true;
      return;
    }
    float ox = 0.0f, oy = 0.0f, sin_y = 0.0f, cos_y = 0.0f;
    for (const auto & p : odom_samples_) {
      ox += p[0] * calib_.odom_pos_scale[0];
      oy += p[1] * calib_.odom_pos_scale[1];
      sin_y += std::sin(p[2]);
      cos_y += std::cos(p[2]);
    }
    ox /= static_cast<float>(odom_samples_.size());
    oy /= static_cast<float>(odom_samples_.size());
    const float oyaw = std::atan2(sin_y, cos_y);

    const float target_x = static_cast<float>(target_arena_x_);
    const float target_y = static_cast<float>(target_arena_y_);
    const float target_yaw = deg2rad(static_cast<float>(target_arena_yaw_deg_));

    const float syaw = wrap_to_pi(target_yaw - (oyaw + calib_.odom_yaw_offset_rad));
    const float cs = std::cos(syaw);
    const float ss = std::sin(syaw);
    const float sx = target_x - (cs * ox - ss * oy);
    const float sy = target_y - (ss * ox + cs * oy);

    calib_.init_pose_arena = {sx, sy, syaw};
    const Vec3f check = odom_to_arena_pose(odom_samples_.back(), calib_);
    std::cout << "\n[cuVSLAM 아레나 시작점(HOME) 정렬 완료]\n"
              << "  - 계산된 init_pose_arena: [" << sx << ", " << sy << ", " << syaw << " rad]\n"
              << "  - 검증 현재 아레나 좌표  : (" << check[0] << " m, " << check[1] << " m, "
              << rad2deg(check[2]) << "°)\n";
    save_with_backup();
    done_ = true;
  }

  void step_gait_scale(double elapsed)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!has_joints_) {
      if (elapsed > 3.0) {
        RCLCPP_ERROR(get_logger(), "/joint_states 미수신으로 카펫 보행 스케일 보정을 중단합니다.");
        exit_code_ = 1;
        done_ = true;
      }
      return;
    }
    if (!has_odom_ && std::abs(measured_delta_) < 1e-4) {
      if (elapsed > 3.0) {
        RCLCPP_ERROR(
          get_logger(),
          "cuVSLAM 미수신 상태입니다. 오도메트리를 켜거나 measured_delta:=실측거리(m)를 지정하세요.");
        exit_code_ = 1;
        done_ = true;
      }
      return;
    }

    if (!captured_start_pose_) {
      start_hw_pose_ = latest_hw_joints_;
      nudge_t0_ = elapsed;
      captured_start_pose_ = true;
      std::cout << "\n[카펫 보행 슬립 스케일 보정] 축=" << gait_axis_
                << ", 시험속도=" << gait_test_speed_ << ", 주행시간=" << duration_s_ << "s\n";
    }

    const double t = elapsed - nudge_t0_;
    const Leg18f stand_hw = LegGaitController::sim_to_hardware_legs(Mk2Kinematics::stand_pose(), calib_);

    if (t < 2.0) {
      const float u = static_cast<float>(t / 2.0);
      const float s = 0.5f * (1.0f - std::cos(static_cast<float>(Mk2Kinematics::kPi) * u));
      Leg18f cmd{};
      for (size_t i = 0; i < 18; ++i) {
        cmd[i] = start_hw_pose_[i] + s * (stand_hw[i] - start_hw_pose_[i]);
      }
      publish_legs(cmd);
      walk_odom_start_ = latest_odom_;
      gait_.reset();
      return;
    }

    if (t < 2.0 + duration_s_) {
      Vec3f v{0.0f, 0.0f, 0.0f};
      const float spd = static_cast<float>(gait_test_speed_);
      if (gait_axis_ == "vx") v[0] = spd;
      else if (gait_axis_ == "vy") v[1] = spd;
      else if (gait_axis_ == "wz") v[2] = spd;
      publish_legs(gait_.step_hardware(v, calib_, 0.01f));
      return;
    }

    if (t < 2.8 + duration_s_) {
      publish_legs(gait_.step_hardware({0.0f, 0.0f, 0.0f}, calib_, 0.01f));
      return;
    }

    const float cmd_total = static_cast<float>(gait_test_speed_ * duration_s_);
    float actual_delta = static_cast<float>(measured_delta_);
    float lateral_drift = 0.0f;
    if (std::abs(actual_delta) < 1e-4f && has_odom_) {
      const float dx_o = (latest_odom_[0] - walk_odom_start_[0]) * calib_.odom_pos_scale[0];
      const float dy_o = (latest_odom_[1] - walk_odom_start_[1]) * calib_.odom_pos_scale[1];
      const float c0 = std::cos(walk_odom_start_[2]);
      const float s0 = std::sin(walk_odom_start_[2]);
      const float body_dx = c0 * dx_o + s0 * dy_o;
      const float body_dy = -s0 * dx_o + c0 * dy_o;
      const float body_dyaw = wrap_to_pi(latest_odom_[2] - walk_odom_start_[2]);
      if (gait_axis_ == "vx") {
        actual_delta = body_dx;
        lateral_drift = body_dy;
      } else if (gait_axis_ == "vy") {
        actual_delta = body_dy;
      } else {
        actual_delta = body_dyaw;
      }
    }

    if (std::abs(actual_delta) < 0.02f) {
      RCLCPP_ERROR(
        get_logger(),
        "측정된 이동량이 너무 작습니다 (%.4f). 보행 여부 또는 줄자 실측값(measured_delta)을 확인하세요.",
        actual_delta);
      exit_code_ = 1;
      done_ = true;
      return;
    }

    const int axis_idx = (gait_axis_ == "vx") ? 0 : ((gait_axis_ == "vy") ? 1 : 2);
    const float ratio = cmd_total / actual_delta;
    const float old_scale = calib_.cmd_vel_scale[axis_idx];
    calib_.cmd_vel_scale[axis_idx] = std::clamp(old_scale * ratio, 0.50f, 1.80f);
    if (gait_axis_ == "vx" && duration_s_ > 0.5) {
      const float drift_rate = static_cast<float>(-lateral_drift / duration_s_);
      calib_.cmd_vel_trim[1] = std::clamp(calib_.cmd_vel_trim[1] + 0.5f * drift_rate, -0.05f, 0.05f);
    }

    std::cout << "\n[카펫 보행 스케일 보정 완료 (" << gait_axis_ << ")]\n"
              << "  - 목표 이동량: " << cmd_total << " | 실제 이동량: " << actual_delta << "\n"
              << "  - cmd_vel_scale[" << gait_axis_ << "]: " << old_scale
              << " -> " << calib_.cmd_vel_scale[axis_idx] << "\n";
    save_with_backup();
    done_ = true;
  }

  void finish_esdf_check()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (esdf_count_ == 0 && lidar_count_ == 0) {
      RCLCPP_ERROR(
        get_logger(),
        "nvBlox DistanceMapSlice / OccupancyGrid 및 /scan_filtered 토픽이 모두 수신되지 않았습니다.");
      exit_code_ = 1;
      done_ = true;
      return;
    }

    Obs41f obs = build_obs_41d(
      latest_odom_, {0.0f, 0.0f, 0.0f}, {1.93f, 0.95f, 3.14f},
      esdf_slice_, {}, {}, calib_.init_pose_arena, latest_imu_,
      {1.93f, 0.95f, 3.14f}, 1.0f, calib_.esdf_frame_mode, 0.0f);
    fuse_lidar_ultrasonic_into_obs(obs, latest_lidar_pts_, latest_sonar_range_);

    std::cout << "\n[nvBlox + 360° LiDAR + 초음파 4x4 로컬 ESDF 그리드 검증 (단위: m)]\n";
    constexpr float gx[4] = {-0.30f, 0.00f, 0.35f, 0.70f};
    std::cout << "         y=-0.45(우)  y=-0.15     y=+0.15     y=+0.45(좌)\n";
    for (int i = 3; i >= 0; --i) {
      std::cout << "x=" << std::setw(5) << std::fixed << std::setprecision(2) << gx[i] << " : ";
      for (int j = 0; j < 4; ++j) {
        std::cout << std::setw(9) << std::setprecision(3) << obs[17 + i * 4 + j] << "   ";
      }
      std::cout << "\n";
    }

    if (known_front_wall_m_ > 0.05) {
      const float expected_at_035 = static_cast<float>(known_front_wall_m_) - 0.35f;
      const float measured_at_035 = 0.5f * (obs[17 + 2 * 4 + 1] + obs[17 + 2 * 4 + 2]);
      calib_.esdf_dist_offset_m = std::clamp(expected_at_035 - measured_at_035, -0.25f, 0.25f);
      std::cout << "  - 전방 벽 기준(" << known_front_wall_m_ << "m): esdf_dist_offset_m="
                << calib_.esdf_dist_offset_m << "m\n";
      save_with_backup();
    }
    done_ = true;
  }

  std::mutex mtx_;
  std::string mode_;
  std::string calib_file_;
  CalibrationConfig calib_;
  LegGaitController gait_;

  double duration_s_ = 3.0;
  double board_yaw_deg_ = 0.0;
  std::string ref_pose_ = "zero";
  int test_joint_idx_ = 0;
  double nudge_deg_ = 6.0;
  bool flip_sign_ = false;
  double target_arena_x_ = 0.734;
  double target_arena_y_ = 0.950;
  double target_arena_yaw_deg_ = 180.0;
  std::string gait_axis_ = "vx";
  double gait_test_speed_ = 0.16;
  double measured_delta_ = 0.0;
  double known_front_wall_m_ = 0.0;

  std::chrono::steady_clock::time_point start_wall_;
  bool done_ = false;
  int exit_code_ = 0;
  bool has_joints_ = false;
  bool has_odom_ = false;
  bool has_esdf_ = false;
  bool captured_start_pose_ = false;
  double nudge_t0_ = 0.0;
  size_t esdf_count_ = 0;
  size_t lidar_count_ = 0;
  size_t sonar_count_ = 0;
  float latest_sonar_range_ = -1.0f;
  std::vector<Vec2f> latest_lidar_pts_;

  std::vector<Vec3f> imu_gyros_;
  std::vector<Vec4f> imu_quats_;
  std::vector<Leg18f> joint_samples_;
  std::vector<Vec3f> odom_samples_;

  Vec6f latest_imu_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f};
  Leg18f latest_hw_joints_{};
  Leg18f start_hw_pose_{};
  Vec3f latest_odom_{0.0f, 0.0f, 0.0f};
  Vec3f walk_odom_start_{0.0f, 0.0f, 0.0f};
  EsdfSlice esdf_slice_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_sub_;
#ifdef XSTING_NAV_HAS_NVBLOX_MSGS
  rclcpp::Subscription<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr nvblox_sub_;
  rclcpp::Subscription<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr nvblox_dyn_sub_;
#endif
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace xsting_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xsting_nav::CalibNode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  while (rclcpp::ok() && !node->is_done()) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const int rc = node->exit_code();
  rclcpp::shutdown();
  return rc;
}
