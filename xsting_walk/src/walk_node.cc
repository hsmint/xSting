#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "xsting_walk/walk_ctrl.h"

namespace xsting_walk {

namespace {

constexpr std::array<float, 6> kTripodOffsets = {0.0f, 0.5f, 0.0f,
                                                 0.5f, 0.0f, 0.5f};

std::string ResolvePath(const std::string& path) {
  if (path.empty()) return path;
  const std::filesystem::path p(path);
  if (p.is_absolute() || std::filesystem::exists(p)) return p.string();
  try {
    const std::filesystem::path share =
        ament_index_cpp::get_package_share_path("xsting_walk");
    if (std::filesystem::exists(share / p)) return (share / p).string();
  } catch (const std::exception&) {
  }
  return path;
}

float ApplyDeadzone(float v, float dz) {
  if (!std::isfinite(v) || std::abs(v) < dz) return 0.0f;
  const float sign = v >= 0.0f ? 1.0f : -1.0f;
  return sign * (std::abs(v) - dz) / std::max(1e-3f, 1.0f - dz);
}

std::string Fmt(const char* fmt, double v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), fmt, v);
  return buf;
}

std::string Fmt3(double x, double y, double z, const char* fmt = "%.4f") {
  char b1[32], b2[32], b3[32];
  std::snprintf(b1, sizeof(b1), fmt, x);
  std::snprintf(b2, sizeof(b2), fmt, y);
  std::snprintf(b3, sizeof(b3), fmt, z);
  return std::string("[") + b1 + ", " + b2 + ", " + b3 + "]";
}

diagnostic_msgs::msg::KeyValue MakeKv(const std::string& k,
                                      const std::string& v) {
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = k;
  kv.value = v;
  return kv;
}

geometry_msgs::msg::Point MakePt(double x, double y, double z) {
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

std::vector<std::string> Split(const std::string& s, char delim) {
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) out.push_back(item);
  return out;
}

void BackupIfExists(const std::string& path) {
  std::error_code ec;
  if (std::filesystem::exists(path, ec)) {
    std::filesystem::copy_file(
        path, path + ".bak",
        std::filesystem::copy_options::overwrite_existing, ec);
  }
}

}  // namespace

// xSting 74D 전방향 카펫 보행 텔레옵·하드웨어·Foxglove 실측 보정 통합 제어 노드
class WalkNode : public rclcpp::Node {
 public:
  WalkNode() : Node("walk_node") {
    const std::string model_bin = ResolvePath(declare_parameter<std::string>(
        "walk_model", "models/walk_policy_74d.bin"));
    calib_path_ = ResolvePath(
        declare_parameter<std::string>("calib_file", "config/calibration.yaml"));

    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.50);
    mode_hold_s_ = declare_parameter<double>("mode_hold_sec", 5.0);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.25);
    hold_arm_walking_ = declare_parameter<bool>("hold_arm_walking", true);
    stand_when_idle_ = declare_parameter<bool>("stand_when_idle", true);
    dry_run_ = declare_parameter<bool>("dry_run", false);
    const bool auto_enable = declare_parameter<bool>("auto_enable", true);

    // 게임패드(/joy) 축·버튼 매핑 파라미터
    use_joy_ = declare_parameter<bool>("use_joy", true);
    joy_axis_vx_ = static_cast<int>(declare_parameter<int>("joy_axis_vx", 1));
    joy_axis_vy_ = static_cast<int>(declare_parameter<int>("joy_axis_vy", 0));
    joy_axis_wz_ = static_cast<int>(declare_parameter<int>("joy_axis_wz", 3));
    joy_btn_enable_ =
        static_cast<int>(declare_parameter<int>("joy_enable_button", -1));
    joy_btn_turbo_ =
        static_cast<int>(declare_parameter<int>("joy_turbo_button", 5));
    joy_btn_stop_ =
        static_cast<int>(declare_parameter<int>("joy_stop_button", 1));
    joy_normal_ratio_ =
        static_cast<float>(declare_parameter<double>("joy_normal_ratio", 0.65));
    joy_deadzone_ =
        static_cast<float>(declare_parameter<double>("joy_deadzone", 0.08));

    Calib calib = Calib::MakeDefault();
    std::string err;
    if (std::filesystem::exists(calib_path_) && !calib.Load(calib_path_, &err)) {
      RCLCPP_WARN(get_logger(), "%s", err.c_str());
    }
    if (!ctrl_.Init(model_bin, calib, &err)) {
      RCLCPP_FATAL(get_logger(), "보행 정책 초기화 실패: %s", err.c_str());
      throw std::runtime_error(err);
    }
    ctrl_.SetEnabled(auto_enable);

    // 입출력 토픽 선언
    const auto cmd_topic =
        declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto cmd_stamped_topic = declare_parameter<std::string>(
        "cmd_vel_stamped_topic", "/cmd_vel_stamped");
    const auto mode_topic =
        declare_parameter<std::string>("mode_topic", "/walk/mode");
    const auto joy_topic = declare_parameter<std::string>("joy_topic", "/joy");
    const auto leg_topic = declare_parameter<std::string>(
        "leg_cmd_topic", "/leg_controller/commands");
    const auto arm_topic = declare_parameter<std::string>(
        "arm_cmd_topic", "/arm_controller/commands");
    const auto joint_topic =
        declare_parameter<std::string>("joint_topic", "/joint_states");
    const auto imu_topic =
        declare_parameter<std::string>("imu_topic", "/imu_broadcaster/imu");
    const auto odom_topic = declare_parameter<std::string>(
        "odom_topic", "/visual_slam/tracking/odometry");

    leg_pub_ =
        create_publisher<std_msgs::msg::Float64MultiArray>(leg_topic, 10);
    if (!arm_topic.empty()) {
      arm_pub_ =
          create_publisher<std_msgs::msg::Float64MultiArray>(arm_topic, 10);
    }
    status_pub_ =
        create_publisher<std_msgs::msg::Float32MultiArray>("/walk/status", 10);

    // Foxglove 실시간 플롯·3D 마커·실측 보정 상태 퍼블리셔
    cmd_user_pub_ =
        create_publisher<geometry_msgs::msg::TwistStamped>("/walk/cmd_user", 10);
    cmd_cal_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/walk/cmd_calibrated", 10);
    vel_meas_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/walk/vel_measured", 10);
    imu_cal_pub_ =
        create_publisher<sensor_msgs::msg::Imu>("/walk/imu_calibrated", 10);
    calib_info_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
        "/walk/calib_info", 10);
    calib_trial_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
        "/walk/calib_trial", 10);
    vis_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("/walk/vis", 10);
    calib_vis_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/walk/calib_vis", 10);

    // 초기 대기 상태 캐시 구성
    cached_trial_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    cached_trial_.name = "xsting_walk/calib_trial";
    cached_trial_.message = "실측 보정 대기 중 (마법사 또는 calib 노드 실행 시 실시간 갱신)";
    cached_trial_.hardware_id = "walk_node_cache";
    cached_trial_.values = {
        MakeKv("상태", "대기 중"),
        MakeKv("실행 방법", "./src/xsting_walk/scripts/calib_walk_wizard.sh"),
        MakeKv("Foxglove 명령", "/walk/calib_cmd 토픽으로 즉시 수정 가능"),
    };

    // 텔레옵 및 Foxglove 보정 명령 구독
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_topic, 10, [this](const geometry_msgs::msg::Twist::SharedPtr m) {
          OnTwist(*m);
        });
    if (!cmd_stamped_topic.empty()) {
      cmd_stamped_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
          cmd_stamped_topic, 10,
          [this](const geometry_msgs::msg::TwistStamped::SharedPtr m) {
            OnTwist(m->twist);
          });
    }
    if (!mode_topic.empty()) {
      mode_sub_ = create_subscription<std_msgs::msg::String>(
          mode_topic, 10,
          [this](const std_msgs::msg::String::SharedPtr m) { OnMode(m->data); });
    }
    calib_cmd_sub_ = create_subscription<std_msgs::msg::String>(
        "/walk/calib_cmd", 10,
        [this](const std_msgs::msg::String::SharedPtr m) {
          OnCalibCmd(m->data);
        });

    // 일회성 보정 노드(calib_gait/imu/joints) 종료 후에도 Foxglove에 마지막 결과 유지
    calib_trial_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
        "/walk/calib_trial", 10,
        [this](const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr m) {
          if (m->hardware_id == "walk_node_cache") return;
          std::lock_guard<std::mutex> lk(mtx_);
          cached_trial_ = *m;
          cached_trial_.hardware_id = "walk_node_cache";
          ext_trial_stamp_ = now();
        });
    calib_vis_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/walk/calib_vis_event", 10,
        [this](const visualization_msgs::msg::MarkerArray::SharedPtr m) {
          std::lock_guard<std::mutex> lk(mtx_);
          cached_calib_vis_ = *m;
          has_cached_calib_vis_ = !m->markers.empty();
          ext_vis_stamp_ = now();
        });

    if (use_joy_ && !joy_topic.empty()) {
      joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
          joy_topic, rclcpp::SensorDataQoS(),
          [this](const sensor_msgs::msg::Joy::SharedPtr m) { OnJoy(*m); });
    }

    // 하드웨어 센서 피드백 구독
    const auto qos = rclcpp::SensorDataQoS();
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_topic, qos,
        [this](const sensor_msgs::msg::JointState::SharedPtr m) {
          std::lock_guard<std::mutex> lk(mtx_);
          ctrl_.UpdateJoints(m->name, m->position, m->velocity);
          seen_joints_ = true;
        });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos, [this](const sensor_msgs::msg::Imu::SharedPtr m) {
          const Eigen::Vector3d gyro(m->angular_velocity.x,
                                     m->angular_velocity.y,
                                     m->angular_velocity.z);
          const Eigen::Vector3d acc(m->linear_acceleration.x,
                                    m->linear_acceleration.y,
                                    m->linear_acceleration.z);
          const bool has_acc = m->linear_acceleration_covariance[0] >= 0.0 &&
                               acc.squaredNorm() > 1e-4;
          const Eigen::Quaterniond quat(m->orientation.w, m->orientation.x,
                                        m->orientation.y, m->orientation.z);
          std::lock_guard<std::mutex> lk(mtx_);
          ctrl_.UpdateImu(gyro, acc, has_acc, quat);
          seen_imu_ = true;
        });
    if (!odom_topic.empty()) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          odom_topic, qos, [this](const nav_msgs::msg::Odometry::SharedPtr m) {
            std::lock_guard<std::mutex> lk(mtx_);
            ctrl_.UpdateOdomVel({static_cast<float>(m->twist.twist.linear.x),
                                 static_cast<float>(m->twist.twist.linear.y)},
                                true);
            odom_stamp_ = now();
            has_odom_ = true;
          });
    }

    InitServices();

    // 50Hz 보행 정책 타이머 및 200Hz 하드웨어 출력 타이머
    policy_timer_ = create_wall_timer(std::chrono::milliseconds(20),
                                      [this] { OnPolicy50Hz(); });
    hw_timer_ = create_wall_timer(std::chrono::milliseconds(5),
                                  [this] { OnHardware200Hz(); });

    RCLCPP_INFO(get_logger(),
                "xSting 전방향 보행 노드 준비 완료 (calib=%s, dry_run=%s)",
                calib_path_.c_str(), dry_run_ ? "true" : "false");
  }

 private:
  void InitServices() {
    using Trigger = std_srvs::srv::Trigger;
    enable_srv_ = create_service<Trigger>(
        "/walk/enable",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> res) {
          std::lock_guard<std::mutex> lk(mtx_);
          ctrl_.SetEnabled(true);
          res->success = true;
          res->message = "보행 제어 활성화";
        });
    disable_srv_ = create_service<Trigger>(
        "/walk/disable",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> res) {
          std::lock_guard<std::mutex> lk(mtx_);
          ctrl_.SetEnabled(false);
          has_cmd_ = false;
          res->success = true;
          res->message = "보행 제어 일시 해제 (관절 직접 점검용)";
        });
    stop_srv_ = create_service<Trigger>(
        "/walk/stop",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> res) {
          std::lock_guard<std::mutex> lk(mtx_);
          ctrl_.StopImmediate();
          has_cmd_ = false;
          active_timeout_ = cmd_timeout_;
          res->success = true;
          res->message = "보행 즉시 정지";
        });
    reset_srv_ = create_service<Trigger>(
        "/walk/reset",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> res) {
          std::lock_guard<std::mutex> lk(mtx_);
          ctrl_.Reset();
          has_cmd_ = false;
          res->success = true;
          res->message = "보행 상태 리셋";
        });
    reload_srv_ = create_service<Trigger>(
        "/walk/reload_calib",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> res) {
          std::string msg;
          const bool ok = ReloadCalibLocked(&msg);
          res->success = ok;
          res->message = msg;
        });
  }

  bool ReloadCalibLocked(std::string* out_msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    Calib c = Calib::MakeDefault();
    std::string err;
    if (!c.Load(calib_path_, &err)) {
      if (out_msg) *out_msg = err;
      return false;
    }
    ctrl_.SetCalib(c);
    const std::string msg = "실측 보정 파일 재로드 완료: " + calib_path_;
    if (out_msg) *out_msg = msg;
    RCLCPP_INFO(get_logger(),
                "보정 재로드 완료 (scale=[%.3f, %.3f, %.3f], drift=[%+.3f, %+.3f], "
                "clearance=%.1fmm)",
                c.gait.cmd_vel_scale.x(), c.gait.cmd_vel_scale.y(),
                c.gait.cmd_vel_scale.z(), c.gait.drift_vy_per_vx,
                c.gait.drift_wz_per_vx, c.gait.swing_clearance_m * 1000.0);
    return true;
  }

  // Foxglove Publish 패널에서 문자열 명령으로 실측 보정값 즉시 수정·저장·적용
  void OnCalibCmd(const std::string& raw_cmd) {
    std::string s = raw_cmd;
    s.erase(0, s.find_first_not_of(" \t\r\n"));
    if (s.empty()) return;
    s.erase(s.find_last_not_of(" \t\r\n") + 1);

    if (s == "reload") {
      std::string msg;
      ReloadCalibLocked(&msg);
      return;
    }
    if (s == "clear_vis") {
      visualization_msgs::msg::Marker m;
      m.header.stamp = now();
      m.header.frame_id = "odom";
      m.ns = "walk_cache_echo";
      m.action = visualization_msgs::msg::Marker::DELETEALL;
      visualization_msgs::msg::MarkerArray arr;
      arr.markers.push_back(m);
      calib_vis_pub_->publish(arr);
      std::lock_guard<std::mutex> lk(mtx_);
      has_cached_calib_vis_ = false;
      cached_calib_vis_.markers.clear();
      RCLCPP_INFO(get_logger(), "Foxglove 보정 궤적 마커 초기화 완료");
      return;
    }

    std::lock_guard<std::mutex> lk(mtx_);
    Calib c = ctrl_.calib();
    std::string summary;
    bool updated = false;

    // 형식 1) apply_vx:속도:시간:실측거리[:좌우편차] / apply_wz:각속도:시간:실측각도(deg)
    if (s.rfind("apply_", 0) == 0) {
      const auto toks = Split(s, ':');
      try {
        if (toks.size() >= 4 &&
            (toks[0] == "apply_vx" || toks[0] == "apply_vy")) {
          const std::string axis = toks[0].substr(6);
          const double spd = std::stod(toks[1]);
          const double dur = std::stod(toks[2]);
          const double dist = std::stod(toks[3]);
          const bool has_lat = toks.size() >= 5;
          const double lat = has_lat ? std::stod(toks[4]) : 0.0;
          updated = UpdateLinearCalib(axis, spd, dur, dist, lat, 0.0, has_lat,
                                      false, &c.gait, &summary);
        } else if (toks.size() >= 4 && toks[0] == "apply_wz") {
          const double spd = std::stod(toks[1]);
          const double dur = std::stod(toks[2]);
          const double deg = std::stod(toks[3]);
          updated = UpdateTurnCalib(spd, dur, deg, &c.gait, &summary);
        }
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Foxglove 보정 명령 파싱 오류: %s", e.what());
        return;
      }
    } else {
      // 형식 2) key=value (예: vx_scale=1.08, swing_clearance=0.04, drift_vy=-0.02)
      const auto eq = s.find('=');
      if (eq != std::string::npos) {
        const std::string k = s.substr(0, eq);
        try {
          const double v = std::stod(s.substr(eq + 1));
          if (k == "vx_scale") {
            c.gait.cmd_vel_scale.x() = std::clamp(v, 0.25, 3.0);
            updated = true;
          } else if (k == "vy_scale") {
            c.gait.cmd_vel_scale.y() = std::clamp(v, 0.25, 3.0);
            updated = true;
          } else if (k == "wz_scale") {
            c.gait.cmd_vel_scale.z() = std::clamp(v, 0.25, 3.0);
            updated = true;
          } else if (k == "drift_vy" || k == "drift_vy_per_vx") {
            c.gait.drift_vy_per_vx = std::clamp(v, -0.5, 0.5);
            updated = true;
          } else if (k == "drift_wz" || k == "drift_wz_per_vx") {
            c.gait.drift_wz_per_vx = std::clamp(v, -2.0, 2.0);
            updated = true;
          } else if (k == "swing_clearance" || k == "swing_clearance_m") {
            c.gait.swing_clearance_m = std::clamp(v, 0.008, 0.080);
            updated = true;
          }
          if (updated) summary = k + "=" + Fmt("%.4f", v);
        } catch (const std::exception&) {
        }
      }
    }

    if (!updated) {
      RCLCPP_WARN(get_logger(), "지원하지 않는 /walk/calib_cmd 형식: '%s'",
                  s.c_str());
      return;
    }

    BackupIfExists(calib_path_);
    std::string err;
    if (!c.Save(calib_path_, &err)) {
      RCLCPP_ERROR(get_logger(), "보정 저장 실패: %s", err.c_str());
      return;
    }
    ctrl_.SetCalib(c);
    cached_trial_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    cached_trial_.name = "xsting_walk/calib_trial";
    cached_trial_.message = "Foxglove 보정 즉시 반영: " + summary;
    cached_trial_.hardware_id = "walk_node_cache";
    cached_trial_.values = {
        MakeKv("반영 내역", summary),
        MakeKv("저장 파일", calib_path_),
        MakeKv("cmd_vel_scale",
               Fmt3(c.gait.cmd_vel_scale.x(), c.gait.cmd_vel_scale.y(),
                    c.gait.cmd_vel_scale.z(), "%.3f")),
    };
    RCLCPP_INFO(get_logger(), "Foxglove 보정 반영 및 저장 완료: %s -> %s",
                summary.c_str(), calib_path_.c_str());
  }

  void OnTwist(const geometry_msgs::msg::Twist& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    ctrl_.SetTeleopCmd({static_cast<float>(msg.linear.x),
                        static_cast<float>(msg.linear.y),
                        static_cast<float>(msg.angular.z)});
    cmd_stamp_ = now();
    active_timeout_ = cmd_timeout_;
    has_cmd_ = true;
  }

  void OnMode(const std::string& name) {
    Eigen::Vector3f c;
    if (!WalkCtrl::NamedModeToCmd(name, &c)) {
      RCLCPP_WARN(get_logger(), "알 수 없는 보행 모드: '%s'", name.c_str());
      return;
    }
    std::lock_guard<std::mutex> lk(mtx_);
    ctrl_.SetTeleopCmd(c);
    cmd_stamp_ = now();
    active_timeout_ = mode_hold_s_;
    has_cmd_ = (name != "stop");
    RCLCPP_INFO(get_logger(), "보행 모드 전환: %s -> [%.2f, %.2f, %.2f]",
                name.c_str(), c.x(), c.y(), c.z());
  }

  void OnJoy(const sensor_msgs::msg::Joy& msg) {
    auto get_btn = [&](int idx) -> bool {
      return idx >= 0 && static_cast<size_t>(idx) < msg.buttons.size() &&
             msg.buttons[idx] != 0;
    };
    auto get_axis = [&](int idx) -> float {
      if (idx < 0 || static_cast<size_t>(idx) >= msg.axes.size()) return 0.0f;
      return ApplyDeadzone(msg.axes[idx], joy_deadzone_);
    };

    if (get_btn(joy_btn_stop_)) {
      std::lock_guard<std::mutex> lk(mtx_);
      ctrl_.StopImmediate();
      has_cmd_ = false;
      return;
    }
    if (joy_btn_enable_ >= 0 && !get_btn(joy_btn_enable_)) {
      if (joy_was_active_) {
        std::lock_guard<std::mutex> lk(mtx_);
        ctrl_.SetTeleopCmd(Eigen::Vector3f::Zero());
        joy_was_active_ = false;
      }
      return;
    }

    const float ratio = get_btn(joy_btn_turbo_) ? 1.0f : joy_normal_ratio_;
    const float ax_vx = get_axis(joy_axis_vx_);
    const float ax_vy = get_axis(joy_axis_vy_);
    const float ax_wz = get_axis(joy_axis_wz_);
    if (ax_vx == 0.0f && ax_vy == 0.0f && ax_wz == 0.0f) {
      if (joy_was_active_) {
        std::lock_guard<std::mutex> lk(mtx_);
        ctrl_.SetTeleopCmd(Eigen::Vector3f::Zero());
        has_cmd_ = false;
        joy_was_active_ = false;
      }
      return;
    }

    std::lock_guard<std::mutex> lk(mtx_);
    const Eigen::Vector3f max_v = ctrl_.calib().gait.max_speed.cast<float>();
    const Eigen::Vector3f cmd(ax_vx * max_v.x() * ratio,
                              ax_vy * max_v.y() * ratio,
                              ax_wz * max_v.z() * ratio);
    ctrl_.SetTeleopCmd(cmd);
    cmd_stamp_ = now();
    active_timeout_ = cmd_timeout_;
    has_cmd_ = true;
    joy_was_active_ = true;
  }

  void OnPolicy50Hz() {
    const rclcpp::Time t_now = now();
    WalkState st;
    Eigen::Vector3f w_b, a_b, g_b;
    Eigen::Matrix<float, kNumLegs, 1> q_sim, q_tgt;
    Calib cal_snap;
    bool seen_imu = false;
    bool seen_joints = false;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (has_cmd_ && active_timeout_ > 0.0 &&
          (t_now - cmd_stamp_).seconds() > active_timeout_) {
        ctrl_.SetTeleopCmd(Eigen::Vector3f::Zero());
        has_cmd_ = false;
      }
      if (has_odom_ && (t_now - odom_stamp_).seconds() > odom_timeout_) {
        ctrl_.UpdateOdomVel(Eigen::Vector2f::Zero(), false);
      }
      st = ctrl_.StepPolicy(0.02f);
      w_b = ctrl_.ang_vel_b();
      a_b = ctrl_.lin_acc_b();
      g_b = ctrl_.proj_grav_b();
      q_sim = ctrl_.q_sim();
      q_tgt = ctrl_.target_legs();
      cal_snap = ctrl_.calib();
      seen_imu = seen_imu_;
      seen_joints = seen_joints_;
    }

    std_msgs::msg::Float32MultiArray msg;
    msg.data = {
        st.enabled ? 1.0f : 0.0f,
        st.moving ? 1.0f : 0.0f,
        st.using_odom_vel ? 1.0f : 0.0f,
        st.phase,
        st.user_cmd.x(),
        st.user_cmd.y(),
        st.user_cmd.z(),
        st.eff_cmd.x(),
        st.eff_cmd.y(),
        st.eff_cmd.z(),
        st.est_vel_xy.x(),
        st.est_vel_xy.y(),
        st.imu_wz,
        st.heading_err_rad * static_cast<float>(180.0 / M_PI),
    };
    status_pub_->publish(msg);

    ++tick_cnt_;
    // 25Hz: Foxglove 실시간 Plot 토픽 발행
    if (tick_cnt_ % 2 == 0) {
      PublishFoxglovePlots(t_now, st, w_b, a_b, g_b);
    }
    // ~16.7Hz: Foxglove 3D 보행·보정 벡터 및 6족 FK 마커 발행
    if (tick_cnt_ % 3 == 0) {
      PublishFoxgloveVis(t_now, st, g_b, q_sim, q_tgt, cal_snap);
    }
    // 5Hz: 현재 적용된 실측 보정 파라미터 대시보드 발행
    if (tick_cnt_ % 10 == 0) {
      PublishCalibInfo(st, a_b, g_b, cal_snap, seen_imu, seen_joints);
    }
    // 1Hz: 일회성 보정 노드가 비활성일 때 최근 시험 결과 캐시 유지
    if (tick_cnt_ % 50 == 0) {
      PublishCachedTrial(t_now);
    }
  }

  void PublishFoxglovePlots(const rclcpp::Time& stamp,
                            const WalkState& st,
                            const Eigen::Vector3f& w_b,
                            const Eigen::Vector3f& a_b,
                            const Eigen::Vector3f& g_b) {
    auto make_tw = [&](float vx, float vy, float wz) {
      geometry_msgs::msg::TwistStamped tw;
      tw.header.stamp = stamp;
      tw.header.frame_id = "base";
      tw.twist.linear.x = vx;
      tw.twist.linear.y = vy;
      tw.twist.angular.z = wz;
      return tw;
    };
    cmd_user_pub_->publish(
        make_tw(st.user_cmd.x(), st.user_cmd.y(), st.user_cmd.z()));
    cmd_cal_pub_->publish(
        make_tw(st.eff_cmd.x(), st.eff_cmd.y(), st.eff_cmd.z()));
    vel_meas_pub_->publish(
        make_tw(st.est_vel_xy.x(), st.est_vel_xy.y(), st.imu_wz));

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "base";
    imu.angular_velocity.x = w_b.x();
    imu.angular_velocity.y = w_b.y();
    imu.angular_velocity.z = w_b.z();
    imu.linear_acceleration.x = a_b.x();
    imu.linear_acceleration.y = a_b.y();
    imu.linear_acceleration.z = a_b.z();
    const Eigen::Quaternionf q_tilt = Eigen::Quaternionf::FromTwoVectors(
        g_b.normalized(), Eigen::Vector3f(0.0f, 0.0f, -1.0f));
    imu.orientation.w = q_tilt.w();
    imu.orientation.x = q_tilt.x();
    imu.orientation.y = q_tilt.y();
    imu.orientation.z = q_tilt.z();
    imu_cal_pub_->publish(imu);
  }

  void PublishFoxgloveVis(const rclcpp::Time& stamp,
                          const WalkState& st,
                          const Eigen::Vector3f& g_b,
                          const Eigen::Matrix<float, kNumLegs, 1>& q_sim,
                          const Eigen::Matrix<float, kNumLegs, 1>& q_tgt,
                          const Calib& cal) {
    using Marker = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray arr;
    constexpr double kVelScale = 1.2;  // 0.25 m/s -> 0.30 m 화살표 길이

    auto make_arrow = [&](int id, const std::string& ns, double dx, double dy,
                          double dz, double z0, float r, float g, float b,
                          float a, double shaft_d = 0.014) {
      Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = "base";
      m.ns = ns;
      m.id = id;
      m.type = Marker::ARROW;
      m.action = Marker::ADD;
      m.pose.orientation.w = 1.0;
      const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
      m.points.push_back(MakePt(0.0, 0.0, z0));
      if (len > 1e-3) {
        m.points.push_back(MakePt(dx, dy, z0 + dz));
      } else {
        m.points.push_back(MakePt(0.001, 0.0, z0));
        a = 0.15f;
      }
      m.scale.x = shaft_d;
      m.scale.y = shaft_d * 1.9;
      m.scale.z = 0.032;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = a;
      return m;
    };

    // 1) 목표 속도(청록), 보정 후 명령 속도(주황), 실측/추정 속도(초록) 화살표
    arr.markers.push_back(make_arrow(
        0, "walk_vel_cmd_user", st.user_cmd.x() * kVelScale,
        st.user_cmd.y() * kVelScale, 0.0, 0.12, 0.0f, 0.90f, 1.0f, 0.90f));
    arr.markers.push_back(make_arrow(
        1, "walk_vel_cmd_calib", st.eff_cmd.x() * kVelScale,
        st.eff_cmd.y() * kVelScale, 0.0, 0.10, 1.0f, 0.60f, 0.0f, 0.90f, 0.011));
    arr.markers.push_back(make_arrow(
        2, "walk_vel_measured", st.est_vel_xy.x() * kVelScale,
        st.est_vel_xy.y() * kVelScale, 0.0, 0.08, 0.10f, 0.95f, 0.35f, 0.95f));

    // 2) 보정된 IMU 중력 방향 화살표 (수평 정지 시 정확히 -Z 수직 하향)
    arr.markers.push_back(
        make_arrow(3, "walk_imu_gravity", g_b.x() * 0.18, g_b.y() * 0.18,
                   g_b.z() * 0.18, 0.05, 0.35f, 0.75f, 1.0f, 0.85f, 0.010));

    // 3) 6족 현재 FK 발끝 위치(초록=지지/Stance, 노랑=스윙/Swing) 및 목표 발끝(보라)
    for (int leg = 0; leg < 6; ++leg) {
      const Eigen::Vector3f p_cur =
          WalkCtrl::LegFk(leg, q_sim.segment<3>(3 * leg));
      const Eigen::Vector3f p_tgt =
          WalkCtrl::LegFk(leg, q_tgt.segment<3>(3 * leg));
      const float lp = std::fmod(st.phase + kTripodOffsets[leg], 1.0f);
      const bool swing = st.moving && (lp < 0.5f);

      Marker m_cur;
      m_cur.header.stamp = stamp;
      m_cur.header.frame_id = "base";
      m_cur.ns = "walk_foot_fk";
      m_cur.id = 10 + leg;
      m_cur.type = Marker::SPHERE;
      m_cur.action = Marker::ADD;
      m_cur.pose.position = MakePt(p_cur.x(), p_cur.y(), p_cur.z());
      m_cur.pose.orientation.w = 1.0;
      m_cur.scale.x = 0.026;
      m_cur.scale.y = 0.026;
      m_cur.scale.z = 0.026;
      m_cur.color.r = swing ? 1.0f : 0.15f;
      m_cur.color.g = swing ? 0.85f : 0.95f;
      m_cur.color.b = swing ? 0.10f : 0.35f;
      m_cur.color.a = 0.92f;
      arr.markers.push_back(m_cur);

      Marker m_tgt = m_cur;
      m_tgt.ns = "walk_foot_target";
      m_tgt.id = 20 + leg;
      m_tgt.pose.position = MakePt(p_tgt.x(), p_tgt.y(), p_tgt.z());
      m_tgt.scale.x = 0.018;
      m_tgt.scale.y = 0.018;
      m_tgt.scale.z = 0.018;
      m_tgt.color.r = 0.85f;
      m_tgt.color.g = 0.35f;
      m_tgt.color.b = 1.0f;
      m_tgt.color.a = 0.65f;
      arr.markers.push_back(m_tgt);
    }

    // 4) 상단 실시간 보정 요약 텍스트 배너
    const double roll_deg =
        std::atan2(-g_b.y(), -g_b.z()) * (180.0 / M_PI);
    const double pitch_deg =
        std::atan2(g_b.x(), std::hypot(g_b.y(), g_b.z())) * (180.0 / M_PI);

    Marker hud;
    hud.header.stamp = stamp;
    hud.header.frame_id = "base";
    hud.ns = "walk_calib_hud";
    hud.id = 30;
    hud.type = Marker::TEXT_VIEW_FACING;
    hud.action = Marker::ADD;
    hud.pose.position = MakePt(0.0, 0.0, 0.28);
    hud.pose.orientation.w = 1.0;
    hud.scale.z = 0.028;
    hud.color.r = 0.95f;
    hud.color.g = 0.98f;
    hud.color.b = 1.0f;
    hud.color.a = 0.95f;

    char buf[320];
    std::snprintf(
        buf, sizeof(buf),
        "[%s | %s] 배율=[%.2f, %.2f, %.2f] 편향=[%+.3f, %+.3f] 발높이=%.0fmm\n"
        "목표=(%+.2f, %+.2f, %+.2f) | 실측=(%+.2f, %+.2f, %+.2f) | 수평=(R%+.1f°, P%+.1f°)",
        st.moving ? "보행중" : "기립대기",
        st.using_odom_vel ? "cuVSLAM" : "6족기구학",
        cal.gait.cmd_vel_scale.x(), cal.gait.cmd_vel_scale.y(),
        cal.gait.cmd_vel_scale.z(), cal.gait.drift_vy_per_vx,
        cal.gait.drift_wz_per_vx, cal.gait.swing_clearance_m * 1000.0,
        st.user_cmd.x(), st.user_cmd.y(), st.user_cmd.z(), st.est_vel_xy.x(),
        st.est_vel_xy.y(), st.imu_wz, roll_deg, pitch_deg);
    hud.text = buf;
    arr.markers.push_back(hud);

    vis_pub_->publish(arr);
  }

  void PublishCalibInfo(const WalkState& st,
                        const Eigen::Vector3f& a_b,
                        const Eigen::Vector3f& g_b,
                        const Calib& cal,
                        bool seen_imu,
                        bool seen_joints) {
    diagnostic_msgs::msg::DiagnosticStatus ds;
    ds.level = (seen_joints && seen_imu)
                   ? diagnostic_msgs::msg::DiagnosticStatus::OK
                   : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    ds.name = "xsting_walk/calib_info";
    ds.hardware_id = "xsting_mk2";
    ds.message = st.moving ? "보행 중 (실측 보정 활성)" : "기립 대기 (실측 보정 활성)";

    const double roll_deg =
        std::atan2(-g_b.y(), -g_b.z()) * (180.0 / M_PI);
    const double pitch_deg =
        std::atan2(g_b.x(), std::hypot(g_b.y(), g_b.z())) * (180.0 / M_PI);

    std::string flipped;
    double max_off_deg = 0.0;
    for (const auto& j : cal.legs) {
      if (j.dir < 0.0) {
        if (!flipped.empty()) flipped += ", ";
        flipped += j.name;
      }
      max_off_deg =
          std::max(max_off_deg, std::abs(j.offset_rad) * (180.0 / M_PI));
    }
    if (flipped.empty()) flipped = "없음 (전 관절 +1.0)";

    ds.values = {
        MakeKv("01_보정파일", calib_path_),
        MakeKv("02_센서수신", std::string("관절=") +
                                  (seen_joints ? "OK" : "대기") + ", IMU=" +
                                  (seen_imu ? "OK" : "대기") + ", 속도원=" +
                                  (st.using_odom_vel ? "cuVSLAM" : "6족기구학")),
        MakeKv("03_속도배율_vx_vy_wz",
               Fmt3(cal.gait.cmd_vel_scale.x(), cal.gait.cmd_vel_scale.y(),
                    cal.gait.cmd_vel_scale.z(), "%.4f")),
        MakeKv("04_직진좌우편향_drift_vy_per_vx",
               Fmt("%+.4f", cal.gait.drift_vy_per_vx)),
        MakeKv("05_직진회전편향_drift_wz_per_vx",
               Fmt("%+.4f", cal.gait.drift_wz_per_vx)),
        MakeKv("06_상시트림_cmd_vel_trim",
               Fmt3(cal.gait.cmd_vel_trim.x(), cal.gait.cmd_vel_trim.y(),
                    cal.gait.cmd_vel_trim.z(), "%+.4f")),
        MakeKv("07_카펫발끝높이_swing_clearance_mm",
               Fmt("%.1f mm", cal.gait.swing_clearance_m * 1000.0)),
        MakeKv("08_보행주파수_gait_freq_hz",
               Fmt("%.2f Hz", cal.gait.gait_freq_hz)),
        MakeKv("09_보정후_차체수평_Roll_Pitch_deg",
               Fmt("%+.2f deg", roll_deg) + " / " +
                   Fmt("%+.2f deg", pitch_deg)),
        MakeKv("10_보정후_선가속도_xyz_mps2",
               Fmt3(a_b.x(), a_b.y(), a_b.z(), "%+.3f") + " (|a|=" +
                   Fmt("%.3f", a_b.norm()) + ")"),
        MakeKv("11_IMU_gyro_bias_rps",
               Fmt3(cal.imu.gyro_bias.x(), cal.imu.gyro_bias.y(),
                    cal.imu.gyro_bias.z(), "%+.5f")),
        MakeKv("12_IMU_acc_bias_mps2",
               Fmt3(cal.imu.acc_bias.x(), cal.imu.acc_bias.y(),
                    cal.imu.acc_bias.z(), "%+.4f")),
        MakeKv("13_부호반전관절", flipped),
        MakeKv("14_최대관절영점오프셋_deg", Fmt("%.2f deg", max_off_deg)),
    };
    calib_info_pub_->publish(ds);
  }

  void PublishCachedTrial(const rclcpp::Time& t_now) {
    diagnostic_msgs::msg::DiagnosticStatus trial_copy;
    visualization_msgs::msg::MarkerArray vis_copy;
    bool pub_trial = false;
    bool pub_vis = false;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if ((t_now - ext_trial_stamp_).seconds() > 0.6) {
        trial_copy = cached_trial_;
        pub_trial = true;
      }
      if (has_cached_calib_vis_ && (t_now - ext_vis_stamp_).seconds() > 1.2) {
        vis_copy = cached_calib_vis_;
        for (auto& m : vis_copy.markers) {
          m.header.stamp = t_now;
        }
        pub_vis = true;
      }
    }
    if (pub_trial) calib_trial_pub_->publish(trial_copy);
    if (pub_vis) calib_vis_pub_->publish(vis_copy);
  }

  void OnHardware200Hz() {
    std::vector<double> leg_cmds;
    std::vector<double> arm_cmds;
    bool pub_legs = false;
    bool pub_arm = false;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      ctrl_.StepHardware(0.005f, &leg_cmds, &arm_cmds);
      const bool dry = dry_run_ || ctrl_.calib().gait.dry_run;
      pub_legs = !dry && ctrl_.enabled() &&
                 (ctrl_.moving() || (stand_when_idle_ && seen_joints_));
      pub_arm = pub_legs && arm_pub_ && hold_arm_walking_ && ctrl_.moving();
    }

    if (pub_legs) {
      std_msgs::msg::Float64MultiArray l_msg;
      l_msg.data = std::move(leg_cmds);
      leg_pub_->publish(l_msg);
    }
    if (pub_arm) {
      std_msgs::msg::Float64MultiArray a_msg;
      a_msg.data = std::move(arm_cmds);
      arm_pub_->publish(a_msg);
    }
  }

  std::mutex mtx_;
  std::string calib_path_;
  double cmd_timeout_ = 0.50;
  double mode_hold_s_ = 5.0;
  double active_timeout_ = 0.50;
  double odom_timeout_ = 0.25;
  bool hold_arm_walking_ = true;
  bool stand_when_idle_ = true;
  bool dry_run_ = false;
  bool seen_joints_ = false;
  bool seen_imu_ = false;
  bool has_cmd_ = false;
  bool has_odom_ = false;
  uint64_t tick_cnt_ = 0;

  bool use_joy_ = true;
  bool joy_was_active_ = false;
  int joy_axis_vx_ = 1;
  int joy_axis_vy_ = 0;
  int joy_axis_wz_ = 3;
  int joy_btn_enable_ = -1;
  int joy_btn_turbo_ = 5;
  int joy_btn_stop_ = 1;
  float joy_normal_ratio_ = 0.65f;
  float joy_deadzone_ = 0.08f;

  WalkCtrl ctrl_;
  rclcpp::Time cmd_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time ext_trial_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time ext_vis_stamp_{0, 0, RCL_ROS_TIME};
  diagnostic_msgs::msg::DiagnosticStatus cached_trial_;
  visualization_msgs::msg::MarkerArray cached_calib_vis_;
  bool has_cached_calib_vis_ = false;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr status_pub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_user_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_cal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_meas_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_cal_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr
      calib_info_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr
      calib_trial_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      calib_vis_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      cmd_stamped_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr calib_cmd_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr
      calib_trial_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      calib_vis_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_srv_;

  rclcpp::TimerBase::SharedPtr policy_timer_;
  rclcpp::TimerBase::SharedPtr hw_timer_;
};

}  // namespace xsting_walk

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xsting_walk::WalkNode>());
  rclcpp::shutdown();
  return 0;
}
