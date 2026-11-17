#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "xsting_walk/calib.h"
#include "xsting_walk/ros_param_util.h"

namespace xsting_walk {

namespace {

double WrapRad(double a) { return std::atan2(std::sin(a), std::cos(a)); }

double QuatYaw(double w, double x, double y, double z) {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

void BackupIfExists(const std::string& path) {
  std::error_code ec;
  if (std::filesystem::exists(path, ec)) {
    std::filesystem::copy_file(
        path, path + ".bak",
        std::filesystem::copy_options::overwrite_existing, ec);
  }
}

std::string Fmt(const char* fmt, double v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), fmt, v);
  return buf;
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

}  // namespace

// 실측(줄자·각도기 + SLAM/IMU 자동계측) 보행 속도 스케일·직진 편향 보정 및 Foxglove 3D 시각화 노드
class CalibGaitNode : public rclcpp::Node {
 public:
  CalibGaitNode() : Node("calib_gait_node") {
    calib_file_ = DeclareString(
        this, "calib_file", "src/xsting_walk/config/calibration.yaml");
    mode_ = DeclareString(this, "mode", "trial");
    axis_ = DeclareString(this, "axis", "vx");
    speed_ = DeclareDouble(this, "speed", 0.20);
    duration_s_ = std::max(0.5, DeclareDouble(this, "duration_s", 3.0));
    settle_s_ = std::max(0.2, DeclareDouble(this, "settle_s", 0.8));

    measured_dist_m_ = DeclareDouble(this, "measured_dist_m", 0.0);
    measured_yaw_deg_ = DeclareDouble(this, "measured_yaw_deg", 0.0);
    measured_lat_drift_m_ =
        DeclareDouble(this, "measured_lat_drift_m", 0.0);
    measured_yaw_drift_deg_ =
        DeclareDouble(this, "measured_yaw_drift_deg", 0.0);
    has_lat_drift_ = DeclareBool(this, "has_lat_drift", false);
    has_yaw_drift_ = DeclareBool(this, "has_yaw_drift", false);
    swing_clearance_m_ =
        DeclareDouble(this, "swing_clearance_m", -1.0);
    auto_apply_sensor_ = DeclareBool(this, "auto_apply_sensor", false);
    result_file_ = DeclareString(this, "result_file", "");

    const auto cmd_topic =
        DeclareString(this, "cmd_vel_topic", "/cmd_vel");
    const auto odom_topic = DeclareString(
        this, "odom_topic", "/visual_slam/tracking/odometry");
    const auto imu_topic =
        DeclareString(this, "imu_topic", "/imu_broadcaster/imu");

    calib_ = Calib::MakeDefault();
    std::string err;
    if (std::filesystem::exists(calib_file_)) calib_.Load(calib_file_, &err);

    reload_cli_ = create_client<std_srvs::srv::Trigger>("/walk/reload_calib");
    trial_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
        "/walk/calib_trial", 10);
    vis_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/walk/calib_vis", 10);
    vis_event_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/walk/calib_vis_event", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/walk/calib_path", 10);

    // 실측값 즉시 반영 모드(mode=apply 또는 mode=set_clearance)
    if (mode_ == "apply" || mode_ == "set_clearance") {
      ApplyAndExit();
      return;
    }

    // 이전 시험 궤적 초기화 마커 전송
    ClearFoxgloveMarkers();

    // 시험 주행 계측 모드(mode=trial)
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
    const auto qos = rclcpp::SensorDataQoS();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr m) { OnOdom(*m); });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos,
        [this](const sensor_msgs::msg::Imu::SharedPtr m) { OnImu(*m); });
    status_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/walk/status", 10,
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr m) {
          OnStatus(*m);
        });

    t0_ = std::chrono::steady_clock::now();
    last_tick_ = t0_;
    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               [this] { OnTrialTick(); });

    RCLCPP_INFO(get_logger(),
                "실측 보행 시험 시작: axis=%s, speed=%+.3f, duration=%.2fs",
                axis_.c_str(), speed_, duration_s_);
  }

 private:
  void ClearFoxgloveMarkers() {
    visualization_msgs::msg::Marker m;
    m.header.stamp = now();
    m.header.frame_id = "odom";
    m.action = visualization_msgs::msg::Marker::DELETEALL;
    visualization_msgs::msg::MarkerArray arr;
    arr.markers.push_back(m);
    vis_pub_->publish(arr);
    vis_event_pub_->publish(arr);
  }

  void PublishApplyStatus(const std::string& title,
                          const std::string& summary,
                          bool ok) {
    // DDS 구독자 매칭을 위한 짧은 대기 후 전송
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    diagnostic_msgs::msg::DiagnosticStatus ds;
    ds.level = ok ? diagnostic_msgs::msg::DiagnosticStatus::OK
                  : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    ds.name = "xsting_walk/calib_trial";
    ds.hardware_id = "calib_gait_node";
    ds.message = title;
    ds.values = {
        MakeKv("01_상태", title),
        MakeKv("02_보정요약", summary),
        MakeKv("03_저장파일", calib_file_),
        MakeKv("04_현재_vx_vy_wz_배율",
               Fmt("[%.4f, ", calib_.gait.cmd_vel_scale.x()) +
                   Fmt("%.4f, ", calib_.gait.cmd_vel_scale.y()) +
                   Fmt("%.4f]", calib_.gait.cmd_vel_scale.z())),
        MakeKv("05_직진편향_vy_wz",
               Fmt("%+.4f / ", calib_.gait.drift_vy_per_vx) +
                   Fmt("%+.4f", calib_.gait.drift_wz_per_vx)),
        MakeKv("06_발끝높이_mm",
               Fmt("%.1f mm", calib_.gait.swing_clearance_m * 1000.0)),
    };
    trial_pub_->publish(ds);
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
  }

  void SaveAndReload(const std::string& summary) {
    const auto parent = std::filesystem::path(calib_file_).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    BackupIfExists(calib_file_);

    std::string err;
    if (!calib_.Save(calib_file_, &err)) {
      RCLCPP_ERROR(get_logger(), "%s", err.c_str());
      PublishApplyStatus("실측 보정 저장 실패", err, false);
      return;
    }
    RCLCPP_INFO(get_logger(), "실측 보정 저장 완료: %s -> %s", summary.c_str(),
                calib_file_.c_str());

    if (reload_cli_->wait_for_service(std::chrono::milliseconds(300))) {
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto fut = reload_cli_->async_send_request(req);
      rclcpp::spin_until_future_complete(get_node_base_interface(), fut,
                                         std::chrono::milliseconds(500));
    }
    PublishApplyStatus("실측 보정 반영 및 리로드 완료", summary, true);
  }

  void ApplyAndExit() {
    std::string summary;
    if (mode_ == "set_clearance") {
      if (swing_clearance_m_ >= 0.008 && swing_clearance_m_ <= 0.080) {
        calib_.gait.swing_clearance_m = swing_clearance_m_;
        summary = "swing_clearance_m=" + Fmt("%.4f", swing_clearance_m_);
        SaveAndReload(summary);
      } else {
        RCLCPP_ERROR(get_logger(),
                     "유효 범위(0.008~0.080m) 밖의 swing_clearance_m=%.4f",
                     swing_clearance_m_);
      }
      rclcpp::shutdown();
      return;
    }

    bool ok = false;
    if (axis_ == "vx" || axis_ == "vy") {
      ok = UpdateLinearCalib(axis_, speed_, duration_s_, measured_dist_m_,
                             measured_lat_drift_m_, measured_yaw_drift_deg_,
                             has_lat_drift_, has_yaw_drift_, &calib_.gait,
                             &summary);
    } else if (axis_ == "wz") {
      ok = UpdateTurnCalib(speed_, duration_s_, measured_yaw_deg_, &calib_.gait,
                           &summary);
    }
    if (swing_clearance_m_ >= 0.008 && swing_clearance_m_ <= 0.080) {
      calib_.gait.swing_clearance_m = swing_clearance_m_;
    }

    if (ok) {
      SaveAndReload(summary);
    } else {
      RCLCPP_ERROR(get_logger(),
                   "실측값이 너무 작거나 목표 방향과 부호가 반대입니다. "
                   "보정을 건너뜁니다.");
      PublishApplyStatus("보정 건너뜀 (유효하지 않은 실측값)", axis_, false);
    }
    rclcpp::shutdown();
  }

  void OnOdom(const nav_msgs::msg::Odometry& msg) {
    const double x = msg.pose.pose.position.x;
    const double y = msg.pose.pose.position.y;
    const auto& q = msg.pose.pose.orientation;
    const double yaw = QuatYaw(q.w, q.x, q.y, q.z);

    std::lock_guard<std::mutex> lk(mtx_);
    if (!has_odom0_) {
      odom_x0_ = x;
      odom_y0_ = y;
      odom_yaw0_ = yaw;
      odom_prev_yaw_ = yaw;
      odom_accum_yaw_ = 0.0;
      has_odom0_ = true;
    } else {
      odom_accum_yaw_ += WrapRad(yaw - odom_prev_yaw_);
      odom_prev_yaw_ = yaw;
    }
    odom_x_ = x;
    odom_y_ = y;

    if (traj_pts_.empty() ||
        std::hypot(x - traj_pts_.back().x, y - traj_pts_.back().y) > 0.008) {
      traj_pts_.push_back(MakePt(x, y, 0.015));
    }
  }

  void OnImu(const sensor_msgs::msg::Imu& msg) {
    const Eigen::Vector3d gyro(msg.angular_velocity.x, msg.angular_velocity.y,
                               msg.angular_velocity.z);
    const Eigen::Vector3d acc(msg.linear_acceleration.x,
                              msg.linear_acceleration.y,
                              msg.linear_acceleration.z);
    const Eigen::Quaterniond quat(msg.orientation.w, msg.orientation.x,
                                  msg.orientation.y, msg.orientation.z);
    Eigen::Vector3f w_b;
    Eigen::Vector3f a_b;
    Eigen::Vector3f g_b;
    float q_yaw = 0.0f;
    bool got_q = false;
    calib_.imu.Transform(gyro, acc, acc.squaredNorm() > 1e-4, quat, &w_b, &a_b,
                         &g_b, &q_yaw, &got_q);

    const auto now_tp = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lk(mtx_);
    if (!has_imu0_) {
      imu_prev_tp_ = now_tp;
      imu_prev_yaw_ = q_yaw;
      has_imu_quat_ = got_q;
      has_imu0_ = true;
      return;
    }
    const double dt =
        std::clamp(std::chrono::duration<double>(now_tp - imu_prev_tp_).count(),
                   0.0, 0.05);
    imu_prev_tp_ = now_tp;
    if (got_q && has_imu_quat_) {
      imu_accum_yaw_ += WrapRad(static_cast<double>(q_yaw) - imu_prev_yaw_);
      imu_prev_yaw_ = q_yaw;
    } else {
      imu_accum_yaw_ += static_cast<double>(w_b.z()) * dt;
    }
  }

  void OnStatus(const std_msgs::msg::Float32MultiArray& msg) {
    if (msg.data.size() < 14) return;
    std::lock_guard<std::mutex> lk(mtx_);
    est_vx_ = msg.data[10];
    est_vy_ = msg.data[11];
    has_status_ = true;
  }

  // 시험 주행 진행 중 및 종료 시 Foxglove 3D 가상 줄자·궤적·계측 보드 발행
  void PublishTrialFoxglove(double elapsed, bool finished) {
    double x0 = 0.0, y0 = 0.0, yaw0 = 0.0;
    double dx_b = 0.0, dy_b = 0.0;
    double odom_dyaw_deg = 0.0, imu_dyaw_deg = 0.0;
    double k_dx = 0.0, k_dy = 0.0;
    bool got_odom = false, got_imu = false;
    std::vector<geometry_msgs::msg::Point> pts;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      got_odom = has_odom0_;
      got_imu = has_imu0_;
      k_dx = kin_dx_;
      k_dy = kin_dy_;
      if (has_odom0_) {
        x0 = odom_x0_;
        y0 = odom_y0_;
        yaw0 = odom_yaw0_;
        const double wx = odom_x_ - odom_x0_;
        const double wy = odom_y_ - odom_y0_;
        const double c = std::cos(-odom_yaw0_);
        const double s = std::sin(-odom_yaw0_);
        dx_b = c * wx - s * wy;
        dy_b = s * wx + c * wy;
        odom_dyaw_deg = odom_accum_yaw_ * (180.0 / M_PI);
        pts = traj_pts_;
      } else {
        dx_b = k_dx;
        dy_b = k_dy;
        pts = kin_pts_;
      }
      if (has_imu0_) imu_dyaw_deg = imu_accum_yaw_ * (180.0 / M_PI);
    }

    const rclcpp::Time stamp = now();
    const double total_t = duration_s_ + settle_s_;
    const double prog_pct =
        finished ? 100.0 : std::clamp(100.0 * elapsed / total_t, 0.0, 100.0);
    const double exp_lin = speed_ * duration_s_;
    const double exp_deg = (speed_ * duration_s_) * (180.0 / M_PI);
    const double best_yaw_deg = got_imu ? imu_dyaw_deg : odom_dyaw_deg;

    // 추천 스케일 배율 실시간 추정
    double rec_scale = 1.0;
    if (axis_ == "vx") {
      rec_scale = calib_.gait.cmd_vel_scale.x();
      if (std::abs(dx_b) > 0.03 && exp_lin * dx_b > 0.0) {
        rec_scale *= (exp_lin / dx_b);
      }
    } else if (axis_ == "vy") {
      rec_scale = calib_.gait.cmd_vel_scale.y();
      if (std::abs(dy_b) > 0.03 && exp_lin * dy_b > 0.0) {
        rec_scale *= (exp_lin / dy_b);
      }
    } else if (axis_ == "wz") {
      rec_scale = calib_.gait.cmd_vel_scale.z();
      if (std::abs(best_yaw_deg) > 5.0 && exp_deg * best_yaw_deg > 0.0) {
        rec_scale *= (exp_deg / best_yaw_deg);
      }
    }
    rec_scale = std::clamp(rec_scale, 0.25, 3.0);

    // 1) /walk/calib_trial 진단 메시지 발행
    diagnostic_msgs::msg::DiagnosticStatus ds;
    ds.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    ds.name = "xsting_walk/calib_trial";
    ds.hardware_id = "calib_gait_node";
    ds.message =
        finished
            ? ("시험 주행 계측 완료 [" + axis_ + "] - 줄자 실측 또는 반영 대기")
            : ("시험 주행 진행 중 [" + axis_ + "] " + Fmt("%.0f%%", prog_pct));
    ds.values = {
        MakeKv("01_시험축_속도_시간",
               axis_ + " | " + Fmt("%+.3f", speed_) + " | " +
                   Fmt("%.2fs", duration_s_)),
        MakeKv("02_진행률", Fmt("%.1f %%", prog_pct)),
        MakeKv("03_목표이동량", axis_ == "wz" ? Fmt("%+.1f deg", exp_deg)
                                              : Fmt("%+.3f m", exp_lin)),
        MakeKv("04_cuVSLAM_전후_좌우_m",
               got_odom ? (Fmt("dx=%+.3f m, ", dx_b) + Fmt("dy=%+.3f m", dy_b))
                        : "미실행 (기구학 폴백 사용)"),
        MakeKv("05_6족기구학_전후_좌우_m",
               Fmt("dx=%+.3f m, ", k_dx) + Fmt("dy=%+.3f m", k_dy)),
        MakeKv("06_회전각_IMU_SLAM_deg",
               Fmt("IMU=%+.1f deg / ", imu_dyaw_deg) +
                   Fmt("SLAM=%+.1f deg", odom_dyaw_deg)),
        MakeKv("07_센서기반_추천배율", Fmt("%.4f", rec_scale)),
        MakeKv("08_Foxglove즉시반영예시",
               axis_ == "wz"
                   ? ("apply_wz:" + Fmt("%.2f:", speed_) +
                      Fmt("%.1f:", duration_s_) + Fmt("%.1f", best_yaw_deg))
                   : ("apply_" + axis_ + ":" + Fmt("%.2f:", speed_) +
                      Fmt("%.1f:", duration_s_) +
                      Fmt("%.3f", axis_ == "vx" ? dx_b : dy_b) +
                      (axis_ == "vx" ? Fmt(":%.3f", dy_b) : ""))),
    };
    trial_pub_->publish(ds);

    // 2) /walk/calib_path 발행
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = stamp;
    path_msg.header.frame_id = "odom";
    for (const auto& pt : pts) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose.position = pt;
      ps.pose.orientation.w = 1.0;
      path_msg.poses.push_back(ps);
    }
    path_pub_->publish(path_msg);

    // 3) /walk/calib_vis: 가상 줄자 눈금(10cm 단위) + 목표선 + 실제 궤적 + 편차 벡터 + 3D 보드
    using Marker = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray arr;
    const double c0 = std::cos(yaw0);
    const double s0 = std::sin(yaw0);
    auto to_world = [&](double bx, double by, double bz = 0.01) {
      return MakePt(x0 + c0 * bx - s0 * by, y0 + s0 * bx + c0 * by, bz);
    };

    // 출발선 & 가상 눈금자 (vx / vy 직선 주행 시 10cm 간격 표시)
    Marker ruler;
    ruler.header.stamp = stamp;
    ruler.header.frame_id = "odom";
    ruler.ns = "calib_ruler";
    ruler.id = 0;
    ruler.type = Marker::LINE_LIST;
    ruler.action = Marker::ADD;
    ruler.pose.orientation.w = 1.0;
    ruler.scale.x = 0.006;
    ruler.color.r = 0.30f;
    ruler.color.g = 0.85f;
    ruler.color.b = 1.0f;
    ruler.color.a = 0.80f;

    if (axis_ == "vx" || axis_ == "vy") {
      const double sign = speed_ >= 0.0 ? 1.0 : -1.0;
      const double total_d = std::abs(exp_lin);
      // 중심 기준선
      if (axis_ == "vx") {
        ruler.points.push_back(to_world(0.0, 0.0));
        ruler.points.push_back(to_world(exp_lin, 0.0));
      } else {
        ruler.points.push_back(to_world(0.0, 0.0));
        ruler.points.push_back(to_world(0.0, exp_lin));
      }
      // 10cm 단위 눈금
      const int n_ticks =
          std::clamp(static_cast<int>(std::ceil(total_d / 0.10)), 1, 30);
      for (int i = 0; i <= n_ticks; ++i) {
        const double d = std::min(total_d, i * 0.10) * sign;
        const double hw = (i == 0 || i == n_ticks) ? 0.18 : 0.06;
        if (axis_ == "vx") {
          ruler.points.push_back(to_world(d, -hw));
          ruler.points.push_back(to_world(d, +hw));
        } else {
          ruler.points.push_back(to_world(-hw, d));
          ruler.points.push_back(to_world(+hw, d));
        }
      }
      arr.markers.push_back(ruler);

      // 목표 도달 지점 구체
      Marker goal;
      goal.header = ruler.header;
      goal.ns = "calib_goal";
      goal.id = 1;
      goal.type = Marker::SPHERE;
      goal.action = Marker::ADD;
      goal.pose.position =
          (axis_ == "vx") ? to_world(exp_lin, 0.0, 0.02)
                          : to_world(0.0, exp_lin, 0.02);
      goal.pose.orientation.w = 1.0;
      goal.scale.x = 0.036;
      goal.scale.y = 0.036;
      goal.scale.z = 0.036;
      goal.color.r = 0.0f;
      goal.color.g = 0.90f;
      goal.color.b = 1.0f;
      goal.color.a = 0.90f;
      arr.markers.push_back(goal);
    } else {
      // wz 제자리 회전: 반지름 0.25m 목표 각도 부채꼴 호 표시
      constexpr double kR = 0.25;
      ruler.points.push_back(to_world(0.0, 0.0));
      ruler.points.push_back(to_world(kR, 0.0));
      const double target_rad = speed_ * duration_s_;
      ruler.points.push_back(to_world(0.0, 0.0));
      ruler.points.push_back(
          to_world(kR * std::cos(target_rad), kR * std::sin(target_rad)));
      for (int i = 0; i < 24; ++i) {
        const double a0 = target_rad * (static_cast<double>(i) / 24.0);
        const double a1 = target_rad * (static_cast<double>(i + 1) / 24.0);
        ruler.points.push_back(
            to_world(kR * std::cos(a0), kR * std::sin(a0)));
        ruler.points.push_back(
            to_world(kR * std::cos(a1), kR * std::sin(a1)));
      }
      arr.markers.push_back(ruler);
    }

    // 실제 이동 궤적 선
    if (pts.size() >= 2) {
      Marker tr;
      tr.header = ruler.header;
      tr.ns = "calib_traj";
      tr.id = 2;
      tr.type = Marker::LINE_STRIP;
      tr.action = Marker::ADD;
      tr.pose.orientation.w = 1.0;
      tr.scale.x = 0.014;
      tr.color.r = got_odom ? 0.15f : 1.0f;
      tr.color.g = 0.95f;
      tr.color.b = got_odom ? 0.35f : 0.20f;
      tr.color.a = 0.95f;
      tr.points = pts;
      arr.markers.push_back(tr);
    }

    // 직진 시 좌우 쏠림(drift) 빨간 오차 화살표
    if (axis_ == "vx" && std::abs(dy_b) > 0.005) {
      Marker dr;
      dr.header = ruler.header;
      dr.ns = "calib_drift";
      dr.id = 3;
      dr.type = Marker::ARROW;
      dr.action = Marker::ADD;
      dr.pose.orientation.w = 1.0;
      dr.points.push_back(to_world(dx_b, 0.0, 0.02));
      dr.points.push_back(to_world(dx_b, dy_b, 0.02));
      dr.scale.x = 0.010;
      dr.scale.y = 0.020;
      dr.scale.z = 0.025;
      dr.color.r = 1.0f;
      dr.color.g = 0.30f;
      dr.color.b = 0.25f;
      dr.color.a = 0.95f;
      arr.markers.push_back(dr);
    }

    // 3D 요약 결과 텍스트 배너
    Marker txt;
    txt.header = ruler.header;
    txt.ns = "calib_summary";
    txt.id = 4;
    txt.type = Marker::TEXT_VIEW_FACING;
    txt.action = Marker::ADD;
    txt.pose.position = to_world(dx_b * 0.5, dy_b * 0.5, 0.42);
    txt.pose.orientation.w = 1.0;
    txt.scale.z = 0.032;
    txt.color.r = finished ? 0.40f : 1.0f;
    txt.color.g = 1.0f;
    txt.color.b = finished ? 0.55f : 0.45f;
    txt.color.a = 0.98f;

    char buf[320];
    if (axis_ == "wz") {
      std::snprintf(
          buf, sizeof(buf),
          "[%s | wz 회전] 목표=%+.1f° | IMU=%+.1f° | SLAM=%+.1f°\n"
          "추천 wz_scale = %.3f",
          finished ? "계측완료" : Fmt("진행 %.0f%%", prog_pct).c_str(), exp_deg,
          imu_dyaw_deg, odom_dyaw_deg, rec_scale);
    } else {
      std::snprintf(
          buf, sizeof(buf),
          "[%s | %s 주행] 목표=%+.3fm | %s=(dx=%+.3fm, dy=%+.3fm) | yaw=%+.1f°\n"
          "추천 %s_scale = %.3f",
          finished ? "계측완료" : Fmt("진행 %.0f%%", prog_pct).c_str(),
          axis_.c_str(), exp_lin, got_odom ? "SLAM" : "기구학", dx_b, dy_b,
          best_yaw_deg, axis_.c_str(), rec_scale);
    }
    txt.text = buf;
    arr.markers.push_back(txt);

    vis_pub_->publish(arr);
    vis_event_pub_->publish(arr);
  }

  void OnTrialTick() {
    const auto now_tp = std::chrono::steady_clock::now();
    const double elapsed =
        std::chrono::duration<double>(now_tp - t0_).count();
    const double dt =
        std::clamp(std::chrono::duration<double>(now_tp - last_tick_).count(),
                   0.0, 0.05);
    last_tick_ = now_tp;

    geometry_msgs::msg::Twist cmd;
    if (elapsed < duration_s_) {
      if (axis_ == "vx") cmd.linear.x = speed_;
      if (axis_ == "vy") cmd.linear.y = speed_;
      if (axis_ == "wz") cmd.angular.z = speed_;
    }
    cmd_pub_->publish(cmd);

    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (has_status_) {
        kin_dx_ += static_cast<double>(est_vx_) * dt;
        kin_dy_ += static_cast<double>(est_vy_) * dt;
        if (kin_pts_.empty() ||
            std::hypot(kin_dx_ - kin_pts_.back().x,
                       kin_dy_ - kin_pts_.back().y) > 0.008) {
          kin_pts_.push_back(MakePt(kin_dx_, kin_dy_, 0.015));
        }
      }
    }

    // 10Hz Foxglove 가상 줄자·궤적·계측 상태 갱신
    if (++tick_cnt_ % 5 == 0) {
      PublishTrialFoxglove(elapsed, false);
    }

    if (elapsed < duration_s_ + settle_s_) return;

    // 정지 명령 재전송 후 계측 요약 산출
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
    FinishTrial();
  }

  void FinishTrial() {
    timer_->cancel();
    PublishTrialFoxglove(duration_s_ + settle_s_, true);

    double odom_dx_b = 0.0;
    double odom_dy_b = 0.0;
    double odom_dyaw_deg = 0.0;
    double imu_dyaw_deg = 0.0;
    bool got_odom = false;
    bool got_imu = false;
    double k_dx = 0.0;
    double k_dy = 0.0;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      got_odom = has_odom0_;
      got_imu = has_imu0_;
      k_dx = kin_dx_;
      k_dy = kin_dy_;
      if (has_odom0_) {
        const double wx = odom_x_ - odom_x0_;
        const double wy = odom_y_ - odom_y0_;
        const double c = std::cos(-odom_yaw0_);
        const double s = std::sin(-odom_yaw0_);
        odom_dx_b = c * wx - s * wy;
        odom_dy_b = s * wx + c * wy;
        odom_dyaw_deg = odom_accum_yaw_ * (180.0 / M_PI);
      }
      if (has_imu0_) {
        imu_dyaw_deg = imu_accum_yaw_ * (180.0 / M_PI);
      }
    }

    const double exp_lin = speed_ * duration_s_;
    const double exp_deg = (speed_ * duration_s_) * (180.0 / M_PI);
    const double best_yaw_deg = got_imu ? imu_dyaw_deg : odom_dyaw_deg;

    if (axis_ == "wz") {
      RCLCPP_INFO(get_logger(),
                  "시험 완료 [wz]: 목표=%.1f deg | IMU=%.1f deg | SLAM=%.1f deg",
                  exp_deg, imu_dyaw_deg, odom_dyaw_deg);
    } else {
      RCLCPP_INFO(
          get_logger(),
          "시험 완료 [%s]: 목표=%+.3f m | SLAM=(dx=%+.3f m, dy=%+.3f m, "
          "dyaw=%+.1f deg) | 기구학=(dx=%+.3f m, dy=%+.3f m) | IMU_yaw=%+.1f deg",
          axis_.c_str(), exp_lin, odom_dx_b, odom_dy_b, odom_dyaw_deg, k_dx,
          k_dy, imu_dyaw_deg);
    }

    if (!result_file_.empty()) {
      std::ofstream ofs(result_file_);
      ofs << std::fixed << std::setprecision(6)
          << "HAS_ODOM=" << (got_odom ? 1 : 0) << "\n"
          << "HAS_IMU=" << (got_imu ? 1 : 0) << "\n"
          << "EXPECTED_M=" << exp_lin << "\n"
          << "EXPECTED_DEG=" << exp_deg << "\n"
          << "ODOM_DX_M=" << odom_dx_b << "\n"
          << "ODOM_DY_M=" << odom_dy_b << "\n"
          << "ODOM_DYAW_DEG=" << odom_dyaw_deg << "\n"
          << "IMU_DYAW_DEG=" << imu_dyaw_deg << "\n"
          << "KIN_DX_M=" << k_dx << "\n"
          << "KIN_DY_M=" << k_dy << "\n";
    }

    if (auto_apply_sensor_) {
      std::string summary;
      bool ok = false;
      if (axis_ == "vx" && got_odom) {
        ok = UpdateLinearCalib("vx", speed_, duration_s_, odom_dx_b, odom_dy_b,
                               best_yaw_deg, true, got_imu || got_odom,
                               &calib_.gait, &summary);
      } else if (axis_ == "vy" && got_odom) {
        ok = UpdateLinearCalib("vy", speed_, duration_s_, odom_dy_b, 0.0, 0.0,
                               false, false, &calib_.gait, &summary);
      } else if (axis_ == "wz" && (got_imu || got_odom)) {
        ok = UpdateTurnCalib(speed_, duration_s_, best_yaw_deg, &calib_.gait,
                             &summary);
      }
      if (ok) {
        SaveAndReload(summary);
      } else {
        RCLCPP_WARN(get_logger(),
                    "센서 자동 보정 조건 미충족(SLAM 미실행 또는 이동량 부족)");
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    rclcpp::shutdown();
  }

  std::mutex mtx_;
  std::string calib_file_;
  std::string mode_;
  std::string axis_;
  std::string result_file_;
  double speed_ = 0.20;
  double duration_s_ = 3.0;
  double settle_s_ = 0.8;
  uint64_t tick_cnt_ = 0;

  double measured_dist_m_ = 0.0;
  double measured_yaw_deg_ = 0.0;
  double measured_lat_drift_m_ = 0.0;
  double measured_yaw_drift_deg_ = 0.0;
  bool has_lat_drift_ = false;
  bool has_yaw_drift_ = false;
  double swing_clearance_m_ = -1.0;
  bool auto_apply_sensor_ = false;

  Calib calib_;
  std::chrono::steady_clock::time_point t0_;
  std::chrono::steady_clock::time_point last_tick_;
  std::chrono::steady_clock::time_point imu_prev_tp_;

  bool has_odom0_ = false;
  double odom_x0_ = 0.0;
  double odom_y0_ = 0.0;
  double odom_yaw0_ = 0.0;
  double odom_prev_yaw_ = 0.0;
  double odom_accum_yaw_ = 0.0;
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  std::vector<geometry_msgs::msg::Point> traj_pts_;

  bool has_imu0_ = false;
  bool has_imu_quat_ = false;
  double imu_prev_yaw_ = 0.0;
  double imu_accum_yaw_ = 0.0;

  bool has_status_ = false;
  float est_vx_ = 0.0f;
  float est_vy_ = 0.0f;
  double kin_dx_ = 0.0;
  double kin_dy_ = 0.0;
  std::vector<geometry_msgs::msg::Point> kin_pts_{{MakePt(0.0, 0.0, 0.015)}};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr
      trial_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      vis_event_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr status_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reload_cli_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace xsting_walk

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<xsting_walk::CalibGaitNode>();
    if (rclcpp::ok()) rclcpp::spin(node);
    node.reset();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("calib_gait_node"),
                 "보행 보정 노드 실행 실패: %s", e.what());
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
