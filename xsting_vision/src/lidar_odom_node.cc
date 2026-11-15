#include "xsting_vision/lidar_odom_node.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

namespace xsting_vision {

namespace {

// 3x3 대칭 양정치 선형계 H * dx = b 풀이 (가우스 소거)
bool Solve3x3(double H[3][3], double b[3], double dx[3]) {
  double A[3][4];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      A[i][j] = H[i][j];
    }
    A[i][3] = b[i];
  }
  for (int col = 0; col < 3; ++col) {
    int pivot = col;
    for (int row = col + 1; row < 3; ++row) {
      if (std::abs(A[row][col]) > std::abs(A[pivot][col])) {
        pivot = row;
      }
    }
    if (std::abs(A[pivot][col]) < 1e-9) {
      return false;
    }
    if (pivot != col) {
      for (int j = col; j < 4; ++j) {
        std::swap(A[col][j], A[pivot][j]);
      }
    }
    for (int row = col + 1; row < 3; ++row) {
      const double factor = A[row][col] / A[col][col];
      for (int j = col; j < 4; ++j) {
        A[row][j] -= factor * A[col][j];
      }
    }
  }
  for (int i = 2; i >= 0; --i) {
    double sum = A[i][3];
    for (int j = i + 1; j < 3; ++j) {
      sum -= A[i][j] * dx[j];
    }
    dx[i] = sum / A[i][i];
  }
  return std::isfinite(dx[0]) && std::isfinite(dx[1]) && std::isfinite(dx[2]);
}

}  // namespace

// =============================================================================
// 생성자: 파라미터 로드, TF 버퍼 및 구독/발행기 초기화
// =============================================================================
LidarOdomNode::LidarOdomNode(const rclcpp::NodeOptions& options)
    : Node("lidar_odom", AddYamlParameters(options)) {
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base");
  lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar");

  scan_in_topic_ = declare_parameter<std::string>("scan_in_topic", "/scan");
  scan_out_topic_ =
      declare_parameter<std::string>("scan_out_topic", "/scan_filtered");
  pointcloud_out_topic_ = declare_parameter<std::string>(
      "pointcloud_out_topic", "/lidar/pointcloud");
  vo_odom_in_topic_ = declare_parameter<std::string>(
      "vo_odom_in_topic", "/visual_slam/tracking/vo_odometry");
  slam_status_topic_ = declare_parameter<std::string>(
      "slam_status_topic", "/visual_slam/status");
  fused_odom_out_topic_ = declare_parameter<std::string>(
      "fused_odom_out_topic", "/visual_slam/tracking/odometry");

  lidar_z_in_base_ = declare_parameter<double>("lidar_z_in_base", 0.0245);
  lidar_roll_in_base_ =
      declare_parameter<double>("lidar_roll_in_base", 3.14159265359);
  min_range_m_ = declare_parameter<double>("min_range_m", 0.12);
  max_range_m_ = declare_parameter<double>("max_range_m", 12.0);
  body_half_x_m_ = declare_parameter<double>("body_half_x_m", 0.145);
  body_half_y_m_ = declare_parameter<double>("body_half_y_m", 0.145);
  leg_mask_radius_m_ = declare_parameter<double>("leg_mask_radius_m", 0.048);
  tibia_length_m_ = declare_parameter<double>("tibia_length_m", 0.135);
  fallback_leg_half_x_m_ =
      declare_parameter<double>("fallback_leg_half_x_m", 0.18);
  fallback_leg_max_y_m_ =
      declare_parameter<double>("fallback_leg_max_y_m", 0.31);

  enable_odom_fusion_ = declare_parameter<bool>("enable_odom_fusion", true);
  publish_odom_tf_ = declare_parameter<bool>("publish_odom_tf", true);
  publish_pointcloud_ = declare_parameter<bool>("publish_pointcloud", true);
  near_wall_dist_m_ = declare_parameter<double>("near_wall_dist_m", 0.35);
  max_vo_speed_mps_ = declare_parameter<double>("max_vo_speed_mps", 1.2);
  max_vo_yaw_rate_rps_ = declare_parameter<double>("max_vo_yaw_rate_rps", 2.5);
  vo_icp_max_pos_diff_m_ =
      declare_parameter<double>("vo_icp_max_pos_diff_m", 0.06);
  vo_icp_max_yaw_diff_rad_ =
      declare_parameter<double>("vo_icp_max_yaw_diff_rad", 0.12);
  min_icp_inliers_ = declare_parameter<int>("min_icp_inliers", 20);
  max_icp_points_ = declare_parameter<int>("max_icp_points", 240);
  keyframe_dist_m_ = declare_parameter<double>("keyframe_dist_m", 0.15);
  keyframe_angle_rad_ = declare_parameter<double>("keyframe_angle_rad", 0.18);
  nvblox_half_elevation_rad_ =
      declare_parameter<double>("nvblox_half_elevation_rad", 0.01);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  scan_pub_ =
      create_publisher<sensor_msgs::msg::LaserScan>(scan_out_topic_, 10);
  if (publish_pointcloud_) {
    pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        pointcloud_out_topic_, rclcpp::SensorDataQoS());
  }
  if (enable_odom_fusion_) {
    odom_pub_ =
        create_publisher<nav_msgs::msg::Odometry>(fused_odom_out_topic_, 10);
    vo_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        vo_odom_in_topic_, rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
          OnVoOdometry(msg);
        });
    status_sub_ = create_subscription<
        isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>(
        slam_status_topic_, rclcpp::SensorDataQoS(),
        [this](const isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus::
                   ConstSharedPtr& msg) { OnSlamStatus(msg); });
  }

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_in_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
        OnScan(msg);
      });
}

// =============================================================================
// YAML 기본 설정 로드 (-p 오버라이드가 우선)
// =============================================================================
rclcpp::NodeOptions LidarOdomNode::AddYamlParameters(
    const rclcpp::NodeOptions& options) {
  std::string config_file_path =
      (ament_index_cpp::get_package_share_path("xsting_vision") / "config" /
       "lidar_odom.yaml")
          .string();
  for (const rclcpp::Parameter& parameter : options.parameter_overrides()) {
    if (parameter.get_name() == "config_file") {
      config_file_path = parameter.as_string();
    }
  }

  std::vector<rclcpp::Parameter> yaml_parameters;
  try {
    for (const auto& node_parameters :
         rclcpp::parameter_map_from_yaml_file(config_file_path)) {
      yaml_parameters.insert(yaml_parameters.end(),
                             node_parameters.second.begin(),
                             node_parameters.second.end());
    }
  } catch (const std::exception&) {
  }

  rclcpp::NodeOptions options_with_yaml = options;
  auto& overrides = options_with_yaml.parameter_overrides();
  overrides.insert(overrides.begin(), yaml_parameters.begin(),
                   yaml_parameters.end());
  return options_with_yaml;
}

// =============================================================================
// cuVSLAM 추적 상태 콜백 (vo_state == 1 이면 정상)
// =============================================================================
void LidarOdomNode::OnSlamStatus(
    const isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus::
        ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  vo_state_ = msg->vo_state;
  last_status_stamp_ = msg->header.stamp;
}

// =============================================================================
// cuVSLAM 내부 오도메트리 콜백: 점프/근접 벽 게이팅 후 고주파 보간 발행
// =============================================================================
void LidarOdomNode::OnVoOdometry(
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  const auto& p = msg->pose.pose.position;
  const auto& q = msg->pose.pose.orientation;
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
  const Pose2D cur_vo{p.x, p.y, yaw};
  const rclcpp::Time stamp(msg->header.stamp);

  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_last_vo_) {
    has_last_vo_ = true;
    last_vo_pose_ = cur_vo;
    last_vo_stamp_ = stamp;
    fused_z_ = p.z;
    fused_roll_ = roll;
    fused_pitch_ = pitch;
    PublishFusedOdometry(stamp, fused_pose_, 0.0, 0.0, 0.0);
    return;
  }

  const double dt = (stamp - last_vo_stamp_).seconds();
  const Pose2D step_vo = InverseComposePose2D(last_vo_pose_, cur_vo);
  last_vo_pose_ = cur_vo;
  last_vo_stamp_ = stamp;

  if (dt <= 1e-4 || dt > 1.0) {
    return;
  }

  const double speed = std::hypot(step_vo.x, step_vo.y) / dt;
  const double yaw_rate = std::abs(step_vo.yaw) / dt;
  const bool status_failed = (vo_state_ == 2);
  const bool near_wall_locked =
      last_icp_valid_ && (min_front_dist_m_ < near_wall_dist_m_);
  const bool step_sane =
      !status_failed && (speed <= max_vo_speed_mps_) &&
      (yaw_rate <= max_vo_yaw_rate_rps_);

  double vx = 0.0, vy = 0.0, vyaw = 0.0;
  if (step_sane) {
    // 높이·기울기는 정상 VO일 때 완만히 반영
    fused_z_ = 0.8 * fused_z_ + 0.2 * p.z;
    fused_roll_ = roll;
    fused_pitch_ = pitch;
    if (!near_wall_locked) {
      vo_delta_since_scan_ = ComposePose2D(vo_delta_since_scan_, step_vo);
      fused_pose_ = ComposePose2D(lidar_anchor_pose_, vo_delta_since_scan_);
      vx = step_vo.x / dt;
      vy = step_vo.y / dt;
      vyaw = step_vo.yaw / dt;
    }
  }

  PublishFusedOdometry(stamp, fused_pose_, vx, vy, vyaw);
}

// =============================================================================
// 각 다리(leg1_knee ~ leg6_knee) 정강이 선분과 스캔 평면(z = scan_z)의 교차점 계산
// =============================================================================
std::vector<std::pair<double, double>> LidarOdomNode::LookupLegMaskCenters(
    double scan_z_in_base) const {
  static const std::array<std::pair<const char*, double>, 6> kLegs = {{
      {"leg1_knee", +1.0},
      {"leg2_knee", +1.0},
      {"leg3_knee", +1.0},
      {"leg4_knee", -1.0},
      {"leg5_knee", -1.0},
      {"leg6_knee", -1.0},
  }};

  std::vector<std::pair<double, double>> centers;
  centers.reserve(6);

  for (const auto& [frame, sign_y] : kLegs) {
    try {
      const auto tf = tf_buffer_->lookupTransform(base_frame_, frame,
                                                  tf2::TimePointZero);
      const auto& t = tf.transform.translation;
      const auto& r = tf.transform.rotation;
      tf2::Matrix3x3 rot(tf2::Quaternion(r.x, r.y, r.z, r.w));
      const tf2::Vector3 knee(t.x, t.y, t.z);
      const tf2::Vector3 foot =
          knee + rot * tf2::Vector3(0.0, sign_y * tibia_length_m_, 0.0);

      const double z0 = knee.z();
      const double z1 = foot.z();
      const double z_min = std::min(z0, z1) - 0.025;
      const double z_max = std::max(z0, z1) + 0.025;
      if (scan_z_in_base < z_min || scan_z_in_base > z_max) {
        continue;
      }
      double alpha = 0.5;
      if (std::abs(z1 - z0) > 1e-4) {
        alpha = std::clamp((scan_z_in_base - z0) / (z1 - z0), 0.0, 1.0);
      }
      const double cx = knee.x() + alpha * (foot.x() - knee.x());
      const double cy = knee.y() + alpha * (foot.y() - knee.y());
      centers.emplace_back(cx, cy);
    } catch (const tf2::TransformException&) {
      return {};
    }
  }
  return centers;
}

// =============================================================================
// LiDAR 스캔 콜백: 3D TF(상하 반전 Roll=π 포함) 반영 자가충돌 제거 → ICP 보정
// =============================================================================
void LidarOdomNode::OnScan(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
  auto filtered = *msg;
  if (filtered.header.frame_id.empty()) {
    filtered.header.frame_id = lidar_frame_;
  }

  // lidar → base 3D 변환 조회 (천장 매달기 Roll=180° 등 모든 3D 회전 정확히 반영)
  tf2::Quaternion q_default;
  q_default.setRPY(lidar_roll_in_base_, 0.0, 0.0);
  tf2::Matrix3x3 rot_bl(q_default);
  tf2::Vector3 trans_bl(0.0, 0.0, lidar_z_in_base_);
  try {
    const auto tf = tf_buffer_->lookupTransform(
        base_frame_, filtered.header.frame_id, tf2::TimePointZero);
    trans_bl.setValue(tf.transform.translation.x, tf.transform.translation.y,
                      tf.transform.translation.z);
    rot_bl.setRotation(tf2::Quaternion(tf.transform.rotation.x,
                                       tf.transform.rotation.y,
                                       tf.transform.rotation.z,
                                       tf.transform.rotation.w));
  } catch (const tf2::TransformException&) {
  }

  const auto leg_centers = LookupLegMaskCenters(trans_bl.z());
  const bool has_leg_tf = (leg_centers.size() == 6);
  const double leg_r2 = leg_mask_radius_m_ * leg_mask_radius_m_;

  std::vector<Point2D> valid_base_pts;
  std::vector<float> valid_lx;
  std::vector<float> valid_ly;
  valid_base_pts.reserve(filtered.ranges.size());
  valid_lx.reserve(filtered.ranges.size());
  valid_ly.reserve(filtered.ranges.size());

  double front_min = 10.0;

  for (size_t i = 0; i < filtered.ranges.size(); ++i) {
    const float r = filtered.ranges[i];
    if (!std::isfinite(r) || r < min_range_m_ || r > max_range_m_) {
      filtered.ranges[i] = std::numeric_limits<float>::infinity();
      continue;
    }

    const double angle =
        filtered.angle_min + static_cast<double>(i) * filtered.angle_increment;
    const double lx = static_cast<double>(r) * std::cos(angle);
    const double ly = static_cast<double>(r) * std::sin(angle);
    const tf2::Vector3 p_base = rot_bl * tf2::Vector3(lx, ly, 0.0) + trans_bl;
    const double bx = p_base.x();
    const double by = p_base.y();

    // 1) 본체 와플판 스탠드오프 기둥 및 전면 초음파 센서 영역 제거
    bool masked = (std::abs(bx) < body_half_x_m_ &&
                   std::abs(by) < body_half_y_m_);

    // 2) 좌·우 6개 다리(정강이 선분 스캔 평면 교차점 반경 또는 폴백 박스) 제거
    if (!masked) {
      if (has_leg_tf) {
        for (const auto& [cx, cy] : leg_centers) {
          const double dx = bx - cx;
          const double dy = by - cy;
          if (dx * dx + dy * dy < leg_r2) {
            masked = true;
            break;
          }
        }
      } else if (std::abs(bx) < fallback_leg_half_x_m_ &&
                 std::abs(by) >= body_half_y_m_ &&
                 std::abs(by) < fallback_leg_max_y_m_) {
        masked = true;
      }
    }

    if (masked) {
      filtered.ranges[i] = std::numeric_limits<float>::infinity();
      continue;
    }

    // 전방(±50°) 최근접 벽 거리 계측
    if (bx > 0.0 && std::abs(by) < 0.84 * bx) {
      front_min = std::min(front_min, std::hypot(bx, by));
    }

    valid_base_pts.push_back(Point2D{bx, by, 0.0, 0.0, false});
    valid_lx.push_back(static_cast<float>(lx));
    valid_ly.push_back(static_cast<float>(ly));
  }

  scan_pub_->publish(filtered);
  if (publish_pointcloud_ && pc_pub_) {
    PublishNvbloxPointCloud(filtered.header, valid_lx, valid_ly);
  }

  if (!enable_odom_fusion_) {
    return;
  }

  // ICP 입력용 등간격 다운샘플링
  std::vector<Point2D> icp_pts;
  if (!valid_base_pts.empty()) {
    const size_t stride = std::max<size_t>(
        1, valid_base_pts.size() / static_cast<size_t>(max_icp_points_));
    icp_pts.reserve(valid_base_pts.size() / stride + 1);
    for (size_t i = 0; i < valid_base_pts.size(); i += stride) {
      icp_pts.push_back(valid_base_pts[i]);
    }
  }

  const rclcpp::Time stamp(filtered.header.stamp);
  std::lock_guard<std::mutex> lock(mutex_);
  min_front_dist_m_ = front_min;

  if (!has_keyframe_) {
    if (static_cast<int>(icp_pts.size()) >= min_icp_inliers_) {
      keyframe_points_ = icp_pts;
      ComputeNormals(keyframe_points_);
      keyframe_pose_ = fused_pose_;
      lidar_anchor_pose_ = fused_pose_;
      vo_delta_since_scan_ = Pose2D{};
      has_keyframe_ = true;
      last_icp_valid_ = true;
    }
    last_scan_stamp_ = stamp;
    PublishFusedOdometry(stamp, fused_pose_, 0.0, 0.0, 0.0);
    return;
  }

  Pose2D guess = ComposePose2D(lidar_anchor_pose_, vo_delta_since_scan_);
  Pose2D icp_pose = guess;
  int inliers = 0;
  const bool icp_ok =
      (static_cast<int>(icp_pts.size()) >= min_icp_inliers_) &&
      AlignScanToKeyframe(icp_pts, keyframe_points_, keyframe_pose_, icp_pose,
                          inliers);

  double vx = 0.0, vy = 0.0, vyaw = 0.0;
  const double dt = (last_scan_stamp_.nanoseconds() > 0)
                        ? (stamp - last_scan_stamp_).seconds()
                        : 0.1;

  if (icp_ok) {
    const Pose2D prev_anchor = lidar_anchor_pose_;
    const Pose2D vo_vs_icp = InverseComposePose2D(icp_pose, guess);
    const bool near_wall = (min_front_dist_m_ < near_wall_dist_m_);
    const bool vo_bad =
        (vo_state_ == 2) ||
        (std::hypot(vo_vs_icp.x, vo_vs_icp.y) > vo_icp_max_pos_diff_m_) ||
        (std::abs(vo_vs_icp.yaw) > vo_icp_max_yaw_diff_rad_);

    // 벽 근접 또는 VO 이상 시 100% LiDAR ICP 고정, 정상 시에도 드리프트 방지를 위해 ICP 앵커 갱신
    if (near_wall || vo_bad) {
      lidar_anchor_pose_ = icp_pose;
    } else {
      lidar_anchor_pose_.x = 0.85 * icp_pose.x + 0.15 * guess.x;
      lidar_anchor_pose_.y = 0.85 * icp_pose.y + 0.15 * guess.y;
      lidar_anchor_pose_.yaw = NormalizeAngle(
          icp_pose.yaw + 0.15 * NormalizeAngle(guess.yaw - icp_pose.yaw));
    }
    vo_delta_since_scan_ = Pose2D{};
    fused_pose_ = lidar_anchor_pose_;
    last_icp_valid_ = true;

    if (dt > 1e-3 && dt < 1.0) {
      const Pose2D step = InverseComposePose2D(prev_anchor, lidar_anchor_pose_);
      vx = step.x / dt;
      vy = step.y / dt;
      vyaw = step.yaw / dt;
    }

    // 이동·회전 임계치 도달 시 키프레임 갱신
    const Pose2D from_key =
        InverseComposePose2D(keyframe_pose_, lidar_anchor_pose_);
    if (std::hypot(from_key.x, from_key.y) >= keyframe_dist_m_ ||
        std::abs(from_key.yaw) >= keyframe_angle_rad_) {
      keyframe_points_ = icp_pts;
      ComputeNormals(keyframe_points_);
      keyframe_pose_ = lidar_anchor_pose_;
    }
  } else {
    last_icp_valid_ = false;
    lidar_anchor_pose_ = guess;
    vo_delta_since_scan_ = Pose2D{};
    fused_pose_ = lidar_anchor_pose_;
  }

  last_scan_stamp_ = stamp;
  PublishFusedOdometry(stamp, fused_pose_, vx, vy, vyaw);
}

// =============================================================================
// 키프레임 점군에서 인접 점 기반 벽면 법선 계산
// =============================================================================
void LidarOdomNode::ComputeNormals(std::vector<Point2D>& pts) {
  const size_t n = pts.size();
  if (n < 3) {
    return;
  }
  for (size_t i = 0; i < n; ++i) {
    const auto& prev = pts[(i + n - 1) % n];
    const auto& next = pts[(i + 1) % n];
    const double dx = next.x - prev.x;
    const double dy = next.y - prev.y;
    const double len = std::hypot(dx, dy);
    if (len > 0.01 && len < 0.25) {
      double nx = -dy / len;
      double ny = dx / len;
      // 센서 원점(0, 0)을 향하도록 부호 정렬
      if (nx * pts[i].x + ny * pts[i].y > 0.0) {
        nx = -nx;
        ny = -ny;
      }
      pts[i].nx = nx;
      pts[i].ny = ny;
      pts[i].has_normal = true;
    }
  }
}

// =============================================================================
// 2D Point-to-Line + Point-to-Point ICP (단일 평면 벽 퇴화 방지 사전 정규화 포함)
// =============================================================================
bool LidarOdomNode::AlignScanToKeyframe(
    const std::vector<Point2D>& scan_pts,
    const std::vector<Point2D>& keyframe_pts,
    const Pose2D& keyframe_pose,
    Pose2D& in_out_pose,
    int& out_inliers) const {
  if (keyframe_pts.empty() || scan_pts.empty()) {
    return false;
  }

  // 키프레임 로컬 좌표계에서의 상대 자세 T_rel 추정
  Pose2D rel = InverseComposePose2D(keyframe_pose, in_out_pose);
  const Pose2D prior_rel = rel;
  constexpr double kHuberDelta = 0.05;
  constexpr double kPriorWeightPos = 2.0;
  constexpr double kPriorWeightYaw = 2.0;

  int inliers = 0;
  for (int iter = 0; iter < 18; ++iter) {
    const double max_corr = (iter < 6) ? 0.28 : 0.14;
    const double max_corr2 = max_corr * max_corr;
    const double c = std::cos(rel.yaw);
    const double s = std::sin(rel.yaw);

    double H[3][3] = {{kPriorWeightPos, 0.0, 0.0},
                      {0.0, kPriorWeightPos, 0.0},
                      {0.0, 0.0, kPriorWeightYaw}};
    double b[3] = {-kPriorWeightPos * (rel.x - prior_rel.x),
                   -kPriorWeightPos * (rel.y - prior_rel.y),
                   -kPriorWeightYaw * NormalizeAngle(rel.yaw - prior_rel.yaw)};
    inliers = 0;

    for (const auto& sp : scan_pts) {
      const double wx = c * sp.x - s * sp.y + rel.x;
      const double wy = s * sp.x + c * sp.y + rel.y;

      int best_idx = -1;
      double best_d2 = max_corr2;
      for (size_t k = 0; k < keyframe_pts.size(); ++k) {
        const double dx = wx - keyframe_pts[k].x;
        const double dy = wy - keyframe_pts[k].y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
          best_d2 = d2;
          best_idx = static_cast<int>(k);
        }
      }
      if (best_idx < 0) {
        continue;
      }
      ++inliers;

      const auto& kp = keyframe_pts[best_idx];
      const double rx = -s * sp.x - c * sp.y;
      const double ry = c * sp.x - s * sp.y;

      if (kp.has_normal) {
        const double err = kp.nx * (wx - kp.x) + kp.ny * (wy - kp.y);
        const double w = (std::abs(err) > kHuberDelta)
                             ? (kHuberDelta / std::abs(err))
                             : 1.0;
        const double J[3] = {kp.nx, kp.ny, kp.nx * rx + kp.ny * ry};
        for (int r_i = 0; r_i < 3; ++r_i) {
          b[r_i] -= w * J[r_i] * err;
          for (int c_i = 0; c_i < 3; ++c_i) {
            H[r_i][c_i] += w * J[r_i] * J[c_i];
          }
        }
      } else {
        const double ex = wx - kp.x;
        const double ey = wy - kp.y;
        const double d = std::sqrt(best_d2);
        const double w = (d > kHuberDelta) ? (0.5 * kHuberDelta / d) : 0.5;
        const double Jx[3] = {1.0, 0.0, rx};
        const double Jy[3] = {0.0, 1.0, ry};
        for (int r_i = 0; r_i < 3; ++r_i) {
          b[r_i] -= w * (Jx[r_i] * ex + Jy[r_i] * ey);
          for (int c_i = 0; c_i < 3; ++c_i) {
            H[r_i][c_i] += w * (Jx[r_i] * Jx[c_i] + Jy[r_i] * Jy[c_i]);
          }
        }
      }
    }

    if (inliers < min_icp_inliers_) {
      out_inliers = inliers;
      return false;
    }

    double dx[3] = {0.0, 0.0, 0.0};
    if (!Solve3x3(H, b, dx)) {
      return false;
    }
    rel.x += dx[0];
    rel.y += dx[1];
    rel.yaw = NormalizeAngle(rel.yaw + dx[2]);
    if (dx[0] * dx[0] + dx[1] * dx[1] < 1e-8 && std::abs(dx[2]) < 1e-4) {
      break;
    }
  }

  // 급격한 비정상 발산 방지
  const Pose2D diff = InverseComposePose2D(prior_rel, rel);
  if (std::hypot(diff.x, diff.y) > 0.35 || std::abs(diff.yaw) > 0.55) {
    out_inliers = inliers;
    return false;
  }

  out_inliers = inliers;
  in_out_pose = ComposePose2D(keyframe_pose, rel);
  return true;
}

// =============================================================================
// SE(2) 보조 연산
// =============================================================================
Pose2D LidarOdomNode::ComposePose2D(const Pose2D& a, const Pose2D& b) {
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  return Pose2D{a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y,
                NormalizeAngle(a.yaw + b.yaw)};
}

Pose2D LidarOdomNode::InverseComposePose2D(const Pose2D& base,
                                           const Pose2D& target) {
  const double dx = target.x - base.x;
  const double dy = target.y - base.y;
  const double c = std::cos(base.yaw);
  const double s = std::sin(base.yaw);
  return Pose2D{c * dx + s * dy, -s * dx + c * dy,
                NormalizeAngle(target.yaw - base.yaw)};
}

double LidarOdomNode::NormalizeAngle(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

// =============================================================================
// 보정된 odom → base TF 및 Odometry 메시지 발행
// =============================================================================
void LidarOdomNode::PublishFusedOdometry(const rclcpp::Time& stamp,
                                         const Pose2D& pose_2d,
                                         double vx,
                                         double vy,
                                         double vyaw) {
  tf2::Quaternion q;
  q.setRPY(fused_roll_, fused_pitch_, pose_2d.yaw);
  q.normalize();

  if (publish_odom_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = pose_2d.x;
    tf_msg.transform.translation.y = pose_2d.y;
    tf_msg.transform.translation.z = fused_z_;
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf_msg);
  }

  if (odom_pub_) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose_2d.x;
    odom.pose.pose.position.y = pose_2d.y;
    odom.pose.pose.position.z = fused_z_;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vyaw;
    odom_pub_->publish(odom);
  }
}

// =============================================================================
// nvblox 3D LiDAR 적분기 규격(2행 elevation)에 맞춘 PointCloud2 발행
// =============================================================================
void LidarOdomNode::PublishNvbloxPointCloud(
    const std_msgs::msg::Header& header,
    const std::vector<float>& lx,
    const std::vector<float>& ly) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(lx.size() * 2);
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 16;
  cloud.row_step = cloud.point_step * cloud.width;

  cloud.fields.resize(4);
  const char* names[4] = {"x", "y", "z", "intensity"};
  for (int i = 0; i < 4; ++i) {
    cloud.fields[i].name = names[i];
    cloud.fields[i].offset = static_cast<uint32_t>(i * 4);
    cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[i].count = 1;
  }

  cloud.data.resize(cloud.row_step);
  const float tan_e = static_cast<float>(std::tan(nvblox_half_elevation_rad_));
  float* ptr = reinterpret_cast<float*>(cloud.data.data());
  for (size_t i = 0; i < lx.size(); ++i) {
    const float r = std::hypot(lx[i], ly[i]);
    const float dz = r * tan_e;
    ptr[0] = lx[i];
    ptr[1] = ly[i];
    ptr[2] = -dz;
    ptr[3] = 1.0f;
    ptr[4] = lx[i];
    ptr[5] = ly[i];
    ptr[6] = +dz;
    ptr[7] = 1.0f;
    ptr += 8;
  }
  pc_pub_->publish(cloud);
}

}  // namespace xsting_vision

RCLCPP_COMPONENTS_REGISTER_NODE(xsting_vision::LidarOdomNode)
