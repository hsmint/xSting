#ifndef XSTING_VISION_LIDAR_ODOM_NODE_H_
#define XSTING_VISION_LIDAR_ODOM_NODE_H_

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <isaac_ros_visual_slam_interfaces/msg/visual_slam_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace xsting_vision {

// =============================================================================
// 2D 평면 점 및 법선 벡터 (스캔 매칭 키프레임용)
// =============================================================================
struct Point2D {
  double x{0.0};
  double y{0.0};
  double nx{0.0};
  double ny{0.0};
  bool has_normal{false};
};

// =============================================================================
// SE(2) 평면 자세 (x, y, yaw)
// =============================================================================
struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// =============================================================================
// 1층 중앙 2D LiDAR 본체·다리 마스킹 + 2D ICP 오도메트리 보정 + nvblox 점군 변환 노드
// =============================================================================
class LidarOdomNode : public rclcpp::Node {
 public:
  explicit LidarOdomNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  static rclcpp::NodeOptions AddYamlParameters(
      const rclcpp::NodeOptions& options);

  void OnScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
  void OnVoOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
  void OnSlamStatus(
      const isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus::
          ConstSharedPtr& msg);

  // 각 다리(leg1_knee ~ leg6_knee) 정강이 선분과 스캔 평면의 2D 교차점 조회
  std::vector<std::pair<double, double>> LookupLegMaskCenters(
      double scan_z_in_base) const;

  // Point-to-Line / Point-to-Point 2D Gauss-Newton ICP 정합
  bool AlignScanToKeyframe(const std::vector<Point2D>& scan_pts,
                           const std::vector<Point2D>& keyframe_pts,
                           const Pose2D& keyframe_pose,
                           Pose2D& in_out_pose,
                           int& out_inliers) const;

  static void ComputeNormals(std::vector<Point2D>& pts);
  static Pose2D ComposePose2D(const Pose2D& a, const Pose2D& b);
  static Pose2D InverseComposePose2D(const Pose2D& base, const Pose2D& target);
  static double NormalizeAngle(double a);

  void PublishFusedOdometry(const rclcpp::Time& stamp,
                            const Pose2D& pose_2d,
                            double vx,
                            double vy,
                            double vyaw);
  void PublishNvbloxPointCloud(const std_msgs::msg::Header& header,
                               const std::vector<float>& lx,
                               const std::vector<float>& ly);

  // 프레임 및 토픽 파라미터
  std::string odom_frame_;
  std::string base_frame_;
  std::string lidar_frame_;
  std::string scan_in_topic_;
  std::string scan_out_topic_;
  std::string pointcloud_out_topic_;
  std::string vo_odom_in_topic_;
  std::string slam_status_topic_;
  std::string fused_odom_out_topic_;

  // 본체 및 6족 다리 자가충돌 마스킹 파라미터
  double lidar_z_in_base_{0.0245};
  double lidar_roll_in_base_{3.14159265359};
  double min_range_m_{0.12};
  double max_range_m_{12.0};
  double body_half_x_m_{0.145};
  double body_half_y_m_{0.145};
  double leg_mask_radius_m_{0.048};
  double tibia_length_m_{0.135};
  double fallback_leg_half_x_m_{0.18};
  double fallback_leg_max_y_m_{0.31};

  // 2D ICP 및 cuVSLAM 근접 벽 보정 파라미터
  bool enable_odom_fusion_{true};
  bool publish_odom_tf_{true};
  bool publish_pointcloud_{true};
  double near_wall_dist_m_{0.35};
  double max_vo_speed_mps_{1.2};
  double max_vo_yaw_rate_rps_{2.5};
  double vo_icp_max_pos_diff_m_{0.06};
  double vo_icp_max_yaw_diff_rad_{0.12};
  int min_icp_inliers_{20};
  int max_icp_points_{240};
  double keyframe_dist_m_{0.15};
  double keyframe_angle_rad_{0.18};
  double nvblox_half_elevation_rad_{0.01};

  // 상태 변수 (뮤텍스 보호)
  mutable std::mutex mutex_;
  uint8_t vo_state_{0};
  rclcpp::Time last_status_stamp_{0, 0, RCL_ROS_TIME};

  bool has_last_vo_{false};
  Pose2D last_vo_pose_{};
  rclcpp::Time last_vo_stamp_{0, 0, RCL_ROS_TIME};
  double fused_z_{0.0};
  double fused_roll_{0.0};
  double fused_pitch_{0.0};

  // LiDAR 기준 앵커 자세 및 스캔 사이 VO 증분 누적치
  Pose2D lidar_anchor_pose_{};
  Pose2D vo_delta_since_scan_{};
  Pose2D fused_pose_{};
  rclcpp::Time last_scan_stamp_{0, 0, RCL_ROS_TIME};
  bool has_keyframe_{false};
  bool last_icp_valid_{false};
  double min_front_dist_m_{10.0};
  Pose2D keyframe_pose_{};
  std::vector<Point2D> keyframe_points_;

  // TF 및 ROS 통신 인터페이스
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_odom_sub_;
  rclcpp::Subscription<isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>::
      SharedPtr status_sub_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

}  // namespace xsting_vision

#endif  // XSTING_VISION_LIDAR_ODOM_NODE_H_
