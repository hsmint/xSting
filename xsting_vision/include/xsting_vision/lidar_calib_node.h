#ifndef XSTING_VISION_LIDAR_CALIB_NODE_H_
#define XSTING_VISION_LIDAR_CALIB_NODE_H_

#include <cuda_runtime.h>

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace xsting_vision {

using ScanMsg = sensor_msgs::msg::LaserScan;
using ImageMsg = sensor_msgs::msg::Image;
using CamInfoMsg = sensor_msgs::msg::CameraInfo;
using CloudMsg = sensor_msgs::msg::PointCloud2;
using TfMsg = geometry_msgs::msg::TransformStamped;

// LiDAR 외부 파라미터 실측 보정 설정
struct LidarCalibCfg {
  std::string mode;
  std::string scan_topic;
  std::string depth_topic;
  std::string base_frame;
  std::string lidar_frame;
  double min_range;
  double max_range;
  double depth_min_z;
  double depth_max_z;
  int num_samples;
  double sample_interval;
  double max_sync_diff;
  double lidar_z;
  double front_wall_dist;
  double left_wall_dist;
  double right_wall_dist;
  bool apply_urdf;
  std::string urdf_path;
};

// 2차원 평면 좌표
struct Vec2d {
  double x = 0.0;
  double y = 0.0;
};

// 법선 벡터를 포함한 2차원 기준 벽면 점
struct WallPt {
  double x = 0.0;
  double y = 0.0;
  double nx = 0.0;
  double ny = 0.0;
  bool has_norm = false;
};

// 단일 프레임의 스캔 점군 및 카메라 벽면 점군 쌍
struct ScanSample {
  std::vector<Vec2d> scan_pts;
  std::vector<WallPt> ref_pts;
};

// base 기준 LiDAR 외부 파라미터(x, y, z, roll, yaw)
struct LidarPose {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0245;
  double roll = 3.14159265359;
  double yaw = 0.0;
};

// 스캔 직선 피팅 및 RealSense 깊이 정합으로 LiDAR TF를 보정하는 노드
class LidarCalibNode : public rclcpp::Node {
 public:
  explicit LidarCalibNode(
      const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());
  ~LidarCalibNode() override;

 private:
  static rclcpp::NodeOptions AddYamlParams(const rclcpp::NodeOptions& opts);
  LidarCalibCfg LoadCfg();
  LidarPose LookupCurPose(const std::string& scan_frame) const;
  void OnScan(const ScanMsg::ConstSharedPtr& scan_msg);
  bool CopyDepth(const ImageMsg& depth_msg);
  std::vector<WallPt> ExtractDepthWall(const ImageMsg& depth_msg,
                                       const CamInfoMsg& info,
                                       const TfMsg& tf_base_cam) const;
  void FinishCamCalib(const LidarPose& cur_pose);
  void FinishWallCalib(const LidarPose& cur_pose);
  void PubClouds(const std_msgs::msg::Header& header, const LidarPose& pose,
                 const std::vector<Vec2d>& scan_pts,
                 const std::vector<WallPt>& ref_pts);
  void ReportResult(const LidarPose& cur_pose, const LidarPose& calib_pose,
                    double before_mm, double after_mm, bool full_xy);
  bool UpdateUrdf(const LidarPose& calib_pose) const;

  LidarCalibCfg cfg_;
  std::mutex mtx_;
  cudaStream_t cuda_stream_ = nullptr;
  std::vector<uint16_t> depth_buf_;
  tf2_ros::Buffer tf_buf_;
  tf2_ros::TransformListener tf_sub_;
  rclcpp::Subscription<ScanMsg>::SharedPtr scan_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr depth_sub_;
  rclcpp::Subscription<CamInfoMsg>::SharedPtr info_sub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr scan_pub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr ref_pub_;
  ImageMsg::ConstSharedPtr latest_depth_;
  CamInfoMsg::ConstSharedPtr depth_info_;
  rclcpp::Time last_sample_stamp_{0, 0, RCL_ROS_TIME};
  std::vector<ScanSample> samples_;
  bool done_ = false;
};

}  // namespace xsting_vision

#endif  // XSTING_VISION_LIDAR_CALIB_NODE_H_
