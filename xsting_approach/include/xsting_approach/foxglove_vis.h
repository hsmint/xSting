#ifndef XSTING_APPROACH_FOXGLOVE_VIS_H_
#define XSTING_APPROACH_FOXGLOVE_VIS_H_

#include <Eigen/Dense>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "xsting_approach/calib.h"
#include "xsting_approach/pca_vis.h"

namespace xsting_approach {

// Foxglove 3D·플롯 시각화에 전달할 실측 보정 및 센서 상태 묶음
struct VisFrame {
  Calib calib;
  double target_gap_m = 0.020;
  bool box_valid = false;
  Eigen::Vector2f box_xy = Eigen::Vector2f::Zero();
  Eigen::Vector2f norm_xy = Eigen::Vector2f::UnitX();
  bool us_valid = false;
  float us_raw_m = 4.0f;
  float us_corr_m = 4.0f;
  Eigen::Vector3f proj_grav_b = Eigen::Vector3f(0.0f, 0.0f, -1.0f);
  std::vector<Eigen::Vector3f> floor_pts;
  std::vector<Eigen::Vector3f> wall_pts;
  std::vector<Eigen::Vector3f> puck_pts;
};

// Foxglove Studio 실측 보정 검증용 마커·점군·수치·프리뷰 영상 퍼블리셔
class FoxgloveVis {
 public:
  explicit FoxgloveVis(rclcpp::Node* node);

  // 구독자 유무 조회 (미구독 시 점군·영상 연산 생략)
  bool Has3dSub() const;
  bool HasCloudSub() const;
  bool HasImgSub() const;

  // 3D 마커, 색상 점군 및 실시간 오차 수치(mm/deg) 발행
  void Publish(const rclcpp::Time& stamp, const VisFrame& f);

  // 깊이 영상 위에 상자·퍽·바닥 ROI를 그린 경량 RGB 프리뷰 발행
  void PublishDepthImg(const rclcpp::Time& stamp,
                       const float* depth_m,
                       int width,
                       int height,
                       const std::vector<Roi>& box_rois,
                       const std::vector<Roi>& puck_rois,
                       const Roi* floor_roi = nullptr);

 private:
  rclcpp::Node* node_ = nullptr;
  rclcpp::Time last_pub_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_img_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mark_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr metric_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
};

}  // namespace xsting_approach

#endif  // XSTING_APPROACH_FOXGLOVE_VIS_H_
