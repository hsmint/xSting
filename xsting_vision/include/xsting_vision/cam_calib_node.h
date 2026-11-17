#ifndef XSTING_VISION_CAM_CALIB_NODE_H_
#define XSTING_VISION_CAM_CALIB_NODE_H_

#include <cuda_runtime.h>

#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace xsting_vision {

using ImageMsg = sensor_msgs::msg::Image;
using CamInfoMsg = sensor_msgs::msg::CameraInfo;
using TfMsg = geometry_msgs::msg::TransformStamped;

// 체커보드 기반 두 카메라 외부 파라미터 보정 설정
struct CamCalibCfg {
  std::string top_img_topic;
  std::string bot_img_topic;
  std::string base_frame;
  std::string top_link_frame;
  int board_cols;
  int board_rows;
  double square_size;
  int num_samples;
  double sample_interval;
  double max_sync_diff;
};

// 3차원 강체 변환(회전 행렬 및 평행이동 벡터)
struct Rigid3d {
  cv::Matx33d rot = cv::Matx33d::eye();
  cv::Vec3d trans = cv::Vec3d(0.0, 0.0, 0.0);
};

// 체커보드 동시 관측으로 상단 카메라 외부 파라미터를 1회성 보정하는 노드
class CamCalibNode : public rclcpp::Node {
 public:
  explicit CamCalibNode(
      const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());
  ~CamCalibNode() override;

 private:
  static rclcpp::NodeOptions AddYamlParams(const rclcpp::NodeOptions& opts);
  CamCalibCfg LoadCfg();
  std::vector<cv::Point3f> BuildBoardPts() const;
  void OnTopImg(const ImageMsg::ConstSharedPtr& top_msg);
  bool CopyImg(const ImageMsg& img_msg, cv::Mat& rgb);
  bool DetectBoard(const cv::Mat& rgb, const CamInfoMsg& info,
                   std::vector<cv::Point2f>& corners, Rigid3d& pose) const;
  void PubDebugImg(const std_msgs::msg::Header& header, const cv::Mat& top_rgb,
                   const std::vector<cv::Point2f>& top_corners, bool top_ok,
                   const cv::Mat& bot_rgb,
                   const std::vector<cv::Point2f>& bot_corners, bool bot_ok);
  void FinishCalib(const Rigid3d& cur_base_from_top) const;

  CamCalibCfg cfg_;
  std::vector<cv::Point3f> board_pts_;
  std::mutex mtx_;
  cudaStream_t cuda_stream_ = nullptr;
  tf2_ros::Buffer tf_buf_;
  tf2_ros::TransformListener tf_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr top_img_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr bot_img_sub_;
  rclcpp::Subscription<CamInfoMsg>::SharedPtr top_info_sub_;
  rclcpp::Subscription<CamInfoMsg>::SharedPtr bot_info_sub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr debug_pub_;
  ImageMsg::ConstSharedPtr latest_bot_img_;
  CamInfoMsg::ConstSharedPtr top_info_;
  CamInfoMsg::ConstSharedPtr bot_info_;
  rclcpp::Time last_try_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_sample_stamp_{0, 0, RCL_ROS_TIME};
  int sample_cnt_ = 0;
  bool done_ = false;
  std::vector<cv::Vec3d> src_pts_;
  std::vector<cv::Vec3d> dst_pts_;
};

}  // namespace xsting_vision

#endif  // XSTING_VISION_CAM_CALIB_NODE_H_
