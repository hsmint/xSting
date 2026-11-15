#ifndef XSTING_VISION_DEPTH_PCA_NODE_H_
#define XSTING_VISION_DEPTH_PCA_NODE_H_

#include <cuda_runtime.h>

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

namespace xsting_vision {

using Det2dMsg = vision_msgs::msg::Detection2D;
using Det2dArray = vision_msgs::msg::Detection2DArray;
using Det3dArray = vision_msgs::msg::Detection3DArray;
using ImageMsg = sensor_msgs::msg::Image;
using CloudMsg = sensor_msgs::msg::PointCloud2;
using CamInfoMsg = sensor_msgs::msg::CameraInfo;
using TfMsg = geometry_msgs::msg::TransformStamped;

// 전경 깊이 필터링 및 두 카메라 점군 통합 PCA 설정
struct DepthPcaCfg {
  std::string target_frame;
  std::string output_topic;
  std::vector<std::string> class_names;
  double color_offset_x;
  double min_depth;
  double max_depth;
  double roi_scale;
  double depth_tol;
  double puck_depth_tol;
  double box_top_z;
  double support_margin;
  int min_points;
  int max_points;
  double merge_dist;
  double merge_timeout;
};

// 단일 카메라 YOLO 박스에서 추출한 기준 좌표계 전경 점군
struct ObjCloud {
  std::string cls_id;
  double score = 0.0;
  cv::Vec3d cam_pos;
  cv::Vec3d centroid;
  std::vector<cv::Vec3d> pts;
};

// PCA로 산출한 3D OBB 중심·정규직교 주축·크기
struct PcaBox {
  bool valid = false;
  std::string cls_id;
  double score = 0.0;
  cv::Vec3d center;
  std::array<cv::Vec3d, 3> axes;
  cv::Vec3d size;
};

// 카메라 한 대의 구독·캐시 및 최신 전경 점군 묶음
struct PcaStream {
  std::string prefix;
  rclcpp::Subscription<Det2dArray>::SharedPtr det_sub;
  rclcpp::Subscription<ImageMsg>::SharedPtr depth_sub;
  rclcpp::Subscription<CamInfoMsg>::SharedPtr color_info_sub;
  rclcpp::Subscription<CamInfoMsg>::SharedPtr depth_info_sub;
  ImageMsg::ConstSharedPtr latest_depth;
  CamInfoMsg::ConstSharedPtr color_info;
  CamInfoMsg::ConstSharedPtr depth_info;
  rclcpp::Time cloud_stamp{0, 0, RCL_ROS_TIME};
  std::vector<ObjCloud> clouds;
};

// YOLO 2D 검출과 Depth 점군 통합 PCA로 3D 중심·자세를 추정하는 노드
class DepthPcaNode : public rclcpp::Node {
 public:
  explicit DepthPcaNode(
      const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());
  ~DepthPcaNode() override;

 private:
  static rclcpp::NodeOptions AddYamlParams(const rclcpp::NodeOptions& opts);
  DepthPcaCfg LoadCfg();
  void AddStream(const std::string& det_topic,
                 const std::string& depth_topic);
  void OnDet(PcaStream& stream, const Det2dArray::ConstSharedPtr& det_msg);
  bool CopyDepth(const ImageMsg& depth_msg);
  std::vector<ObjCloud> ExtractClouds(const PcaStream& stream,
                                      const Det2dArray& dets,
                                      const ImageMsg& depth_msg,
                                      const TfMsg& tf_msg);
  std::vector<cv::Vec3d> ExtractBoxPts(const PcaStream& stream,
                                       const ImageMsg& depth_msg,
                                       const Det2dMsg& det, bool is_box) const;
  std::vector<PcaBox> FuseAndFit(const PcaStream& cur_stream,
                                 const rclcpp::Time& stamp) const;
  PcaBox FitPca(const std::vector<cv::Vec3d>& pts, const std::string& cls_id,
                double score, const cv::Vec3d& view_dir, bool multi_view,
                double support_z) const;
  Det3dArray BuildDet3d(const rclcpp::Time& stamp,
                        const std::vector<PcaBox>& boxes) const;
  void PubCloud(const PcaStream& cur_stream, const rclcpp::Time& stamp);
  std::string GetClassName(const std::string& cls_id) const;

  DepthPcaCfg cfg_;
  std::mutex mtx_;
  cudaStream_t cuda_stream_ = nullptr;
  std::vector<uint16_t> depth_buf_;
  tf2_ros::Buffer tf_buf_;
  tf2_ros::TransformListener tf_sub_;
  rclcpp::Publisher<Det3dArray>::SharedPtr det3d_pub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr pts_pub_;
  std::vector<std::unique_ptr<PcaStream>> streams_;
};

}  // namespace xsting_vision

#endif  // XSTING_VISION_DEPTH_PCA_NODE_H_
