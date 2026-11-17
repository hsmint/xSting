#ifndef XSTING_VISION_REALSENSE_SPLITTER_NODE_H_
#define XSTING_VISION_REALSENSE_SPLITTER_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include <message_filters/subscriber.hpp>
#include <message_filters/time_synchronizer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realsense2_camera_msgs/msg/metadata.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace xsting_vision {

using ImageMetadataSynchronizer =
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image,
                                      realsense2_camera_msgs::msg::Metadata>;

// =============================================================================
// 스트림 하나(infra1, infra2, depth)의 이미지·metadata 동기화와 퍼블리셔 묶음
// =============================================================================
struct SplitStream {
  // 이 에미터 상태(0 꺼짐, 1 켜짐)의 프레임만 발행
  int emitter_mode;
  message_filters::Subscriber<sensor_msgs::msg::Image> image_subscriber;
  std::unique_ptr<ImageMetadataSynchronizer> synchronizer;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
};

// =============================================================================
// 카메라 한 대의 metadata 구독(세 스트림이 함께 사용)과 스트림 목록
// =============================================================================
struct SplitCamera {
  message_filters::Subscriber<realsense2_camera_msgs::msg::Metadata>
      metadata_subscriber;
  std::vector<std::unique_ptr<SplitStream>> streams;
};

// =============================================================================
// 에미터 on/off 교차 프레임을 나눠 발행 (rclcpp component)
// (에미터 꺼짐 → infra1·infra2, 에미터 켜짐 → depth)
// =============================================================================
class RealsenseSplitterNode : public rclcpp::Node {
 public:
  explicit RealsenseSplitterNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  // YAML 설정 파일을 직접 읽어 NodeOptions에 추가
  static rclcpp::NodeOptions AddYamlParameters(
      const rclcpp::NodeOptions& options);

  // 카메라별 metadata 구독, 스트림별 동기화·퍼블리셔 생성
  void CreateCamera(const std::string& camera_name);
  void CreateStream(SplitCamera& camera, const std::string& camera_name,
                    const std::string& stream_name,
                    const std::string& output_topic_name, int emitter_mode);

  // 같은 stamp의 이미지·metadata로 에미터 상태를 보고 발행
  void OnSynchronized(
      const SplitStream& stream,
      const sensor_msgs::msg::Image::ConstSharedPtr& image,
      const realsense2_camera_msgs::msg::Metadata::ConstSharedPtr& metadata);
  static int GetEmitterMode(const std::string& json_data);

  int sync_queue_size_;
  std::vector<std::unique_ptr<SplitCamera>> cameras_;
};

}  // namespace xsting_vision

#endif  // XSTING_VISION_REALSENSE_SPLITTER_NODE_H_
