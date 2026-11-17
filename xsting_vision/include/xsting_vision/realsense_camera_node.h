#ifndef XSTING_VISION_REALSENSE_CAMERA_NODE_H_
#define XSTING_VISION_REALSENSE_CAMERA_NODE_H_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <cuda_runtime.h>
#include <librealsense2/rs.hpp>
#include <nvjpeg.h>
#include <rclcpp/rclcpp.hpp>
#include <realsense2_camera_msgs/msg/metadata.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace xsting_vision {

using StreamKey = std::pair<rs2_stream, int>;

// =============================================================================
// 카메라 한 대의 기본 설정 (YAML 파일에서 읽음, depth 값은 IR에도 적용)
// =============================================================================
struct CameraSettings {
  std::string serial_number;
  int color_width;
  int color_height;
  int color_fps;
  int depth_width;
  int depth_height;
  int depth_fps;
  bool enable_infra;
  double compressed_fps;
  int jpeg_quality;
};

// =============================================================================
// 스트림 하나(color, depth, infra1, infra2)의 퍼블리셔 묶음
// =============================================================================
struct StreamPublisher {
  std::string frame_id;
  sensor_msgs::msg::CameraInfo camera_info;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_publisher;
};

// =============================================================================
// ROS 파라미터와 연결된 카메라 옵션 하나
// =============================================================================
struct CameraOption {
  rs2::sensor sensor;
  rs2_option option;
};

// =============================================================================
// 카메라 한 대의 파이프라인, GPU 복사용 CUDA stream, 스트림·metadata 퍼블리셔
// =============================================================================
struct RealsenseCamera {
  std::string camera_name;
  CameraSettings camera_settings;
  rs2::pipeline pipeline;
  cudaStream_t cuda_stream = nullptr;
  nvjpegHandle_t jpeg_handle = nullptr;
  nvjpegJpegState_t jpeg_state = nullptr;
  nvjpegEncoderState_t enc_state = nullptr;
  nvjpegEncoderParams_t enc_params = nullptr;
  rclcpp::Time last_comp_stamp{0, 0, RCL_ROS_TIME};
  std::map<StreamKey, StreamPublisher> stream_publishers;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
      compressed_pub;
  rclcpp::Publisher<realsense2_camera_msgs::msg::Metadata>::SharedPtr
      metadata_publisher;
};

// =============================================================================
// 노드 하나에서 RealSense 카메라 여러 대를 함께 실행 (rclcpp component)
// =============================================================================
class RealsenseCameraNode : public rclcpp::Node {
 public:
  explicit RealsenseCameraNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~RealsenseCameraNode() override;

 private:
  // YAML 설정 파일을 직접 읽어 NodeOptions에 추가
  static rclcpp::NodeOptions AddYamlParameters(
      const rclcpp::NodeOptions& options);

  // 카메라 설정 및 시작
  CameraSettings LoadCameraSettings(const std::string& camera_name);
  void StartCamera(const std::string& camera_name);
  void CreateStreamPublisher(RealsenseCamera& camera,
                             const std::string& stream_name,
                             const rs2::stream_profile& stream_profile);

  // 카메라 옵션 파라미터
  void DeclareCameraOptions(const std::string& camera_name,
                            const rs2::device& device);
  void DeclareCameraOption(const std::string& parameter_name,
                           const rs2::sensor& sensor, rs2_option option);
  rcl_interfaces::msg::SetParametersResult OnParametersChanged(
      const std::vector<rclcpp::Parameter>& parameters);

  // 프레임 수신 및 CUDA 버퍼(rosidl::Buffer)로 발행
  void OnFramesetReceived(RealsenseCamera& camera, const rs2::frame& frame);
  void PublishFrame(RealsenseCamera& camera,
                    const rs2::video_frame& video_frame,
                    const rclcpp::Time& stamp);
  void PublishCompressed(RealsenseCamera& camera, const std::string& frame_id,
                         const rclcpp::Time& stamp);

  // depth 프레임의 에미터 상태를 metadata(JSON)로 발행
  void PublishMetadata(const RealsenseCamera& camera,
                       const rs2::depth_frame& depth_frame,
                       const rclcpp::Time& stamp);

  // 메시지 변환
  static sensor_msgs::msg::CameraInfo CreateCameraInfo(
      const rs2::video_stream_profile& video_profile,
      const std::string& frame_id);
  static std::string ToImageEncoding(rs2_format format);

  std::vector<std::unique_ptr<RealsenseCamera>> cameras_;
  std::map<std::string, CameraOption> camera_options_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace xsting_vision

#endif  // XSTING_VISION_REALSENSE_CAMERA_NODE_H_
