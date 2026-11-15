#ifndef XSTING_VISION_ARDUCAM_CAMERA_NODE_H_
#define XSTING_VISION_ARDUCAM_CAMERA_NODE_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <cuda_runtime.h>
#include <nppdefs.h>
#include <nvjpeg.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace xsting_vision {

// =============================================================================
// 카메라 기본 설정 (YAML 파일에서 읽음)
// =============================================================================
struct CameraSettings {
  std::string camera_name;
  std::string device_path;
  int image_width;
  int image_height;
  int frames_per_second;
  int compressed_width;
  int compressed_height;
  double compressed_fps;
  int jpeg_quality;
};

// =============================================================================
// V4L2 드라이버와 공유하는 MJPEG 프레임 버퍼 하나
// =============================================================================
struct FrameBuffer {
  void* data = nullptr;
  size_t size = 0;
};

// =============================================================================
// Arducam UC-684 MJPEG을 GPU에서 디코딩·왜곡 보정해 CUDA 버퍼로 발행
// =============================================================================
class ArducamCameraNode : public rclcpp::Node {
 public:
  explicit ArducamCameraNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ArducamCameraNode() override;

 private:
  // YAML 설정 파일을 직접 읽어 NodeOptions에 추가
  static rclcpp::NodeOptions AddYamlParameters(
      const rclcpp::NodeOptions& options);

  // 카메라 설정 및 시작, 정지
  CameraSettings LoadCameraSettings();
  void CreatePublishers();
  void OpenDevice();
  void StopCamera();

  // 캘리브레이션 읽기, GPU 왜곡 보정 준비
  void LoadCalibration();
  void CreateNppContext();
  void CreateEncoder();
  void CreateUndistortMaps();

  // 프레임 수신, GPU 디코딩·왜곡 보정 후 CUDA 버퍼(rosidl::Buffer)로 발행
  void OnCaptureTimer();
  void PublishFrame(const uint8_t* jpeg_data, size_t jpeg_size,
                    const rclcpp::Time& stamp);
  void PublishCompressed(const rclcpp::Time& stamp);

  CameraSettings camera_settings_;
  std::string frame_id_;
  sensor_msgs::msg::CameraInfo camera_info_;

  int device_fd_ = -1;
  std::vector<FrameBuffer> frame_buffers_;

  cudaStream_t cuda_stream_ = nullptr;
  nvjpegHandle_t jpeg_handle_ = nullptr;
  nvjpegJpegState_t jpeg_state_ = nullptr;
  nvjpegEncoderState_t enc_state_ = nullptr;
  nvjpegEncoderParams_t enc_params_ = nullptr;
  NppStreamContext npp_context_{};
  uint8_t* decoded_image_ = nullptr;
  uint8_t* comp_img_ = nullptr;
  float* undistort_map_x_ = nullptr;
  float* undistort_map_y_ = nullptr;
  rclcpp::Time last_comp_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
      compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_publisher_;
};

}  // namespace xsting_vision

#endif  // XSTING_VISION_ARDUCAM_CAMERA_NODE_H_
