#include "xsting_vision/arducam_camera_node.h"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <utility>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <cuda_buffer/cuda_buffer_api.hpp>
#include <nppi_geometry_transforms.h>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace xsting_vision {

namespace {

constexpr int kFrameBufferCount = 4;

}  // namespace

// =============================================================================
// 생성자: YAML 읽기 → CUDA 준비 → 보정 맵 → 퍼블리셔 → 카메라 시작
// =============================================================================
ArducamCameraNode::ArducamCameraNode(const rclcpp::NodeOptions& options)
    : Node("arducam_camera", AddYamlParameters(options)) {
  camera_settings_ = LoadCameraSettings();
  LoadCalibration();

  cudaStreamCreateWithFlags(&cuda_stream_, cudaStreamNonBlocking);
  cudaMalloc(&decoded_image_,
             camera_settings_.image_width * camera_settings_.image_height * 3);
  cudaMalloc(&comp_img_, camera_settings_.compressed_width *
                             camera_settings_.compressed_height * 3);
  nvjpegCreateSimple(&jpeg_handle_);
  nvjpegJpegStateCreate(jpeg_handle_, &jpeg_state_);
  CreateEncoder();
  CreateNppContext();
  CreateUndistortMaps();

  CreatePublishers();
  OpenDevice();

  capture_timer_ = create_wall_timer(std::chrono::milliseconds(1),
                                     [this]() { OnCaptureTimer(); });

  const CameraSettings& settings = camera_settings_;
  RCLCPP_INFO(get_logger(), "%s (%s) started: %dx%d @ %d fps",
              settings.camera_name.c_str(), settings.device_path.c_str(),
              settings.image_width, settings.image_height,
              settings.frames_per_second);
}

// =============================================================================
// 소멸자: 캡처 중지 → 카메라 정지 → GPU 메모리, nvJPEG, CUDA stream 정리
// =============================================================================
ArducamCameraNode::~ArducamCameraNode() {
  capture_timer_->cancel();
  StopCamera();
  cudaStreamSynchronize(cuda_stream_);
  cudaFree(decoded_image_);
  cudaFree(comp_img_);
  cudaFree(undistort_map_x_);
  cudaFree(undistort_map_y_);
  nvjpegEncoderParamsDestroy(enc_params_);
  nvjpegEncoderStateDestroy(enc_state_);
  nvjpegJpegStateDestroy(jpeg_state_);
  nvjpegDestroy(jpeg_handle_);
  cudaStreamDestroy(cuda_stream_);
}

// =============================================================================
// YAML 파일을 직접 읽어 파라미터를 NodeOptions 앞쪽에 추가 (-p 값이 우선)
// =============================================================================
rclcpp::NodeOptions ArducamCameraNode::AddYamlParameters(
    const rclcpp::NodeOptions& options) {
  std::string config_file_path =
      (ament_index_cpp::get_package_share_path("xsting_vision") / "config" /
       "arducam_camera.yaml")
          .string();
  for (const rclcpp::Parameter& parameter : options.parameter_overrides()) {
    if (parameter.get_name() == "config_file") {
      config_file_path = parameter.as_string();
    }
  }

  std::vector<rclcpp::Parameter> yaml_parameters;
  for (const auto& node_parameters :
       rclcpp::parameter_map_from_yaml_file(config_file_path)) {
    yaml_parameters.insert(yaml_parameters.end(),
                           node_parameters.second.begin(),
                           node_parameters.second.end());
  }

  rclcpp::NodeOptions options_with_yaml = options;
  std::vector<rclcpp::Parameter>& parameter_overrides =
      options_with_yaml.parameter_overrides();
  parameter_overrides.insert(parameter_overrides.begin(),
                             yaml_parameters.begin(), yaml_parameters.end());
  return options_with_yaml;
}

// =============================================================================
// YAML에서 카메라 기본 설정 읽기
// =============================================================================
CameraSettings ArducamCameraNode::LoadCameraSettings() {
  CameraSettings camera_settings;
  camera_settings.camera_name =
      declare_parameter<std::string>("camera_name", "arducam");
  camera_settings.device_path =
      declare_parameter<std::string>("device_path", "/dev/video0");
  camera_settings.image_width = declare_parameter<int>("image_width", 1920);
  camera_settings.image_height = declare_parameter<int>("image_height", 1080);
  camera_settings.frames_per_second =
      declare_parameter<int>("frames_per_second", 30);
  camera_settings.compressed_width =
      std::max<int>(16, declare_parameter<int>("compressed_width", 640));
  camera_settings.compressed_height =
      std::max<int>(16, declare_parameter<int>("compressed_height", 360));
  camera_settings.compressed_fps =
      declare_parameter<double>("compressed_fps", 6.0);
  camera_settings.jpeg_quality =
      declare_parameter<int>("jpeg_quality", 60);
  return camera_settings;
}

// =============================================================================
// Image, CameraInfo 퍼블리셔 생성
// =============================================================================
void ArducamCameraNode::CreatePublishers() {
  const std::string& camera_name = camera_settings_.camera_name;

  frame_id_ = camera_name + "_frame";
  camera_info_.header.frame_id = frame_id_;
  camera_info_.width = camera_settings_.image_width;
  camera_info_.height = camera_settings_.image_height;
  image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      camera_name + "/image_rect", rclcpp::SensorDataQoS());
  compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      camera_name + "/image_rect/compressed", rclcpp::SensorDataQoS());
  camera_info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      camera_name + "/camera_info", rclcpp::SensorDataQoS());
}

// =============================================================================
// V4L2 장치 열기 → MJPEG 해상도·fps 설정 → mmap 버퍼 등록 → 스트리밍 시작
// =============================================================================
void ArducamCameraNode::OpenDevice() {
  const CameraSettings& settings = camera_settings_;

  device_fd_ = open(settings.device_path.c_str(), O_RDWR | O_NONBLOCK);

  v4l2_format format{};
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  format.fmt.pix.width = settings.image_width;
  format.fmt.pix.height = settings.image_height;
  format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  format.fmt.pix.field = V4L2_FIELD_ANY;
  ioctl(device_fd_, VIDIOC_S_FMT, &format);

  v4l2_streamparm stream_parameter{};
  stream_parameter.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  stream_parameter.parm.capture.timeperframe.numerator = 1;
  stream_parameter.parm.capture.timeperframe.denominator =
      settings.frames_per_second;
  ioctl(device_fd_, VIDIOC_S_PARM, &stream_parameter);

  v4l2_requestbuffers buffer_request{};
  buffer_request.count = kFrameBufferCount;
  buffer_request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buffer_request.memory = V4L2_MEMORY_MMAP;
  ioctl(device_fd_, VIDIOC_REQBUFS, &buffer_request);

  for (uint32_t index = 0; index < buffer_request.count; ++index) {
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = index;
    ioctl(device_fd_, VIDIOC_QUERYBUF, &buffer);

    FrameBuffer frame_buffer;
    frame_buffer.size = buffer.length;
    frame_buffer.data = mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE,
                             MAP_SHARED, device_fd_, buffer.m.offset);
    frame_buffers_.push_back(frame_buffer);

    ioctl(device_fd_, VIDIOC_QBUF, &buffer);
  }

  v4l2_buf_type stream_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  ioctl(device_fd_, VIDIOC_STREAMON, &stream_type);
}

// =============================================================================
// 스트리밍 정지 → mmap 해제 → 장치 닫기
// =============================================================================
void ArducamCameraNode::StopCamera() {
  v4l2_buf_type stream_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  ioctl(device_fd_, VIDIOC_STREAMOFF, &stream_type);
  for (const FrameBuffer& frame_buffer : frame_buffers_) {
    munmap(frame_buffer.data, frame_buffer.size);
  }
  frame_buffers_.clear();
  close(device_fd_);
  device_fd_ = -1;
}

// =============================================================================
// YAML에서 캘리브레이션(D·K·R·P)을 읽어 CameraInfo 채우기
// =============================================================================
void ArducamCameraNode::LoadCalibration() {
  camera_info_.distortion_model =
      declare_parameter<std::string>("distortion_model");
  camera_info_.d =
      declare_parameter<std::vector<double>>("distortion_coefficients");

  // 행렬 파라미터를 CameraInfo 배열로 복사
  auto read_matrix = [this](const std::string& name, double* matrix) {
    const std::vector<double> values =
        declare_parameter<std::vector<double>>(name);
    std::copy(values.begin(), values.end(), matrix);
  };
  read_matrix("camera_matrix", camera_info_.k.data());
  read_matrix("rectification_matrix", camera_info_.r.data());
  read_matrix("projection_matrix", camera_info_.p.data());
}

// =============================================================================
// NPP 함수가 노드의 CUDA stream에서 돌도록 stream context 채우기
// =============================================================================
void ArducamCameraNode::CreateNppContext() {
  cudaGetDevice(&npp_context_.nCudaDeviceId);
  cudaDeviceProp device{};
  cudaGetDeviceProperties(&device, npp_context_.nCudaDeviceId);

  npp_context_.hStream = cuda_stream_;
  npp_context_.nMultiProcessorCount = device.multiProcessorCount;
  npp_context_.nMaxThreadsPerMultiProcessor =
      device.maxThreadsPerMultiProcessor;
  npp_context_.nMaxThreadsPerBlock = device.maxThreadsPerBlock;
  npp_context_.nSharedMemPerBlock = device.sharedMemPerBlock;
  npp_context_.nCudaDevAttrComputeCapabilityMajor = device.major;
  npp_context_.nCudaDevAttrComputeCapabilityMinor = device.minor;
  cudaStreamGetFlags(cuda_stream_, &npp_context_.nStreamFlags);
}

// GPU JPEG 압축 인코더 상태 및 화질·색차 서브샘플링(4:2:0) 설정
void ArducamCameraNode::CreateEncoder() {
  const int quality = std::clamp(camera_settings_.jpeg_quality, 1, 100);
  nvjpegEncoderStateCreate(jpeg_handle_, &enc_state_, cuda_stream_);
  nvjpegEncoderParamsCreate(jpeg_handle_, &enc_params_, cuda_stream_);
  nvjpegEncoderParamsSetQuality(enc_params_, quality, cuda_stream_);
  nvjpegEncoderParamsSetSamplingFactors(enc_params_, NVJPEG_CSS_420,
                                        cuda_stream_);
}

// =============================================================================
// 보정 영상 픽셀마다 원본 좌표(plumb_bob)를 계산해 GPU 보정 맵 만들기
// =============================================================================
void ArducamCameraNode::CreateUndistortMaps() {
  const int width = camera_settings_.image_width;
  const int height = camera_settings_.image_height;
  const auto& camera_matrix = camera_info_.k;
  const auto& projection = camera_info_.p;
  const double k1 = camera_info_.d[0];
  const double k2 = camera_info_.d[1];
  const double p1 = camera_info_.d[2];
  const double p2 = camera_info_.d[3];
  const double k3 = camera_info_.d[4];

  std::vector<float> map_x(width * height);
  std::vector<float> map_y(width * height);
  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      // 보정 영상 픽셀(P) → 정규화 좌표 (단안 카메라라 R은 단위 행렬)
      const double x = (u - projection[2]) / projection[0];
      const double y = (v - projection[6]) / projection[5];

      // plumb_bob 왜곡 적용 → 원본 영상 픽셀(K)
      const double r2 = x * x + y * y;
      const double radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
      const double distorted_x =
          x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
      const double distorted_y =
          y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
      map_x[v * width + u] = camera_matrix[0] * distorted_x + camera_matrix[2];
      map_y[v * width + u] = camera_matrix[4] * distorted_y + camera_matrix[5];
    }
  }

  // 맵을 GPU 메모리로 복사
  const size_t map_bytes = map_x.size() * sizeof(float);
  cudaMalloc(&undistort_map_x_, map_bytes);
  cudaMalloc(&undistort_map_y_, map_bytes);
  cudaMemcpy(undistort_map_x_, map_x.data(), map_bytes,
             cudaMemcpyHostToDevice);
  cudaMemcpy(undistort_map_y_, map_y.data(), map_bytes,
             cudaMemcpyHostToDevice);
}

// =============================================================================
// 캡처 타이머: 프레임이 올 때까지 대기 → 꺼내서 발행 → 버퍼 반납
// =============================================================================
void ArducamCameraNode::OnCaptureTimer() {
  pollfd poll_fd{device_fd_, POLLIN, 0};
  const int timeout_ms = 2000 / camera_settings_.frames_per_second;
  if (poll(&poll_fd, 1, timeout_ms) <= 0) {
    return;
  }

  v4l2_buffer buffer{};
  buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buffer.memory = V4L2_MEMORY_MMAP;
  ioctl(device_fd_, VIDIOC_DQBUF, &buffer);

  PublishFrame(static_cast<const uint8_t*>(frame_buffers_[buffer.index].data),
               buffer.bytesused, now());
  cudaStreamSynchronize(cuda_stream_);

  ioctl(device_fd_, VIDIOC_QBUF, &buffer);
}

// =============================================================================
// JPEG을 GPU에서 rgb8로 디코딩·왜곡 보정해 CUDA 버퍼 Image와 CameraInfo 발행
// =============================================================================
void ArducamCameraNode::PublishFrame(const uint8_t* jpeg_data,
                                     size_t jpeg_size,
                                     const rclcpp::Time& stamp) {
  const int width = camera_settings_.image_width;
  const int height = camera_settings_.image_height;
  const bool need_comp =
      compressed_pub_->get_subscription_count() > 0 &&
      camera_settings_.compressed_fps > 0.0 &&
      (last_comp_stamp_.nanoseconds() == 0 ||
       std::abs((stamp - last_comp_stamp_).seconds()) >=
           1.0 / camera_settings_.compressed_fps);

  auto image = std::make_unique<sensor_msgs::msg::Image>();
  image->header.stamp = stamp;
  image->header.frame_id = frame_id_;
  image->width = width;
  image->height = height;
  image->encoding = sensor_msgs::image_encodings::RGB8;
  image->step = width * 3;

  // JPEG → 디코딩 버퍼
  nvjpegImage_t decoded{};
  decoded.channel[0] = decoded_image_;
  decoded.pitch[0] = image->step;
  nvjpegDecode(jpeg_handle_, jpeg_state_, jpeg_data, jpeg_size,
               NVJPEG_OUTPUT_RGBI, &decoded, cuda_stream_);

  // 디코딩 버퍼 → 왜곡 보정 → CUDA 버퍼 (원본 밖 픽셀은 검정)
  image->data = cuda_buffer_backend::allocate_buffer(image->step * height);
  {
    cuda_buffer_backend::WriteHandle write_handle =
        cuda_buffer_backend::from_output_buffer(image->data, cuda_stream_);
    const NppiSize size{width, height};
    const NppiRect roi{0, 0, width, height};
    const int map_step = width * sizeof(float);
    cudaMemsetAsync(write_handle.get_ptr(), 0, image->step * height,
                    cuda_stream_);
    nppiRemap_8u_C3R_Ctx(decoded_image_, size, image->step, roi,
                         undistort_map_x_, map_step, undistort_map_y_,
                         map_step, write_handle.get_ptr(), image->step, size,
                         NPPI_INTER_LINEAR, npp_context_);

    // 압축 토픽 구독자가 있으면 GPU에서 축소 후 JPEG 인코딩
    if (need_comp) {
      const int comp_w = camera_settings_.compressed_width;
      const int comp_h = camera_settings_.compressed_height;
      nvjpegImage_t enc_src{};
      if (comp_w == width && comp_h == height) {
        enc_src.channel[0] = write_handle.get_ptr();
        enc_src.pitch[0] = image->step;
      } else {
        const NppiSize dst_size{comp_w, comp_h};
        const NppiRect dst_roi{0, 0, comp_w, comp_h};
        const int dst_step = comp_w * 3;
        const int interp = (comp_w < width && comp_h < height)
                               ? NPPI_INTER_SUPER
                               : NPPI_INTER_LINEAR;
        nppiResize_8u_C3R_Ctx(write_handle.get_ptr(), image->step, size, roi,
                              comp_img_, dst_step, dst_size, dst_roi, interp,
                              npp_context_);
        enc_src.channel[0] = comp_img_;
        enc_src.pitch[0] = dst_step;
      }
      nvjpegEncodeImage(jpeg_handle_, enc_state_, enc_params_, &enc_src,
                        NVJPEG_INPUT_RGBI, comp_w, comp_h, cuda_stream_);
    }
  }
  image_publisher_->publish(std::move(image));

  if (need_comp) {
    last_comp_stamp_ = stamp;
    PublishCompressed(stamp);
  }

  auto camera_info =
      std::make_unique<sensor_msgs::msg::CameraInfo>(camera_info_);
  camera_info->header.stamp = stamp;
  camera_info_publisher_->publish(std::move(camera_info));
}

// 인코딩된 GPU JPEG 비트스트림을 호스트로 꺼내 압축 토픽 발행
void ArducamCameraNode::PublishCompressed(const rclcpp::Time& stamp) {
  cudaStreamSynchronize(cuda_stream_);
  size_t comp_len = 0;
  if (nvjpegEncodeRetrieveBitstream(jpeg_handle_, enc_state_, nullptr,
                                    &comp_len, cuda_stream_) !=
          NVJPEG_STATUS_SUCCESS ||
      comp_len == 0) {
    return;
  }

  auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
  msg->header.stamp = stamp;
  msg->header.frame_id = frame_id_;
  msg->format = "jpeg";
  msg->data.resize(comp_len);
  if (nvjpegEncodeRetrieveBitstream(jpeg_handle_, enc_state_, msg->data.data(),
                                    &comp_len, cuda_stream_) ==
      NVJPEG_STATUS_SUCCESS) {
    cudaStreamSynchronize(cuda_stream_);
    msg->data.resize(comp_len);
    compressed_pub_->publish(std::move(msg));
  }
}

}  // namespace xsting_vision

// =============================================================================
// rclcpp component로 등록 (component container에 올려서 실행)
// =============================================================================
RCLCPP_COMPONENTS_REGISTER_NODE(xsting_vision::ArducamCameraNode)
