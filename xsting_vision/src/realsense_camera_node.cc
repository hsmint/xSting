#include "xsting_vision/realsense_camera_node.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iterator>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <cuda_buffer/cuda_buffer_api.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace xsting_vision {

// =============================================================================
// 생성자: YAML을 읽어 카메라를 모두 시작한 뒤 옵션 변경 콜백 등록
// =============================================================================
RealsenseCameraNode::RealsenseCameraNode(const rclcpp::NodeOptions& options)
    : Node("realsense_camera", AddYamlParameters(options)) {
  const std::vector<std::string> camera_names =
      declare_parameter<std::vector<std::string>>("camera_names",
                                                  std::vector<std::string>{});
  for (const std::string& camera_name : camera_names) {
    StartCamera(camera_name);
  }

  parameter_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) {
        return OnParametersChanged(parameters);
      });
}

// =============================================================================
// 소멸자: 모든 카메라 스트리밍 정지 후 CUDA stream 정리
// =============================================================================
RealsenseCameraNode::~RealsenseCameraNode() {
  for (const std::unique_ptr<RealsenseCamera>& camera : cameras_) {
    camera->pipeline.stop();
    cudaStreamSynchronize(camera->cuda_stream);
    if (camera->enc_params) {
      nvjpegEncoderParamsDestroy(camera->enc_params);
    }
    if (camera->enc_state) {
      nvjpegEncoderStateDestroy(camera->enc_state);
    }
    if (camera->jpeg_state) {
      nvjpegJpegStateDestroy(camera->jpeg_state);
    }
    if (camera->jpeg_handle) {
      nvjpegDestroy(camera->jpeg_handle);
    }
    cudaStreamDestroy(camera->cuda_stream);
  }
}

// =============================================================================
// YAML 파일을 직접 읽어 파라미터를 NodeOptions 앞쪽에 추가 (-p 값이 우선)
// =============================================================================
rclcpp::NodeOptions RealsenseCameraNode::AddYamlParameters(
    const rclcpp::NodeOptions& options) {
  // -p config_file:=<경로>가 있으면 그 파일, 없으면 설치된 기본 파일 사용
  std::string config_file_path =
      (ament_index_cpp::get_package_share_path("xsting_vision") / "config" /
       "realsense_camera.yaml")
          .string();
  for (const rclcpp::Parameter& parameter : options.parameter_overrides()) {
    if (parameter.get_name() == "config_file") {
      config_file_path = parameter.as_string();
    }
  }

  std::vector<rclcpp::Parameter> yaml_parameters;
  for (const auto& node_parameters :
       rclcpp::parameter_map_from_yaml_file(config_file_path)) {
    for (const rclcpp::Parameter& yaml_param : node_parameters.second) {
      bool overridden = false;
      for (const rclcpp::Parameter& override_param :
           options.parameter_overrides()) {
        if (override_param.get_name() == yaml_param.get_name()) {
          overridden = true;
          break;
        }
      }
      if (!overridden) {
        yaml_parameters.push_back(yaml_param);
      }
    }
  }

  rclcpp::NodeOptions options_with_yaml = options;
  std::vector<rclcpp::Parameter>& parameter_overrides =
      options_with_yaml.parameter_overrides();
  parameter_overrides.insert(parameter_overrides.begin(),
                             yaml_parameters.begin(), yaml_parameters.end());
  return options_with_yaml;
}

// =============================================================================
// YAML에서 카메라 한 대의 기본 설정 읽기 ("카메라이름.항목" 형식)
// =============================================================================
CameraSettings RealsenseCameraNode::LoadCameraSettings(
    const std::string& camera_name) {
  CameraSettings camera_settings;
  camera_settings.serial_number =
      declare_parameter<std::string>(camera_name + ".serial_number", "");
  camera_settings.color_width =
      declare_parameter<int>(camera_name + ".color_width", 1280);
  camera_settings.color_height =
      declare_parameter<int>(camera_name + ".color_height", 720);
  camera_settings.color_fps =
      declare_parameter<int>(camera_name + ".color_fps", 30);
  camera_settings.depth_width =
      declare_parameter<int>(camera_name + ".depth_width", 1280);
  camera_settings.depth_height =
      declare_parameter<int>(camera_name + ".depth_height", 720);
  camera_settings.depth_fps =
      declare_parameter<int>(camera_name + ".depth_fps", 30);
  camera_settings.enable_infra =
      declare_parameter<bool>(camera_name + ".enable_infra", true);
  camera_settings.compressed_fps =
      declare_parameter<double>(camera_name + ".compressed_fps", 10.0);
  camera_settings.jpeg_quality =
      declare_parameter<int>(camera_name + ".jpeg_quality", 75);
  return camera_settings;
}

// =============================================================================
// 카메라 한 대: CUDA stream → 스트림 구성 → 퍼블리셔 → 옵션 → 스트리밍 시작
// (color와 depth·IR은 해상도·fps를 따로 설정)
// =============================================================================
void RealsenseCameraNode::StartCamera(const std::string& camera_name) {
  auto camera = std::make_unique<RealsenseCamera>();
  camera->camera_name = camera_name;
  camera->camera_settings = LoadCameraSettings(camera_name);

  // 카메라 프레임을 GPU로 복사할 때 쓸 전용 CUDA stream
  CUDA_CHECK(cudaStreamCreateWithFlags(&camera->cuda_stream,
                                       cudaStreamNonBlocking));

  const CameraSettings& settings = camera->camera_settings;

  // 시리얼 번호로 카메라를 고르고 RGB, Depth, IR1, IR2 스트림 설정
  rs2::config config;
  if (!settings.serial_number.empty()) {
    config.enable_device(settings.serial_number);
  }
  config.enable_stream(RS2_STREAM_COLOR, settings.color_width,
                       settings.color_height, RS2_FORMAT_RGB8,
                       settings.color_fps);
  config.enable_stream(RS2_STREAM_DEPTH, settings.depth_width,
                       settings.depth_height, RS2_FORMAT_Z16,
                       settings.depth_fps);
  if (settings.enable_infra) {
    config.enable_stream(RS2_STREAM_INFRARED, 1, settings.depth_width,
                         settings.depth_height, RS2_FORMAT_Y8,
                         settings.depth_fps);
    config.enable_stream(RS2_STREAM_INFRARED, 2, settings.depth_width,
                         settings.depth_height, RS2_FORMAT_Y8,
                         settings.depth_fps);
  }

  // 카메라 시작 전에 스트림 정보를 먼저 받아 퍼블리셔 생성
  const rs2::pipeline_profile pipeline_profile =
      config.resolve(camera->pipeline);

  CreateStreamPublisher(*camera, "color",
                        pipeline_profile.get_stream(RS2_STREAM_COLOR));
  CreateStreamPublisher(*camera, "depth",
                        pipeline_profile.get_stream(RS2_STREAM_DEPTH));
  if (settings.enable_infra) {
    const rs2::stream_profile left_infrared_profile =
        pipeline_profile.get_stream(RS2_STREAM_INFRARED, 1);
    const rs2::stream_profile right_infrared_profile =
        pipeline_profile.get_stream(RS2_STREAM_INFRARED, 2);
    CreateStreamPublisher(*camera, "infra1", left_infrared_profile);
    CreateStreamPublisher(*camera, "infra2", right_infrared_profile);

    // IR2 camera_info의 P[3]에 스테레오 baseline 반영
    const rs2_extrinsics left_to_right_extrinsics =
        left_infrared_profile.get_extrinsics_to(right_infrared_profile);
    sensor_msgs::msg::CameraInfo& right_camera_info =
        camera->stream_publishers.at({RS2_STREAM_INFRARED, 2}).camera_info;
    right_camera_info.p[3] =
        right_camera_info.p[0] * left_to_right_extrinsics.translation[0];
  }

  // 에미터 상태 metadata 퍼블리셔 및 Foxglove용 GPU JPEG 압축 퍼블리셔
  camera->metadata_publisher =
      create_publisher<realsense2_camera_msgs::msg::Metadata>(
          camera_name + "/depth/metadata", rclcpp::SensorDataQoS());
  nvjpegCreateSimple(&camera->jpeg_handle);
  nvjpegJpegStateCreate(camera->jpeg_handle, &camera->jpeg_state);
  nvjpegEncoderStateCreate(camera->jpeg_handle, &camera->enc_state,
                           camera->cuda_stream);
  nvjpegEncoderParamsCreate(camera->jpeg_handle, &camera->enc_params,
                            camera->cuda_stream);
  const int quality = std::clamp(settings.jpeg_quality, 1, 100);
  nvjpegEncoderParamsSetQuality(camera->enc_params, quality,
                                camera->cuda_stream);
  nvjpegEncoderParamsSetSamplingFactors(camera->enc_params, NVJPEG_CSS_420,
                                        camera->cuda_stream);
  camera->compressed_pub =
      create_publisher<sensor_msgs::msg::CompressedImage>(
          camera_name + "/color/image_raw/compressed", rclcpp::SensorDataQoS());

  DeclareCameraOptions(camera_name, pipeline_profile.get_device());

  // 프레임 콜백과 함께 카메라 스트리밍 시작
  RealsenseCamera* camera_pointer = camera.get();
  camera->pipeline.start(
      config, [this, camera_pointer](const rs2::frame& frame) {
        OnFramesetReceived(*camera_pointer, frame);
      });
  RCLCPP_INFO(get_logger(),
              "%s (%s) started: color %dx%d @ %d fps, depth %dx%d @ %d fps",
              camera_name.c_str(), settings.serial_number.c_str(),
              settings.color_width, settings.color_height, settings.color_fps,
              settings.depth_width, settings.depth_height, settings.depth_fps);

  cameras_.push_back(std::move(camera));
}

// =============================================================================
// 스트림 하나의 Image, CameraInfo 퍼블리셔 생성
// =============================================================================
void RealsenseCameraNode::CreateStreamPublisher(
    RealsenseCamera& camera, const std::string& stream_name,
    const rs2::stream_profile& stream_profile) {
  const std::string image_topic_name =
      stream_profile.stream_type() == RS2_STREAM_COLOR ? "image_raw"
                                                       : "image_rect_raw";
  const std::string topic_prefix = camera.camera_name + "/" + stream_name;

  StreamPublisher stream_publisher;
  stream_publisher.frame_id =
      camera.camera_name + "_" + stream_name + "_frame";
  stream_publisher.camera_info =
      CreateCameraInfo(stream_profile.as<rs2::video_stream_profile>(),
                       stream_publisher.frame_id);
  stream_publisher.image_publisher = create_publisher<sensor_msgs::msg::Image>(
      topic_prefix + "/" + image_topic_name, rclcpp::SensorDataQoS());
  stream_publisher.camera_info_publisher =
      create_publisher<sensor_msgs::msg::CameraInfo>(
          topic_prefix + "/camera_info", rclcpp::SensorDataQoS());

  camera.stream_publishers.emplace(
      StreamKey{stream_profile.stream_type(), stream_profile.stream_index()},
      std::move(stream_publisher));
}

// =============================================================================
// 카메라 한 대의 Depth, Color 센서 옵션 목록 선언
// =============================================================================
void RealsenseCameraNode::DeclareCameraOptions(const std::string& camera_name,
                                               const rs2::device& device) {
  const std::string depth_prefix = camera_name + ".depth.";
  const std::string color_prefix = camera_name + ".color.";

  // Depth 센서 옵션
  const rs2::depth_sensor depth_sensor = device.first<rs2::depth_sensor>();
  DeclareCameraOption(depth_prefix + "enable_auto_exposure", depth_sensor,
                      RS2_OPTION_ENABLE_AUTO_EXPOSURE);
  DeclareCameraOption(depth_prefix + "exposure", depth_sensor,
                      RS2_OPTION_EXPOSURE);
  DeclareCameraOption(depth_prefix + "gain", depth_sensor, RS2_OPTION_GAIN);
  DeclareCameraOption(depth_prefix + "emitter_enabled", depth_sensor,
                      RS2_OPTION_EMITTER_ENABLED);
  DeclareCameraOption(depth_prefix + "emitter_on_off", depth_sensor,
                      RS2_OPTION_EMITTER_ON_OFF);
  DeclareCameraOption(depth_prefix + "laser_power", depth_sensor,
                      RS2_OPTION_LASER_POWER);
  DeclareCameraOption(depth_prefix + "inter_cam_sync_mode", depth_sensor,
                      RS2_OPTION_INTER_CAM_SYNC_MODE);

  // Color 센서 옵션
  const rs2::color_sensor color_sensor = device.first<rs2::color_sensor>();
  DeclareCameraOption(color_prefix + "enable_auto_exposure", color_sensor,
                      RS2_OPTION_ENABLE_AUTO_EXPOSURE);
  DeclareCameraOption(color_prefix + "exposure", color_sensor,
                      RS2_OPTION_EXPOSURE);
  DeclareCameraOption(color_prefix + "gain", color_sensor, RS2_OPTION_GAIN);
  DeclareCameraOption(color_prefix + "enable_auto_white_balance",
                      color_sensor, RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE);
  DeclareCameraOption(color_prefix + "white_balance", color_sensor,
                      RS2_OPTION_WHITE_BALANCE);
  DeclareCameraOption(color_prefix + "brightness", color_sensor,
                      RS2_OPTION_BRIGHTNESS);
  DeclareCameraOption(color_prefix + "contrast", color_sensor,
                      RS2_OPTION_CONTRAST);
  DeclareCameraOption(color_prefix + "saturation", color_sensor,
                      RS2_OPTION_SATURATION);
  DeclareCameraOption(color_prefix + "sharpness", color_sensor,
                      RS2_OPTION_SHARPNESS);
}

// =============================================================================
// 옵션 하나를 ROS 파라미터로 선언하고 YAML 값이 있으면 카메라에 적용
// =============================================================================
void RealsenseCameraNode::DeclareCameraOption(const std::string& parameter_name,
                                              const rs2::sensor& sensor,
                                              rs2_option option) {
  if (!sensor.supports(option)) {
    return;
  }

  // 카메라가 허용하는 값 범위를 파라미터 범위로 지정
  const rs2::option_range option_range = sensor.get_option_range(option);
  rcl_interfaces::msg::FloatingPointRange value_range;
  value_range.from_value = option_range.min;
  value_range.to_value = option_range.max;
  value_range.step = 0.0;

  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = sensor.get_option_description(option);
  descriptor.floating_point_range.push_back(value_range);

  // YAML 값과 현재 카메라 값이 다르면 카메라에 적용
  const double current_value = sensor.get_option(option);
  const double requested_value =
      declare_parameter<double>(parameter_name, current_value, descriptor);
  if (requested_value != current_value) {
    sensor.set_option(option, static_cast<float>(requested_value));
  }

  camera_options_.emplace(parameter_name, CameraOption{sensor, option});
}

// =============================================================================
// 실행 중 파라미터 변경 시 해당 카메라 옵션에 반영
// =============================================================================
rcl_interfaces::msg::SetParametersResult
RealsenseCameraNode::OnParametersChanged(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const rclcpp::Parameter& parameter : parameters) {
    const auto camera_option = camera_options_.find(parameter.get_name());
    if (camera_option == camera_options_.end()) {
      continue;
    }
    try {
      camera_option->second.sensor.set_option(
          camera_option->second.option,
          static_cast<float>(parameter.as_double()));
    } catch (const rs2::error& error) {
      result.successful = false;
      result.reason = parameter.get_name() + ": " + error.what();
    }
  }
  return result;
}

// =============================================================================
// 카메라 콜백: frameset 안의 모든 스트림과 depth metadata를 같은 시간으로 발행
// =============================================================================
void RealsenseCameraNode::OnFramesetReceived(RealsenseCamera& camera,
                                             const rs2::frame& frame) {
  const rs2::frameset frameset = frame.as<rs2::frameset>();
  if (!frameset) {
    return;
  }

  const rclcpp::Time stamp = now();
  for (const rs2::frame& stream_frame : frameset) {
    PublishFrame(camera, stream_frame.as<rs2::video_frame>(), stamp);
  }

  // color만 든 frameset에는 depth가 없음 (color와 depth fps가 다를 때)
  const rs2::depth_frame depth_frame = frameset.get_depth_frame();
  if (depth_frame) {
    PublishMetadata(camera, depth_frame, stamp);
  }
}

// =============================================================================
// 프레임 하나를 CUDA 버퍼 Image와 CameraInfo 메시지로 만들어 발행
// =============================================================================
void RealsenseCameraNode::PublishFrame(RealsenseCamera& camera,
                                       const rs2::video_frame& video_frame,
                                       const rclcpp::Time& stamp) {
  const rs2::stream_profile stream_profile = video_frame.get_profile();
  const auto stream_publisher = camera.stream_publishers.find(
      StreamKey{stream_profile.stream_type(), stream_profile.stream_index()});
  if (stream_publisher == camera.stream_publishers.end()) {
    return;
  }
  const StreamPublisher& publisher = stream_publisher->second;
  const bool need_comp =
      stream_profile.stream_type() == RS2_STREAM_COLOR &&
      camera.compressed_pub &&
      camera.compressed_pub->get_subscription_count() > 0 &&
      camera.camera_settings.compressed_fps > 0.0 &&
      (camera.last_comp_stamp.nanoseconds() == 0 ||
       std::abs((stamp - camera.last_comp_stamp).seconds()) >=
           1.0 / camera.camera_settings.compressed_fps);

  auto image = std::make_unique<sensor_msgs::msg::Image>();
  image->header.stamp = stamp;
  image->header.frame_id = publisher.frame_id;
  image->width = video_frame.get_width();
  image->height = video_frame.get_height();
  image->encoding = ToImageEncoding(stream_profile.format());
  image->step = video_frame.get_stride_in_bytes();

  // GPU 메모리 풀에서 CUDA 버퍼를 받아 프레임을 호스트 → GPU로 복사
  const size_t frame_size = video_frame.get_data_size();
  image->data = cuda_buffer_backend::allocate_buffer(frame_size);
  {
    cuda_buffer_backend::WriteHandle write_handle =
        cuda_buffer_backend::from_output_buffer(image->data,
                                                camera.cuda_stream);
    cuda_buffer_backend::to_buffer(video_frame.get_data(), frame_size,
                                   write_handle, camera.cuda_stream,
                                   cudaMemcpyHostToDevice);
    if (need_comp) {
      nvjpegImage_t enc_src{};
      enc_src.channel[0] = write_handle.get_ptr();
      enc_src.pitch[0] = image->step;
      nvjpegEncodeImage(camera.jpeg_handle, camera.enc_state,
                        camera.enc_params, &enc_src, NVJPEG_INPUT_RGBI,
                        image->width, image->height, camera.cuda_stream);
    }
  }
  publisher.image_publisher->publish(std::move(image));

  if (need_comp) {
    camera.last_comp_stamp = stamp;
    PublishCompressed(camera, publisher.frame_id, stamp);
  }

  // 미리 만든 CameraInfo에 시간만 바꿔 발행
  auto camera_info =
      std::make_unique<sensor_msgs::msg::CameraInfo>(publisher.camera_info);
  camera_info->header.stamp = stamp;
  publisher.camera_info_publisher->publish(std::move(camera_info));
}

// 인코딩된 GPU JPEG 비트스트림을 호스트로 꺼내 압축 컬러 토픽 발행
void RealsenseCameraNode::PublishCompressed(RealsenseCamera& camera,
                                            const std::string& frame_id,
                                            const rclcpp::Time& stamp) {
  cudaStreamSynchronize(camera.cuda_stream);
  size_t comp_len = 0;
  if (nvjpegEncodeRetrieveBitstream(camera.jpeg_handle, camera.enc_state,
                                    nullptr, &comp_len, camera.cuda_stream) !=
          NVJPEG_STATUS_SUCCESS ||
      comp_len == 0) {
    return;
  }

  auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
  msg->header.stamp = stamp;
  msg->header.frame_id = frame_id;
  msg->format = "jpeg";
  msg->data.resize(comp_len);
  if (nvjpegEncodeRetrieveBitstream(camera.jpeg_handle, camera.enc_state,
                                    msg->data.data(), &comp_len,
                                    camera.cuda_stream) ==
      NVJPEG_STATUS_SUCCESS) {
    cudaStreamSynchronize(camera.cuda_stream);
    msg->data.resize(comp_len);
    camera.compressed_pub->publish(std::move(msg));
  }
}

// =============================================================================
// depth 프레임 번호와 에미터 상태(0 꺼짐, 1 켜짐)를 JSON metadata로 발행
// =============================================================================
void RealsenseCameraNode::PublishMetadata(const RealsenseCamera& camera,
                                          const rs2::depth_frame& depth_frame,
                                          const rclcpp::Time& stamp) {
  auto metadata = std::make_unique<realsense2_camera_msgs::msg::Metadata>();
  metadata->header.stamp = stamp;
  metadata->header.frame_id = camera.camera_name + "_depth_frame";
  metadata->json_data =
      "{\"frame_number\":" + std::to_string(depth_frame.get_frame_number()) +
      ",\"frame_emitter_mode\":" +
      std::to_string(depth_frame.get_frame_metadata(
          RS2_FRAME_METADATA_FRAME_EMITTER_MODE)) +
      "}";
  camera.metadata_publisher->publish(std::move(metadata));
}

// =============================================================================
// 카메라 내부 파라미터(intrinsics)로 CameraInfo 생성
// =============================================================================
sensor_msgs::msg::CameraInfo RealsenseCameraNode::CreateCameraInfo(
    const rs2::video_stream_profile& video_profile,
    const std::string& frame_id) {
  const rs2_intrinsics intrinsics = video_profile.get_intrinsics();

  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.header.frame_id = frame_id;
  camera_info.width = intrinsics.width;
  camera_info.height = intrinsics.height;
  camera_info.distortion_model = "plumb_bob";
  camera_info.d.assign(std::begin(intrinsics.coeffs),
                       std::end(intrinsics.coeffs));
  camera_info.k = {intrinsics.fx, 0.0, intrinsics.ppx,
                   0.0, intrinsics.fy, intrinsics.ppy,
                   0.0, 0.0, 1.0};
  camera_info.r = {1.0, 0.0, 0.0,
                   0.0, 1.0, 0.0,
                   0.0, 0.0, 1.0};
  camera_info.p = {intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
                   0.0, intrinsics.fy, intrinsics.ppy, 0.0,
                   0.0, 0.0, 1.0, 0.0};
  return camera_info;
}

// =============================================================================
// RealSense 포맷을 ROS 이미지 encoding 문자열로 변환
// =============================================================================
std::string RealsenseCameraNode::ToImageEncoding(rs2_format format) {
  switch (format) {
    case RS2_FORMAT_RGB8:
      return sensor_msgs::image_encodings::RGB8;
    case RS2_FORMAT_Z16:
      return sensor_msgs::image_encodings::TYPE_16UC1;
    case RS2_FORMAT_Y8:
      return sensor_msgs::image_encodings::MONO8;
    default:
      return "";
  }
}

}  // namespace xsting_vision

// =============================================================================
// rclcpp component로 등록 (component container에 올려서 실행)
// =============================================================================
RCLCPP_COMPONENTS_REGISTER_NODE(xsting_vision::RealsenseCameraNode)
