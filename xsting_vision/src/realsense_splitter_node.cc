#include "xsting_vision/realsense_splitter_node.h"

#include <utility>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace xsting_vision {

// =============================================================================
// 생성자: YAML 읽기 → 카메라별 metadata·이미지 구독과 퍼블리셔 생성
// =============================================================================
RealsenseSplitterNode::RealsenseSplitterNode(
    const rclcpp::NodeOptions& options)
    : Node("realsense_splitter", AddYamlParameters(options)) {
  sync_queue_size_ = declare_parameter<int>("sync_queue_size", 10);
  const std::vector<std::string> camera_names =
      declare_parameter<std::vector<std::string>>("camera_names",
                                                  std::vector<std::string>{});
  for (const std::string& camera_name : camera_names) {
    CreateCamera(camera_name);
  }
}

// =============================================================================
// YAML 파일을 직접 읽어 파라미터를 NodeOptions 앞쪽에 추가 (-p 값이 우선)
// =============================================================================
rclcpp::NodeOptions RealsenseSplitterNode::AddYamlParameters(
    const rclcpp::NodeOptions& options) {
  // -p config_file:=<경로>가 있으면 그 파일, 없으면 설치된 기본 파일 사용
  std::string config_file_path =
      (ament_index_cpp::get_package_share_path("xsting_vision") / "config" /
       "realsense_splitter.yaml")
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
// 카메라 한 대: metadata 구독 → infra1·infra2(에미터 꺼짐), depth(켜짐) 스트림
// =============================================================================
void RealsenseSplitterNode::CreateCamera(const std::string& camera_name) {
  auto camera = std::make_unique<SplitCamera>();
  camera->metadata_subscriber.subscribe(
      this, camera_name + "/depth/metadata", rclcpp::SensorDataQoS());

  CreateStream(*camera, camera_name, "infra1", "image_emitter_off", 0);
  CreateStream(*camera, camera_name, "infra2", "image_emitter_off", 0);
  CreateStream(*camera, camera_name, "depth", "image_emitter_on", 1);

  cameras_.push_back(std::move(camera));
}

// =============================================================================
// 스트림 하나: 이미지 구독 → metadata와 동기화 → 출력 퍼블리셔
// =============================================================================
void RealsenseSplitterNode::CreateStream(SplitCamera& camera,
                                         const std::string& camera_name,
                                         const std::string& stream_name,
                                         const std::string& output_topic_name,
                                         int emitter_mode) {
  auto stream = std::make_unique<SplitStream>();
  stream->emitter_mode = emitter_mode;
  const SplitStream* stream_pointer = stream.get();
  const std::string topic_prefix = camera_name + "/" + stream_name;

  // 이미지는 CPU로 바꾸지 않고 GPU 버퍼 그대로 받음
  rclcpp::SubscriptionOptions cuda_options;
  cuda_options.acceptable_buffer_backends = "cuda";
  stream->image_subscriber.subscribe(this, topic_prefix + "/image_rect_raw",
                                     rclcpp::SensorDataQoS(), cuda_options);

  // 같은 stamp의 이미지·metadata가 모이면 콜백 호출
  stream->synchronizer = std::make_unique<ImageMetadataSynchronizer>(
      sync_queue_size_, stream->image_subscriber, camera.metadata_subscriber);
  stream->synchronizer->registerCallback(
      [this, stream_pointer](
          const sensor_msgs::msg::Image::ConstSharedPtr& image,
          const realsense2_camera_msgs::msg::Metadata::ConstSharedPtr&
              metadata) { OnSynchronized(*stream_pointer, image, metadata); });

  stream->image_publisher = create_publisher<sensor_msgs::msg::Image>(
      topic_prefix + "/" + output_topic_name, rclcpp::SensorDataQoS());

  camera.streams.push_back(std::move(stream));
}

// =============================================================================
// 에미터 상태가 스트림 설정과 같을 때만 이미지 발행 (GPU 버퍼째 복사)
// =============================================================================
void RealsenseSplitterNode::OnSynchronized(
    const SplitStream& stream,
    const sensor_msgs::msg::Image::ConstSharedPtr& image,
    const realsense2_camera_msgs::msg::Metadata::ConstSharedPtr& metadata) {
  if (GetEmitterMode(metadata->json_data) != stream.emitter_mode) {
    return;
  }
  stream.image_publisher->publish(*image);
}

// =============================================================================
// JSON metadata에서 "frame_emitter_mode" 값 읽기
// =============================================================================
int RealsenseSplitterNode::GetEmitterMode(const std::string& json_data) {
  const std::string key = "\"frame_emitter_mode\":";
  return std::stoi(json_data.substr(json_data.find(key) + key.size()));
}

}  // namespace xsting_vision

// =============================================================================
// rclcpp component로 등록 (component container에 올려서 실행)
// =============================================================================
RCLCPP_COMPONENTS_REGISTER_NODE(xsting_vision::RealsenseSplitterNode)
