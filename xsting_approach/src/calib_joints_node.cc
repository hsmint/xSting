#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "xsting_approach/calib.h"

namespace xsting_approach {

// 다리 18축·팔 6축 관절 영점 오프셋(offset_rad) 실기 보정 노드
class CalibJointsNode : public rclcpp::Node {
 public:
  CalibJointsNode() : Node("calib_joints_node") {
    // 보정 파일 경로, 기준 자세(boot/stand/zero), 수집 샘플 수 선언
    calib_file_ = declare_parameter<std::string>(
        "calib_file", "src/xsting_approach/config/calibration.yaml");
    const auto topic =
        declare_parameter<std::string>("joint_topic", "/joint_states");
    pose_ = declare_parameter<std::string>("pose", "boot");
    include_arm_ = declare_parameter<bool>("include_arm", true);
    target_cnt_ =
        std::max(10, static_cast<int>(declare_parameter<int>("samples", 100)));

    // 기존 보정 파일 로드
    calib_ = Calib::MakeDefault();
    std::string err;
    if (std::filesystem::exists(calib_file_)) calib_.Load(calib_file_, &err);

    RCLCPP_INFO(get_logger(),
                "관절 영점 보정 시작 (pose=%s, samples=%d, topic=%s)",
                pose_.c_str(), target_cnt_, topic.c_str());

    // /joint_states 평균 수집 구독
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
        topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr m) { OnMsg(*m); });
  }

 private:
  // 기준 자세 모드에 따른 목표 시뮬레이션 각도 반환
  double RefRad(const JointCalib& j) const {
    if (pose_ == "zero") return 0.0;
    if (pose_ == "stand") return j.stand_rad;
    return GetBootRad(j.name);
  }

  // 관절 상태 누적 후 목표 샘플 도달 시 offset_rad 계산 및 저장
  void OnMsg(const sensor_msgs::msg::JointState& msg) {
    if (done_) return;
    const size_t n = std::min(msg.name.size(), msg.position.size());
    if (n == 0) return;

    for (size_t i = 0; i < n; ++i) {
      sum_[msg.name[i]] += msg.position[i];
      cnt_[msg.name[i]] += 1;
    }
    if (++frames_ < target_cnt_) return;

    // 다리 및 팔 관절 오프셋 갱신
    int leg_cnt = 0;
    for (auto& j : calib_.legs) {
      if (cnt_[j.ros_name] > 0) {
        const double mean_q = sum_[j.ros_name] / cnt_[j.ros_name];
        j.offset_rad = mean_q - j.dir * RefRad(j);
        ++leg_cnt;
      }
    }
    if (include_arm_) {
      for (auto& j : calib_.arm) {
        if (cnt_[j.ros_name] > 0) {
          const double mean_q = sum_[j.ros_name] / cnt_[j.ros_name];
          j.offset_rad = mean_q - j.dir * RefRad(j);
        }
      }
    }

    // YAML 보정 파일 기록 후 종료
    const auto parent = std::filesystem::path(calib_file_).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    std::string err;
    if (calib_.Save(calib_file_, &err)) {
      RCLCPP_INFO(get_logger(), "관절 보정 완료 (%d/18축) -> %s", leg_cnt,
                  calib_file_.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "%s", err.c_str());
    }
    done_ = true;
    rclcpp::shutdown();
  }

  std::string calib_file_;
  std::string pose_;
  bool include_arm_ = true;
  int target_cnt_ = 100;
  int frames_ = 0;
  bool done_ = false;
  Calib calib_;
  std::unordered_map<std::string, double> sum_;
  std::unordered_map<std::string, int> cnt_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

}  // namespace xsting_approach

// 관절 보정 노드 실행
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xsting_approach::CalibJointsNode>());
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
