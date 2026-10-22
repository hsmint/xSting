#pragma once
#include <cstdint>
#include <string>
#include <rclcpp/qos.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
namespace xsting_vision {
struct TargetRules {
  std::string frame_id;
  int64_t max_age_ns{};
  double min_confidence{};
};
// Validate an already transformed detection; this function never substitutes latest TF.
std::string reject_reason(const vision_msgs::msg::Detection3D &, int64_t now_ns, const TargetRules &);
inline rclcpp::QoS detection_qos() { return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(); }
}
