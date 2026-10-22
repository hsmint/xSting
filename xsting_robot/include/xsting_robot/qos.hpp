#pragma once
#include <rclcpp/qos.hpp>
namespace xsting_robot {
// Q2: a single-shot input/target must not silently use a sensor best-effort profile.
inline rclcpp::QoS arm_qos() { return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(); }
}
