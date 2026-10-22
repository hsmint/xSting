#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include "xsting_interfaces/msg/arm_target.hpp"
namespace xsting_robot {
struct CommandContext {
  std::string task_id, model_version, frame_id;
  uint64_t generation{}, input_id{};
  int64_t now_ns{}, input_stamp_ns{}, joint_stamp_ns{}, max_age_ns{};
  bool active{false};
  std::vector<std::string> joint_names;
  std::vector<double> positions, lower, upper, max_delta;
};
// Pure validation only. Caller must serialize cancellation and final dispatch.
std::string reject_reason(const xsting_interfaces::msg::ArmTarget &, const CommandContext &);
}
