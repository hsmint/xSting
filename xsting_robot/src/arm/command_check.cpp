#include "xsting_robot/command_check.hpp"
#include <algorithm>
#include <cmath>
#include <set>
namespace xsting_robot {
namespace {
int64_t ns(const builtin_interfaces::msg::Time &t) {
  return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
}
bool valid_time(const builtin_interfaces::msg::Time &t) { return t.sec >= 0 && t.nanosec < 1000000000U; }
bool fresh(int64_t stamp, const CommandContext &c) {
  return stamp >= 0 && stamp <= c.now_ns && c.now_ns - stamp <= c.max_age_ns;
}
}
std::string reject_reason(const xsting_interfaces::msg::ArmTarget &t, const CommandContext &c) {
  if (!c.active) return "inactive";
  if (c.task_id.empty() || t.task_id != c.task_id || t.generation != c.generation || t.input_id != c.input_id) return "identity";
  if (c.model_version.empty() || t.model_version != c.model_version) return "model";
  if (c.frame_id.empty() || t.header.frame_id != c.frame_id) return "frame";
  if (c.max_age_ns <= 0 || c.now_ns < 0 || !valid_time(t.header.stamp) || !valid_time(t.expires_at)) return "time";
  if (!fresh(ns(t.header.stamp), c) || ns(t.expires_at) <= c.now_ns ||
      !fresh(c.input_stamp_ns,c) || !fresh(c.joint_stamp_ns,c) || ns(t.header.stamp) < c.input_stamp_ns) return "stale";
  const auto n=c.joint_names.size();
  if (!n || t.joint_names!=c.joint_names || t.delta_positions.size()!=n || c.positions.size()!=n ||
      c.lower.size()!=n || c.upper.size()!=n || c.max_delta.size()!=n) return "shape";
  if (std::set<std::string>(c.joint_names.begin(),c.joint_names.end()).size()!=n ||
      std::any_of(c.joint_names.begin(),c.joint_names.end(),[](const auto &x){return x.empty();})) return "names";
  for (size_t i=0;i<n;++i) {
    const double q=c.positions[i], d=t.delta_positions[i], goal=q+d;
    if (!std::isfinite(q)||!std::isfinite(d)||!std::isfinite(goal)||!std::isfinite(c.lower[i])||
        !std::isfinite(c.upper[i])||!std::isfinite(c.max_delta[i])) return "nonfinite";
    if (c.lower[i]>=c.upper[i] || c.max_delta[i]<=0) return "limits_config";
    if (std::abs(d)>c.max_delta[i] || q<c.lower[i] || q>c.upper[i] || goal<c.lower[i] || goal>c.upper[i]) return "limits";
  }
  return {};
}
}
