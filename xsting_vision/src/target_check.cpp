#include "xsting_vision/target_check.hpp"
#include <cmath>
namespace xsting_vision {
std::string reject_reason(const vision_msgs::msg::Detection3D &d, int64_t now, const TargetRules &r) {
  if (r.frame_id.empty() || r.max_age_ns<=0 || !std::isfinite(r.min_confidence) || r.min_confidence<0 || r.min_confidence>1) return "config";
  if (d.header.frame_id != r.frame_id) return "frame";
  if (d.id.empty()) return "id";
  if (d.header.stamp.sec<0 || d.header.stamp.nanosec>=1000000000U || now<0) return "time";
  const int64_t stamp=static_cast<int64_t>(d.header.stamp.sec)*1000000000LL+d.header.stamp.nanosec;
  if (stamp>now || now-stamp>r.max_age_ns) return "stale";
  if (d.results.empty()) return "empty";
  bool accepted=false;
  for (const auto &h:d.results) {
    const auto &p=h.pose.pose.position; const auto &q=h.pose.pose.orientation;
    const double score=h.hypothesis.score;
    if (h.hypothesis.class_id.empty() || !std::isfinite(score)||score<0||score>1 ||
        !std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||
        !std::isfinite(q.x)||!std::isfinite(q.y)||!std::isfinite(q.z)||!std::isfinite(q.w)) return "invalid";
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if (!std::isfinite(norm)||std::abs(norm-1.0)>1e-3) return "rotation";
    accepted=accepted || score>=r.min_confidence;
  }
  return accepted? "" : "confidence";
}
}
