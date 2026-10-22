#pragma once
#include <cstdint>
#include <optional>
namespace xsting_hardware {
// Host-side reference logic, not the firmware watchdog. Use a monotonic clock.
class LinkGuard {
 public:
  explicit LinkGuard(int64_t timeout_ns);
  // Called only after a validated handshake. Does not arm the robot.
  void connect(uint64_t boot_id);
  void disconnect();
  bool feedback(uint64_t boot_id, uint64_t sequence, int64_t now_ns);
  bool arm(int64_t now_ns);
  bool ready(int64_t now_ns);  // timeout latches disarmed until explicit arm()
 private:
  int64_t timeout_;
  bool connected_{false}, armed_{false};
  uint64_t boot_{};
  std::optional<uint64_t> sequence_;
  std::optional<int64_t> received_, checked_;
  bool fresh(int64_t now);
};
}
