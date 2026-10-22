#include "xsting_hardware/link_guard.hpp"
#include <stdexcept>
namespace xsting_hardware {
LinkGuard::LinkGuard(int64_t timeout):timeout_(timeout) { if (timeout<=0) throw std::invalid_argument("positive timeout required"); }
void LinkGuard::connect(uint64_t boot) { connected_=true; armed_=false; boot_=boot; sequence_.reset(); received_.reset(); checked_.reset(); }
void LinkGuard::disconnect() { connected_=false; armed_=false; sequence_.reset(); received_.reset(); checked_.reset(); }
bool LinkGuard::feedback(uint64_t boot, uint64_t seq, int64_t now) {
  if (!connected_ || boot!=boot_ || now<0 || (checked_ && now<*checked_) || (received_ && now<*received_)) { disconnect(); return false; }
  if (sequence_ && seq<=*sequence_) return false; // replay must not refresh the timeout
  if (received_ && now-*received_>=timeout_) armed_=false;
  sequence_=seq; received_=now; checked_=now; return true;
}
bool LinkGuard::fresh(int64_t now) {
  if (!connected_ || !received_ || now<0 || now<*received_ || (checked_ && now<*checked_)) { armed_=false; return false; }
  checked_=now;
  if (now-*received_>=timeout_) { armed_=false; return false; }
  return true;
}
bool LinkGuard::arm(int64_t now) { armed_=fresh(now); return armed_; }
bool LinkGuard::ready(int64_t now) { const bool ok=fresh(now); return ok && armed_; }
}
