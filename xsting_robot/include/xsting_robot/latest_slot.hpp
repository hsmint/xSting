#pragma once
#include <memory>
#include <mutex>
namespace xsting_robot {
// One waiting item, immutable ownership transferred to the worker on take().
template<class T> class LatestSlot {
 public:
  void put(T value) { auto p=std::make_shared<const T>(std::move(value)); std::lock_guard<std::mutex> lock(mutex_); next_=std::move(p); }
  std::shared_ptr<const T> take() { std::lock_guard<std::mutex> lock(mutex_); auto p=std::move(next_); return p; }
 private:
  std::mutex mutex_;
  std::shared_ptr<const T> next_;
};
}
