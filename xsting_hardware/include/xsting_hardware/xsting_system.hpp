#ifndef XSTING_HARDWARE_XSTING_SYSTEM_HPP_
#define XSTING_HARDWARE_XSTING_SYSTEM_HPP_

#include <array>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "xsting_hardware/opencr.hpp"

namespace xsting_hardware {

// 보드별 독립 USB 포트 통신을 매 주기 스레드 재생성 없이 병렬·파이프라인 실행하는 상주 워커
class BoardWorker {
 public:
  using Fn = void (*)(void*);

  BoardWorker() : thread_([this] { Loop(); }) {}
  ~BoardWorker() {
    {
      std::lock_guard<std::mutex> lk(mu_);
      stop_ = true;
    }
    cv_work_.notify_one();
    if (thread_.joinable()) thread_.join();
  }

  BoardWorker(const BoardWorker&) = delete;
  BoardWorker& operator=(const BoardWorker&) = delete;

  // 고정 2슬롯 큐에 작업 추가 (이전 Write 실행 중에도 블로킹 없이 Read 예약 가능)
  void Enqueue(void* ctx, Fn fn) {
    {
      std::unique_lock<std::mutex> lk(mu_);
      cv_done_.wait(lk, [this] { return count_ < 2; });
      const size_t tail = (head_ + count_) & 1;
      queue_[tail] = {ctx, fn};
      ++count_;
    }
    cv_work_.notify_one();
  }

  // 큐에 남은 작업과 현재 실행 중인 작업이 모두 끝날 때까지 대기
  void WaitIdle() {
    std::unique_lock<std::mutex> lk(mu_);
    cv_done_.wait(lk, [this] { return count_ == 0 && !running_; });
  }

 private:
  struct Task {
    void* ctx = nullptr;
    Fn fn = nullptr;
  };

  void Loop() {
    std::unique_lock<std::mutex> lk(mu_);
    while (true) {
      cv_work_.wait(lk, [this] { return count_ > 0 || stop_; });
      if (stop_ && count_ == 0) return;
      const Task t = queue_[head_];
      head_ = (head_ + 1) & 1;
      --count_;
      running_ = true;
      lk.unlock();
      t.fn(t.ctx);
      lk.lock();
      running_ = false;
      lk.unlock();
      cv_done_.notify_all();
      lk.lock();
    }
  }

  std::mutex mu_;
  std::condition_variable cv_work_;
  std::condition_variable cv_done_;
  Task queue_[2] = {};
  size_t head_ = 0;
  size_t count_ = 0;
  bool running_ = false;
  bool stop_ = false;
  std::thread thread_;
};

struct JointMap {
  std::string name;
  uint8_t idx = 0;
  double dir = 1.0;
  double off = 0.0;
};

// 다리 보드(200, 201)와 팔 보드(202)를 옵션으로 켜고 끄는 ros2_control 하드웨어
class XstingSystem : public hardware_interface::SystemInterface {
 public:
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params)
      override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& prev_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& prev_state) override;
  hardware_interface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& prev_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  std::vector<std::string> UsedPorts() const;
  bool CheckBattery(const OpenCrBase& board, const char* label);
  void CheckLegTorque(const LegBoard& board,
                      const std::array<JointMap, kLegJoints>& joints,
                      const LegBoard::Array& tx, const LegBoard::Array& pos,
                      std::array<uint16_t, kLegJoints>& limp_cnt,
                      const char* label);

  bool use_leg_ = true;
  bool use_arm_ = true;
  bool stand_ = true;
  bool reboot_leg_ = false;
  bool reboot_arm_ = false;

  uint32_t baud_ = kDefaultBaud;
  std::string front_port_ = "auto";
  std::string rear_port_ = "auto";
  std::string arm_port_ = "auto";

  // 다리 보드 상태 및 관절 매핑
  LegBoard front_{kFrontId};
  LegBoard rear_{kRearId};
  std::array<JointMap, kLegJoints> front_joints_ = {};
  std::array<JointMap, kLegJoints> rear_joints_ = {};
  LegBoard::Array front_start_ = {};
  LegBoard::Array rear_start_ = {};
  double stand_sec_ = 0.0;

  // 팔 보드 상태 및 관절 매핑
  ArmBoard arm_;
  std::array<JointMap, kArmJoints> arm_joints_ = {};
  std::vector<std::string> arm_mimics_;
  ArmBoard::Array arm_start_ = {};
  double arm_sec_ = 0.0;
  int16_t grip_cur_ = 250;
  bool holding_grip_ = false;
  double hold_pos_ = 0.0;
  uint16_t grip_stall_cnt_ = 0;

  // 비동기 송수신 버퍼 및 상태 플래그
  bool front_ok_ = false;
  bool rear_ok_ = false;
  bool arm_ok_ = false;
  bool has_tx_leg_ = false;
  LegBoard::Array tx_front_ = {};
  LegBoard::Array tx_rear_ = {};
  std::array<uint16_t, kLegJoints> front_limp_cnt_ = {};
  std::array<uint16_t, kLegJoints> rear_limp_cnt_ = {};
  ArmBoard::Array tx_arm_ = {};
  int16_t tx_grip_cur_ = 250;
  bool tx_write_grip_ = false;

  // 보드별 USB 통신 병렬·파이프라인 워커 (보드 객체보다 먼저 소멸되도록 마지막에 배치)
  BoardWorker front_worker_;
  BoardWorker rear_worker_;
  BoardWorker arm_worker_;
};

}  // namespace xsting_hardware

#endif  // XSTING_HARDWARE_XSTING_SYSTEM_HPP_
