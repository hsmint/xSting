#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "xsting_walk/calib.h"
#include "xsting_walk/ros_param_util.h"

namespace xsting_walk {

namespace {

void BackupIfExists(const std::string& path) {
  std::error_code ec;
  if (std::filesystem::exists(path, ec)) {
    std::filesystem::copy_file(
        path, path + ".bak",
        std::filesystem::copy_options::overwrite_existing, ec);
  }
}

diagnostic_msgs::msg::KeyValue MakeKv(const std::string& k,
                                      const std::string& v) {
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = k;
  kv.value = v;
  return kv;
}

}  // namespace

// 다리 18축·팔 6축 관절 영점(offset), 회전 부호 점검(nudge/flip_sign) 및 보행 기립(stand) 검증 노드
class CalibJointsNode : public rclcpp::Node {
 public:
  CalibJointsNode() : Node("calib_joints_node") {
    calib_file_ = DeclareString(
        this, "calib_file", "src/xsting_walk/config/calibration.yaml");
    action_ = DeclareString(this, "action", "offset");
    pose_ = DeclareString(this, "pose", "boot");
    include_arm_ = DeclareBool(this, "include_arm", true);
    joint_idx_ = DeclareInt(this, "joint_index", 0);
    nudge_deg_ = DeclareDouble(this, "nudge_deg", 6.0);
    hold_sec_ = DeclareDouble(this, "hold_sec", 2.5);
    target_cnt_ = std::max(10, DeclareInt(this, "samples", 100));

    const auto joint_topic =
        DeclareString(this, "joint_topic", "/joint_states");
    const auto leg_cmd_topic =
        DeclareString(this, "leg_cmd_topic", "/leg_controller/commands");

    calib_ = Calib::MakeDefault();
    std::string err;
    if (std::filesystem::exists(calib_file_)) calib_.Load(calib_file_, &err);

    trial_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
        "/walk/calib_trial", 10);
    reload_cli_ = create_client<std_srvs::srv::Trigger>("/walk/reload_calib");
    disable_cli_ = create_client<std_srvs::srv::Trigger>("/walk/disable");
    enable_cli_ = create_client<std_srvs::srv::Trigger>("/walk/enable");

    // 부호 즉시 반전 모드(센서 대기 불필요)
    if (action_ == "flip_sign") {
      FlipSignAndExit();
      return;
    }

    leg_pub_ =
        create_publisher<std_msgs::msg::Float64MultiArray>(leg_cmd_topic, 10);
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr m) { OnMsg(*m); });

    if (action_ == "stand" || action_ == "nudge") {
      SetWalkNodeEnabled(false);
      t0_ = std::chrono::steady_clock::now();
      timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                 [this] { OnMotionTick(); });
      RCLCPP_INFO(get_logger(), "관절 동작 검증 시작 (action=%s, joint_idx=%d)",
                  action_.c_str(), joint_idx_);
    } else {
      RCLCPP_INFO(get_logger(),
                  "관절 영점 수집 시작 (pose=%s, samples=%d, topic=%s)",
                  pose_.c_str(), target_cnt_, joint_topic.c_str());
    }
  }

 private:
  double RefRad(const JointCalib& j) const {
    if (pose_ == "zero") return 0.0;
    if (pose_ == "stand") return j.stand_rad;
    return GetBootRad(j.name);
  }

  void PublishStatus(const std::string& msg,
                     const std::vector<diagnostic_msgs::msg::KeyValue>& kvs) {
    diagnostic_msgs::msg::DiagnosticStatus ds;
    ds.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    ds.name = "xsting_walk/calib_trial";
    ds.hardware_id = "calib_joints_node";
    ds.message = msg;
    ds.values = kvs;
    trial_pub_->publish(ds);
  }

  void TriggerReload() {
    if (reload_cli_->wait_for_service(std::chrono::milliseconds(250))) {
      auto fut = reload_cli_->async_send_request(
          std::make_shared<std_srvs::srv::Trigger::Request>());
      rclcpp::spin_until_future_complete(get_node_base_interface(), fut,
                                         std::chrono::milliseconds(400));
    }
  }

  void SetWalkNodeEnabled(bool en) {
    auto& cli = en ? enable_cli_ : disable_cli_;
    if (cli->wait_for_service(std::chrono::milliseconds(200))) {
      cli->async_send_request(
          std::make_shared<std_srvs::srv::Trigger::Request>());
    }
  }

  void FlipSignAndExit() {
    if (joint_idx_ < 0 || joint_idx_ >= kNumLegs) {
      RCLCPP_ERROR(get_logger(), "유효하지 않은 joint_index=%d (0..17 필요)",
                   joint_idx_);
      rclcpp::shutdown();
      return;
    }
    auto& j = calib_.legs[joint_idx_];
    j.dir = (j.dir >= 0.0) ? -1.0 : 1.0;
    const auto parent = std::filesystem::path(calib_file_).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    BackupIfExists(calib_file_);
    std::string err;
    if (calib_.Save(calib_file_, &err)) {
      RCLCPP_INFO(get_logger(), "관절 [%d] %s direction -> %.1f 저장 완료 (%s)",
                  joint_idx_, j.name.c_str(), j.dir, calib_file_.c_str());
      TriggerReload();
      std::this_thread::sleep_for(std::chrono::milliseconds(60));
      PublishStatus("관절 회전 부호 반전 저장 완료: " + j.name,
                    {MakeKv("01_관절", j.name),
                     MakeKv("02_신규방향", j.dir > 0.0 ? "+1.0" : "-1.0"),
                     MakeKv("03_저장파일", calib_file_)});
      std::this_thread::sleep_for(std::chrono::milliseconds(60));
    } else {
      RCLCPP_ERROR(get_logger(), "%s", err.c_str());
    }
    done_ = true;
    rclcpp::shutdown();
  }

  void OnMsg(const sensor_msgs::msg::JointState& msg) {
    if (done_) return;
    const size_t n = std::min(msg.name.size(), msg.position.size());
    if (n == 0) return;

    if (action_ == "stand" || action_ == "nudge") {
      if (!got_init_) {
        for (size_t i = 0; i < n; ++i) sum_[msg.name[i]] = msg.position[i];
        for (int i = 0; i < kNumLegs; ++i) {
          const auto& j = calib_.legs[i];
          init_hw_[i] = sum_.count(j.ros_name) ? sum_[j.ros_name]
                                               : j.ToHw(j.stand_rad);
        }
        t0_ = std::chrono::steady_clock::now();
        got_init_ = true;
      }
      return;
    }

    for (size_t i = 0; i < n; ++i) {
      sum_[msg.name[i]] += msg.position[i];
      cnt_[msg.name[i]] += 1;
    }
    ++frames_;
    if (frames_ % 20 == 0 || frames_ == target_cnt_) {
      PublishStatus("관절 영점 수집 중 (" + pose_ + ")",
                    {MakeKv("01_기준자세", pose_),
                     MakeKv("02_샘플", std::to_string(frames_) + " / " +
                                           std::to_string(target_cnt_))});
    }
    if (frames_ < target_cnt_) return;

    int leg_cnt = 0;
    double max_deg = 0.0;
    for (auto& j : calib_.legs) {
      if (cnt_[j.ros_name] > 0) {
        const double mean_q = sum_[j.ros_name] / cnt_[j.ros_name];
        j.offset_rad = mean_q - j.dir * RefRad(j);
        max_deg = std::max(max_deg, std::abs(j.offset_rad) * (180.0 / M_PI));
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

    const auto parent = std::filesystem::path(calib_file_).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    BackupIfExists(calib_file_);
    std::string err;
    if (calib_.Save(calib_file_, &err)) {
      RCLCPP_INFO(get_logger(), "관절 영점 보정 완료 (%d/18축) -> %s", leg_cnt,
                  calib_file_.c_str());
      TriggerReload();
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%.2f deg", max_deg);
      PublishStatus("관절 영점 보정 완료 (" + std::to_string(leg_cnt) + "/18축)",
                    {MakeKv("01_기준자세", pose_),
                     MakeKv("02_최대오프셋", buf),
                     MakeKv("03_저장파일", calib_file_)});
      std::this_thread::sleep_for(std::chrono::milliseconds(80));
    } else {
      RCLCPP_ERROR(get_logger(), "%s", err.c_str());
    }
    done_ = true;
    rclcpp::shutdown();
  }

  void OnMotionTick() {
    if (done_ || !got_init_) return;
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_)
            .count();

    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.resize(kNumLegs, 0.0);

    if (action_ == "stand") {
      constexpr double kRampSec = 2.5;
      const double r = std::clamp(elapsed / kRampSec, 0.0, 1.0);
      const double a = 0.5 * (1.0 - std::cos(M_PI * r));
      for (int i = 0; i < kNumLegs; ++i) {
        const double tgt = calib_.legs[i].ToHw(calib_.legs[i].stand_rad);
        cmd.data[i] = init_hw_[i] + a * (tgt - init_hw_[i]);
      }
      leg_pub_->publish(cmd);
      if (++motion_ticks_ % 15 == 0) {
        PublishStatus("표준 보행 기립 자세 전환 중",
                      {MakeKv("01_동작", "stand (yaw=+-30/0, hip=15, knee=-90)")});
      }
      if (elapsed >= kRampSec + hold_sec_) {
        RCLCPP_INFO(get_logger(),
                    "xsting_walk 표준 기립 자세 전환 완료 (yaw=+-30/0, hip=15, "
                    "knee=-90 deg)");
        PublishStatus("표준 보행 기립 자세 전환 완료",
                      {MakeKv("01_동작", "stand 완료")});
        SetWalkNodeEnabled(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        done_ = true;
        rclcpp::shutdown();
      }
      return;
    }

    if (action_ == "nudge") {
      constexpr double kNudgeDur = 2.0;
      const int idx = std::clamp(joint_idx_, 0, kNumLegs - 1);
      const double wave =
          std::sin(M_PI * std::clamp(elapsed / kNudgeDur, 0.0, 1.0));
      const double delta_sim = (nudge_deg_ * M_PI / 180.0) * wave;
      for (int i = 0; i < kNumLegs; ++i) {
        cmd.data[i] = init_hw_[i];
      }
      const double base_sim = calib_.legs[idx].ToSim(init_hw_[idx]);
      cmd.data[idx] = calib_.legs[idx].ToHw(base_sim + delta_sim);
      leg_pub_->publish(cmd);
      if (++motion_ticks_ % 15 == 0) {
        PublishStatus(
            "관절 부호 점검 왕복 중: " + calib_.legs[idx].name,
            {MakeKv("01_관절", "[" + std::to_string(idx) + "] " +
                                   calib_.legs[idx].name),
             MakeKv("02_기준양방향",
                    "yaw(+)=발 앞쪽, hip(+)=허벅지 위, knee(+)=발끝 위")});
      }
      if (elapsed >= kNudgeDur + 0.2) {
        RCLCPP_INFO(get_logger(), "관절 [%d] %s +%g deg 왕복 검증 완료", idx,
                    calib_.legs[idx].name.c_str(), nudge_deg_);
        PublishStatus("관절 부호 왕복 점검 완료: " + calib_.legs[idx].name,
                      {MakeKv("01_관절", calib_.legs[idx].name)});
        SetWalkNodeEnabled(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        done_ = true;
        rclcpp::shutdown();
      }
    }
  }

  std::string calib_file_;
  std::string action_;
  std::string pose_;
  bool include_arm_ = true;
  int joint_idx_ = 0;
  double nudge_deg_ = 6.0;
  double hold_sec_ = 2.5;
  int target_cnt_ = 100;
  int frames_ = 0;
  int motion_ticks_ = 0;
  bool got_init_ = false;
  bool done_ = false;

  Calib calib_;
  std::array<double, kNumLegs> init_hw_{};
  std::chrono::steady_clock::time_point t0_;
  std::unordered_map<std::string, double> sum_;
  std::unordered_map<std::string, int> cnt_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr
      trial_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reload_cli_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr disable_cli_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr enable_cli_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace xsting_walk

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<xsting_walk::CalibJointsNode>();
    if (rclcpp::ok()) rclcpp::spin(node);
    node.reset();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("calib_joints_node"),
                 "관절 보정 노드 실행 실패: %s", e.what());
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
