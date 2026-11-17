#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace xsting_pick_place {

// 실기 arm_controller 또는 Isaac Sim JointState 토픽과 MoveIt 궤적 액션을 연결하는 브리지
class TrajBridge : public rclcpp::Node {
 public:
  using TrajAct = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<TrajAct>;

  TrajBridge() : Node("traj_bridge") {
    use_js_cmd_ = declare_parameter("use_joint_state_cmd", false);
    const std::string cmd_topic = declare_parameter(
        "cmd_topic",
        use_js_cmd_ ? "/isaac_joint_commands" : "/arm_controller/commands");
    const std::string state_topic = declare_parameter(
        "state_topic", use_js_cmd_ ? "/isaac_joint_states" : "/joint_states");

    // 모드에 맞춰 명령·상태 퍼블리셔 및 관절 구독기 생성
    if (use_js_cmd_) {
      js_cmd_pub_ =
          create_publisher<sensor_msgs::msg::JointState>(cmd_topic, 10);
      js_relay_pub_ =
          create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    } else {
      arr_cmd_pub_ =
          create_publisher<std_msgs::msg::Float64MultiArray>(cmd_topic, 10);
    }
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        state_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          OnJoints(msg);
        });

    // 팔과 그리퍼 FollowJointTrajectory 액션 서버 등록
    arm_srv_ = MakeServer("/arm_trajectory_controller/follow_joint_trajectory");
    grip_srv_ =
        MakeServer("/gripper_trajectory_controller/follow_joint_trajectory");
  }

 private:
  static constexpr std::array<const char*, 6> kJointOrder = {
      "arm_yaw",   "arm_shoulder", "arm_elbow",
      "arm_wrist", "arm_roll",     "finger_l"};
  static constexpr std::array<double, 6> kMinBnd = {
      -3.1415, -1.6580, -2.0070, -1.8325, -3.1415, -0.0018};
  static constexpr std::array<double, 6> kMaxBnd = {
      3.1415, 1.9198, 1.5707, 2.1816, 3.1415, 0.0250};

  // 공통 궤적 액션 서버 생성 헬퍼
  rclcpp_action::Server<TrajAct>::SharedPtr MakeServer(
      const std::string& name) {
    return rclcpp_action::create_server<TrajAct>(
        this, name,
        [](const rclcpp_action::GoalUUID&,
           std::shared_ptr<const TrajAct::Goal> goal) {
          return goal->trajectory.points.empty()
                     ? rclcpp_action::GoalResponse::REJECT
                     : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<GoalHandle>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle> gh) {
          std::thread(&TrajBridge::RunTraj, this, gh).detach();
        });
  }

  // 최초 수신된 관절 상태·속도·시뮬레이션 시각을 동기화하고 /joint_states 중계
  void OnJoints(const sensor_msgs::msg::JointState::SharedPtr msg) {
    auto out = *msg;
    const double raw_sim =
        static_cast<double>(msg->header.stamp.sec) +
        1e-9 * static_cast<double>(msg->header.stamp.nanosec);
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (raw_sim > 0.0 && raw_sim < sim_sec_ - 0.5) sim_sec_ = raw_sim;
      const double dt_sim =
          (sim_sec_ > 0.0 && raw_sim > sim_sec_) ? (raw_sim - sim_sec_) : 0.0;

      for (size_t i = 0; i < out.name.size() && i < out.position.size(); ++i) {
        for (size_t idx = 0; idx < 6; ++idx) {
          if (out.name[i] == kJointOrder[idx]) {
            double v = out.position[i];
            v = std::isfinite(v)
                    ? std::clamp(v, kMinBnd[idx], kMaxBnd[idx])
                    : cmd_[idx];
            const double prev_v = state_[idx];
            out.position[i] = v;
            state_[idx] = v;
            if (dt_sim > 1e-4) {
              vel_[idx] = 0.5 * vel_[idx] + 0.5 * ((v - prev_v) / dt_sim);
            } else if (i < out.velocity.size() &&
                       std::isfinite(out.velocity[i])) {
              vel_[idx] = out.velocity[i];
            }
            if (!synced_ && !cmd_set_[idx]) cmd_[idx] = v;
          }
        }
        if (out.name[i] == "finger_r") {
          const double v = out.position[i];
          out.position[i] =
              std::isfinite(v) ? std::clamp(v, -0.0018, 0.0250) : 0.0;
        }
      }
      if (raw_sim > sim_sec_) sim_sec_ = raw_sim;
      if (!out.position.empty()) synced_ = true;
    }

    // 현재 노드 시계 기준으로 단조 증가 타임스탬프를 부여해 TF_OLD_DATA 방지
    if (js_relay_pub_) {
      const rclcpp::Time cur = now();
      if (cur.nanoseconds() <= 0) return;
      {
        std::lock_guard<std::mutex> lock(mtx_);
        if (last_stamp_.nanoseconds() > 0 &&
            (cur - last_stamp_).nanoseconds() < -500000000LL) {
          last_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        }
        if (last_stamp_.nanoseconds() > 0 && cur <= last_stamp_) return;
        last_stamp_ = cur;
      }
      out.header.stamp = cur;
      js_relay_pub_->publish(out);
    }
  }

  // 보간된 관절 명령을 실기 배열 또는 양쪽 손가락 동기화 Isaac JointState로 발행
  void PublishCmd(const std::array<double, 6>& q) {
    if (use_js_cmd_) {
      sensor_msgs::msg::JointState js;
      js.header.stamp = now();
      js.name.assign(kJointOrder.begin(), kJointOrder.end());
      js.position.assign(q.begin(), q.end());
      js.name.push_back("finger_r");
      js.position.push_back(q[5]);
      js_cmd_pub_->publish(js);
      return;
    }
    std_msgs::msg::Float64MultiArray out;
    out.data.assign(q.begin(), q.end());
    arr_cmd_pub_->publish(out);
  }

  // 시뮬레이션 시간 동기화 보간 및 목표 관절 수렴 확인 후 완료 반환
  void RunTraj(const std::shared_ptr<GoalHandle> gh) {
    const auto& traj = gh->get_goal()->trajectory;
    auto res = std::make_shared<TrajAct::Result>();

    std::array<int, 6> map_idx = {-1, -1, -1, -1, -1, -1};
    for (size_t i = 0; i < traj.joint_names.size(); ++i) {
      for (size_t idx = 0; idx < 6; ++idx) {
        if (traj.joint_names[i] == kJointOrder[idx]) {
          map_idx[idx] = static_cast<int>(i);
        }
      }
    }

    std::array<double, 6> start_q = {};
    double prev_sim = 0.0;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      start_q = synced_ ? cmd_ : state_;
      if (synced_ && map_idx[5] >= 0) start_q[5] = state_[5];
      prev_sim = sim_sec_;
    }

    auto prev_wall = std::chrono::steady_clock::now();
    auto last_sim_adv = prev_wall;
    double elapsed = 0.0;
    double post_sim = 0.0;
    double post_wall = 0.0;
    rclcpp::WallRate rate(100.0);
    size_t seg = 0;

    while (rclcpp::ok()) {
      if (gh->is_canceling()) {
        gh->canceled(res);
        return;
      }

      const auto now_wall = std::chrono::steady_clock::now();
      const double dt_wall =
          std::chrono::duration<double>(now_wall - prev_wall).count();
      prev_wall = now_wall;

      double cur_sim = 0.0;
      {
        std::lock_guard<std::mutex> lock(mtx_);
        cur_sim = sim_sec_;
      }

      double dt_step = dt_wall;
      if (use_js_cmd_ && prev_sim > 0.0) {
        if (cur_sim > prev_sim) {
          dt_step = cur_sim - prev_sim;
          prev_sim = cur_sim;
          last_sim_adv = now_wall;
        } else if (cur_sim < prev_sim - 0.5) {
          prev_sim = cur_sim;
          last_sim_adv = now_wall;
          dt_step = 0.0;
        } else if (std::chrono::duration<double>(now_wall - last_sim_adv)
                       .count() <= 0.5) {
          dt_step = 0.0;
        }
      } else if (cur_sim > 0.0) {
        prev_sim = cur_sim;
        last_sim_adv = now_wall;
      }
      elapsed += dt_step;

      while (seg + 1 < traj.points.size() &&
             elapsed > rclcpp::Duration(traj.points[seg].time_from_start)
                           .seconds()) {
        ++seg;
      }

      const double t_prev =
          (seg == 0)
              ? 0.0
              : rclcpp::Duration(traj.points[seg - 1].time_from_start)
                    .seconds();
      const double t_cur = std::max(
          t_prev + 0.01,
          rclcpp::Duration(traj.points[seg].time_from_start).seconds());
      const double alpha =
          std::clamp((elapsed - t_prev) / (t_cur - t_prev), 0.0, 1.0);

      // 현재 세그먼트 보간 후 명령 발행 및 물리 수렴 여부 점검
      std::array<double, 6> cur_cmd = {};
      bool settled = true;
      bool is_synced = false;
      {
        std::lock_guard<std::mutex> lock(mtx_);
        is_synced = synced_;
        for (size_t idx = 0; idx < 6; ++idx) {
          const int m = map_idx[idx];
          if (m >= 0 &&
              static_cast<size_t>(m) < traj.points[seg].positions.size()) {
            const double p0 =
                (seg == 0)
                    ? start_q[idx]
                    : traj.points[seg - 1].positions[static_cast<size_t>(m)];
            const double p1 =
                traj.points[seg].positions[static_cast<size_t>(m)];
            const double blend =
                (traj.points.size() == 1)
                    ? (0.5 * (1.0 - std::cos(3.14159265358979323846 * alpha)))
                    : alpha;
            cmd_[idx] = p0 + blend * (p1 - p0);
            cmd_set_[idx] = true;
          }
          cur_cmd[idx] = cmd_[idx];
        }

        if (seg + 1 == traj.points.size() && alpha >= 1.0 && is_synced) {
          const double arm_pos_tol = use_js_cmd_ ? 0.055 : 0.035;
          const double arm_vel_tol = use_js_cmd_ ? 0.080 : 0.060;
          const double stall_wait = use_js_cmd_ ? post_sim : post_wall;
          for (size_t idx = 0; idx < 5; ++idx) {
            if (map_idx[idx] < 0) continue;
            if (std::abs(state_[idx] - cur_cmd[idx]) > arm_pos_tol ||
                std::abs(vel_[idx]) > arm_vel_tol) {
              settled = false;
            }
          }
          if (map_idx[5] >= 0) {
            const bool pos_ok = std::abs(state_[5] - cur_cmd[5]) < 0.0009;
            const bool stall_ok =
                (stall_wait >= 0.14) && (std::abs(vel_[5]) < 0.0040);
            if (!pos_ok && !stall_ok) settled = false;
          }
        }
      }
      PublishCmd(cur_cmd);

      if (seg + 1 == traj.points.size() && alpha >= 1.0) {
        if (!is_synced || settled) break;
        post_sim += dt_step;
        post_wall += dt_wall;
        if (use_js_cmd_ ? (post_sim >= 0.45 || post_wall >= 1.2)
                        : (post_wall >= 0.35)) {
          break;
        }
      }
      rate.sleep();
    }

    res->error_code = TrajAct::Result::SUCCESSFUL;
    gh->succeed(res);
  }

  bool use_js_cmd_ = false;
  std::mutex mtx_;
  bool synced_ = false;
  double sim_sec_ = 0.0;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  std::array<double, 6> state_ = {0.0, -1.0508, 1.0707, 0.0, 0.0, 0.0};
  std::array<double, 6> vel_ = {};
  std::array<double, 6> cmd_ = {0.0, -1.0508, 1.0707, 0.0, 0.0, 0.0};
  std::array<bool, 6> cmd_set_ = {};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arr_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_relay_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp_action::Server<TrajAct>::SharedPtr arm_srv_;
  rclcpp_action::Server<TrajAct>::SharedPtr grip_srv_;
};

}  // namespace xsting_pick_place

// 브리지 노드 실행
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xsting_pick_place::TrajBridge>());
  rclcpp::shutdown();
  return 0;
}
