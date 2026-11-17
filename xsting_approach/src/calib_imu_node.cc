#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "xsting_approach/calib.h"

namespace xsting_approach {

// IMU 장착 회전(1자세/2자세) 및 자이로 바이어스 실기 보정 노드
class CalibImuNode : public rclcpp::Node {
 public:
  CalibImuNode() : Node("calib_imu_node") {
    // 보정 파일, IMU 토픽, 2자세 여부 및 샘플 수 선언
    calib_file_ = declare_parameter<std::string>(
        "calib_file", "src/xsting_approach/config/calibration.yaml");
    const auto topic =
        declare_parameter<std::string>("imu_topic", "/imu_broadcaster/imu");
    two_pose_ = declare_parameter<bool>("two_pose", false);
    stand_legs_ = declare_parameter<bool>("stand_legs", true);
    yaw_deg_ = declare_parameter<double>("board_yaw_deg", 0.0);
    target_cnt_ =
        std::max(20, static_cast<int>(declare_parameter<int>("samples", 200)));

    // 기존 보정 파일 로드 및 WalkNet 기립 목표각 준비
    calib_ = Calib::MakeDefault();
    std::string err;
    if (std::filesystem::exists(calib_file_)) calib_.Load(calib_file_, &err);
    stand_cmd_ = BuildStandLegCmd(calib_);
    start_time_ = now();

    // 실기 구동 중일 때 다리를 WalkNet 서기 자세(base_z=21.3mm)로 부드럽게 전환
    if (stand_legs_) {
      leg_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
          "/leg_controller/commands", 10);
      joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states", 10,
          [this](const sensor_msgs::msg::JointState::SharedPtr m) {
            OnJoints(*m);
          });
      stand_timer_ = create_wall_timer(
          std::chrono::milliseconds(20), [this]() { StepStand(); });
    }

    // 2자세 보정 시 전방 기울임 측정 시작 트리거 서비스
    if (two_pose_) {
      srv_ = create_service<std_srvs::srv::Trigger>(
          "/calib_imu/capture_pose2",
          [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            if (stage_ != 1) {
              res->success = false;
              res->message = "1단계 수평 수집이 아직 끝나지 않았습니다";
              return;
            }
            stage_ = 2;
            cnt2_ = 0;
            grav2_.setZero();
            res->success = true;
            res->message = "2단계 전방 기울임 자세 수집 시작";
          });
    }

    RCLCPP_INFO(get_logger(),
                "IMU 보정 시작 (two_pose=%s, samples=%d, topic=%s)",
                two_pose_ ? "true" : "false", target_cnt_, topic.c_str());

    sub_ = create_subscription<sensor_msgs::msg::Imu>(
        topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr m) { OnImu(*m); });
  }

 private:
  // IMU 메시지의 쿼터니언 또는 가속도계로부터 센서 프레임 중력 방향 추출
  static Eigen::Vector3d ExtractGrav(const sensor_msgs::msg::Imu& msg) {
    Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x,
                         msg.orientation.y, msg.orientation.z);
    if (q.norm() > 1e-4) {
      q.normalize();
      return q.conjugate() * Eigen::Vector3d(0.0, 0.0, -1.0);
    }
    const Eigen::Vector3d acc(msg.linear_acceleration.x,
                              msg.linear_acceleration.y,
                              msg.linear_acceleration.z);
    if (acc.norm() > 1.0) return (-acc).normalized();
    return Eigen::Vector3d(0.0, 0.0, -1.0);
  }

  // IMU 샘플 누적 후 1자세 또는 2자세 보정 솔버 실행
  void OnImu(const sensor_msgs::msg::Imu& msg) {
    if (done_ || !StandReady()) return;
    const Eigen::Vector3d gyro(msg.angular_velocity.x, msg.angular_velocity.y,
                               msg.angular_velocity.z);
    const Eigen::Vector3d grav = ExtractGrav(msg);

    if (stage_ == 0) {
      gyro_sum_ += gyro;
      grav1_ += grav;
      if (++cnt1_ < target_cnt_) return;
      if (!two_pose_) {
        Finish(SolveImu1Pose(gyro_sum_ / cnt1_, grav1_ / cnt1_, yaw_deg_));
      } else {
        stage_ = 1;
        RCLCPP_INFO(get_logger(),
                    "1단계 완료 -> 차체를 앞으로 15~25도 숙인 뒤 "
                    "/calib_imu/capture_pose2 호출");
      }
      return;
    }

    if (stage_ == 2) {
      grav2_ += grav;
      if (++cnt2_ < target_cnt_) return;
      Finish(SolveImu2Pose(gyro_sum_ / cnt1_, grav1_ / cnt1_, grav2_ / cnt2_));
    }
  }

  // 산출된 IMU 보정값을 저장하고 노드 종료
  void Finish(const ImuCalib& solved) {
    calib_.imu = solved;
    const auto parent = std::filesystem::path(calib_file_).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    std::string err;
    if (calib_.Save(calib_file_, &err)) {
      RCLCPP_INFO(get_logger(), "IMU 보정 저장 완료 -> %s",
                  calib_file_.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "%s", err.c_str());
    }
    done_ = true;
    rclcpp::shutdown();
  }

  // 최초 관절 상태를 받아 서기 자세 보간 시작점으로 저장
  void OnJoints(const sensor_msgs::msg::JointState& msg) {
    if (has_joints_) return;
    init_cmd_ = stand_cmd_;
    for (int out_i = 0; out_i < kNumLegs; ++out_i) {
      const std::string& jname =
          static_cast<size_t>(out_i) < calib_.out_leg_order.size()
              ? calib_.out_leg_order[out_i]
              : calib_.legs[out_i].ros_name;
      for (size_t k = 0; k < msg.name.size() && k < msg.position.size(); ++k) {
        if (msg.name[k] == jname) {
          init_cmd_[out_i] = msg.position[k];
          break;
        }
      }
    }
    stand_start_ = now();
    has_joints_ = true;
  }

  // 1.5초 동안 WalkNet 서기 자세로 선형 보간 발행
  void StepStand() {
    if (!has_joints_ || !leg_pub_) return;
    const double t = (now() - stand_start_).seconds();
    const double a = std::clamp(t / 1.5, 0.0, 1.0);
    std_msgs::msg::Float64MultiArray out;
    out.data.resize(kNumLegs);
    for (int i = 0; i < kNumLegs; ++i) {
      out.data[i] = (1.0 - a) * init_cmd_[i] + a * stand_cmd_[i];
    }
    leg_pub_->publish(out);
  }

  // 서기 자세 전환 및 진동 감쇠(총 2.0초) 완료 여부 확인
  bool StandReady() const {
    if (!stand_legs_) return true;
    if (!has_joints_) return (now() - start_time_).seconds() > 0.6;
    return (now() - stand_start_).seconds() >= 2.0;
  }

  std::string calib_file_;
  bool two_pose_ = false;
  bool stand_legs_ = true;
  bool has_joints_ = false;
  double yaw_deg_ = 0.0;
  int target_cnt_ = 200;
  int stage_ = 0;
  int cnt1_ = 0;
  int cnt2_ = 0;
  bool done_ = false;

  rclcpp::Time start_time_;
  rclcpp::Time stand_start_;
  std::array<double, kNumLegs> init_cmd_{};
  std::array<double, kNumLegs> stand_cmd_{};
  Eigen::Vector3d gyro_sum_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d grav1_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d grav2_ = Eigen::Vector3d::Zero();
  Calib calib_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr stand_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
};

}  // namespace xsting_approach

// IMU 보정 노드 실행
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xsting_approach::CalibImuNode>());
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
