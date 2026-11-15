#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
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

std::string Fmt3(double x, double y, double z, const char* fmt = "%+.5f") {
  char b1[32], b2[32], b3[32];
  std::snprintf(b1, sizeof(b1), fmt, x);
  std::snprintf(b2, sizeof(b2), fmt, y);
  std::snprintf(b3, sizeof(b3), fmt, z);
  return std::string("[") + b1 + ", " + b2 + ", " + b3 + "]";
}

diagnostic_msgs::msg::KeyValue MakeKv(const std::string& k,
                                      const std::string& v) {
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = k;
  kv.value = v;
  return kv;
}

}  // namespace

// IMU 장착 수평 회전(1자세/2자세), 자이로 영점 및 가속도계(9.81 m/s^2) 바이어스 보정 노드
class CalibImuNode : public rclcpp::Node {
 public:
  CalibImuNode() : Node("calib_imu_node") {
    calib_file_ = DeclareString(
        this, "calib_file", "src/xsting_walk/config/calibration.yaml");
    const auto topic =
        DeclareString(this, "imu_topic", "/imu_broadcaster/imu");
    two_pose_ = DeclareBool(this, "two_pose", false);
    yaw_deg_ = DeclareDouble(this, "board_yaw_deg", 0.0);
    target_cnt_ = std::max(20, DeclareInt(this, "samples", 200));

    calib_ = Calib::MakeDefault();
    std::string err;
    if (std::filesystem::exists(calib_file_)) calib_.Load(calib_file_, &err);

    trial_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
        "/walk/calib_trial", 10);
    reload_cli_ = create_client<std_srvs::srv::Trigger>("/walk/reload_calib");

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
                "IMU 실측 보정 시작 (two_pose=%s, samples=%d, topic=%s)",
                two_pose_ ? "true" : "false", target_cnt_, topic.c_str());

    sub_ = create_subscription<sensor_msgs::msg::Imu>(
        topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr m) { OnImu(*m); });
  }

 private:
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

  void PublishProgress(const std::string& title,
                       int cur,
                       int total,
                       const Eigen::Vector3d& gyro_avg,
                       const Eigen::Vector3d& acc_avg) {
    diagnostic_msgs::msg::DiagnosticStatus ds;
    ds.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    ds.name = "xsting_walk/calib_trial";
    ds.hardware_id = "calib_imu_node";
    ds.message = title;
    ds.values = {
        MakeKv("01_보정단계", title),
        MakeKv("02_샘플수집",
               std::to_string(cur) + " / " + std::to_string(total)),
        MakeKv("03_원시자이로평균_rps",
               Fmt3(gyro_avg.x(), gyro_avg.y(), gyro_avg.z(), "%+.5f")),
        MakeKv("04_원시가속도평균_mps2",
               Fmt3(acc_avg.x(), acc_avg.y(), acc_avg.z(), "%+.4f")),
    };
    trial_pub_->publish(ds);
  }

  void OnImu(const sensor_msgs::msg::Imu& msg) {
    if (done_) return;
    const Eigen::Vector3d gyro(msg.angular_velocity.x, msg.angular_velocity.y,
                               msg.angular_velocity.z);
    const Eigen::Vector3d acc(msg.linear_acceleration.x,
                              msg.linear_acceleration.y,
                              msg.linear_acceleration.z);
    const Eigen::Vector3d grav = ExtractGrav(msg);

    if (stage_ == 0) {
      gyro_sum_ += gyro;
      acc_sum_ += acc;
      grav1_ += grav;
      ++cnt1_;
      if (cnt1_ % 20 == 0 || cnt1_ == target_cnt_) {
        PublishProgress("IMU 1단계(수평 정지) 수집 중", cnt1_, target_cnt_,
                        gyro_sum_ / cnt1_, acc_sum_ / cnt1_);
      }
      if (cnt1_ < target_cnt_) return;
      if (!two_pose_) {
        Finish(SolveImu1Pose(gyro_sum_ / cnt1_, acc_sum_ / cnt1_,
                             grav1_ / cnt1_, yaw_deg_));
      } else {
        stage_ = 1;
        PublishProgress(
            "IMU 1단계 완료 -> 차체를 앞으로 15~25도 숙인 뒤 /calib_imu/capture_pose2 호출",
            cnt1_, target_cnt_, gyro_sum_ / cnt1_, acc_sum_ / cnt1_);
        RCLCPP_INFO(get_logger(),
                    "1단계 완료 -> 차체를 앞으로 15~25도 숙인 뒤 "
                    "'ros2 service call /calib_imu/capture_pose2 "
                    "std_srvs/srv/Trigger {}' 실행");
      }
      return;
    }

    if (stage_ == 2) {
      grav2_ += grav;
      ++cnt2_;
      if (cnt2_ % 20 == 0 || cnt2_ == target_cnt_) {
        PublishProgress("IMU 2단계(전방 숙임) 수집 중", cnt2_, target_cnt_,
                        gyro_sum_ / cnt1_, acc_sum_ / cnt1_);
      }
      if (cnt2_ < target_cnt_) return;
      Finish(SolveImu2Pose(gyro_sum_ / cnt1_, acc_sum_ / cnt1_, grav1_ / cnt1_,
                           grav2_ / cnt2_));
    }
  }

  void Finish(const ImuCalib& solved) {
    calib_.imu = solved;
    const auto parent = std::filesystem::path(calib_file_).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    BackupIfExists(calib_file_);

    std::string err;
    if (calib_.Save(calib_file_, &err)) {
      RCLCPP_INFO(
          get_logger(),
          "IMU 보정 완료 -> %s\n"
          "  gyro_bias=[%.5f, %.5f, %.5f] rad/s\n"
          "  acc_bias =[%.5f, %.5f, %.5f] m/s^2",
          calib_file_.c_str(), solved.gyro_bias.x(), solved.gyro_bias.y(),
          solved.gyro_bias.z(), solved.acc_bias.x(), solved.acc_bias.y(),
          solved.acc_bias.z());

      if (reload_cli_->wait_for_service(std::chrono::milliseconds(250))) {
        reload_cli_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>());
      }

      diagnostic_msgs::msg::DiagnosticStatus ds;
      ds.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      ds.name = "xsting_walk/calib_trial";
      ds.hardware_id = "calib_imu_node";
      ds.message = "IMU 실측 보정 저장 및 리로드 완료";
      ds.values = {
          MakeKv("01_상태", "IMU 보정 완료"),
          MakeKv("02_저장파일", calib_file_),
          MakeKv("03_gyro_bias_rps",
                 Fmt3(solved.gyro_bias.x(), solved.gyro_bias.y(),
                      solved.gyro_bias.z(), "%+.5f")),
          MakeKv("04_acc_bias_mps2",
                 Fmt3(solved.acc_bias.x(), solved.acc_bias.y(),
                      solved.acc_bias.z(), "%+.4f")),
      };
      trial_pub_->publish(ds);
      std::this_thread::sleep_for(std::chrono::milliseconds(80));
    } else {
      RCLCPP_ERROR(get_logger(), "%s", err.c_str());
    }
    done_ = true;
    rclcpp::shutdown();
  }

  std::string calib_file_;
  bool two_pose_ = false;
  double yaw_deg_ = 0.0;
  int target_cnt_ = 200;
  int stage_ = 0;
  int cnt1_ = 0;
  int cnt2_ = 0;
  bool done_ = false;

  Eigen::Vector3d gyro_sum_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_sum_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d grav1_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d grav2_ = Eigen::Vector3d::Zero();
  Calib calib_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr
      trial_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reload_cli_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
};

}  // namespace xsting_walk

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<xsting_walk::CalibImuNode>();
    if (rclcpp::ok()) rclcpp::spin(node);
    node.reset();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("calib_imu_node"),
                 "IMU 보정 노드 실행 실패: %s", e.what());
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
