#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace xsting_walk {

namespace {

constexpr double kDiag = 0.70710678;

// 터미널 Raw 비블로킹 입력 RAII 가드
class RawTerminal {
 public:
  RawTerminal() {
    if (isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &orig_) == 0) {
      termios raw = orig_;
      raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
      raw.c_cc[VMIN] = 0;
      raw.c_cc[VTIME] = 0;
      if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) active_ = true;
    }
  }
  ~RawTerminal() {
    if (active_) tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
  }
  bool active() const { return active_; }

 private:
  termios orig_{};
  bool active_ = false;
};

}  // namespace

// xSting 8방향 전방향 보행 + 제자리/커브 회전 C++ 키보드 텔레옵 노드
class TeleopKeyNode : public rclcpp::Node {
 public:
  TeleopKeyNode() : Node("teleop_key_node") {
    const auto cmd_topic =
        declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    max_vx_ = declare_parameter<double>("max_vx", 0.35);
    max_vy_ = declare_parameter<double>("max_vy", 0.30);
    max_wz_ = declare_parameter<double>("max_wz", 1.25);
    speed_ratio_ = std::clamp(declare_parameter<double>("speed_ratio", 0.65),
                              0.15, 1.00);
    latch_mode_ = declare_parameter<bool>("latch_mode", false);
    key_hold_s_ = declare_parameter<double>("key_hold_sec", 0.45);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
    reload_cli_ = create_client<std_srvs::srv::Trigger>("/walk/reload_calib");
    stop_cli_ = create_client<std_srvs::srv::Trigger>("/walk/stop");

    status_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/walk/status", 10,
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr m) {
          if (m->data.size() < 14) return;
          std::lock_guard<std::mutex> lk(mtx_);
          est_vx_ = m->data[10];
          est_vy_ = m->data[11];
          imu_wz_ = m->data[12];
          h_err_deg_ = m->data[13];
          has_status_ = true;
        });

    PrintBanner();
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               [this] { OnTick20Hz(); });
  }

  ~TeleopKeyNode() override {
    if (cmd_pub_) {
      geometry_msgs::msg::Twist zero;
      cmd_pub_->publish(zero);
    }
    std::printf("\n");
  }

 private:
  void PrintBanner() const {
    std::printf(
        "\n==================================================================\n"
        "  xSting 전방향 보행 C++ 키보드 텔레옵 (xsting_walk)\n"
        "------------------------------------------------------------------\n"
        "   [U] 좌전방     [W / ↑] 전진       [O] 우전방\n"
        "   [A] 좌횡이동   [Space/K] 정지     [D] 우횡이동\n"
        "   [J] 좌후방     [S / ↓] 후진       [L] 우후방\n"
        "\n"
        "   [Q / ←] 제자리 좌회전    [E / →] 제자리 우회전\n"
        "   [Z]     좌커브 전진      [C]     우커브 전진\n"
        "\n"
        "   [1..5] 속도 단수 (20%%~100%%)   [+/-] 속도 10%% 증감\n"
        "   [M]    유지 모드 전환 (Hold: 키 떼면 정지 / Latch: 속도 유지)\n"
        "   [R]    실측 보정 파일 즉시 재로드 (/walk/reload_calib)\n"
        "   [X]    비상 정지 (/walk/stop)   [Ctrl+C] 종료\n"
        "==================================================================\n");
    std::fflush(stdout);
  }

  bool ReadKeys() {
    bool any = false;
    char ch = 0;
    while (read(STDIN_FILENO, &ch, 1) == 1) {
      if (ch == '\x03') {  // Ctrl+C
        rclcpp::shutdown();
        return false;
      }
      // 방향키 ESC 시퀀스 처리 (\x1b [ A/B/C/D)
      if (ch == '\x1b') {
        char seq[2]{};
        if (read(STDIN_FILENO, &seq[0], 1) == 1 && seq[0] == '[' &&
            read(STDIN_FILENO, &seq[1], 1) == 1) {
          if (seq[1] == 'A') ch = 'w';
          if (seq[1] == 'B') ch = 's';
          if (seq[1] == 'C') ch = 'e';
          if (seq[1] == 'D') ch = 'q';
        } else {
          continue;
        }
      }
      if (ch >= 'A' && ch <= 'Z') ch = static_cast<char>(ch - 'A' + 'a');
      HandleKey(ch);
      any = true;
    }
    return any;
  }

  void HandleKey(char k) {
    bool motion_key = true;
    switch (k) {
      case 'w':
        dir_ = {1.0, 0.0, 0.0};
        break;
      case 's':
        dir_ = {-1.0, 0.0, 0.0};
        break;
      case 'a':
        dir_ = {0.0, 1.0, 0.0};
        break;
      case 'd':
        dir_ = {0.0, -1.0, 0.0};
        break;
      case 'u':
        dir_ = {kDiag, kDiag, 0.0};
        break;
      case 'o':
        dir_ = {kDiag, -kDiag, 0.0};
        break;
      case 'j':
        dir_ = {-kDiag, kDiag, 0.0};
        break;
      case 'l':
        dir_ = {-kDiag, -kDiag, 0.0};
        break;
      case 'q':
        dir_ = {0.0, 0.0, 1.0};
        break;
      case 'e':
        dir_ = {0.0, 0.0, -1.0};
        break;
      case 'z':
        dir_ = {0.80, 0.0, 0.76};
        break;
      case 'c':
        dir_ = {0.80, 0.0, -0.76};
        break;
      case ' ':
      case 'k':
        dir_ = {0.0, 0.0, 0.0};
        break;
      case 'x':
        dir_ = {0.0, 0.0, 0.0};
        if (stop_cli_->service_is_ready()) {
          stop_cli_->async_send_request(
              std::make_shared<std_srvs::srv::Trigger::Request>());
        }
        break;
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
        speed_ratio_ = 0.20 * static_cast<double>(k - '0');
        motion_key = false;
        break;
      case '+':
      case '=':
      case ']':
        speed_ratio_ = std::min(1.00, speed_ratio_ + 0.10);
        motion_key = false;
        break;
      case '-':
      case '_':
      case '[':
        speed_ratio_ = std::max(0.15, speed_ratio_ - 0.10);
        motion_key = false;
        break;
      case 'm':
        latch_mode_ = !latch_mode_;
        if (!latch_mode_) dir_ = {0.0, 0.0, 0.0};
        motion_key = false;
        break;
      case 'r':
        if (reload_cli_->service_is_ready()) {
          reload_cli_->async_send_request(
              std::make_shared<std_srvs::srv::Trigger::Request>());
          std::printf("\n[보정 재로드 요청 전송 완료]\n");
        } else {
          std::printf("\n[/walk/reload_calib 서비스 대기 중]\n");
        }
        motion_key = false;
        break;
      default:
        motion_key = false;
        break;
    }
    if (motion_key) last_key_time_ = now();
  }

  void OnTick20Hz() {
    ReadKeys();
    if (!rclcpp::ok()) return;

    if (!latch_mode_ &&
        (now() - last_key_time_).seconds() > key_hold_s_) {
      dir_ = {0.0, 0.0, 0.0};
    }

    geometry_msgs::msg::Twist msg;
    msg.linear.x = dir_[0] * max_vx_ * speed_ratio_;
    msg.linear.y = dir_[1] * max_vy_ * speed_ratio_;
    msg.angular.z = dir_[2] * max_wz_ * speed_ratio_;
    cmd_pub_->publish(msg);

    float evx = 0.0f;
    float evy = 0.0f;
    float iwz = 0.0f;
    float herr = 0.0f;
    bool got = false;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      evx = est_vx_;
      evy = est_vy_;
      iwz = imu_wz_;
      herr = h_err_deg_;
      got = has_status_;
    }

    if (got) {
      std::printf(
          "\r[%s %3.0f%%] cmd=(%+5.2f, %+5.2f, %+5.2f) | "
          "est=(%+5.2f, %+5.2f) wz=%+5.2f herr=%+5.1fdeg   ",
          latch_mode_ ? "LATCH" : "HOLD ", speed_ratio_ * 100.0, msg.linear.x,
          msg.linear.y, msg.angular.z, evx, evy, iwz, herr);
    } else {
      std::printf("\r[%s %3.0f%%] cmd=(%+5.2f, %+5.2f, %+5.2f)   ",
                  latch_mode_ ? "LATCH" : "HOLD ", speed_ratio_ * 100.0,
                  msg.linear.x, msg.linear.y, msg.angular.z);
    }
    std::fflush(stdout);
  }

  RawTerminal raw_term_;
  std::mutex mtx_;
  std::array<double, 3> dir_{0.0, 0.0, 0.0};
  double max_vx_ = 0.35;
  double max_vy_ = 0.30;
  double max_wz_ = 1.25;
  double speed_ratio_ = 0.65;
  bool latch_mode_ = false;
  double key_hold_s_ = 0.45;
  rclcpp::Time last_key_time_{0, 0, RCL_ROS_TIME};

  float est_vx_ = 0.0f;
  float est_vy_ = 0.0f;
  float imu_wz_ = 0.0f;
  float h_err_deg_ = 0.0f;
  bool has_status_ = false;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr status_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reload_cli_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_cli_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace xsting_walk

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xsting_walk::TeleopKeyNode>());
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
