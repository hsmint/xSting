#ifndef XSTING_WALK_CALIB_H_
#define XSTING_WALK_CALIB_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <string>
#include <vector>

namespace xsting_walk {

// 다리 18자유도(leg1..6_yaw/hip/knee) 및 팔 6자유도 상수
inline constexpr int kNumLegs = 18;
inline constexpr int kNumArm = 6;
inline constexpr double kGravityMps2 = 9.81;

// 단일 관절 방향·오프셋·한계·기립각 보정 정보
struct JointCalib {
  std::string name;
  std::string ros_name;
  double dir = 1.0;
  double offset_rad = 0.0;
  double lower_rad = -3.14159;
  double upper_rad = 3.14159;
  double stand_rad = 0.0;

  // 하드웨어 관절각·속도와 시뮬레이션 좌표계 상호 변환
  double ToSim(double q_hw) const { return (q_hw - offset_rad) * dir; }
  double ToHw(double q_sim) const { return dir * q_sim + offset_rad; }
  double VelToSim(double dq_hw) const { return dq_hw * dir; }

  // Isaac Lab soft_joint_pos_limit_factor 적용 소프트 한계
  double SoftLower(double factor) const {
    const double mid = 0.5 * (lower_rad + upper_rad);
    return mid - 0.5 * (upper_rad - lower_rad) * factor;
  }
  double SoftUpper(double factor) const {
    const double mid = 0.5 * (lower_rad + upper_rad);
    return mid + 0.5 * (upper_rad - lower_rad) * factor;
  }
};

// IMU 장착 회전, 자이로 바이어스 및 가속도계(9.81 m/s^2 기준) 바이어스
struct ImuCalib {
  Eigen::Quaterniond mount_quat = Eigen::Quaterniond::Identity();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();

  // 원시 자이로·가속도·쿼터니언을 차체 각속도, 선가속도, 중력투영, 차체 Yaw로 변환
  void Transform(const Eigen::Vector3d& gyro_raw,
                 const Eigen::Vector3d& acc_raw,
                 bool has_acc,
                 const Eigen::Quaterniond& quat_raw,
                 Eigen::Vector3f* ang_vel_b,
                 Eigen::Vector3f* lin_acc_b,
                 Eigen::Vector3f* proj_grav_b,
                 float* body_yaw_rad = nullptr,
                 bool* has_quat_yaw = nullptr) const;
};

// 카펫 전방향 보행 CPG·정책·실측 슬립/편향 보정 파라미터
struct WalkGaitCalib {
  double action_scale = 0.28;
  double action_ema = 0.55;
  double gait_freq_hz = 3.5;
  Eigen::Vector2d stride_gain_xy = Eigen::Vector2d(1.08, 1.24);
  double stride_gain_yaw = 1.04;
  double swing_clearance_m = 0.032;
  double stance_press_m = 0.003;
  double soft_limit_factor = 0.95;

  // 최대 목표 속도 (vx [m/s], vy [m/s], wz [rad/s])
  Eigen::Vector3d max_speed = Eigen::Vector3d(0.38, 0.32, 1.40);

  // 실측 보정: 목표 속도 대비 시뮬레이션 명령 배율 및 상시/비례 편향 보정
  Eigen::Vector3d cmd_vel_scale = Eigen::Vector3d::Ones();
  Eigen::Vector3d cmd_vel_trim = Eigen::Vector3d::Zero();
  double drift_vy_per_vx = 0.0;
  double drift_wz_per_vx = 0.0;

  // 텔레옵 가감속 제한 및 스텝당 관절 변화량 한계
  Eigen::Vector3d cmd_accel_lim = Eigen::Vector3d(1.20, 1.00, 3.50);
  double max_step_rad = 0.075;
  bool use_odom_vel = true;
  bool dry_run = false;
};

// xSting 보행 실기 보정 전체 설정
struct Calib {
  std::array<JointCalib, kNumLegs> legs;
  std::array<JointCalib, kNumArm> arm;
  std::vector<std::string> out_leg_order;
  ImuCalib imu;
  WalkGaitCalib gait;

  static Calib MakeDefault();
  bool Load(const std::string& path, std::string* err = nullptr);
  bool Save(const std::string& path, std::string* err = nullptr) const;
};

// 표준 다리(leg1..6_yaw/hip/knee) 및 팔 관절 이름, xsting_hardware 초기 부팅각 조회
const std::array<std::string, kNumLegs>& GetLegNames();
const std::array<std::string, kNumArm>& GetArmNames();
double GetBootRad(const std::string& joint_name);

// IMU 1자세(수평) 및 2자세(수평 + 전방 숙임) 장착 회전·자이로·가속도 바이어스 솔버
ImuCalib SolveImu1Pose(const Eigen::Vector3d& gyro_mean,
                       const Eigen::Vector3d& acc_mean,
                       const Eigen::Vector3d& grav_imu,
                       double yaw_deg = 0.0);
ImuCalib SolveImu2Pose(const Eigen::Vector3d& gyro_mean,
                       const Eigen::Vector3d& acc_mean,
                       const Eigen::Vector3d& grav_level,
                       const Eigen::Vector3d& grav_tilt);

// 실측 직선 주행(vx 또는 vy) 결과로부터 속도 스케일 및 직진 편향 계수 갱신
bool UpdateLinearCalib(const std::string& axis,
                       double cmd_speed_mps,
                       double duration_s,
                       double measured_dist_m,
                       double measured_lat_drift_m,
                       double measured_yaw_drift_deg,
                       bool has_lat_drift,
                       bool has_yaw_drift,
                       WalkGaitCalib* gait,
                       std::string* summary = nullptr);

// 실측 제자리 회전(wz) 결과로부터 요레이트 스케일 갱신
bool UpdateTurnCalib(double cmd_wz_rps,
                     double duration_s,
                     double measured_yaw_deg,
                     WalkGaitCalib* gait,
                     std::string* summary = nullptr);

}  // namespace xsting_walk

#endif  // XSTING_WALK_CALIB_H_
