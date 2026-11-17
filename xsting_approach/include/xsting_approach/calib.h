#ifndef XSTING_APPROACH_CALIB_H_
#define XSTING_APPROACH_CALIB_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <string>
#include <vector>

namespace xsting_approach {

// 다리 18자유도 및 팔 6자유도 상수
inline constexpr int kNumLegs = 18;
inline constexpr int kNumArm = 6;

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
};

// IMU 장착 회전 및 자이로 영점 바이어스 보정 정보
struct ImuCalib {
  Eigen::Quaterniond mount_quat = Eigen::Quaterniond::Identity();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();

  // 원시 자이로·쿼터니언을 차체 각속도와 중력 투영 벡터로 변환
  void Transform(const Eigen::Vector3d& gyro_raw,
                 const Eigen::Quaterniond& quat_raw,
                 Eigen::Vector3f* ang_vel_b,
                 Eigen::Vector3f* proj_grav_b) const;
};

// 전면 D435 외부 파라미터 및 상자 정밀 접근 목표 치수
struct CamCalib {
  Eigen::Vector3d cam_pos_m = Eigen::Vector3d(0.157793, 0.017500, 0.107498);
  Eigen::Vector3d cam_rpy_deg = Eigen::Vector3d::Zero();
  double base_z_m = 0.0213;
  double robot_front_x_m = 0.162;
  double target_gap_m = 0.020;
  double box_half_m = 0.1025;
  double box_height_m = 0.070;
  double puck_radius_m = 0.015;
  double standoff_m = 0.2845;
  double pos_tol_m = 0.020;
  double yaw_tol_rad = 0.050;
  double hold_time_s = 2.0;

  // 깊이 광학 좌표계를 차체(base) 좌표계로 회전시키는 행렬 계산
  Eigen::Matrix3f OptToBaseRot() const;
};

// 전면 HC-SR04 초음파 센서 위치·영점·근접 융합 설정
struct SonarCalib {
  bool enabled = true;
  Eigen::Vector3d us_pos_m = Eigen::Vector3d(0.127180, 0.0, 0.024522);
  double min_range_m = 0.020;
  double max_range_m = 4.000;
  double range_bias_m = 0.0;
  bool close_fusion = true;
  double brake_gap_m = 0.008;

  // 초음파 센서에서 차체 전면 범퍼까지의 X축 거리와 목표 거리 계산
  double BumperOff(double front_x) const { return front_x - us_pos_m.x(); }
  double GoalRange(double front_x, double gap) const {
    return BumperOff(front_x) + gap;
  }
};

// WalkNet 보행 스케일, 정지 히스테리시스 및 사각지대 추측항법 설정
struct GaitCalib {
  double walk_scale = 0.50;
  double gait_freq_hz = 3.0;
  Eigen::Vector3d speed_lim = Eigen::Vector3d(0.20, 0.10, 0.50);
  Eigen::Vector3d speed_scale = Eigen::Vector3d::Ones();
  double stop_enter = 0.12;
  double stop_exit = 0.22;
  double deadband = 0.08;
  double max_step_rad = 0.075;
  bool blind_dr = true;
  double blind_max_m = 0.28;
  bool dry_run = false;
};

// xSting 전체 실기 보정 데이터 묶음
struct Calib {
  std::array<JointCalib, kNumLegs> legs;
  std::array<JointCalib, kNumArm> arm;
  std::vector<std::string> out_leg_order;
  ImuCalib imu;
  CamCalib cam;
  SonarCalib sonar;
  GaitCalib gait;

  // 기본 xSting 보정치 생성 및 YAML 입출력
  static Calib MakeDefault();
  bool Load(const std::string& path, std::string* err = nullptr);
  bool Save(const std::string& path, std::string* err = nullptr) const;
};

// WalkNet·표준 다리·팔 관절 이름 배열 및 하드웨어 초기·WalkNet 기립각 조회
const std::array<std::string, kNumLegs>& GetWalkLegNames();
const std::array<std::string, kNumLegs>& GetStdLegNames();
const std::array<std::string, kNumArm>& GetArmNames();
double GetBootRad(const std::string& joint_name);
std::array<double, kNumLegs> BuildStandLegCmd(const Calib& calib);

// IMU 1자세·2자세 장착각 및 자이로 바이어스 산출
ImuCalib SolveImu1Pose(const Eigen::Vector3d& gyro,
                       const Eigen::Vector3d& grav_imu,
                       double yaw_deg = 0.0);
ImuCalib SolveImu2Pose(const Eigen::Vector3d& gyro,
                       const Eigen::Vector3d& grav_level,
                       const Eigen::Vector3d& grav_tilt);

// 바닥 평면·기준 상자·초음파 영점 실기 보정 솔버
bool SolveFloor(const std::vector<Eigen::Vector3f>& pts_opt,
                CamCalib* cam,
                double* rmse_m = nullptr);
bool SolveBox(const std::vector<Eigen::Vector3f>& pts_opt,
              CamCalib* cam,
              Eigen::Vector2f* box_xy = nullptr,
              Eigen::Vector2f* norm_xy = nullptr,
              double gap_m = -1.0);
bool SolveSonar(const std::vector<double>& ranges_m,
                const CamCalib& cam,
                SonarCalib* sonar,
                double gap_m = -1.0,
                double* corr_m = nullptr);

}  // namespace xsting_approach

#endif  // XSTING_APPROACH_CALIB_H_
