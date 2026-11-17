#ifndef XSTING_NAV__NAV_CORE_HPP_
#define XSTING_NAV__NAV_CORE_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace xsting_nav
{

using Vec2f = std::array<float, 2>;
using Vec3f = std::array<float, 3>;
using Vec4f = std::array<float, 4>;
using Vec6f = std::array<float, 6>;
using Obs41f = std::array<float, 41>;
using Leg18f = std::array<float, 18>;
using Arm6f = std::array<float, 6>;

/// MK2 physical constants matching nav2/leg_gait.py and xsting_description URDF.
struct Mk2Kinematics
{
  static constexpr double kPi = 3.14159265358979323846;
  static constexpr float kL1 = 0.081f;
  static constexpr float kTipR = 0.12875f;
  static constexpr float kTipZ = -0.020f;
  static constexpr float kHipR = 0.0605f;
  static constexpr float kHipZ = 0.072722f;
  static constexpr float kHip0 = static_cast<float>(30.0 * kPi / 180.0);
  static constexpr float kKnee0 = static_cast<float>(-110.0 * kPi / 180.0);
  static constexpr float kArmHomeShoulder = -1.2962634016f;
  static constexpr float kArmHomeElbow = 1.2962634016f;

  // Box & robot front bumper geometry (ensuring pre-approach stays outside D435 12cm blind zone).
  static constexpr float kBoxHalfM = 0.1025f;
  static constexpr float kRobotFrontXM = 0.162f;
  static constexpr float kNominalPreApproachStandoffM = 0.462f;  // ~19.7cm bumper gap

  // Front (+X), Left (+Y), and Right (-Y) RealSense D435 nvBlox extrinsics in base frame.
  static constexpr Vec4f kFrontCamPose{0.157793f, 0.0175f, 0.107498f, 0.0f};
  static constexpr Vec4f kLeftCamPose{-0.0175f, 0.1198f, 0.123722f, static_cast<float>(0.5 * kPi)};
  static constexpr Vec4f kRightCamPose{0.0175f, -0.1198f, 0.123722f, static_cast<float>(-0.5 * kPi)};

  // Center-mounted inverted 360-deg 2D ToF LiDAR (COIN-D4, roll=pi) & front sonar in base frame.
  static constexpr Vec4f kLidarPoseBase{0.0f, 0.0f, 0.0245f, static_cast<float>(kPi)};
  static constexpr float kLidarMinRange = 0.12f;
  static constexpr float kLidarMaxRange = 3.50f;
  static constexpr float kLidarLegMaskRadius = 0.048f;
  static constexpr Vec3f kUltrasonicPoseBase{0.12718f, 0.0f, 0.0245219f};

  static constexpr std::array<float, 6> kMountX{0.11f, 0.0f, -0.11f, 0.11f, 0.0f, -0.11f};
  static constexpr std::array<float, 6> kMountY{0.10f, 0.10f, 0.10f, -0.10f, -0.10f, -0.10f};
  static constexpr std::array<float, 6> kSide{1.0f, 1.0f, 1.0f, -1.0f, -1.0f, -1.0f};
  static constexpr std::array<float, 6> kYaw0Deg{30.0f, 0.0f, -30.0f, 30.0f, 0.0f, -30.0f};

  static constexpr Leg18f kLegLow{
    -0.4363f, -1.6580f, -2.0071f,
    -0.4363f, -1.6580f, -2.0071f,
    -1.5708f, -1.6580f, -2.0071f,
    -0.4363f, -1.6580f, -2.0071f,
    -0.7505f, -1.6580f, -2.0071f,
    -1.5708f, -1.6580f, -2.0071f
  };

  static constexpr Leg18f kLegHigh{
    1.5708f, 1.6580f, 1.7453f,
    0.7505f, 1.6580f, 1.7453f,
    0.4363f, 1.6580f, 1.7453f,
    1.5708f, 1.6580f, 1.7453f,
    0.4363f, 1.6580f, 1.7453f,
    0.4363f, 1.6580f, 1.7453f
  };

  static Leg18f stand_pose();
  static Arm6f arm_home_pose();
  static std::array<std::string, 18> sim_leg_joint_names();
  static std::array<std::string, 18> mk2_hw_joint_names();
  static std::array<std::string, 6> arm_joint_names();
};

/// Persistent real-robot calibration configuration loaded/saved from calibration.yaml.
struct CalibrationConfig
{
  // Hardware profile:
  // - "xsting": mk2_ws/src/xsting_hardware (/leg_controller/commands, /arm_controller/commands,
  //             /imu_broadcaster/imu, /sonar_broadcaster/range, /scan_filtered, URDF joint limits)
  // - "mk2_hexapod": legacy OpenCR v4 (/position_controller/commands, /mk2/front/imu, ID 4 & 13 ticks)
  // - "leg_arm_split": alias compatible with "xsting" split leg/arm controllers
  std::string hardware_profile = "xsting";
  std::string last_calibrated_utc = "uncalibrated_defaults";

  // 1. IMU calibration (board mounting rotation + gyro bias + level floor alignment quaternion)
  float imu_board_yaw_deg = 0.0f;
  Vec3f imu_gyro_bias_xyz{0.0f, 0.0f, 0.0f};
  Vec4f imu_level_quat_wxyz{1.0f, 0.0f, 0.0f, 0.0f};

  // 2. 18-DOF leg & 6-DOF arm motor zero-offset and assembly sign calibration
  Leg18f leg_sign{};
  Leg18f leg_offset_rad{};
  Leg18f leg_hw_min_rad{};
  Leg18f leg_hw_max_rad{};
  Arm6f arm_sign{};
  Arm6f arm_offset_rad{};
  float max_joint_slew_rad_s = 7.0f;
  float soft_start_duration_s = 2.0f;

  // 3. cuVSLAM arena origin (defaults to xsting_mission HOME = [0.734, 0.950, pi]) & gait slip scale
  Vec3f init_pose_arena{0.734f, 0.950f, static_cast<float>(Mk2Kinematics::kPi)};
  Vec2f odom_pos_scale{1.0f, 1.0f};
  float odom_yaw_offset_rad = 0.0f;
  Vec3f cmd_vel_scale{1.0f, 1.0f, 1.0f};
  Vec3f cmd_vel_trim{0.0f, 0.0f, 0.0f};

  // 4. nvBlox ESDF, 360-deg COIN-D4 2D LiDAR, front ultrasonic, & YOLO-PCA camera extrinsics
  std::string esdf_frame_mode = "odom";
  float esdf_dist_offset_m = 0.0f;
  std::string yolo_cloud_frame = "base";
  Vec3f cam_pos_base_xyz{0.157793f, 0.0175f, 0.107498f};
  Vec3f cam_rpy_base_rad{0.0f, 0.0f, 0.0f};
  Vec3f lidar_pos_base_xyz{0.0f, 0.0f, 0.0245f};
  float lidar_roll_rad = static_cast<float>(Mk2Kinematics::kPi);
  float lidar_yaw_offset_rad = 0.0f;
  Vec3f ultrasonic_pos_base_xyz{0.12718f, 0.0f, 0.0245219f};

  static CalibrationConfig make_defaults(const std::string & profile = "xsting");
  static CalibrationConfig load_yaml(const std::string & yaml_path);
  void save_yaml(const std::string & yaml_path) const;
};

/// 2D/3D math helpers matching Isaac Lab & nav2/leg_gait.py.
float wrap_to_pi(float angle);
Vec3f quat_to_grav(const Vec4f & quat_wxyz);
float quat_to_yaw(float qx, float qy, float qz, float qw);
Vec4f quat_mul(const Vec4f & a_wxyz, const Vec4f & b_wxyz);
Vec3f quat_rotate(const Vec4f & q_wxyz, const Vec3f & v);
Vec4f solve_gravity_level_quat(const Vec3f & measured_grav_xyz);
Vec2f pca_axis_2d(const std::vector<Vec2f> & pts);

/// Calibrated IMU preprocessor converting raw sensor_msgs/Imu into 6D RL IMU state.
Vec6f transform_calibrated_imu(
  const Vec3f & raw_gyro_xyz,
  const Vec4f & raw_quat_wxyz,
  const CalibrationConfig & calib);

/// Transform 3D point cloud into robot base 2D XY points for YOLO-PCA.
std::vector<Vec2f> transform_yolo_points_xy(
  const std::vector<Vec3f> & raw_pts,
  const CalibrationConfig & calib);

/// Synthesize 5 front-face points in base XY from a 3D Detection3D paper_box bounding box.
std::vector<Vec2f> synthesize_box_front_points(
  const Vec2f & center_base_xy,
  float box_yaw_base_rad,
  float half_extent_m = Mk2Kinematics::kBoxHalfM);

/// Compute calibrated odom pose (ox, oy, oyaw) before map/arena frame transform.
Vec3f calibrated_odom_pose(const Vec3f & raw_odom_pose, const CalibrationConfig & calib);

/// Compose two SE(2) poses: a * b where each is (x, y, yaw_rad).
Vec3f compose_se2(const Vec3f & a, const Vec3f & b);

/// Solve T_map_odom such that compose_se2(T_map_odom, eff_odom_pose) == arena_pose.
Vec3f solve_map_to_odom_se2(const Vec3f & arena_pose, const Vec3f & eff_odom_pose);

/// Smoothly blend SE(2) transform toward target_tf with step clamping for drift-free TF updates.
Vec3f blend_se2(
  const Vec3f & current_tf,
  const Vec3f & target_tf,
  float pos_alpha,
  float yaw_alpha,
  float max_step_pos_m,
  float max_step_yaw_rad);

/// Convert cuVSLAM odom pose into arena (mx, my, myaw) using calibrated init_pose & scale.
Vec3f odom_to_arena_pose(const Vec3f & raw_odom_pose, const CalibrationConfig & calib);

/// Convert an arena (mx, my, myaw) pose back into an equivalent odom pose relative to init_pose.
Vec3f arena_to_odom_pose(const Vec3f & arena_pose, const Vec3f & init_pose);

/// Result of box-aware pre-approach target refinement.
struct BoxPreApproachRefinement
{
  bool accepted = false;
  Vec2f box_arena_xy{0.0f, 0.0f};
  Vec3f refined_goal{0.0f, 0.0f, 0.0f};
};

/// Refine pre-approach goal (~20cm bumper gap) from a visually detected paper_box in base frame.
/// Rejects detections farther than assoc_radius_m (default 0.35m < half of 0.734m inter-box spacing).
BoxPreApproachRefinement refine_box_pre_approach_goal(
  const Vec3f & nominal_goal,
  const Vec2f & nominal_box_xy,
  const Vec3f & robot_arena_pose,
  const Vec2f & det_box_base_xy,
  float assoc_radius_m = 0.35f,
  float max_shift_m = 0.25f);

/// Evaluate whether robot has detected the box AND arrived within 5~10cm (pos_tol_m) of pre-approach pose.
bool should_trigger_precision_approach(
  const Vec3f & robot_arena_pose,
  const Vec3f & pre_approach_goal,
  bool box_visible,
  float pos_tol_m = 0.08f,
  float yaw_tol_rad = 0.20f);

/// 2D ESDF map slice container matching nvblox_msgs/msg/DistanceMapSlice.
struct EsdfSlice
{
  int width = 64;
  int height = 64;
  float origin_x = -1.5f;
  float origin_y = -1.5f;
  float resolution = 0.05f;
  float unknown_value = 1000.0f;
  std::vector<float> data;

  static EsdfSlice make_clear_default(int w = 64, int h = 64, float clear_m = 1.0f);
  static EsdfSlice from_occupancy_grid(
    int w, int h, float origin_x, float origin_y, float res,
    const std::vector<int8_t> & occ_data, int occ_thresh = 50, float max_dist_m = 1.5f);
};

/// Exact C++ port of build_obs() in nav2/nav_env.py (41D observation vector).
Obs41f build_obs_41d(
  const Vec3f & odom_pose,
  const Vec3f & odom_vel,
  const Vec3f & goal,
  const EsdfSlice & esdf_slice,
  const std::vector<Vec2f> & box_pts_body,
  const std::vector<Vec2f> & puck_pts_body,
  const Vec3f & init_pose,
  const Vec6f & imu_state,
  const Vec3f & nav_waypoint,
  float esdf_max = 1.0f,
  const std::string & esdf_frame_mode = "odom",
  float esdf_dist_offset_m = 0.0f);

/// Project 360-deg 2D LaserScan (e.g. /scan_filtered in "lidar" frame with roll=pi) into base XY.
std::vector<Vec2f> project_lidar_scan_to_base_xy(
  const std::vector<float> & ranges,
  float angle_min,
  float angle_increment,
  const CalibrationConfig & calib,
  const std::string & frame_id = "lidar",
  bool apply_self_mask = false,
  float min_range_m = Mk2Kinematics::kLidarMinRange,
  float max_range_m = Mk2Kinematics::kLidarMaxRange);

/// Fuse live 360-deg LiDAR base XY points and front ultrasonic range into 41D obs[17..32] ESDF grid.
void fuse_lidar_ultrasonic_into_obs(
  Obs41f & obs,
  const std::vector<Vec2f> & lidar_pts_base,
  float ultrasonic_range_m = -1.0f,
  float esdf_max = 1.0f,
  const Vec3f & ultrasonic_pos_base = Mk2Kinematics::kUltrasonicPoseBase);

/// Exact C++ port of skrl RunningStandardScaler + 3-layer ELU MLP policy.
class MlpPolicy
{
public:
  void load_json(const std::string & json_path);
  bool is_loaded() const { return loaded_; }

  Vec3f act_raw(const Obs41f & obs) const;
  Vec3f act_shielded(const Obs41f & obs) const;

  const Vec3f & speed_limits() const { return speed_limits_; }

private:
  struct LinearLayer
  {
    int in_dim = 0;
    int out_dim = 0;
    std::vector<float> weight;
    std::vector<float> bias;
  };

  bool loaded_ = false;
  float clip_obs_ = 5.0f;
  float eps_ = 1e-8f;
  float clip_act_ = 1.0f;
  Vec3f speed_limits_{0.35f, 0.22f, 1.80f};
  Obs41f mean_{};
  Obs41f inv_std_{};
  std::array<LinearLayer, 3> layers_{};
};

/// Exact C++ port of nvblox_shield() and dock_vel() in nav2/leg_gait.py.
struct SafetyShield
{
  static Vec3f apply(const Obs41f & obs, const Vec3f & raw_act);

  struct DockResult
  {
    Vec3f vel_cmd{0.0f, 0.0f, 0.0f};
    bool aligned = false;
  };
  static DockResult dock_vel(const std::array<float, 5> & box_obs5, float standoff = 0.315f);
};

/// Exact C++ port of RealNavEnv 55x79 static ESDF + 10.0s human-blocking wait & Dijkstra detour.
class DetourPlanner
{
public:
  void load_arena_json(const std::string & json_path);
  bool is_loaded() const { return loaded_; }

  /// Map xsting_mission place_id ("HOME", "S1".."S6", "L1".."L3") <-> goal_id (0..9).
  static int place_id_to_goal_id(const std::string & place_id);
  static std::string goal_id_to_place_id(int goal_id);
  static bool is_box_goal_id(int goal_id) { return goal_id >= 1 && goal_id <= 9; }

  void set_goal_by_id(int goal_id, const Vec2f & robot_arena_xy);
  void set_custom_goal(const Vec3f & goal_pose, const Vec2f & robot_arena_xy);
  void update_goal_pose_preserving_blocks(const Vec3f & goal_pose, const Vec2f & robot_arena_xy);
  void reset_episode_blocks(const Vec2f & robot_arena_xy);

  bool nominal_box_xy(int goal_id, Vec2f * out_xy) const;
  bool is_goal_unreachable(float mx, float my) const;

  float static_clearance(float x, float y) const;
  float static_esdf_bilinear(
    float x, float y, float * grad_x = nullptr, float * grad_y = nullptr) const;

  struct ArenaLocalizationResult
  {
    bool converged = false;
    Vec3f arena_pose{0.0f, 0.0f, 0.0f};
    Vec3f map_to_odom{0.0f, 0.0f, 0.0f};
    float rmse_m = 0.0f;
    int inliers = 0;
  };

  /// Align 360-deg 2D LiDAR base points against static arena ESDF (coarse grid + Huber Gauss-Newton).
  ArenaLocalizationResult localize_lidar_on_arena(
    const std::vector<Vec2f> & lidar_pts_base,
    const Vec3f & prior_arena_pose,
    const Vec3f & eff_odom_pose,
    bool global_search = false,
    int min_inliers = 20,
    float max_rmse_m = 0.065f) const;

  float path_distance(float mx, float my) const;
  Vec3f raw_waypoint(float mx, float my) const;
  Vec3f active_waypoint(float mx, float my, float myaw) const;

  void update_blocking_replan(
    float mx, float my, float myaw,
    const Obs41f & probe_obs,
    float step_dt = 0.05f,
    float human_wait_timeout_s = 10.0f,
    float goal_tol_m = 0.12f,
    bool reroute_unreachable_goal = true);

  bool is_human_waiting() const { return human_waiting_; }
  int block_steps() const { return block_steps_; }
  int human_replans() const { return human_replans_; }
  int goal_id() const { return active_gid_; }
  const Vec3f & goal() const { return active_goal_; }
  const std::vector<Vec3f> & goals() const { return goals_; }
  const std::vector<Vec2f> & boxes() const { return boxes_; }
  const std::vector<Vec2f> & blocked_points() const { return blocked_pts_; }

  std::vector<float> solve_grid_dijkstra(
    const Vec2f & goal_xy,
    const Vec2f & rob_xy,
    const std::vector<Vec2f> & blocked_pts) const;

private:
  bool loaded_ = false;
  int nx_ = 55;
  int ny_ = 79;
  float res_ = 0.08f;
  std::vector<float> static_ce_;
  std::vector<Vec3f> goals_;
  std::vector<Vec2f> boxes_;

  int active_gid_ = 1;
  Vec3f active_goal_{1.930f, 0.950f, 3.14159265f};
  int block_steps_ = 0;
  bool human_waiting_ = false;
  int human_replans_ = 0;
  std::vector<Vec2f> blocked_pts_;
  std::vector<float> dist_map_;
};

/// Exact C++ port of 100Hz MK2 LegGait + analytical 3-DOF x 6-leg IK + hardware calibration.
class LegGaitController
{
public:
  explicit LegGaitController(float freq = 2.2f, float step_h = 0.030f, float max_stride = 0.042f);

  static Leg18f solve_leg_ik(const std::array<Vec3f, 6> & foot_body);
  static Leg18f compute_gait_joints(
    const Vec3f & vel_body, float phase,
    float freq = 2.2f, float step_h = 0.030f, float max_stride = 0.042f);

  Leg18f reset();
  void sync_from_hardware(const Leg18f & q_hw, const CalibrationConfig & calib);
  Leg18f step_sim(const Vec3f & vel_body, float dt = 0.01f);

  Leg18f step_hardware(
    const Vec3f & cmd_vel_body,
    const CalibrationConfig & calib,
    float dt = 0.01f);

  static Leg18f sim_to_hardware_legs(const Leg18f & q_sim, const CalibrationConfig & calib);
  static Leg18f hardware_to_sim_legs(const Leg18f & q_hw, const CalibrationConfig & calib);
  static Arm6f sim_to_hardware_arm(const Arm6f & q_arm_sim, const CalibrationConfig & calib);

  const Leg18f & current_sim_joints() const { return q_sim_; }
  float phase() const { return phase_; }

private:
  float freq_ = 2.2f;
  float step_h_ = 0.030f;
  float max_stride_ = 0.042f;
  float phase_ = 0.0f;
  Leg18f q_sim_{};
};

}  // namespace xsting_nav

#endif  // XSTING_NAV__NAV_CORE_HPP_
