#include "xsting_nav/nav_core.hpp"

#include <ament_index_cpp/get_package_share_path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <xsting_interfaces/action/approach_box.hpp>
#include <xsting_interfaces/action/run_task.hpp>
#include <xsting_interfaces/msg/error_code.hpp>

#ifdef XSTING_NAV_HAS_NVBLOX_MSGS
#include <nvblox_msgs/msg/distance_map_slice.hpp>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace xsting_nav
{

namespace
{
std::string resolve_pkg_path(const std::string & path_or_rel)
{
  if (path_or_rel.empty()) {
    return path_or_rel;
  }
  const std::filesystem::path p(path_or_rel);
  if (p.is_absolute() || std::filesystem::exists(p)) {
    return p.string();
  }
  try {
    const std::filesystem::path share = ament_index_cpp::get_package_share_path("xsting_nav");
    const std::filesystem::path cand = share / p;
    if (std::filesystem::exists(cand)) {
      return cand.string();
    }
  } catch (const std::exception &) {
  }
  return path_or_rel;
}

bool is_box_class(const std::string & cls)
{
  return cls == "4" || cls == "paper_box" || cls == "box";
}

bool is_puck_class(const std::string & cls)
{
  return cls == "0" || cls == "1" || cls == "2" || cls == "3" ||
         cls.find("square") != std::string::npos ||
         cls.find("cylinder") != std::string::npos ||
         cls.find("puck") != std::string::npos;
}

std::vector<Vec3f> extract_xyz_from_pointcloud2(
  const sensor_msgs::msg::PointCloud2 & msg,
  size_t max_pts = 512)
{
  std::vector<Vec3f> pts;
  const size_t total = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  if (total == 0 || msg.point_step == 0 || msg.data.empty()) {
    return pts;
  }

  int off_x = -1, off_y = -1, off_z = -1;
  for (const auto & f : msg.fields) {
    if (f.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      if (f.name == "x") off_x = static_cast<int>(f.offset);
      else if (f.name == "y") off_y = static_cast<int>(f.offset);
      else if (f.name == "z") off_z = static_cast<int>(f.offset);
    }
  }
  if (off_x < 0 || off_y < 0 || off_z < 0) {
    return pts;
  }

  const size_t stride = std::max<size_t>(1, total / max_pts);
  pts.reserve(std::min(total, max_pts));
  for (size_t i = 0; i < total; i += stride) {
    const size_t base = i * msg.point_step;
    if (base + static_cast<size_t>(std::max({off_x, off_y, off_z}) + 4) > msg.data.size()) {
      break;
    }
    float x = 0.0f, y = 0.0f, z = 0.0f;
    std::memcpy(&x, &msg.data[base + off_x], sizeof(float));
    std::memcpy(&y, &msg.data[base + off_y], sizeof(float));
    std::memcpy(&z, &msg.data[base + off_z], sizeof(float));
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
      pts.push_back({x, y, z});
    }
  }
  return pts;
}

double duration_to_sec(const builtin_interfaces::msg::Duration & d, double fallback_s)
{
  const double v = static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
  return (v > 0.1) ? v : fallback_s;
}
}  // namespace

/// XSting RL Navigation + Box Detection (5~10cm Pre-Approach) -> /approach_box Handoff Node
class NavNode : public rclcpp::Node
{
public:
  using RunTask = xsting_interfaces::action::RunTask;
  using RunGoalHandle = rclcpp_action::ServerGoalHandle<RunTask>;
  using ApproachBox = xsting_interfaces::action::ApproachBox;
  using ApproachGoalHandle = rclcpp_action::ClientGoalHandle<ApproachBox>;
  using ErrorCode = xsting_interfaces::msg::ErrorCode;

  enum class Stage
  {
    Idle,
    Navigating,
    SearchingBox,
    SettlingBeforeApproach,
    CallingApproach,
    ApproachAligned,
    ReachedHold,
    Failed
  };

  NavNode()
  : Node("nav_node")
  {
    // 1. Load policy, static arena ESDF, and real-robot calibration
    const std::string policy_path = resolve_pkg_path(
      declare_parameter<std::string>("policy_json", "config/mk2_nav_policy.json"));
    const std::string arena_path = resolve_pkg_path(
      declare_parameter<std::string>("arena_json", "config/arena_static_esdf.json"));
    calib_path_ = resolve_pkg_path(
      declare_parameter<std::string>("calib_file", "config/calibration.yaml"));
    const std::string profile_override =
      declare_parameter<std::string>("hardware_profile", "");

    policy_.load_json(policy_path);
    planner_.load_arena_json(arena_path);

    if (std::filesystem::exists(calib_path_)) {
      calib_ = CalibrationConfig::load_yaml(calib_path_);
    } else {
      calib_ = CalibrationConfig::make_defaults(
        profile_override.empty() ? "xsting" : profile_override);
    }
    if (!profile_override.empty() && profile_override != calib_.hardware_profile) {
      calib_ = CalibrationConfig::make_defaults(profile_override);
    }

    const auto init_override = declare_parameter<std::vector<double>>(
      "init_pose_arena_override", std::vector<double>{});
    if (init_override.size() == 3) {
      calib_.init_pose_arena = {
        static_cast<float>(init_override[0]),
        static_cast<float>(init_override[1]),
        static_cast<float>(init_override[2])
      };
    }

    // 2. Navigation, 5~10cm box pre-approach, and /approach_box handoff parameters
    const int initial_goal_id = declare_parameter<int>("goal_id", -1);
    human_wait_timeout_s_ = static_cast<float>(
      declare_parameter<double>("human_wait_timeout_s", 10.0));
    default_nav_timeout_s_ = declare_parameter<double>("default_nav_timeout_s", 150.0);

    // Pre-approach tolerance strictly clamped to [0.05, 0.10] m (5~10 cm)
    pre_approach_tol_m_ = std::clamp(
      static_cast<float>(declare_parameter<double>("pre_approach_tol_m", 0.08)),
      0.05f, 0.10f);
    pre_approach_yaw_tol_rad_ = static_cast<float>(
      declare_parameter<double>("pre_approach_yaw_tol_rad", 0.20));
    pre_approach_hold_steps_ = std::max(
      1, static_cast<int>(declare_parameter<int>("pre_approach_hold_steps", 5)));

    goal_pos_tol_m_ = static_cast<float>(
      declare_parameter<double>("goal_pos_tol_m", 0.08));
    goal_yaw_tol_rad_ = static_cast<float>(
      declare_parameter<double>("goal_yaw_tol_rad", 0.18));

    box_vis_timeout_s_ = declare_parameter<double>("box_vis_timeout_s", 0.60);
    box_search_timeout_s_ = declare_parameter<double>("box_search_timeout_s", 5.0);
    box_detect_min_score_ = static_cast<float>(
      declare_parameter<double>("box_detect_min_score", 0.25));
    box_assoc_radius_m_ = static_cast<float>(
      declare_parameter<double>("box_assoc_radius_m", 0.35));
    max_target_refine_m_ = static_cast<float>(
      declare_parameter<double>("max_target_refine_m", 0.25));
    require_box_before_approach_ =
      declare_parameter<bool>("require_box_before_approach", true);
    chain_approach_for_box_goals_ =
      declare_parameter<bool>("chain_approach_for_box_goals", true);
    chain_approach_on_custom_goal_ =
      declare_parameter<bool>("chain_approach_on_custom_goal", false);

    approach_timeout_s_ = declare_parameter<double>("approach_timeout_s", 35.0);
    approach_server_wait_s_ = declare_parameter<double>("approach_server_wait_s", 2.5);

    dry_run_ = declare_parameter<bool>("dry_run", false);
    stand_when_idle_ = declare_parameter<bool>("stand_when_idle", false);
    hold_arm_walking_ = declare_parameter<bool>("hold_arm_walking", true);
    odom_watchdog_timeout_s_ = declare_parameter<double>("odom_watchdog_timeout_s", 0.60);
    max_safe_tilt_deg_ = static_cast<float>(
      declare_parameter<double>("max_safe_tilt_deg", 38.0));

    // Pose & TF parameters
    pose_source_ = declare_parameter<std::string>("pose_source", "odom_calibrated");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    arena_frame_ = declare_parameter<std::string>("arena_frame", "arena");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base");
    publish_arena_tf_ = declare_parameter<bool>("publish_arena_tf", true);
    publish_map_to_odom_mode_ =
      declare_parameter<std::string>("publish_map_to_odom_tf", "auto");

    // 360-deg 2D LiDAR -> static arena ESDF localization & continuous drift correction
    enable_lidar_localization_ =
      declare_parameter<bool>("enable_lidar_localization", true);
    auto_initial_relocalize_ =
      declare_parameter<bool>("auto_initial_relocalize", true);
    loc_update_period_s_ =
      declare_parameter<double>("loc_update_period_s", 0.40);
    loc_pos_alpha_ = static_cast<float>(
      declare_parameter<double>("loc_pos_alpha", 0.25));
    loc_yaw_alpha_ = static_cast<float>(
      declare_parameter<double>("loc_yaw_alpha", 0.20));
    loc_max_step_pos_m_ = static_cast<float>(
      declare_parameter<double>("loc_max_step_pos_m", 0.06));
    loc_max_step_yaw_rad_ = static_cast<float>(
      declare_parameter<double>("loc_max_step_yaw_rad", 0.08));
    loc_min_inliers_ =
      static_cast<int>(declare_parameter<int64_t>("loc_min_inliers", 20));
    loc_max_rmse_m_ = static_cast<float>(
      declare_parameter<double>("loc_max_rmse_m", 0.065));
    map_to_odom_pose_ = calib_.init_pose_arena;

    const float gait_freq = static_cast<float>(declare_parameter<double>("gait_freq_hz", 2.2));
    const float step_h = static_cast<float>(declare_parameter<double>("step_height_m", 0.030));
    const float max_stride = static_cast<float>(declare_parameter<double>("max_stride_m", 0.042));
    gait_ = LegGaitController(gait_freq, step_h, max_stride);
    esdf_slice_ = EsdfSlice::make_clear_default(64, 64, 1.0f);
    last_hw_cmd_ = LegGaitController::sim_to_hardware_legs(Mk2Kinematics::stand_pose(), calib_);

    // 3. Topics, Action Server (/nav/run), and Action Client (/approach_box)
    const bool is_mk2_hw = (calib_.hardware_profile == "mk2_hexapod");
    const std::string leg_topic = declare_parameter<std::string>(
      "leg_cmd_topic", is_mk2_hw ? "/position_controller/commands" : "/leg_controller/commands");
    const std::string arm_topic = declare_parameter<std::string>(
      "arm_cmd_topic", is_mk2_hw ? "" : "/arm_controller/commands");
    const std::string odom_topic = declare_parameter<std::string>(
      "odom_topic", "/visual_slam/tracking/odometry");
    const std::string imu_topic = declare_parameter<std::string>(
      "imu_topic", is_mk2_hw ? "/mk2/front/imu" : "/imu_broadcaster/imu");
    const std::string joint_topic = declare_parameter<std::string>(
      "joint_topic", "/joint_states");
    const std::string esdf_topic = declare_parameter<std::string>(
      "esdf_topic", "/nvblox_node/static_map_slice");
    const std::string lidar_scan_topic = declare_parameter<std::string>(
      "lidar_scan_topic", "/scan_filtered");
    const std::string ultrasonic_topic = declare_parameter<std::string>(
      "ultrasonic_topic", "/sonar_broadcaster/range");
    const std::string det3d_topic = declare_parameter<std::string>(
      "det3d_topic", "/detections_3d");
    const std::string det2d_topic = declare_parameter<std::string>(
      "det2d_topic", "/camera/color/detections");
    const std::string box_cloud_topic = declare_parameter<std::string>(
      "box_cloud_topic", "");
    const std::string nav_action_name = declare_parameter<std::string>(
      "nav_action", "/nav/run");
    approach_action_name_ = declare_parameter<std::string>(
      "approach_action", "/approach_box");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    leg_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(leg_topic, 10);
    if (!arm_topic.empty()) {
      arm_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(arm_topic, 10);
    }
    vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/nav/cmd_vel_body", 10);
    wp_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/nav/active_waypoint", 10);
    reached_pub_ = create_publisher<std_msgs::msg::Bool>("/nav/goal_reached", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/nav/status", 10);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, sensor_qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr m) { on_odom(*m); });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, sensor_qos,
      [this](const sensor_msgs::msg::Imu::SharedPtr m) { on_imu(*m); });
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_topic, sensor_qos,
      [this](const sensor_msgs::msg::JointState::SharedPtr m) { on_joints(*m); });

    if (!det3d_topic.empty()) {
      det3d_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
        det3d_topic, sensor_qos,
        [this](const vision_msgs::msg::Detection3DArray::SharedPtr m) { on_det3d(*m); });
    }
    if (!det2d_topic.empty()) {
      det2d_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        det2d_topic, 10,
        [this](const vision_msgs::msg::Detection2DArray::SharedPtr m) { on_det2d(*m); });
    }
    if (!box_cloud_topic.empty()) {
      box_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        box_cloud_topic, sensor_qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr m) { on_box_cloud(*m); });
    }

    if (!lidar_scan_topic.empty()) {
      lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        lidar_scan_topic, sensor_qos,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr m) {
          const bool self_mask = (m->header.frame_id != "base" && m->header.frame_id != "base_link" &&
            m->ranges.size() > 0);
          auto pts = project_lidar_scan_to_base_xy(
            m->ranges, m->angle_min, m->angle_increment, calib_, m->header.frame_id, self_mask,
            std::max(Mk2Kinematics::kLidarMinRange, m->range_min),
            std::min(Mk2Kinematics::kLidarMaxRange, m->range_max));
          std::lock_guard<std::mutex> lk(mtx_);
          lidar_pts_base_ = std::move(pts);
          last_lidar_time_ = now();
          has_lidar_ = true;
        });
    }
    if (!ultrasonic_topic.empty()) {
      sonar_sub_ = create_subscription<sensor_msgs::msg::Range>(
        ultrasonic_topic, sensor_qos,
        [this](const sensor_msgs::msg::Range::SharedPtr m) {
          std::lock_guard<std::mutex> lk(mtx_);
          ultrasonic_range_m_ = m->range;
          last_sonar_time_ = now();
          has_sonar_ = true;
        });
    }
    occ_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap", rclcpp::QoS(5),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr m) { on_occ_grid(*m); });

#ifdef XSTING_NAV_HAS_NVBLOX_MSGS
    auto nvblox_cb = [this](const nvblox_msgs::msg::DistanceMapSlice::SharedPtr m) {
        on_nvblox_slice(*m);
      };
    nvblox_sub_ = create_subscription<nvblox_msgs::msg::DistanceMapSlice>(
      esdf_topic, sensor_qos, nvblox_cb);
    if (esdf_topic != "/nvblox_node/combined_dynamic_map_slice") {
      nvblox_dyn_sub_ = create_subscription<nvblox_msgs::msg::DistanceMapSlice>(
        "/nvblox_node/combined_dynamic_map_slice", sensor_qos, nvblox_cb);
    }
#endif

    // Topic goal interfaces (standalone / RViz / CLI testing)
    goal_id_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/nav/goal_id", 10,
      [this](const std_msgs::msg::Int32::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (run_goal_) {
          RCLCPP_WARN(get_logger(), "/nav/run 액션 실행 중에는 /nav/goal_id 토픽 요청을 무시합니다.");
          return;
        }
        start_goal_by_id_locked(m->data, chain_approach_for_box_goals_);
      });
    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (run_goal_) {
          RCLCPP_WARN(get_logger(), "/nav/run 액션 실행 중에는 /goal_pose 토픽 요청을 무시합니다.");
          return;
        }
        const auto & p = m->pose.position;
        const auto & q = m->pose.orientation;
        const float yaw = quat_to_yaw(
          static_cast<float>(q.x), static_cast<float>(q.y),
          static_cast<float>(q.z), static_cast<float>(q.w));
        start_custom_goal_locked(
          {static_cast<float>(p.x), static_cast<float>(p.y), yaw},
          "CUSTOM", chain_approach_on_custom_goal_);
      });

    // Services: E-Stop, Stop, Reload Calibration, Manual Start Precision Approach
    estop_srv_ = create_service<std_srvs::srv::Trigger>(
      "/nav/estop",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        cancel_approach_if_active_and_stop(true, "비상 정지(/nav/estop) 호출됨");
        res->success = true;
        res->message = "E-STOP 활성화: 보행 및 정밀 접근 즉시 중단";
      });
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "/nav/stop",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        cancel_approach_if_active_and_stop(false, "정지(/nav/stop) 호출됨");
        res->success = true;
        res->message = "주행 정지 및 대기(Idle) 상태 전환 완료";
      });
    reload_calib_srv_ = create_service<std_srvs::srv::Trigger>(
      "/nav/reload_calib",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        try {
          std::lock_guard<std::mutex> lk(mtx_);
          calib_ = CalibrationConfig::load_yaml(calib_path_);
          map_to_odom_pose_ = calib_.init_pose_arena;
          initial_relocalized_ = false;
          res->success = true;
          res->message = "보정 파일 재로드 완료: " + calib_path_;
          RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
        } catch (const std::exception & e) {
          res->success = false;
          res->message = std::string("보정 재로드 실패: ") + e.what();
        }
      });
    relocalize_srv_ = create_service<std_srvs::srv::Trigger>(
      "/nav/relocalize",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        std::lock_guard<std::mutex> lk(mtx_);
        const rclcpp::Time t_now = now();
        if (!has_odom_ || (t_now - last_odom_time_).seconds() > odom_watchdog_timeout_s_) {
          res->success = false;
          res->message = "유효한 오도메트리(/visual_slam/tracking/odometry) 수신 대기 중";
          return;
        }
        if (!has_lidar_ || lidar_pts_base_.empty() || (t_now - last_lidar_time_).seconds() > 0.60) {
          res->success = false;
          res->message = "유효한 2D LiDAR(/scan_filtered) 스캔 수신 대기 중";
          return;
        }
        const Vec3f eff_odom = calibrated_odom_pose(odom_pose_, calib_);
        const Vec3f prior = current_arena_pose_locked();
        const auto loc = planner_.localize_lidar_on_arena(
          lidar_pts_base_, prior, eff_odom, /*global_search=*/true,
          loc_min_inliers_, loc_max_rmse_m_);
        if (!loc.converged) {
          res->success = false;
          res->message = "전역 스캔 매칭 수렴 실패 (유효 벽면 인라이어 부족 또는 RMSE 초과)";
          return;
        }
        map_to_odom_pose_ = loc.map_to_odom;
        initial_relocalized_ = true;
        last_loc_rmse_m_ = loc.rmse_m;
        last_loc_time_ = t_now;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3)
            << "전역 위치 재정합 완료: pose=(" << loc.arena_pose[0] << ", "
            << loc.arena_pose[1] << ", "
            << (loc.arena_pose[2] * 180.0 / Mk2Kinematics::kPi)
            << " deg) rmse=" << loc.rmse_m << "m inliers=" << loc.inliers;
        res->success = true;
        res->message = oss.str();
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
      });
    start_approach_srv_ = create_service<std_srvs::srv::Trigger>(
      "/nav/start_approach",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        double rem_timeout = approach_timeout_s_;
        uint64_t req_seq = 0;
        {
          std::lock_guard<std::mutex> lk(mtx_);
          estopped_ = false;
          cmd_vel_body_ = {0.0f, 0.0f, 0.0f};
          leg_owner_active_ = false;
          stage_ = Stage::CallingApproach;
          req_seq = prepare_approach_state_locked(&rem_timeout);
        }
        std::string err;
        const bool ok = send_approach_goal_unlocked(req_seq, rem_timeout, &err);
        if (!ok) {
          std::lock_guard<std::mutex> lk(mtx_);
          if (req_seq == approach_req_seq_) {
            stage_ = Stage::Failed;
          }
        }
        res->success = ok;
        res->message = ok ? "/approach_box 정밀 접근 액션 호출 시작" : err;
      });

    // Action Client (/approach_box) & Action Server (/nav/run)
    approach_cli_ = rclcpp_action::create_client<ApproachBox>(this, approach_action_name_);
    run_srv_ = rclcpp_action::create_server<RunTask>(
      this,
      nav_action_name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const RunTask::Goal> goal) {
        return handle_run_goal(goal);
      },
      [this](const std::shared_ptr<RunGoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<RunGoalHandle> gh) {
        accept_run_goal(gh);
      });

    start_time_ = now();
    if (initial_goal_id >= 0) {
      std::lock_guard<std::mutex> lk(mtx_);
      start_goal_by_id_locked(initial_goal_id, chain_approach_for_box_goals_);
    }

    policy_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() { tick_policy_20hz(); });
    gait_timer_ = create_wall_timer(
      std::chrono::milliseconds(10), [this]() { tick_gait_100hz(); });

    RCLCPP_INFO(
      get_logger(),
      "nav_node 준비 완료 (action='%s' -> '%s' | pre_approach_tol=%.2fm [5~10cm] | dry_run=%s)",
      nav_action_name.c_str(), approach_action_name_.c_str(),
      pre_approach_tol_m_, dry_run_ ? "true" : "false");
  }

private:
  // ==================== Action Server (/nav/run) ====================
  rclcpp_action::GoalResponse handle_run_goal(std::shared_ptr<const RunTask::Goal> goal)
  {
    if (!goal->operation.empty() && goal->operation != "NAVIGATE") {
      RCLCPP_WARN(get_logger(), "지원하지 않는 RunTask operation 거부: '%s'", goal->operation.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void accept_run_goal(const std::shared_ptr<RunGoalHandle> & gh)
  {
    ApproachGoalHandle::SharedPtr cancel_prev_app;
    std::shared_ptr<RunGoalHandle> prev_run;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (run_goal_ && run_goal_->is_active()) {
        prev_run = run_goal_;
        cancel_prev_app = approach_goal_handle_;
        approach_goal_handle_.reset();
      }
      estopped_ = false;
      run_goal_ = gh;
      run_start_time_ = now();
      const auto req = gh->get_goal();
      run_timeout_s_ = duration_to_sec(req->timeout, default_nav_timeout_s_);

      const int gid = DetourPlanner::place_id_to_goal_id(req->place_id);
      if (gid >= 0) {
        const bool is_box = DetourPlanner::is_box_goal_id(gid) && chain_approach_for_box_goals_;
        start_goal_by_id_locked(gid, is_box);
        if (gid == 0) {
          // If HOME destination pose was explicitly supplied in meters, respect valid arena coordinates.
          const auto & p = req->destination.pose.position;
          const auto & q = req->destination.pose.orientation;
          const double qn = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
          if (qn > 0.5 && p.x > 0.1 && p.x < 4.3 && p.y > 0.1 && p.y < 6.2) {
            const float yaw = quat_to_yaw(
              static_cast<float>(q.x), static_cast<float>(q.y),
              static_cast<float>(q.z), static_cast<float>(q.w));
            nominal_goal_ = {static_cast<float>(p.x), static_cast<float>(p.y), yaw};
            const Vec3f cur_arena = current_arena_pose_locked();
            planner_.update_goal_pose_preserving_blocks(
              nominal_goal_, {cur_arena[0], cur_arena[1]});
          }
        }
      } else if (req->place_id.empty()) {
        const auto & p = req->destination.pose.position;
        const auto & q = req->destination.pose.orientation;
        const float yaw = quat_to_yaw(
          static_cast<float>(q.x), static_cast<float>(q.y),
          static_cast<float>(q.z), static_cast<float>(q.w));
        start_custom_goal_locked(
          {static_cast<float>(p.x), static_cast<float>(p.y), yaw},
          "CUSTOM", chain_approach_on_custom_goal_);
      } else {
        // Unknown place_id -> reject immediately inside accept via abort
        auto res = std::make_shared<RunTask::Result>();
        res->success = false;
        res->error_code = ErrorCode::BAD_REQUEST;
        res->message = "알 수 없는 place_id: " + req->place_id;
        run_goal_.reset();
        stage_ = Stage::Failed;
        gh->abort(res);
        return;
      }

      RCLCPP_INFO(
        get_logger(),
        "[/nav/run 수락] place='%s' (goal_id=%d, box_approach=%s, timeout=%.1fs)",
        active_place_id_.c_str(), planner_.goal_id(),
        use_box_approach_ ? "true" : "false", run_timeout_s_);
    }

    if (cancel_prev_app) {
      approach_cli_->async_cancel_goal(cancel_prev_app);
    }
    if (prev_run && prev_run->is_active()) {
      auto res = std::make_shared<RunTask::Result>();
      res->success = false;
      res->error_code = ErrorCode::CANCELED;
      res->message = "새 주행 목표로 대체됨";
      prev_run->abort(res);
    }
  }

  // ==================== Goal Initialization & Ownership ====================
  Vec3f current_arena_pose_locked() const
  {
    if (pose_source_ == "tf_map" && has_tf_map_pose_) {
      return tf_map_pose_;
    }
    return compose_se2(map_to_odom_pose_, calibrated_odom_pose(odom_pose_, calib_));
  }

  void acquire_leg_ownership_locked()
  {
    if (!leg_owner_active_) {
      if (has_joints_) {
        last_hw_cmd_ = latest_hw_joints_;
        gait_.sync_from_hardware(latest_hw_joints_, calib_);
      } else {
        gait_.reset();
        last_hw_cmd_ = LegGaitController::sim_to_hardware_legs(
          Mk2Kinematics::stand_pose(), calib_);
      }
      owner_engage_time_ = now();
      leg_owner_active_ = true;
    }
  }

  void reset_box_tracking_locked()
  {
    box_seen_for_goal_ = false;
    has_refined_box_ = false;
    pre_approach_hold_count_ = 0;
    goal_hold_count_ = 0;
    approach_ever_visible_ = false;
    approach_finished_ = false;
    approach_ok_ = false;
    approach_canceled_ = false;
    approach_timed_out_ = false;
    approach_result_msg_.clear();
  }

  void start_goal_by_id_locked(int gid, bool use_box_approach)
  {
    const Vec3f cur = current_arena_pose_locked();
    planner_.set_goal_by_id(gid, {cur[0], cur[1]});
    nominal_goal_ = planner_.goal();
    active_place_id_ = DetourPlanner::goal_id_to_place_id(planner_.goal_id());
    has_nominal_box_ = planner_.nominal_box_xy(planner_.goal_id(), &nominal_box_xy_);
    use_box_approach_ = use_box_approach && DetourPlanner::is_box_goal_id(planner_.goal_id());
    initial_goal_dist_ = std::max(
      0.20f, std::hypot(nominal_goal_[0] - cur[0], nominal_goal_[1] - cur[1]));
    reset_box_tracking_locked();
    acquire_leg_ownership_locked();
    stage_ = Stage::Navigating;
    publish_reached(false);
  }

  void start_custom_goal_locked(
    const Vec3f & pose, const std::string & label, bool use_box_approach)
  {
    const Vec3f cur = current_arena_pose_locked();
    planner_.set_custom_goal(pose, {cur[0], cur[1]});
    nominal_goal_ = pose;
    active_place_id_ = label;
    has_nominal_box_ = false;
    use_box_approach_ = use_box_approach;
    initial_goal_dist_ = std::max(
      0.20f, std::hypot(nominal_goal_[0] - cur[0], nominal_goal_[1] - cur[1]));
    reset_box_tracking_locked();
    acquire_leg_ownership_locked();
    stage_ = Stage::Navigating;
    publish_reached(false);
  }

  // ==================== Sensor Callbacks ====================
  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    const float raw_yaw = quat_to_yaw(
      static_cast<float>(q.x), static_cast<float>(q.y),
      static_cast<float>(q.z), static_cast<float>(q.w));

    const auto & v = msg.twist.twist.linear;
    const auto & w = msg.twist.twist.angular;
    Vec3f body_vel{
      static_cast<float>(v.x),
      static_cast<float>(v.y),
      static_cast<float>(w.z)
    };
    if (msg.child_frame_id.find("odom") != std::string::npos ||
        msg.child_frame_id.find("map") != std::string::npos)
    {
      const float c = std::cos(raw_yaw);
      const float s = std::sin(raw_yaw);
      body_vel[0] = static_cast<float>(c * v.x + s * v.y);
      body_vel[1] = static_cast<float>(-s * v.x + c * v.y);
    }

    std::lock_guard<std::mutex> lk(mtx_);
    odom_pose_ = {static_cast<float>(p.x), static_cast<float>(p.y), raw_yaw};
    odom_vel_ = body_vel;
    last_odom_time_ = now();
    has_odom_ = true;
  }

  void on_imu(const sensor_msgs::msg::Imu & msg)
  {
    const Vec3f raw_g{
      static_cast<float>(msg.angular_velocity.x),
      static_cast<float>(msg.angular_velocity.y),
      static_cast<float>(msg.angular_velocity.z)
    };
    const Vec4f raw_q{
      static_cast<float>(msg.orientation.w),
      static_cast<float>(msg.orientation.x),
      static_cast<float>(msg.orientation.y),
      static_cast<float>(msg.orientation.z)
    };
    std::lock_guard<std::mutex> lk(mtx_);
    imu_state_ = transform_calibrated_imu(raw_g, raw_q, calib_);
    has_imu_ = true;
  }

  void on_joints(const sensor_msgs::msg::JointState & msg)
  {
    std::unordered_map<std::string, float> jmap;
    const size_t n = std::min(msg.name.size(), msg.position.size());
    for (size_t i = 0; i < n; ++i) {
      jmap[msg.name[i]] = static_cast<float>(msg.position[i]);
    }
    const auto sim_names = Mk2Kinematics::sim_leg_joint_names();
    const auto mk2_names = Mk2Kinematics::mk2_hw_joint_names();
    Leg18f cur{};
    int found = 0;
    for (size_t i = 0; i < 18; ++i) {
      auto it = jmap.find(sim_names[i]);
      if (it == jmap.end()) it = jmap.find(mk2_names[i]);
      if (it != jmap.end() && std::isfinite(it->second)) {
        cur[i] = it->second;
        ++found;
      }
    }
    if (found == 18) {
      std::lock_guard<std::mutex> lk(mtx_);
      latest_hw_joints_ = cur;
      if (!has_joints_) {
        last_hw_cmd_ = cur;
        gait_.sync_from_hardware(cur, calib_);
        has_joints_ = true;
      }
    }
  }

  void on_det3d(const vision_msgs::msg::Detection3DArray & msg)
  {
    // Parse class-separated 3D detections from xsting_vision::DepthPcaNode (in base frame)
    bool found_box = false;
    float best_box_dist = 1e9f;
    Vec2f best_box_xy{0.0f, 0.0f};
    float best_box_yaw = 0.0f;
    std::vector<Vec2f> puck_pts;

    const bool is_optical = (msg.header.frame_id.find("optical") != std::string::npos);

    for (const auto & det : msg.detections) {
      if (det.results.empty()) continue;
      const std::string & cls = det.results[0].hypothesis.class_id;
      const double score = det.results[0].hypothesis.score;

      Vec3f pos{
        static_cast<float>(det.bbox.center.position.x),
        static_cast<float>(det.bbox.center.position.y),
        static_cast<float>(det.bbox.center.position.z)
      };
      if (!std::isfinite(pos[0]) || !std::isfinite(pos[1])) continue;
      if (is_optical) {
        pos = {
          pos[2] + calib_.cam_pos_base_xyz[0],
          -pos[0] + calib_.cam_pos_base_xyz[1],
          -pos[1] + calib_.cam_pos_base_xyz[2]
        };
      }

      if (is_box_class(cls) && score >= box_detect_min_score_) {
        const float d = std::hypot(pos[0], pos[1]);
        if (pos[0] >= 0.13f && d <= 2.40f && std::abs(pos[1]) <= 1.30f && d < best_box_dist) {
          best_box_dist = d;
          best_box_xy = {pos[0], pos[1]};
          const auto & o = det.bbox.center.orientation;
          best_box_yaw = quat_to_yaw(
            static_cast<float>(o.x), static_cast<float>(o.y),
            static_cast<float>(o.z), static_cast<float>(o.w));
          found_box = true;
        }
      } else if (is_puck_class(cls) && score >= 0.20) {
        if (pos[0] >= 0.10f && pos[0] <= 2.40f && std::abs(pos[1]) <= 1.30f) {
          puck_pts.push_back({pos[0], pos[1]});
        }
      }
    }

    const rclcpp::Time t_now = now();
    std::lock_guard<std::mutex> lk(mtx_);
    has_det3d_stream_ = true;
    if (found_box) {
      latest_box_det_base_ = best_box_xy;
      latest_box_yaw_base_ = best_box_yaw;
      last_box_3d_time_ = t_now;
      has_box_3d_ = true;
      box_pts_ = synthesize_box_front_points(best_box_xy, best_box_yaw);
    }
    if (!puck_pts.empty()) {
      // Ensure >= 3 points if at least 1 puck seen so build_obs_41d puck features activate cleanly
      while (puck_pts.size() < 3) {
        const Vec2f p0 = puck_pts.front();
        puck_pts.push_back({p0[0], p0[1] + 0.01f * static_cast<float>(puck_pts.size())});
      }
      puck_pts_ = std::move(puck_pts);
      last_puck_time_ = t_now;
    }
  }

  void on_det2d(const vision_msgs::msg::Detection2DArray & msg)
  {
    for (const auto & det : msg.detections) {
      if (det.results.empty()) continue;
      const auto & hyp = det.results[0].hypothesis;
      if (is_box_class(hyp.class_id) && hyp.score >= box_detect_min_score_) {
        std::lock_guard<std::mutex> lk(mtx_);
        last_box_2d_time_ = now();
        has_box_2d_ = true;
        return;
      }
    }
  }

  void on_box_cloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    auto raw = extract_xyz_from_pointcloud2(msg);
    std::lock_guard<std::mutex> lk(mtx_);
    if (!has_det3d_stream_) {
      box_pts_ = transform_yolo_points_xy(raw, calib_);
      if (box_pts_.size() >= 3) {
        float cx = 0.0f, cy = 0.0f;
        for (const auto & p : box_pts_) {
          cx += p[0];
          cy += p[1];
        }
        cx /= static_cast<float>(box_pts_.size());
        cy /= static_cast<float>(box_pts_.size());
        latest_box_det_base_ = {cx + Mk2Kinematics::kBoxHalfM, cy};
        last_box_3d_time_ = now();
        has_box_3d_ = true;
      }
    }
  }

  void on_occ_grid(const nav_msgs::msg::OccupancyGrid & msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (has_native_nvblox_) {
      return;
    }
    esdf_slice_ = EsdfSlice::from_occupancy_grid(
      static_cast<int>(msg.info.width),
      static_cast<int>(msg.info.height),
      static_cast<float>(msg.info.origin.position.x),
      static_cast<float>(msg.info.origin.position.y),
      msg.info.resolution,
      msg.data);
  }

#ifdef XSTING_NAV_HAS_NVBLOX_MSGS
  void on_nvblox_slice(const nvblox_msgs::msg::DistanceMapSlice & msg)
  {
    if (msg.width == 0 || msg.height == 0 || msg.data.empty()) {
      return;
    }
    std::lock_guard<std::mutex> lk(mtx_);
    esdf_slice_.width = static_cast<int>(msg.width);
    esdf_slice_.height = static_cast<int>(msg.height);
    esdf_slice_.origin_x = static_cast<float>(msg.origin.x);
    esdf_slice_.origin_y = static_cast<float>(msg.origin.y);
    esdf_slice_.resolution = msg.resolution;
    esdf_slice_.unknown_value = msg.unknown_value;
    esdf_slice_.data = msg.data;
    has_native_nvblox_ = true;
  }
#endif

  // ==================== /approach_box Action Client ====================
  uint64_t prepare_approach_state_locked(double * rem_timeout_out)
  {
    double rem_timeout = approach_timeout_s_;
    if (run_goal_) {
      const double elapsed = (now() - run_start_time_).seconds();
      rem_timeout = std::clamp(run_timeout_s_ - elapsed, 8.0, approach_timeout_s_);
    }
    if (rem_timeout_out) {
      *rem_timeout_out = rem_timeout;
    }

    approach_finished_ = false;
    approach_ok_ = false;
    approach_canceled_ = false;
    approach_timed_out_ = false;
    approach_ever_visible_ = box_seen_for_goal_;
    approach_fb_dist_m_ = 0.0f;
    approach_fb_yaw_deg_ = 0.0f;
    approach_fb_hold_s_ = 0.0f;
    approach_goal_handle_.reset();
    return ++approach_req_seq_;
  }

  bool send_approach_goal_unlocked(
    uint64_t req_seq, double rem_timeout, std::string * err_out)
  {
    if (!approach_cli_->action_server_is_ready()) {
      if (!approach_cli_->wait_for_action_server(
          std::chrono::duration<double>(approach_server_wait_s_)))
      {
        if (err_out) {
          *err_out = "/approach_box 액션 서버가 실행 중이지 않습니다";
        }
        return false;
      }
    }

    ApproachBox::Goal goal;
    goal.timeout_sec = static_cast<float>(rem_timeout);
    goal.dry_run = dry_run_;

    rclcpp_action::Client<ApproachBox>::SendGoalOptions opts;
    opts.goal_response_callback =
      [this, req_seq](const ApproachGoalHandle::SharedPtr & gh) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (req_seq != approach_req_seq_) return;
        if (!gh) {
          approach_finished_ = true;
          approach_ok_ = false;
          approach_result_msg_ = "/approach_box 목표가 거부되었습니다";
          return;
        }
        approach_goal_handle_ = gh;
      };

    opts.feedback_callback =
      [this, req_seq](
      ApproachGoalHandle::SharedPtr,
      const std::shared_ptr<const ApproachBox::Feedback> fb) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (req_seq != approach_req_seq_) return;
        approach_fb_dist_m_ = fb->dist_m;
        approach_fb_yaw_deg_ = fb->yaw_deg;
        approach_fb_hold_s_ = fb->hold_sec;
        if (fb->visible) {
          approach_ever_visible_ = true;
        }
      };

    opts.result_callback =
      [this, req_seq](const ApproachGoalHandle::WrappedResult & wr) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (req_seq != approach_req_seq_) return;
        approach_goal_handle_.reset();
        approach_finished_ = true;
        if (wr.code == rclcpp_action::ResultCode::SUCCEEDED && wr.result && wr.result->success) {
          approach_ok_ = true;
          approach_fb_dist_m_ = wr.result->dist_m;
          approach_fb_yaw_deg_ = wr.result->yaw_deg;
          std::ostringstream oss;
          oss << std::fixed << std::setprecision(3)
              << wr.result->message << " (dist=" << wr.result->dist_m
              << "m, yaw=" << std::setprecision(1) << wr.result->yaw_deg << "deg)";
          approach_result_msg_ = oss.str();
        } else if (wr.code == rclcpp_action::ResultCode::CANCELED) {
          approach_canceled_ = true;
          approach_result_msg_ = wr.result ? wr.result->message : "정밀 접근 취소됨";
        } else {
          approach_ok_ = false;
          if (wr.result) {
            approach_fb_dist_m_ = wr.result->dist_m;
            approach_fb_yaw_deg_ = wr.result->yaw_deg;
            approach_result_msg_ = wr.result->message;
            if (wr.result->message.find("시간 초과") != std::string::npos ||
                wr.result->message.find("timeout") != std::string::npos)
            {
              approach_timed_out_ = true;
            }
          } else {
            approach_result_msg_ = "/approach_box 비정상 종료";
          }
        }
      };

    approach_cli_->async_send_goal(goal, opts);
    RCLCPP_INFO(
      get_logger(),
      "[정밀 접근 전환] 상자 검출 및 %.1fcm 이내 도달 완료 -> %s 호출 (timeout=%.1fs)",
      pre_approach_tol_m_ * 100.0f, approach_action_name_.c_str(), rem_timeout);
    return true;
  }

  void cancel_approach_if_active_and_stop(bool is_estop, const std::string & reason)
  {
    ApproachGoalHandle::SharedPtr cancel_app;
    std::shared_ptr<RunGoalHandle> abort_run;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (is_estop) {
        estopped_ = true;
      }
      ++approach_req_seq_;
      cancel_app = approach_goal_handle_;
      approach_goal_handle_.reset();
      cmd_vel_body_ = {0.0f, 0.0f, 0.0f};
      leg_owner_active_ = false;
      stage_ = Stage::Idle;
      if (run_goal_ && run_goal_->is_active()) {
        abort_run = run_goal_;
        run_goal_.reset();
      }
    }
    if (cancel_app) {
      approach_cli_->async_cancel_goal(cancel_app);
    }
    if (abort_run) {
      auto res = std::make_shared<RunTask::Result>();
      res->success = false;
      res->error_code = ErrorCode::CANCELED;
      res->message = reason;
      abort_run->abort(res);
    }
  }

  // ==================== TF Helpers ====================
  void update_tf_and_broadcast(const rclcpp::Time & t_now)
  {
    // 1. Broadcast arena -> odom (and optionally map -> odom when no SLAM map->odom publisher exists)
    Vec3f tf_pose{};
    {
      std::lock_guard<std::mutex> lk(mtx_);
      tf_pose = map_to_odom_pose_;
    }
    const float syaw = tf_pose[2];
    const float half_yaw = 0.5f * syaw;
    auto make_tf = [&](const std::string & parent, const std::string & child) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = t_now;
        tf.header.frame_id = parent;
        tf.child_frame_id = child;
        tf.transform.translation.x = tf_pose[0];
        tf.transform.translation.y = tf_pose[1];
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = std::sin(half_yaw);
        tf.transform.rotation.w = std::cos(half_yaw);
        return tf;
      };

    if (publish_arena_tf_ && !arena_frame_.empty() && arena_frame_ != odom_frame_) {
      tf_broadcaster_->sendTransform(make_tf(arena_frame_, odom_frame_));
    }

    if (publish_map_to_odom_mode_ == "true") {
      own_map_to_odom_tf_ = true;
    } else if (publish_map_to_odom_mode_ == "auto" && !checked_external_map_tf_) {
      if ((t_now - start_time_).seconds() > 0.5) {
        const bool ext_exists = tf_buffer_->canTransform(
          map_frame_, odom_frame_, tf2::TimePointZero);
        own_map_to_odom_tf_ = !ext_exists;
        checked_external_map_tf_ = true;
      }
    }
    if (own_map_to_odom_tf_ && map_frame_ != odom_frame_) {
      tf_broadcaster_->sendTransform(make_tf(map_frame_, odom_frame_));
    }

    // 2. Optional TF map -> base pose lookup
    if (pose_source_ == "tf_map") {
      try {
        const auto tf = tf_buffer_->lookupTransform(
          map_frame_, base_frame_, tf2::TimePointZero);
        const auto & t = tf.transform.translation;
        const auto & r = tf.transform.rotation;
        std::lock_guard<std::mutex> lk(mtx_);
        tf_map_pose_ = {
          static_cast<float>(t.x),
          static_cast<float>(t.y),
          quat_to_yaw(
            static_cast<float>(r.x), static_cast<float>(r.y),
            static_cast<float>(r.z), static_cast<float>(r.w))
        };
        last_tf_map_time_ = t_now;
        has_tf_map_pose_ = true;
      } catch (const std::exception &) {
      }
    }
  }

  void publish_reached(bool val)
  {
    std_msgs::msg::Bool m;
    m.data = val;
    reached_pub_->publish(m);
  }

  // ==================== 20 Hz Policy & State Machine Loop ====================
  void tick_policy_20hz()
  {
    const rclcpp::Time t_now = now();
    update_tf_and_broadcast(t_now);

    std::shared_ptr<RunGoalHandle> finish_gh;
    RunTask::Result::SharedPtr finish_res;
    bool finish_as_canceled = false;
    bool finish_as_succeed = false;
    ApproachGoalHandle::SharedPtr cancel_app_handle;

    std::shared_ptr<RunGoalHandle> fb_gh;
    std::string fb_phase;
    float fb_progress = 0.0f;

    bool need_send_approach = false;
    uint64_t pending_approach_seq = 0;
    double pending_approach_timeout_s = approach_timeout_s_;

    Vec3f pub_vel{0.0f, 0.0f, 0.0f};
    Vec3f pub_wp{0.0f, 0.0f, 0.0f};
    bool has_pub_wp = false;
    std::string status_str;
    bool emit_reached_true = false;

    {
      std::lock_guard<std::mutex> lk(mtx_);

      // Helper to schedule RunTask completion outside mutex
      auto queue_run_finish = [&](
        bool ok, uint16_t err_code, const std::string & msg, bool canceled = false) {
          if (!run_goal_ || !run_goal_->is_active()) return;
          finish_gh = run_goal_;
          run_goal_.reset();
          finish_res = std::make_shared<RunTask::Result>();
          finish_res->success = ok;
          finish_res->error_code = err_code;
          finish_res->message = msg;
          finish_as_canceled = canceled;
          finish_as_succeed = ok && !canceled;
        };

      // 1. Handle active /nav/run cancellation & global timeout
      if (run_goal_ && run_goal_->is_active()) {
        if (run_goal_->is_canceling()) {
          ++approach_req_seq_;
          cancel_app_handle = approach_goal_handle_;
          approach_goal_handle_.reset();
          cmd_vel_body_ = {0.0f, 0.0f, 0.0f};
          leg_owner_active_ = false;
          stage_ = Stage::Idle;
          queue_run_finish(false, ErrorCode::CANCELED, "주행 작업이 취소되었습니다", true);
        } else if ((t_now - run_start_time_).seconds() > run_timeout_s_) {
          ++approach_req_seq_;
          cancel_app_handle = approach_goal_handle_;
          approach_goal_handle_.reset();
          cmd_vel_body_ = {0.0f, 0.0f, 0.0f};
          leg_owner_active_ = false;
          stage_ = Stage::Failed;
          queue_run_finish(false, ErrorCode::TIMEOUT, "주행 제한 시간 초과");
        }
      }

      // 2. Check IMU rollover protection
      const float tilt_rad = std::acos(std::clamp(-imu_state_[5], -1.0f, 1.0f));
      const float tilt_deg = static_cast<float>(tilt_rad * 180.0 / Mk2Kinematics::kPi);
      if (has_imu_ && tilt_deg > max_safe_tilt_deg_) {
        estopped_ = true;
        cmd_vel_body_ = {0.0f, 0.0f, 0.0f};
        leg_owner_active_ = false;
        if (stage_ != Stage::Failed) {
          RCLCPP_ERROR(
            get_logger(),
            "기체 과기울기 감지(%.1f deg > %.1f deg)! 다리 출력을 즉시 차단합니다.",
            tilt_deg, max_safe_tilt_deg_);
        }
        stage_ = Stage::Failed;
        queue_run_finish(false, ErrorCode::NAV_FAILED, "기체 과기울기 안전 차단");
      }

      // 3. Sensor freshness checks
      const bool odom_fresh =
        (pose_source_ == "tf_map")
        ? (has_tf_map_pose_ && (t_now - last_tf_map_time_).seconds() <= odom_watchdog_timeout_s_)
        : (has_odom_ && (t_now - last_odom_time_).seconds() <= odom_watchdog_timeout_s_);
      const bool box_3d_fresh =
        has_box_3d_ && ((t_now - last_box_3d_time_).seconds() <= box_vis_timeout_s_);
      const bool box_2d_fresh =
        has_box_2d_ && ((t_now - last_box_2d_time_).seconds() <= box_vis_timeout_s_);
      if (!box_3d_fresh) {
        box_pts_.clear();
      }
      if (!has_puck_fresh(t_now)) {
        puck_pts_.clear();
      }
      if (!has_lidar_ || (t_now - last_lidar_time_).seconds() > 0.50) {
        lidar_pts_base_.clear();
      }
      const float fresh_sonar_m =
        (has_sonar_ && (t_now - last_sonar_time_).seconds() <= 0.50) ? ultrasonic_range_m_ : -1.0f;

      // 3b. Global initial alignment & continuous drift correction (2D LiDAR <-> static arena ESDF)
      if (enable_lidar_localization_ && pose_source_ != "tf_map" &&
          odom_fresh && !lidar_pts_base_.empty() &&
          (!has_imu_ || tilt_deg < 16.0f))
      {
        const Vec3f eff_odom_loc = calibrated_odom_pose(odom_pose_, calib_);
        const Vec3f prior_arena = current_arena_pose_locked();
        if (auto_initial_relocalize_ && !initial_relocalized_ &&
            (stage_ == Stage::Idle || stage_ == Stage::Navigating))
        {
          const auto loc = planner_.localize_lidar_on_arena(
            lidar_pts_base_, prior_arena, eff_odom_loc, /*global_search=*/true,
            loc_min_inliers_, loc_max_rmse_m_);
          if (loc.converged) {
            map_to_odom_pose_ = loc.map_to_odom;
            initial_relocalized_ = true;
            last_loc_rmse_m_ = loc.rmse_m;
            last_loc_time_ = t_now;
            RCLCPP_INFO(
              get_logger(),
              "[초기 전역 정합 완료] arena_pose=(%.3f, %.3f, %.1f deg) rmse=%.3fm inliers=%d",
              loc.arena_pose[0], loc.arena_pose[1],
              loc.arena_pose[2] * 180.0 / Mk2Kinematics::kPi,
              loc.rmse_m, loc.inliers);
          }
        } else if ((stage_ == Stage::Idle || stage_ == Stage::Navigating ||
                    stage_ == Stage::ReachedHold) &&
                   std::abs(odom_vel_[2]) < 0.85f &&
                   (t_now - last_loc_time_).seconds() >= loc_update_period_s_)
        {
          const auto loc = planner_.localize_lidar_on_arena(
            lidar_pts_base_, prior_arena, eff_odom_loc, /*global_search=*/false,
            loc_min_inliers_, loc_max_rmse_m_);
          if (loc.converged) {
            map_to_odom_pose_ = blend_se2(
              map_to_odom_pose_, loc.map_to_odom,
              loc_pos_alpha_, loc_yaw_alpha_,
              loc_max_step_pos_m_, loc_max_step_yaw_rad_);
            initial_relocalized_ = true;
            last_loc_rmse_m_ = loc.rmse_m;
          }
          last_loc_time_ = t_now;
        }
      }

      const Vec3f arena_pose = current_arena_pose_locked();
      const float mx = arena_pose[0];
      const float my = arena_pose[1];
      const float myaw = arena_pose[2];

      // 4. Update dynamic box pre-approach target (~20cm bumper gap) when paper_box 3D is detected
      if (use_box_approach_ && box_3d_fresh && odom_fresh &&
          (stage_ == Stage::Navigating || stage_ == Stage::SearchingBox))
      {
        const Vec2f ref_box_xy = has_nominal_box_
          ? nominal_box_xy_
          : Vec2f{
              nominal_goal_[0] + Mk2Kinematics::kNominalPreApproachStandoffM * std::cos(nominal_goal_[2]),
              nominal_goal_[1] + Mk2Kinematics::kNominalPreApproachStandoffM * std::sin(nominal_goal_[2])
            };
        const auto ref = refine_box_pre_approach_goal(
          nominal_goal_, ref_box_xy, arena_pose, latest_box_det_base_,
          box_assoc_radius_m_, max_target_refine_m_);
        if (ref.accepted) {
          box_seen_for_goal_ = true;
          if (!has_refined_box_) {
            smoothed_refined_goal_ = ref.refined_goal;
            has_refined_box_ = true;
          } else {
            smoothed_refined_goal_[0] =
              0.65f * smoothed_refined_goal_[0] + 0.35f * ref.refined_goal[0];
            smoothed_refined_goal_[1] =
              0.65f * smoothed_refined_goal_[1] + 0.35f * ref.refined_goal[1];
            smoothed_refined_goal_[2] = ref.refined_goal[2];
          }
          const Vec3f cur_g = planner_.goal();
          if (std::hypot(smoothed_refined_goal_[0] - cur_g[0],
                         smoothed_refined_goal_[1] - cur_g[1]) > 0.015f)
          {
            planner_.update_goal_pose_preserving_blocks(smoothed_refined_goal_, {mx, my});
          }
        } else if (std::hypot(nominal_goal_[0] - mx, nominal_goal_[1] - my) < 0.85f) {
          // Close to target corridor and seeing a valid front paper_box
          box_seen_for_goal_ = true;
        }
      }

      const bool box_visible_now =
        box_3d_fresh || (box_seen_for_goal_ && box_2d_fresh);

      const Vec3f active_goal = planner_.goal();
      const float dist_to_goal = std::hypot(active_goal[0] - mx, active_goal[1] - my);
      const float yaw_err = wrap_to_pi(active_goal[2] - myaw);

      // 5. Execute stage machine
      switch (stage_) {
        case Stage::Idle:
        case Stage::ApproachAligned:
        case Stage::Failed: {
          cmd_vel_body_ = {0.0f, 0.0f, 0.0f};
          if (!stand_when_idle_) {
            leg_owner_active_ = false;
          }
          break;
        }

        case Stage::ReachedHold: {
          cmd_vel_body_ = {0.0f, 0.0f, 0.0f};
          if (settle_ticks_20hz_ > 0) {
            --settle_ticks_20hz_;
          } else if (!stand_when_idle_) {
            leg_owner_active_ = false;
          }
          break;
        }

        case Stage::Navigating: {
          if (estopped_ || !odom_fresh) {
            cmd_vel_body_ = {0.0f, 0.0f, 0.0f};
            break;
          }

          // Prepare effective odom pose consistent with build_obs_41d
          const Vec3f eff_odom = (pose_source_ == "tf_map")
            ? arena_to_odom_pose(arena_pose, map_to_odom_pose_)
            : calibrated_odom_pose(odom_pose_, calib_);

          const Vec3f raw_wp = planner_.raw_waypoint(mx, my);
          Obs41f probe_obs = build_obs_41d(
            eff_odom, odom_vel_, active_goal, esdf_slice_, box_pts_, puck_pts_,
            map_to_odom_pose_, imu_state_, raw_wp,
            1.0f, calib_.esdf_frame_mode, calib_.esdf_dist_offset_m);
          fuse_lidar_ultrasonic_into_obs(
            probe_obs, lidar_pts_base_, fresh_sonar_m, 1.0f, calib_.ultrasonic_pos_base_xyz);

          const float arrival_tol = use_box_approach_ ? pre_approach_tol_m_ : goal_pos_tol_m_;
          const bool allow_random_reroute = (run_goal_ == nullptr);
          planner_.update_blocking_replan(
            mx, my, myaw, probe_obs, 0.05f, human_wait_timeout_s_,
            arrival_tol, allow_random_reroute);

          if (run_goal_ && planner_.is_goal_unreachable(mx, my)) {
            cmd_vel_body_ = {0.0f, 0.0f, 0.0f};
            leg_owner_active_ = false;
            stage_ = Stage::Failed;
            queue_run_finish(
              false, ErrorCode::OBSTACLE_BLOCKED,
              active_place_id_ + " 경로가 장애물로 완전히 차단되었습니다");
            break;
          }

          const Vec3f wp = planner_.active_waypoint(mx, my, myaw);
          pub_wp = wp;
          has_pub_wp = true;

          if (use_box_approach_) {
            // === Box Goal (S1~S6, L1~L3): Check 5~10cm arrival + box detection ===
            if (should_trigger_precision_approach(
                arena_pose, active_goal, box_visible_now,
                pre_approach_tol_m_, pre_approach_yaw_tol_rad_))
            {
              cmd_vel_body_ = {0.0f, 0.0f, 0.0f};
              ++pre_approach_hold_count_;
              if (pre_approach_hold_count_ >= pre_approach_hold_steps_) {
                settle_ticks_20hz_ = 5;  // 0.25s stand-pose settle before handing off legs
                stage_ = Stage::SettlingBeforeApproach;
              }
              break;
            }
            pre_approach_hold_count_ = 0;

            if (dist_to_goal <= pre_approach_tol_m_) {
              if (std::abs(yaw_err) > pre_approach_yaw_tol_rad_) {
                // Inside 5~10cm circle: rotate purely in place toward box heading
                cmd_vel_body_ = {
                  0.0f,
                  0.0f,
                  std::clamp(1.6f * yaw_err, -0.75f, 0.75f)
                };
              } else if (!box_visible_now) {
                // Reached within 5~10cm & yaw-aligned, scan gently to confirm box visually
                search_start_time_ = t_now;
                cmd_vel_body_ = {0.0f, 0.0f, 0.0f};
                stage_ = Stage::SearchingBox;
              }
              break;
            }
          } else {
            // === Non-Box Goal (HOME / Custom): Standard pose arrival ===
            if (dist_to_goal <= goal_pos_tol_m_ && std::abs(yaw_err) <= goal_yaw_tol_rad_) {
              cmd_vel_body_ = {0.0f, 0.0f, 0.0f};
              ++goal_hold_count_;
              if (goal_hold_count_ >= pre_approach_hold_steps_) {
                settle_ticks_20hz_ = 8;  // 0.40s stand settle, then yield if !stand_when_idle_
                stage_ = Stage::ReachedHold;
                emit_reached_true = true;
                queue_run_finish(
                  true, ErrorCode::OK, active_place_id_ + " 목표 지점 도달 완료");
              }
              break;
            }
            goal_hold_count_ = 0;
            if (dist_to_goal <= goal_pos_tol_m_) {
              cmd_vel_body_ = {
                0.0f, 0.0f, std::clamp(1.6f * yaw_err, -0.75f, 0.75f)
              };
              break;
            }
          }

          if (planner_.is_human_waiting()) {
            cmd_vel_body_ = {0.0f, 0.0f, 0.0f};
          } else {
            Obs41f obs = build_obs_41d(
              eff_odom, odom_vel_, active_goal, esdf_slice_, box_pts_, puck_pts_,
              map_to_odom_pose_, imu_state_, wp,
              1.0f, calib_.esdf_frame_mode, calib_.esdf_dist_offset_m);
            fuse_lidar_ultrasonic_into_obs(
              obs, lidar_pts_base_, fresh_sonar_m, 1.0f, calib_.ultrasonic_pos_base_xyz);

            Vec3f act = policy_.act_shielded(obs);

            // Fine convergence assist inside final 25cm so robot enters <=8cm (5~10cm) cleanly
            if (dist_to_goal < 0.25f) {
              const float cm = std::cos(myaw);
              const float sm = std::sin(myaw);
              const float dx = active_goal[0] - mx;
              const float dy = active_goal[1] - my;
              const float bx_err = cm * dx + sm * dy;
              const float by_err = -sm * dx + cm * dy;
              const Vec3f fine_act = SafetyShield::apply(
                obs,
                {
                  std::clamp(1.8f * bx_err, -0.30f, 0.55f),
                  std::clamp(1.8f * by_err, -0.55f, 0.55f),
                  std::clamp(1.4f * yaw_err, -0.65f, 0.65f)
                });
              const float w_fine = std::clamp((0.25f - dist_to_goal) / 0.17f, 0.0f, 0.75f);
              for (int k = 0; k < 3; ++k) {
                act[k] = (1.0f - w_fine) * act[k] + w_fine * fine_act[k];
              }
            }

            const Vec3f & lim = policy_.speed_limits();
            cmd_vel_body_ = {act[0] * lim[0], act[1] * lim[1], act[2] * lim[2]};
          }
          break;
        }

        case Stage::SearchingBox: {
          if (estopped_ || !odom_fresh) {
            cmd_vel_body_ = {0.0f, 0.0f, 0.0f};
            break;
          }
          if (box_visible_now) {
            if (dist_to_goal > pre_approach_tol_m_) {
              stage_ = Stage::Navigating;
            } else if (std::abs(yaw_err) > pre_approach_yaw_tol_rad_) {
              cmd_vel_body_ = {0.0f, 0.0f, std::clamp(1.5f * yaw_err, -0.65f, 0.65f)};
            } else {
              cmd_vel_body_ = {0.0f, 0.0f, 0.0f};
              settle_ticks_20hz_ = 5;
              stage_ = Stage::SettlingBeforeApproach;
            }
            break;
          }

          const double search_elapsed = (t_now - search_start_time_).seconds();
          if (search_elapsed > box_search_timeout_s_) {
            cmd_vel_body_ = {0.0f, 0.0f, 0.0f};
            if (require_box_before_approach_) {
              leg_owner_active_ = false;
              stage_ = Stage::Failed;
              queue_run_finish(
                false, ErrorCode::TARGET_NOT_FOUND,
                active_place_id_ + " 상자를 탐색 시간 내에 찾지 못했습니다");
            } else {
              settle_ticks_20hz_ = 5;
              stage_ = Stage::SettlingBeforeApproach;
            }
            break;
          }

          // Gentle sinusoidal yaw scan (±18 deg) around nominal approach yaw
          const float target_scan_yaw = wrap_to_pi(
            nominal_goal_[2] + 0.32f * std::sin(static_cast<float>(1.6 * search_elapsed)));
          const float scan_err = wrap_to_pi(target_scan_yaw - myaw);
          cmd_vel_body_ = {0.0f, 0.0f, std::clamp(1.5f * scan_err, -0.50f, 0.50f)};
          break;
        }

        case Stage::SettlingBeforeApproach: {
          cmd_vel_body_ = {0.0f, 0.0f, 0.0f};
          if (settle_ticks_20hz_ > 0) {
            --settle_ticks_20hz_;
            break;
          }
          // Release leg & arm command ownership BEFORE calling /approach_box
          leg_owner_active_ = false;
          pending_approach_seq = prepare_approach_state_locked(&pending_approach_timeout_s);
          need_send_approach = true;
          stage_ = Stage::CallingApproach;
          break;
        }

        case Stage::CallingApproach: {
          cmd_vel_body_ = {0.0f, 0.0f, 0.0f};
          leg_owner_active_ = false;
          if (approach_finished_) {
            if (approach_ok_) {
              stage_ = Stage::ApproachAligned;
              emit_reached_true = true;
              queue_run_finish(
                true, ErrorCode::OK,
                active_place_id_ + " 주행 및 정밀 접근 완료: " + approach_result_msg_);
            } else if (approach_canceled_) {
              stage_ = Stage::Idle;
              queue_run_finish(
                false, ErrorCode::CANCELED, approach_result_msg_, true);
            } else {
              stage_ = Stage::Failed;
              uint16_t code = ErrorCode::NAV_FAILED;
              if (approach_timed_out_) {
                code = ErrorCode::TIMEOUT;
              } else if (!approach_ever_visible_) {
                code = ErrorCode::TARGET_NOT_FOUND;
              }
              queue_run_finish(
                false, code,
                active_place_id_ + " 정밀 접근 실패: " + approach_result_msg_);
            }
          }
          break;
        }
      }

      // 6. Prepare RunTask feedback & status string (throttled to ~4 Hz for RunTask)
      pub_vel = cmd_vel_body_;
      if (run_goal_ && run_goal_->is_active() && (++fb_div_ % 5 == 0)) {
        fb_gh = run_goal_;
        const float nav_ratio = std::clamp(
          1.0f - (dist_to_goal / std::max(0.20f, initial_goal_dist_)), 0.0f, 1.0f);
        std::ostringstream ph;
        ph << std::fixed << std::setprecision(2);
        if (stage_ == Stage::Navigating) {
          if (planner_.is_human_waiting()) {
            ph << "human_wait(" << (planner_.block_steps() * 0.05f) << "s) d=" << dist_to_goal << "m";
          } else {
            ph << "nav(" << active_place_id_ << ") d=" << dist_to_goal
               << "m box=" << (box_visible_now ? "yes" : "no");
          }
          fb_progress = use_box_approach_ ? (0.65f * nav_ratio) : (0.95f * nav_ratio);
        } else if (stage_ == Stage::SearchingBox) {
          ph << "searching_box(" << active_place_id_ << ")";
          fb_progress = 0.66f;
        } else if (stage_ == Stage::SettlingBeforeApproach) {
          ph << "handoff_to_approach(" << active_place_id_ << ")";
          fb_progress = 0.68f;
        } else if (stage_ == Stage::CallingApproach) {
          ph << std::setprecision(3) << "approach dist=" << approach_fb_dist_m_
             << "m yaw=" << std::setprecision(1) << approach_fb_yaw_deg_
             << "deg hold=" << approach_fb_hold_s_ << "s";
          fb_progress = std::clamp(0.70f + 0.14f * approach_fb_hold_s_, 0.70f, 0.99f);
        }
        fb_phase = ph.str();
      }

      std::ostringstream st;
      st << std::fixed << std::setprecision(3)
         << "stage=" << stage_name(stage_)
         << " place=" << active_place_id_
         << " pose=(" << mx << "," << my << "," << myaw << ")"
         << " dist=" << dist_to_goal
         << " box_vis=" << (box_visible_now ? 1 : 0)
         << " replans=" << planner_.human_replans()
         << " leg_own=" << (leg_owner_active_ ? 1 : 0);
      status_str = st.str();
    }

    // 7. Publish telemetry & complete/cancel/dispatch actions outside mutex
    if (cancel_app_handle) {
      approach_cli_->async_cancel_goal(cancel_app_handle);
    }

    if (need_send_approach) {
      std::string err;
      if (!send_approach_goal_unlocked(pending_approach_seq, pending_approach_timeout_s, &err)) {
        std::shared_ptr<RunGoalHandle> fail_gh;
        RunTask::Result::SharedPtr fail_res;
        {
          std::lock_guard<std::mutex> lk(mtx_);
          if (pending_approach_seq == approach_req_seq_) {
            stage_ = Stage::Failed;
            if (run_goal_ && run_goal_->is_active()) {
              fail_gh = run_goal_;
              run_goal_.reset();
              fail_res = std::make_shared<RunTask::Result>();
              fail_res->success = false;
              fail_res->error_code = ErrorCode::INTERNAL_ERROR;
              fail_res->message = err;
            }
          }
        }
        if (fail_gh && fail_res) {
          fail_gh->abort(fail_res);
        }
      }
    }

    geometry_msgs::msg::TwistStamped vmsg;
    vmsg.header.stamp = t_now;
    vmsg.header.frame_id = base_frame_;
    vmsg.twist.linear.x = pub_vel[0];
    vmsg.twist.linear.y = pub_vel[1];
    vmsg.twist.angular.z = pub_vel[2];
    vel_pub_->publish(vmsg);

    if (has_pub_wp) {
      geometry_msgs::msg::PoseStamped wmsg;
      wmsg.header.stamp = t_now;
      wmsg.header.frame_id = map_frame_;
      wmsg.pose.position.x = pub_wp[0];
      wmsg.pose.position.y = pub_wp[1];
      wmsg.pose.orientation.z = std::sin(pub_wp[2] * 0.5f);
      wmsg.pose.orientation.w = std::cos(pub_wp[2] * 0.5f);
      wp_pub_->publish(wmsg);
    }

    if (emit_reached_true) {
      publish_reached(true);
    }

    if (fb_gh && fb_gh->is_active()) {
      auto fb = std::make_shared<RunTask::Feedback>();
      fb->phase = fb_phase;
      fb->progress = fb_progress;
      fb_gh->publish_feedback(fb);
    }

    if (finish_gh && finish_res) {
      if (finish_as_canceled) {
        finish_gh->canceled(finish_res);
      } else if (finish_as_succeed) {
        finish_gh->succeed(finish_res);
      } else {
        finish_gh->abort(finish_res);
      }
    }

    std_msgs::msg::String smsg;
    smsg.data = status_str;
    status_pub_->publish(smsg);
  }

  // ==================== 100 Hz Leg Gait & Arm Home Loop ====================
  void tick_gait_100hz()
  {
    Leg18f out_legs{};
    Arm6f out_arm{};
    bool do_pub_legs = false;
    bool do_pub_arm = false;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      const bool should_drive = !dry_run_ && !estopped_ &&
        (leg_owner_active_ || (stand_when_idle_ && stage_ != Stage::CallingApproach));
      if (!should_drive) {
        return;
      }

      const Vec3f vel = leg_owner_active_ ? cmd_vel_body_ : Vec3f{0.0f, 0.0f, 0.0f};
      const Leg18f target_hw = gait_.step_hardware(vel, calib_, 0.01f);

      const double since_engage = (now() - owner_engage_time_).seconds();
      const float soft_alpha = (calib_.soft_start_duration_s > 0.05f)
        ? std::clamp(static_cast<float>(since_engage / calib_.soft_start_duration_s), 0.15f, 1.0f)
        : 1.0f;
      const float max_delta = calib_.max_joint_slew_rad_s * 0.01f * soft_alpha;

      for (size_t i = 0; i < 18; ++i) {
        const float diff = std::clamp(target_hw[i] - last_hw_cmd_[i], -max_delta, max_delta);
        last_hw_cmd_[i] += diff;
      }
      out_legs = last_hw_cmd_;
      do_pub_legs = true;

      if (arm_pub_ && hold_arm_walking_ && leg_owner_active_ && (++arm_div_ % 5 == 0)) {
        out_arm = LegGaitController::sim_to_hardware_arm(Mk2Kinematics::arm_home_pose(), calib_);
        do_pub_arm = true;
      }
    }

    if (do_pub_legs) {
      std_msgs::msg::Float64MultiArray lmsg;
      lmsg.data.resize(18);
      for (size_t i = 0; i < 18; ++i) {
        lmsg.data[i] = static_cast<double>(out_legs[i]);
      }
      leg_pub_->publish(lmsg);
    }

    if (do_pub_arm) {
      std_msgs::msg::Float64MultiArray amsg;
      amsg.data.resize(6);
      for (size_t i = 0; i < 6; ++i) {
        amsg.data[i] = static_cast<double>(out_arm[i]);
      }
      arm_pub_->publish(amsg);
    }
  }

  bool has_puck_fresh(const rclcpp::Time & t_now) const
  {
    return !puck_pts_.empty() && (t_now - last_puck_time_).seconds() <= box_vis_timeout_s_;
  }

  static const char * stage_name(Stage s)
  {
    switch (s) {
      case Stage::Idle: return "IDLE";
      case Stage::Navigating: return "NAVIGATING";
      case Stage::SearchingBox: return "SEARCHING_BOX";
      case Stage::SettlingBeforeApproach: return "SETTLING_BEFORE_APPROACH";
      case Stage::CallingApproach: return "CALLING_APPROACH";
      case Stage::ApproachAligned: return "APPROACH_ALIGNED";
      case Stage::ReachedHold: return "REACHED_HOLD";
      case Stage::Failed: return "FAILED";
    }
    return "UNKNOWN";
  }

  // ==================== Member State ====================
  std::mutex mtx_;
  std::string calib_path_;
  CalibrationConfig calib_;
  MlpPolicy policy_;
  DetourPlanner planner_;
  LegGaitController gait_;
  EsdfSlice esdf_slice_;

  Stage stage_ = Stage::Idle;
  std::string active_place_id_ = "NONE";
  Vec3f nominal_goal_{0.734f, 0.950f, static_cast<float>(Mk2Kinematics::kPi)};
  Vec3f smoothed_refined_goal_{0.0f, 0.0f, 0.0f};
  Vec2f nominal_box_xy_{0.0f, 0.0f};
  bool has_nominal_box_ = false;
  bool use_box_approach_ = false;
  bool box_seen_for_goal_ = false;
  bool has_refined_box_ = false;
  float initial_goal_dist_ = 1.0f;

  // Parameters
  float human_wait_timeout_s_ = 10.0f;
  double default_nav_timeout_s_ = 150.0;
  float pre_approach_tol_m_ = 0.08f;
  float pre_approach_yaw_tol_rad_ = 0.20f;
  int pre_approach_hold_steps_ = 5;
  float goal_pos_tol_m_ = 0.08f;
  float goal_yaw_tol_rad_ = 0.18f;
  double box_vis_timeout_s_ = 0.60;
  double box_search_timeout_s_ = 5.0;
  float box_detect_min_score_ = 0.25f;
  float box_assoc_radius_m_ = 0.35f;
  float max_target_refine_m_ = 0.25f;
  bool require_box_before_approach_ = true;
  bool chain_approach_for_box_goals_ = true;
  bool chain_approach_on_custom_goal_ = false;
  double approach_timeout_s_ = 35.0;
  double approach_server_wait_s_ = 2.5;
  bool dry_run_ = false;
  bool stand_when_idle_ = false;
  bool hold_arm_walking_ = true;
  double odom_watchdog_timeout_s_ = 0.60;
  float max_safe_tilt_deg_ = 38.0f;

  std::string pose_source_ = "odom_calibrated";
  std::string map_frame_ = "map";
  std::string arena_frame_ = "arena";
  std::string odom_frame_ = "odom";
  std::string base_frame_ = "base";
  bool publish_arena_tf_ = true;
  std::string publish_map_to_odom_mode_ = "auto";
  bool checked_external_map_tf_ = false;
  bool own_map_to_odom_tf_ = false;
  std::string approach_action_name_ = "/approach_box";

  // 2D LiDAR <-> static arena ESDF localization state
  Vec3f map_to_odom_pose_{0.734f, 0.950f, static_cast<float>(Mk2Kinematics::kPi)};
  bool enable_lidar_localization_ = true;
  bool auto_initial_relocalize_ = true;
  bool initial_relocalized_ = false;
  double loc_update_period_s_ = 0.40;
  float loc_pos_alpha_ = 0.25f;
  float loc_yaw_alpha_ = 0.20f;
  float loc_max_step_pos_m_ = 0.06f;
  float loc_max_step_yaw_rad_ = 0.08f;
  int loc_min_inliers_ = 20;
  float loc_max_rmse_m_ = 0.065f;
  float last_loc_rmse_m_ = 0.0f;
  rclcpp::Time last_loc_time_{0, 0, RCL_ROS_TIME};

  // Live sensor caches
  bool has_odom_ = false;
  bool has_imu_ = false;
  bool has_joints_ = false;
  bool has_native_nvblox_ = false;
  bool has_lidar_ = false;
  bool has_sonar_ = false;
  bool has_det3d_stream_ = false;
  bool has_box_3d_ = false;
  bool has_box_2d_ = false;
  bool has_tf_map_pose_ = false;
  bool estopped_ = false;
  bool leg_owner_active_ = false;

  int pre_approach_hold_count_ = 0;
  int goal_hold_count_ = 0;
  int settle_ticks_20hz_ = 0;
  uint32_t fb_div_ = 0;
  uint32_t arm_div_ = 0;

  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time owner_engage_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tf_map_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_lidar_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_sonar_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_box_3d_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_box_2d_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_puck_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time search_start_time_{0, 0, RCL_ROS_TIME};

  Vec3f odom_pose_{0.0f, 0.0f, 0.0f};
  Vec3f odom_vel_{0.0f, 0.0f, 0.0f};
  Vec3f tf_map_pose_{0.0f, 0.0f, 0.0f};
  Vec6f imu_state_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f};
  Vec2f latest_box_det_base_{0.0f, 0.0f};
  float latest_box_yaw_base_ = 0.0f;
  std::vector<Vec2f> box_pts_;
  std::vector<Vec2f> puck_pts_;
  std::vector<Vec2f> lidar_pts_base_;
  float ultrasonic_range_m_ = -1.0f;
  Vec3f cmd_vel_body_{0.0f, 0.0f, 0.0f};
  Leg18f latest_hw_joints_{};
  Leg18f last_hw_cmd_{};

  // Active /nav/run & /approach_box action handles
  std::shared_ptr<RunGoalHandle> run_goal_;
  rclcpp::Time run_start_time_{0, 0, RCL_ROS_TIME};
  double run_timeout_s_ = 150.0;

  ApproachGoalHandle::SharedPtr approach_goal_handle_;
  uint64_t approach_req_seq_ = 0;
  bool approach_finished_ = false;
  bool approach_ok_ = false;
  bool approach_canceled_ = false;
  bool approach_timed_out_ = false;
  bool approach_ever_visible_ = false;
  float approach_fb_dist_m_ = 0.0f;
  float approach_fb_yaw_deg_ = 0.0f;
  float approach_fb_hold_s_ = 0.0f;
  std::string approach_result_msg_;

  // ROS interfaces
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr wp_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr det3d_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det2d_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr box_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr goal_id_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
#ifdef XSTING_NAV_HAS_NVBLOX_MSGS
  rclcpp::Subscription<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr nvblox_sub_;
  rclcpp::Subscription<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr nvblox_dyn_sub_;
#endif

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_calib_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_approach_srv_;

  rclcpp_action::Client<ApproachBox>::SharedPtr approach_cli_;
  rclcpp_action::Server<RunTask>::SharedPtr run_srv_;

  rclcpp::TimerBase::SharedPtr policy_timer_;
  rclcpp::TimerBase::SharedPtr gait_timer_;
};

}  // namespace xsting_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xsting_nav::NavNode>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
