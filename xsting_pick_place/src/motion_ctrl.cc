#include "xsting_pick_place/motion_ctrl.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "xsting_pick_place/mission_rules.h"

namespace xsting_pick_place {

namespace {

constexpr double kPi = 3.14159265358979323846;

// 박스 형태의 MoveIt 충돌 객체 메시지 생성
moveit_msgs::msg::CollisionObject MakeBoxObj(const std::string& id,
                                             const Vec3& size,
                                             const Vec3& center) {
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = "base";
  obj.id = id;
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive prim;
  prim.type = shape_msgs::msg::SolidPrimitive::BOX;
  prim.dimensions = {size[0], size[1], size[2]};

  geometry_msgs::msg::Pose pose;
  pose.position.x = center[0];
  pose.position.y = center[1];
  pose.position.z = center[2];
  pose.orientation.w = 1.0;

  obj.primitives.push_back(prim);
  obj.primitive_poses.push_back(pose);
  return obj;
}

// 바스켓 슬롯 기적재 퍽 표현을 위한 직육면체 충돌 객체 생성 (cuMotion CYLINDER 미지원 대응)
moveit_msgs::msg::CollisionObject MakeCylObj(const std::string& id,
                                             double radius, double height,
                                             const Vec3& center) {
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = "base";
  obj.id = id;
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive prim;
  prim.type = shape_msgs::msg::SolidPrimitive::BOX;
  prim.dimensions = {2.0 * radius, 2.0 * radius, height};

  geometry_msgs::msg::Pose pose;
  pose.position.x = center[0];
  pose.position.y = center[1];
  pose.position.z = center[2];
  pose.orientation.w = 1.0;

  obj.primitives.push_back(prim);
  obj.primitive_poses.push_back(pose);
  return obj;
}

// 속도 배율(vel_scale)에 연동하되 정밀 접촉용 최소 시간을 보장하는 구간 시간 계산
double ScaleDur(double base_sec, const MotionCfg& cfg, double min_sec) {
  const double s = std::clamp(cfg.vel_scale, 0.20, 1.0);
  return std::max(min_sec, base_sec * (0.45 / s));
}

// 팔 뻗음 거리에 따른 중력 처짐 Z 보정치 산출
double GetDroopZ(const MotionCfg& cfg, double x, double y) {
  const double reach = std::hypot(x - 0.108, y);
  return cfg.droop_k * std::max(0.0, reach - cfg.droop_r0);
}

// 바스켓 슬롯 안착·인출 시 실기 바깥쪽 5.5도 처짐을 상쇄하도록 안쪽 피치(95.5도) 우선 역기구학 풀이
bool SolveSlotDropIk(const Vec3& pos, double grip_yaw, const Joint5& ref,
                     Joint5& out) {
  for (double p = 0.5 * kPi + 0.096; p >= 0.72; p -= 0.015) {
    if (SolvePadIk(pos, p, grip_yaw, ref, out)) return true;
  }
  return SolveHoverIk(pos, grip_yaw, ref, out, false);
}

}  // namespace

MotionCtrl::MotionCtrl(const rclcpp::Node::SharedPtr& node) : node_(node) {}

// MoveGroupInterface 및 팔·그리퍼 FollowJointTrajectory 클라이언트 준비
bool MotionCtrl::Init(const MotionCfg& cfg) {
  js_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
        if (!msg->position.empty()) {
          js_ready_.store(true, std::memory_order_relaxed);
        }
        const size_t n = std::min(msg->name.size(), msg->position.size());
        for (size_t i = 0; i < n; ++i) {
          if (msg->name[i] == "finger_l" && std::isfinite(msg->position[i])) {
            finger_pos_.store(msg->position[i], std::memory_order_relaxed);
            break;
          }
        }
      });

  arm_cli_ = rclcpp_action::create_client<TrajAct>(
      node_, "/arm_trajectory_controller/follow_joint_trajectory");
  grip_cli_ = rclcpp_action::create_client<TrajAct>(
      node_, "/gripper_trajectory_controller/follow_joint_trajectory");

  if (!arm_cli_->wait_for_action_server(std::chrono::seconds(15)) ||
      !grip_cli_->wait_for_action_server(std::chrono::seconds(15))) {
    RCLCPP_ERROR(node_->get_logger(), "궤적 컨트롤러 액션 서버 연결 실패");
    return false;
  }

  arm_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      node_, "arm", std::shared_ptr<tf2_ros::Buffer>(),
      rclcpp::Duration::from_seconds(15.0));
  scene_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>(
      "", true);
  arm_group_->setMaxVelocityScalingFactor(cfg.vel_scale);
  arm_group_->setMaxAccelerationScalingFactor(cfg.acc_scale);
  arm_group_->setPlanningTime(3.0);
  arm_group_->startStateMonitor(2.0);
  arducam_ = std::make_unique<ArducamAlign>(node_);
  return true;
}

// 바스켓 12개 슬롯의 기적재 퍽 점유 배열 일괄 갱신
void MotionCtrl::SetOccupiedSlots(const std::array<bool, 12>& used) {
  std::lock_guard<std::mutex> lock(slot_mtx_);
  slot_used_ = used;
}

// 단일 바스켓 슬롯(1~12)의 퍽 점유 상태 갱신
void MotionCtrl::SetSlotOccupied(int slot, bool used) {
  if (slot < 1 || slot > 12) return;
  std::lock_guard<std::mutex> lock(slot_mtx_);
  slot_used_[static_cast<size_t>(slot - 1)] = used;
}

// 팔 ArduCam 영상 스트림 수신 여부 반환
bool MotionCtrl::HasArducamStream() const {
  return arducam_ && arducam_->HasActiveStream();
}

// 시뮬레이션 Play 클릭 전 요청 시 에러 로그 없이 관절 상태가 들어올 때까지 대기
bool MotionCtrl::WaitReady(const CancelFn& is_canceled) {
  bool logged = false;
  for (int i = 0; i < 120 && rclcpp::ok(); ++i) {
    if (is_canceled()) return false;
    if (js_ready_.load(std::memory_order_relaxed) &&
        arm_group_->getCurrentState(0.5)) {
      return true;
    }
    if (!logged) {
      RCLCPP_INFO(node_->get_logger(), "관절 상태(/joint_states) 수신 대기 중");
      logged = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  }
  RCLCPP_ERROR(node_->get_logger(), "관절 상태 수신 시간 초과");
  return false;
}

// 현재 MoveGroup 상태로부터 5축 팔 관절값 읽기
Joint5 MotionCtrl::GetArmJoints() const {
  const auto vals = arm_group_->getCurrentJointValues();
  Joint5 out = kHomeJoints;
  for (size_t i = 0; i < 5 && i < vals.size(); ++i) out[i] = vals[i];
  return out;
}

// 작업 상자, 상자 위 비대상 퍽, 바스켓 기적재 퍽들을 PlanningScene에 갱신
void MotionCtrl::SyncScene(double box_x, double box_y, double ground_z,
                           const std::vector<Vec3>& obs_pucks,
                           const Vec3& slot_off, int skip_slot) {
  if (!obs_ids_.empty()) {
    scene_->removeCollisionObjects(obs_ids_);
    obs_ids_.clear();
  }

  std::vector<moveit_msgs::msg::CollisionObject> objs;
  const Vec3 box_center = {box_x, box_y, ground_z + 0.5 * kBoxSize[2]};
  objs.push_back(MakeBoxObj("work_box", kBoxSize, box_center));

  for (size_t i = 0; i < obs_pucks.size(); ++i) {
    const std::string id = "obs_puck_" + std::to_string(i);
    objs.push_back(MakeBoxObj(id, kPuckSize, obs_pucks[i]));
    obs_ids_.push_back(id);
  }

  std::array<bool, 12> used_copy = {};
  {
    std::lock_guard<std::mutex> lock(slot_mtx_);
    used_copy = slot_used_;
  }
  for (int s = 1; s <= 12; ++s) {
    if (!used_copy[static_cast<size_t>(s - 1)] || s == skip_slot) continue;
    const SlotPose sp = GetSlotPose(s);
    const Vec3 sc = {sp.pos[0] + slot_off[0], sp.pos[1] + slot_off[1],
                     sp.pos[2] + 0.5 * kPuckHeight + slot_off[2]};
    const std::string id = "slot_puck_" + std::to_string(s);
    objs.push_back(MakeCylObj(id, 0.0145, kPuckHeight, sc));
    obs_ids_.push_back(id);
  }
  scene_->applyCollisionObjects(objs);
}

// 이송 중 충돌 검사를 위해 그리퍼 끝단에 퍽 부착 또는 분리
void MotionCtrl::SetHeldPuck(bool attach, bool is_square) {
  const std::string id = "held_puck";
  if (!attach) {
    if (!has_held_) return;
    arm_group_->detachObject(id);
    scene_->removeCollisionObjects({id});
    has_held_ = false;
    return;
  }

  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = "gripper";
  obj.id = id;
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive prim;
  if (is_square) {
    prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    prim.dimensions = {kPuckHeight, kPuckSize[0], kPuckSize[1]};
  } else {
    prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    prim.dimensions = {kPuckHeight, kPuckRadius};
  }

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.150;
  if (!is_square) {
    pose.orientation.y = 0.70710678;
    pose.orientation.w = 0.70710678;
  } else {
    pose.orientation.w = 1.0;
  }

  obj.primitives.push_back(prim);
  obj.primitive_poses.push_back(pose);
  scene_->applyCollisionObject(obj);
  arm_group_->attachObject(id, "gripper", {"gripper", "finger_l", "finger_r"});
  has_held_ = true;
}

// cuMotion 파이프라인으로 우선 계획하고 실패 시 OMPL로 재시도하여 실행
bool MotionCtrl::PlanMove(const Joint5& goal, const MotionCfg& cfg,
                          const CancelFn& is_canceled) {
  if (is_canceled()) return false;
  arm_group_->setMaxVelocityScalingFactor(cfg.vel_scale);
  arm_group_->setMaxAccelerationScalingFactor(cfg.acc_scale);

  const std::vector<double> q_vec(goal.begin(), goal.end());
  arm_group_->setJointValueTarget(q_vec);

  std::vector<std::string> pipes = {cfg.pipeline};
  if (cfg.pipeline != "ompl") pipes.push_back("ompl");

  for (const auto& pipe : pipes) {
    for (int attempt = 0; attempt < 2; ++attempt) {
      if (is_canceled()) return false;
      arm_group_->setStartStateToCurrentState();
      arm_group_->setPlanningPipelineId(pipe);
      if (arm_group_->move() == moveit::core::MoveItErrorCode::SUCCESS) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(80));
    }
  }
  RCLCPP_WARN(node_->get_logger(), "MoveIt 경로 계획·실행 실패");
  return false;
}

// 직선 접근·인출·기립용 다중 관절 포인트를 컨트롤러로 전송
bool MotionCtrl::RunArmPath(const std::vector<Joint5>& path, double total_sec,
                            const CancelFn& is_canceled) {
  if (path.empty() || is_canceled()) return false;

  TrajAct::Goal goal;
  goal.trajectory.joint_names.assign(kArmJoints.begin(), kArmJoints.end());
  for (size_t i = 0; i < path.size(); ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(path[i].begin(), path[i].end());
    if (i + 1 == path.size()) pt.velocities.assign(5, 0.0);
    const double t =
        total_sec * static_cast<double>(i + 1) / static_cast<double>(path.size());
    pt.time_from_start = rclcpp::Duration::from_seconds(t);
    goal.trajectory.points.push_back(pt);
  }

  auto gh_fut = arm_cli_->async_send_goal(goal);
  if (gh_fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    return false;
  }
  auto gh = gh_fut.get();
  if (!gh) return false;

  auto res_fut = arm_cli_->async_get_result(gh);
  while (rclcpp::ok()) {
    if (is_canceled()) {
      arm_cli_->async_cancel_goal(gh);
      return false;
    }
    if (res_fut.wait_for(std::chrono::milliseconds(40)) ==
        std::future_status::ready) {
      const bool ok =
          res_fut.get().code == rclcpp_action::ResultCode::SUCCEEDED;
      std::this_thread::sleep_for(std::chrono::milliseconds(80));
      return ok;
    }
  }
  return false;
}

// 그리퍼 손가락(finger_l) 목표 위치 명령 실행
bool MotionCtrl::SetGrip(double width, double sec) {
  TrajAct::Goal goal;
  goal.trajectory.joint_names = {"finger_l"};

  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions = {width};
  pt.time_from_start = rclcpp::Duration::from_seconds(sec);
  goal.trajectory.points.push_back(pt);

  auto gh_fut = grip_cli_->async_send_goal(goal);
  if (gh_fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    return false;
  }
  auto gh = gh_fut.get();
  if (!gh) return false;

  auto res_fut = grip_cli_->async_get_result(gh);
  if (res_fut.wait_for(std::chrono::seconds(6)) != std::future_status::ready) {
    return false;
  }
  return res_fut.get().code == rclcpp_action::ResultCode::SUCCEEDED;
}

// 그리퍼 손가락 위치(finger_l)가 빈손 임계치 이상인지로 30mm 퍽 파지 여부 판정
bool MotionCtrl::IsPuckHeld(const MotionCfg& cfg) const {
  if (!cfg.check_grasp || cfg.grip_close >= cfg.grip_empty_th) return true;
  return finger_pos_.load(std::memory_order_relaxed) >= cfg.grip_empty_th;
}

// 먼 끝 도달 반경 초과(>0.252m) 또는 IK 실패 시 로봇 쪽 끝을 잡아 상자 안쪽으로 견인
bool MotionCtrl::PullFarLyingPuck(double box_x, double box_y, Puck& target,
                                  const Puck* other, double lyaw,
                                  const MotionCfg& cfg,
                                  const PhaseFn& on_phase,
                                  const CancelFn& is_canceled) {
  on_phase("pull_closer");
  if (!SetGrip(cfg.grip_open_box, 0.45)) return false;

  const double box_top = cfg.ground_z + kBoxSize[2];
  const double near_r = std::max(0.010, cfg.right_tip_r - 0.001);
  const double nx =
      target.pos[0] - near_r * std::cos(lyaw) + cfg.pick_off[0];
  const double ny =
      target.pos[1] - near_r * std::sin(lyaw) + cfg.pick_off[1];
  const double nz =
      box_top + cfg.right_grasp_dz + GetDroopZ(cfg, nx, ny);

  double cx_dst = 0.0;
  double cy_dst = 0.0;
  SelectRightSpot(box_x, box_y, target, other, cx_dst, cy_dst);
  const double dst_x = cx_dst - near_r * std::cos(lyaw) + cfg.pick_off[0];
  const double dst_y = cy_dst - near_r * std::sin(lyaw) + cfg.pick_off[1];
  const double dst_z =
      box_top + cfg.right_grasp_dz + GetDroopZ(cfg, dst_x, dst_y);

  const Vec3 hover_near = {nx, ny, cfg.box_hover_z};
  const Vec3 grasp_near = {nx, ny, nz};
  const Vec3 lift_near = {nx, ny, nz + 0.009};
  const Vec3 lift_dst = {dst_x, dst_y, dst_z + 0.009};
  const Vec3 drop_dst = {dst_x, dst_y, dst_z};
  const Vec3 hover_dst = {dst_x, dst_y, cfg.box_hover_z};

  const Joint5 cur_q = GetArmJoints();
  Joint5 q_hover = {};
  if (!SolveHoverIk(hover_near, lyaw, cur_q, q_hover)) return false;
  q_hover[4] = WrapPi(std::atan2(ny, nx - 0.108) - lyaw);

  const Vec3 cur_pad = ForwardPad(cur_q);
  const bool short_hop =
      std::hypot(hover_near[0] - cur_pad[0], hover_near[1] - cur_pad[1]) <
          0.085 &&
      std::abs(hover_near[2] - cur_pad[2]) < 0.055;
  if (!(short_hop ? RunArmPath({q_hover}, 0.55, is_canceled)
                  : PlanMove(q_hover, cfg, is_canceled))) {
    return false;
  }

  // 비대칭 편심 파지 상태에서 손목 롤 180도 반전 방지를 위해 경로 각도 연속 고정
  auto lock_roll = [&](std::vector<Joint5>& path) {
    for (auto& q : path) {
      const Vec3 pt = ForwardPad(q);
      q[4] = WrapPi(std::atan2(pt[1], pt[0] - 0.108) - lyaw);
    }
  };

  const double h_pitch = q_hover[1] + q_hover[2] + q_hover[3];
  std::vector<Joint5> down_path;
  if (!MakeCartLine(hover_near, h_pitch, grasp_near, 0.5 * kPi, lyaw, q_hover,
                    8, down_path)) {
    return false;
  }
  lock_roll(down_path);
  if (!RunArmPath(down_path, 0.85, is_canceled) ||
      !SetGrip(cfg.grip_close, 0.55)) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(180));

  if (!IsPuckHeld(cfg)) {
    RCLCPP_WARN(node_->get_logger(),
                "먼 퍽 견인 파지 실패(빈손, finger_l=%.4fm)",
                finger_pos_.load(std::memory_order_relaxed));
    SetGrip(cfg.grip_open_box, 0.45);
    std::vector<Joint5> abort_up;
    if (MakeCartLine(grasp_near, 0.5 * kPi, hover_near, h_pitch, lyaw,
                     down_path.back(), 6, abort_up)) {
      RunArmPath(abort_up, 0.70, is_canceled);
    }
    return false;
  }

  // 상판 마찰 회피 높이(+9mm)로 살짝 들어 저속(~0.2배속)으로 상자 중앙까지 이동
  std::vector<Joint5> lift_path;
  std::vector<Joint5> pull_path;
  std::vector<Joint5> drop_path;
  if (!MakeCartLine(grasp_near, 0.5 * kPi, lift_near, 0.5 * kPi, lyaw,
                    down_path.back(), 4, lift_path)) {
    SetGrip(cfg.grip_open_box, 0.45);
    return false;
  }
  lock_roll(lift_path);
  if (!MakeCartLine(lift_near, 0.5 * kPi, lift_dst, 0.5 * kPi, lyaw,
                    lift_path.back(), 10, pull_path)) {
    SetGrip(cfg.grip_open_box, 0.45);
    return false;
  }
  lock_roll(pull_path);
  if (!MakeCartLine(lift_dst, 0.5 * kPi, drop_dst, 0.5 * kPi, lyaw,
                    pull_path.back(), 4, drop_path)) {
    SetGrip(cfg.grip_open_box, 0.45);
    return false;
  }
  lock_roll(drop_path);

  if (!RunArmPath(lift_path, 0.35, is_canceled) ||
      !RunArmPath(pull_path, 1.35, is_canceled) ||
      !RunArmPath(drop_path, 0.40, is_canceled)) {
    SetGrip(cfg.grip_open_box, 0.45);
    return false;
  }
  if (!SetGrip(cfg.grip_open_box, 0.45)) return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  Joint5 q_dhover = {};
  if (!SolveHoverIk(hover_dst, lyaw, drop_path.back(), q_dhover)) {
    return false;
  }
  const double d_pitch = q_dhover[1] + q_dhover[2] + q_dhover[3];
  std::vector<Joint5> up_path;
  if (!MakeCartLine(drop_dst, 0.5 * kPi, hover_dst, d_pitch, lyaw,
                    drop_path.back(), 8, up_path) ||
      !RunArmPath(up_path, 0.75, is_canceled)) {
    return false;
  }

  target.pos = {cx_dst, cy_dst, box_top + 0.015};
  target.yaw = lyaw;
  target.is_lying = true;
  return true;
}

// 상자 전체 조망 자세(+300mm)로 이동해 YOLOv8로 상자와 퍽들을 검출
bool MotionCtrl::ScanBoxWithArducam(const std::string& filter_cls,
                                    MotionCfg& cfg, const PhaseFn& on_phase,
                                    const CancelFn& is_canceled,
                                    std::vector<Puck>& out_pucks,
                                    bool allow_empty) {
  if (!cfg.use_arducam || !HasArducamStream()) return false;
  if (!WaitReady(is_canceled)) return false;

  on_phase("arducam_scan");
  SetHeldPuck(false, true);
  if (!SetGrip(cfg.grip_open_box, 0.35)) return false;
  SyncScene(cfg.box_x, cfg.box_y, cfg.ground_z, {}, cfg.slot_off, 0);

  const double box_top = cfg.ground_z + kBoxSize[2];
  const Vec3 scan_pt = {cfg.box_x, cfg.box_y, box_top + 0.300};
  const Joint5 cur_q = GetArmJoints();
  Joint5 q_scan = {};
  if (!SolveArducamIk(scan_pt, cur_q, q_scan)) return false;

  const Vec3 cur_pad = ForwardPad(cur_q);
  const Vec3 scan_pad = ForwardPad(q_scan);
  const bool near_scan =
      std::hypot(scan_pad[0] - cur_pad[0], scan_pad[1] - cur_pad[1]) < 0.095 &&
      std::abs(scan_pad[2] - cur_pad[2]) < 0.075;
  const bool moved =
      near_scan ? RunArmPath({q_scan}, ScaleDur(0.50, cfg, 0.32), is_canceled)
                : PlanMove(q_scan, cfg, is_canceled);
  if (!moved) return false;

  std::this_thread::sleep_for(std::chrono::milliseconds(110));
  return arducam_->ScanBox(GetArmJoints(), cfg.ground_z, filter_cls,
                           cfg.auto_box_det, cfg.box_x, cfg.box_y, out_pucks,
                           allow_empty);
}

// 팔 ArduCam 직상방 2-Pass 관측으로 60mm 측면 시차를 완전 제거하고 XY·Yaw 정밀 보정
bool MotionCtrl::AlignWithArducam(double& box_x, double& box_y, Puck& target,
                                  const Puck* other, const MotionCfg& cfg,
                                  const PhaseFn& on_phase,
                                  const CancelFn& is_canceled) {
  if (!cfg.use_arducam || !HasArducamStream()) return true;

  on_phase("arducam_align");
  if (!SetGrip(cfg.grip_open_box, 0.35)) return false;

  const double box_top = cfg.ground_z + kBoxSize[2];
  const double prev_x = target.pos[0];
  const double prev_y = target.pos[1];
  const Vec3 cam_pt = {prev_x, prev_y, box_top + cfg.arducam_look_dz};
  const Joint5 cur_q = GetArmJoints();
  Joint5 q_look = {};
  if (!SolveArducamIk(cam_pt, cur_q, q_look)) return true;

  const Vec3 cur_pad = ForwardPad(cur_q);
  const Vec3 look_pad = ForwardPad(q_look);
  const bool near_box =
      std::hypot(look_pad[0] - cur_pad[0], look_pad[1] - cur_pad[1]) < 0.095 &&
      std::abs(look_pad[2] - cur_pad[2]) < 0.065;
  const bool moved =
      near_box ? RunArmPath({q_look}, ScaleDur(0.50, cfg, 0.32), is_canceled)
               : PlanMove(q_look, cfg, is_canceled);
  if (!moved) return false;

  std::this_thread::sleep_for(std::chrono::milliseconds(110));
  const bool ok1 = arducam_->RefinePuck(
      box_x, box_y, box_top, cfg.auto_box_det, GetArmJoints(), other, target);

  // 1차 보정 이동량이 4.5mm를 넘으면 퍽 직상방 무(無)시차 원뿔 내부로 재정렬해 2차 측정
  if (ok1 && std::hypot(target.pos[0] - prev_x, target.pos[1] - prev_y) >
                 0.0045) {
    const Vec3 cam_pt2 = {target.pos[0], target.pos[1],
                          box_top + cfg.arducam_look_dz};
    Joint5 q_look2 = {};
    if (SolveArducamIk(cam_pt2, GetArmJoints(), q_look2) &&
        RunArmPath({q_look2}, ScaleDur(0.42, cfg, 0.28), is_canceled)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(90));
      arducam_->RefinePuck(box_x, box_y, box_top, cfg.auto_box_det,
                           GetArmJoints(), other, target);
    }
  }
  return true;
}

// 파지 객체를 분리하고 기적재 퍽을 회피하며 하드웨어 접힌 홈 자세로 복귀
bool MotionCtrl::MoveHome(const MotionCfg& cfg, const CancelFn& is_canceled) {
  if (!WaitReady(is_canceled)) return false;
  SetHeldPuck(false, true);
  SetGrip(cfg.grip_open_slot, 0.38);
  const Joint5 cur_q = GetArmJoints();
  const Vec3 cur_pt = ForwardPad(cur_q);
  if (cur_pt[0] < 0.115 && cur_pt[2] < cfg.slot_hover_z - 0.015) {
    const Vec3 lift_pt = {cur_pt[0], cur_pt[1], cfg.slot_hover_z};
    Joint5 q_lift = {};
    std::vector<Joint5> lift_path;
    if (SolveHoverIk(lift_pt, cur_q[0], cur_q, q_lift, true) &&
        MakeCartLine(cur_pt, cur_q[1] + cur_q[2] + cur_q[3], lift_pt,
                     q_lift[1] + q_lift[2] + q_lift[3], cur_q[0], cur_q, 8,
                     lift_path)) {
      RunArmPath(lift_path, 0.68, is_canceled);
    }
  }
  SyncScene(cfg.box_x, cfg.box_y, cfg.ground_z, {}, cfg.slot_off, 0);
  return PlanMove(kHomeJoints, cfg, is_canceled);
}

// 카메라 시야 확보를 위한 관측 자세로 이동
bool MotionCtrl::MoveObserve(const MotionCfg& cfg,
                             const CancelFn& is_canceled) {
  if (!WaitReady(is_canceled)) return false;
  SetHeldPuck(false, true);
  SetGrip(cfg.grip_open_box, 0.38);
  const Joint5 cur_q = GetArmJoints();
  const Vec3 cur_pt = ForwardPad(cur_q);
  if (cur_pt[0] < 0.115 && cur_pt[2] < cfg.slot_hover_z - 0.015) {
    const Vec3 lift_pt = {cur_pt[0], cur_pt[1], cfg.slot_hover_z};
    Joint5 q_lift = {};
    std::vector<Joint5> lift_path;
    if (SolveHoverIk(lift_pt, cur_q[0], cur_q, q_lift, true) &&
        MakeCartLine(cur_pt, cur_q[1] + cur_q[2] + cur_q[3], lift_pt,
                     q_lift[1] + q_lift[2] + q_lift[3], cur_q[0], cur_q, 8,
                     lift_path)) {
      RunArmPath(lift_path, 0.68, is_canceled);
    }
  }
  SyncScene(cfg.box_x, cfg.box_y, cfg.ground_z, {}, cfg.slot_off, 0);
  return PlanMove(kObsJoints, cfg, is_canceled);
}

// 실측 보정 목표 위치로 호버 경유 또는 직선 이동 후 현재 정기구학 좌표 반환
bool MotionCtrl::ProbePose(const Vec3& target, double yaw, const MotionCfg& cfg,
                           const CancelFn& is_canceled, Vec3& meas_out) {
  if (!WaitReady(is_canceled)) return false;
  SetHeldPuck(false, true);
  const Joint5 cur_q = GetArmJoints();
  const Vec3 cur_pt = ForwardPad(cur_q);
  const bool in_slot = (target[0] < 0.12);

  Joint5 q_tgt = {};
  const bool ok_tgt = in_slot
                          ? SolveSlotDropIk(target, yaw, cur_q, q_tgt)
                          : SolveHoverIk(target, yaw, cur_q, q_tgt, false);
  const double t_pitch =
      ok_tgt ? (q_tgt[1] + q_tgt[2] + q_tgt[3]) : (0.5 * kPi);

  if (std::hypot(target[0] - cur_pt[0], target[1] - cur_pt[1]) < 0.045 &&
      std::abs(target[2] - cur_pt[2]) < 0.060) {
    const double cur_p = cur_q[1] + cur_q[2] + cur_q[3];
    std::vector<Joint5> jog_path;
    if (!MakeCartLine(cur_pt, cur_p, target, t_pitch, yaw, cur_q, 6,
                      jog_path) ||
        !RunArmPath(jog_path, 0.65, is_canceled)) {
      return false;
    }
    meas_out = ForwardPad(GetArmJoints());
    return true;
  }

  int skip_slot = 0;
  if (in_slot) {
    double best_d = 0.030;
    for (int s = 1; s <= 12; ++s) {
      const SlotPose sp = GetSlotPose(s);
      const double d = std::hypot(target[0] - (sp.pos[0] + cfg.slot_off[0]),
                                  target[1] - (sp.pos[1] + cfg.slot_off[1]));
      if (d < best_d) {
        best_d = d;
        skip_slot = s;
      }
    }
  }
  SyncScene(cfg.box_x, cfg.box_y, cfg.ground_z, {}, cfg.slot_off, skip_slot);

  const double hz =
      std::max(target[2], in_slot ? cfg.slot_hover_z : cfg.box_hover_z);
  const Vec3 hover = {target[0], target[1], hz};
  Joint5 q_hover = {};
  if (!SolveHoverIk(hover, yaw, cur_q, q_hover, in_slot)) return false;
  const double arm_yaw = std::atan2(target[1], target[0] - 0.108);
  if (!in_slot && std::abs(WrapPi(arm_yaw - yaw)) <= 0.5 * kPi) {
    q_hover[4] = WrapPi(arm_yaw - yaw);
  }
  if (!PlanMove(q_hover, cfg, is_canceled)) return false;

  if (hz - target[2] > 0.002) {
    const double h_pitch = q_hover[1] + q_hover[2] + q_hover[3];
    std::vector<Joint5> down_path;
    if (!MakeCartLine(hover, h_pitch, target, t_pitch, yaw, q_hover, 8,
                      down_path) ||
        !RunArmPath(down_path, ScaleDur(0.75, cfg, 0.52), is_canceled)) {
      return false;
    }
  }
  meas_out = ForwardPad(GetArmJoints());
  return true;
}

// 검출된 직립·누운 퍽을 ArduCam으로 정밀 정렬한 뒤 실제 파지 보정 위치로 프로브 이동
bool MotionCtrl::ProbeVisionPuck(double& box_x, double& box_y, Puck& target,
                                 const Puck* other, double hover,
                                 const MotionCfg& cfg, const PhaseFn& on_phase,
                                 const CancelFn& is_canceled, Vec3& cmd_out,
                                 Vec3& meas_out, double& yaw_out) {
  if (!WaitReady(is_canceled)) return false;
  if (!AlignWithArducam(box_x, box_y, target, other, cfg, on_phase,
                        is_canceled)) {
    return false;
  }

  const double box_top = cfg.ground_z + kBoxSize[2];
  if (target.is_lying) {
    const double base_ayaw = std::atan2(target.pos[1], target.pos[0] - 0.108);
    double lyaw = target.yaw;
    if (std::abs(WrapPi(base_ayaw - lyaw)) > 0.5 * kPi) {
      lyaw = WrapPi(lyaw + kPi);
    }
    const double gx =
        target.pos[0] + cfg.right_tip_r * std::cos(lyaw) + cfg.pick_off[0];
    const double gy =
        target.pos[1] + cfg.right_tip_r * std::sin(lyaw) + cfg.pick_off[1];
    const double gz =
        box_top + cfg.right_grasp_dz + GetDroopZ(cfg, gx, gy) + hover;
    cmd_out = {gx, gy, gz};
    yaw_out = lyaw;
  } else {
    const double gx = target.pos[0] + cfg.pick_off[0];
    const double gy = target.pos[1] + cfg.pick_off[1];
    const double gz = box_top + 0.053 + cfg.pick_off[2] +
                      GetDroopZ(cfg, gx, gy) + hover;
    cmd_out = {gx, gy, gz};
    yaw_out = SelectGraspYaw(target, other);
  }
  return ProbePose(cmd_out, yaw_out, cfg, is_canceled, meas_out);
}

// 누운 사각 퍽 끝단을 집어 들어 올린 뒤 그리퍼를 살짝 풀어 중력으로 회전 기립
bool MotionCtrl::RightPuck(double& box_x, double& box_y, Puck& target,
                           const Puck* other, const MotionCfg& cfg,
                           const PhaseFn& on_phase,
                           const CancelFn& is_canceled) {
  if (!WaitReady(is_canceled)) return false;
  const double box_top = cfg.ground_z + kBoxSize[2];
  std::vector<Vec3> obs;
  if (other) obs.push_back(other->pos);
  SyncScene(box_x, box_y, cfg.ground_z, obs, cfg.slot_off, 0);

  // ArduCam+YOLO 수직 관측으로 누운 퍽 중심·장축 각도 정밀 보정 후 편심 파지 준비
  double lyaw = 0.0;
  Vec3 grasp_pt = {};
  std::vector<Joint5> down_path;
  bool grasped = false;

  for (int att = 0; att < 2; ++att) {
    if (!AlignWithArducam(box_x, box_y, target, other, cfg, on_phase,
                          is_canceled)) {
      return false;
    }
    if (!target.is_lying) return true;
    SyncScene(box_x, box_y, cfg.ground_z, obs, cfg.slot_off, 0);

    const double base_ayaw =
        std::atan2(target.pos[1], target.pos[0] - 0.108);
    lyaw = target.yaw;
    if (std::abs(WrapPi(base_ayaw - lyaw)) > 0.5 * kPi) {
      lyaw = WrapPi(lyaw + kPi);
    }

    double gx =
        target.pos[0] + cfg.right_tip_r * std::cos(lyaw) + cfg.pick_off[0];
    double gy =
        target.pos[1] + cfg.right_tip_r * std::sin(lyaw) + cfg.pick_off[1];
    double gz = box_top + cfg.right_grasp_dz + GetDroopZ(cfg, gx, gy);
    Vec3 hover_pick = {gx, gy, cfg.box_hover_z};
    grasp_pt = {gx, gy, gz};

    Joint5 cur_rq = GetArmJoints();
    Joint5 q_hover = {};
    Joint5 q_grasp_chk = {};
    const bool too_far = std::hypot(gx - 0.108, gy) > 0.252;
    const bool ik_ok =
        !too_far && SolveHoverIk(hover_pick, lyaw, cur_rq, q_hover) &&
        SolvePadIk(grasp_pt, 0.5 * kPi, lyaw, q_hover, q_grasp_chk);

    // 먼 끝이 도달 반경(>0.252m) 밖이거나 IK 실패 시 로봇 쪽 끝을 잡아 끌어온 뒤 재정렬
    if (!ik_ok) {
      RCLCPP_INFO(node_->get_logger(),
                  "먼 위치 누운 퍽 감지(r=%.3fm) -> 상자 안쪽 견인 수행",
                  std::hypot(gx - 0.108, gy));
      if (!PullFarLyingPuck(box_x, box_y, target, other, lyaw, cfg, on_phase,
                            is_canceled) ||
          !AlignWithArducam(box_x, box_y, target, other, cfg, on_phase,
                            is_canceled)) {
        return false;
      }
      if (!target.is_lying) return true;
      SyncScene(box_x, box_y, cfg.ground_z, obs, cfg.slot_off, 0);

      const double post_ayaw =
          std::atan2(target.pos[1], target.pos[0] - 0.108);
      lyaw = target.yaw;
      if (std::abs(WrapPi(post_ayaw - lyaw)) > 0.5 * kPi) {
        lyaw = WrapPi(lyaw + kPi);
      }
      gx = target.pos[0] + cfg.right_tip_r * std::cos(lyaw) + cfg.pick_off[0];
      gy = target.pos[1] + cfg.right_tip_r * std::sin(lyaw) + cfg.pick_off[1];
      gz = box_top + cfg.right_grasp_dz + GetDroopZ(cfg, gx, gy);
      hover_pick = {gx, gy, cfg.box_hover_z};
      grasp_pt = {gx, gy, gz};
      cur_rq = GetArmJoints();
      if (!SolveHoverIk(hover_pick, lyaw, cur_rq, q_hover)) return false;
    }

    on_phase("right_approach");
    if (!SetGrip(cfg.grip_open_box, 0.35)) return false;
    q_hover[4] = WrapPi(std::atan2(gy, gx - 0.108) - lyaw);
    const Vec3 r_pad = ForwardPad(cur_rq);
    const bool short_hop =
        std::hypot(hover_pick[0] - r_pad[0], hover_pick[1] - r_pad[1]) <
            0.085 &&
        std::abs(hover_pick[2] - r_pad[2]) < 0.055;
    if (!(short_hop
              ? RunArmPath({q_hover}, ScaleDur(0.48, cfg, 0.32), is_canceled)
              : PlanMove(q_hover, cfg, is_canceled))) {
      return false;
    }

    // 수직 하강해 먼 끝단을 단단히 집고 빈손 여부를 1차 검증
    on_phase("right_grasp");
    const double pick_pitch = q_hover[1] + q_hover[2] + q_hover[3];
    down_path.clear();
    if (!MakeCartLine(hover_pick, pick_pitch, grasp_pt, 0.5 * kPi, lyaw,
                      q_hover, 8, down_path) ||
        !RunArmPath(down_path, ScaleDur(0.75, cfg, 0.54), is_canceled) ||
        !SetGrip(cfg.grip_close, 0.45)) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(140));

    if (IsPuckHeld(cfg)) {
      grasped = true;
      break;
    }

    RCLCPP_WARN(node_->get_logger(),
                "기립 파지 빈손 감지(시도 %d/2, finger_l=%.4fm)", att + 1,
                finger_pos_.load(std::memory_order_relaxed));
    SetGrip(cfg.grip_open_box, 0.38);
    std::vector<Joint5> retry_up;
    if (MakeCartLine(grasp_pt, 0.5 * kPi, hover_pick, pick_pitch, lyaw,
                     down_path.back(), 6, retry_up)) {
      RunArmPath(retry_up, ScaleDur(0.65, cfg, 0.45), is_canceled);
    }
  }
  if (!grasped) return false;

  double rx = 0.0;
  double ry = 0.0;
  SelectRightSpot(box_x, box_y, target, other, rx, ry);
  const double ryaw = std::atan2(ry, rx - 0.108);
  const double rz_off = cfg.right_off_z + GetDroopZ(cfg, rx, ry);
  const Vec3 pivot_air = {rx, ry, box_top + cfg.right_lift_dz + rz_off};
  const Vec3 seat_pt = {rx, ry, box_top + 0.053 + rz_off};
  const Vec3 right_hover = {rx, ry, cfg.box_hover_z};

  std::vector<Joint5> lift_path;
  if (!MakeCartLine(grasp_pt, 0.5 * kPi, pivot_air, 1.52, ryaw,
                    down_path.back(), 10, lift_path) ||
      !RunArmPath(lift_path, ScaleDur(0.95, cfg, 0.72), is_canceled)) {
    SetGrip(cfg.grip_open_box, 0.40);
    return false;
  }

  // 그리퍼를 살짝 풀어(grip_loosen) 퍽이 손가락 사이에서 중력으로 ~90도 회전하게 유도
  on_phase("right_loosen");
  Joint5 q_pivot = {};
  if (SolvePadIk(pivot_air, 0.5 * kPi, ryaw, lift_path.back(), q_pivot)) {
    RunArmPath({q_pivot}, 0.28, is_canceled);
  } else {
    q_pivot = lift_path.back();
  }
  if (!SetGrip(cfg.grip_loosen, 0.32)) {
    SetGrip(cfg.grip_open_box, 0.40);
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(650));

  // 회전해 수직이 된 퍽 바닥을 상자 상판에 사뿐히 안착시킨 뒤 개방·상승
  on_phase("right_seat");
  std::vector<Joint5> seat_path;
  std::vector<Joint5> up_path;
  if (!MakeCartLine(pivot_air, 0.5 * kPi, seat_pt, 0.5 * kPi, ryaw, q_pivot, 8,
                    seat_path) ||
      !RunArmPath(seat_path, ScaleDur(0.75, cfg, 0.56), is_canceled)) {
    SetGrip(cfg.grip_open_box, 0.40);
    return false;
  }
  if (!SetGrip(cfg.grip_open_box, 0.38)) return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(220));

  Joint5 q_rhover = {};
  if (!SolveHoverIk(right_hover, ryaw, seat_path.back(), q_rhover)) {
    return false;
  }
  const double rhover_pitch = q_rhover[1] + q_rhover[2] + q_rhover[3];
  if (!MakeCartLine(seat_pt, 0.5 * kPi, right_hover, rhover_pitch, ryaw,
                    seat_path.back(), 8, up_path) ||
      !RunArmPath(up_path, ScaleDur(0.68, cfg, 0.45), is_canceled)) {
    return false;
  }

  // 세워진 사각 퍽의 안착 중심 좌표와 직립 상태로 타겟 정보 갱신
  target.pos = {rx + 0.0053 * std::cos(ryaw), ry + 0.0053 * std::sin(ryaw),
                box_top + 0.5 * kPuckHeight};
  target.yaw = ryaw;
  target.is_lying = false;
  return true;
}

// 세워진 퍽 상단(+53mm)을 파지해 바스켓 슬롯 바닥 1.2mm 위에 수직으로 직립 적재
bool MotionCtrl::PickToSlot(double& box_x, double& box_y, const Puck& puck,
                            const Puck* other, int slot, const MotionCfg& cfg,
                            const PhaseFn& on_phase,
                            const CancelFn& is_canceled) {
  if (!WaitReady(is_canceled)) return false;
  const double box_top = cfg.ground_z + kBoxSize[2];
  std::vector<Vec3> obs;
  if (other) obs.push_back(other->pos);
  SyncScene(box_x, box_y, cfg.ground_z, obs, cfg.slot_off, slot);

  // ArduCam+YOLO 정밀 정렬 및 2회 파지 판정(닫은 직후·인양 직후) 기반 자동 재시도
  Puck target = puck;
  double py = 0.0;
  std::vector<Joint5> up_path;
  bool picked = false;

  for (int att = 0; att < 2; ++att) {
    if (!AlignWithArducam(box_x, box_y, target, other, cfg, on_phase,
                          is_canceled)) {
      return false;
    }
    if (target.is_lying) {
      if (!RightPuck(box_x, box_y, target, other, cfg, on_phase, is_canceled) ||
          !AlignWithArducam(box_x, box_y, target, other, cfg, on_phase,
                            is_canceled) ||
          target.is_lying) {
        return false;
      }
    }
    SyncScene(box_x, box_y, cfg.ground_z, obs, cfg.slot_off, slot);

    on_phase("pick_hover");
    if (!SetGrip(cfg.grip_open_box, 0.35)) return false;

    const double gyaw = SelectGraspYaw(target, other);
    const double px = target.pos[0] + cfg.pick_off[0];
    py = target.pos[1] + cfg.pick_off[1];
    const Vec3 hover_pick = {px, py, cfg.box_hover_z};
    const Vec3 grasp_pt = {
        px, py, box_top + 0.053 + cfg.pick_off[2] + GetDroopZ(cfg, px, py)};

    const Joint5 cur_pq = GetArmJoints();
    Joint5 q_hover = {};
    if (!SolveHoverIk(hover_pick, gyaw, cur_pq, q_hover)) {
      return false;
    }
    const Vec3 p_pad = ForwardPad(cur_pq);
    const bool short_hop =
        std::hypot(hover_pick[0] - p_pad[0], hover_pick[1] - p_pad[1]) <
            0.085 &&
        std::abs(hover_pick[2] - p_pad[2]) < 0.055;
    if (!(short_hop
              ? RunArmPath({q_hover}, ScaleDur(0.48, cfg, 0.32), is_canceled)
              : PlanMove(q_hover, cfg, is_canceled))) {
      return false;
    }

    // 수직 하강·밀착 파지 후 1차(닫은 뒤)·2차(들어 올린 뒤) 파지 판정 수행
    on_phase("pick_grasp");
    const double h_pitch = q_hover[1] + q_hover[2] + q_hover[3];
    std::vector<Joint5> down_path;
    up_path.clear();
    if (!MakeCartLine(hover_pick, h_pitch, grasp_pt, 0.5 * kPi, gyaw, q_hover,
                      8, down_path) ||
        !RunArmPath(down_path, ScaleDur(0.75, cfg, 0.54), is_canceled)) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(90));
    if (!SetGrip(cfg.grip_close, 0.45)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(140));

    if (!IsPuckHeld(cfg)) {
      RCLCPP_WARN(node_->get_logger(),
                  "상자 파지 빈손 감지(1차, 시도 %d/2, finger_l=%.4fm)",
                  att + 1, finger_pos_.load(std::memory_order_relaxed));
      SetGrip(cfg.grip_open_box, 0.38);
      if (MakeCartLine(grasp_pt, 0.5 * kPi, hover_pick, h_pitch, gyaw,
                       down_path.back(), 8, up_path)) {
        RunArmPath(up_path, ScaleDur(0.65, cfg, 0.45), is_canceled);
      }
      continue;
    }

    if (!MakeCartLine(grasp_pt, 0.5 * kPi, hover_pick, h_pitch, gyaw,
                      down_path.back(), 8, up_path) ||
        !RunArmPath(up_path, ScaleDur(0.72, cfg, 0.50), is_canceled)) {
      SetGrip(cfg.grip_open_box, 0.40);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(90));

    if (!IsPuckHeld(cfg)) {
      RCLCPP_WARN(node_->get_logger(),
                  "상자 인양 후 퍽 이탈 감지(2차, 시도 %d/2, finger_l=%.4fm)",
                  att + 1, finger_pos_.load(std::memory_order_relaxed));
      SetGrip(cfg.grip_open_box, 0.38);
      continue;
    }

    picked = true;
    break;
  }
  if (!picked) return false;

  const Vec3 carry_pt = {0.220, std::clamp(py, -0.050, 0.050),
                         cfg.slot_hover_z};
  const double cyaw = std::atan2(carry_pt[1], carry_pt[0] - 0.108);
  Joint5 q_carry = {};
  if (SolveHoverIk(carry_pt, cyaw, up_path.back(), q_carry)) {
    q_carry[4] = up_path.back()[4];
    if (!RunArmPath({q_carry}, ScaleDur(0.55, cfg, 0.38), is_canceled)) {
      SetGrip(cfg.grip_open_box, 0.40);
      return false;
    }
  }
  SetHeldPuck(true, target.is_square);

  // 기적재 슬롯 상단 여유 고도(slot_hover_z=0.318m) 경유 후 바닥 1.2mm 위에 무낙하 수직 안착
  on_phase("slot_place");
  const SlotPose sp = GetSlotPose(slot);
  const double sx = sp.pos[0] + cfg.slot_off[0];
  const double sy = sp.pos[1] + cfg.slot_off[1];
  const double align_z =
      ((slot >= 9) ? 0.258 : ((slot <= 4) ? 0.248 : 0.268)) + cfg.slot_off[2];
  const Vec3 slot_hover = {sx, sy, cfg.slot_hover_z};
  const Vec3 slot_align = {sx, sy, align_z};
  const Vec3 slot_drop = {sx, sy, sp.pos[2] + 0.0542 + cfg.slot_off[2]};

  Joint5 q_shover = {};
  Joint5 q_salign = {};
  Joint5 q_sdrop = {};
  if (!SolveHoverIk(slot_hover, sp.yaw, GetArmJoints(), q_shover, true) ||
      !SolveHoverIk(slot_align, sp.yaw, q_shover, q_salign, false) ||
      !SolveSlotDropIk(slot_drop, sp.yaw, q_salign, q_sdrop) ||
      !PlanMove(q_shover, cfg, is_canceled)) {
    SetHeldPuck(false, target.is_square);
    return false;
  }
  SetHeldPuck(false, target.is_square);

  const double s_pitch = q_shover[1] + q_shover[2] + q_shover[3];
  const double a_pitch = q_salign[1] + q_salign[2] + q_salign[3];
  const double d_pitch = q_sdrop[1] + q_sdrop[2] + q_sdrop[3];
  std::vector<Joint5> align_path;
  std::vector<Joint5> in_path;
  std::vector<Joint5> out_align_path;
  std::vector<Joint5> out_path;
  if (!MakeCartLine(slot_hover, s_pitch, slot_align, a_pitch, sp.yaw,
                    q_shover, 6, align_path) ||
      !RunArmPath(align_path, ScaleDur(0.60, cfg, 0.40), is_canceled) ||
      !MakeCartLine(slot_align, a_pitch, slot_drop, d_pitch, sp.yaw,
                    align_path.back(), 8, in_path) ||
      !RunArmPath(in_path, ScaleDur(0.82, cfg, 0.58), is_canceled)) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(130));
  if (!SetGrip(cfg.grip_open_slot, 0.42)) return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(120));

  return MakeCartLine(slot_drop, d_pitch, slot_align, a_pitch, sp.yaw,
                      in_path.back(), 8, out_align_path) &&
         RunArmPath(out_align_path, ScaleDur(0.62, cfg, 0.42), is_canceled) &&
         MakeCartLine(slot_align, a_pitch, slot_hover, s_pitch, sp.yaw,
                      out_align_path.back(), 6, out_path) &&
         RunArmPath(out_path, ScaleDur(0.55, cfg, 0.36), is_canceled);
}

// 바스켓 슬롯에서 기적재 퍽들을 회피해 퍽 상단부(+53mm)를 수직 인출 후 상자 위에 직립 배치
bool MotionCtrl::PlaceFromSlot(double box_x, double box_y, int slot,
                               double dst_x, double dst_y,
                               const std::vector<Vec3>& box_obs,
                               const MotionCfg& cfg, const PhaseFn& on_phase,
                               const CancelFn& is_canceled) {
  if (!WaitReady(is_canceled)) return false;
  const double box_top = cfg.ground_z + kBoxSize[2];
  SyncScene(box_x, box_y, cfg.ground_z, box_obs, cfg.slot_off, slot);

  // 바스켓 슬롯 상공에서 수직 정렬 후 직선 하강하여 상단 파지·인출
  on_phase("slot_pick");
  if (!SetGrip(cfg.grip_open_slot, 0.38)) return false;

  const SlotPose sp = GetSlotPose(slot);
  const double sx = sp.pos[0] + cfg.slot_off[0];
  const double sy = sp.pos[1] + cfg.slot_off[1];
  constexpr double kPickDz = 0.053;
  const double align_z =
      ((slot >= 9) ? 0.258 : ((slot <= 4) ? 0.248 : 0.268)) + cfg.slot_off[2];
  const Vec3 slot_hover = {sx, sy, cfg.slot_hover_z};
  const Vec3 slot_align = {sx, sy, align_z};
  const Vec3 slot_pick = {sx, sy, sp.pos[2] + kPickDz + cfg.slot_off[2]};

  Joint5 q_shover = {};
  Joint5 q_salign = {};
  Joint5 q_spick = {};
  if (!SolveHoverIk(slot_hover, sp.yaw, GetArmJoints(), q_shover, true) ||
      !SolveHoverIk(slot_align, sp.yaw, q_shover, q_salign, false) ||
      !SolveSlotDropIk(slot_pick, sp.yaw, q_salign, q_spick) ||
      !PlanMove(q_shover, cfg, is_canceled)) {
    return false;
  }

  const double s_pitch = q_shover[1] + q_shover[2] + q_shover[3];
  const double a_pitch = q_salign[1] + q_salign[2] + q_salign[3];
  const double p_pitch = q_spick[1] + q_spick[2] + q_spick[3];
  std::vector<Joint5> align_path;
  std::vector<Joint5> in_path;
  std::vector<Joint5> out_align_path;
  std::vector<Joint5> out_path;
  if (!MakeCartLine(slot_hover, s_pitch, slot_align, a_pitch, sp.yaw,
                    q_shover, 6, align_path) ||
      !RunArmPath(align_path, ScaleDur(0.60, cfg, 0.40), is_canceled)) {
    return false;
  }

  // 슬롯 내부 파지 시에도 닫은 직후·인출 직후 2회 파지 판정 수행
  Joint5 align_q = align_path.back();
  bool slot_picked = false;
  for (int att = 0; att < 2; ++att) {
    const Vec3 cur_pick = {slot_pick[0], slot_pick[1],
                           slot_pick[2] - 0.0015 * static_cast<double>(att)};
    in_path.clear();
    out_align_path.clear();
    if (!SetGrip(cfg.grip_open_slot, 0.38) ||
        !MakeCartLine(slot_align, a_pitch, cur_pick, p_pitch, sp.yaw,
                      align_q, 8, in_path) ||
        !RunArmPath(in_path, ScaleDur(0.78, cfg, 0.54), is_canceled)) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!SetGrip(cfg.grip_close, 0.45)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(140));

    if (!IsPuckHeld(cfg)) {
      RCLCPP_WARN(node_->get_logger(),
                  "슬롯 파지 빈손 감지(1차, 시도 %d/2, finger_l=%.4fm)",
                  att + 1, finger_pos_.load(std::memory_order_relaxed));
      SetGrip(cfg.grip_open_slot, 0.38);
      if (MakeCartLine(cur_pick, p_pitch, slot_align, a_pitch, sp.yaw,
                       in_path.back(), 8, out_align_path) &&
          RunArmPath(out_align_path, ScaleDur(0.65, cfg, 0.45), is_canceled)) {
        align_q = out_align_path.back();
      }
      continue;
    }

    if (!MakeCartLine(cur_pick, p_pitch, slot_align, a_pitch, sp.yaw,
                      in_path.back(), 8, out_align_path) ||
        !RunArmPath(out_align_path, ScaleDur(0.68, cfg, 0.48), is_canceled)) {
      SetGrip(cfg.grip_open_slot, 0.40);
      return false;
    }
    align_q = out_align_path.back();
    std::this_thread::sleep_for(std::chrono::milliseconds(90));

    if (!IsPuckHeld(cfg)) {
      RCLCPP_WARN(node_->get_logger(),
                  "슬롯 인출 후 퍽 이탈 감지(2차, 시도 %d/2, finger_l=%.4fm)",
                  att + 1, finger_pos_.load(std::memory_order_relaxed));
      SetGrip(cfg.grip_open_slot, 0.38);
      continue;
    }

    slot_picked = true;
    break;
  }
  if (!slot_picked) return false;

  if (!MakeCartLine(slot_align, a_pitch, slot_hover, s_pitch, sp.yaw,
                    align_q, 6, out_path) ||
      !RunArmPath(out_path, ScaleDur(0.55, cfg, 0.38), is_canceled)) {
    SetGrip(cfg.grip_open_slot, 0.40);
    return false;
  }
  SetHeldPuck(true, true);

  // 바스켓 기적재 퍽·격벽 회피 고도(z=0.318m) 전방 선회 후 상자 상판 1.5mm 위에 수직 안착
  on_phase("box_place");
  const double bx = dst_x + cfg.place_off[0];
  const double by = dst_y + cfg.place_off[1];
  const double byaw = std::atan2(by, bx - 0.108);
  const Vec3 carry_pt = {0.220, std::clamp(by, -0.050, 0.050),
                         cfg.slot_hover_z};
  const double cyaw = std::atan2(carry_pt[1], carry_pt[0] - 0.108);
  const Vec3 box_hover = {bx, by, cfg.box_hover_z};
  const Vec3 box_drop = {bx, by,
                         box_top + 0.0545 + cfg.place_off[2] +
                             GetDroopZ(cfg, bx, by)};

  Joint5 q_carry = {};
  Joint5 q_bhover = {};
  if (!SolveHoverIk(carry_pt, cyaw, GetArmJoints(), q_carry) ||
      !SolveHoverIk(box_hover, byaw, q_carry, q_bhover)) {
    SetHeldPuck(false, true);
    return false;
  }
  q_carry[4] = q_bhover[4];
  if (!PlanMove(q_carry, cfg, is_canceled)) {
    SetHeldPuck(false, true);
    return false;
  }
  SetHeldPuck(false, true);
  if (!RunArmPath({q_bhover}, ScaleDur(0.55, cfg, 0.38), is_canceled)) {
    return false;
  }

  const double b_pitch = q_bhover[1] + q_bhover[2] + q_bhover[3];
  std::vector<Joint5> down_path;
  std::vector<Joint5> up_path;
  if (!MakeCartLine(box_hover, b_pitch, box_drop, 0.5 * kPi, byaw, q_bhover, 10,
                    down_path) ||
      !RunArmPath(down_path, ScaleDur(0.85, cfg, 0.60), is_canceled)) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(130));
  if (!SetGrip(cfg.grip_open_box, 0.42)) return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(120));

  return MakeCartLine(box_drop, 0.5 * kPi, box_hover, b_pitch, byaw,
                      down_path.back(), 8, up_path) &&
         RunArmPath(up_path, ScaleDur(0.68, cfg, 0.45), is_canceled);
}

}  // namespace xsting_pick_place
