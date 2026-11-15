#ifndef XSTING_PICK_PLACE_MOTION_CTRL_H_
#define XSTING_PICK_PLACE_MOTION_CTRL_H_

#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "xsting_pick_place/arducam_align.h"
#include "xsting_pick_place/arm_kin.h"
#include "xsting_pick_place/puck_det.h"

namespace xsting_pick_place {

// 모션 계획 및 실기 보정 파라미터 구조체
struct MotionCfg {
  std::string pipeline = "isaac_ros_cumotion";
  double vel_scale = 0.70;
  double acc_scale = 0.60;
  double ground_z = -0.0213;
  double box_x = 0.285;
  double box_y = 0.0;
  bool auto_box_det = true;
  bool use_arducam = true;
  double arducam_look_dz = 0.235;
  double grip_open_box = 0.0165;
  double grip_open_slot = 0.0078;
  double grip_loosen = 0.00474;
  double grip_close = 0.0008;
  bool check_grasp = true;
  double grip_empty_th = 0.0025;
  double box_hover_z = 0.215;
  double slot_hover_z = 0.318;
  Vec3 pick_off = {0.0, 0.0, 0.0};
  Vec3 place_off = {0.0, 0.0, 0.0};
  Vec3 slot_off = {0.0, 0.0, 0.0};
  double right_tip_r = 0.0190;
  double right_grasp_dz = 0.0210;
  double right_lift_dz = 0.0880;
  double right_off_z = 0.0;
  double droop_r0 = 0.200;
  double droop_k = 0.0;
};

// MoveIt + cuMotion 계획 및 정밀 그리퍼·직선 제어 클래스
class MotionCtrl {
 public:
  using TrajAct = control_msgs::action::FollowJointTrajectory;
  using PhaseFn = std::function<void(const std::string&)>;
  using CancelFn = std::function<bool()>;

  explicit MotionCtrl(const rclcpp::Node::SharedPtr& node);

  // MoveGroup 및 컨트롤러 액션 클라이언트 연결 초기화
  bool Init(const MotionCfg& cfg);

  // 바스켓 슬롯(1~12)의 기적재 퍽 점유 상태를 전체 또는 개별 갱신
  void SetOccupiedSlots(const std::array<bool, 12>& used);
  void SetSlotOccupied(int slot, bool used);

  // 팔 ArduCam 스트림이 활성 상태인지 확인
  bool HasArducamStream() const;

  // 팔 ArduCam + YOLOv8로 상자 전체를 조망하여 상자(paper_box)와 퍽 목록 검출
  bool ScanBoxWithArducam(const std::string& filter_cls, MotionCfg& cfg,
                          const PhaseFn& on_phase, const CancelFn& is_canceled,
                          std::vector<Puck>& out_pucks,
                          bool allow_empty = false);

  // 그리퍼 개방 후 홈 자세 복귀
  bool MoveHome(const MotionCfg& cfg, const CancelFn& is_canceled);

  // 상자 관측용 팔 자세로 이동
  bool MoveObserve(const MotionCfg& cfg, const CancelFn& is_canceled);

  // 실측 보정을 위해 지정 패드 좌표·Yaw로 안전 이동 후 도달 좌표 반환
  bool ProbePose(const Vec3& target, double yaw, const MotionCfg& cfg,
                 const CancelFn& is_canceled, Vec3& meas_out);

  // 검출된 직립·누운 퍽을 ArduCam으로 정밀 정렬한 뒤 실제 파지 보정 위치로 프로브 이동
  bool ProbeVisionPuck(double& box_x, double& box_y, Puck& target,
                       const Puck* other, double hover, const MotionCfg& cfg,
                       const PhaseFn& on_phase, const CancelFn& is_canceled,
                       Vec3& cmd_out, Vec3& meas_out, double& yaw_out);

  // 누운 사각 퍽을 집고 그리퍼를 살짝 풀어 직립 상태로 세우기
  bool RightPuck(double& box_x, double& box_y, Puck& target, const Puck* other,
                 const MotionCfg& cfg, const PhaseFn& on_phase,
                 const CancelFn& is_canceled);

  // 세워진 퍽을 상자에서 집어 지정된 바스켓 슬롯에 수직으로 적재
  bool PickToSlot(double& box_x, double& box_y, const Puck& puck,
                  const Puck* other, int slot, const MotionCfg& cfg,
                  const PhaseFn& on_phase, const CancelFn& is_canceled);

  // 바스켓 슬롯의 퍽을 꺼내 상자 위 목표 좌표에 올려놓기
  bool PlaceFromSlot(double box_x, double box_y, int slot, double dst_x,
                     double dst_y, const std::vector<Vec3>& box_obs,
                     const MotionCfg& cfg, const PhaseFn& on_phase,
                     const CancelFn& is_canceled);

 private:
  // 시뮬레이션 Play 또는 하드웨어 관절 상태가 수신될 때까지 대기
  bool WaitReady(const CancelFn& is_canceled);

  // 현재 팔 5축 관절 각도 조회
  Joint5 GetArmJoints() const;

  // PlanningScene에 상자, 상자 위 퍽, 바스켓 기적재 퍽 장애물 동기화
  void SyncScene(double box_x, double box_y, double ground_z,
                 const std::vector<Vec3>& obs_pucks,
                 const Vec3& slot_off = {0.0, 0.0, 0.0}, int skip_slot = 0);

  // 그리퍼에 파지 퍽 충돌 객체 부착 또는 해제
  void SetHeldPuck(bool attach, bool is_square);

  // cuMotion 우선 및 OMPL 폴백으로 관절 목표 계획·실행
  bool PlanMove(const Joint5& goal, const MotionCfg& cfg,
                const CancelFn& is_canceled);

  // 다중 관절 웨이포인트를 팔 컨트롤러 액션으로 연속 실행
  bool RunArmPath(const std::vector<Joint5>& path, double total_sec,
                  const CancelFn& is_canceled);

  // 그리퍼 목표 폭 명령 전송 및 완료 대기
  bool SetGrip(double width, double sec);

  // 그리퍼 손가락 위치(finger_l >= grip_empty_th)로 실제 퍽 파지 여부 판정
  bool IsPuckHeld(const MotionCfg& cfg) const;

  // 먼 위치·역기구학 한계의 누운 퍽 로봇 쪽 끝을 잡아 상자 중앙으로 끌어오기
  bool PullFarLyingPuck(double box_x, double box_y, Puck& target,
                        const Puck* other, double lyaw, const MotionCfg& cfg,
                        const PhaseFn& on_phase, const CancelFn& is_canceled);

  // 활성 ArduCam 스트림이 있을 때 2-Pass 직상방 수직 관측으로 시차 제거 정밀 보정
  bool AlignWithArducam(double& box_x, double& box_y, Puck& target,
                        const Puck* other, const MotionCfg& cfg,
                        const PhaseFn& on_phase, const CancelFn& is_canceled);

  rclcpp::Node::SharedPtr node_;
  std::atomic<bool> js_ready_{false};
  std::atomic<double> finger_pos_{0.010};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  std::unique_ptr<ArducamAlign> arducam_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> scene_;
  rclcpp_action::Client<TrajAct>::SharedPtr arm_cli_;
  rclcpp_action::Client<TrajAct>::SharedPtr grip_cli_;
  std::vector<std::string> obs_ids_;
  mutable std::mutex slot_mtx_;
  std::array<bool, 12> slot_used_{};
  bool has_held_ = false;
};

}  // namespace xsting_pick_place

#endif  // XSTING_PICK_PLACE_MOTION_CTRL_H_
