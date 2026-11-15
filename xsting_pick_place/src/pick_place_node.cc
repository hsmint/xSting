#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <xsting_interfaces/action/pick_place.hpp>

#include "xsting_pick_place/mission_rules.h"
#include "xsting_pick_place/motion_ctrl.h"
#include "xsting_pick_place/puck_det.h"

namespace xsting_pick_place {

// cuMotion + MoveIt 기반 상자 퍽 적재·기립·배치 액션 서버 노드
class PickNode : public rclcpp::Node {
 public:
  static constexpr uint8_t kCmdRightOnly = 5;
  static constexpr uint8_t kCmdVisionProbe = 6;
  using PickAct = xsting_interfaces::action::PickPlace;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PickAct>;
  using DetArray = vision_msgs::msg::Detection3DArray;

  PickNode(const rclcpp::Node::SharedPtr& move_node)
      : Node("pick_place"), ctrl_(move_node) {
    cfg_ = LoadParams();

    // 3D 비전 검출 구독 및 실시간 파라미터 콜백 등록
    det_sub_ = create_subscription<DetArray>(
        "/detections_3d", 10, [this](const DetArray::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx_);
          det_msg_ = msg;
        });
    param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
          return OnParams(params);
        });

    // 비동기 초기화 및 액션 서버 시작
    init_thread_ = std::thread([this]() {
      try {
        ready_ = ctrl_.Init(GetCfg());
        if (ready_) RCLCPP_INFO(get_logger(), "cuMotion + MoveIt 준비 완료");
      } catch (const std::exception& e) {
        if (rclcpp::ok()) RCLCPP_ERROR(get_logger(), "초기화 예외: %s", e.what());
      }
    });

    server_ = rclcpp_action::create_server<PickAct>(
        this, "pick_place",
        [this](const rclcpp_action::GoalUUID&,
               std::shared_ptr<const PickAct::Goal>) {
          return ready_ ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                        : rclcpp_action::GoalResponse::REJECT;
        },
        [](const std::shared_ptr<GoalHandle>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle> gh) {
          std::thread(&PickNode::ExecuteGoal, this, gh).detach();
        });
  }

  ~PickNode() override {
    if (init_thread_.joinable()) init_thread_.join();
  }

 private:
  // ROS 파라미터 선언 및 초기 설정 구조체 생성
  MotionCfg LoadParams() {
    MotionCfg cfg;
    cfg.pipeline = declare_parameter("pipeline", cfg.pipeline);
    cfg.vel_scale = declare_parameter("vel_scale", cfg.vel_scale);
    cfg.acc_scale = declare_parameter("acc_scale", cfg.acc_scale);
    cfg.ground_z = declare_parameter("ground_z", cfg.ground_z);
    cfg.box_x = declare_parameter("box_x", cfg.box_x);
    cfg.box_y = declare_parameter("box_y", cfg.box_y);
    cfg.auto_box_det = declare_parameter("auto_box_det", cfg.auto_box_det);
    cfg.use_arducam = declare_parameter("use_arducam", cfg.use_arducam);
    cfg.arducam_look_dz =
        declare_parameter("arducam_look_dz", cfg.arducam_look_dz);
    cfg.grip_open_box = declare_parameter("grip_open_box", cfg.grip_open_box);
    cfg.grip_open_slot =
        declare_parameter("grip_open_slot", cfg.grip_open_slot);
    cfg.grip_loosen = declare_parameter("grip_loosen", cfg.grip_loosen);
    cfg.grip_close = declare_parameter("grip_close", cfg.grip_close);
    cfg.check_grasp = declare_parameter("check_grasp", cfg.check_grasp);
    cfg.grip_empty_th = declare_parameter("grip_empty_th", cfg.grip_empty_th);
    cfg.box_hover_z = declare_parameter("box_hover_z", cfg.box_hover_z);
    cfg.slot_hover_z = declare_parameter("slot_hover_z", cfg.slot_hover_z);
    cfg.pick_off = {declare_parameter("pick_off_x", 0.0),
                    declare_parameter("pick_off_y", 0.0),
                    declare_parameter("pick_off_z", 0.0)};
    cfg.place_off = {declare_parameter("place_off_x", 0.0),
                     declare_parameter("place_off_y", 0.0),
                     declare_parameter("place_off_z", 0.0)};
    cfg.slot_off = {declare_parameter("slot_off_x", 0.0),
                    declare_parameter("slot_off_y", 0.0),
                    declare_parameter("slot_off_z", 0.0)};
    cfg.right_tip_r = declare_parameter("right_tip_r", cfg.right_tip_r);
    cfg.right_grasp_dz =
        declare_parameter("right_grasp_dz", cfg.right_grasp_dz);
    cfg.right_lift_dz = declare_parameter("right_lift_dz", cfg.right_lift_dz);
    cfg.right_off_z = declare_parameter("right_off_z", 0.0);
    cfg.droop_r0 = declare_parameter("droop_r0", cfg.droop_r0);
    cfg.droop_k = declare_parameter("droop_k", 0.0);

    const auto occ =
        declare_parameter("occupied_slots", std::vector<int64_t>{});
    std::array<bool, 12> used = {};
    for (int64_t s : occ) {
      if (s >= 1 && s <= 12) used[static_cast<size_t>(s - 1)] = true;
    }
    ctrl_.SetOccupiedSlots(used);
    return cfg;
  }

  // 스레드 안전하게 현재 설정 복사본 반환
  MotionCfg GetCfg() {
    std::lock_guard<std::mutex> lock(mtx_);
    return cfg_;
  }

  // 자동 검출된 상자 중심 좌표를 내부 설정에도 동기화
  void SaveBoxCenter(double box_x, double box_y) {
    std::lock_guard<std::mutex> lock(mtx_);
    cfg_.box_x = box_x;
    cfg_.box_y = box_y;
  }

  // 런타임 파라미터 변경 반영
  rcl_interfaces::msg::SetParametersResult OnParams(
      const std::vector<rclcpp::Parameter>& params) {
    std::lock_guard<std::mutex> lock(mtx_);
    for (const auto& p : params) {
      const auto& n = p.get_name();
      if (n == "vel_scale") cfg_.vel_scale = p.as_double();
      else if (n == "acc_scale") cfg_.acc_scale = p.as_double();
      else if (n == "ground_z") cfg_.ground_z = p.as_double();
      else if (n == "box_x") cfg_.box_x = p.as_double();
      else if (n == "box_y") cfg_.box_y = p.as_double();
      else if (n == "auto_box_det") cfg_.auto_box_det = p.as_bool();
      else if (n == "use_arducam") cfg_.use_arducam = p.as_bool();
      else if (n == "arducam_look_dz") cfg_.arducam_look_dz = p.as_double();
      else if (n == "grip_open_box") cfg_.grip_open_box = p.as_double();
      else if (n == "grip_open_slot") cfg_.grip_open_slot = p.as_double();
      else if (n == "grip_loosen") cfg_.grip_loosen = p.as_double();
      else if (n == "grip_close") cfg_.grip_close = p.as_double();
      else if (n == "check_grasp") cfg_.check_grasp = p.as_bool();
      else if (n == "grip_empty_th") cfg_.grip_empty_th = p.as_double();
      else if (n == "box_hover_z") cfg_.box_hover_z = p.as_double();
      else if (n == "slot_hover_z") cfg_.slot_hover_z = p.as_double();
      else if (n == "occupied_slots") {
        std::array<bool, 12> used = {};
        for (int64_t s : p.as_integer_array()) {
          if (s >= 1 && s <= 12) used[static_cast<size_t>(s - 1)] = true;
        }
        ctrl_.SetOccupiedSlots(used);
      }
      else if (n == "pick_off_x") cfg_.pick_off[0] = p.as_double();
      else if (n == "pick_off_y") cfg_.pick_off[1] = p.as_double();
      else if (n == "pick_off_z") cfg_.pick_off[2] = p.as_double();
      else if (n == "place_off_x") cfg_.place_off[0] = p.as_double();
      else if (n == "place_off_y") cfg_.place_off[1] = p.as_double();
      else if (n == "place_off_z") cfg_.place_off[2] = p.as_double();
      else if (n == "slot_off_x") cfg_.slot_off[0] = p.as_double();
      else if (n == "slot_off_y") cfg_.slot_off[1] = p.as_double();
      else if (n == "slot_off_z") cfg_.slot_off[2] = p.as_double();
      else if (n == "right_tip_r") cfg_.right_tip_r = p.as_double();
      else if (n == "right_grasp_dz") cfg_.right_grasp_dz = p.as_double();
      else if (n == "right_lift_dz") cfg_.right_lift_dz = p.as_double();
      else if (n == "right_off_z") cfg_.right_off_z = p.as_double();
      else if (n == "droop_r0") cfg_.droop_r0 = p.as_double();
      else if (n == "droop_k") cfg_.droop_k = p.as_double();
    }
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    return res;
  }

  // 액션 Goal의 수동 입력 배열을 정렬된 퍽 목록으로 변환
  std::vector<Puck> BuildManualPucks(const PickAct::Goal& goal,
                                     double box_top) const {
    const size_t n = std::min({goal.puck_x.size(), goal.puck_y.size(),
                               goal.is_square.size(), goal.is_lying.size()});
    std::vector<Puck> pucks;
    for (size_t i = 0; i < n; ++i) {
      const bool lying = goal.is_square[i] && goal.is_lying[i];
      const double def_z = box_top + (lying ? 0.015 : 0.030);
      const double pz = (i < goal.puck_z.size()) ? goal.puck_z[i] : def_z;
      const double yaw = (i < goal.puck_yaw.size()) ? goal.puck_yaw[i] : 0.0;
      const std::string cls =
          (i < goal.puck_classes.size()) ? goal.puck_classes[i] : "manual_puck";
      pucks.push_back({{goal.puck_x[i], goal.puck_y[i], pz},
                       yaw,
                       goal.is_square[i],
                       lying,
                       cls});
    }

    std::sort(pucks.begin(), pucks.end(), [](const Puck& a, const Puck& b) {
      if (a.is_lying != b.is_lying) return !a.is_lying;
      return std::hypot(a.pos[0] - 0.108, a.pos[1]) <
             std::hypot(b.pos[0] - 0.108, b.pos[1]);
    });
    return pucks;
  }

  // 3D 비전(/detections_3d) 또는 팔 ArduCam + YOLOv8 스캔으로 상자 위 퍽 검출
  std::vector<Puck> DetectBoxPucks(const std::string& filter_cls,
                                   MotionCfg& cfg,
                                   const MotionCtrl::PhaseFn& on_phase,
                                   const MotionCtrl::CancelFn& is_canceled,
                                   bool force_look) {
    const double box_top = cfg.ground_z + kBoxSize[2];
    DetArray::SharedPtr snap;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      snap = det_msg_;
    }

    if (snap) {
      if (!force_look) {
        auto found = FindBoxPucks(*snap, filter_cls, box_top, cfg.box_x,
                                  cfg.box_y, cfg.auto_box_det);
        if (!found.empty()) {
          if (cfg.auto_box_det) SaveBoxCenter(cfg.box_x, cfg.box_y);
          return found;
        }
      }
      if (!ctrl_.MoveObserve(cfg, is_canceled)) return {};
      std::this_thread::sleep_for(std::chrono::milliseconds(450));
      {
        std::lock_guard<std::mutex> lock(mtx_);
        snap = det_msg_;
      }
      if (snap) {
        auto found = FindBoxPucks(*snap, filter_cls, box_top, cfg.box_x,
                                  cfg.box_y, cfg.auto_box_det);
        if (!found.empty()) {
          if (cfg.auto_box_det) SaveBoxCenter(cfg.box_x, cfg.box_y);
          return found;
        }
      }
    }

    // 헤드 3D 검출이 없거나 비어 있으면 팔 ArduCam YOLOv8 상자·퍽 스캔 수행
    std::vector<Puck> cam_pucks;
    if (ctrl_.ScanBoxWithArducam(filter_cls, cfg, on_phase, is_canceled,
                                 cam_pucks) &&
        !cam_pucks.empty()) {
      if (cfg.auto_box_det) SaveBoxCenter(cfg.box_x, cfg.box_y);
      return cam_pucks;
    }

    if (!snap) {
      if (!ctrl_.MoveObserve(cfg, is_canceled)) return {};
      std::this_thread::sleep_for(std::chrono::milliseconds(450));
      {
        std::lock_guard<std::mutex> lock(mtx_);
        snap = det_msg_;
      }
      if (snap) {
        auto found = FindBoxPucks(*snap, filter_cls, box_top, cfg.box_x,
                                  cfg.box_y, cfg.auto_box_det);
        if (cfg.auto_box_det && !found.empty()) {
          SaveBoxCenter(cfg.box_x, cfg.box_y);
        }
        return found;
      }
    }
    return {};
  }

  // 상자 위 랜덤 퍽 1~2개를 순서대로 세우고 바스켓 슬롯에 수직 적재
  bool RunLoadBox(const PickAct::Goal& goal, MotionCfg cfg,
                  const std::shared_ptr<GoalHandle>& gh, PickAct::Result& res) {
    std::vector<int> slots = goal.slots;
    if (slots.empty()) slots = {1, 2};
    const int target_cnt =
        (goal.max_count > 0)
            ? std::min(goal.max_count, static_cast<int>(slots.size()))
            : static_cast<int>(slots.size());

    auto is_canceled = [&]() { return gh->is_canceling(); };
    PickAct::Feedback fb;
    fb.total_count = target_cnt;
    auto on_phase = [&](const std::string& name) {
      fb.phase = name;
      gh->publish_feedback(std::make_shared<PickAct::Feedback>(fb));
    };

    const double box_top = cfg.ground_z + kBoxSize[2];
    for (int s : goal.occupied_slots) ctrl_.SetSlotOccupied(s, true);
    std::vector<Puck> manual =
        goal.use_manual ? BuildManualPucks(goal, box_top) : std::vector<Puck>{};

    for (int i = 0; i < target_cnt; ++i) {
      fb.current_idx = i + 1;
      std::vector<Puck> pucks =
          goal.use_manual ? manual
                          : DetectBoxPucks(goal.target_class, cfg, on_phase,
                                           is_canceled, i > 0);
      if (pucks.empty()) {
        res.message = "상자 위에서 퍽을 찾지 못했습니다";
        ctrl_.MoveHome(cfg, is_canceled);
        return res.handled_count > 0;
      }

      Puck cur = pucks.front();
      const Puck* other = (pucks.size() > 1) ? &pucks[1] : nullptr;

      // 누워 있는 사각 퍽이면 집고 그리퍼를 살짝 풀어 세운 뒤 정밀 파지
      if (cur.is_lying) {
        if (!ctrl_.RightPuck(cfg.box_x, cfg.box_y, cur, other, cfg, on_phase,
                             is_canceled)) {
          res.message = "누운 사각 퍽 기립 실패";
          ctrl_.MoveHome(cfg, is_canceled);
          return false;
        }
        if (cfg.auto_box_det) SaveBoxCenter(cfg.box_x, cfg.box_y);
        ++res.righted_count;

        if (!goal.use_manual && !ctrl_.HasArducamStream()) {
          auto refreshed =
              DetectBoxPucks(cur.cls_id, cfg, on_phase, is_canceled, true);
          if (!refreshed.empty() && !refreshed.front().is_lying) {
            cur = refreshed.front();
          }
        }
      }

      // 직립 상태의 퍽을 파지해 지정 슬롯에 수직 적재
      const int slot = slots[static_cast<size_t>(i)];
      if (!ctrl_.PickToSlot(cfg.box_x, cfg.box_y, cur, other, slot, cfg,
                            on_phase, is_canceled)) {
        res.message = "바스켓 슬롯 적재 실패";
        ctrl_.MoveHome(cfg, is_canceled);
        return false;
      }
      if (cfg.auto_box_det) SaveBoxCenter(cfg.box_x, cfg.box_y);

      ctrl_.SetSlotOccupied(slot, true);
      ++res.handled_count;
      res.used_slots.push_back(slot);
      res.detected_classes.push_back(cur.cls_id);
      res.observed_puck_conf.push_back(cur.score);
      if (goal.use_manual && !manual.empty()) manual.erase(manual.begin());
    }

    res.message = "상자 퍽 적재 완료";
    return ctrl_.MoveHome(cfg, is_canceled);
  }

  // 바스켓에 적재된 퍽들을 꺼내 상자 위 좌표 또는 주사위 눈 격자에 올려놓기
  bool RunUnloadBox(const PickAct::Goal& goal, const MotionCfg& cfg,
                    const std::shared_ptr<GoalHandle>& gh,
                    PickAct::Result& res) {
    std::vector<int> slots = goal.slots;
    if (slots.empty()) slots = {1, 2};

    // 사용자 지정 오프셋(place_xy) 또는 주사위 눈/기본 2개 좌표 선택
    std::vector<std::pair<double, double>> offsets;
    for (size_t i = 0; i + 1 < goal.place_xy.size(); i += 2) {
      offsets.emplace_back(goal.place_xy[i], goal.place_xy[i + 1]);
    }
    if (offsets.empty()) offsets = GetBoxOffsets(goal.dice_eye);

    const int count =
        std::min(static_cast<int>(slots.size()),
                 static_cast<int>(offsets.size()));
    auto is_canceled = [&]() { return gh->is_canceling(); };
    PickAct::Feedback fb;
    fb.total_count = count;
    auto on_phase = [&](const std::string& name) {
      fb.phase = name;
      gh->publish_feedback(std::make_shared<PickAct::Feedback>(fb));
    };

    const double box_top = cfg.ground_z + kBoxSize[2];
    std::vector<Vec3> placed_obs;
    for (int s : goal.occupied_slots) ctrl_.SetSlotOccupied(s, true);
    for (int s : slots) ctrl_.SetSlotOccupied(s, true);

    for (int i = 0; i < count; ++i) {
      fb.current_idx = i + 1;
      const int slot = slots[static_cast<size_t>(i)];
      const double dx = cfg.box_x + offsets[static_cast<size_t>(i)].first;
      const double dy = cfg.box_y + offsets[static_cast<size_t>(i)].second;

      if (!ctrl_.PlaceFromSlot(cfg.box_x, cfg.box_y, slot, dx, dy, placed_obs,
                               cfg, on_phase, is_canceled)) {
        res.message = "상자 위 퍽 배치 실패";
        ctrl_.MoveHome(cfg, is_canceled);
        return false;
      }

      ctrl_.SetSlotOccupied(slot, false);
      placed_obs.push_back({dx, dy, box_top + 0.5 * kPuckHeight});
      ++res.handled_count;
      res.used_slots.push_back(slot);
    }

    res.message = "상자 위 퍽 배치 완료";
    return ctrl_.MoveHome(cfg, is_canceled);
  }

  // 상자 위 누운 사각 퍽만 단독으로 세우고 홈 복귀(누운 퍽 실측 보정 검증용)
  bool RunRightOnly(const PickAct::Goal& goal, MotionCfg cfg,
                    const std::shared_ptr<GoalHandle>& gh,
                    PickAct::Result& res) {
    auto is_canceled = [&]() { return gh->is_canceling(); };
    PickAct::Feedback fb;
    fb.current_idx = 1;
    fb.total_count = 1;
    auto on_phase = [&](const std::string& name) {
      fb.phase = name;
      gh->publish_feedback(std::make_shared<PickAct::Feedback>(fb));
    };

    for (int s : goal.occupied_slots) ctrl_.SetSlotOccupied(s, true);
    const double box_top = cfg.ground_z + kBoxSize[2];
    const std::string cls_flt =
        (goal.target_class == "lying") ? "" : goal.target_class;
    std::vector<Puck> pucks =
        goal.use_manual ? BuildManualPucks(goal, box_top)
                        : DetectBoxPucks(cls_flt, cfg, on_phase, is_canceled,
                                         true);
    if (pucks.empty()) {
      res.message = "상자 위에서 퍽을 찾지 못했습니다";
      ctrl_.MoveHome(cfg, is_canceled);
      return false;
    }

    size_t idx = pucks.size();
    for (size_t i = 0; i < pucks.size(); ++i) {
      if (pucks[i].is_lying) {
        idx = i;
        break;
      }
    }
    if (idx >= pucks.size()) {
      res.message = "상자 위에 누운 사각 퍽이 없습니다";
      ctrl_.MoveHome(cfg, is_canceled);
      return false;
    }

    Puck cur = pucks[idx];
    const Puck* other = nullptr;
    for (size_t i = 0; i < pucks.size(); ++i) {
      if (i != idx) {
        other = &pucks[i];
        break;
      }
    }

    if (!ctrl_.RightPuck(cfg.box_x, cfg.box_y, cur, other, cfg, on_phase,
                         is_canceled)) {
      res.message = "누운 사각 퍽 단독 기립 실패";
      ctrl_.MoveHome(cfg, is_canceled);
      return false;
    }
    if (cfg.auto_box_det) SaveBoxCenter(cfg.box_x, cfg.box_y);
    res.handled_count = 1;
    res.righted_count = 1;
    res.detected_classes.push_back(cur.cls_id);
    res.final_xyz = {cur.pos[0], cur.pos[1], cur.pos[2]};
    res.message = "누운 사각 퍽 단독 기립 완료";
    return ctrl_.MoveHome(cfg, is_canceled);
  }

  // ArduCam YOLOv8 또는 3D 비전으로 퍽을 정밀 검출해 직립·누운 파지 보정점으로 이동
  bool RunVisionProbe(const PickAct::Goal& goal, MotionCfg cfg,
                      const std::shared_ptr<GoalHandle>& gh,
                      PickAct::Result& res) {
    auto is_canceled = [&]() { return gh->is_canceling(); };
    PickAct::Feedback fb;
    fb.current_idx = 1;
    fb.total_count = 1;
    auto on_phase = [&](const std::string& name) {
      fb.phase = name;
      gh->publish_feedback(std::make_shared<PickAct::Feedback>(fb));
    };

    const bool want_lying = (goal.target_class == "lying");
    const std::string cls_flt = want_lying ? "" : goal.target_class;
    const double hover = goal.probe_xyz.empty() ? 0.025 : goal.probe_xyz[0];
    const double box_top = cfg.ground_z + kBoxSize[2];
    std::vector<Puck> pucks =
        goal.use_manual ? BuildManualPucks(goal, box_top)
                        : DetectBoxPucks(cls_flt, cfg, on_phase, is_canceled,
                                         true);
    if (pucks.empty()) {
      res.message = "비전으로 검출된 퍽이 없습니다";
      return false;
    }

    size_t idx = 0;
    if (want_lying) {
      idx = pucks.size();
      for (size_t i = 0; i < pucks.size(); ++i) {
        if (pucks[i].is_lying) {
          idx = i;
          break;
        }
      }
      if (idx >= pucks.size()) {
        res.message = "검출된 퍽 중 누운 사각 퍽이 없습니다";
        return false;
      }
    }

    Puck cur = pucks[idx];
    const Puck* other = nullptr;
    for (size_t i = 0; i < pucks.size(); ++i) {
      if (i != idx) {
        other = &pucks[i];
        break;
      }
    }

    Vec3 cmd_pt = {};
    Vec3 meas_pt = {};
    double pyaw = 0.0;
    const bool ok = ctrl_.ProbeVisionPuck(cfg.box_x, cfg.box_y, cur, other,
                                          hover, cfg, on_phase, is_canceled,
                                          cmd_pt, meas_pt, pyaw);
    if (cfg.auto_box_det) SaveBoxCenter(cfg.box_x, cfg.box_y);
    res.righted_count = cur.is_lying ? 1 : 0;
    res.detected_classes.push_back(cur.cls_id);
    res.final_xyz = {meas_pt[0], meas_pt[1], meas_pt[2], cmd_pt[0], cmd_pt[1],
                     cmd_pt[2], pyaw, cur.pos[0], cur.pos[1], cur.pos[2]};
    res.message = ok ? (cur.is_lying ? "누운 퍽 비전 프로브 완료"
                                     : "직립 퍽 비전 프로브 완료")
                     : "비전 프로브 이동 실패";
    return ok;
  }

  // 상자 위 3x3 배치 재검증(CMD_OBSERVE): 팔 끝 ArduCam 탑뷰 스캔 우선 수행 후 폴백
  bool RunObserveBox(const PickAct::Goal& goal, MotionCfg cfg,
                     const std::shared_ptr<GoalHandle>& gh,
                     PickAct::Result& res) {
    auto is_canceled = [&]() { return gh->is_canceling(); };
    PickAct::Feedback fb;
    fb.current_idx = 1;
    fb.total_count = 1;
    auto on_phase = [&](const std::string& name) {
      fb.phase = name;
      gh->publish_feedback(std::make_shared<PickAct::Feedback>(fb));
    };

    for (int s : goal.occupied_slots) ctrl_.SetSlotOccupied(s, true);

    std::vector<Puck> pucks;
    if (ctrl_.ScanBoxWithArducam(goal.target_class, cfg, on_phase, is_canceled,
                                 pucks, /*allow_empty=*/true)) {
      if (cfg.auto_box_det) SaveBoxCenter(cfg.box_x, cfg.box_y);
      res.observe_source = "arducam";
      res.observed_box_xy = {cfg.box_x, cfg.box_y};
      res.handled_count = static_cast<int32_t>(pucks.size());
      for (const auto& p : pucks) {
        res.detected_classes.push_back(p.cls_id);
        res.observed_puck_xyz.insert(res.observed_puck_xyz.end(),
                                     {p.pos[0], p.pos[1], p.pos[2]});
        res.observed_puck_conf.push_back(p.score);
      }
      on_phase("observe_home");
      const bool home_ok = ctrl_.MoveHome(cfg, is_canceled);
      res.message = home_ok ? "ArduCam 탑뷰 상자 관측 및 홈 복귀 완료"
                            : "ArduCam 탑뷰 상자 관측 완료(홈 복귀 실패)";
      return home_ok;
    }

    on_phase("move_observe_pose");
    if (!ctrl_.MoveObserve(cfg, is_canceled)) {
      res.message = "관측 자세 이동 실패";
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(450));
    DetArray::SharedPtr snap;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      snap = det_msg_;
    }
    if (snap) {
      const double box_top = cfg.ground_z + kBoxSize[2];
      auto found = FindBoxPucks(*snap, goal.target_class, box_top, cfg.box_x,
                                cfg.box_y, cfg.auto_box_det);
      if (cfg.auto_box_det && !found.empty()) {
        SaveBoxCenter(cfg.box_x, cfg.box_y);
      }
      res.observe_source = "det3d";
      res.observed_box_xy = {cfg.box_x, cfg.box_y};
      res.handled_count = static_cast<int32_t>(found.size());
      for (const auto& p : found) {
        res.detected_classes.push_back(p.cls_id);
        res.observed_puck_xyz.insert(res.observed_puck_xyz.end(),
                                     {p.pos[0], p.pos[1], p.pos[2]});
        res.observed_puck_conf.push_back(p.score);
      }
    }
    res.message = "관측 자세 이동 완료";
    return true;
  }

  // 수신된 액션 명령 분기 실행
  void ExecuteGoal(const std::shared_ptr<GoalHandle> gh) {
    const auto goal = gh->get_goal();
    auto res = std::make_shared<PickAct::Result>();
    const MotionCfg cfg = GetCfg();
    auto is_canceled = [&]() { return gh->is_canceling(); };

    if (goal->command == PickAct::Goal::CMD_HOME) {
      res->success = ctrl_.MoveHome(cfg, is_canceled);
      res->message = res->success ? "홈 복귀 완료" : "홈 복귀 실패";
    } else if (goal->command == PickAct::Goal::CMD_OBSERVE) {
      res->success = RunObserveBox(*goal, cfg, gh, *res);
    } else if (goal->command == PickAct::Goal::CMD_LOAD_BOX) {
      res->success = RunLoadBox(*goal, cfg, gh, *res);
    } else if (goal->command == PickAct::Goal::CMD_UNLOAD_BOX) {
      res->success = RunUnloadBox(*goal, cfg, gh, *res);
    } else if (goal->command == kCmdRightOnly) {
      res->success = RunRightOnly(*goal, cfg, gh, *res);
    } else if (goal->command == kCmdVisionProbe) {
      res->success = RunVisionProbe(*goal, cfg, gh, *res);
    } else if (goal->command == PickAct::Goal::CMD_PROBE &&
               goal->probe_xyz.size() >= 3) {
      Vec3 meas = {};
      const Vec3 target = {goal->probe_xyz[0], goal->probe_xyz[1],
                           goal->probe_xyz[2]};
      res->success =
          ctrl_.ProbePose(target, goal->probe_yaw, cfg, is_canceled, meas);
      res->final_xyz.assign(meas.begin(), meas.end());
      res->message = res->success ? "보정 프로브 이동 완료" : "프로브 이동 실패";
    } else {
      res->success = false;
      res->message = "알 수 없는 명령 번호";
    }

    if (gh->is_canceling()) {
      gh->canceled(res);
    } else {
      gh->succeed(res);
    }
  }

  MotionCtrl ctrl_;
  MotionCfg cfg_;
  std::mutex mtx_;
  bool ready_ = false;
  std::thread init_thread_;
  DetArray::SharedPtr det_msg_;
  rclcpp::Subscription<DetArray>::SharedPtr det_sub_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp_action::Server<PickAct>::SharedPtr server_;
};

}  // namespace xsting_pick_place

// MoveGroup 전용 스핀 스레드와 액션 서버 노드를 함께 실행
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions move_opts;
  move_opts.automatically_declare_parameters_from_overrides(true);
  auto move_node = std::make_shared<rclcpp::Node>("pick_move_group", move_opts);

  rclcpp::executors::MultiThreadedExecutor move_exec;
  move_exec.add_node(move_node);
  std::thread spin_thread([&]() { move_exec.spin(); });

  auto pick_node = std::make_shared<xsting_pick_place::PickNode>(move_node);
  rclcpp::spin(pick_node);

  move_exec.cancel();
  if (spin_thread.joinable()) spin_thread.join();
  pick_node.reset();
  move_node.reset();
  rclcpp::shutdown();
  return 0;
}
