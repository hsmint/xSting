#include "xsting_hardware/xsting_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace xsting_hardware {

namespace {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

constexpr double kStandDuration = 4.5;
constexpr double kArmHomeDuration = 3.0;

// xsting.urdf 기준 다리 평면 기구학 상수 (hip 원점 z = +72.72mm, 대퇴 81mm, 정강이 끝 (132.75, -20)mm)
constexpr double kHipHeightM = 0.0727219;
constexpr double kFemurLenM = 0.081;
constexpr double kTibiaLenM = 0.1342481378;   // hypot(0.13275, -0.020)
constexpr double kTibiaPhi0 = -0.1495342188;  // atan2(-0.020, 0.13275)
constexpr double kAirClearanceM = 0.008;
constexpr double kAirRadiusMarginM = 0.006;

double SmoothStep(double u) {
  return 0.5 * (1.0 - std::cos(M_PI * std::clamp(u, 0.0, 1.0)));
}

void LegFkRZ(double q_hip, double q_knee, double* r, double* z_base) {
  const double th = q_hip + q_knee + kTibiaPhi0;
  *r = kFemurLenM * std::cos(q_hip) + kTibiaLenM * std::cos(th);
  *z_base = kHipHeightM + kFemurLenM * std::sin(q_hip) + kTibiaLenM * std::sin(th);
}

void LegIkRZ(double r, double z_base, double* q_hip, double* q_knee) {
  const double z_rel = z_base - kHipHeightM;
  const double c2 = std::clamp(
      (r * r + z_rel * z_rel - kFemurLenM * kFemurLenM -
       kTibiaLenM * kTibiaLenM) /
          (2.0 * kFemurLenM * kTibiaLenM),
      -1.0, 1.0);
  const double psi2 = -std::acos(c2);
  *q_knee = psi2 - kTibiaPhi0;
  *q_hip = std::atan2(z_rel, r) -
           std::atan2(kTibiaLenM * std::sin(psi2),
                      kFemurLenM + kTibiaLenM * std::cos(psi2));
}

// 저부하 직교 수직 기립: 공중에서 보행 시작 발끝 XY 위로 정렬한 뒤 수평 미끄러짐 없이 수직 기립한다
void InterpLegStand(const LegBoard::Array& start, const LegBoard::Array& target,
                    double ratio, LegBoard::Array& out) {
  constexpr double kPhase1 = 0.38;
  for (size_t leg = 0; leg < 3; ++leg) {
    const size_t iy = 3 * leg;
    const size_t ih = iy + 1;
    const size_t ik = iy + 2;
    const double sy = start[iy], sh = start[ih], sk = start[ik];
    const double ty = target[iy], th = target[ih], tk = target[ik];

    double rs = 0.0, zs = 0.0, rt = 0.0, zt = 0.0;
    LegFkRZ(sh, sk, &rs, &zs);
    LegFkRZ(th, tk, &rt, &zt);

    const bool in_stance = (zs < -0.010) && (sk < -65.0 * M_PI / 180.0);
    if (in_stance || zt >= 0.0) {
      const double a = SmoothStep(ratio);
      out[iy] = sy + a * (ty - sy);
      out[ih] = sh + a * (th - sh);
      out[ik] = sk + a * (tk - sk);
      continue;
    }

    double q1_air = 0.0, q2_air = 0.0;
    LegIkRZ(rt + kAirRadiusMarginM, kAirClearanceM, &q1_air, &q2_air);

    if (ratio < kPhase1) {
      const double u = ratio / kPhase1;
      const double a = SmoothStep(u);
      const double ah =
          (sh < 45.0 * M_PI / 180.0) ? SmoothStep(std::min(1.0, u * 1.5)) : a;
      out[iy] = sy + a * (ty - sy);
      out[ih] = sh + ah * (q1_air - sh);
      out[ik] = sk + a * (q2_air - sk);
    } else {
      const double a = SmoothStep((ratio - kPhase1) / (1.0 - kPhase1));
      const double ar = SmoothStep(std::min(1.0, a / 0.15));
      const double r_cur =
          (rt + kAirRadiusMarginM) - ar * kAirRadiusMarginM;
      const double z_cur = kAirClearanceM + a * (zt - kAirClearanceM);
      double q1 = 0.0, q2 = 0.0;
      LegIkRZ(r_cur, z_cur, &q1, &q2);
      out[iy] = ty;
      out[ih] = q1;
      out[ik] = q2;
    }
  }
}

// 종료 복귀: 수직 IK로 배를 바닥에 부드럽게 내려놓은 뒤 공중에서 대기 자세로 접는다
void InterpLegReturn(const LegBoard::Array& start, const LegBoard::Array& wait,
                     double ratio, LegBoard::Array& out) {
  constexpr double kPhase1 = 0.40;
  for (size_t leg = 0; leg < 3; ++leg) {
    const size_t iy = 3 * leg;
    const size_t ih = iy + 1;
    const size_t ik = iy + 2;
    const double sy = start[iy], sh = start[ih], sk = start[ik];
    const double wy = wait[iy], wh = wait[ih], wk = wait[ik];

    double rs = 0.0, zs = 0.0;
    LegFkRZ(sh, sk, &rs, &zs);
    const bool from_stand =
        (zs < -0.005) && (sk < -65.0 * M_PI / 180.0) && (rs > 0.055);

    if (!from_stand) {
      const double a = SmoothStep(ratio);
      out[iy] = sy + a * (wy - sy);
      out[ih] = sh + a * (wh - sh);
      out[ik] = sk + a * (wk - sk);
      continue;
    }

    double q1_air = 0.0, q2_air = 0.0;
    LegIkRZ(rs + kAirRadiusMarginM, kAirClearanceM, &q1_air, &q2_air);

    if (ratio < kPhase1) {
      const double a = SmoothStep(ratio / kPhase1);
      const double ar =
          (a > 0.85) ? SmoothStep((a - 0.85) / 0.15) : 0.0;
      const double r_cur = rs + ar * kAirRadiusMarginM;
      const double z_cur = zs + a * (kAirClearanceM - zs);
      double q1 = 0.0, q2 = 0.0;
      LegIkRZ(r_cur, z_cur, &q1, &q2);
      out[iy] = sy;
      out[ih] = q1;
      out[ik] = q2;
    } else {
      const double a = SmoothStep((ratio - kPhase1) / (1.0 - kPhase1));
      out[iy] = sy + a * (wy - sy);
      out[ih] = q1_air + a * (wh - q1_air);
      out[ik] = q2_air + a * (wk - q2_air);
    }
  }
}

bool ParamBool(
    const std::unordered_map<std::string, std::string>& params,
    const std::string& key, bool default_val) {
  auto it = params.find(key);
  if (it == params.end()) return default_val;
  return it->second == "true" || it->second == "1";
}

JointMap ParseJoint(const hardware_interface::ComponentInfo& j,
                    size_t default_idx) {
  const auto& jp = j.parameters;
  JointMap m;
  m.name = j.name;
  m.idx = jp.count("motor_index") ? std::stoi(jp.at("motor_index"))
                                  : static_cast<uint8_t>(default_idx);
  m.dir = jp.count("direction") ? std::stod(jp.at("direction")) : 1.0;
  m.off = jp.count("offset") ? std::stod(jp.at("offset")) : 0.0;
  return m;
}

}  // namespace

std::vector<std::string> XstingSystem::UsedPorts() const {
  std::vector<std::string> ports;
  if (!front_.port_path().empty()) ports.push_back(front_.port_path());
  if (!rear_.port_path().empty()) ports.push_back(rear_.port_path());
  if (!arm_.port_path().empty()) ports.push_back(arm_.port_path());
  return ports;
}

bool XstingSystem::CheckBattery(const OpenCrBase& board, const char* label) {
  if (board.status() == -2) {
    RCLCPP_FATAL(get_logger(),
                 "치명적 저전압 (%.2fV, %s ID %u) - 현재 자세를 고정합니다!",
                 board.bat_volt(), label, board.id());
    return false;
  }
  if (board.status() == 1) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "배터리 부족 (%s ID %u): %.2fV", label, board.id(),
                         board.bat_volt());
  }
  return true;
}

void XstingSystem::CheckLegTorque(
    const LegBoard& board, const std::array<JointMap, kLegJoints>& joints,
    const LegBoard::Array& tx, const LegBoard::Array& pos,
    std::array<uint16_t, kLegJoints>& limp_cnt, const char* label) {
  if (!has_tx_leg_) return;
  const auto cur = board.GetCur();
  const auto& dxl_ids = board.DxlIds();
  for (size_t i = 0; i < kLegJoints; ++i) {
    const auto& j = joints[i];
    const uint8_t idx = j.idx;
    const double err_rad = pos[idx] - tx[idx];
    if (std::abs(err_rad) > 0.35 && std::abs(cur[idx]) <= 4) {
      if (limp_cnt[idx] < 1000) ++limp_cnt[idx];
      if (limp_cnt[idx] >= 50) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "모터 토크 상실 감지: %s (%s ID %u, DXL ID %u, 오차 %+.1fdeg, 전류 %d)"
            " - 과부하 셧다운(적색 LED 점멸) 또는 커넥터 순단 상태입니다. 보드 12V 전원을 재인가하세요!",
            j.name.c_str(), label, board.id(), dxl_ids[idx],
            err_rad * 180.0 / M_PI, cur[idx]);
      }
    } else {
      limp_cnt[idx] = 0;
    }
  }
}

CallbackReturn XstingSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto& hp = info_.hardware_parameters;
  use_leg_ = ParamBool(hp, "use_leg", true);
  use_arm_ = ParamBool(hp, "use_arm", true);
  stand_ = ParamBool(hp, "stand", true);
  const bool reboot_all = ParamBool(hp, "reboot", false) ||
                          ParamBool(hp, "reset_on_init", false);
  reboot_leg_ = reboot_all || ParamBool(hp, "reboot_leg", false);
  reboot_arm_ = reboot_all || ParamBool(hp, "reboot_arm", false);

  if (hp.count("opencr_baud_rate")) baud_ = std::stoi(hp.at("opencr_baud_rate"));
  if (hp.count("front_usb_port")) front_port_ = hp.at("front_usb_port");
  if (hp.count("rear_usb_port")) rear_port_ = hp.at("rear_usb_port");
  if (hp.count("arm_usb_port")) arm_port_ = hp.at("arm_usb_port");
  if (hp.count("gripper_current")) grip_cur_ = std::stoi(hp.at("gripper_current"));

  const size_t min_joints =
      (use_leg_ ? 2 * kLegJoints : 0) + (use_arm_ ? kArmJoints : 0);
  if (info_.joints.size() < min_joints) {
    RCLCPP_FATAL(get_logger(),
                 "관절 수(%zu)가 use_leg/use_arm 최소 요구치(%zu)보다 적습니다",
                 info_.joints.size(), min_joints);
    return CallbackReturn::ERROR;
  }

  size_t cursor = 0;
  if (use_leg_) {
    for (size_t i = 0; i < kLegJoints; ++i) {
      front_joints_[i] = ParseJoint(info_.joints[cursor++], i);
    }
    for (size_t i = 0; i < kLegJoints; ++i) {
      rear_joints_[i] = ParseJoint(info_.joints[cursor++], i);
    }
  }
  if (use_arm_) {
    for (size_t i = 0; i < kArmJoints; ++i) {
      arm_joints_[i] = ParseJoint(info_.joints[cursor++], i);
    }
    while (cursor < info_.joints.size()) {
      arm_mimics_.push_back(info_.joints[cursor++].name);
    }
  }

  // 켜진 보드 연결
  if (use_leg_) {
    if (!front_.Connect(front_port_, baud_, UsedPorts())) {
      RCLCPP_FATAL(get_logger(), "앞다리 OpenCR (ID %u) 연결 실패", kFrontId);
      return CallbackReturn::ERROR;
    }
    if (!rear_.Connect(rear_port_, baud_, UsedPorts())) {
      RCLCPP_FATAL(get_logger(), "뒷다리 OpenCR (ID %u) 연결 실패", kRearId);
      return CallbackReturn::ERROR;
    }
  }
  if (use_arm_) {
    if (!arm_.Connect(arm_port_, baud_, UsedPorts())) {
      RCLCPP_FATAL(get_logger(), "팔 OpenCR (ID %u) 연결 실패", kArmId);
      return CallbackReturn::ERROR;
    }
  }

  // 리셋 옵션이 지정된 보드만 Reset() 전송 후 부팅 완료까지 재연결 폴링
  const bool do_leg_reset = use_leg_ && reboot_leg_;
  const bool do_arm_reset = use_arm_ && reboot_arm_;
  if (do_leg_reset || do_arm_reset) {
    if (do_leg_reset) {
      RCLCPP_INFO(get_logger(), "다리 OpenCR (ID 200, 201) 리셋 전송...");
      front_.Reset();
      rear_.Reset();
    }
    if (do_arm_reset) {
      RCLCPP_INFO(get_logger(), "팔 OpenCR (ID 202) 리셋 전송...");
      arm_.Reset();
    }
    const int init_wait_sec = do_leg_reset ? 8 : 3;
    RCLCPP_INFO(get_logger(),
                "보드 재부팅 대기 중 (%d초 대기 후 최대 10초 재연결 시도)...",
                init_wait_sec);
    std::this_thread::sleep_for(std::chrono::seconds(init_wait_sec));

    bool front_ready = !do_leg_reset;
    bool rear_ready = !do_leg_reset;
    bool arm_ready = !do_arm_reset;
    for (int attempt = 0; attempt <= 20; ++attempt) {
      if (!front_ready) front_ready = front_.Connect("auto", baud_, UsedPorts());
      if (!rear_ready) rear_ready = rear_.Connect("auto", baud_, UsedPorts());
      if (!arm_ready) arm_ready = arm_.Connect("auto", baud_, UsedPorts());
      if (front_ready && rear_ready && arm_ready) break;
      if (attempt < 20) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }

    if (!front_ready || !rear_ready) {
      RCLCPP_FATAL(get_logger(),
                   "리셋 후 다리 OpenCR 재연결 실패 (front=%d, rear=%d)",
                   front_ready, rear_ready);
      return CallbackReturn::ERROR;
    }
    if (!arm_ready) {
      RCLCPP_FATAL(get_logger(), "리셋 후 팔 OpenCR 재연결 실패");
      return CallbackReturn::ERROR;
    }
  }

  // 모터 연결 상태 확인
  if (use_leg_) {
    if (!front_.ReadAll() || !front_.motors_ok()) {
      RCLCPP_FATAL(get_logger(), "앞다리 모터 미연결 (OpenCR ID %u)", kFrontId);
      return CallbackReturn::ERROR;
    }
    if (!rear_.ReadAll() || !rear_.motors_ok()) {
      RCLCPP_FATAL(get_logger(), "뒷다리 모터 미연결 (OpenCR ID %u)", kRearId);
      return CallbackReturn::ERROR;
    }
  }
  if (use_arm_) {
    if (!arm_.ReadAll() || !arm_.motors_ok()) {
      RCLCPP_FATAL(get_logger(), "팔 모터 미연결 (OpenCR ID %u)", kArmId);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn XstingSystem::on_activate(const rclcpp_lifecycle::State&) {
  if (use_leg_) {
    if (!front_.SetTorque(true) || !front_.ReadAll() ||
        !rear_.SetTorque(true) || !rear_.ReadAll()) {
      return CallbackReturn::ERROR;
    }

    front_start_ = front_.GetPos();
    rear_start_ = rear_.GetPos();
    stand_sec_ = stand_ ? 0.0 : kStandDuration;
    has_tx_leg_ = false;
    front_limp_cnt_.fill(0);
    rear_limp_cnt_.fill(0);

    const auto& f_target = stand_ ? front_.StandPose() : front_start_;
    const auto& r_target = stand_ ? rear_.StandPose() : rear_start_;
    for (size_t i = 0; i < kLegJoints; ++i) {
      const auto& fj = front_joints_[i];
      set_state(fj.name + "/position", fj.dir * front_start_[fj.idx] + fj.off);
      set_state(fj.name + "/velocity", 0.0);
      set_command(fj.name + "/position", fj.dir * f_target[fj.idx] + fj.off);

      const auto& rj = rear_joints_[i];
      set_state(rj.name + "/position", rj.dir * rear_start_[rj.idx] + rj.off);
      set_state(rj.name + "/velocity", 0.0);
      set_command(rj.name + "/position", rj.dir * r_target[rj.idx] + rj.off);
    }
    for (const auto& sensor : info_.sensors) {
      if (has_state(sensor.name + "/range")) {
        set_state(sensor.name + "/range", front_.GetRange());
      }
    }
    front_.PlaySound(1);
    rear_.PlaySound(1);
  }

  if (use_arm_) {
    if (!arm_.SetTorque(true) || !arm_.WriteGripperCur(grip_cur_) ||
        !arm_.ReadAll()) {
      return CallbackReturn::ERROR;
    }

    arm_start_ = arm_.GetPos();
    arm_sec_ = 0.0;
    holding_grip_ = false;
    grip_stall_cnt_ = 0;

    for (size_t i = 0; i < kArmJoints; ++i) {
      const auto& aj = arm_joints_[i];
      set_state(aj.name + "/position", aj.dir * arm_start_[aj.idx] + aj.off);
      set_state(aj.name + "/velocity", 0.0);
      set_command(aj.name + "/position", aj.dir * kArmHome[aj.idx] + aj.off);
    }
    const auto& gj = arm_joints_[5];
    for (const auto& mimic : arm_mimics_) {
      set_state(mimic + "/position", gj.dir * arm_start_[gj.idx] + gj.off);
      set_state(mimic + "/velocity", 0.0);
    }
    if (has_command(gj.name + "/effort")) {
      set_command(gj.name + "/effort", static_cast<double>(grip_cur_));
    }
    arm_.PlaySound(1);
  }

  return CallbackReturn::SUCCESS;
}

// 종료 시 켜져 있는 보드를 10초 동안 펌웨어 대기 자세로 천천히 복귀시킨다
CallbackReturn XstingSystem::on_deactivate(const rclcpp_lifecycle::State&) {
  if (use_leg_) {
    front_worker_.WaitIdle();
    rear_worker_.WaitIdle();
  }
  if (use_arm_) arm_worker_.WaitIdle();

  const bool has_leg = use_leg_ && front_.ReadAll() && rear_.ReadAll();
  const bool has_arm = use_arm_ && arm_.ReadAll();
  if (!has_leg && !has_arm) return CallbackReturn::SUCCESS;

  const auto f_start = has_leg ? front_.GetPos() : LegBoard::Array{};
  const auto r_start = has_leg ? rear_.GetPos() : LegBoard::Array{};
  const auto a_start = has_arm ? arm_.GetPos() : ArmBoard::Array{};
  const auto& f_wait = front_.WaitPose();
  const auto& r_wait = rear_.WaitPose();

  RCLCPP_INFO(get_logger(), "펌웨어 대기 자세로 천천히 복귀 중 (10초)...");
  constexpr double kReturnSec = 10.0;
  const auto t0 = std::chrono::steady_clock::now();

  while (true) {
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
            .count();
    const double ratio = std::clamp(elapsed / kReturnSec, 0.0, 1.0);
    const double a = SmoothStep(ratio);

    if (has_leg) {
      InterpLegReturn(f_start, f_wait, ratio, tx_front_);
      InterpLegReturn(r_start, r_wait, ratio, tx_rear_);
      front_worker_.Enqueue(this, [](void* p) {
        auto* s = static_cast<XstingSystem*>(p);
        s->front_.WritePos(s->tx_front_);
      });
      rear_worker_.Enqueue(this, [](void* p) {
        auto* s = static_cast<XstingSystem*>(p);
        s->rear_.WritePos(s->tx_rear_);
      });
    }

    if (has_arm) {
      for (size_t i = 0; i < kArmJoints; ++i) {
        tx_arm_[i] = a_start[i] + a * (kArmHome[i] - a_start[i]);
      }
      arm_worker_.Enqueue(this, [](void* p) {
        auto* s = static_cast<XstingSystem*>(p);
        s->arm_.WritePos(s->tx_arm_);
      });
    }

    if (has_leg) {
      front_worker_.WaitIdle();
      rear_worker_.WaitIdle();
    }
    if (has_arm) arm_worker_.WaitIdle();

    if (ratio >= 1.0) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn XstingSystem::on_shutdown(const rclcpp_lifecycle::State& prev) {
  if (prev.label() == hardware_interface::lifecycle_state_names::ACTIVE) {
    return on_deactivate(prev);
  }
  return CallbackReturn::SUCCESS;
}

return_type XstingSystem::read(const rclcpp::Time&, const rclcpp::Duration&) {
  if (use_leg_) {
    front_worker_.Enqueue(this, [](void* p) {
      auto* s = static_cast<XstingSystem*>(p);
      s->front_ok_ = s->front_.ReadAll();
    });
    rear_worker_.Enqueue(this, [](void* p) {
      auto* s = static_cast<XstingSystem*>(p);
      s->rear_ok_ = s->rear_.ReadAll();
    });
  }
  if (use_arm_) {
    arm_worker_.Enqueue(this, [](void* p) {
      auto* s = static_cast<XstingSystem*>(p);
      s->arm_ok_ = s->arm_.ReadAll();
    });
  }

  if (use_leg_) {
    front_worker_.WaitIdle();
    rear_worker_.WaitIdle();
  }
  if (use_arm_) arm_worker_.WaitIdle();

  if (use_leg_) {
    if (front_ok_) {
      if (!CheckBattery(front_, "앞다리")) return return_type::ERROR;
      const auto pos = front_.GetPos();
      const auto vel = front_.GetVel();
      CheckLegTorque(front_, front_joints_, tx_front_, pos, front_limp_cnt_,
                     "앞다리");
      for (size_t i = 0; i < kLegJoints; ++i) {
        const auto& j = front_joints_[i];
        set_state(j.name + "/position", j.dir * pos[j.idx] + j.off);
        set_state(j.name + "/velocity", j.dir * vel[j.idx]);
      }
      for (const auto& sensor : info_.sensors) {
        const std::string& s = sensor.name;
        if (has_state(s + "/range")) {
          set_state(s + "/range", front_.GetRange());
        } else if (front_.imu_ready() && has_state(s + "/orientation.w")) {
          const auto imu = front_.GetImu();
          set_state(s + "/orientation.w", imu.quat[0]);
          set_state(s + "/orientation.x", imu.quat[1]);
          set_state(s + "/orientation.y", imu.quat[2]);
          set_state(s + "/orientation.z", imu.quat[3]);
          set_state(s + "/angular_velocity.x", imu.gyro[0]);
          set_state(s + "/angular_velocity.y", imu.gyro[1]);
          set_state(s + "/angular_velocity.z", imu.gyro[2]);
          set_state(s + "/linear_acceleration.x", imu.acc[0]);
          set_state(s + "/linear_acceleration.y", imu.acc[1]);
          set_state(s + "/linear_acceleration.z", imu.acc[2]);
        }
      }
    }

    if (rear_ok_) {
      if (!CheckBattery(rear_, "뒷다리")) return return_type::ERROR;
      const auto pos = rear_.GetPos();
      const auto vel = rear_.GetVel();
      CheckLegTorque(rear_, rear_joints_, tx_rear_, pos, rear_limp_cnt_,
                     "뒷다리");
      for (size_t i = 0; i < kLegJoints; ++i) {
        const auto& j = rear_joints_[i];
        set_state(j.name + "/position", j.dir * pos[j.idx] + j.off);
        set_state(j.name + "/velocity", j.dir * vel[j.idx]);
      }
    }
  }

  if (use_arm_ && arm_ok_) {
    if (!CheckBattery(arm_, "팔")) return return_type::ERROR;
    const auto pos = arm_.GetPos();
    const auto vel = arm_.GetVel();
    for (size_t i = 0; i < kArmJoints; ++i) {
      const auto& j = arm_joints_[i];
      set_state(j.name + "/position", j.dir * pos[j.idx] + j.off);
      set_state(j.name + "/velocity", j.dir * vel[j.idx]);
    }
    const auto& gj = arm_joints_[5];
    for (const auto& mimic : arm_mimics_) {
      set_state(mimic + "/position", gj.dir * pos[gj.idx] + gj.off);
      set_state(mimic + "/velocity", gj.dir * vel[gj.idx]);
    }
  }

  return return_type::OK;
}

return_type XstingSystem::write(const rclcpp::Time&,
                             const rclcpp::Duration& period) {
  if (use_leg_) {
    front_worker_.WaitIdle();
    rear_worker_.WaitIdle();

    for (size_t i = 0; i < kLegJoints; ++i) {
      const auto& fj = front_joints_[i];
      const double fq = get_command<double>(fj.name + "/position");
      tx_front_[fj.idx] = (fq - fj.off) * fj.dir;

      const auto& rj = rear_joints_[i];
      const double rq = get_command<double>(rj.name + "/position");
      tx_rear_[rj.idx] = (rq - rj.off) * rj.dir;
    }

    if (stand_ && stand_sec_ < kStandDuration) {
      stand_sec_ = std::min(kStandDuration,
                            stand_sec_ + std::clamp(period.seconds(), 0.0, 0.05));
      const double ratio = stand_sec_ / kStandDuration;
      InterpLegStand(front_start_, tx_front_, ratio, tx_front_);
      InterpLegStand(rear_start_, tx_rear_, ratio, tx_rear_);
    }
    has_tx_leg_ = true;

    front_worker_.Enqueue(this, [](void* p) {
      auto* s = static_cast<XstingSystem*>(p);
      s->front_.WritePos(s->tx_front_);
    });
    rear_worker_.Enqueue(this, [](void* p) {
      auto* s = static_cast<XstingSystem*>(p);
      s->rear_.WritePos(s->tx_rear_);
    });
  }

  if (use_arm_) {
    arm_worker_.WaitIdle();
    tx_write_grip_ = false;

    const auto& gj = arm_joints_[5];
    if (has_command(gj.name + "/effort")) {
      const double cur = get_command<double>(gj.name + "/effort");
      if (std::isfinite(cur) && static_cast<int16_t>(cur) != grip_cur_) {
        grip_cur_ = static_cast<int16_t>(cur);
        tx_grip_cur_ = grip_cur_;
        tx_write_grip_ = true;
      }
    }

    for (size_t i = 0; i < kArmJoints; ++i) {
      const auto& aj = arm_joints_[i];
      const double q = get_command<double>(aj.name + "/position");
      tx_arm_[aj.idx] = (q - aj.off) * aj.dir;
    }

    if (arm_sec_ < kArmHomeDuration) {
      arm_sec_ = std::min(kArmHomeDuration,
                          arm_sec_ + std::clamp(period.seconds(), 0.0, 0.05));
      const double a =
          0.5 * (1.0 - std::cos(M_PI * arm_sec_ / kArmHomeDuration));
      for (size_t i = 0; i < kArmJoints; ++i) {
        tx_arm_[i] = arm_start_[i] + a * (tx_arm_[i] - arm_start_[i]);
      }
    }

    // 과도한 파지 와인드업 방지 및 내측 예압 유지 (실제 정지·스톨 지속 시에만 래치)
    const double cur_pos = arm_.GetPos()[5];
    const double cur_vel = arm_.GetVel()[5];
    const int16_t cur_th = std::max<int16_t>(140, (grip_cur_ * 4) / 5);
    if (arm_sec_ < kArmHomeDuration) {
      holding_grip_ = false;
      grip_stall_cnt_ = 0;
    } else if (holding_grip_) {
      if (tx_arm_[5] > hold_pos_ + 0.0008 || tx_arm_[5] >= cur_pos) {
        holding_grip_ = false;
        grip_stall_cnt_ = 0;
      }
    } else if (tx_arm_[5] < cur_pos - 0.0008 &&
               std::abs(arm_.grip_cur()) > cur_th &&
               std::abs(cur_vel) < 0.0025) {
      if (++grip_stall_cnt_ >= 30) {
        hold_pos_ = std::max(tx_arm_[5], cur_pos - 0.0010);
        holding_grip_ = true;
      }
    } else {
      grip_stall_cnt_ = 0;
    }
    if (holding_grip_) tx_arm_[5] = hold_pos_;

    arm_worker_.Enqueue(this, [](void* p) {
      auto* s = static_cast<XstingSystem*>(p);
      if (s->tx_write_grip_) s->arm_.WriteGripperCur(s->tx_grip_cur_);
      s->arm_.WritePos(s->tx_arm_);
    });
  }

  return return_type::OK;
}

}  // namespace xsting_hardware

PLUGINLIB_EXPORT_CLASS(xsting_hardware::XstingSystem,
                       hardware_interface::SystemInterface)
