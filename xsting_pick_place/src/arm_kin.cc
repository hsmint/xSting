#include "xsting_pick_place/arm_kin.h"

#include <algorithm>
#include <cmath>

namespace xsting_pick_place {

namespace {

// xsting.urdf 기준 팔 링크 길이·오프셋·관절 가동 범위 상수
constexpr double kPi = 3.14159265358979323846;
constexpr double kJ1X = 0.108;
constexpr double kJ2Z = 0.237222;
constexpr double kL2X = 0.024;
constexpr double kL2Z = 0.128;
constexpr double kL3 = 0.149;
constexpr double kL4Pad = 0.17255;

constexpr Joint5 kMinQ = {-3.14159, -1.65806, -2.00713, -1.83260, -3.14159};
// 실기 arm_wrist(ID 14) 기계적 상단 스토퍼(108.8도) 보호를 위해 1.890rad(108.3도)로 제한
constexpr Joint5 kMaxQ = {3.14159, 1.91986, 1.57080, 1.89000, 3.14159};

// 관절 배열이 cuMotion 여유 마진(0.02rad)을 포함한 URDF 범위 내인지 검사
bool InLimits(const Joint5& joints) {
  constexpr double kMargin = 0.008;
  for (size_t i = 0; i < 5; ++i) {
    if (joints[i] < kMinQ[i] + kMargin || joints[i] > kMaxQ[i] - kMargin) {
      return false;
    }
  }
  return true;
}

// 기준 관절과의 가중 거리 산출
double JointDist(const Joint5& a, const Joint5& b) {
  double sum = 0.0;
  for (size_t i = 0; i < 5; ++i) {
    const double diff = WrapPi(a[i] - b[i]);
    sum += diff * diff;
  }
  return sum;
}

}  // namespace

// 임의 라디안 각도를 [-pi, pi] 구간으로 변환
double WrapPi(double rad) { return std::atan2(std::sin(rad), std::cos(rad)); }

// 그리퍼 대칭 주기를 활용해 불필요한 손목 롤 회전 최소화
double WrapSym(double target, double ref, double period) {
  const double half = 0.5 * period;
  double base = std::fmod(target + half, period);
  if (base < 0.0) base += period;
  base -= half;

  const double alt = (base >= 0.0) ? (base - period) : (base + period);
  if (std::abs(alt - ref) + 0.05 < std::abs(base - ref) &&
      alt >= kMinQ[4] && alt <= kMaxQ[4]) {
    return alt;
  }
  return base;
}

// 5축 정기구학으로 그리퍼 패드 중심 좌표 산출
Vec3 ForwardPad(const Joint5& joints) {
  const double l2 = std::hypot(kL2X, kL2Z);
  const double alpha = std::atan2(kL2X, kL2Z);
  const double pitch = joints[1] + joints[2] + joints[3];
  const double rad = l2 * std::sin(alpha + joints[1]) +
                     kL3 * std::cos(joints[1] + joints[2]) +
                     kL4Pad * std::cos(pitch);
  const double z = kJ2Z + l2 * std::cos(alpha + joints[1]) -
                   kL3 * std::sin(joints[1] + joints[2]) -
                   kL4Pad * std::sin(pitch);
  return {kJ1X + rad * std::cos(joints[0]), rad * std::sin(joints[0]), z};
}

// 5축 정기구학으로 arducam_frame 광학 원점과 우·하·전방 단위축 산출
void ForwardArducam(const Joint5& joints, Vec3& cam_pos,
                    std::array<Vec3, 3>& cam_axes) {
  const double l2 = std::hypot(kL2X, kL2Z);
  const double alpha = std::atan2(kL2X, kL2Z);
  const double yaw = joints[0];
  const double pitch = joints[1] + joints[2] + joints[3];
  const double roll = joints[4];

  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);

  // 손목에서 gripper 원점 및 회전 후 로컬 X·Y·Z 축 구성
  const double rw = l2 * std::sin(alpha + joints[1]) +
                    kL3 * std::cos(joints[1] + joints[2]);
  const double zw = kJ2Z + l2 * std::cos(alpha + joints[1]) -
                    kL3 * std::sin(joints[1] + joints[2]);

  const Vec3 gx = {cp * cy, cp * sy, -sp};
  const Vec3 gy0 = {-sy, cy, 0.0};
  const Vec3 gz0 = {sp * cy, sp * sy, cp};
  const Vec3 gy = {cr * gy0[0] + sr * gz0[0], cr * gy0[1] + sr * gz0[1],
                   cr * gy0[2] + sr * gz0[2]};
  const Vec3 gz = {-sr * gy0[0] + cr * gz0[0], -sr * gy0[1] + cr * gz0[1],
                   -sr * gy0[2] + cr * gz0[2]};

  const Vec3 p_grip = {kJ1X + rw * cy + 0.0485 * gx[0],
                       rw * sy + 0.0485 * gx[1], zw + 0.0485 * gx[2]};
  cam_pos = {p_grip[0] + 0.0830322 * gx[0] + 0.04695 * gz[0],
             p_grip[1] + 0.0830322 * gx[1] + 0.04695 * gz[1],
             p_grip[2] + 0.0830322 * gx[2] + 0.04695 * gz[2]};

  // arducam 장착 피치(+30도) 및 광학 좌표계(-90, 0, -90) 회전 적용
  constexpr double kC30 = 0.8660254037844386;
  constexpr double kS30 = 0.5;
  const Vec3 ax_c = {kC30 * gx[0] - kS30 * gz[0], kC30 * gx[1] - kS30 * gz[1],
                     kC30 * gx[2] - kS30 * gz[2]};
  const Vec3 az_c = {kS30 * gx[0] + kC30 * gz[0], kS30 * gx[1] + kC30 * gz[1],
                     kS30 * gx[2] + kC30 * gz[2]};
  cam_axes[0] = {-gy[0], -gy[1], -gy[2]};
  cam_axes[1] = {-az_c[0], -az_c[1], -az_c[2]};
  cam_axes[2] = ax_c;
}

// 그리퍼 피치 60도(광축 수직 하향) 조건으로 카메라 목표 위치 역기구학 풀이
bool SolveArducamIk(const Vec3& cam_target, const Joint5& ref, Joint5& out) {
  const double yaw = std::atan2(cam_target[1], cam_target[0] - kJ1X);
  const Vec3 pad_pt = {cam_target[0] - 0.02015 * std::cos(yaw),
                       cam_target[1] - 0.02015 * std::sin(yaw),
                       cam_target[2] - 0.05900};
  if (SolvePadIk(pad_pt, kPi / 3.0, yaw, ref, out)) {
    out[4] = 0.0;
    return true;
  }
  return false;
}

// 지정된 피치와 파지 방향에 대해 엘보 업·다운 중 최적 역기구학 해 선택
bool SolvePadIk(const Vec3& pos, double pitch, double grip_yaw,
                const Joint5& ref, Joint5& out) {
  const double l2 = std::hypot(kL2X, kL2Z);
  const double alpha = std::atan2(kL2X, kL2Z);
  const double dx = pos[0] - kJ1X;
  const double dy = pos[1];
  const double yaw = std::atan2(dy, dx);
  const double rad = std::hypot(dx, dy);

  // 손목 관절 중심의 반경·높이 거리 계산
  const double rw = rad - kL4Pad * std::cos(pitch);
  const double zw = (pos[2] - kJ2Z) + kL4Pad * std::sin(pitch);
  const double dist = std::hypot(rw, zw);
  if (dist > l2 + kL3 - 1e-4 || dist < std::abs(l2 - kL3) + 1e-4) {
    return false;
  }

  // 코사인 법칙으로 엘보 두 분기 풀이 및 기준 자세 최근접 해 선택
  const double cos_b =
      std::clamp((dist * dist + l2 * l2 - kL3 * kL3) / (2.0 * dist * l2),
                 -1.0, 1.0);
  const double theta = std::atan2(rw, zw);
  const double raw_roll = WrapSym(WrapPi(yaw - grip_yaw), ref[4], kPi);

  bool found = false;
  double best_cost = 1e9;
  for (double sign : {-1.0, 1.0}) {
    const double phi = theta + sign * std::acos(cos_b);
    const double sh = WrapPi(phi - alpha);
    const double r2 = l2 * std::sin(phi);
    const double z2 = l2 * std::cos(phi);
    const double sh_el = std::atan2(-(zw - z2), rw - r2);
    const double el = WrapPi(sh_el - sh);
    const double wr = WrapPi(pitch - (sh + el));
    const Joint5 cand = {yaw, sh, el, wr, raw_roll};
    if (!InLimits(cand)) continue;

    const double cost = JointDist(cand, ref) + ((sign < 0.0) ? 0.0 : 0.08);
    if (cost < best_cost) {
      best_cost = cost;
      out = cand;
      found = true;
    }
  }
  return found;
}

// 수직(90도)을 우선하되 원거리·후방 슬롯 호버에 맞춰 피치를 자동 탐색
bool SolveHoverIk(const Vec3& pos, double grip_yaw, const Joint5& ref,
                  Joint5& out, bool high_slot) {
  // 후방 외곽 바스켓 호버는 본체 구체 충돌 회피를 위해 완만한 피치 우선 적용
  if (high_slot && pos[0] < -0.04) {
    for (double p = 1.05; p <= 1.45; p += 0.03) {
      if (SolvePadIk(pos, p, grip_yaw, ref, out)) return true;
    }
  }

  // 일반 파지·호버는 수직 하향(pi/2)부터 손목 가동 한계 내에서 순차 탐색
  for (double p = 0.5 * kPi; p >= 0.72; p -= 0.02) {
    if (SolvePadIk(pos, p, grip_yaw, ref, out)) return true;
  }
  for (double p = 0.5 * kPi + 0.02; p <= 1.68; p += 0.02) {
    if (SolvePadIk(pos, p, grip_yaw, ref, out)) return true;
  }
  return false;
}

// 작업공간 직선 보간 궤적을 관절 시퀀스로 변환
bool MakeCartLine(const Vec3& start_pos, double start_pitch,
                  const Vec3& end_pos, double end_pitch, double grip_yaw,
                  const Joint5& seed, int steps, std::vector<Joint5>& path) {
  path.clear();
  const int count = std::max(2, steps);
  Joint5 prev = seed;

  for (int i = 1; i <= count; ++i) {
    const double lin = static_cast<double>(i) / static_cast<double>(count);
    const double ratio = 0.5 * (1.0 - std::cos(kPi * lin));
    const Vec3 pt = {start_pos[0] + ratio * (end_pos[0] - start_pos[0]),
                     start_pos[1] + ratio * (end_pos[1] - start_pos[1]),
                     start_pos[2] + ratio * (end_pos[2] - start_pos[2])};
    const double pitch = start_pitch + ratio * (end_pitch - start_pitch);

    Joint5 next = {};
    if (!SolvePadIk(pt, pitch, grip_yaw, prev, next) &&
        !SolveHoverIk(pt, grip_yaw, prev, next, false)) {
      return false;
    }
    path.push_back(next);
    prev = next;
  }
  return true;
}

}  // namespace xsting_pick_place
