#include "xsting_pick_place/puck_det.h"

#include <algorithm>
#include <cmath>

namespace xsting_pick_place {

namespace {

constexpr double kPi = 3.14159265358979323846;

// 쿼터니언으로부터 로컬 X·Y·Z 단위축 계산
void QuatAxes(const geometry_msgs::msg::Quaternion& q, Vec3& ax, Vec3& ay,
              Vec3& az) {
  const double xx = q.x * q.x;
  const double yy = q.y * q.y;
  const double zz = q.z * q.z;
  const double xy = q.x * q.y;
  const double xz = q.x * q.z;
  const double yz = q.y * q.z;
  const double wx = q.w * q.x;
  const double wy = q.w * q.y;
  const double wz = q.w * q.z;

  ax = {1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz), 2.0 * (xz - wy)};
  ay = {2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx)};
  az = {2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy)};
}

}  // namespace

// Detection3DArray에서 종이 상자와 작업 반경 내 모든 유효 퍽을 추출하고 우선순위 정렬
std::vector<Puck> FindBoxPucks(const vision_msgs::msg::Detection3DArray& msg,
                               const std::string& filter_cls, double box_top_z,
                               double& box_x, double& box_y, bool update_box) {
  std::vector<Puck> pucks;

  // 종이 상자 위치 갱신 및 퍽 후보 파싱
  for (const auto& det : msg.detections) {
    if (det.results.empty()) continue;
    const std::string& cls_id = det.results[0].hypothesis.class_id;
    const auto& pt = det.bbox.center.position;

    if (cls_id.find("box") != std::string::npos) {
      if (update_box && pt.x > 0.18 && pt.x < 0.40 && std::abs(pt.y) < 0.15) {
        box_x = pt.x;
        box_y = pt.y;
      }
      continue;
    }

    if (!filter_cls.empty() && cls_id.find(filter_cls) == std::string::npos) {
      continue;
    }
    if (pt.x < 0.15 || std::hypot(pt.x, pt.y) > 0.43) continue;

    // OBB 최장축(60mm 방향)과 측면축 추출
    Vec3 ax = {};
    Vec3 ay = {};
    Vec3 az = {};
    QuatAxes(det.bbox.center.orientation, ax, ay, az);
    const auto& sz = det.bbox.size;
    Vec3 long_ax = ax;
    Vec3 side_ax = ay;
    if (sz.z > sz.x && sz.z >= sz.y) {
      long_ax = az;
      side_ax = ax;
    } else if (sz.y > sz.x && sz.y > sz.z) {
      long_ax = ay;
      side_ax = ax;
    }

    // 원형 퍽은 항상 직립, 사각 퍽은 장축 Z성분 및 높이로 눕힘 여부 판별
    const bool is_sq = (cls_id.find("cylinder") == std::string::npos);
    const bool lying =
        is_sq && (std::abs(long_ax[2]) < 0.58 || pt.z < box_top_z + 0.023);

    double yaw = 0.0;
    if (lying) {
      yaw = std::atan2(long_ax[1], long_ax[0]);
      if (yaw > 0.5 * kPi) yaw -= kPi;
      if (yaw < -0.5 * kPi) yaw += kPi;
    } else if (is_sq) {
      const double raw = std::atan2(side_ax[1], side_ax[0]);
      yaw = 0.25 * std::atan2(std::sin(4.0 * raw), std::cos(4.0 * raw));
    }

    const double raw_score = det.results[0].hypothesis.score;
    const double score =
        (raw_score > 0.0 && raw_score <= 1.0) ? raw_score : 0.90;
    pucks.push_back({{pt.x, pt.y, pt.z}, yaw, is_sq, lying, cls_id, score});
  }

  // 세워진 퍽을 먼저 집어 상자 공간을 확보하고 거리순으로 정렬
  std::sort(pucks.begin(), pucks.end(), [](const Puck& a, const Puck& b) {
    if (a.is_lying != b.is_lying) return !a.is_lying;
    return std::hypot(a.pos[0] - 0.108, a.pos[1]) <
           std::hypot(b.pos[0] - 0.108, b.pos[1]);
  });
  return pucks;
}

// 인접한 두 번째 퍽과 손가락 개방 방향이 겹치지 않도록 파지 각도 선택
double SelectGraspYaw(const Puck& target, const Puck* other) {
  if (target.is_lying) return target.yaw;
  const double arm_yaw = std::atan2(target.pos[1], target.pos[0] - 0.108);
  if (!other) return target.is_square ? target.yaw : arm_yaw;

  const double dx = other->pos[0] - target.pos[0];
  const double dy = other->pos[1] - target.pos[1];
  const double dist = std::hypot(dx, dy);
  if (dist > 0.080 || dist < 1e-4) {
    return target.is_square ? target.yaw : arm_yaw;
  }

  // 원형 퍽은 두 퍽 연결선 방향으로 손가락 몸체를 정렬하여 간섭 회피
  const double link_yaw = std::atan2(dy, dx);
  if (!target.is_square) return link_yaw;

  // 사각 퍽은 90도 대칭 두 후보 중 인접 퍽과 손가락 충돌이 적은 각도 선택
  const double ux = dx / dist;
  const double uy = dy / dist;
  const double cand1 = target.yaw;
  const double cand2 = WrapPi(target.yaw + 0.5 * kPi);
  const double dot1 = std::abs(-std::sin(cand1) * ux + std::cos(cand1) * uy);
  const double dot2 = std::abs(-std::sin(cand2) * ux + std::cos(cand2) * uy);
  return (dot2 + 0.10 < dot1) ? cand2 : cand1;
}

// 205x205 상자 중앙부에서 인접 퍽과 간섭 없이 파지내 회전 기립할 지점 산출
void SelectRightSpot(double box_x, double box_y, const Puck& target,
                     const Puck* other, double& rx, double& ry) {
  rx = std::clamp(box_x + 0.005, 0.275, 0.305);
  ry = std::clamp(target.pos[1] * 0.5, box_y - 0.045, box_y + 0.045);
  if (!other) return;

  if (other->pos[1] >= box_y) {
    ry = std::clamp(other->pos[1] - 0.070, box_y - 0.055, box_y + 0.020);
  } else {
    ry = std::clamp(other->pos[1] + 0.070, box_y - 0.020, box_y + 0.055);
  }
  if (std::hypot(rx - other->pos[0], ry - other->pos[1]) < 0.065) {
    rx = (other->pos[0] > box_x) ? (box_x - 0.025) : (box_x + 0.025);
  }
}

}  // namespace xsting_pick_place
