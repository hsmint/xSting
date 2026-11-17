#ifndef XSTING_PICK_PLACE_PUCK_DET_H_
#define XSTING_PICK_PLACE_PUCK_DET_H_

#include <string>
#include <vector>

#include <vision_msgs/msg/detection3_d_array.hpp>

#include "xsting_pick_place/arm_kin.h"

namespace xsting_pick_place {

// 검출된 단일 퍽 상태 구조체
struct Puck {
  Vec3 pos;
  double yaw;
  bool is_square;
  bool is_lying;
  std::string cls_id;
  double score = 0.90;
};

// 3D 검출 결과에서 상자 중심 XY(선택)와 정렬된 퍽 목록 추출
std::vector<Puck> FindBoxPucks(const vision_msgs::msg::Detection3DArray& msg,
                               const std::string& filter_cls, double box_top_z,
                               double& box_x, double& box_y, bool update_box);

// 주변 퍽 간섭을 피하도록 직립 사각·원형 퍽의 파지 Yaw 보정
double SelectGraspYaw(const Puck& target, const Puck* other);

// 상자 위 다른 퍽과 충돌하지 않는 기립 작업 좌표(x, y) 계산
void SelectRightSpot(double box_x, double box_y, const Puck& target,
                     const Puck* other, double& rx, double& ry);

}  // namespace xsting_pick_place

#endif  // XSTING_PICK_PLACE_PUCK_DET_H_
