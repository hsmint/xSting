#ifndef XSTING_PICK_PLACE_MISSION_RULES_H_
#define XSTING_PICK_PLACE_MISSION_RULES_H_

#include <array>
#include <string>
#include <utility>
#include <vector>

namespace xsting_pick_place {

// 상자(205x205x70) 및 퍽(30x30x60) 물리 치수 상수(단위: m)
inline constexpr std::array<double, 3> kBoxSize = {0.205, 0.205, 0.070};
inline constexpr std::array<double, 3> kPuckSize = {0.030, 0.030, 0.060};
inline constexpr double kPuckRadius = 0.015;
inline constexpr double kPuckHeight = 0.060;

// 바스켓 슬롯 바닥 좌표 및 회전각 구조체
struct SlotPose {
  std::array<double, 3> pos;
  double yaw;
};

// 퍽 식별자·색상·모양 정보 구조체
struct PuckAttr {
  int id;
  char color;
  bool is_square;
};

// 전체 할당 결과 구조체
struct Allocation {
  std::vector<int> r_ids;
  std::vector<int> y_ids;
  std::vector<int> g_ids;
};

// 슬롯 번호(1~12)의 base 기준 바닥 좌표와 회전각 반환
SlotPose GetSlotPose(int slot);

// 주사위 눈(1~6) 또는 기본 2개 배치(0)의 상자 중심 기준 오프셋 반환
std::vector<std::pair<double, double>> GetBoxOffsets(int dice_eye);

// 바코드 문자열(P2-01~04)을 오름차순 여부로 변환
bool ParseBarcode(const std::string& code, bool& is_asc);

// 12회 신호 시퀀스를 검증하고 R·Y·G 횟수 산출
bool CountSignals(const std::string& seq, std::array<int, 3>& counts);

// 신호 횟수와 정렬 방향에 따라 L1·L2·L3 그룹 문자 반환
std::array<char, 3> AssignGroups(const std::array<int, 3>& counts, bool is_asc);

// Y 모양 통일 규칙을 만족하는 12개 퍽 전역 할당 탐색
bool AllocatePucks(const std::vector<PuckAttr>& pucks,
                   const std::array<int, 3>& counts, Allocation& out);

}  // namespace xsting_pick_place

#endif  // XSTING_PICK_PLACE_MISSION_RULES_H_
