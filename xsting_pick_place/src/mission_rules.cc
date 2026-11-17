#include "xsting_pick_place/mission_rules.h"

#include <algorithm>

namespace xsting_pick_place {

namespace {

// xsting_description 기준 12칸 바스켓 슬롯 실제 지지 바닥 좌표(base 기준)
constexpr std::array<SlotPose, 12> kSlots = {{
    {{0.0609, 0.0767, 0.163722}, 2.12151},
    {{0.0231, 0.0300, 0.162722}, 2.80193},
    {{0.0231, -0.0300, 0.162722}, -2.80193},
    {{0.0609, -0.0767, 0.163722}, -2.12151},
    {{-0.0112, 0.0771, 0.172722}, 2.56746},
    {{-0.0314, 0.0270, 0.172722}, 2.95027},
    {{-0.0314, -0.0270, 0.172722}, -2.95027},
    {{-0.0112, -0.0771, 0.172722}, -2.56746},
    {{-0.0660, 0.0788, 0.184722}, 2.71635},
    {{-0.0811, 0.0270, 0.184722}, 2.99977},
    {{-0.0811, -0.0270, 0.184722}, -2.99977},
    {{-0.0660, -0.0788, 0.184722}, -2.71635},
}};

}  // namespace

// 슬롯 번호(1~12)를 인덱스로 클램프하여 슬롯 자세 반환
SlotPose GetSlotPose(int slot) {
  const int idx = std::clamp(slot, 1, 12) - 1;
  return kSlots[static_cast<size_t>(idx)];
}

// mission.pdf 주사위 눈(1~6) 3x3 격자 및 시작 상자 2개 기본 배치 좌표 반환
std::vector<std::pair<double, double>> GetBoxOffsets(int dice_eye) {
  switch (dice_eye) {
    case 1:
      return {{0.00, 0.00}};
    case 2:
      return {{-0.06, 0.06}, {0.06, -0.06}};
    case 3:
      return {{-0.06, 0.06}, {0.00, 0.00}, {0.06, -0.06}};
    case 4:
      return {{-0.06, 0.06}, {0.06, 0.06}, {-0.06, -0.06}, {0.06, -0.06}};
    case 5:
      return {{-0.06, 0.06},
              {0.06, 0.06},
              {0.00, 0.00},
              {-0.06, -0.06},
              {0.06, -0.06}};
    case 6:
      return {{-0.06, 0.06}, {-0.06, 0.00}, {-0.06, -0.06},
              {0.06, 0.06},  {0.06, 0.00},  {0.06, -0.06}};
    default:
      return {{-0.04, 0.00}, {0.04, 0.00}};
  }
}

// 바코드 문자열 유효성 확인 및 정렬 방향 파싱
bool ParseBarcode(const std::string& code, bool& is_asc) {
  if (code == "P2-01" || code == "P2-03") {
    is_asc = true;
    return true;
  }
  if (code == "P2-02" || code == "P2-04") {
    is_asc = false;
    return true;
  }
  return false;
}

// 12회 점멸 문자열에서 R·Y·G 횟수와 1~6 범위 조건 검사
bool CountSignals(const std::string& seq, std::array<int, 3>& counts) {
  if (seq.size() != 12) return false;
  counts = {0, 0, 0};

  for (char ch : seq) {
    if (ch == 'R') {
      ++counts[0];
    } else if (ch == 'Y') {
      ++counts[1];
    } else if (ch == 'G') {
      ++counts[2];
    } else {
      return false;
    }
  }

  for (int val : counts) {
    if (val < 1 || val > 6) return false;
  }
  return true;
}

// 동률 시 R->Y->G 순서를 유지하며 L1·L2·L3에 그룹 배정
std::array<char, 3> AssignGroups(const std::array<int, 3>& counts,
                                 bool is_asc) {
  constexpr std::array<char, 3> kNames = {'R', 'Y', 'G'};
  std::array<int, 3> order = {0, 1, 2};

  std::sort(order.begin(), order.end(), [&](int a, int b) {
    const int ca = is_asc ? counts[static_cast<size_t>(a)]
                          : -counts[static_cast<size_t>(a)];
    const int cb = is_asc ? counts[static_cast<size_t>(b)]
                          : -counts[static_cast<size_t>(b)];
    return (ca != cb) ? (ca < cb) : (a < b);
  });

  return {kNames[static_cast<size_t>(order[0])],
          kNames[static_cast<size_t>(order[1])],
          kNames[static_cast<size_t>(order[2])]};
}

// Y 그룹을 단일 모양으로 통일하면서 R·G 개수를 모두 만족하는 조합 탐색
bool AllocatePucks(const std::vector<PuckAttr>& pucks,
                   const std::array<int, 3>& counts, Allocation& out) {
  if (pucks.size() != 12) return false;

  // 원기둥(false)과 사각(true) 순서로 Y 그룹 모양 후보 탐색
  for (bool y_sq : {false, true}) {
    std::vector<int> pool;
    for (size_t i = 0; i < pucks.size(); ++i) {
      if (pucks[i].is_square == y_sq) pool.push_back(static_cast<int>(i));
    }
    if (static_cast<int>(pool.size()) < counts[1]) continue;

    std::vector<bool> mask(pool.size(), false);
    std::fill(mask.begin(), mask.begin() + counts[1], true);

    do {
      std::vector<bool> used(pucks.size(), false);
      Allocation cand;
      for (size_t i = 0; i < pool.size(); ++i) {
        if (mask[i]) {
          const int idx = pool[i];
          used[static_cast<size_t>(idx)] = true;
          cand.y_ids.push_back(pucks[static_cast<size_t>(idx)].id);
        }
      }

      for (size_t i = 0; i < pucks.size(); ++i) {
        if (used[i]) continue;
        if (pucks[i].color == 'R') cand.r_ids.push_back(pucks[i].id);
        if (pucks[i].color == 'G') cand.g_ids.push_back(pucks[i].id);
      }

      if (static_cast<int>(cand.r_ids.size()) == counts[0] &&
          static_cast<int>(cand.g_ids.size()) == counts[2]) {
        out = std::move(cand);
        return true;
      }
    } while (std::prev_permutation(mask.begin(), mask.end()));
  }
  return false;
}

}  // namespace xsting_pick_place
