#include <cmath>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "xsting_pick_place/arm_kin.h"
#include "xsting_pick_place/mission_rules.h"

namespace {

void Check(bool cond, const std::string& msg) {
  if (!cond) throw std::runtime_error("TEST FAILED: " + msg);
}

void CheckNear(double a, double b, double tol, const std::string& msg) {
  if (std::abs(a - b) > tol) {
    throw std::runtime_error("TEST FAILED: " + msg + " got=" +
                             std::to_string(a) + " expected=" +
                             std::to_string(b));
  }
}

}  // namespace

int main() {
  using namespace xsting_pick_place;
  try {
    // 1. 바코드 파싱 검증 (P2-01/03 오름차순, P2-02/04 내림차순)
    bool asc = false;
    Check(ParseBarcode("P2-01", asc) && asc, "P2-01 asc");
    Check(ParseBarcode("P2-03", asc) && asc, "P2-03 asc");
    Check(ParseBarcode("P2-02", asc) && !asc, "P2-02 desc");
    Check(ParseBarcode("P2-04", asc) && !asc, "P2-04 desc");
    Check(!ParseBarcode("P2-99", asc), "Invalid barcode rejected");

    // 2. 신호등 12회 집계 및 R->Y->G 동률 우선순위 검증
    std::array<int, 3> counts = {};
    Check(CountSignals("RGRRGRGGYGYG", counts), "Q1 signal valid");
    Check(counts[0] == 4 && counts[1] == 2 && counts[2] == 6, "Q1 counts 4,2,6");
    Check(CountSignals("RGRRGRRGYGYG", counts), "Tie signal valid");
    Check(counts[0] == 5 && counts[1] == 2 && counts[2] == 5, "Tie counts 5,2,5");
    Check(!CountSignals("RRRRRRRYYYYG", counts), "Count > 6 rejected");
    Check(!CountSignals("RRRRYYYY", counts), "Length != 12 rejected");

    const auto grp_asc = AssignGroups({5, 2, 5}, true);
    Check(grp_asc[0] == 'Y' && grp_asc[1] == 'R' && grp_asc[2] == 'G',
          "Asc tie break Y, R, G");
    const auto grp_desc = AssignGroups({4, 4, 4}, false);
    Check(grp_desc[0] == 'R' && grp_desc[1] == 'Y' && grp_desc[2] == 'G',
          "Desc 3-way tie keeps R, Y, G");

    // 3. Challenge B 할당 검증 (유효 4건 + 불가능 2건)
    const std::vector<PuckAttr> pucks_std = {
        {0, 'R', false}, {1, 'R', false}, {2, 'R', false},
        {3, 'R', true},  {4, 'R', true},  {5, 'R', true},
        {6, 'G', false}, {7, 'G', false}, {8, 'G', false},
        {9, 'G', true},  {10, 'G', true}, {11, 'G', true},
    };
    for (const auto& c : std::vector<std::array<int, 3>>{
             {5, 2, 5}, {4, 4, 4}, {3, 6, 3}, {3, 4, 5}}) {
      Allocation alloc;
      Check(AllocatePucks(pucks_std, c, alloc), "Valid allocation case");
      Check(static_cast<int>(alloc.r_ids.size()) == c[0], "R count matches");
      Check(static_cast<int>(alloc.y_ids.size()) == c[1], "Y count matches");
      Check(static_cast<int>(alloc.g_ids.size()) == c[2], "G count matches");

      std::set<int> uniq;
      uniq.insert(alloc.r_ids.begin(), alloc.r_ids.end());
      uniq.insert(alloc.y_ids.begin(), alloc.y_ids.end());
      uniq.insert(alloc.g_ids.begin(), alloc.g_ids.end());
      Check(uniq.size() == 12, "All 12 pucks assigned without duplicates");

      const bool y_sq = pucks_std[static_cast<size_t>(alloc.y_ids.front())].is_square;
      for (int yid : alloc.y_ids) {
        Check(pucks_std[static_cast<size_t>(yid)].is_square == y_sq,
              "Y group shape unified");
      }
    }

    // 불가능 1: 빨강 6개 필요한데 빨강 퍽이 5개뿐
    std::vector<PuckAttr> pucks_5r = pucks_std;
    pucks_5r[5].color = 'G';
    Allocation bad_alloc;
    Check(!AllocatePucks(pucks_5r, {6, 2, 4}, bad_alloc),
          "Impossible color shortage rejected");

    // 불가능 2: 빨강은 전부 원기둥(6), 초록은 전부 사각(6)인데 R=4, Y=4, G=4 요청
    std::vector<PuckAttr> pucks_split;
    for (int i = 0; i < 6; ++i) pucks_split.push_back({i, 'R', false});
    for (int i = 6; i < 12; ++i) pucks_split.push_back({i, 'G', true});
    Check(!AllocatePucks(pucks_split, {4, 4, 4}, bad_alloc),
          "Impossible Y shape unification rejected");

    // 4. 주사위 눈 격자(1~6) 개수 검증
    for (int eye = 1; eye <= 6; ++eye) {
      Check(static_cast<int>(GetBoxOffsets(eye).size()) == eye,
            "Dice eye offset count " + std::to_string(eye));
    }

    // 5. 5축 매니퓰레이터 그리퍼 IK 및 ArduCam 수직 하향 탑뷰 IK 왕복 정합성 검증
    Joint5 q_hover = {};
    const Vec3 target_pad = {0.265, -0.025, 0.165};
    Check(SolveHoverIk(target_pad, 0.15, kHomeJoints, q_hover, false),
          "SolveHoverIk reachable");
    const Vec3 fk_pad = ForwardPad(q_hover);
    CheckNear(fk_pad[0], target_pad[0], 1e-4, "Hover FK X");
    CheckNear(fk_pad[1], target_pad[1], 1e-4, "Hover FK Y");
    CheckNear(fk_pad[2], target_pad[2], 1e-4, "Hover FK Z");

    Joint5 q_cam = {};
    const Vec3 target_cam = {0.285, 0.0, 0.3487};
    Check(SolveArducamIk(target_cam, kHomeJoints, q_cam),
          "SolveArducamIk top-view reachable");
    Vec3 cam_pos = {};
    std::array<Vec3, 3> cam_axes = {};
    ForwardArducam(q_cam, cam_pos, cam_axes);
    CheckNear(cam_pos[0], target_cam[0], 1e-3, "ArduCam FK X");
    CheckNear(cam_pos[1], target_cam[1], 1e-3, "ArduCam FK Y");
    CheckNear(cam_pos[2], target_cam[2], 1e-3, "ArduCam FK Z");
    CheckNear(cam_axes[2][2], -1.0, 1e-3, "ArduCam optical +Z points straight down (-Z)");

    std::cout << "[PASS] All xsting_pick_place mission rules and kinematics tests passed.\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
