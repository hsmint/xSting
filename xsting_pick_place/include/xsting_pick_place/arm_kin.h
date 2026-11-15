#ifndef XSTING_PICK_PLACE_ARM_KIN_H_
#define XSTING_PICK_PLACE_ARM_KIN_H_

#include <array>
#include <vector>

namespace xsting_pick_place {

// 5자유도 팔 관절 배열 및 3차원 좌표 별칭
using Joint5 = std::array<double, 5>;
using Vec3 = std::array<double, 3>;

// xsting_description·xsting_hardware 기준 팔 관절 이름 및 접힌 홈·관측 자세 상수
inline constexpr std::array<const char*, 5> kArmJoints = {
    "arm_yaw", "arm_shoulder", "arm_elbow", "arm_wrist", "arm_roll"};
inline constexpr Joint5 kHomeJoints = {0.0, -1.0508, 1.0707, 0.0, 0.0};
inline constexpr Joint5 kZeroJoints = {0.0, 0.0, 0.0, 0.0, 0.0};
inline constexpr Joint5 kObsJoints = {0.0, -0.45, 0.75, 0.85, 0.0};

// 각도를 [-pi, pi] 범위로 정규화
double WrapPi(double rad);

// 대칭 주기(pi 또는 pi/2) 기준 현재 롤과 가장 가까운 각도로 래핑
double WrapSym(double target, double ref, double period);

// 5관절 각도로 그리퍼 패드 중심의 base 기준 좌표 계산
Vec3 ForwardPad(const Joint5& joints);

// 5관절 각도로 팔 ArduCam 광학 프레임(arducam_frame)의 base 위치·3축 계산
void ForwardArducam(const Joint5& joints, Vec3& cam_pos,
                    std::array<Vec3, 3>& cam_axes);

// 지정 XY 상공에서 ArduCam 광축이 수직 하향(-Z)을 향하는 관측 역기구학 풀이
bool SolveArducamIk(const Vec3& cam_target, const Joint5& ref, Joint5& out);

// 단일 피치 각도에 대해 패드 중심 목표의 5축 역기구학 풀이
bool SolvePadIk(const Vec3& pos, double pitch, double grip_yaw,
                const Joint5& ref, Joint5& out);

// 수직 파지·호버 목표에 대해 도달 가능한 최적 피치를 탐색하여 역기구학 풀이
bool SolveHoverIk(const Vec3& pos, double grip_yaw, const Joint5& ref,
                  Joint5& out, bool high_slot = false);

// 두 작업공간 포즈 사이를 직선 보간한 관절 웨이포인트 목록 생성
bool MakeCartLine(const Vec3& start_pos, double start_pitch,
                  const Vec3& end_pos, double end_pitch, double grip_yaw,
                  const Joint5& seed, int steps, std::vector<Joint5>& path);

}  // namespace xsting_pick_place

#endif  // XSTING_PICK_PLACE_ARM_KIN_H_
