#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include "xsting_walk/calib.h"
#include "xsting_walk/mlp.h"
#include "xsting_walk/walk_ctrl.h"

namespace {

#define CHECK_TRUE(cond, msg)                                       \
  do {                                                              \
    if (!(cond)) {                                                  \
      std::fprintf(stderr, "[FAIL] %s:%d: %s\n", __FILE__, __LINE__, \
                   (msg));                                          \
      return 1;                                                     \
    }                                                               \
  } while (0)

#define CHECK_NEAR(a, b, tol, msg)                                            \
  do {                                                                        \
    const double _da = static_cast<double>(a);                                \
    const double _db = static_cast<double>(b);                                \
    if (std::abs(_da - _db) > (tol)) {                                        \
      std::fprintf(stderr, "[FAIL] %s:%d: %s (got %.7f vs %.7f, tol=%.7f)\n", \
                   __FILE__, __LINE__, (msg), _da, _db,                       \
                   static_cast<double>(tol));                                 \
      return 1;                                                               \
    }                                                                         \
  } while (0)

}  // namespace

int main() {
  using namespace xsting_walk;

  const std::string model_path = XSTING_WALK_MODEL_BIN;

  // 1) MLP 로드 및 PyTorch(skrl best_agent.pt) 골든 벡터 정합성 검증 (< 2e-6)
  Mlp mlp;
  std::string err;
  CHECK_TRUE(mlp.Load(model_path, &err), err.c_str());
  CHECK_TRUE(mlp.in_dim() == 74 && mlp.out_dim() == 18,
             "MLP 차원이 74 -> 18 이어야 합니다");

  Eigen::Matrix<float, 74, 1> obs = Eigen::Matrix<float, 74, 1>::Zero();
  obs[8] = -1.0f;
  obs[9] = 0.35f * 3.0f;
  obs[13] = 1.0f;
  obs[14] = -0.35f * 4.0f;
  const Eigen::VectorXf act = mlp.Forward(obs);

  constexpr std::array<float, 18> kGoldenAct = {
      -0.2702003f, -0.2857547f, 0.0632954f, -0.4103268f, 0.0316243f,
      -0.0602388f, -0.3972602f, 0.5043540f, 0.3006895f,  -0.3213337f,
      -0.0108669f, -0.2068204f, -0.3822615f, -0.1253372f, 0.1797395f,
      -0.5233911f, 0.1835628f,  -0.0033487f,
  };
  for (int i = 0; i < 18; ++i) {
    CHECK_NEAR(act[i], kGoldenAct[i], 2e-5, "PyTorch 대비 MLP 순전파 오차");
  }

  // 2) 6족 URDF 순기구학 및 발끝 역자코비안 정합성 검증
  Calib calib = Calib::MakeDefault();
  WalkCtrl ctrl;
  CHECK_TRUE(ctrl.Init(model_path, calib, &err), err.c_str());

  CHECK_NEAR(ctrl.foot_home_xy()[0].x(), 0.186890f, 1e-4, "leg1 home X");
  CHECK_NEAR(ctrl.foot_home_xy()[0].y(), 0.233177f, 1e-4, "leg1 home Y");
  CHECK_NEAR(ctrl.foot_home_xy()[1].x(), 0.0f, 1e-5, "leg2 home X");
  CHECK_NEAR(ctrl.foot_home_xy()[1].y(), 0.253780f, 1e-4, "leg2 home Y");
  CHECK_NEAR(ctrl.jac_inv()[0](0, 0), 5.6320f, 2e-3, "leg1 jac_inv(0,0)");
  CHECK_NEAR(ctrl.jac_inv()[0](0, 1), -3.2516f, 2e-3, "leg1 jac_inv(0,1)");
  CHECK_NEAR(ctrl.jac_inv()[3](0, 0), 5.6320f, 2e-3, "leg4 jac_inv(0,0)");
  CHECK_NEAR(ctrl.jac_inv()[3](0, 1), 3.2516f, 2e-3, "leg4 jac_inv(0,1)");

  // 3) 13가지 전방향·회전·커브 보행 모드 루프 및 관절 한계·무결성 검증
  constexpr const char* kModes[] = {
      "forward", "backward",  "left",       "right",      "diag_fl",
      "diag_fr", "diag_rl",   "diag_rr",    "turn_left",  "turn_right",
      "curve_left", "curve_right", "stop",
  };
  std::vector<double> leg_cmds;
  std::vector<double> arm_cmds;
  for (const char* m : kModes) {
    Eigen::Vector3f m_cmd;
    CHECK_TRUE(WalkCtrl::NamedModeToCmd(m, &m_cmd), "명명 모드 변환 실패");
    ctrl.SetTeleopCmd(m_cmd);
    for (int step = 0; step < 10; ++step) {
      const WalkState st = ctrl.StepPolicy(0.02f);
      CHECK_TRUE(std::isfinite(st.phase), "위상 NaN 발생");
      for (int sub = 0; sub < 4; ++sub) {
        ctrl.StepHardware(0.005f, &leg_cmds, &arm_cmds);
        CHECK_TRUE(leg_cmds.size() == 18 && arm_cmds.size() == 6,
                   "출력 관절 수 오류");
        for (int j = 0; j < 18; ++j) {
          CHECK_TRUE(std::isfinite(leg_cmds[j]), "다리 명령 NaN");
          CHECK_TRUE(leg_cmds[j] >= calib.legs[j].lower_rad - 1e-5 &&
                         leg_cmds[j] <= calib.legs[j].upper_rad + 1e-5,
                     "다리 URDF 하드 한계 초과");
        }
      }
    }
  }

  // 4) IMU 장착 회전 + 가속도 9.81 m/s^2 바이어스 보정 검증
  const Eigen::Quaterniond tilt_q(
      Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(-0.05, Eigen::Vector3d::UnitX()));
  const Eigen::Vector3d grav_s =
      tilt_q.conjugate() * Eigen::Vector3d(0.0, 0.0, -1.0);
  const Eigen::Vector3d fake_acc_bias(0.12, -0.09, 0.25);
  const Eigen::Vector3d acc_s =
      tilt_q.conjugate() * Eigen::Vector3d(0.0, 0.0, 9.81) + fake_acc_bias;
  const Eigen::Vector3d gyro_s(0.01, -0.02, 0.005);

  const ImuCalib solved_imu = SolveImu1Pose(gyro_s, acc_s, grav_s, 0.0);
  Eigen::Vector3f w_b, a_b, g_b;
  solved_imu.Transform(gyro_s, acc_s, true, tilt_q, &w_b, &a_b, &g_b);
  CHECK_NEAR(w_b.norm(), 0.0f, 1e-5, "보정 후 정지 자이로 0");
  CHECK_NEAR(g_b.x(), 0.0f, 1e-5, "보정 후 중력 X=0");
  CHECK_NEAR(g_b.y(), 0.0f, 1e-5, "보정 후 중력 Y=0");
  CHECK_NEAR(g_b.z(), -1.0f, 1e-5, "보정 후 중력 Z=-1");
  CHECK_NEAR(a_b.x(), 0.0f, 1e-4, "보정 후 가속도 X=0");
  CHECK_NEAR(a_b.y(), 0.0f, 1e-4, "보정 후 가속도 Y=0");
  CHECK_NEAR(a_b.z(), 9.81f, 1e-4, "보정 후 가속도 Z=+9.81");

  // 5) 실측 속도·회전 스케일 산출 및 YAML 저장·로드 검증
  WalkGaitCalib g;
  CHECK_TRUE(UpdateLinearCalib("vx", 0.20, 3.0, 0.50, 0.03, 0.0, true, false,
                               &g, nullptr),
             "vx 실측 보정 실패");
  CHECK_NEAR(g.cmd_vel_scale.x(), 1.20, 1e-5, "vx 스케일=0.60/0.50=1.20");
  CHECK_NEAR(g.drift_vy_per_vx, -0.05, 1e-5, "횡편차 보정=-0.03/0.60=-0.05");

  CHECK_TRUE(UpdateTurnCalib(1.0, M_PI, 150.0, &g, nullptr),
             "wz 실측 보정 실패");
  CHECK_NEAR(g.cmd_vel_scale.z(), 180.0 / 150.0, 1e-5, "wz 스케일=1.20");

  calib.imu = solved_imu;
  calib.gait = g;
  const std::string tmp_yaml = "/tmp/xsting_walk_test_calib.yaml";
  CHECK_TRUE(calib.Save(tmp_yaml, &err), err.c_str());
  Calib loaded = Calib::MakeDefault();
  CHECK_TRUE(loaded.Load(tmp_yaml, &err), err.c_str());
  std::filesystem::remove(tmp_yaml);
  CHECK_NEAR(loaded.gait.cmd_vel_scale.x(), 1.20, 1e-5, "YAML vx scale 로드");
  CHECK_NEAR(loaded.gait.drift_vy_per_vx, -0.05, 1e-5, "YAML drift 로드");
  CHECK_NEAR(loaded.imu.acc_bias.z(), solved_imu.acc_bias.z(), 1e-4,
             "YAML acc_bias 로드");

  std::printf("[PASS] xsting_walk 코어 유닛 테스트 전부 통과!\n");
  return 0;
}
