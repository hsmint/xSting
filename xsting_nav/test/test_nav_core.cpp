#include "xsting_nav/json_loader.hpp"
#include "xsting_nav/nav_core.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace xsting_nav;

namespace
{
void check_true(bool cond, const std::string & msg)
{
  if (!cond) {
    throw std::runtime_error("TEST FAILED: " + msg);
  }
}

void check_near(float a, float b, float tol, const std::string & msg)
{
  if (std::abs(a - b) > tol) {
    throw std::runtime_error(
      "TEST FAILED: " + msg + " | got=" + std::to_string(a) +
      " expected=" + std::to_string(b) + " diff=" + std::to_string(std::abs(a - b)));
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const std::filesystem::path pkg_dir =
      (argc > 1) ? std::filesystem::path(argv[1]) : std::filesystem::path(XSTING_TEST_PKG_DIR);
    const std::string policy_json = (pkg_dir / "config/mk2_nav_policy.json").string();
    const std::string arena_json = (pkg_dir / "config/arena_static_esdf.json").string();

    // 1. Load exported policy & golden PyTorch parity vectors
    MlpPolicy policy;
    policy.load_json(policy_json);
    check_true(policy.is_loaded(), "Policy loaded");

    const JsonValue root = JsonValue::parse_file(policy_json);
    const auto & gv = root.at("golden_verification");

    // 1a. Verify C++ build_obs_41d matches PyTorch build_obs() to < 1e-5
    EsdfSlice dummy_slice;
    dummy_slice.width = 80;
    dummy_slice.height = 80;
    dummy_slice.origin_x = -1.5f;
    dummy_slice.origin_y = -1.5f;
    dummy_slice.resolution = 0.05f;
    dummy_slice.unknown_value = 1000.0f;
    dummy_slice.data.assign(80 * 80, 0.75f);
    for (int iy = 42; iy < 48; ++iy) {
      for (int ix = 38; ix < 43; ++ix) {
        dummy_slice.data[iy * 80 + ix] = 0.02f;
      }
    }

    const Vec3f grav = quat_to_grav({0.998f, 0.03f, -0.04f, 0.02f});
    const Vec6f imu_state{0.04f, -0.02f, 0.10f, grav[0], grav[1], grav[2]};
    const std::vector<Vec2f> box_pts{{0.46f, -0.08f}, {0.46f, 0.00f}, {0.46f, 0.08f}, {0.47f, 0.04f}};
    const std::vector<Vec2f> puck_pts{{0.46f, -0.04f}, {0.46f, 0.00f}, {0.46f, 0.04f}};
    const auto goal_vec = gv.at("obs_case1_inputs").at("goal").as_float_vec();
    const auto init_vec = gv.at("obs_case1_inputs").at("init_pose").as_float_vec();

    const Obs41f cpp_obs1 = build_obs_41d(
      {0.25f, 0.10f, 0.15f},
      {0.18f, -0.03f, 0.12f},
      {goal_vec[0], goal_vec[1], goal_vec[2]},
      dummy_slice, box_pts, puck_pts,
      {init_vec[0], init_vec[1], init_vec[2]},
      imu_state, {0.95f, 0.95f, 0.0f});

    const auto py_obs1 = gv.at("obs_batch")[0].as_float_vec();
    for (size_t i = 0; i < 41; ++i) {
      check_near(cpp_obs1[i], py_obs1[i], 1e-5f, "build_obs_41d dim " + std::to_string(i));
    }

    // 1b. Verify MlpPolicy::act_raw and SafetyShield::apply across all 6 golden batch samples
    const auto & obs_batch = gv.at("obs_batch");
    const auto & raw_acts = gv.at("raw_actions");
    const auto & shield_acts = gv.at("shield_actions");
    float max_act_err = 0.0f;
    for (size_t b = 0; b < obs_batch.size(); ++b) {
      const auto ov = obs_batch[b].as_float_vec();
      const auto rv = raw_acts[b].as_float_vec();
      const auto sv = shield_acts[b].as_float_vec();
      Obs41f ob{};
      for (size_t i = 0; i < 41; ++i) ob[i] = ov[i];

      const Vec3f c_raw = policy.act_raw(ob);
      const Vec3f c_safe = policy.act_shielded(ob);
      for (int k = 0; k < 3; ++k) {
        max_act_err = std::max(max_act_err, std::abs(c_raw[k] - rv[k]));
        check_near(c_raw[k], rv[k], 1e-5f, "act_raw sample " + std::to_string(b));
        check_near(c_safe[k], sv[k], 1e-5f, "act_shielded sample " + std::to_string(b));
      }
    }

    // 1c. Verify 100Hz MK2 LegGaitController IK parity against Python LegGait
    LegGaitController gait(2.2f, 0.030f, 0.042f);
    Leg18f q_step{};
    for (int s = 0; s < 5; ++s) {
      q_step = gait.step_sim({0.24f, -0.06f, 0.35f}, 0.01f);
    }
    const auto py_gait = gv.at("gait_step5_joints").as_float_vec();
    for (size_t i = 0; i < 18; ++i) {
      check_near(q_step[i], py_gait[i], 1e-5f, "LegGait joint " + std::to_string(i));
    }

    // 1d. Verify dock_vel parity + crab-walk + D435 side shield
    const auto py_dock = gv.at("dock_vel_sample").as_float_vec();
    const auto cpp_dock = SafetyShield::dock_vel({1.0f, 0.44f, -0.03f, 0.98f, 0.199f}, 0.315f);
    check_near(cpp_dock.vel_cmd[0], py_dock[0], 1e-5f, "dock_vel vx");
    check_near(cpp_dock.vel_cmd[1], py_dock[1], 1e-5f, "dock_vel vy");
    check_near(cpp_dock.vel_cmd[2], py_dock[2], 1e-5f, "dock_vel wz");

    Obs41f clear_obs{};
    clear_obs.fill(0.8f);
    clear_obs[33] = 0.0f;
    const Vec3f left_crab = SafetyShield::apply(clear_obs, {0.0f, 0.85f, 0.0f});
    const Vec3f right_crab = SafetyShield::apply(clear_obs, {0.0f, -0.85f, 0.0f});
    check_near(left_crab[1], 0.85f, 1e-5f, "Pure left crab-walk vy allowed at vx=0");
    check_near(right_crab[1], -0.85f, 1e-5f, "Pure right crab-walk vy allowed at vx=0");

    Obs41f left_hum_obs = clear_obs;
    left_hum_obs[17 + 1 * 4 + 3] = -0.18f;
    const Vec3f blocked_left = SafetyShield::apply(left_hum_obs, {0.0f, 0.85f, 0.0f});
    check_near(blocked_left[1], 0.0f, 1e-5f, "Left D435 nvBlox guard stops +vy into person");

    // 1e. Verify 360-deg COIN-D4 2D LiDAR & Ultrasonic ESDF fusion
    CalibrationConfig xsting_calib = CalibrationConfig::make_defaults("xsting");
    check_near(xsting_calib.leg_offset_rad[3], 0.0f, 1e-5f, "xsting profile leg2_yaw zero offset");
    check_near(xsting_calib.leg_hw_min_rad[0], Mk2Kinematics::kLegLow[0], 1e-5f, "xsting URDF min limit");
    check_near(xsting_calib.init_pose_arena[0], 0.734f, 1e-5f, "xsting HOME x = 0.734m");
    check_near(xsting_calib.init_pose_arena[1], 0.950f, 1e-5f, "xsting HOME y = 0.950m");

    const std::vector<float> test_ranges{0.50f, 0.40f};
    const auto lidar_xy = project_lidar_scan_to_base_xy(
      test_ranges, 0.0f, static_cast<float>(0.5 * Mk2Kinematics::kPi), xsting_calib, "lidar", false);
    check_true(lidar_xy.size() == 2, "Projected 2 valid 360 LiDAR rays");
    check_near(lidar_xy[0][0], 0.50f, 1e-4f, "LiDAR 0 deg -> +X forward in base");
    check_near(lidar_xy[1][1], -0.40f, 1e-4f, "LiDAR +90 deg with roll=pi -> -Y right in base");

    Obs41f fused_obs = clear_obs;
    fuse_lidar_ultrasonic_into_obs(fused_obs, {{0.36f, 0.0f}}, 0.28f, 1.0f);
    check_true(fused_obs[17 + 2 * 4 + 1] < 0.20f, "360 LiDAR + sonar fused into front 4x4 ESDF cell");
    const Vec3f rev_ok = SafetyShield::apply(fused_obs, {-0.25f, 0.0f, 0.0f});
    check_near(rev_ok[0], -0.25f, 1e-5f, "Rear-clear 360 LiDAR allows controlled reverse avoidance");
    Obs41f rear_blocked_obs = fused_obs;
    rear_blocked_obs[17 + 0 * 4 + 1] = 0.03f;
    const Vec3f rev_stopped = SafetyShield::apply(rear_blocked_obs, {-0.25f, 0.0f, 0.0f});
    check_near(rev_stopped[0], 0.0f, 1e-5f, "360 LiDAR rear guard stops reverse when r_near < 0.05m");

    // 2. Verify 10.0s human-blocking wait + dynamic Dijkstra detour replanning
    DetourPlanner planner;
    planner.load_arena_json(arena_json);
    check_true(planner.boxes().size() == 9, "Loaded all 9 arena boxes (S1..S6, L1..L3)");
    check_true(DetourPlanner::place_id_to_goal_id("HOME") == 0, "HOME -> goal_id 0");
    check_true(DetourPlanner::place_id_to_goal_id("S1") == 1, "S1 -> goal_id 1");
    check_true(DetourPlanner::place_id_to_goal_id("S6") == 6, "S6 -> goal_id 6");
    check_true(DetourPlanner::place_id_to_goal_id("L1") == 7, "L1 -> goal_id 7");
    check_true(DetourPlanner::place_id_to_goal_id("L3") == 9, "L3 -> goal_id 9");
    check_near(planner.goals()[0][0], 0.734f, 1e-4f, "Arena HOME goal x == 0.734m");
    check_near(planner.goals()[0][1], 0.950f, 1e-4f, "Arena HOME goal y == 0.950m");

    planner.set_goal_by_id(4, {1.60f, 3.05f});  // S4: (0.734, 4.890, pi/2)
    Obs41f block_obs{};
    block_obs.fill(0.8f);
    block_obs[17 + 2 * 4 + 1] = 0.18f;
    const float mx = 1.60f, my = 3.05f;
    const Vec3f wp0 = planner.raw_waypoint(mx, my);
    const float myaw = std::atan2(wp0[1] - my, wp0[0] - mx);

    for (int step = 1; step <= 199; ++step) {
      planner.update_blocking_replan(mx, my, myaw, block_obs, 0.05f, 10.0f, 0.12f, true);
      check_true(planner.is_human_waiting(), "Must wait in place before 10.0s");
      check_true(planner.human_replans() == 0, "Must not replan before 10.0s");
    }
    planner.update_blocking_replan(mx, my, myaw, block_obs, 0.05f, 10.0f, 0.12f, true);
    check_true(!planner.is_human_waiting(), "Waiting cleared at 10.0s");
    check_true(planner.human_replans() == 1, "Replanned alternate route at 10.0s");
    check_true(planner.blocked_points().size() == 1, "Masked blocking obstacle");

    // 3. Verify Box Detection + 5~10cm Pre-Approach -> Precision Approach Handoff Logic
    planner.set_goal_by_id(1, {0.734f, 0.950f});  // S1: goal=(1.93, 0.95, pi), box=(1.468, 0.95)
    Vec2f s1_box{};
    check_true(planner.nominal_box_xy(1, &s1_box), "S1 nominal box exists");
    check_near(s1_box[0], 1.468f, 1e-4f, "S1 box x");
    check_near(s1_box[1], 0.950f, 1e-4f, "S1 box y");

    // Robot at (2.05, 0.95, pi) facing -X sees S1 shifted +4cm in Y -> base det = (0.582, -0.04)
    const Vec3f rob_pose{2.05f, 0.95f, static_cast<float>(Mk2Kinematics::kPi)};
    const auto ref = refine_box_pre_approach_goal(
      planner.goal(), s1_box, rob_pose, {0.582f, -0.04f}, 0.35f, 0.25f);
    check_true(ref.accepted, "Valid S1 3D box detection accepted");
    check_near(ref.box_arena_xy[0], 1.468f, 1e-3f, "Refined S1 box x");
    check_near(ref.box_arena_xy[1], 0.990f, 1e-3f, "Refined S1 box y (+4cm offset)");
    check_near(ref.refined_goal[0], 1.930f, 1e-3f, "Refined S1 pre-approach x (~20cm bumper gap)");
    check_near(ref.refined_goal[1], 0.990f, 1e-3f, "Refined S1 pre-approach y");

    // Adjacent box (e.g. 0.734m away) must be rejected by assoc_radius_m=0.35m
    const auto wrong_box = refine_box_pre_approach_goal(
      planner.goal(), s1_box, rob_pose, {0.582f, -0.734f}, 0.35f, 0.25f);
    check_true(!wrong_box.accepted, "Adjacent box (>0.35m offset) rejected");

    // Precision approach trigger: requires box_visible=true AND dist <= 0.08m (inside 5~10cm)
    const Vec3f far_pose{2.05f, 0.99f, static_cast<float>(Mk2Kinematics::kPi)};   // 12cm > 8cm
    const Vec3f near_pose{1.99f, 0.99f, static_cast<float>(Mk2Kinematics::kPi)};  // 6cm <= 8cm
    check_true(
      !should_trigger_precision_approach(far_pose, ref.refined_goal, true, 0.08f, 0.20f),
      "Do not trigger precision approach at 12cm (> 8cm)");
    check_true(
      !should_trigger_precision_approach(near_pose, ref.refined_goal, false, 0.08f, 0.20f),
      "Do not trigger precision approach at 6cm when box is NOT detected");
    check_true(
      should_trigger_precision_approach(near_pose, ref.refined_goal, true, 0.08f, 0.20f),
      "Trigger precision approach when box detected AND within 6cm (<= 8cm, inside 5~10cm)");

    // Synthesized front-face points produce valid vis5 in build_obs_41d
    const auto synth_pts = synthesize_box_front_points({0.462f, 0.0f}, 0.0f);
    check_true(synth_pts.size() == 5, "Synthesized 5 front-face points");
    const Obs41f synth_obs = build_obs_41d(
      {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, ref.refined_goal,
      dummy_slice, synth_pts, {}, xsting_calib.init_pose_arena,
      imu_state, ref.refined_goal);
    check_near(synth_obs[33], 1.0f, 1e-5f, "Synthesized box points set visible=1.0");
    check_near(synth_obs[34], 0.462f - Mk2Kinematics::kBoxHalfM, 0.01f, "Front face centroid x");

    // 4. Verify CalibrationConfig YAML round-trip & OpenCR tick limits
    CalibrationConfig calib = CalibrationConfig::make_defaults("mk2_hexapod");
    const Leg18f hw_stand = LegGaitController::sim_to_hardware_legs(Mk2Kinematics::stand_pose(), calib);
    check_true(hw_stand[3] > -4.713f && hw_stand[3] < -1.570f, "OpenCR ID 4 continuous yaw range");
    check_true(hw_stand[12] > 1.571f && hw_stand[12] < 4.712f, "OpenCR ID 13 continuous yaw range");
    const Leg18f roundtrip_sim = LegGaitController::hardware_to_sim_legs(hw_stand, calib);
    for (size_t i = 0; i < 18; ++i) {
      check_near(roundtrip_sim[i], Mk2Kinematics::stand_pose()[i], 1e-4f, "Joint calibration inverse");
    }

    const std::filesystem::path tmp_yaml = std::filesystem::temp_directory_path() / "xsting_nav_calib_test.yaml";
    calib.cmd_vel_scale = {1.18f, 1.24f, 1.09f};
    calib.save_yaml(tmp_yaml.string());
    const CalibrationConfig loaded_calib = CalibrationConfig::load_yaml(tmp_yaml.string());
    check_near(loaded_calib.cmd_vel_scale[0], 1.18f, 1e-5f, "YAML round-trip vx_scale");
    std::filesystem::remove(tmp_yaml);

    // 5. Verify 2D LiDAR -> Static Arena ESDF Global Initial Alignment & Drift Correction
    float gx_w = 0.0f;
    float gy_w = 0.0f;
    const float left_wall_d = planner.static_esdf_bilinear(0.0f, 0.92f, &gx_w, &gy_w);
    check_near(left_wall_d, 0.0f, 0.015f, "Outer left wall zero-crossing at x=0.0m");
    check_near(gx_w, 1.0f, 0.05f, "Outer left wall ESDF gradient +1 in X");

    // Helper: sphere-trace synthetic 360-deg LiDAR scan in arena ESDF from true_pose
    auto synth_arena_scan = [&](const Vec3f & true_pose) {
      std::vector<Vec2f> pts;
      const float ct = std::cos(true_pose[2]);
      const float st = std::sin(true_pose[2]);
      for (int k = 0; k < 180; ++k) {
        const float ang_b = static_cast<float>(-Mk2Kinematics::kPi + (2.0 * Mk2Kinematics::kPi * k) / 180.0);
        const float dir_bx = std::cos(ang_b);
        const float dir_by = std::sin(ang_b);
        const float dir_wx = ct * dir_bx - st * dir_by;
        const float dir_wy = st * dir_bx + ct * dir_by;
        float t = 0.14f;
        bool hit = false;
        for (int step = 0; step < 64 && t < 3.45f; ++step) {
          const float wx = true_pose[0] + t * dir_wx;
          const float wy = true_pose[1] + t * dir_wy;
          const float d = planner.static_esdf_bilinear(wx, wy);
          if (d <= 0.002f) {
            hit = true;
            break;
          }
          t += std::max(0.015f, 0.85f * d);
        }
        if (hit && t >= 0.14f && t <= 3.45f) {
          pts.push_back({t * dir_bx, t * dir_by});
        }
      }
      return pts;
    };

    // 5a. Global initial relocalization at HOME with (+13cm, -10cm, +9.5 deg) placement offset
    const Vec3f true_home{0.864f, 0.850f, wrap_to_pi(static_cast<float>(Mk2Kinematics::kPi) + 0.165f)};
    const Vec3f prior_home = xsting_calib.init_pose_arena;  // (0.734, 0.950, pi)
    const Vec3f zero_odom{0.0f, 0.0f, 0.0f};
    const auto scan_home = synth_arena_scan(true_home);
    check_true(scan_home.size() >= 80, "Synthetic 360 scan at HOME hit arena walls/boxes");

    const auto loc_init = planner.localize_lidar_on_arena(
      scan_home, prior_home, zero_odom, /*global_search=*/true, 20, 0.065f);
    check_true(loc_init.converged, "Global initial relocalization converged at HOME");
    check_near(loc_init.arena_pose[0], true_home[0], 0.025f, "Global reloc HOME x within 2.5cm");
    check_near(loc_init.arena_pose[1], true_home[1], 0.025f, "Global reloc HOME y within 2.5cm");
    check_near(wrap_to_pi(loc_init.arena_pose[2] - true_home[2]), 0.0f, 0.03f, "Global reloc HOME yaw");

    // 5b. Continuous walking drift correction mid-corridor with (-4.5cm, +3.5cm, -3.5 deg) odom drift
    const Vec3f true_mid{2.20f, 2.45f, 0.52f};
    const Vec3f eff_odom_mid{1.40f, -1.55f, -2.55f};
    const Vec3f drifted_prior{2.155f, 2.485f, 0.459f};
    const auto scan_mid = synth_arena_scan(true_mid);
    const auto loc_drift = planner.localize_lidar_on_arena(
      scan_mid, drifted_prior, eff_odom_mid, /*global_search=*/false, 20, 0.065f);
    check_true(loc_drift.converged, "Continuous local drift correction converged");
    const Vec3f check_composed = compose_se2(loc_drift.map_to_odom, eff_odom_mid);
    check_near(check_composed[0], true_mid[0], 0.025f, "Drift-corrected map_to_odom -> arena x");
    check_near(check_composed[1], true_mid[1], 0.025f, "Drift-corrected map_to_odom -> arena y");
    check_near(wrap_to_pi(check_composed[2] - true_mid[2]), 0.0f, 0.03f, "Drift-corrected yaw");

    std::cout << "[PASS] All xsting_nav parity, 10s-detour, 5~10cm box->approach handoff, LiDAR global localization, and calibration tests passed "
              << "(max MLP abs err = " << max_act_err << ").\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
