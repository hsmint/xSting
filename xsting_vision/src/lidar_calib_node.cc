#include "xsting_vision/lidar_calib_node.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <sstream>
#include <utility>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <cuda_buffer/cuda_buffer_api.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/exceptions.hpp>

namespace xsting_vision {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

// 평면 직선 방정식(nx * x + ny * y = dist, dist >= 0)
struct Line2d {
  double nx = 1.0;
  double ny = 0.0;
  double dist = 0.0;
  int inliers = 0;
};

// 각도를 [-pi, pi] 범위로 정규화
double NormAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

// Roll 상하 반전 여부에 따른 Y축 부호(+1 또는 -1) 반환
double GetRollSignY(double roll) {
  return (std::abs(NormAngle(roll)) > 1.5) ? -1.0 : 1.0;
}

// LiDAR 로컬 점(x, y)을 외부 파라미터로 base 평면 좌표로 변환
Vec2d TransformScanPt(const LidarPose& pose, const Vec2d& pt) {
  const double sy = GetRollSignY(pose.roll) * pt.y;
  const double cos_a = std::cos(pose.yaw);
  const double sin_a = std::sin(pose.yaw);
  return {cos_a * pt.x - sin_a * sy + pose.x,
          sin_a * pt.x + cos_a * sy + pose.y};
}

// 3x3 선형 방정식(hess * delta = rhs) 가우스 소거 풀이
bool SolveLinear3(const double hess[3][3], const double rhs[3],
                  double delta[3]) {
  double mat[3][4];
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      mat[row][col] = hess[row][col];
    }
    mat[row][3] = rhs[row];
  }
  for (int col = 0; col < 3; ++col) {
    int pivot = col;
    for (int row = col + 1; row < 3; ++row) {
      if (std::abs(mat[row][col]) > std::abs(mat[pivot][col])) {
        pivot = row;
      }
    }
    if (std::abs(mat[pivot][col]) < 1e-9) {
      return false;
    }
    if (pivot != col) {
      for (int idx = col; idx < 4; ++idx) {
        std::swap(mat[col][idx], mat[pivot][idx]);
      }
    }
    for (int row = col + 1; row < 3; ++row) {
      const double factor = mat[row][col] / mat[col][col];
      for (int idx = col; idx < 4; ++idx) {
        mat[row][idx] -= factor * mat[col][idx];
      }
    }
  }
  for (int row = 2; row >= 0; --row) {
    double sum = mat[row][3];
    for (int col = row + 1; col < 3; ++col) {
      sum -= mat[row][col] * delta[col];
    }
    delta[row] = sum / mat[row][row];
  }
  return std::isfinite(delta[0]) && std::isfinite(delta[1]) &&
         std::isfinite(delta[2]);
}

// 방위각 순으로 정렬된 기준 벽면 점군에서 인접 점 기반 법선 벡터 계산
void FillWallNorms(std::vector<WallPt>& pts) {
  if (pts.size() < 3) {
    return;
  }
  for (size_t i = 1; i + 1 < pts.size(); ++i) {
    const double dx = pts[i + 1].x - pts[i - 1].x;
    const double dy = pts[i + 1].y - pts[i - 1].y;
    const double span = std::hypot(dx, dy);
    if (span >= 0.01 && span <= 0.18) {
      double nx = -dy / span;
      double ny = dx / span;
      if (nx * pts[i].x + ny * pts[i].y > 0.0) {
        nx = -nx;
        ny = -ny;
      }
      pts[i].nx = nx;
      pts[i].ny = ny;
      pts[i].has_norm = true;
    }
  }
}

// 스캔 점군과 기준 벽면 점군 사이의 Point-to-Line RMS 정합 오차(mm) 계산
double CalcAlignRmsMm(const std::vector<ScanSample>& samples,
                      const LidarPose& pose, double max_corr) {
  const double max_sq = max_corr * max_corr;
  double sq_sum = 0.0;
  int cnt = 0;
  for (const ScanSample& sample : samples) {
    for (const Vec2d& pt : sample.scan_pts) {
      const Vec2d base_pt = TransformScanPt(pose, pt);
      int best_i = -1;
      double best_sq = max_sq;
      for (size_t i = 0; i < sample.ref_pts.size(); ++i) {
        const double dx = base_pt.x - sample.ref_pts[i].x;
        const double dy = base_pt.y - sample.ref_pts[i].y;
        const double dist_sq = dx * dx + dy * dy;
        if (dist_sq < best_sq) {
          best_sq = dist_sq;
          best_i = static_cast<int>(i);
        }
      }
      if (best_i >= 0) {
        const WallPt& ref = sample.ref_pts[static_cast<size_t>(best_i)];
        const double err = ref.has_norm
                               ? (ref.nx * (base_pt.x - ref.x) +
                                  ref.ny * (base_pt.y - ref.y))
                               : std::sqrt(best_sq);
        sq_sum += err * err;
        ++cnt;
      }
    }
  }
  if (cnt == 0) {
    return 999.9;
  }
  return std::sqrt(sq_sum / static_cast<double>(cnt)) * 1000.0;
}

// 360도 회전각 및 상하 반전(roll=0, pi) 전역 탐색으로 초기 자세 결정
LidarPose FindCoarsePose(const std::vector<ScanSample>& samples,
                         const LidarPose& cur_pose) {
  LidarPose best_pose = cur_pose;
  double best_cost = std::numeric_limits<double>::infinity();
  const std::array<double, 2> rolls = {kPi, 0.0};
  const size_t sam_step = std::max<size_t>(1, samples.size() / 5);

  for (double roll_cand : rolls) {
    for (int step = 0; step < 180; ++step) {
      LidarPose cand = cur_pose;
      cand.roll = roll_cand;
      cand.yaw = NormAngle(-kPi + step * (2.0 * kPi / 180.0));

      double cost_sum = 0.0;
      int fov_cnt = 0;
      for (size_t sam_i = 0; sam_i < samples.size(); sam_i += sam_step) {
        const ScanSample& sample = samples[sam_i];
        if (sample.ref_pts.size() < 2) {
          continue;
        }
        const double min_az =
            std::atan2(sample.ref_pts.front().y, sample.ref_pts.front().x) +
            0.03;
        const double max_az =
            std::atan2(sample.ref_pts.back().y, sample.ref_pts.back().x) -
            0.03;
        const size_t pt_step =
            std::max<size_t>(1, sample.scan_pts.size() / 90);

        for (size_t i = 0; i < sample.scan_pts.size(); i += pt_step) {
          const Vec2d base_pt = TransformScanPt(cand, sample.scan_pts[i]);
          const double azim = std::atan2(base_pt.y, base_pt.x);
          if (azim < min_az || azim > max_az) {
            continue;
          }
          double min_sq = 0.14 * 0.14;
          for (const WallPt& ref : sample.ref_pts) {
            const double dx = base_pt.x - ref.x;
            const double dy = base_pt.y - ref.y;
            min_sq = std::min(min_sq, dx * dx + dy * dy);
          }
          cost_sum += min_sq;
          ++fov_cnt;
        }
      }
      if (fov_cnt >= 12) {
        const double mean_cost = cost_sum / static_cast<double>(fov_cnt);
        if (mean_cost < best_cost) {
          best_cost = mean_cost;
          best_pose = cand;
        }
      }
    }
  }
  return best_pose;
}

// Huber 가중 Point-to-Line 정합으로 (x, y, yaw) 최적화 및 가관측성 판별
bool RefineCamPose(const std::vector<ScanSample>& samples,
                   const LidarPose& anchor_pose, LidarPose& pose,
                   bool& full_xy) {
  constexpr double kHuber = 0.015;
  const double sign_y = GetRollSignY(pose.roll);
  full_xy = false;

  for (int iter = 0; iter < 20; ++iter) {
    const double max_corr = (iter < 7) ? 0.18 : 0.06;
    const double max_sq = max_corr * max_corr;
    const double cos_a = std::cos(pose.yaw);
    const double sin_a = std::sin(pose.yaw);

    double hess[3][3] = {{1e-4, 0.0, 0.0},
                         {0.0, 1e-4, 0.0},
                         {0.0, 0.0, 1e-4}};
    double rhs[3] = {0.0, 0.0, 0.0};
    double info_xx = 0.0;
    double info_xy = 0.0;
    double info_yy = 0.0;
    int inliers = 0;

    for (const ScanSample& sample : samples) {
      for (const Vec2d& pt : sample.scan_pts) {
        const double sy = sign_y * pt.y;
        const double wx = cos_a * pt.x - sin_a * sy + pose.x;
        const double wy = sin_a * pt.x + cos_a * sy + pose.y;

        int best_i = -1;
        double best_sq = max_sq;
        for (size_t i = 0; i < sample.ref_pts.size(); ++i) {
          const double dx = wx - sample.ref_pts[i].x;
          const double dy = wy - sample.ref_pts[i].y;
          const double dist_sq = dx * dx + dy * dy;
          if (dist_sq < best_sq) {
            best_sq = dist_sq;
            best_i = static_cast<int>(i);
          }
        }
        if (best_i < 0) {
          continue;
        }
        const WallPt& ref = sample.ref_pts[static_cast<size_t>(best_i)];
        if (!ref.has_norm) {
          continue;
        }
        ++inliers;

        const double drx = -sin_a * pt.x - cos_a * sy;
        const double dry = cos_a * pt.x - sin_a * sy;
        const double err = ref.nx * (wx - ref.x) + ref.ny * (wy - ref.y);
        const double abs_err = std::abs(err);
        const double weight = (abs_err > kHuber) ? (kHuber / abs_err) : 1.0;
        const double jac[3] = {ref.nx, ref.ny, ref.nx * drx + ref.ny * dry};

        info_xx += weight * ref.nx * ref.nx;
        info_xy += weight * ref.nx * ref.ny;
        info_yy += weight * ref.ny * ref.ny;
        for (int row = 0; row < 3; ++row) {
          rhs[row] -= weight * jac[row] * err;
          for (int col = 0; col < 3; ++col) {
            hess[row][col] += weight * jac[row] * jac[col];
          }
        }
      }
    }
    if (inliers < 20) {
      return false;
    }

    // 단일 평면 벽 관측 시 미관측 접선 방향 이동을 기준값으로 정규화
    const double trace = info_xx + info_yy;
    const double disc = std::hypot(info_xx - info_yy, 2.0 * info_xy);
    const double eig_max = 0.5 * (trace + disc);
    const double eig_min = 0.5 * (trace - disc);
    full_xy = (eig_max > 1.0) && (eig_min >= 0.05 * eig_max);
    if (!full_xy && eig_max > 1e-6) {
      const double major_ang =
          0.5 * std::atan2(2.0 * info_xy, info_xx - info_yy);
      const double ux = -std::sin(major_ang);
      const double uy = std::cos(major_ang);
      const double prior_w = 0.25 * eig_max + 10.0;
      const double proj =
          ux * (pose.x - anchor_pose.x) + uy * (pose.y - anchor_pose.y);
      hess[0][0] += prior_w * ux * ux;
      hess[0][1] += prior_w * ux * uy;
      hess[1][0] += prior_w * uy * ux;
      hess[1][1] += prior_w * uy * uy;
      rhs[0] -= prior_w * ux * proj;
      rhs[1] -= prior_w * uy * proj;
    }

    double delta[3] = {0.0, 0.0, 0.0};
    if (!SolveLinear3(hess, rhs, delta)) {
      return false;
    }
    pose.x += delta[0];
    pose.y += delta[1];
    pose.yaw = NormAngle(pose.yaw + delta[2]);
    if (delta[0] * delta[0] + delta[1] * delta[1] < 1e-10 &&
        std::abs(delta[2]) < 1e-5) {
      break;
    }
  }
  return true;
}

// 2D 점군에서 현(chord) 합의 탐색과 직교 최소제곱(PCA)으로 최적 직선 추출
bool FitWallLine(const std::vector<Vec2d>& pts, Line2d& out_line,
                 std::vector<uint8_t>* inlier_mask) {
  const size_t cnt = pts.size();
  if (cnt < 15) {
    return false;
  }
  constexpr double kInlierTol = 0.018;
  const std::array<size_t, 3> chords = {std::max<size_t>(2, cnt / 12),
                                        std::max<size_t>(4, cnt / 6),
                                        std::max<size_t>(6, cnt / 4)};
  int best_cnt = 0;
  double best_nx = 1.0;
  double best_ny = 0.0;
  double best_dist = 0.0;

  for (size_t chord : chords) {
    for (size_t i = 0; i < cnt; ++i) {
      const Vec2d& pt_a = pts[i];
      const Vec2d& pt_b = pts[(i + chord) % cnt];
      const double dx = pt_b.x - pt_a.x;
      const double dy = pt_b.y - pt_a.y;
      const double span = std::hypot(dx, dy);
      if (span < 0.08 || span > 2.0) {
        continue;
      }
      double nx = -dy / span;
      double ny = dx / span;
      double dist = nx * pt_a.x + ny * pt_a.y;
      if (dist < 0.0) {
        nx = -nx;
        ny = -ny;
        dist = -dist;
      }
      int cur_cnt = 0;
      for (const Vec2d& pt : pts) {
        if (std::abs(nx * pt.x + ny * pt.y - dist) <= kInlierTol) {
          ++cur_cnt;
        }
      }
      if (cur_cnt > best_cnt) {
        best_cnt = cur_cnt;
        best_nx = nx;
        best_ny = ny;
        best_dist = dist;
      }
    }
  }
  if (best_cnt < 15) {
    return false;
  }

  // 인라이어 점군의 공분산 고유벡터로 정밀 직선 법선 및 수직 거리 산출
  double mean_x = 0.0;
  double mean_y = 0.0;
  std::vector<size_t> inlier_idx;
  inlier_idx.reserve(static_cast<size_t>(best_cnt));
  for (size_t i = 0; i < cnt; ++i) {
    if (std::abs(best_nx * pts[i].x + best_ny * pts[i].y - best_dist) <=
        kInlierTol) {
      mean_x += pts[i].x;
      mean_y += pts[i].y;
      inlier_idx.push_back(i);
    }
  }
  const double inv_n = 1.0 / static_cast<double>(inlier_idx.size());
  mean_x *= inv_n;
  mean_y *= inv_n;

  double cov_xx = 0.0;
  double cov_xy = 0.0;
  double cov_yy = 0.0;
  for (size_t idx : inlier_idx) {
    const double dx = pts[idx].x - mean_x;
    const double dy = pts[idx].y - mean_y;
    cov_xx += dx * dx;
    cov_xy += dx * dy;
    cov_yy += dy * dy;
  }
  const double tangent_ang =
      0.5 * std::atan2(2.0 * cov_xy, cov_xx - cov_yy);
  double nx = -std::sin(tangent_ang);
  double ny = std::cos(tangent_ang);
  double dist = nx * mean_x + ny * mean_y;
  if (dist < 0.0) {
    nx = -nx;
    ny = -ny;
    dist = -dist;
  }

  out_line = {nx, ny, dist, static_cast<int>(inlier_idx.size())};
  if (inlier_mask != nullptr) {
    inlier_mask->assign(cnt, 0);
    for (size_t idx : inlier_idx) {
      (*inlier_mask)[idx] = 1;
    }
  }
  return true;
}

// 2D 점 목록을 RViz·Foxglove 확인용 PointCloud2 메시지로 변환
CloudMsg MakeCloudMsg(const std_msgs::msg::Header& header,
                      const std::string& frame_id, double z_val,
                      const std::vector<Vec2d>& pts) {
  CloudMsg cloud;
  cloud.header.stamp = header.stamp;
  cloud.header.frame_id = frame_id;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(pts.size());
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 12;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(3);
  const std::array<const char*, 3> names = {"x", "y", "z"};
  for (size_t i = 0; i < 3; ++i) {
    cloud.fields[i].name = names[i];
    cloud.fields[i].offset = static_cast<uint32_t>(i * 4);
    cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[i].count = 1;
  }
  cloud.data.resize(cloud.row_step);
  float* ptr = reinterpret_cast<float*>(cloud.data.data());
  for (const Vec2d& pt : pts) {
    ptr[0] = static_cast<float>(pt.x);
    ptr[1] = static_cast<float>(pt.y);
    ptr[2] = static_cast<float>(z_val);
    ptr += 3;
  }
  return cloud;
}

}  // namespace

// YAML 설정 로드 후 스캔·깊이 구독 및 디버그 점군 퍼블리셔 초기화
LidarCalibNode::LidarCalibNode(const rclcpp::NodeOptions& opts)
    : Node("lidar_calib", AddYamlParams(opts)),
      tf_buf_(get_clock()),
      tf_sub_(tf_buf_, *this, false) {
  cfg_ = LoadCfg();
  cudaStreamCreateWithFlags(&cuda_stream_, cudaStreamNonBlocking);

  const auto sensor_qos = rclcpp::SensorDataQoS();
  const auto latched_qos = rclcpp::QoS(1).transient_local();
  scan_pub_ =
      create_publisher<CloudMsg>("lidar_calib/scan_cloud", latched_qos);
  ref_pub_ = create_publisher<CloudMsg>("lidar_calib/ref_cloud", latched_qos);

  if (cfg_.mode == "camera") {
    rclcpp::SubscriptionOptions any_opts;
    any_opts.acceptable_buffer_backends = "any";
    const std::string depth_prefix =
        cfg_.depth_topic.substr(0, cfg_.depth_topic.rfind('/'));
    info_sub_ = create_subscription<CamInfoMsg>(
        depth_prefix + "/camera_info", sensor_qos,
        [this](const CamInfoMsg::ConstSharedPtr& msg) {
          std::lock_guard<std::mutex> lock(mtx_);
          depth_info_ = msg;
        });
    depth_sub_ = create_subscription<ImageMsg>(
        cfg_.depth_topic, sensor_qos,
        [this](const ImageMsg::ConstSharedPtr& msg) {
          std::lock_guard<std::mutex> lock(mtx_);
          latest_depth_ = msg;
        },
        any_opts);
  }

  scan_sub_ = create_subscription<ScanMsg>(
      cfg_.scan_topic, sensor_qos,
      [this](const ScanMsg::ConstSharedPtr& msg) { OnScan(msg); });

  RCLCPP_INFO(get_logger(),
              "LiDAR calib (mode=%s, samples=%d, range=%.2f~%.2fm)",
              cfg_.mode.c_str(), cfg_.num_samples, cfg_.min_range,
              cfg_.max_range);
}

// 구독 해제 및 CUDA 스트림 정리
LidarCalibNode::~LidarCalibNode() {
  scan_sub_.reset();
  depth_sub_.reset();
  info_sub_.reset();
  cudaStreamSynchronize(cuda_stream_);
  cudaStreamDestroy(cuda_stream_);
}

// 기본 YAML 설정 파일을 읽어 NodeOptions 앞쪽에 병합
rclcpp::NodeOptions LidarCalibNode::AddYamlParams(
    const rclcpp::NodeOptions& opts) {
  std::string cfg_path =
      (ament_index_cpp::get_package_share_path("xsting_vision") / "config" /
       "lidar_calib.yaml")
          .string();
  for (const rclcpp::Parameter& param : opts.parameter_overrides()) {
    if (param.get_name() == "config_file") {
      cfg_path = param.as_string();
    }
  }

  std::vector<rclcpp::Parameter> yaml_params;
  try {
    for (const auto& entry : rclcpp::parameter_map_from_yaml_file(cfg_path)) {
      yaml_params.insert(yaml_params.end(), entry.second.begin(),
                         entry.second.end());
    }
  } catch (const std::exception&) {
  }

  rclcpp::NodeOptions out = opts;
  auto& overrides = out.parameter_overrides();
  overrides.insert(overrides.begin(), yaml_params.begin(), yaml_params.end());
  return out;
}

// 보정 모드·필터·실측 거리 파라미터 선언 및 로드
LidarCalibCfg LidarCalibNode::LoadCfg() {
  LidarCalibCfg cfg;
  cfg.mode = declare_parameter<std::string>("mode", "camera");
  cfg.scan_topic = declare_parameter<std::string>("scan_topic", "/scan");
  cfg.depth_topic = declare_parameter<std::string>(
      "depth_topic", "/camera/depth/image_emitter_on");
  cfg.base_frame = declare_parameter<std::string>("base_frame", "base");
  cfg.lidar_frame = declare_parameter<std::string>("lidar_frame", "lidar");
  cfg.min_range = declare_parameter<double>("min_range", 0.38);
  cfg.max_range = declare_parameter<double>("max_range", 2.50);
  cfg.depth_min_z = declare_parameter<double>("depth_min_z", 0.04);
  cfg.depth_max_z = declare_parameter<double>("depth_max_z", 0.28);
  const int raw_samples =
      static_cast<int>(declare_parameter<int>("num_samples", 30));
  cfg.num_samples = std::max(5, raw_samples);
  cfg.sample_interval = declare_parameter<double>("sample_interval", 0.10);
  cfg.max_sync_diff = declare_parameter<double>("max_sync_diff", 0.08);
  cfg.lidar_z = declare_parameter<double>("lidar_z", -1.0);
  cfg.front_wall_dist = declare_parameter<double>("front_wall_dist", 0.0);
  cfg.left_wall_dist = declare_parameter<double>("left_wall_dist", 0.0);
  cfg.right_wall_dist = declare_parameter<double>("right_wall_dist", 0.0);
  cfg.apply_urdf = declare_parameter<bool>("apply_urdf", false);
  cfg.urdf_path = declare_parameter<std::string>(
      "urdf_path", "src/xsting_description/urdf/xsting.urdf");
  return cfg;
}

// 현재 TF 트리에서 base -> lidar 초기 외부 파라미터 조회
LidarPose LidarCalibNode::LookupCurPose(const std::string& scan_frame) const {
  LidarPose pose;
  try {
    const TfMsg tf_msg = tf_buf_.lookupTransform(cfg_.base_frame, scan_frame,
                                                 tf2::TimePointZero);
    pose.x = tf_msg.transform.translation.x;
    pose.y = tf_msg.transform.translation.y;
    pose.z = tf_msg.transform.translation.z;
    const auto& rot_q = tf_msg.transform.rotation;
    double roll_rad = kPi;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
    tf2::Matrix3x3(tf2::Quaternion(rot_q.x, rot_q.y, rot_q.z, rot_q.w))
        .getRPY(roll_rad, pitch_rad, yaw_rad);
    pose.roll = (std::abs(NormAngle(roll_rad)) > 1.5) ? kPi : 0.0;
    pose.yaw = yaw_rad;
  } catch (const tf2::TransformException&) {
  }
  if (cfg_.lidar_z >= 0.0) {
    pose.z = cfg_.lidar_z;
  }
  return pose;
}

// 스캔 수신 시 유효 점군을 추출하고 모드별 샘플 누적 후 보정 실행
void LidarCalibNode::OnScan(const ScanMsg::ConstSharedPtr& scan_msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (done_) {
    return;
  }

  const rclcpp::Time stamp(scan_msg->header.stamp);
  const double dt_sec = std::abs((stamp - last_sample_stamp_).seconds());
  if (!samples_.empty() && dt_sec < cfg_.sample_interval) {
    return;
  }

  // 본체·다리 반경을 벗어난 유효 스캔 점 추출
  std::vector<Vec2d> scan_pts;
  scan_pts.reserve(scan_msg->ranges.size());
  const double angle_inc = scan_msg->angle_increment;
  for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
    const float range = scan_msg->ranges[i];
    if (!std::isfinite(range) || range < cfg_.min_range ||
        range > cfg_.max_range) {
      continue;
    }
    const double angle =
        scan_msg->angle_min + static_cast<double>(i) * angle_inc;
    scan_pts.push_back({static_cast<double>(range) * std::cos(angle),
                        static_cast<double>(range) * std::sin(angle)});
  }
  if (scan_pts.size() < 20) {
    return;
  }

  const std::string scan_frame = scan_msg->header.frame_id.empty()
                                     ? cfg_.lidar_frame
                                     : scan_msg->header.frame_id;
  const LidarPose cur_pose = LookupCurPose(scan_frame);

  // camera 모드: 동기화된 RealSense 깊이 영상에서 기준 벽면 점군 추출
  std::vector<WallPt> ref_pts;
  if (cfg_.mode == "camera") {
    if (!latest_depth_ || !depth_info_) {
      return;
    }
    const rclcpp::Time depth_stamp(latest_depth_->header.stamp);
    if (std::abs((stamp - depth_stamp).seconds()) > cfg_.max_sync_diff) {
      return;
    }
    TfMsg tf_base_cam;
    try {
      tf_base_cam = tf_buf_.lookupTransform(
          cfg_.base_frame, depth_info_->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException&) {
      return;
    }
    if (!CopyDepth(*latest_depth_)) {
      return;
    }
    ref_pts = ExtractDepthWall(*latest_depth_, *depth_info_, tf_base_cam);
    if (ref_pts.size() < 15) {
      return;
    }
  }

  samples_.push_back({scan_pts, ref_pts});
  last_sample_stamp_ = stamp;
  RCLCPP_INFO(get_logger(), "Sample %zu / %d collected (scan=%zu, ref=%zu)",
              samples_.size(), cfg_.num_samples, scan_pts.size(),
              ref_pts.size());

  // 목표 샘플 수 도달 시 보정 계산·디버그 발행 후 종료
  if (static_cast<int>(samples_.size()) >= cfg_.num_samples) {
    done_ = true;
    if (cfg_.mode == "camera") {
      FinishCamCalib(cur_pose);
    } else {
      FinishWallCalib(cur_pose);
    }
    rclcpp::sleep_for(std::chrono::milliseconds(150));
    rclcpp::shutdown();
  }
}

// GPU 또는 CPU 16UC1 깊이 버퍼를 호스트 메모리로 복사
bool LidarCalibNode::CopyDepth(const ImageMsg& depth_msg) {
  if (depth_msg.encoding != "16UC1" || depth_msg.width == 0 ||
      depth_msg.height == 0) {
    return false;
  }
  const size_t row_bytes = depth_msg.width * sizeof(uint16_t);
  const size_t min_bytes =
      static_cast<size_t>(depth_msg.height - 1) * depth_msg.step + row_bytes;
  if (depth_msg.data.size() < min_bytes) {
    return false;
  }

  depth_buf_.resize(static_cast<size_t>(depth_msg.width) * depth_msg.height);
  {
    auto handle =
        cuda_buffer_backend::from_input_buffer(depth_msg.data, cuda_stream_);
    cudaMemcpy2DAsync(depth_buf_.data(), row_bytes, handle.get_ptr(),
                      depth_msg.step, row_bytes, depth_msg.height,
                      cudaMemcpyDeviceToHost, cuda_stream_);
  }
  cudaStreamSynchronize(cuda_stream_);
  return true;
}

// 깊이 영상을 base 좌표계 벽면 높이로 슬라이스해 방위각별 중앙값 점군 추출
std::vector<WallPt> LidarCalibNode::ExtractDepthWall(
    const ImageMsg& depth_msg, const CamInfoMsg& info,
    const TfMsg& tf_base_cam) const {
  const double fx = info.k[0];
  const double cx = info.k[2];
  const double fy = info.k[4];
  const double cy = info.k[5];
  if (fx <= 0.0 || fy <= 0.0) {
    return {};
  }

  const auto& tr = tf_base_cam.transform.translation;
  const auto& rq = tf_base_cam.transform.rotation;
  const tf2::Matrix3x3 rot(tf2::Quaternion(rq.x, rq.y, rq.z, rq.w));
  const tf2::Vector3 trans(tr.x, tr.y, tr.z);

  constexpr int kNumBins = 720;
  std::vector<std::vector<double>> bins(kNumBins);
  for (uint32_t row = 0; row < depth_msg.height; row += 2) {
    const uint16_t* row_ptr =
        depth_buf_.data() + static_cast<size_t>(row) * depth_msg.width;
    for (uint32_t col = 0; col < depth_msg.width; col += 2) {
      const uint16_t depth_mm = row_ptr[col];
      if (depth_mm == 0) {
        continue;
      }
      const double z_cam = static_cast<double>(depth_mm) * 0.001;
      const double x_cam = (static_cast<double>(col) - cx) * z_cam / fx;
      const double y_cam = (static_cast<double>(row) - cy) * z_cam / fy;
      const tf2::Vector3 cam_pt(x_cam, y_cam, z_cam);
      const tf2::Vector3 pt_b = rot * cam_pt + trans;
      if (pt_b.z() < cfg_.depth_min_z || pt_b.z() > cfg_.depth_max_z) {
        continue;
      }
      const double dist = std::hypot(pt_b.x(), pt_b.y());
      if (dist < cfg_.min_range || dist > cfg_.max_range) {
        continue;
      }
      const double azim = std::atan2(pt_b.y(), pt_b.x());
      const int bin_i = std::clamp(
          static_cast<int>((azim + kPi) / (2.0 * kPi) * kNumBins), 0,
          kNumBins - 1);
      bins[static_cast<size_t>(bin_i)].push_back(dist);
    }
  }

  std::vector<WallPt> wall_pts;
  for (int i = 0; i < kNumBins; ++i) {
    auto& vals = bins[static_cast<size_t>(i)];
    if (vals.size() < 4) {
      continue;
    }
    const size_t mid = vals.size() / 2;
    std::nth_element(vals.begin(), vals.begin() + static_cast<ptrdiff_t>(mid),
                     vals.end());
    const double med_dist = vals[mid];
    const double azim =
        -kPi + (static_cast<double>(i) + 0.5) * (2.0 * kPi / kNumBins);
    wall_pts.push_back({med_dist * std::cos(azim), med_dist * std::sin(azim),
                        0.0, 0.0, false});
  }
  FillWallNorms(wall_pts);
  return wall_pts;
}

// camera 모드: 전역 탐색 후 다중 프레임 Point-to-Line 정합으로 TF 산출
void LidarCalibNode::FinishCamCalib(const LidarPose& cur_pose) {
  LidarPose calib_pose = FindCoarsePose(samples_, cur_pose);
  bool full_xy = false;
  if (!RefineCamPose(samples_, cur_pose, calib_pose, full_xy)) {
    RCLCPP_ERROR(get_logger(), "Camera-LiDAR alignment failed to converge");
    return;
  }

  const double before_mm = CalcAlignRmsMm(samples_, cur_pose, 0.25);
  const double after_mm = CalcAlignRmsMm(samples_, calib_pose, 0.08);

  std_msgs::msg::Header hdr;
  hdr.stamp = last_sample_stamp_;
  PubClouds(hdr, calib_pose, samples_.back().scan_pts, samples_.back().ref_pts);
  ReportResult(cur_pose, calib_pose, before_mm, after_mm, full_xy);
}

// wall 모드: 전방 벽 및 선택적 직교 측면 벽 직선 피팅으로 TF 산출
void LidarCalibNode::FinishWallCalib(const LidarPose& cur_pose) {
  std::vector<Vec2d> raw_pts;
  const size_t sam_step = std::max<size_t>(1, samples_.size() / 10);
  for (size_t i = 0; i < samples_.size(); i += sam_step) {
    raw_pts.insert(raw_pts.end(), samples_[i].scan_pts.begin(),
                   samples_[i].scan_pts.end());
  }

  // 측면 벽 실측값이 주어지면 상하 반전(roll=pi vs 0) 방향도 함께 검증
  std::vector<double> rolls = {cur_pose.roll};
  if (cfg_.left_wall_dist > 0.0 || cfg_.right_wall_dist > 0.0) {
    rolls = {kPi, 0.0};
  }

  LidarPose calib_pose = cur_pose;
  Line2d best_front;
  Line2d best_side;
  bool has_side = false;
  bool found = false;

  for (double roll_cand : rolls) {
    const double sign_y = GetRollSignY(roll_cand);
    const double cos_cur = std::cos(cur_pose.yaw);
    const double sin_cur = std::sin(cur_pose.yaw);

    // 현재 정면(+X, +-50도) 영역 점군에서 전방 벽 직선 피팅
    std::vector<Vec2d> front_cand;
    for (const Vec2d& pt : raw_pts) {
      const double sy = sign_y * pt.y;
      const double rx = cos_cur * pt.x - sin_cur * sy;
      const double ry = sin_cur * pt.x + cos_cur * sy;
      if (rx > 0.15 && std::abs(ry) < 1.2 * rx) {
        front_cand.push_back({pt.x, sy});
      }
    }
    Line2d front_line;
    if (!FitWallLine(front_cand, front_line, nullptr)) {
      continue;
    }
    const double yaw_cand = std::atan2(-front_line.ny, front_line.nx);

    // 전방 벽과 직교(90도 +-10도)하는 측면 코너 벽 탐색
    std::vector<Vec2d> side_cand;
    for (const Vec2d& pt : raw_pts) {
      const double sy = sign_y * pt.y;
      if (std::abs(front_line.nx * pt.x + front_line.ny * sy -
                   front_line.dist) > 0.04) {
        side_cand.push_back({pt.x, sy});
      }
    }
    Line2d side_line;
    bool side_ok = FitWallLine(side_cand, side_line, nullptr) &&
                   (std::abs(front_line.nx * side_line.nx +
                             front_line.ny * side_line.ny) < 0.18);

    // 좌·우 실측 벽 방향과 부호가 일치하는지 확인
    const double side_sin =
        std::sin(yaw_cand) * side_line.nx + std::cos(yaw_cand) * side_line.ny;
    if (side_ok && cfg_.left_wall_dist > 0.0 && side_sin < 0.5) {
      continue;
    }
    if (side_ok && cfg_.right_wall_dist > 0.0 && side_sin > -0.5) {
      continue;
    }

    calib_pose.roll = roll_cand;
    calib_pose.yaw = yaw_cand;
    best_front = front_line;
    best_side = side_line;
    has_side = side_ok;
    found = true;
    break;
  }

  if (!found) {
    RCLCPP_ERROR(get_logger(), "Front wall line fitting failed on /scan");
    return;
  }

  // 실측 수직 거리가 입력된 축의 평행이동 오프셋 계산
  bool full_xy = false;
  if (cfg_.front_wall_dist > 0.0) {
    calib_pose.x = cfg_.front_wall_dist - best_front.dist;
  } else {
    RCLCPP_INFO(get_logger(),
                "LiDAR front wall dist = %.4f m (set -p front_wall_dist:=<m>)",
                best_front.dist);
  }

  if (has_side) {
    const double raw_dot =
        best_front.nx * best_side.nx + best_front.ny * best_side.ny;
    const double dot_val = std::clamp(raw_dot, -1.0, 1.0);
    const double corner_deg = std::acos(std::abs(dot_val)) * kRadToDeg;
    const double side_sign =
        (std::sin(calib_pose.yaw) * best_side.nx +
             std::cos(calib_pose.yaw) * best_side.ny >=
         0.0)
            ? 1.0
            : -1.0;
    RCLCPP_INFO(get_logger(),
                "Corner wall detected (angle=%.2f deg, side_y=%+.4f m)",
                corner_deg, side_sign * best_side.dist);
    if (cfg_.left_wall_dist > 0.0 && side_sign > 0.0) {
      calib_pose.y = cfg_.left_wall_dist - best_side.dist;
      full_xy = (cfg_.front_wall_dist > 0.0);
    } else if (cfg_.right_wall_dist > 0.0 && side_sign < 0.0) {
      calib_pose.y = -cfg_.right_wall_dist + best_side.dist;
      full_xy = (cfg_.front_wall_dist > 0.0);
    }
  }

  // 피팅된 벽면 기준 가상 참조 점군을 만들어 정합 RMS 산출 및 시각화
  const double target_x =
      (cfg_.front_wall_dist > 0.0) ? cfg_.front_wall_dist
                                   : (best_front.dist + cur_pose.x);
  std::vector<WallPt> syn_wall;
  for (int i = -60; i <= 60; ++i) {
    syn_wall.push_back({target_x, i * 0.015, -1.0, 0.0, true});
  }
  for (ScanSample& sample : samples_) {
    sample.ref_pts = syn_wall;
  }

  const double before_mm = CalcAlignRmsMm(samples_, cur_pose, 0.25);
  const double after_mm = CalcAlignRmsMm(samples_, calib_pose, 0.08);
  std_msgs::msg::Header hdr;
  hdr.stamp = last_sample_stamp_;
  PubClouds(hdr, calib_pose, samples_.back().scan_pts, syn_wall);
  ReportResult(cur_pose, calib_pose, before_mm, after_mm, full_xy);
}

// 보정된 스캔 점군과 기준 벽면 점군을 base 좌표계로 발행
void LidarCalibNode::PubClouds(const std_msgs::msg::Header& header,
                               const LidarPose& pose,
                               const std::vector<Vec2d>& scan_pts,
                               const std::vector<WallPt>& ref_pts) {
  std::vector<Vec2d> aligned_pts;
  aligned_pts.reserve(scan_pts.size());
  for (const Vec2d& pt : scan_pts) {
    aligned_pts.push_back(TransformScanPt(pose, pt));
  }
  std::vector<Vec2d> wall_xy;
  wall_xy.reserve(ref_pts.size());
  for (const WallPt& ref : ref_pts) {
    wall_xy.push_back({ref.x, ref.y});
  }
  scan_pub_->publish(
      MakeCloudMsg(header, cfg_.base_frame, pose.z, aligned_pts));
  ref_pub_->publish(MakeCloudMsg(header, cfg_.base_frame, pose.z, wall_xy));
}

// 보정 전·후 오차, 반전 상태, 가관측성 및 URDF 태그를 출력하고 선택 시 저장
void LidarCalibNode::ReportResult(const LidarPose& cur_pose,
                                  const LidarPose& calib_pose, double before_mm,
                                  double after_mm, bool full_xy) {
  const double dx_mm = (calib_pose.x - cur_pose.x) * 1000.0;
  const double dy_mm = (calib_pose.y - cur_pose.y) * 1000.0;
  const double dyaw_deg = NormAngle(calib_pose.yaw - cur_pose.yaw) * kRadToDeg;
  const bool inverted = (std::abs(NormAngle(calib_pose.roll)) > 1.5);

  RCLCPP_INFO(get_logger(), "=== LiDAR Extrinsic Calibration Result ===");
  RCLCPP_INFO(get_logger(), "Align RMS error : %.2f mm -> %.2f mm", before_mm,
              after_mm);
  RCLCPP_INFO(get_logger(), "Mount roll check: %s (roll=%.6f rad)",
              inverted ? "INVERTED (pi)" : "UPRIGHT (0)", calib_pose.roll);
  RCLCPP_INFO(get_logger(),
              "Delta (cur->new): dX=%+.2f mm, dY=%+.2f mm, dYaw=%+.3f deg",
              dx_mm, dy_mm, dyaw_deg);
  if (!full_xy) {
    RCLCPP_WARN(
        get_logger(),
        "Single flat wall observed: X/Yaw calibrated (use L-corner for Y)");
  }
  RCLCPP_INFO(get_logger(), "URDF (%s -> %s):", cfg_.base_frame.c_str(),
              cfg_.lidar_frame.c_str());
  RCLCPP_INFO(get_logger(),
              "<origin xyz=\"%.6f %.6f %.6f\" rpy=\"%.6f 0 %.6f\" />",
              calib_pose.x, calib_pose.y, calib_pose.z, calib_pose.roll,
              calib_pose.yaw);

  if (cfg_.apply_urdf) {
    if (UpdateUrdf(calib_pose)) {
      RCLCPP_INFO(get_logger(), "Updated URDF: %s", cfg_.urdf_path.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to update URDF: %s",
                   cfg_.urdf_path.c_str());
    }
  }
}

// 지정된 URDF 파일 내 lidar_joint의 origin 태그를 보정값으로 교체
bool LidarCalibNode::UpdateUrdf(const LidarPose& calib_pose) const {
  std::ifstream in_file(cfg_.urdf_path);
  if (!in_file.is_open()) {
    return false;
  }
  std::ostringstream oss;
  oss << in_file.rdbuf();
  std::string urdf_text = oss.str();
  in_file.close();

  const size_t joint_pos = urdf_text.find("<joint name=\"lidar_joint\"");
  if (joint_pos == std::string::npos) {
    return false;
  }
  const size_t end_joint = urdf_text.find("</joint>", joint_pos);
  const size_t orig_pos = urdf_text.find("<origin ", joint_pos);
  if (end_joint == std::string::npos || orig_pos == std::string::npos ||
      orig_pos > end_joint) {
    return false;
  }
  const size_t orig_end = urdf_text.find("/>", orig_pos);
  if (orig_end == std::string::npos || orig_end > end_joint) {
    return false;
  }

  char tag_buf[160];
  std::snprintf(tag_buf, sizeof(tag_buf),
                "<origin xyz=\"%.6f %.6f %.6f\" rpy=\"%.6f 0 %.6f\" />",
                calib_pose.x, calib_pose.y, calib_pose.z, calib_pose.roll,
                calib_pose.yaw);
  urdf_text.replace(orig_pos, orig_end + 2 - orig_pos, tag_buf);

  std::ofstream out_file(cfg_.urdf_path, std::ios::trunc);
  if (!out_file.is_open()) {
    return false;
  }
  out_file << urdf_text;
  return out_file.good();
}

}  // namespace xsting_vision

RCLCPP_COMPONENTS_REGISTER_NODE(xsting_vision::LidarCalibNode)
