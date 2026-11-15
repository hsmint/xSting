#include "xsting_approach/pca_vis.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>

namespace xsting_approach {

namespace {

// 점군 평균과 로봇에서 물체 안쪽을 향하는 2D 수평 법선 계산
bool SolveCentroidNormal(const std::vector<Eigen::Vector3f>& pts,
                         Eigen::Vector3f* mu_out,
                         Eigen::Vector2f* norm_out) {
  if (pts.size() < 4) return false;

  Eigen::Vector3d mu = Eigen::Vector3d::Zero();
  for (const auto& p : pts) mu += p.cast<double>();
  mu /= static_cast<double>(pts.size());

  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto& p : pts) {
    const Eigen::Vector3d d = p.cast<double>() - mu;
    cov += d * d.transpose();
  }
  cov /= static_cast<double>(pts.size());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
  if (eig.info() != Eigen::Success) return false;

  Eigen::Vector2d norm = eig.eigenvectors().col(0).head<2>();
  norm /= std::max(1e-6, norm.norm());
  if (norm.dot(mu.head<2>()) < 0.0) norm = -norm;

  *mu_out = mu.cast<float>();
  *norm_out = norm.cast<float>();
  return true;
}

}  // namespace

// 상자 전면 벽면 점군으로부터 중심 XY와 내향 법선 추정
BoxPca FitBoxPca(const std::vector<Eigen::Vector3f>& wall_pts, float half_m) {
  BoxPca res;
  Eigen::Vector3f mu;
  Eigen::Vector2f norm;
  if (!SolveCentroidNormal(wall_pts, &mu, &norm)) return res;
  res.valid = true;
  res.norm_xy = norm;
  res.box_xy = mu.head<2>() + half_m * norm;
  return res;
}

// 상자 위 퍽 상단 점군으로부터 퍽 중심 XY 추정
bool FitPuckPca(const std::vector<Eigen::Vector3f>& top_pts,
                float radius_m,
                Eigen::Vector2f* puck_xy) {
  Eigen::Vector3f mu;
  Eigen::Vector2f norm;
  if (!SolveCentroidNormal(top_pts, &mu, &norm)) return false;
  if (puck_xy) *puck_xy = mu.head<2>() + radius_m * norm;
  return true;
}

// 상위 접근 정책 입력용 9차원 비전 관측 벡터 패킹
Eigen::Matrix<float, 9, 1> PackVis(const Eigen::Vector2f& box_xy,
                                   const Eigen::Vector2f& norm_xy,
                                   const Eigen::Vector2f& p1_xy,
                                   const Eigen::Vector2f& p2_xy,
                                   float vis_flag,
                                   float standoff_m) {
  Eigen::Matrix<float, 9, 1> out;
  const Eigen::Vector2f err = box_xy - standoff_m * norm_xy;
  const Eigen::Vector2f p1_rel = p1_xy - box_xy;
  const Eigen::Vector2f p2_rel = p2_xy - box_xy;
  out << err.x(), err.y(), norm_xy.x(), norm_xy.y(), p1_rel.x(), p1_rel.y(),
      p2_rel.x(), p2_rel.y(), vis_flag;
  return out;
}

// 정밀 위치·각도·시야각 4차원 보조 특징 생성
Eigen::Vector4f MakeFineVis(const Eigen::Matrix<float, 9, 1>& vis,
                            float standoff_m,
                            const Eigen::Vector2f& cam_xy) {
  const Eigen::Vector2f err = vis.segment<2>(0);
  const Eigen::Vector2f norm = vis.segment<2>(2);
  const Eigen::Vector2f box_xy = err + standoff_m * norm;
  const float fe_x = std::clamp(err.x() * 10.0f, -1.5f, 1.5f);
  const float fe_y = std::clamp(err.y() * 10.0f, -1.5f, 1.5f);
  const float fyaw =
      std::clamp(std::atan2(norm.y(), norm.x()) * 8.0f, -1.5f, 1.5f);
  const float bear = std::atan2(box_xy.y() - cam_xy.y(),
                                std::max(0.03f, box_xy.x() - cam_xy.x()));
  return Eigen::Vector4f(fe_x, fe_y, fyaw, bear);
}

// 초음파 정규화 거리 및 목표 간격 오차 3차원 특징 생성
Eigen::Vector3f MakeSonarObs(float range_m,
                             bool valid,
                             float bumper_off_m,
                             float gap_m,
                             float min_range_m) {
  if (!valid || !std::isfinite(range_m)) {
    return Eigen::Vector3f(1.0f, 1.5f, 0.0f);
  }
  const float dist_norm = std::clamp(range_m, min_range_m, 1.50f) / 1.50f;
  const float fine_err =
      std::clamp((range_m - (bumper_off_m + gap_m)) * 10.0f, -1.5f, 1.5f);
  return Eigen::Vector3f(dist_norm, fine_err, 1.0f);
}

// 생성자에서 카메라·초음파 보정 적용 및 캐시 초기화
PcaVis::PcaVis(const CamCalib& cam, const SonarCalib& sonar) {
  SetCalib(cam, sonar);
  Reset();
}

// 카메라 회전·위치 및 초음파 설정 갱신
void PcaVis::SetCalib(const CamCalib& cam, const SonarCalib& sonar) {
  cam_ = cam;
  sonar_ = sonar;
  r_opt_to_base_ = cam_.OptToBaseRot();
  t_cam_base_ = cam_.cam_pos_m.cast<float>();
}

// 비전 관측 캐시를 기본 미검출 상태로 리셋
void PcaVis::Reset() {
  last_vis_ << 0.5f, 0.0f, 1.0f, 0.0f, 0.0f, -0.04f, 0.0f, 0.04f, 0.0f;
}

// 근접 사각지대(<35cm)에서 초음파 실측 거리로 상자 전후방 오차 보정
void PcaVis::FuseSonar(float us_range_m, bool us_valid) {
  if (!us_valid || !sonar_.enabled || !sonar_.close_fusion ||
      !std::isfinite(us_range_m)) {
    return;
  }
  if (us_range_m < static_cast<float>(sonar_.min_range_m) ||
      us_range_m >= 0.35f) {
    return;
  }

  const Eigen::Vector2f norm = last_vis_.segment<2>(2);
  if (norm.x() <= 0.85f) return;
  const float standoff = static_cast<float>(cam_.standoff_m);
  Eigen::Vector2f box_xy = last_vis_.segment<2>(0) + standoff * norm;
  if (std::abs(box_xy.y()) > 0.09f) return;

  const float wall_x =
      static_cast<float>(sonar_.us_pos_m.x()) + us_range_m * norm.x();
  box_xy.x() = wall_x + static_cast<float>(cam_.box_half_m) * norm.x();
  last_vis_.segment<2>(0) = box_xy - standoff * norm;
}

// 컬러 ROI로 재투영되는 깊이 픽셀들을 깊이 광학 좌표계 3D 점군으로 역투영
std::vector<Eigen::Vector3f> PcaVis::BackprojectOpt(const float* depth_m,
                                                    int width,
                                                    int height,
                                                    const Roi& color_roi,
                                                    const Intr& depth_intr,
                                                    const Intr& color_intr,
                                                    float color_off_x,
                                                    int stride) {
  std::vector<Eigen::Vector3f> pts;
  if (!depth_m || width <= 0 || height <= 0) return pts;
  if (color_roi.x2 <= color_roi.x1 || color_roi.y2 <= color_roi.y1) return pts;

  const int step = std::max(1, stride);
  const float off_x = -color_off_x;
  const float x1 = static_cast<float>(color_roi.x1);
  const float y1 = static_cast<float>(color_roi.y1);
  const float x2 = static_cast<float>(color_roi.x2);
  const float y2 = static_cast<float>(color_roi.y2);

  // 스테레오 오프셋과 내부 파라미터 비율을 반영해 깊이 탐색 범위 산출
  const int row_min = std::clamp(
      static_cast<int>(std::floor(
          depth_intr.cy + (y1 - color_intr.cy) * depth_intr.fy / color_intr.fy)),
      0, height);
  const int row_max = std::clamp(
      static_cast<int>(std::ceil(
          depth_intr.cy + (y2 - color_intr.cy) * depth_intr.fy / color_intr.fy)),
      row_min, height);
  const float col_l =
      depth_intr.cx + (x1 - color_intr.cx) * depth_intr.fx / color_intr.fx;
  const float col_r =
      depth_intr.cx + (x2 - color_intr.cx) * depth_intr.fx / color_intr.fx;
  const float shift_near = depth_intr.fx * off_x / 0.12f;
  const float shift_far = depth_intr.fx * off_x / 2.00f;
  const int col_min = std::clamp(
      static_cast<int>(
          std::floor(std::min(col_l - shift_near, col_l - shift_far))),
      0, width);
  const int col_max = std::clamp(
      static_cast<int>(
          std::ceil(std::max(col_r - shift_near, col_r - shift_far))),
      col_min, width);

  // 컬러 ROI 내부로 투영되는 깊이 점만 수집 (좌표는 camera_depth_frame 유지)
  pts.reserve(static_cast<size_t>((col_max - col_min) * (row_max - row_min) /
                                  (step * step) +
                                  1));
  for (int y = row_min; y < row_max; y += step) {
    const float* row = depth_m + static_cast<size_t>(y) * width;
    for (int x = col_min; x < col_max; x += step) {
      const float z = row[x];
      if (!std::isfinite(z) || z <= 0.12f || z >= 2.0f) continue;
      const float xd = (static_cast<float>(x) - depth_intr.cx) * z / depth_intr.fx;
      const float yd = (static_cast<float>(y) - depth_intr.cy) * z / depth_intr.fy;
      const float px = color_intr.cx + color_intr.fx * (xd + off_x) / z;
      const float py = color_intr.cy + color_intr.fy * yd / z;
      if (px < x1 || px >= x2 || py < y1 || py >= y2) continue;
      pts.emplace_back(xd, yd, z);
    }
  }
  return pts;
}

// 역투영된 광학 점군을 차체(base) 좌표계로 변환
std::vector<Eigen::Vector3f> PcaVis::BackprojectBase(const float* depth_m,
                                                     int width,
                                                     int height,
                                                     const Roi& color_roi,
                                                     const Intr& depth_intr,
                                                     const Intr& color_intr,
                                                     float color_off_x,
                                                     int stride) const {
  auto pts = BackprojectOpt(depth_m, width, height, color_roi, depth_intr,
                            color_intr, color_off_x, stride);
  for (auto& p : pts) p = r_opt_to_base_ * p + t_cam_base_;
  return pts;
}

// 상자 수직 벽면 슬라이스 PCA와 상단 퍽 PCA를 수행해 9차원 관측치 반환
Eigen::Matrix<float, 9, 1> PcaVis::Detect(const float* depth_m,
                                          int width,
                                          int height,
                                          const Intr& depth_intr,
                                          const Intr& color_intr,
                                          float color_off_x,
                                          const std::vector<Roi>& box_rois,
                                          const std::vector<Roi>& puck_rois,
                                          float us_range_m,
                                          bool us_valid) {
  if (box_rois.empty()) {
    last_vis_[8] = 0.0f;
    FuseSonar(us_range_m, us_valid);
    return last_vis_;
  }

  // 지면 위 수직 벽면(8mm ~ box_h-8mm) 점군만 분리해 PCA 수행
  const auto box_pts = BackprojectBase(depth_m, width, height, box_rois.front(),
                                       depth_intr, color_intr, color_off_x, 1);
  std::vector<Eigen::Vector3f> wall;
  wall.reserve(box_pts.size());
  const float base_z = static_cast<float>(cam_.base_z_m);
  const float box_h = static_cast<float>(cam_.box_height_m);
  for (const auto& p : box_pts) {
    const float z_gnd = p.z() + base_z;
    if (z_gnd > 0.008f && z_gnd < box_h - 0.008f) wall.push_back(p);
  }

  const BoxPca pca = FitBoxPca(wall, static_cast<float>(cam_.box_half_m));
  if (wall.size() < 6 || !pca.valid) {
    last_vis_[8] = 0.0f;
    FuseSonar(us_range_m, us_valid);
    return last_vis_;
  }

  // 상자 윗면(box_h+2mm 이상) 퍽 최대 2개 중심 추정 및 좌우 정렬
  std::vector<Eigen::Vector2f> pucks;
  for (size_t i = 0; i < std::min<size_t>(2, puck_rois.size()); ++i) {
    const auto p_pts = BackprojectBase(depth_m, width, height, puck_rois[i],
                                       depth_intr, color_intr, color_off_x, 1);
    std::vector<Eigen::Vector3f> top;
    for (const auto& p : p_pts) {
      if (p.z() + base_z > box_h + 0.002f) top.push_back(p);
    }
    Eigen::Vector2f p_xy;
    if (top.size() >= 4 &&
        FitPuckPca(top, static_cast<float>(cam_.puck_radius_m), &p_xy)) {
      pucks.push_back(p_xy);
    }
  }
  while (pucks.size() < 2) pucks.push_back(pca.box_xy);
  if (pucks[0].y() > pucks[1].y()) std::swap(pucks[0], pucks[1]);

  last_vis_ = PackVis(pca.box_xy, pca.norm_xy, pucks[0], pucks[1], 1.0f,
                      static_cast<float>(cam_.standoff_m));
  return last_vis_;
}

// 외부 3D 검출 결과(/detections_3d)로부터 9차원 비전 관측치 갱신
Eigen::Matrix<float, 9, 1> PcaVis::UpdateDet3d(
    const Eigen::Vector2f& box_xy,
    const Eigen::Vector2f& norm_xy,
    const std::vector<Eigen::Vector2f>& pucks_xy) {
  Eigen::Vector2f norm =
      norm_xy.norm() < 1e-6f ? Eigen::Vector2f::UnitX() : norm_xy.normalized();
  std::vector<Eigen::Vector2f> pair = pucks_xy;
  while (pair.size() < 2) pair.push_back(box_xy);
  if (pair[0].y() > pair[1].y()) std::swap(pair[0], pair[1]);

  last_vis_ = PackVis(box_xy, norm, pair[0], pair[1], 1.0f,
                      static_cast<float>(cam_.standoff_m));
  return last_vis_;
}

}  // namespace xsting_approach
