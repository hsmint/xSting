#ifndef XSTING_APPROACH_PCA_VIS_H_
#define XSTING_APPROACH_PCA_VIS_H_

#include <Eigen/Dense>
#include <vector>

#include "xsting_approach/calib.h"

namespace xsting_approach {

// 카메라 내부 파라미터 및 2D 관심 영역 구조체
struct Intr {
  float fx = 425.0f;
  float fy = 425.0f;
  float cx = 424.0f;
  float cy = 240.0f;
};

struct Roi {
  int x1 = 0;
  int y1 = 0;
  int x2 = 0;
  int y2 = 0;
};

struct BoxPca {
  bool valid = false;
  Eigen::Vector2f box_xy = Eigen::Vector2f::Zero();
  Eigen::Vector2f norm_xy = Eigen::Vector2f::UnitX();
};

// 상자 벽면·상단 퍽 PCA 추정 및 정책 관측 벡터 생성 함수
BoxPca FitBoxPca(const std::vector<Eigen::Vector3f>& wall_pts, float half_m);
bool FitPuckPca(const std::vector<Eigen::Vector3f>& top_pts,
                float radius_m,
                Eigen::Vector2f* puck_xy);
Eigen::Matrix<float, 9, 1> PackVis(const Eigen::Vector2f& box_xy,
                                   const Eigen::Vector2f& norm_xy,
                                   const Eigen::Vector2f& p1_xy,
                                   const Eigen::Vector2f& p2_xy,
                                   float vis_flag,
                                   float standoff_m);
Eigen::Vector4f MakeFineVis(const Eigen::Matrix<float, 9, 1>& vis,
                            float standoff_m,
                            const Eigen::Vector2f& cam_xy);
Eigen::Vector3f MakeSonarObs(float range_m,
                             bool valid,
                             float bumper_off_m,
                             float gap_m,
                             float min_range_m);

// 스테레오 오프셋 대응 D435 깊이 점군 PCA 및 초음파 근접 융합기
class PcaVis {
 public:
  explicit PcaVis(const CamCalib& cam = CamCalib{},
                  const SonarCalib& sonar = SonarCalib{});

  // 보정 파라미터 갱신 및 비전 캐시 초기화
  void SetCalib(const CamCalib& cam, const SonarCalib& sonar);
  void Reset();

  // 컬러 ROI와 스테레오 오프셋을 반영해 깊이 영상을 광학/차체 점군으로 역투영
  static std::vector<Eigen::Vector3f> BackprojectOpt(
      const float* depth_m,
      int width,
      int height,
      const Roi& color_roi,
      const Intr& depth_intr,
      const Intr& color_intr,
      float color_off_x = 0.0f,
      int stride = 1);
  std::vector<Eigen::Vector3f> BackprojectBase(const float* depth_m,
                                               int width,
                                               int height,
                                               const Roi& color_roi,
                                               const Intr& depth_intr,
                                               const Intr& color_intr,
                                               float color_off_x = 0.0f,
                                               int stride = 1) const;

  // 깊이 영상과 2D 검출 박스로부터 9차원 비전 관측치 산출
  Eigen::Matrix<float, 9, 1> Detect(const float* depth_m,
                                    int width,
                                    int height,
                                    const Intr& depth_intr,
                                    const Intr& color_intr,
                                    float color_off_x,
                                    const std::vector<Roi>& box_rois,
                                    const std::vector<Roi>& puck_rois,
                                    float us_range_m = 4.0f,
                                    bool us_valid = false);

  // 외부 3D 검출(/detections_3d) 폴백 갱신 및 초음파 근접 거리 융합
  Eigen::Matrix<float, 9, 1> UpdateDet3d(
      const Eigen::Vector2f& box_xy,
      const Eigen::Vector2f& norm_xy,
      const std::vector<Eigen::Vector2f>& pucks_xy);
  void FuseSonar(float us_range_m, bool us_valid);

  const Eigen::Matrix<float, 9, 1>& last_vis() const { return last_vis_; }

 private:
  CamCalib cam_;
  SonarCalib sonar_;
  Eigen::Matrix3f r_opt_to_base_ = Eigen::Matrix3f::Identity();
  Eigen::Vector3f t_cam_base_ = Eigen::Vector3f::Zero();
  Eigen::Matrix<float, 9, 1> last_vis_;
};

}  // namespace xsting_approach

#endif  // XSTING_APPROACH_PCA_VIS_H_
