#include "xsting_approach/foxglove_vis.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace xsting_approach {

namespace {

// 라디안-도 변환 상수 및 BGRA 색상 코드
constexpr float kRadToDeg = static_cast<float>(180.0 / M_PI);
constexpr uint32_t kColFloor = 0xFF60A5FAU;
constexpr uint32_t kColWall = 0xFFFF9F1CU;
constexpr uint32_t kColPuck = 0xFF2EC4B6U;

// 3차원 좌표를 geometry_msgs::Point로 변환
geometry_msgs::msg::Point MakePt(double x, double y, double z) {
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// 공통 헤더·색상·수명이 채워진 기본 마커 생성
visualization_msgs::msg::Marker InitMark(const rclcpp::Time& stamp,
                                         const char* ns,
                                         int id,
                                         int type,
                                         float r,
                                         float g,
                                         float b,
                                         float a) {
  visualization_msgs::msg::Marker m;
  m.header.stamp = stamp;
  m.header.frame_id = "base";
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  m.lifetime.nanosec = 400000000;
  return m;
}

// PointCloud2 단일 필드 메타데이터 생성
sensor_msgs::msg::PointField MakeField(const char* name,
                                       uint32_t offset,
                                       uint8_t dtype) {
  sensor_msgs::msg::PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = dtype;
  f.count = 1;
  return f;
}

// 깊이(0.12~1.5m)를 직관적인 난색-한색 RGB로 매핑
void ColorDepth(float z, uint8_t* rgb) {
  if (!std::isfinite(z) || z <= 0.05f) {
    rgb[0] = 28;
    rgb[1] = 30;
    rgb[2] = 36;
    return;
  }
  const float t = std::clamp((z - 0.12f) / 1.28f, 0.0f, 1.0f);
  rgb[0] = static_cast<uint8_t>(255.0f * std::clamp(1.5f - 2.0f * t, 0.0f, 1.0f));
  rgb[1] = static_cast<uint8_t>(
      255.0f * std::clamp(1.0f - 2.0f * std::abs(t - 0.45f), 0.0f, 1.0f));
  rgb[2] = static_cast<uint8_t>(255.0f * std::clamp(2.0f * t - 0.3f, 0.0f, 1.0f));
}

// 축소된 RGB 버퍼 위에 2D 사각형 테두리 그리기
void DrawRect(uint8_t* buf,
              int w,
              int h,
              const Roi& r,
              int scale,
              uint8_t cr,
              uint8_t cg,
              uint8_t cb) {
  const int x1 = std::clamp(r.x1 / scale, 0, w - 1);
  const int y1 = std::clamp(r.y1 / scale, 0, h - 1);
  const int x2 = std::clamp(r.x2 / scale, 0, w - 1);
  const int y2 = std::clamp(r.y2 / scale, 0, h - 1);
  if (x2 <= x1 || y2 <= y1) return;

  auto put = [&](int x, int y) {
    uint8_t* p = buf + (static_cast<size_t>(y) * w + x) * 3;
    p[0] = cr;
    p[1] = cg;
    p[2] = cb;
  };
  for (int x = x1; x <= x2; ++x) {
    put(x, y1);
    put(x, y2);
  }
  for (int y = y1; y <= y2; ++y) {
    put(x1, y);
    put(x2, y);
  }
}

}  // namespace

// Foxglove 시각화용 퍼블리셔 4종 생성
FoxgloveVis::FoxgloveVis(rclcpp::Node* node) : node_(node) {
  mark_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/approach/markers", 10);
  cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/approach/points", 5);
  metric_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/approach/metrics", 10);
  img_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
      "/approach/depth_preview", 5);
}

// 마커·수치·점군·프리뷰 구독 상태 확인
bool FoxgloveVis::Has3dSub() const {
  return mark_pub_->get_subscription_count() > 0 ||
         metric_pub_->get_subscription_count() > 0 || HasCloudSub();
}

bool FoxgloveVis::HasCloudSub() const {
  return cloud_pub_->get_subscription_count() > 0;
}

bool FoxgloveVis::HasImgSub() const {
  return img_pub_->get_subscription_count() > 0;
}

// 10Hz 주기로 실측 보정 3D 마커, 점군 및 오차 수치 발행
void FoxgloveVis::Publish(const rclcpp::Time& stamp, const VisFrame& f) {
  if (!Has3dSub()) return;
  if (last_pub_.nanoseconds() != 0 &&
      std::abs((stamp - last_pub_).seconds()) < 0.09) {
    return;
  }
  last_pub_ = stamp;

  const auto& cam = f.calib.cam;
  const auto& us = f.calib.sonar;
  const double gnd_z = -cam.base_z_m;
  const double box_sz = 2.0 * cam.box_half_m;
  const double ref_wall_x = cam.robot_front_x_m + f.target_gap_m;
  const double ref_box_x = ref_wall_x + cam.box_half_m;

  // 실측 오차 수치 계산 및 Plot용 배열 발행
  const float err_x_mm =
      f.box_valid ? static_cast<float>((f.box_xy.x() - ref_box_x) * 1000.0)
                  : 0.0f;
  const float err_y_mm = f.box_valid ? f.box_xy.y() * 1000.0f : 0.0f;
  const float yaw_deg =
      f.box_valid ? std::atan2(f.norm_xy.y(), f.norm_xy.x()) * kRadToDeg : 0.0f;
  const float bumper_off = static_cast<float>(us.BumperOff(cam.robot_front_x_m));
  const float us_gap_mm =
      f.us_valid ? (f.us_corr_m - bumper_off) * 1000.0f : 0.0f;
  const float us_err_mm =
      f.us_valid ? us_gap_mm - static_cast<float>(f.target_gap_m * 1000.0)
                 : 0.0f;
  const float imu_roll =
      std::atan2(-f.proj_grav_b.y(), -f.proj_grav_b.z()) * kRadToDeg;
  const float imu_pitch =
      std::atan2(f.proj_grav_b.x(),
                 std::hypot(f.proj_grav_b.y(), f.proj_grav_b.z())) *
      kRadToDeg;

  if (metric_pub_->get_subscription_count() > 0) {
    std_msgs::msg::Float32MultiArray met;
    met.data = {err_x_mm,
                err_y_mm,
                yaw_deg,
                us_gap_mm,
                us_err_mm,
                static_cast<float>(cam.base_z_m * 1000.0),
                imu_roll,
                imu_pitch};
    metric_pub_->publish(met);
  }

  // 3D 기준 평면·범퍼선·기준 상자·PCA 실측 상자·초음파·IMU·HUD 마커 조립
  if (mark_pub_->get_subscription_count() > 0) {
    using M = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray arr;

    M clr;
    clr.header.stamp = stamp;
    clr.header.frame_id = "base";
    clr.action = M::DELETEALL;
    arr.markers.push_back(clr);

    M floor_m = InitMark(stamp, "calib", 1, M::CUBE, 0.20f, 0.50f, 0.90f, 0.18f);
    floor_m.pose.position = MakePt(0.38, 0.0, gnd_z - 0.001);
    floor_m.scale.x = 0.80;
    floor_m.scale.y = 0.70;
    floor_m.scale.z = 0.002;
    arr.markers.push_back(floor_m);

    M bump_m =
        InitMark(stamp, "calib", 2, M::LINE_STRIP, 0.95f, 0.95f, 0.95f, 0.95f);
    bump_m.scale.x = 0.006;
    bump_m.points = {MakePt(cam.robot_front_x_m, -0.14, gnd_z + 0.015),
                     MakePt(cam.robot_front_x_m, 0.14, gnd_z + 0.015)};
    arr.markers.push_back(bump_m);

    M ref_m = InitMark(stamp, "calib", 3, M::CUBE, 0.15f, 0.92f, 0.38f, 0.24f);
    ref_m.pose.position = MakePt(ref_box_x, 0.0, gnd_z + 0.5 * cam.box_height_m);
    ref_m.scale.x = box_sz;
    ref_m.scale.y = box_sz;
    ref_m.scale.z = cam.box_height_m;
    arr.markers.push_back(ref_m);

    M ref_edge =
        InitMark(stamp, "calib", 4, M::LINE_STRIP, 0.20f, 0.98f, 0.42f, 0.95f);
    ref_edge.scale.x = 0.004;
    const double h = cam.box_half_m;
    const double z_top = gnd_z + cam.box_height_m;
    ref_edge.points = {MakePt(ref_box_x - h, -h, z_top),
                       MakePt(ref_box_x + h, -h, z_top),
                       MakePt(ref_box_x + h, h, z_top),
                       MakePt(ref_box_x - h, h, z_top),
                       MakePt(ref_box_x - h, -h, z_top)};
    arr.markers.push_back(ref_edge);

    if (f.box_valid) {
      const bool ok =
          std::hypot(err_x_mm, err_y_mm) < cam.pos_tol_m * 1000.0 &&
          std::abs(yaw_deg) < cam.yaw_tol_rad * kRadToDeg;
      M meas_m = InitMark(stamp, "calib", 5, M::CUBE, ok ? 0.10f : 1.00f,
                          ok ? 0.95f : 0.55f, ok ? 0.45f : 0.10f, 0.52f);
      meas_m.pose.position =
          MakePt(f.box_xy.x(), f.box_xy.y(), gnd_z + 0.5 * cam.box_height_m);
      const double yaw = std::atan2(f.norm_xy.y(), f.norm_xy.x());
      meas_m.pose.orientation.z = std::sin(0.5 * yaw);
      meas_m.pose.orientation.w = std::cos(0.5 * yaw);
      meas_m.scale.x = box_sz;
      meas_m.scale.y = box_sz;
      meas_m.scale.z = cam.box_height_m;
      arr.markers.push_back(meas_m);

      M norm_m = InitMark(stamp, "calib", 6, M::ARROW, 1.0f, 0.92f, 0.15f, 1.0f);
      norm_m.scale.x = 0.008;
      norm_m.scale.y = 0.016;
      norm_m.scale.z = 0.020;
      const Eigen::Vector2f wall_xy =
          f.box_xy - static_cast<float>(cam.box_half_m) * f.norm_xy;
      norm_m.points = {
          MakePt(wall_xy.x(), wall_xy.y(), gnd_z + 0.5 * cam.box_height_m),
          MakePt(f.box_xy.x(), f.box_xy.y(), gnd_z + 0.5 * cam.box_height_m)};
      arr.markers.push_back(norm_m);
    }

    if (f.us_valid) {
      const double hit_x = us.us_pos_m.x() + f.us_corr_m;
      M ray_m =
          InitMark(stamp, "calib", 7, M::LINE_STRIP, 0.0f, 0.90f, 1.0f, 0.95f);
      ray_m.scale.x = 0.005;
      ray_m.points = {MakePt(us.us_pos_m.x(), us.us_pos_m.y(), us.us_pos_m.z()),
                      MakePt(hit_x, us.us_pos_m.y(), us.us_pos_m.z())};
      arr.markers.push_back(ray_m);

      M hit_m = InitMark(stamp, "calib", 8, M::SPHERE, 0.0f, 0.95f, 1.0f, 1.0f);
      hit_m.pose.position = MakePt(hit_x, us.us_pos_m.y(), us.us_pos_m.z());
      hit_m.scale.x = 0.018;
      hit_m.scale.y = 0.018;
      hit_m.scale.z = 0.018;
      arr.markers.push_back(hit_m);
    }

    M imu_m = InitMark(stamp, "calib", 9, M::ARROW, 0.95f, 0.35f, 0.95f, 0.95f);
    imu_m.scale.x = 0.007;
    imu_m.scale.y = 0.014;
    imu_m.scale.z = 0.018;
    const Eigen::Vector3f g = 0.10f * f.proj_grav_b;
    imu_m.points = {MakePt(0.0, 0.0, 0.11),
                    MakePt(g.x(), g.y(), 0.11 + g.z())};
    arr.markers.push_back(imu_m);

    char hud[320];
    std::snprintf(
        hud, sizeof(hud),
        "기준간격 %.1fcm | PCA 오차 X:%+.1fmm Y:%+.1fmm Yaw:%+.2fdeg\n"
        "초음파 간격:%.1fmm (오차:%+.1fmm, bias:%+.1fmm)\n"
        "바닥 base_z:%.1fmm CamRPY:[%.1f, %.1f, %.1f]deg | IMU R:%+.1f P:%+.1fdeg",
        f.target_gap_m * 100.0, err_x_mm, err_y_mm, yaw_deg, us_gap_mm,
        us_err_mm, us.range_bias_m * 1000.0, cam.base_z_m * 1000.0,
        cam.cam_rpy_deg.x(), cam.cam_rpy_deg.y(), cam.cam_rpy_deg.z(), imu_roll,
        imu_pitch);

    M txt_m = InitMark(stamp, "calib", 10, M::TEXT_VIEW_FACING, 1.0f, 1.0f,
                       1.0f, 0.98f);
    txt_m.pose.position = MakePt(ref_box_x, 0.0, z_top + 0.11);
    txt_m.scale.z = 0.022;
    txt_m.text = hud;
    arr.markers.push_back(txt_m);

    mark_pub_->publish(std::move(arr));
  }

  // 바닥(하늘색)·상자벽면(주황색)·퍽(청록색) 통합 PointCloud2 발행
  if (HasCloudSub()) {
    auto sub_cnt = [](size_t n, size_t cap) { return std::min(n, cap); };
    const size_t nf = sub_cnt(f.floor_pts.size(), 600);
    const size_t nw = sub_cnt(f.wall_pts.size(), 500);
    const size_t np = sub_cnt(f.puck_pts.size(), 200);
    const size_t tot = nf + nw + np;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = "base";
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(tot);
    cloud.fields = {
        MakeField("x", 0, sensor_msgs::msg::PointField::FLOAT32),
        MakeField("y", 4, sensor_msgs::msg::PointField::FLOAT32),
        MakeField("z", 8, sensor_msgs::msg::PointField::FLOAT32),
        MakeField("rgb", 12, sensor_msgs::msg::PointField::UINT32),
    };
    cloud.is_bigendian = false;
    cloud.point_step = 16;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.is_dense = true;
    cloud.data.resize(cloud.row_step);

    uint8_t* dst = cloud.data.data();
    auto pack_pts = [&](const std::vector<Eigen::Vector3f>& src, size_t cnt,
                        uint32_t col) {
      for (size_t i = 0; i < cnt; ++i) {
        const auto& p = src[i * src.size() / cnt];
        std::memcpy(dst, p.data(), sizeof(float) * 3);
        std::memcpy(dst + 12, &col, sizeof(uint32_t));
        dst += 16;
      }
    };
    pack_pts(f.floor_pts, nf, kColFloor);
    pack_pts(f.wall_pts, nw, kColWall);
    pack_pts(f.puck_pts, np, kColPuck);
    cloud_pub_->publish(std::move(cloud));
  }
}

// 6Hz 주기로 축소된 깊이 컬러맵과 검출 ROI 프리뷰 영상 발행
void FoxgloveVis::PublishDepthImg(const rclcpp::Time& stamp,
                                  const float* depth_m,
                                  int width,
                                  int height,
                                  const std::vector<Roi>& box_rois,
                                  const std::vector<Roi>& puck_rois,
                                  const Roi* floor_roi) {
  if (!HasImgSub() || !depth_m || width <= 0 || height <= 0) return;
  if (last_img_.nanoseconds() != 0 &&
      std::abs((stamp - last_img_).seconds()) < 0.15) {
    return;
  }
  last_img_ = stamp;

  const int scale = std::max(1, width / 320);
  const int ow = width / scale;
  const int oh = height / scale;
  if (ow <= 0 || oh <= 0) return;

  sensor_msgs::msg::Image img;
  img.header.stamp = stamp;
  img.header.frame_id = "camera_depth_frame";
  img.width = static_cast<uint32_t>(ow);
  img.height = static_cast<uint32_t>(oh);
  img.encoding = "rgb8";
  img.step = static_cast<uint32_t>(ow * 3);
  img.data.resize(static_cast<size_t>(oh) * img.step);

  for (int y = 0; y < oh; ++y) {
    const float* row = depth_m + static_cast<size_t>(y * scale) * width;
    uint8_t* dst = img.data.data() + static_cast<size_t>(y) * img.step;
    for (int x = 0; x < ow; ++x) ColorDepth(row[x * scale], dst + x * 3);
  }

  uint8_t* raw = img.data.data();
  if (floor_roi) DrawRect(raw, ow, oh, *floor_roi, scale, 96, 165, 250);
  for (const auto& r : box_rois) {
    DrawRect(raw, ow, oh, r, scale, 255, 160, 28);
  }
  for (const auto& r : puck_rois) {
    DrawRect(raw, ow, oh, r, scale, 46, 196, 182);
  }
  img_pub_->publish(std::move(img));
}

}  // namespace xsting_approach
