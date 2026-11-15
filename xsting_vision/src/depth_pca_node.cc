#include "xsting_vision/depth_pca_node.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <cuda_buffer/cuda_buffer_api.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/exceptions.hpp>

namespace xsting_vision {

namespace {

// 쿼터니언을 3x3 회전 행렬로 변환
cv::Matx33d MakeRotMat(const geometry_msgs::msg::Quaternion& quat) {
  const double xx = quat.x * quat.x;
  const double yy = quat.y * quat.y;
  const double zz = quat.z * quat.z;
  const double xy = quat.x * quat.y;
  const double xz = quat.x * quat.z;
  const double yz = quat.y * quat.z;
  const double wx = quat.w * quat.x;
  const double wy = quat.w * quat.y;
  const double wz = quat.w * quat.z;
  return cv::Matx33d(1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
                     2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
                     2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy));
}

// 평행이동 메시지를 3차원 벡터로 변환
cv::Vec3d MakeTransVec(const geometry_msgs::msg::Vector3& vec) {
  return cv::Vec3d(vec.x, vec.y, vec.z);
}

// 오른손 정규직교 주축 3개를 쿼터니언으로 변환
geometry_msgs::msg::Quaternion MakeQuat(
    const std::array<cv::Vec3d, 3>& axes) {
  const double r00 = axes[0][0];
  const double r01 = axes[1][0];
  const double r02 = axes[2][0];
  const double r10 = axes[0][1];
  const double r11 = axes[1][1];
  const double r12 = axes[2][1];
  const double r20 = axes[0][2];
  const double r21 = axes[1][2];
  const double r22 = axes[2][2];

  geometry_msgs::msg::Quaternion quat;
  const double trace = r00 + r11 + r22;
  if (trace > 0.0) {
    const double scale = 2.0 * std::sqrt(trace + 1.0);
    quat.w = 0.25 * scale;
    quat.x = (r21 - r12) / scale;
    quat.y = (r02 - r20) / scale;
    quat.z = (r10 - r01) / scale;
  } else if (r00 > r11 && r00 > r22) {
    const double scale = 2.0 * std::sqrt(1.0 + r00 - r11 - r22);
    quat.w = (r21 - r12) / scale;
    quat.x = 0.25 * scale;
    quat.y = (r01 + r10) / scale;
    quat.z = (r02 + r20) / scale;
  } else if (r11 > r22) {
    const double scale = 2.0 * std::sqrt(1.0 + r11 - r00 - r22);
    quat.w = (r02 - r20) / scale;
    quat.x = (r01 + r10) / scale;
    quat.y = 0.25 * scale;
    quat.z = (r12 + r21) / scale;
  } else {
    const double scale = 2.0 * std::sqrt(1.0 + r22 - r00 - r11);
    quat.w = (r10 - r01) / scale;
    quat.x = (r02 + r20) / scale;
    quat.y = (r12 + r21) / scale;
    quat.z = 0.25 * scale;
  }
  return quat;
}

// PCA 주축 부호를 일관되게 고정하고 오른손 직교 좌표계로 보정
bool StabilizeAxes(cv::Vec3d& axis0, cv::Vec3d& axis1, cv::Vec3d& axis2) {
  if (std::abs(axis0[2]) >= 0.5) {
    if (axis0[2] < 0.0) {
      axis0 = -axis0;
    }
  } else if (std::abs(axis0[1]) >= std::abs(axis0[0])) {
    if (axis0[1] < 0.0) {
      axis0 = -axis0;
    }
  } else if (axis0[0] < 0.0) {
    axis0 = -axis0;
  }

  if (std::abs(axis2[0]) >= 0.1) {
    if (axis2[0] > 0.0) {
      axis2 = -axis2;
    }
  } else if (axis2[2] < 0.0) {
    axis2 = -axis2;
  }

  axis1 = axis2.cross(axis0);
  const double norm1 = cv::norm(axis1);
  if (norm1 < 1e-6) {
    return false;
  }
  axis1 /= norm1;
  axis2 = axis0.cross(axis1);
  return true;
}

// PointCloud2 단일 필드 정보 생성
sensor_msgs::msg::PointField MakeField(const std::string& name,
                                       uint32_t offset, uint8_t datatype) {
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}

}  // namespace

// YAML 설정을 읽고 카메라별 구독과 통합 퍼블리셔 초기화
DepthPcaNode::DepthPcaNode(const rclcpp::NodeOptions& opts)
    : Node("depth_pca", AddYamlParams(opts)),
      tf_buf_(get_clock()),
      tf_sub_(tf_buf_, *this, false) {
  cfg_ = LoadCfg();
  cudaStreamCreateWithFlags(&cuda_stream_, cudaStreamNonBlocking);
  det3d_pub_ = create_publisher<Det3dArray>(cfg_.output_topic, 10);
  pts_pub_ = create_publisher<CloudMsg>(cfg_.output_topic + "/points", 10);

  const auto det_topics = declare_parameter<std::vector<std::string>>(
      "detection_topics", std::vector<std::string>{});
  const auto depth_topics = declare_parameter<std::vector<std::string>>(
      "depth_topics", std::vector<std::string>{});
  for (size_t i = 0; i < det_topics.size(); ++i) {
    const std::string depth_topic =
        i < depth_topics.size() ? depth_topics[i] : "";
    AddStream(det_topics[i], depth_topic);
  }
}

// 구독 해제 후 CUDA stream 정리
DepthPcaNode::~DepthPcaNode() {
  streams_.clear();
  cudaStreamSynchronize(cuda_stream_);
  cudaStreamDestroy(cuda_stream_);
}

// YAML 파일 파라미터를 NodeOptions 앞쪽에 추가
rclcpp::NodeOptions DepthPcaNode::AddYamlParams(
    const rclcpp::NodeOptions& opts) {
  std::string cfg_path =
      (ament_index_cpp::get_package_share_path("xsting_vision") / "config" /
       "depth_pca.yaml")
          .string();
  for (const rclcpp::Parameter& param : opts.parameter_overrides()) {
    if (param.get_name() == "config_file") {
      cfg_path = param.as_string();
    }
  }

  std::vector<rclcpp::Parameter> yaml_params;
  for (const auto& entry : rclcpp::parameter_map_from_yaml_file(cfg_path)) {
    yaml_params.insert(yaml_params.end(), entry.second.begin(),
                       entry.second.end());
  }

  rclcpp::NodeOptions out = opts;
  auto& overrides = out.parameter_overrides();
  overrides.insert(overrides.begin(), yaml_params.begin(), yaml_params.end());
  return out;
}

// 전경 필터링 및 점군 통합 PCA 파라미터 읽기
DepthPcaCfg DepthPcaNode::LoadCfg() {
  DepthPcaCfg cfg;
  cfg.target_frame = declare_parameter<std::string>("target_frame", "base");
  cfg.output_topic =
      declare_parameter<std::string>("output_topic", "/detections_3d");
  cfg.class_names = declare_parameter<std::vector<std::string>>(
      "class_names", std::vector<std::string>{});
  cfg.color_offset_x = declare_parameter<double>("color_offset_x", -0.015);
  cfg.min_depth = declare_parameter<double>("min_depth", 0.08);
  cfg.max_depth = declare_parameter<double>("max_depth", 2.50);
  cfg.roi_scale = declare_parameter<double>("roi_scale", 0.90);
  cfg.depth_tol = declare_parameter<double>("depth_tol", 0.08);
  cfg.puck_depth_tol = declare_parameter<double>("puck_depth_tol", 0.035);
  cfg.box_top_z = declare_parameter<double>("box_top_z", 0.0487);
  cfg.support_margin = declare_parameter<double>("support_margin", 0.004);
  cfg.min_points = declare_parameter<int>("min_points", 20);
  cfg.max_points = declare_parameter<int>("max_points", 400);
  cfg.merge_dist = declare_parameter<double>("merge_dist", 0.10);
  cfg.merge_timeout = declare_parameter<double>("merge_timeout", 0.10);
  return cfg;
}

// 카메라 한 대의 2D 검출·깊이·카메라정보 구독 생성
void DepthPcaNode::AddStream(const std::string& det_topic,
                             const std::string& depth_topic) {
  if (depth_topic.empty()) {
    return;
  }
  auto stream = std::make_unique<PcaStream>();
  stream->prefix = det_topic.substr(0, det_topic.rfind('/'));
  PcaStream* ptr = stream.get();
  const auto qos = rclcpp::SensorDataQoS();

  rclcpp::SubscriptionOptions any_opts;
  any_opts.acceptable_buffer_backends = "any";

  // 깊이 영상 및 컬러·깊이 내부 파라미터 캐시 구독
  stream->depth_sub = create_subscription<ImageMsg>(
      depth_topic, qos,
      [ptr](const ImageMsg::ConstSharedPtr& msg) { ptr->latest_depth = msg; },
      any_opts);
  stream->color_info_sub = create_subscription<CamInfoMsg>(
      stream->prefix + "/camera_info", qos,
      [ptr](const CamInfoMsg::ConstSharedPtr& msg) { ptr->color_info = msg; });
  const std::string depth_prefix =
      depth_topic.substr(0, depth_topic.rfind('/'));
  stream->depth_info_sub = create_subscription<CamInfoMsg>(
      depth_prefix + "/camera_info", qos,
      [ptr](const CamInfoMsg::ConstSharedPtr& msg) { ptr->depth_info = msg; });

  // YOLO 2D 검출 수신 콜백 등록
  stream->det_sub = create_subscription<Det2dArray>(
      det_topic, 10,
      [this, ptr](const Det2dArray::ConstSharedPtr& msg) { OnDet(*ptr, msg); });

  streams_.push_back(std::move(stream));
}

// YOLO 2D 검출 수신 시 전경 점군 추출·통합 PCA 및 결과 발행
void DepthPcaNode::OnDet(PcaStream& stream,
                         const Det2dArray::ConstSharedPtr& det_msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!stream.color_info) {
    return;
  }

  // 카메라 좌표계에서 기준 좌표계(base)로의 TF 조회
  TfMsg tf_msg;
  try {
    tf_msg = tf_buf_.lookupTransform(cfg_.target_frame,
                                     stream.color_info->header.frame_id,
                                     tf2::TimePointZero);
  } catch (const tf2::TransformException&) {
    return;
  }

  // 유효 깊이 영상이 있으면 YOLO 박스별 전경 점군 갱신
  const rclcpp::Time stamp(det_msg->header.stamp);
  stream.clouds.clear();
  stream.cloud_stamp = stamp;
  if (stream.latest_depth && stream.depth_info) {
    const double depth_age =
        std::abs((stamp - rclcpp::Time(stream.latest_depth->header.stamp))
                     .seconds());
    if (depth_age <= 0.2 && !det_msg->detections.empty() &&
        CopyDepth(*stream.latest_depth)) {
      stream.clouds =
          ExtractClouds(stream, *det_msg, *stream.latest_depth, tf_msg);
    }
  }

  // 활성 카메라 점군 병합 PCA 후 통합 3D 검출 및 전경 점군 발행
  const std::vector<PcaBox> fused = FuseAndFit(stream, stamp);
  det3d_pub_->publish(BuildDet3d(stamp, fused));
  if (pts_pub_->get_subscription_count() > 0) {
    PubCloud(stream, stamp);
  }
}

// GPU 16UC1 깊이 버퍼를 호스트 작업 버퍼로 복사
bool DepthPcaNode::CopyDepth(const ImageMsg& depth_msg) {
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

// 카메라의 모든 YOLO 박스에서 전경 점군을 추출해 기준 좌표계로 변환
std::vector<ObjCloud> DepthPcaNode::ExtractClouds(const PcaStream& stream,
                                                  const Det2dArray& dets,
                                                  const ImageMsg& depth_msg,
                                                  const TfMsg& tf_msg) {
  const cv::Matx33d rot = MakeRotMat(tf_msg.transform.rotation);
  const cv::Vec3d trans = MakeTransVec(tf_msg.transform.translation);
  const size_t max_pts = static_cast<size_t>(std::max(1, cfg_.max_points));

  std::vector<ObjCloud> clouds;
  for (const Det2dMsg& det : dets.detections) {
    if (det.results.empty()) {
      continue;
    }
    const std::string cls_id = GetClassName(det.results[0].hypothesis.class_id);
    const bool is_box = (cls_id.find("box") != std::string::npos);
    const std::vector<cv::Vec3d> cam_pts =
        ExtractBoxPts(stream, depth_msg, det, is_box);
    if (static_cast<int>(cam_pts.size()) < cfg_.min_points) {
      continue;
    }

    // 표면 가중치 균형과 연산 속도를 위해 최대 점 개수로 균등 샘플링
    const size_t cnt = std::min(cam_pts.size(), max_pts);
    ObjCloud cloud;
    cloud.cls_id = cls_id;
    cloud.score = det.results[0].hypothesis.score;
    cloud.cam_pos = trans;
    cloud.pts.reserve(cnt);

    cv::Vec3d sum(0.0, 0.0, 0.0);
    for (size_t idx = 0; idx < cnt; ++idx) {
      const size_t src_idx = idx * cam_pts.size() / cnt;
      const cv::Vec3d pt_base = rot * cam_pts[src_idx] + trans;
      cloud.pts.push_back(pt_base);
      sum += pt_base;
    }
    cloud.centroid = sum * (1.0 / static_cast<double>(cnt));
    clouds.push_back(std::move(cloud));
  }
  return clouds;
}

// 단일 2D 박스 내부로 재투영되는 전경 3D 점군(광학 좌표계) 추출
std::vector<cv::Vec3d> DepthPcaNode::ExtractBoxPts(const PcaStream& stream,
                                                   const ImageMsg& depth_msg,
                                                   const Det2dMsg& det,
                                                   bool is_box) const {
  const auto& kc = stream.color_info->k;
  const auto& kd = stream.depth_info->k;
  if (kc[0] <= 0.0 || kc[4] <= 0.0 || kd[0] <= 0.0 || kd[4] <= 0.0) {
    return {};
  }

  // 축소된 컬러 박스 범위와 스테레오 오프셋을 반영한 깊이 탐색 영역 계산
  const double cx = det.bbox.center.position.x;
  const double cy = det.bbox.center.position.y;
  const double roi_s = is_box ? cfg_.roi_scale : std::min(cfg_.roi_scale, 0.84);
  const double half_w = 0.5 * det.bbox.size_x * roi_s;
  const double half_h = 0.5 * det.bbox.size_y * roi_s;
  if (half_w <= 1.0 || half_h <= 1.0) {
    return {};
  }
  const double x_min = cx - half_w;
  const double x_max = cx + half_w;
  const double y_min = cy - half_h;
  const double y_max = cy + half_h;
  const double core_w = 0.5 * half_w;
  const double core_h = 0.5 * half_h;

  const int width = static_cast<int>(depth_msg.width);
  const int height = static_cast<int>(depth_msg.height);
  const double offset_x = -cfg_.color_offset_x;
  const int row_min = std::clamp(
      static_cast<int>(std::floor(kd[5] + (y_min - kc[5]) * kd[4] / kc[4])), 0,
      height - 1);
  const int row_max = std::clamp(
      static_cast<int>(std::ceil(kd[5] + (y_max - kc[5]) * kd[4] / kc[4])), 0,
      height - 1);
  const double col_l = kd[2] + (x_min - kc[2]) * kd[0] / kc[0];
  const double col_r = kd[2] + (x_max - kc[2]) * kd[0] / kc[0];
  const double shift_near = kd[0] * offset_x / cfg_.min_depth;
  const double shift_far = kd[0] * offset_x / cfg_.max_depth;
  const int col_min = std::clamp(
      static_cast<int>(
          std::floor(std::min(col_l - shift_near, col_l - shift_far))),
      0, width - 1);
  const int col_max = std::clamp(
      static_cast<int>(
          std::ceil(std::max(col_r - shift_near, col_r - shift_far))),
      0, width - 1);

  // 컬러 박스 안으로 재투영되는 유효 3D 점과 중심부 깊이 수집
  std::vector<cv::Vec3d> cand_pts;
  std::vector<double> core_depths;
  for (int row = row_min; row <= row_max; ++row) {
    const size_t row_off = static_cast<size_t>(row) * width;
    for (int col = col_min; col <= col_max; ++col) {
      const uint16_t raw_mm = depth_buf_[row_off + static_cast<size_t>(col)];
      if (raw_mm == 0) {
        continue;
      }
      const double z = raw_mm * 0.001;
      if (z < cfg_.min_depth || z > cfg_.max_depth) {
        continue;
      }
      const double x = (col - kd[2]) * z / kd[0] + offset_x;
      const double y = (row - kd[5]) * z / kd[4];
      const double proj_x = kc[2] + kc[0] * x / z;
      const double proj_y = kc[5] + kc[4] * y / z;
      if (proj_x < x_min || proj_x > x_max || proj_y < y_min ||
          proj_y > y_max) {
        continue;
      }
      cand_pts.emplace_back(x, y, z);
      if (std::abs(proj_x - cx) <= core_w && std::abs(proj_y - cy) <= core_h) {
        core_depths.push_back(z);
      }
    }
  }
  if (static_cast<int>(cand_pts.size()) < cfg_.min_points) {
    return {};
  }

  // 중심부 중앙 깊이(부족 시 전체 하위 30% 깊이)를 기준 전경 깊이로 선택
  double ref_z = 0.0;
  if (static_cast<int>(core_depths.size()) >= std::max(5, cfg_.min_points / 2)) {
    const size_t mid = core_depths.size() / 2;
    std::nth_element(core_depths.begin(), core_depths.begin() + mid,
                     core_depths.end());
    ref_z = core_depths[mid];
  } else {
    std::vector<double> all_z;
    all_z.reserve(cand_pts.size());
    for (const cv::Vec3d& pt : cand_pts) {
      all_z.push_back(pt[2]);
    }
    const size_t pct = all_z.size() * 30 / 100;
    std::nth_element(all_z.begin(), all_z.begin() + pct, all_z.end());
    ref_z = all_z[pct];
  }

  // 배경·바닥 노이즈를 제거한 전경 점군 필터링
  const double tol = is_box ? cfg_.depth_tol : cfg_.puck_depth_tol;
  std::vector<cv::Vec3d> fg_pts;
  fg_pts.reserve(cand_pts.size());
  for (const cv::Vec3d& pt : cand_pts) {
    if (std::abs(pt[2] - ref_z) <= tol) {
      fg_pts.push_back(pt);
    }
  }
  return fg_pts;
}

// 활성 카메라들의 전경 점군을 병합해 통합 PCA 박스 생성
std::vector<PcaBox> DepthPcaNode::FuseAndFit(
    const PcaStream& cur_stream, const rclcpp::Time& stamp) const {
  // 유효 시간차 이내의 활성 스트림 목록 구성
  std::vector<const PcaStream*> active;
  for (const auto& stream : streams_) {
    if (stream.get() == &cur_stream) {
      active.push_back(stream.get());
      continue;
    }
    if (stream->clouds.empty() || stream->cloud_stamp.nanoseconds() == 0) {
      continue;
    }
    if (std::abs((stamp - stream->cloud_stamp).seconds()) <=
        cfg_.merge_timeout) {
      active.push_back(stream.get());
    }
  }

  // 상자 검출 점군이 있으면 실측 상판 높이를 지지 평면 기준값으로 갱신
  double support_z = cfg_.box_top_z;
  for (const PcaStream* st : active) {
    for (const ObjCloud& cl : st->clouds) {
      if (cl.cls_id.find("box") == std::string::npos || cl.pts.size() < 10) {
        continue;
      }
      std::vector<double> zs;
      zs.reserve(cl.pts.size());
      for (const cv::Vec3d& pt : cl.pts) zs.push_back(pt[2]);
      const size_t hi = zs.size() * 95 / 100;
      std::nth_element(zs.begin(), zs.begin() + hi, zs.end());
      if (std::abs(zs[hi] - cfg_.box_top_z) <= 0.045) {
        support_z = zs[hi];
      }
    }
  }

  // 다른 카메라의 동일 클래스·근접 점군을 하나로 합쳐 통합 PCA 수행
  std::vector<std::vector<bool>> used(active.size());
  for (size_t i = 0; i < active.size(); ++i) {
    used[i].assign(active[i]->clouds.size(), false);
  }

  std::vector<PcaBox> out;
  for (size_t s_idx = 0; s_idx < active.size(); ++s_idx) {
    const PcaStream* base_stream = active[s_idx];
    for (size_t c_idx = 0; c_idx < base_stream->clouds.size(); ++c_idx) {
      if (used[s_idx][c_idx]) {
        continue;
      }
      used[s_idx][c_idx] = true;
      const ObjCloud& base_cloud = base_stream->clouds[c_idx];
      std::vector<cv::Vec3d> merged = base_cloud.pts;
      double best_score = base_cloud.score;
      bool multi_view = false;

      for (size_t o_idx = s_idx + 1; o_idx < active.size(); ++o_idx) {
        const PcaStream* other_stream = active[o_idx];
        double best_dist = cfg_.merge_dist;
        int match_idx = -1;
        for (size_t m_idx = 0; m_idx < other_stream->clouds.size(); ++m_idx) {
          if (used[o_idx][m_idx]) {
            continue;
          }
          const ObjCloud& cand = other_stream->clouds[m_idx];
          if (cand.cls_id != base_cloud.cls_id) {
            continue;
          }
          const double dist = cv::norm(base_cloud.centroid - cand.centroid);
          if (dist <= best_dist) {
            best_dist = dist;
            match_idx = static_cast<int>(m_idx);
          }
        }
        if (match_idx >= 0) {
          const size_t idx = static_cast<size_t>(match_idx);
          used[o_idx][idx] = true;
          const ObjCloud& matched = other_stream->clouds[idx];
          merged.insert(merged.end(), matched.pts.begin(), matched.pts.end());
          best_score = std::max(best_score, matched.score);
          multi_view = true;
        }
      }

      cv::Vec3d view_dir(base_cloud.centroid[0] - base_cloud.cam_pos[0],
                         base_cloud.centroid[1] - base_cloud.cam_pos[1], 0.0);
      const double v_norm = cv::norm(view_dir);
      if (v_norm > 1e-6) {
        view_dir /= v_norm;
      } else {
        view_dir = cv::Vec3d(1.0, 0.0, 0.0);
      }

      const PcaBox box = FitPca(merged, base_cloud.cls_id, best_score,
                                view_dir, multi_view, support_z);
      if (box.valid) {
        out.push_back(box);
      }
    }
  }
  return out;
}

// 상자 평면 혼입 제거 및 전면 반쪽 껍질의 체적 중심 보정을 포함한 OBB 산출
PcaBox DepthPcaNode::FitPca(const std::vector<cv::Vec3d>& pts,
                            const std::string& cls_id, double score,
                            const cv::Vec3d& view_dir, bool multi_view,
                            double support_z) const {
  PcaBox box;
  if (static_cast<int>(pts.size()) < cfg_.min_points) {
    return box;
  }

  // 종이 상자는 기존 3차원 전체 PCA로 중심과 외곽 크기 산출
  if (cls_id.find("box") != std::string::npos) {
    cv::Mat mat(static_cast<int>(pts.size()), 3, CV_64F);
    for (size_t i = 0; i < pts.size(); ++i) {
      const int row = static_cast<int>(i);
      mat.at<double>(row, 0) = pts[i][0];
      mat.at<double>(row, 1) = pts[i][1];
      mat.at<double>(row, 2) = pts[i][2];
    }
    const cv::PCA pca(mat, cv::Mat(), cv::PCA::DATA_AS_ROW);
    if (pca.eigenvectors.rows < 3) {
      return box;
    }

    const cv::Vec3d mean(pca.mean.at<double>(0, 0), pca.mean.at<double>(0, 1),
                         pca.mean.at<double>(0, 2));
    cv::Vec3d axis0(pca.eigenvectors.at<double>(0, 0),
                    pca.eigenvectors.at<double>(0, 1),
                    pca.eigenvectors.at<double>(0, 2));
    cv::Vec3d axis1;
    cv::Vec3d axis2(pca.eigenvectors.at<double>(2, 0),
                    pca.eigenvectors.at<double>(2, 1),
                    pca.eigenvectors.at<double>(2, 2));
    if (!StabilizeAxes(axis0, axis1, axis2)) {
      return box;
    }
    box.axes = {axis0, axis1, axis2};
    box.center = mean;

    std::vector<double> proj(pts.size());
    const size_t lo_idx = pts.size() * 2 / 100;
    const size_t hi_idx = pts.size() - 1 - lo_idx;
    for (size_t axis_idx = 0; axis_idx < 3; ++axis_idx) {
      const cv::Vec3d& axis = box.axes[axis_idx];
      for (size_t i = 0; i < pts.size(); ++i) {
        proj[i] = (pts[i] - mean).dot(axis);
      }
      std::nth_element(proj.begin(), proj.begin() + lo_idx, proj.end());
      const double lo = proj[lo_idx];
      std::nth_element(proj.begin(), proj.begin() + hi_idx, proj.end());
      const double hi = proj[hi_idx];
      box.center += 0.5 * (lo + hi) * axis;
      box.size[static_cast<int>(axis_idx)] = std::max(0.01, hi - lo);
    }
    box.valid = true;
    box.cls_id = cls_id;
    box.score = score;
    return box;
  }

  // 퍽 점군에서 상자 윗면 바닥 노이즈를 제거해 순수 물체 점만 분리
  std::vector<cv::Vec3d> clean;
  clean.reserve(pts.size());
  for (const cv::Vec3d& pt : pts) {
    if (pt[2] > support_z + cfg_.support_margin) {
      clean.push_back(pt);
    }
  }
  const std::vector<cv::Vec3d>& work =
      (static_cast<int>(clean.size()) >= std::max(8, cfg_.min_points / 2))
          ? clean
          : pts;

  // 수평면(XY) 2D 공분산 고유분해로 L자 부분 점군에서도 주축 기울림 방지
  double mx = 0.0;
  double my = 0.0;
  std::vector<double> zs(work.size());
  for (size_t i = 0; i < work.size(); ++i) {
    mx += work[i][0];
    my += work[i][1];
    zs[i] = work[i][2];
  }
  mx /= static_cast<double>(work.size());
  my /= static_cast<double>(work.size());

  double cxx = 0.0;
  double cxy = 0.0;
  double cyy = 0.0;
  for (const cv::Vec3d& pt : work) {
    const double dx = pt[0] - mx;
    const double dy = pt[1] - my;
    cxx += dx * dx;
    cxy += dx * dy;
    cyy += dy * dy;
  }
  const double yaw_2d = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
  cv::Vec3d h0(std::cos(yaw_2d), std::sin(yaw_2d), 0.0);
  if (h0[0] < 0.0) h0 = -h0;
  cv::Vec3d h1(-h0[1], h0[0], 0.0);

  const size_t lo_i = work.size() * 2 / 100;
  const size_t hi_i = work.size() - 1 - lo_i;
  std::nth_element(zs.begin(), zs.begin() + hi_i, zs.end());
  const double z_hi = zs[hi_i];

  auto span_along = [&](const cv::Vec3d& ax, double& lo, double& hi) {
    std::vector<double> pr(work.size());
    for (size_t i = 0; i < work.size(); ++i) {
      pr[i] = (work[i][0] - mx) * ax[0] + (work[i][1] - my) * ax[1];
    }
    std::nth_element(pr.begin(), pr.begin() + lo_i, pr.end());
    lo = pr[lo_i];
    std::nth_element(pr.begin(), pr.begin() + hi_i, pr.end());
    hi = pr[hi_i];
    return hi - lo;
  };

  double lo0 = 0.0;
  double hi0 = 0.0;
  double lo1 = 0.0;
  double hi1 = 0.0;
  const double sp0 = span_along(h0, lo0, hi0);
  const double sp1 = span_along(h1, lo1, hi1);

  const bool is_cyl = (cls_id.find("cylinder") != std::string::npos);
  const bool is_lying =
      !is_cyl && ((z_hi - support_z) < 0.045 || sp0 > 0.044);

  // 단안 깊이 카메라의 전면 껍질 편향을 시선 방향 두께 보정으로 복원
  cv::Vec3d center(mx, my, 0.0);
  if (is_cyl) {
    const cv::Vec3d hv = view_dir;
    const cv::Vec3d hl(-hv[1], hv[0], 0.0);
    double lov = 0.0;
    double hiv = 0.0;
    double lol = 0.0;
    double hil = 0.0;
    const double spv = span_along(hv, lov, hiv);
    span_along(hl, lol, hil);
    const double comp =
        multi_view ? 0.0 : 0.5 * std::clamp(0.030 - spv, 0.0, 0.016);
    center += (0.5 * (lov + hiv) + comp) * hv + (0.5 * (lol + hil)) * hl;
    center[2] = std::max(support_z + 0.030, z_hi - 0.030);
    box.axes = {cv::Vec3d(1.0, 0.0, 0.0), cv::Vec3d(0.0, 1.0, 0.0),
                cv::Vec3d(0.0, 0.0, 1.0)};
    box.size = cv::Vec3d(0.030, 0.030, 0.060);
  } else {
    const double exp0 = is_lying ? 0.060 : 0.030;
    const double exp1 = 0.030;
    auto shift_axis = [&](const cv::Vec3d& ax, double lo, double hi,
                          double sp, double exp_len) {
      double mid = 0.5 * (lo + hi);
      if (!multi_view) {
        const double dv = ax.dot(view_dir);
        if (std::abs(dv) > 0.30 && sp < 0.88 * exp_len) {
          const double sgn = (dv >= 0.0) ? 1.0 : -1.0;
          mid += sgn * 0.5 *
                 std::clamp(exp_len - sp, 0.0, 0.55 * exp_len) * std::abs(dv);
        }
      }
      return mid;
    };
    center += shift_axis(h0, lo0, hi0, sp0, exp0) * h0 +
              shift_axis(h1, lo1, hi1, sp1, exp1) * h1;
    center[2] = is_lying ? std::max(support_z + 0.015, z_hi - 0.015)
                         : std::max(support_z + 0.030, z_hi - 0.030);
    box.axes = {h0, h1, cv::Vec3d(0.0, 0.0, 1.0)};
    box.size = is_lying ? cv::Vec3d(0.060, 0.030, 0.030)
                        : cv::Vec3d(0.030, 0.030, 0.060);
  }
  box.center = center;

  box.valid = true;
  box.cls_id = cls_id;
  box.score = score;
  return box;
}

// 통합 PCA 박스 목록을 Detection3DArray 메시지로 변환
Det3dArray DepthPcaNode::BuildDet3d(const rclcpp::Time& stamp,
                                    const std::vector<PcaBox>& boxes) const {
  Det3dArray out;
  out.header.stamp = stamp;
  out.header.frame_id = cfg_.target_frame;
  for (const PcaBox& box : boxes) {
    if (!box.valid) {
      continue;
    }
    vision_msgs::msg::Detection3D det3d;
    det3d.header = out.header;
    det3d.bbox.center.position.x = box.center[0];
    det3d.bbox.center.position.y = box.center[1];
    det3d.bbox.center.position.z = box.center[2];
    det3d.bbox.center.orientation = MakeQuat(box.axes);
    det3d.bbox.size.x = box.size[0];
    det3d.bbox.size.y = box.size[1];
    det3d.bbox.size.z = box.size[2];

    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.class_id = box.cls_id;
    hyp.hypothesis.score = box.score;
    hyp.pose.pose = det3d.bbox.center;
    det3d.results.push_back(hyp);
    out.detections.push_back(det3d);
  }
  return out;
}

// 활성 카메라들의 전경 3D 점군을 카메라별 색상으로 구분해 발행
void DepthPcaNode::PubCloud(const PcaStream& cur_stream,
                            const rclcpp::Time& stamp) {
  using sensor_msgs::msg::PointField;
  constexpr std::array<uint32_t, 3> kCamColors = {
      0xFF00DCFFu,
      0xFFFFD200u,
      0xFFFF50FFu,
  };

  // 유효 시간차 이내의 전체 전경 점 개수 집계
  size_t total_pts = 0;
  for (const auto& stream : streams_) {
    const bool is_cur = stream.get() == &cur_stream;
    const bool is_fresh =
        !stream->clouds.empty() && stream->cloud_stamp.nanoseconds() != 0 &&
        std::abs((stamp - stream->cloud_stamp).seconds()) <=
            cfg_.merge_timeout;
    if (!is_cur && !is_fresh) {
      continue;
    }
    for (const ObjCloud& cloud : stream->clouds) {
      total_pts += cloud.pts.size();
    }
  }

  // PointCloud2 헤더·필드 설정 및 좌표·색상 버퍼 기록
  CloudMsg msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = cfg_.target_frame;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(total_pts);
  msg.fields = {
      MakeField("x", 0, PointField::FLOAT32),
      MakeField("y", 4, PointField::FLOAT32),
      MakeField("z", 8, PointField::FLOAT32),
      MakeField("rgb", 12, PointField::UINT32),
  };
  msg.is_bigendian = false;
  msg.point_step = 16;
  msg.row_step = msg.point_step * msg.width;
  msg.is_dense = true;
  msg.data.resize(msg.row_step);

  uint8_t* ptr = msg.data.data();
  for (size_t s_idx = 0; s_idx < streams_.size(); ++s_idx) {
    const PcaStream* stream = streams_[s_idx].get();
    const bool is_cur = stream == &cur_stream;
    const bool is_fresh =
        !stream->clouds.empty() && stream->cloud_stamp.nanoseconds() != 0 &&
        std::abs((stamp - stream->cloud_stamp).seconds()) <=
            cfg_.merge_timeout;
    if (!is_cur && !is_fresh) {
      continue;
    }
    const uint32_t rgb = kCamColors[std::min(s_idx, kCamColors.size() - 1)];
    for (const ObjCloud& cloud : stream->clouds) {
      for (const cv::Vec3d& pt : cloud.pts) {
        const float coords[3] = {
            static_cast<float>(pt[0]),
            static_cast<float>(pt[1]),
            static_cast<float>(pt[2]),
        };
        std::memcpy(ptr, coords, sizeof(coords));
        std::memcpy(ptr + 12, &rgb, sizeof(rgb));
        ptr += msg.point_step;
      }
    }
  }
  pts_pub_->publish(std::move(msg));
}

// 숫자 문자열 클래스 ID를 설정된 클래스 이름으로 변환
std::string DepthPcaNode::GetClassName(const std::string& cls_id) const {
  if (cls_id.empty()) {
    return cls_id;
  }
  size_t idx = 0;
  for (char ch : cls_id) {
    if (ch < '0' || ch > '9') {
      return cls_id;
    }
    idx = idx * 10 + static_cast<size_t>(ch - '0');
    if (idx >= cfg_.class_names.size()) {
      return cls_id;
    }
  }
  return cfg_.class_names[idx];
}

}  // namespace xsting_vision

RCLCPP_COMPONENTS_REGISTER_NODE(xsting_vision::DepthPcaNode)
