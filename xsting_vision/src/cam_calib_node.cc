#include "xsting_vision/cam_calib_node.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <cuda_buffer/cuda_buffer_api.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.hpp>

namespace xsting_vision {

namespace {

// TF 메시지를 3차원 강체 변환 구조체로 변환
Rigid3d MakeRigid(const TfMsg& tf_msg) {
  const auto& quat = tf_msg.transform.rotation;
  const auto& vec = tf_msg.transform.translation;
  const double xx = quat.x * quat.x;
  const double yy = quat.y * quat.y;
  const double zz = quat.z * quat.z;
  const double xy = quat.x * quat.y;
  const double xz = quat.x * quat.z;
  const double yz = quat.y * quat.z;
  const double wx = quat.w * quat.x;
  const double wy = quat.w * quat.y;
  const double wz = quat.w * quat.z;

  Rigid3d out;
  out.rot =
      cv::Matx33d(1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
                  2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
                  2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy));
  out.trans = cv::Vec3d(vec.x, vec.y, vec.z);
  return out;
}

// 두 3차원 강체 변환을 순서대로 합성
Rigid3d ComposeRigid(const Rigid3d& lhs, const Rigid3d& rhs) {
  Rigid3d out;
  out.rot = lhs.rot * rhs.rot;
  out.trans = lhs.rot * rhs.trans + lhs.trans;
  return out;
}

// 3x3 CV_64F 행렬을 cv::Matx33d로 변환
cv::Matx33d CopyMat33(const cv::Mat& mat) {
  cv::Matx33d out;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      out(row, col) = mat.at<double>(row, col);
    }
  }
  return out;
}

// 3차원 대응점 쌍에 대한 변환의 평균 제곱근(RMS) 오차(mm) 계산
double CalcRmsMm(const Rigid3d& tf_rigid, const std::vector<cv::Vec3d>& src,
                 const std::vector<cv::Vec3d>& dst) {
  if (src.empty()) {
    return 0.0;
  }
  double sq_sum = 0.0;
  for (size_t i = 0; i < src.size(); ++i) {
    const cv::Vec3d diff = (tf_rigid.rot * src[i] + tf_rigid.trans) - dst[i];
    sq_sum += diff.dot(diff);
  }
  return std::sqrt(sq_sum / static_cast<double>(src.size())) * 1000.0;
}

// 회전 행렬에서 URDF 고정축 기준 Roll·Pitch·Yaw(rad) 추출
cv::Vec3d ExtractRpy(const cv::Matx33d& rot) {
  const double cos_p = std::hypot(rot(0, 0), rot(1, 0));
  const double pitch = std::atan2(-rot(2, 0), cos_p);
  if (cos_p > 1e-6) {
    const double roll = std::atan2(rot(2, 1), rot(2, 2));
    const double yaw = std::atan2(rot(1, 0), rot(0, 0));
    return cv::Vec3d(roll, pitch, yaw);
  }
  return cv::Vec3d(std::atan2(-rot(1, 2), rot(1, 1)), pitch, 0.0);
}

}  // namespace

// YAML 설정을 읽고 상·하단 카메라 영상·정보 구독 및 디버그 퍼블리셔 생성
CamCalibNode::CamCalibNode(const rclcpp::NodeOptions& opts)
    : Node("cam_calib", AddYamlParams(opts)),
      tf_buf_(get_clock()),
      tf_sub_(tf_buf_, *this, false) {
  cfg_ = LoadCfg();
  board_pts_ = BuildBoardPts();
  cudaStreamCreateWithFlags(&cuda_stream_, cudaStreamNonBlocking);

  const auto qos = rclcpp::SensorDataQoS();
  rclcpp::SubscriptionOptions any_opts;
  any_opts.acceptable_buffer_backends = "any";

  const std::string top_prefix =
      cfg_.top_img_topic.substr(0, cfg_.top_img_topic.rfind('/'));
  const std::string bot_prefix =
      cfg_.bot_img_topic.substr(0, cfg_.bot_img_topic.rfind('/'));

  top_info_sub_ = create_subscription<CamInfoMsg>(
      top_prefix + "/camera_info", qos,
      [this](const CamInfoMsg::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        top_info_ = msg;
      });
  bot_info_sub_ = create_subscription<CamInfoMsg>(
      bot_prefix + "/camera_info", qos,
      [this](const CamInfoMsg::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        bot_info_ = msg;
      });
  bot_img_sub_ = create_subscription<ImageMsg>(
      cfg_.bot_img_topic, qos,
      [this](const ImageMsg::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        latest_bot_img_ = msg;
      },
      any_opts);
  top_img_sub_ = create_subscription<ImageMsg>(
      cfg_.top_img_topic, qos,
      [this](const ImageMsg::ConstSharedPtr& msg) { OnTopImg(msg); }, any_opts);
  debug_pub_ = create_publisher<ImageMsg>("cam_calib/debug_image", qos);

  RCLCPP_INFO(get_logger(), "Checkerboard %dx%d (%.1fmm), waiting %d samples",
              cfg_.board_cols, cfg_.board_rows, cfg_.square_size * 1000.0,
              cfg_.num_samples);
}

// 구독 해제 후 CUDA stream 정리
CamCalibNode::~CamCalibNode() {
  top_img_sub_.reset();
  bot_img_sub_.reset();
  cudaStreamSynchronize(cuda_stream_);
  cudaStreamDestroy(cuda_stream_);
}

// YAML 파일 파라미터를 NodeOptions 앞쪽에 추가
rclcpp::NodeOptions CamCalibNode::AddYamlParams(
    const rclcpp::NodeOptions& opts) {
  std::string cfg_path =
      (ament_index_cpp::get_package_share_path("xsting_vision") / "config" /
       "cam_calib.yaml")
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

// 체커보드 규격 및 샘플링 파라미터 읽기
CamCalibCfg CamCalibNode::LoadCfg() {
  CamCalibCfg cfg;
  cfg.top_img_topic = declare_parameter<std::string>(
      "top_image_topic", "top_camera/color/image_raw");
  cfg.bot_img_topic = declare_parameter<std::string>(
      "bottom_image_topic", "bottom_camera/color/image_raw");
  cfg.base_frame = declare_parameter<std::string>("base_frame", "base");
  cfg.top_link_frame =
      declare_parameter<std::string>("top_link_frame", "top_camera");
  cfg.board_cols = declare_parameter<int>("board_cols", 9);
  cfg.board_rows = declare_parameter<int>("board_rows", 6);
  cfg.square_size = declare_parameter<double>("square_size", 0.025);
  cfg.num_samples = declare_parameter<int>("num_samples", 50);
  cfg.sample_interval = declare_parameter<double>("sample_interval", 0.20);
  cfg.max_sync_diff = declare_parameter<double>("max_sync_diff", 0.02);
  return cfg;
}

// 체커보드 평면의 3차원 기준 코너 좌표 목록 생성
std::vector<cv::Point3f> CamCalibNode::BuildBoardPts() const {
  std::vector<cv::Point3f> pts;
  pts.reserve(static_cast<size_t>(cfg_.board_cols * cfg_.board_rows));
  const float sq = static_cast<float>(cfg_.square_size);
  for (int row = 0; row < cfg_.board_rows; ++row) {
    for (int col = 0; col < cfg_.board_cols; ++col) {
      pts.emplace_back(col * sq, row * sq, 0.0f);
    }
  }
  return pts;
}

// 상·하단 동기 프레임에서 체커보드를 찾아 3D 코너 대응점 누적
void CamCalibNode::OnTopImg(const ImageMsg::ConstSharedPtr& top_msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (done_ || !top_info_ || !bot_info_ || !latest_bot_img_) {
    return;
  }

  // 동기 허용차 및 샘플링 간격 확인
  const rclcpp::Time stamp(top_msg->header.stamp);
  const double sync_diff =
      std::abs((stamp - rclcpp::Time(latest_bot_img_->header.stamp)).seconds());
  if (sync_diff > cfg_.max_sync_diff) {
    return;
  }
  if (last_try_stamp_.nanoseconds() > 0 &&
      std::abs((stamp - last_try_stamp_).seconds()) < 0.10) {
    return;
  }
  if (sample_cnt_ > 0 &&
      std::abs((stamp - last_sample_stamp_).seconds()) < cfg_.sample_interval) {
    return;
  }
  last_try_stamp_ = stamp;

  // 보정에 필요한 정적 TF 변환 조회
  TfMsg tf_base_bot;
  TfMsg tf_link_top;
  TfMsg tf_base_top;
  try {
    tf_base_bot = tf_buf_.lookupTransform(
        cfg_.base_frame, bot_info_->header.frame_id, tf2::TimePointZero);
    tf_link_top = tf_buf_.lookupTransform(
        cfg_.top_link_frame, top_info_->header.frame_id, tf2::TimePointZero);
    tf_base_top = tf_buf_.lookupTransform(
        cfg_.base_frame, cfg_.top_link_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException&) {
    return;
  }

  // 두 카메라 영상 복사 후 체커보드 검출 및 PnP 수행
  cv::Mat top_rgb;
  cv::Mat bot_rgb;
  if (!CopyImg(*top_msg, top_rgb) || !CopyImg(*latest_bot_img_, bot_rgb)) {
    return;
  }
  std::vector<cv::Point2f> top_corners;
  std::vector<cv::Point2f> bot_corners;
  Rigid3d top_pose;
  Rigid3d bot_pose;
  const bool top_ok = DetectBoard(top_rgb, *top_info_, top_corners, top_pose);
  const bool bot_ok = DetectBoard(bot_rgb, *bot_info_, bot_corners, bot_pose);

  // 양쪽 모두 검출되면 3D 코너 대응점 누적
  if (top_ok && bot_ok) {
    const Rigid3d base_from_board =
        ComposeRigid(MakeRigid(tf_base_bot), bot_pose);
    const Rigid3d link_from_board =
        ComposeRigid(MakeRigid(tf_link_top), top_pose);
    for (const cv::Point3f& pt : board_pts_) {
      const cv::Vec3d board_pt(pt.x, pt.y, pt.z);
      src_pts_.push_back(link_from_board.rot * board_pt +
                         link_from_board.trans);
      dst_pts_.push_back(base_from_board.rot * board_pt +
                         base_from_board.trans);
    }
    ++sample_cnt_;
    last_sample_stamp_ = stamp;
    RCLCPP_INFO(get_logger(), "Sample %d / %d collected", sample_cnt_,
                cfg_.num_samples);
  }

  if (debug_pub_->get_subscription_count() > 0) {
    PubDebugImg(top_msg->header, top_rgb, top_corners, top_ok, bot_rgb,
                bot_corners, bot_ok);
  }

  // 목표 샘플 수 도달 시 SVD 정합 결과 출력 후 종료
  if (sample_cnt_ >= cfg_.num_samples) {
    done_ = true;
    FinishCalib(MakeRigid(tf_base_top));
    rclcpp::shutdown();
  }
}

// GPU 또는 CPU 이미지 버퍼를 호스트 RGB 행렬로 복사
bool CamCalibNode::CopyImg(const ImageMsg& img_msg, cv::Mat& rgb) {
  if (img_msg.width == 0 || img_msg.height == 0 || img_msg.encoding != "rgb8") {
    return false;
  }
  const size_t row_bytes = static_cast<size_t>(img_msg.width) * 3;
  const size_t min_bytes =
      static_cast<size_t>(img_msg.height - 1) * img_msg.step + row_bytes;
  if (img_msg.data.size() < min_bytes) {
    return false;
  }

  rgb.create(static_cast<int>(img_msg.height), static_cast<int>(img_msg.width),
             CV_8UC3);
  {
    auto handle =
        cuda_buffer_backend::from_input_buffer(img_msg.data, cuda_stream_);
    cudaMemcpy2DAsync(rgb.data, rgb.step, handle.get_ptr(), img_msg.step,
                      row_bytes, img_msg.height, cudaMemcpyDeviceToHost,
                      cuda_stream_);
  }
  cudaStreamSynchronize(cuda_stream_);
  return true;
}

// 단일 영상에서 체커보드 코너 방향을 통일하고 카메라 기준 보드 자세 추정
bool CamCalibNode::DetectBoard(const cv::Mat& rgb, const CamInfoMsg& info,
                               std::vector<cv::Point2f>& corners,
                               Rigid3d& pose) const {
  cv::Mat gray;
  cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
  const cv::Size pat_size(cfg_.board_cols, cfg_.board_rows);
  const int flags = cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE;
  if (!cv::findChessboardCornersSB(gray, pat_size, corners, flags)) {
    return false;
  }

  // 대칭 체커보드의 180도 회전 모호성 방지를 위해 첫 행 방향 고정
  const cv::Point2f span =
      corners[static_cast<size_t>(cfg_.board_cols - 1)] - corners[0];
  const bool rev =
      std::abs(span.x) >= std::abs(span.y) ? (span.x < 0.0f) : (span.y < 0.0f);
  if (rev) {
    std::reverse(corners.begin(), corners.end());
  }

  // 내부 파라미터와 왜곡 계수로 보드 6D 자세 계산
  const cv::Matx33d cam_mat(info.k[0], info.k[1], info.k[2], info.k[3],
                            info.k[4], info.k[5], info.k[6], info.k[7],
                            info.k[8]);
  cv::Mat rvec;
  cv::Mat tvec;
  if (!cv::solvePnP(board_pts_, corners, cam_mat, info.d, rvec, tvec)) {
    return false;
  }
  if (tvec.at<double>(2) <= 0.1) {
    return false;
  }

  cv::Mat rot_mat;
  cv::Rodrigues(rvec, rot_mat);
  pose.rot = CopyMat33(rot_mat);
  pose.trans =
      cv::Vec3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
  return true;
}

// 상·하단 체커보드 검출 상태와 수집 진행률을 나란히 그려 디버그 발행
void CamCalibNode::PubDebugImg(const std_msgs::msg::Header& header,
                               const cv::Mat& top_rgb,
                               const std::vector<cv::Point2f>& top_corners,
                               bool top_ok, const cv::Mat& bot_rgb,
                               const std::vector<cv::Point2f>& bot_corners,
                               bool bot_ok) {
  cv::Mat top_vis = top_rgb.clone();
  cv::Mat bot_vis = bot_rgb.clone();
  const cv::Size pat_size(cfg_.board_cols, cfg_.board_rows);
  cv::drawChessboardCorners(top_vis, pat_size, top_corners, top_ok);
  cv::drawChessboardCorners(bot_vis, pat_size, bot_corners, bot_ok);
  if (bot_vis.size() != top_vis.size()) {
    cv::resize(bot_vis, bot_vis, top_vis.size());
  }

  cv::Mat merged;
  cv::hconcat(top_vis, bot_vis, merged);
  char status[80];
  std::snprintf(status, sizeof(status), "Samples: %d / %d", sample_cnt_,
                cfg_.num_samples);
  cv::putText(merged, status, cv::Point(16, 36), cv::FONT_HERSHEY_SIMPLEX, 0.9,
              cv::Scalar(255, 255, 0), 2);

  auto msg = std::make_unique<ImageMsg>();
  msg->header = header;
  msg->width = static_cast<uint32_t>(merged.cols);
  msg->height = static_cast<uint32_t>(merged.rows);
  msg->encoding = "rgb8";
  msg->step = static_cast<uint32_t>(merged.step);
  msg->data.assign(merged.data, merged.data + merged.step * merged.rows);
  debug_pub_->publish(std::move(msg));
}

// 수집된 모든 3D 코너 대응점에 Kabsch SVD 정합을 수행해 URDF 보정값 출력
void CamCalibNode::FinishCalib(const Rigid3d& cur_base_from_top) const {
  if (src_pts_.empty()) {
    return;
  }

  // 두 좌표계 점군의 무게중심 및 교차 공분산 행렬 계산
  cv::Vec3d mean_src(0.0, 0.0, 0.0);
  cv::Vec3d mean_dst(0.0, 0.0, 0.0);
  for (size_t i = 0; i < src_pts_.size(); ++i) {
    mean_src += src_pts_[i];
    mean_dst += dst_pts_[i];
  }
  const double inv_n = 1.0 / static_cast<double>(src_pts_.size());
  mean_src *= inv_n;
  mean_dst *= inv_n;

  cv::Matx33d cov = cv::Matx33d::zeros();
  for (size_t i = 0; i < src_pts_.size(); ++i) {
    const cv::Vec3d diff_s = src_pts_[i] - mean_src;
    const cv::Vec3d diff_d = dst_pts_[i] - mean_dst;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        cov(row, col) += diff_d[row] * diff_s[col];
      }
    }
  }

  // SVD로 최적 SO(3) 회전 행렬과 평행이동 벡터 계산
  cv::Mat w_svd;
  cv::Mat u_mat;
  cv::Mat vt_mat;
  cv::SVD::compute(cv::Mat(cov), w_svd, u_mat, vt_mat, cv::SVD::FULL_UV);
  cv::Mat diag = cv::Mat::eye(3, 3, CV_64F);
  if (cv::determinant(u_mat * vt_mat) < 0.0) {
    diag.at<double>(2, 2) = -1.0;
  }
  const cv::Mat rot_mat = u_mat * diag * vt_mat;

  Rigid3d calib;
  calib.rot = CopyMat33(rot_mat);
  calib.trans = mean_dst - calib.rot * mean_src;

  // 보정 전·후 3D 정합 오차 및 URDF 적용 태그 출력
  const double before_mm = CalcRmsMm(cur_base_from_top, src_pts_, dst_pts_);
  const double after_mm = CalcRmsMm(calib, src_pts_, dst_pts_);
  const cv::Vec3d rpy = ExtractRpy(calib.rot);

  RCLCPP_INFO(get_logger(), "Calib RMS error: %.2f mm -> %.2f mm", before_mm,
              after_mm);
  RCLCPP_INFO(get_logger(), "URDF (%s -> %s):", cfg_.base_frame.c_str(),
              cfg_.top_link_frame.c_str());
  RCLCPP_INFO(get_logger(),
              "<origin xyz=\"%.6f %.6f %.6f\" rpy=\"%.6f %.6f %.6f\" />",
              calib.trans[0], calib.trans[1], calib.trans[2], rpy[0], rpy[1],
              rpy[2]);
}

}  // namespace xsting_vision

RCLCPP_COMPONENTS_REGISTER_NODE(xsting_vision::CamCalibNode)
