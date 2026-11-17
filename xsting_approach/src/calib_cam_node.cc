#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "xsting_approach/calib.h"
#include "xsting_approach/foxglove_vis.h"
#include "xsting_approach/pca_vis.h"

namespace xsting_approach {

namespace {

// 16UC1/32FC1 깊이 이미지를 미터 단위 float 배열로 디코딩
bool DecodeDepth(const sensor_msgs::msg::Image& msg, std::vector<float>* out) {
  const size_t n = static_cast<size_t>(msg.width) * msg.height;
  if (n == 0) return false;
  out->resize(n);

  if (msg.encoding == "16UC1" || msg.encoding == "mono16") {
    const size_t min_sz =
        static_cast<size_t>(msg.height - 1) * msg.step + msg.width * 2;
    if (msg.data.size() < min_sz) return false;
    for (uint32_t y = 0; y < msg.height; ++y) {
      const auto* src =
          reinterpret_cast<const uint16_t*>(msg.data.data() + y * msg.step);
      float* dst = out->data() + static_cast<size_t>(y) * msg.width;
      for (uint32_t x = 0; x < msg.width; ++x) {
        dst[x] = static_cast<float>(src[x]) * 0.001f;
      }
    }
    return true;
  }

  if (msg.encoding == "32FC1") {
    const size_t min_sz =
        static_cast<size_t>(msg.height - 1) * msg.step + msg.width * 4;
    if (msg.data.size() < min_sz) return false;
    for (uint32_t y = 0; y < msg.height; ++y) {
      const auto* src =
          reinterpret_cast<const float*>(msg.data.data() + y * msg.step);
      float* dst = out->data() + static_cast<size_t>(y) * msg.width;
      std::memcpy(dst, src, sizeof(float) * msg.width);
    }
    return true;
  }
  return false;
}

// 컬러 픽셀 ROI를 깊이 프리뷰 화면용 픽셀 ROI로 근사 변환
Roi MapRoiToDepth(const Roi& r,
                  const Intr& d_intr,
                  const Intr& c_intr,
                  float color_off_x) {
  const float shift = d_intr.fx * (-color_off_x) / 0.35f;
  auto map_x = [&](int x) {
    return static_cast<int>(std::round(
        d_intr.cx + (x - c_intr.cx) * d_intr.fx / c_intr.fx - shift));
  };
  auto map_y = [&](int y) {
    return static_cast<int>(
        std::round(d_intr.cy + (y - c_intr.cy) * d_intr.fy / c_intr.fy));
  };
  return {map_x(r.x1), map_y(r.y1), map_x(r.x2), map_y(r.y2)};
}

}  // namespace

// Foxglove 실시간 검증을 지원하는 D435 바닥·상자 PCA·초음파·IMU 통합 보정 노드
class CalibCamNode : public rclcpp::Node {
 public:
  CalibCamNode() : Node("calib_cam_node"), vis_(this) {
    // 보정 파일, 토픽 이름 및 Foxglove 실시간 대기(live) 옵션 선언
    calib_file_ = declare_parameter<std::string>(
        "calib_file", "src/xsting_approach/config/calibration.yaml");
    const auto depth_topic = declare_parameter<std::string>(
        "depth_topic", "/camera/depth/image_emitter_on");
    const auto depth_info_topic = declare_parameter<std::string>(
        "depth_info_topic", "/camera/depth/camera_info");
    const auto color_info_topic = declare_parameter<std::string>(
        "color_info_topic", "/camera/color/camera_info");
    const auto det_topic = declare_parameter<std::string>(
        "det2d_topic", "/camera/color/detections");
    const auto range_topic =
        declare_parameter<std::string>("range_topic", "/sonar_broadcaster/range");
    const auto alt_range_topic =
        declare_parameter<std::string>("alt_range_topic", "/ultrasonic/range");
    const auto imu_topic =
        declare_parameter<std::string>("imu_topic", "/imu_broadcaster/imu");

    color_off_x_ =
        static_cast<float>(declare_parameter<double>("color_offset_x", -0.015));
    gap_m_ = declare_parameter<double>("box_gap_m", 0.200);
    live_ = declare_parameter<bool>("live", false);
    stand_legs_ = declare_parameter<bool>("stand_legs", true);
    do_floor_ = declare_parameter<bool>("calib_floor", true);
    do_box_ = declare_parameter<bool>("calib_box", true);
    do_sonar_ = declare_parameter<bool>("calib_sonar", true);
    target_frames_ =
        std::max(5, static_cast<int>(declare_parameter<int>("frames", 25)));

    // 기존 보정 파일 로드 및 WalkNet 서기 자세 목표각 준비
    calib_ = Calib::MakeDefault();
    std::string err;
    if (std::filesystem::exists(calib_file_)) calib_.Load(calib_file_, &err);
    stand_cmd_ = BuildStandLegCmd(calib_);
    start_time_ = now();

    // 실기 구동 중일 때 다리를 WalkNet 서기 자세(base_z=21.3mm)로 부드럽게 전환
    if (stand_legs_) {
      leg_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
          "/leg_controller/commands", 10);
      joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states", 10,
          [this](const sensor_msgs::msg::JointState::SharedPtr m) {
            OnJoints(*m);
          });
      stand_timer_ = create_wall_timer(
          std::chrono::milliseconds(20), [this]() { StepStand(); });
    }

    // Foxglove 대화형 보정 트리거 서비스(/calib/cam, /calib/imu, /calib/reload) 등록
    InitServices();

    // 카메라 정보·IMU·초음파·2D 검출·깊이 영상 구독
    const auto qos = rclcpp::SensorDataQoS();
    depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        depth_info_topic, qos,
        [this](const sensor_msgs::msg::CameraInfo::SharedPtr m) {
          if (m->k[0] <= 1.0 || m->k[4] <= 1.0) return;
          std::lock_guard<std::mutex> lk(mtx_);
          depth_intr_ = {static_cast<float>(m->k[0]),
                         static_cast<float>(m->k[4]),
                         static_cast<float>(m->k[2]),
                         static_cast<float>(m->k[5])};
          has_depth_intr_ = true;
        });
    color_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        color_info_topic, qos,
        [this](const sensor_msgs::msg::CameraInfo::SharedPtr m) {
          if (m->k[0] <= 1.0 || m->k[4] <= 1.0) return;
          std::lock_guard<std::mutex> lk(mtx_);
          color_intr_ = {static_cast<float>(m->k[0]),
                         static_cast<float>(m->k[4]),
                         static_cast<float>(m->k[2]),
                         static_cast<float>(m->k[5])};
          has_color_intr_ = true;
        });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos,
        [this](const sensor_msgs::msg::Imu::SharedPtr m) { OnImu(*m); });

    auto on_range = [this](const sensor_msgs::msg::Range::SharedPtr m) {
      const double r = static_cast<double>(m->range);
      std::lock_guard<std::mutex> lk(mtx_);
      us_raw_ = std::isfinite(r) ? static_cast<float>(r) : 4.0f;
      us_ok_ = std::isfinite(r) && r >= 0.015 && r < 3.5;
      if (us_ok_) {
        range_q_.push_back(r);
        if (range_q_.size() > 120) range_q_.pop_front();
      }
    };
    if (!range_topic.empty()) {
      range_sub_ =
          create_subscription<sensor_msgs::msg::Range>(range_topic, qos, on_range);
    }
    if (!alt_range_topic.empty() && alt_range_topic != range_topic) {
      alt_range_sub_ = create_subscription<sensor_msgs::msg::Range>(
          alt_range_topic, qos, on_range);
    }

    det_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        det_topic, 10,
        [this](const vision_msgs::msg::Detection2DArray::SharedPtr m) {
          std::lock_guard<std::mutex> lk(mtx_);
          has_box_roi_ = false;
          for (const auto& det : m->detections) {
            if (det.results.empty()) continue;
            const auto& cls = det.results[0].hypothesis.class_id;
            if (cls == "4" || cls == "paper_box" || cls == "box") {
              const double cx = det.bbox.center.position.x;
              const double cy = det.bbox.center.position.y;
              const double hw = 0.5 * det.bbox.size_x;
              const double hh = 0.5 * det.bbox.size_y;
              box_roi_ = {static_cast<int>(std::round(cx - hw)),
                          static_cast<int>(std::round(cy - hh)),
                          static_cast<int>(std::round(cx + hw)),
                          static_cast<int>(std::round(cy + hh))};
              has_box_roi_ = true;
              break;
            }
          }
        });

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic, qos,
        [this](const sensor_msgs::msg::Image::SharedPtr m) { OnDepth(*m); });

    RCLCPP_INFO(get_logger(),
                "실측 보정 노드 시작 (전방 %.1fcm 상자 기준, live=%s, file=%s)",
                gap_m_ * 100.0, live_ ? "true" : "false", calib_file_.c_str());
  }

 private:
  // Foxglove에서 클릭으로 호출 가능한 IMU·카메라·재로드 서비스 생성
  void InitServices() {
    using Trigger = std_srvs::srv::Trigger;
    cam_srv_ = create_service<Trigger>(
        "/calib/cam",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> res) {
          std::lock_guard<std::mutex> lk(mtx_);
          res->success = RunCamCalibLocked(&res->message);
        });
    imu_srv_ = create_service<Trigger>(
        "/calib/imu",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> res) {
          std::lock_guard<std::mutex> lk(mtx_);
          res->success = RunImuCalibLocked(&res->message);
        });
    reload_srv_ = create_service<Trigger>(
        "/calib/reload",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> res) {
          std::lock_guard<std::mutex> lk(mtx_);
          std::string err;
          res->success = calib_.Load(calib_file_, &err);
          res->message = res->success ? "보정 파일 재로드 완료" : err;
        });
  }

  // IMU 샘플 버퍼링 및 실시간 중력 벡터 갱신
  void OnImu(const sensor_msgs::msg::Imu& msg) {
    const Eigen::Vector3d gyro(msg.angular_velocity.x, msg.angular_velocity.y,
                               msg.angular_velocity.z);
    Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x,
                         msg.orientation.y, msg.orientation.z);
    Eigen::Vector3d grav(0.0, 0.0, -1.0);
    if (q.norm() > 1e-4) {
      q.normalize();
      grav = q.conjugate() * Eigen::Vector3d(0.0, 0.0, -1.0);
    } else {
      const Eigen::Vector3d acc(msg.linear_acceleration.x,
                                msg.linear_acceleration.y,
                                msg.linear_acceleration.z);
      if (acc.norm() > 1.0) grav = (-acc).normalized();
    }

    std::lock_guard<std::mutex> lk(mtx_);
    Eigen::Vector3f w_b;
    calib_.imu.Transform(gyro, q, &w_b, &proj_grav_b_);
    imu_q_.emplace_back(gyro, grav);
    if (imu_q_.size() > 200) imu_q_.pop_front();
  }

  // 깊이 프레임에서 바닥·상자 점군 추출, Foxglove 시각화 발행 및 배치 보정 수행
  void OnDepth(const sensor_msgs::msg::Image& msg) {
    if (done_) return;
    std::vector<float> depth_m;
    if (!DecodeDepth(msg, &depth_m)) return;

    const int w = static_cast<int>(msg.width);
    const int h = static_cast<int>(msg.height);
    const rclcpp::Time stamp = now();

    std::lock_guard<std::mutex> lk(mtx_);
    if (!has_depth_intr_) return;

    const Eigen::Matrix3f r_cur = calib_.cam.OptToBaseRot();
    const Eigen::Vector3f t_cur = calib_.cam.cam_pos_m.cast<float>();
    const double exp_wall_x = calib_.cam.robot_front_x_m + gap_m_;

    // 현재 프레임 바닥 후보 점군(광학/차체) 추출
    const Roi floor_roi{w / 8, h / 3, (w * 7) / 8, (h * 15) / 16};
    const auto all_opt = PcaVis::BackprojectOpt(depth_m.data(), w, h, floor_roi,
                                                depth_intr_, depth_intr_, 0.0f, 3);
    std::vector<Eigen::Vector3f> f_opt;
    std::vector<Eigen::Vector3f> f_base;
    for (const auto& po : all_opt) {
      const Eigen::Vector3f pb = r_cur * po + t_cur;
      const double z_gnd = pb.z() + calib_.cam.base_z_m;
      if (std::abs(z_gnd) < 0.045 && pb.x() < 1.30f &&
          (std::abs(pb.y()) > 0.13 || pb.x() < exp_wall_x - 0.02 ||
           pb.x() > exp_wall_x + 0.24)) {
        f_opt.push_back(po);
        f_base.push_back(pb);
      }
    }

    // 현재 프레임 상자 전면 벽면 점군 및 실시간 PCA 추출
    const bool use_color = has_box_roi_ && has_color_intr_;
    const Intr c_intr = use_color ? color_intr_ : depth_intr_;
    const float off_x = use_color ? color_off_x_ : 0.0f;
    const Roi b_roi =
        has_box_roi_ ? box_roi_ : Roi{w / 4, h / 6, (w * 3) / 4, (h * 5) / 6};
    const auto roi_opt = PcaVis::BackprojectOpt(depth_m.data(), w, h, b_roi,
                                                depth_intr_, c_intr, off_x, 2);
    std::vector<Eigen::Vector3f> b_opt;
    std::vector<Eigen::Vector3f> wall_base;
    for (const auto& po : roi_opt) {
      const Eigen::Vector3f pb = r_cur * po + t_cur;
      if (std::abs(pb.x() - exp_wall_x) < 0.10 && std::abs(pb.y()) < 0.14) {
        b_opt.push_back(po);
        const double z_gnd = pb.z() + calib_.cam.base_z_m;
        if (z_gnd > 0.008 && z_gnd < calib_.cam.box_height_m - 0.008) {
          wall_base.push_back(pb);
        }
      }
    }

    floor_q_.push_back(std::move(f_opt));
    box_q_.push_back(std::move(b_opt));
    while (static_cast<int>(floor_q_.size()) > target_frames_) {
      floor_q_.pop_front();
    }
    while (static_cast<int>(box_q_.size()) > target_frames_) {
      box_q_.pop_front();
    }

    // Foxglove 3D 마커·점군·수치 및 깊이 ROI 프리뷰 발행
    const BoxPca live_pca =
        FitBoxPca(wall_base, static_cast<float>(calib_.cam.box_half_m));
    VisFrame vf;
    vf.calib = calib_;
    vf.target_gap_m = gap_m_;
    vf.box_valid = live_pca.valid;
    vf.box_xy = live_pca.box_xy;
    vf.norm_xy = live_pca.norm_xy;
    vf.us_valid = us_ok_;
    vf.us_raw_m = us_raw_;
    vf.us_corr_m = us_raw_ + static_cast<float>(calib_.sonar.range_bias_m);
    vf.proj_grav_b = proj_grav_b_;
    vf.floor_pts = std::move(f_base);
    vf.wall_pts = std::move(wall_base);
    vis_.Publish(stamp, vf);

    const Roi d_box_roi =
        use_color ? MapRoiToDepth(b_roi, depth_intr_, color_intr_, color_off_x_)
                  : b_roi;
    vis_.PublishDepthImg(stamp, depth_m.data(), w, h, {d_box_roi}, {},
                         &floor_roi);

    // 배치 모드(live=false)일 때는 서기 자세 안정화 후 목표 프레임 도달 시 자동 저장·종료
    if (!live_ && StandReady() && ++frames_ >= target_frames_) {
      std::string summary;
      RunCamCalibLocked(&summary);
      done_ = true;
      rclcpp::shutdown();
    }
  }

  // 누적된 최근 프레임으로 바닥·상자·초음파 보정을 실행하고 YAML에 저장
  bool RunCamCalibLocked(std::string* msg_out) {
    std::vector<Eigen::Vector3f> all_floor;
    std::vector<Eigen::Vector3f> all_box;
    for (const auto& v : floor_q_) {
      all_floor.insert(all_floor.end(), v.begin(), v.end());
    }
    for (const auto& v : box_q_) {
      all_box.insert(all_box.end(), v.begin(), v.end());
    }
    const std::vector<double> all_us(range_q_.begin(), range_q_.end());

    double rmse = 0.0;
    const bool ok_f =
        do_floor_ && SolveFloor(all_floor, &calib_.cam, &rmse);
    Eigen::Vector2f box_xy = Eigen::Vector2f::Zero();
    Eigen::Vector2f norm_xy = Eigen::Vector2f::UnitX();
    const bool ok_b =
        do_box_ && SolveBox(all_box, &calib_.cam, &box_xy, &norm_xy, gap_m_);
    double us_corr = 0.0;
    const bool ok_u = do_sonar_ && SolveSonar(all_us, calib_.cam,
                                              &calib_.sonar, gap_m_, &us_corr);

    if (!ok_f && !ok_b && !ok_u) {
      if (msg_out) *msg_out = "점군/초음파 샘플 부족으로 보정 실패";
      RCLCPP_WARN(get_logger(), "%s", msg_out ? msg_out->c_str() : "");
      return false;
    }

    const auto parent = std::filesystem::path(calib_file_).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    std::string err;
    if (!calib_.Save(calib_file_, &err)) {
      if (msg_out) *msg_out = err;
      RCLCPP_ERROR(get_logger(), "%s", err.c_str());
      return false;
    }

    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "저장 완료: base_z=%.1fmm(RMSE %.1fmm), cam_pos=[%.3f,%.3f], "
                  "us_bias=%+.1fmm",
                  calib_.cam.base_z_m * 1000.0, rmse * 1000.0,
                  calib_.cam.cam_pos_m.x(), calib_.cam.cam_pos_m.y(),
                  calib_.sonar.range_bias_m * 1000.0);
    if (msg_out) *msg_out = buf;
    RCLCPP_INFO(get_logger(), "%s -> %s", buf, calib_file_.c_str());
    return true;
  }

  // 누적된 최근 IMU 샘플로 수평 장착각·자이로 바이어스 보정 및 저장
  bool RunImuCalibLocked(std::string* msg_out) {
    if (imu_q_.size() < 20) {
      if (msg_out) *msg_out = "IMU 샘플 부족 (최소 20개 필요)";
      return false;
    }
    Eigen::Vector3d g_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_sum = Eigen::Vector3d::Zero();
    for (const auto& item : imu_q_) {
      g_sum += item.first;
      v_sum += item.second;
    }
    const double inv = 1.0 / static_cast<double>(imu_q_.size());
    calib_.imu = SolveImu1Pose(g_sum * inv, v_sum * inv, 0.0);

    const auto parent = std::filesystem::path(calib_file_).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    std::string err;
    if (!calib_.Save(calib_file_, &err)) {
      if (msg_out) *msg_out = err;
      return false;
    }
    if (msg_out) *msg_out = "IMU 수평·자이로 보정 저장 완료";
    RCLCPP_INFO(get_logger(), "IMU 보정 완료 -> %s", calib_file_.c_str());
    return true;
  }

  // 최초 관절 상태를 받아 서기 자세 보간 시작점으로 저장
  void OnJoints(const sensor_msgs::msg::JointState& msg) {
    if (has_joints_) return;
    init_cmd_ = stand_cmd_;
    for (int out_i = 0; out_i < kNumLegs; ++out_i) {
      const std::string& jname =
          static_cast<size_t>(out_i) < calib_.out_leg_order.size()
              ? calib_.out_leg_order[out_i]
              : calib_.legs[out_i].ros_name;
      for (size_t k = 0; k < msg.name.size() && k < msg.position.size(); ++k) {
        if (msg.name[k] == jname) {
          init_cmd_[out_i] = msg.position[k];
          break;
        }
      }
    }
    stand_start_ = now();
    has_joints_ = true;
  }

  // 1.5초 동안 WalkNet 서기 자세로 선형 보간 발행
  void StepStand() {
    if (!has_joints_ || !leg_pub_) return;
    const double t = (now() - stand_start_).seconds();
    const double a = std::clamp(t / 1.5, 0.0, 1.0);
    std_msgs::msg::Float64MultiArray out;
    out.data.resize(kNumLegs);
    for (int i = 0; i < kNumLegs; ++i) {
      out.data[i] = (1.0 - a) * init_cmd_[i] + a * stand_cmd_[i];
    }
    leg_pub_->publish(out);
  }

  // 서기 자세 전환 및 진동 감쇠(총 2.0초) 완료 여부 확인
  bool StandReady() const {
    if (!stand_legs_) return true;
    if (!has_joints_) return (now() - start_time_).seconds() > 0.6;
    return (now() - stand_start_).seconds() >= 2.0;
  }

  std::mutex mtx_;
  FoxgloveVis vis_;
  std::string calib_file_;
  float color_off_x_ = -0.015f;
  double gap_m_ = 0.200;
  bool live_ = false;
  bool stand_legs_ = true;
  bool has_joints_ = false;
  bool do_floor_ = true;
  bool do_box_ = true;
  bool do_sonar_ = true;
  int target_frames_ = 25;
  int frames_ = 0;
  bool done_ = false;

  rclcpp::Time start_time_;
  rclcpp::Time stand_start_;
  std::array<double, kNumLegs> init_cmd_{};
  std::array<double, kNumLegs> stand_cmd_{};

  Intr depth_intr_;
  Intr color_intr_;
  bool has_depth_intr_ = false;
  bool has_color_intr_ = false;
  Roi box_roi_;
  bool has_box_roi_ = false;

  float us_raw_ = 4.0f;
  bool us_ok_ = false;
  Eigen::Vector3f proj_grav_b_ = Eigen::Vector3f(0.0f, 0.0f, -1.0f);

  std::deque<std::vector<Eigen::Vector3f>> floor_q_;
  std::deque<std::vector<Eigen::Vector3f>> box_q_;
  std::deque<double> range_q_;
  std::deque<std::pair<Eigen::Vector3d, Eigen::Vector3d>> imu_q_;
  Calib calib_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr stand_timer_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr alt_range_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cam_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr imu_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_srv_;
};

}  // namespace xsting_approach

// 카메라·상자·초음파·IMU 통합 보정 노드 실행
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xsting_approach::CalibCamNode>());
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
