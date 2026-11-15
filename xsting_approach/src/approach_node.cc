#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <xsting_interfaces/action/approach_box.hpp>

#include "xsting_approach/approach_ctrl.h"
#include "xsting_approach/foxglove_vis.h"
#include "xsting_approach/pca_vis.h"

namespace xsting_approach {

namespace {

// 패키지 공유 디렉터리 기준 상대 경로 해석
std::string ResolvePath(const std::string& path) {
  if (path.empty()) return path;
  const std::filesystem::path p(path);
  if (p.is_absolute() || std::filesystem::exists(p)) return p.string();
  try {
    const std::filesystem::path share =
        ament_index_cpp::get_package_share_path("xsting_approach");
    if (std::filesystem::exists(share / p)) return (share / p).string();
  } catch (const std::exception&) {
  }
  return path;
}

// xsting_vision YOLO 클래스(4=paper_box, 0..3=puck) 판별
bool IsBoxCls(const std::string& cls) {
  return cls == "4" || cls == "paper_box" || cls == "box";
}

bool IsPuckCls(const std::string& cls) {
  return cls == "0" || cls == "1" || cls == "2" || cls == "3" ||
         cls.find("square") != std::string::npos ||
         cls.find("cylinder") != std::string::npos ||
         cls.find("puck") != std::string::npos;
}

// 16UC1(mm) 또는 32FC1(m) 깊이 이미지를 미터 단위 버퍼로 변환
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

// 컬러 ROI를 깊이 프리뷰 화면용 픽셀 ROI로 근사 변환
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

// 상자 2cm 정밀 접근 강화학습 액션·서비스·Foxglove 시각화 통합 노드
class ApproachNode : public rclcpp::Node {
 public:
  using ApproachAct = xsting_interfaces::action::ApproachBox;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ApproachAct>;

  ApproachNode() : Node("approach_node"), vis_(this) {
    // 모델·보정 경로 및 제어 옵션 파라미터 선언
    const std::string app_bin = ResolvePath(declare_parameter<std::string>(
        "approach_model", "models/approach_policy_xsting.bin"));
    const std::string walk_bin = ResolvePath(declare_parameter<std::string>(
        "walk_model", "models/walk_actor_26d.bin"));
    calib_path_ = ResolvePath(
        declare_parameter<std::string>("calib_file", "config/calibration.yaml"));

    color_off_x_ =
        static_cast<float>(declare_parameter<double>("color_offset_x", -0.015));
    vis_timeout_ = declare_parameter<double>("vis_timeout", 0.35);
    sonar_timeout_ = declare_parameter<double>("sonar_timeout", 0.35);
    def_goal_timeout_ = declare_parameter<double>("default_goal_timeout", 35.0);
    hold_arm_walking_ = declare_parameter<bool>("hold_arm_walking", true);
    stand_when_idle_ = declare_parameter<bool>("stand_when_idle", false);
    dry_run_ = declare_parameter<bool>("dry_run", false);
    const bool auto_start = declare_parameter<bool>("auto_start", false);

    // 실기 보정 YAML 및 강화학습 신경망 로드
    Calib calib = Calib::MakeDefault();
    std::string err;
    if (std::filesystem::exists(calib_path_) && !calib.Load(calib_path_, &err)) {
      RCLCPP_WARN(get_logger(), "%s", err.c_str());
    }
    if (!ctrl_.Init(app_bin, walk_bin, calib, &err)) {
      RCLCPP_FATAL(get_logger(), "강화학습 모델 초기화 실패: %s", err.c_str());
      throw std::runtime_error(err);
    }
    pca_.SetCalib(calib.cam, calib.sonar);
    ctrl_.SetActive(auto_start);

    // 토픽 이름 선언
    const auto leg_topic = declare_parameter<std::string>(
        "leg_cmd_topic", "/leg_controller/commands");
    const auto arm_topic = declare_parameter<std::string>(
        "arm_cmd_topic", "/arm_controller/commands");
    const auto joint_topic =
        declare_parameter<std::string>("joint_topic", "/joint_states");
    const auto imu_topic =
        declare_parameter<std::string>("imu_topic", "/imu_broadcaster/imu");
    const auto range_topic =
        declare_parameter<std::string>("range_topic", "/sonar_broadcaster/range");
    const auto alt_range_topic =
        declare_parameter<std::string>("alt_range_topic", "/ultrasonic/range");
    const auto depth_topic = declare_parameter<std::string>(
        "depth_topic", "/camera/depth/image_emitter_on");
    const auto depth_info_topic = declare_parameter<std::string>(
        "depth_info_topic", "/camera/depth/camera_info");
    const auto color_info_topic = declare_parameter<std::string>(
        "color_info_topic", "/camera/color/camera_info");
    const auto det2d_topic = declare_parameter<std::string>(
        "det2d_topic", "/camera/color/detections");
    const auto det3d_topic =
        declare_parameter<std::string>("det3d_topic", "/detections_3d");
    const auto odom_topic = declare_parameter<std::string>(
        "odom_topic", "/visual_slam/tracking/odometry");

    // 제어 명령 및 상태 퍼블리셔 생성
    leg_pub_ =
        create_publisher<std_msgs::msg::Float64MultiArray>(leg_topic, 10);
    if (!arm_topic.empty()) {
      arm_pub_ =
          create_publisher<std_msgs::msg::Float64MultiArray>(arm_topic, 10);
    }
    status_pub_ =
        create_publisher<std_msgs::msg::Float32MultiArray>("/approach/status", 10);
    reached_pub_ = create_publisher<std_msgs::msg::Bool>("/approach/reached", 10);

    // 하드웨어 관절·IMU·초음파·오도메트리 구독 생성
    const auto qos = rclcpp::SensorDataQoS();
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_topic, qos, [this](const sensor_msgs::msg::JointState::SharedPtr m) {
          std::lock_guard<std::mutex> lk(mtx_);
          ctrl_.UpdateJoints(m->name, m->position, m->velocity);
          seen_joints_ = true;
        });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos, [this](const sensor_msgs::msg::Imu::SharedPtr m) {
          std::lock_guard<std::mutex> lk(mtx_);
          ctrl_.UpdateImu(
              {m->angular_velocity.x, m->angular_velocity.y,
               m->angular_velocity.z},
              {m->orientation.w, m->orientation.x, m->orientation.y,
               m->orientation.z});
        });
    if (!range_topic.empty()) {
      range_sub_ = create_subscription<sensor_msgs::msg::Range>(
          range_topic, qos,
          [this](const sensor_msgs::msg::Range::SharedPtr m) { OnRange(*m); });
    }
    if (!alt_range_topic.empty() && alt_range_topic != range_topic) {
      alt_range_sub_ = create_subscription<sensor_msgs::msg::Range>(
          alt_range_topic, qos,
          [this](const sensor_msgs::msg::Range::SharedPtr m) { OnRange(*m); });
    }
    if (!odom_topic.empty()) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          odom_topic, qos, [this](const nav_msgs::msg::Odometry::SharedPtr m) {
            std::lock_guard<std::mutex> lk(mtx_);
            ctrl_.UpdateVel(
                {static_cast<float>(m->twist.twist.linear.x),
                 static_cast<float>(m->twist.twist.linear.y)},
                static_cast<float>(m->twist.twist.angular.z));
          });
    }

    // D435 카메라 정보·깊이·2D/3D 검출 구독 생성
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
    if (!color_info_topic.empty()) {
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
    }
    det2d_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        det2d_topic, 10,
        [this](const vision_msgs::msg::Detection2DArray::SharedPtr m) {
          OnDet2d(*m);
        });
    if (!det3d_topic.empty()) {
      det3d_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
          det3d_topic, 10,
          [this](const vision_msgs::msg::Detection3DArray::SharedPtr m) {
            OnDet3d(*m);
          });
    }
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic, qos,
        [this](const sensor_msgs::msg::Image::SharedPtr m) { OnDepth(*m); });

    // CLI 제어용 서비스 및 상위 미션 연동용 액션 서버 등록
    InitServicesAndAction();

    // 50Hz 상위 정책 타이머 및 200Hz 하위 WalkNet 타이머 시작
    policy_timer_ = create_wall_timer(std::chrono::milliseconds(20),
                                      [this] { OnPolicy50Hz(); });
    walk_timer_ = create_wall_timer(std::chrono::milliseconds(5),
                                    [this] { OnWalk200Hz(); });

    RCLCPP_INFO(get_logger(),
                "정밀 접근 노드 준비 완료 (calib=%s, dry_run=%s)",
                calib_path_.c_str(), dry_run_ ? "true" : "false");
  }

 private:
  // 시작·정지·리셋·보정재로드 서비스 및 /approach_box 액션 서버 생성
  void InitServicesAndAction() {
    using Trigger = std_srvs::srv::Trigger;
    start_srv_ = create_service<Trigger>(
        "/approach/start",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> res) {
          std::lock_guard<std::mutex> lk(mtx_);
          ctrl_.SetActive(true);
          res->success = true;
          res->message = "정밀 접근 시작";
        });
    stop_srv_ = create_service<Trigger>(
        "/approach/stop",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> res) {
          std::lock_guard<std::mutex> lk(mtx_);
          ctrl_.SetActive(false);
          res->success = true;
          res->message = "정밀 접근 정지";
        });
    reset_srv_ = create_service<Trigger>(
        "/approach/reset",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> res) {
          std::lock_guard<std::mutex> lk(mtx_);
          pca_.Reset();
          ctrl_.Reset();
          res->success = true;
          res->message = "정밀 접근 리셋";
        });
    reload_srv_ = create_service<Trigger>(
        "/approach/reload_calib",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> res) {
          std::lock_guard<std::mutex> lk(mtx_);
          Calib c = Calib::MakeDefault();
          std::string err;
          if (!c.Load(calib_path_, &err)) {
            res->success = false;
            res->message = err;
            return;
          }
          ctrl_.SetCalib(c);
          pca_.SetCalib(c.cam, c.sonar);
          res->success = true;
          res->message = "실기 보정 파일 재로드 완료";
        });

    action_srv_ = rclcpp_action::create_server<ApproachAct>(
        this, "approach_box",
        [this](const rclcpp_action::GoalUUID&,
               std::shared_ptr<const ApproachAct::Goal>) {
          std::lock_guard<std::mutex> lk(mtx_);
          return goal_ ? rclcpp_action::GoalResponse::REJECT
                       : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<GoalHandle>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle> gh) {
          std::lock_guard<std::mutex> lk(mtx_);
          const auto g = gh->get_goal();
          goal_ = gh;
          goal_start_ = now();
          goal_timeout_ = (g->timeout_sec > 0.0f)
                              ? static_cast<double>(g->timeout_sec)
                              : def_goal_timeout_;
          goal_dry_run_ = g->dry_run;
          pca_.Reset();
          ctrl_.Reset();
          ctrl_.SetActive(true);
          RCLCPP_INFO(get_logger(), "ApproachBox 액션 시작 (timeout=%.1fs)",
                      goal_timeout_);
        });
  }

  // 초음파 거리 수신 및 타임스탬프 기록
  void OnRange(const sensor_msgs::msg::Range& msg) {
    const double r = static_cast<double>(msg.range);
    const bool valid = std::isfinite(r) && r > 0.0 &&
                       (msg.max_range <= 0.0f || r < msg.max_range);
    std::lock_guard<std::mutex> lk(mtx_);
    ctrl_.UpdateSonar(r, valid);
    sonar_stamp_ = now();
    has_sonar_ = true;
  }

  // 컬러 2D YOLO 검출 박스를 상자(4)와 퍽(0..3) ROI로 분리 저장
  void OnDet2d(const vision_msgs::msg::Detection2DArray& msg) {
    std::vector<Roi> boxes;
    std::vector<Roi> pucks;
    for (const auto& det : msg.detections) {
      if (det.results.empty()) continue;
      const std::string& cls = det.results[0].hypothesis.class_id;
      const double cx = det.bbox.center.position.x;
      const double cy = det.bbox.center.position.y;
      const double hw = 0.5 * det.bbox.size_x;
      const double hh = 0.5 * det.bbox.size_y;
      const Roi roi{static_cast<int>(std::round(cx - hw)),
                    static_cast<int>(std::round(cy - hh)),
                    static_cast<int>(std::round(cx + hw)),
                    static_cast<int>(std::round(cy + hh))};
      if (IsBoxCls(cls)) {
        boxes.push_back(roi);
      } else if (IsPuckCls(cls)) {
        pucks.push_back(roi);
      }
    }
    std::lock_guard<std::mutex> lk(mtx_);
    box_rois_ = std::move(boxes);
    puck_rois_ = std::move(pucks);
    det2d_stamp_ = now();
    has_det2d_ = true;
  }

  // 원시 깊이 벽면 PCA가 없을 때만 /detections_3d 폴백 사용
  void OnDet3d(const vision_msgs::msg::Detection3DArray& msg) {
    const rclcpp::Time t_now = now();
    std::lock_guard<std::mutex> lk(mtx_);
    if (has_raw_vis_ && (t_now - raw_vis_stamp_).seconds() <= 0.25) return;

    bool found = false;
    Eigen::Vector2f box_xy = Eigen::Vector2f::Zero();
    Eigen::Vector2f norm_xy = Eigen::Vector2f::UnitX();
    std::vector<Eigen::Vector2f> pucks;

    for (const auto& det : msg.detections) {
      if (det.results.empty()) continue;
      const std::string& cls = det.results[0].hypothesis.class_id;
      const auto& p = det.bbox.center.position;
      const auto& q = det.bbox.center.orientation;
      if (IsBoxCls(cls) && !found) {
        box_xy = {static_cast<float>(p.x), static_cast<float>(p.y)};
        Eigen::Quaternionf qf(static_cast<float>(q.w), static_cast<float>(q.x),
                              static_cast<float>(q.y), static_cast<float>(q.z));
        if (qf.norm() > 1e-5f) {
          qf.normalize();
          Eigen::Vector2f nz = (qf * Eigen::Vector3f::UnitZ()).head<2>();
          if (nz.norm() < 1e-4f) nz = (qf * Eigen::Vector3f::UnitX()).head<2>();
          if (nz.norm() > 1e-4f) norm_xy = nz.normalized();
        }
        if (norm_xy.dot(box_xy) < 0.0f) norm_xy = -norm_xy;
        found = true;
      } else if (IsPuckCls(cls)) {
        pucks.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));
      }
    }
    if (found) {
      pca_.UpdateDet3d(box_xy, norm_xy, pucks);
      vis_stamp_ = t_now;
      has_vis_ = true;
    }
  }

  // 깊이 영상과 최신 컬러 ROI를 결합해 벽면 PCA 및 Foxglove 점군·프리뷰 생성
  void OnDepth(const sensor_msgs::msg::Image& msg) {
    Intr d_intr;
    Intr c_intr;
    bool use_color = false;
    float off_x = 0.0f;
    std::vector<Roi> boxes;
    std::vector<Roi> pucks;
    float us_r = 4.0f;
    bool us_ok = false;
    const rclcpp::Time t_now = now();

    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!has_depth_intr_) return;
      d_intr = depth_intr_;
      use_color = has_color_intr_;
      c_intr = use_color ? color_intr_ : depth_intr_;
      off_x = use_color ? color_off_x_ : 0.0f;
      if (has_det2d_ && (t_now - det2d_stamp_).seconds() <= vis_timeout_) {
        boxes = box_rois_;
        pucks = puck_rois_;
      }
      us_r = ctrl_.state().range_m;
      us_ok = ctrl_.state().range_valid;
    }

    if (!DecodeDepth(msg, &depth_buf_)) return;
    const int w = static_cast<int>(msg.width);
    const int h = static_cast<int>(msg.height);

    // Foxglove 프리뷰 구독 시에만 깊이+ROI 축소 영상 생성
    if (vis_.HasImgSub()) {
      std::vector<Roi> d_boxes;
      std::vector<Roi> d_pucks;
      for (const auto& r : boxes) {
        d_boxes.push_back(use_color ? MapRoiToDepth(r, d_intr, c_intr, off_x) : r);
      }
      for (const auto& r : pucks) {
        d_pucks.push_back(use_color ? MapRoiToDepth(r, d_intr, c_intr, off_x) : r);
      }
      vis_.PublishDepthImg(t_now, depth_buf_.data(), w, h, d_boxes, d_pucks);
    }

    std::lock_guard<std::mutex> lk(mtx_);
    const auto vis = pca_.Detect(depth_buf_.data(), w, h, d_intr, c_intr, off_x,
                                 boxes, pucks, us_r, us_ok);
    if (vis[8] > 0.5f) {
      vis_stamp_ = t_now;
      raw_vis_stamp_ = t_now;
      has_vis_ = true;
      has_raw_vis_ = true;
    }

    // Foxglove 점군 구독 시에만 바닥·상자벽면·퍽 점군 캐시 갱신
    if (vis_.HasCloudSub()) {
      vis_floor_.clear();
      vis_wall_.clear();
      vis_pucks_.clear();
      const auto& cam = ctrl_.calib().cam;
      const Roi f_roi{w / 8, h / 3, (w * 7) / 8, (h * 15) / 16};
      for (const auto& p : pca_.BackprojectBase(depth_buf_.data(), w, h, f_roi,
                                                d_intr, d_intr, 0.0f, 5)) {
        if (std::abs(p.z() + cam.base_z_m) < 0.030 && p.x() < 1.2f) {
          vis_floor_.push_back(p);
        }
      }
      if (!boxes.empty()) {
        for (const auto& p : pca_.BackprojectBase(
                 depth_buf_.data(), w, h, boxes.front(), d_intr, c_intr, off_x, 2)) {
          const double zg = p.z() + cam.base_z_m;
          if (zg > 0.008 && zg < cam.box_height_m - 0.008) vis_wall_.push_back(p);
        }
      }
      for (size_t i = 0; i < std::min<size_t>(2, pucks.size()); ++i) {
        for (const auto& p : pca_.BackprojectBase(
                 depth_buf_.data(), w, h, pucks[i], d_intr, c_intr, off_x, 2)) {
          if (p.z() + cam.base_z_m > cam.box_height_m + 0.002) {
            vis_pucks_.push_back(p);
          }
        }
      }
    }
  }

  // 50Hz 상위 정책 실행, 텔레메트리·Foxglove 발행 및 액션 상태 처리
  void OnPolicy50Hz() {
    const rclcpp::Time t_now = now();
    CtrlState st;
    VisFrame vf;
    const bool want_vis = vis_.Has3dSub();
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (has_sonar_ && (t_now - sonar_stamp_).seconds() > sonar_timeout_) {
        ctrl_.UpdateSonar(4.0, false);
      }
      Eigen::Matrix<float, 9, 1> vis = pca_.last_vis();
      if (!has_vis_ || (t_now - vis_stamp_).seconds() > vis_timeout_) {
        vis[8] = 0.0f;
      }
      st = ctrl_.StepPolicy(vis, 0.02f);
      HandleGoalLocked(t_now, st);

      if (want_vis) {
        vf.calib = ctrl_.calib();
        vf.target_gap_m = vf.calib.cam.target_gap_m;
        vf.box_valid = vis[8] > 0.5f;
        vf.norm_xy = vis.segment<2>(2);
        vf.box_xy = vis.segment<2>(0) +
                    static_cast<float>(vf.calib.cam.standoff_m) * vf.norm_xy;
        vf.us_valid = st.range_valid;
        vf.us_raw_m = ctrl_.us_raw_m();
        vf.us_corr_m = st.range_m;
        vf.proj_grav_b = ctrl_.proj_grav_b();
        vf.floor_pts = vis_floor_;
        vf.wall_pts = vis_wall_;
        vf.puck_pts = vis_pucks_;
      }
    }

    if (want_vis) vis_.Publish(t_now, vf);

    std_msgs::msg::Float32MultiArray msg;
    msg.data = {st.active ? 1.0f : 0.0f,
                st.stopped ? 1.0f : 0.0f,
                st.aligned ? 1.0f : 0.0f,
                st.reached ? 1.0f : 0.0f,
                st.visible,
                st.dist_m,
                st.yaw_rad,
                st.hold_sec,
                st.cmd_vel.x(),
                st.cmd_vel.y(),
                st.cmd_vel.z(),
                st.range_m};
    status_pub_->publish(msg);

    std_msgs::msg::Bool r_msg;
    r_msg.data = st.reached;
    reached_pub_->publish(r_msg);
  }

  // 활성 ApproachBox 액션 목표에 대한 피드백 및 종료 응답 처리
  void HandleGoalLocked(const rclcpp::Time& t_now, const CtrlState& st) {
    if (!goal_) return;
    const float yaw_deg = st.yaw_rad * static_cast<float>(180.0 / M_PI);

    auto make_res = [&](bool ok, const std::string& text) {
      auto res = std::make_shared<ApproachAct::Result>();
      res->success = ok;
      res->message = text;
      res->dist_m = st.dist_m;
      res->yaw_deg = yaw_deg;
      res->range_m = st.range_m;
      return res;
    };

    if (goal_->is_canceling()) {
      ctrl_.SetActive(false);
      goal_->canceled(make_res(false, "정밀 접근 취소됨"));
      goal_.reset();
      return;
    }
    if (st.reached) {
      ctrl_.SetActive(false);
      goal_->succeed(make_res(true, "2cm 정밀 접근 및 정지 완료"));
      goal_.reset();
      RCLCPP_INFO(get_logger(), "2cm 정밀 접근 완료 (dist=%.3fm, yaw=%.2fdeg)",
                  st.dist_m, yaw_deg);
      return;
    }
    if ((t_now - goal_start_).seconds() > goal_timeout_) {
      ctrl_.SetActive(false);
      goal_->abort(make_res(false, "정밀 접근 제한 시간 초과"));
      goal_.reset();
      RCLCPP_WARN(get_logger(), "정밀 접근 시간 초과 (dist=%.3fm)", st.dist_m);
      return;
    }

    if (++fb_tick_ % 5 == 0) {
      auto fb = std::make_shared<ApproachAct::Feedback>();
      fb->dist_m = st.dist_m;
      fb->yaw_deg = yaw_deg;
      fb->hold_sec = st.hold_sec;
      fb->range_m = st.range_m;
      fb->stopped = st.stopped;
      fb->visible = st.visible > 0.5f;
      goal_->publish_feedback(fb);
    }
  }

  // 200Hz WalkNet 실행 및 보행 중에만 팔 홈 자세 발행 (도달 후 팔 제어권 양보)
  void OnWalk200Hz() {
    std::vector<double> leg_cmds;
    std::vector<double> arm_cmds;
    bool pub_legs = false;
    bool pub_arm = false;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      ctrl_.StepWalk(0.005f, &leg_cmds, &arm_cmds);
      const bool dry = dry_run_ || ctrl_.calib().gait.dry_run ||
                       (goal_ && goal_dry_run_);
      pub_legs = !dry && (ctrl_.active() || (stand_when_idle_ && seen_joints_));
      pub_arm = pub_legs && arm_pub_ && hold_arm_walking_ && ctrl_.active() &&
                !ctrl_.reached();
    }

    if (pub_legs) {
      std_msgs::msg::Float64MultiArray l_msg;
      l_msg.data = std::move(leg_cmds);
      leg_pub_->publish(l_msg);
    }
    if (pub_arm) {
      std_msgs::msg::Float64MultiArray a_msg;
      a_msg.data = std::move(arm_cmds);
      arm_pub_->publish(a_msg);
    }
  }

  std::mutex mtx_;
  FoxgloveVis vis_;
  std::string calib_path_;
  float color_off_x_ = -0.015f;
  double vis_timeout_ = 0.35;
  double sonar_timeout_ = 0.35;
  double def_goal_timeout_ = 35.0;
  bool hold_arm_walking_ = true;
  bool stand_when_idle_ = false;
  bool dry_run_ = false;
  bool seen_joints_ = false;

  ApproachCtrl ctrl_;
  PcaVis pca_;
  std::vector<float> depth_buf_;
  std::vector<Eigen::Vector3f> vis_floor_;
  std::vector<Eigen::Vector3f> vis_wall_;
  std::vector<Eigen::Vector3f> vis_pucks_;

  Intr depth_intr_;
  Intr color_intr_;
  bool has_depth_intr_ = false;
  bool has_color_intr_ = false;

  std::vector<Roi> box_rois_;
  std::vector<Roi> puck_rois_;
  rclcpp::Time det2d_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time vis_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time raw_vis_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time sonar_stamp_{0, 0, RCL_ROS_TIME};
  bool has_det2d_ = false;
  bool has_vis_ = false;
  bool has_raw_vis_ = false;
  bool has_sonar_ = false;

  std::shared_ptr<GoalHandle> goal_;
  rclcpp::Time goal_start_{0, 0, RCL_ROS_TIME};
  double goal_timeout_ = 35.0;
  bool goal_dry_run_ = false;
  uint32_t fb_tick_ = 0;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr alt_range_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det2d_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr det3d_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_srv_;
  rclcpp_action::Server<ApproachAct>::SharedPtr action_srv_;

  rclcpp::TimerBase::SharedPtr policy_timer_;
  rclcpp::TimerBase::SharedPtr walk_timer_;
};

}  // namespace xsting_approach

// 정밀 접근 노드 실행
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xsting_approach::ApproachNode>());
  rclcpp::shutdown();
  return 0;
}
