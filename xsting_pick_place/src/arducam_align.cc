#include "xsting_pick_place/arducam_align.h"

#include <NvInfer.h>
#include <NvOnnxParser.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_path.hpp>
#include <cuda_buffer/cuda_buffer_api.hpp>
#include <opencv2/imgproc.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

namespace xsting_pick_place {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kBevSize = 320;
constexpr double kStepM = 0.0005;
constexpr int kYoloW = 640;
constexpr int kYoloH = 480;
constexpr int kYoloAnchors = 6300;
constexpr int kYoloClasses = 5;
constexpr int kYoloRows = 4 + kYoloClasses;

constexpr std::array<const char*, kYoloClasses> kClassNames = {
    "red_square", "green_square", "red_cylinder", "green_cylinder",
    "paper_box"};

// TensorRT 경고·오류만 ROS 로거로 전달하는 로거 구현체
class TrtLogger : public nvinfer1::ILogger {
 public:
  explicit TrtLogger(rclcpp::Logger logger) : logger_(logger) {}

  void log(Severity severity, const char* msg) noexcept override {
    if (severity == Severity::kINTERNAL_ERROR || severity == Severity::kERROR) {
      RCLCPP_WARN(logger_, "[TRT] %s", msg);
    }
  }

 private:
  rclcpp::Logger logger_;
};

// 클래스 필터 문자열이 타겟 클래스명과 부합하는지 확인
bool MatchFilter(const std::string& cls_name, const std::string& filter) {
  if (filter.empty() || filter == "any" || filter == "all") return true;
  return cls_name == filter || cls_name.find(filter) != std::string::npos;
}

// 두 2D 바운딩 박스의 IoU 산출
double CalcIou(const ArducamAlign::YoloDet& a,
               const ArducamAlign::YoloDet& b) {
  const double ix1 = std::max(a.x1, b.x1);
  const double iy1 = std::max(a.y1, b.y1);
  const double ix2 = std::min(a.x2, b.x2);
  const double iy2 = std::min(a.y2, b.y2);
  const double iw = std::max(0.0, ix2 - ix1);
  const double ih = std::max(0.0, iy2 - iy1);
  const double inter = iw * ih;
  if (inter <= 0.0) return 0.0;
  const double area_a = std::max(1.0, (a.x2 - a.x1) * (a.y2 - a.y1));
  const double area_b = std::max(1.0, (b.x2 - b.x1) * (b.y2 - b.y1));
  return inter / (area_a + area_b - inter);
}

}  // namespace

// TensorRT YOLOv8 런타임·엔진·실행 컨텍스트 및 입출력 버퍼 관리체
struct ArducamAlign::TrtYolo {
  TrtLogger logger;
  std::mutex mtx;
  std::unique_ptr<nvinfer1::IRuntime> runtime;
  std::unique_ptr<nvinfer1::ICudaEngine> engine;
  std::unique_ptr<nvinfer1::IExecutionContext> context;
  float* d_in = nullptr;
  float* d_out = nullptr;
  std::vector<float> h_in;
  std::vector<float> h_out;
  bool ready = false;

  explicit TrtYolo(rclcpp::Logger log)
      : logger(log),
        h_in(3 * kYoloH * kYoloW, 0.0f),
        h_out(kYoloRows * kYoloAnchors, 0.0f) {}

  ~TrtYolo() {
    context.reset();
    engine.reset();
    runtime.reset();
    if (d_in) cudaFree(d_in);
    if (d_out) cudaFree(d_out);
  }

  // 직렬화된 엔진 파일 역직렬화 시도
  bool LoadEngineFile(const std::filesystem::path& path) {
    if (!std::filesystem::exists(path)) return false;
    std::ifstream ifs(path, std::ios::binary | std::ios::ate);
    if (!ifs) return false;
    const std::streamsize sz = ifs.tellg();
    if (sz <= 0) return false;
    ifs.seekg(0, std::ios::beg);
    std::vector<char> buf(static_cast<size_t>(sz));
    if (!ifs.read(buf.data(), sz)) return false;

    engine.reset(runtime->deserializeCudaEngine(buf.data(), buf.size()));
    return engine != nullptr;
  }

  // 엔진 버전 불일치 시 ONNX 원본으로부터 FP16 엔진 빌드 및 캐싱
  bool BuildFromOnnx(const std::filesystem::path& onnx_path,
                     const std::filesystem::path& cache_path) {
    if (!std::filesystem::exists(onnx_path)) return false;
    std::unique_ptr<nvinfer1::IBuilder> builder(
        nvinfer1::createInferBuilder(logger));
    if (!builder) return false;
    std::unique_ptr<nvinfer1::INetworkDefinition> net(
        builder->createNetworkV2(0U));
    if (!net) return false;
    std::unique_ptr<nvonnxparser::IParser> parser(
        nvonnxparser::createParser(*net, logger));
    if (!parser || !parser->parseFromFile(
                       onnx_path.c_str(),
                       static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
      return false;
    }

    std::unique_ptr<nvinfer1::IBuilderConfig> cfg(
        builder->createBuilderConfig());
    if (!cfg) return false;
    cfg->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE,
                            1ULL << 29);

    std::unique_ptr<nvinfer1::IHostMemory> plan(
        builder->buildSerializedNetwork(*net, *cfg));
    if (!plan) return false;

    std::ofstream ofs(cache_path, std::ios::binary);
    if (ofs) {
      ofs.write(static_cast<const char*>(plan->data()),
                static_cast<std::streamsize>(plan->size()));
    }
    engine.reset(runtime->deserializeCudaEngine(plan->data(), plan->size()));
    return engine != nullptr;
  }

  // xsting_vision 공유 경로에서 5클래스 YOLOv8 모델 로드 및 GPU 메모리 바인딩
  bool Init(const rclcpp::Logger& log) {
    std::filesystem::path model_dir;
    try {
      model_dir =
          ament_index_cpp::get_package_share_path("xsting_vision") / "model";
    } catch (const std::exception&) {
      return false;
    }

#if defined(__aarch64__)
    const std::string arch = "arm64";
#else
    const std::string arch = "x86_64";
#endif

    runtime.reset(nvinfer1::createInferRuntime(logger));
    if (!runtime) return false;

    const std::filesystem::path p_small =
        model_dir / arch / "rg_small_batch1_640_fp16.engine";
    const std::filesystem::path p_nano =
        model_dir / arch / "rg_nano_batch1_640_fp16.engine";
    const std::filesystem::path p_cache =
        "/tmp/xsting_arducam_rg_small_640_fp16.engine";
    const std::filesystem::path p_onnx_s =
        model_dir / "onnx" / "rg_small_batch1_640_fp16.onnx";
    const std::filesystem::path p_onnx_n =
        model_dir / "onnx" / "rg_nano_batch1_640_fp16.onnx";

    std::string loaded_name;
    if (LoadEngineFile(p_cache)) {
      loaded_name = p_cache.filename().string();
    } else if (LoadEngineFile(p_small)) {
      loaded_name = p_small.filename().string();
    } else if (BuildFromOnnx(p_onnx_s, p_cache)) {
      loaded_name = "rg_small_batch1_640_fp16.onnx(built)";
    } else if (LoadEngineFile(p_nano)) {
      loaded_name = p_nano.filename().string();
    } else if (BuildFromOnnx(p_onnx_n, p_cache)) {
      loaded_name = "rg_nano_batch1_640_fp16.onnx(built)";
    } else {
      RCLCPP_WARN(log, "ArduCam YOLOv8 엔진 로드 실패: 컬러 기하 보정만 사용");
      return false;
    }

    context.reset(engine->createExecutionContext());
    if (!context) return false;

    if (cudaMalloc(reinterpret_cast<void**>(&d_in),
                   h_in.size() * sizeof(float)) != cudaSuccess ||
        cudaMalloc(reinterpret_cast<void**>(&d_out),
                   h_out.size() * sizeof(float)) != cudaSuccess) {
      return false;
    }

    ready = context->setTensorAddress("images", d_in) &&
            context->setTensorAddress("output0", d_out);
    if (ready) {
      RCLCPP_INFO(log, "ArduCam YOLOv8 5클래스(퍽·상자) 엔진 준비 완료: %s",
                  loaded_name.c_str());
    }
    return ready;
  }
};

// ArduCam 왜곡 보정 영상 구독 및 YOLOv8 엔진 초기화
ArducamAlign::ArducamAlign(const rclcpp::Node::SharedPtr& node,
                           const std::string& topic_ns)
    : node_(node), yolo_(std::make_unique<TrtYolo>(node->get_logger())) {
  bool sim_time = false;
  node_->get_parameter_or("use_sim_time", sim_time, false);
  if (!node_->has_parameter("arducam_flip_180")) {
    flip_180_ = node_->declare_parameter("arducam_flip_180", !sim_time);
  } else {
    node_->get_parameter("arducam_flip_180", flip_180_);
  }
  cudaStreamCreateWithFlags(&cuda_stream_, cudaStreamNonBlocking);
  yolo_->Init(node_->get_logger());

  const auto qos = rclcpp::SensorDataQoS();
  rclcpp::SubscriptionOptions any_opts;
  any_opts.acceptable_buffer_backends = "any";

  info_sub_ = node_->create_subscription<CamInfoMsg>(
      topic_ns + "/camera_info", qos,
      [this](const CamInfoMsg::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        latest_info_ = msg;
      });
  img_sub_ = node_->create_subscription<ImageMsg>(
      topic_ns + "/image_rect", qos,
      [this](const ImageMsg::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        latest_img_ = msg;
        last_rx_ = std::chrono::steady_clock::now();
        ++img_seq_;
      },
      any_opts);
  det_pub_ = node_->create_publisher<Det2DArray>(topic_ns + "/detections", 10);
  debug_pub_ =
      node_->create_publisher<ImageMsg>(topic_ns + "/puck_align_debug", qos);
}

// 구독 해제 후 YOLOv8 컨텍스트와 CUDA 스트림 정리
ArducamAlign::~ArducamAlign() {
  img_sub_.reset();
  info_sub_.reset();
  if (cuda_stream_) cudaStreamSynchronize(cuda_stream_);
  yolo_.reset();
  if (cuda_stream_) cudaStreamDestroy(cuda_stream_);
}

// 최근 1.5초 이내에 카메라 프레임과 내부 파라미터가 수신되었는지 판별
bool ArducamAlign::HasActiveStream() const {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!latest_img_ || !latest_info_) return false;
  const auto age = std::chrono::steady_clock::now() - last_rx_;
  return age <= std::chrono::milliseconds(1500);
}

// 팔 정지 전 인플라이트 렌더링 프레임을 비우기 위해 최소 min_new장 이후 프레임 대기
bool ArducamAlign::WaitFreshFrame(int min_new, int timeout_ms,
                                  ImageMsg::ConstSharedPtr& out_img,
                                  CamInfoMsg::ConstSharedPtr& out_info) {
  uint64_t base_seq = 0;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!latest_info_) return false;
    base_seq = img_seq_;
  }

  const int steps = std::max(1, timeout_ms / 10);
  for (int i = 0; i < steps; ++i) {
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (img_seq_ >= base_seq + static_cast<uint64_t>(min_new) &&
          latest_img_ && latest_info_) {
        out_img = latest_img_;
        out_info = latest_info_;
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::lock_guard<std::mutex> lock(mtx_);
  out_img = latest_img_;
  out_info = latest_info_;
  return static_cast<bool>(out_img && out_info);
}

// GPU 버퍼(실기체)와 CPU 버퍼(Isaac Sim) 모두에서 RGB8 이미지를 복사
bool ArducamAlign::CopyRgb(const ImageMsg& msg, cv::Mat& rgb) {
  if (msg.width == 0 || msg.height == 0) return false;
  const bool is_rgb = (msg.encoding == "rgb8" || msg.encoding == "bgr8");
  const bool is_rgba = (msg.encoding == "rgba8" || msg.encoding == "bgra8");
  if (!is_rgb && !is_rgba) return false;

  const size_t ch = is_rgba ? 4 : 3;
  const size_t row_bytes = static_cast<size_t>(msg.width) * ch;
  const size_t min_bytes =
      static_cast<size_t>(msg.height - 1) * msg.step + row_bytes;
  if (msg.data.size() < min_bytes) return false;

  cv::Mat raw(static_cast<int>(msg.height), static_cast<int>(msg.width),
              is_rgba ? CV_8UC4 : CV_8UC3);
  {
    auto handle =
        cuda_buffer_backend::from_input_buffer(msg.data, cuda_stream_);
    cudaMemcpy2DAsync(raw.data, raw.step, handle.get_ptr(), msg.step, row_bytes,
                      msg.height, cudaMemcpyDeviceToHost, cuda_stream_);
  }
  cudaStreamSynchronize(cuda_stream_);

  if (msg.encoding == "rgb8") {
    rgb = raw;
  } else if (msg.encoding == "bgr8") {
    cv::cvtColor(raw, rgb, cv::COLOR_BGR2RGB);
  } else if (msg.encoding == "rgba8") {
    cv::cvtColor(raw, rgb, cv::COLOR_RGBA2RGB);
  } else {
    cv::cvtColor(raw, rgb, cv::COLOR_BGRA2RGB);
  }
  return true;
}

// CameraInfo로부터 이미지 크기에 맞춰 스케일된 초점거리·주점 산출
bool ArducamAlign::GetScaledIntrinsics(const CamInfoMsg& info, int cols,
                                       int rows, double& fx, double& fy,
                                       double& cx, double& cy) const {
  const bool use_p = (info.p[0] > 1.0 && info.p[5] > 1.0);
  fx = use_p ? info.p[0] : info.k[0];
  fy = use_p ? info.p[5] : info.k[4];
  cx = use_p ? info.p[2] : info.k[2];
  cy = use_p ? info.p[6] : info.k[5];
  if (fx <= 1.0 || fy <= 1.0) return false;

  if (info.width > 0 && info.height > 0 &&
      (static_cast<int>(info.width) != cols ||
       static_cast<int>(info.height) != rows)) {
    const double sx = static_cast<double>(cols) / info.width;
    const double sy = static_cast<double>(rows) / info.height;
    fx *= sx;
    cx *= sx;
    fy *= sy;
    cy *= sy;
  }
  return true;
}

// 이미지 픽셀 광선을 월드 수평면 z = z_plane과 교차시켜 (wx, wy) 계산
bool ArducamAlign::PixelToPlane(double u, double v, double fx, double fy,
                                double cx, double cy, const Vec3& cam_pos,
                                const std::array<Vec3, 3>& cam_axes,
                                double z_plane, double& wx,
                                double& wy) const {
  const double rx = (u - cx) / fx;
  const double ry = (v - cy) / fy;
  const Vec3 dir = {
      rx * cam_axes[0][0] + ry * cam_axes[1][0] + cam_axes[2][0],
      rx * cam_axes[0][1] + ry * cam_axes[1][1] + cam_axes[2][1],
      rx * cam_axes[0][2] + ry * cam_axes[1][2] + cam_axes[2][2]};
  if (dir[2] >= -1e-4) return false;

  const double t = (z_plane - cam_pos[2]) / dir[2];
  if (t <= 0.02) return false;
  wx = cam_pos[0] + t * dir[0];
  wy = cam_pos[1] + t * dir[1];
  return true;
}

// 640x480 레터박스 전처리 후 YOLOv8 추론·NMS 수행 및 2D 검출 결과 발행
std::vector<ArducamAlign::YoloDet> ArducamAlign::InferYolo(
    const cv::Mat& rgb, const std_msgs::msg::Header& header, float conf_th) {
  if (!yolo_ || !yolo_->ready || rgb.empty()) return {};
  std::lock_guard<std::mutex> lock(yolo_->mtx);

  const double scale = std::min(static_cast<double>(kYoloW) / rgb.cols,
                                static_cast<double>(kYoloH) / rgb.rows);
  const int nw = std::max(1, static_cast<int>(std::round(rgb.cols * scale)));
  const int nh = std::max(1, static_cast<int>(std::round(rgb.rows * scale)));
  const int pad_x = (kYoloW - nw) / 2;
  const int pad_y = (kYoloH - nh) / 2;

  cv::Mat canvas(kYoloH, kYoloW, CV_8UC3, cv::Scalar(114, 114, 114));
  cv::Mat roi = canvas(cv::Rect(pad_x, pad_y, nw, nh));
  if (nw == rgb.cols && nh == rgb.rows) {
    rgb.copyTo(roi);
  } else {
    cv::resize(rgb, roi, cv::Size(nw, nh), 0.0, 0.0,
               (scale < 1.0) ? cv::INTER_AREA : cv::INTER_LINEAR);
  }

  // CHW RGB [0, 1] 정규화 버퍼 적재
  const size_t plane = static_cast<size_t>(kYoloW) * kYoloH;
  float* ch_r = yolo_->h_in.data();
  float* ch_g = ch_r + plane;
  float* ch_b = ch_g + plane;
  constexpr float kInv255 = 1.0f / 255.0f;
  for (int y = 0; y < kYoloH; ++y) {
    const uint8_t* row = canvas.ptr<uint8_t>(y);
    const size_t base = static_cast<size_t>(y) * kYoloW;
    for (int x = 0; x < kYoloW; ++x) {
      const size_t idx = base + static_cast<size_t>(x);
      ch_r[idx] = static_cast<float>(row[3 * x + 0]) * kInv255;
      ch_g[idx] = static_cast<float>(row[3 * x + 1]) * kInv255;
      ch_b[idx] = static_cast<float>(row[3 * x + 2]) * kInv255;
    }
  }

  cudaMemcpyAsync(yolo_->d_in, yolo_->h_in.data(),
                  yolo_->h_in.size() * sizeof(float), cudaMemcpyHostToDevice,
                  cuda_stream_);
  if (!yolo_->context->enqueueV3(cuda_stream_)) return {};
  cudaMemcpyAsync(yolo_->h_out.data(), yolo_->d_out,
                  yolo_->h_out.size() * sizeof(float), cudaMemcpyDeviceToHost,
                  cuda_stream_);
  cudaStreamSynchronize(cuda_stream_);

  // 9x6300 출력 텐서 디코딩 및 원본 해상도 좌표 복원
  std::vector<YoloDet> raw;
  const float* out = yolo_->h_out.data();
  for (int a = 0; a < kYoloAnchors; ++a) {
    int best_c = -1;
    float best_s = conf_th;
    for (int c = 0; c < kYoloClasses; ++c) {
      const float s = out[(4 + c) * kYoloAnchors + a];
      if (s > best_s) {
        best_s = s;
        best_c = c;
      }
    }
    if (best_c < 0) continue;

    const double bx = out[0 * kYoloAnchors + a];
    const double by = out[1 * kYoloAnchors + a];
    const double bw = out[2 * kYoloAnchors + a];
    const double bh = out[3 * kYoloAnchors + a];

    YoloDet d;
    d.cls_id = best_c;
    d.cls_name = kClassNames[static_cast<size_t>(best_c)];
    d.score = best_s;
    d.x1 = std::clamp((bx - 0.5 * bw - pad_x) / scale, 0.0,
                      static_cast<double>(rgb.cols - 1));
    d.y1 = std::clamp((by - 0.5 * bh - pad_y) / scale, 0.0,
                      static_cast<double>(rgb.rows - 1));
    d.x2 = std::clamp((bx + 0.5 * bw - pad_x) / scale, 0.0,
                      static_cast<double>(rgb.cols - 1));
    d.y2 = std::clamp((by + 0.5 * bh - pad_y) / scale, 0.0,
                      static_cast<double>(rgb.rows - 1));
    if (d.x2 - d.x1 >= 6.0 && d.y2 - d.y1 >= 6.0) raw.push_back(d);
  }

  std::sort(raw.begin(), raw.end(),
            [](const YoloDet& a, const YoloDet& b) { return a.score > b.score; });

  // 퍽(0~3)은 클래스 통합 NMS로 중복 검출을 제거하고 상자(4)는 별도 NMS 적용
  std::vector<YoloDet> kept;
  for (const auto& cand : raw) {
    bool sup = false;
    for (const auto& prev : kept) {
      const bool same_group = (cand.cls_id == 4) == (prev.cls_id == 4);
      if (same_group && CalcIou(cand, prev) > 0.45) {
        sup = true;
        break;
      }
    }
    if (!sup) kept.push_back(cand);
  }

  if (det_pub_) {
    Det2DArray msg;
    msg.header = header;
    for (const auto& d : kept) {
      vision_msgs::msg::Detection2D item;
      item.header = header;
      item.bbox.center.position.x = 0.5 * (d.x1 + d.x2);
      item.bbox.center.position.y = 0.5 * (d.y1 + d.y2);
      item.bbox.size_x = d.x2 - d.x1;
      item.bbox.size_y = d.y2 - d.y1;
      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = d.cls_name;
      hyp.hypothesis.score = d.score;
      item.results.push_back(hyp);
      msg.detections.push_back(item);
    }
    det_pub_->publish(msg);
  }
  return kept;
}

// 프레임 내 온전히 들어온 paper_box(클래스 4)로부터 상자 중심 XY 자동 보정
void ArducamAlign::TryUpdateBoxCenter(const std::vector<YoloDet>& dets,
                                      int cols, int rows, double fx, double fy,
                                      double cx, double cy,
                                      const Vec3& cam_pos,
                                      const std::array<Vec3, 3>& cam_axes,
                                      double box_top_z, double& io_box_x,
                                      double& io_box_y) const {
  for (const auto& d : dets) {
    if (d.cls_id != 4 || d.score < 0.42f) continue;
    if (d.x1 <= 8.0 || d.y1 <= 8.0 || d.x2 >= cols - 9.0 ||
        d.y2 >= rows - 9.0) {
      continue;
    }

    double wx1 = 0.0, wy1 = 0.0, wx2 = 0.0, wy2 = 0.0, wcx = 0.0, wcy = 0.0;
    if (!PixelToPlane(d.x1, d.y1, fx, fy, cx, cy, cam_pos, cam_axes, box_top_z,
                      wx1, wy1) ||
        !PixelToPlane(d.x2, d.y2, fx, fy, cx, cy, cam_pos, cam_axes, box_top_z,
                      wx2, wy2) ||
        !PixelToPlane(0.5 * (d.x1 + d.x2), 0.5 * (d.y1 + d.y2), fx, fy, cx, cy,
                      cam_pos, cam_axes, box_top_z, wcx, wcy)) {
      continue;
    }

    const double span_x = std::abs(wx2 - wx1);
    const double span_y = std::abs(wy2 - wy1);
    if (span_x >= 0.145 && span_x <= 0.275 && span_y >= 0.145 &&
        span_y <= 0.275 && wcx >= 0.18 && wcx <= 0.40 &&
        std::abs(wcy) <= 0.15) {
      io_box_x = wcx;
      io_box_y = wcy;
      RCLCPP_INFO(node_->get_logger(),
                  "YOLOv8 상자(paper_box) 검출: center=(%.3f, %.3f) score=%.2f",
                  io_box_x, io_box_y, d.score);
      break;
    }
  }
}

// 타겟 YOLO 퍽 주변 여유 영역만 남기고 다른 퍽 내부를 마스킹한 이미지 ROI 생성
cv::Mat ArducamAlign::BuildRoiMask(int cols, int rows,
                                   const std::vector<YoloDet>& dets,
                                   int target_idx) const {
  if (target_idx < 0 || static_cast<size_t>(target_idx) >= dets.size()) {
    return cv::Mat();
  }
  const auto& t = dets[static_cast<size_t>(target_idx)];
  const double pad_w = std::max(18.0, 0.22 * (t.x2 - t.x1));
  const double pad_h = std::max(18.0, 0.22 * (t.y2 - t.y1));

  const int rx1 = std::clamp(static_cast<int>(std::floor(t.x1 - pad_w)), 0,
                             cols - 1);
  const int ry1 = std::clamp(static_cast<int>(std::floor(t.y1 - pad_h)), 0,
                             rows - 1);
  const int rx2 = std::clamp(static_cast<int>(std::ceil(t.x2 + pad_w)), 0,
                             cols - 1);
  const int ry2 = std::clamp(static_cast<int>(std::ceil(t.y2 + pad_h)), 0,
                             rows - 1);

  cv::Mat mask(rows, cols, CV_8UC1, cv::Scalar(0));
  if (rx2 > rx1 && ry2 > ry1) {
    cv::rectangle(mask, cv::Point(rx1, ry1), cv::Point(rx2, ry2),
                  cv::Scalar(255), cv::FILLED);
  }

  const double tcx = 0.5 * (t.x1 + t.x2);
  const double tcy = 0.5 * (t.y1 + t.y2);
  for (size_t i = 0; i < dets.size(); ++i) {
    if (static_cast<int>(i) == target_idx || dets[i].cls_id == 4) continue;
    const auto& o = dets[i];
    const double mx = 0.18 * (o.x2 - o.x1);
    const double my = 0.18 * (o.y2 - o.y1);
    const int ox1 = std::clamp(static_cast<int>(std::round(o.x1 + mx)), 0,
                               cols - 1);
    const int oy1 = std::clamp(static_cast<int>(std::round(o.y1 + my)), 0,
                               rows - 1);
    const int ox2 = std::clamp(static_cast<int>(std::round(o.x2 - mx)), 0,
                               cols - 1);
    const int oy2 = std::clamp(static_cast<int>(std::round(o.y2 - my)), 0,
                               rows - 1);
    if (ox2 > ox1 && oy2 > oy1 &&
        !(tcx >= ox1 && tcx <= ox2 && tcy >= oy1 && tcy <= oy2)) {
      cv::rectangle(mask, cv::Point(ox1, oy1), cv::Point(ox2, oy2),
                    cv::Scalar(0), cv::FILLED);
    }
  }
  return mask;
}

// 상자 전체가 보이는 고고도 관측 자세에서 YOLOv8로 상자와 퍽들을 검출
bool ArducamAlign::ScanBox(const Joint5& arm_joints, double ground_z,
                           const std::string& filter_cls, bool auto_box_det,
                           double& io_box_x, double& io_box_y,
                           std::vector<Puck>& out_pucks, bool allow_empty) {
  out_pucks.clear();
  ImageMsg::ConstSharedPtr snap_img;
  CamInfoMsg::ConstSharedPtr snap_info;
  if (!WaitFreshFrame(2, 550, snap_img, snap_info)) return false;

  cv::Mat rgb;
  if (!CopyRgb(*snap_img, rgb)) return false;

  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
  if (!GetScaledIntrinsics(*snap_info, rgb.cols, rgb.rows, fx, fy, cx, cy)) {
    return false;
  }

  Vec3 cam_pos = {};
  std::array<Vec3, 3> cam_axes = {};
  ForwardArducam(arm_joints, cam_pos, cam_axes);
  if (flip_180_) {
    for (int k = 0; k < 3; ++k) {
      cam_axes[0][k] = -cam_axes[0][k];
      cam_axes[1][k] = -cam_axes[1][k];
    }
  }

  const double box_top_z = ground_z + 0.070;
  const auto dets = InferYolo(rgb, snap_info->header, 0.28f);
  if (auto_box_det) {
    TryUpdateBoxCenter(dets, rgb.cols, rgb.rows, fx, fy, cx, cy, cam_pos,
                       cam_axes, box_top_z, io_box_x, io_box_y);
  }

  for (size_t i = 0; i < dets.size(); ++i) {
    const auto& d = dets[i];
    if (d.cls_id < 0 || d.cls_id >= 4) continue;
    if (!MatchFilter(d.cls_name, filter_cls)) continue;

    double sx = 0.0;
    double sy = 0.0;
    if (!PixelToPlane(0.5 * (d.x1 + d.x2), 0.5 * (d.y1 + d.y2), fx, fy, cx, cy,
                      cam_pos, cam_axes, box_top_z + 0.045, sx, sy)) {
      continue;
    }
    if (std::abs(sx - io_box_x) > 0.115 || std::abs(sy - io_box_y) > 0.115) {
      continue;
    }

    const bool is_sq = (d.cls_id == 0 || d.cls_id == 1);
    const double det_score =
        std::clamp(static_cast<double>(d.score), 0.01, 1.0);
    Puck cand = {
        {sx, sy, box_top_z + 0.030}, 0.0, is_sq, false, d.cls_name, det_score};
    const cv::Mat roi_mask =
        BuildRoiMask(rgb.cols, rgb.rows, dets, static_cast<int>(i));

    bool det_lying = false;
    if (FitOnPlane(rgb, roi_mask, *snap_info, cam_pos, cam_axes, io_box_x,
                   io_box_y, box_top_z + 0.060, nullptr, cand, det_lying)) {
      if (det_lying) {
        Puck lying_cand = cand;
        bool check_lying = true;
        if (FitOnPlane(rgb, roi_mask, *snap_info, cam_pos, cam_axes, io_box_x,
                       io_box_y, box_top_z + 0.030, nullptr, lying_cand,
                       check_lying)) {
          cand = lying_cand;
          det_lying = check_lying;
        }
      }
      cand.is_lying = det_lying;
    } else if (FitOnPlane(rgb, roi_mask, *snap_info, cam_pos, cam_axes,
                          io_box_x, io_box_y, box_top_z + 0.030, nullptr, cand,
                          det_lying)) {
      cand.is_lying = det_lying;
    }
    cand.pos[2] = box_top_z + (cand.is_lying ? 0.015 : 0.030);
    cand.score = det_score;

    bool dup = false;
    for (const auto& prev : out_pucks) {
      if (std::hypot(cand.pos[0] - prev.pos[0], cand.pos[1] - prev.pos[1]) <
          0.024) {
        dup = true;
        break;
      }
    }
    if (!dup) out_pucks.push_back(cand);
  }

  std::sort(out_pucks.begin(), out_pucks.end(),
            [](const Puck& a, const Puck& b) {
              if (a.is_lying != b.is_lying) return !a.is_lying;
              return std::hypot(a.pos[0] - 0.108, a.pos[1]) <
                     std::hypot(b.pos[0] - 0.108, b.pos[1]);
            });

  RCLCPP_INFO(node_->get_logger(),
              "ArduCam YOLOv8 상자 스캔 완료: 퍽 %zu개 검출 (box=%.3f, %.3f)",
              out_pucks.size(), io_box_x, io_box_y);
  return allow_empty ? true : !out_pucks.empty();
}

// 팔 정지 후 새 프레임을 수신해 YOLOv8 ROI + L*a*b*/HSV 탑뷰로 정밀 보정
bool ArducamAlign::RefinePuck(double& io_box_x, double& io_box_y,
                              double box_top_z, bool auto_box_det,
                              const Joint5& arm_joints, const Puck* other,
                              Puck& target) {
  ImageMsg::ConstSharedPtr snap_img;
  CamInfoMsg::ConstSharedPtr snap_info;
  if (!WaitFreshFrame(2, 500, snap_img, snap_info)) return false;

  cv::Mat rgb;
  if (!CopyRgb(*snap_img, rgb)) return false;

  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
  if (!GetScaledIntrinsics(*snap_info, rgb.cols, rgb.rows, fx, fy, cx, cy)) {
    return false;
  }

  Vec3 cam_pos = {};
  std::array<Vec3, 3> cam_axes = {};
  ForwardArducam(arm_joints, cam_pos, cam_axes);
  if (flip_180_) {
    for (int k = 0; k < 3; ++k) {
      cam_axes[0][k] = -cam_axes[0][k];
      cam_axes[1][k] = -cam_axes[1][k];
    }
  }

  Puck cand = target;
  cv::Mat roi_mask;
  bool has_yolo_pt = false;
  double yolo_wx = target.pos[0];
  double yolo_wy = target.pos[1];

  // YOLOv8로 상자·타겟 퍽 클래스를 검증하고 이미지 ROI 마스크 구성
  const auto dets = InferYolo(rgb, snap_info->header, 0.26f);
  if (auto_box_det) {
    TryUpdateBoxCenter(dets, rgb.cols, rgb.rows, fx, fy, cx, cy, cam_pos,
                       cam_axes, box_top_z, io_box_x, io_box_y);
  }

  const bool req_red = (target.cls_id.find("red") != std::string::npos);
  const bool req_grn = (target.cls_id.find("green") != std::string::npos);
  const double z_seed = box_top_z + (target.is_lying ? 0.030 : 0.060);

  int best_det = -1;
  double best_dist = 0.075;
  for (size_t i = 0; i < dets.size(); ++i) {
    const auto& d = dets[i];
    if (d.cls_id < 0 || d.cls_id >= 4) continue;
    if (req_red && d.cls_name.find("red") == std::string::npos) continue;
    if (req_grn && d.cls_name.find("green") == std::string::npos) continue;

    double wx = 0.0;
    double wy = 0.0;
    if (!PixelToPlane(0.5 * (d.x1 + d.x2), 0.5 * (d.y1 + d.y2), fx, fy, cx, cy,
                      cam_pos, cam_axes, z_seed, wx, wy)) {
      continue;
    }
    const double dist = std::hypot(wx - target.pos[0], wy - target.pos[1]);
    if (other) {
      const double d_other =
          std::hypot(wx - other->pos[0], wy - other->pos[1]);
      if (d_other + 0.008 < dist) continue;
    }
    if (dist < best_dist) {
      best_dist = dist;
      best_det = static_cast<int>(i);
      yolo_wx = wx;
      yolo_wy = wy;
    }
  }

  if (best_det >= 0) {
    const auto& md = dets[static_cast<size_t>(best_det)];
    roi_mask = BuildRoiMask(rgb.cols, rgb.rows, dets, best_det);
    has_yolo_pt = (md.score >= 0.45f);
    if (cand.cls_id.empty() || cand.cls_id == "manual_puck") {
      cand.cls_id = md.cls_name;
      cand.is_square = (md.cls_id == 0 || md.cls_id == 1);
    }
    if (best_dist > 0.012) {
      cand.pos[0] = yolo_wx;
      cand.pos[1] = yolo_wy;
    }
  }

  // 예상 높이와 재판별된 높이에서 순차 평면 투영하여 정확한 스케일 보장
  const Puck seed_cand = cand;
  bool det_lying = cand.is_lying;
  const double z1 = box_top_z + (cand.is_lying ? 0.030 : 0.060);
  bool fit_ok = FitOnPlane(rgb, roi_mask, *snap_info, cam_pos, cam_axes,
                           io_box_x, io_box_y, z1, other, cand, det_lying);
  if (!fit_ok) {
    const double z_alt = box_top_z + (seed_cand.is_lying ? 0.060 : 0.030);
    cand = seed_cand;
    fit_ok = FitOnPlane(rgb, roi_mask, *snap_info, cam_pos, cam_axes, io_box_x,
                        io_box_y, z_alt, other, cand, det_lying);
  } else if (det_lying != seed_cand.is_lying) {
    const double z2 = box_top_z + (det_lying ? 0.030 : 0.060);
    Puck refined = seed_cand;
    bool check_lying = det_lying;
    if (FitOnPlane(rgb, roi_mask, *snap_info, cam_pos, cam_axes, io_box_x,
                   io_box_y, z2, other, refined, check_lying)) {
      cand = refined;
      det_lying = check_lying;
    }
  }

  if (!fit_ok) {
    if (!has_yolo_pt) return false;
    cand.pos[0] = yolo_wx;
    cand.pos[1] = yolo_wy;
    det_lying = target.is_lying;
  }

  cand.is_lying = det_lying;
  cand.pos[2] = box_top_z + (det_lying ? 0.015 : 0.030);
  RCLCPP_INFO(node_->get_logger(),
              "ArduCam+YOLO 보정[%s]: (%.3f, %.3f, %.1fdeg) -> "
              "(%.3f, %.3f, %.1fdeg) lying=%d",
              cand.cls_id.c_str(), target.pos[0], target.pos[1],
              target.yaw * 180.0 / kPi, cand.pos[0], cand.pos[1],
              cand.yaw * 180.0 / kPi, static_cast<int>(cand.is_lying));
  target = cand;
  return true;
}

// 160x160mm 실척 탑뷰(0.5mm/px)에서 L*a*b*+HSV 광택 강건 분할 및 볼록 껍질 피팅
bool ArducamAlign::FitOnPlane(const cv::Mat& rgb, const cv::Mat& roi_img,
                              const CamInfoMsg& info, const Vec3& cam_pos,
                              const std::array<Vec3, 3>& cam_axes,
                              double box_x, double box_y, double z_plane,
                              const Puck* other, Puck& io_puck,
                              bool& out_lying) {
  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
  if (!GetScaledIntrinsics(info, rgb.cols, rgb.rows, fx, fy, cx, cy)) {
    return false;
  }

  // 타겟 추정 위치를 중심으로 상자 상판 범위 내 실척 역투영 맵 구성
  const double x0 = io_puck.pos[0];
  const double y0 = io_puck.pos[1];
  const double half_px = 0.5 * (kBevSize - 1);
  cv::Mat map_u(kBevSize, kBevSize, CV_32FC1, cv::Scalar(-1.0f));
  cv::Mat map_v(kBevSize, kBevSize, CV_32FC1, cv::Scalar(-1.0f));

  for (int r = 0; r < kBevSize; ++r) {
    const double wy = y0 + (r - half_px) * kStepM;
    if (std::abs(wy - box_y) > 0.104) continue;
    float* u_row = map_u.ptr<float>(r);
    float* v_row = map_v.ptr<float>(r);

    for (int c = 0; c < kBevSize; ++c) {
      const double wx = x0 + (c - half_px) * kStepM;
      if (std::abs(wx - box_x) > 0.104) continue;

      const Vec3 d = {wx - cam_pos[0], wy - cam_pos[1], z_plane - cam_pos[2]};
      const double zc = d[0] * cam_axes[2][0] + d[1] * cam_axes[2][1] +
                        d[2] * cam_axes[2][2];
      if (zc <= 0.03) continue;
      const double xc = d[0] * cam_axes[0][0] + d[1] * cam_axes[0][1] +
                        d[2] * cam_axes[0][2];
      const double yc = d[0] * cam_axes[1][0] + d[1] * cam_axes[1][1] +
                        d[2] * cam_axes[1][2];
      u_row[c] = static_cast<float>(cx + fx * xc / zc);
      v_row[c] = static_cast<float>(cy + fy * yc / zc);
    }
  }

  cv::Mat bev_rgb;
  cv::remap(rgb, bev_rgb, map_u, map_v, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

  cv::Mat bev_roi;
  if (!roi_img.empty()) {
    cv::remap(roi_img, bev_roi, map_u, map_v, cv::INTER_NEAREST,
              cv::BORDER_CONSTANT);
  }

  // CIE L*a*b* 색차와 HSV 색상 결합으로 하이라이트 탈색 보존 및 목재 상자 배제
  cv::Mat hsv;
  cv::Mat lab;
  cv::cvtColor(bev_rgb, hsv, cv::COLOR_RGB2HSV);
  cv::cvtColor(bev_rgb, lab, cv::COLOR_RGB2Lab);

  const bool want_red = (io_puck.cls_id.find("red") != std::string::npos);
  const bool want_grn = (io_puck.cls_id.find("green") != std::string::npos);
  const cv::Mat k_open =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  const cv::Mat k_close =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

  std::vector<cv::Mat> masks;
  if (want_red || !want_grn) {
    cv::Mat h1, h2, lab_r;
    cv::inRange(hsv, cv::Scalar(0, 22, 28), cv::Scalar(12, 255, 255), h1);
    cv::inRange(hsv, cv::Scalar(165, 22, 28), cv::Scalar(180, 255, 255), h2);
    cv::inRange(lab, cv::Scalar(20, 141, 0), cv::Scalar(255, 255, 255), lab_r);
    cv::Mat rm = (h1 | h2) & lab_r;
    if (!bev_roi.empty()) rm &= bev_roi;
    cv::morphologyEx(rm, rm, cv::MORPH_OPEN, k_open);
    cv::morphologyEx(rm, rm, cv::MORPH_CLOSE, k_close);
    masks.push_back(rm);
  }
  if (want_grn || !want_red) {
    cv::Mat h_g, lab_g;
    cv::inRange(hsv, cv::Scalar(32, 22, 25), cv::Scalar(95, 255, 255), h_g);
    cv::inRange(lab, cv::Scalar(18, 0, 0), cv::Scalar(255, 116, 255), lab_g);
    cv::Mat gm = h_g & lab_g;
    if (!bev_roi.empty()) gm &= bev_roi;
    cv::morphologyEx(gm, gm, cv::MORPH_OPEN, k_open);
    cv::morphologyEx(gm, gm, cv::MORPH_CLOSE, k_close);
    masks.push_back(gm);
  }

  // 유효 면적 윤곽선에 볼록 껍질을 적용해 반사 패임을 메우고 최근접 후보 선택
  std::vector<std::vector<cv::Point>> contours;
  for (const cv::Mat& cm : masks) {
    std::vector<std::vector<cv::Point>> sub;
    cv::findContours(cm, sub, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    contours.insert(contours.end(), sub.begin(), sub.end());
  }

  const double px_mm2 = (kStepM * 1000.0) * (kStepM * 1000.0);
  int best_idx = -1;
  double best_dist = 0.065;
  cv::RotatedRect best_rect;
  std::vector<cv::Point> best_poly;
  double best_mx = x0;
  double best_my = y0;

  for (size_t i = 0; i < contours.size(); ++i) {
    const double raw_mm2 = cv::contourArea(contours[i]) * px_mm2;
    if (raw_mm2 < 320.0 || raw_mm2 > 3000.0) continue;

    std::vector<cv::Point> hull;
    cv::convexHull(contours[i], hull);
    const double hull_mm2 = cv::contourArea(hull) * px_mm2;
    const auto& poly = (hull_mm2 <= 2350.0) ? hull : contours[i];

    std::vector<cv::Point2f> pts_m;
    pts_m.reserve(poly.size());
    for (const cv::Point& p : poly) {
      pts_m.emplace_back(static_cast<float>(x0 + (p.x - half_px) * kStepM),
                         static_cast<float>(y0 + (p.y - half_px) * kStepM));
    }
    const cv::RotatedRect rr = cv::minAreaRect(pts_m);
    const cv::Moments mu = cv::moments(poly, false);
    const double mx =
        (mu.m00 > 1.0) ? (x0 + (mu.m10 / mu.m00 - half_px) * kStepM)
                       : static_cast<double>(rr.center.x);
    const double my =
        (mu.m00 > 1.0) ? (y0 + (mu.m01 / mu.m00 - half_px) * kStepM)
                       : static_cast<double>(rr.center.y);

    const double dist = std::hypot(mx - x0, my - y0);
    if (other) {
      const double d_other =
          std::hypot(mx - other->pos[0], my - other->pos[1]);
      if (d_other + 0.008 < dist) continue;
    }
    if (dist < best_dist) {
      best_dist = dist;
      best_idx = static_cast<int>(i);
      best_rect = rr;
      best_poly = poly;
      best_mx = mx;
      best_my = my;
    }
  }
  if (best_idx < 0) return false;

  // 실척 회전 사각형의 장단변 길이와 방향으로부터 눕힘 여부 및 정밀 Yaw 산출
  cv::Point2f corners[4];
  best_rect.points(corners);
  const cv::Point2f e01 = corners[1] - corners[0];
  const cv::Point2f e12 = corners[2] - corners[1];
  const double l01 = std::hypot(e01.x, e01.y);
  const double l12 = std::hypot(e12.x, e12.y);
  const cv::Point2f maj = (l01 >= l12) ? e01 : e12;
  const double len_mm = std::max(l01, l12) * 1000.0;
  const double wid_mm = std::max(1.0, std::min(l01, l12) * 1000.0);

  if (len_mm >= 46.0 && len_mm > 1.42 * wid_mm) io_puck.is_square = true;
  out_lying = io_puck.is_square && (len_mm >= 44.5 && len_mm > 1.38 * wid_mm);

  if (io_puck.is_square || out_lying) {
    io_puck.pos[0] = 0.5 * (static_cast<double>(best_rect.center.x) + best_mx);
    io_puck.pos[1] = 0.5 * (static_cast<double>(best_rect.center.y) + best_my);
  } else {
    io_puck.pos[0] = best_mx;
    io_puck.pos[1] = best_my;
  }

  if (out_lying) {
    double yaw = std::atan2(maj.y, maj.x);
    if (yaw > 0.5 * kPi) yaw -= kPi;
    if (yaw < -0.5 * kPi) yaw += kPi;
    io_puck.yaw = yaw;
  } else if (io_puck.is_square) {
    const double raw = std::atan2(maj.y, maj.x);
    io_puck.yaw = 0.25 * std::atan2(std::sin(4.0 * raw), std::cos(4.0 * raw));
  }

  // 디버그 토픽 구독자가 있을 때만 탑뷰 볼록 껍질·중심 십자선 시각화 영상 발행
  if (debug_pub_->get_subscription_count() > 0) {
    cv::Mat vis = bev_rgb.clone();
    const std::vector<std::vector<cv::Point>> draw_polys = {best_poly};
    cv::drawContours(vis, draw_polys, 0, cv::Scalar(255, 220, 0), 2);
    const int cx_px =
        static_cast<int>(std::round((io_puck.pos[0] - x0) / kStepM + half_px));
    const int cy_px =
        static_cast<int>(std::round((io_puck.pos[1] - y0) / kStepM + half_px));
    cv::drawMarker(vis, cv::Point(cx_px, cy_px), cv::Scalar(0, 255, 255),
                   cv::MARKER_CROSS, 14, 2);

    ImageMsg dbg;
    dbg.header = info.header;
    dbg.width = kBevSize;
    dbg.height = kBevSize;
    dbg.encoding = "rgb8";
    dbg.step = kBevSize * 3;
    dbg.data.assign(vis.datastart, vis.dataend);
    debug_pub_->publish(dbg);
  }
  return true;
}

}  // namespace xsting_pick_place
