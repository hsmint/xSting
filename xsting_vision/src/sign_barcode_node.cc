#include "xsting_vision/sign_barcode_node.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <utility>

#include <ZXing/ReadBarcode.h>
#include <ament_index_cpp/get_package_share_path.hpp>
#include <cuda_buffer/cuda_buffer_api.hpp>
#include <nppi_color_conversion.h>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace xsting_vision {

namespace {

// 파란 배경·그림자를 밝기 변화에 강건하게 판별하기 위한 색상 임계값
constexpr int kBlueMin = 35;
constexpr int kBlueDiff = 6;
constexpr int kGreenDiff = 5;

// 원거리 저해상도 Code 128(P2-01~04) 1D 템플릿 매칭 상수
constexpr int kModCnt = 90;
constexpr int kTplScale = 16;
constexpr int kTplLen = kModCnt * kTplScale;
constexpr int kBoxH = 9;
constexpr int kBoxW = 25;
constexpr double kMinBoxScore = 10.0;

// 신호등 연속 상태 유지를 허용하는 최대 미검출 간격(초)
constexpr double kMaxDetGapSec = 0.5;

// 저조도에서도 비율 조건을 함께 사용해 파란 배경·그림자 여부 판별
bool IsBlueBg(int red, int green, int blue) {
  return blue >= kBlueMin && (blue - red) >= kBlueDiff &&
         (green - red) >= kGreenDiff && (blue * 6) >= (red * 7) &&
         (green * 8) >= (red * 9);
}

struct CodeTpl {
  const char* code;
  std::array<double, kTplLen> val;
};

// P2-01~04의 90모듈 Code 128B 광학 블러 템플릿 4종 생성
const std::array<CodeTpl, 4>& GetCodeTpls() {
  static const std::array<CodeTpl, 4> tpls = []() {
    constexpr const char* kPrefix = "211214313121223211122132123122";
    constexpr const char* kStop = "2331112";
    constexpr std::array<std::pair<const char*, const char*>, 4> kSpecs = {{
        {"P2-01", "123221141122"},
        {"P2-02", "223211122411"},
        {"P2-03", "221132413111"},
        {"P2-04", "221231121241"},
    }};

    std::array<double, 49> kern{};
    double ksum = 0.0;
    for (int i = -24; i <= 24; ++i) {
      const double ratio = static_cast<double>(i) / 8.8;
      kern[static_cast<size_t>(i + 24)] = std::exp(-0.5 * ratio * ratio);
      ksum += kern[static_cast<size_t>(i + 24)];
    }
    for (double& weight : kern) {
      weight /= ksum;
    }

    std::array<CodeTpl, 4> out{};
    for (size_t idx = 0; idx < kSpecs.size(); ++idx) {
      out[idx].code = kSpecs[idx].first;
      const std::string pat =
          std::string(kPrefix) + kSpecs[idx].second + kStop;
      std::array<double, kTplLen> raw{};
      size_t pos = 0;
      for (size_t i = 0; i < pat.size(); ++i) {
        const double bit = (i % 2 == 0) ? 0.0 : 1.0;
        const int run = (pat[i] - '0') * kTplScale;
        for (int step = 0; step < run && pos < kTplLen; ++step) {
          raw[pos++] = bit;
        }
      }
      for (int i = 0; i < kTplLen; ++i) {
        double acc = 0.0;
        for (int tap = -24; tap <= 24; ++tap) {
          const int src = i - tap;
          if (src >= 0 && src < kTplLen) {
            acc += raw[static_cast<size_t>(src)] *
                   kern[static_cast<size_t>(tap + 24)];
          }
        }
        out[idx].val[static_cast<size_t>(i)] = acc;
      }
    }
    return out;
  }();
  return tpls;
}

// 두 1차원 신호 벡터의 정규화 피어슨 상관계수 계산
double CalcCorr(const std::vector<double>& lhs,
                const std::vector<double>& rhs) {
  if (lhs.size() != rhs.size() || lhs.size() < 4) {
    return 0.0;
  }
  double mean_l = 0.0;
  double mean_r = 0.0;
  for (size_t i = 0; i < lhs.size(); ++i) {
    mean_l += lhs[i];
    mean_r += rhs[i];
  }
  const double inv_n = 1.0 / static_cast<double>(lhs.size());
  mean_l *= inv_n;
  mean_r *= inv_n;

  double dot = 0.0;
  double norm_l = 0.0;
  double norm_r = 0.0;
  for (size_t i = 0; i < lhs.size(); ++i) {
    const double dl = lhs[i] - mean_l;
    const double dr = rhs[i] - mean_r;
    dot += dl * dr;
    norm_l += dl * dl;
    norm_r += dr * dr;
  }
  const double denom = std::sqrt(norm_l * norm_r);
  return denom > 1e-9 ? (dot / denom) : 0.0;
}

// 지정된 시작점·폭으로 프로파일과 템플릿의 구간별(0:공통, 1:전체, 2:식별) 상관 계산
double EvalSpan(const std::vector<double>& prof,
                const std::array<double, kTplLen>& tpl, double start,
                double span_w, int mode) {
  const int x_beg = std::max(0, static_cast<int>(std::floor(start)));
  const int x_end = std::min(static_cast<int>(prof.size()),
                             static_cast<int>(std::ceil(start + span_w)) + 1);
  std::vector<double> obs;
  std::vector<double> ref;
  obs.reserve(static_cast<size_t>(std::max(0, x_end - x_beg)));
  ref.reserve(obs.capacity());

  for (int x = x_beg; x < x_end; ++x) {
    const double mod = (static_cast<double>(x) - start) / span_w * kModCnt;
    if (mod < 0.0 || mod >= static_cast<double>(kModCnt)) {
      continue;
    }
    if (mode == 0 && mod >= 55.0 && mod < 77.0) {
      continue;
    }
    if (mode == 2 && (mod < 54.0 || mod >= 77.5)) {
      continue;
    }
    const int tidx =
        std::clamp(static_cast<int>(mod * kTplScale), 0, kTplLen - 1);
    obs.push_back(prof[static_cast<size_t>(x)]);
    ref.push_back(tpl[static_cast<size_t>(tidx)]);
  }
  if (mode != 2 && obs.size() < 25) {
    return -1.0;
  }
  return CalcCorr(obs, ref);
}

// 공통 프리픽스·종료 패턴 기준으로 바코드 시작 위치와 폭을 서브픽셀 정렬
double AlignSpan(const std::vector<double>& prof,
                 const std::array<double, kTplLen>& tpl, int left_idx,
                 int est_w, double& best_start, double& best_w) {
  double best_corr = -1.0;
  best_start = static_cast<double>(left_idx);
  best_w = static_cast<double>(est_w);
  const double min_w = std::max(36.0, static_cast<double>(est_w) - 16.0);
  const double max_w = static_cast<double>(est_w) + 4.01;

  for (double st = left_idx - 4.0; st <= left_idx + 10.01; st += 0.5) {
    for (double wd = min_w; wd <= max_w; wd += 0.5) {
      const double corr = EvalSpan(prof, tpl, st, wd, 0);
      if (corr > best_corr) {
        best_corr = corr;
        best_start = st;
        best_w = wd;
      }
    }
  }

  const double st_ref = best_start;
  const double wd_ref = best_w;
  for (double st = st_ref - 0.6; st <= st_ref + 0.61; st += 0.1) {
    for (double wd = wd_ref - 0.6; wd <= wd_ref + 0.61; wd += 0.1) {
      const double corr = EvalSpan(prof, tpl, st, wd, 0);
      if (corr > best_corr) {
        best_corr = corr;
        best_start = st;
        best_w = wd;
      }
    }
  }
  return best_corr;
}

// 후보 중심 주변에서 1차원 바코드 프로파일을 추출해 P2-01~04 일치 여부 판정
std::string TryMatchRoi(const uint8_t* gray, int width, int height, int cx,
                        int cy) {
  const int y0 = std::max(0, cy - 12);
  const int y1 = std::min(height, cy + 13);
  const int x0 = std::max(0, cx - 80);
  const int x1 = std::min(width, cx + 81);
  const int roi_w = x1 - x0;
  const int roi_h = y1 - y0;
  if (roi_w < 50 || roi_h < 6) {
    return "";
  }

  // 수평 에지 강도가 높은 바코드 막대 행들만 골라 1차원 밝기 프로파일 생성
  std::vector<double> row_grad(static_cast<size_t>(roi_h), 0.0);
  double max_grad = 0.0;
  for (int y = 0; y < roi_h; ++y) {
    const uint8_t* row = gray + static_cast<size_t>(y0 + y) * width + x0;
    double sum = 0.0;
    for (int x = 0; x + 1 < roi_w; ++x) {
      sum += std::abs(static_cast<int>(row[x + 1]) - static_cast<int>(row[x]));
    }
    row_grad[static_cast<size_t>(y)] = sum / static_cast<double>(roi_w - 1);
    max_grad = std::max(max_grad, row_grad[static_cast<size_t>(y)]);
  }

  std::vector<double> prof(static_cast<size_t>(roi_w), 0.0);
  int row_cnt = 0;
  for (int y = 0; y < roi_h; ++y) {
    if (row_grad[static_cast<size_t>(y)] <= 0.55 * max_grad) {
      continue;
    }
    const uint8_t* row = gray + static_cast<size_t>(y0 + y) * width + x0;
    for (int x = 0; x < roi_w; ++x) {
      prof[static_cast<size_t>(x)] += row[x];
    }
    ++row_cnt;
  }
  if (row_cnt < 4) {
    return "";
  }
  for (double& val : prof) {
    val /= static_cast<double>(row_cnt);
  }

  // 중심 부근의 연속된 고주파 막대 구간 좌우 경계 및 폭 추정
  const int diff_len = roi_w - 1;
  std::vector<double> adx(static_cast<size_t>(diff_len), 0.0);
  for (int x = 0; x < diff_len; ++x) {
    const double diff =
        prof[static_cast<size_t>(x + 1)] - prof[static_cast<size_t>(x)];
    adx[static_cast<size_t>(x)] = std::abs(diff);
  }
  std::vector<double> gdx(static_cast<size_t>(diff_len), 0.0);
  for (int x = 0; x < diff_len; ++x) {
    double sum = 0.0;
    for (int tap = -4; tap <= 4; ++tap) {
      const int idx = x + tap;
      if (idx >= 0 && idx < diff_len) {
        sum += adx[static_cast<size_t>(idx)];
      }
    }
    gdx[static_cast<size_t>(x)] = sum / 9.0;
  }

  const int c_rel = std::clamp(cx - x0, 0, diff_len - 1);
  double peak = 0.0;
  for (int x = std::max(0, c_rel - 25); x < std::min(diff_len, c_rel + 25);
       ++x) {
    peak = std::max(peak, gdx[static_cast<size_t>(x)]);
  }
  const double thr = 0.25 * peak;
  int left_idx = c_rel;
  while (left_idx > 0 && gdx[static_cast<size_t>(left_idx - 1)] >= thr) {
    --left_idx;
  }
  int right_idx = c_rel;
  while (right_idx < diff_len && gdx[static_cast<size_t>(right_idx)] >= thr) {
    ++right_idx;
  }
  const int est_w = right_idx - left_idx;
  if (est_w < 35 || est_w > 150) {
    return "";
  }

  // 템플릿 정렬 후 전체 구간 및 식별 구간 상관계수·격차 검증
  const auto& tpls = GetCodeTpls();
  double start = 0.0;
  double span_w = 0.0;
  if (AlignSpan(prof, tpls[0].val, left_idx, est_w, start, span_w) < 0.88) {
    return "";
  }

  struct CandScore {
    double disc;
    double full;
    const char* code;
  };
  std::array<CandScore, 4> scores{};
  for (size_t i = 0; i < tpls.size(); ++i) {
    scores[i] = {EvalSpan(prof, tpls[i].val, start, span_w, 2),
                 EvalSpan(prof, tpls[i].val, start, span_w, 1), tpls[i].code};
  }
  std::sort(scores.begin(), scores.end(),
            [](const CandScore& lhs, const CandScore& rhs) {
              return lhs.disc > rhs.disc;
            });

  if (scores[0].full >= 0.90 && scores[0].disc >= 0.85 &&
      (scores[0].disc - scores[1].disc) >= 0.20) {
    return scores[0].code;
  }
  return "";
}

// 적분 영상으로 수직 막대 밀집 후보를 찾아 저해상도 Code 128 보조 판독 수행
std::string MatchBarcode(const uint8_t* gray, int width, int height) {
  if (width < 100 || height < 40) {
    return "";
  }
  const int in_w = width - 2;
  const int in_h = height - 2;
  const int stride = in_w + 1;
  std::vector<int> integ(static_cast<size_t>(in_h + 1) * stride, 0);

  for (int y = 0; y < in_h; ++y) {
    const uint8_t* prev = gray + static_cast<size_t>(y) * width;
    const uint8_t* curr = prev + width;
    const uint8_t* next = curr + width;
    int row_sum = 0;
    for (int x = 0; x < in_w; ++x) {
      const int dx = std::abs(static_cast<int>(curr[x + 2]) - curr[x]);
      const int dy = std::abs(static_cast<int>(next[x + 1]) - prev[x + 1]);
      row_sum += std::max(0, dx - dy);
      integ[static_cast<size_t>(y + 1) * stride + x + 1] =
          integ[static_cast<size_t>(y) * stride + x + 1] + row_sum;
    }
  }

  const int box_w = in_w - kBoxW + 1;
  const int box_h = in_h - kBoxH + 1;
  if (box_w <= 0 || box_h <= 0) {
    return "";
  }
  std::vector<double> box(static_cast<size_t>(box_w) * box_h, 0.0);
  const double inv_area = 1.0 / static_cast<double>(kBoxW * kBoxH);
  for (int y = 0; y < box_h; ++y) {
    for (int x = 0; x < box_w; ++x) {
      const int sum =
          integ[static_cast<size_t>(y + kBoxH) * stride + x + kBoxW] -
          integ[static_cast<size_t>(y) * stride + x + kBoxW] -
          integ[static_cast<size_t>(y + kBoxH) * stride + x] +
          integ[static_cast<size_t>(y) * stride + x];
      box[static_cast<size_t>(y) * box_w + x] = sum * inv_area;
    }
  }

  for (int cand = 0; cand < 3; ++cand) {
    const auto it = std::max_element(box.begin(), box.end());
    if (it == box.end() || *it < kMinBoxScore) {
      break;
    }
    const int idx = static_cast<int>(it - box.begin());
    const int my = idx / box_w;
    const int mx = idx % box_w;
    for (int y = std::max(0, my - 20); y < std::min(box_h, my + 21); ++y) {
      for (int x = std::max(0, mx - 50); x < std::min(box_w, mx + 51); ++x) {
        box[static_cast<size_t>(y) * box_w + x] = 0.0;
      }
    }
    const std::string code =
        TryMatchRoi(gray, width, height, mx + 1 + kBoxW / 2,
                    my + 1 + kBoxH / 2);
    if (!code.empty()) {
      return code;
    }
  }
  return "";
}

// YOLO 클래스 ID(0~3 또는 클래스명)를 신호등 4상태로 변환
SignLamp MapClassToLamp(const std::string& cls_id) {
  if (cls_id == "0" || cls_id == "sign_tower") {
    return SignLamp::kOff;
  }
  if (cls_id == "1" || cls_id == "sign_tower_R") {
    return SignLamp::kRed;
  }
  if (cls_id == "2" || cls_id == "sign_tower_Y") {
    return SignLamp::kYellow;
  }
  if (cls_id == "3" || cls_id == "sign_tower_G") {
    return SignLamp::kGreen;
  }
  return SignLamp::kNone;
}

// 점등 상태를 기록용 문자('R', 'Y', 'G')로 변환
char ToLampChar(SignLamp lamp) {
  switch (lamp) {
    case SignLamp::kRed:
      return 'R';
    case SignLamp::kYellow:
      return 'Y';
    case SignLamp::kGreen:
      return 'G';
    default:
      return '\0';
  }
}

}  // namespace

// YAML 로드 후 CUDA 스트림, 구독자, Action 서버, 타이머 초기화
SignBarcodeNode::SignBarcodeNode(const rclcpp::NodeOptions& opts)
    : Node("sign_barcode", AddYamlParams(opts)) {
  cfg_ = LoadCfg();
  cudaStreamCreateWithFlags(&cuda_stream_, cudaStreamNonBlocking);
  InitNppCtx();

  // 신호등 검출 구독 및 CUDA 백엔드 기반 바코드 영상 구독 생성
  det_sub_ = create_subscription<Det2dArray>(
      cfg_.det_topic, 10,
      [this](const Det2dArray::ConstSharedPtr& msg) { OnDet(msg); });

  rclcpp::SubscriptionOptions cuda_opts;
  cuda_opts.acceptable_buffer_backends = "cuda";
  img_sub_ = create_subscription<ImageMsg>(
      cfg_.barcode_img_topic, rclcpp::SensorDataQoS(),
      [this](const ImageMsg::ConstSharedPtr& msg) { OnImg(msg); }, cuda_opts);

  // Action 서버와 취소·타임아웃 감시 타이머 생성
  server_ = rclcpp_action::create_server<ReadAction>(
      this, cfg_.action_name,
      [this](const rclcpp_action::GoalUUID& uuid,
             std::shared_ptr<const ReadAction::Goal> goal) {
        return OnGoal(uuid, std::move(goal));
      },
      [this](const std::shared_ptr<GoalHandle> goal) {
        return OnCancel(goal);
      },
      [this](const std::shared_ptr<GoalHandle> goal) { OnAccept(goal); });

  timer_ = create_wall_timer(std::chrono::milliseconds(100),
                             [this]() { OnTick(); });

  // 완료 후 언로드할 컨테이너 서비스 클라이언트 생성
  if (!cfg_.container_name.empty() && !cfg_.unload_nodes.empty()) {
    list_cli_ = create_client<ListNodesSrv>(cfg_.container_name +
                                            "/_container/list_nodes");
    unload_cli_ = create_client<UnloadNodeSrv>(cfg_.container_name +
                                                "/_container/unload_node");
  }
}

// 구독·타이머 해제 후 GPU 메모리와 CUDA 스트림 정리
SignBarcodeNode::~SignBarcodeNode() {
  timer_->cancel();
  det_sub_.reset();
  img_sub_.reset();
  cudaStreamSynchronize(cuda_stream_);
  cudaFree(gpu_gray_);
  cudaStreamDestroy(cuda_stream_);
}

// YAML 설정 파일을 읽어 NodeOptions 앞쪽에 삽입
rclcpp::NodeOptions SignBarcodeNode::AddYamlParams(
    const rclcpp::NodeOptions& opts) {
  std::string cfg_path =
      (ament_index_cpp::get_package_share_path("xsting_vision") / "config" /
       "sign_barcode.yaml")
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

// 신호등·바코드 인식 파라미터 선언 및 읽기
SignCfg SignBarcodeNode::LoadCfg() {
  SignCfg cfg;
  cfg.action_name =
      declare_parameter<std::string>("action_name", "read_sign_barcode");
  cfg.det_topic =
      declare_parameter<std::string>("det_topic", "sign_yolo/detections");
  cfg.barcode_img_topic = declare_parameter<std::string>(
      "barcode_img_topic", "top_camera/color/image_raw");
  cfg.conf_thres = declare_parameter<double>("conf_thres", 0.5);
  cfg.expected_flashes = declare_parameter<int>("expected_flashes", 12);
  cfg.max_retries = declare_parameter<int>("max_retries", 3);
  cfg.min_on_sec = declare_parameter<double>("min_on_sec", 0.25);
  cfg.min_off_sec = declare_parameter<double>("min_off_sec", 0.25);
  cfg.start_off_sec = declare_parameter<double>("start_off_sec", 3.0);
  cfg.end_off_sec = declare_parameter<double>("end_off_sec", 3.0);
  cfg.timeout_sec = declare_parameter<double>("timeout_sec", 160.0);
  cfg.container_name =
      declare_parameter<std::string>("container_name", "/vision");
  cfg.unload_nodes = declare_parameter<std::vector<std::string>>(
      "unload_nodes", std::vector<std::string>{});
  return cfg;
}

// NPP 그레이스케일 변환용 CUDA 스트림 컨텍스트 설정
void SignBarcodeNode::InitNppCtx() {
  cudaGetDevice(&npp_ctx_.nCudaDeviceId);
  cudaDeviceProp dev{};
  cudaGetDeviceProperties(&dev, npp_ctx_.nCudaDeviceId);

  npp_ctx_.hStream = cuda_stream_;
  npp_ctx_.nMultiProcessorCount = dev.multiProcessorCount;
  npp_ctx_.nMaxThreadsPerMultiProcessor = dev.maxThreadsPerMultiProcessor;
  npp_ctx_.nMaxThreadsPerBlock = dev.maxThreadsPerBlock;
  npp_ctx_.nSharedMemPerBlock = dev.sharedMemPerBlock;
  npp_ctx_.nCudaDevAttrComputeCapabilityMajor = dev.major;
  npp_ctx_.nCudaDevAttrComputeCapabilityMinor = dev.minor;
  cudaStreamGetFlags(cuda_stream_, &npp_ctx_.nStreamFlags);
}

// 진행 중인 Action이 없을 때만 새 요청 수락
rclcpp_action::GoalResponse SignBarcodeNode::OnGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const ReadAction::Goal> goal) {
  (void)uuid;
  (void)goal;
  std::lock_guard<std::mutex> lock(mtx_);
  if (goal_ || done_) {
    RCLCPP_WARN(get_logger(), "[Action] read_sign_barcode rejected");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 실행 중인 Action 취소 요청 수락
rclcpp_action::CancelResponse SignBarcodeNode::OnCancel(
    const std::shared_ptr<GoalHandle> goal) {
  (void)goal;
  return rclcpp_action::CancelResponse::ACCEPT;
}

// 새 Action 실행을 위해 신호등·바코드 관측 상태 초기화
void SignBarcodeNode::OnAccept(const std::shared_ptr<GoalHandle> goal) {
  std::lock_guard<std::mutex> lock(mtx_);
  const float req_timeout = goal->get_goal()->timeout_sec;
  const double limit = req_timeout > 0.0f ? req_timeout : cfg_.timeout_sec;

  const rclcpp::Time now_time = now();
  goal_ = goal;
  deadline_ = now_time + rclcpp::Duration::from_seconds(limit);
  seq_.clear();
  barcode_.clear();
  ascending_ = true;
  sign_done_ = false;
  synced_ = false;
  armed_off_ = false;
  retry_cnt_ = 0;
  active_lamp_ = SignLamp::kNone;
  cand_lamp_ = SignLamp::kNone;
  cand_since_ = now_time;
  last_det_ = now_time;
  RCLCPP_INFO(get_logger(),
              "[Action] read_sign_barcode started (timeout=%.1fs)", limit);
  PubFeedback();
}

// 주기적으로 완료 후 언로드 시각, 취소 요청, 제한 시간 초과 확인
void SignBarcodeNode::OnTick() {
  std::lock_guard<std::mutex> lock(mtx_);
  if (done_) {
    if (now() >= unload_at_) {
      timer_->cancel();
      RequestUnload();
    }
    return;
  }
  if (!goal_) {
    return;
  }
  if (goal_->is_canceling()) {
    auto res = std::make_shared<ReadAction::Result>();
    res->success = false;
    res->error_msg = "canceled";
    res->sequence = seq_;
    res->barcode = barcode_;
    RCLCPP_WARN(get_logger(),
                "[Action] read_sign_barcode canceled (seq=%s, barcode=%s)",
                seq_.c_str(), barcode_.c_str());
    goal_->canceled(res);
    goal_.reset();
    return;
  }
  if (now() > deadline_) {
    FinishJob(false, "timeout");
  }
}

// 신호등 검출 수신 시 미검출 공백을 보정하고 점멸 상태머신 갱신
void SignBarcodeNode::OnDet(const Det2dArray::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!goal_ || sign_done_) {
    return;
  }
  const rclcpp::Time now_time = now();
  if ((now_time - last_det_).seconds() > kMaxDetGapSec) {
    cand_lamp_ = SignLamp::kNone;
  }
  const SignLamp lamp = GetLampState(*msg);
  if (lamp == SignLamp::kNone) {
    return;
  }
  last_det_ = now_time;
  UpdateLamp(lamp, now_time);
}

// 검출 배열에서 신뢰도 임계값 이상인 최고 점수 신호등 상태 반환
SignLamp SignBarcodeNode::GetLampState(const Det2dArray& msg) const {
  double best_score = cfg_.conf_thres;
  SignLamp best_lamp = SignLamp::kNone;
  for (const auto& det : msg.detections) {
    for (const auto& res : det.results) {
      const SignLamp lamp = MapClassToLamp(res.hypothesis.class_id);
      if (lamp != SignLamp::kNone && res.hypothesis.score >= best_score) {
        best_score = res.hypothesis.score;
        best_lamp = lamp;
      }
    }
  }
  return best_lamp;
}

// 시작 3초 소등 확인 후 12회 점멸과 종료 3초 소등 판정
void SignBarcodeNode::UpdateLamp(SignLamp lamp, const rclcpp::Time& now_time) {
  if (lamp != cand_lamp_) {
    cand_lamp_ = lamp;
    cand_since_ = now_time;
  }
  const double held = (now_time - cand_since_).seconds();

  // 시작 기준 3초 연속 소등이 확인될 때까지 점멸 기록 대기
  if (!synced_) {
    if (lamp == SignLamp::kOff && held >= cfg_.start_off_sec) {
      synced_ = true;
      armed_off_ = true;
      active_lamp_ = SignLamp::kNone;
      RCLCPP_INFO(get_logger(),
                  "[Action] start off %.1fs confirmed (try %d/%d)", held,
                  retry_cnt_ + 1, cfg_.max_retries + 1);
    }
    return;
  }

  // OFF 상태: 소등 구간 복귀 및 종료 3초 연속 소등 시 결과 검증·재시도
  if (lamp == SignLamp::kOff) {
    if (held >= cfg_.min_off_sec) {
      armed_off_ = true;
      active_lamp_ = SignLamp::kNone;
    }
    const int count = static_cast<int>(seq_.size());
    if (count == 0 || held < cfg_.end_off_sec) {
      return;
    }
    if (count == cfg_.expected_flashes) {
      int r_cnt = 0;
      int y_cnt = 0;
      int g_cnt = 0;
      if (!CheckCounts(r_cnt, y_cnt, g_cnt)) {
        RetrySign(held >= cfg_.start_off_sec, "invalid ryg count");
        return;
      }
      sign_done_ = true;
      RCLCPP_INFO(get_logger(),
                  "[Action] sign sequence done: %s (R=%d, Y=%d, G=%d)",
                  seq_.c_str(), r_cnt, y_cnt, g_cnt);
      if (!barcode_.empty()) {
        FinishJob(true, "");
      }
      return;
    }
    RetrySign(held >= cfg_.start_off_sec, "flash count underflow");
    return;
  }

  // RED·YELLOW·GREEN 점등 상태: 기준 시간 유지 시 점멸 기록 및 오류 검사
  if (held < cfg_.min_on_sec) {
    return;
  }
  if (!armed_off_) {
    if (lamp != active_lamp_) {
      RetrySign(false, "ambiguous lamp color");
    }
    return;
  }
  if (static_cast<int>(seq_.size()) >= cfg_.expected_flashes) {
    RetrySign(false, "flash count exceeded");
    return;
  }

  // 새 점멸 1회 확정 후 피드백 발행
  seq_.push_back(ToLampChar(lamp));
  armed_off_ = false;
  active_lamp_ = lamp;
  RCLCPP_INFO(get_logger(), "[Action] flash %zu/%d: %c (seq=%s)", seq_.size(),
              cfg_.expected_flashes, seq_.back(), seq_.c_str());
  PubFeedback();
}

// 신호등 판독 실패 시 최대 횟수 내에서 초기화 후 재시도하거나 종료
void SignBarcodeNode::RetrySign(bool synced, const std::string& err) {
  if (retry_cnt_ >= cfg_.max_retries) {
    FinishJob(false, err);
    return;
  }
  ++retry_cnt_;
  RCLCPP_WARN(get_logger(),
              "[Action] sign read failed (%s, seq=%s), retry %d/%d",
              err.c_str(), seq_.c_str(), retry_cnt_, cfg_.max_retries);
  seq_.clear();
  synced_ = synced;
  armed_off_ = synced;
  active_lamp_ = SignLamp::kNone;
  PubFeedback();
}

// R·Y·G 횟수를 집계하고 각각 1~6회 및 합계 일치 여부 검증
bool SignBarcodeNode::CheckCounts(int& r_cnt, int& y_cnt, int& g_cnt) const {
  r_cnt = 0;
  y_cnt = 0;
  g_cnt = 0;
  for (char ch : seq_) {
    if (ch == 'R') {
      ++r_cnt;
    } else if (ch == 'Y') {
      ++y_cnt;
    } else if (ch == 'G') {
      ++g_cnt;
    }
  }
  return r_cnt >= 1 && r_cnt <= 6 && y_cnt >= 1 && y_cnt <= 6 && g_cnt >= 1 &&
         g_cnt <= 6 && (r_cnt + y_cnt + g_cnt) == cfg_.expected_flashes;
}

// 바코드 미인식 상태에서 GPU 영상을 받아 Code 128 해독 및 정렬 방향 판정
void SignBarcodeNode::OnImg(const ImageMsg::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!goal_ || !barcode_.empty()) {
    return;
  }

  const std::string code = ReadCode(*msg);
  if (code.empty()) {
    return;
  }
  if (!ParseBarcode(code, ascending_)) {
    FinishJob(false, "unknown barcode: " + code);
    return;
  }

  // 유효 바코드 저장 후 신호등 관측도 끝났다면 Action 완료
  barcode_ = code;
  RCLCPP_INFO(get_logger(), "[Action] barcode decoded: %s (%s)",
              barcode_.c_str(), ascending_ ? "asc" : "desc");
  PubFeedback();
  if (sign_done_) {
    FinishJob(true, "");
  }
}

// rosidl::Buffer(CUDA) 영상을 GPU에서 그레이스케일로 바꿔 ZXing으로 해독
std::string SignBarcodeNode::ReadCode(const ImageMsg& msg) {
  const int width = static_cast<int>(msg.width);
  const int height = static_cast<int>(msg.height);
  if (width <= 0 || height <= 0 || msg.data.size() == 0) {
    return "";
  }
  const size_t gray_bytes = static_cast<size_t>(width) * height;
  const bool is_rgb = (msg.encoding == sensor_msgs::image_encodings::RGB8);
  const bool is_bgr = (msg.encoding == sensor_msgs::image_encodings::BGR8);
  gray_buf_.resize(gray_bytes);

  // 인코딩에 맞춰 GPU 그레이스케일 변환 후 호스트 버퍼로 복사
  if (msg.encoding == sensor_msgs::image_encodings::MONO8) {
    auto handle =
        cuda_buffer_backend::from_input_buffer(msg.data, cuda_stream_);
    cudaMemcpy2DAsync(gray_buf_.data(), width, handle.get_ptr(), msg.step,
                      width, height, cudaMemcpyDeviceToHost, cuda_stream_);
  } else if (is_rgb || is_bgr) {
    if (gpu_gray_cap_ < gray_bytes) {
      cudaFree(gpu_gray_);
      cudaMalloc(&gpu_gray_, gray_bytes);
      gpu_gray_cap_ = gray_bytes;
    }
    auto handle =
        cuda_buffer_backend::from_input_buffer(msg.data, cuda_stream_);
    const NppiSize roi{width, height};
    nppiRGBToGray_8u_C3C1R_Ctx(handle.get_ptr(), msg.step, gpu_gray_, width,
                               roi, npp_ctx_);
    cudaMemcpyAsync(gray_buf_.data(), gpu_gray_, gray_bytes,
                    cudaMemcpyDeviceToHost, cuda_stream_);
  } else {
    return "";
  }
  cudaStreamSynchronize(cuda_stream_);

  // 기본 그레이스케일 영상으로 Code 128 바코드 1차 해독
  ZXing::ReaderOptions opts;
  opts.setFormats(ZXing::BarcodeFormat::Code128);
  ZXing::ImageView view(gray_buf_.data(), width, height,
                        ZXing::ImageFormat::Lum);
  const auto res = ZXing::ReadBarcode(view, opts);
  if (res.isValid()) {
    return res.text();
  }

  // 파란 배경을 흰 여백(255)으로 치환해 Code 128 바코드 2차 해독
  if (is_rgb || is_bgr) {
    const size_t row_bytes = static_cast<size_t>(width) * 3;
    gray_buf_.resize(gray_bytes * 4);
    uint8_t* rgb_ptr = gray_buf_.data() + gray_bytes;
    {
      auto handle =
          cuda_buffer_backend::from_input_buffer(msg.data, cuda_stream_);
      cudaMemcpy2DAsync(rgb_ptr, row_bytes, handle.get_ptr(), msg.step,
                        row_bytes, height, cudaMemcpyDeviceToHost,
                        cuda_stream_);
    }
    cudaStreamSynchronize(cuda_stream_);

    for (size_t i = 0; i < gray_bytes; ++i) {
      const int red = is_rgb ? rgb_ptr[i * 3] : rgb_ptr[i * 3 + 2];
      const int green = rgb_ptr[i * 3 + 1];
      const int blue = is_rgb ? rgb_ptr[i * 3 + 2] : rgb_ptr[i * 3];
      if (IsBlueBg(red, green, blue)) {
        rgb_ptr[i] = 255;
      } else {
        rgb_ptr[i] =
            static_cast<uint8_t>((77 * red + 150 * green + 29 * blue) >> 8);
      }
    }

    ZXing::ImageView bg_view(rgb_ptr, width, height, ZXing::ImageFormat::Lum);
    const auto bg_res = ZXing::ReadBarcode(bg_view, opts);
    if (bg_res.isValid()) {
      return bg_res.text();
    }
  }

  // 원거리 저해상도 영상용 1D Code 128 템플릿 상관 매칭 3차 보조 해독
  return MatchBarcode(gray_buf_.data(), width, height);
}

// 등록된 바코드(P2-01~04)인지 확인하고 오름·내림차순 설정
bool SignBarcodeNode::ParseBarcode(const std::string& code, bool& ascending) {
  if (code == "P2-01" || code == "P2-03") {
    ascending = true;
    return true;
  }
  if (code == "P2-02" || code == "P2-04") {
    ascending = false;
    return true;
  }
  return false;
}

// 현재까지 기록된 점멸 문자열과 바코드를 Action 피드백으로 전송
void SignBarcodeNode::PubFeedback() {
  if (!goal_) {
    return;
  }
  auto fb = std::make_shared<ReadAction::Feedback>();
  fb->sequence = seq_;
  fb->barcode = barcode_;
  goal_->publish_feedback(fb);
}

// Action 최종 결과를 채워 성공 또는 중단으로 종료
void SignBarcodeNode::FinishJob(bool ok, const std::string& err) {
  if (!goal_) {
    return;
  }
  int r_cnt = 0;
  int y_cnt = 0;
  int g_cnt = 0;
  CheckCounts(r_cnt, y_cnt, g_cnt);

  auto res = std::make_shared<ReadAction::Result>();
  res->success = ok;
  res->error_msg = err;
  res->sequence = seq_;
  res->r_count = r_cnt;
  res->y_count = y_cnt;
  res->g_count = g_cnt;
  res->barcode = barcode_;
  res->ascending = ascending_;

  if (ok) {
    RCLCPP_INFO(get_logger(),
                "[Action] read_sign_barcode succeeded: seq=%s "
                "(R=%d,Y=%d,G=%d), barcode=%s (%s)",
                seq_.c_str(), r_cnt, y_cnt, g_cnt, barcode_.c_str(),
                ascending_ ? "asc" : "desc");
    goal_->succeed(res);
  } else {
    RCLCPP_ERROR(get_logger(),
                 "[Action] read_sign_barcode aborted: %s (seq=%s, barcode=%s)",
                 err.c_str(), seq_.c_str(), barcode_.c_str());
    goal_->abort(res);
  }
  goal_.reset();

  // 1회 성공 완료 시 구독·GPU 버퍼를 즉시 해제하고 컴포넌트 언로드 예약
  if (ok) {
    done_ = true;
    unload_at_ = now() + rclcpp::Duration::from_seconds(0.5);
    det_sub_.reset();
    img_sub_.reset();
    cudaStreamSynchronize(cuda_stream_);
    cudaFree(gpu_gray_);
    gpu_gray_ = nullptr;
    gpu_gray_cap_ = 0;
  }
}

// 컨테이너에서 완료된 신호등·바코드 관련 컴포넌트를 순서대로 언로드
void SignBarcodeNode::RequestUnload() {
  if (!list_cli_ || !unload_cli_ || !list_cli_->service_is_ready()) {
    return;
  }
  auto cli = unload_cli_;
  const std::vector<std::string> targets = cfg_.unload_nodes;
  auto req = std::make_shared<ListNodesSrv::Request>();
  list_cli_->async_send_request(
      req, [cli, targets](rclcpp::Client<ListNodesSrv>::SharedFuture fut) {
        const auto& res = fut.get();
        for (const std::string& name : targets) {
          for (size_t i = 0; i < res->full_node_names.size(); ++i) {
            if (res->full_node_names[i] == name && i < res->unique_ids.size()) {
              auto unload_req = std::make_shared<UnloadNodeSrv::Request>();
              unload_req->unique_id = res->unique_ids[i];
              cli->async_send_request(unload_req);
            }
          }
        }
      });
}

}  // namespace xsting_vision

RCLCPP_COMPONENTS_REGISTER_NODE(xsting_vision::SignBarcodeNode)
