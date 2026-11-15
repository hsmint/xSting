#ifndef XSTING_VISION_SIGN_BARCODE_NODE_H_
#define XSTING_VISION_SIGN_BARCODE_NODE_H_

#include <cuda_runtime.h>
#include <nppdefs.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <composition_interfaces/srv/list_nodes.hpp>
#include <composition_interfaces/srv/unload_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <xsting_interfaces/action/read_sign_barcode.hpp>

namespace xsting_vision {

using ReadAction = xsting_interfaces::action::ReadSignBarcode;
using GoalHandle = rclcpp_action::ServerGoalHandle<ReadAction>;
using Det2dArray = vision_msgs::msg::Detection2DArray;
using ImageMsg = sensor_msgs::msg::Image;
using ListNodesSrv = composition_interfaces::srv::ListNodes;
using UnloadNodeSrv = composition_interfaces::srv::UnloadNode;

// 분리 학습된 신호등 YOLO 4개 클래스 상태(OFF·RED·YELLOW·GREEN) 및 미검출
enum class SignLamp {
  kNone,
  kOff,
  kRed,
  kYellow,
  kGreen,
};

// 신호등 점멸 판독 및 타임아웃 설정
struct SignCfg {
  std::string action_name;
  std::string det_topic;
  std::string barcode_img_topic;
  double conf_thres;
  int expected_flashes;
  int max_retries;
  double min_on_sec;
  double min_off_sec;
  double start_off_sec;
  double end_off_sec;
  double timeout_sec;
  std::string container_name;
  std::vector<std::string> unload_nodes;
};

// Action 요청 시 신호등 점멸 순서와 Code 128 바코드를 인식하는 노드
class SignBarcodeNode : public rclcpp::Node {
 public:
  explicit SignBarcodeNode(
      const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());
  ~SignBarcodeNode() override;

 private:
  // YAML 파라미터 로드 및 NPP 스트림 초기화
  static rclcpp::NodeOptions AddYamlParams(const rclcpp::NodeOptions& opts);
  SignCfg LoadCfg();
  void InitNppCtx();

  // Action 요청·취소·수락 콜백 및 주기 타이머 처리
  rclcpp_action::GoalResponse OnGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ReadAction::Goal> goal);
  rclcpp_action::CancelResponse OnCancel(
      const std::shared_ptr<GoalHandle> goal);
  void OnAccept(const std::shared_ptr<GoalHandle> goal);
  void OnTick();

  // 신호등 YOLO 검출 처리 및 점멸 상태 갱신
  void OnDet(const Det2dArray::ConstSharedPtr& msg);
  SignLamp GetLampState(const Det2dArray& msg) const;
  void UpdateLamp(SignLamp lamp, const rclcpp::Time& now_time);
  void RetrySign(bool synced, const std::string& err);
  bool CheckCounts(int& r_cnt, int& y_cnt, int& g_cnt) const;

  // CUDA 버퍼 영상 그레이스케일 변환 및 Code 128 바코드 해독
  void OnImg(const ImageMsg::ConstSharedPtr& msg);
  std::string ReadCode(const ImageMsg& msg);
  static bool ParseBarcode(const std::string& code, bool& ascending);

  // 진행 피드백 발행, Action 결과 반환 및 완료 후 컴포넌트 언로드
  void PubFeedback();
  void FinishJob(bool ok, const std::string& err);
  void RequestUnload();

  SignCfg cfg_;
  std::mutex mtx_;

  cudaStream_t cuda_stream_ = nullptr;
  NppStreamContext npp_ctx_{};
  uint8_t* gpu_gray_ = nullptr;
  size_t gpu_gray_cap_ = 0;
  std::vector<uint8_t> gray_buf_;

  std::shared_ptr<GoalHandle> goal_;
  rclcpp::Time deadline_;
  bool done_ = false;
  rclcpp::Time unload_at_;
  std::string seq_;
  std::string barcode_;
  bool ascending_ = true;
  bool sign_done_ = false;
  bool synced_ = false;
  bool armed_off_ = false;
  int retry_cnt_ = 0;
  SignLamp active_lamp_ = SignLamp::kNone;
  SignLamp cand_lamp_ = SignLamp::kNone;
  rclcpp::Time cand_since_;
  rclcpp::Time last_det_;

  rclcpp::Subscription<Det2dArray>::SharedPtr det_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr img_sub_;
  rclcpp_action::Server<ReadAction>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<ListNodesSrv>::SharedPtr list_cli_;
  rclcpp::Client<UnloadNodeSrv>::SharedPtr unload_cli_;
};

}  // namespace xsting_vision

#endif  // XSTING_VISION_SIGN_BARCODE_NODE_H_
