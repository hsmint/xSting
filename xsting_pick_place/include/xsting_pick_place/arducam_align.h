#ifndef XSTING_PICK_PLACE_ARDUCAM_ALIGN_H_
#define XSTING_PICK_PLACE_ARDUCAM_ALIGN_H_

#include <cuda_runtime.h>

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "xsting_pick_place/arm_kin.h"
#include "xsting_pick_place/puck_det.h"

namespace xsting_pick_place {

// 팔 끝단 ArduCam + YOLOv8 TensorRT로 상자·퍽을 인식하고 서브밀리미터 정밀 정렬하는 클래스
class ArducamAlign {
 public:
  using ImageMsg = sensor_msgs::msg::Image;
  using CamInfoMsg = sensor_msgs::msg::CameraInfo;
  using Det2DArray = vision_msgs::msg::Detection2DArray;

  struct YoloDet {
    int cls_id = -1;
    std::string cls_name;
    float score = 0.0f;
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
  };

  explicit ArducamAlign(const rclcpp::Node::SharedPtr& node,
                        const std::string& topic_ns = "/arducam");
  ~ArducamAlign();

  // 최근 1.5초 이내 유효한 ArduCam 영상과 카메라 정보가 수신 중인지 확인
  bool HasActiveStream() const;

  // 상자 상공 관측 자세에서 YOLOv8로 상자(paper_box)와 퍽 목록을 일괄 검출
  bool ScanBox(const Joint5& arm_joints, double ground_z,
               const std::string& filter_cls, bool auto_box_det,
               double& io_box_x, double& io_box_y,
               std::vector<Puck>& out_pucks, bool allow_empty = false);

  // 팔 정지 후 신규 프레임과 YOLOv8 ROI + L*a*b*/HSV 실척 탑뷰로 퍽 위치·자세 정밀 갱신
  bool RefinePuck(double& io_box_x, double& io_box_y, double box_top_z,
                  bool auto_box_det, const Joint5& arm_joints,
                  const Puck* other, Puck& target);

 private:
  struct TrtYolo;

  // 관측 자세 정지 이후의 신규 프레임 최소 min_new장 수신 대기
  bool WaitFreshFrame(int min_new, int timeout_ms,
                      ImageMsg::ConstSharedPtr& out_img,
                      CamInfoMsg::ConstSharedPtr& out_info);

  // GPU 또는 CPU 이미지 버퍼를 호스트 RGB8 행렬로 복사
  bool CopyRgb(const ImageMsg& msg, cv::Mat& rgb);

  // 카메라 내부 파라미터를 현재 해상도에 맞춰 추출
  bool GetScaledIntrinsics(const CamInfoMsg& info, int cols, int rows,
                           double& fx, double& fy, double& cx,
                           double& cy) const;

  // 픽셀 좌표를 지정 수평면(z_plane) 월드 XY 좌표로 역투영
  bool PixelToPlane(double u, double v, double fx, double fy, double cx,
                    double cy, const Vec3& cam_pos,
                    const std::array<Vec3, 3>& cam_axes, double z_plane,
                    double& wx, double& wy) const;

  // TensorRT YOLOv8 5클래스(퍽 4종 + paper_box) 추론 및 /arducam/detections 발행
  std::vector<YoloDet> InferYolo(const cv::Mat& rgb,
                                 const std_msgs::msg::Header& header,
                                 float conf_th);

  // 미클리핑 paper_box 검출 시 상자 상판 높이 기준으로 상자 중심 XY 갱신
  void TryUpdateBoxCenter(const std::vector<YoloDet>& dets, int cols, int rows,
                          double fx, double fy, double cx, double cy,
                          const Vec3& cam_pos,
                          const std::array<Vec3, 3>& cam_axes,
                          double box_top_z, double& io_box_x,
                          double& io_box_y) const;

  // 타겟 YOLO 박스를 중심으로 주변 타 퍽 중심부를 배제한 이미지 ROI 마스크 생성
  cv::Mat BuildRoiMask(int cols, int rows, const std::vector<YoloDet>& dets,
                       int target_idx) const;

  // 지정 상판 높이(z_plane)로의 실척 탑뷰 역투영 및 광택 강건 볼록 껍질 피팅
  bool FitOnPlane(const cv::Mat& rgb, const cv::Mat& roi_img,
                  const CamInfoMsg& info, const Vec3& cam_pos,
                  const std::array<Vec3, 3>& cam_axes, double box_x,
                  double box_y, double z_plane, const Puck* other,
                  Puck& io_puck, bool& out_lying);

  rclcpp::Node::SharedPtr node_;
  bool flip_180_ = false;
  cudaStream_t cuda_stream_ = nullptr;
  std::unique_ptr<TrtYolo> yolo_;
  mutable std::mutex mtx_;
  ImageMsg::ConstSharedPtr latest_img_;
  CamInfoMsg::ConstSharedPtr latest_info_;
  std::chrono::steady_clock::time_point last_rx_{};
  uint64_t img_seq_ = 0;
  rclcpp::Subscription<ImageMsg>::SharedPtr img_sub_;
  rclcpp::Subscription<CamInfoMsg>::SharedPtr info_sub_;
  rclcpp::Publisher<Det2DArray>::SharedPtr det_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr debug_pub_;
};

}  // namespace xsting_pick_place

#endif  // XSTING_PICK_PLACE_ARDUCAM_ALIGN_H_
