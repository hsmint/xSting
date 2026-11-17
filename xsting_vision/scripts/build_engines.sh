#!/usr/bin/env bash
# 현재 CPU 아키텍처(arm64 / x86_64)에 맞는 TensorRT 엔진 2종 생성
set -e

# 아키텍처 판별 및 입출력 폴더 설정
cd "$(dirname "$0")/.."
case "$(uname -m)" in
    aarch64|arm64) ARCH="arm64" ;;
    x86_64|amd64)  ARCH="x86_64" ;;
    *) echo "지원하지 않는 아키텍처: $(uname -m)"; exit 1 ;;
esac
ONNX_DIR="$PWD/model/onnx"
OUT_DIR="$PWD/model/$ARCH"
mkdir -p "$OUT_DIR"

# 임시 컴포넌트 컨테이너 실행 및 종료 시 자동 정리
ros2 run rclcpp_components component_container \
    --ros-args -r __node:=trt_builder &
PID=$!
trap 'kill $PID 2>/dev/null || true' EXIT
sleep 2

# 단일 ONNX 모델을 TensorRTNode로 로드해 엔진 파일로 저장
build_engine() {
    local name=$1
    echo "빌드 시작 ($ARCH): ${name}.engine"
    ros2 component load --no-daemon /trt_builder isaac_ros_tensor_rt \
        nvidia::isaac_ros::dnn_inference::TensorRTNode \
        -n "trt_${name%%_*}" \
        -p "model_file_path:=$ONNX_DIR/${name}.onnx" \
        -p "engine_file_path:=$OUT_DIR/${name}.engine" \
        -p "force_engine_update:=true" \
        -p "input_tensor_names:=['input_tensor']" \
        -p "input_binding_names:=['images']" \
        -p "output_tensor_names:=['output_tensor']" \
        -p "output_binding_names:=['output0']"
}

# 퍽·박스 모델과 신호등(yolo_sign_v2) 모델 순서대로 빌드
build_engine "rg_nano_batch1_640_fp16"
build_engine "sign_nano_batch1_640_fp16"
echo "완료: $OUT_DIR"
