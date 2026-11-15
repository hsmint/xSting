#!/usr/bin/env bash
set -euo pipefail

# 스크립트 위치 기준 워크스페이스 calibration.yaml 기본 경로 및 상자 간격(m) 설정
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_SRC="/home/pa03/Documents/FINAL/mk2_ws/src/xsting_approach/config/calibration.yaml"
if [[ ! -d "$(dirname "${PKG_SRC}")" ]]; then
  PKG_SRC="$(cd "${SCRIPT_DIR}/.." && pwd)/config/calibration.yaml"
fi
CALIB_FILE="${1:-${PKG_SRC}}"
BOX_GAP_M="${2:-0.200}"

echo "=========================================================="
echo " xSting 2cm 정밀 접근 실기 보정 마법사"
echo " 저장 파일: ${CALIB_FILE}"
echo " 상자 기준 간격: ${BOX_GAP_M} m"
echo "=========================================================="

# 1단계: 선택적 관절 영점 보정 (위치제어 로봇은 기본 건너뜀 권장)
read -r -p "[1/3] 관절 영점 보정을 실행할까요? (y/N): " DO_JOINTS
if [[ "${DO_JOINTS:-N}" =~ ^[Yy]$ ]]; then
  read -r -p "현재 로봇 자세 선택 [boot/stand/zero] (기본 boot): " POSE_MODE
  ros2 run xsting_approach calib_joints_node --ros-args \
    -p calib_file:="${CALIB_FILE}" \
    -p pose:="${POSE_MODE:-boot}"
fi

# 2단계: 평지 기립 상태 IMU 장착 회전 및 자이로 바이어스 보정
echo ""
read -r -p "[2/3] 평지에 로봇을 정지시킨 뒤 Enter를 누르면 IMU를 보정합니다..." _
ros2 run xsting_approach calib_imu_node --ros-args \
  -p calib_file:="${CALIB_FILE}"

# 3단계: 전면 D435 바닥·상자 PCA·초음파 영점 동시 보정
echo ""
echo "[3/3] 차체 전면(X=0.162m)에서 정확히 ${BOX_GAP_M} m 앞에 상자를 정방향으로 놓아주세요."
read -r -p "배치 완료 후 Enter를 누르면 카메라·초음파 보정을 시작합니다..." _
ros2 run xsting_approach calib_cam_node --ros-args \
  -p calib_file:="${CALIB_FILE}" \
  -p box_gap_m:="${BOX_GAP_M}"

echo ""
echo "실기 보정 완료: ${CALIB_FILE}"
