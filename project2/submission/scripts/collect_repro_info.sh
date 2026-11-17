#!/usr/bin/env bash
# ==============================================================================
# xSting MK2 — 팀원 재현 시험 환경·버전·커밋·테스트 결과 자동 채취 스크립트
# 비밀번호, API 키 등 비밀 환경변수는 절대 읽거나 출력하지 않습니다.
# 사용법:
#   bash submission/scripts/collect_repro_info.sh [--run-tests] [실행자이름]
# ==============================================================================
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUN_TESTS=false
TESTER_NAME="${USER}"

for arg in "$@"; do
  if [ "$arg" = "--run-tests" ]; then
    RUN_TESTS=true
  else
    TESTER_NAME="$arg"
  fi
done

NOW_KST="$(TZ=Asia/Seoul date '+%Y-%m-%d %H:%M:%S %Z')"
OS_STR="$(lsb_release -ds 2>/dev/null || grep PRETTY_NAME /etc/os-release | cut -d= -f2 | tr -d '\"')"
KERNEL_STR="$(uname -srp)"
GPU_STR="$(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null | head -n1 || echo 'N/A')"
CUDA_STR="$(nvcc --version 2>/dev/null | awk -F'release ' '/release/{print $2}' || echo 'N/A')"
PY_STR="$(python3 --version 2>/dev/null || echo 'N/A')"

GIT_COMMIT="$(git -C "${WS_DIR}" rev-parse --short=12 HEAD 2>/dev/null || git -C "${WS_DIR}/src" rev-parse --short=12 HEAD 2>/dev/null || echo 'N/A (non-git copy)')"
GIT_BRANCH="$(git -C "${WS_DIR}" rev-parse --abbrev-ref HEAD 2>/dev/null || git -C "${WS_DIR}/src" rev-parse --abbrev-ref HEAD 2>/dev/null || echo 'N/A')"

pkg_ver() {
  dpkg-query -W -f='${Version}' "$1" 2>/dev/null || echo "미설치"
}

ROS_VER="$(pkg_ver ros-lyrical-desktop)"
CUVSLAM_VER="$(pkg_ver ros-lyrical-isaac-ros-cuvslam)"
NVBLOX_VER="$(pkg_ver ros-lyrical-nvblox-ros)"
MOVEIT_VER="$(pkg_ver ros-lyrical-moveit)"
FG_VER="$(pkg_ver ros-lyrical-foxglove-bridge)"

ACM_DEVS="$(ls /dev/ttyACM* 2>/dev/null | tr '\n' ' ' || echo '미연결')"
LIDAR_DEV="$([ -e /dev/lidar ] && echo '/dev/lidar 인식됨' || echo '미연결')"
ARDU_DEV="$([ -e /dev/arducam ] && echo '/dev/arducam 인식됨' || echo '미연결')"
RS_DEV="$(lsusb 2>/dev/null | grep -i 'Intel.*RealSense\|8086:0b07' | head -n1 || echo '미연결')"

TEST_SUMMARY="미실행 (--run-tests 옵션 추가 시 자동 수행)"
if [ "$RUN_TESTS" = true ]; then
  cd "${WS_DIR}"
  # shellcheck disable=SC1091
  source /opt/ros/lyrical/setup.bash
  if [ -f "${WS_DIR}/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${WS_DIR}/install/setup.bash"
  fi
  if colcon test --packages-select xsting_nav xsting_pick_place xsting_mission >/dev/null 2>&1; then
    TEST_SUMMARY="$(colcon test-result --all | tail -n 1)"
  else
    TEST_SUMMARY="일부 테스트 실패 (colcon test-result --verbose 확인 필요)"
  fi
fi

cat <<EOF
### 재현 시험 자동 채취 스냅샷 (${NOW_KST})

| 항목 | 실측 기록 |
|---|---|
| **실행 일시** | \`${NOW_KST}\` |
| **실행자** | \`${TESTER_NAME}\` |
| **워크스페이스 경로** | \`${WS_DIR}\` |
| **Git 브랜치 / 커밋** | \`${GIT_BRANCH}\` / \`${GIT_COMMIT}\` |
| **OS / 커널** | \`${OS_STR}\` / \`${KERNEL_STR}\` |
| **GPU / 드라이버 / CUDA** | \`${GPU_STR}\` / CUDA \`${CUDA_STR}\` |
| **ROS 2 / MoveIt / Foxglove** | \`${ROS_VER}\` / \`${MOVEIT_VER}\` / \`${FG_VER}\` |
| **cuVSLAM / nvblox / Python** | \`${CUVSLAM_VER}\` / \`${NVBLOX_VER}\` / \`${PY_STR}\` |
| **연결 장치 (OpenCR / LiDAR / ArduCam)** | OpenCR: \`${ACM_DEVS:-미연결}\` / LiDAR: \`${LIDAR_DEV}\` / ArduCam: \`${ARDU_DEV}\` |
| **전면 RealSense USB** | \`${RS_DEV}\` |
| **자동 단위 테스트 결과** | \`${TEST_SUMMARY}\` |
EOF
