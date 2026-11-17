#!/usr/bin/env bash
set -euo pipefail

# xSting 전방향 보행(xsting_walk) 대화형 실측 보정 마법사
CALIB_FILE="${1:-src/xsting_walk/config/calibration.yaml}"
TRIAL_ENV="/tmp/xsting_walk_trial_$$.env"
trap 'rm -f "${TRIAL_ENV}"' EXIT

# 노드 실행 중 눌린 잔여 키 입력(버퍼된 Enter 등)을 비워 다음 프롬프트 밀림 방지
flush_stdin() {
  if [[ -t 0 ]]; then
    while read -r -t 0.01 -n 10000 _ 2>/dev/null; do :; done || true
  fi
}

# 자식 ROS 2 프로세스가 터미널 stdin을 가로채지 않도록 격리 실행
run_ros2() {
  local rc=0
  "$@" </dev/null || rc=$?
  flush_stdin
  if (( rc >= 128 )); then
    exit "${rc}"
  fi
  return "${rc}"
}

# walk_node 미실행 상태에서 무한 대기하지 않도록 타임아웃 적용 리로드
reload_walk_calib() {
  timeout 2s ros2 service call /walk/reload_calib std_srvs/srv/Trigger "{}" \
    </dev/null >/dev/null 2>&1 || true
  flush_stdin
}

wait_enter() {
  flush_stdin
  read -r -p "$1" _ || exit 1
}

# y/n 입력 검증 헬퍼 (오타 입력 시 재질문)
ask_yn() {
  local __var="$1"
  local prompt="$2"
  local def="$3"
  local raw=""
  while true; do
    read -r -p "${prompt}" raw || exit 1
    raw="${raw#"${raw%%[![:space:]]*}"}"
    raw="${raw%"${raw##*[![:space:]]}"}"
    if [[ -z "${raw}" ]]; then
      printf -v "${__var}" '%s' "${def}"
      return 0
    elif [[ "${raw}" =~ ^[Yy]([Ee][Ss])?$ ]]; then
      printf -v "${__var}" 'Y'
      return 0
    elif [[ "${raw}" =~ ^[Nn]([Oo])?$ ]]; then
      printf -v "${__var}" 'N'
      return 0
    fi
    echo "    [입력 오류] 'y' 또는 'n' 을 입력해주세요 (Enter 시 기본값: ${def})."
  done
}

# 정수(예: 1, 180)나 소수를 ROS 2 double 파라미터 호환 문자열(예: 1.0, 180.0)로 정규화
normalize_float() {
  LC_ALL=C awk -v v="$1" 'BEGIN {
    s = sprintf("%.6f", v + 0.0)
    sub(/0+$/, "", s)
    sub(/\.$/, ".0", s)
    if (s == "-0.0") s = "0.0"
    print s
  }'
}

# 실수 입력 검증 헬퍼 (mode: any | nonzero | positive | optional | clearance)
ask_float() {
  local __var="$1"
  local prompt="$2"
  local def="$3"
  local mode="${4:-any}"
  local raw=""
  while true; do
    read -r -p "${prompt}" raw || exit 1
    raw="${raw#"${raw%%[![:space:]]*}"}"
    raw="${raw%"${raw##*[![:space:]]}"}"

    if [[ -z "${raw}" ]]; then
      if [[ -n "${def}" ]]; then
        raw="${def}"
      elif [[ "${mode}" == "optional" || "${mode}" == "clearance" ]]; then
        printf -v "${__var}" ''
        return 0
      else
        echo "    [입력 오류] 숫자 값을 입력해주세요."
        continue
      fi
    elif [[ "${mode}" == "optional" || "${mode}" == "clearance" ]] && \
         [[ "${raw}" =~ ^([Ss]|[Ss][Kk][Ii][Pp]|[Qq])$ ]]; then
      printf -v "${__var}" ''
      return 0
    fi

    if [[ ! "${raw}" =~ ^[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
      echo "    [입력 오류] '${raw}' 은(는) 유효한 숫자가 아닙니다. 숫자를 다시 입력해주세요."
      continue
    fi

    if [[ "${mode}" == "positive" ]] && \
       ! LC_ALL=C awk -v v="${raw}" 'BEGIN { exit !(v + 0.0 > 0.0) }'; then
      echo "    [입력 오류] 0보다 큰 값을 입력해주세요."
      continue
    fi

    if [[ "${mode}" == "nonzero" ]] && \
       ! LC_ALL=C awk -v v="${raw}" 'BEGIN { exit !(v + 0.0 != 0.0) }'; then
      echo "    [입력 오류] 0이 아닌 값을 입력해주세요."
      continue
    fi

    if [[ "${mode}" == "clearance" ]] && \
       ! LC_ALL=C awk -v v="${raw}" 'BEGIN { x = v + 0.0; exit !(x >= 0.008 && x <= 0.080) }'; then
      echo "    [입력 오류] m 단위로 0.008 ~ 0.080 범위 내 값을 입력해주세요 (예: 0.036)."
      continue
    fi

    printf -v "${__var}" '%s' "$(normalize_float "${raw}")"
    return 0
  done
}

ask_pose() {
  local __var="$1"
  local raw=""
  while true; do
    read -r -p "  현재 로봇 자세 선택 [boot/stand/zero] (기본 boot): " raw || exit 1
    raw="${raw#"${raw%%[![:space:]]*}"}"
    raw="${raw%"${raw##*[![:space:]]}"}"
    raw="${raw:-boot}"
    if [[ "${raw}" == "boot" || "${raw}" == "stand" || "${raw}" == "zero" ]]; then
      printf -v "${__var}" '%s' "${raw}"
      return 0
    fi
    echo "    [입력 오류] boot, stand, zero 중 하나를 입력해주세요."
  done
}

ask_joint_or_q() {
  local __var="$1"
  local raw=""
  while true; do
    read -r -p "  점검할 joint_index (0~17, 종료는 q): " raw || exit 1
    raw="${raw#"${raw%%[![:space:]]*}"}"
    raw="${raw%"${raw##*[![:space:]]}"}"
    if [[ -z "${raw}" || "${raw}" =~ ^[Qq]$ ]]; then
      printf -v "${__var}" 'q'
      return 0
    fi
    if [[ "${raw}" =~ ^[0-9]+$ ]] && (( 10#${raw} >= 0 && 10#${raw} <= 17 )); then
      printf -v "${__var}" '%d' "$(( 10#${raw} ))"
      return 0
    fi
    echo "    [입력 오류] 0~17 사이의 정수 또는 종료('q')를 입력해주세요."
  done
}

echo "=================================================================="
echo "  xSting 전방향 카펫 보행(xsting_walk) 실측 보정 마법사"
echo "  저장 파일: ${CALIB_FILE}"
echo "=================================================================="
echo "  * 사전 준비: xsting_hardware (및 보행 시험 시 walk_node) 실행 상태"
echo "  * 보정 항목:"
echo "    [1/4] 관절 영점(offset)·회전 부호·보행 기립 자세 점검"
echo "    [2/4] IMU 수평 회전·자이로·가속도(9.81 m/s^2) 영점 보정"
echo "    [3/4] 실측(줄자·각도기 + SLAM/IMU) 전후·횡이동·회전 스케일 보정"
echo "    [4/4] 카펫 발끝 높이(swing_clearance) 미세 조정 및 실시간 반영"
echo "=================================================================="

# ------------------------------------------------------------------
# 1단계: 관절 영점·부호·기립 자세 점검
# ------------------------------------------------------------------
echo ""
ask_yn DO_JOINTS "[1/4] 관절 영점 보정을 실행할까요? (위치제어 기본 건너뜀 권장) (y/N): " "N"
if [[ "${DO_JOINTS}" == "Y" ]]; then
  ask_pose POSE_MODE
  run_ros2 ros2 run xsting_walk calib_joints_node --ros-args \
    -p calib_file:="${CALIB_FILE}" \
    -p action:="offset" \
    -p pose:="${POSE_MODE}" || \
    echo "  [경고] 관절 영점 보정 실행 중 오류가 발생했습니다."
fi

ask_yn DO_NUDGE "[1/4-보조] 개별 관절 회전 방향(+6deg 왕복)을 점검할까요? (y/N): " "N"
while [[ "${DO_NUDGE}" == "Y" ]]; do
  echo "  관절 인덱스: 0..2=leg1(yaw,hip,knee), 3..5=leg2, 6..8=leg3, 9..11=leg4, 12..14=leg5, 15..17=leg6"
  ask_joint_or_q JIDX
  if [[ "${JIDX}" == "q" ]]; then
    break
  fi
  if run_ros2 ros2 run xsting_walk calib_joints_node --ros-args \
      -p calib_file:="${CALIB_FILE}" \
      -p action:="nudge" \
      -p joint_index:="${JIDX}"; then
    echo "  기준 양(+) 방향: yaw(+) = 발 앞쪽(+X), hip(+) = 허벅지 위로, knee(+) = 발끝 위로"
    ask_yn DO_FLIP "  반대 방향으로 움직였나요? 부호를 반전할까요? (y/N): " "N"
    if [[ "${DO_FLIP}" == "Y" ]]; then
      run_ros2 ros2 run xsting_walk calib_joints_node --ros-args \
        -p calib_file:="${CALIB_FILE}" \
        -p action:="flip_sign" \
        -p joint_index:="${JIDX}" || \
        echo "  [경고] 관절 부호 반전 저장 중 오류가 발생했습니다."
    fi
  else
    echo "  [경고] 관절 왕복 점검 실행에 실패했습니다."
  fi
  ask_yn DO_NUDGE "  다른 관절도 점검할까요? (y/N): " "N"
done

# ------------------------------------------------------------------
# 2단계: IMU 장착 회전 + 자이로 + 가속도 영점 실측 보정
# ------------------------------------------------------------------
echo ""
ask_yn DO_IMU "[2/4] 평평한 카펫 위에 로봇을 기립 정지시킨 뒤 IMU를 보정할까요? (Y/n): " "Y"
if [[ "${DO_IMU}" == "Y" ]]; then
  ask_yn TWO_POSE "  2자세(수평 + 전방 숙임) 정밀 보정을 사용할까요? (y/N, 기본 1자세 수평): " "N"
  if [[ "${TWO_POSE}" == "Y" ]]; then
    echo "  -> 먼저 수평 정지 상태로 대기합니다. '1단계 완료' 로그가 뜨면 다른 터미널이나 안내에 따라 차체를 숙여주세요."
    run_ros2 ros2 run xsting_walk calib_imu_node --ros-args \
      -p calib_file:="${CALIB_FILE}" \
      -p two_pose:=true || \
      echo "  [경고] IMU 2자세 보정 실행 중 오류가 발생했습니다."
  else
    wait_enter "  정지 상태를 확인하고 Enter를 누르면 IMU 수집을 시작합니다..."
    run_ros2 ros2 run xsting_walk calib_imu_node --ros-args \
      -p calib_file:="${CALIB_FILE}" \
      -p two_pose:=false || \
      echo "  [경고] IMU 보정 실행 중 오류가 발생했습니다."
  fi
  # 실행 중인 walk_node가 있으면 즉시 리로드 (미실행 시 무한 대기 방지)
  reload_walk_calib
fi

# ------------------------------------------------------------------
# 3단계: 실측 보행 속도 배율(vx, vy, wz) 및 직진 편향 보정
# ------------------------------------------------------------------
echo ""
echo "[3/4] 실측 보행 속도 스케일 및 직진 편향 보정"
echo "  * 이 단계는 walk_node가 실행 중이어야 합니다:"
echo "    ros2 launch xsting_walk walk_teleop.launch.py calib_file:=${CALIB_FILE}"

# (A) 전진(+vx) 실측 보정
echo ""
ask_yn DO_VX "  [3-A] 전진(+vx) 실측 시험 주행을 진행할까요? (Y/n): " "Y"
if [[ "${DO_VX}" == "Y" ]]; then
  ask_float VX_SPD "    목표 전진 속도 [m/s] (기본 0.20): " "0.20" "nonzero"
  ask_float VX_DUR "    주행 시간 [s] (기본 3.0): " "3.0" "positive"
  echo "    차체 전방 약 1m 공간을 확보하고 바닥에 출발 기준선을 표시한 뒤 Enter를 누르세요..."
  wait_enter "    준비 완료 시 Enter..."

  rm -f "${TRIAL_ENV}"
  if run_ros2 ros2 run xsting_walk calib_gait_node --ros-args \
      -p calib_file:="${CALIB_FILE}" \
      -p mode:="trial" \
      -p axis:="vx" \
      -p speed:="${VX_SPD}" \
      -p duration_s:="${VX_DUR}" \
      -p result_file:="${TRIAL_ENV}"; then

    HAS_ODOM=0; HAS_IMU=0; EXPECTED_M=0; ODOM_DX_M=0; ODOM_DY_M=0; IMU_DYAW_DEG=0
    if [[ -f "${TRIAL_ENV}" ]]; then
      # shellcheck disable=SC1090
      source "${TRIAL_ENV}"
    fi

    DEF_DIST=""
    DEF_LAT="0.0"
    if [[ "${HAS_ODOM}" == "1" ]]; then
      DEF_DIST="${ODOM_DX_M}"
      DEF_LAT="${ODOM_DY_M}"
      echo "    [센서 계측] 목표=${EXPECTED_M} m | cuVSLAM 전진=${ODOM_DX_M} m, 좌우편차=${ODOM_DY_M} m | IMU 회전편차=${IMU_DYAW_DEG} deg"
    else
      echo "    [목표 이동거리] ${EXPECTED_M} m (cuVSLAM 미실행 -> 줄자 실측값을 입력해주세요)"
    fi

    ask_float MEAS_X "    줄자 실측 전진 거리 [m] 입력 (기본값 '${DEF_DIST:-건너뜀}', 건너뛰기 s): " "${DEF_DIST}" "optional"
    if [[ -n "${MEAS_X}" ]]; then
      ask_float MEAS_Y "    줄자 실측 좌우 쏠림 [m, +좌측/-우측] (기본 '${DEF_LAT}'): " "${DEF_LAT}" "any"
      run_ros2 ros2 run xsting_walk calib_gait_node --ros-args \
        -p calib_file:="${CALIB_FILE}" \
        -p mode:="apply" \
        -p axis:="vx" \
        -p speed:="${VX_SPD}" \
        -p duration_s:="${VX_DUR}" \
        -p measured_dist_m:="${MEAS_X}" \
        -p measured_lat_drift_m:="${MEAS_Y}" \
        -p has_lat_drift:=true || \
        echo "    [경고] 전진(+vx) 보정값 반영에 실패했습니다."
    fi
  else
    echo "    [경고] 전진(+vx) 시험 주행 실행에 실패했습니다."
  fi
fi

# (B) 좌측 횡이동(+vy) 실측 보정
echo ""
ask_yn DO_VY "  [3-B] 좌측 게걸음(+vy) 실측 시험 주행을 진행할까요? (y/N): " "N"
if [[ "${DO_VY}" == "Y" ]]; then
  ask_float VY_SPD "    목표 좌횡이동 속도 [m/s] (기본 0.18): " "0.18" "nonzero"
  ask_float VY_DUR "    주행 시간 [s] (기본 3.0): " "3.0" "positive"
  wait_enter "    차체 좌측(+Y) 약 0.8m 공간을 확보하고 Enter를 누르세요..."

  rm -f "${TRIAL_ENV}"
  if run_ros2 ros2 run xsting_walk calib_gait_node --ros-args \
      -p calib_file:="${CALIB_FILE}" \
      -p mode:="trial" \
      -p axis:="vy" \
      -p speed:="${VY_SPD}" \
      -p duration_s:="${VY_DUR}" \
      -p result_file:="${TRIAL_ENV}"; then

    HAS_ODOM=0; EXPECTED_M=0; ODOM_DY_M=0
    if [[ -f "${TRIAL_ENV}" ]]; then
      # shellcheck disable=SC1090
      source "${TRIAL_ENV}"
    fi

    DEF_DIST=""
    if [[ "${HAS_ODOM}" == "1" ]]; then
      DEF_DIST="${ODOM_DY_M}"
      echo "    [센서 계측] 목표=${EXPECTED_M} m | cuVSLAM 좌횡이동=${ODOM_DY_M} m"
    else
      echo "    [목표 좌횡이동 거리] ${EXPECTED_M} m"
    fi

    ask_float MEAS_VY "    줄자 실측 좌횡이동 거리 [m] 입력 (기본값 '${DEF_DIST:-건너뜀}', 건너뛰기 s): " "${DEF_DIST}" "optional"
    if [[ -n "${MEAS_VY}" ]]; then
      run_ros2 ros2 run xsting_walk calib_gait_node --ros-args \
        -p calib_file:="${CALIB_FILE}" \
        -p mode:="apply" \
        -p axis:="vy" \
        -p speed:="${VY_SPD}" \
        -p duration_s:="${VY_DUR}" \
        -p measured_dist_m:="${MEAS_VY}" || \
        echo "    [경고] 좌횡이동(+vy) 보정값 반영에 실패했습니다."
    fi
  else
    echo "    [경고] 좌횡이동(+vy) 시험 주행 실행에 실패했습니다."
  fi
fi

# (C) 제자리 좌회전(+wz) 실측 보정
echo ""
ask_yn DO_WZ "  [3-C] 제자리 좌회전(+wz, 목표 약 180 deg) 실측 시험을 진행할까요? (y/N): " "N"
if [[ "${DO_WZ}" == "Y" ]]; then
  ask_float WZ_SPD "    목표 회전 각속도 [rad/s] (기본 0.90): " "0.90" "nonzero"
  ask_float WZ_DUR "    회전 시간 [s] (기본 3.49 -> 목표 180.0 deg): " "3.49" "positive"
  wait_enter "    차체 주변 회전 공간을 확보하고 출발 방향을 표시한 뒤 Enter를 누르세요..."

  rm -f "${TRIAL_ENV}"
  if run_ros2 ros2 run xsting_walk calib_gait_node --ros-args \
      -p calib_file:="${CALIB_FILE}" \
      -p mode:="trial" \
      -p axis:="wz" \
      -p speed:="${WZ_SPD}" \
      -p duration_s:="${WZ_DUR}" \
      -p result_file:="${TRIAL_ENV}"; then

    HAS_IMU=0; HAS_ODOM=0; EXPECTED_DEG=0; IMU_DYAW_DEG=0; ODOM_DYAW_DEG=0
    if [[ -f "${TRIAL_ENV}" ]]; then
      # shellcheck disable=SC1090
      source "${TRIAL_ENV}"
    fi

    DEF_DEG=""
    if [[ "${HAS_IMU}" == "1" ]]; then
      DEF_DEG="${IMU_DYAW_DEG}"
    elif [[ "${HAS_ODOM}" == "1" ]]; then
      DEF_DEG="${ODOM_DYAW_DEG}"
    fi
    echo "    [계측 요약] 목표=${EXPECTED_DEG} deg | IMU=${IMU_DYAW_DEG} deg | cuVSLAM=${ODOM_DYAW_DEG} deg"

    ask_float MEAS_DEG "    각도기/눈금 실측 회전 각도 [deg] 입력 (기본값 '${DEF_DEG:-건너뜀}', 건너뛰기 s): " "${DEF_DEG}" "optional"
    if [[ -n "${MEAS_DEG}" ]]; then
      run_ros2 ros2 run xsting_walk calib_gait_node --ros-args \
        -p calib_file:="${CALIB_FILE}" \
        -p mode:="apply" \
        -p axis:="wz" \
        -p speed:="${WZ_SPD}" \
        -p duration_s:="${WZ_DUR}" \
        -p measured_yaw_deg:="${MEAS_DEG}" || \
        echo "    [경고] 좌회전(+wz) 보정값 반영에 실패했습니다."
    fi
  else
    echo "    [경고] 좌회전(+wz) 시험 실행에 실패했습니다."
  fi
fi

# ------------------------------------------------------------------
# 4단계: 카펫 발끝 클리어런스(swing_clearance_m) 미세 조정
# ------------------------------------------------------------------
echo ""
ask_yn DO_CLR "[4/4] 카펫 두께에 맞춰 스윙 발끝 높이(swing_clearance_m, 기본 0.032m)를 변경할까요? (y/N): " "N"
if [[ "${DO_CLR}" == "Y" ]]; then
  ask_float NEW_CLR "  새 swing_clearance_m 입력 [0.015 ~ 0.055] (예: 0.036, 취소는 Enter): " "" "clearance"
  if [[ -n "${NEW_CLR}" ]]; then
    run_ros2 ros2 run xsting_walk calib_gait_node --ros-args \
      -p calib_file:="${CALIB_FILE}" \
      -p mode:="set_clearance" \
      -p swing_clearance_m:="${NEW_CLR}" || \
      echo "  [경고] 스윙 발끝 높이 반영에 실패했습니다."
  fi
fi

reload_walk_calib

echo ""
echo "=================================================================="
echo "  실측 보정 완료! 저장 위치: ${CALIB_FILE}"
echo "  텔레옵 실행 예시: ros2 run xsting_walk teleop_key_node"
echo "=================================================================="
