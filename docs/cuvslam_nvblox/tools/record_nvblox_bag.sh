#!/usr/bin/env bash
# Record the robot's raw sensor streams on the Jetson's own disk, to run cuVSLAM and nvblox on the laptop
# later without the robot or Wi-Fi (replay steps: README "bag으로 PC에서 nvblox 돌리기").
#
# Before (default, cuVSLAM off on the Jetson): robot base A with use_mock_base:=false use_mock_hardware:=true
# wheel_odom_tf:=false, apply_calib B, top D435 C with IR + depth (+ color), imu_time_shift D. The 3rd-deck D435
# (C2, camera_name:=lower_camera) is recorded too whenever it publishes; the script prints which cameras it found.
#
#   bash record_nvblox_bag.sh                   # top IR pair + IMU (cuVSLAM), depth + color of every running D435
#   bash record_nvblox_bag.sh room2 --no-color  # without color (much smaller)
#   bash record_nvblox_bag.sh room3 --lower     # fail instead of recording if the 3rd-deck D435 is not up
#   bash record_nvblox_bag.sh room4 --with-slam # cuVSLAM runs on the Jetson (imu:=true tf:=true); its /tf and
#                                               # path go into the bag, so the laptop only runs nvblox
# Ctrl+C stops. Copy ~/bags/<name>_<date> to the laptop.
#
# /tf is recorded either way. Without --with-slam it must not hold odom -> base_footprint, because cuVSLAM on
# the laptop publishes that: so the wheel odom TF has to be off and cuVSLAM must not run here. Both are checked.
# Data rate per D435 at IR + depth 1280x720 and color 1920x1080 (30 fps) is ~300 MB/s; check a short take with
# ros2 bag info first (every image topic should have ~30 messages per second).
set -euo pipefail

name=loop
color=1
need_lower=0
slam=0
for arg in "$@"; do
  case "$arg" in
    --no-color) color=0 ;;
    --lower) need_lower=1 ;;
    --with-slam) slam=1 ;;
    --with-ir) ;;  # IR is always recorded now; kept so old commands still work
    -*) echo "unknown option $arg" >&2; exit 1 ;;
    *) name=$arg ;;
  esac
done

top=/camera/camera
lower=/camera/lower_camera
# Only topics someone publishes: `ros2 topic list` also lists topics that merely have a subscriber, so a
# cuVSLAM waiting on /opencr/imu_synced would make a missing imu_time_shift.py look fine.
available=$(ros2 topic list -v | awk '/^Published topics:/ {p = 1; next} /^Subscribed topics:/ {p = 0}
                                      p && $1 == "*" {print $2}')
published() { grep -qx "$1" <<<"$available"; }

# Required: missing ones stop the script. Optional: recorded when published.
required=(
  $top/infra1/image_rect_raw $top/infra1/camera_info $top/infra2/image_rect_raw $top/infra2/camera_info
  /opencr/imu_synced
  $top/depth/image_rect_raw $top/depth/camera_info
  /tf /tf_static
)
optional=()
((color)) && required+=($top/color/image_raw $top/color/camera_info)
((slam)) && required+=(/visual_slam/tracking/slam_path /visual_slam/tracking/odometry)

cameras="top"
if published $lower/depth/image_rect_raw || ((need_lower)); then
  cameras="top + 3rd deck"
  required+=($lower/depth/image_rect_raw $lower/depth/camera_info)
  ((color)) && optional+=($lower/color/image_raw $lower/color/camera_info)
  optional+=($lower/infra1/image_rect_raw $lower/infra1/camera_info
             $lower/infra2/image_rect_raw $lower/infra2/camera_info)
fi

# A missing input would only show up on the laptop, so check here first.
missing=()
for t in "${required[@]}"; do
  published "$t" || missing+=("$t")
done
if ((${#missing[@]})); then
  echo "Not published yet: ${missing[*]}" >&2
  echo "Top D435 (C): enable_infra1/2:=true enable_depth:=true, enable_color:=true (or --no-color)." >&2
  echo "/opencr/imu_synced: robot base with use_mock_base:=false, apply_calib_node, imu_time_shift.py." >&2
  echo "/tf: robot base launch (robot_state_publisher)." >&2
  echo "3rd-deck D435: realsense2_camera camera_name:=lower_camera enable_depth:=true (README C2)." >&2
  if ((slam)); then echo "slam_path / odometry: cuVSLAM with imu:=true tf:=true." >&2; fi
  exit 1
fi

if ((!slam)); then
  # cuVSLAM on the laptop owns odom -> base_footprint; a second source in the bag would fight it.
  if published /visual_slam/tracking/odometry; then
    echo "cuVSLAM is running here. Stop it (it runs on the laptop from this bag), or use --with-slam." >&2
    exit 1
  fi
  if ros2 param get /diff_drive_controller enable_odom_tf 2>/dev/null | grep -qi true; then
    echo "The wheel odometry publishes odom -> base_footprint. Restart the robot base with wheel_odom_tf:=false." >&2
    exit 1
  fi
fi

topics=("${required[@]}")
for t in "${optional[@]}"; do
  if published "$t"; then topics+=("$t"); fi
done

mkdir -p ~/bags
out=~/bags/${name}_$(date +%m%d_%H%M)
free_gb=$(df --output=avail -BG ~/bags | tail -1 | tr -dc 0-9)
if ((slam)); then where="recorded from the Jetson"; else where="run it on the laptop"; fi
echo "Cameras: $cameras. cuVSLAM: $where."
echo "Recording ${#topics[@]} topics to $out  (${free_gb} GB free)"
echo "Check a short take first: ros2 bag info $out  (image topics ~30 messages per second)"
echo "Leave the robot still for 3 s first, then drive the loop. Ctrl+C to stop."
# /tf has a VOLATILE and a TRANSIENT_LOCAL publisher; without the override the recorder picks the latter and
# records no robot_state_publisher /tf at all (2026-11-02). The yaml sits next to this script.
qos="$(dirname "$(readlink -f "$0")")/tf_qos_override.yaml"
exec ros2 bag record --storage mcap -o "$out" --qos-profile-overrides-path "$qos" --topics "${topics[@]}"
