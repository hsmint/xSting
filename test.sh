#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "먼저 source /opt/ros/<배포판>/setup.bash 를 실행하세요." >&2
  exit 2
fi
packages=(xsting_interfaces xsting_bringup xsting_robot xsting_vision xsting_hardware)
colcon build --base-paths "${packages[@]}" --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
# ROS setup scripts may refer to unset variables.
set +u
source install/setup.bash
set -u
status=0
colcon test --packages-select "${packages[@]}" --return-code-on-test-failure || status=$?
colcon test-result --verbose || status=$?
python3 -m unittest discover -s training/tests -v || status=$?
exit "$status"
