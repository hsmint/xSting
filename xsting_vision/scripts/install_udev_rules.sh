#!/bin/bash
# Arducam 및 2D LiDAR(COIN-D4) udev 규칙 설치 후 바로 적용
cd "$(dirname "$0")"
sudo cp 99-arducam.rules 99-lidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo udevadm settle
ls -l /dev/arducam /dev/lidar 2>/dev/null || true
