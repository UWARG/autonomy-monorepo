#!/usr/bin/env bash

set -eo pipefail

export PYTHONPATH="/monorepo${PYTHONPATH:+:${PYTHONPATH}}"

# Non-optimal Jetson image installs CUDA OpenCV under /opt/opencv.
if [[ -f /etc/opencv-python-path ]]; then
  export PYTHONPATH="$(cat /etc/opencv-python-path)${PYTHONPATH:+:${PYTHONPATH}}"
fi
if [[ -f /etc/profile.d/opencv-python.sh ]]; then
  # shellcheck source=/dev/null
  source /etc/profile.d/opencv-python.sh
fi

source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash

mkdir -p /images
shopt -s nullglob
rm -f /images/takeoff_*.png /images/landing_*.png
shopt -u nullglob

# Outdoor Arducam V4L2 controls (auto AE + capped gain) before the camera node opens.
if [[ -e /dev/video0 ]] && [[ -x /monorepo/camera/scripts/setup_arducam_v4l2.sh ]]; then
  /monorepo/camera/scripts/setup_arducam_v4l2.sh || true
fi

exec "$@"
