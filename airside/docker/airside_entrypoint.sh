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

exec "$@"
