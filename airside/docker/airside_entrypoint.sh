#!/usr/bin/env bash

set -eo pipefail

export PYTHONPATH="/monorepo/camera/src:/monorepo/utils/src${PYTHONPATH:+:${PYTHONPATH}}"

source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash

exec "$@"
