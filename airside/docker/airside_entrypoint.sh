#!/usr/bin/env bash

set -eo pipefail

export PYTHONPATH="/monorepo${PYTHONPATH:+:${PYTHONPATH}}"

source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash

exec "$@"
