#!/usr/bin/env bash
# Run installed Airside package tests in the rebuilt image.
set -euo pipefail
cd /ros_ws
colcon test
colcon test-result --verbose
