#!/usr/bin/env bash
# Optimize Arducam OV9782 V4L2 controls for outdoor capture.
# Manual exposure (auto_exposure=1) so exposure_time_absolute is active.
#
# Usage:
#   ./scripts/setup_arducam_v4l2.sh
#   DEVICE=/dev/video0 EXPOSURE=40 BRIGHTNESS=-10 ./scripts/setup_arducam_v4l2.sh
#
# Env overrides:
#   DEVICE      video device          (default: /dev/video0)
#   WIDTH       capture width         (default: 640)
#   HEIGHT      capture height        (default: 480)
#   BRIGHTNESS  -64..64               (default: -10)
#   EXPOSURE    exposure_time_absolute 1..5000  (default: 40)
#   GAIN        0..100                (default: 0)

set -euo pipefail

DEVICE="${DEVICE:-/dev/video0}"
WIDTH="${WIDTH:-640}"
HEIGHT="${HEIGHT:-480}"
BRIGHTNESS="${BRIGHTNESS:--10}"
EXPOSURE="${EXPOSURE:-40}"
GAIN="${GAIN:-0}"

if ! command -v v4l2-ctl >/dev/null 2>&1; then
  echo "error: v4l2-ctl not found (install v4l-utils)" >&2
  exit 1
fi

# USB cameras often appear a few seconds after boot.
WAIT_SECS="${WAIT_SECS:-60}"
for ((i = 1; i <= WAIT_SECS; i++)); do
  if [[ -e "$DEVICE" ]]; then
    break
  fi
  sleep 1
done
if [[ ! -e "$DEVICE" ]]; then
  echo "error: device not found after ${WAIT_SECS}s: $DEVICE" >&2
  exit 1
fi

echo "Configuring $DEVICE for outdoor capture..."

# Match ROS camera_node / Arducam publish size (640x480 MJPG).
v4l2-ctl -d "$DEVICE" --set-fmt-video="width=${WIDTH},height=${HEIGHT},pixelformat=MJPG"

# Manual mode first — exposure_time_absolute is inactive until this is 1.
v4l2-ctl -d "$DEVICE" --set-ctrl=auto_exposure=1
v4l2-ctl -d "$DEVICE" --set-ctrl="exposure_time_absolute=${EXPOSURE}"
v4l2-ctl -d "$DEVICE" --set-ctrl="brightness=${BRIGHTNESS}"
v4l2-ctl -d "$DEVICE" --set-ctrl="gain=${GAIN}"

# Keep WB automatic; freeze other defaults that matter outdoors.
v4l2-ctl -d "$DEVICE" \
  --set-ctrl=white_balance_automatic=1 \
  --set-ctrl=contrast=32 \
  --set-ctrl=gamma=100 \
  --set-ctrl=backlight_compensation=0 \
  --set-ctrl=exposure_dynamic_framerate=0

echo "Applied: auto_exposure=1 exposure_time_absolute=${EXPOSURE} brightness=${BRIGHTNESS} gain=${GAIN}"
echo "Current controls:"
v4l2-ctl -d "$DEVICE" --get-ctrl=auto_exposure,exposure_time_absolute,brightness,gain
v4l2-ctl -d "$DEVICE" --get-fmt-video
