#!/usr/bin/env bash
# Optimize Arducam OV9782 V4L2 controls for bright outdoor capture.
#
# Defaults use firmware AE (auto_exposure=3) with gain/brightness pinned dark.
# Manual absolute exposure=1 still blows out in sun on this UVC bridge; AE is
# more reliable. Override with PREFER_AUTO=0 for forced manual.
#
# Usage:
#   ./scripts/setup_arducam_v4l2.sh
#   PREFER_AUTO=0 EXPOSURE=1 ./scripts/setup_arducam_v4l2.sh

set -euo pipefail

DEVICE="${DEVICE:-/dev/video0}"
WIDTH="${WIDTH:-640}"
HEIGHT="${HEIGHT:-480}"
BRIGHTNESS="${BRIGHTNESS:--20}"
EXPOSURE="${EXPOSURE:-1}"
GAIN="${GAIN:-0}"
GAMMA="${GAMMA:-100}"
CONTRAST="${CONTRAST:-32}"
PREFER_AUTO="${PREFER_AUTO:-1}"

if ! command -v v4l2-ctl >/dev/null 2>&1; then
  echo "error: v4l2-ctl not found (install v4l-utils)" >&2
  exit 1
fi

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

v4l2-ctl -d "$DEVICE" --set-fmt-video="width=${WIDTH},height=${HEIGHT},pixelformat=MJPG"

v4l2-ctl -d "$DEVICE" --set-ctrl="brightness=${BRIGHTNESS}"
v4l2-ctl -d "$DEVICE" --set-ctrl="gain=${GAIN}"
v4l2-ctl -d "$DEVICE" --set-ctrl="gamma=${GAMMA}"
v4l2-ctl -d "$DEVICE" --set-ctrl="contrast=${CONTRAST}"
v4l2-ctl -d "$DEVICE" --set-ctrl=backlight_compensation=0
v4l2-ctl -d "$DEVICE" --set-ctrl=exposure_dynamic_framerate=0

if [[ "$PREFER_AUTO" == "1" ]]; then
  v4l2-ctl -d "$DEVICE" --set-ctrl=auto_exposure=3
  echo "Mode: auto_exposure=3 (Aperture Priority) + dark gain/brightness"
else
  v4l2-ctl -d "$DEVICE" --set-ctrl=auto_exposure=1
  v4l2-ctl -d "$DEVICE" --set-ctrl="exposure_time_absolute=${EXPOSURE}"
  echo "Mode: manual exposure_time_absolute=${EXPOSURE}"
fi

echo "Current controls:"
v4l2-ctl -d "$DEVICE" --get-ctrl=auto_exposure,exposure_time_absolute,brightness,gain,gamma,contrast
v4l2-ctl -d "$DEVICE" --get-fmt-video
