#!/usr/bin/env bash
# Install + enable the Arducam V4L2 outdoor setup to run on every boot.
#
# Usage:
#   sudo ./scripts/install_arducam_v4l2_service.sh
#   sudo ./scripts/install_arducam_v4l2_service.sh --crontab   # alternative to systemd

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_SCRIPT="${SCRIPT_DIR}/setup_arducam_v4l2.sh"
UNIT_SRC="${SCRIPT_DIR}/arducam-v4l2.service"
UNIT_DST="/etc/systemd/system/arducam-v4l2.service"

if [[ "${EUID}" -ne 0 ]]; then
  echo "error: run as root (sudo)" >&2
  exit 1
fi

chmod +x "$SETUP_SCRIPT"

if [[ "${1:-}" == "--crontab" ]]; then
  # Remove any previous @reboot line for this script, then add a fresh one.
  existing="$(crontab -l 2>/dev/null || true)"
  filtered="$(printf '%s\n' "$existing" | grep -v 'setup_arducam_v4l2.sh' || true)"
  {
    printf '%s\n' "$filtered"
    echo "@reboot /bin/bash ${SETUP_SCRIPT} >> /var/log/arducam-v4l2.log 2>&1"
  } | crontab -
  echo "Installed crontab @reboot entry. Current crontab:"
  crontab -l
  echo "Note: prefer systemd (run without --crontab) — it retries if /dev/video0 is late."
  exit 0
fi

sed "s|ExecStart=.*|ExecStart=${SETUP_SCRIPT}|" "$UNIT_SRC" > "$UNIT_DST"
systemctl daemon-reload
systemctl enable arducam-v4l2.service
systemctl restart arducam-v4l2.service
systemctl --no-pager --full status arducam-v4l2.service || true
echo
echo "Enabled arducam-v4l2.service for boot."
echo "Logs: journalctl -u arducam-v4l2.service -b"
