#!/usr/bin/env bash
set -euo pipefail

PX4_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)/PX4-Autopilot"

usage() {
  cat <<EOF
Usage: $(basename "$0") [gz|classic]

Default: try Gazebo (Sim) gz_x500, then fallback to Gazebo Classic.

Tips:
  - Force classic:    $(basename "$0") classic
  - Headless classic: HEADLESS=1 $(basename "$0") classic
  - If Gazebo (Sim) GUI crashes (Wayland/EGL), try classic or set QT_QPA_PLATFORM=xcb
EOF
}

MODE="auto"
if [[ ${1-} == "-h" || ${1-} == "--help" ]]; then
  usage; exit 0
elif [[ ${1-:-} == "gz" ]]; then
  MODE="gz"
elif [[ ${1-:-} == "classic" ]]; then
  MODE="classic"
fi

if [ ! -d "$PX4_DIR" ]; then
  echo "[ERR] PX4-Autopilot directory not found at $PX4_DIR"
  echo "      Run: bash scripts/get_px4.sh"
  exit 1
fi

cd "$PX4_DIR"

run_gz() {
  echo "[PX4] Launching Gazebo (Sim): gz_x500"
  if [[ "${HEADLESS:-0}" = "1" ]]; then
    echo "[WARN] HEADLESS requested but not supported for gz target via make."
    echo "       Consider using: $(basename "$0") classic with HEADLESS=1"
  fi
  make px4_sitl gz_x500
}

run_classic() {
  if [[ "${HEADLESS:-0}" = "1" ]]; then
    echo "[PX4] Launching Gazebo Classic (headless)"
    HEADLESS=1 make px4_sitl gazebo
  else
    echo "[PX4] Launching Gazebo Classic"
    make px4_sitl gazebo
  fi
}

if [[ "$MODE" == "gz" ]]; then
  run_gz
elif [[ "$MODE" == "classic" ]]; then
  run_classic
else
  set +e
  run_gz
  RET=$?
  set -e
  if [ $RET -ne 0 ]; then
    echo "[PX4] Gazebo (Sim) failed (code $RET). Falling back to Classic..."
    echo "       If the error mentions Wayland/EGL or gz_bridge timeouts, your GUI stack may be missing."
    echo "       Try Classic headless: HEADLESS=1 $(basename "$0") classic"
    run_classic
  fi
fi
