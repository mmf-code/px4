#!/usr/bin/env bash
set -e

echo "[PX4] Cloning PX4-Autopilot (v1.14.x recommended) into ./PX4-Autopilot"

if [ ! -d PX4-Autopilot ]; then
  git clone https://github.com/PX4/PX4-Autopilot.git PX4-Autopilot
fi

cd PX4-Autopilot

# Checkout a stable tag (adjust if needed)
if git tag | grep -q "^v1.14"; then
  git checkout v1.14.3 || true
fi

echo "[PX4] Updating submodules"
git submodule update --init --recursive

echo "[PX4] NOTE: PX4 SITL Ubuntu dependencies may be required."
echo "      You can run: bash Tools/setup/ubuntu.sh --no-nuttx"
echo "      (This installs Gazebo and toolchains for SITL.)"

echo "[PX4] Building SITL target (first build may take a while)"
make px4_sitl_default || true

echo "[PX4] Done. Use scripts/run_px4_sitl.sh to start the simulator."
