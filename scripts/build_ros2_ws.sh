#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
WS_DIR="$ROOT_DIR/ros2_ws"

if [ ! -d "$WS_DIR/src" ]; then
  echo "[ERR] Not found: $WS_DIR/src"
  exit 1
fi

echo "[ROS2] Checking ROS 2 Humble environment"
if ! bash "$ROOT_DIR/scripts/check_ros2_humble.sh"; then
  echo "[ROS2] Please install missing components above, then retry."
  exit 1
fi

source /opt/ros/humble/setup.bash

echo "[ROS2] Resolving dependencies via rosdep"
# Some distros lack a rosdep rule for 'ament_python'; skip it if present
rosdep install --rosdistro humble --from-paths "$WS_DIR/src" -y --ignore-src \
  --skip-keys "ament_python ament_cmake" || true

echo "[ROS2] Building workspace"
pushd "$WS_DIR" >/dev/null
colcon build
popd >/dev/null

echo "[ROS2] Done. Source: source $WS_DIR/install/setup.bash"
