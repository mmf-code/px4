#!/usr/bin/env bash

# Quick verifier for ROS 2 Humble + essentials (mavros, mavros_msgs, colcon, rosdep)
# Usage:
#   bash scripts/check_ros2_humble.sh
# Exits 0 if everything needed is present, 1 otherwise.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"

ok=true

echo "[CHECK] Looking for /opt/ros/humble/setup.bash"
if [ -f /opt/ros/humble/setup.bash ]; then
  export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  echo "[ OK ] Found ROS 2 Humble setup script"
else
  echo "[FAIL] /opt/ros/humble/setup.bash not found (ROS 2 Humble not installed?)"
  ok=false
fi

echo "[CHECK] ros2 CLI availability"
if command -v ros2 >/dev/null 2>&1; then
  echo "[ OK ] ros2 CLI found: $(ros2 --version 2>/dev/null || echo unknown)"
else
  echo "[FAIL] ros2 CLI not found in PATH"
  ok=false
fi

echo "[CHECK] ROS_DISTRO == humble"
if [ "${ROS_DISTRO:-}" = "humble" ]; then
  echo "[ OK ] ROS_DISTRO=${ROS_DISTRO}"
else
  echo "[FAIL] ROS_DISTRO is '${ROS_DISTRO:-unset}', expected 'humble'"
  ok=false
fi

echo "[CHECK] colcon"
if command -v colcon >/dev/null 2>&1; then
  echo "[ OK ] colcon found"
else
  echo "[FAIL] colcon not found (install python3-colcon-common-extensions)"
  ok=false
fi

echo "[CHECK] rosdep"
if command -v rosdep >/dev/null 2>&1; then
  echo "[ OK ] rosdep found"
else
  echo "[FAIL] rosdep not found (install python3-rosdep)"
  ok=false
fi

echo "[CHECK] mavros + mavros_msgs packages"
have_mavros=false
have_mavros_msgs=false
if command -v ros2 >/dev/null 2>&1; then
  pkgs="$(ros2 pkg list 2>/dev/null || true)"
  if printf '%s\n' "$pkgs" | grep -qx "mavros"; then have_mavros=true; fi
  if printf '%s\n' "$pkgs" | grep -qx "mavros_msgs"; then have_mavros_msgs=true; fi
fi

if $have_mavros; then
  echo "[ OK ] mavros package present"
else
  echo "[WARN] mavros package missing (install ros-humble-mavros)"
fi

if $have_mavros_msgs; then
  echo "[ OK ] mavros_msgs package present"
else
  echo "[WARN] mavros_msgs package missing (install ros-humble-mavros or ros-humble-mavros-extras)"
fi

# Final result
if $ok && $have_mavros && $have_mavros_msgs; then
  echo "[RESULT] ROS 2 Humble environment looks GOOD."
  exit 0
else
  echo
  echo "[RESULT] Missing components detected. Suggested installs:"
  echo "  sudo apt update && sudo apt install -y \\" 
  echo "    ros-humble-desktop python3-colcon-common-extensions python3-rosdep \\" 
  echo "    ros-humble-mavros ros-humble-mavros-extras"
  echo "  sudo rosdep init || true && rosdep update"
  exit 1
fi
