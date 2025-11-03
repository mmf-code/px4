#!/usr/bin/env bash
set -e

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux not found. Install with: sudo apt install -y tmux"
  exit 1
fi

SESSION="px4_demo"
tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n sim

# Pane 1: PX4 SITL
tmux send-keys -t "$SESSION":0 "cd $(pwd) && bash scripts/run_px4_sitl.sh classic" C-m

# Pane 2: MAVROS
tmux split-window -h -t "$SESSION":0
tmux send-keys -t "$SESSION":0.1 "source /opt/ros/humble/setup.bash && ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557 pluginlists_yaml:=/opt/ros/humble/share/mavros/launch/px4_pluginlists.yaml config_yaml:=/opt/ros/humble/share/mavros/launch/px4_config.yaml" C-m

# Pane 3: Offboard PID + RViz
tmux split-window -v -t "$SESSION":0.1
tmux send-keys -t "$SESSION":0.2 "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch px4_offboard_demo pid_mission.launch.py params_file:=ros2_ws/src/px4_offboard_demo/config/waypoints_demo.yaml" C-m

tmux select-pane -t "$SESSION":0.0
tmux attach -t "$SESSION"

