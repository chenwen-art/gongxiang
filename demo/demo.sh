#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SETUP_BASH="$SCRIPT_DIR/devel/setup.bash"
DEFAULT_MAP_FILE="$SCRIPT_DIR/src/robot_slam/maps/my_lab.yaml"
MAP_FILE="${1:-$DEFAULT_MAP_FILE}"

if [ ! -f "$SETUP_BASH" ]; then
  echo "未找到 $SETUP_BASH，请先在 demo 工作空间执行 catkin_make"
  exit 1
fi

if [ ! -f "$MAP_FILE" ]; then
  echo "未找到地图文件: $MAP_FILE"
  echo "请把导航地图放到 $DEFAULT_MAP_FILE，或运行:"
  echo "  bash demo.sh /绝对路径/你的地图.yaml"
  exit 1
fi

source "$SETUP_BASH"
roslaunch abot_bringup robot_with_imu.launch  &
PID1=$!
sleep 15
roslaunch robot_slam competition_ready.launch map_file:="$MAP_FILE"
