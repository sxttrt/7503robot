#!/bin/bash
set -e
cd "$(dirname "$0")/.."
PROJECT="$PWD"
if [[ ! -f install/setup.bash || ! -f .build-root || "$(cat .build-root)" != "$PROJECT" ]]; then
  echo '请先在当前项目执行 bash scripts/build.sh --robot'; exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$PROJECT/install/setup.bash"
export ROBOT_PROJECT_ROOT="$PROJECT"
# Same default ROS domain as robot.launch.py; an explicitly configured domain is preserved.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
exec ros2 run robot_navigation chassis_test --ros-args \
  -r __ns:=/team2/robot -p use_sim_time:=false \
  -p config_file:="$PROJECT/src/robot_bringup/config/navigation_robot.yaml" "$@"
