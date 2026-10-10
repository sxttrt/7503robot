#!/bin/bash
set -e
cd "$(dirname "$0")/.."
PROJECT="$PWD"
if [[ ! -f install/setup.bash || ! -f .build-root || "$(cat .build-root)" != "$PROJECT" ]]; then
  echo '请先在当前项目执行 bash scripts/build.sh'; exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$PROJECT/install/setup.bash"
export ROBOT_PROJECT_ROOT="$PROJECT"
# 不拉起雷达驱动、ESP32、QR、升降或 Gazebo；这些模块的进程由相应负责人接入。
exec ros2 launch robot_bringup robot.launch.py "$@"
