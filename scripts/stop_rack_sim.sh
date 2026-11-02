#!/bin/bash
set -e
PROJECT="$(cd "$(dirname "$0")/.." && pwd -P)"
source /opt/ros/jazzy/setup.bash
source "$PROJECT/install/setup.bash"
export ROBOT_RUNTIME_DIR="${ROBOT_RUNTIME_DIR:-$PROJECT/runtime_logs}"
exec ros2 run rack_simulation stop_simulation
