#!/bin/bash
set -e
cd "$(dirname "$0")/.."
PROJECT="$(pwd -P)"
if [[ ! -f install/setup.bash || ! -f .build-root || "$(cat .build-root)" != "$PROJECT" ]]; then
  echo '请先在当前项目执行 bash scripts/build.sh'; exit 1
fi
if [[ ! -f install/rack_simulation/lib/librack_hybrid_actuator.so ]]; then
  echo '仿真执行插件尚未构建：请运行 bash scripts/build.sh（不要使用 --robot）'; exit 2
fi
source /opt/ros/jazzy/setup.bash
source "$PROJECT/install/setup.bash"
export ROBOT_RUNTIME_DIR="${ROBOT_RUNTIME_DIR:-$PROJECT/runtime_logs}"
exec ros2 run rack_simulation run_simulation --hybrid "$@"
