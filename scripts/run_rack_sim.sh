#!/bin/bash
set -e
cd "$(dirname "$0")/.."
FRAMEWORK="$PWD"
if [[ ! -f "$FRAMEWORK/install/setup.bash" || ! -f "$FRAMEWORK/.build-root" || "$(cat "$FRAMEWORK/.build-root")" != "$FRAMEWORK" ]]; then
  echo "首次使用或项目位置已改变：请在此目录运行 bash scripts/build.sh"; exit 2
fi
if [[ ! -f install/rack_simulation/lib/librack_hybrid_actuator.so ]]; then
  echo '仿真执行插件尚未构建：请运行 bash scripts/build.sh（不要使用 --robot）'; exit 2
fi
source /opt/ros/jazzy/setup.bash
source "$FRAMEWORK/install/setup.bash"
export ROBOT_PROJECT_ROOT="$FRAMEWORK"
python3 "$FRAMEWORK/tools/generate_rack_targets.py"
exec bash "$FRAMEWORK/navigation/scripts/run.sh" --framework "$FRAMEWORK" "$@"
