#!/bin/bash
set -e
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export GZ_SIM_RESOURCE_PATH="$PWD/worlds:$PWD/materials:${GZ_SIM_RESOURCE_PATH:-}"
RUN="$PWD/log/run_$(date +%m%d_%H%M%S)_$$"
mkdir -p "$RUN"
export ROS_LOG_DIR="$RUN/ros"
exec 9>"$PWD/log/start.lock"
if ! flock -n 9; then echo "另一个启动脚本正在运行"; exit 2; fi
# Refuse to replace a live managed run. Stop it explicitly with scripts/stop.sh.
setsid nohup python3 scripts/stack_manager.py --run-dir "$RUN" "$@" \
  > "$RUN/run.log" 2>&1 < /dev/null 9>&- &
SUPERVISOR=$!
previous=0
while kill -0 "$SUPERVISOR" 2>/dev/null; do
  total=$(wc -l < "$RUN/run.log")
  if (( total > previous )); then sed -n "$((previous+1)),${total}p" "$RUN/run.log"; previous=$total; fi
  if grep -q '^READY$' "$RUN/status" 2>/dev/null; then
    echo "日志: $RUN"
    if [[ " $* " == *" --framework "* ]]; then
      echo '框架自动推进；识别阶段等待 /team2/sim/qr/detections，不再使用 /rack/next。'
      echo '联调时在框架根目录执行 python3 scripts/publish_qr_result.py START（对应 WAIT_START）。'
    else
      echo '历史独立入口：另一个终端 source /opt/ros/jazzy/setup.bash，再发布:'
      echo 'ros2 topic pub --once /rack/next std_msgs/msg/Bool "{data: true}"'
    fi
    echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION ROS_AUTOMATIC_DISCOVERY_RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE"
    exit 0
  fi
  sleep 1
done
cat "$RUN/run.log"
echo "启动失败；查看上述原因和 $RUN"
exit 1
