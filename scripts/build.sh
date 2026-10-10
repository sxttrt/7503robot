#!/bin/bash
set -e
cd "$(dirname "$0")/.."
PROJECT="$PWD"
source /opt/ros/jazzy/setup.bash
# Colcon/CMake outputs contain machine-specific absolute paths. Preserve old
# generated outputs locally before rebuilding at a different project location.
if [[ ! -f .build-root || "$(cat .build-root)" != "$PROJECT" ]]; then
  if [[ -d build || -d install ]]; then
    backup="$PROJECT/.build_cache_backups/$(date +%Y%m%d_%H%M%S)_$$"
    mkdir -p "$backup"
    for folder in build install; do
      if [[ -e "$PROJECT/$folder" ]]; then mv -- "$PROJECT/$folder" "$backup/$folder"; fi
    done
    echo "旧构建输出已保留到 $backup"
  fi
fi
unset ROBOT_PROJECT_ROOT RACK_V3_ROOT
if [[ "${1:-}" == "--robot" ]]; then
  colcon build --symlink-install --packages-skip rack_simulation
elif [[ $# -eq 0 ]]; then
  colcon build --symlink-install
else
  echo '用法：bash scripts/build.sh [--robot]'; exit 2
fi
printf '%s\n' "$PROJECT" > .build-root
