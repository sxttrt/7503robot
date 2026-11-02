#!/bin/bash
set -e
cd "$(dirname "$0")/.."
PROJECT="$(pwd -P)"
clean=false
build_args=()
for arg in "$@"; do
  case "$arg" in
    --clean) clean=true ;;
    --robot) build_args=(--packages-skip rack_simulation) ;;
    -h|--help) echo '用法：bash scripts/build.sh [--clean] [--robot]'; exit 0 ;;
    *) echo '用法：bash scripts/build.sh [--clean] [--robot]'; exit 2 ;;
  esac
done
if [[ ! -d src || ! -f .7503robot-project ]]; then
  echo '缺少项目源码或项目标记，请使用完整源码目录'; exit 2
fi
exec 8>"$PROJECT/.build.lock"
if ! flock -n 8; then echo '当前项目已有构建正在进行'; exit 2; fi
# Re-sourcing /opt/ros alone does not remove a previously sourced workspace.
# Build from the Jazzy underlay, not a copied install or an obsolete overlay.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH COLCON_CURRENT_PREFIX
unset PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH ROBOT_PROJECT_ROOT RACK_V3_ROOT
source /opt/ros/jazzy/setup.bash
# Colcon/CMake outputs contain machine-specific absolute paths. Preserve old
# generated outputs locally before rebuilding at a different project location.
if $clean || [[ ! -f .build-root || "$(cat .build-root)" != "$PROJECT" ]]; then
  if [[ -e build || -L build || -e install || -L install ]]; then
    if [[ -L .build_cache_backups ]]; then
      echo '.build_cache_backups 不能是符号链接，请先检查备份位置'; exit 2
    fi
    mkdir -p "$PROJECT/.build_cache_backups"
    touch "$PROJECT/.build_cache_backups/COLCON_IGNORE"
    backup="$(mktemp -d "$PROJECT/.build_cache_backups/build_XXXXXXXX")"
    for folder in build install; do
      if [[ -e "$PROJECT/$folder" || -L "$PROJECT/$folder" ]]; then
        mv -- "$PROJECT/$folder" "$backup/$folder"
      fi
    done
    echo "旧构建输出已保留到 $backup"
  fi
fi
# A failed rebuild must not leave the previous success marker usable by run.sh.
rm -f -- "$PROJECT/.build-root"
# Explicit source discovery also excludes integration/build backups.
colcon build --base-paths src --symlink-install "${build_args[@]}"
printf '%s\n' "$PROJECT" > .build-root
