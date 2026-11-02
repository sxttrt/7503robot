#!/bin/bash
# Export exactly the committed project; never zip a built working directory.
set -e
cd "$(dirname "$0")/.."
PROJECT="$(pwd -P)"
if [[ $# -gt 1 ]]; then echo '用法：bash scripts/export_source.sh [输出.zip]'; exit 2; fi
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo '用法：bash scripts/export_source.sh [输出.zip]（仅打包已提交源码）'; exit 0
fi
if [[ "$(git rev-parse --show-toplevel 2>/dev/null)" != "$PROJECT" ]]; then
  echo '请在 git clone 得到的本项目里导出；下载的源码 ZIP 无需再次导出'; exit 2
fi
if [[ -n "$(git status --porcelain --untracked-files=normal)" ]]; then
  echo '有未提交的修改或新文件，请先检查并提交，避免源码包遗漏改动'; exit 2
fi
if [[ -n "$(git ls-files -- .build-root .build.lock build install log runtime_logs \
    .pytest_cache .build_cache_backups integration_backups navigation/log '*.pyc')" ]]; then
  echo '发现被 Git 跟踪的构建/运行产物，拒绝生成源码包'; exit 2
fi
output="${1:-$PROJECT/../7503robot-source-$(git rev-parse --short HEAD).zip}"
if [[ -e "$output" || -L "$output" ]]; then
  echo "输出文件已存在，请另选名称：$output"; exit 2
fi
git archive --format=zip --prefix=7503robot/ --output="$output" HEAD
echo "源码包已生成：$output"
echo '解压到任意目录，在 Ubuntu/ROS Jazzy 下执行 bash scripts/build.sh。'
