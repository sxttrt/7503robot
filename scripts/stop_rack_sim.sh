#!/bin/bash
set -e
PROJECT="$(cd "$(dirname "$0")/.." && pwd)"
exec bash "$PROJECT/navigation/scripts/stop.sh"
