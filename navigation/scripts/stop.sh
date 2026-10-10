#!/bin/bash
set -e
cd "$(dirname "$0")/.."
python3 scripts/process_control.py
