#!/usr/bin/env bash
set -euo pipefail

CLI_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="/home/xiaohai/miniconda3/envs/DummyRobot/bin/python"
CLI_PATH="${DUMMYROBOT_CLI_PATH:-usb}"

if [[ ! -x "$PYTHON" ]]; then
    echo "找不到 DummyRobot Conda 环境解释器：$PYTHON" >&2
    echo "请先创建并安装 DummyRobot 环境。" >&2
    exit 1
fi

exec "$PYTHON" "$CLI_DIR/run_shell.py" --path "$CLI_PATH" "$@"
