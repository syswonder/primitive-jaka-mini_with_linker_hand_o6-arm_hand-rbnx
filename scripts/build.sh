#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
ROS_SETUP="${RBNX_ROS_SETUP:-/opt/ros/humble/setup.bash}"

# The proprietary jkrc SDK binaries are NOT distributed with this repo.
# Fail fast with placement instructions before any build work.
case "$(uname -m)" in
    aarch64) _SDK_ARCH_DIR=aarch64-linux-gnu ;;
    x86_64) _SDK_ARCH_DIR=x86_64-linux-gnu ;;
    *) _SDK_ARCH_DIR="" ;;
esac
if [[ -z "$_SDK_ARCH_DIR" || ! -f "$PKG/jaka_arm/jaka_sdk/$_SDK_ARCH_DIR/jkrc.so" ]]; then
    echo "[jaka_arm/build] ERROR: JAKA SDK binaries missing for this platform." >&2
    echo "  Download the Linux SDK (jkrc.so + libjakaAPI.so) from the JAKA" >&2
    echo "  official download page and copy both files into:" >&2
    echo "    jaka_arm/jaka_sdk/${_SDK_ARCH_DIR:-<aarch64-linux-gnu|x86_64-linux-gnu>}/" >&2
    exit 1
fi

# Package-local venv with system site-packages so rclpy from the system ROS 2
# installation is visible. robonix_api itself is loaded from the Robonix
# source tree at start time via `rbnx path robonix-api`.
VENV="$PKG/.venv"
if [[ ! -x "$VENV/bin/python" ]]; then
  python3 -m venv --system-site-packages "$VENV"
fi
"$VENV/bin/pip" install -q -r "$PKG/requirements.txt"

# Generate gRPC/MCP/ROS 2 code with the package venv interpreter, then build
# the ROS 2 message overlay that start.sh sources.
# ROS setup.bash references unset variables under `set -u`; relax it around
# the source calls.
set +u
source "$ROS_SETUP"
set -u
rbnx codegen -p "$PKG" --mcp --ros2 --python "$VENV/bin/python"
cd "$PKG/rbnx-build/codegen/ros2_idl"
colcon build

echo "[jaka_arm] build done"
