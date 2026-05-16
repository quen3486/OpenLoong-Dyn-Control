#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${REPO_ROOT}/build"

BIN_NAME="walk_mpc_wbc_leg"
BIN_PATH="${BUILD_DIR}/${BIN_NAME}"
CFG_PATH="${REPO_ROOT}/common/controller_config_v4_leg.json"

if ! command -v ros2 >/dev/null 2>&1; then
  if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  fi
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERROR] ros2 command not found. Please install/source ROS2 humble first."
  exit 1
fi

if [ ! -x "${BIN_PATH}" ]; then
  echo "[INFO] binary not found, building first..."
  cmake -S "${REPO_ROOT}" -B "${BUILD_DIR}"
  cmake --build "${BUILD_DIR}" -j4
fi

if [ ! -f "${CFG_PATH}" ]; then
  echo "[ERROR] controller config not found: ${CFG_PATH}"
  exit 1
fi

echo "[Control] mode   : ros2_real"
echo "[Control] variant: leg"
echo "[Control] binary : ${BIN_PATH}"
echo "[Control] config : ${CFG_PATH}"
echo "[Control] real   : fixed leg control only; visualization is provided by the controller workspace."
echo "[Control] gate   : press G in this terminal to publish control commands"

cd "${BUILD_DIR}"

CONTROLLER_CONFIG="${CFG_PATH}" \
./"${BIN_NAME}" --ros2-real
