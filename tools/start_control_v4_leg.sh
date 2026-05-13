#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${REPO_ROOT}/build"

BIN_NAME="walk_mpc_wbc_leg"
BIN_PATH="${BUILD_DIR}/${BIN_NAME}"
CFG_PATH="${REPO_ROOT}/common/controller_config_v4_leg.json"
RVIZ_SCRIPT="${REPO_ROOT}/tools/real_robot/start_rviz_real_leg.sh"
RVIZ_CFG_PATH="${REPO_ROOT}/tools/real_robot/speedbot_v4_leg.rviz"
RVIZ_URDF_PATH="${REPO_ROOT}/models/speedbot_v4/speedbot_v4_leg.urdf"

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

if [ ! -x "${RVIZ_SCRIPT}" ]; then
  echo "[ERROR] RViz script not found: ${RVIZ_SCRIPT}"
  exit 1
fi

echo "[Control] mode   : ros2_real"
echo "[Control] variant: leg"
echo "[Control] binary : ${BIN_PATH}"
echo "[Control] config : ${CFG_PATH}"
echo "[Control] rviz   : ${RVIZ_CFG_PATH}"
echo "[Control] urdf   : ${RVIZ_URDF_PATH}"
echo "[Control] real   : fixed leg + RViz; press G in the controller terminal to publish"

RVIZ_PID=""
cleanup() {
  if [ -n "${RVIZ_PID}" ] && kill -0 "${RVIZ_PID}" >/dev/null 2>&1; then
    kill "${RVIZ_PID}" >/dev/null 2>&1 || true
    wait "${RVIZ_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

RVIZ_CONFIG="${RVIZ_CFG_PATH}" \
ROBOT_URDF="${RVIZ_URDF_PATH}" \
"${RVIZ_SCRIPT}" &
RVIZ_PID=$!

cd "${BUILD_DIR}"

CONTROLLER_CONFIG="${CFG_PATH}" \
./"${BIN_NAME}" --ros2-real
