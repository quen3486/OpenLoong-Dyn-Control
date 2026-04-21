#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${REPO_ROOT}/build"

# 统一模式： mujoco / mujoco_ros2 / ros2_real
MODE="${CONTROL_MODE:-${OPENLOONG_CONTROL_MODE:-mujoco_ros2}}"
# 机器人版本切换：v4 / leg （兼容旧变量 TARGET）
ROBOT_VARIANT="${ROBOT_VARIANT:-${TARGET:-leg}}"
SIM_PUB_DT="${SIM_ROS_PUBLISH_DT:-${OPENLOONG_SIM_ROS_PUBLISH_DT:-0.01}}"

if [ -n "${START_RVIZ-}" ]; then
  START_RVIZ="${START_RVIZ}"
elif [ -n "${OPENLOONG_START_RVIZ-}" ]; then
  START_RVIZ="${OPENLOONG_START_RVIZ}"
elif [ "${MODE}" = "mujoco_ros2" ] || [ "${MODE}" = "ros2_real" ]; then
  START_RVIZ="1"
else
  START_RVIZ="0"
fi

AUTO_WALK="${AUTOWALK:-0}"

case "${ROBOT_VARIANT}" in
  v4)
    BIN_NAME="walk_mpc_wbc_v4"
    CFG_DEFAULT="${REPO_ROOT}/common/controller_config_v4.json"
    RVIZ_CFG_DEFAULT="${REPO_ROOT}/tools/real_robot/speedbot_v4.rviz"
    RVIZ_URDF_DEFAULT="${REPO_ROOT}/models/speedbot_v4/speedbot_v4.urdf"
    ;;
  leg)
    BIN_NAME="walk_mpc_wbc_leg"
    CFG_DEFAULT="${REPO_ROOT}/common/controller_config_v4_leg.json"
    RVIZ_CFG_DEFAULT="${REPO_ROOT}/tools/real_robot/speedbot_v4_leg.rviz"
    RVIZ_URDF_DEFAULT="${REPO_ROOT}/models/speedbot_v4/speedbot_v4_leg.urdf"
    ;;
  *)
    echo "[ERROR] invalid ROBOT_VARIANT=${ROBOT_VARIANT}, expected: v4|leg"
    exit 1
    ;;
esac

BIN_PATH="${BUILD_DIR}/${BIN_NAME}"
CFG_PATH="${OPENLOONG_CONTROLLER_CONFIG:-${CFG_DEFAULT}}"
RVIZ_CFG_PATH="${OPENLOONG_RVIZ_CONFIG:-${RVIZ_CFG_DEFAULT}}"
RVIZ_URDF_PATH="${OPENLOONG_ROBOT_URDF:-${RVIZ_URDF_DEFAULT}}"

# 目前 ros2_real 仅支持 leg；mujoco_ros2 支持 v4/leg
if [ "${MODE}" = "ros2_real" ] && [ "${ROBOT_VARIANT}" != "leg" ]; then
  echo "[ERROR] MODE=ros2_real currently supports only ROBOT_VARIANT=leg."
  echo "        Use: ROBOT_VARIANT=leg CONTROL_MODE=ros2_real ./tools/start_control_v4_leg.sh"
  exit 1
fi

need_ros2=0
if [ "${MODE}" = "mujoco_ros2" ] || [ "${MODE}" = "ros2_real" ] || [ "${START_RVIZ}" = "1" ]; then
  need_ros2=1
fi

if [ "${need_ros2}" = "1" ]; then
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

case "${MODE}" in
  mujoco|mujoco_ros2|ros2_real)
    ;;
  *)
    echo "[ERROR] invalid CONTROL_MODE=${MODE}, expected: mujoco|mujoco_ros2|ros2_real"
    exit 1
    ;;
esac

echo "[Control] mode   : ${MODE}"
echo "[Control] variant: ${ROBOT_VARIANT}"
echo "[Control] binary : ${BIN_PATH}"
echo "[Control] config : ${CFG_PATH}"

RVIZ_SCRIPT="${REPO_ROOT}/tools/real_robot/start_rviz_real_leg.sh"
RVIZ_PID=""
cleanup() {
  if [ -n "${RVIZ_PID}" ] && kill -0 "${RVIZ_PID}" >/dev/null 2>&1; then
    kill "${RVIZ_PID}" >/dev/null 2>&1 || true
    wait "${RVIZ_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [ "${START_RVIZ}" = "1" ]; then
  if [ ! -x "${RVIZ_SCRIPT}" ]; then
    echo "[ERROR] RViz script not found: ${RVIZ_SCRIPT}"
    exit 1
  fi

  echo "[Control] rviz   : ${RVIZ_CFG_PATH}"
  echo "[Control] urdf   : ${RVIZ_URDF_PATH}"

  OPENLOONG_RVIZ_CONFIG="${RVIZ_CFG_PATH}" \
  OPENLOONG_ROBOT_URDF="${RVIZ_URDF_PATH}" \
  "${RVIZ_SCRIPT}" &
  RVIZ_PID=$!
fi

cd "${BUILD_DIR}"

if [ "${MODE}" = "mujoco" ]; then
  OPENLOONG_CONTROLLER_CONFIG="${CFG_PATH}" \
  AUTOWALK="${AUTO_WALK}" \
  ./"${BIN_NAME}"
elif [ "${MODE}" = "ros2_real" ]; then
  OPENLOONG_CONTROLLER_CONFIG="${CFG_PATH}" \
  CONTROL_MODE="${MODE}" \
  AUTOWALK="${AUTO_WALK}" \
  ./"${BIN_NAME}"
elif [ "${MODE}" = "mujoco_ros2" ]; then
  OPENLOONG_CONTROLLER_CONFIG="${CFG_PATH}" \
  CONTROL_MODE="${MODE}" \
  SIM_ROS_PUBLISH_DT="${SIM_PUB_DT}" \
  AUTOWALK="${AUTO_WALK}" \
  ./"${BIN_NAME}"
else
  OPENLOONG_CONTROLLER_CONFIG="${CFG_PATH}" \
  CONTROL_MODE="${MODE}" \
  AUTOWALK="${AUTO_WALK}" \
  ./"${BIN_NAME}"
fi
