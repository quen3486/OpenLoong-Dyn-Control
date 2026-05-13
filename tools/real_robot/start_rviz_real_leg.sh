#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

URDF_DEFAULT="${REPO_ROOT}/models/speedbot_v4/speedbot_v4_leg.urdf"
RVIZ_CFG_DEFAULT="${SCRIPT_DIR}/speedbot_v4_leg.rviz"

JOINT_TOPIC="${ROS_TOPIC_JOINT_STATES:-/joint_states}"
IMU_TOPIC="${ROS_TOPIC_IMU:-/imu/data}"
URDF_PATH="${ROBOT_URDF:-${URDF_DEFAULT}}"
RVIZ_CFG="${RVIZ_CONFIG:-${RVIZ_CFG_DEFAULT}}"

cleanup_old_robot_state_publishers() {
  local old_pids=()
  local proc
  for proc in /proc/[0-9]*; do
    [ -r "${proc}/cmdline" ] || continue
    local pid="${proc#/proc/}"
    local cmdline
    cmdline="$(tr '\0' ' ' < "${proc}/cmdline" 2>/dev/null || true)"
    if [[ "${cmdline}" == *"robot_state_publisher"* &&
          "${cmdline}" == *"/tmp/openloong_rviz_urdf."* &&
          "${cmdline}" == *"/joint_states:=${JOINT_TOPIC}"* ]]; then
      old_pids+=("${pid}")
      echo "[RViz] cleanup old robot_state_publisher: pid=${pid} cmd=${cmdline}"
    fi
  done

  if [ "${#old_pids[@]}" -eq 0 ]; then
    return
  fi

  local pid
  for pid in "${old_pids[@]}"; do
    kill "${pid}" >/dev/null 2>&1 || true
  done

  for _ in {1..20}; do
    local alive=0
    for pid in "${old_pids[@]}"; do
      if kill -0 "${pid}" >/dev/null 2>&1; then
        alive=1
      fi
    done
    [ "${alive}" -eq 0 ] && break
    sleep 0.1
  done

  for pid in "${old_pids[@]}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      echo "[RViz] force cleanup old robot_state_publisher: pid=${pid}"
      kill -KILL "${pid}" >/dev/null 2>&1 || true
    fi
  done

  rm -f /tmp/openloong_rviz_urdf.*.urdf >/dev/null 2>&1 || true
}

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

if [ ! -f "${URDF_PATH}" ]; then
  echo "[ERROR] URDF not found: ${URDF_PATH}"
  exit 1
fi

if [ ! -f "${RVIZ_CFG}" ]; then
  echo "[ERROR] RViz config not found: ${RVIZ_CFG}"
  exit 1
fi

cleanup_old_robot_state_publishers

URDF_DIR="$(cd -- "$(dirname -- "${URDF_PATH}")" && pwd)"
TMP_URDF="$(mktemp /tmp/openloong_rviz_urdf.XXXXXX.urdf)"
# 把 URDF 中 ./meshes/... 改成绝对 file:// 路径，避免 RViz 报 "Error retrieving file"
sed -E \
  -e "s#filename=\"\\./#filename=\"file://${URDF_DIR}/#g" \
  -e "s#filename=\"meshes/#filename=\"file://${URDF_DIR}/meshes/#g" \
  "${URDF_PATH}" > "${TMP_URDF}"

echo "[RViz] URDF       : ${URDF_PATH}"
echo "[RViz] joint topic: ${JOINT_TOPIC}"
echo "[RViz] imu topic  : ${IMU_TOPIC}"

ros2 run robot_state_publisher robot_state_publisher "${TMP_URDF}" \
  --ros-args -r /joint_states:="${JOINT_TOPIC}" &
RSP_PID=$!

cleanup() {
  rm -f "${TMP_URDF}" >/dev/null 2>&1 || true
  if kill -0 "${RSP_PID}" >/dev/null 2>&1; then
    kill "${RSP_PID}" >/dev/null 2>&1 || true
    wait "${RSP_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

ros2 run rviz2 rviz2 -d "${RVIZ_CFG}" --ros-args -r /imu/data:="${IMU_TOPIC}"
