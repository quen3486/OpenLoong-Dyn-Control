#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

CFG_FILE="${REPO_ROOT}/common/controller_config_v4_leg.json"
ROS_IF="${REPO_ROOT}/sim_interface/ROS2_interface_v4_leg.cpp"
DEMO_FILE="${REPO_ROOT}/demo/walk_mpc_wbc_leg.cpp"

pass_count=0
fail_count=0

check_pattern() {
  local file="$1"
  local pattern="$2"
  local desc="$3"
  if rg -n --no-heading "$pattern" "$file" >/tmp/ros2_check_tmp.txt 2>/dev/null; then
    local loc
    loc="$(head -n 1 /tmp/ros2_check_tmp.txt)"
    echo "[PASS] ${desc} (${loc})"
    pass_count=$((pass_count + 1))
  else
    echo "[FAIL] ${desc}"
    fail_count=$((fail_count + 1))
  fi
}

check_order_in_file() {
  local file="$1"
  local desc="$2"
  shift 2

  local last_line=0
  local ok=1
  local i=1
  while [[ $# -gt 0 ]]; do
    local pattern="$1"
    local line
    line="$(rg -n --no-heading "$pattern" "$file" | head -n 1 | cut -d: -f1 || true)"
    if [[ -z "$line" ]]; then
      ok=0
      break
    fi
    if (( line <= last_line )); then
      ok=0
      break
    fi
    last_line=$line
    i=$((i + 1))
    shift
  done

  if (( ok == 1 )); then
    echo "[PASS] ${desc}"
    pass_count=$((pass_count + 1))
  else
    echo "[FAIL] ${desc}"
    fail_count=$((fail_count + 1))
  fi
}

check_order_in_run_ros2_real() {
  local file="$1"
  local desc="$2"
  local p1="$3"
  local p2="$4"
  local p3="$5"
  local p4="$6"
  local p5="$7"

  local result
  result="$(awk \
    -v p1="$p1" \
    -v p2="$p2" \
    -v p3="$p3" \
    -v p4="$p4" \
    -v p5="$p5" \
    '
      {
        if (!in_fn && $0 ~ /int[[:space:]]+runRos2Real[[:space:]]*\(/) {
          in_fn = 1
        }
        if (!in_fn) {
          next
        }
        if ($0 ~ /^#else/) {
          in_fn = 0
          next
        }
        if (l1 == 0 && $0 ~ p1) l1 = NR
        if (l2 == 0 && $0 ~ p2) l2 = NR
        if (l3 == 0 && $0 ~ p3) l3 = NR
        if (l4 == 0 && $0 ~ p4) l4 = NR
        if (l5 == 0 && $0 ~ p5) l5 = NR
      }
      END {
        if (l1 > 0 && l1 < l2 && l2 < l3 && l3 < l4 && l4 < l5) {
          print "PASS"
        } else {
          print "FAIL"
        }
      }
    ' "$file")"
  if [[ "$result" == "PASS" ]]; then
    echo "[PASS] ${desc}"
    pass_count=$((pass_count + 1))
  else
    echo "[FAIL] ${desc}"
    fail_count=$((fail_count + 1))
  fi
}

check_pattern "$CFG_FILE" '"ros_topic_imu"\s*:\s*"/imu/data"' "配置话题: /imu/data"
check_pattern "$CFG_FILE" '"ros_topic_joint_states"\s*:\s*"/joint_states"' "配置话题: /joint_states"
check_pattern "$CFG_FILE" '"ros_topic_action_cmd"\s*:\s*"/rl_motion_control_command"' "配置话题: /rl_motion_control_command"

check_pattern "$ROS_IF" 'topicImu_ = config\.rosTopicImu' "ROS2接口读取IMU话题配置"
check_pattern "$ROS_IF" 'topicJointStates_ = config\.rosTopicJointStates' "ROS2接口读取joint_states话题配置"
check_pattern "$ROS_IF" 'topicActionCmd_ = config\.rosTopicActionCmd' "ROS2接口读取action_cmd话题配置"
check_pattern "$ROS_IF" 'msg\.data\.assign\(29, 0\.0\)' "动作发布长度为29(12\+17)"

check_pattern "$ROS_IF" 'busIn\.motors_tor_cur\[i\] = effCopy' "输入力矩来自/joint_states effort"
check_pattern "$ROS_IF" 'busIn\.basePos\[0\] = 0\.0' "basePos不使用真值"
check_pattern "$ROS_IF" 'busIn\.baseLinVel\[0\] = 0\.0' "baseLinVel不使用真值"
check_pattern "$ROS_IF" 'busIn\.fL\[0\] = 0\.0' "fL不注入仿真触地真值"
check_pattern "$ROS_IF" 'busIn\.fR\[0\] = 0\.0' "fR不注入仿真触地真值"

check_order_in_run_ros2_real "$DEMO_FILE" "ros2_real时序: dataBusWrite -> Estimation -> StateMachine -> MPC/WBC -> setMotorsPosition" \
  'ros2Interface\.dataBusWrite\(RobotState\)' \
  'runEstimationAndDynamics\(RobotState, kinDynSolver, StateModule, ctrlTime\)' \
  'applyLegControlStateMachine\(' \
  'runControlPipeline\(' \
  'ros2Interface\.setMotorsPosition\(RobotState\.motors_pos_des\)'

echo
if (( fail_count == 0 )); then
  echo "[OK] ros2_real leg contract check passed (${pass_count} checks)."
  exit 0
fi

echo "[ERROR] ros2_real leg contract check failed: ${fail_count} failed, ${pass_count} passed."
exit 1
