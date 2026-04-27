#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

CFG_FILE="${REPO_ROOT}/common/controller_config_v4_leg.json"
JOINT_CFG="${REPO_ROOT}/common/joint_ctrl_config_v4_leg.json"
URDF_FILE="${REPO_ROOT}/models/speedbot_v4/speedbot_v4_leg.urdf"
ROS_IF="${REPO_ROOT}/sim_interface/ROS2_interface_v4_leg.cpp"
DEMO_FILE="${REPO_ROOT}/demo/walk_mpc_wbc_leg.cpp"
START_SCRIPT="${REPO_ROOT}/tools/start_control_v4_leg.sh"

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

check_joint_limits_match() {
  local desc="joint_ctrl_config_v4_leg.json 与 URDF limit 一致"
  if python3 - "$JOINT_CFG" "$URDF_FILE" <<'PY'
import json
import math
import sys
import xml.etree.ElementTree as ET

joint_cfg_path, urdf_path = sys.argv[1:3]
names = [
    "left_hip_roll_joint", "left_hip_yaw_joint", "left_hip_pitch_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_roll_joint", "right_hip_yaw_joint", "right_hip_pitch_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
]
with open(joint_cfg_path, "r", encoding="utf-8") as f:
    cfg = json.load(f)
root = ET.parse(urdf_path).getroot()
urdf = {}
for joint in root.findall("joint"):
    limit = joint.find("limit")
    if limit is not None:
        urdf[joint.attrib["name"]] = {
            "minPos": float(limit.attrib["lower"]),
            "maxPos": float(limit.attrib["upper"]),
            "maxSpeed": float(limit.attrib["velocity"]),
            "maxTorque": float(limit.attrib["effort"]),
        }
for name in names:
    if name not in cfg:
        raise SystemExit(f"missing config joint {name}")
    if name not in urdf:
        raise SystemExit(f"missing URDF joint {name}")
    for key in ("minPos", "maxPos", "maxSpeed", "maxTorque"):
        if not math.isclose(float(cfg[name][key]), urdf[name][key], rel_tol=0.0, abs_tol=1e-9):
            raise SystemExit(f"{name}.{key}: json={cfg[name][key]} urdf={urdf[name][key]}")
PY
  then
    echo "[PASS] ${desc}"
    pass_count=$((pass_count + 1))
  else
    echo "[FAIL] ${desc}"
    fail_count=$((fail_count + 1))
  fi
}

check_pattern "$CFG_FILE" '"ros_topic_imu"\s*:\s*"/imu/data"' "配置话题: /imu/data"
check_pattern "$CFG_FILE" '"ros_topic_joint_states"\s*:\s*"/joint_states"' "配置话题: /joint_states"
check_pattern "$CFG_FILE" '"ros_topic_action_cmd"\s*:\s*"/rl_motion_control_command_with_torque"' "配置话题: /rl_motion_control_command_with_torque"
check_pattern "$CFG_FILE" '"real_pvt_torque_limit_scale"\s*:\s*0\.5' "真机PVT合力矩安全阈值比例: 0.5"
check_pattern "$CFG_FILE" '"contact_confirm_time_sec"\s*:\s*0\.02' "换相接触确认时间: 0.02s"
check_pattern "$CFG_FILE" '"contact_force_blend_time_sec"\s*:\s*0\.03' "新支撑腿前馈力矩渐入时间: 0.03s"

check_pattern "$START_SCRIPT" 'MODE="\$\{CONTROL_MODE:-\$\{OPENLOONG_CONTROL_MODE:-ros2_real\}\}"' "启动脚本默认ros2_real"
check_pattern "$START_SCRIPT" 'ROBOT_VARIANT="leg"' "真机启动固定leg"
check_pattern "$START_SCRIPT" 'START_RVIZ="1"' "真机启动固定开启RViz"
check_pattern "$START_SCRIPT" 'AUTO_WALK="0"' "真机启动固定禁用AUTOWALK"

check_pattern "$ROS_IF" 'topicImu_ = config\.rosTopicImu' "ROS2接口读取IMU话题配置"
check_pattern "$ROS_IF" 'topicJointStates_ = config\.rosTopicJointStates' "ROS2接口读取joint_states话题配置"
check_pattern "$ROS_IF" 'topicActionCmd_ = config\.rosTopicActionCmd' "ROS2接口读取action_cmd话题配置"
check_pattern "$ROS_IF" 'msg\.data\.assign\(58, 0\.0\)' "动作发布长度为58(29位置\+29力矩前馈)"
check_pattern "$ROS_IF" 'msg\.data\[29 \+ i\] = tauFfIn\[i\]' "腿部力矩前馈写入动作后半段"

check_pattern "$ROS_IF" 'busIn\.motors_tor_cur\[i\] = effCopy' "输入力矩来自/joint_states effort"
check_pattern "$ROS_IF" 'busIn\.basePos\[0\] = 0\.0' "basePos不使用真值"
check_pattern "$ROS_IF" 'busIn\.baseLinVel\[0\] = 0\.0' "baseLinVel不使用真值"
check_pattern "$ROS_IF" 'busIn\.fL\[0\] = 0\.0' "fL不注入仿真触地真值"
check_pattern "$ROS_IF" 'busIn\.fR\[0\] = 0\.0' "fR不注入仿真触地真值"

check_pattern "$DEMO_FILE" 'buttonState\.key_g' "真机终端G键发布门控"
check_pattern "$DEMO_FILE" 'bool publishEnabled = false' "ros2_real默认不发布控制消息"
check_pattern "$DEMO_FILE" 'stopPublishingForSafety' "安全超限后停发控制消息"
check_pattern "$DEMO_FILE" 'kp \* \(qDes - qCur\) \+ param\.kd \* \(0\.0 - dqCur\)' "PVT合力矩按PD+前馈估算"
check_pattern "$DEMO_FILE" 'contactConfirmTimeSec = controllerConfig\.contactConfirmTimeSec' "GaitScheduler接入接触确认配置"
check_pattern "$DEMO_FILE" 'contactForceBlendTimeSec' "换相后新支撑腿力矩渐入配置接入"

check_joint_limits_match

check_order_in_run_ros2_real "$DEMO_FILE" "ros2_real时序: dataBusWrite -> Estimation -> StateMachine -> MPC/WBC -> safety-gated setMotorsCommand" \
  'ros2Interface\.dataBusWrite\(RobotState\)' \
  'runEstimationAndDynamics\(RobotState, kinDynSolver, StateModule, ctrlTime\)' \
  'applyLegControlStateMachine\(' \
  'runControlPipeline\(' \
  'ros2Interface\.setMotorsCommand\(RobotState\.motors_pos_des, RobotState\.motors_tor_des\)'

echo
if (( fail_count == 0 )); then
  echo "[OK] ros2_real leg contract check passed (${pass_count} checks)."
  exit 0
fi

echo "[ERROR] ros2_real leg contract check failed: ${fail_count} failed, ${pass_count} passed."
exit 1
