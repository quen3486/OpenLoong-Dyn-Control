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
REAL_JOINT_STATE_SAMPLE="${REPO_ROOT}/msg"
ROS_CONTROLLER_FILE="${ROS_CONTROLLER_FILE:-/home/huangkun/workspaces/ros2_ws/src/mit_controller/src/joint_group_mit_controller.cpp}"

pass_count=0
fail_count=0

check_pattern() {
  local file="$1"
  local pattern="$2"
  local desc="$3"
  if rg -n --no-heading -- "$pattern" "$file" >/tmp/ros2_check_tmp.txt 2>/dev/null; then
    local loc
    loc="$(head -n 1 /tmp/ros2_check_tmp.txt)"
    echo "[PASS] ${desc} (${loc})"
    pass_count=$((pass_count + 1))
  else
    echo "[FAIL] ${desc}"
    fail_count=$((fail_count + 1))
  fi
}

check_no_pattern() {
  local file="$1"
  local pattern="$2"
  local desc="$3"
  if rg -n --no-heading -- "$pattern" "$file" >/tmp/ros2_check_tmp.txt 2>/dev/null; then
    echo "[FAIL] ${desc} ($(head -n 1 /tmp/ros2_check_tmp.txt))"
    fail_count=$((fail_count + 1))
  else
    echo "[PASS] ${desc}"
    pass_count=$((pass_count + 1))
  fi
}

check_path_absent() {
  local path="$1"
  local desc="$2"
  if [[ -e "$path" ]]; then
    echo "[FAIL] ${desc}: still exists (${path})"
    fail_count=$((fail_count + 1))
  else
    echo "[PASS] ${desc}"
    pass_count=$((pass_count + 1))
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
    line="$(rg -n --no-heading -- "$pattern" "$file" | head -n 1 | cut -d: -f1 || true)"
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

check_closed_loop_pd_fields() {
  local desc="joint_ctrl_config_v4_leg.json 包含12个闭环PVT增益"
  if python3 - "$JOINT_CFG" <<'PY'
import json
import sys

joint_cfg_path = sys.argv[1]
names = [
    "left_hip_roll_joint", "left_hip_yaw_joint", "left_hip_pitch_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_roll_joint", "right_hip_yaw_joint", "right_hip_pitch_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
]
with open(joint_cfg_path, "r", encoding="utf-8") as f:
    cfg = json.load(f)
for name in names:
    joint = cfg.get(name)
    if not isinstance(joint, dict):
        raise SystemExit(f"missing config joint {name}")
    for key in ("closedLoopKp", "closedLoopKd"):
        if key not in joint or not isinstance(joint[key], (int, float)):
            raise SystemExit(f"{name} missing numeric {key}")
PY
  then
    echo "[PASS] ${desc}"
    pass_count=$((pass_count + 1))
  else
    echo "[FAIL] ${desc}"
    fail_count=$((fail_count + 1))
  fi
}

check_real_pvt_torque_scale() {
  local desc="真机PVT合力矩安全阈值比例在[0,1]"
  if python3 - "$CFG_FILE" <<'PY'
import json
import re
import sys

cfg_path = sys.argv[1]
with open(cfg_path, "r", encoding="utf-8") as f:
    text = f.read()
text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
text = re.sub(r"//.*", "", text)
cfg = json.loads(text)
value = cfg.get("real_pvt_torque_limit_scale")
if not isinstance(value, (int, float)):
    raise SystemExit("real_pvt_torque_limit_scale is missing or non-numeric")
if not 0.0 <= float(value) <= 1.0:
    raise SystemExit(f"real_pvt_torque_limit_scale out of range: {value}")
PY
  then
    echo "[PASS] ${desc}"
    pass_count=$((pass_count + 1))
  else
    echo "[FAIL] ${desc}"
    fail_count=$((fail_count + 1))
  fi
}

check_real_joint_state_sample() {
  local desc="msg样本joint_states顺序与项目配置一致"
  if python3 - "$REAL_JOINT_STATE_SAMPLE" <<'PY'
import re
import sys

sample_path = sys.argv[1]
expected_real_order = [
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_hip_pitch_joint",
    "left_ankle_roll_joint",
    "left_ankle_pitch_joint",
    "right_hip_yaw_joint",
    "left_knee_joint",
    "right_knee_joint",
    "right_hip_roll_joint",
    "right_ankle_pitch_joint",
    "right_hip_pitch_joint",
    "right_ankle_roll_joint",
]

def parse_list_block(text, key):
    marker = key + ":\n"
    start = text.find(marker)
    if start < 0:
        raise SystemExit(f"missing {key} block")
    items = []
    for line in text[start + len(marker):].splitlines():
        if not line.startswith("- "):
            if line and not line.startswith("  "):
                break
            continue
        items.append(line[2:].strip())
    return items

with open(sample_path, "r", encoding="utf-8") as f:
    sample = f.read()

names = parse_list_block(sample, "name")
positions = parse_list_block(sample, "position")
velocities = parse_list_block(sample, "velocity")
efforts = parse_list_block(sample, "effort")
if names[:12] != expected_real_order:
    raise SystemExit(f"unexpected joint_states order: {names[:12]}")
if "waist_yaw_joint" in names[:12]:
    raise SystemExit("waist_yaw_joint must not be inside the leg command slice")
if len(names) < 12:
    raise SystemExit(f"joint_states sample has only {len(names)} names")
for label, values in (("position", positions), ("velocity", velocities), ("effort", efforts)):
    if len(values) != len(names):
        raise SystemExit(f"{label} length {len(values)} != name length {len(names)}")
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
check_pattern "$CFG_FILE" '"ros_topic_action_cmd"\s*:\s*"/rl_motion_control_command"' "配置话题: /rl_motion_control_command"
check_no_pattern "$CFG_FILE" 'sim_enable_ros2_state_pub' "配置不再保留MuJoCo ROS2状态发布开关"
check_real_pvt_torque_scale
check_pattern "$CFG_FILE" '"contact_confirm_time_sec"\s*:\s*0\.02' "换相接触确认时间: 0.02s"
check_pattern "$CFG_FILE" '"contact_force_blend_time_sec"\s*:\s*0\.03' "新支撑腿前馈力矩渐入时间: 0.03s"

check_pattern "$START_SCRIPT" 'BIN_NAME="walk_mpc_wbc_leg"' "启动脚本固定leg真机demo"
check_pattern "$START_SCRIPT" '--ros2-real' "启动脚本通过显式参数进入ros2_real"
check_no_pattern "$START_SCRIPT" 'CONTROL_MODE|ROBOT_VARIANT|TARGET|START_RVIZ|SIM_ROS_PUBLISH_DT|mujoco_ros2|AUTOWALK|rviz|RVIZ|start_rviz|robot_state_publisher' "启动脚本不再启动RViz或仿真可视化"
check_path_absent "${REPO_ROOT}/tools/real_robot/start_rviz_real_leg.sh" "MPC项目不再保留RViz启动脚本"
check_path_absent "${REPO_ROOT}/tools/real_robot/speedbot_v4_leg.rviz" "MPC项目不再保留leg RViz配置"
check_path_absent "${REPO_ROOT}/tools/real_robot/speedbot_v4.rviz" "MPC项目不再保留v4 RViz配置"
check_path_absent "${REPO_ROOT}/sim_interface/ROS2_state_pub_v4_leg.cpp" "MPC项目不再保留leg仿真状态发布实现"
check_path_absent "${REPO_ROOT}/sim_interface/ROS2_state_pub_v4.cpp" "MPC项目不再保留v4仿真状态发布实现"

check_pattern "$ROS_IF" 'topicImu_ = config\.rosTopicImu' "ROS2接口读取IMU话题配置"
check_pattern "$ROS_IF" 'topicJointStates_ = config\.rosTopicJointStates' "ROS2接口读取joint_states话题配置"
check_pattern "$ROS_IF" 'topicActionCmd_ = config\.rosTopicActionCmd' "ROS2接口读取action_cmd话题配置"
check_pattern "$ROS_IF" 'kCommandJointCount = 23' "动作发布关节数为23"
check_pattern "$ROS_IF" 'kCommandSize == 69' "动作发布长度为69([pos23][vel23][torque23])"
check_pattern "$ROS_IF" 'msg\.data\.assign\(kCommandSize, 0\.0\)' "动作发布按69长度初始化"
check_pattern "$ROS_IF" 'msg\.data\[13 \+ i\] = qDesIn\[12 \+ i\]' "V4左臂位置跳过waist_yaw后写入动作位置段"
check_pattern "$ROS_IF" 'msg\.data\[18 \+ i\] = qDesIn\[17 \+ i\]' "V4右臂位置跳过waist_yaw后写入动作位置段"

if [[ -f "$ROS_CONTROLLER_FILE" ]]; then
  check_pattern "$ROS_CONTROLLER_FILE" 'kControlledJointCount = 23' "mit_controller订阅端固定23关节"
  check_pattern "$ROS_CONTROLLER_FILE" 'kMotionCommandSize = kControlledJointCount \* kCommandSections' "mit_controller订阅端固定69维命令规模"
  check_pattern "$ROS_CONTROLLER_FILE" '"/rl_motion_control_command"' "mit_controller订阅统一动作话题"
  check_pattern "$ROS_CONTROLLER_FILE" 'Float64MultiArray\[69\]=\[pos23\]\[vel23\]\[torque23\]' "mit_controller日志声明69维命令合同"
  check_pattern "$ROS_CONTROLLER_FILE" 'msg->data\[kControlledJointCount \+ i\]' "mit_controller读取期望速度段"
  check_pattern "$ROS_CONTROLLER_FILE" 'msg->data\[2 \* kControlledJointCount \+ i\]' "mit_controller读取前馈力矩段"
  check_pattern "$ROS_CONTROLLER_FILE" 'controller is not in RL_MOTION_CONTROL mode yet' "mit_controller未ready时拒绝MPC命令"
  check_pattern "$ROS_CONTROLLER_FILE" '"waist_yaw_joint"' "mit_controller合同包含腰关节"
  check_pattern "$ROS_CONTROLLER_FILE" '"left_shoulder_pitch_joint"' "mit_controller合同包含v4左臂关节"
  check_pattern "$ROS_CONTROLLER_FILE" '"right_wrist_roll_joint"' "mit_controller合同包含v4右臂末端关节"
else
  echo "[SKIP] 未找到mit_controller源码，跳过订阅端静态核对: ${ROS_CONTROLLER_FILE}"
fi

check_pattern "$ROS_IF" 'busIn\.motors_tor_cur\[i\] = effCopy' "输入力矩来自/joint_states effort"
check_pattern "$ROS_IF" 'busIn\.basePos\[0\] = 0\.0' "basePos不使用真值"
check_pattern "$ROS_IF" 'busIn\.baseLinVel\[0\] = 0\.0' "baseLinVel不使用真值"
check_pattern "$ROS_IF" 'busIn\.fL\[0\] = 0\.0' "fL不注入仿真触地真值"
check_pattern "$ROS_IF" 'busIn\.fR\[0\] = 0\.0' "fR不注入仿真触地真值"
check_pattern "$ROS_IF" 'nameToIndex\.find\(kJointNamesPinOrder\[j\]\)' "真机joint_states按关节名重排，不依赖反馈数组顺序"
if [[ -f "$REAL_JOINT_STATE_SAMPLE" ]]; then
  check_real_joint_state_sample
else
  echo "[SKIP] 未提供msg样本，跳过真机joint_states顺序样本核对"
fi

check_pattern "$DEMO_FILE" 'buttonState\.key_p' "真机终端P键发布门控"
check_pattern "$DEMO_FILE" 'bool publishEnabled = false' "ros2_real默认不发布控制消息"
check_pattern "$DEMO_FILE" 'stopPublishingForSafety' "安全超限后停发控制消息"
check_no_pattern "$DEMO_FILE" 'ROS2_StatePub|ROS2_state_pub|sim_enable_ros2_state_pub|simEnableRos2StatePub' "leg demo不再发布MuJoCo ROS2可视化状态"
check_pattern "$DEMO_FILE" 'kp \* \(qDes - qCur\) \+ param\.kd \* \(dqDes - dqCur\)' "PVT合力矩按PD(含期望速度)+前馈估算"
check_pattern "$DEMO_FILE" 'closedLoopKp' "真机安全估算读取闭环PVT增益"
check_pattern "$DEMO_FILE" 'contactConfirmTimeSec = controllerConfig\.contactConfirmTimeSec' "GaitScheduler接入接触确认配置"
check_pattern "$DEMO_FILE" 'contactForceBlendTimeSec' "换相后新支撑腿力矩渐入配置接入"
check_pattern "$DEMO_FILE" 'applyClosedLoopPD\(\)' "闭环PVT使用配置化增益"
check_no_pattern "$DEMO_FILE" 'setJointPD\(400 \* kp|setJointPD\(300 \* kp|setJointPD\(200 \* kp' "v4_leg闭环PVT不再使用硬编码setJointPD"

check_joint_limits_match
check_closed_loop_pd_fields

check_order_in_run_ros2_real "$DEMO_FILE" "ros2_real时序: dataBusWrite -> Estimation -> StateMachine -> MPC/WBC -> safety-gated setMotorsCommand" \
  'ros2Interface\.dataBusWrite\(RobotState\)' \
  'runEstimationAndDynamics\(RobotState, kinDynSolver, StateModule, ctrlTime\)' \
  'applyLegControlStateMachine\(' \
  'runControlPipeline\(' \
  'ros2Interface\.setMotorsCommand\(RobotState\.motors_pos_des,'

echo
if (( fail_count == 0 )); then
  echo "[OK] ros2_real leg contract check passed (${pass_count} checks)."
  exit 0
fi

echo "[ERROR] ros2_real leg contract check failed: ${fail_count} failed, ${pass_count} passed."
exit 1
