#!/usr/bin/env python3
"""Publish a one-shot joint position test command to the real MIT controller.

Edit JOINT_ANGLE_DEG below, then run this script directly.

The ROS command contract is:
  /rl_motion_control_command Float64MultiArray[69] = [pos23][vel23][torque23]

This script sends position commands only. Velocity and feed-forward torque are
kept at zero.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import select
import sys
import time
from typing import Dict, Iterable, List, Mapping, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


CONTROLLER_JOINT_ORDER: List[str] = [
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_hip_pitch_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_hip_pitch_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
]
ETHERCAT_POSITION_BY_INDEX: List[int] = [
    1, 2, 3, 4, 5, 6,
    7, 8, 9, 10, 11, 12,
    14,
    15, 16, 17, 18, 19,
    20, 21, 22, 23, 24,
]
ETHERCAT_POSITION_TO_INDEX = {
    position: index for index, position in enumerate(ETHERCAT_POSITION_BY_INDEX)
}

# Modify target joint positions here, in degrees. Names must match
# CONTROLLER_JOINT_ORDER and mit_controller::kControllerJointOrder.
# The script ramps from live /joint_states to these targets.
JOINT_ANGLE_DEG: Dict[str, float] = {
    "left_hip_roll_joint": 0.0,
    "left_hip_yaw_joint": 0.0,
    "left_hip_pitch_joint": 0.0,
    "left_knee_joint": 0.0,
    "left_ankle_pitch_joint": 0.0,
    "left_ankle_roll_joint": 0.0,

    "right_hip_roll_joint": 0.0,
    "right_hip_yaw_joint": 0.0,
    "right_hip_pitch_joint": 0.0,
    "right_knee_joint": 0.0,
    "right_ankle_pitch_joint": 0.0,
    "right_ankle_roll_joint": 0.0,

    "waist_yaw_joint": 0.0,

    "left_shoulder_pitch_joint": 0.0,
    "left_shoulder_roll_joint": 0.0,
    "left_shoulder_yaw_joint": 0.0,
    "left_elbow_joint": 0.0,
    "left_wrist_roll_joint": 0.0,
    
    "right_shoulder_pitch_joint": 0.0,
    "right_shoulder_roll_joint": 0.0,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 0.0,
    "right_wrist_roll_joint": 0.0,
}

COMMAND_SIZE = len(CONTROLLER_JOINT_ORDER) * 3
REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
DEFAULT_LIMIT_FILE = REPO_ROOT / "common" / "joint_ctrl_config_v4.json"
DEFAULT_JOG_JOINT = "right_shoulder_pitch_joint"
DEFAULT_JOG_STEP_DEG = 3.0
RIGHT_ARM_INDICES = list(range(18, 23))


def smooth_step01(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def deg(rad: float) -> float:
    return math.degrees(rad)


def rad(deg_value: float) -> float:
    return math.radians(deg_value)


def strip_json_comments(text: str) -> str:
    """Remove C/C++ style comments while preserving string literals."""
    out: List[str] = []
    i = 0
    in_string = False
    escape = False
    while i < len(text):
        ch = text[i]
        nxt = text[i + 1] if i + 1 < len(text) else ""
        if in_string:
            out.append(ch)
            if escape:
                escape = False
            elif ch == "\\":
                escape = True
            elif ch == '"':
                in_string = False
            i += 1
            continue
        if ch == '"':
            in_string = True
            out.append(ch)
            i += 1
            continue
        if ch == "/" and nxt == "/":
            i += 2
            while i < len(text) and text[i] not in "\r\n":
                i += 1
            continue
        if ch == "/" and nxt == "*":
            i += 2
            while i + 1 < len(text) and not (text[i] == "*" and text[i + 1] == "/"):
                i += 1
            i += 2
            continue
        out.append(ch)
        i += 1
    return "".join(out)


def read_joint_angles_from_python_config() -> Dict[str, float]:
    if not isinstance(JOINT_ANGLE_DEG, Mapping):
        raise ValueError("JOINT_ANGLE_DEG must be a mapping")
    unknown = sorted(set(JOINT_ANGLE_DEG.keys()) - set(CONTROLLER_JOINT_ORDER))
    if unknown:
        raise ValueError("unknown joint names in JOINT_ANGLE_DEG: " + ", ".join(unknown))

    angles: Dict[str, float] = {}
    for name, value in JOINT_ANGLE_DEG.items():
        try:
            angle_deg = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"JOINT_ANGLE_DEG[{name!r}] is not numeric: {value!r}") from exc
        if not math.isfinite(angle_deg):
            raise ValueError(f"JOINT_ANGLE_DEG[{name!r}] is not finite: {value!r}")
        angles[name] = rad(angle_deg)
    return angles


def load_position_limits(path: pathlib.Path) -> Dict[str, tuple[float, float]]:
    with path.open("r", encoding="utf-8") as f:
        root = json.loads(strip_json_comments(f.read()))
    limits: Dict[str, tuple[float, float]] = {}
    for name in CONTROLLER_JOINT_ORDER:
        joint = root.get(name)
        if not isinstance(joint, Mapping):
            continue
        if "minPos" in joint and "maxPos" in joint:
            limits[name] = (float(joint["minPos"]), float(joint["maxPos"]))
    return limits


def validate_position_limits(target: List[float], limits: Mapping[str, tuple[float, float]]) -> None:
    for i, name in enumerate(CONTROLLER_JOINT_ORDER):
        if name not in limits:
            continue
        min_pos, max_pos = limits[name]
        if target[i] < min_pos or target[i] > max_pos:
            raise ValueError(
                f"{name} target {deg(target[i]):.3f} deg ({target[i]:.6f} rad) is outside "
                f"[{deg(min_pos):.3f}, {deg(max_pos):.3f}] deg"
            )


def resolve_joint_selector(selector: str) -> int:
    selector = selector.strip()
    selector_lower = selector.lower()
    if selector_lower.startswith("ec:"):
        ec_text = selector_lower[3:].strip()
        if not ec_text.isdigit():
            raise ValueError(f"invalid EtherCAT selector: {selector!r}")
        ec_position = int(ec_text)
        if ec_position in ETHERCAT_POSITION_TO_INDEX:
            return ETHERCAT_POSITION_TO_INDEX[ec_position]
        raise ValueError(f"EtherCAT position {ec_position} is not in the 23-joint controller map")
    if selector_lower.startswith("ec "):
        ec_text = selector_lower[3:].strip()
        if not ec_text.isdigit():
            raise ValueError(f"invalid EtherCAT selector: {selector!r}")
        ec_position = int(ec_text)
        if ec_position in ETHERCAT_POSITION_TO_INDEX:
            return ETHERCAT_POSITION_TO_INDEX[ec_position]
        raise ValueError(f"EtherCAT position {ec_position} is not in the 23-joint controller map")
    if selector.isdigit():
        idx = int(selector)
        if 0 <= idx < len(CONTROLLER_JOINT_ORDER):
            if idx in ETHERCAT_POSITION_TO_INDEX and idx != ETHERCAT_POSITION_BY_INDEX[idx]:
                ec_idx = ETHERCAT_POSITION_TO_INDEX[idx]
                print(
                    f"[Jog] note: numeric selector '{idx}' is command data index {idx} "
                    f"({CONTROLLER_JOINT_ORDER[idx]}). If you mean EtherCAT position {idx}, "
                    f"use 'ec {idx}' for {CONTROLLER_JOINT_ORDER[ec_idx]}."
                )
            return idx
        raise ValueError(f"joint index {idx} is outside [0, {len(CONTROLLER_JOINT_ORDER) - 1}]")
    if selector in CONTROLLER_JOINT_ORDER:
        return CONTROLLER_JOINT_ORDER.index(selector)
    matches = [i for i, name in enumerate(CONTROLLER_JOINT_ORDER) if selector in name]
    if len(matches) == 1:
        return matches[0]
    if len(matches) > 1:
        raise ValueError(
            "ambiguous joint selector, matches: "
            + ", ".join(f"{i}:{CONTROLLER_JOINT_ORDER[i]}" for i in matches)
        )
    raise ValueError(f"unknown joint selector: {selector!r}")


def print_joint_list(target: List[float], current: Mapping[str, float]) -> None:
    print("[Jog] joint list:")
    for i, name in enumerate(CONTROLLER_JOINT_ORDER):
        cur = current.get(name)
        cur_text = "nan" if cur is None else f"{deg(cur):.3f} deg"
        print(
            f"  cmd[{i:02d}] ec[{ETHERCAT_POSITION_BY_INDEX[i]:02d}] "
            f"{name:<28} target={deg(target[i]): .3f} deg feedback={cur_text}"
        )


def print_jog_help() -> None:
    print(
        "[Jog] commands: '+' jog positive, '-' jog negative, "
        "'joint <cmd_index|name>' or 'ec <position>' select joint, 'step <deg>' set step, "
        "'set <deg>' set selected target, 'order', 'list', 'show', 'help', 'q'"
    )


def print_selected_joint(selected_idx: int, target: List[float], current: Mapping[str, float], step_deg: float) -> None:
    name = CONTROLLER_JOINT_ORDER[selected_idx]
    cur = current.get(name)
    cur_text = "nan" if cur is None else f"{deg(cur):.3f} deg"
    print(
        f"[Jog] selected {selected_idx:02d}:{name}, "
        f"target={deg(target[selected_idx]):.3f} deg, feedback={cur_text}, step={step_deg:.3f} deg"
    )


def apply_jog_target(
    target: List[float],
    selected_idx: int,
    proposed: float,
    limits: Mapping[str, tuple[float, float]],
    enable_limit_check: bool,
) -> bool:
    name = CONTROLLER_JOINT_ORDER[selected_idx]
    if enable_limit_check and name in limits:
        min_pos, max_pos = limits[name]
        clamped = max(min_pos, min(max_pos, proposed))
        if abs(clamped - proposed) > 1.0e-12:
            print(
                f"[Jog] {name} request {deg(proposed):.3f} deg clipped to limit "
                f"[{deg(min_pos):.3f}, {deg(max_pos):.3f}] deg -> {deg(clamped):.3f} deg"
            )
        proposed = clamped
    if not math.isfinite(proposed):
        print("[Jog] ignored non-finite target")
        return False
    target[selected_idx] = proposed
    print(f"[Jog] {name} target={deg(target[selected_idx]):.3f} deg")
    return True


def print_right_arm_order() -> None:
    print("[Jog] right arm command order in /rl_motion_control_command pos23:")
    for idx in RIGHT_ARM_INDICES:
        print(
            f"  data[{idx:02d}] / ec[{ETHERCAT_POSITION_BY_INDEX[idx]:02d}] "
            f"= {CONTROLLER_JOINT_ORDER[idx]}"
        )


def print_feedback_diagnosis(
    selected_idx: int,
    baseline: Mapping[str, float],
    current: Mapping[str, float],
    threshold: float,
) -> None:
    selected_name = CONTROLLER_JOINT_ORDER[selected_idx]
    deltas = []
    for idx in RIGHT_ARM_INDICES:
        name = CONTROLLER_JOINT_ORDER[idx]
        if name in baseline and name in current:
            delta = current[name] - baseline[name]
            deltas.append((abs(delta), delta, idx, name))
    if not deltas:
        print("[JogDiag] no right-arm feedback available for diagnosis")
        return

    deltas.sort(reverse=True)
    print("[JogDiag] right-arm feedback delta after jog:")
    for abs_delta, delta, idx, name in deltas:
        print(f"  {idx:02d}:{name:<28} delta={deg(delta):+.3f} deg ({delta:+.6f} rad)")

    top_abs, _, top_idx, top_name = deltas[0]
    if selected_idx in RIGHT_ARM_INDICES and top_idx != selected_idx and top_abs >= threshold:
        print(
            "[JogDiag] possible right-arm order mismatch: selected "
            f"{selected_idx:02d}:{selected_name}, largest feedback change is "
            f"{top_idx:02d}:{top_name}"
        )
    elif selected_idx in RIGHT_ARM_INDICES and top_abs < threshold:
        print(
            f"[JogDiag] feedback change below threshold {deg(threshold):.3f} deg; "
            "increase --step or wait for the controller to settle."
        )


class JointCommandPublisher(Node):
    def __init__(self, command_topic: str, joint_state_topic: str) -> None:
        super().__init__("speedbot_joint_position_command_test")
        self.current_positions: Dict[str, float] = {}
        self.command_pub = self.create_publisher(Float64MultiArray, command_topic, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, joint_state_topic, self._joint_state_callback, 10
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        n = min(len(msg.name), len(msg.position))
        for i in range(n):
            value = float(msg.position[i])
            if math.isfinite(value):
                self.current_positions[msg.name[i]] = value

    def wait_for_subscriber(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.command_pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.command_pub.get_subscription_count() > 0

    def wait_for_joint_positions(self, required_names: Iterable[str], timeout_sec: float) -> bool:
        required = list(required_names)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if all(name in self.current_positions for name in required):
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return all(name in self.current_positions for name in required)

    def publish_command(self, positions: List[float]) -> None:
        msg = Float64MultiArray()
        zero = [0.0] * len(CONTROLLER_JOINT_ORDER)
        msg.data = list(positions) + zero + zero
        self.command_pub.publish(msg)


def make_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Publish a 69-value /rl_motion_control_command position test command."
    )
    parser.add_argument("--topic", default="/rl_motion_control_command", help="command topic")
    parser.add_argument("--joint-state-topic", default="/joint_states", help="joint state feedback topic")
    parser.add_argument("--rate", type=float, default=500.0, help="publish rate in Hz")
    parser.add_argument("--ramp", type=float, default=3.0, help="smooth ramp time in seconds")
    parser.add_argument(
        "--hold",
        type=float,
        default=10.0,
        help="hold time after ramp in seconds; use a negative value to hold until Ctrl+C",
    )
    parser.add_argument(
        "--joint-state-timeout",
        type=float,
        default=5.0,
        help="timeout waiting for /joint_states before starting",
    )
    parser.add_argument(
        "--subscriber-timeout",
        type=float,
        default=5.0,
        help="timeout waiting for a command subscriber",
    )
    parser.add_argument(
        "--require-all",
        action="store_true",
        help="require all 23 joints to be present in JOINT_ANGLE_DEG",
    )
    parser.add_argument(
        "--start-from-zero",
        action="store_true",
        help="do not use /joint_states as ramp start; only for bench/offline tests",
    )
    parser.add_argument(
        "--no-limit-check",
        action="store_true",
        help="skip minPos/maxPos validation from joint_ctrl_config_v4.json",
    )
    parser.add_argument(
        "--limit-file",
        type=pathlib.Path,
        default=DEFAULT_LIMIT_FILE,
        help="joint limit JSON file",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the configured command without publishing",
    )
    parser.add_argument(
        "--jog",
        action="store_true",
        help="interactive jog mode: start from /joint_states and use +/- to move one selected joint",
    )
    parser.add_argument(
        "--joint",
        default=DEFAULT_JOG_JOINT,
        help="initial selected jog joint, by name or index",
    )
    parser.add_argument(
        "--step",
        type=float,
        default=DEFAULT_JOG_STEP_DEG,
        help="interactive jog step in degrees",
    )
    parser.add_argument(
        "--diagnose",
        action="store_true",
        help="in jog mode, report which right-arm feedback joint changed most after each jog",
    )
    parser.add_argument(
        "--diagnose-window",
        type=float,
        default=0.6,
        help="seconds to wait before printing jog feedback diagnosis",
    )
    parser.add_argument(
        "--diagnose-threshold",
        type=float,
        default=0.3,
        help="minimum feedback delta in degrees for right-arm mismatch warning",
    )
    return parser


def run_jog_mode(
    node: JointCommandPublisher,
    args: argparse.Namespace,
    start: List[float],
    limits: Mapping[str, tuple[float, float]],
) -> int:
    try:
        selected_idx = resolve_joint_selector(args.joint)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    if args.step <= 0.0 or not math.isfinite(args.step):
        raise SystemExit("--step must be positive")

    target = list(start)
    step_deg = float(args.step)
    step_rad = rad(step_deg)
    period = 1.0 / args.rate
    next_time = time.monotonic()
    publish_count = 0
    enable_limit_check = not args.no_limit_check
    diagnose_baseline: Optional[Dict[str, float]] = None
    diagnose_selected_idx: Optional[int] = None
    diagnose_due_time = 0.0

    print("[Jog] interactive mode started. The command keeps publishing at %.1f Hz." % args.rate)
    print_jog_help()
    print_right_arm_order()
    print_selected_joint(selected_idx, target, node.current_positions, step_deg)
    print("[Jog] input> ", end="", flush=True)

    while rclpy.ok():
        node.publish_command(target)
        publish_count += 1
        rclpy.spin_once(node, timeout_sec=0.0)
        if (
            args.diagnose
            and diagnose_baseline is not None
            and diagnose_selected_idx is not None
            and time.monotonic() >= diagnose_due_time
        ):
            print_feedback_diagnosis(
                diagnose_selected_idx,
                diagnose_baseline,
                node.current_positions,
                rad(args.diagnose_threshold),
            )
            diagnose_baseline = None
            diagnose_selected_idx = None
            print("[Jog] input> ", end="", flush=True)

        while sys.stdin in select.select([sys.stdin], [], [], 0.0)[0]:
            line = sys.stdin.readline()
            if line == "":
                print("\n[Jog] stdin closed, exiting.")
                return 0
            command = line.strip()
            if not command:
                print("[Jog] input> ", end="", flush=True)
                continue
            parts = command.split()
            op = parts[0].lower()

            try:
                if op in ("q", "quit", "exit"):
                    print("[Jog] exiting.")
                    return 0
                if op in ("h", "help", "?"):
                    print_jog_help()
                elif op in ("list", "ls"):
                    print_joint_list(target, node.current_positions)
                elif op == "order":
                    print_right_arm_order()
                elif op == "show":
                    print_selected_joint(selected_idx, target, node.current_positions, step_deg)
                elif op in ("+", "="):
                    baseline = dict(node.current_positions)
                    updated = apply_jog_target(
                        target,
                        selected_idx,
                        target[selected_idx] + step_rad,
                        limits,
                        enable_limit_check,
                    )
                    if args.diagnose and updated:
                        diagnose_baseline = baseline
                        diagnose_selected_idx = selected_idx
                        diagnose_due_time = time.monotonic() + max(0.05, args.diagnose_window)
                elif op == "-":
                    baseline = dict(node.current_positions)
                    updated = apply_jog_target(
                        target,
                        selected_idx,
                        target[selected_idx] - step_rad,
                        limits,
                        enable_limit_check,
                    )
                    if args.diagnose and updated:
                        diagnose_baseline = baseline
                        diagnose_selected_idx = selected_idx
                        diagnose_due_time = time.monotonic() + max(0.05, args.diagnose_window)
                elif op in ("joint", "j"):
                    if len(parts) < 2:
                        print("[Jog] usage: joint <cmd_index|name|ec:position>")
                    else:
                        selected_idx = resolve_joint_selector(" ".join(parts[1:]))
                        print_selected_joint(selected_idx, target, node.current_positions, step_deg)
                elif op in ("ec", "ethercat"):
                    if len(parts) != 2:
                        print("[Jog] usage: ec <ethercat_position>")
                    else:
                        selected_idx = resolve_joint_selector("ec " + parts[1])
                        print_selected_joint(selected_idx, target, node.current_positions, step_deg)
                elif op == "step":
                    if len(parts) != 2:
                        print("[Jog] usage: step <deg>")
                    else:
                        new_step = float(parts[1])
                        if new_step <= 0.0 or not math.isfinite(new_step):
                            print("[Jog] step must be positive and finite")
                        else:
                            step_deg = new_step
                            step_rad = rad(step_deg)
                            print(f"[Jog] step={step_deg:.3f} deg")
                elif op == "set":
                    if len(parts) != 2:
                        print("[Jog] usage: set <deg>")
                    else:
                        baseline = dict(node.current_positions)
                        updated = apply_jog_target(
                            target,
                            selected_idx,
                            rad(float(parts[1])),
                            limits,
                            enable_limit_check,
                        )
                        if args.diagnose and updated:
                            diagnose_baseline = baseline
                            diagnose_selected_idx = selected_idx
                            diagnose_due_time = time.monotonic() + max(0.05, args.diagnose_window)
                else:
                    print(f"[Jog] unknown command: {command!r}")
                    print_jog_help()
            except ValueError as exc:
                print(f"[Jog] {exc}")
            print("[Jog] input> ", end="", flush=True)

        next_time += period
        sleep_time = next_time - time.monotonic()
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        else:
            next_time = time.monotonic()

    print(f"\n[Jog] stopped after publishing {publish_count} messages.")
    return 0


def main() -> int:
    args = make_arg_parser().parse_args()
    if args.rate <= 0.0 or not math.isfinite(args.rate):
        raise SystemExit("--rate must be positive")
    if args.ramp < 0.0 or not math.isfinite(args.ramp):
        raise SystemExit("--ramp must be finite and >= 0")
    if args.jog and (args.step <= 0.0 or not math.isfinite(args.step)):
        raise SystemExit("--step must be positive")

    overrides: Dict[str, float] = {}
    if not args.jog:
        try:
            overrides = read_joint_angles_from_python_config()
            if args.require_all:
                missing = [name for name in CONTROLLER_JOINT_ORDER if name not in overrides]
                if missing:
                    raise ValueError("JOINT_ANGLE_DEG is missing joints: " + ", ".join(missing))
        except Exception as exc:
            raise SystemExit(f"failed to read JOINT_ANGLE_DEG: {exc}") from exc

    if args.dry_run:
        if args.jog:
            try:
                selected_idx = resolve_joint_selector(args.joint)
            except ValueError as exc:
                raise SystemExit(str(exc)) from exc
            print(
                f"[Jog] dry-run: topic={args.topic}, size={COMMAND_SIZE}, "
                f"layout=[pos23][zero_vel23][zero_torque23]"
            )
            print(
                f"[Jog] rate={args.rate:.1f} Hz, selected="
                f"{selected_idx:02d}:{CONTROLLER_JOINT_ORDER[selected_idx]}, "
                f"step={args.step:.3f} deg"
            )
            return 0
        start = [0.0] * len(CONTROLLER_JOINT_ORDER)
        target = list(start)
        for name, angle in overrides.items():
            target[CONTROLLER_JOINT_ORDER.index(name)] = angle
        if not args.no_limit_check:
            try:
                limits = load_position_limits(args.limit_file)
                validate_position_limits(target, limits)
            except Exception as exc:
                raise SystemExit(f"joint limit check failed: {exc}") from exc
        max_delta = max(abs(t - s) for s, t in zip(start, target))
        print(
            f"[JointCmdTest] command: topic={args.topic}, size={COMMAND_SIZE}, "
            f"layout=[pos23][zero_vel23][zero_torque23]"
        )
        print(
            f"[JointCmdTest] rate={args.rate:.1f} Hz, ramp={args.ramp:.3f} s, "
            f"hold={'forever' if args.hold < 0.0 else f'{args.hold:.3f} s'}, "
            f"max_delta={deg(max_delta):.3f} deg ({max_delta:.6f} rad)"
        )
        for name, angle in overrides.items():
            print(f"  {name}: 0.000 deg -> {deg(angle):.3f} deg ({angle:.6f} rad)")
        print("[JointCmdTest] dry-run complete; not publishing.")
        return 0

    rclpy.init()
    node = JointCommandPublisher(args.topic, args.joint_state_topic)
    try:
        if not args.start_from_zero:
            print(f"[JointCmdTest] waiting for {args.joint_state_topic} with all 23 controller joints...")
            if not node.wait_for_joint_positions(CONTROLLER_JOINT_ORDER, args.joint_state_timeout):
                missing = [name for name in CONTROLLER_JOINT_ORDER if name not in node.current_positions]
                raise SystemExit(
                    "missing joint feedback before start: "
                    + ", ".join(missing)
                    + " (use --start-from-zero only for bench/offline tests)"
                )
            start = [node.current_positions[name] for name in CONTROLLER_JOINT_ORDER]
        else:
            start = [0.0] * len(CONTROLLER_JOINT_ORDER)

        limits: Dict[str, tuple[float, float]] = {}
        if not args.no_limit_check:
            try:
                limits = load_position_limits(args.limit_file)
            except Exception as exc:
                raise SystemExit(f"joint limit file load failed: {exc}") from exc

        print(f"[JointCmdTest] waiting for subscriber on {args.topic}...")
        if not node.wait_for_subscriber(args.subscriber_timeout):
            raise SystemExit(f"no subscriber on {args.topic}; is mit_controller running?")

        if args.jog:
            return run_jog_mode(node, args, start, limits)

        target = list(start)
        changed = []
        for name, angle in overrides.items():
            idx = CONTROLLER_JOINT_ORDER.index(name)
            target[idx] = angle
            changed.append((name, start[idx], angle))

        if not args.no_limit_check:
            try:
                validate_position_limits(target, limits)
            except Exception as exc:
                raise SystemExit(f"joint limit check failed: {exc}") from exc

        max_delta = max(abs(t - s) for s, t in zip(start, target))
        print(
            f"[JointCmdTest] command: topic={args.topic}, size={COMMAND_SIZE}, "
            f"layout=[pos23][zero_vel23][zero_torque23]"
        )
        print(
            f"[JointCmdTest] rate={args.rate:.1f} Hz, ramp={args.ramp:.3f} s, "
            f"hold={'forever' if args.hold < 0.0 else f'{args.hold:.3f} s'}, "
            f"max_delta={deg(max_delta):.3f} deg ({max_delta:.6f} rad)"
        )
        for name, start_value, target_value in changed:
            print(
                f"  {name}: {deg(start_value):.3f} deg -> {deg(target_value):.3f} deg "
                f"({start_value:.6f} rad -> {target_value:.6f} rad)"
            )

        period = 1.0 / args.rate
        start_time = time.monotonic()
        next_time = start_time
        publish_count = 0
        print("[JointCmdTest] publishing. Press Ctrl+C to stop.")
        while rclpy.ok():
            now = time.monotonic()
            elapsed = now - start_time
            if args.ramp <= 1.0e-9:
                alpha = 1.0
            else:
                alpha = smooth_step01(elapsed / args.ramp)
            command = [(1.0 - alpha) * s + alpha * t for s, t in zip(start, target)]
            node.publish_command(command)
            publish_count += 1

            if elapsed >= args.ramp and args.hold >= 0.0 and elapsed >= args.ramp + args.hold:
                break

            rclpy.spin_once(node, timeout_sec=0.0)
            next_time += period
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_time = time.monotonic()

        print(f"[JointCmdTest] stopped after publishing {publish_count} messages.")
        return 0
    except KeyboardInterrupt:
        print("\n[JointCmdTest] interrupted by user.")
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
