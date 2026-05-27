#!/usr/bin/env python3
"""Analyze and plot the latest right-arm welding tracking datalog."""

from __future__ import annotations

import argparse
import csv
import math
import re
import shutil
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np


JOINT_NAMES = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
]

MOTION_STATE_WELD = 4


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def parse_matlab_mapping(path: Path) -> Dict[str, Tuple[int, int]]:
    range_pattern = re.compile(
        r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*dataRec\(:,\s*(\d+)\s*:\s*(\d+)\s*\);"
    )
    single_pattern = re.compile(
        r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*dataRec\(:,\s*(\d+)\s*\);"
    )
    mapping: Dict[str, Tuple[int, int]] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = range_pattern.match(line)
        if match:
            name = match.group(1)
            start = int(match.group(2)) - 1
            end = int(match.group(3))
            mapping[name] = (start, end)
            continue
        match = single_pattern.match(line)
        if match:
            name = match.group(1)
            start = int(match.group(2)) - 1
            mapping[name] = (start, start + 1)
    return mapping


def column(data: np.ndarray, mapping: Dict[str, Tuple[int, int]], name: str) -> np.ndarray:
    if name not in mapping:
        raise KeyError(f"missing datalog field: {name}")
    start, end = mapping[name]
    return data[:, start:end]


def optional_column(
    data: np.ndarray,
    mapping: Dict[str, Tuple[int, int]],
    name: str,
    fill_value: float = math.nan,
) -> np.ndarray:
    if name not in mapping:
        return np.full(data.shape[0], fill_value, dtype=float)
    return column(data, mapping, name).reshape(-1)


def load_data(log_path: Path, script_path: Path):
    mapping = parse_matlab_mapping(script_path)
    first_line = ""
    with log_path.open("r", encoding="utf-8", errors="replace") as log_file:
        for line in log_file:
            if line.strip():
                first_line = line
                break
    delimiter = "," if "," in first_line else None
    data = np.loadtxt(log_path, delimiter=delimiter)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return data, mapping


def compute_stats(err_rad: np.ndarray) -> Dict[str, np.ndarray]:
    abs_err = np.abs(err_rad)
    max_indices = np.argmax(abs_err, axis=0)
    return {
        "max_abs_rad": np.max(abs_err, axis=0),
        "rms_rad": np.sqrt(np.mean(err_rad * err_rad, axis=0)),
        "mean_abs_rad": np.mean(abs_err, axis=0),
        "p95_abs_rad": np.percentile(abs_err, 95, axis=0),
        "final_rad": err_rad[-1, :],
        "max_abs_indices": max_indices,
    }


def finite_stats(values: np.ndarray) -> Optional[Dict[str, float]]:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return None
    return {
        "mean": float(np.mean(finite)),
        "median": float(np.median(finite)),
        "p95": float(np.percentile(finite, 95)),
        "max": float(np.max(finite)),
        "min": float(np.min(finite)),
    }


def print_realtime_summary(
    label: str,
    t: np.ndarray,
    active: np.ndarray,
    published_this_cycle: np.ndarray,
    total_published: np.ndarray,
    publish_period: np.ndarray,
    subscription_count: np.ndarray,
    feedback_age: np.ndarray,
    main_loop_wall_dt: np.ndarray,
    publish_wall_dt: np.ndarray,
) -> None:
    if t.size < 2:
        return
    dt = np.diff(t)
    median_dt = float(np.median(dt))
    p95_dt = float(np.percentile(dt, 95))
    max_dt = float(np.max(dt))
    rate = 1.0 / median_dt if median_dt > 0.0 else math.nan
    print(
        f"[Realtime] {label}: datalog_dt median={median_dt:.6f}s "
        f"p95={p95_dt:.6f}s max={max_dt:.6f}s, rate={rate:.1f}Hz"
    )

    main_wall_stats = finite_stats(main_loop_wall_dt[main_loop_wall_dt > 0.0] * 1000.0)
    if main_wall_stats is not None:
        print(
            f"[Realtime] {label}: main_loop_wall_dt median={main_wall_stats['median']:.3f}ms "
            f"p95={main_wall_stats['p95']:.3f}ms max={main_wall_stats['max']:.3f}ms"
        )

    if np.all(~np.isfinite(published_this_cycle)) or np.all(~np.isfinite(total_published)):
        print(f"[Realtime] {label}: publish diagnostic fields not present in this datalog")
        return

    active_mask = active > 0.5
    if not np.any(active_mask):
        print(f"[Realtime] {label}: real command publishing was not active in this segment")
        return

    active_publish_flags = published_this_cycle[active_mask]
    active_total = total_published[active_mask]
    published_rows = int(np.count_nonzero(active_publish_flags > 0.5))
    active_rows = int(active_publish_flags.size)
    skipped_rows = active_rows - published_rows
    total_delta = float(active_total[-1] - active_total[0]) if active_total.size > 1 else 0.0
    expected_delta = max(0, active_rows - 1)
    print(
        f"[Realtime] {label}: active_rows={active_rows}, published_rows={published_rows}, "
        f"skipped_rows={skipped_rows}, total_publish_delta={total_delta:.0f}, "
        f"expected_delta_at_1k={expected_delta}"
    )

    period_stats = finite_stats(publish_period[active_mask])
    if period_stats is not None:
        print(
            f"[Realtime] {label}: publish_period median={period_stats['median']:.6f}s "
            f"min={period_stats['min']:.6f}s max={period_stats['max']:.6f}s"
        )

    publish_wall_stats = finite_stats(publish_wall_dt[(active_mask) & (publish_wall_dt > 0.0)] * 1000.0)
    if publish_wall_stats is not None:
        print(
            f"[Realtime] {label}: publish_wall_dt median={publish_wall_stats['median']:.3f}ms "
            f"p95={publish_wall_stats['p95']:.3f}ms max={publish_wall_stats['max']:.3f}ms"
        )

    sub_stats = finite_stats(subscription_count[active_mask])
    if sub_stats is not None:
        print(
            f"[Realtime] {label}: subscription_count min={sub_stats['min']:.0f} "
            f"median={sub_stats['median']:.0f}"
        )

    age_stats = finite_stats(feedback_age[active_mask] * 1000.0)
    if age_stats is not None:
        print(
            f"[Realtime] {label}: feedback_age median={age_stats['median']:.3f}ms "
            f"p95={age_stats['p95']:.3f}ms max={age_stats['max']:.3f}ms"
        )


def write_markdown_summary(path: Path, name: str, t: np.ndarray, err_rad: np.ndarray) -> None:
    stats = compute_stats(err_rad)
    duration = float(t[-1] - t[0]) if t.size > 1 else 0.0
    lines = [
        f"# Right Arm Tracking Summary: {name}",
        "",
        f"- samples: {t.size}",
        f"- duration_s: {duration:.6f}",
        "",
        "| joint | max_abs_deg | rms_deg | mean_abs_deg | p95_abs_deg | final_err_deg |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for i, joint in enumerate(JOINT_NAMES):
        lines.append(
            "| {joint} | {max_deg:.6f} | {rms_deg:.6f} | {mean_deg:.6f} | {p95_deg:.6f} | {final_deg:.6f} |".format(
                joint=joint,
                max_deg=math.degrees(float(stats["max_abs_rad"][i])),
                rms_deg=math.degrees(float(stats["rms_rad"][i])),
                mean_deg=math.degrees(float(stats["mean_abs_rad"][i])),
                p95_deg=math.degrees(float(stats["p95_abs_rad"][i])),
                final_deg=math.degrees(float(stats["final_rad"][i])),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_csv_summary(path: Path, t: np.ndarray, err_rad: np.ndarray) -> None:
    stats = compute_stats(err_rad)
    t_rel = t - t[0]
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "joint",
                "max_abs_err_deg",
                "rms_err_deg",
                "mean_abs_err_deg",
                "p95_abs_err_deg",
                "final_err_deg",
                "max_abs_err_time_rel_s",
            ]
        )
        for i, joint in enumerate(JOINT_NAMES):
            max_index = int(stats["max_abs_indices"][i])
            writer.writerow(
                [
                    joint,
                    f"{math.degrees(float(stats['max_abs_rad'][i])):.9g}",
                    f"{math.degrees(float(stats['rms_rad'][i])):.9g}",
                    f"{math.degrees(float(stats['mean_abs_rad'][i])):.9g}",
                    f"{math.degrees(float(stats['p95_abs_rad'][i])):.9g}",
                    f"{math.degrees(float(stats['final_rad'][i])):.9g}",
                    f"{float(t_rel[max_index]):.9g}",
                ]
            )


def write_segment_csv(
    path: Path,
    t: np.ndarray,
    motion_state: np.ndarray,
    weld_active: np.ndarray,
    weld_phase: np.ndarray,
    weld_prepare_phase: np.ndarray,
    weld_recover_phase: np.ndarray,
    cmd: np.ndarray,
    fb: np.ndarray,
    err: np.ndarray,
) -> None:
    t_rel = t - t[0]
    header = (
        ["time_s", "time_rel_s", "motionState", "weld_active", "weld_phase", "weld_prepare_phase", "weld_recover_phase"]
        + [f"cmd_{joint}_rad" for joint in JOINT_NAMES]
        + [f"fb_{joint}_rad" for joint in JOINT_NAMES]
        + [f"err_{joint}_rad" for joint in JOINT_NAMES]
    )
    table = np.column_stack(
        [
            t,
            t_rel,
            motion_state,
            weld_active,
            weld_phase,
            weld_prepare_phase,
            weld_recover_phase,
            cmd,
            fb,
            err,
        ]
    )
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(header)
        writer.writerows(table)


def copy_latest(path: Path, latest_path: Path) -> None:
    if path.resolve() != latest_path.resolve():
        shutil.copyfile(path, latest_path)


def plot_old_format(
    path: Path,
    title: str,
    t: np.ndarray,
    cmd: np.ndarray,
    fb: np.ndarray,
    err: np.ndarray,
    feedback_age: np.ndarray,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t_rel = t - t[0] if t.size else t
    cmd_deg = np.rad2deg(cmd)
    fb_deg = np.rad2deg(fb)
    err_deg = np.rad2deg(err)
    stats = compute_stats(err)
    feedback_age_ms = feedback_age * 1000.0

    fig, axes = plt.subplots(len(JOINT_NAMES), 2, figsize=(15, 12), sharex=True)
    fig.suptitle(
        "{title}, t={t0:.3f}-{t1:.3f}s, duration={duration:.3f}s, samples={samples}, "
        "feedback age mean={age_mean:.3f}ms".format(
            title=title,
            t0=float(t[0]),
            t1=float(t[-1]),
            duration=float(t[-1] - t[0]),
            samples=t.size,
            age_mean=float(np.nanmean(feedback_age_ms)),
        ),
        fontsize=12,
    )

    axes[0, 0].set_title("Right arm joint position: command vs feedback")
    axes[0, 1].set_title("Tracking error: cmd - feedback")
    for i, joint in enumerate(JOINT_NAMES):
        label = joint.replace("right_", "").replace("_joint", "")
        axes[i, 0].plot(t_rel, cmd_deg[:, i], label="cmd", linewidth=1.2)
        axes[i, 0].plot(t_rel, fb_deg[:, i], label="feedback", linewidth=1.0)
        axes[i, 0].set_ylabel(f"{label}\n(deg)")
        axes[i, 0].grid(True, alpha=0.3)

        axes[i, 1].plot(t_rel, err_deg[:, i], color="tab:red", linewidth=1.0)
        axes[i, 1].axhline(0.0, color="black", linewidth=0.7, alpha=0.45)
        axes[i, 1].set_ylabel("err (deg)")
        axes[i, 1].grid(True, alpha=0.3)
        axes[i, 1].text(
            0.01,
            0.92,
            "max={max_abs:.3f} deg  rms={rms:.3f} deg  p95={p95:.3f} deg".format(
                max_abs=math.degrees(float(stats["max_abs_rad"][i])),
                rms=math.degrees(float(stats["rms_rad"][i])),
                p95=math.degrees(float(stats["p95_abs_rad"][i])),
            ),
            transform=axes[i, 1].transAxes,
            fontsize=8,
            va="top",
        )
        if i == 0:
            axes[i, 0].legend(loc="best", ncol=2, fontsize=8)

    axes[-1, 0].set_xlabel("Weld segment time (s)")
    axes[-1, 1].set_xlabel("Weld segment time (s)")
    fig.tight_layout(rect=[0.0, 0.0, 1.0, 0.965])
    fig.savefig(path, dpi=160)
    plt.close(fig)


def select_segment(name: str, mask: np.ndarray, t: np.ndarray, cmd: np.ndarray, fb: np.ndarray, err: np.ndarray):
    if not np.any(mask):
        print(f"[RightArmTracking] skip {name}: no valid samples")
        return None
    return t[mask], cmd[mask], fb[mask], err[mask]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    root = repo_root()
    parser.add_argument("--log", default=root / "record" / "datalog.log", type=Path)
    parser.add_argument("--script", default=root / "record" / "matlabReadDataScript.txt", type=Path)
    parser.add_argument("--out-dir", default=root / "record", type=Path)
    parser.add_argument("--prefix", default="weld_right_arm_tracking", help="output filename prefix")
    args = parser.parse_args()

    data, mapping = load_data(args.log, args.script)
    t = column(data, mapping, "dyn_time").reshape(-1)
    cmd = column(data, mapping, "real_right_arm_cmd_pos")
    fb = column(data, mapping, "real_right_arm_fb_pos")
    err = column(data, mapping, "real_right_arm_track_err")
    valid = column(data, mapping, "real_right_arm_track_valid").reshape(-1) > 0.5
    motion_state = column(data, mapping, "motionState").reshape(-1)
    weld_active = column(data, mapping, "weld_active").reshape(-1) if "weld_active" in mapping else np.zeros_like(t)
    weld_phase = column(data, mapping, "weld_phase").reshape(-1) if "weld_phase" in mapping else np.zeros_like(t)
    weld_prepare_phase = (
        column(data, mapping, "weld_prepare_phase").reshape(-1) if "weld_prepare_phase" in mapping else np.zeros_like(t)
    )
    weld_recover_phase = (
        column(data, mapping, "weld_recover_phase").reshape(-1) if "weld_recover_phase" in mapping else np.zeros_like(t)
    )
    feedback_age = (
        column(data, mapping, "real_right_arm_feedback_age").reshape(-1)
        if "real_right_arm_feedback_age" in mapping
        else np.full_like(t, np.nan)
    )
    publish_active = optional_column(data, mapping, "real_command_publish_active", 0.0)
    publish_period = optional_column(data, mapping, "real_command_publish_period")
    published_this_cycle = optional_column(data, mapping, "real_command_published_this_cycle")
    total_published = optional_column(data, mapping, "real_command_total_published")
    subscription_count = optional_column(data, mapping, "real_command_subscription_count")
    main_loop_wall_dt = optional_column(data, mapping, "real_main_loop_wall_dt")
    publish_wall_dt = optional_column(data, mapping, "real_command_publish_wall_dt")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    dt = np.diff(t)
    median_dt = float(np.median(dt)) if dt.size else math.nan
    stamp = Path(args.log).stat().st_mtime
    stamp_text = __import__("datetime").datetime.fromtimestamp(stamp).strftime("%Y%m%d_%H%M")
    motion_state_int = np.rint(motion_state).astype(int)
    mask = valid & (motion_state_int == MOTION_STATE_WELD)
    selected = select_segment("weld", mask, t, cmd, fb, err)
    if selected is None:
        return 1

    seg_t, seg_cmd, seg_fb, seg_err = selected
    seg_mask_indices = np.flatnonzero(mask)
    seg_motion_state = motion_state[seg_mask_indices]
    seg_weld_active = weld_active[seg_mask_indices]
    seg_weld_phase = weld_phase[seg_mask_indices]
    seg_weld_prepare_phase = weld_prepare_phase[seg_mask_indices]
    seg_weld_recover_phase = weld_recover_phase[seg_mask_indices]
    seg_feedback_age = feedback_age[seg_mask_indices]
    seg_publish_active = publish_active[seg_mask_indices]
    seg_publish_period = publish_period[seg_mask_indices]
    seg_published_this_cycle = published_this_cycle[seg_mask_indices]
    seg_total_published = total_published[seg_mask_indices]
    seg_subscription_count = subscription_count[seg_mask_indices]
    seg_main_loop_wall_dt = main_loop_wall_dt[seg_mask_indices]
    seg_publish_wall_dt = publish_wall_dt[seg_mask_indices]

    csv_path = args.out_dir / f"{args.prefix}_weld_segment_{stamp_text}.csv"
    summary_csv_path = args.out_dir / f"{args.prefix}_weld_segment_summary_{stamp_text}.csv"
    summary_md_path = args.out_dir / f"{args.prefix}_weld_segment_summary_{stamp_text}.md"
    png_path = args.out_dir / f"{args.prefix}_error_old_format_{stamp_text}.png"

    write_segment_csv(
        csv_path,
        seg_t,
        seg_motion_state,
        seg_weld_active,
        seg_weld_phase,
        seg_weld_prepare_phase,
        seg_weld_recover_phase,
        seg_cmd,
        seg_fb,
        seg_err,
    )
    write_csv_summary(summary_csv_path, seg_t, seg_err)
    write_markdown_summary(summary_md_path, "weld", seg_t, seg_err)
    plot_old_format(
        png_path,
        "Weld right-arm tracking",
        seg_t,
        seg_cmd,
        seg_fb,
        seg_err,
        seg_feedback_age,
    )

    copy_latest(csv_path, args.out_dir / f"{args.prefix}_weld_segment_latest.csv")
    copy_latest(summary_csv_path, args.out_dir / f"{args.prefix}_weld_segment_summary_latest.csv")
    copy_latest(png_path, args.out_dir / f"{args.prefix}_error_latest.png")

    stats = compute_stats(seg_err)
    print(
        f"[RightArmTracking] latest log: {args.log} "
        f"samples={data.shape[0]}, dt={median_dt:.6f}s, rate={1.0 / median_dt if median_dt > 0 else math.nan:.3f}Hz"
    )
    print(
        f"[RightArmTracking] weld: t={seg_t[0]:.3f}-{seg_t[-1]:.3f}s, "
        f"duration={seg_t[-1] - seg_t[0]:.3f}s, samples={seg_t.size}"
    )
    print_realtime_summary(
        "all",
        t,
        publish_active,
        published_this_cycle,
        total_published,
        publish_period,
        subscription_count,
        feedback_age,
        main_loop_wall_dt,
        publish_wall_dt,
    )
    print_realtime_summary(
        "weld",
        seg_t,
        seg_publish_active,
        seg_published_this_cycle,
        seg_total_published,
        seg_publish_period,
        seg_subscription_count,
        seg_feedback_age,
        seg_main_loop_wall_dt,
        seg_publish_wall_dt,
    )
    for i, joint in enumerate(JOINT_NAMES):
        print(
            f"[RightArmTracking] {joint}: "
            f"max_abs={math.degrees(float(stats['max_abs_rad'][i])):.4f}deg, "
            f"rms={math.degrees(float(stats['rms_rad'][i])):.4f}deg, "
            f"p95={math.degrees(float(stats['p95_abs_rad'][i])):.4f}deg"
        )
    print(f"[RightArmTracking] plot: {png_path}")
    print(f"[RightArmTracking] latest plot: {args.out_dir / f'{args.prefix}_error_latest.png'}")
    print(f"[RightArmTracking] csv: {csv_path}")
    print(f"[RightArmTracking] summary: {summary_csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
