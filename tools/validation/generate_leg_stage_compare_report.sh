#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

usage() {
  cat <<USAGE
Usage:
  $(basename "$0") \
    --stage1 <stage1_log> \
    --stage2 <stage2_log> \
    --stage3 <stage3_log> \
    [--output <report_md_path>]

Example:
  $(basename "$0") \
    --stage1 record/datalog_stage1.log \
    --stage2 record/datalog_stage2.log \
    --stage3 record/datalog_stage3.log
USAGE
}

stage1=""
stage2=""
stage3=""
output=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --stage1)
      stage1="$2"
      shift 2
      ;;
    --stage2)
      stage2="$2"
      shift 2
      ;;
    --stage3)
      stage3="$2"
      shift 2
      ;;
    --output)
      output="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] unknown arg: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ -z "$stage1" || -z "$stage2" || -z "$stage3" ]]; then
  echo "[ERROR] stage1/stage2/stage3 are required." >&2
  usage
  exit 1
fi

resolve_path() {
  local p="$1"
  if [[ "$p" = /* ]]; then
    echo "$p"
  else
    echo "${REPO_ROOT}/$p"
  fi
}

stage1="$(resolve_path "$stage1")"
stage2="$(resolve_path "$stage2")"
stage3="$(resolve_path "$stage3")"

for f in "$stage1" "$stage2" "$stage3"; do
  if [[ ! -f "$f" ]]; then
    echo "[ERROR] log not found: $f" >&2
    exit 1
  fi
  if [[ ! -s "$f" ]]; then
    echo "[ERROR] log is empty: $f" >&2
    exit 1
  fi
done

if [[ -z "$output" ]]; then
  output="${REPO_ROOT}/record/leg_stage_compare_$(date +%Y%m%d_%H%M%S).md"
else
  output="$(resolve_path "$output")"
fi

mkdir -p "$(dirname "$output")"

overview_row() {
  local label="$1"
  local log="$2"

  awk -F',' -v label="$label" '
    function abs(x){return x<0?-x:x}
    {
      rows++
      last_t = $1 + 0.0
      ms = $178 + 0
      if (ms==0) ms0++
      else if (ms==1) ms1++
      else if (ms==2) ms2++

      if (ms==1) {
        if (!walk_seen) {
          walk_first = $1 + 0.0
          walk_seen = 1
        }
        walk_last = $1 + 0.0
        walk_n++
      }

      if (($1+0.0) >= 2.0) {
        n++
        ex=$137+0.0; ey=$138+0.0; ez=$139+0.0
        evx=$140+0.0; evy=$141+0.0; evz=$142+0.0
        eyaw=$143+0.0
        ep = sqrt(ex*ex + ey*ey + ez*ez)
        ev = sqrt(evx*evx + evy*evy + evz*evz)
        yaw = abs(eyaw)
        sum_ep += ep
        sum_ev += ev
        sum_yaw += yaw
        if (($176+0.0) != 0.0) qp_bad++
        if (ms==1 || ms==2) {
          walk_phase_n++
          if (($176+0.0) != 0.0) qp_bad_walk++
        }
      }
    }
    END {
      walk_dur = (walk_n>0) ? (walk_last-walk_first) : 0.0
      mean_ep = (n>0) ? (sum_ep/n) : 0.0
      mean_ev = (n>0) ? (sum_ev/n) : 0.0
      mean_yaw = (n>0) ? (sum_yaw/n) : 0.0
      qp_ratio = (n>0) ? (qp_bad/n) : 0.0
      qp_ratio_walk = (walk_phase_n>0) ? (qp_bad_walk/walk_phase_n) : 0.0

      printf("| %s | %d | %.3f | %d/%d/%d | %.3f | %.6f | %.6f | %.6f | %.6f | %.6f |\n",
             label, rows, last_t, ms0, ms1, ms2, walk_dur,
             mean_ep, mean_ev, mean_yaw, qp_ratio, qp_ratio_walk)
    }
  ' "$log"
}

phase_rows() {
  local label="$1"
  local log="$2"

  awk -F',' -v label="$label" '
    function abs(x){return x<0?-x:x}
    function upd(prefix, ex, ey, ez, evx, evy, evz, eyaw, qp){
      ep = sqrt(ex*ex + ey*ey + ez*ez)
      ev = sqrt(evx*evx + evy*evy + evz*evz)
      yaw = abs(eyaw)
      n[prefix]++
      sum_ep[prefix] += ep
      sum_ev[prefix] += ev
      sum_yaw[prefix] += yaw
      if (ep > max_ep[prefix]) max_ep[prefix] = ep
      if (ev > max_ev[prefix]) max_ev[prefix] = ev
      if (yaw > max_yaw[prefix]) max_yaw[prefix] = yaw
      if (qp != 0.0) qp_bad[prefix]++
    }
    function out(prefix, phase_name){
      if (n[prefix] <= 0) {
        printf("| %s | %s | 0 | - | - | - | - | - | - | - |\n", label, phase_name)
        return
      }
      printf("| %s | %s | %d | %.6f | %.6f | %.6f | %.6f | %.6f | %.6f | %.6f |\n",
             label, phase_name, n[prefix],
             sum_ep[prefix]/n[prefix], sum_ev[prefix]/n[prefix], sum_yaw[prefix]/n[prefix],
             max_ep[prefix], max_ev[prefix], max_yaw[prefix], qp_bad[prefix]/n[prefix])
    }
    {
      t = $1 + 0.0
      ex=$137+0.0; ey=$138+0.0; ez=$139+0.0
      evx=$140+0.0; evy=$141+0.0; evz=$142+0.0
      eyaw=$143+0.0
      qp=$176+0.0

      if (t >= 2.0 && t < 5.0) upd("open", ex, ey, ez, evx, evy, evz, eyaw, qp)
      if (t >= 5.0 && t < 10.0) upd("closed", ex, ey, ez, evx, evy, evz, eyaw, qp)
      if (t >= 10.0 && t < 26.0) upd("walk", ex, ey, ez, evx, evy, evz, eyaw, qp)
      if (t >= 2.0) upd("all", ex, ey, ez, evx, evy, evz, eyaw, qp)
    }
    END {
      out("open", "open-loop(2-5s)")
      out("closed", "closed-stand(5-10s)")
      out("walk", "walk(10-26s)")
      out("all", "all(>=2s)")
    }
  ' "$log"
}

{
  echo "# Leg 阶段对比报告"
  echo
  echo "- 生成时间: $(date '+%Y-%m-%d %H:%M:%S %Z')"
  echo "- Stage1 日志: \`$stage1\`"
  echo "- Stage2 日志: \`$stage2\`"
  echo "- Stage3 日志: \`$stage3\`"
  echo
  echo "## 总览"
  echo
  echo "| Stage | Rows | EndTime(s) | MotionState(Stand/Walk/Walk2Stand) | WalkDur(s) | Mean|pos_err|(m) | Mean|vel_err|(m/s) | Mean|yaw_err|(rad) | QP非零占比(all>=2s) | QP非零占比(walk) |"
  echo "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"
  overview_row "Stage1" "$stage1"
  overview_row "Stage2" "$stage2"
  overview_row "Stage3" "$stage3"
  echo
  echo "## 分阶段误差与QP"
  echo
  echo "| Stage | Phase | N | Mean|pos_err|(m) | Mean|vel_err|(m/s) | Mean|yaw_err|(rad) | Max|pos_err|(m) | Max|vel_err|(m/s) | Max|yaw_err|(rad) | QP非零占比 |"
  echo "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|"
  phase_rows "Stage1" "$stage1"
  phase_rows "Stage2" "$stage2"
  phase_rows "Stage3" "$stage3"
  echo
  echo "## 判定模板"
  echo
  echo "- 构建通过: [ ] Stage1  [ ] Stage2  [ ] Stage3"
  echo "- 稳定性（无快速跌倒）: [ ] Stage1  [ ] Stage2  [ ] Stage3"
  echo "- QP非零占比 <= 1%: [ ] Stage1  [ ] Stage2  [ ] Stage3"
  echo "- Stage2/3 相对 Stage1 劣化 <= 20%: [ ] Stage2  [ ] Stage3"
  echo
  echo "## 备注"
  echo
  echo "- 本报告默认使用列索引: est_err_pos(137:139), est_err_vel(140:142), est_err_yaw(143), qpStatus_MPC(176), motionState(178)。"
  echo "- 若日志列发生变更，请同步更新脚本索引。"
} > "$output"

echo "[OK] report generated: $output"
