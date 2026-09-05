#!/usr/bin/env python3
"""
G3.3 gate-sweep analysis. Aggregates run_gate.sh output directories by
contention preset and computes the full-window RPE-1s metric (the only
metric/window that held up in §47/§48 -- endpoint error, windowed
drift-rate slope, and windowed RPE were all tried and rejected there).

Reads each run's rosbag directly via triage_incident.py's read_bag (not
vslam_compare.csv -- see §43's contamination lesson), matching the
convention of every prior throwaway analysis script in this project
(remetric_mini_dose.py, power_analysis.py, windowed_rpe.py), now made a
permanent, reusable tool per this task's request.

Run naming convention: <prefix>_<level>_r<rep>, e.g. gate_sweep_none_r03.
Levels are matched by substring against LEVELS below.

Usage:
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/ros2_ws/install/setup.bash
  python3 analyze_sweep.py --results-dir <dir> --prefix gate_sweep \
      [--n-boot 20000] [--out-json <path>]
"""
import argparse
import bisect
import csv
import json
import math
import os
import re
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from triage_incident import read_bag  # noqa: E402

import numpy as np  # noqa: E402
from scipy import stats  # noqa: E402

LEVELS = ["none", "low", "medium", "high", "extreme"]
LEVEL_CODE = {lvl: i for i, lvl in enumerate(LEVELS)}

TIME_WINDOW_S = 1.0
TIME_WINDOW_TOL_S = 0.15


def full_align(vslam, gt):
    """Origin-subtract each trajectory at its own first sample and return
    per-sample position vectors (not just norm error) -- needed for RPE,
    which compares position deltas, not just their magnitude. Same
    convention as triage_incident.align_and_compute_error."""
    if not vslam or not gt:
        return []
    gt_t0 = (gt[0].x, gt[0].y, gt[0].z)
    vslam_t0 = (vslam[0].x, vslam[0].y, vslam[0].z)
    gt_sim_times = [s.t_sim for s in gt]

    rows = []
    for s in vslam:
        idx = bisect.bisect_left(gt_sim_times, s.t_sim)
        best = None
        for cand in (idx - 1, idx):
            if 0 <= cand < len(gt):
                dt = abs(gt[cand].t_sim - s.t_sim)
                if best is None or dt < best[0]:
                    best = (dt, gt[cand])
        if best is None or best[0] > 0.5:
            continue
        g = best[1]
        vx, vy, vz = s.x - vslam_t0[0], s.y - vslam_t0[1], s.z - vslam_t0[2]
        gx, gy, gz = g.x - gt_t0[0], g.y - gt_t0[1], g.z - gt_t0[2]
        pos_err = math.sqrt((vx - gx) ** 2 + (vy - gy) ** 2 + (vz - gz) ** 2)
        rows.append({"t_recv": s.t_recv, "vx": vx, "vy": vy, "vz": vz,
                      "gx": gx, "gy": gy, "gz": gz, "pos_err": pos_err})
    return rows


def attach_matched_series(rows, series, key):
    if not series:
        for r in rows:
            r[key] = None
        return rows
    times = [t for t, _ in series]
    for r in rows:
        idx = bisect.bisect_left(times, r["t_recv"])
        best = None
        for cand in (idx - 1, idx):
            if 0 <= cand < len(series):
                dt = abs(series[cand][0] - r["t_recv"])
                if best is None or dt < best[0]:
                    best = (dt, series[cand][1])
        r[key] = best[1] if best else None
    return rows


def rpe_time_window(rows, window_s, tol_s):
    n = len(rows)
    vals = []
    for i in range(n):
        target = rows[i]["t_recv"] + window_s
        best_j, best_dt = None, None
        for j in range(i + 1, n):
            dt = abs(rows[j]["t_recv"] - target)
            if best_dt is None or dt < best_dt:
                best_dt, best_j = dt, j
            if rows[j]["t_recv"] - rows[i]["t_recv"] > window_s + tol_s:
                break
        if best_j is None or best_dt > tol_s:
            continue
        j = best_j
        dvx, dvy, dvz = (rows[j]["vx"] - rows[i]["vx"],
                         rows[j]["vy"] - rows[i]["vy"],
                         rows[j]["vz"] - rows[i]["vz"])
        dgx, dgy, dgz = (rows[j]["gx"] - rows[i]["gx"],
                         rows[j]["gy"] - rows[i]["gy"],
                         rows[j]["gz"] - rows[i]["gz"])
        vals.append(math.sqrt((dvx - dgx) ** 2 + (dvy - dgy) ** 2 + (dvz - dgz) ** 2))
    return vals


def detect_level(exp_id, prefix):
    rest = exp_id[len(prefix):].lstrip("_") if exp_id.startswith(prefix) else exp_id
    for lvl in sorted(LEVELS, key=len, reverse=True):
        if re.search(rf"(^|_){re.escape(lvl)}(_|$)", rest):
            return lvl
    return None


def analyze_run(run_dir, exp_id, prefix):
    bag_path = os.path.join(run_dir, "rosbag")
    if not os.path.isdir(bag_path):
        return None
    data, _ = read_bag(bag_path, None, None)
    rows = full_align(data.vslam, data.gt)
    if len(rows) < 10:
        return None
    rows = attach_matched_series(rows, data.feature_count, "feature_count")
    rows = attach_matched_series(rows, data.vo_state, "vo_state")

    rpe_t = rpe_time_window(rows, TIME_WINDOW_S, TIME_WINDOW_TOL_S)
    if not rpe_t:
        return None

    fc_all = [r["feature_count"] for r in rows if r["feature_count"] is not None]
    had_dropout = any(fc == 0 for fc in fc_all)
    vo_states = set(r["vo_state"] for r in rows if r["vo_state"] is not None)

    gpu_path = os.path.join(run_dir, "gpu_log.csv")
    gpu_util, gpu_temp = [], []
    try:
        with open(gpu_path) as f:
            for row in csv.DictReader(f):
                gpu_util.append(float(row["gpu_util_pct"]))
                gpu_temp.append(float(row["temp_c"]))
    except FileNotFoundError:
        pass

    level = detect_level(exp_id, prefix)
    return {
        "exp_id": exp_id,
        "level": level,
        "n_matched": len(rows),
        "rpe_1s_mean": statistics.mean(rpe_t),
        "rpe_1s_median": statistics.median(rpe_t),
        "rpe_1s_n": len(rpe_t),
        "had_dropout": had_dropout,
        "min_feature_count": min(fc_all) if fc_all else None,
        "vo_states_seen": sorted(vo_states),
        "gpu_util_mean": statistics.mean(gpu_util) if gpu_util else None,
        "gpu_temp_mean": statistics.mean(gpu_temp) if gpu_temp else None,
        "gpu_temp_max": max(gpu_temp) if gpu_temp else None,
    }


def bootstrap_mean_ci(vals, n_boot, rng):
    if len(vals) < 2:
        return (float("nan"), float("nan"))
    arr = np.array(vals)
    boots = rng.choice(arr, size=(n_boot, len(arr)), replace=True).mean(axis=1)
    lo, hi = np.percentile(boots, [2.5, 97.5])
    return float(lo), float(hi)


def bootstrap_slope_ci(level_vals, n_boot, rng):
    arrs = {lvl: np.array(v) for lvl, v in level_vals.items() if v}
    if len(arrs) < 3:
        return (float("nan"), float("nan"))
    slopes = []
    for _ in range(n_boot):
        codes, ys = [], []
        for lvl, arr in arrs.items():
            resampled = rng.choice(arr, size=len(arr), replace=True)
            codes.extend([LEVEL_CODE[lvl]] * len(arr))
            ys.extend(resampled)
        slope, *_ = stats.linregress(codes, ys)
        slopes.append(slope)
    lo, hi = np.percentile(slopes, [2.5, 97.5])
    return float(lo), float(hi)


def trend_test(level_vals):
    codes, ys = [], []
    for lvl in LEVELS:
        for v in level_vals.get(lvl, []):
            codes.append(LEVEL_CODE[lvl])
            ys.append(v)
    if len(set(codes)) < 3:
        return None
    slope, intercept, r, p, se = stats.linregress(codes, ys)
    return {"slope": slope, "r": r, "p": p, "n": len(ys)}


def discrete_vs_graded(level_means):
    """Heuristic only (matches triage_incident.py's own house style: state
    what was observed, not a proof). Flags a discrete jump if one
    adjacent-level step accounts for most of the total none->worst-level
    range; otherwise calls the rise graded."""
    present = [lvl for lvl in LEVELS if lvl in level_means]
    if len(present) < 3:
        return {"flag": "insufficient_levels", "detail": ""}
    vals = [level_means[lvl] for lvl in present]
    total_range = vals[-1] - vals[0]
    if total_range <= 0:
        return {"flag": "no_rise", "detail": f"levels {present}: {vals}"}
    steps = [vals[i + 1] - vals[i] for i in range(len(vals) - 1)]
    max_step = max(steps)
    max_step_idx = steps.index(max_step)
    frac = max_step / total_range
    if frac > 0.7:
        flag = "discrete_jump"
        detail = (f"{max_step/total_range:.0%} of the none->{present[-1]} rise happens in one step, "
                  f"{present[max_step_idx]}->{present[max_step_idx+1]} "
                  f"({vals[max_step_idx]:.4f}->{vals[max_step_idx+1]:.4f})")
    else:
        flag = "graded"
        detail = f"largest single step is {frac:.0%} of the total rise ({present})"
    return {"flag": flag, "detail": detail}


def run_analysis(results_dir, prefix, n_boot, seed=20260906):
    rng = np.random.default_rng(seed)
    all_dirs = sorted(d for d in os.listdir(results_dir)
                       if d.startswith(prefix) and os.path.isdir(os.path.join(results_dir, d)))
    runs = []
    for exp_id in all_dirs:
        run_dir = os.path.join(results_dir, exp_id)
        r = analyze_run(run_dir, exp_id, prefix)
        if r is None:
            print(f"WARNING: {exp_id} produced no usable data (skipped)", file=sys.stderr)
            continue
        if r["level"] is None:
            print(f"WARNING: {exp_id} -- could not detect level from name (skipped)", file=sys.stderr)
            continue
        runs.append(r)

    by_level = {lvl: [r for r in runs if r["level"] == lvl] for lvl in LEVELS}
    level_vals_all = {lvl: [r["rpe_1s_median"] for r in by_level[lvl]] for lvl in LEVELS}
    level_vals_clean = {lvl: [r["rpe_1s_median"] for r in by_level[lvl] if not r["had_dropout"]] for lvl in LEVELS}

    summary = {"n_runs_total": len(runs), "levels": {}}
    level_means_all = {}
    for lvl in LEVELS:
        vals = level_vals_all[lvl]
        if not vals:
            continue
        m = statistics.mean(vals)
        sd = statistics.stdev(vals) if len(vals) > 1 else 0.0
        lo, hi = bootstrap_mean_ci(vals, n_boot, rng)
        n_dropout = sum(1 for r in by_level[lvl] if r["had_dropout"])
        gpu_utils = [r["gpu_util_mean"] for r in by_level[lvl] if r["gpu_util_mean"] is not None]
        gpu_temps = [r["gpu_temp_max"] for r in by_level[lvl] if r["gpu_temp_max"] is not None]
        summary["levels"][lvl] = {
            "n": len(vals), "mean": m, "sd": sd, "ci95": [lo, hi],
            "n_dropout_events": n_dropout,
            "gpu_util_mean": statistics.mean(gpu_utils) if gpu_utils else None,
            "gpu_temp_max_mean": statistics.mean(gpu_temps) if gpu_temps else None,
        }
        level_means_all[lvl] = m

    trend_all = trend_test(level_vals_all)
    trend_all_ci = bootstrap_slope_ci(level_vals_all, n_boot, rng)
    trend_clean = trend_test(level_vals_clean)
    trend_clean_ci = bootstrap_slope_ci(level_vals_clean, n_boot, rng)

    d_none_extreme = None
    if level_vals_all.get("none") and level_vals_all.get("extreme"):
        mn, sdn = statistics.mean(level_vals_all["none"]), (statistics.stdev(level_vals_all["none"]) if len(level_vals_all["none"]) > 1 else 0)
        me, sde = statistics.mean(level_vals_all["extreme"]), (statistics.stdev(level_vals_all["extreme"]) if len(level_vals_all["extreme"]) > 1 else 0)
        pooled = math.sqrt((sdn ** 2 + sde ** 2) / 2)
        d_none_extreme = (me - mn) / pooled if pooled > 0 else float("nan")

    summary["trend_all_runs"] = trend_all
    summary["trend_all_runs_slope_ci95"] = list(trend_all_ci)
    summary["trend_clean_only"] = trend_clean
    summary["trend_clean_only_slope_ci95"] = list(trend_clean_ci)
    summary["cohens_d_none_to_extreme"] = d_none_extreme
    summary["discrete_vs_graded"] = discrete_vs_graded(level_means_all)
    summary["runs"] = runs
    return summary


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results-dir", required=True)
    ap.add_argument("--prefix", required=True, help="exp_id prefix, e.g. gate_sweep")
    ap.add_argument("--n-boot", type=int, default=20000)
    ap.add_argument("--out-json", default=None)
    args = ap.parse_args()

    summary = run_analysis(args.results_dir, args.prefix, args.n_boot)

    print("=" * 90)
    print(f"Per-level RPE-1s summary ({summary['n_runs_total']} runs total)")
    print("=" * 90)
    for lvl in LEVELS:
        s = summary["levels"].get(lvl)
        if not s:
            print(f"  {lvl:8s} -- no data")
            continue
        print(f"  {lvl:8s} n={s['n']:3d} mean={s['mean']:.4f} sd={s['sd']:.4f} "
              f"95%CI=({s['ci95'][0]:.4f},{s['ci95'][1]:.4f}) dropout_events={s['n_dropout_events']} "
              f"gpu_util_mean={s['gpu_util_mean']} gpu_temp_max_mean={s['gpu_temp_max_mean']}")

    print()
    print("=" * 90)
    print("Monotonic dose trend (OLS, level coded 0-4), all runs")
    print("=" * 90)
    t = summary["trend_all_runs"]
    if t:
        ci = summary["trend_all_runs_slope_ci95"]
        print(f"  slope={t['slope']:.4f}/level  r={t['r']:.4f}  p={t['p']:.4f}  n={t['n']}  "
              f"bootstrap slope 95%CI=({ci[0]:.4f},{ci[1]:.4f})")
    else:
        print("  insufficient levels for a trend test")
    print(f"  Cohen's d (none->extreme): {summary['cohens_d_none_to_extreme']}")

    print()
    print("=" * 90)
    print("Monotonic dose trend, CLEAN-TRACKING RUNS ONLY (dropout events excluded)")
    print("=" * 90)
    t = summary["trend_clean_only"]
    if t:
        ci = summary["trend_clean_only_slope_ci95"]
        print(f"  slope={t['slope']:.4f}/level  r={t['r']:.4f}  p={t['p']:.4f}  n={t['n']}  "
              f"bootstrap slope 95%CI=({ci[0]:.4f},{ci[1]:.4f})")
    else:
        print("  insufficient clean-tracking levels for a trend test")

    print()
    print("=" * 90)
    print("Discrete jump vs. graded rise")
    print("=" * 90)
    dvg = summary["discrete_vs_graded"]
    print(f"  {dvg['flag']}: {dvg['detail']}")

    if args.out_json:
        with open(args.out_json, "w") as f:
            json.dump(summary, f, indent=2, default=str)
        print(f"\nWrote {args.out_json}")


if __name__ == "__main__":
    main()
