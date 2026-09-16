#!/usr/bin/env python3
"""R2-2 analysis: divergence, EV fusion, message-age/dropped-measurement
timing, and post-hoc min obstacle clearance / collision flag (computed
from ground truth against vio_test.sdf's known, fixed obstacle geometry
-- no live collision sensor needed, and reactive_esdf_avoidance.py
itself is not modified/queried)."""
import argparse
import bisect
import math
import os
import re
import statistics
import sys

import numpy as np
from scipy import stats

sys.path.insert(0, "/workspaces/isaac_ros-dev/tools/feasibility_gate")
from triage_incident import read_bag  # noqa: E402

RESULTS_DIR = "/workspaces/isaac_ros-dev/results/route2_probe"
CONDITIONS = ["L0", "L1", "L2"]
COND_CODE = {"L0": 0, "L1": 1, "L2": 2}

# Obstacle geometry extracted from vio_test.sdf (marker_01..16, pillar_01..08):
# (name, center_x, center_y, center_z, kind, dims) -- box dims=(sx,sy,sz)
# half-extents computed at use; cylinder dims=(radius, length).
OBSTACLES = [
    ("marker_01", 4, 0, 1, "box", (0.5, 0.5, 2)),
    ("marker_02", -4, 3, 1.5, "cyl", (0.4, 3)),
    ("marker_03", 0, 5, 2, "box", (0.6, 0.6, 4)),
    ("marker_04", -3, -4, 2.5, "box", (0.5, 0.5, 5)),
    ("marker_05", 5, -3, 3, "cyl", (0.4, 6)),
    ("marker_06", 2, -6, 3.5, "box", (0.5, 0.5, 7)),
    ("marker_07", -5, -2, 4, "box", (0.6, 0.6, 8)),
    ("marker_08", -2, 6, 4.5, "cyl", (0.4, 9)),
    ("marker_09", 6, 2, 1, "box", (0.5, 0.5, 2)),
    ("marker_10", -6, 1, 1.5, "cyl", (0.35, 3)),
    ("marker_11", 1, -3, 2, "box", (0.4, 0.4, 4)),
    ("marker_12", -1, 4, 2.5, "box", (0.5, 0.5, 5)),
    ("marker_13", 3, 4, 1, "cyl", (0.4, 2)),
    ("marker_14", -3, -1, 1.5, "box", (0.5, 0.5, 3)),
    ("marker_15", 0, -5, 2, "box", (0.5, 0.5, 4)),
    ("marker_16", 4, -1, 2.5, "cyl", (0.4, 5)),
    ("pillar_01", 2.5, 0, 5, "box", (0.4, 0.4, 10)),
    ("pillar_02", 1.77, 1.77, 5, "box", (0.4, 0.4, 10)),
    ("pillar_03", 0, 2.5, 5, "box", (0.4, 0.4, 10)),
    ("pillar_04", -1.77, 1.77, 5, "box", (0.4, 0.4, 10)),
    ("pillar_05", -2.5, 0, 5, "box", (0.4, 0.4, 10)),
    ("pillar_06", -1.77, -1.77, 5, "box", (0.4, 0.4, 10)),
    ("pillar_07", 0, -2.5, 5, "box", (0.4, 0.4, 10)),
    ("pillar_08", 1.77, -1.77, 5, "box", (0.4, 0.4, 10)),
]


def point_to_box_dist(px, py, pz, cx, cy, cz, sx, sy, sz):
    """Signed-ish surface distance (>=0 outside, 0 if inside) from point
    to an axis-aligned box centered at (cx,cy,cz) with full sizes
    (sx,sy,sz)."""
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    dx = max(abs(px - cx) - hx, 0.0)
    dy = max(abs(py - cy) - hy, 0.0)
    dz = max(abs(pz - cz) - hz, 0.0)
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def point_to_cyl_dist(px, py, pz, cx, cy, cz, radius, length):
    """Vertical cylinder (axis along world Z) centered at (cx,cy,cz)."""
    dz = max(abs(pz - cz) - length / 2, 0.0)
    dr = max(math.sqrt((px - cx) ** 2 + (py - cy) ** 2) - radius, 0.0)
    return math.sqrt(dr * dr + dz * dz)


def min_clearance(px, py, pz):
    best = float("inf")
    for name, cx, cy, cz, kind, dims in OBSTACLES:
        if kind == "box":
            d = point_to_box_dist(px, py, pz, cx, cy, cz, *dims)
        else:
            d = point_to_cyl_dist(px, py, pz, cx, cy, cz, *dims)
        if d < best:
            best = d
    return best


def read_full_bag(bag_path):
    data, _ = read_bag(bag_path, None, None)
    est = [(s.t_recv, s.x, s.y, s.z) for s in data.vslam] if False else []
    return data


def analyze_run(exp_id, results_dir=RESULTS_DIR):
    bag_path = os.path.join(results_dir, f"r2_2_{exp_id}", "rosbag")
    if not os.path.isdir(bag_path):
        return None
    data, _ = read_bag(bag_path, None, None)

    # local_position_v1 and ground_truth/odom were captured via the same
    # generic vslam/gt reader slots in triage_incident's read_bag? No --
    # those are px4_msgs/nav_msgs types read_bag doesn't natively parse
    # (it only knows VSLAM_ODOM_TOPIC/GT_ODOM_TOPIC/etc). Re-read raw.
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    msg_classes = {}

    def get_class(topic):
        if topic not in msg_classes:
            msg_classes[topic] = get_message(type_map[topic])
        return msg_classes[topic]

    est, gt = [], []
    aid_pos, aid_hgt, aid_yaw = [], [], []
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic == "/fmu/out/vehicle_local_position_v1":
            msg = deserialize_message(raw, get_class(topic))
            est.append((t_ns * 1e-9, msg.x, msg.y, msg.z))
        elif topic == "/ground_truth/odom":
            msg = deserialize_message(raw, get_class(topic))
            p = msg.pose.pose.position
            gt.append((t_ns * 1e-9, p.x, p.y, p.z))
        elif topic == "/fmu/out/estimator_aid_src_ev_pos":
            msg = deserialize_message(raw, get_class(topic))
            age_s = (msg.timestamp - msg.timestamp_sample) * 1e-6
            aid_pos.append((msg.innovation_rejected, msg.fused, age_s))
        elif topic == "/fmu/out/estimator_aid_src_ev_hgt":
            msg = deserialize_message(raw, get_class(topic))
            age_s = (msg.timestamp - msg.timestamp_sample) * 1e-6
            aid_hgt.append((msg.innovation_rejected, msg.fused, age_s))
        elif topic == "/fmu/out/estimator_aid_src_ev_yaw":
            msg = deserialize_message(raw, get_class(topic))
            age_s = (msg.timestamp - msg.timestamp_sample) * 1e-6
            aid_yaw.append((msg.innovation_rejected, msg.fused, age_s))

    if not est or not gt:
        return None

    gx0, gy0 = gt[0][1], gt[0][2]
    gt_times = [g[0] for g in gt]
    divergences = []
    clearances = []
    for t, ex, ey, ez in est:
        idx = bisect.bisect_left(gt_times, t)
        best = None
        for cand in (idx - 1, idx):
            if 0 <= cand < len(gt):
                dt = abs(gt_times[cand] - t)
                if best is None or dt < best[0]:
                    best = (dt, gt[cand])
        if best is None or best[0] > 0.5:
            continue
        _, gx, gy, gz = best[1]
        gt_ned_x = gy - gy0
        gt_ned_y = gx - gx0
        divergences.append(math.sqrt((ex - gt_ned_x) ** 2 + (ey - gt_ned_y) ** 2))
        # clearance computed against RAW (world-frame) ground truth --
        # obstacle geometry is defined in the world's own absolute frame
        clearances.append(min_clearance(gx, gy, gz))

    if not divergences:
        return None

    runaway_path = os.path.join(results_dir, f"r2_2_{exp_id}", "runaway_event.json")
    had_runaway = os.path.isfile(runaway_path)
    min_clear = min(clearances) if clearances else None
    had_collision = min_clear is not None and min_clear <= 0.0

    def fusion_stats(aid):
        if not aid:
            return None
        n = len(aid)
        rej = sum(1 for r in aid if r[0])
        fused = sum(1 for r in aid if r[1])
        ages = [r[2] for r in aid]
        return {
            "n": n, "rejected_pct": 100 * rej / n, "fused_pct": 100 * fused / n,
            "mean_age_s": statistics.mean(ages), "max_age_s": max(ages),
        }

    return {
        "exp_id": exp_id,
        "n_divergence_samples": len(divergences),
        "max_divergence": max(divergences),
        "mean_divergence": statistics.mean(divergences),
        "had_runaway": had_runaway,
        "min_clearance": min_clear,
        "had_collision": had_collision,
        "ev_pos": fusion_stats(aid_pos),
        "ev_hgt": fusion_stats(aid_hgt),
        "ev_yaw": fusion_stats(aid_yaw),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--divergence-threshold", type=float, default=1.0,
                     help="Large-divergence reproduction rate threshold (m)")
    args = ap.parse_args()

    with open(args.manifest) as f:
        exp_ids = [line.strip() for line in f if line.strip()]

    by_cond = {c: [] for c in CONDITIONS}
    for exp_id in exp_ids:
        cond = re.sub(r"_r\d+$", "", exp_id)
        r = analyze_run(exp_id)
        if r is None:
            print(f"WARNING: {exp_id} produced no usable data", file=sys.stderr)
            continue
        by_cond[cond].append(r)
        ev_pos_str = (f"ev_pos_fused={r['ev_pos']['fused_pct']:.1f}% ev_pos_age={r['ev_pos']['mean_age_s']*1000:.1f}ms"
                      if r["ev_pos"] else "ev_pos_fused=N/A (no aid-source data)")
        ev_yaw_str = f"ev_yaw_fused={r['ev_yaw']['fused_pct']:.1f}%" if r["ev_yaw"] else "ev_yaw_fused=N/A"
        print(f"{r['exp_id']:8s} mean_div={r['mean_divergence']:.4f}m max_div={r['max_divergence']:.4f}m "
              f"runaway={r['had_runaway']} min_clear={r['min_clearance']:.3f}m collision={r['had_collision']} "
              f"{ev_pos_str} {ev_yaw_str}")

    print()
    print("=" * 100)
    print("Per-condition summary")
    print("=" * 100)
    for cond in CONDITIONS:
        runs = by_cond[cond]
        if not runs:
            print(f"  {cond}: no data")
            continue
        means = [r["mean_divergence"] for r in runs]
        maxes = [r["max_divergence"] for r in runs]
        n_runaway = sum(1 for r in runs if r["had_runaway"])
        n_collision = sum(1 for r in runs if r["had_collision"])
        n_large_div = sum(1 for r in runs if r["max_divergence"] > args.divergence_threshold)
        clearances = [r["min_clearance"] for r in runs]
        ages = [r["ev_pos"]["mean_age_s"] for r in runs if r["ev_pos"]]
        rejected = [r["ev_pos"]["rejected_pct"] for r in runs if r["ev_pos"]]
        print(f"\n-- {cond} (n={len(runs)}) --")
        print(f"  mean_divergence: mean={statistics.mean(means):.4f}m sd={statistics.stdev(means):.4f}m range=({min(means):.4f},{max(means):.4f})")
        print(f"  max_divergence:  mean={statistics.mean(maxes):.4f}m sd={statistics.stdev(maxes):.4f}m range=({min(maxes):.4f},{max(maxes):.4f})")
        print(f"  large-divergence (>{args.divergence_threshold}m) reproduction rate: {n_large_div}/{len(runs)} ({100*n_large_div/len(runs):.1f}%)")
        print(f"  runaway rate: {n_runaway}/{len(runs)} ({100*n_runaway/len(runs):.1f}%)")
        print(f"  collision rate: {n_collision}/{len(runs)} ({100*n_collision/len(runs):.1f}%)")
        print(f"  min_clearance: mean={statistics.mean(clearances):.3f}m min={min(clearances):.3f}m")
        print(f"  ev_pos message age: mean={statistics.mean(ages)*1000:.2f}ms")
        print(f"  ev_pos rejected%: mean={statistics.mean(rejected):.2f}%")

    print()
    print("=" * 100)
    print("Trend tests (OLS, condition coded L0=0/L1=1/L2=2)")
    print("=" * 100)
    for metric_name, extractor in [
        ("mean_divergence", lambda r: r["mean_divergence"]),
        ("ev_pos_message_age_ms", lambda r: r["ev_pos"]["mean_age_s"] * 1000),
        ("ev_pos_rejected_pct", lambda r: r["ev_pos"]["rejected_pct"]),
    ]:
        codes, ys = [], []
        for cond in CONDITIONS:
            for r in by_cond[cond]:
                if r["ev_pos"] is None:
                    continue
                codes.append(COND_CODE[cond])
                ys.append(extractor(r))
        if len(set(codes)) < 2:
            continue
        slope, intercept, r_val, p, se = stats.linregress(codes, ys)
        print(f"  {metric_name}: slope={slope:.5f}/level r={r_val:.4f} p={p:.4f} n={len(ys)}")

    rng = np.random.default_rng(20260907)
    n_boot = 20000
    arrs = {c: np.array([r["mean_divergence"] for r in by_cond[c]]) for c in CONDITIONS if by_cond[c]}
    if len(arrs) == 3:
        slopes = []
        for _ in range(n_boot):
            bc, by_ = [], []
            for c, arr in arrs.items():
                resampled = rng.choice(arr, size=len(arr), replace=True)
                bc.extend([COND_CODE[c]] * len(arr))
                by_.extend(resampled)
            s, *_ = stats.linregress(bc, by_)
            slopes.append(s)
        lo, hi = np.percentile(slopes, [2.5, 97.5])
        print(f"\n  mean_divergence bootstrap 95% CI on slope: ({lo:.5f}, {hi:.5f})")


if __name__ == "__main__":
    main()
