#!/usr/bin/env python3
"""
G1 forensic triage: classify a cuVSLAM/nvblox rosbag against the mundane-
cause table in the feasibility-gate task, for the OPEN-LOOP gate
(EKF2_EV_CTRL=0 throughout -- cuVSLAM is logged, never fused into control).

Must be run with ROS Humble sourced (needs rosbag2_py/rclpy):
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/ros2_ws/install/setup.bash
  python3 triage_incident.py --bag <path> [--start SEC] [--end SEC] \
      [--gpu-log <gpu_sampler.py CSV>] [--out-dir <dir>]

There is no historical incident bag in this repo (checked: the 4 existing
rosbags under ros2_ws/rosbags/ carry no /visual_slam/* topics at all --
they're Milestone-1 GPS-only flights). This script is meant to run against
a bag recorded during a G3 reproduction run via run_gate.sh, not the
original 2026-08 incident.

IMPORTANT SCOPING NOTE (do not remove): the original ~55m event was
CLOSED-LOOP (EKF2_EV_CTRL nonzero, PX4 actually fusing cuVSLAM's drifted
pose into control, which is what made it dangerous). This gate is
OPEN-LOOP (EKF2_EV_CTRL=0) by hard constraint, so it measures raw cuVSLAM
tracking error against ground truth -- it does not and cannot reproduce
the runaway itself. The "EKF accepted a high-innovation measurement"
failure mode is only observable closed-loop, because PX4's EKF2 only
advertises/populates the per-aid-source topics (estimator_aid_src_ev_pos/
_ev_hgt/_ev_yaw/_ev_vel) when the matching EKF2_EV_CTRL bit is set
(verified directly against EKF2.cpp:294-314) -- under EV_CTRL=0 those
topics never exist on the bus, so this script detects their absence and
reports that row as N/A by design, not as missing data to chase.
"""

import argparse
import bisect
import csv
import math
import os
import sys
from dataclasses import dataclass, field

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

VSLAM_ODOM_TOPIC = "/visual_slam/tracking/odometry"
VSLAM_STATUS_TOPIC = "/visual_slam/status"
VSLAM_FEATURES_TOPIC = "/visual_slam/vis/observations_cloud"
GT_ODOM_TOPIC = "/ground_truth/odom"
CLOCK_TOPIC = "/clock"
ESTIMATOR_STATUS_TOPIC = "/fmu/out/estimator_status_flags"
# Only ever present on a bag recorded while EKF2_EV_CTRL was nonzero (see
# module docstring) -- this gate keeps EV_CTRL=0, so these are expected
# absent. Listed so we can detect+report if one somehow shows up (e.g. a
# misconfigured run), rather than silently ignore it.
EV_AID_SRC_TOPICS = [
    "/fmu/out/estimator_aid_src_ev_pos",
    "/fmu/out/estimator_aid_src_ev_hgt",
    "/fmu/out/estimator_aid_src_ev_yaw",
    "/fmu/out/estimator_aid_src_ev_vel",
]

# cuVSLAM configured for 30Hz nominal (33.3ms); vslam.launch.py's own
# comment notes this dev machine actually runs ~36-40ms under load. Flag
# anything past 2x the nominal period as a dropped/late frame.
NOMINAL_FRAME_PERIOD_S = 1.0 / 30.0
FRAME_DROP_THRESHOLD_S = 2.0 * NOMINAL_FRAME_PERIOD_S


def yaw_from_quat(w, x, y, z):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


@dataclass
class OdomSample:
    t_sim: float       # header.stamp, seconds (sim time if use_sim_time honored)
    t_recv: float      # bag recv time, seconds (wall time of the recording process)
    x: float
    y: float
    z: float
    yaw: float


@dataclass
class BagData:
    topics_present: set = field(default_factory=set)
    vslam: list = field(default_factory=list)      # OdomSample
    gt: list = field(default_factory=list)          # OdomSample
    vo_state: list = field(default_factory=list)    # (t_recv, vo_state)
    feature_count: list = field(default_factory=list)  # (t_recv, width)
    clock: list = field(default_factory=list)       # (t_recv, sim_time_from_clock)
    ev_aid_src_present: list = field(default_factory=list)


def read_bag(bag_path, t_start, t_end):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    data = BagData(topics_present=set(type_map.keys()))

    for topic in EV_AID_SRC_TOPICS:
        if topic in data.topics_present:
            data.ev_aid_src_present.append(topic)

    msg_classes = {}

    def get_class(topic):
        if topic not in msg_classes:
            msg_classes[topic] = get_message(type_map[topic])
        return msg_classes[topic]

    bag_t0 = None
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        t_recv = t_ns * 1e-9
        if bag_t0 is None:
            bag_t0 = t_recv
        rel_t = t_recv - bag_t0
        if t_start is not None and rel_t < t_start:
            continue
        if t_end is not None and rel_t > t_end:
            continue

        if topic == VSLAM_ODOM_TOPIC:
            msg = deserialize_message(raw, get_class(topic))
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            data.vslam.append(OdomSample(
                t_sim=stamp_to_sec(msg.header.stamp), t_recv=t_recv,
                x=p.x, y=p.y, z=p.z, yaw=yaw_from_quat(o.w, o.x, o.y, o.z),
            ))
        elif topic == GT_ODOM_TOPIC:
            msg = deserialize_message(raw, get_class(topic))
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            data.gt.append(OdomSample(
                t_sim=stamp_to_sec(msg.header.stamp), t_recv=t_recv,
                x=p.x, y=p.y, z=p.z, yaw=yaw_from_quat(o.w, o.x, o.y, o.z),
            ))
        elif topic == VSLAM_STATUS_TOPIC:
            msg = deserialize_message(raw, get_class(topic))
            data.vo_state.append((t_recv, int(msg.vo_state)))
        elif topic == VSLAM_FEATURES_TOPIC:
            msg = deserialize_message(raw, get_class(topic))
            data.feature_count.append((t_recv, int(msg.width)))
        elif topic == CLOCK_TOPIC:
            msg = deserialize_message(raw, get_class(topic))
            data.clock.append((t_recv, stamp_to_sec(msg.clock)))

    return data, bag_t0


def align_and_compute_error(vslam, gt):
    """Origin-subtract each trajectory at its own first sample (same
    convention as vslam_compare.py -- cuVSLAM has no global reference), and
    for yaw compare *change from own start* rather than absolute yaw, since
    cuVSLAM's yaw origin is arbitrary (see visual_odometry_bridge.py
    docstring). Matches gt samples to vslam samples by nearest header
    timestamp within a generous tolerance.
    """
    if not vslam or not gt:
        return []

    gt_t0 = (gt[0].x, gt[0].y, gt[0].z, gt[0].yaw)
    vslam_t0 = (vslam[0].x, vslam[0].y, vslam[0].z, vslam[0].yaw)
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

        v_dyaw = wrap_angle(s.yaw - vslam_t0[3])
        g_dyaw = wrap_angle(g.yaw - gt_t0[3])
        yaw_err = abs(wrap_angle(v_dyaw - g_dyaw))

        dist_travelled = math.sqrt(gx ** 2 + gy ** 2 + gz ** 2)
        rows.append((s.t_recv, pos_err, yaw_err, dist_travelled))
    return rows


def classify_pattern(error_rows, feature_count, vo_state):
    """Heuristic-only classification against the task's mundane-cause
    table. Explicitly a heuristic: states what it observed, not a proof.
    """
    if len(error_rows) < 5:
        return "insufficient_data", "Fewer than 5 matched error samples -- cannot classify."

    errs = [r[1] for r in error_rows]
    dists = [r[3] for r in error_rows]
    max_err = max(errs)
    final_err = errs[-1]

    # Sudden jump: one step contributes most of the total error growth.
    steps = [abs(errs[i] - errs[i - 1]) for i in range(1, len(errs))]
    max_step = max(steps) if steps else 0.0
    total_growth = max(errs) - errs[0]
    sudden_jump = total_growth > 0.5 and max_step > 0.6 * total_growth

    # Tracking loss just before/at the largest step.
    tracking_loss_nearby = False
    if vo_state and steps:
        jump_idx = steps.index(max_step) + 1
        jump_t = error_rows[jump_idx][0]
        window = [v for (t, v) in vo_state if abs(t - jump_t) < 2.0]
        tracking_loss_nearby = any(v == 2 for v in window)  # 2 == Failed

    # Feature collapse anywhere in the window, independent of load.
    feature_collapse = False
    if feature_count:
        counts = [c for (_, c) in feature_count]
        feature_collapse = min(counts) < 0.2 * (sum(counts) / len(counts))

    # Smooth, roughly-proportional-to-distance growth.
    proportional = False
    if len(dists) > 5 and max(dists) > 1.0:
        # Exclude the near-origin regime: any fixed per-sample offset (e.g.
        # cuVSLAM's own tracking-init settling, or plain timestamp-matching
        # noise) dominates err/dist once dist is small, regardless of
        # whether a real proportional-to-distance mechanism is present --
        # a relative (not fixed-meter) floor so this scales with the
        # flight's own distance range instead of one absolute constant.
        d_floor = max(0.3, 0.1 * max(dists))
        ratios = sorted(e / d for e, d in zip(errs, dists) if d > d_floor)
        if len(ratios) > 3:
            mean_ratio = sum(ratios) / len(ratios)
            # Trimmed range (drop the extreme 10% each tail) instead of raw
            # min-max: a handful of residual noisy points near d_floor
            # shouldn't be able to single-handedly veto an otherwise tight
            # proportional relationship the way a bare max-min spread can.
            trim = max(1, len(ratios) // 10)
            trimmed = ratios[trim:-trim] if len(ratios) > 2 * trim else ratios
            spread = trimmed[-1] - trimmed[0]
            proportional = mean_ratio > 0 and spread < 0.6 * mean_ratio and not sudden_jump

    if sudden_jump and tracking_loss_nearby:
        return "sudden_jump_tracking_loss", (
            f"Single dominant error step ({max_step:.2f}m of {total_growth:.2f}m total "
            "growth) coinciding with a reported tracking-loss window -- "
            "consistent with dropped/late frames or a relocalize event, not a "
            "graded accuracy effect."
        )
    if feature_collapse:
        return "texture_confound", (
            "Tracked-feature count collapsed independent of any load signal in this "
            "bag -- consistent with a texture/low-feature confound rather than a "
            "load-driven accuracy effect."
        )
    if proportional:
        return "proportional_to_distance", (
            f"Error grows roughly proportionally to distance travelled (mean "
            f"error/distance ratio ~{mean_ratio:.3f}, low spread) -- consistent with "
            "a frame/yaw/extrinsic misconfiguration rather than a load effect."
        )
    if sudden_jump:
        return "sudden_jump_unexplained", (
            f"Single dominant error step ({max_step:.2f}m of {total_growth:.2f}m "
            "total growth) with no tracking-loss flag nearby in this bag -- jump "
            "shape without an obvious mundane cause; worth a closer manual look "
            "before treating it as reproducible."
        )
    return "graded_unclassified", (
        f"Error grew gradually to {final_err:.2f}m (max {max_err:.2f}m) without a "
        "single dominant step, proportional-to-distance signature, or feature "
        "collapse. If this tracks with a contention/load level in analyze_sweep.py's "
        "aggregation across repeats, it is the candidate target phenomenon -- a "
        "single bag alone cannot establish that."
    )


def check_timestamp_integrity(vslam):
    """Stale/repeated/decreasing header stamps, and how header-stamp (sim
    time, if use_sim_time is honored) delta tracks bag-recv (wall time)
    delta -- a large mismatch flags either a use_sim_time gap on the
    publishing node or a real-time-factor far from 1.0 (both worth knowing,
    neither by itself proof of a bug -- see vslam.launch.py's own note that
    this dev machine doesn't hold RTF=1.0 under load).
    """
    issues = []
    stale_or_decreasing = 0
    for i in range(1, len(vslam)):
        if vslam[i].t_sim <= vslam[i - 1].t_sim:
            stale_or_decreasing += 1
    if stale_or_decreasing:
        issues.append(
            f"{stale_or_decreasing} messages with stale/decreasing header.stamp "
            f"out of {len(vslam) - 1} consecutive pairs."
        )

    if len(vslam) > 2:
        sim_span = vslam[-1].t_sim - vslam[0].t_sim
        recv_span = vslam[-1].t_recv - vslam[0].t_recv
        if recv_span > 0:
            rtf = sim_span / recv_span
            issues.append(
                f"sim-time span / wall-recv-time span (approx. realtime factor) = "
                f"{rtf:.3f} over the window."
            )
            if rtf < 0.5 or rtf > 1.5:
                issues.append(
                    "  -> far from 1.0: either use_sim_time isn't honored somewhere "
                    "in this chain, or Gazebo genuinely isn't running at RTF~1 -- "
                    "check before trusting wall-clock-keyed correlations (e.g. the "
                    "GPU log) against this bag."
                )
    return issues


def check_frame_intervals(vslam):
    if len(vslam) < 2:
        return [], []
    intervals = [vslam[i].t_sim - vslam[i - 1].t_sim for i in range(1, len(vslam))]
    drops = [(vslam[i].t_recv, intervals[i - 1]) for i in range(1, len(vslam))
             if intervals[i - 1] > FRAME_DROP_THRESHOLD_S]
    return intervals, drops


def load_gpu_log(path):
    rows = []
    if not path:
        return rows
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            rows.append((float(row["t_wall"]), float(row["gpu_util_pct"])))
    return rows


def make_plots(out_dir, error_rows, intervals, feature_count, vo_state, gpu_log, bag_t0):
    os.makedirs(out_dir, exist_ok=True)

    if error_rows:
        t0 = error_rows[0][0]
        ts = [r[0] - t0 for r in error_rows]
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        ax1.plot(ts, [r[1] for r in error_rows])
        ax1.set_ylabel("position error vs ground truth (m)")
        ax1.set_title("cuVSLAM position error (origin-aligned)")
        ax2.plot(ts, [math.degrees(r[2]) for r in error_rows], color="tab:orange")
        ax2.set_ylabel("yaw error (deg, own-start-referenced)")
        ax2.set_xlabel("time since first matched sample (s)")
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "position_yaw_error.png"), dpi=130)
        plt.close(fig)

    if intervals:
        fig, ax = plt.subplots(figsize=(9, 3))
        ax.plot(range(len(intervals)), [i * 1000.0 for i in intervals])
        ax.axhline(FRAME_DROP_THRESHOLD_S * 1000.0, color="red", linestyle="--",
                   label=f"drop threshold ({FRAME_DROP_THRESHOLD_S*1000:.0f}ms)")
        ax.set_ylabel("frame interval (ms)")
        ax.set_xlabel("frame index")
        ax.legend()
        ax.set_title("cuVSLAM tracking/odometry frame intervals")
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "frame_intervals.png"), dpi=130)
        plt.close(fig)

    if feature_count:
        t0 = feature_count[0][0]
        fig, ax = plt.subplots(figsize=(9, 3))
        ax.plot([t - t0 for (t, _) in feature_count], [c for (_, c) in feature_count])
        ax.set_ylabel("tracked features (observations_cloud width)")
        ax.set_xlabel("time (s)")
        ax.set_title("cuVSLAM tracked-feature count")
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "feature_count.png"), dpi=130)
        plt.close(fig)

    if gpu_log and error_rows:
        t0 = min(error_rows[0][0], gpu_log[0][0])
        fig, ax1 = plt.subplots(figsize=(9, 4))
        ax1.plot([r[0] - t0 for r in error_rows], [r[1] for r in error_rows], color="tab:blue")
        ax1.set_ylabel("position error (m)", color="tab:blue")
        ax1.set_xlabel("time (s, wall-clock-aligned)")
        ax2 = ax1.twinx()
        ax2.plot([t - t0 for (t, _) in gpu_log], [u for (_, u) in gpu_log], color="tab:red", alpha=0.6)
        ax2.set_ylabel("GPU utilization (%)", color="tab:red")
        ax1.set_title("Position error vs GPU utilization (wall-clock aligned)")
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "gpu_alignment.png"), dpi=130)
        plt.close(fig)


CAUSE_TABLE = [
    ("Single sudden jump, coincides with tracking loss / relocalize",
     "dropped or late frames, tracking loss", "timing pathway; not the novel accuracy story"),
    ("Smooth error proportional to distance",
     "frame / yaw / extrinsic misconfig", "mundane config bug"),
    ("Runaway after EKF accepts a high-innovation measurement",
     "estimator gating / tuning",
     "N/A in this open-loop gate (EKF2_EV_CTRL=0) -- only observable closed-loop"),
    ("Error grows with GPU load, graded, reproducible",
     "contention-driven accuracy degradation", "candidate target phenomenon"),
    ("Coincides with low texture / low feature count regardless of load",
     "texture confound", "confound, not the effect"),
]


def write_triage_note(out_dir, bag_path, classification, explanation, ev_aid_src_present,
                       timestamp_issues, drops, feature_count, error_rows):
    path = os.path.join(out_dir, "triage_note.md")
    with open(path, "w") as f:
        f.write("# Triage note\n\n")
        f.write(f"Bag: `{bag_path}`\n\n")
        f.write(
            "**Scope reminder:** this is an OPEN-LOOP gate (`EKF2_EV_CTRL=0` the "
            "entire time). It measures raw cuVSLAM tracking error against "
            "`/ground_truth/odom`; it does not, and by design cannot, reproduce "
            "the original ~55m closed-loop runaway (that required EKF2 to be "
            "actively fusing a drifted vision pose into control).\n\n"
        )

        f.write("## Mundane-cause table\n\n")
        f.write("| Observed pattern | Likely cause | Implication |\n|---|---|---|\n")
        for row in CAUSE_TABLE:
            f.write(f"| {row[0]} | {row[1]} | {row[2]} |\n")
        f.write("\n")

        f.write("## Classification for this bag\n\n")
        f.write(f"**{classification}**\n\n{explanation}\n\n")

        f.write("## EKF gating (closed-loop-only signal)\n\n")
        if ev_aid_src_present:
            f.write(
                f"UNEXPECTED: {ev_aid_src_present} present in this bag despite the "
                "open-loop constraint -- EKF2_EV_CTRL may not have been 0 for this "
                "run. Re-check the run's config before trusting this bag as an "
                "open-loop gate sample.\n\n"
            )
        else:
            f.write(
                "N/A by design: none of the `estimator_aid_src_ev_*` topics are "
                "present in this bag. PX4's EKF2 only advertises/populates them "
                "when the matching `EKF2_EV_CTRL` bit is set (EKF2.cpp:294-314); "
                "under this gate's mandated `EKF2_EV_CTRL=0`, they never exist on "
                "the bus. The \"EKF accepted a high-innovation measurement\" "
                "failure mode is observable only in a closed-loop run "
                "(out of scope for this gate -- would be M4/RQ3).\n\n"
            )

        f.write("## Timestamp integrity\n\n")
        if timestamp_issues:
            for line in timestamp_issues:
                f.write(f"- {line}\n")
        else:
            f.write("- No stale/decreasing header stamps detected.\n")
        f.write("\n")

        f.write("## Frame drops\n\n")
        if drops:
            f.write(f"- {len(drops)} intervals exceeded {FRAME_DROP_THRESHOLD_S*1000:.0f}ms "
                    f"(worst: {max(d[1] for d in drops)*1000:.0f}ms).\n")
        else:
            f.write("- No frame-interval drops above threshold detected.\n")
        f.write("\n")

        f.write("## Feature count\n\n")
        if feature_count:
            counts = [c for (_, c) in feature_count]
            f.write(f"- min={min(counts)}, mean={sum(counts)/len(counts):.0f}, max={max(counts)} "
                    f"over {len(counts)} samples.\n")
        else:
            f.write("- No `/visual_slam/vis/observations_cloud` data in this bag "
                    "(check `enable_observations_view` was true, and that the topic "
                    "was included in the recording).\n")
        f.write("\n")

        f.write("## Error summary\n\n")
        if error_rows:
            errs = [r[1] for r in error_rows]
            f.write(f"- {len(errs)} matched gt/vslam samples, "
                    f"final error={errs[-1]:.2f}m, max error={max(errs):.2f}m.\n")
        else:
            f.write("- No matched gt/vslam samples (check both "
                    f"`{GT_ODOM_TOPIC}` and `{VSLAM_ODOM_TOPIC}` are in the bag "
                    "and overlap in time).\n")
        f.write("\nPlots (if generated): `position_yaw_error.png`, `frame_intervals.png`, "
                "`feature_count.png`, `gpu_alignment.png` (only if `--gpu-log` given).\n")
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True, help="Path to a ros2 bag directory (sqlite3 storage).")
    ap.add_argument("--start", type=float, default=None, help="Seconds into the bag to start (relative to first message).")
    ap.add_argument("--end", type=float, default=None, help="Seconds into the bag to stop.")
    ap.add_argument("--gpu-log", default=None, help="CSV from gpu_sampler.py, for wall-clock-aligned overlay.")
    ap.add_argument("--out-dir", default=None,
                    help="Output dir. Default: results/feasibility_gate/triage/<bag-basename>")
    args = ap.parse_args()

    if not os.path.exists(args.bag):
        print(f"ERROR: bag path does not exist: {args.bag}", file=sys.stderr)
        sys.exit(1)

    bag_name = os.path.basename(os.path.normpath(args.bag))
    out_dir = args.out_dir or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "results", "feasibility_gate", "triage", bag_name,
    )
    os.makedirs(out_dir, exist_ok=True)

    print(f"Reading bag: {args.bag}")
    data, bag_t0 = read_bag(args.bag, args.start, args.end)

    missing = [t for t in (VSLAM_ODOM_TOPIC, GT_ODOM_TOPIC) if t not in data.topics_present]
    if missing:
        print(f"WARNING: bag is missing required topic(s): {missing}. "
              "Classification will be degraded or empty.", file=sys.stderr)

    print(f"  vslam odom samples: {len(data.vslam)}")
    print(f"  ground-truth odom samples: {len(data.gt)}")
    print(f"  status samples: {len(data.vo_state)}")
    print(f"  feature-count samples: {len(data.feature_count)}")
    print(f"  EV aid-source topics present: {data.ev_aid_src_present or 'none (expected, open-loop)'}")

    error_rows = align_and_compute_error(data.vslam, data.gt)
    intervals, drops = check_frame_intervals(data.vslam)
    timestamp_issues = check_timestamp_integrity(data.vslam)
    classification, explanation = classify_pattern(error_rows, data.feature_count, data.vo_state)
    gpu_log = load_gpu_log(args.gpu_log)

    make_plots(out_dir, error_rows, intervals, data.feature_count, data.vo_state, gpu_log, bag_t0)
    note_path = write_triage_note(
        out_dir, args.bag, classification, explanation, data.ev_aid_src_present,
        timestamp_issues, drops, data.feature_count, error_rows,
    )

    print(f"\nClassification: {classification}")
    print(explanation)
    print(f"\nWrote: {note_path}")
    print(f"Plots in: {out_dir}")


if __name__ == "__main__":
    main()
