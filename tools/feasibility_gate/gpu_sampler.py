#!/usr/bin/env python3
"""
G2 instrumentation: samples GPU utilization/memory over time to a CSV
keyed by wall-clock time, for alignment against a rosbag's own wall-clock
message-recv timestamps (see triage_incident.py's --gpu-log).

Desktop backend (default) shells out to `nvidia-smi` once per tick --
confirmed present both on the host and inside isaac_ros_dev_persistent
(RTX 4060 Laptop, driver 580.173.02). Runs standalone; does not need ROS
sourced.

Jetson backend is a stub, not implemented: this project's target hardware
is a Jetson Orin Nano, which has no `nvidia-smi` (embedded Tegra parts use
`tegrastats` instead, with a different text format -- percent-of-shared
memory-controller-bandwidth semantics, not a discrete-GPU utilization
number). Left unimplemented rather than guessed, since untested against
real tegrastats output would just be a plausible-looking fabrication --
see ARCHITECTURE.md's Milestone 2 note that Jetson GPU-contention behavior
is explicitly unverified against this laptop's results.

Usage:
  python3 gpu_sampler.py --out results/feasibility_gate/<exp_id>/gpu_log.csv \
      --interval 0.2 [--duration 60]
  (Ctrl-C stops it early if --duration is omitted; the CSV is flushed per
  row, so a partial run is still usable.)
"""

import argparse
import csv
import re
import subprocess
import sys
import time

NVIDIA_SMI_QUERY = "utilization.gpu,memory.used,memory.total,temperature.gpu"
CSV_FIELDS = ["t_wall", "gpu_util_pct", "mem_used_mib", "mem_total_mib", "temp_c"]


def sample_nvidia_smi():
    """One nvidia-smi call, parsed. Returns None on any failure (GPU busy,
    driver hiccup, etc.) rather than raising -- a missed sample shouldn't
    kill a long sweep run."""
    try:
        out = subprocess.run(
            ["nvidia-smi", f"--query-gpu={NVIDIA_SMI_QUERY}",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=2.0,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        print(f"gpu_sampler: nvidia-smi call failed: {e}", file=sys.stderr)
        return None
    if out.returncode != 0:
        print(f"gpu_sampler: nvidia-smi exited {out.returncode}: {out.stderr.strip()}", file=sys.stderr)
        return None
    line = out.stdout.strip().splitlines()[0] if out.stdout.strip() else ""
    parts = [p.strip() for p in line.split(",")]
    if len(parts) != 4:
        print(f"gpu_sampler: unexpected nvidia-smi output: {line!r}", file=sys.stderr)
        return None
    try:
        util, mem_used, mem_total, temp = (float(re.sub(r"[^\d.]", "", p)) for p in parts)
    except ValueError:
        print(f"gpu_sampler: could not parse nvidia-smi output: {line!r}", file=sys.stderr)
        return None
    return util, mem_used, mem_total, temp


def sample_tegrastats():
    raise NotImplementedError(
        "Jetson backend not implemented -- see module docstring. Needs a real "
        "Orin Nano to develop/validate tegrastats parsing against; do not guess "
        "the format here."
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="CSV output path.")
    ap.add_argument("--interval", type=float, default=0.2, help="Seconds between samples (default 0.2 = 5Hz).")
    ap.add_argument("--duration", type=float, default=None, help="Seconds to run. Omit to run until Ctrl-C.")
    ap.add_argument("--backend", choices=["nvidia-smi", "tegrastats"], default="nvidia-smi")
    args = ap.parse_args()

    sample_fn = sample_nvidia_smi if args.backend == "nvidia-smi" else sample_tegrastats

    with open(args.out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        f.flush()

        t_start = time.time()
        n = 0
        print(f"gpu_sampler: writing to {args.out} every {args.interval}s "
              f"({'until Ctrl-C' if args.duration is None else f'for {args.duration}s'})...")
        try:
            while True:
                t_wall = time.time()
                if args.duration is not None and t_wall - t_start > args.duration:
                    break
                sample = sample_fn()
                if sample is not None:
                    util, mem_used, mem_total, temp = sample
                    writer.writerow({
                        "t_wall": f"{t_wall:.6f}",
                        "gpu_util_pct": util,
                        "mem_used_mib": mem_used,
                        "mem_total_mib": mem_total,
                        "temp_c": temp,
                    })
                    f.flush()
                    n += 1
                sleep_for = args.interval - (time.time() - t_wall)
                if sleep_for > 0:
                    time.sleep(sleep_for)
        except KeyboardInterrupt:
            pass

    print(f"gpu_sampler: wrote {n} samples to {args.out}")


if __name__ == "__main__":
    main()
