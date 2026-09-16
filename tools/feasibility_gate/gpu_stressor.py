#!/usr/bin/env python3
"""
G2 instrumentation: standalone synthetic CUDA load generator, for the
`--workload synthetic` arm of run_gate.sh. Its job is to reproduce
GPU-scheduling contention against cuVSLAM/nvblox's own CUDA context
without depending on nvblox actually being configured a certain way --
useful to separate "cuVSLAM degrades under GPU contention in general" from
"cuVSLAM degrades specifically under nvblox's own kernel/memory pattern"
(the `--workload nvblox` arm, driven instead by raising
`integrate_depth_rate_hz` -- see run_gate.sh).

Uses PyTorch (confirmed present in the isaac_ros_dev_persistent container,
2.13.0+cu130, torch.cuda.is_available()=True) doing repeated large matmuls
across N concurrent CUDA streams. Deliberately not pycuda/numba -- neither
is installed in this container, and installing new packages into a
carefully-pinned Isaac ROS image is out of scope for this tooling task.

Intensity presets are approximate, uncalibrated GPU-load levels, not a
guarantee of a specific utilization percentage -- calibrate against
`gpu_sampler.py`'s own readings on this specific 8GB GPU before treating
the level labels as meaningful across machines (explicitly a G3 prep step,
not done here).

Usage:
  python3 gpu_stressor.py --intensity medium --duration 60
  python3 gpu_stressor.py --matrix-size 4096 --streams 4 --duty-cycle 0.8 --duration 60
"""

import argparse
import sys
import time

try:
    import torch
except ImportError:
    print("gpu_stressor: PyTorch not importable -- run this inside "
          "isaac_ros_dev_persistent, not on the host.", file=sys.stderr)
    sys.exit(1)

# (matrix_size, num_streams, duty_cycle). duty_cycle is the fraction of
# each ~0.1s tick spent issuing matmuls vs idle -- gives a coarse extra
# knob beyond just raising matrix size, without needing per-GPU
# calibration to land on a specific utilization number.
INTENSITY_PRESETS = {
    "none": (256, 1, 0.0),
    "low": (1024, 1, 0.3),
    "medium": (2048, 2, 0.6),
    "high": (3072, 4, 0.9),
    "extreme": (4096, 4, 1.0),
}

TICK_S = 0.1


def run(matrix_size, num_streams, duty_cycle, duration):
    if not torch.cuda.is_available():
        print("gpu_stressor: torch.cuda.is_available() is False -- aborting.", file=sys.stderr)
        sys.exit(1)

    device = torch.device("cuda")
    streams = [torch.cuda.Stream() for _ in range(num_streams)]
    tensors = [
        (torch.randn(matrix_size, matrix_size, device=device),
         torch.randn(matrix_size, matrix_size, device=device))
        for _ in range(num_streams)
    ]

    print(f"gpu_stressor: matrix_size={matrix_size} streams={num_streams} "
          f"duty_cycle={duty_cycle} duration={'until Ctrl-C' if duration is None else f'{duration}s'}")

    t_start = time.time()
    iters = 0
    try:
        while True:
            now = time.time()
            if duration is not None and now - t_start > duration:
                break
            tick_end = now + TICK_S
            busy_until = now + TICK_S * duty_cycle
            while time.time() < busy_until:
                for s, (a, b) in zip(streams, tensors):
                    with torch.cuda.stream(s):
                        c = a @ b
                        a.copy_(c)
                iters += 1
            for s in streams:
                s.synchronize()
            sleep_for = tick_end - time.time()
            if sleep_for > 0:
                time.sleep(sleep_for)
    except KeyboardInterrupt:
        pass

    elapsed = time.time() - t_start
    print(f"gpu_stressor: stopped after {elapsed:.1f}s, {iters} matmul batches issued.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    preset_group = ap.add_mutually_exclusive_group()
    preset_group.add_argument("--intensity", choices=list(INTENSITY_PRESETS), default=None,
                               help="Named preset (matrix size/streams/duty-cycle).")
    ap.add_argument("--matrix-size", type=int, default=None, help="Override: square matmul size.")
    ap.add_argument("--streams", type=int, default=None, help="Override: concurrent CUDA streams.")
    ap.add_argument("--duty-cycle", type=float, default=None, help="Override: fraction of each 0.1s tick spent busy (0-1).")
    ap.add_argument("--duration", type=float, default=None, help="Seconds to run. Omit to run until Ctrl-C.")
    args = ap.parse_args()

    if args.intensity:
        matrix_size, streams, duty_cycle = INTENSITY_PRESETS[args.intensity]
    else:
        matrix_size, streams, duty_cycle = INTENSITY_PRESETS["medium"]
    matrix_size = args.matrix_size if args.matrix_size is not None else matrix_size
    streams = args.streams if args.streams is not None else streams
    duty_cycle = args.duty_cycle if args.duty_cycle is not None else duty_cycle

    run(matrix_size, streams, duty_cycle, args.duration)


if __name__ == "__main__":
    main()
