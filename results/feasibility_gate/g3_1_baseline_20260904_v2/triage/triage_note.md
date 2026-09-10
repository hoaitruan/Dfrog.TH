# Triage note

Bag: `/workspaces/isaac_ros-dev/results/feasibility_gate/g3_1_baseline_20260904_v2/rosbag`

**Scope reminder:** this is an OPEN-LOOP gate (`EKF2_EV_CTRL=0` the entire time). It measures raw cuVSLAM tracking error against `/ground_truth/odom`; it does not, and by design cannot, reproduce the original ~55m closed-loop runaway (that required EKF2 to be actively fusing a drifted vision pose into control).

## Mundane-cause table

| Observed pattern | Likely cause | Implication |
|---|---|---|
| Single sudden jump, coincides with tracking loss / relocalize | dropped or late frames, tracking loss | timing pathway; not the novel accuracy story |
| Smooth error proportional to distance | frame / yaw / extrinsic misconfig | mundane config bug |
| Runaway after EKF accepts a high-innovation measurement | estimator gating / tuning | N/A in this open-loop gate (EKF2_EV_CTRL=0) -- only observable closed-loop |
| Error grows with GPU load, graded, reproducible | contention-driven accuracy degradation | candidate target phenomenon |
| Coincides with low texture / low feature count regardless of load | texture confound | confound, not the effect |

## Classification for this bag

**proportional_to_distance**

Error grows roughly proportionally to distance travelled (mean error/distance ratio ~0.086, low spread) -- consistent with a frame/yaw/extrinsic misconfiguration rather than a load effect.

## EKF gating (closed-loop-only signal)

N/A by design: none of the `estimator_aid_src_ev_*` topics are present in this bag. PX4's EKF2 only advertises/populates them when the matching `EKF2_EV_CTRL` bit is set (EKF2.cpp:294-314); under this gate's mandated `EKF2_EV_CTRL=0`, they never exist on the bus. The "EKF accepted a high-innovation measurement" failure mode is observable only in a closed-loop run (out of scope for this gate -- would be M4/RQ3).

## Timestamp integrity

- sim-time span / wall-recv-time span (approx. realtime factor) = 0.697 over the window.

## Frame drops

- No frame-interval drops above threshold detected.

## Feature count

- min=138, mean=262, max=417 over 810 samples.

## Error summary

- 811 matched gt/vslam samples, final error=0.83m, max error=0.85m.

Plots (if generated): `position_yaw_error.png`, `frame_intervals.png`, `feature_count.png`, `gpu_alignment.png` (only if `--gpu-log` given).
