# Phase 0 — Explore (48h dual probe: H3 baseline vulnerability, H4 map churn)

Read-only exploration, no logging code written, no runs executed. Branch
`probe-48h` (from `main` @ `c5d6898`). All names below verified against
source at the paths/lines given — nothing here is guessed.

## 0a. Verified names

| What | Name / mechanism | Source (file:line) |
|---|---|---|
| ESDF query | **Service, not a topic**: `/nvblox_node/get_esdf_and_gradient` (`nvblox_msgs/srv/EsdfAndGradients`) | `nvblox_node.cpp:437,1664`; already consumed by this repo's `esdf_recorder.py:55` |
| ESDF pointcloud topic | `~/static_esdf_pointcloud` **exists but is dead code in this project's config** — see blocker below | `nvblox_node.cpp:363` (create), `nvblox_node.cpp:787` (only publish call site, inside a `k2D`-only branch) |
| Fast-Planner B-spline (control points) | Topic `/planning/bspline`, type `quadrotor_msgs/msg/Bspline` (`pos_pts`, `knots`, `traj_id`) | publish: `kino_replan_fsm.cpp:90` (create), `:755` (watchdog-hold republish), `:943` (real replan republish); subscribe (existing consumer, model for ours): `traj_server.cpp:34` |
| nvblox update-rate params | `integrate_depth_rate_hz` (depth integration), `update_esdf_rate_hz` (ESDF recompute) — **already exposed as launch args** from the feasibility-gate merge | `nvblox.launch.py:81-90,129-134`; gating logic `nvblox_node.cpp:564-573` (`shouldProcess`), call sites `:988-990` and `:627` |
| Disable nvblox updates | Set `integrate_depth_rate_hz<=0` (and/or `update_esdf_rate_hz<=0`) — `shouldProcess()` returns `false` unconditionally for `desired_frequency_hz<=0.f`, no core change needed | `nvblox_node.cpp:568-570` |
| Static/prebuilt map | `~/save_map` and `~/load_map` services (`nvblox_msgs/srv/FilePath`) already exist in `nvblox_ros` | `nvblox_node.cpp:421-427` |
| Depth stream (raw, pre-nvblox) | Topic `/depth_camera` (`sensor_msgs/msg/Image`) from the `ros_gz` bridge, remapped to nvblox's internal `camera_0/depth/image` inside the launch file | `run.sh:95` (bridge); `nvblox.launch.py:137-141` (remap) |

**Arm 0 (static map) recipe, fully config-level, no core change:** (1) fly once with nvblox running normally to build a map, (2) call `~/save_map`, (3) for every Arm-0 run, launch nvblox with `integrate_depth_rate_hz=0` + `update_esdf_rate_hz=0` and call `~/load_map` before the flight — `get_esdf_and_gradient` then returns the identical static grid all run, verified by `shouldProcess`'s own gating logic, not an assumption.

**Depth-fault injection, fully config-level:** a new node subscribes `/depth_camera`, republishes a corrupted copy on `/depth_camera/faulted`, and the launch invocation's existing remap target (`nvblox.launch.py:139`, currently `"/depth_camera"`) is swapped to `/depth_camera/faulted` for Corrupted-condition runs only. No cuVSLAM/nvblox core touched — cuVSLAM has its own separate stereo input, entirely unaffected by this depth-topic swap.

## 0b. Task 1 — H3 weight/confidence API feasibility (read-code only)

**PASS.** `TsdfVoxel` (the core voxel struct) already has a `weight` field —
`nvblox/map/voxels.h:26-32`: `float weight; // How many observations/how
confident we are in this observation.` (`EsdfVoxel`, by contrast, has no
weight field at all — only a bool `observed` — so weight only exists on
the TSDF side, not ESDF.)

This weight is **already read at the ROS-wrapper level**, not buried in
CUDA-only internals: `layer_publishing.cpp:108-114`
(`tsdfVoxelToRgb`) computes `v = clamp(voxel.weight / 5.0, 0, 1)` and
colormaps it for the existing TSDF-layer visualization topic, and
`TsdfVoxelFilter` (`layer_publishing.cpp:169-183`) already filters
voxels by a `min_tsdf_weight` threshold (also user-facing:
`layer_visualization_min_tsdf_weight` param, `node_params.hpp:168`).

**No existing service exports raw weight as a number** — only the
colormapped visualization exists (lossy, would need reverse-engineering
a colormap to approximate weight, not recommended). The clean path: a
new `nvblox_msgs/srv/*.srv` + a new service handler in `nvblox_node.cpp`
modeled directly on the existing `getEsdfAndGradientService`
(`nvblox_node.cpp:1664`, ~70 lines) but reading `tsdf_layer_` instead of
`esdf_layer_` and packing `(distance, weight)` per voxel instead of just
distance. This is a `nvblox_ros`/`nvblox_msgs` (wrapper) change — **it
does not touch `nvblox_core`** (the CUDA library itself), since the field
already exists there.

**Effort estimate: ~0.5-1 day** for someone already oriented in this
codebase (new `.srv` message + service handler closely modeled on an
existing one + rebuild `nvblox_msgs`+`nvblox_ros` + smoke test). One
concrete risk already documented in this repo: the existing
`get_esdf_and_gradient` service required `component_container_mt` to
avoid an unconditional single-threaded-executor deadlock
(`nvblox.launch.py`'s own comment, traced via `gdb`) — a new service
sharing the same task-queue pattern inherits that same requirement,
already satisfied by the current launch config, so this is a known risk,
not a new one. **Not implemented in this probe — feasibility only, per
instruction.**

## 0c. 15m straight path — feasibility and setup

**Can be built, with one judgment call flagged below (not decided
silently).** Fast-Planner's goal is **not** set by
`fast_planner_trigger.py`'s message content — its own docstring says so
explicitly (`fast_planner_trigger.py:6-8`: "target_type_==PRESET_TARGET
... uses the launch file's baked-in waypoint0_x/y/z instead"). The
trigger message only needs to exist to flip the FSM out of
`WAIT_TARGET`; the real goal is
`fast_planner_px4.launch.py:79-81`'s `fsm/waypoint0_x/y/z` launch
parameters, currently `(0.0, 8.0, 5.0)` — the same goal used throughout
this project's history (this is the `(0,8,5)` that runs directly through
`pillar_03` at `(0,2.5,5)`, per §26 of `flight_test_log.html`). Its 3D
distance from origin spawn is `sqrt(8²+5²)≈9.4m`, not 15m — a new goal is
needed. `(0, 15, 5)` gives `sqrt(15²+5²)≈15.8m`, closest clean value
without introducing a new heading; can be tuned to exactly 15m if wanted
(`(0, 14.14, 5)` → exactly 15.0m). Setting this requires **no code
change** — `waypoint0_x/y/z` are already plain launch parameters; a probe
script can override them at launch time the same way `nvblox.launch.py`'s
rate args are already overridden.

**World:** two static worlds already exist and are both usable without
building anything new:
- `PX4-Autopilot/Tools/simulation/gz/worlds/vio_test.sdf` — the obstacle
  world (8-pillar ring, 2.5m radius, centered at `(0,0,5)`, plus 16
  scattered markers further out). 100x100m ground plane.
- `PX4-Autopilot/Tools/simulation/gz/worlds/vio_test_r2_obstaclefree.sdf`
  — the obstacle-free world already used for R2-0/R2-1.

**Judgment call, flagged for correction if wrong:** H4 (map churn) and H3
(baseline vulnerability) plausibly want *different* worlds — H4 only
needs the drone moving so nvblox has new depth to (re)integrate, so the
obstacle-free world is cleaner (isolates churn from real obstacle-driven
replanning); H3 needs `d_min` to mean something, which requires an actual
obstacle, so it needs `vio_test.sdf`'s pillar ring. Proceeding with this
split (obstacle-free for H4, obstacle world for H3) at the same `(0,15,5)`-
style 15m goal in both, unless told otherwise — this is a config choice,
not a code change, so it costs nothing to revisit.

## Blockers

**One real blocker, with a stated workaround (per instruction: state it,
don't hide it, don't force a fix):**

- `~/static_esdf_pointcloud` (the "obvious" ESDF topic) never publishes in
  this project's mandatory `esdf_mode: "3d"` configuration — confirmed
  dead code, not a subscriber/QoS issue (this repo's own
  `esdf_recorder.py` already diagnosed and fixed this exact confusion,
  see its docstring). **Workaround, already proven in this repo:** poll
  `/nvblox_node/get_esdf_and_gradient` on a timer and reconstruct a
  voxel grid from the response, exactly as `esdf_recorder.py` already
  does. `churn_logger.py` (Phase 1) will be built this way, not as a
  topic subscriber.

No blocker on the B-spline topic (it exists and is directly subscribable
— `plan_drift_logger.py` needs no workaround). No blocker on disabling
nvblox updates, building a static map, or depth-fault injection — all
three are plain configuration, verified above.
