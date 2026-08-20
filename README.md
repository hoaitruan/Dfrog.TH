# Drone Perception + Navigation Pipeline — isaac_ros-dev (Humble container)

**Status: all 8 phases complete.** cuVSLAM (stereo-inertial VIO) → PX4 EKF2
external vision → nvblox (ESDF mapping) → Fast-Planner (kinodynamic
B-spline replanning), running end-to-end against PX4 SITL + Gazebo Harmonic,
entirely inside a Humble ROS 2 Docker container. A real flight was tested
with the drone climbing, replanning around an obstacle field with the
kinodynamic planner, and reaching its goal cleanly.

This document supersedes the project's original README, which covered an
earlier, narrower plan (RealSense D435i validated directly on the host,
against a ROS 2 Jazzy-side reactive planner). That approach was abandoned in
favor of the full-perception plan described below — see "Why the pivot"
below for the reasoning, kept for context.

## Why the pivot

The goal is for simulation to exercise the **same perception chain** that
will run on the real hardware (stereo+IMU → cuVSLAM), not a shortcut using
PX4's simulated ground-truth pose. cuVSLAM and nvblox are Humble-only and
use NITROS zero-copy transport, which breaks across ROS distro boundaries —
so PX4 SITL, Gazebo, and the whole perception stack all had to move into one
Humble container rather than splitting PX4 on the host (often Jazzy) from
perception in a separate container.

## Environment

- Host: Ubuntu 24.04, RTX 4060 Mobile (8GB VRAM — this is Isaac ROS's
  stated minimum; watch for memory pressure when cuVSLAM + nvblox + Gazebo
  run concurrently), NVIDIA driver + Container Toolkit (`--runtime nvidia`).
- Container: Humble-based Isaac ROS image, extra Dockerfile layer
  `isaac_ros_common/docker/Dockerfile.px4_gz` (Gazebo Harmonic + PX4 build
  deps), image key `x86_64.ros2_humble.realsense.px4_gz`.
- Run as a **persistent detached container**, not via `run_dev.sh` (which
  assumes an interactive TTY and fails here):
  ```
  docker run -d --name isaac_ros_dev_persistent \
    --privileged --runtime nvidia \
    --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh \
    <image> sleep infinity
  ```
  Stopping/starting the same container (`docker stop` / `docker start`)
  preserves everything; only `docker rm` loses anything installed via
  `apt-get` inside a running container instead of baked into the image.
- `PX4-Autopilot` is a **separate local clone** at
  `~/Drone/isaac_ros-dev/PX4-Autopilot` (own `build/px4_sitl_default`,
  deliberately not shared with any host-side PX4 checkout — Jammy vs Noble
  compiled objects would collide).

## Directory structure

```
~/Drone/isaac_ros-dev/                     <- bind-mounted into the container
│                                              at /workspaces/isaac_ros-dev
├── PX4-Autopilot/                         <- local clone, own SITL build
│   └── ROMFS/px4fmu_common/init.d-posix/airframes/
│       └── 4022_gz_x500_depth_stereo      <- custom airframe (stereo+IMU+depth rig)
├── ros2_ws/
│   ├── src/
│   │   ├── px4_vslam_bridge/              <- all custom bridge/glue nodes (below)
│   │   ├── fast_planner/                  <- ported Fast-Planner (8 packages)
│   │   ├── quadrotor_msgs/                <- Bspline.msg, PositionCommand.msg
│   │   ├── isaac_ros_nvblox/              <- built from source (release-3.2)
│   │   ├── px4_msgs/, px4_ros_com/        <- PX4 ROS 2 interop
│   │   ├── ros_gz/                        <- built from source (Harmonic support)
│   │   └── px4_offboard_test/             <- Phase 1 minimal offboard test
│   └── verification.rviz                  <- RViz2 config, see "Visualizing it" below
├── isaac_ros_assets/                      <- NGC quickstart rosbag (early cuVSLAM test)
└── src/isaac_ros_common/                  <- Dockerfile layers, run_dev.sh
```

## Architecture

```
Gazebo (SITL world, vio_test.sdf)
  │ stereo images + IMU + depth   (ros_gz bridge)
  ▼
┌─────────────────────────────────────────────────────────────┐
│ cuVSLAM (isaac_ros_visual_slam)                              │
│   -> /visual_slam/tracking/odometry   ("odom" frame, standalone) │
└─────────────────────────────────────────────────────────────┘
  │ visual_odometry_bridge.py (FRD, outlier-filtered)
  ▼
PX4 EKF2 (external vision fusion, EKF2_EV_CTRL)  ─┐
  │ /fmu/out/vehicle_odometry (GPS-based, "ground truth" for our purposes)
  ▼                                                │
ground_truth_tf.py  ── map -> base_link_gt -> camera_link_optical_gt (TF)
  │                                                │
  ▼                                                │
nvblox (NvbloxNode, component_container_mt)        │
  -> ESDF (get_esdf_and_gradient service, + mesh/pointcloud for RViz)
  │                                                │
  ▼                                                │
Fast-Planner (fast_planner_node + traj_server)      │
  -> quadrotor_msgs/PositionCommand (/planning/pos_cmd)
  │                                                │
  ▼                                                ▼
fast_planner_bridge.py  ────────────────>  PX4 TrajectorySetpoint
  (pos + vel + acc + yaw + yawspeed feedforward)   (Offboard control)
```

**Key design point (Phase 6 safety fix):** cuVSLAM is tracked but **not
trusted for control** (`EKF2_EV_CTRL 0`). PX4 flies on GPS alone; nvblox and
everything downstream of it is rooted in PX4's own ground-truth pose chain
(`base_link_gt`), not cuVSLAM's `odom` frame. cuVSLAM's own TF tree
(`odom -> base_link -> stereo_imu_link_*`) still runs standalone,
disconnected from `map`, for its own verification purposes only.

## Custom nodes (`px4_vslam_bridge`)

| Node | Role |
|---|---|
| `ground_truth_tf` | Publishes `map->base_link_gt` (dynamic, from PX4 odometry) + two static TF chains (cuVSLAM's camera mount, and the ground-truth-rooted one nvblox/Fast-Planner use). Must run with `use_sim_time:=true`. |
| `visual_odometry_bridge` | Republishes cuVSLAM odometry as `px4_msgs/VehicleOdometry` (POSE_FRAME_FRD) for EKF2 external vision fusion, with a velocity-jump outlier filter. |
| `reactive_esdf_avoidance` | Phase 6: 20Hz offboard setpoint loop + ~3Hz async `get_esdf_and_gradient` query, potential-field avoidance (attract-to-goal / repel-from-nearest-obstacle). No planning ahead — reactive only. |
| `fast_planner_bridge` | Phase 7: relays `quadrotor_msgs/PositionCommand` into PX4 `TrajectorySetpoint`, full pos+vel+acc+yaw+yawspeed feedforward, plus a `POS_CMD_MAX_AGE_S=0.5` staleness guard. |
| `fast_planner_trigger` | Publishes the one-shot `nav_msgs/Path` that starts Fast-Planner's FSM, waiting for real subscriber discovery first (fixes a DDS one-shot-publish race). |
| `vslam_compare` | cuVSLAM-vs-ground-truth error comparison, used for Phase 3's gate check. |

## Running it (inside the container)

Always `docker exec -u admin` — see "Operational gotchas" below for why
root breaks this silently.

```bash
docker exec -u admin -it isaac_ros_dev_persistent bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/ros2_ws/install/setup.bash

# 0. uXRCE-DDS Agent -- the PX4<->ROS2 bridge. NOT started by any launch
#    file; without it PX4 boots fine but zero /fmu/* topics ever appear
#    on the ROS2 side, with no error on either side.
/workspaces/isaac_ros-dev/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888 &

# 1. PX4 SITL + Gazebo (vio_test.sdf world)
#    use `< <(tail -f /dev/null)` as stdin -- see pxh gotcha below
make px4_sitl gz_x500_depth_stereo_vio_test < <(tail -f /dev/null)

# 2. Ground truth TF (REQUIRED: use_sim_time)
ros2 run px4_vslam_bridge ground_truth_tf --ros-args -p use_sim_time:=true

# 3. cuVSLAM
ros2 launch px4_vslam_bridge vslam.launch.py

# 4. nvblox
ros2 launch px4_vslam_bridge nvblox.launch.py

# 5a. Reactive-only avoidance (Phase 6), OR:
ros2 run px4_vslam_bridge reactive_esdf_avoidance
# 5b. Fast-Planner kinodynamic replanning (Phase 7):
ros2 launch plan_manage fast_planner_px4.launch.py
ros2 run px4_vslam_bridge fast_planner_bridge
ros2 run px4_vslam_bridge fast_planner_trigger   # triggers the flight
```

**Safe-testing protocol** (established after real flight incidents, see
below): always climb to a stable, verified hover first using the proven
arm/climb pattern, confirm real position via Gazebo ground truth if
anything looks off, and only *then* trigger a new/unvalidated behavior.
Force-disarm (`VEHICLE_CMD_COMPONENT_ARM_DISARM`, param2=21196) immediately
on any dangerous divergence. **Before arming after any restart**, confirm
the vehicle actually spawned near true origin (`/fmu/out/vehicle_local_position_v1`)
— see the `CLIMB_TARGET_NED` open item below for why this matters.

## Visualizing it (RViz2)

Config at `ros2_ws/verification.rviz`, launched inside the container:
```bash
xhost +local:docker   # on the HOST, once per host session
docker exec -d -u admin isaac_ros_dev_persistent bash -c '
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/ros2_ws/install/setup.bash
  ros2 run rviz2 rviz2 -d /workspaces/isaac_ros-dev/ros2_ws/verification.rviz \
    --ros-args -p use_sim_time:=true'
```

| Phase | Display | Topic | Notes |
|---|---|---|---|
| 2 | Image ×2 | `/stereo/left/image_mono8`, `/stereo/right/image_mono8` | right disabled by default |
| 2 | PointCloud2 | `/depth_camera/points` | |
| 2 | TF | — | |
| 3 | Odometry + Path | `/visual_slam/tracking/odometry`, `/visual_slam/tracking/slam_path` | **disabled by default** — needs Fixed Frame = `odom`, see below |
| 3 | PointCloud2 | `/visual_slam/vis/observations_cloud` | same — needs Fixed Frame = `odom` |
| 3 | Odometry (ground truth) | `/ground_truth/odom` | Fixed Frame = `map` |
| 5 | NvbloxMesh (`nvblox_rviz_plugin`) | `/nvblox_node/mesh` | **disabled by default, see below** |
| 5 | PointCloud2 | `/nvblox_node/static_esdf_pointcloud` | lazy-publish, populates once subscribed |
| 6 | — | — | **not published** — `reactive_esdf_avoidance.py` has no Path/MarkerArray output; would need adding if wanted |
| 7 | Marker ×3 | `/planning/travel_traj`, `/planning_vis/trajectory`, `/planning/position_cmd_vis` | single `Marker`, not `MarkerArray` |

**Fixed-Frame split:** cuVSLAM's `odom` TF tree is intentionally
disconnected from `map` (the Phase 6 safety fix rooted nvblox/Fast-Planner
to PX4 ground truth instead). RViz has one global Fixed Frame, so Phase 3's
cuVSLAM-only displays must be checked with Fixed Frame switched to `odom`;
everything else uses `map`.

**RViz2 crashes on real mesh data (`nvblox_rviz_plugin`'s NvbloxMesh
display):** `Ogre::ItemIdentityException: Unable to locate geometry
program called rviz/glsl150/box.geom` — a missing/unregistered Ogre media
resource in this build, confirmed reproducible in isolation with the Mesh
display as the only one enabled. Disabled by default in `verification.rviz`
as a result. **Caution:** the same crash recurred once in a later session
with the Mesh display already disabled (~80s after launch, cause not
re-isolated) — so it may not be exclusively the Mesh display's doing, just
its most reliable trigger. Treat RViz2 as disposable: it doesn't affect
the actual pipeline (PX4/cuVSLAM/nvblox/Fast-Planner keep running fine
through an RViz2 crash) — if it dies, just relaunch it.

**Docker/X11:** `xhost +local:docker` on the host is required — X11
authorization otherwise fails (`Authorization required, but no
authorization protocol specified`) even with the socket bind-mounted.
**This grant does not persist** across an X server restart/host
reboot/sleep — re-run it once per host session before launching any GUI.

## Phase-by-phase summary

**Phase 0 — Humble container + PX4 SITL + Gazebo.** Biggest fix: a
protobuf 26.0-vs-3.12.4 ABI collision (Isaac ROS's own PyTorch build vs.
`gz-msgs10`) across 4 independent resolution paths, fixed by relocating the
newer protobuf to `/opt/protobuf26-hidden/`. Also: `/bin/sh` in the base
image was a broken custom shim, repointed to `/bin/dash`; `pxh`'s busy-loop
under `docker exec` needs a stdin that blocks forever
(`< <(tail -f /dev/null)`), not `< /dev/null`.

**Phase 1 — uXRCE-DDS + basic Offboard.** `px4_msgs`/`px4_ros_com`/
`Micro-XRCE-DDS-Agent` built in-workspace (`UAGENT_USE_SYSTEM_FASTCDR=OFF`,
since Humble's `ros-humble-fastcdr` is too old). Minimal arm+climb+hold
converged to ~2cm using PX4's own sim pose.

**Phase 2 — Stereo+IMU+depth Gazebo rig.** Custom `StereoIMU` and
`x500_depth_stereo` models + `4022_gz_x500_depth_stereo` airframe. `ros_gz`
built from source (Harmonic isn't supported by Humble's prebuilt bridge).

**Phase 3 — cuVSLAM standalone verification.** Three real bugs: (1)
`camera_optical_frames` pointed both cameras at the same TF frame — cuVSLAM
derives stereo baseline from TF between the two frames, not camera_info, so
this gave a degenerate zero-baseline stereo pair and permanently zero
features. (2) Two nodes both publishing a different parent for `base_link`
corrupted the shared TF buffer — renamed the ground-truth node's output to
`base_link_gt`. (3) The test world was texture-sparse (solid-color props)
and geometrically out of the camera's FOV cone at hover altitude — added a
real textured checkerboard and a close-range pillar ring at hover height.
Result: 328 tracked features, ~0.2–0.25m steady-state error vs. ground
truth.

**Phase 4 — cuVSLAM → PX4 EKF2 external vision.** `visual_odometry_bridge`
republishes as `VehicleOdometry` in `POSE_FRAME_FRD` (cuVSLAM's `odom` frame
has arbitrary yaw, not North-aligned). `EKF2_EV_CTRL 11` (position + yaw,
no velocity — cuVSLAM's twist field is unpopulated). GPS aiding stays on
simultaneously so a fusion bug shows as a mismatch, not silent divergence.

**Phase 5 — nvblox ESDF mapping.** Root cause of an indefinite
`get_esdf_and_gradient` hang, found via `gdb -p <pid> -batch -ex 'thread
apply all bt'`: an **unconditional deadlock by nvblox_ros's own design**
under a single-threaded executor — the service handler blocks the same
thread that would need to drain its own task queue. Fixed by using
`component_container_mt`. Also: `esdf_mode` forced to `"3d"` (the `"2d"`
default's `static_map_slice` only covers a ground-level height band,
useless for a hovering drone).

**Phase 6 — Reactive ESDF avoidance (MILESTONE), with a real safety
incident.** First cruising flight test caused cuVSLAM tracking to drift
~55m from true position under GPU contention with nvblox's concurrent CUDA
context, while EKF2 was still fusing it — PX4 chased the phantom error with
real thrust. Caught and force-disarmed before damage. **Fix:** stopped
trusting cuVSLAM for control entirely (`EKF2_EV_CTRL 0`, GPS-only); nvblox
re-rooted to a new ground-truth-based TF chain. A second bug found in the
same pass: the ground-truth TF node wasn't declaring `use_sim_time`, so its
wall-clock-stamped TF silently never matched sim-clock-stamped depth
images — zero ESDF data, zero errors logged. Fixed by launching it with
`use_sim_time:=true`.

**Phase 7 — Fast-Planner (kinodynamic B-spline replanning).** Ported 8
packages from a WIP Foxy community fork
(`RohitPawar2406/Fast-Planner-ROS2`), fixing 5 build bugs (missing
`quadrotor_msgs` port, `double`-as-Eigen-index under newer Eigen, a missing
`pcl_conversions` dependency, an NLopt CMake config-mode lookup needing an
explicit `-DNLopt_DIR`, and missing transitive `ament_export_dependencies`
in `plan_env`). Three flight-safety incidents during integration testing,
all caught live and force-disarmed:
1. **Stale-plan chase** — a one-shot `ros2 topic pub --once` re-trigger
   silently lost the DDS discovery race, so the bridge kept chasing an old
   trajectory. Fixed with a `POS_CMD_MAX_AGE_S=0.5` freshness guard plus
   `fast_planner_trigger` waiting for real subscriber match.
2. **PX4 failure-detector latch** — `fd_critical_failure` doesn't self-clear
   from a calm hover after a violent excursion; needs a full restart.
3. **Open-loop replanning divergence (the real bug)** — Fast-Planner's
   `REPLAN_TRAJ` state sources its next start state from evaluating the
   *previous* B-spline at the current time, not real odometry feedback —
   valid only with tight trajectory tracking, which position-only control
   didn't provide. Confirmed via ground truth: altitude swung from -5m
   through ground level. **Fixed** by adding full velocity + acceleration +
   yawspeed feedforward to `fast_planner_bridge`. After the fix: clean
   convergence to goal, <3cm jitter.

**Two more incidents found in a later full end-to-end retest**, both
during the plain climb phase (before Fast-Planner's own trajectory was
even involved):

4. **`OffboardControlMode` heartbeat/data mismatch** — incident #3's fix
   made `_offboard_mode_msg()` unconditionally claim `velocity=True` and
   `acceleration=True` on every heartbeat, but the climb/hold phases still
   send NaN velocity/acceleration (position-only). Claiming a field valid
   to PX4 while sending NaN in it is undefined behavior, not "ignored" —
   caused a real runaway (peak ~47m position error) during a plain climb
   to a fixed point. **Fixed** by publishing the heartbeat atomically with
   each setpoint, in `_publish_trajectory_setpoint`, flagging
   velocity/acceleration valid only when real (non-NaN) data is actually
   being sent that tick.
5. **`CLIMB_TARGET_NED` is an absolute point, not relative to spawn** —
   this is the fix in #4 turned out to be necessary-but-insufficient: a
   *second* runaway (peak ~44m error) happened even after the fix, because
   the drone had drifted to `(x≈0, y≈7)` from an earlier test in the same
   long-running container, turning the climb into an untested ~8-11m
   diagonal step command that PX4's default offboard position controller
   handled by oscillating hard rather than converging. A full PX4/Gazebo
   restart (fresh spawn at true origin) resolved it without any further
   code change — the same climb-to-`(0,0,-5)` command is normally only a
   trivial ~5m vertical move. **Not fixed in code** — see the open item
   below. Both incidents were caught live via ground truth and
   force-disarmed before damage; the retry after a clean restart converged
   smoothly and the full flight (climb → Fast-Planner trajectory → goal)
   succeeded cleanly, holding <10cm jitter at the goal for 10+ seconds.

## Operational gotchas (container restarts / GUI)

- **`docker exec` defaults to root; the whole pipeline runs as `admin`.**
  Root-context ROS2 CLI calls (and GUI apps) silently receive zero DDS
  data — no errors, topics just never deliver. Made the pipeline look
  completely stalled during a health check that was actually fine, and
  meant an RViz2 instance launched as root sat rendering nothing the
  entire time it ran. Always `docker exec -u admin`.
- **The container can die unexpectedly.** Observed one `Exited (137)`
  (SIGKILL) after a long idle gap between sessions — `OOMKilled: false`
  per Docker, cause not confirmed (possibly a host sleep/restart).
  Recover with `docker start isaac_ros_dev_persistent`; the bind-mounted
  workspace and build artifacts survive, but every process inside
  (PX4, Gazebo, MicroXRCEAgent, all ROS2 nodes) does not — the entire
  "Running it" sequence above needs to be redone from scratch, including
  the Agent.
- **Window placement:** `xdotool windowmove`/`windowsize` are silently
  ignored by mutter (GNOME's compositor) for these windows. Use `wmctrl -e`
  instead, which uses the proper `_NET_MOVERESIZE_WINDOW` protocol. Gazebo
  and RViz2 windows may also launch already maximized, which silently
  overrides any resize until you first
  `wmctrl -b remove,maximized_vert,maximized_horz` and let it settle
  (~1s) before resizing — otherwise the two windows land stacked directly
  on top of each other, and mouse input meant for one goes to the other.

## Known limitations / open items

- **`CLIMB_TARGET_NED` in `fast_planner_bridge.py` is a hardcoded absolute
  NED point, not relative to wherever the drone actually spawns.** Root
  cause of incident #5 above. Safe as long as PX4/Gazebo were freshly
  restarted (spawns at true origin), but a real latent risk in any
  long-running session where the drone has drifted from a prior test —
  worth making relative-to-spawn or interpolated before relying on this
  again without a fresh restart first.
- **Phase 6 has no RViz visualization** — `reactive_esdf_avoidance.py`
  publishes no Path/MarkerArray. Would need adding if visual verification
  of that phase specifically is wanted.
- **cuVSLAM is not flight-safe under concurrent nvblox GPU load** on this
  8GB laptop GPU — architecturally worked around (GPS-only control) rather
  than fixed. Worth re-testing on target Jetson hardware, which won't share
  a GPU with a desktop-class simultaneous workload the same way.
  Real-hardware D435i stereo-inertial mode also previously hit a USB
  Motion-Module lockup in-container (RSUSB backend vs. host kernel driver
  binding) — not yet root-caused, may be worth revisiting against actual
  Jetson USB controller/kernel before relying on it.
- **Fast-Planner's local-minimum behavior** near closely-packed obstacle
  clusters (seen in Phase 6's simpler reactive planner, largely but not
  fully resolved by Phase 7's kinodynamic search) hasn't been stress-tested
  against denser or more adversarial obstacle fields.
