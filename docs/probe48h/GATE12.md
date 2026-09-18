# H4 config-fix + existence gate — 2026-09-18

Branch `probe-48h`. Purpose: fix two config red flags that would
otherwise produce a **false null** for H4 (no depth noise → no physical
source of churn at all; ground-truth logger reading an EKF2 estimate
instead of physics → churn/drift measured against the wrong reference),
then run a small (n=4/arm) existence probe — not the batch.

## Phase A — Inspect

### A1. Depth sensor noise (critical gate)

**File:** `PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite/model.sdf`,
sensor `StereoOV7251` (`type="depth_camera"`, lines 55-87), the sensor
that ultimately feeds nvblox — reached via
`x500_depth_stereo/model.sdf` → `<include>x500_depth</include>` →
`x500_depth/model.sdf` → `<include>model://OakD-Lite</include>`.

**Finding: no `<noise>` tag anywhere in this sensor's `<camera>` block.**
Just `<horizontal_fov>`, `<image>` (640x480, `R_FLOAT32`), `<clip>`
(0.2-19.1m), `<update_rate>30</update_rate>`. The depth stream is
**perfectly noiseless** in the current config — confirmed by absence,
not inferred. This alone would make an H4 churn-from-noise effect
structurally impossible to observe, regardless of anything else: with
zero sensor noise, repeated integration of the *same* static geometry
from a *static* viewpoint produces bit-identical depth every frame, so
there is nothing for nvblox to churn on. **This is the false-null risk
the task named — confirmed real, not hypothetical.**

**cuVSLAM uses a fully separate stereo input — confirmed from config,
not assumed.** `x500_depth_stereo/model.sdf`'s own docstring: "x500_depth
(keeps its OakD-Lite depth camera, feeding nvblox) plus StereoIMU
(D435i-like left/right IR + IMU, feeding cuVSLAM)" — two separate
`<include merge='true'>` blocks pulling in two separate models. Verified
directly in `StereoIMU/model.sdf`: sensors `left`/`right`
(`type="camera"`), topics `stereo/left/image` / `stereo/right/image` —
entirely different sensors and topics from `OakD-Lite`'s `depth_camera`
topic. Adding noise to the depth camera therefore cannot touch cuVSLAM's
input at all. (Aside: `StereoIMU`'s IMU sensor already has
`<noise type="gaussian">` blocks on it — an existing, working example of
the noise-tag syntax in this exact SDF version, used as the template for
B1 below.)

### A2. True physics ground truth (critical gate)

**Confirmed: `/ground_truth/odom` (currently used by both
`dmin_logger.py:82` and `churn_logger.py:39`) is EKF2-derived, not
physics** — `run.sh`'s own comment block (lines 68-82) says so
explicitly: "`/ground_truth/odom` ... IS EKF2-derived ... NOT literal
Gazebo simulator truth, despite the topic name."

**Recommended true-physics source, already established and verified
working in this project:** `/world/$GZ_WORLD_NAME/dynamic_pose/info`
(defaults to `/world/vio_test/dynamic_pose/info` for the obstacle world
this probe uses), type `tf2_msgs/msg/TFMessage`. This is gz-sim's
**SceneBroadcaster** system — physics-engine pose, no PX4/EKF2 in the
loop at all — already bridged unconditionally in `run.sh:99`, and
already has a working reference implementation in this repo:
`position_logger.py` subscribes it and filters
`transforms[i].child_frame_id == "x500_depth_stereo_0"` (the model root)
to extract this drone's pose out of the message's per-entity list.
Confirmed live and used in project history, not just written and
untested (`flight_test_log.html`: "Pivoted mid-task to gz-sim's
SceneBroadcaster, already loaded unconditionally, **confirmed live
before use**").

**No cleaner alternative exists — checked, not assumed.** A dedicated
`gz-sim-pose-publisher-system` plugin was already added directly to
`x500_depth_stereo/model.sdf` (lines 27-61) specifically to be a cleaner,
single-model-scoped alternative to parsing the whole-world
SceneBroadcaster message — but `run.sh`'s comment states it "advertises
a topic but, in practice, never actually publishes on it (root cause not
yet diagnosed)," confirmed as a real, live-tested dead end, not a
theoretical one. The SceneBroadcaster + filter-by-child-frame-id approach
is the one working path.

### A3. World SDF — wind / dynamic light

**Clean, confirmed by inspection of `vio_test.sdf` in full:**
- No `<wind>` element at world scope, and the only `<enable_wind>` tag
  present is `false`, on the static `ground_plane` model (line 90) — no
  wind plugin (`*-wind-*.so`) is loaded anywhere in the file.
- One light, `sunUTC` (`type="directional"`, lines 95-113): fixed
  `<pose>`, fixed `<direction>`, fixed `<intensity>1</intensity>`, no
  `<plugin>` animating it, no day/night-cycle mechanism. Static,
  non-flickering.

**Verdict: world is clean, nothing to patch for A3.**

## Phase C setup note: goal geometry constraint found, adjusted honestly

`fast_planner_px4.launch.py`'s hardcoded `waypoint0` was exposed as a
launch argument (`waypoint0_x/y/z`, defaulting to the existing
`(0,8,5)`, same pattern as the nvblox rate-arg exposure) so this probe
could fly its own goal without touching every other flight's default.

**A straight `(0,15,5)` goal (the task's suggested "~15m through the
pillar field") is not safe — checked, not assumed.**
`plan_env/sdf_map.cpp:144`: `map_origin_ = (-x_size/2, -y_size/2,
ground_height)` with `map_size_x/y = 20.0` — this is a **fixed** global
map centered at world origin, bounded to roughly `[-10, +10]` on X and Y
(not an egocentric/rolling map). A goal at `y=15` sits 5m outside the map
entirely.

**Adjusted goal: `(9, 9, 5)`** — 3D distance from origin
`sqrt(9²+9²+5²) ≈ 13.0m`, not exactly "~15m" as suggested, but the
closest safe distance with a real margin (1m) from the discovered
boundary. Still passes directly through the pillar ring near the start
(any path leaving the origin does, since all 8 pillars sit on a 2.5m-
radius ring centered at the origin) forcing the same kind of sustained
replanning the smoke test and micro-verify already observed, then
continues into the sparser outer-marker region for the remainder of the
flight. This is a config choice, disclosed here rather than either
silently trying an out-of-bounds goal or stopping the whole gate over a
adjustable parameter.

## Arm S (frozen map) — BLOCKER, not attempted further; pivoting to Arm L first

Diagnosed through direct investigation, not guessed, in this order:

1. **First attempt** (build map flying once, `save_map` at the goal, then
   `integrate_depth_rate_hz=0 + update_esdf_rate_hz=0 + load_map` for the
   frozen runs): run 1 flew almost nowhere in 75s and `fast_planner_node`
   logged repeated `kinodynamic search fail!` / `error from
   kinodynamic_astar`. `churn_logger.py`'s CSV had zero rows beyond the
   header — the ESDF service call was never completing.
2. **Root cause 1, confirmed via source**: `nvblox_base.yaml`'s
   `map_clearing_radius_m: 7.0` erases map blocks more than 7m from the
   drone's current position, at 1Hz — a **separate** rate from
   `integrate_depth_rate_hz`/`update_esdf_rate_hz`, not disabled by
   zeroing those two. Since this probe's path is ~13m, the corridor near
   the start was already erased by the time `save_map` ran at the far
   goal. Fixed by exposing `map_clearing_radius_m` as a new
   `nvblox.launch.py` launch arg (same pattern as the other rate args,
   default unchanged) and setting it to `-1.0` ("no clearing", per the
   yaml's own comment) for both the map-build flight and the frozen
   loads.
3. **Root cause 2, found after re-running with fix #2 applied and still
   getting zero real voxels near either pillar checked**: querying the
   *entire* path's AABB directly (not through the flight, a standalone
   service call) showed real (non-sentinel) voxels **only** in a small
   bubble at `x:9.0-10.0, y:8.4-10.0` — i.e., only right around the final
   hover point, nothing along the rest of the ~13m corridor, despite
   clearing being fully disabled. Most likely explanation, consistent
   with how TSDF integration works (a voxel needs multiple consistent
   observations to cross `min_tsdf_weight` before it's usable): a single
   flythrough at cruise speed gives each voxel along the corridor only a
   brief, one-off glimpse, which apparently isn't enough weight to
   register as "observed" — only the ~10+ seconds spent hovering at the
   goal gave sustained enough observation to build real map data there.

**This is a genuine blocker for Arm S exactly as specified** ("build the
map once, save it") — not a tuning failure, and not something to paper
over with a fourth unvalidated workaround (e.g. flying slower or adding
deliberate hover waypoints during the build pass, neither of which was
in the original recipe and both would need their own validation before
being trusted). Per this task's own rule ("stop at genuine blockers,
report, don't improvise a workaround that hides a problem"), this is
reported here rather than iterated on further right now.

**This does not block Q1's primary question**, which only needs Arm L
(live nvblox, no freeze/reload involved at all): "is churn(t) meaningfully
>0 in Arm L?" If Arm L's answer is no, H4 is dead regardless of whether
Arm S's freeze recipe can be fixed. Proceeding to Arm L now; Arm S is
revisited only if Arm L's result makes it necessary for Q2.
