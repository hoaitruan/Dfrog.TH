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
