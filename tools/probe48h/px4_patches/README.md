# PX4-Autopilot patches for probe-48h

`PX4-Autopilot/` is a vendored local clone, gitignored by this repo (see
`.gitignore`: "Vendored / upstream projects"). Files edited there for
this probe are backed up and diffed here so the change survives a
fresh PX4 checkout/rebuild.

## OakD-Lite depth camera: added Gaussian noise

**File:** `PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite/model.sdf`
**Sensor:** `StereoOV7251` (`type="depth_camera"`), the sensor that feeds
nvblox (see `docs/probe48h/GATE12.md` A1).

- `OakD-Lite_model.sdf.orig` — the file as found (no noise at all).
- `OakD-Lite_model.sdf.patched` — the file after the fix (full copy, not
  just the sensor block, so a diff-apply or a straight copy both work).
- `OakD-Lite_model.sdf.diff` — unified diff between the two, for `patch
  -p0` if a fresh PX4 checkout needs the same fix re-applied.

**What changed:** added `<noise type="gaussian"><mean>0.0</mean>
<stddev>0.02</stddev></noise>` inside the sensor's `<camera>` block.
Nothing else in the file changed.

**Why 0.02, and why uniform, not range-dependent:** the task asked for
noise "approximating a RealSense D435i" with stddev "≈0.01-0.02m at
close range, growing ~z²". Checked against `/usr/share/sdformat14/1.9/
camera.sdf`'s own schema before assuming this was possible: sdformat's
camera `<noise>` element supports exactly three sub-elements — `type`,
`mean`, `stddev` — a single fixed value, with no range-dependent
parameter anywhere in the spec. True z²-scaling noise would need a
custom Gazebo sensor/render plugin (real simulation code, not
configuration) — out of scope for this task ("do not modify... algorithm
cores"). `0.02` (the upper end of the stated close-range band) was used
as a single representative value. **This is a disclosed simplification,
not a hidden one**: the real sensor's noise grows with range, this
config's noise does not — treat any churn measured under this config as
a lower bound relative to a hypothetical range-accurate model, not an
exact match to real D435i behavior.

**Reproducing after a fresh PX4 checkout:**
```bash
cp tools/probe48h/px4_patches/OakD-Lite_model.sdf.patched \
   PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite/model.sdf
# or: patch -p0 < tools/probe48h/px4_patches/OakD-Lite_model.sdf.diff
```

## A3 (wind / dynamic light): no patch needed

`vio_test.sdf` was inspected (`docs/probe48h/GATE12.md` A3) and found
clean — no wind plugin, one static directional light. Nothing to patch.
