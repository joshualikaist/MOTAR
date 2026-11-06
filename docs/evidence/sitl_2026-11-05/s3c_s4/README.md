# S3c v1 and S4 v1 — preparation record (2026-11-06)

**Status:** preregistered and reviewed, **NOT_RUN**. Both units are reported without a gate, as the plan's S3c and S4
rows say. Read the preregistrations first:
- [S3c v1](../../../plans/prereg_sitl_s3c_v1_2026-11-06.md): PX4's iris, and a one-step command delay;
- [S4 v1](../../../plans/prereg_sitl_s4_v1_2026-11-06.md): Gazebo's own LiDAR, RGB and depth, with ground-filtered and
  raw arms.

Both run [S3b formal v1](../s3b_formal/README.md)'s episode and unit loop. Each has a development smoke01 (one
episode) that must be COMPLETE and VALID, at the same HEAD and source bytes, before its attempt01.

## S4 component smoke (development, before preregistration)

Record: [`s4_component_smoke.json`](s4_component_smoke.json). Its script and output are bound in
[`validation.json`](validation.json). The smoke ran gzserver only, without PX4 or a policy, with OpenGL on display :1:
a sensor rig at 1 m, a bar 3 m ahead and one 2 m to the left, and the target sphere.
- **LiDAR order:** verified. The front bar reads 2.800 m in MOTAR column 36 at 0° elevation, 2.843 m at ±10° and
  2.979 m at +20°. The left bar reads 1.800 m in column 18.
- **Depth:** 2.700 m on the front bar.
- **Target:** a moved target appears in the next frame.
- **Found:**
  - At 4.1 m, F's colour rule passes 4 of the 35 reddish target pixels, because Gazebo's shading darkens most of the
    sphere below the 0.55 threshold.
  - The −10° LiDAR row hits the ground at about 5.5 m.
  - MOTAR's camera fusion takes each column's minimum depth over all rows, so a floor in the lower image rows reads
    as obstacles about 2 m ahead. That is why S4 has a ground-filtered arm.

## Independent reviews (Claude Opus 5.5, fresh context)

- [r1](REVIEW_r1.md): **S3c PASS_WITH_LIMITS, S4 FAIL.**
  - **S4 BLOCKER:** `navrl_quad_s4` kept navrl_quad's visuals. Ray casts put the iris body mesh and the propeller discs
    in the LiDAR beams at 0.20–0.31 m and in the camera at 0.05–0.15 m. The smoke rig had no vehicle, so it could not
    show this.
  - **S4 MAJOR:** recall used fused visibility, which includes LiDAR association.
  - **Both, MAJOR:** the analyzers lacked S3b's integrity checks and timing.
  - **Repairs:**
    - visuals removed, with a vacuity check;
    - a 0.35 m hover guard;
    - camera-only recall, with LiDAR-only counted separately;
    - the shadow depth clamp, frame age in lateness, −inf → 0;
    - render-thread RGB;
    - port, GPU and plugin-age refusals;
    - the target height offset;
    - analyzers with integrity, timing and a SHA-bound S3b reference;
    - an iris-footprint report;
    - smoke gates;
    - wording.
- [r2](REVIEW_r2.md): **S3c PASS_WITH_LIMITS, S4 PASS_WITH_LIMITS.** No BLOCKER or MAJOR.
  - **S4 verification:** from the Gazebo 11 source, collisions are not rendered by cameras or gpu_ray. The two included
    sensor visuals (rangefinder, GPS) are inside the 0.2 m near clip and behind the camera.
  - **Repaired after r2:**
    - s1: RGB and depth are stamped with the rendering scene's simulation time;
    - s2: the smoke must match HEAD and the source bytes;
    - s3: `--analyze` refuses a smoke;
    - the chain launcher stops on a host fault, builds the plugin in a clean directory, and has 3 h timeouts.
  - **Open LOWs:**
    - the Gazebo–PX4 clock offset is not computed;
    - steps with RGB older than the target update are not summarised;
    - no test feeds a near depth value into the hover guard.

## PX4-tree additions (outside every repository)

- `navrl_iris` model and airframe 10091 (S3c).
- `navrl_quad_s4` model (visuals removed) and airframe 10092, which is 10090's file with only its @name changed (S4).
- The PX4 binary's SHA-256 is unchanged. All SHA-256s are in [`validation.json`](validation.json).
