# S4 v1 — Gazebo's own LiDAR, RGB and depth in place of the bridge's ray casts (preregistration, 2026-11-06)

**State:** preregistered, **NOT_RUN**. Runner `tools/sitl_s4.py`. Gazebo pieces:
- `tools/sitl_s4_gazebo.py`: client, conversions, ground removal, world;
- `tools/sitl_s4_model.py`: the `navrl_quad_s4` model and airframe 10092;
- `tools/sitl_s4/motar_s4_bridge.cc`: the world plugin.

Tests are in `tests/test_sitl_s3c_s4.py`. This is plan stage S4 of
[`sitl_validation_plan_v1.md`](sitl_validation_plan_v1.md). Amendments go in a new dated file.

## Question

MOTAR's sensors in SITL so far were its own ray casts against a ground-free mesh. What changes when F perceives
through Gazebo's rendered sensors instead? Those sensors are a gpu_ray LiDAR, an RGB camera and a depth camera,
body-fixed, with Gazebo's lighting and a ground plane.
- What fraction of the steps where the target would be detectable under F's training renderer is still detected by
  F's colour rule on Gazebo's image?
- How do capture and crash change against S3b on the same scenes?

The plan's S4 is **reported, not gated**: "reported; detector recall in Gazebo".

## Why two arms

MOTAR's perception takes each camera column's minimum depth over all image rows as an obstacle range
(`navrl_perception.py`, the camera fusion into the static scan). MOTAR's renderer never draws a ground, but a real
or Gazebo depth camera at 1 m sees the floor in its lower rows, at about 2 m.

The 2026-11-06 component smoke below confirmed that Gazebo's −10° LiDAR row also hits the ground, at about 5.5 m. Two
arms separate this from everything else:

| Arm | Sensors | Scenes |
| --- | --- | --- |
| `ground` (primary) | Gazebo's frames, with returns that hit the ground plane removed: LiDAR to 12 m, depth to 20 m. A hit counts as ground when it is below 0.10 m height, using the vehicle's true attitude and height (truth altitude + 0.09 m settled offset), as an onboard filter would use its estimate | first 40 eligible 70-bar scenes |
| `raw` | Gazebo's frames as delivered | first 20 eligible 70-bar scenes |

## Fixed treatment

- **Vehicle visuals removed:** `navrl_quad_s4` has every `<visual>` of navrl_quad removed, keeping its collisions,
  inertias, joints and plugins. Gazebo's cameras and gpu_ray render visuals, and the S3c/S4 review r1 found by ray
  casting that the iris body mesh and propeller discs would otherwise fill LiDAR beams at 0.20–0.31 m and the camera
  at 0.05–0.15 m.
- **Hover guard:** after the start check, a LiDAR return or depth pixel nearer than 0.35 m makes the episode
  HARNESS_INVALID.
- **As S3b:** S3b formal v1's episode, step for step. Frozen F, perception, governor, 10 Hz control, take-off, start
  check, termination, owned fresh-rootfs session, parameter read-back, lateness bound, G8 r2 inputs and pins, and
  the launch refusals. The vehicle dynamics are navrl_quad's: airframe 10092 is 10090's file with its @name line
  changed.
- **Only the sensors differ.** Each step, the newest LiDAR, RGB and depth frames from the plugin are converted to the
  arrays MOTAR's perception takes and given to perception, policy and governor in place of the bridge's casts:
  - LiDAR: MOTAR's beam order; beyond 12 m, +inf or NaN becomes 12 m; −inf or below 0.2 m becomes 0, MOTAR's
    "too close";
  - RGB: scaled to [0, 1];
  - depth: optical-axis depth converted to range along each pixel's ray, with no return or beyond 20 m set to
    20 m.
- **Target:** the replayed target position is sent to Gazebo one step ahead, with its height raised by the vehicle's
  0.09 m resting offset. The bridge's truth heights are measured from the resting origin. It is a kinematic, collision-free
  sphere of radius 0.15 m in F's target colour (0.88, 0.08, 0.045), so both the camera and the LiDAR see it.
- **Shadow render:** MOTAR's own render at the same true pose is computed beside it, for the record only. It gives
  the renderer-private target pixel count and the LiDAR and depth differences.

## Disclosed sensor differences from MOTAR's contract

| Item | MOTAR's renderer | Gazebo here |
| --- | --- | --- |
| Mount | yaw-level LiDAR and camera | body-fixed, tilting with roll and pitch, as on a real vehicle |
| Target | analytic sphere, flat red, unshaded; LiDAR sphere 0.20 m | one 0.15 m sphere, lit and shaded by Gazebo |
| Ground | none | ground plane (the `raw` arm keeps it) |
| Pixels and FOV | 87° × 58°, non-square pixels | 87° × 56.2°, square pixels |
| Obstacle depth | 40×24 rays to 10 m, upsampled | 160×90 to 20 m, with no 10 m obstacle clamp |
| Frame timing | rendered at the step's pose | the newest 20 Hz frame. RGB and depth are copied in Gazebo's rendering thread and stamped with the rendering scene's simulation time; LiDAR is read in the update thread with its last measurement time. A step's lateness is the larger of the state sample's lateness and the frame age, under the 0.15 s bound. Geometry nearer than the 0.2 m LiDAR near clip is not seen |
| Noise | none (perturbations off in F) | none (Gazebo's sensors are noiseless here) |

## Development record before preregistration (2026-11-06, CPU and GPU rendering, no PX4, no policy)

The plugin component smoke ran gzserver only, on display :1. It used a static sensor rig at 1 m, a bar 3 m ahead,
one 2 m to the left, and the target 4.1 m away. Record:
`~/motar_tmp/s4_gazebo_20261106/smoke/smoke_result.json`.
- **LiDAR mapping:** the front bar falls in MOTAR column 36 at 2.800 m at 0° elevation, and 2.843 m at ±10°. The left
  bar falls in column 18 at 1.800 m.
- **Depth:** 2.700 m on the front bar, with the camera 0.10 m forward.
- **Target:** a moved target appears in the next rendered frame. At 4.1 m, F's colour rule passes 4 of the 35
  reddish target pixels, because Gazebo's shading darkens most of the sphere below the 0.55 threshold.

This was a component check, not a measurement of this unit.

## Development smoke before the attempt

- **smoke01:** the first scheduled episode (arm `ground`, the first scene), run once with the same code into
  `~/motar_tmp/s4_20261106/smoke01`. This stack has not flown before.
- **Not part of the reading.** attempt01 refuses unless smoke01 is COMPLETE with its one episode VALID. A failed smoke
  stops the unit for repair and a new review.
- **Approval:** the approval names both smoke01 and attempt01.

## Validity and stop rules

- **Episode validity and checks:** as S3b and S3c, through `tools/sitl_s3b_formal.run_unit` with every arm primary.
  Per episode there is an idle check (HOST_BUSY) and a runtime binding. The binding covers the PX4/Gazebo stack and
  the S4 plugin's SHA-256, and is compared within each arm (BINDING_CHANGED).
- **The unit stops (ABORTED)** in any of these cases:
  - a fault rise;
  - a tree change;
  - an INVALID first episode;
  - three consecutive INVALID episodes;
  - more than 10 of the 60 episodes INVALID.
- An episode's lateness at each step is the larger of the state sample's lateness and the oldest frame's age.
- Any exception, termination request or SIGTERM is recorded in `final.json`.
- **No retries.**

## Reported (fixed analyzer: `tools/sitl_s4.py --analyze`)

For each arm:
- capture, crash and timeout counts and rates;
- Δ against Isaac on the same VALID scenes;
- agreement with Isaac;
- the matched-scene table against S3b attempt01 (pairs, same outcome, captures and crashes in each);
- **camera recall in Gazebo:** among the steps where the shadow renderer's target mask has at least 2 pixels, the
  fraction where perception's camera path reports the target visible. That is `camera_visible`: F's colour rule on
  Gazebo's image.
  - Perception's fused `visible` also includes LiDAR association. The LiDAR-only detections are reported
    separately.
  - The denominator uses MOTAR's yaw-level camera, while Gazebo's camera tilts with the body, so this recall mixes
    detection with mount geometry;
- median colour-rule and shadow pixel counts on those steps;
- frame age;
- LiDAR and depth differences from the shadow.

There is no pass or fail label.

## What follows, and what does not

- **It shows two things:**
  - how F's SITL outcomes change with Gazebo's sensors in place of MOTAR's idealised renderer. Mount, lighting,
    field of view, resolution and frame timing change together; only the ground is separated, by the two arms;
  - how much the ground returns alone change the outcomes (`raw` against `ground`).
- **Inputs to S5:** the S5 checklist uses the reported recall and gap, for the onboard ground filter and the
  detector.
- **It does not follow:**
  - real-camera detection: the target is still a red sphere, and the real target is a drone, whose real-image
    detector lineage (YOLOv5s on NPS and Det-Fly, P8–P10) is not connected here;
  - a tuned ground filter;
  - S3b acceptance;
  - flight readiness.

## Execution envelope

- **Process:** CPU process as S3b (cores 6,7,12,13; `nice -n 10`; one thread; CUDA hidden).
- **Launch refusals, in addition to S3b's:**
  - the plugin is missing, or older than its committed source. The chain rebuilds it from the committed `.cc`
    right before the smoke, and `attempt.json` binds both SHA-256s;
  - TCP 127.0.0.1:4590 is in use;
  - a CUDA job is listed by `nvidia-smi`;
  - there is no DISPLAY.
- **GPU rendering:** Gazebo's cameras render with OpenGL on the RTX 3070 through display :1. No CUDA is used, and no
  other GPU job may run.
- **Host and tree:** idle host; frozen tree; `timeout -k 30 7200`.
- **Single use:** attempt `~/motar_tmp/s4_20261106/attempt01`.
- **Approval:** `~/motar_tmp/s4_20261106/execution_approval.json`, naming `tools/sitl_s4.py`'s `SCOPE` and the commit.
- **Duration:** expected about 60 min.
