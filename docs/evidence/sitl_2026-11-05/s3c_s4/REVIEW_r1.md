# Review r1: S3c v1 and S4 v1

Reviewer: Claude Opus 5.5 (claude-opus-5-5), fresh context, read-only. Tree: `camonly/dev` e4364c1 plus uncommitted
units. The files changed while I reviewed (code 15:57 KST, preregs 16:00). The verdicts apply to `snapshot_0658/`
(SHA-256s in `probes/tests_v2_sources.sha256`). Nothing was started: no PX4, Gazebo or GPU, no attempt01, no approval.

## Probes (`probes/`)
- **Tests:** `tests/test_sitl_s3c_s4.py`, 11/11 OK on the first snapshot and 12/12 OK on the current one (97.5 s).
- **`observe_with_probe`:** the real bridge, 3 G8 scenes × 80 steps.
  - With MOTAR's own render as input, `observe_with` equals `observe` bit for bit in 240/240 steps: obs, LiDAR,
    `visible` and the governed command.
  - A perturbed-LiDAR control differs in 240/240. The shadow differences are 0.
- **`sigterm_probe`:** the real OwnedSession, SIGTERM inside an episode.
  - Without a runner handler, as in the first snapshot, the process dies (exit 143) with no final.json.
  - With `SF._on_sigterm` installed, as the current `launch_unit` does, final.json reads ABORTED (`sigterm_v2`).
    This is resolved.
- **`self_occlusion_probe`, `facing_probe`, `prop_camera_probe`:** S4's rays cast against navrl_quad_s4's own
  meshes.

## S4

**B1 BLOCKER: the sensors see the vehicle itself.**
- navrl_quad_s4 keeps navrl_quad's visuals: the `iris.stl` body (±0.24 m) and iris props of radius 0.133 m at
  (±0.13, ±0.13, **z 0**). The LiDAR is at the origin; the cameras are at (0.10, 0, 0.03).
- **LiDAR, body:** 10 beams (rows 0°/−10°, azimuth ±55–60° and ±120–125°) hit front-facing body triangles at
  0.20–0.24 m. These returns are permanent and lie inside the 0.2–12 m window.
- **LiDAR, props:** the 0° row lies in the prop plane. The swept discs cover 68 of 72 columns at 0.2–0.31 m, so the
  blades give intermittent returns.
- **Camera:** each front prop is in the image for about 50 % of blade angles, over about 38 columns at 0.05–0.15 m.
  The column-minimum fusion (navrl_perception.py:1909–1921) turns this into obstacles about 0.1 m ahead.
- **No arm removes them:** the ground filter is height-based.
- **Not caught:** the smoke rig had no vehicle visuals.
- **Failure scenario:** both arms measure self-occlusion.
- **Caveat:** this is a geometry result. It assumes Gazebo classic's rendering sensors draw the parent model, which
  is their default.
- **Fix:**
  - strip the `<visual>`s from navrl_quad_s4 (physics is unchanged);
  - at hover, set HARNESS_INVALID when a LiDAR return is under 0.35 m or depth pixels are near;
  - run an approved integrated smoke before attempt01, whose first episode would otherwise be this stack's first
    flight.

**M1 MAJOR: "detector recall" includes LiDAR association.**
- The recorded flag is `diag["visible"] = visible | lidar_visible` (:2286, :2394).
- `NAVRL_LIDAR_TARGET_ASSOC` defaults on and is absent from the receipt.
- The association fires exactly when the camera misses while the track lives (:2085–2091).
- The prereg's recall ("F's colour rule on Gazebo's image") is therefore overstated.
- Fix: record `camera_visible` and `lidar_visible`, compute the recall from `camera_visible`, and report the fused
  value beside it.

**m1 MINOR: the shadow depth difference is an artefact.**
- MOTAR fills no-return pixels with 10 m, Gazebo with 20 m.
- With identical geometry, the median |Δ| is 10.0 m in the median step.
- Fix: clamp both to 10 m before comparing. Perception treats ≥ 10 m as no return (:216–229).

**m2 MINOR: the frame-age bound is 0.25 s.**
- The code adds the age beyond 0.10 s to the lateness and then applies the 0.15 s bound.
- Say this explicitly, or make an age over 0.10 s INVALID.
- Log `sim_now` to audit the PX4/Gazebo clock domain.

**m3 MINOR: LiDAR near returns.**
- Gazebo 11 gpu_ray reports ≤ min as −inf (REP 117), which `motar_lidar` maps to 12 m, not 0.
- Geometry closer than 0.2 m is near-clipped and seen through.
- Only a finite 0.1 m case is tested.
- Fix: map −inf to 0 and disclose the see-through.

**m4 MINOR: plugin.**
- Problems:
  - `ImageData()` is copied in the physics thread while the render thread writes it, so torn frames are possible;
  - depth is stamped with world time at the callback, so its age is underestimated;
  - TCP 4590 is not in the session PORTS;
  - the `.so` is recorded but not bound to the `.cc`.
- Correct:
  - a missing sensor gives None → ValueError → ERROR (fails closed);
  - no listener gives HARNESS_INVALID;
  - socket timeouts bound every wait;
  - every exit path closes the client and the link.

**m5 MINOR: wording.**
- "Depends on its idealised renderer": only the ground is separated. Mount, lighting, FOV, frame age and B1 are
  confounded.
- "A ground filter is a precondition for flight" contradicts "flight readiness does not follow".
- Disclose that the recall denominator uses a yaw-level 58° camera, while Gazebo's is body-fixed and tilts.

**LOW:**
- The target z lacks SF_H0, so the target sits about 0.09 m low.
- The copied `_fly` has drifted (no `truth_age_max_s`).
- There is no GPU-idle check.

**Correct:**
- LiDAR mapping. The smoke agrees: col36 2.98/2.84/2.80/2.84, col18 1.80, ground only in the −10° row.
- The clamps.
- Depth z→ray, with pixel centres equal to the detector's.
- RGB order and scale.
- FRD→NED.
- The target transform equals the bars'; the one-step-ahead target is right.
- The shadow's side effects equal S3b's per-step render.
- The model and airframe regenerate byte for byte.

## S3c

- **Identity:**
  - `SF.run_episode`/`_fly` read from V0 only WORLDS, ODE_ITERS, SPAWN, rot, PARAMS, DT, TM_*, ALT, MAX_STEPS,
    COMMAND_YAW, seg_min_dist, box_hit, CAPTURE_M and OOB_MARGIN, and the namespace copies all of them.
  - `ParamLink` subclasses the real `V0.Link`, which reads no PARAMS.
  - So `v0_with_params` changes only PARAMS.
  - The airframe (10091) comes from `SF.AIRFRAMES`. The binding is per arm.
  - The delay is tested through `run_episode`, with a d=0 control.
- **Iris configuration:**
  - The unset rate gains and THR_MDL_FAC are disclosed, and so is the footprint (0.14 m against iris's 0.235 m).
  - The rangefinder pose equals upstream iris_opt_flow's.
  - The PX4 binary SHA b1da06f0… is verified.
  - The builds ran with DONT_RUN.
- **Stop rules:** the current prereg matches `run_unit`: first episode, 3 consecutive, >12/80, faults, tree,
  HOST_BUSY, BINDING_CHANGED.

**m6 MINOR: wording.**
- "Sensitive to vehicle mass and thrust margin": iris changes mass, inertia, geometry, gains, motors and collision
  size together.
- "That decides" has no rule, and n = 40.
- Write: "estimates the change with PX4's iris; informs GR-1b/GR-2".

**m7 MINOR: the iris goes first and has never flown in this harness.**
- A harness fault consumes attempt01.
- Fix: run one approved development episode first.
- Report, ungated, the crashes at iris's own footprint, from the `steps.npy` clearance.

## Both

**M2 MAJOR: the analyzers omit preregistered items and S3b's integrity checks.**
- S3c has no timing summary.
- S4's `reading.json` lacks the frame age, the pixel medians and the LiDAR/depth differences.
- Neither applies `integrity_problems` (schedule prefix, faults, tree). This reintroduces S3b r1 m4, against the
  host-fault rule. `integrity_problems` and `timing_summary` are keyed on `block`: parameterize them on `key`.
- The S3b ledger used for the pairs is not bound (no SHA, no label). With S3b absent, the pairs are silently 0.

**Tests:**
- Non-vacuous: delay, conversions, ground removal (kept-bar control), stops, binding, fake plugin.
- Missing: the real `observe_with` (the probe covers it), the `launch_unit` refusals, real-session SIGTERM, the
  analyzers, and −inf LiDAR.

**S3b r1:** B1, M1, M2 and m1 do not recur. Links close, waits are bounded, the refusals are shared, and the fault
baseline is read before mkdir.

## Verdicts
- **S3c v1: PASS_WITH_LIMITS.** The episode path and guards are sound. Fix M2 (the S3c part) and m6 before approval.
  m7 is recommended.
- **S4 v1: FAIL.** B1 would void every episode, and M1 misstates the headline metric. Fix both, smoke the integrated
  stack, and review again.
