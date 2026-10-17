# EQ-0 / EQ-G2 independent qualification audit — 2026-10-17

READ-ONLY / CPU RECORD AND MATH FORENSICS. No simulator, Warp initialization, GPU, PPO, rollout,
source/raw edits, deletion, commits, or EQ-1 experiment was performed. All new files are in this
scratch folder. This is not a retrospective change to OCC-P2's endpoints or verdicts.

## Scope and conclusion

OCC-P2 measured source `5ff285ad053f181c5a1c86dddb2462bb44e6327b`; review checkout
`7cea3c52d8c8d7ab6dbeef5fae6e929acfe279a2`. All 15 sensor/geometry/task/analysis files checked here
equal their measurement-commit bytes. All 112 receipt/trace metadata pairs identify the measured
commit, source_dirty=false, analytic_flat target rendering, no dynamic-mesh treatment, zero
distractors, dt=0.1 s, zero-latency render delivery, and sensor capture time NOT_RECORDED.
These statements were independently read from raw records, not inferred from the current defaults.

**Overall EQ-G2: LIMITED_VALIDITY for the frozen simulated OCC center-LOS estimand. A narrowly
defined calibrated camera-to-LiDAR fusion subgate FAILS.** There is no present evidence that this
failure invalidates the registered count/exposure result: it is part of the unchanged closed loop
whose trajectories OCC measured. It does preclude presenting that loop as independently qualified
metric multisensor geometry. A future recovery experiment needs its own declared sensing,
identity, timing and controller contract; no H-R definition was supplied, so no H-R pass/fail is
invented here.

| Subgate | Status | Severity and limit |
| --- | --- | --- |
| G2.1 measured-source and sensor-path provenance | PASS | All 112 records consistent; current files equal measured bytes; G0 runtime lineage unchanged. This is provenance, not hardware calibration. |
| G2.2 target RGB-D camera internal model | LIMITED_VALIDITY | Correct documented ideal pinhole/ray-sphere model, but low resolution, ideal metric range, flat color and yaw-only camera. Not an RGB-only or calibrated hardware camera qualification. |
| G2.3 camera depth → LiDAR static-map registration | FAIL | Medium for interpretation of existing closed-loop sensing; high before a claim of calibrated cross-sensor geometry. Source uses linear angular column mapping where renderer is pinhole, and does not reproject camera-origin returns to LiDAR origin. |
| G2.4 LiDAR scan grid / target-return model | LIMITED_VALIDITY | Angular formula is self-consistent; actual GPU LiDAR range calibration is NOT_TESTED by this audit/G0 camera parity. Sparse ideal rays, different target radius, no noise. |
| G2.5 target association / persistent identity | LIMITED_VALIDITY | GT target state is confined to rendering, but raw-range association can lock to a bar. Fused-valid is not identity-correct target visibility; no re-ID guarantee. |
| G2.6 primary center LOS / numeric boundaries | LIMITED_VALIDITY | Strong recorded G0/P2 agreement, five boundary frames localized; no primary count change. No independent physical-world or all-geometry validation. |
| G2.7 full visual silhouette / zero pixels | NOT_TESTED | Finite rim rays are a proxy; zero rendered pixels can arise without any occluder. No target-alone support denominator was recorded. |
| G2.8 pose / state time alignment | LIMITED_VALIDITY | Zero-latency same-render state pipeline is source-supported; hardware exposure time, clock error, scan duration and rolling shutter are NOT_RECORDED/NOT_TESTED. |
| G2.9 GT firewall in actual perception branch | PASS | API/source inspection establishes no target_position/semantic input to perception.observe; rendered sensor ranges necessarily depend on scene geometry. This is structural, not adversarial binary/runtime taint validation. |

## Evidence inventory and actual values

Raw root: `/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/occlusion_confirmatory_v1/OCC-P2`.
New artifacts: `provenance.json`, `independent_math.json`, `boundary_audit.json`, and the reproducible
`independent_checks.py`. The script imports no project geometry/perception functions, Torch or Warp.
It parses records directly, reconstructs rotations with a matrix, solves ray/sphere geometry and
face-inequality segment bounds independently, and verifies the five inspected trace/layout hashes.
The input hash bindings and current-versus-measured source hashes are in `provenance.json`.

* Recorded in every cell: camera HFOV 87 degrees, VFOV 58 degrees, center range 20 m, camera
  translation [0.10, 0, 0.03] m in the yaw-only vehicle frame, target radius 0.15 m, step 0.1 s.
* Recorded receipt env in every cell: LiDAR 72 horizontal x 4 vertical rays, max 12 m; configured
  dropout 0.3, RGB noise 0.015 and depth noise 0.02. These magnitudes alone do not activate noise.
* Source-derived active camera resolution: 160 x 90, detection resolution identical; no camera
  width/height override, detector checkpoint, FOV/mount perturbation or empirical-error model in
  any receipt. The source defaults must be cited with these values: the raw trace does not save
  the live ray table, intrinsics matrix or runtime segmenter weights.
* Source-derived actual perturbation gate: OFF. `NAVRL_PERCEPTION_PERTURB` is absent in all 112
  receipts; task config line 277 defaults False; task line 6477 passes that boolean as `training`;
  perception lines 1549–1553 and 1596–1597 apply these three perturbations only under training.
  The runner clears inherited NAVRL variables (`occlusion_confirmatory.py:90`), and the fixed
  launcher does not enable this gate. Therefore do not describe this run as 30% injected-dropout
  sensing. This is a source-and-receipt derivation, not a saved live enable-flag measurement.
* Actual target mode and no distractors are recorded in result.condition. The no-checkpoint
  bootstrap classifier is source-derived: `navrl_perception.py:1002`, config `:275`, receipt
  absence. Its weights are sigmoid(3R-2G-2B-0.9), zero depth coefficient (`:621–633`). No detector
  training or live real-image classifier evidence is created by OCC-P2.

## Renderer and camera geometry

`navrl_detector.py:_render_target_camera_kernel` (53–132) intersects pixel rays with an analytic
sphere (physical-target OBB is a separate unused branch). A pixel survives only when the same
scene mesh has no hit before the target's front surface. `_render_obstacle_depth_kernel` (28–49)
casts scene-mesh rays at 40 x 24, max 10 m. `render_raw_rgbd` (894–968) bilinearly upsamples that
obstacle depth to 160 x 90 and shades it with a depth-dependent neutral color, then paints the
visible target's flat color and exact analytic range. This is not full scene photorealistic RGB,
stereo correspondence, a camera axial-z depth image, target mesh shading, or a D455 hardware model.

Ray lengths are **Euclidean radial ranges** because rays are normalized (`:375`, `:39`, `:48`,
`:132`). Perception averages these surface ranges and adds the known target radius, then multiplies
a normalized centroid ray (`navrl_perception.py:1662`, `1694–1704`), so range conventions agree
within this renderer. A real depth camera reporting optical-axis z would require z-to-ray-range
conversion; a hardware RGB-only camera cannot supply these metric 3D measurements without a
separate depth/range/scale interface. No current RGB-only transfer is established.

At 160 x 90 and 87 x 58 degrees, independent pinhole calculation gives fx=84.3024100225,
fy=81.1821489872, cx=79.5, cy=44.5. The rectangular image uses different focal values; the two FOVs
are independently specified, not evidence of square-pixel calibrated optics. Pixel-center rays
occupy slightly less than the nominal edge-to-edge FOV, correctly reflecting finite sampling.
Camera origin is p + R_yaw[.1,0,.03]; camera orientation equals yaw-only vehicle orientation
(`navrl_detector.py:806–818`). Roll/pitch therefore do not tilt this ideal camera. Mount errors
are absent, not a measured gimbal/calibration robustness result.

**Independent no-obstacle counterexample to zero-pixels⇒occlusion:** the radius-.15 m target
exactly centered on the optical axis yields 4 pixel hits at 10 m, 4 at 17 m, and 0 at 17.6 m or
20 m. With even dimensions, the nearest pixel centers are half a pixel off both axes. Their
distance to the target center exceeds its radius beyond 17.5435709456 m. This is subpixel phase
and angular sampling, without an occluder. Perception additionally needs 2 classified pixels,
whereas the renderer's visible-pixel count can be 1. Partial occlusion or appearance rejection
can remove camera measurements while center LOS remains clear, and LiDAR can resume a fused
measurement while the camera remains occluded. All are distinct events.

The mean surface-depth-plus-radius center estimate is approximate even on an unobstructed sphere:
independent biases at on-axis 1/5/10/15/17 m are +0.04495/+0.03588/+0.02639/+0.07166/+0.11233 m.
This is not a raw measurement of recorded tracker error: it is reference geometry illustrating
that averaging surface points and adding radius does not reconstruct the true center exactly.

## Confirmed camera-to-LiDAR fusion mismatch (G2.3 FAIL)

The renderer uses u=cx-fx*tan(theta) (`navrl_detector.py:363–375`), and camera measurements use the
corresponding inverse pinhole (`navrl_perception.py:1694–1704`). But `_fuse_static_and_extract_obstacles`
assigns camera depth columns to LiDAR bearings with
u=(HFOV/2-theta)/HFOV*(W-1), rounded (`navrl_perception.py:1825–1830`). This is a uniform-angle image
formula, not a pinhole formula.

For theta=+20 degrees, expected u=48.8164; code selects u=43, whose actual camera ray is +23.4109
degrees. At -20 degrees the symmetric error occurs. Across actual forward LiDAR bins this gives
max 3.4109268661 degrees error / 5.8164 pixels. This exceeds pure half-pixel resampling. At 5 m,
the isolated angular discrepancy corresponds to about 0.30 m lateral separation. It can assign
a nearby obstacle's depth to the wrong bearing in the actor's static map and obstacle selection.

There is a separate origin mismatch: the camera is translated relative to the body-center LiDAR,
but the copied column range is not reprojected. A front plane at body x=5 m is at camera range
4.9 m for its center ray; inserting it as a body-origin scan gives -0.10 m range error even with
zero angular error. Column min also loses elevation, then the same value is expanded to all
vertical scan lines (`:1823–1829`). Treat this as a fused angular obstacle profile approximation,
not a calibrated 3D registration. Fixing it would alter frozen Policy F inputs: no change is
recommended inside OCC or without a new registered sensing lineage.

The existing G0 comparisons cannot discover a shared registration error: OFF and ON both execute
this same fusion, and offline center-LOS calculation does not consume the fused obstacle map.

## LiDAR geometry, range and identity limits

`NavRLLidarConfig` (`navrl_lidar_config.py:35–139`) sets exact 72 azimuths +180,+175,...,-175 degrees;
four elevations +20,+10,0,-10 degrees, range [0.2,12] m, body-center origin, yaw-only attachment,
no placement randomization and no sensor noise. `WarpLidar.initialize_ray_vectors` (`warp_lidar.py:68–92`)
and `lidar_bin_bearings` (`navrl_perception.py:196–213`) agree on the decreasing azimuth convention.
Independent equations produce 72 distinct azimuths and 5-degree spacing, with no duplicated +/-180 ray.
Actual return calibration through the GPU path was not independently re-run in EQ-0. G0 camera
probe/renderer parity is not a LiDAR calibration test.

The LiDAR uses mesh nearest-hit ranges, plus an analytic target sphere radius **0.20 m**, unlike
camera/perception radius **0.15 m**. Target overrides only a farther scene hit
(`warp_lidar_kernels.py:182–251`); range clipping and normalization happen in `warp_sensor.py:220–243`.
This is a virtual target, not a common physical mesh observed identically by both modalities.
A directly aligned return from a target center at distance d is d-.20; association adds .15,
creating an isolated -0.05 m center-range bias. Other ray offsets can change its sign/magnitude.
This is a known model mismatch, not proof of every episode's realized bias; raw LiDAR arrays
were not saved in the OCC trace.

Independent sparse-ray counterexamples: a clear radius-.20 target at bearing 2.5 degrees (halfway
between horizontal rays) and elevation 0 has no returns at 5 or 12 m, although it is inside nominal
360-degree coverage and range. At bearing 0, elevation 5 degrees it also misses at 5/12 m.
Horizontal half-bin guaranteed interception ends around .20/sin(2.5deg)=4.585 m; vertical half-gap
limit is .20/sin(5deg)=2.295 m. Nominal range is not guaranteed target detection range.

Perception receives LiDAR metric ranges, not semantic ids (`navrl_task.py:6464–6469`). Target
semantic 50 is generated in the rendering kernel but not passed into this perception API.
`_associate_lidar_target` (`navrl_perception.py:1925–2003`) requires a previously active camera-
initialized track, no current camera flag, nearest predicted horizontal bin, any vertical minimum
range agreeing with predicted surface range within (0.35+2*position_sigma), capped at 1 m.
It cannot bootstrap a never-acquired target from LiDAR. It does not verify the returned object:
a bar at a compatible range/bearing can satisfy the gate. It reconstructs bearing and z from
the track's prediction, not a new LiDAR angular/height observation (`:1960–1978`). The default
full 3D covariance correction therefore is not justified as three independently measured axes;
the optional range-only update is OFF by default (`navrl_task_config.py:415`, absent receipts).
This is material before covariance-calibrated recovery or persistent-identity claims.

The five raw cells inspected contain 263,424 total recorded frame-env rows including nonprimary
tails; independently checking their flags gives zero fused != (camera OR final associated-LiDAR)
disagreements. This validates diagnostic lineage on that sample, not true-target identity.
Fused resumption is not validated physical re-identification.

## Occlusion semantics, parity and five numerical boundary frames

OCC primary is the in-frustum/in-range **center segment** intersecting recorded scene boxes.
For upright bars_h3, these boxes match the static collision/visual contract. The point is target
center, not front-surface visibility, a full silhouette, or a camera detection. Eight sampled rim
rays are explicitly only `geometric_full_in_view` proxy (`occlusion_geometry.py:1–16,85–122`).
The rim construction is a center-plane ring, itself an approximation to an exact sphere tangent
circle at close distance. Finite sampled rays cannot prove blockage of all unsampled image rays.

G0 report SHA ae17bfa538777639328b4554b3929b72f7d7c2acccf2422a79dfdb242ba19632 records PASS
noninterference on eight OFF/ON pairs, probe/box agreement 165,277/165,277, and renderer diagnostic
thresholds passed. Runtime source paths aerial_gym/motar_eval/resources/robots are identical from
G0 commit 6441c1f to OCC measurement source; receipt source hashes also match. Inherited G0 support
has this static-bar, zero-latency simulation scope; it does not qualify unrelated scene models.

P2 analysis parity aggregates 2,320,985 primary frame-env records: online-box disagreement 4,
mesh-probe disagreement 3. These span **five unique frames**, not seven independent failures.
Recorded image diagnostics are 86 zero-pixel frames among 1,568,239 clear/in-view probe frames
and 109 positive-pixel frames among 122,185 geometric-full-proxy frames. These are disagreements
between differently defined observables, not evidence that exact full silhouette was measured.

| Cell suffix / env,episode / frame | Independent face/FOV boundary | Interval counts offline/online/probe |
| --- | --- | --- |
| moving160 seed4309 / 53,3 / 6 | ray miss ~22 micrometers | 1/1/1 |
| static115 seed4307 / 60,0 / 32 | ray miss ~3.7 micrometers | 2/2/2 |
| static160 seed4307 / 94,3 / 33 | HFOV margin -0.199 micrometers | 0/0/0 |
| static205 seed4303 / 123,2 / 21 | ray overlap ~2.95 micrometers; outside FOV | 0/0/0 |
| static205 seed4305 / 78,3 / 4 | ray overlap ~16.6 micrometers | 1/1/1 |

These independent values cast center/half to double before computing faces; the earlier quick
diagnostic cast already-computed float32 faces and gave ~20.2/5.1/1.3/17.7 micrometers. Their
small difference is arithmetic-order sensitivity, not a changed recorded state. The miss/overlap
signs and primary frame labels agree. Three in-view cases shift a boundary by one 0.1 s frame;
no event counts change, no acquisitions/exposures change, and hence primary per-seed rates/tests
remain unchanged under replacing these labels with recorded online/probe alternatives.
The margins are consistent with floating-point grazing/FOV sensitivity; exact GPU rounding
causality remains unproven. EQ-0 does not reclassify or overwrite the frozen endpoints.

## Time and pose alignment

The task consumes observation k before advancing target and stepping physics (`navrl_task.py:5015–5058`).
After physics it resolves terminal outcomes and finalizes completed episodes before reset
(`:5145–5159`). For continuing episodes it syncs target to LiDAR, renders new sensor state, and
after task reset syncs/renders again (`:5163–5175`). Camera rendering and perception observe are
adjacent and use the same current p/yaw/target (`:6461–6491`). GT is rendered into pixels/ranges,
not passed as an actor input. The adapter stages those exact diagnostics/geometry, and writes them
only when the observation is consumed (`kroc_eval_adapter.py:1227–1255`, `:384–422`), preventing
one-step bookkeeping drift and false expiry caused by terminal reset.

All 112 metadata records attest ZERO_LATENCY_RENDER_DELIVERY; the guard checks configured latency,
latency steps and absent empirical injector (`kroc_eval_adapter.py:596–600`). The mesh probe reads
actual detector origins/quaternions/targets and own output buffers (`recovery_visibility_probe.py:220–256`).
Its matching opaque token is caller-attested stage correlation, not a native sensor exposure stamp
(`:204–209`). The source has no rolling exposure/scan clock/integration-duration model. No actual
capture timestamp or independent latency measurement was saved. Sim state simultaneity is supported;
hardware synchronization, real communication delay and timebase accuracy remain NOT_TESTED.

## Minimal separately approved EQ-1 proposal (no implementation or execution here)

1. **CPU oracle fixtures and sensor-contract receipt (3–5 development hours).** Add an independent
   pinhole + SE(3) reprojection oracle, exact sphere/box rays, pixel-phase grid, ray-grid uniqueness,
   active/noise-gate and radius-convention assertions. Sweep nominal forward LiDAR bearings with
   known landmarks. Acceptance for a calibrated fusion claim: correct pixel within 0.5 px;
   body-origin converted range within 1e-4 m for nongrazing, exact fixtures. Current linear mapping
   and omitted translation will fail; retain this outcome. A future fix requires a new sensing
   lineage / policy-observation compatibility review, never silently replacing F's frozen inputs.
2. **Explicit renderer sensor fixture capture (3–5 development hours; one RTX3070, roughly 5–15
   minutes GPU, no policy/PPO).** Separately authorize a deterministic fixture runner at modest
   environment count, with exact known target/occluder/camera poses and off-axis landmarks; save
   raw RGB, depth, ray tables, actual camera/scan poses and stage stamps. Compare actual returned
   pixels/ranges to the independent oracle. Proposed preregistered nongrazing tolerance 1e-4 m
   and 0.5 px; all such fixtures pass. Classify |geometric boundary margin|<=1e-4 m as boundary
   diagnostics, reported separately rather than selectively removed from the original experiment.
   Include reset and first-action frames to assert state/render/consumer frame indices exactly.
3. **Target-alone support and identity stress fixtures (2–4 development hours, same capture run).**
   Same-pose target-alone vs occluded renders must save nonzero-alone support before calling zero
   visible pixels full image occlusion. Distinguish in-view/out-of-view/subpixel-no-support/partial
   and full obstruction. Test LiDAR empty rays, half-bin misses, target/bar nearest-hit ordering,
   radius .15/.20 conventions, and a bar replacing the unseen target at predicted range. For a
   claim of true-target reacquisition, require no false target association in the designated
   target-absent control fixtures; otherwise explicitly retain only range-association semantics.
   Persistent re-ID and covariance calibration require their own criteria/observations, not an
   inference from this fixture alone.

Estimated minimum qualification work: 8–14 development hours plus CPU checks, <=0.25 GPU-hours
for a separately approved sensor fixture run. Hardware calibration/data collection is additional
and not estimated or authorized. EQ-0 artifacts do not count as EQ-1 execution. Until a future H-R
protocol is actually defined, list these dependencies without asserting that its scientific gate
has passed or failed. No new controller or sensor behavior is proposed for current OCC analysis.
