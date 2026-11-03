# Preregistration: C-Planner M2 — MOTAR 72x4 status mapping (V1) and certifiability on replay (G-alpha)

Fixed 2026-11-04 before the gate measurements. Charter: [`cplanner_charter_v1.md`](cplanner_charter_v1.md)
(milestone M2, gate G-alpha in §3). Entry condition: R2' (`prereg_cplanner_r2prime_v1.md`) verdict CONDITIONAL and
the independent review R3' with no in-scope P0-blocking finding; if R3' fails, these measurements are not run.
CPU only, `taskset -c 0-3,8-11,14-23`, at most 12 workers, kernel fault record compared before and after every
command (charter §7). No GPU, no production code change.

## Code

- `cplanner/motar/replay.py`: CPU replay with MOTAR's own kernel
  `LidarWarpKernels.draw_optimized_kernel_range_segmentation_target` (file
  `aerial_gym/sensors/warp/warp_kernels/warp_lidar_kernels.py`, loaded by path because importing `aerial_gym`
  imports Isaac Gym), Warp CPU device, static mesh of the recorded active bars (12 triangles per box, recorded
  centre, collision half extents and quaternion), analytic target sphere r = 0.20 m at the recorded target
  position, far plane 12 m; ray vectors, range limits and normalisation repeat `warp_lidar.initialize_ray_vectors`
  and `warp_sensor.apply_range_limits / normalize_observation` in float32.
- `cplanner/motar/adapter.py`: normalised value -> status: 1.0 NO_HIT (range 12), -1.0 NEAR_CLIPPED, non-finite
  SENSOR_ERROR, otherwise HIT at 12 v - 1 mm (`HIT_MARGIN_M`); a shortened range below 0.2 m is NEAR_CLIPPED.
  Health RUNTIME, noise 0, timestamp = frame x 0.1 s, sensor position = recorded `p_pos`, yaw from `p_quat`
  (`robot_orientation`) computed as production does (`replay.sensor_yaw`: float32 `get_euler_xyz` yaw `% 2 pi`,
  then `quat_from_euler_xyz(0, 0, yaw)`), wrapped to [-pi, pi] for the certifier header.
- Certifier: `cplanner/src/` unchanged (R2' code), configuration `MOTAR-72x4-RUNTIME` (runtime health only, no
  nearest-hit assurance, arena 0-40 m).

## Replay fidelity facts (read-only code survey, 2026-11-04; file:line in the M2 record)

- The B1-R Warp scene holds only the 300 bar slots per env: no ground plane, walls, distractors or physical target
  (`create_ground_plane = False`; `NAVRL_DISTRACTOR_COUNT` 0; target dynamics `bounded`). Inactive slots
  (index >= `NAVRL_NUM_BARS`) are moved to (-1000, -1000, -1000) at every reset (`asset_manager.py:121`), out of
  reach of a 12 m ray, so the replay omits them.
- Bar meshes are the URDF visual boxes (`trimesh.creation.box`, 12 triangles), the same size as the recorded
  collision half extents; orientation identity; mesh refit only at reset. The trace layout is read from the same
  tensor after the refit.
- Kernel: `draw_optimized_kernel_range_segmentation_target` with far plane 12 m, miss value 1000, target sphere
  radius 0.20 m (`navrl_lidar_config.target_radius`; the trace metadata's 0.15 m is the camera detector proxy),
  oriented-box target off. The kernel's target centre is `target_position`, recorded exactly as `t_pos`.
- Sensor position = `robot_position` (`nominal_position` is not read; no placement randomisation); sensor
  quaternion = yaw-only from `robot_orientation` (= `p_quat`; `p_vehicle_quat` lags one physics substep).
- Frame k's scan is rendered after the last state change of that step, from the recorded `p_pos`, `p_quat`,
  `t_pos` and the episode's reset layout.
- Production adapter note for later milestones: the status must be derived from the sensor buffer
  `depth_range_pixels` before `NavRLTask._lidar_distance_m`, which maps NaN/inf to 12 m and near-clipped to 0 m.

## V1 — status mapping on replayed scans (`cplanner/tests/v1_hitbit.py`)

Sample: 400 frames per cell (sampling seed 20,261,104) from 6 cells: `A/F_SHADOW` at 70, 115, 160 and 205 bars
and `C-PP/B-PP` at 160 and 205 bars, seed 4301. Every one of the 288 beams of every sampled frame is compared with
an independent float64 ray caster (oriented boxes and the target sphere; no shared code with Warp, the replay or
the certifier), cast along the certifier's own manifest direction rotated by the declared yaw.

PASS iff all hold:
1. 0 non-finite values; 0 beams with value 1.0 while the oracle hits within 12 m (outside a 0.1 mm band at 0.2 m
   and 12 m, which is counted and reported); 0 beams with -1.0 while the oracle is beyond 0.2 m (same band);
   0 HIT values without an oracle hit;
2. every HIT value within 0.5 mm of the oracle distance (half the adapter margin);
3. soundness of the declared evidence: 0 HIT beams whose declared range exceeds the oracle distance along the
   manifest direction; the adapter status equals the mapping above on every beam;
4. positive controls: HIT > 0, NO_HIT > 0, target-sphere hits > 0.
The maximum beam-direction difference between the float32 kernel ray and the float64 manifest ray is reported.

## G-alpha — certifiability on replay (`cplanner/tests/galpha_replay.py`)

Sample: 6 episodes per cell (sampling seed 20,261,105) from 28 cells: `A/F_SHADOW` and `C-PP/B-PP` at 160 and
205 bars, seeds 4301-4307 (168 episodes). Every recorded frame of an episode is replayed and ingested in frame
order by one certifier core. A decision step is a frame with an even index, an accepted packet, and the target
centre more than 0.9 m away. At each decision step 16 segments start at the vehicle: headings 0, ±20, ±40, ±70 and
180 deg from the bearing to the recorded target position, lengths 1 m and 2 m. A segment is progressing if its end
point is at least 0.1 m closer to the target. A decision step is certifiable if at least one progressing segment is
STATIC_BAR_CERTIFIED.

Reading (charter §3, pooled over the 28 cells):
- any FALSE_CERT (a certified segment within 0.45 m of an active bar, exact 2-D distance): **STOP_FALSE_CERT**;
- certifiable rate >= 60 %: **FILTER_ARM_RETAINED**; 20-60 %: **MONITOR_ONLY_PILOT**; < 20 %: **FILTER_PAUSED**;
- incomplete run: **FAIL_INCOMPLETE**.
Reported, not gated: rates per arm and density, per target-distance bin, refusal reasons, and step / certify
times (measured with 12 concurrent workers; G-beta is measured separately in M3).

## Limitations fixed in advance

- The replay is a reconstruction. Raw GPU scans were never recorded, so CPU-replay equality with the delivered GPU
  scan is argued from the shared kernel and inputs, not measured. Differences can come from float32 arithmetic
  on CUDA versus CPU and from anything in the GPU scene the layout does not record (see the fidelity notes in the
  M2 record).
- The intercept proxy is the recorded target position, which the planner will not have. The trajectories are F's
  and B-PP's, not C's.
- `MOTAR-72x4-RUNTIME` has no nearest-hit assurance, so the first 0.2 m of every beam is never free evidence.
- The 12 m NO_HIT end point is not shortened (the core requires the exact fill); oracle hits within 0.1 mm of
  12 m are counted. Candidate segments are at most 2 m long and start at the sensor.

## Disclosed development runs (not evidence)

`~/motar_tmp/cplanner_m2_20261104/`: `probe/` (the kernel runs on the Warp CPU device: a box face at 2.85 m read
2.8500001 m, the target sphere at 4.8 m read 4.799998 m); `dev/` V1 on 20 frames of one 160-bar cell (5,760
beams, no status mismatch, maximum range error 5.2e-5 m, maximum direction difference 1.5e-7 rad; the first
tolerance of 1e-5 m was too tight, which set the 1 mm margin and the 0.5 mm tolerance above); G-alpha on 2
episodes of one 205-bar cell (50 decision steps, certifiable rate 0.94, 0 FALSE_CERT; this is above the
60 % threshold, which was fixed in the charter before this run). After the production yaw computation was copied
into the replay: V1 on the same 20 frames passed (maximum range error 5.3e-5 m, direction difference 1.1e-7 rad);
G-alpha on 1 B-PP episode (160 bars, seed 4302) read 0.95 with 0 FALSE_CERT.
