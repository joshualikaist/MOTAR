# After P0-R: interfaces, remaining work and research strategy (DESIGN ONLY)

Date 2026-10-29. Nothing below is implemented or approved. W = `src/MOTAR-mt-p1` (codex/mt-p1 @ 15c6764),
M = `src/aerial_gym_simulator`. Source facts come from a read-only survey (file:line given).

## 1. MOTAR 72x4 sensor connection (answers Q5)

**Where the hit/no-hit information is lost** (per ray, in order):

| Stage | Function | What happens |
| --- | --- | --- |
| Kernel | `LidarWarpKernels.draw_optimized_kernel_range_segmentation_target` (`warp_lidar_kernels.py:167-251`) | nearest mesh hit, overridden by the analytic 0.20 m target sphere if nearer; **miss written as 1000** (`:201`); `segmentation_pixels` keeps -2 for a miss (`:202`) |
| Limits | `WarpSensor.apply_range_limits` (`warp_sensor.py:220-238`) | > 12 -> 12 (**miss and "exactly 12 m" become identical**, `:236`); < 0.2 -> -12 (`:237`) |
| Normalise | `normalize_observation` (`warp_sensor.py:240-243`) | / 12 -> [-1, 1]; written to `obs_dict["depth_range_pixels"]` |
| Task | `NavRLTask._lidar_distance_m` (`navrl_task.py:6013-6017`) | `nan_to_num(nan=1, posinf=1, neginf=1)` then clamp [0, 1] x 12: **NaN/inf become 12 m (looks like no obstacle)**, near (-1) becomes 0 m |
| Perception | `_fuse_static_and_extract_obstacles` (`navrl_perception.py:1835-1925`) | target-like bins (+-15 deg, +-0.55 m) carved to 12 m, then min-fused with the camera depth columns: not evidence |

**Channels a certifying adapter needs** (packet = the P0-R arrays):
- `ranges[4, 72]` and an explicit `status[4, 72]`. Read `depth_range_pixels` **before** `_lidar_distance_m`
  (NaN still distinguishable: -> INVALID; -1 -> NEAR_CLIPPED; 1.0 -> NO_HIT only after V1 qualifies "1.0 <=> no
  intersection within 12 m"; otherwise HIT at 12 x value). Until V1 passes: `DISTANCE_ONLY` -> nothing certifies.
- Alternatives for the hit bit, compared:

  | Option | Information | Cost / risk |
  | --- | --- | --- |
  | A. Derived from the value after CPU V1 qualification | physical hit/miss of this sensor model only | CPU only; an intersection at exactly 12.0 m stays ambiguous (measure zero, declared) |
  | B. Production per-ray flag in the warp kernel | exact | production change + approval; GPU re-qualification |
  | C. `segmentation_pixels != -2` | exact hit bit, but the same array carries the semantic id (GT-privileged, `Q02A_SPEC.md:59`) | policy risk: must strip the id inside the adapter; needs an explicit user decision |
  | D. Keep AMBIGUOUS | none | C cannot certify on MOTAR; P1 stops at evidence mapping |

- Header: `sim_steps` (acquisition step; time = 0.1 x step, `DECLARED_SIM_SCHEDULE`), env/episode ids, robot
  (x, y, z, yaw) of the same post-step state (LiDAR is yaw-only at the body centre, `warp_sensor.py:184-212`).
- Camera (labels only): detection bearing/range with the full camera extrinsic ([0.10, 0, 0.03] body, full attitude,
  `navrl_detector.py:820-831`) expressed in the yaw-level LiDAR frame by the adapter.
- Manifest check in V1: row order (+20, +10, 0, -10) and column azimuth 180 - 5 j deg (`warp_lidar.py:76-84`) are
  already encoded in `MOTAR-72x4` and match the warp formula exactly (`results/r2/check_static.json`).

## 2. Controller and Governor error in the certificate (Q6)

- The chain is action x 2.5 m/s (vehicle frame) -> riskcap cap 2.0-3.54 m/s on the xy magnitude (direction kept) ->
  altitude PI -> Lee velocity loop accel = 2.5 (v_cmd - v), 45 deg tilt limit (`navrl_task.py:4953-5009,5262-5365`;
  `speed_governor.py:189-255`; `lee_controller_config.py:24-45`; `velocity_control.py:20-97`).
- Consequence: the vehicle follows a first-order velocity response (time constant about 1/2.5 = 0.4 s), so a heading
  change of dtheta at 2.5 m/s displaces the actual path by up to about v tau sin(dtheta) (about 1 m for 90 deg). A
  straight commanded segment is therefore NOT the flown path unless it is aligned with the current velocity.
- Plan (G2): certify the **predicted** closed-loop trajectory, not the command. Represent it as a polyline of short
  segments and certify each with `_coverage_B` at radius R' = r_body + eps(t) + chord error, where eps(t) is an
  empirically bounded tracking error (high quantile + margin) from eval-only logs of command, governor output, Lee
  command, position, velocity and yaw per step; the budget today is 0.45 - 0.202 = 0.248 m. Known data: braking from
  2.49 m/s stops in 0.9-1.0 s over 1.04-1.05 m (`results/navrl_v2_speed_governor_braking.json`); motor tau 0.04 s
  (EQ-1B); step response, tracking error and delay: NOT_RECORDED. A CPU replica of the Lee/governor equations can
  design the model, but its validity needs the GPU logs (approval).
- The riskcap governor only scales speed along the commanded direction, so it shortens the flown prefix of a
  certified straight segment and does not leave it; the Lee response does.

## 3. 100 ms feasibility (Q7)

Measured on CPU, single process (`results/r2/timing_p0r.json`, `results/r2/regress_v7.json`): see the report. The
coverage core is pure Python; its per-segment cost has a long tail (p99 above 100 ms, max about 0.7 s), so 8-32
candidates per step do not fit 100 ms in this form. Options, none of which may weaken the certificate:
1. **Per-packet exclusion grid.** Compute once per packet which bar-centre cells are excluded (the same `covered`
   predicate), then each candidate only looks up its colliding cells: O(cells near the path) per candidate. Needs a
   proof that it equals `_coverage_B` cell by cell (an equivalence test against the verbatim core).
2. **Early rejection** (already in place: arena, map veto, observed-surface test before coverage) and candidate
   ordering by expected cost; a hard per-step node/time budget with `DEADLINE_UNKNOWN` (in place between candidates;
   a single coverage call is bounded only by its node budget).
3. **Vectorisation** with numpy over cells (numba is not installed in the env; adding it needs approval).
4. **Fewer, better candidates** (G1: receding horizon 3 s, replan 0.1 s, a small lattice first).
Simulation time does not advance during computation, so in simulation the budget is a throughput limit (B1-R scale
is about 17 M decisions); a real-time claim needs measured p95/p99/max and deadline-miss rates of the full chain.

## 4. What remains after P0-R (Q8) and the stage plan

| Stage | Development goal | Entry condition | Status |
| --- | --- | --- | --- |
| P0-R | array boundary, static-bar certificate, veto-only map, camera labels only | current CPU work | this package |
| P1 | MOTAR adapter (V1 hit-bit qualification, CPU warp replay), 72x4 frame, accumulated map on replayed B1-R poses/layouts | P0-R PASS + approval | not started |
| G1 | receding-horizon candidate generation toward the predicted target (Kalman CV prediction), multi-segment certified trajectories, fallback command when everything is refused | P1 contracts | not started |
| G2 | dynamics/tracking-error identification (eval-only GPU logs) and the robust swept region | log approval | not started |
| G3 | real-time budget, reacquisition/search behaviour, terminal mode (`TERMINAL_INTERCEPTION_SPEC.md`) | G1/G2 | spec only |
| G4 | GPU shadow mode (C computes alongside frozen F, does not act) | earlier gates | not started |
| G5 | closed-loop matched comparison with F, B-PP, B-PN | separate approval | not started |

Implemented C-Planner functionality today, by component (adapter, map, static certificate, tracking, prediction,
candidate generation, dynamics model, terminal mode, reacquisition, real-time budget, integration, closed-loop
evaluation): P0-R covers the map (minimal) and the static certificate on synthetic data, plus the boundary the
adapter will feed; tracking exists in MOTAR (reused, not new). Roughly 2-3 of 12 components at prototype level, 0 of
them connected to MOTAR data or closed loop.

## 5. Track A versus Track B (Q9)

| | Track A: certificate-centric | Track B: practical predictive baseline |
| --- | --- | --- |
| Core | every executed segment STATIC_BAR_CERTIFIED (P0-R) | map + collision prediction + controller model, no proof |
| Claim | conditional static-bar safety per segment, explicit scope | none; empirical crash/capture rates only |
| Strength | no wrong obstacle exclusion by construction | moving-target pursuit and capture testable soon; PPO comparison possible |
| Weakness | refuses under UNKNOWN; capture is uncertified; needs V1 hit bit, G2 dynamics, G3 speed-up | cannot be called a safety planner; must disclose priors and information access |
| Earliest closed-loop result | after P1 + G1 + G2 + G4 (several approvals, GPU) | after P1 adapter + G1 (+ GPU run approval) |

Information access (must be in any preregistration): F sees a target-carved, camera-fused 4x72 scan, a 5-step
history, tracker features, no raw image (`navrl_perception.py:2346-2351`); B-PP/B-PN see exact own state, the tracker
mean/active flag, the raw `_lidar_distance_m()` scan and `last_target_like`, and command through the same 4-D action,
governor and altitude loop (`prereg_b1r_classical_baseline_v1.md:61-75`). C (either track) would see the raw scan
status channel (after V1), the declared pose, the tracker, and camera labels; the bar catalog and its axis-aligned,
full-height prior are extra prior knowledge that F does not get and must be disclosed.

**Recommendation.** For the KRoC timeline (internal 2026-11-13, submission 2026-11-26; the 2026-10-28 simulation
freeze has passed) a certificate-centric closed-loop result is not realistic: it needs V1, a planner, a dynamics
bound and GPU runs, each separately approved. The schedule-compatible path is a **hybrid**: a Track-B predictive
planner for the PPO comparison, with the P0-R certificate run alongside as a monitor that labels each executed
segment certified/uncertified (reported, never claimed as overall safety). Track A stays the longer-term research
line. This is a recommendation; the choice is the user's.

## 6. Data and material

| Data | Status | Source / if missing |
| --- | --- | --- |
| LiDAR config, kernel, fills, ray order | AVAILABLE | W source (above) |
| Bar catalog (40 URDF, rpy 0, 0-3 m) | AVAILABLE | `W/resources/models/environment_assets/bars_h3`, `env_object_config.py:812-857` |
| Per-frame poses and obstacle layouts (B1-R, OCC) | AVAILABLE (read-only raw) | `M/results/b1r_classical_baseline_v1/**`, `occlusion_frames.bin`, `occlusion_layouts.npz` |
| Detector statistics | AVAILABLE | `M/results/navrl_detector_offline_gate_v7_confirmatory/summary.md` (range MAE 0.178 m, recall 0.9938); closed-loop profile `navrl_detector_v7_error_profile_seed419/profile.json` (range std 0.705 m); note F ran the bootstrap colour segmenter |
| Occlusion / reacquisition | AVAILABLE | OCC-P1 `analysis.json`, OCC-P2 `analysis.json` (median interval 0.9 s, p95 2.0-2.1 s) |
| Braking | AVAILABLE | `M/results/navrl_v2_speed_governor_braking.json` |
| Per-beam raw LiDAR sequences | MISSING | CPU warp replay from logged poses/layouts (P1, after V1); or an eval-only recorder (GPU approval) |
| Per-step command, governor output, Lee command, actual velocity | MISSING | eval-only recorder (GPU approval); CPU model of the equations meanwhile |
| Velocity step response, tracking error, delay | MISSING | as above |
| Sensor capture timestamps | MISSING by design | declared schedule (0.1 s x step) |
| Per-component compute timing in MOTAR | MISSING | only whole-step timing exists (42-76 ms per step at 32-128 envs) |
| Collision-free capture | NOT_RECORDED in B1-R | needed for terminal-mode evaluation |
