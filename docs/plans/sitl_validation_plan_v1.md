# SITL validation before real flight: can Policy F fly a PX4 vehicle? (plan v1 in the repository, 2026-11-05)

**Goal:** real-vehicle flight. **This plan's goal** is to check first, in Gazebo Classic 11 + PX4 v1.14.3 SITL, that
MOTAR's policy can close the loop through a real autopilot stack:
- PX4's estimator and controllers;
- offboard velocity setpoints;
- real latency.

It supersedes the ordering of `SITL_PLAN.md`, whose stage definitions it keeps. Every GPU step needs a committed
preregistration. Gazebo and PX4 run on the CPU (cores 6,7,12,13, `nice -n 10`, no CUDA), alongside GPU jobs under
D-2026-11-05-1.

**State (2026-11-05):**
- System deps are installed. PX4 v1.14.3 SITL + Gazebo Classic builds (`px4_ws/build_sitl.log`).
- Headless iris boots to "Ready for takeoff" (`px4_ws/sitl_smoke_a0.log`).
- MAVSDK 4.1.0 and pymavlink are in the `px4` conda env.
- The navrl_quad SDF and airframe drafts pass the CPU spec check, but are untested in Gazebo.

## Stages

| Stage | What | Where | Pass |
| --- | --- | --- | --- |
| **S1** PX4 offboard baseline (iris, = old Stage B) | A0 arm + offboard; A2 hover 60 s; A3 body-frame velocity steps at 10 Hz (0 → 1.0 → 2.5 → 0 m/s, x and y); A4 yaw-rate steps ±3 rad/s; A6 MAVSDK command round trip | CPU | A0 no failsafe; A2 alt RMS < 0.05 m; A3/A4 reported; A6 median and p95 recorded |
| **S2** training-spec vehicle (navrl_quad, = old Stage A) | install the model and airframe 10090; A0-A6; rate and attitude tuning recorded before and after | CPU | mass 0.25 kg ± 1 %; hover throttle explained by T/W 3.26; A2 as S1; A5 tilt ≤ 45°, speed ≤ 2.5 m/s |
| **S3a** observation equivalence | a bridge that rebuilds F's 898-D observation outside Isaac Gym from vehicle and target states and the bar layout, with MOTAR's own code: Warp LiDAR 72x4 and obstacle-camera kernels, `navrl_perception`, the detector, the 5-step histories, the riskcap governor. Compare it against Isaac's own observations and F's actions on recorded steps | CPU + one small GPU dump run (stride-1 observation dump, about 10 min) | observation max abs error ≤ 1e-4 per element (the TF32 hazard checked), F action error ≤ 1e-5, governor output equal |
| **S3b** closed loop, MOTAR sensors | Gazebo world from recorded B1-R `bars_h3` layouts; target moved by the bridge (E2 model) and mirrored into Gazebo; navrl_quad under PX4; the bridge senses with MOTAR's kernels against the known world, runs F + riskcap and streams 10 Hz body-frame velocity setpoints (altitude held by PX4). N episodes per density (70 and 205 bars), with initial states copied from Isaac cells | CPU (F inference on CPU) | capture and crash reported against Isaac on the same initial states; "feasible in SITL" if crash ≤ Isaac + 5 pp at 70 bars (preregistered at S3b) |
| **S3c** latency / dynamics gap | S3b with iris instead of navrl_quad, and with an injected delay of 1 step | CPU | the gap reported; it decides whether GR-1b/GR-2 policies are needed |
| **S4** sensor realism | Gazebo `gpu_ray` LiDAR and depth-camera plugins replace the bridge's ray casts; noise, dropout and detector input from Gazebo RGB (domain gap) | GPU (rendering), when free | reported; detector recall in Gazebo |
| **S5** real-vehicle readiness | a Gazebo model of the user's real vehicle; onboard compute and latency budget; geofence, kill switch and RC override; companion-computer bridge | CPU, then hardware | checklist; no real flight before S3b and S5 pass and the user decides |

## GPU jobs this plan adds to the queue

| Item | What | GPU |
| --- | --- | --- |
| G8 | stride-1 observation and state dump of Policy F (a few episodes, 70 and 205 bars), the S3a reference | about 10 min |
| G9 | MOTAR Lee-controller step responses in Isaac (the A3 sequence), the reference for S1 and S2 | about 10 min |

## Needed from the user (for S5; not blocking S1-S3)

The real vehicle:
- frame and mass;
- motors and propellers (thrust curve);
- flight controller and PX4 version;
- companion computer;
- LiDAR model (MOTAR assumes 72 x 4 beams at 12 m);
- depth or RGB-D camera (MOTAR assumes 87° HFOV);
- the flight area.

The training vehicle (0.25 kg, T/W 3.26) differs a lot from typical real frames (iris 1.5 kg, T/W 1.88), so S3c and
GR-1b/GR-2 matter.

---

# MOTAR ↔ PX4 SITL bridge, design v0 (2026-11-05)

Source of the facts: a read-only code map of the 898-D observation path (Opus Explore agent, 2026-11-05; file:line
references in this session's transcript), `SITL_VALIDATION_PLAN_v2.md`, and S1/S2 results.

## What Policy F needs, and what it does not

**Facts that shape the bridge:**
- **No randomness in F's observation.** Perturbations are off (`cfg_perception_perturb=False`). Dropout 0.3 and
  depth noise 0.02 are inert. The detector is the fixed colour segmenter sigmoid(3R−2G−2B−0.9), not a learned
  model. So step-by-step equivalence is testable; GPU TF32 only needs a tolerance.
- **Contract values the bridge must set:**
  - `NAVRL_OBSTACLE_SELECTOR=cluster_sector` (the code default differs);
  - max velocity 2.5, yaw-rate max 3.0, LiDAR 72x4 at 12 m, camera FOV 87°, detector range 20 m;
  - flight altitude 1.0 m (altitude PI: e = 1.0 − z).
- **Observation (898):**

  | Slice | Content | History |
  | --- | --- | --- |
  | 0-287 | fused static scan | — |
  | 288-767 | 5 × 8 obstacle slots × 12 | 5 |
  | 768-817 | 5 × robot (10): body velocity, yaw rate, prev_action (4), z/3, 1 | 5 |
  | 818-897 | 5 × target track (16) | 5 |

  - Histories shift every 5 steps on a **global** `step_count % 5`. Slot 4 is always "now".
  - The model normalises the vector with the checkpoint's `running_mean_std`, clamped to ±5.
- **Action:** mean × (1, 0.4, 1, 1), tanh, then velocity = a[0:2] × 2.5 in the vehicle frame and yaw = a[3] × 3.0.
  The riskcap governor runs on the **raw** LiDAR, with `last_target_like` returns removed. `prev_action` is the
  governed xy over 2.5, plus the raw a_z and a_yaw.

**Real-flight implication found here:** F's target "detector" is a colour rule for a red sphere rendered by MOTAR. A
real target needs a real detector that delivers the same measurement:
- visible, u, v, surface range, confidence, pixel count;
- then the same Kalman tracker.

This is a sim-to-real blocker of its own (S5), separate from dynamics.

## Bridge v0 (S3b): Gazebo/PX4 vehicle, MOTAR sensors computed in the bridge

| Part | v0 choice | Reuse |
| --- | --- | --- |
| Scene | one recorded `bars_h3` layout per episode (B1-R or GR-0 traces), written as a Gazebo world of static boxes, and as a Warp mesh for the bridge's sensors | `motar_eval/occlusion_trace.py` layouts |
| Target | v0: replay the target trajectory recorded in Isaac for the same episode (`t_pos` per frame). E2's response to the pursuer is lost, a disclosed approximation. v1: run MOTAR's target motion model in the bridge | traces |
| LiDAR | Warp CPU kernel against the merged mesh plus a 0.20 m target sphere; production range limits | `MOTAR-cplanner/cplanner/motar/replay.py` |
| Camera | Warp CPU obstacle-depth 40x24 (far 10 m), upsampled to 160x90, and a 0.15 m target sphere at 160x90 (far 20 m); RGB as production | `navrl_detector.py` kernels by file path (to be confirmed, as the sensor check did) |
| Perception | `NavRLPerceptionModule` on CPU, unchanged | `tools/audits/eq1_common.production()` |
| Policy | CPU player with Policy F's checkpoint and settings | `tools/navrl_reflection_offline_audit.build_player` |
| Governor | `speed_governor` riskcap on CPU, raw LiDAR | same module |
| Vehicle state in | PX4 `LOCAL_POSITION_NED` and `ATTITUDE` (estimator output, not ground truth), converted to MOTAR's world frame | S1 client |
| Command out | 10 Hz in PX4 sim time: body (vx, vy) rotated to NED, z position 1.0 m (PX4 holds altitude in place of MOTAR's PI), yaw rate | S1 client |
| Episodes | restart SITL per episode (about 30-40 s each); start pose and yaw copied from the Isaac episode | — |
| Crash / capture | crash = the vehicle's 0.28 x 0.28 x 0.08 m box meets a bar (geometric check each step, plus a Gazebo contact as a cross-check); capture = MOTAR's capture rule (0.5 m) | MOTAR definitions |
| Timing | lockstep SITL. The bridge acts every 100 ms of sim time. If its compute exceeds the budget, Gazebo's real-time factor is lowered. RTF and compute time are recorded | — |

## S3a equivalence before closing the loop

1. **G8 (GPU, about 10 min):** Policy F, condition `full`, 70 and 205 bars, seed 4501, quota 1, with
   `NAVRL_OBS_DUMP` (stride 1, MAX raised) and `NAVRL_KROC2027_OCCLUSION_TRACE=1`.
   - The dump gives obs[898] per env per step; the trace gives states and layouts.
   - Missing for the check: yaw rate, prev_action, the global step phase, tracker state.
   - Either add a small opt-in sidecar recording those, or derive them: yaw rate by difference, prev_action from
     the bridge's own actions, phase from `call_index − ep_step`.
2. **Replay on the CPU:** feed the recorded states, step by step from each episode's start, into the bridge's
   sensors, perception and policy. Compare with the dump.
   - Gate: per-element |Δobs| ≤ 1e-3 (TF32 allowance, tightened where it can be) and |Δaction| ≤ 1e-4.
   - Positive control: perturb one input (a yaw offset of 1°), and the gate must fail.
3. **Then S3b:** the closed loop in SITL on the same layouts and initial states. Capture and crash are compared with
   the Isaac episodes.

## Dynamics gap measured so far

See `docs/evidence/sitl_2026-11-05/README.md`; it has the run table and the fixes: ODE damping, thrust model and
the norm speed limit.
