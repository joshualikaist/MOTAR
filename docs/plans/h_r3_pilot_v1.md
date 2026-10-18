# H-R3 pilot v1: active translation for reobservation after occlusion (preregistration)

Status: preregistered before any H-R3 code or measurement. Base `b9c5839` on `codex/mt-p1`.
Machine-readable contract: [`h_r3_pilot_v1.json`](h_r3_pilot_v1.json). Authority: the user's
2026-10-18 "Fast Research Hypothesis Validation: H-R3 Pilot". **This is an exploratory pilot.**
Its sample size is not justified by statistical power, it makes no confirmatory claim, and H-R3
stays PROVISIONAL, with no PPO and no confirmatory run.

## Question

When a moving target is hidden by a static obstacle, does actively translating to a reobservation
viewpoint give additional benefit over prediction-only and yaw-only control?

## Arms

All arms run Frozen Policy F unchanged until a branch point. From the branch point, for at most the
window W, each arm acts as follows:

| Arm | Behaviour in the window |
| --- | --- |
| B0 | Frozen Policy F, unchanged. |
| B1 Prediction-only | Scripted pursuit toward the **online tracker belief** (KF position). Yaw rate 0; no lateral visibility manoeuvre. |
| B2 Prediction + yaw-only | B1, plus a yaw-rate command that points the camera at the belief. |
| B3 Prediction + active translation | B2, plus a lateral velocity toward the nearest candidate viewpoint whose line of sight to the belief clears every raw-LiDAR obstacle point. |

B1, B2 and B3 are one controller with two switches, so they share:

- the same inputs: own pose and velocity, the tracker belief, and the raw LiDAR scan from the body
  origin;
- the same pursuit speed;
- the same candidate evaluation, which runs in every arm (equal compute);
- the same LiDAR safety filter;
- the task's unchanged riskcap speed governor and altitude hold.

None of them reads target GT, occlusion classes, renderer masks or LiDAR semantics. After
reobservation or the end of W, control returns to Policy F.

## Matching (deterministic replay)

`get_env_state` and `set_env_state` restore only the curriculum state. They do not restore the
physics, tracker, RNG or controller, so a snapshot fork is **not** used. Each arm is instead a
separate process with the same seed, environment and Policy F.

- **Branch point (online signal only).** For each env, the branch point t0 is the first step in its
  first episode at which the camera detection drops (true at t0−1, false at t0) after at least one
  detected step, with t0 + W within the run.
- **Matched events.** An event counts as matched only if the per-env, per-step digests are
  identical in all four arms for every step before t0, and t0 is identical. The digests cover
  actor observation, pursuer state, target state, tracker state and the camera flag. Unmatched
  events are counted and reported; they are not analysed as matched.
- **One event per env.** The env diverges after its branch point, so it contributes at most one
  event.
- **Target independence.** Target trajectories in the window are compared across arms. Any
  dependence on the arm is reported.

## Event classes (evaluator only)

The E-L0 class at t0 divides the events:

- **Geometric occlusion:** FULL, CENTRE_ONLY or PARTIAL. This is the primary population.
- **Separate:** FOV_EXIT, RANGE_EXIT, SCALE_LOSS, and VISIBLE (a detector miss). These are
  analysed separately and never pooled with the primary population.

## Primary metric

The primary outcome is **identity-correct camera reobservation within W = 20 steps (2.0 s)**. It
counts as achieved if, for some j in 0…19, the observation after the action at t0+j has a camera
detection and E-L0 renderer target pixels ≥ 1.

- **Denominator:** all matched geometric-occlusion events, including those that end in a crash,
  capture or reset inside W. Those count as failures unless reobservation happened first.
- **Contrasts:**
  - B3 vs B2: the added effect of translation;
  - B3 vs B1;
  - B2 vs B1;
  - each scripted arm vs B0.
- **Reported statistics:** paired exact McNemar tests, a paired bootstrap 95% CI of the
  difference, and Wilson intervals, all exploratory.
- **Natural unocclusion:** reobservations that also occur in B1 (no manoeuvre) are attributed to
  natural unocclusion. The B3 − B1 and B3 − B2 discordant pairs are what the manoeuvre adds.

## Secondary metrics

- time to reobservation;
- collisions in W, and collision-free reobservation;
- the change in pursuer–target distance over W;
- pursuer translation distance in W;
- FOV retention (evaluator centre-in-frustum fraction of W);
- observation source (camera or LiDAR);
- capture, timeout and incomplete status at W and at W + 3 s;
- controller compute time (mean, P95, P99).

## Run

Each arm is one process with the same configuration:

- moving `E2_direct_l2_v1` receipt, density 160, legacy sensor lineage (off), N = 128, 600 steps,
  seed 4502;
- if fewer than 20 matched geometric events result, seed 4503 runs once more, for a total of at
  most 2 seeds and 8 processes.

The run uses the S-L1 supervision:

- watchdog;
- launch-time GPU cap (measured free memory − 512 MiB);
- a stop below 8 GB free disk;
- frozen source and tree;
- no retry.

**Safety gate (smoke).** Before the pilot, a smoke run uses N = 16 and 300 steps. If any scripted
arm collides in more than 30% of its windows, the intervention does not run and only shadow-mode
outputs are reported.

## Declared limits

- **No physical safety claim** (P-L1 and G-L1 are not done). The existing crash rule and the
  speed/brake limits apply equally to all arms and are recorded.
- **Legacy sensor errors.** Policy F (B0) sees the legacy sensors, including the A1/A2 fusion
  errors, the A7 camera bias and the B-carve retention issue. The scripted arms see the same
  tracker. Their obstacle geometry comes from raw LiDAR.
- **Not used:** S-L1C/D are not used, and D-v1's classes are never a planner signal.
- **Belief error.** Belief error is real and is not removed.
- **Scope.** A scripted B3 tests the viewpoint idea, not an optimal planner. The result is one
  pilot seed, so effect sizes are uncertain by design.
