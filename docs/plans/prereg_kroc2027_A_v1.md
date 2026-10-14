# Preregistration — KROC2027 Experiment A: initial target acquisition (v1)

- **Written:** 2026-10-14, before any KROC2027 data exist. No KROC2027 GPU cell has run.
- **Authority:** the user approved package P1 of the KROC2027 plan v2 on 2026-10-14. Every GPU run
  (canary, main) still needs its own explicit approval.
- **Machine-readable contract:** [`kroc2027_protocol_v1.json`](kroc2027_protocol_v1.json),
  SHA-256 `ad706d7121fc213360b179796e45c70227f82b1afedff50fcc0fe60cf6e0c803`. Where this text and the
  JSON differ, the JSON governs and the difference is a defect to report.
- **Exploratory table:** [`kroc2027_pilot_v1.json`](kroc2027_pilot_v1.json),
  SHA-256 `6d463c178fed6e41724849c905b3fe7fcbf78ea89d0c3ab5b337f6ed85847dc7`.
- **Executable half:** `tools/kroc2027_eval.py`, `motar_eval/`, and
  `aerial_gym/task/navrl_task/kroc_eval_adapter.py`, in the commit that adds this file.
- **Policy:** Policy F, `last_gen_ppo_ep_25000_rew_39.742134.pth`, SHA-256
  `f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40`, evaluated unchanged by the
  pinned launcher `eval_navrl_target_behavior_arms.sh` (SHA-256 `4077e271…`). No training.

This file is a path contract once committed: it is never edited. Any change is a new, dated
amendment file that says what changed and why, and it must exist before the measurement it affects.

## 1. Questions and hypotheses

- **H-A (initial acquisition).** In a cluttered, partially observed scene, the pursuer's initial view
  of the target changes the final capture rate of the unchanged policy. Predicted direction:
  capture is higher when the target starts in view (Delta_A > 0).
- **H-A2 (interaction with target motion).** The effect of the initial view differs between a static
  and a moving target. Predicted direction: the static target loses more from a hidden start
  (I_A < 0), because a moving target can enter the field of view by itself. The prediction comes from
  already-observed data (section 2); the test stays two-sided.
- **H-B** (conditional active search) has its own gate here (section 9) and needs its own
  preregistration before any B cell runs.

## 2. Exploratory motivation from existing data (already observed; not confirmatory)

Source: `results/target_motion_replication_7seed/*/result.json` → `first_acquisition`, fused
measurement (camera, or LiDAR associated with a camera-started track). All 112 files were checked
against `RAW_MANIFEST.json` before re-aggregation (`python -B tools/kroc2027_eval.py pilot`). These
counts cover the legacy pooled stopping window (they equal `legacy_episodes` in every cell), not the
per-environment quota estimand. That window under-represents long episodes: the timeout rate is
lower in it than under the quota for every arm (table), so never-acquired timeouts are, if anything,
understated here. Starts used a uniformly random yaw at 6–28 m.

| Arm | Never acquired (SD over seeds) | NA and timeout | NA and crash | NA share of failures | Timeout, legacy / quota |
|---|---|---|---|---|---|
| E0_static | 13.79 (0.33) | 6.99 | 6.76 | 86.12 | 7.33 / 8.93 |
| E1_cv | 10.21 (0.44) | 3.83 | 6.34 | 69.85 | 4.49 / 5.47 |
| E2_obstacle_aware | 6.03 (0.23) | 0.93 | 5.03 | 57.65 | 1.14 / 1.52 |
| H_historical | 7.84 (0.19) | 2.23 | 5.56 | 68.06 | 2.61 / 3.29 |

Percent of episodes; seed-level means over 7 seeds with the four densities weighted equally. Pooled
over episodes instead, the last-but-one column reads 85.4 / 68.4 / 55.9 / 66.3. An early crash also
ends the chance to acquire, so "never acquired and crash" does not show that acquisition failure
caused the crash. This table motivated H-A and the predicted sign of I_A; it enters no confirmatory
test.

## 3. Design

| Arm | Target model | Initial view |
|---|---|---|
| A1 | E0_static | visible |
| A2 | E0_static | hidden |
| A3 | moving model (section 5) | visible |
| A4 | moving model (section 5) | hidden |

- Densities 70, 115, 160 and 205 bars; seeds 4201–4207 (no prior use in any seed context of this
  repository, checked on 2026-10-14); 4 arms × 4 densities × 7 seeds = 112 cells.
- Each cell: 128 environments; the first 16 completed episodes of each environment are the primary
  sample (2,048 episodes; per-environment quota). Later episodes are kept as `NONPRIMARY_TAIL` and
  never used. Timeout 600 actions; RL step 0.1 s; physics step 0.01 s.
- Primary metric: capture, the existing swept 0.5 m test (`capture_0p5m`), unchanged.
- Everything else is the frozen evaluation contract the pinned launcher exports (sensors, detector,
  governor, observation and normalisation). KROC2027 settings reach the task only through
  `NAVRL_KROC2027_*` variables, which the launcher does not touch.
- Exogenous streams are `UNPAIRED_STREAM`: the arms are independent samples from one sampler. Seed
  labels block cells by seed; they do not pair episodes.

## 4. Treatment: the initial view

For every arm, each episode's start is drawn by one sampler (`motar_eval/initial_view.py`), with one
acceptance rule:

- the target is exactly 10 m from the pursuer, at the same altitude (`flight_altitude`), in a
  uniformly random direction;
- it lies inside the arena inset by 1 m;
- it passes the drone-spawn footprint rule (`_spawn_footprint_clearance_accepted` with
  `_spawn_required_surface_margin_m`) against every solid obstacle, for static and moving targets
  alike, so all four arms share one target distribution;
- the centre ray from the camera (0.10 m forward, 0.03 m up on the yaw-only frame) to the target
  centre touches no obstacle box, evaluated with the camera facing the target in both views.

Acceptance never looks at a detector, a confidence or an outcome. Up to 64 candidate directions × 32
rounds are tried; a pursuer with no admissible target is respawned by the task's own sampler, at most
16 times, after which the reset fails closed and the cell stops.

Then `visible` points the camera boresight at the target and `hidden` turns the pursuer by 180°.
"Hidden" therefore means outside the 87° × 58° field of view, not occluded by an obstacle; occlusion
during the episode is recorded separately. The yaw also changes the ego heading and what the other
sensors see, so results are about the initial heading and observability condition, not a pure
visibility manipulation.

## 5. Target motion and its gates

- Static: E0_static. Moving candidates, in order: E2_obstacle_aware, then E1_cv. The historical H
  model (reflection and push-out paths) is not run.
- **Choice rule.** The first candidate whose canary gates all pass is the moving model for A3, A4 and
  B. E1 and E2 share the bounded executor, so an executor defect fails both. If none passes, A3/A4
  are not run and H-A2 is `NOT_TESTED`; a new motion lineage would need its own approval.
- **CPU precondition.** `tools/audits/audit_e2_target_validity.py`, unchanged, exits 0. It did on
  2026-10-14 (921,600 environment-steps over four densities, no violation of any kind).
- **G1** judges the generator's own velocity state in float64: speed at most min(1.5 m/s, the
  episode's nominal speed), acceleration at most 4.0 m/s², course change at most 150 °/s where the
  current speed exceeds 0.10 m/s, each with a 1e-4 relative tolerance (as the existing audit), a
  1e-6 m/s floor for zero limits, and a 1e-6 m/s band above the 0.10 m/s threshold in which turning is
  counted but not judged.
- **G2** judges the written-back float32 positions. With u = ulp(40 m) = 2⁻¹⁸ m, dt = 0.1 s and k = 2:
  speed ≤ 1.5 + 5.39e-05 m/s, acceleration ≤ 4.0 + 1.079e-03 m/s², position correction
  |Δp − v·dt| ≤ 5.39e-06 m, and course change ≤ 150 °/s + k√2·u/(v_min·dt²) where both
  finite-difference speeds are at least 0.20 m/s (0.31 °/s at 0.20 m/s). A position whose float32
  spacing exceeds u is its own violation. Derivation: `motar_eval/contracts.py`. With fixed 1e-4
  tolerances on finite differences, fault-free float32 motion at 31–41 m produced 349,313 false
  turn-rate and 2,402 false acceleration violations in 2.45 M steps; with these bounds, none.
- **Collision and wall.** The swept segment of the target centre stays at least 0.15 m (the rendered
  target radius) from every solid obstacle footprint, and the centre stays inside the arena.
- **On violation.** In the main run the cell stops and the first violation of each gate is kept with
  its state and episode key. In the canary the cell continues and the violation is recorded, so the
  choice rule can act.

## 6. Recorder and event definitions

`NAVRL_KROC2027_RECORDER=1` writes `kroc2027_episodes.jsonl` and `kroc2027_summary.json` next to each
cell's result. It reads detached tensors, draws no randomness and writes into no task buffer;
ground-truth geometry stays in these files and never reaches the actor observation.

- An observation is counted when an action uses it (index 1 drives the first action), the convention
  of the task's own first-acquisition telemetry; each episode carries both chronologies and any
  disagreement is a cell failure.
- Separate signals: camera measurement, fused measurement, tracker active (it coasts through misses
  and expires after its memory), ground truth "target centre inside the frustum", and ground truth
  "inside the frustum with a clear centre ray".
- Acquisition is the first fused measurement. A never-acquired episode reports `null`, never 0;
  latency in seconds is (index − 1) × 0.1. Losses and reacquisitions are counted for the measurement
  and for the track; a loss still open at the end is reported as censored.
- Distances: end-of-step minimum (as the ledger) and the swept minimum that decides capture, as
  separate fields.
- Speeds: requested, commanded and actual horizontal pursuer speed, the target's step speed, and
  counts of steps above the real pursuer's 2.5 m/s (descriptive; Policy F can command 3.54 m/s).

## 7. Analysis (fixed now)

- **Aggregation.** Per seed, the four density cells weighted equally. Inference unit: seed, n = 7.
- **Contrasts.** Delta_A = ½[(A1 − A2) + (A3 − A4)] and I_A = (A3 − A4) − (A1 − A2), in capture
  fraction, per seed.
- **Primary test.** Two-sided paired t over seeds (df = 6) for each; Holm over {Delta_A, I_A}; α = 0.05.
- **Decisions.** `INVALID` if any integrity check failed; otherwise `INCONCLUSIVE` if the
  Holm-adjusted p ≥ 0.05; `OPPOSITE_DIRECTION` if significant against the prediction; `SUPPORTED` if
  significant in the predicted direction; `PRACTICALLY_SUPPORTED` if, in addition, the Bonferroni
  97.5 % t interval clears 3 percentage points in the predicted direction. Whether it clears 1 and 5
  points is also reported. The threshold is not changed after seeing results.
- **Sensitivity.** BCa 95 % intervals (20,000 resamples, seed 4200) and exact sign-flip p values, from
  `tools/target_motion_replication.py` unmodified (SHA-256 pinned in the protocol). With 7 seeds the
  smallest two-sided sign-flip p is 2/128 = 0.0156, so it passes 0.025 only when all seven seeds agree.
- **Secondary:** crash, timeout, ever acquired, never acquired and timed out, first-acquisition
  latency among acquired episodes (reported with the acquisition rate), reacquisition latency, loss
  duration, both distances, both speeds; and the static-only and moving-only visible-minus-hidden
  contrasts.
- **Exploratory:** I_A on the log-odds scale (a risk-difference interaction can arise from a ceiling
  alone), per-density contrasts, conditional latency.

Planning power (paired t, n = 7, 80 %, two-sided; minimum detectable effect = 1.49 SD at α = 0.025 and
1.27 SD at α = 0.05; SD(I_A) = 2 × SD(Delta_A) for independent arms):

| SD(Delta_A) | MDE Delta_A (α = 0.025) | SD(I_A) | MDE I_A (α = 0.025 / 0.05) |
|---|---|---|---|
| 0.40 pp | 0.60 pp | 0.80 pp | 1.19 / 1.02 pp |
| 0.50 pp | 0.74 pp | 1.00 pp | 1.49 / 1.27 pp |
| 0.75 pp | 1.12 pp | 1.50 pp | 2.23 / 1.91 pp |
| 1.00 pp | 1.49 pp | 2.00 pp | 2.98 / 2.55 pp |

In the 2026-09-25 replication the seed-level SD was 0.39 pp for an average of two contrasts and
0.86 pp for a difference of two; hidden arms capture less and should vary more. These are planning
figures, not achieved power.

## 8. Canary (run id KROC27-A-C01, separate approval)

Seed 4190 (unused elsewhere), 4 quota episodes per environment except where stated. It checks
integrity, chooses the moving model and measures timing; its capture, crash and timeout counts are
recorded and never used for any decision.

1. **Off path:** E0_static, 70 bars, seed 4104, every `NAVRL_KROC2027_*` variable unset, ledger on,
   16 per environment: the ledger rows reproduce the 2026-09-25 replication cell byte for byte
   (`rows_sha256`). Simulator code is unchanged between that run's commit and this branch's base.
2. **Recorder non-interference:** A1 treatment, 70 bars, recorder off and on, trajectory digest and
   observation dump on: identical result (except the run nonce), digest, sampled actor observations
   and ledger rows.
3. **Motion gates:** A3 and A4 with E2_obstacle_aware and with E1_cv at all four densities.
4. **Instrumentation:** A1 and A2 at all four densities: one recorder record per ledger episode,
   outcomes and first-acquisition chronologies agree, import origins inside the hashed tree.
5. **Initial-view acceptance:** rounds and respawns per density; no fail-closed abort.
6. **B1 integrity** (the supervisor is in this source): hidden static arm with B1 on, 115 and 205 bars:
   the supervisor searched, and while searching never commanded horizontal motion or more than
   0.6 rad/s. A failure here blocks B, not A.
7. **Timing and scale rule:** the main run's hours are projected from canary seconds per quota
   episode × 16. If the projection exceeds 30 h − canary hours used − 4.9 h (B at two densities,
   stressed, plus smoke) − 2.0 h reserve, A runs at 115 and 205 bars only. Timing only.

The main run (KROC27-A-M01) refuses without a `CANARY_PASS` on the same commit.

## 9. Experiment B (conditional; summary only)

After the A analysis, `tools/kroc2027_eval.py analyse` selects the hidden arm with the larger
never-acquired failure rate (within 0.5 points: static), and `b-gate` evaluates once: A integrity
passed; in the selected pair, visible minus hidden capture and ever-acquired both have 95 % paired-t
intervals above 0; never-acquired failures exceed half of the selected hidden arm's failures; the
remaining GPU hours cover B plus 1 h; and a B preregistration is committed and the B run approved.
B compares B0 (Policy F) with B1 (Policy F plus the yaw-scanning supervisor) on the hidden view, seeds
4301–4307, four densities if the budget leaves 2 h after the stressed estimate, otherwise 115 and
205. Its primary test is capture(B1) − capture(B0), two-sided paired t, α = 0.05.

## 10. Stop rules

Stop on any hash, schema, receipt or import-origin mismatch; any motion-gate, swept-clearance or wall
violation; a non-finite value; an incomplete quota or a missing recorder record; or exceeding the
approved GPU hours. Never replace a seed or delete a cell because results look unfavourable.
Incomplete work is reported as incomplete.

## 11. GPU budget (RTX 3070 only; never pooled with the GTX 1650 Ti host)

| Item | If E2 is the moving model | If E1 is |
|---|---|---|
| Canary | 2.5 h | 2.5 h |
| A main, 112 cells | 15.7 h | 17.4 h |
| B main (static worst case) | 9.0 h, four densities | 4.65 h, two densities |
| B smoke | 0.25 h | 0.25 h |
| Reserve | 2.55 h | 5.2 h |

Planned = the larger of existing throughput × 1.25 and a Monte Carlo stress (hidden-arm timeouts
doubled; B1 adds 45 steps per episode). A fixed 10 m start is shorter than the past 6–28 m, so the
real time is likely lower.

## 12. Decisions made during implementation (not fixed by the plan text)

1. Canary seed 4190; canary quota 4 per environment; the off-path, non-interference and B1 checks
   above; both moving candidates gated in one canary.
2. Initial view: 64 × 32 candidates, 16 respawns, then fail closed; the footprint rule for every arm
   (stricter than the bounded executor's 0.77 m centre clearance), so static and moving arms share one
   target distribution; line of sight evaluated with the camera facing the target in both views.
3. G1: the existing audit's 1e-4 relative tolerance, a 1e-6 m/s floor for zero limits and a 1e-6 m/s
   heading band. G2: u from the declared 40 m arena bound; k = 2; turn judged from 0.20 m/s.
4. Collision proxy radius 0.15 m (`camera_target_radius`); wall = target centre inside the bounds.
5. Observations counted when an action uses them, cross-checked against the task's own telemetry.
6. Practical claims from Bonferroni 97.5 % intervals; B arm tie within 0.5 points goes to static;
   BCa seed 4200.
7. Main runs stop on a motion violation; the canary records it.

## 13. Limitations stated in advance

- "Hidden" is out of view, not behind a tree; real occlusion is only recorded, not manipulated.
- The yaw manipulation also changes the ego heading and the LiDAR and obstacle view.
- Policy F can command 3.54 m/s horizontally; the real pursuer's limit is 2.5 m/s. Speed telemetry is
  descriptive; no speed-limited treatment (D1) is part of this experiment.
- Static and moving results come from different target processes; the interaction is on the
  risk-difference scale.
- Simulation only; no real-flight claim and no claim about real target appearance.
