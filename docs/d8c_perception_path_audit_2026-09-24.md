# D8c perception-path audit, read-only part (2026-09-24)

**Status:** read-only audit of recorded D8b data and frozen source. **No GPU run, no training, no new
measurement.** The diagnostic run in the appendix is a **draft**. It is not a preregistration and is
not authorised.

**Question:** where along the path from rendered target to policy outcome does D8b's loss appear, and
does the existing record localise it well enough to decide on retraining?

## 1. What D8b measured

D8b is recorded in `results/dynamic_mesh_policy_sensitivity_d8b_2026-09-13/`. Its preregistration is
`docs/preregistration_dynamic_mesh_policy_sensitivity_d8b_2026-09-13.md`.

- **Policy:** frozen policy **R**, ref5in D1 ep1900 (SHA-256 `197ea269…278e`). This is not policy F.
- **Design:** 70 bars; evaluation seeds 593, 599 and 601; 2,049 episodes per cell; 128 environments.
- **Primary rule (preregistered):** the mean of the seed-paired `capture(mesh_shaded) − capture(analytic_flat)`,
  with a two-sided 95 % Student-t interval (df = 2).
  - `MATERIAL_LOSS` if the upper bound is below the preregistered margin of −3.0 pp.
  - Result: **−48.967 pp [−50.113, −47.821]**, `MATERIAL_LOSS`.

| Arm | Geometry and paint | Capture by seed, % (593 / 599 / 601) | Detector-side visible fraction of steps | Never acquired, % of episodes |
| --- | --- | --- | --- | --- |
| analytic_flat | analytic box, flat red | 71.01 / 70.96 / 71.94 | 0.184 / 0.179 / 0.183 | 18.4 / 19.4 / 17.6 |
| mesh_flat | v3 mesh, flat red | 64.81 / 63.15 / 64.42 | 0.117 / 0.113 / 0.116 | 24.1 / 24.9 / 24.4 |
| mesh_shaded | same mesh, Lambertian shading × material gain | 22.40 / 22.16 / 22.45 | 0.024 / 0.026 / 0.026 | 67.2 / 67.5 / 66.9 |

The sources are `summary.json` → `cells[]` and `cells/*/70bars.json` → `target_motion.first_acquisition`.
`causality_vs_d8b = NOT_TESTED` belongs to the renderer-characterisation track: no experiment has
isolated which renderer property causes the loss.

## 2. How the target reaches the actor

Code citations are to the frozen D8b source snapshot, abbreviated `S/`, which is
`results/dynamic_mesh_policy_sensitivity_d8b_2026-09-13/source_bundle/source_snapshot/aerial_gym/task/navrl_task/`.

- **Same detector in all three arms.** The analytic arm does not bypass it. The RGB-D frame goes
  through `perception.observe` (`S/navrl_task.py` 6260–6283). The ground-truth-mask detector runs only
  in oracle mode, which D8b did not use.
- **Detector:** a fixed colour score, logit = 3R − 2G − 2B − 0.9 (`S/navrl_perception.py` 613–637).
  - A pixel counts as target when its score is ≥ 0.55 and its depth is < 20 m.
  - The target is visible when at least 2 pixels count (1556–1564).
  - Bearing comes from the mask centroid and range from the mean masked depth (1658–1703).
- **Tracking:** a Kalman tracker associates LiDAR only while the track is already active and the camera
  does not see the target (1955). LiDAR therefore sits downstream of camera acquisition.
- **Actor target features:** relative position and velocity from the tracker, covariance, camera and
  LiDAR confidence, and track age (2005–2040). No ground truth enters the actor.
- **mesh_flat → mesh_shaded:** the mask and depth are identical (D8a, hash-verified), and only the RGB
  differs.
  - With the nominal red (0.88, 0.08, 0.045), a pixel passes the threshold only when its shade is
    ≥ about 0.46. This is **derived from code constants, not measured**.
  - Shading therefore removes darker target faces from the detector's mask.
- **Second pathway, untested:** target pixels the detector rejects stay in the obstacle depth map,
  because depth blanking uses the detector mask (`S/navrl_perception.py` 1819). A partly rejected
  target can appear to the policy as an obstacle.

## 3. Metric matrix

The record format is the same for every arm, so one status per row covers all three arms.

| Stage | Metric | Status | Source or reason |
| --- | --- | --- | --- |
| Visibility | Ground-truth target visibility in the closed loop | **NOT_RECORDED** | `target_visible_fraction` is detector-side, not ground truth. The D8a open-loop fixture records renderer-mask frames (53,726 / 53,195 / 53,195) without a policy |
| Visibility | Target pixel footprint | **NOT_RECORDED** in D8b | D8a records ground-truth pixel sums (mesh-to-analytic median area ratio 0.556). Detector-accepted pixels under shading are recorded nowhere |
| Detector | Detection (visible) rate | **RECORDED** | `70bars.json` → `action.context.target_visible.fraction`; per outcome in `target_motion.outcome_telemetry` |
| Detector | Confidence distribution, false positives | **NOT_RECORDED** | With no distractors, false positives are zero by construction (code-derived, not measured) |
| Range / bearing | Error against ground truth | **NOT_RECORDED** | No telemetry compares the estimate with ground truth |
| Association | First acquisition, never-acquired, visible↔hidden transitions | **RECORDED** | `target_motion.first_acquisition.<outcome>` |
| Association | Transitions per episode, P(capture \| acquired) | **DERIVABLE** (CPU, from the counts above) | See section 4 |
| Association | Track switches, lost-track fraction | **NOT_RECORDED** | — |
| Actor input | Observation statistics or distribution shift | **NOT_RECORDED** | The observation dump (`NAVRL_OBS_DUMP`) was off |
| Action | Mean absolute action, mean speed, edge rates | **RECORDED** | `action.mean_abs`, `action.motion.mean_speed_mps` |
| Action | Action standard deviation, measured yaw rate | **NOT_RECORDED** | — |
| Outcome | Capture, crash, timeout, crash causes, outcome steps | **RECORDED** | `outcome`, `crash_causes`, `speed_governor.outcome_steps` |
| Outcome | Minimum relative distance for every episode | **NOT_RECORDED** | `closest_nocrash_mean_m` averages over non-crash episodes only |
| Records | Per-episode rows, per-step traces | **NOT_RECORDED** | Aggregate export only |

## 4. Derived from the recorded counts

These are CPU computations over recorded fields. They are descriptive and were not preregistered.

| Quantity | analytic_flat | mesh_flat | mesh_shaded |
| --- | --- | --- | --- |
| Never acquired, % (per seed) | 18.4 / 19.4 / 17.6 | 24.1 / 24.9 / 24.4 | 67.2 / 67.5 / 66.9 |
| Capture given acquired, % (per seed) | 87.0 / 88.1 / 87.3 | 85.3 / 84.1 / 85.2 | 67.3 / 66.5 / 67.0 |
| Mean first-visible step, captured episodes (seed 593) | 131.2 | 170.4 | 286.2 |

- **Shading-only contrast**, mesh_shaded − mesh_flat: −42.41 / −41.00 / −41.97 pp. The mean is
  −41.79 pp, with a seed-t 95 % interval of [−43.59, −39.99].
- **Geometry-and-area contrast**, mesh_flat − analytic_flat (recorded secondary): −7.17 pp, seed-t 95 %
  interval [−9.31, −5.04].

Capture factors into P(acquired) × P(capture | acquired), and both factors fall under shading. Most
of the drop is in acquisition:
- The target is never acquired in about two thirds of mesh-shaded episodes.
- Among acquired episodes, capture falls by about 20 pp.

All of this is **closed-loop association**. The policy's own trajectory changes what the camera sees,
so visibility is not an independent input here.

## 5. What the record supports

- **The loss appears first at the detector stage, and it is recorded there.** Mask and depth are
  identical between the flat and shaded mesh arms, yet the detector-side visible fraction falls from
  about 0.116 to 0.025, and never-acquired rises from about 24 % to 67 %.
- **Causal localisation is not established.** Several quantities are NOT_RECORDED:
  - range and bearing error;
  - actor-observation statistics;
  - detector-accepted pixels;
  - ground-truth visibility in the closed loop.

  The phantom-obstacle pathway is untested, and visibility is confounded by the policy's trajectory.
- **Whether mesh-shaded rendering is the intended final observation distribution: NOT_STATED.**
  - The D8b preregistration (lines 22–23) frames the mesh arms as out-of-distribution sensitivity for
    a policy trained on the analytic arm.
  - No project document names mesh shading as the deployment observation.

## 6. Retraining decision (decision tree of 2026-09-24)

| Case | Condition | What the record shows | Verdict if it holds |
| --- | --- | --- | --- |
| A | Perception or target-state validity collapses under mesh-shaded input | Detector acceptance collapses (recorded). Target-state error is NOT_RECORDED | `DO_NOT_RETRAIN_POLICY_YET` · `FIX_OR_VALIDATE_PERCEPTION_CONTRACT_FIRST` |
| B | Perception and actor input stay valid, yet the policy still collapses under the intended final distribution | Contradicted at the detector stage by the recorded collapse | `NEW_OBSERVATION_CONTRACT_RETRAINING_STUDY_JUSTIFIED` (new preregistration and lineage) |
| C | Mesh-shaded is not the intended final distribution | NOT_STATED; this is the user's decision | `NO_RETRAINING_REQUIRED_FOR_CURRENT_CLAIM` · `D8B_REMAINS_SENSITIVITY_EVIDENCE` |

**Retraining: `NOT_YET_JUSTIFIED`.** The record points to Case A at the detector stage, but it does
not establish target-state validity or the causal pathway. If the user states that mesh shading is not
the intended deployment observation, Case C applies without any further run. No materiality threshold
is introduced here. The only threshold in force is D8b's preregistered −3.0 pp margin.

## Appendix — draft diagnostic run D8c-DX

This is not a preregistration. It must be copied into a dated preregistration, with every threshold
fixed, and approved by the user before any run.

- **Stage 1: open loop, no policy.** Replay a fixed set of recorded poses (the D8a fixture) under
  analytic_flat, mesh_flat and mesh_shaded. Per frame, record:
  - ground-truth mask pixels and detector-accepted pixels;
  - detector confidence;
  - range and bearing error against ground truth;
  - tracker state.

  This localises the perception stage without policy confounding.
- **Stage 2: closed loop, policy R.** Hold the rendered image fixed and swap the two pathways,
  2 × 2: target features from the flat or the shaded detector, crossed with obstacle depth from the
  flat or the shaded render.
  - Log per episode (outcome, minimum distance for all episodes, first acquisition) and per step
    (actor-observation summaries, actions).
  - Use six fresh evaluation seeds.
- **Instrumentation contract:**
  - Ground-truth error channels are written to a side file and never enter the actor observation.
  - An on/off invariance test on actor-observation tensors must pass before any measured cell.
- **Decision rules to fix before data** (placeholders, not chosen):
  - the share of the mesh_shaded − mesh_flat loss that a swap must recover to name a pathway;
  - the interval that recovery must clear.
