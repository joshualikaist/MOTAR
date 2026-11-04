# Preregistration: camera-only study, GPU stages C1 (identity) and C2 (zero-shot conditions)

Dated 2026-11-04. Plan: [`camonly_study_plan_v1.md`](camonly_study_plan_v1.md), including its "After review r1"
and "After review r2" sections. Decision D-2026-11-04-3. C0 is PASS (review r2,
`docs/evidence/camonly_2026-11-04/c0/REVIEW_r2.md`).

**Approval.** The user approved the camera-only GPU runs on 2026-11-04 ("카메라 전용 gpu실행 승인할게. ㄱㄱ").
This preregistration covers C1 and C2 only. C3 and C4 (training) need their own preregistration and approval.

## Question

Policy F was trained with RGB-D camera + LiDAR. Without changing F, what happens to capture and crash when the
LiDAR is removed?
- everywhere;
- only inside the camera's 87 deg field of view;
- only outside it;
- from perception only, with the speed governor keeping it.

The answer is a zero-shot diagnostic of F. It is not a camera-only system trained for its sensor.

## Common condition

- **Policy and grid:** Frozen Policy F (SHA `f7022139…`). GR-0's S00 grid: the E2 moving target, A3 placement,
  4 densities (70, 115, 160, 205 bars) x 3 seeds (4501-4503), quota 16 episodes per environment, 128
  environments. Controller gains are nominal (GR's switch OFF).
- **Runner:** `tools/camonly_run.py`, built on `tools/gr_run.py`'s cell environment and checks.
- **Launcher:** `aerial_gym/rl_training/rl_games/eval_navrl_camonly_arms.sh`, pinned by SHA-256 in the runner. It
  is the pinned `eval_navrl_target_behavior_arms.sh` with three marked changes:
  - it requires the condition and absolute interpreter paths;
  - each cell has its own ablation receipt;
  - only the `play_navrl.sh` call runs through `tools/camonly_python`.
- **Ablation:** `aerial_gym/task/navrl_task/navrl_sensor_ablation.py` at its C0-reviewed bytes, unchanged since
  `842fcf5`.
- **Cell checks.** A cell is valid only if it passes all of these:
  - GR-0's checks: receipt OK on this commit and clean tree, Policy F's SHA, no `NAVRL_GAIN_RAND*`, the distance
    backend, the ledger quota, import origins inside the tree, no recorder violations, host unchanged;
  - the ablation receipt exists and names the cell's condition, the committed module SHA-256, paths inside the
    tree and a 72 x 4 LiDAR;
  - the receipt's NAVRL snapshot names the same condition.
- **Host faults:** a rise in the kernel fault count voids the run.
- **Budget:** C1 has a 1-hour cap; C2 has a 3-hour cap (60 cells at about 2.3 min).

## C1: identity, with a positive and a negative control

Both cells are S00, 205 bars, seed 4501, through the camera-only launcher and entry point:

| Cell | Requirement |
| --- | --- |
| `full` | `result.json` (minus the per-run nonce and the checkpoint path) and `episodes.jsonl` equal GR-0's S00 cell exactly |
| `no_lidar` | `episodes.jsonl` differs from GR-0's S00 cell. This shows the switch takes effect in the simulator |

- **Reading:** C1_IDENTITY_PASS if both hold. C1_IDENTITY_FAIL otherwise; then C2 does not run.
- **What a pass admits:** GR-0's S00 cells as condition A of C2, without rerunning them.

## C2: five zero-shot conditions

Order: density, then seed, then condition. Each condition runs on the 12 blocks.

| Key | Condition | LiDAR inside FOV | LiDAR outside FOV | Governor | OOD |
| --- | --- | --- | --- | --- | --- |
| A | `full` (GR-0 S00) | yes | yes | raw LiDAR | no |
| B_in | `no_lidar_in_fov` | no | yes | same sensors | no |
| B_out | `no_lidar_out_fov` | yes | no | same sensors, K2 | yes |
| C | `no_lidar` | no | no | camera only, K2 | yes |
| C_gL | `no_lidar_obs_only` | no | no | raw LiDAR | yes |
| B_in_gL | `no_lidar_in_fov_obs_only` | no | yes | raw LiDAR | no |

- **OOD:** without out-of-FOV LiDAR, F reads the missing bins with the K1 fill, "no return", as free space. Those
  conditions are out of F's training distribution.
- **The optional association-off condition is not run** in this preregistration.

## Analysis (fixed): `tools/camonly_c2_analyse.py`

**Bindings first.** Any failure reads EVAL_BINDING_FAIL:
- C1_IDENTITY_PASS on the C2 commit;
- C2 COMPLETE;
- every C2 cell's ablation receipt and snapshot name its condition and the module SHA;
- the grid is complete;
- each C2 cell differs from the matching `full` cell.

**Quantities.**
- Rates are quota rates per block; the 12 (density, seed) pairs are the blocks.
- Intervals: a percentile bootstrap over blocks (10,000 resamples, seed 20,261,112), with a t-interval as a
  sensitivity check.

**Contrasts,** in pp, each for capture and for crash:

| Contrast | Formula | Reads as |
| --- | --- | --- |
| total | C − A | removing all LiDAR |
| in_fov | B_in − A | the in-FOV LiDAR: the camera's misses and range, the in-FOV association, and the governor's immunity to camera phantoms |
| out_fov | B_out − A | coverage outside the FOV (OOD) |
| interaction | C − B_in − B_out + A | how far the two do not add up (OOD terms) |
| obs_in | B_in_gL − A | the in-FOV LiDAR in perception only. Includes the governor's target carve-out after a camera loss (plan, after r2, L-r2-1) |
| gov_in | B_in − B_in_gL | the in-FOV LiDAR in the governor |
| gov_all | C − C_gL | the governor's LiDAR once perception is camera-only (OOD terms) |

**Labels,** per contrast and metric:
- LOWER if the CI's upper end is below 0;
- HIGHER if the CI's lower end is above 0;
- NO_DETECTED_CHANGE otherwise.

The reading is REPORTED. No adoption decision is taken, and nothing about F or the sensor contract changes.

**Reported, not gated:**
- pooled rates per condition;
- crash causes (bar contacts and floor strikes per 100 episodes).

## What may be said, and what may not

- **May be said:** "With Policy F as trained, removing the LiDAR [where] changes capture by X pp [CI] and crash by
  Y pp [CI] on this grid." Every contrast with an OOD term must carry "zero-shot, out of distribution".
- **May not be said:**
  - anything about a camera-only policy trained for its sensor (that is C3-C4);
  - anything about real cameras or real flight;
  - that one sensor is "unnecessary".
- **Disclosed, from the plan:**
  - L1: the previous-action field and the ego velocity carry the governor's effect in the `*_obs_only`
    conditions;
  - L2: those conditions' target mask uses camera visibility (plus out-of-FOV association in B_in_gL);
  - M-r2-2: the camera's linear angle-to-column map is a residual shared by every condition that uses the camera.

## Controls of the machinery, run before launch (CPU)

The runner and the analysis are smoke-tested end to end on the CPU before this file's commit is used for a GPU
run. The smoke tests stub only the simulator and copy real GR-0 cells.

**Runner known-bad cases:**
- a missing ablation receipt fails;
- a wrong condition fails;
- a `no_lidar` cell identical to GR-0 gives C1_IDENTITY_FAIL;
- C2 without a C1 pass gives PREFLIGHT_FAIL.

**Analysis known-good case:** a synthetic loss in C is labelled LOWER capture.

**Analysis known-bad cases:**
- a receipt naming another condition gives EVAL_BINDING_FAIL;
- a C2 cell identical to `full` gives EVAL_BINDING_FAIL.

The results go to `docs/evidence/camonly_2026-11-04/c1c2/`.

## Review

- An independent design review (Opus 5.5) of this preregistration, the launcher copy, the runner and the analysis
  runs in parallel with C1. C2 starts only after C1_IDENTITY_PASS and a design review without blocking findings.
- An independent review of the C2 results follows. Until it ends, the readings are CONDITIONAL.
