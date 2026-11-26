# CAM-REF stage 2 — which P_T → P_S factor carries Policy F's bar-contact loss (preregistration)

- **Status:** preregistration, committed before any stage-2 GPU cell (2026-11-26). Approved by the user ("2단계 평가
  진행해", 2026-11-26) after the stage-1 report. Stage 1:
  [`prereg_camonly_ref_matched_protocol_2026-11-26.md`](prereg_camonly_ref_matched_protocol_2026-11-26.md) (unchanged;
  it named stage 2 as needing its own approval), results commit 1614b15,
  `docs/evidence/camonly_2026-11-04/camref/analysis_gpu1.json`.
- **Branch:** camonly/s2b (no new branch, user 2026-11-26). Unit folder `~/motar_tmp/camref2_20261126/`.

## Question
Stage 1 measured Policy F (ep25000 riskcap, camera 87° + 360° LiDAR) at 205 bars with bar-contact crash 9.0 % at P_T
(F4) and 22.6 % at P_S (F8), zero-shot. F8 adds five factors to F4 at once. Which of them carry the +13.6 pp?
The answer feeds two open decisions: the yaw-rate cap of the next training (Jetson side proposes 90°/s) and the
priority of the runtime latency work (measured 0.34 s capture → command).

This is a measurement of Policy F, which never trained under any of these factors. It is not a measurement of a
camera-only policy, and it makes no flight claim.

## Cells (205 bars, seeds 4501-4503, first 8 episodes per env at 128 envs = 1,024 episodes per cell)
Each new rung is stage 1's F4 (P_T task rules, D8 target, F's own stack) plus exactly one factor, with the same keys and
module seeds F8 used for that factor:

| Rung | Factor added to F4 | Keys (as in F8) |
| --- | --- | --- |
| F4D | action delay U{1,2} steps | `NAVRL_CAMREF_DELAY*`, delay seed 10 × seed + 4 |
| F4R | vehicle (T/W 3.5 raise, ±15 %, imbalance ±5 %, τ 0.02-0.04 s) and gain (±20 %) randomisation | `NAVRL_VEH_RAND*`, `NAVRL_GAIN_RAND*`, S2b-FT's module seeds |
| F4Y | yaw clip 1.047 rad/s (60°/s) on the governed command, action scale 3.0 kept | `NAVRL_YAWCLIP*` |
| F4V | speed-governor FREE 2.5 m/s (F's native 3.536) | `NAVRL_SPEED_GOVERNOR_FREE_MPS` |
| F4A | altitude 1.5 m / camera mount 0.08 m | `NAVRL_CAMREF_GEOMETRY*` |

Reference arms: stage 1's F4 and F8 cells, seeds 4501-4503 (`~/motar_tmp/camref_20261126/gpu1/eval/{F4,F8}`), every file
the identity stage and the analysis read pinned by SHA-256 in `tools/camref_stage2_run.py` (`REF_SHA`).

- **Smoke (never counted):** F4D, F4R, F4Y, F4V, F4A at 205 / 4501, quota 1. SMOKE_OK needs every cell OK with ≥ 1
  finished episode, CAM-REF rules telemetry with ≥ 1 capture or target contact, every rendered step-0 bearing inside
  ±28.35° (+ tolerance), and per factor (each a positive control with its negative in the other four): delay draws of
  both 1 and 2 only in F4D; a `[veh-rand]` init line only in F4R; a `[yawclip]` line with commands > 0 only in F4Y.
  F4V is checked by the result condition (FREE 2.5 only in F4V). Geometry: the rule module's own record
  (`camref_receipt.json` → `camref_rules.config_after`) must read altitude 1.5 m / camera x 0.08 m in F4A and
  1.0 m / 0.10 m in the other four; this check runs on every cell with the CAM-REF rules (review r1 P2-1; stage-1's 27
  cells all pass it).
- **Identity:** F4ID and F8ID rerun stage 1's F4 and F8 at 205 / 4501 on this commit. Per-episode outcomes must equal
  the stage-1 cells' (first 8 per env); any difference is IDENTITY_FAIL and the chain stops. This shows the factor-table
  refactor of `tools/camref_eval_run.py` builds stage 1's F4 and F8 unchanged.
- **Eval:** F4D, F4R, F4Y, F4V, F4A × 4501-4503, block-major by seed (15 cells).
- **Wall cap 2.0 h** shared by smoke (sub-cap 0.5 h), identity (0.5 h) and eval (2.0 h), with stage 1's projection /
  BUDGET_SKIPPED rule. Expected about 1.3 h (stage 1: F cells 3-6 min each, smoke 1.6 min).
- **Guards:** stage 1's, unchanged: approval file `~/motar_tmp/camref2_20261126/execution_approval.json` naming this
  scope (`tools/camref_stage2_run.py scope`) and HEAD; clean tree; branch camonly/s2b; launcher pin; idle host; host
  fault count / boot id / online CPUs unchanged per cell and stage; the runner's environment / receipt / telemetry
  checks per cell.

## Readings (tools/camref_stage2_analyse.py)
Counts, rates, the episode bootstrap (10,000 resamples, numpy default_rng(4611), cells in the order F4, F4D, F4R, F4Y,
F4V, F4A, F8 × seeds) and paired seed-block differences are stage 1's (`tools/camref_eval_analyse.py`).

- **Q1 (labelled):** X − F4 for each factor rung, mean of the paired seed-block differences (pp). Labels on bar-contact
  crash, crash (all) and capture: MATERIAL (|difference| ≥ 5 pp and the bootstrap 95 % interval excludes 0), SMALL
  (interval excludes 0, |difference| < 5 pp), NO_DETECTED_CHANGE (otherwise). Target-contact and out-of-bounds crash:
  reported.
- **Q2 (reported):** F8 − F4 with the same labels, and the interaction share (F8 − F4) − Σ(X − F4) per metric with its
  bootstrap interval.
- **Use of the result (fixed):**
  - Factors with Q1 bar-contact MATERIAL and positive are named carriers of F's bar-contact loss from P_T to P_S,
    largest first.
  - No carrier while Q2 bar-contact F8 − F4 is MATERIAL → no single factor carries it (interaction, or several small
    ones).
  - If F4Y is a carrier: the 60°/s cap is a measured contributor for a policy with 360° ranging. This supports testing a
    higher cap in the next training; it does not show a camera-only policy gains from it.
  - If F4D is a carrier: the latency reduction (sparse reprojection, 1-thread pinning) is prioritised before training.
- **Analysis controls** (run before the eval is read; a miss reads CONTROL_FAIL): synthetic cells with every factor
  equal to F4 → all NO_DETECTED_CHANGE; F4Y bar crash +8 pp in every block → F4Y MATERIAL, the other four
  NO_DETECTED_CHANGE, carriers = [F4Y].
- Only complete seed blocks are read; none complete → NOT_RUN.

## Code (committed with this file)
- `tools/camref_eval_run.py`: the F4 / F8 deltas, installs, telemetry expectations and smoke controls now come from one
  factor table (`F_FACTORS`, `f_factor_delta`); the five rungs and F4ID / F8ID added to `ARMS`; `preflight` receives
  `UNIT_DIR` from `main`. `tests/test_camref_stage2.py` checks that every stage-1 rung delta, install list, protocol
  point, mismatch allowance and constant equals the runner committed at 358c363.
- `tools/camref_stage2_run.py` (stage-2 constants, installed into the stage-1 runner, which then runs unchanged),
  `tools/camref_stage2_analyse.py`, `tests/test_camref_stage2.py`.

## Limits
- Single policy (F), single density, three seeds; one-factor-at-a-time design. Interactions are estimated only as the
  remainder Q2.
- F4V changes the governor's free speed, which is both a speed cap and part of F's trained stack; F4Y clips F's yaw
  command after its 3.0 rad/s scale, which F never saw. Both are zero-shot perturbations of F, as F8 was.
- The 5 pp MATERIAL threshold is stage 1's; a factor of 3-4 pp reads SMALL at best.
- Gain randomisation (half of F4R) writes no telemetry: it is read at import in `lee_controller_config_navrl.py`, and
  only the environment keys and snapshots show it was set (as in stage 1's F8). F4R's vehicle half has its
  `[veh-rand]` init record.
- Identity is checked at seed 4501 only (F4ID, F8ID); seeds 4502-4503 of F4 / F8 are reused from stage 1 by SHA pin.
- Independent review r1 (Claude Opus 5.5, fresh context, read-only): GO_WITH_FIXES, no P1; P2-1 (geometry record) and
  P3-1..5 fixed before this commit; report `~/motar_tmp/camref2_20261126/review_r1/REVIEW.md`.
