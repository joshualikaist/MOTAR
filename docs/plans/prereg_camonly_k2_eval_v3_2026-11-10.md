# CAM-K2-E3 v1: matched evaluation of CAM-K2 v3 against CAM-K2 v2 (preregistration, 2026-11-10)

**State:** **NOT_RUN** at its commit. The independent review is recorded under *Review before the commit* at the end.
This file is never edited after the commit; amendments go in a new dated file.

- Runner: `tools/camk2v3e_run.py`.
- Analysis: `tools/camk2v3e_analyse.py`.
- Tests: `tests/test_camk2v3e.py`.
- Unit folder: `~/motar_tmp/camk2v3e_20261110/`.

## Why

- **CAM-K2 v2** (`aa0e691`, trained `04f341c`) is CAM-K1 continued for 1,000 epochs under a simulated real vehicle:
  - the per-motor limit raised to T/W 4.0;
  - thrust ±15 % and motor imbalance ±5 %;
  - τ 0.02–0.04 s;
  - the OAK-D Pro depth model.
- **CAM-K2-E** (`d2a018b`, results review PASS) found that v2 crashes 3.80 / 4.74 pp less than CAM-K1 under those
  treatments. It did not test whether the depth model's near-bar gap changes the crash rate.
- **The gap:** with MinZ 0.4 m and invalid read as no return, a face-on bar vanishes from the obstacle tokens in about
  the last 0.26 m of approach (inferred from the synthetic probe's geometry). A CPU probe found this, and its review
  read PASS_WITH_LIMITS.
- **CAM-K2 v3** (`d85589d`, trained `326e307`) is v2 retrained with the depth-validity rule `blob_near`: a dense
  invalid blob becomes a surface at MinZ.
  - Its training diagnostic read DIAG_OK: hit-bar tokens of 0.70 / 0.69, against v2's 0.17 / 0.19.
  - Whether v3 crashes less than v2 in evaluation is untested. This unit tests it.
- **The user** (2026-11-10, about 02:2x KST): "중복 잘 합쳐주고, 용량이 나오면 그때 v3 평가 시작해줘" (merge the duplicates
  well, and start the v3 evaluation once there is space). The disk cleanup `abd9007` left 6.15 GB free. This approves
  this unit's GPU run once the preregistration is committed, reviewed and preflighted.
  - The approval is for this unit as preregistered, V2R arms included. The 24 V2R cells (about 1.1 h) are this unit's
    design choice, made to separate the rule at evaluation from training with it. The approval file names them, and
    the report to the user says so (review r1 m-3).

## Questions

1. **Does the v3 package crash less than v2?** Both run under v2's treatments; v3 runs with the rule, as trained, and
   v2 without it, as trained. This is gate D.
2. **How does D split?** Reported with labels, not gated:
   - R = V2R − V2: the rule added at evaluation to v2's unchanged policy. This includes v2's mismatch between training
     and evaluation inputs.
   - T = V3 − V2R: v3 against v2, both evaluated with the rule. This is training with the rule, minus that mismatch,
     plus the variance between two separate training runs (review r1 m-1).
   - D = R + T exactly on the same blocks. No causal split beyond this arithmetic is claimed.

## Design

- **Condition:** CAM-K2-E's, unchanged.
  - GR-0's S00 grid: E2 target, A3 placement, 70 / 115 / 160 / 205 bars × seeds 4501–4503, quota 16, 128 envs.
  - `no_lidar` at 66 × 54° through `tools/realveh_python`.
  - CAM-K2 v2's treatments are ON in every cell, with module seeds 10 × seed + 1 / + 2, matched across arms.
  - CAM-K2-E's launcher `eval_navrl_camk2e_arms.sh`, unchanged and SHA-pinned.
- **The rule:** set by `NAVRL_DEPTH_VALIDITY=blob_near`, `NAVRL_DEPTH_VALIDITY_WINDOW=5` and
  `NAVRL_DEPTH_VALIDITY_DENSITY=0.5`, the values v3 trained with.

| Arm | Checkpoint | Rule | Cells |
| --- | --- | --- | --- |
| V3s1, V3s2 | CAM-K2 v3 seed s (`888e8056…`, `203deea4…`) | on | new, 12 each |
| V2Rs1, V2Rs2 | CAM-K2 v2 seed s (`769a26f4…`, `e76eb048…`) | on | new, 12 each |
| V2s1, V2s2 | CAM-K2 v2 seed s | off | **CAM-K2-E's eval K2s1 / K2s2 cells, reused** |

- **Reuse of CAM-K2-E's v2 cells.** They are bound by CAM-K2-E's committed `SHA256_MANIFEST.json`. The identity stage
  shows that this commit reproduces two of them (one per seed) exactly. Since CAM-K2-E, `tools/realveh_entry.py` gained the opt-in
  rule; with the rule unset it reproduces v2, as v3's reviews found.
- **Identity stage** (5 cells, 205 bars, seed 4501):
  - `V2s1_identity` and `V2s2_identity` (v2 s1 and s2, rule off) must equal CAM-K2-E's eval K2s1 and K2s2 cells
    exactly (`result_equal` and `episodes_equal`), one per reference seed (review r1 LOW).
  - `V2Rs1_negative` (v2 s1, rule on) must differ from it. This shows the rule acts in an evaluation cell.
  - `V2Rs1_repeat` must equal `V2Rs1_negative` (determinism with the rule).
  - `V3s1_smoke` (v3 s1, rule on) must be valid and differ from `V2Rs1_negative`.
  - Anything else reads IDENTITY_FAIL, and the eval does not start.
- **Eval stage:** 48 cells, ordered by density, then seed, then arm (V3s1, V3s2, V2Rs1, V2Rs2).
  - **Rate gate:** 48 × CAM-K2-E's mean eval cell (157.2 s) × (this identity mean / CAM-K2-E's identity mean) × 1.10
    must not exceed 3.5 h. Otherwise it refuses. The projection is recorded in the run report.
  - The cap is 3.5 h per stage.
- **Cell checks:** every cell must pass CAM-K2-E's checks (`camk2e_run.receipt_problems` for a treated arm), plus the
  rule's own checks.
  - **Receipt:** `settings.depth_validity` equals the table; the install record is present; `depth_validity_sha256`
    equals the module committed at HEAD.
  - **Snapshot:** has the three `NAVRL_DEPTH_VALIDITY*` keys.
  - **Telemetry:** one install line with the settings; a first and a summary line (50 calls), each with
    0 ≤ near ≤ invalid ≤ pixels and `sentinel_mismatch` 0.
  - **The near count is recorded but not required to be > 0.** Unlike training at 205 bars, an evaluation cell's first
    50 frames need not come near a bar. `V2Rs1_negative ≠ V2s1_identity` shows instead that the rule acts.
  - **With the rule off:** no rule keys and no rule telemetry.
  - A failing cell stops the stage.
- **Preflight:** CAM-K2-E's preflight, plus:
  - approval `~/motar_tmp/camk2v3e_20261110/execution_approval.json`, naming `SCOPE` and HEAD;
  - free disk ≥ 1 GB;
  - the v2 and v3 checkpoints at their pins, with TRAINED train reports and sidecar manifests;
  - a COMPLETE CAM-K2-E eval folder.
  A rise in the host fault count voids the run.

## Analysis (fixed before the run)

Blocks are (density, seed) pairs, and rates are per-block quota rates. The intervals are CAM-K2-E's:
- a stratified percentile bootstrap, 10,000 resamples, seed 20,261,108, with seeds resampled within density;
- a stratified t-interval with df 8 over the 12 blocks.

Contrasts are in pp. A label needs both intervals to agree.

**Gate D, crash rate, per seed pair s: D_s = V3s − V2s.** This is CAM-K2-E's gate-A rule with renamed labels:

| Label | Rule |
| --- | --- |
| UNRESOLVED | the two intervals disagree on whether 0 is excluded, or on the side |
| MATERIAL_REDUCTION | both upper ends < −5 pp |
| SMALL_REDUCTION | both upper ends < 0 and the mean ≤ −2 pp |
| WORSE | both lower ends > 0 and the mean ≥ +2 pp |
| NEGLIGIBLE | both intervals inside (−2, +2) pp |
| NO_DETECTED_CHANGE | otherwise |

A reduction with timeouts up by 2 pp or more reads **CRASH_TO_TIMEOUT**. The lineage reading is the label both seeds
share, else **SEEDS_DISAGREE**.

**Reported with the same labels, not gated:**
- R_s = V2Rs − V2s;
- T_s = V3s − V2Rs;
- the seed spreads V3s1 − V3s2 and V2Rs1 − V2Rs2;
- capture and timeout contrasts, labelled with C2's scheme (capture band 3 pp);
- pooled and per-density rates;
- crash causes;
- speed-governor telemetry;
- the rule's summary counts per cell.

**Controls** (computed on existing data before the commit; any miss reads CONTROL_FAIL, and no gate label is given):

| Control | Required | Computed |
| --- | --- | --- |
| the gate rule on CAM-K1-E's F66 − K1s1 (known bad) | WORSE | WORSE, +37.67 pp |
| on CAM-K1-E's K1s1 − K1s2 (null) | not MATERIAL | NEGLIGIBLE, −0.92 pp |
| on CAM-K1-E's K1s1 − F66 (MATERIAL reachable) | MATERIAL | MATERIAL, −37.67 pp |
| gate N on CAM-K1-E's K1s2 − K1s1, 6 blocks | NO_MATERIAL_LOSS | NO_MATERIAL_LOSS |
| gate N on CAM-K1-E's F66 − K1s1, 6 blocks | MATERIAL_LOSS | MATERIAL_LOSS |
| gate D on CAM-K2-E's K2s1 − K2s2 (null on this unit's reference cells) | not MATERIAL_REDUCTION, not WORSE | NEGLIGIBLE, 0.00 pp, bootstrap [−0.53, +0.54], t [−0.78, +0.78] |
| gate D on CAM-K2-E's treated K1s1 − K2s1 (known bad at the claim's scale; review r1 M-1) | WORSE | WORSE, +3.80 pp, bootstrap [+2.93, +4.63], t [+2.57, +5.04] |
| gate D on CAM-K2-E's K2s1 − K1s1 (known good at the claim's scale) | SMALL_ or MATERIAL_REDUCTION | SMALL_REDUCTION, −3.80 pp, bootstrap [−4.64, −2.92], t [−5.04, −2.57] |
| gate D on CAM-K2-E's treated K1s2 − K2s2 | WORSE | WORSE, +4.74 pp, bootstrap [+4.24, +5.26], t [+4.01, +5.48] |
| gate D on CAM-K2-E's K2s2 − K1s2 | SMALL_ or MATERIAL_REDUCTION | SMALL_REDUCTION, −4.74 pp, bootstrap [−5.26, −4.24], t [−5.48, −4.01] |

The last row's half-widths (0.54 / 0.78 pp) are this design's resolution on one seed pair's difference.

**Bindings** (EVAL_BINDING_FAIL otherwise), checked before any number:
- identity IDENTITY_PASS on the eval commit, and it is the identity report the eval named;
- eval COMPLETE, 48 cells in order;
- the same runner environment and CAM-K2-E folder;
- every cell passes the runner's checks again, with its checkpoint SHA;
- each cell's result condition equals CAM-K2-E's K2s cell of the same block;
- its NAVRL snapshot differs from that cell's only in the run ids, the checkpoint, the realveh receipt path and the
  rule's keys;
- complete grids;
- in every block, V3s ≠ V2s, V2Rs ≠ V2s, V3s ≠ V2Rs and V3s1 ≠ V3s2;
- the identity cells `V2Rs1_negative` and `V3s1_smoke` equal the eval's V2Rs1 and V3s1 cells at 205 / 4501;
- CAM-K2-E's reference cells at its committed manifest; the controls' CAM-K2-E K1s1 / K1s2 cells too, as a condition
  of the controls (review r2 m2-1);
- the checkpoints at their pins;
- the analysis script's bytes equal the eval commit's.

## What may be said, and what may not

- **May be said** (if bound and the controls pass): "On this grid, in simulation, under CAM-K2 v2's treatments,
  CAM-K2 v3 with its depth-validity rule changes the crash rate by X pp [CI; t CI] against CAM-K2 v2 for seed s
  (gate D: label)." R and T may be reported with their labels as "the rule added at evaluation to v2" and "v3
  against v2, both with the rule", and D = R + T as arithmetic.
- **May not be said:**
  - anything about the real OAK-D Pro, real images or real flight. The rule's sim-to-real validity is NOT_TESTED: real
    sky, textureless, shadow and specular invalid bands could become phantom surfaces;
  - that the near-bar gap "caused" v2's crashes, or that the rule "fixes" them as a mechanism. Gate D compares bundled
    packages, and R and T are contrasts, not a causal decomposition;
  - that T measures "the effect of training with the rule" alone (it also carries R's mismatch with the opposite sign
    and the variance between training runs);
  - an effect from one seed alone, or a pooled interval for "the v3 recipe";
  - any comparison with training-rollout rates, with F with camera + LiDAR, or with CAM-K1 (no CAM-K1 arm here);
  - generalisation beyond these four densities.

## Disclosed limits

- The v2 reference cells were run on 2026-11-08/09 at `1237f9f`. The identity stage checks 2 of their 24 cells for
  exact reproduction (one per seed); the others rely on the same code path, the committed manifest and determinism.
  Some of their raw files were hard-linked on 2026-11-10 (`abd9007`); their bytes and SHA-256 are unchanged.
- The rule's per-cell counts cover only the first 50 frames of each cell (descriptive).
- V2R shows the rule's effect on a policy that never trained with it. A mismatch between training and evaluation
  inputs may make R negative or positive for reasons unrelated to real sensing.
- One seed pair per contrast, one checkpoint per seed. The intervals are over evaluation blocks.
- The rule reports a near bar at MinZ (0.4 m), not at its true distance.
- The rule depends on the simulator's invalid mask, which arises only from MinZ and random holes.

## Review before the commit

- **Design review r1** (Claude Opus 5.5, fresh context): **AMEND_FIRST**,
  `docs/evidence/camonly_2026-11-04/camk2e/REVIEW_v3e_design_r1.md`. The rule's env forwarding, the reuse of
  `receipt_problems`, the snapshot binding and the decision not to require near > 0 were found sound. Repaired:
  - M-1: claim-scale controls (four rows above);
  - m-1: R/T wording;
  - m-2: `snapshot_ok` tested with real cells, both accepting and rejecting;
  - m-3: the approval names the V2R cells;
  - LOW: the `V2s2_identity` cell, the 0.26 m figure marked inferred, the reference-coverage text.
- **Design review r2** (focused): **PASS_WITH_LIMITS**, `.../REVIEW_v3e_design_r2.md`. The repairs were verified and the
  four control numbers recomputed. Repaired before the commit:
  - m2-1: the controls' K1 cells bound to the manifest;
  - LOW: the approval wording, a note on the rate ratio, and a null test of the controls.
- **CPU tests:** `tests/test_camk2v3e.py` OK. PUBLIC_CPU before the r2 repairs: 3,824 run, 0 failures, 0 errors. The
  final suite is in `~/motar_tmp/camk2v3e_20261110/public_cpu_precommit2.log`.
