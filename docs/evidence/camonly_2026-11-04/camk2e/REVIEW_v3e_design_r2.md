# CAM-K2-E3 v1 — independent pre-launch design review r2 (repairs of r1)

- Reviewer: Claude Opus 5.5 (fresh context, read-only), 2026-11-10.
- Object: `MOTAR-camonly` (branch `camonly/dev`, HEAD `abd9007`), staged + unstaged state of the prereg
  `docs/plans/prereg_camonly_k2_eval_v3_2026-11-10.md`, `tools/camk2v3e_run.py`, `tools/camk2v3e_analyse.py`,
  `tests/test_camk2v3e.py`, `docs/EXPERIMENTS.md`. Round 1: `../review_r1/REVIEW_design_r1.md` (AMEND_FIRST).
- Nothing in the worktree was edited. No GPU/simulator, no runner stage. CPU only, cores 8-9, nice 10:
  `test_camk2v3e.py` 16 tests OK; `test_experiment_index.py` 9 tests OK; read-only recomputations on
  `~/motar_tmp/camk2e_20261108/raw` (scripts in the session scratchpad, not in the repo).

## Repairs verified

1. **M-1 claim-scale controls — correct.** `claim_scale_controls` (analyse.py:150-165) computes K1s−K2s (must be
   WORSE) and K2s−K1s (must be SMALL_/MATERIAL_REDUCTION) for s = 1, 2 on CAM-K2-E's treated `eval_01/K1s*` cells
   (`TREATED_ARMS` includes K1s1/K1s2) against the V2s reference cells, with gate_d and its own `Random(BOOT_SEED)`;
   `main_eval` adds them to `ctrl` and `ctrl_ok` requires every `ok` (analyse.py:209-213). The main contrasts use a
   fresh generator, so D/R/T numbers are unaffected by the added controls. Recomputed with the staged code:

   | Control | Label | Mean pp | Bootstrap | t |
   | --- | --- | --- | --- | --- |
   | K1s1 − K2s1 | WORSE (ok) | +3.80 | [2.93, 4.63] | [2.57, 5.04] |
   | K2s1 − K1s1 | SMALL_REDUCTION (ok) | −3.80 | [−4.64, −2.92] | [−5.04, −2.57] |
   | K1s2 − K2s2 | WORSE (ok) | +4.74 | [4.24, 5.26] | [4.01, 5.48] |
   | K2s2 − K1s2 | SMALL_REDUCTION (ok) | −4.74 | [−5.26, −4.24] | [−5.48, −4.01] |

   All four equal the prereg table (rows 135-138). Independent from-scratch means (plain Python over `in_quota`
   rows of `episodes.jsonl`, 2048 quota episodes per cell, 12 blocks): s1 +3.8005 pp, s2 +4.7445 pp — match.
   Non-vacuity: substituting a null (V2s2 cells as "K1s1", V2s1 as "K1s2") makes all four read NEGLIGIBLE, ok False.
   Null control still NEGLIGIBLE.
2. **m-1 R/T wording — correct and consistent** across prereg (§Questions 2, §May be said, new *May not be said*
   line) and the analysis docstring.
3. **m-2 snapshot_ok — correct.** Factored out (analyse.py:137-140) and used by `load_eval`; the test accepts real
   CAM-K2-E pairs (identity K2s1_smoke vs eval K2s1; K2s2 vs K2s1), rejects changed `NAVRL_DEPTH_SEED`,
   `NAVRL_VEH_RAND_SEED`, `NAVRL_SENSOR_ABLATION`, and accepts the three rule keys. Positive and negative paired.
4. **m-3 approval scope — done** in the prereg Background (see LOW-2 on wording). `SCOPE` itself names "CAM-K2 v2
   seeds 1 and 2 with the rule added at evaluation", and `approval_problems(..., scope=SCOPE)` binds it.
5. **LOW V2s2_identity — correct.**
   - `IDENTITY_PLAN` 5 cells; `arm_ckpt` maps it to K2s2; `rule()` is False; `run_cell` passes
     `R2.treatment_env(4501)`, i.e. module seeds 45011/45012, the same as CAM-K2-E's K2s2 205/4501 cell (module seeds
     depend only on the grid seed). `receipt_problems(..., "K2s1", ...)` depends on the arm only via `treated()`.
   - Reference: `IDENTITY_REFERENCES.get(arm, IDENTITY_REFERENCE)` → K2s2 cell for V2s2_identity, K2s1 for all
     others (the V2Rs1_negative "must differ" check stays against K2s1, as designed). Preflight checks both files.
   - `ok` (run.py:364-368): all 5 arms compared (so all OK) AND both identity arms `result_equal` and
     `episodes_equal` AND the three rule conditions. Short-circuiting is safe. `episodes.jsonl` rows carry
     `arm: "e2_obstacle_aware"`, not the runner arm label, and `identity_compare` drops the checkpoint path, so the
     cross-arm comparison is not defeated by naming.
   - **Rate gate:** `len(walls) != len(IDENTITY_PLAN)` now expects 5; the ratio is mean(5 cells)/mean(CAM-K2-E's 4
     identity cells) — a ratio of means, so differing counts are fine. All 9 cells are 205/4501. Arm mix differs
     (CAM-K2-E's included an untreated K1 cell at 193.7 s; its 205-bar eval cells average 172 s), so the ratio is
     a host-speed proxy with ≈ ±10 % arm-mix bias. Expected ratio ≈ 0.96 → projection ≈ 7,970 s (2.2 h) against a
     12,600 s cap; the gate refuses only above ratio ≈ 1.52, and `GpuBudget(3.5 h)` and the wrapper's
     `timeout 13500` still bound the stage. Still meaningful for its purpose (LOW-3).
6. **0.26 m "inferred", reference-cell text (2 of 24), first-50-frames note, hard-link note — present and correct.**
7. **Analysis bindings.** `identity_V2Rs1_negative_equals_eval_V2Rs1` and `identity_V3s1_smoke_equals_eval_V3s1`
   compare the 205/4501 `episodes.jsonl` in identity and eval dirs (analyse.py:129-133) and enter the
   all-bindings gate. The identity report's own V2s2 comparison is bound through IDENTITY_PASS + same commit +
   script bytes equal to the eval commit.
8. **Counts/SCOPE.** SCOPE says "5-cell identity control; 53 cells" = 5 + 48: correct. No "52" / "4-cell" left in
   runner, analysis, tests, prereg or EXPERIMENTS.md. The prereg does not restate SCOPE, so nothing to drift; the
   approval file does not exist yet and must copy `camk2v3e_run.SCOPE` verbatim. Wrapper
   `~/motar_tmp/camk2v3e_20261110/run_camk2v3e.sh` has no cell counts; its identity timeout 3600 s covers
   5 × ~175 s.

## Findings

### MINOR

**m2-1. Claim-scale control inputs are not manifest-bound.** r1's M-1 fix asked to extend the manifest check to the
K1s cells; `ctrl_ok` = controls ok AND `A2.camk1e_manifest_ok(k1e, ...)` only, and `camk2e_manifest_ok` (analyse.py:79-82)
still lists only K2s1/K2s2. The 24 `eval_01/K1s1|K1s2` cells feeding the four new controls are therefore read
unbound. They are in CAM-K2-E's committed manifest (108 entries each), and I checked that `episodes.jsonl`,
`result.json`, `receipt.json` of all 24 match it today (True), so no current defect — but the frozen analysis would
not detect a later change. Fix (before commit; the script is byte-bound to the eval commit): make
`camk2e_manifest_ok` iterate `("K2s1", "K2s2", "K1s1", "K1s2")`, or add `camk2e_k1_manifest_ok` and AND it into
`ctrl_ok`; extend `test_claim_scale_controls` with `assertTrue(...)` on the real folder and `assertFalse` on another
root (as the CAM-K1-E test does). Name it in the prereg's Bindings or Controls line.

### LOW

- **L2-1. Stale sentence.** Prereg §Design, "Reuse of CAM-K2-E's v2 cells": "The identity stage shows that this
  commit reproduces one of them exactly." → "two of them (one per seed)".
- **L2-2. Approval wording.** "The approval covers the V3 arms. The 24 V2R cells … are this unit's own design choice"
  reads as if the V2R cells were outside the approval while the next sentence's approval file includes them. Prefer:
  "The user's words name the v3 evaluation; the V2R arms (24 cells, ≈1.1 h) are an extension this unit adds. The
  approval file's SCOPE names them; the user is told so when asked to approve and in the report."
- **L2-3. Rate ratio arm mix.** Document in the runner docstring/prereg that the ratio compares different arm mixes
  at 205/4501 (≈ ±10 %), covered by the 1.10 margin and the budget cap. No code change needed.
- **L2-4. No negative unit test for the claim-scale controls.** My check above (null input → all ok False) could be
  added as a test so the controls' non-vacuity is pinned (AGENTS.md: pair negative and positive).
- **L2-5. The identity `ok` expression is untested** (inline in `main`). Optional: factor it into
  `identity_ok(comp)` with tests for a missing arm, a V2s2 mismatch and a passing dict.
- **L2-6. Cosmetic:** run.py lines 17 and 82 exceed 130 characters; runner docstring line 27 says
  "V2Rs1_negative != V2s1_identity", while the code compares V2Rs1_negative with CAM-K2-E's K2s1 cell (equivalent
  only given V2s1_identity passes; fine, but say "differs from CAM-K2-E's K2s1 cell").
- r1 L-4, L-5, L-6 remain as accepted (not repaired; acceptable as stated in r1).

## Predictable refusals / vacuous passes

None found. Preflight reference-file loop, 5-cell rate-gate length check, identity references and analysis bindings
are reachable with real data on this host; the new controls pass on real data and fail on a null.

## Recommendation

Fix m2-1 (and L2-1) before the commit; no further review round is needed if the change is limited to the manifest
rels/control gating, its test and the prereg sentence. The other LOW items are optional.

VERDICT: PASS_WITH_LIMITS
