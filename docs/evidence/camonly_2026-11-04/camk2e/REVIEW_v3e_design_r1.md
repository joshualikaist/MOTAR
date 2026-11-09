# CAM-K2-E3 v1 — independent pre-launch design review r1

- Reviewer: Claude Opus 5.5 (fresh context, read-only), 2026-11-10.
- Object: staged unit in `MOTAR-camonly` (branch `camonly/dev`, HEAD `abd9007`): prereg
  `docs/plans/prereg_camonly_k2_eval_v3_2026-11-10.md`, `tools/camk2v3e_run.py`, `tools/camk2v3e_analyse.py`,
  `tests/test_camk2v3e.py`, `docs/EXPERIMENTS.md`. Line numbers below are of the staged files; `camk2e_run.py` etc. at HEAD.
- Nothing in the worktree was edited, staged or run on GPU. CPU work: `tests/test_camk2v3e.py` (14 tests OK, cores 8-9) and
  read-only recomputations on CAM-K2-E / CAM-K1-E raw with the staged functions.

## What was verified (evidence)

1. **Code delta since CAM-K2-E (`1237f9f`).** `git diff --stat 1237f9f HEAD` outside docs/tests touches only
   `tools/realveh_entry.py` (+27), the new `navrl_depth_validity.py`, `train_navrl_camk2v3.sh`, `tools/camk2v3_train.py`.
   In `realveh_entry.py` the rule is an opt-in branch: `settings()` adds `depth_validity` only when
   `NAVRL_DEPTH_VALIDITY` is set (realveh_entry.py:147-160) and `main()` installs only then (:210-215); `HASHED` keys are
   unchanged (the module is not in `HASHED`, :46/:50-63), so `receipt_problems`'s `set(shas) == set(committed)` holds.
   The launcher is SHA-pinned and unchanged. CAM-K2-E's own `identity_01/K2s1_smoke` equals its `eval_01/K2s1` 205/4501
   cell's `episodes.jsonl` (checked: True), i.e. determinism across stages is already shown for this checkpoint/path.
   => A 1-cell identity reproduction is a sound sentinel; the remaining 23 reference cells rest on that code argument.
2. **Hard links (dedup 2026-11-10).** `camk2e_manifest_ok(K2E, blocks)` = True on all 72 reference files (re-run here).
   The identity reference `eval_01/K2s1/...205bars__seed4501/{result.json,episodes.jsonl}` have link count 1; only its
   `kroc2027_episodes.jsonl` is linked (to `K2s1_smoke`), which no check reads. Nothing in the unit writes into CAM-K2-E raw.
3. **Env forwarding.** `run_cell` puts `VALIDITY_ENV` into `extra` (camk2v3e_run.py:191-193); `gr_run.cell_env` applies
   `extra` after `K.base_env` scrubbing (gr_run.py:102-110: the scrub only filters the *inherited* env); the launcher only
   tests `NAVRL_VEH_RAND`/`NAVRL_DEPTH_MODEL` (eval_navrl_camk2e_arms.sh:49-53), exports through `play_navrl.sh` without
   `env -i`, and snapshots every `NAVRL_*` (:236). No NAVRL allowlist in kroc2027_eval/recorder refuses unknown keys.
   `navrl_task` does not read the rule's keys, so `result.json` conditions are unaffected. String values "5"/"0.5" parse
   to the receipt's `5`/`0.5` (navrl_depth_validity.py:64-79), equal to `T3.DEPTH_VALIDITY`.
4. **`receipt_problems(cell, "K2s1", ...)`.** Arm-dependence is only `treated(arm)` (camk2e_run.py:278); settings are
   checked per key of `expected_settings(seed, on)` (:287-289), so the extra `depth_validity` key is tolerated; snapshot
   treatment keys are `TREATMENT_KEYS` only (:319-322). On the real CAM-K2-E K2s1 cell the only problem with HEAD's
   committed hashes is the `realveh_entry` SHA (expected: old commit), so new cells will pass.
5. **Snapshot binding.** Real CAM-K2-E diffs: K2s1 vs K2s2 (same block) = {BULK_EVAL_JSON, EVAL_CHECKPOINT,
   EVAL_RUN_NONCE, REALVEH_RECEIPT}; eval vs identity adds KROC2027_RUN_ID; conditions equal in all pairs, including
   K2 vs K1 checkpoints. `SNAPSHOT_EXPECTED` (camk2v3e_analyse.py:64-65) covers these plus the 3 rule keys and excludes
   the module-seed keys, so matched seeds (10·seed+1/+2) are bound exactly. No spurious failure expected.
6. **Checkpoints.** `V.checkpoints()` on this host: 4 checkpoints, 0 problems (K2s1 769a26f4, K2s2 e76eb048, V3s1
   888e8056, V3s2 203deea4). Prereg table matches.
7. **Rate gate.** CAM-K2-E mean eval cell 157.24 s (all 60; main arms only 157.96 s), identity mean 177.21 s. At ratio
   1.0 the projection is 8,302 s (2.31 h); the gate refuses only above ratio 1.52. Disk: CAM-K2-E raw ≈ 7.4 MB/cell →
   ≈ 0.4 GB for 52 cells (5.8 GB free).
8. **Controls/prereg numbers.** Recomputed: null D on K2s1−K2s2 = NEGLIGIBLE, 0.00 pp, boot [−0.533, +0.537],
   t ±0.781 (prereg exact); CAM-K1-E controls as in the table (+37.67 WORSE, −0.92 NEGLIGIBLE, −37.67 MATERIAL, N labels).
   CAM-K2-E A_s1/A_s2 −3.80/−4.74, hit_token 0.6958/0.6898 vs 0.1668/0.1916 — all match.
9. **Validity checks are non-vacuous.** Tests cover a positive on/off cell and 13 negatives; on the real CAM-K2-E cell
   OFF passes and ON fails. Not requiring near>0 is justified: per-block `V2Rs ≠ V2s` (analysis :172, `A2.differ`) plus the
   rule's bit-for-bit passthrough of non-dense pixels (module docstring) means any difference implies the rule acted.

## Findings

### MAJOR

**M-1. No gate-D control at claim scale.** The gate's decision band is 2–5 pp, but the known-bad control is +37.67 pp and
the MATERIAL-reachability control −37.67 pp (prereg table rows 1, 3); only the null is at claim scale. Repo rule
("controls at claim scale"; the 2026-11-04 gate checklist) is not met, and CAM-K2-E data provide free claim-scale
controls on this very grid, condition and interval scheme (computed here with `gate_d`, fresh generator seed 20,261,108):

| Control (CAM-K2-E eval cells, treated) | Label | Mean | Bootstrap | t |
| --- | --- | --- | --- | --- |
| K1s1 − K2s1 (known bad at claim scale) | WORSE | +3.80 | [2.93, 4.63] | [2.57, 5.04] |
| K2s1 − K1s1 (SMALL reachable) | SMALL_REDUCTION | −3.80 | [−4.63, −2.93] | [−5.04, −2.57] |
| K1s2 − K2s2 / K2s2 − K1s2 | WORSE / SMALL_REDUCTION | ±4.74 | ±[4.24, 5.25] | ±[4.01, 5.48] |

Fix: add at least "gate D on CAM-K2-E's K1s1 − K2s1 must read WORSE" and "on K2s1 − K1s1 must read SMALL_REDUCTION" to
the prereg table and to `main_eval` controls (load the K1s1 cells from `k2e/eval_01/K1s1`, extend `camk2e_manifest_ok`
rels to K1s1), with a test. Before commit only; no GPU cost.

### MINOR

**m-1. T is not "training with the rule" alone.** T_s = V3s − V2Rs inherits V2R's train/eval input mismatch with the
opposite sign (if the rule hurts v2's unchanged policy, T looks like a benefit of training), and it includes run-to-run
training variance of two separate 1,000-epoch runs (one checkpoint each). Prereg (§Questions 2, §May be said, §Disclosed
limits 2) discloses the mismatch only for R. Fix: word T as "v3 against v2, both evaluated with the rule (v2 outside its
training inputs)"; add to *May not be said*: "that T measures the benefit of training with the rule"; note that the
K2s1−K2s2 null (0.00 pp) is the only available proxy for between-run training variance.

**m-2. Analysis path not smoke-tested end to end.** No test exercises `load_eval`/`main_eval` (bindings, snapshot
allowance with a real CAM-K2-E snapshot as positive control, refusal on a changed seed key). I verified the key sets by
hand (item 5), but per the gate checklist, add a test that copies one real CAM-K2-E cell's snapshot, swaps the
expected keys and adds the 3 rule keys (accepted), then changes `NAVRL_DEPTH_SEED` (rejected); ideally a synthetic
`main_eval` run on a temp dir to `EVAL_BINDING_FAIL`.

**m-3. Approval scope.** The user's quote asks to "start the v3 evaluation". Half the eval cells (24 V2R cells, ≈1.1 h)
and the R/T decomposition are a design extension. The prereg asserts this approves the unit; state in the execution
approval/report that V2R arms are included, so the scope the user approved is explicit (AGENTS.md: approvals name scope).

### LOW

- L-1. §Disclosed limits 1 says "one of their 12 blocks": the reference has 24 cells over 2 checkpoints; V2s2 is never
  re-run. Optionally add a 5th identity cell `V2s2_identity` (e.g. 70 bars / 4503, rule off) ≈ 2.5 min; otherwise reword.
- L-2. "about the last 0.26 m" is an inference of the depth-probe review (REVIEW.md:45, "inferred"); say so.
- L-3. `validity_summary` covers the first 50 frames only; say "first-50-frame counts" in the reported list.
- L-4. The analysis trusts the eval report's `validity_sha256`/`committed_sha256`; `script_equals_eval_commit` makes this
  acceptable, but a one-line check against `git show <commit>:navrl_depth_validity.py` would close it.
- L-5. The analysis does not re-check the reference cells' `receipt.checkpoint_sha256` against `K2_SHA`; the committed
  manifest binds them, so this is cosmetic.
- L-6. Identity runs all 4 cells even after a failure (camk2v3e_run.py:350-357); cost ≤ 12 min, as CAM-K2-E.

## Predictable refusals / vacuous passes

None found. Preflight inputs (approval file aside, which is the user's), checkpoints, CAM-K2-E folder, launcher pin,
committed-file list, receipt/telemetry checks and the rate gate all pass or are reachable with the real data on this
host. The only untested-on-GPU risk is the rule's freshness RuntimeError in play mode; the identity stage
(V2Rs1_negative) would catch it after one cell, which is its purpose.

VERDICT: AMEND_FIRST
