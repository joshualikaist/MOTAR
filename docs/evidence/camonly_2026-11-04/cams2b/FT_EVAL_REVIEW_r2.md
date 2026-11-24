# CAM-S2b-FT-E — delta review r2

- Reviewer: **Claude Opus 5.5**. Same context as r1; read-only; this file is the only file written.
- Files reviewed: `tools/cams2b_eval_run.py` (`e2a70941…`), `tools/cams2b_eval_analyse.py` (`e8c82f2d…`),
  `tests/test_cams2b_eval.py`, prereg DRAFT r2. Worktree HEAD is `dca6e54`; the training package is now committed and
  this package is still uncommitted.
- Tests: `-m unittest test_cams2b_eval` from `tests/`, CPU, cores 6,7,12,13, no CUDA. **27 tests, OK, none skipped.**
- CPU probe on the real tree (read-only): `reuse_problems` currently returns exactly one problem, CAM-S2a-E's
  `eval_run_report.json` not yet written. This is expected.
  - `rate_projection` at a 717.7 s wall gives 8,999.96 s against the 9,000 s cap, so the prereg's "≤ 718 s" is
    correct.

## Verdict: **GO**

There are no P0, P1 or P2 findings. Every r1 P1 and P2 is addressed. The changed paths add no failure that would show
only on the GPU. Five P3 notes follow.

## Changed paths checked

1. **Identity loop** (`run:932-955`):
   - FT_identity runs first, and its FAIL breaks the loop before A60_repro.
   - A VOID result in either cell breaks the loop and voids the stage.
   - `identity_verdict` (`:649-654`) needs FT_identity OK and A60_repro OK or FAIL, with both records present.
   - Before A60_repro, the budget leaves at least about 700 s, because FT_identity ≤ 718 s is the eval gate's own
     limit and the cap is 1,440 s. A60_repro at 175 bars needs about 350 s, so a cap cut is unlikely.
   - The four `reuse_decision` readings (`:617-629`) are ordered correctly: path FAIL, then input problems, then
     reproduction.
   - Stage a60 runs only on RERUN_REQUIRED (`:821-823`).
   - The analysis maps A60_NOT_REPRODUCED and A60_PATH_FAIL to source None, which reads G1_NOT_ANSWERED.
2. **`check`** (`:835-865`):
   - Without `--approved-commit` it uses HEAD, and the other stages still call `ap.error`.
   - It writes nothing:
     - `out_root` is only tested with `exists()`.
     - `import_origin` runs `python -B`.
     - `committed_sha256_at` uses `git show`.
     - The link stage is `"smoke"`, which reads nothing.
   - Invoking the runner without `-B` would write `tools/__pycache__`, but that path is git-ignored
     (`.gitignore:17`).
3. **Reuse:**
   - The `result_sha256` cross-check (`:603-606`) reads `rec.stats.result_sha256`. The CAM-S2a-E runner writes that
     field: it is the launcher receipt's `result_sha256`, the SHA-256 of `result.json`.
   - The manifest has 6 cells × 8 files + 2 reports = 50 entries. `manifest_problems` uses the same count.
   - The side file exists in all 20 CAM-S2a-E cells so far, and `realveh_entry.py:78` writes it.
   - The `raw=None` defaults resolve at call time. That lets the tests patch `S2A_E_RAW` and keeps production on
     `raw_a1`.
4. **Rate gate:**
   - Eval: 11.4 × w × 1.1 ≤ 9,000 s. a60: 8.01 × w × 1.1 ≤ 4,500 s, so w ≤ 511 s; CAM-S2a-E's 175-bar YC12k cells
     took 332–362 s.
   - Both factors match CAM-S2a-E's receipts.
5. **Analysis:**
   - G1 code (`analyse:71-82`) matches prereg row `:193`. A capture gain with a crash label of NO_DETECTED_CHANGE or
     UNRESOLVED now reads FT_UNRESOLVED.
   - The recheck uses `ev["tree"]` and `a60_report["tree"]`; reused cells still use `S2A_E_TREE`.
   - The reuse-mode end-to-end test is a real positive and negative control:
     - `reuse_problems` returns `[]` on a faithful CAM-S2a-E-shaped tree. The labels read REPORTED / REUSED.
     - Changing one reused `result.json` is caught both by the manifest and by the record SHA, and G1 then reads
       A60_BINDING_FAIL.
   - A wrong recorded tree is refused.
6. **Prereg r2 against the code:**
   - Branch `camonly/dev` only (`BRANCHES`).
   - Caps 0.25 / 0.4 / 2.5 / 1.25 h appear in both SCOPE and `MAX_HOURS`.
   - The controls SHA `e8c82f2d…` equals the current analysis file, and `controls_camk1e_6blocks.json` records the same
     `script_sha256` with `ok: true`.
   - The 50-file manifest, the FT_identity ≤ 718 s limit and the 27-test count all match the code.

## Findings (P3 only)

| ID | Finding | Where |
| --- | --- | --- |
| P3-1 | `reuse_decision` returns A60_PATH_FAIL when A60_repro never ran (FT_identity FAIL; harmless, the stage is IDENTITY_FAIL anyway) or was cut by the identity cap ("launcher exit" at budget), which the label reads as "the A60 path fails under this runner". A distinct `A60_NOT_RUN` / budget reading would be more truthful; low probability given ≥ ~700 s left. | run:617-629, 953 |
| P3-2 | The eval gate's margin is thin at its edge. Using the prereg's own factors (FT45 is about 0.88× FT at 175 bars and 1.33× at 205; YC12k against S12k in CAM-S2a-E), an eval at w = 718 s is expected to take about 12.07 w ≈ 8,660 s of the 9,000 s cap, about 4 % slack. A cut reads INCOMPLETE, so neither gate is labelled. The factor is disclosed (prereg:134-136) and acceptable. | run:665-677 |
| P3-3 | `check` lists "no approval file" until the approval exists and then exits 2, even when nothing else is wrong. A chain must not gate on its exit code before approval. It checks no stage links (it uses `"smoke"`), so before identity, eval or a60 it does not show link problems. Say both in the prereg's `check` paragraph, and give the exact command with `python -B`. | run:859-865; prereg:109-111 |
| P3-4 | `ft_training_summary` reads `run_report.json` at analysis time without comparing its SHA to `ev["ft_train_report_sha256"]`. It is reported only, so add the SHA comparison to the output. | analyse:280-289 |
| P3-5 | Process: the prereg says it was written before S2b-FT trained. Commit this package before the FT launch, so the record shows that the rules were fixed before the policy they judge existed. Any later edit of the analysis changes `e8c82f2d…` and requires regenerating the controls. | prereg:3-6, 215 |
