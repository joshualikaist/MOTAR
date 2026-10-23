# B1-R implementation audit before GPU (2026-10-24)

This record covers the implementation of `docs/plans/prereg_b1r_classical_baseline_v1.md`
(SHA-256 `e4607aa2…36e3`, commit `2acc2d5`), checked against the preregistration. It is a CPU-only
check. The reviewer was Claude Opus 5.5, working in the same session as the implementation. That
makes this a self-audit, not an independent review.

- **Resume state.** The six untracked B1-R files were kept and extended. Nothing was regenerated
  from scratch.
- **Starting point.** The work resumed on top of base `8633dce`, which holds the test-isolation fix
  for `motar-6nn.5`.

## Status by item

| Item (preregistration §) | Status | Notes |
| --- | --- | --- |
| Player hook install; fail-closed exit (§3) | COMPLETE | `sitecustomize` loads only the standard library. On an install error it exits with code 3. It patches only `navrl_players`. |
| F path: action and RNG do not interfere (§2, G0) | COMPLETE (CPU) | `F_SHADOW` returns F's action bit for bit, tested in a child process. The baseline uses no RNG, no matmul and no in-place writes to task tensors. **The real check is G0 on the GPU.** |
| B-PN / B-PP laws, saturation, slow-course fallback (§2.2) | COMPLETE | |
| Target LiDAR exclusion, (a) mask and (b) belief radius (§2.3) | COMPLETE | |
| H-R3 filter and speed-scaled r_s; bound; yaw; no-belief scan (§2.3) | COMPLETE | |
| Sidecar writing and integrity (§3, G2) | **REPAIRED** | See repair 1. |
| No overwrite of existing results (§3, §6) | COMPLETE | The stage directory and every cell directory are refused if they already exist. |
| Per-stage GPU wall caps and disk floors (§6 G3) | COMPLETE | Preflight checks free disk against 9.5 / 10.9 / 13.0 / 13.0 GB. During a stage the floor is 8.2 GB. Caps: 900 s per cell, plus the stage caps. |
| Tuning: 32-cell completeness (§5) | COMPLETE | `select` refuses an incomplete or duplicate grid, and refuses quota counts that do not add up. |
| Recorder rows missing or duplicated (§7, G2) | COMPLETE | `check_cell` holds the ledger and recorder checks, and `cell_metrics` performs a strict join. |
| Source SHA and raw manifest checks (§6) | COMPLETE | Preflight requires every source file to be committed and the CPU evidence to be in HEAD. `complete_report` verifies the manifest and every file SHA. |
| Scene disjointness (§5) | **REPAIRED** | See repair 2. |
| Stage B seed census (§5) | **REPAIRED** | See repair 3. |
| CPU evidence and selection binding records (§6) | **ADDED** | `record-cpu`, which accepts relative paths, and `bind-selection`. |
| R01 exploratory crash hazard (§7, Exploratory) | **R01_SOURCE_UNCOMMITTED, not implemented** | See the section below. |

## Repairs

1. **Sidecar write time.**
   - *Problem.* The sidecar was written only by `atexit`, and it reads counters that live on the
     GPU. A comment in the task code (`navrl_task.py:1969-1972`) warns that, at `atexit` time, the
     simulator may already be torn down. If that happened, every cell would fail G2.
   - *Fix.* The patch now also wraps `player.run`, so the sidecar is written in the `finally` block
     when the quota `SystemExit` passes through. `atexit` writes it only if that did not happen.
     The sidecar records which path wrote it in `written_by`.
2. **Scene-hash count.**
   - *Problem.* `select` required exactly `num_envs` scene hashes per cell. But
     `occlusion_layouts.npz` holds one hash per layout, at every reset; there are 1,156 in an
     OCC-P2 cell. The check would therefore have failed every time, after Stage B had already run.
   - *Fix.* The rule is now at least `num_envs` hashes, and it is tested against a real OCC-P2 cell.
3. **Census and dangling symlinks.**
   - *Problem.* The real census took 31 s over 10,419 files and found no hits. Its 26 errors were
     all symlinks whose targets do not exist: VOID or foreign-user `source_manifest.json` links.
     The census treated them as parse errors, so it would always have refused Stage B.
   - *Fix.* A dangling link is now recorded separately, and its name and target text are still
     checked for the seed. A real read or parse failure still fails the census closed.

## R01

- **Where the definition is.** The preregistration's exploratory endpoint names "the R01 method on
  the occlusion frame trace". That method exists only as uncommitted scratch code from session
  `35a63340`, at `/tmp/.../35a63340…/scratchpad/r01/`.
- **What the code does:**
  - it keeps per-frame `lost` and `reacq_recent` flags;
  - "reacquisition" means detection returns after a loss of 5 or more frames, followed by a
    20-frame window;
  - crash labels use horizons of 10 and 20 frames;
  - it takes geometry features from the recorded layouts (oracle, used for evaluation only).
- **Why it is not implemented here.** The code is not committed or frozen. A copy, with no `.npy`
  files, is kept outside the repository at
  `transfers/b1r_handoff_2026-10-24/r01_source_uncommitted_35a63340/`. No new formula is introduced
  under the R01 name.
- **Why it does not block the GPU run.**
  - The endpoint is Exploratory. It is not a gate, a primary endpoint or a secondary endpoint.
  - The raw `occlusion_frames.bin` is recorded in every cell.
  - The analysis can therefore be added later, by a dated amendment that freezes the state
    definitions. Until then it is reported as `NOT_ANALYSED`.

## CPU validation

- **B1-R tests:** 42 PASS in a standalone run. These include the real OCC-P2 G0 comparator, the
  tuning-environment template check and the scene-hash count; none of them was skipped.
- **PUBLIC_CPU:** 3,219 ran, 0 failures, 0 errors, 10 skipped, 5 excluded by profile.
  - Receipt: `public_cpu_receipt.txt`.
  - Machine record: `cpu_validation.json`, which binds the SHA-256 of all six source files.
- **Dry-run preflight for Stage A** (before the commit). The only problems reported were "source not
  committed" and "CPU evidence missing". All of the following passed:
  - the import origin under the hook environment;
  - the GPU check (RTX 3070);
  - the disk check (26.6 GB free);
  - the launcher, player and checkpoint pins;
  - the presence of the templates.
- **Not verifiable on CPU:** the hook inside the real Isaac Gym process. G0 in Stage A is the
  preregistered test of that.
