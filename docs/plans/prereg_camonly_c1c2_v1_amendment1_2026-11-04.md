# Amendment 1 to the C1/C2 preregistration (after the design review AMEND_FIRST, before C2)

Dated 2026-11-04. Amends [`prereg_camonly_c1c2_v1.md`](prereg_camonly_c1c2_v1.md); that file is unchanged.

**Timing.** C1 ran on `1908fd8` and read C1_IDENTITY_PASS:
- `full` equals GR-0's cell exactly;
- `no_lidar` differs: 798 captures and 1,250 crashes against 1,864 and 186.

That C1 is recorded (`results/camonly_c1c2_v1/c1/`), but this amendment's commit changes HEAD. So C1 runs again on
this commit (`c1_r2`) before C2, and C2 accepts only that. No C2 cell exists before this amendment.

**Review.** Independent pre-measurement design review (Claude Opus 5.5, fresh context): **AMEND_FIRST**, with no
blocking finding (`docs/evidence/camonly_2026-11-04/c1c2/REVIEW_design_r1.md`). Each change below names the finding
it answers. Nothing changes the ablation module, the entry point or the launcher copy.

## Analysis (`tools/camonly_c2_analyse.py`)

- **M1. New contrast `obs_all` = C_gL − A** (OOD): removing all LiDAR from perception only. It answers the
  preregistration's fourth question, and total = obs_all + gov_all.
- **M4. Labels with a practical band.** The band is 3 pp for capture and 2 pp for crash, GR-0's bands. GR-0's S00
  block SD of paired differences, 0.7-1.2 pp, gives an 80 % minimum detectable effect of about 1 pp, so 12 blocks
  suffice. The labels:

  | Label | Condition |
  | --- | --- |
  | UNRESOLVED | the bootstrap CI and the t-interval disagree on excluding 0, or on the side |
  | LOWER / HIGHER | both exclude 0 on that side, and \|mean\| >= band |
  | SMALL_LOWER / SMALL_HIGHER | both exclude 0 on that side, and \|mean\| < band |
  | NEGLIGIBLE | neither excludes 0, and both intervals lie inside the band |
  | NO_DETECTED_CHANGE | otherwise |

- **New bindings.** Any failure reads EVAL_BINDING_FAIL:
  - L1: GR-0's report is COMPLETE, and its 12 S00 cells (episodes, result, receipt) match the committed manifest
    `docs/evidence/gain_randomization_2026-11-04/SHA256_MANIFEST.json`. C1 and C2 name the same `--gr0-dir`.
  - M3: each C2 cell's recorded condition (`result.json` minus the nonce) equals its GR-0 cell's. Its NAVRL
    snapshot differs only in the five keys measured on C1 against GR-0: `NAVRL_BULK_EVAL_JSON`,
    `NAVRL_EVAL_RUN_NONCE`, `NAVRL_KROC2027_RUN_ID`, `NAVRL_SENSOR_ABLATION` and
    `NAVRL_SENSOR_ABLATION_RECEIPT`.
  - M3: C1 and C2 recorded the same allowlisted runner environment.
  - M2: in every block, B_in differs from B_in_gL and C differs from C_gL. This shows the governor ablation
    engaged in the simulator; C1's `no_lidar` control could not show it.
  - L9: C1's `no_lidar` cell (205 bars, seed 4501) equals C2's.
- **Reported, not gated:**
  - M2: speed-governor telemetry per condition, sample-weighted from `result.json`: intervention rate, mean
    executed speed, mean clearance, mean scale.
  - L6: crash causes now include out of bounds. They are counted over the aggregate window of `result.json`, not
    the quota.

## Runner (`tools/camonly_run.py`)

- **M3.**
  - Both reports record the allowlisted runner environment: `AERIAL_*`, `PYTHON*`, `PATH`, `LD_*`, `CUDA*`,
    `NVIDIA*`, `OMP_*`, `MKL_*`, `TORCH*`, `CONDA*`, `LANG`, `LC_*`, `HOME`, `USER`, `DISPLAY`, `VK_*`, `__GL*`.
    No secret matches the list.
  - C2 refuses to start unless that environment, and the `--gr0-dir`, equal C1's.
- **L5. Budget.** The default cap is 3.5 h. GR-0's S00 cells average 143 s, so 60 cells take about 2.4 h.
  - If C2 ends INCOMPLETE on the budget, it is rerun whole into a new folder with a 4.5 h cap.
  - A partial C2 is never pooled with a rerun.
  - Any other INCOMPLETE stops the unit for a decision.

## Readings and wording (M5, L2, L3, L4, L7)

- **M5. "May not be said", added:**
  - "`in_fov` or `obs_in` is camera quality". Both also contain the in-FOV LiDAR target association, which this
    design does not separate.
  - "no effect" for NO_DETECTED_CHANGE. Only NEGLIGIBLE supports "no change larger than the band".
- **L2.** The camera's linear angle-to-column map (plan, after r2, M-r2-2) does not cancel between conditions.
  It is exposed when the in-FOV LiDAR is removed, and in B_in and C it reaches the governor. It is part of
  `in_fov`.
- **L3. Contrasts, stated precisely:**
  - `out_fov`, `total` and `interaction` also contain the K2 cap, which removing out-of-FOV LiDAR brings with it.
  - B_in is a smaller shift than the OOD conditions, but it is not shown to be in distribution.
  - `interaction` is a difference of differences. It reads "how far removing both differs from the sum of
    removing each", not a mechanism.
- **L4.** K3's 2 x 2 (observation x governor) has only 3 of its 4 cells for the in-FOV split, and C_gL / C for the
  full split; the "observation keeps the LiDAR, governor loses it" cell is not run.
- **L7. Multiplicity.** 8 contrasts x 2 metrics are reported with no correction. The labels are descriptive, not
  tests of a family-wise hypothesis. Capture and crash are nearly complementary (timeouts are about 0), so they
  are not independent evidence.

## Machinery controls, rerun (L8)

`docs/evidence/camonly_2026-11-04/c1c2/smoke_02.json` passes 14 of 14 checks:
- Each condition now carries a distinct synthetic effect. The analysis's eight contrast means equal a computation
  made directly from the cells, to within 1e-9, so a swapped condition mapping would fail.
- The known-bad cases each give EVAL_BINDING_FAIL or PREFLIGHT_FAIL with the named binding:
  - a dead governor (B_in_gL equal to B_in in one block);
  - an unexpected snapshot key;
  - a C1 / C2 `no_lidar` mismatch;
  - a GR-0 copy outside the manifest;
  - a runner environment different from C1's.
- The earlier 9 checks still pass.

## Not changed

Conditions, grid, quota, cells, the C1 requirements, the ablation module, the entry point and the launcher copy.
