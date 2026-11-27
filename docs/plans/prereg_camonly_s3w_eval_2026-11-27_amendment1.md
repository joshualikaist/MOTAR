# CAM-S3-E amendment 1 — review r1 fixes before any data (2026-11-27)

- **Amends:** [`prereg_camonly_s3w_eval_2026-11-27.md`](prereg_camonly_s3w_eval_2026-11-27.md) (committed at 8fbf950,
  byte-unchanged). Written before any CAM-S3-E GPU cell. Where the two differ, this file holds.
- **Why:** independent review r1 (Claude Opus 5.5, fresh context, read-only;
  `~/motar_tmp/cams3e_20261127/review_r1/REVIEW.md`).
  - Verdict: NO_GO on P1-1 (authorization) only; the code is GO_WITH_FIXES.
  - Four P2 on the decision rules and the GPU budget; ten P3.

## Authorization
**Pending the user's explicit approval of this scope (review r1 P1-1).**
- The user's reply "우리 원래 하던거부터 진행하자" answered "평가 사전등록을 작성해서 리뷰를 받을까요?". It approves writing
  and reviewing this package, not the GPU run.
- The GPU stages start only after the user explicitly approves this scope, recorded in their own words in
  `~/motar_tmp/cams3e_20261127/execution_approval.json` with this scope's SHA-256 (`tools/camref_stage3_run.py scope`).
  The scope string changed with this amendment, so the earlier SHA-256 3bfb10f1… no longer applies.
- The user is told before approving:
  - the five-arm design, including S0M and S2b-FT at its own 60 deg/s;
  - about 1.4 h expected, the 2.0 h chain cap and the 2.4 h unit ledger.
- **Rerun rule:** any rerun of a stage needs the user's approval naming the hours the unit ledger reports as remaining.
  This covers a new output folder, a failed or voided attempt, and an exception that wrote no report.
- The CAM-S3 conditional approval and its 6 h total are unchanged. This unit stays inside the 2.4 h left of it.

## Changes
### P2-1 — a crash reduction with a capture loss is not an improvement
- Each labelled Q2 comparison gets a status:
  - **IMPROVES:** crash MATERIAL and negative, and capture not MATERIAL and negative.
  - **CRASH_DOWN_CAPTURE_DOWN:** both MATERIAL and negative, i.e. a more passive policy.
  - **NO_IMPROVEMENT:** otherwise.
- The "helps, not enough" pointer needs IMPROVES (see the pointer table below).

### P2-2 — W − S0M is co-primary; the 60 deg/s departure
- **Q2 (labelled, co-primary)** is now W − S0, **W − S0M** and WT − S0T.
- W − S0 compares the two flight candidates, each in its own trained runtime and sensor model. W − S0M compares them
  under the same measured sensor model. When the two disagree, the pointer says so instead of choosing one.
- Q3 keeps S0M − S0, reported and unlabelled.
- **Departure from the S3-W preregistration.** Its "What is compared later" section sketched both policies "with the
  vehicle's 90 deg/s". Here each policy runs at its own trained action scale: S2b-FT at 1.0472 rad/s (60 deg/s), S3-W at
  1.5708 rad/s (90 deg/s).
  - The yaw-rate ceiling is the action scale: the policy's yaw output times NAVRL_YAW_RATE_MAX, with the Lee yaw clamp at
    the same value.
  - Running S2b-FT at 1.5708 would multiply every yaw output it learned by 1.5. That is an action mapping it never
    trained on, and the task flags it as a CONFIG MISMATCH.
  - A clip at 90 deg/s would never bind for S2b-FT, whose scale stops at 60 deg/s.
  - Each candidate would fly with its own runtime, so S2b-FT at 60 deg/s is its flight configuration. The comparison asks
    which candidate is better as it would fly, not what 90 deg/s does to S2b-FT.
- W − S0 and W − S0M therefore include the yaw ceiling among the confounded changes. Nothing here measures 90 deg/s
  alone.

### P2-3 — the bar reads stage 1's Wilson rule next to the point estimates
- **Q1 label renamed:** BAR_MET_POINT / BAR_NOT_MET_POINT. The test is unchanged: crash ≤ 20 %, capture ≥ 70 %,
  timeout ≤ 10 %, on the pooled point estimates.
- **Reported with it:** stage 1's one-sided 95 % Wilson label (`camref_eval_analyse.bar_label`: BELOW_20 / UNRESOLVED /
  ABOVE_20) for all crashes and for bar-contact crashes. The same is reported for WT, S0, S0T and S0M.

**Use of the result (replaces the preregistration's):**

| Q1 | All-crash Wilson label / Q2 statuses (W − S0, W − S0M) | Pointer |
| --- | --- | --- |
| BAR_MET_POINT | BELOW_20 | RUNTIME_NEXT (runtime unit, then the Orin timing) |
| BAR_MET_POINT | UNRESOLVED or ABOVE_20 | RUNTIME_NEXT_QUALIFIED (bar met on point estimates only; the user chooses between more seeds and the runtime unit) |
| BAR_NOT_MET_POINT | both IMPROVES | HELPS_NOT_ENOUGH |
| BAR_NOT_MET_POINT | exactly one IMPROVES | SENSOR_MODEL_DEPENDENT (both intervals reported; the user decides) |
| BAR_NOT_MET_POINT | none IMPROVES, one CRASH_DOWN_CAPTURE_DOWN | CRASH_DOWN_CAPTURE_DOWN (structural next) |
| BAR_NOT_MET_POINT | otherwise | STRUCTURAL_NEXT (memory S3-M, or the learning rate; for the user) |
| NOT_RUN (no complete block) | — | NOT_RUN |

### P2-4 — a unit-wide GPU ledger
- **UNIT_GPU_H = 2.4 h:** the CAM-S3 approval's 6 h minus S3-W's recorded use. S3-W used the two smokes (1 and 3 min)
  and its 3.53 h run (`docs/evidence/camonly_2026-11-04/cams3/*run_report.json`), rounded down.
- **What `unit_ledger` counts:** every `*_report.json` under `~/motar_tmp/cams3e_20261127`, whatever its stage, verdict
  or output folder (failed and voided attempts included). It also counts every cell directory, one with a `cell.log` or
  `receipt.json`, that no report lists, i.e. a run whose runner raised before writing its report. Such a cell counts the
  launcher receipt's `duration_s`, else the span of its files' modification times.
- An unreadable report counts the span of its folder's files.
- **A stage's cap** is min(the sub-cap, 2.0 h minus this chain's earlier stages, 2.4 h minus the ledger). The stage-1
  machinery then applies it: a projection that does not fit skips the cell, and an exhausted cap refuses the stage.
- The report field `earlier_stages_h` now holds the unit ledger's hours.
- The rerun rule (Authorization) applies on top.

### P3 (code and tests; wording)
- **P3-1:** the analysis binding refuses an eval report whose verdict is not COMPLETE / INCOMPLETE (e.g.
  VOID_HOST_FAULT). The identity report must still read IDENTITY_PASS.
- **P3-2:** per-cell summaries read only the complete seed blocks.
- **P3-3:** the pointer is NOT_RUN when Q1 or a co-primary comparison is NOT_RUN.
- **P3-4:** the stage-3 liveness controls (measured depth, sector k 24 over 13 bins, yaw scale, arena line) are checked
  on every stage-3 cell by the per-cell receipt check, not only in smoke.
- **P3-5:** the test loaders of earlier runners remove their temporary folder whole, so a run without `-B` leaves none
  and does not fail. This covers `tests/test_cams3_eval.py` and the same defect in `tests/test_camref_stage2.py`.
- **P3-6:** the preregistration's "the CPU suite passes" means the full PUBLIC_CPU suite shows no failure or error
  beyond the recorded baseline. The baseline is listed in the WORKLOG: 9 failures / 8 errors with
  `MKL_NUM_THREADS=1 OMP_NUM_THREADS=1`; the suite was not green before this unit.
- **P3-7, wording:**
  - "Does it crash less than S2b-FT under one protocol?" reads "under one task protocol, each policy in its own runtime".
  - "the CAM-S3 changes lower the crash" reads "W crashes less than S0 (yaw, arena, masks, sector rule and 3,000 epochs
    confounded)".
- **P3-8 (reported only):** camsafe and speed-governor intervention shares per cell. They are taken from the last
  `[camsafe] calls=… capped_frac=… penalised_frac=…` line and the last `NavRL speedgov | mode=… intervene=… stop=…`
  line of each cell log, as those modules log them (running shares). Each log's SHA-256 is recorded; a log without
  them reads NOT_RECORDED. W is set beside S0 and S0M. This follows the user's rule that filters are auxiliary; the
  shares carry no label.
- **P3-9:** the analysis binds `camref_stage3_analyse.py`, `camref_eval_analyse.py`, `camref_stage3_run.py` and
  `camref_eval_run.py` to the eval commit.
- **P3-10:** the launcher copy's three stale comments name S3W. Its SHA-256 pin changes to 94dc5ffe…; the stage-1
  launcher is still 51f949d0….

## Analysis controls (replace the preregistration's list; a miss reads CONTROL_FAIL)

| Case | Required result |
| --- | --- |
| All cells equal (29 % crash) | Q2 all NO_DETECTED_CHANGE; BAR_NOT_MET_POINT; STRUCTURAL_NEXT |
| W crash −8 pp at 21 % | W − S0 and W − S0M IMPROVES; WT − S0T NO_DETECTED_CHANGE; HELPS_NOT_ENOUGH |
| W at 18 / 73 / 9 % (crash / capture / timeout) | BAR_MET_POINT, BELOW_20, RUNTIME_NEXT |
| W at 22 / 70 / 8 % | BAR_NOT_MET_POINT |
| W at 17 / 71 / 12 % | BAR_NOT_MET_POINT |
| Review probe 1: 19.6 / 70.5 / 9.9 % | BAR_MET_POINT, UNRESOLVED, RUNTIME_NEXT_QUALIFIED |
| Review probe 2: W 53 % capture / 21 % crash against 65 / 29 | W − S0 CRASH_DOWN_CAPTURE_DOWN, CRASH_DOWN_CAPTURE_DOWN |
| Review probe 3: S0M at 40 % crash, W at 27 % | W − S0 NO_IMPROVEMENT, W − S0M IMPROVES, SENSOR_MODEL_DEPENDENT |
| W with one seed block only | Q1 NOT_RUN, pointer NOT_RUN, no per-cell summary of W |

## Unchanged
Arms, environments, smoke, identity, eval plan, the 2.0 h chain cap and sub-caps, the bootstrap
(`default_rng(4711)`), the MATERIAL / SMALL labels and the limits of the preregistration.

## Review
The fix diff (8fbf950 → this amendment's commit) is re-reviewed before any GPU stage, and the user's explicit approval
of the new scope SHA-256 is required (P1-1).
