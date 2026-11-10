# S4R-1 v1 amendment 2 — independent review

Date: 2026-11-10. Reviewer: Codex fresh-context delegated reviewer `/root/amend2_review`; inherited root Codex model. Exact deployment/model identifier is not exposed to this reviewer. This is not an Opus review.

Verdict after the repair below: **PASS_WITH_LIMITS**. **No open required source repair.** Scope is preparation of the new bounded attempt, not a repeatability result or execution approval.

## Findings and repair

**MINOR-1, repaired and re-reviewed:** the initial diff's `test_previous_approvals_are_refused_under_amendment2` read the mutable real `execution_approval.json`, allowed only historical v1/amendment1 scopes and required rejection. Installing the required valid amendment2 approval would then make PUBLIC_CPU fail. Root removed that optional on-disk assertion. Current `tests/test_sitl_s4r1.py:1030-1045` retains explicit synthetic v1/amendment1 rejection controls and a current-SCOPE/HEAD acceptance control. Repair changes no operational rule; independently rerun scope tests pass. No other BLOCKER, MAJOR, MINOR or LOW finding in this amendment.

## Scope and evidence

Reviewed actual working diff against HEAD `fe7daed4a521bde2ded617f66cdcfd7934d50f72`, new amendment 2, original preregistration and amendment 1, relevant tests and shared launch/validity helpers.

- `tools/sitl_s4r1.py:52-57,79-92` names new smoke03_contact_on_env0 and full attempt02, new SCOPE, original preregistration and both amendments in source binding. Scope is quoted verbatim in amendment 2; current positive and two historical negative approval tests pass.
- Operational code is otherwise unchanged: 0.15 s LATE validity (`tools/sitl_s3b_formal.py:226-232`); launch passes `invalid_abort=0` (`tools/sitl_s4r1.py:1182`), shared `S3C.launch_unit` passes it to `SF.run_unit` with `primary=None`, which stops at first invalid anywhere. There is no continuation, replacement boot, trimming or threshold change.
- Resource constants remain 4 GiB launch free, 1 GiB minimum free, 3 GiB output, 8 GiB owned RSS, 180 s per boot, 600/1800 s outer and 60 s grace (`tools/sitl_s4r1.py:62-69`). Launch applies disk, resource, runtime, plugin/model and instrumentation-smoke refusals before execution (`:1136-1183`). Shared smoke check binds HEAD/source bytes. Scene, seed, sensor, policy, bridge, contact tolerance and analysis code are unchanged.
- Historical attempt01 stays ABORTED/INCOMPLETE_INVALID: its reading records eight boots, invalid member7, zero comparisons. Raw reading is byte-identical to committed `attempt01_reading.json` (`cmp` PASS). Amendment 2 explicitly preserves it and labels host scheduling stall as a hypothesis (`amendment2:10-14`), then fixes one new attempt at the same bound (`:18-30,40-44`). No original preregistration or historical evidence is in the diff; not all raw files were independently rehashed.
- The experiment-index update for S4 amendment2 was still pending at review time. Root identified it as part of the second separate unit commit; it must be included before that commit, with current status/lineage. This is a record-completion prerequisite, not a source repair.

## Independent verification and limits

Ran `python -B tests/test_sitl_s4r1.py TestS4R1.test_prereg_quotes_scope_and_names_xauthority TestS4R1.test_previous_approvals_are_refused_under_amendment2 -v`: **2 tests PASS**, no skip. CUDA hidden, user site disabled, OMP/MKL/OpenBLAS threads 1, cores 6,7, nice 10, 6 GiB address-space cap, 90 s timeout/5 s kill grace. `git diff --check` PASS. No live PX4/Gazebo/GPU or historical reanalysis run. Root owns the focused/full CPU qualification; no full-suite result is claimed by this reviewer.

Remaining workflow gates: successful focused/full CPU checks, records/index/WORKLOG and separate local commit, then new scoped final-HEAD user approval. Live smoke03/attempt02 remain NOT_RUN; closing Cursor neither establishes prior delay cause nor ensures next validity. No subsequent attempt is implied.

## Reviewed byte bindings

| File | SHA-256 |
| --- | --- |
| `tools/sitl_s4r1.py` | `9c50cd23983af2e0739d8f7eefe2e5e80a13796b84f631e4d9c450e44aabd83f` |
| `tests/test_sitl_s4r1.py` | `8ee4c956af66765beb09b5d4f209e173a7518c9a33088f879df590dcabf5348b` |
| `docs/plans/prereg_sitl_s4r1_v1_amendment2_2026-11-10.md` | `cdc9254bfd664add6eee5459e6e79da8b2e10e85fb03cb5af1db8d0dc751137b` |

Git Traceability: branch `camonly/dev`; base/review HEAD `fe7daed4a521bde2ded617f66cdcfd7934d50f72`; no reviewer commit; only this external audit report written; repository dirty paths belong to root; tests above; Remote Push NO; Main Merge NO; raw/checkpoint integrity: reviewer made no writes.
