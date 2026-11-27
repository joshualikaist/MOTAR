# S5 safety v2 amendment 2 — independent review

Date: 2026-11-10. Reviewer: Codex fresh-context delegated reviewer `/root/amend2_review`; inherited root Codex model. Exact deployment/model identifier is not exposed to this reviewer. This is not an Opus review.

Verdict: **PASS_WITH_LIMITS** for the reviewed implementation and amendment. **No open required source repair.** This is a CPU/code review, not a live safety result or permission to execute.

## Scope and findings

Reviewed actual working diff against HEAD `fe7daed4a521bde2ded617f66cdcfd7934d50f72`, new amendment 2, original immutable preregistration and amendment 1, associated tests, runtime/analysis helpers and historical smoke02 ledger/final record. No BLOCKER, MAJOR, MINOR or LOW defect found in this amendment.

- `tools/sitl_s5_safety.py:1029-1047` binds installed mavutil and both current-dialect v10/v20 source files without selecting or importing a decoder. Missing files and invalid names raise; the binding update is inside the link-close `finally` (`:1078-1102`). Live `Link` always supplies `mu` (`tools/sitl_s3b_closed_loop.py:123-127`), so the no-mavutil escape is confined to injected doubles.
- The installed `mavutil.py:284-310` really selects v20 at the first v2 frame and retains the global dialect. The regression tests use two real connections in a fresh subprocess; old active-decoder binding changes, new three-file binding remains equal (`tests/test_sitl_s5_safety.py:1728-1781`). Positive two-record COMPLETE and old-binding ABORTED controls make the check non-vacuous.
- Each of the three source-byte mutations still aborts, and a missing v20 file raises (`tests/test_sitl_s5_safety.py:1783-1805`). `SF.binding_core` excludes only generated world and otherwise compares all file SHA values (`tools/sitl_s3b_formal.py:213-217`); the new source keys participate. Per-episode path/log variation remains outside comparison. Analysis still checks every bound file against disk (`tools/sitl_s5_safety.py:1874-1877`).
- New SCOPE/smoke03 and both amendment source bindings are explicit (`tools/sitl_s5_safety.py:90-104`); prior scopes are rejected with a current-scope positive control. First-invalid stop (`:1634-1680`), ten scenarios, three-null smoke, parameter/acceptance rules, 8 GiB RSS and launch/resource bounds are unchanged.
- Amendment 2 preserves both aborted smokes, proposes fresh smoke03 and the never-created attempt01, and requires final-HEAD approval. Raw smoke02 ledger is byte-identical to its committed evidence copy (`cmp` PASS). No historical evidence or original preregistration is in the diff; this review did not exhaustively rehash all raw files.

## Independent verification and limits

Ran `python -B tests/test_sitl_s5_safety.py TestPymavlinkBinding -v`: **3 tests PASS**, no skip, 0.443 s; actual two-connection subprocess executed. CUDA hidden, user site disabled, OMP/MKL/OpenBLAS threads 1, cores 6,7, nice 10, 3 GiB address-space cap, 90 s timeout/5 s kill grace. `git diff --check` PASS. No PX4, Gazebo or GPU launched; full PUBLIC_CPU is root-owned and was not duplicated or claimed complete here.

Remaining workflow gates: successful focused/full CPU checks, recorded review/index/WORKLOG and separate local commit, then a new scoped user approval at final HEAD. Live smoke03 and failsafe discrimination remain NOT_TESTED. These are existing workflow prerequisites, not open review repairs.

## Reviewed byte bindings

| File | SHA-256 |
| --- | --- |
| `tools/sitl_s5_safety.py` | `5c3a457824d7d9243940de15a649d20fc8c1c398111965195e31b1cceb52811c` |
| `tests/test_sitl_s5_safety.py` | `cb34835854ded433191f3b2fc128f0cde1b649705d179ca948ef20ad052bef9f` |
| `docs/plans/prereg_sitl_s5_safety_v2_amendment2_2026-11-10.md` | `9b51afee14aba722a6e5db46490159a9fd57d8e2b54711a20cbaffefa17a5a83` |

Git Traceability: branch `camonly/dev`; base/review HEAD `fe7daed4a521bde2ded617f66cdcfd7934d50f72`; no reviewer commit; only this external audit report written; repository dirty paths belong to root; tests above; Remote Push NO; Main Merge NO; raw/checkpoint integrity: reviewer made no writes.
