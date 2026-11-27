# CAM-K2 v3 pre-launch design review r2 (focused re-check of the r1 repairs)

- Reviewer: Claude Code subagent, Opus 5.5 (claude-opus-5-5), fresh context, read-only on the worktree.
- Date: 2026-11-09, about 12:40-12:55 KST.
- Target: MOTAR-camonly `camonly/dev`, HEAD `d2a018b`, staged unit (9 files; worktree == index for all of them).
- Input: r1 `~/motar_tmp/camk2v3_20261108/review_r1/REVIEW_design_r1.md`. The staged evidence copy
  `docs/evidence/camonly_2026-11-04/camk2/REVIEW_v3_design_r1.md` is byte-identical (sha256 ead66ecb...).
- Method: I diffed every staged file against its `wip/` copy (the r1 "as reviewed" state, `wip/SHA256SUMS.txt`).
  The only changes are the repairs listed below. `tools/realveh_entry.py` and `tests/test_depth_validity.py` are
  unchanged (49d1db92..., 2d799e64...).
- What I ran (cores 8,9, nice 10, OMP/MKL=1):
  - `test_camk2v3_train` 18 OK; `test_depth_validity` 17 OK; `test_experiment_index` 9 OK.
  - A mutation check and a sys.path check (below).
  - I did not run the full suite. The coordinator's PUBLIC_CPU run (`public_cpu_precommit.log`, OMP/MKL exported)
    was still running at 12:39 with no result line.

## Repairs verified

**B-1 (fixed).** `tests/test_camk2v3_train.py:43` sets `_PATH_RESTORED = sys.path == _saved_path` right after the
`finally`, and `:354` asserts it. This is v2's pattern.
- Check: I imported the module, inserted a sys.path entry afterwards (as later suite modules do), then ran
  `TestIsolation`. Result: OK.
- The r0 suite log shows the old failure. That run had no OMP setting, and its only unit failure was this one.

**M-1 (fixed, cannot refuse a normal run).**
- `tools/camk2v3_train.py:452` now requires `near_pixels > 0` on the summary line only. `near <= invalid` then
  implies `invalid > 0`.
- Semantics (`navrl_depth_validity.py:_telemetry`): `pixels`, `invalid_pixels` and `near_pixels` on the summary
  line are cumulative sums over the first 50 calls × num_envs frames (128 envs, so 6,400 frames), all pixels.
  They are not per-frame counts.
- Real v2 depth-model summaries (same 50-call window, same CAM-K1 ep28000 start):
  - `invalid_minz` was 0.03698 (smoke), 0.03775 (s1) and 0.03748 (s2) of about 46.0-46.3 M eligible pixels, that is
    about 1.7 M sub-MinZ pixels per run. The first frame was 0.0 in all three.
  - The probe turned 99.99 % of sub-MinZ pixels into near. A refusal would need essentially zero dense sub-MinZ
    pixels across 6,400 frames, so the risk is negligible.
  - Placing the clause on the summary line only is correct.
- The negative test `rule_never_acted` (`tests/test_camk2v3_train.py:257`) really exercises the clause:
  - With the clause, the case produces exactly one problem, "summary line implausible".
  - With the clause deleted (a mutant exec'd under the real `__file__`), the case produces no problem at all, so the
    test would fail.
  - The control (`:264`) still passes.

**M-2 (fixed, derivation consistent).**
- `MIN_FREE_GB = 0.6` (`:74`), checked at every preflight on the same filesystem. The test pins (0.6, v2 1.0).
- Measured:
  - v2 s1/s2 run folders are now 21.62 MB each. The 19 deleted intermediates per seed are 8,875,860-8,876,630 B,
    so a seed with intermediates kept is 21.6 + 168.6 + 28.6 (unit dir) = 0.219 GB.
  - Smoke: 17.8 + 26.7 = 0.045 GB.
  - `df -B1 /` Available is 1,312,428,032 B.
- Projected free space:

  | Point in the chain | Free |
  | --- | --- |
  | Seed-1 preflight | 1.27 GB |
  | Seed-2 preflight | 1.05 GB (about 0.45 GB above the floor) |
  | End | 0.83 GB |

  This matches the prereg's 0.22 / 0.045 / 0.84 GB. The "about 0.35 GB headroom" figure is 0.38 GB, which is fine.

**m-1 to m-4 (fixed).**
- m-1: the index bullet now sits after CAM-K2-E's sub-bullets, at the right nesting.
- m-2: the other invalid-band types are disclosed.
- m-3: the detector-mask sentence is present.
- m-4: the DIAG_LOW wording is softened.

**m-5 and L-2/L-4 (fixed).**
- m-5: the chain script is named. It stops unless smoke reads SMOKE_OK and seed 1 reads TRAINED. An empty verdict
  also stops it.
- L-2: the docstring is corrected.
- L-4: the "draft" headers are removed.

## Findings

### MINOR

- **N-1. A stale test-status claim in a file that is frozen at commit.**
  - In the prereg, *Tests* is headed "superseded by the worktree runs in *Review before the commit*", but that
    section records no worktree run.
  - The same section also still says "The PUBLIC_CPU suite has not run. It must run in the worktree before the
    commit."
  - *Fix:* before the commit, add one line to *Review before the commit*: the PUBLIC_CPU result (log path, Ran /
    failures / errors, faults before and after). Name the expected residual: the 2 `test_boia_planner` 3 GB disk-floor
    errors. Also add this r2 verdict and its evidence copy.
  - Commit only if the running suite shows no failure other than those.

### LOW

- **L-a. The probe's code hash no longer matches.** `navrl_depth_validity.py` is now f06a8897...; the probe ran
  8be04135.... The diff is docstring lines 1 and 22 only, and the launcher and wrapper changes are likewise comments
  or the repairs. *Fix:* one sentence in the prereg, so the hash mismatch is explained.
- **L-b. "3.7 % of pixels" should read "of eligible pixels".** The rule's `pixels` includes no-hit pixels, so in the
  rule's own units the share is smaller.
- **L-c. The chain script is not pinned.** It is named without its SHA-256 (r1 m-5 asked for both). r1 L-3 (bind
  `wip/SHA256SUMS.txt`) and L-5 (the receipt's `sha256.realveh_entry` value changes) are not addressed. These are
  optional.

## Verdict

All B/M repairs are correct and complete, and they introduced no new defect. The one remaining item before the commit
is to record the PUBLIC_CPU result in the prereg text (N-1).

VERDICT: PASS_WITH_LIMITS
