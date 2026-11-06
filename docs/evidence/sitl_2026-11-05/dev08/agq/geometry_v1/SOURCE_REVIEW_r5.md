# Source review r5: AGQ geometry diagnostic (unit A) and S3a-P1 policy precision (unit B)

| Unit | Verdict | Open after r5 |
| --- | --- | --- |
| **A**, AGQ geometry-v1 | **PASS_WITH_LIMITS** | BLOCKER 0, MAJOR 0, MINOR 1 (new: R5-m1), LOW 5 (R5-l1 to R5-l3, plus residual survivors from R4-l2 and R4-l6) |
| **B**, S3a-P1 policy precision | **PASS_WITH_LIMITS** | BLOCKER 0, MAJOR 0, MINOR 0, LOW 3 (N13 residual, N14 module-path residual, new B5-l1), plus r1–r3 LOWs not re-scored |

**All round-4 findings claimed as repaired are resolved**, except N13 (unit B, editorial), which is only partly resolved.

**One new MINOR in unit A, to be decided before the preregistration is committed (R5-m1).** The R4-m1 repair works:
a stop or bounce at the contact can no longer enlarge N's reference. But it trades a lenient bias for a strict one,
and the preregistration does not say so:
- When N decelerates or turns away from the bar just before the contact, the extrapolated candidate
  underestimates N's clearance at `t_c`.
- X's minimum is still estimated from chords, which overestimate a curved closest approach.
- So a box member that came *closer* than N, or exactly as close, can read INCONCLUSIVE_NOT_EXPOSED. Whether it does
  depends on the contact's phase between checkpoints.

None of my probes gave a wrong conclusive reading. The cost is power, and this study reads one pair per scene.

- **Reviewer:** Claude Opus 5.5 (`claude-opus-5-5`), fresh context. This was an independent, read-only audit
  (round 5).
- **Date:** 2026-11-06, 03:50–04:03 KST.
- **Object:** `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `0abb766`. The 7
  files are staged (`A`), not committed.
- **What I wrote:** only under `source_review_r5/`.
  - Nothing went into any worktree, and I changed no Git state. I ran only `git rev-parse` and `git status`, and
    `git_state()` ran inside the unit-B tests.
  - Every test and probe ran with `TMPDIR=source_review_r5/tmp/`. That folder is empty at the end: I deleted my own
    mutant copies, `tmp/mutants_r5`.
- **No simulator and no GPU.** I started no Gazebo, PX4 or SITL process, and set `CUDA_VISIBLE_DEVICES=""` for every
  process.

## Hash check (`sha256sum -c BYTES_UNDER_REVIEW.sha256`)

| File | SHA-256 | Start 03:50:46 | End 04:00:02, and 04:01:57 after writing this report |
| --- | --- | --- | --- |
| tools/sitl_agq_geometry_diag.py | `540bc71c…` | OK | OK |
| tools/sitl_agq_geometry_launch.py | `189e05ef…` | OK | OK |
| tests/test_sitl_agq_geometry_diag.py | `65a48deb…` | OK | OK |
| docs/plans/prereg_sitl_agq_geometry_v1_2026-11-06.md | `3b2d416d…` | OK | OK |
| tools/sitl_s3a_policy_precision.py | `94f32e51…` | OK | OK |
| tests/test_sitl_s3a_policy_precision.py | `c461eb6e…` | OK | OK |
| docs/plans/prereg_sitl_s3a_policy_precision_v1_2026-11-06.md | `85eecbe1…` | OK | OK |

**Changes in the worktree during the review.** None of them is mine, and none touches the reviewed bytes.
- The `tests/` and `aerial_gym/` directory mtimes changed at 03:52:23 and 03:52:24, and
  `aerial_gym/rl_training/rl_games/train_session_logs/.navrl_training.lock` was written at 03:52:36. These coincide
  with the author's concurrent PUBLIC_CPU run (`tools/run_tests.py`, PID 3560866, started 03:50:53, cores 8–11).
  My two test runs had ended by about 03:52:10.
- `docs/evidence/sitl_2026-11-05/dev08/agq/geometry_v1/README.md` was rewritten by another session. It is not
  reviewed here.

**R4-l5 is resolved.** `tools/__pycache__/` no longer holds an `agq` or `s3a` `.pyc`.

**The kernel's machine-check, segfault and GP-fault count was 0 at the end** (`journalctl -k -b 0`).

---

# Unit A: AGQ geometry diagnostic

## Round-4 findings

| ID | r4 severity | r5 status | Evidence |
| --- | --- | --- | --- |
| **R4-m1**: the chord reference is enlarged by a stop or bounce at the contact | MINOR | **Resolved.** It introduces R5-m1. | `_path_extrapolated` (diag:703-710); `ref = min(...)` over both candidates (diag:776-781). I re-ran r4's probe (`r4probes_on_r5_bytes.json`). In all 12 P2 cases (stop or bounce; δ₁ = 0.01, 0.05, 0.09 s; X at 0.26 or 0.27 m), the reference is exactly 0.25 m and the reading is NOT_EXPOSED. In r4, 6 of the 12 read CONSISTENT. **Paired positive controls** (`p2_positive_controls_r5.json`): with X at 0.25 or 0.24 m under the same stop or bounce, all 12 read CONSISTENT. r4's P1 band case and its control are unchanged: NOT_EXPOSED and CONSISTENT. The new test (test:567-575) kills the author's `no_extrapolation` mutant, and my `ref_max_of_candidates` and `ref_extrapolation_only`. |
| **R4-m2**: false floor-impact sentence | MINOR | **Resolved** | The new prereg :290-292 is true for diag:751-756. A floor onset is read only if every earlier disturbance had a contact before it. If it is read, it gives DISTURBANCE_WITHOUT_CONTACT with its own `onset_s`. Otherwise an earlier free disturbance is reported. r4's floor probe on these bytes (`r4floor_on_r5_bytes.json`, byte-identical to r4's output) gives onset 6.95 for both contact-preceded cases, and INCONCLUSIVE_NOT_REPRODUCED for the control. |
| **R4-l1**: floating-point drift of the 5 ms grid | LOW | **Resolved** | Integer-indexed grid (diag:721-722) with 1e-9 m slack (diag:82, :784). r4's P4 `identical_V` now reads CONSISTENT at all four phases (r4: NOT_EXPOSED at all four). Samples per window: 201 grid points plus the end (P3). The mutant `grid_accumulated` is killed. **Residual (R5-l2):** the `slack_zero` mutant survives. Prereg :206 matches the code. |
| **R4-l2**: test gaps for targeted mutants | LOW | **Partially resolved** | The author's survivors are now killed: `box_checkpoints_only` by the straddle case (test:563-566) and `nonfinite_clearance_kept` by test:609-610. `mutants_v6/run.log` (complete) reads control OK, 4 of 4 killed, `ALL_KILLED_AND_CONTROL_PASSES True`. My `exposure_step_50ms` is now killed too. Still alive against the Reading class (`mutants_r5.log`): `in_progress_inclusive`, `box_window_narrow_0p2`, `box_after_1p0`, `slack_zero`, `extrap_strictly_before` (equivalent for the test paths) and `extrap_from_first_and_last` (R5-l2). The launcher's `run01` refusal is untested, as in r4. |
| **R4-l3**: "X's window" wording | LOW | **Resolved** | Prereg :199-205 now says "its exposure window `[t_c − 0.5, t_c + 0.5]` s" and "over the exposure window". This matches diag:782. |
| R4-l4: v5 presented as fully pinned | LOW | **Resolved** | v6 reports its own summary, True. |
| R4-l5: stale `.pyc` | LOW | **Resolved** | See above. |
| R4-l6: `inf` clearance fields would crash `json.dump` | LOW | **Open** (unreachable, as in r4) | diag:781-783 unchanged in this respect. |

## Adversarial check of `_path_extrapolated` and the `min()` reference

| Case | Result |
| --- | --- |
| N approaches uniformly (straight, constant speed) | Both candidates agree to 1e-15 m. Q2 (`r5_reference_bias_probe.json`): at δ = 0, 0.03 and 0.07 s, X at 0.26 m reads NOT_EXPOSED; X at 0.25 or 0.24 m reads CONSISTENT. The 1e-9 m slack absorbs a 9e-16 m difference at δ = 0.07. |
| `t_c` exactly on a checkpoint | The extrapolation returns that checkpoint (w = 0, Q3), and so does the chord. P4 `identical_chord_tc_5.7000` reads CONSISTENT. Correct. |
| Fewer than two checkpoints at or before `t_c` | `None`, so the chord alone is used (Q3 `one_before`). This matches r4's behaviour. Correct. |
| The last two checkpoints share one time | `None`, so the chord is used. No division by zero. |
| `t_c` after N's last checkpoint | The extrapolation is defined but the chord is not. This cannot be reached when reproduction holds (`t_c ≤ onset ≤ fin_n`). |
| Is the reference ever larger than N's true clearance? | Not for motion with a constant second derivative. If the clearance curve is convex (decelerating or turning away), the chord is too large and the extrapolation too small. If it is concave, the reverse holds. The `min()` therefore never exceeds the truth, and a post-contact stop or bounce cannot move the extrapolation. So the code comment (diag:776-779) and prereg :202-203 are true. |
| Is it ever *smaller* than the truth, so that it rejects valid cases? | **Yes, systematically, when N decelerates or turns away before the contact: R5-m1.** |

## New findings, unit A

### MINOR

**R5-m1. The `min()` reference rejects valid shallow-graze pairs when N decelerates or turns away before the
contact. The rejection depends on the contact's phase between checkpoints, and the preregistration does not
disclose it.**

Where:
- `tools/sitl_agq_geometry_diag.py:776-784`, the reference and the comparison;
- prereg :199-208, §6.5.

The mechanism:
- With normal acceleration a away from the bar and checkpoint spacing Δ, the extrapolated candidate overshoots
  toward the bar by a·δ₁(δ₁+Δ)/2. Here δ₁ is the time from the last checkpoint to `t_c`, so the overshoot is at most
  a·Δ².
- The chord candidate is larger in this case, so `min()` selects the extrapolation.
- X's minimum is still taken from chords (5 ms samples on chords). Chords overestimate a curved closest approach.
- The two estimates are therefore biased in opposite directions. In r4 they were biased the same way.

Probe Q1 (`r5_reference_bias_probe.json`):
- A smooth turn-away: the clearance follows c* + a(t−t*)²/2 near t*, with slope 1.1 m/s outside.
- N's rotor touches when c = 0.25 m. X flies the same curve to c* < 0.25 m without contact.
- So every case is valid: X came closer than N.
- N after the contact either continues on the curve or stops; both gave identical counts.
- Grid: c* ∈ {0.245, 0.24, 0.23, 0.22, 0.20} m and δ₁ ∈ {0.01, 0.03, 0.05, 0.07, 0.09} s.

| a (m/s²) | r5 NOT_EXPOSED (of 25) | r4 chord-only NOT_EXPOSED (of 25) | Largest reference deficit |
| --- | --- | --- | --- |
| 1 | 2 | 0 | 0.009 m |
| 2 | 5 | 0 | 0.017 m |
| 4 | 11 | 0 | 0.034 m (ref 0.2158 against a true 0.25) |

- The rejections lie where t* − t_c ≲ 0.12 s, that is, where X's true minimum is within about 3 cm of N's contact
  clearance.
- They grow with δ₁. For example, at a = 4 and c* = 0.24, δ₁ = 0.01 and 0.03 read CONSISTENT, while δ₁ ≥ 0.05
  reads NOT_EXPOSED.
- **The same asymmetry fails r4's identical-path positive control.** In P4 `identical_chord` (X's checkpoint path
  equals N's, with a V-shaped closest approach at `t_c`), 3 of 4 phases now read NOT_EXPOSED: X measures 0.261 to
  0.285 m against a reference of 0.25 m. r4 read CONSISTENT in those three phases.

Why MINOR and not MAJOR:
- The error is conservative. A valid pair becomes INCONCLUSIVE_NOT_EXPOSED; no probe produced a wrong conclusive
  reading.
- It gates CONSISTENT and DISTURBANCE_PERSISTS_WITHOUT_CONTACT alike, so it removes evidence on both sides.
- The only recorded trace (env5 native, r3) approached straight at about 1.1 m/s through the contact. There the two
  candidates agree.

Why it still matters:
- The study reads one pair per scene, so one lost pair is the whole reading for its scene.
- A rotor graze during an avoidance turn is a natural way for a rotor contact to happen.
- Plausible PX4 horizontal accelerations are 2–5 m/s² (default `MPC_ACC_HOR` 3). That is NOT_VERIFIED for these
  members.
- No prereg sentence is false: :208 "counts **only if** X came at least as close" is a necessary condition, and it
  still holds. But a reader would take §6.5 to mean that "as close as N" is enough, and it is not.

Fix before commit (the preregistration is immutable afterwards). Do one of these:
1. **At minimum, disclose and record.**
   - Add to §6.5 or §8: "When N decelerates or turns away from the bar before the contact, the reference can be
     smaller than N's clearance at `t_c` by up to about a·Δ² (≈ 0.02 m at 2 m/s², 0.04 m at 4 m/s²). A box member
     whose closest approach falls in that band reads INCONCLUSIVE_NOT_EXPOSED."
   - Write both candidates into the output (`native_clearance_interpolated_m`, `native_clearance_extrapolated_m`),
     so that a pair decided by their difference is visible.
2. **Or remove the asymmetry.** Use the 50 Hz ULog `vehicle_local_position_groundtruth` for both N's reference and
   X's path. It is already recorded and SHA-bound. That cuts both errors by about 25×, to ≈ 1.6 mm at 4 m/s², and
   also removes R5-l1. It needs the NED-to-MOTAR transform and new tests, so it is a larger change.

### LOW

| ID | Location | Finding | Fix |
| --- | --- | --- | --- |
| R5-l1 | diag:671 with `sitl_dev08_link.py:60-61`, :337-343 | **Truth timing (pre-existing, not introduced by this repair).** `p_true` is the latest truth received within [−30 ms, +3 ms] of the position sample whose time is `sim_s`. So it lags `sim_s` by up to 30 ms, by about half a 50 Hz period on average. A lag enlarges N's interpolated and extrapolated clearance at `t_c` by v_n·τ, about 0.011 m at 1.1 m/s and 10 ms, and at most 0.033 m. X's *spatial* minimum is unaffected by a uniform lag, so this is favourable to exposure, and `min()` does not remove it. Its magnitude is NOT_VERIFIED: I found no recorded `snapshots.jsonl` under `~/motar_tmp`. | Report the bracketing checkpoints' `truth_recv_skew_s` with the pair, or state the bound in §8. Option 2 of R5-m1 removes it. |
| R5-l2 | test file | **Surviving targeted mutants** (`mutants_r5.log`; Reading class only): `slack_zero` (R4-l1's slack is not pinned; Q2's δ = 0.07 case shows that a 9e-16 m difference would flip without it); `extrap_from_first_and_last` (any secant through pre-contact checkpoints passes, because every test path is straight before `t_c`); and r4's `in_progress_inclusive`, `box_window_narrow_0p2` and `box_after_1p0`. | Add a curved pre-contact N path, a case equal to 1e-15 m, and r4's paired cases. |
| R5-l3 | test:547 | The test is still named `test_exposure_does_not_depend_on_sampling_phase`. Under R5-m1 the reading does depend on the phase, in the strict direction. Only the name is wrong. | Optional rename. |

## Tests (unit A)

- `python -B tests/test_sitl_agq_geometry_diag.py -v`: **25/25 OK in 63.5 s**, rc 0, max RSS 50 MB (`test_agq.log`,
  cores 16–17).
- The fixtures write under `TMPDIR`, which is empty at the end.
- Nothing appeared in the worktree from my runs (see the changes noted above), and no `.pyc` was created under
  `tools/` or `tests/`.

---

# Unit B: S3a-P1 policy precision

## Round-4 findings

| ID | r4 severity | r5 status | Evidence |
| --- | --- | --- | --- |
| **B4-l1**: Unicode `\d` | LOW | **Resolved** | `re.fullmatch(r"[0-9]+", …)` (tool:384). The test refuses `-1`, `٣` (U+0663), `0,1` and `" 0"`, and asserts the reason "one GPU index" (test:207-210). Absent GPU indices (for example `7`) still pass, as accepted in r4. |
| **B4-l2**: Policy F's pin checked after `mkdir` | LOW | **Resolved** | `sha256_file(_f_checkpoint()) != F_SHA` runs before `mkdir`, inside the `try` that turns errors into "inputs: …" (tool:386-391). `_f_checkpoint` imports `sitl_bridge` for its constants only; its top-level imports are stdlib. Prereg :113 ("an input or Policy F differs from its pin") is now true. Tested at test:214-215 with the reason asserted. The check inside `measure` (tool:248) stays as a second check. |
| **N11(d)**: post-run `git_state()` not caught | LOW | **Resolved** | tool:420-423: `git = {"error": …}`, and `result.json` is still written. Residual B5-l1 below. |
| **N13**: editorial | LOW | **Partially resolved** | The "This applies when either density is unfaithful" phrase is gone (:66). "Otherwise each density …" (:69) now follows a blank line, so it is no longer a lazy continuation of item 3. **Residual:** items 4 and 5 (:74, :77) still say "for both densities" under a per-density classification. This is not false at the reading level, because the reading is the common label only when both densities agree. But it is ambiguous for `density_readings`. |
| **B4-l3**: refusal tests use the host's G8 and assert only rc | LOW | **Resolved** | `verify_inputs`, `_f_checkpoint` and `F_SHA` are pinned for the whole test (test:170-174). `refused()` asserts rc 2 and the reason for the CUDA, inputs, Policy F and single-use cases. The `1e999` and NaN cases still assert only rc 2. With every other refusal reason pinned away, their only remaining cause is strict JSON, so they are now non-vacuous by elimination. |
| N14: `loaded_module_files` not checked to lie inside ROOT | LOW | Open (unchanged) | — |

## Preflight `cpu_only_v4`

- `tool.sha256` = `94f32e51…`, the reviewed bytes.
- The npz `83c26c21…` is byte-identical to `cpu_only_v3`, and so to v1 and v2.
- Every JSON field equals v3's except `actions_npz.path`, `affinity` (8–11 against 16–19), `git.dirty` and
  `wall_s`.
- The reading is CPU_ONLY, with `density_readings` null.
- Prereg :101-103 ("`cpu_only_v4/` from the bytes committed with this file") is true, provided exactly these bytes
  are committed.

## New finding, unit B

| ID | Severity | Location | Finding | Fix |
| --- | --- | --- | --- | --- |
| B5-l1 | LOW | tool:405 | `attempt.json` calls `git or git_state()` again, after `mkdir` and outside any `try`. If `git` failed between :383 and :405, the directory would be spent without `attempt.json` or `aborted.json`. This is practically unreachable, because the call at :383 has just succeeded. | Reuse the state from :383. |

## Tests (unit B)

- `python -B tests/test_sitl_s3a_policy_precision.py -v`: **13/13 OK in 28.9 s**, rc 0, max RSS 0.73 GB
  (`test_s3a.log`, cores 18–19).
- The tests ran `git` read-only, and `test_refusals` read `journalctl -k`.
- `ROOT / "gpu_run01"` is asserted absent (test:197).
- Nothing new appeared in the worktree.

---

## Commands run

All Python ran under `CUDA_VISIBLE_DEVICES="" PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
PYTHONDONTWRITEBYTECODE=1 TMPDIR=source_review_r5/tmp timeout -k 10 900 nice -n 10 taskset -c <16,17 | 18,19 | 16-19>
bash -c 'ulimit -v 12582912; /home/fair/miniconda3/envs/aerialgym/bin/python -B [-I] …'`.

1. `sha256sum -c BYTES_UNDER_REVIEW.sha256` at the start and the end. `git rev-parse HEAD` and `git status --short`.
   `find <worktree> -newer .marker_before`. `ls tools/__pycache__`. `journalctl -k -b 0 | grep -c …`.
2. The tests: `tests/test_sitl_agq_geometry_diag.py -v` (16–17) and `tests/test_sitl_s3a_policy_precision.py -v`
   (18–19), under `/usr/bin/time -v`. Output in `test_agq.log` and `test_s3a.log`.
3. r4's `r4_exposure_probes.py` and `r4_floor_onset_probe.py` on these bytes, giving `r4probes_on_r5_bytes.json`
   (`ea9df7cd…`) and `r4floor_on_r5_bytes.json` (`7e200ab8…`, identical to r4's).
4. P2's paired positive controls, an inline script, giving `p2_positive_controls_r5.json`.
5. `scripts/r5_reference_bias_probe.py` (`f4e903b3…`) → `r5_reference_bias_probe.json` (`85e9d504…`). It compares
   against an in-memory chord-only patch (r4's behaviour):
   - Q1, the smooth turn-away grid;
   - Q2, the uniform-approach control;
   - Q3, the boundaries of `_path_extrapolated`.
6. `scripts/r5_mutants.py` (`8738a220…`; r4's harness with patterns updated to these bytes) →
   `mutants_r5_results.json` (`5602d658…`) and `mutants_r5.log`. 15 mutants plus the control, against the Reading
   class: 9 killed and 6 survive.
7. Read-only:
   - r4's REVIEW and scripts;
   - `mutants_v6/run_mutants.py` and `run.log`;
   - `cpu_only_v3/v4` (hashes and a field comparison);
   - `sitl_dev08_step.py:110-160` and `sitl_dev08_link.py:55-61`, :337-425 (the provenance of `p_true` and
     `sim_s`);
   - the changed sections of both preregistrations and the code.

## Limits

- No live Gazebo, PX4 or GPU run. These are NOT_VERIFIED:
  - N's real kinematics through a rotor contact, which decides whether R5-m1 matters;
  - the truth lag (R5-l1);
  - every GPU reading of unit B.
- R5-m1's counts come from a synthetic parabolic turn-away on a grid I chose. They show the mechanism and its scale,
  not a rate in the real runs.
- My mutants ran against the Reading class only. The author's v6 run covered the full test file, with 4 mutants.
- The PUBLIC_CPU suite was not run by me. The author's concurrent run (PID 3560866) was still running when I
  finished, and I did not read its result.
- Not reviewed: the evidence READMEs, `docs/EXPERIMENTS.md` (staged, modified) and the launcher beyond r4.
- Unit B's LOWs from r1–r3 were not re-scored.
