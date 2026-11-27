# AGQ geometry-v1 source review r3

**Verdict: FAIL.** Open findings: 0 BLOCKER, 1 MAJOR (new: R3-M1), 7 MINOR (R3-m1 to R3-m7; R3-m7 is already fixed in a post-freeze test edit), and notes.

Round 2's two MAJORs are fixed:
- **N1:** confirmed on real ULog bytes with the px4 environment.
- **N2:** confirmed by code reading, the tests and mutants.

N3, N5, N7 and N9 are resolved. N4, N6, N8 and N10 are partially resolved.

The new MAJOR is in the exposure rule of the fixed reading, which freezes when the preregistration is committed:
- The reference is N's last checkpoint up to 0.1 s *before* the contact.
- At the observed approach speed, that reference overstates N's clearance at the contact by up to about 0.1 m.
- So, depending only on the arbitrary phase of the contact against the 10 Hz checkpoint grid, a training-box member that stays beyond the maximum rotor reach can read `CONSISTENT_WITH_ROTOR_CONTACT`.
- The fix is small, and the same fix also clears R3-m1.

- **Reviewer:** Claude Opus 5.5 (`claude-opus-5-5`), fresh context, independent read-only audit (round 3).
- **Date:** 2026-11-06, 03:07–03:40 +09:00.
- **Repository:** `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `0abb766`. The reviewed files are untracked.
- **Writes:** only under `/home/fair/motar_tmp/agq_geometry_diag_20261106/source_review_r3/`.
  - Nothing was written into any Git worktree.
  - No Git state was changed: only `git status`/`log` were run.
  - The probes and mutants ran with `TMPDIR` set to `source_review_r3/tmp/`, which the fixtures cleaned up. Two plain test-file runs used the system `/tmp`, as the test file does by default, and also cleaned up.
- **Simulator and GPU:** no gzserver, gz, px4, Gazebo or SITL process, and no GPU.

## Hash check (`sha256sum -c BYTES_UNDER_REVIEW.sha256`)

| File | SHA-256 | Start 03:07:42 | End 03:26 |
| --- | --- | --- | --- |
| tools/sitl_agq_geometry_diag.py | `3e86c658…` | OK | OK |
| tools/sitl_agq_geometry_launch.py | `189e05ef…` | OK | OK |
| tests/test_sitl_agq_geometry_diag.py | `db008d81…` | OK | **FAILED**: now `fdb89bec…` |
| docs/plans/prereg_sitl_agq_geometry_v1_2026-11-06.md | `c366d621…` | OK | OK |
| tools/sitl_training_geometry.py | `cadd5167…` | OK | OK |

**The test file changed during this review.** Its mtime is 03:13:58, which is not my doing.
- **What changed:** I reconstructed the text from my mutant copy, and its SHA-256 equals `fdb89bec…`. The only change is in `Launcher.test_execute_runs_only_inside_the_envelope`: the subprocess now runs `sys.executable, "-B", "-c", …`. The one-line call became two lines; the file is now 885 lines instead of 884.
- **Which bytes I tested:**
  - both full test runs ended before the change (03:11:04 and 03:13:04), so they ran the reviewed bytes `db008d81…`;
  - my mutants (03:16–03:20) and the ULog-binding probe ran the new bytes. They do not run the changed Launcher test.
- **Effect of the change:** it fixes R3-m7, below. These bytes are not the reviewed ones; re-hash them for the next round.

## Round-2 findings

| ID | Status | Evidence in the current bytes |
| --- | --- | --- |
| **N1** numpy types / JSON crash | **Resolved** | `ulog_series` casts every timestamp, rate, height and dropout to a Python `float` (diag:813-816). `takeoff_check` returns `bool(...)` (:797). Real-bytes probe `n1_real_ulog_probe.json` (px4 env, the real `ulog_series` on the old dev08 members 33, 26, 32 and 27): (a) series types `builtins.float` only, 4327 rate samples for 33, no dropouts; (b) `takeoff_check` at the ULog liftoff gives `ok: true`, a `bool`, with a rise of +0.12/+0.10/+0.12/+0.12 s; at ±0.2 s it gives `false`, the paired control; (c) `member_events` output holds only builtin types and passes `json.dumps(..., allow_nan=False)` (27–34 kB); (d) `pair_reading` on these real-typed events serialises. The new test (test:566-571) feeds `numpy.float64`/`float32`. The fixture `series` in test:690-713 is numpy-typed, and the test asserts `SUPPORTED` and `json.dumps` of the whole `analyze()` result. |
| **N2** box window not covered | **Resolved** | (a) **Rate coverage:** `_covered` (diag:682-685) requires the ULog span to include the window and no inter-sample gap > 50 ms or pyulog dropout to overlap it. It is applied to N over `[0, fin_n]` (:704) and to X over its window (:733). (b) **Box window:** truncated at X's final: `hi = min(t_c+1.5, fin_b)` (:732). Because `hi ≤ fin_b ≤ final+0.5` and `lo = t_c−0.5 ≥ −0.5`, the contact-continuity window `[start−1, final+0.5]` covers it, as prereg:191-192 states. (c) **Units:** dropout units are correct (`duration` ms → s, :816). (d) **Paired tests:** span ends at 6.0 / gap at 6.0–6.2 give NOT_EVALUABLE; a gap at 8.0 gives CONSISTENT; a native span that is too short gives NOT_EVALUABLE; a rate after X's final is ignored (test:506-510, :529-530). (e) The author's v4 mutants `box_coverage_off`, `native_coverage_off` and `box_window_not_truncated` were still pending at my end, but the cases above kill them by inspection. (f) **Old data:** the ULogs cover to final +0.99–1.01 s, so `covered_prefix` is true for all four old members. |
| N3 fault rise at the final read | **Resolved** | `void` also holds when `kernel_faults_end` is None, when it differs from the start, or when `fault_check_error` is set (diag:837-839). The paired test (test:755-759) has a count 0,0,0,0,0,1, which gives COMPLETE but void, with no member evaluable. Its positive control is the complete unit, where all members are evaluable. Prereg:170-171 says so. |
| N4 after-final onsets | **Partially resolved** | Onsets after N's final no longer count (diag:707), and test:527-528 is paired. Residual R3-m3: an onset at or just before the final, such as a floor impact, still counts. |
| N5 excursion in progress | **Resolved** | A native window that began before `t_c` and overlaps `[t_c−0.2, t_c)` blocks reproduction (diag:709). Tests: 523-526. My mutant `in_progress_inclusive` survives; it is a boundary case at `w[0] == t_c` (see R3-m5). |
| N6 exposure reference | **Partially resolved; replaced by R3-M1** | The +0.02 m tolerance and the post-contact samples are gone, and "any bar" is now the contacted bar `b` (diag:736-744). The new reference, "N's last checkpoint in `[t_c−0.1, t_c]`", is pre-contact by up to 0.1 s. That makes it favourable to X (R3-M1). It is also sometimes empty, which makes it unfavourable (R3-m1). |
| N7 900/1200 s budget | **Resolved (note)** | The overall budget is 850 s (diag:58). Prereg:255 states 850 + 240 + 95 = 1185 s. That sum leaves out the post-member audit and hashing, the idle check (≤ 3 s) and the two kernel-log reads (append, finish). It also leaves out the pre-open setup, about 3.4 s in dev08. Measured here: `journalctl -k` took 0.01 s, hashing a 9 MB ULog took 8 ms, and the fixture spent 2.6 s between members. The realistic total is still under 1200 s. Any overrun is a fail-closed RUNNER_EXCEPTION. |
| N8 test gaps | **Partially resolved** | Now tested: (a) the envelope gate in `main` (test:851-854 asserts the "envelope" refusal and that no attempt directory exists, which kills `envelope_off` by inspection); (b) missing stdbuf (test:435-443); (c) numpy series; (d) ULog coverage; (e) a final-read fault. Still open, see R3-m5: (a) the exposure windows are unpinned; (b) the ULog binding is unexercised; (c) the launcher's refusal of an existing `attempt01`/`run01` is untested. |
| N9 only the contact log verified | **Resolved** | (a) **Recording:** `execute` records `member_files` (the SHA-256 of `checkpoints.jsonl` and every `*.ulg` under the member, keyed by relative path; diag:1079-1082) after `run_episode` has closed the session and written the checkpoints. (b) **Verification:** `analyze` checks the contact log, the snapshots (from the recording manifest), the checkpoints and each ULog, and treats a missing binding as a failure (diag:854-864). (c) **Probe** (`ulog_binding_probe.json`; the real `execute` and `analyze` with a fake 1 MB ULog written into each member's `rootfs/log/`): `member_files` carries `rootfs/log/2026-11-06/12_00_00.ulg`, and all four members are evaluable. A changed or a removed ULog makes that member not evaluable, with the path named. Restoring the bytes makes it evaluable again. (d) **Live run:** `rglob` does not follow the rootfs symlinks (`etc`, `test_data`): 21 entries and 1 ULog for old member 33. Hashing adds no measurable delay. |
| N10 false prereg statements | **Partially resolved** | (a) §3 now scopes the receiver identity to "one synthetic stream". (b) §2 says "A dry binding check found 60". (c) §9 lists Rate5 on any axis. (d) "Silence never counts as no contact" is now true for every reading window. (e) The 1200 s arithmetic is stated (see N7). New statement problems: R3-M1 ("no tolerance", "X came at least as close") and R3-m4/R3-m6. |

## New findings

### MAJOR

**R3-M1. The exposure reference is a pre-contact checkpoint up to 0.1 s before `t_c`. A box member that stays beyond any rotor reach can read `CONSISTENT_WITH_ROTOR_CONTACT`, depending only on the contact's phase against the checkpoint grid.**

Where:
- `tools/sitl_agq_geometry_diag.py:81`, `:739-744`;
- prereg :193-197:
  - "N's clearance to bar b at N's last checkpoint in [t_c − 0.1, t_c]";
  - "There is no tolerance";
  - "Absence of contact counts only if X came at least as close to the bar that N's rotor touched".

The mechanism:
- N approaches the bar until its rotor touches at `t_c`. Its clearance at the last checkpoint at or before `t_c` therefore exceeds its clearance at the contact by about `v_perp · δ`, where `δ = t_c − t_k` is spread over [0, 0.1] s.
- The old dev08 checkpoint spacing is 0.096/0.104 s (member 33) and 0.08–0.12 s (members 26, 27, 32) (`scene_dump.json`).
- On the old traces, with the DIAGNOSIS proxy contact times, the rule's reference against N's clearance interpolated at `t_c` is (`n1_real_ulog_probe.json`):

| Member | δ | Rule reference | Interpolated at `t_c` | Excess |
| --- | --- | --- | --- | --- |
| env5 native | 0.024 s | 0.2755 m | 0.2484 m | +0.027 m |
| env2 native | 0.020 s | 0.3010 m | 0.2908 m | +0.010 m |
| env2 candidate | 0.016 s | 0.2582 m | 0.2464 m | +0.012 m |

- The env5 approach is about 1.1 m/s. With δ near 0.1 s the excess is therefore about 0.1 m. That equals the whole band between the base box's reach (0.198 m) and the maximum level rotor reach (0.312 m), which is the band this unit tests.
- This replaces round 2's explicit +0.02 m tolerance with an implicit one. Its expected size at 1.1 m/s is about 0.05 m.

Failure scenario (`r7_exposure_band.json`; `scripts/r7_exposure_band.py`). Setup:
- N touches at a clearance of 0.25 m, approaching at 1.1 m/s, with its onset 40 ms later;
- X has an identical path up to `t_c−0.2`, then stays 0.33 m from the bar (beyond the 0.312 m rotor reach), with no contact and no rate.

Result:
- with the contact **0.09 s** after N's checkpoint, the reference is 0.349 m, so X reads as exposed: **`CONSISTENT_WITH_ROTOR_CONTACT`**;
- paired control, the contact **0.01 s** after the checkpoint: the reference is 0.261 m, giving `INCONCLUSIVE_NOT_EXPOSED`.

`r5_exposure_slack.json` shows the same on the old env5 geometry: X at 0.265 m reads CONSISTENT, although N was at about 0.248 m at the contact.

So:
- the reading turns on a sampling phase unrelated to the mechanism, and in the hypothesis-favourable direction;
- the two preregistration sentences above are false in that band.

Fix, before commit:
1. **Reference.** Use N's clearance at `t_c`, linearly interpolated between N's last checkpoint at or before `t_c` and its first checkpoint after `t_c`. Alternatively, extrapolate it from N's last two checkpoints at or before `t_c`, which uses pre-contact data only. Optionally use the 50 Hz ULog truth.
2. **Comparison.** Require X's minimum to be at or below that reference.
3. **Text.** Correct prereg §6.5 accordingly.
4. **Tests.** Add the paired test above (δ 0.09 against 0.01). Add a test that pins X's window `[t_c−0.5, t_c+0.5]`: my mutant that widens it to `t_c+1.5` survives the current tests.

This also removes R3-m1.

### MINOR

**R3-m1. When N has no checkpoint in `[t_c−0.1, t_c]`, every box member reads `INCONCLUSIVE_NOT_EXPOSED`, so a valid case is rejected.**

Where: diag:739, :742.

The checkpoint spacing exceeds 0.1 s about half the time (old members: 80/120 ms and 96/104 ms patterns). For a uniformly placed `t_c`, the window is empty with probability **10 %** for the 80/120 pattern and **2 %** for the 96/104 pattern (`reading_probes_r3.json`, R1).

Probe R1: `t_c` 0.115 s after a checkpoint, with a box member passing closer (0.215 m), gives `native_clearance_to_bar_m: null` and INCONCLUSIVE_NOT_EXPOSED. The control, `t_c` 0.085 s after a checkpoint, gives CONSISTENT. This is conservative, but it is a guard rejecting a valid input by construction.

Fix: the R3-M1 fix, which needs no lower window bound, or a bound of at least the maximum checkpoint spacing (≥ 0.15 s).

**R3-m2. A non-finite `clearance_m` in `checkpoints.jsonl` crashes `--analyze` after the run.**

Where:
- diag:670 copies each checkpoint's `clearance_m` into `events.path`;
- diag:1158-1159 opens the `x` file, then calls `json.dump(..., allow_nan=False)`;
- `checkpoints.jsonl` is written with `default=float` (`sitl_s3b_dev08_prefix.py` run_episode `finally`).

How it happens:
- `box_hit` returns `inf` when no bar spans the vehicle's height;
- the bars here span z ∈ [0, 3] m (`scene_dump.json`);
- a checkpoint with `p_true[2] < 0` (a crash final below the rest height) or within (3, 4] m (not `alt_bad`) gives `Infinity`.

Consequences: `analyze` returns, but `main` raises ValueError and leaves an empty `--out` file. Fixing that would change a bound source after measurement.

This is unlikely: the old minimum `p_true[2]` is 0.018 m (env5 native final), and no old checkpoint holds Infinity or NaN. Round 2 listed it only as a note. It is cheap to close.

Fix: map non-finite floats to `None` (or `"inf"`) in the analysis output, and test it.

**R3-m3 (N4 residual). In the not-reproduced branch, a native onset at or shortly before N's final checkpoint still counts.**

Where: diag:711-716.

The case is an altitude crash whose floor impact precedes the `z < 0.1` checkpoint. An onset that has no bar contact in the 0.5 s before it turns INCONCLUSIVE_NOT_REPRODUCED into **DISTURBANCE_WITHOUT_CONTACT**. Probe R3: native onsets 6.3 (preceded by its contact) and 6.95 (final 7.0) give DISTURBANCE_WITHOUT_CONTACT, onset 6.95. The control without the 6.95 onset gives INCONCLUSIVE_NOT_REPRODUCED.

The bias is against the rotor hypothesis, but the label is still wrong. The old env5 native reached the floor 8 ms after its final, so either side is possible.

Fix: either ignore onsets at or after N's first post-start floor contact (`contact_audit` would need to record that time), or evaluate the earliest disturbance only. State the choice in §6.

**R3-m4. CONTACT_PERSISTS is keyed on the *first* overlapping box rate window. The prereg says "a rate window".**

Where: diag:750-751; prereg :200-202.

Probe R2: box windows [5.9, 6.0] (no contact) and [6.5, 6.6] (contact at 6.45). The code gives DISTURBANCE_PERSISTS_WITHOUT_CONTACT. Under an "any window" reading of the prereg, the first bullet (CONTACT_PERSISTS) would apply.

Both labels are non-supporting, but they direct different follow-ups (base-box hit against command replay).

Fix: write "the first rate window of X that overlaps it" in §6.7, or change the code to match the text.

**R3-m5. Tests: several fixed-reading boundaries are unpinned, and the ULog binding is not exercised.**

My mutants (`mutants/results.json`): control OK; 2 killed, 7 survive.

| Mutant | Result | What it shows |
| --- | --- | --- |
| `box_exposure_window_wide` (X's exposure window to `t_c+1.5`; favourable) | survives | X's exposure window is unpinned |
| `exposure_native_window_0p5` | survives | the native reference window is unpinned |
| `exposure_ref_min` | survives | the native reference choice is unpinned |
| `box_crash_any_time` (the crash rule ignores `fin_b ≤ t_c+1.5`) | survives | the crash rule's time condition is unpinned |
| `in_progress_inclusive` (`w[0] ≤ t_c`) | survives | the in-progress boundary is unpinned |
| `ulog_not_bound` (ULogs dropped from `member_files`) | survives | the fixture has no `.ulg` file |
| `recomputed_audit_ignored` | survives | equivalent in practice, since the hash check plus `recorder_problems` already gate the same function on the same bytes |
| `native_first_onset_only` | killed | |
| `snapshots_not_bound` | killed | the required-file fallback fails closed |

The launcher's refusals of an existing `attempt01` (launch:68-69) and `run01` (launch:92) are untested.

Fix:
- add paired reading tests for X's exposure window, the crash-time condition and `w[0] == t_c`;
- write a small `.ulg` in the Unit fixture's run, and assert the changed-ULog refusal (my probe shows the code is correct);
- add a launcher test with a pre-existing `attempt01` and `run01`.

**R3-m6. Preregistration wording:**
- **Ledger fields (prereg :74-75).** The §3 list of ledger additions omits `member_files`, `contact_files` and `scene_geometry`, although §6 relies on them.
- **Clock check (prereg :207-216).** With the observed 0.10–0.12 s liftoff-to-rise lag, the window [−0.05, +0.3] passes clock offsets from about −0.16 to +0.19 s. "Cross-clock order SUPPORTED" therefore does not verify the order at the 40 ms contact-to-onset scale, or at the 0.2 s in-progress window, that §6 uses. That order rests on the source-level identity in §3. Say so.
- **Budget arithmetic (prereg :255).** Name what the 1185 s leaves out (see N7), or say that the outer timeout may end a last overlong member as RUNNER_EXCEPTION.
- **Onset filter (prereg :181).** "Only onsets up to N's final checkpoint" sits under *Reproduction*, but the code also applies it to the DISTURBANCE_WITHOUT_CONTACT branch. Say so.

**R3-m7. In the reviewed bytes, the envelope test's subprocess can write bytecode into the worktree.**

Where: test:866 in `db008d81…`.

The test spawns `sys.executable -c <code>`, and that code `exec_module`s `tools/sitl_agq_geometry_diag.py`. It has no `-B` and no `PYTHONDONTWRITEBYTECODE`, so the child can write `tools/__pycache__/sitl_agq_geometry_diag.cpython-38.pyc`. The path is gitignored (`.gitignore:17`), but it is a runtime file under `tools/`, which AGENTS.md forbids during a freeze.

That file exists, mtime 03:14:22. Both of my test runs ended earlier, so the current file was written by another process, probably the author's v4 mutant run, which uses the older test text. My two runs may have rewritten an earlier copy.

The concurrent edit (`fdb89bec…`) adds `-B` and fixes this. It needs to be part of the next reviewed bytes.

### Notes (no change required)

- **Parser robustness.** In 20 000 random cuts of a Contacts line, and in cuts with stderr text spliced in, `contacts_message` raised only `ParseError`. Non-gz-shaped crafted text such as `time: 5` or `contact: 3 …` raises `AttributeError`. That would end `execute` as RUNNER_EXCEPTION (fail-closed) and crash `analyze`. gz does not emit such text. The subscriber's stderr shares the log file description (`sitl_owned_session.py:587-588`). A stderr line spliced into a stdout line longer than the stdio buffer loses that message, which the continuity audit reports as RECORDER_LOSS (fail-closed).
- **First rotor contact.** The reading is keyed on N's first rotor–bar contact. An earlier harmless graze makes the pair INCONCLUSIVE_NOT_REPRODUCED, even when a later contact is followed by the disturbance (probe R4). This is conservative and preregistered; it is worth a sentence in §8.
- **Missing final.json.** `analyze` needs `final.json`. After an outer SIGKILL it raises in `read_ledger`, and no output file is created (the `x` open comes after `analyze`). This is fail-closed.
- **scene_geometry.** It is recorded from `specs[key]` (centers, halves, p0, yaw0, as floats). `bar_clearance` on the old dev08 checkpoints' `p_true` with these arrays reproduces the runner's own `clearance_m` (box_hit, minimum over bars) to 2.2e-16 m on all four old 205-bar members (`scene_dump.json`). The frame, indices and units agree.
- **Box window ends at X's final.** A box disturbance after a capture inside `[t_c+0.5, t_c+1.5]` is ignored. This is consistent with "it ends with X's flight" and necessary for N2. A crash there reads INCONCLUSIVE.
- **Author's mutant run v4** (`mutants_v4/run.log`, still running at 03:23): control OK; 19 of about 40 run, all killed. `hash_unchecked`, `void_final_faults_ignored`, `envelope_off`, the coverage, truncation and in-progress mutants, `order_swapped` and `skip_not_checked` were pending. I did not judge them beyond the inspections stated above.

## Check of the requested reading rules

| Rule | Result |
| --- | --- |
| In progress | Code (:709) equals prereg :182-183. A window starting before `t_c` with its end ≥ `t_c−0.2` blocks reproduction. The not-reproduced branch then reads its onset as DISTURBANCE_WITHOUT_CONTACT when no contact precedes it, which is correct. The equality boundary is untested (R3-m5). |
| Onsets only up to N's final | Correct (:707). The not-reproduced branch uses the same filter. Residual R3-m3. |
| Exposure by box_hit clearance to bar b | The clearance function and the scene are exact (note above). The reference choice is the R3-M1 MAJOR, and the empty window is R3-m1. "No tolerance" is true only literally. |
| Box crashed in its window | Code (:748) equals prereg :198. It is reached only after the paths, coverage and exposure checks pass, and rates take precedence. Its time condition is untested (R3-m5). |
| scene_geometry from the spec | Correct and verified against the old traces. A missing or different scene gives NOT_EVALUABLE (:735-737). |
| Evaluability in `analyze` | Correct: void includes a final-read rise or error; the hashes (contact log, snapshots, checkpoints, every ULog) are compared, and a missing binding fails. Members that are not VALID are never read. |
| `execute` recording | `member_files`, `contact_files` and `scene_geometry` are correct (probe). Cost: hashing a 9 MB ULog took 8 ms, `rglob` 0.6 ms, and `rglob` does not follow the rootfs symlinks. No new raise path in the member loop: a missing directory or file gives None, which `analyze` rejects. |

## Tests

- **Results:**
  - 24/24 OK pinned (cores 16–19, nice 10): `focused_tests_pinned.log`, 62.7 s;
  - 24/24 OK with a different affinity set (cores 0–3 and 16–19, nice 10): `focused_tests_affinity8.log`, 60.5 s.
- **Affinity and nice independence:** the Unit recorder fixture uses `cores=frozenset(os.sched_getaffinity(0))`, `min_nice=-20` (test:625), and the environment check is injected (test:604-605), so the tests do not depend on affinity or nice by construction. I did not repeat r2's nice 0 run, because the given envelope fixes nice 10 and cores 6/7/12/13 and 20–23 are busy.
- **Envelope test:** it passes under an inherited `ulimit -v` (8 GiB) because of the hard-limit branch (test:863-865). It counts 3 problems irrespective of the caller's parent (test:845-846).
- **Paired controls exist for:** every reading label (±0.03 m exposure, shift 0.29/0.31, gap in or after the window, final 6.0/6.5, crash), the void rule, the hash refusal and the clock check (±0.2 s). The gaps are in R3-m5.

## Commands run

All Python ran under `CUDA_VISIBLE_DEVICES="" PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 timeout -k 10 900 nice -n 10 taskset -c 16,17,18,19 bash -c 'ulimit -v 8388608; <python> -B …'`, with two exceptions:
- the second test run used `taskset -c 0,1,2,3,16,17,18,19`;
- two one-line checks (`rglob`, timing) used `timeout 60 nice -n 10 taskset -c 16-19` without `ulimit`.

1. `sha256sum -c BYTES_UNDER_REVIEW.sha256`, at start and end.
2. `python -B tests/test_sitl_agq_geometry_diag.py -v`, twice: `focused_tests_pinned.log` and `focused_tests_affinity8.log`.
3. `scripts/scene_dump.py` (aerialgym env): `scene_specs` scene geometry, the clearance identity on old checkpoints, and the checkpoint spacing → `scene_dump.json`.
4. `scripts/n1_real_ulog_probe.py` (**px4 env**): the real `ulog_series`, then `takeoff_check`, `member_events`, `pair_reading` and `json.dumps(allow_nan=False)` on old members 33/26/32/27 → `n1_real_ulog_probe.json`.
5. `scripts/reading_probes_r3.py` → `reading_probes_r3.json` (R1–R4, R6 parser fuzz). `scripts/r5_exposure_slack.py` → `r5_exposure_slack.json`. `scripts/r7_exposure_band.py` → `r7_exposure_band.json`.
6. `scripts/ulog_binding_probe.py`: the real `execute`/`analyze` on the CPU fixture, with fake ULogs → `ulog_binding_probe.json`.
7. `scripts/mutants_r3.py`: ten targeted mutants on copies in `mutants/` → `mutants/results.json`, `mutants_r3.log`.
8. Read-only: the old dev08 `attempt01` ledger, checkpoints and ULogs; `journalctl -k -b 0 | wc -l` (timing only); `mutants_v4/run.log`.

## Limits

- No live Gazebo or PX4 run. NOT_VERIFIED:
  - the subscriber's real output and its interleaving;
  - the real contact-to-onset timing;
  - the live checkpoint phase distribution.
- R3-M1 and R3-m1 use the old spacing patterns, the DIAGNOSIS proxy contact times and synthetic trajectories. The 2–10 % and "about 0.1 m" figures are reasoning from those inputs, not measured misclassification rates.
- The full PUBLIC_CPU suite (`tools/run_tests.py`) was not run.
- The author's mutant run v4 was incomplete at my end. Its pending results are not judged.
- The test file changed during the review (see the hash check). The full test runs cover the reviewed bytes only.
- My two plain test runs executed the reviewed test, whose envelope subprocess may write an ignored `.pyc` under the worktree's `tools/__pycache__/` (R3-m7). I did not delete or alter any worktree file.
- `sitl_training_geometry.py` was read only as context; the package bytes were reviewed in earlier rounds.
