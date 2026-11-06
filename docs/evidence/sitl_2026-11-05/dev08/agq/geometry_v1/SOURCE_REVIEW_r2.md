# AGQ geometry-v1 source review r2

**Verdict: FAIL.** Open findings: 0 BLOCKER, 2 MAJOR (both new), 8 MINOR (new or residual).

Of the 15 round-1 findings:
- 14 are resolved in the current bytes;
- M1 is resolved for the three named scenarios, but its replacement rules carry two residual items (N5, N6).

The two MAJORs are in the fixed reading, which freezes when the preregistration is committed:
1. **N1.** With real pyulog data the clock check can never pass. `--analyze` then crashes while writing its output.
2. **N2.** The training-box member's decision window is not covered by verified data, so missing ULog samples or a contact-log gap can produce `CONSISTENT_WITH_ROTOR_CONTACT`.

Both fixes are small.

- **Reviewer:** Claude Opus 5.5 (`claude-opus-5-5`), fresh context, independent read-only audit (round 2).
- **Date:** 2026-11-06, 02:41–03:00 +09:00.
- **Repository:** `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `0abb7664b1828dabd11bd2d45f770a8cafec7883`. The reviewed files are untracked.
- **Writes:** only under `/home/fair/motar_tmp/agq_geometry_diag_20261106/source_review_r2/`.
  - Nothing was written into any Git worktree.
  - No Git state was changed: only `git status`/`rev-parse` were run.
  - No `*env*.txt` under `~/motar_tmp/gainrand_20261104/` was read.
- **Simulator and GPU:** no gzserver, gz, px4, Gazebo or SITL process, and no GPU.

## Hash check (`sha256sum -c BYTES_UNDER_REVIEW.sha256`)

| File | SHA-256 | Start 02:41:20 | End 02:58:06 |
| --- | --- | --- | --- |
| tools/sitl_agq_geometry_diag.py | `c58cbe81…` | OK | OK |
| tools/sitl_agq_geometry_launch.py | `189e05ef…` | OK | OK |
| tests/test_sitl_agq_geometry_diag.py | `26e2e074…` | OK | OK |
| docs/plans/prereg_sitl_agq_geometry_v1_2026-11-06.md | `54f7b185…` | OK | OK |
| tools/sitl_training_geometry.py | `cadd5167…` | OK | OK |

## Round-1 findings

| ID | Status | Evidence in the current bytes |
| --- | --- | --- |
| **M1** box-favourable pair reading | **Resolved for (a)(b)(c); residual N5, N6** | **(a)** `pair_reading` now uses every box rate window overlapping `[t_c−0.5, t_c+1.5]` (diag:707-716). The native side accepts any onset in `[t_c, t_c+0.5]` (:679-680). **(b)** The box window extends to `t_c+1.5`. **(c)** The new exposure rule (:701-706) returns INCONCLUSIVE_NOT_EXPOSED for a box member kept away. Each round-1 P2 case re-run on the new code (`reading_probes_r2.json` Q7): 4.9 s only → CONSISTENT (legitimate, before the window); 6.3 s → DISTURBANCE_PERSISTS_WITHOUT_CONTACT; 6.3 s with contact at 6.25 s → CONTACT_PERSISTS; kept 0.3 m away → INCONCLUSIVE_NOT_EXPOSED. Tests :469-500 pin these with paired controls. Mutants killed (author v3): `window_1s`, `path_guard_off`, `exposure_off`, `box_window_short`, `box_first_rate_only`, `native_first_rate_only`. Residuals: the exposure reference (N6) and an excursion already in progress at `t_c` (N5). |
| **M2** clock anchor after the crash | **Resolved** (but see N1) | The anchor is now `floor.last_t_before_start` (diag:362-363, :833). The test at :121-126 shows a post-crash touchdown leaving the anchor at takeoff. Mutant `takeoff_anchor_last` is killed. Round-1 post-end probe re-run (`post_end_floor_rerun.txt`): env5 native is back at rest +0.008 s after its final checkpoint, which no longer matters. New old-ULog evidence (`liftoff_trace_probe.json`, 4 members): truth z is flat to the mm until liftoff, and the 1 cm rise follows the first mm by 0.10–0.12 s. A clock offset under about +0.18/−0.17 s therefore passes `[−0.05, +0.3]`, and the window is adequate. The rule itself is broken in practice by N1. |
| **M3** invalidated members read | **Resolved; residual N3** | `void` for VOID_HOST_FAULT or FAULT_CHECK_ERROR (diag:796). `timing_problems` now gates evaluability (:806). Tests :683-693 cover TIMING_GUARD and VOID with paired controls; mutants `void_ignored` and `timing_not_gating` are killed. Residual N3: a fault rise found only at the final read is not voided. |
| **M4** affinity/nice dependence | **Resolved** | The recorder fixture passes `cores=frozenset(os.sched_getaffinity(0)), min_nice=-20` (test:576). **22/22 OK**: pinned (16–19, nice 10) in 52.6 s, and unpinned (affinity 0-3,6-23, nice 0) in 51.3 s. The full PUBLIC_CPU suite was not run (limits). |
| m1 control cost/side effects | Resolved | `tap.parameters` is kept before the control (diag:506). `t_entry`/`t_return` are recorded before a raise (:507-511). The preregistration states the ~0.1 s delay (prereg:95-97). Tests :268-283. |
| m2 control freshness | Resolved | The last message must be within 0.5 s of the newest LOCAL_POSITION_NED (diag:549-553). Test :435-457 is paired (0.2 s pass, 0.6 s fail, none fail). Mutant `freshness_off` is killed. |
| m3 boot gaps stop the unit | Resolved | Continuity is gated only on `[start−1, final+0.5]` (diag:289-294, :383-388). Test :128-133 is paired. Mutant `gaps_counted_everywhere` is killed. N2 is a new consequence: the reading window is wider than this. |
| m4 raw log unbound | Resolved; residual N9 | `contact_files` stores the SHA-256 of `contacts.log` and `contacts.jsonl` in the ledger (diag:1020-1027). `analyze` refuses a changed raw log (:809-811), and a recomputed problem makes the member not evaluable (:819-820). Test :664-666. Residual N9: other read files are not verified. |
| m5 cross-clock label | Resolved in logic, broken by N1 | Both members are now required, and NOT_READ_CLOCK_UNVERIFIED keeps a `provisional_reading` (diag:842-845). Mutant `clock_not_withheld` was pending in the author's run. |
| m6 unreadable ULog crashes | Resolved | The exception is caught: NOT_RECORDED, then NOT_EVALUABLE (diag:821-826). Test :661-663. A readable but incomplete ULog is not covered (N2). |
| m7 command/response pairing | Resolved | `velocities_at` reports the truth at k, the command in effect from k−1 and the new command at k (diag:720-741). Test :533-544. |
| m8 limits only in the launcher | Resolved; test gap in N8 | `envelope_problems` checks the fixed `attempt01`, RLIMIT_AS ≤ 16 GiB and parent == `timeout` (diag:1061-1077). It is called before the approval is read (:1107-1109). My probe (`envelope_chain_probe.txt`) shows `nice`/`taskset` exec in place under `timeout`: the chain is python ← timeout, and `envelope_problems(ATTEMPT_DIR) == []`. The launcher's real envelope is accepted. Without `timeout` the parent is `bash` and the check refuses. |
| m9 narrowings / stdbuf unbound | Mostly resolved | Section 9 lists the MAVLink, impulse and velocity narrowings. `/usr/bin/stdbuf` and `/usr/lib/x86_64-linux-gnu/coreutils/libstdbuf.so` are hashed into the binding and the core comparison (diag:59-60, :897-900). The libdir matches stdbuf's compiled-in `/usr/lib/x86_64-linux-gnu/coreutils`, and `stdbuf` is the only one on PATH. Residual: Rate5 on any axis, against the diagnosis's yaw-only rule, is defined in §6 but not listed as a difference in §9. |
| m10 ODE claim | Resolved | Prereg §8 cites the `ODEPhysics` disabled-body bypass. |
| m11 kill margin | Resolved | `-k 120` (launch:24-27). My estimate of the worst case: watchdog join ≤ 35 s + second terminate ≤ ~30 s + journalctl ≤ 30 s ≈ 95 s, which is inside 120 s. See N7 for the 1200 s side. |

## New findings

### MAJOR

**N1. With real pyulog data, `takeoff_check` returns `numpy.bool_`. `analyze()` then never reports SUPPORTED, and `--analyze` crashes in `json.dump`.**

Where:
- `tools/sitl_agq_geometry_diag.py:774-776`: `ulog_series` builds times as `t * 1e-6` from numpy `uint64`, which gives `numpy.float64`;
- `:757-758`: `ok = window[0] <= d <= window[1]` is a `numpy.bool_`;
- `:842`: `... ["ok"] is True`;
- `:1094`: `json.dump(result, ..., allow_nan=False)` has no default.

Failure scenario. Two probes confirm it.
- `numpy_bool_probe.json` (px4 env, real dev08 env5 ULog, real `ulog_series`). With the anchor at the observed liftoff, the rise follows 0.12 s later:
  - `ok` is `numpy.bool_(True)`;
  - `ok is True` is `False`;
  - the analyze clock rule gives `LIMITED_VALIDITY`;
  - `json.dumps` raises `TypeError: Object of type bool_ is not JSON serializable`.
- `reading_probes_r2.json` Q1 (the real `execute()` and `analyze()` on the test fixture). The same attempt reads:
  - with Python-float series: SUPPORTED for both pairs, and the JSON is fine;
  - with `numpy.float64` series: both pairs `NOT_READ_CLOCK_UNVERIFIED`/`LIMITED_VALIDITY`, and `main()`'s `json.dump` raises.

Consequences in the real run:
- every evaluable pair is withheld, so a valid case is rejected by construction;
- the prescribed `--analyze ... --out` exits with a traceback, leaving the `x`-opened output file empty or partial;
- fixing the code after the run would change a bound source (OWN_SOURCES) after measurement.

The tests use Python floats only, so they cannot catch this. My mutant `takeoff_bool_cast` adds `bool(...)` and still passes every test.

Fix:
- cast in `ulog_series`: `float(t) * 1e-6` for both series;
- also `bool(...)` the `ok` value;
- add a test that feeds `numpy.float64`/`numpy.uint64` series through `analyze()` and `json.dump(..., allow_nan=False)`, or a small real ULog fixture.

**N2. The box member's decision window `[t_c−0.5, t_c+1.5]` is not covered by verified data. Missing data reads as "no disturbance" or "no contact".**

Where:
- `tools/sitl_agq_geometry_diag.py:644-646`: rate windows are built from whatever samples exist;
- `:707-716`: the CONSISTENT branch;
- `:289-294` and `:383-388`: contact continuity is only required on `[start−1, final+0.5]`;
- `:699`: the box only has to last until `t_c+0.5`;
- preregistration :94 ("Silence never counts as no contact"), :101-102 and :179.

Failure scenarios (`reading_probes_r2.json` Q2, Q3):
- **ULog coverage.**
  - Neither member's ULog is checked for sample coverage or `dropouts` in the window.
  - A box ULog with complete samples and an excursion at elapsed 6.5 reads DISTURBANCE_PERSISTS_WITHOUT_CONTACT.
  - The same series, ending or dropping out at elapsed 6.0, reads **CONSISTENT_WITH_ROTOR_CONTACT**, with `rate_source` still `ULog`.
  - In the old ULogs the log ends 0.99–1.01 s after the final checkpoint. A box member that ends near `t_c+0.5` is therefore covered only to about `t_c+1.49`.
- **Contact continuity.**
  - When the box member's final is in `[t_c+0.5, t_c+1.0)`, up to 0.5 s of its window lies after `final+0.5`. Gaps there are "reported only".
  - Q3: a 0.3 s gap at 16.9–17.2 s (final 16.3) hides a bar contact and gives `problems == []`, `gaps == 1`.
  - The pair would read CONSISTENT instead of INCONCLUSIVE.
  - That part of the window is also the runner's after-phase, with the AUTO mode switch and no policy commands.

This is the hypothesis-favourable direction. It contradicts the preregistration's own §4 sentence and AGENTS.md "Never fabricate a missing metric". It is unlikely on an idle host: the old ULogs have no dropouts and a maximum sample gap of 16 ms. But the outcome is by construction.

Fix, before commit:
1. **ULog coverage.** For N over `[t_c, t_c+0.5]` and for X over its window, require: the first sample at or before `lo`, the last at or after `hi`, a maximum gap of at most `RATE_JOIN_S`, and no pyulog `dropouts` overlapping. Otherwise NOT_EVALUABLE (rates incomplete).
2. **Contact continuity.** Either require it through `t_c+1.5` for the box member, or clip `hi` to `min(t_c+1.5, X.final+0.5)` and state that clipping.
3. Add paired tests.

### MINOR

**N3. A fault rise found only by the final read, or a final fault-read error, does not void the attempt in `analyze()`.**

Where: diag:796; prereg :157; AGENTS.md "Host faults void a run".

`reading_probes_r2.json` Q6: the fault count is 0 for the open and the four appends, then 1 at the final read. The result:
- final `COMPLETE`, `kernel_faults_end 1 != start 0`;
- `void false`;
- all four members evaluable;
- both pairs read.

`final.fault_check_error` is also ignored, and so is the launcher's `run01` `kernel_faults_after`. The realistic path is a RUNNER_EXCEPTION abort (signal or runner error) that coincides with a fault. The launcher's exit code flags it, but the reading does not.

Fix: also void when `final.kernel_faults_end != final.kernel_faults_start` or `final.fault_check_error` is set. Optionally also read `run01/execution_process_record.json`. Add the test.

**N4. In the not-reproduced branch, DISTURBANCE_WITHOUT_CONTACT counts any later onset, including after-phase and floor-impact excursions.**

Where: diag:681-688.

`q4_post_episode_probe.json`:
- N has a rotor contact at 5.73–5.80, an onset at 6.3 and final 7.0. Alone, that reads INCONCLUSIVE_NOT_REPRODUCED.
- Adding an onset at 7.6, after the final checkpoint (for example the floor impact of an altitude crash; the old env5 native is back on the floor +0.008 s after its final), gives **DISTURBANCE_WITHOUT_CONTACT, onset 7.6**.

The label then wrongly keeps the controller/plant hypothesis open. This bias is against the rotor hypothesis, but it is still a wrong reading.

Fix: consider only disturbances at or before N's final checkpoint, and before N's first post-start floor contact. Preferably evaluate the earliest disturbance only.

**N5. "Reproduced" accepts a native excursion that is already in progress at `t_c`.**

Where: diag:679-680.

`reading_probes_r2.json` Q5:
- N's rate windows are `[5.68, 5.72]` and `[5.78, 5.90]`, with rotor contact at 5.73;
- the second onset at 5.78 counts as reproduction;
- an inactive box member then gives **CONSISTENT_WITH_ROTOR_CONTACT**, although N's departure began before the contact.

The window splits whenever the rate dips below 5 rad/s for more than 30 ms. The old env2 native has split windows ([1.0, 1.152], [1.28, 1.54]).

Fix: require that no native rate window contains `[t_c − 0.04, t_c]`. Otherwise read INCONCLUSIVE_NOT_REPRODUCED.

**N6. The exposure reference includes post-contact native samples. Its +0.02 m tolerance is in the box member's favour, and it accepts any bar.**

Where: diag:701-706, :81; prereg :176-178.

On the old env5 native trace (`old_clearance_probe.json`):
- clearance 0.2755 m at `t_c−0.024`;
- 0.1674 m at `t_c+0.072`, which is post-contact, carried by momentum;
- interpolated at `t_c`: ≈ 0.249 m, close to DIAGNOSIS's 0.239 m.

With the rule as written, X must pass within 0.187 m, closer than the base box's own half-diagonal of 0.198 m. A box member that passes within the native rotor reach (for example 0.22 m; level reach 0.312 m) reads INCONCLUSIVE_NOT_EXPOSED, so a valid case is rejected.

Conversely:
- when N's sampled minimum is the pre-contact one, the +0.02 m tolerance lets X be farther than N was;
- X's ±0.5 s window against N's ±0.1 s, and "nearest any bar", make exposure easier for X.

The test does not pin the native ±0.1 s window: my mutant `exposure_native_window_0p5` survives.

Fix: use N's clearance at `t_c`, interpolated between the adjacent checkpoints or computed from the 50 Hz ULog truth against the bar boxes, and require X's minimum to be at or below it with no favourable tolerance. Optionally require the same bar, or state "any bar" as a limit.

**N7. The 900 s budget does not leave room for the claimed worst case inside 1200 s.**

Where: diag:57 (comment); prereg :233-237.

- 900 s + 240 s watchdog + the preregistration's own ~95 s worst-case cleanup = 1235 s, above 1200 s.
- The budget runs from `rec.open()`. The dev08 setup before open was 3.4 s (`execution_process_record.json` started_at against `attempt.json` wall_start).
- The post-member contact audit of a 240 s log also costs time, about 20–25 s at round 1's measured 0.17 s/MiB.
- If it happens, the outer SIGTERM turns it into a recorded RUNNER_EXCEPTION, which is fail-closed, and the 120 s kill margin still covers cleanup.
- In practice it is unreachable: members take 45–51 s (dev08), and a member that is not VALID stops the unit.

Fix: lower `BUDGET_S["overall"]` to 800 s, or correct the two statements to say the outer timeout may end the last member as RUNNER_EXCEPTION.

**N8. Test gaps where a guard has no effective negative control** (my mutants, `mutants/results.json`):
- **`envelope_gate_off_in_main` survives.**
  - test:781-783 calls `main(["--execute", ...])` with a missing approval file. That returns 1 whether or not the envelope gate exists.
  - Expected for the author's pending `envelope_off` mutant: survives, since it is the identical change.
  - Fix: write a valid approval JSON in tmp, and assert that the printed error contains `envelope:` and that no attempt directory exists.
- **`stdbuf_missing_not_refused` survives.** There is no test in which a missing stdbuf or libstdbuf refuses (diag:897-900).
- **`exposure_native_window_0p5` survives** (N6).
- Also untested:
  - the launcher refusing an existing `attempt01`/`run01`;
  - numpy-typed series (N1);
  - ULog coverage (N2);
  - a fault rise at the final read (N3).

**N9. `analyze()` verifies only the raw contact log.** `checkpoints.jsonl` (paths, clearances, Z0.8, Err0.5), `recording/snapshots.jsonl` (velocities) and the ULog (rates, clock check) are read without a hash check (diag:815-816, :822).
- `snapshots.jsonl` has a SHA-256 in the recording manifest, but `analyze()` does not compare it.
- `checkpoints.jsonl` and the ULog are bound nowhere.

Fix: hash `checkpoints.jsonl` and the ULog into the ledger record after `run_episode` returns, and verify all of them in `analyze()`.

**N10. Preregistration statements that are false or unsupported:**
- :94 "Silence never counts as no contact": false for the part of the box window after `final+0.5`, and for ULog rates (N2).
- :234-235 "the 1200 s outer timeout leaves room for one member's 240 s watchdog and its cleanup": not true for the stated worst case (N7).
- :77-78 "Tests show the Inbox contents and the gate's alignment and checkpoint output are identical": supported only for one synthetic stream and checkpoint 0 (test:240-257). Say so.
- §9 does not list "Rate5 = any body axis" (the diagnosis used yaw only) as a reading choice. That is m9's residual.
- :52-53 "60 identical runtime files" is a dry-check result from before the stdbuf binding. The live core now also has the two stdbuf files. This is a nit; say "60 at the dry check".

### Notes (no change required)
- `clearance_m` is `box_hit`'s horizontal distance from the vehicle centre to the nearest axis-aligned bar box (`sitl_s3b_closed_loop.py:166-173`), sampled at 10 Hz. At about 2 m/s the sampled minimum can exceed the true minimum by about 0.02 m. This bounds every exposure comparison.
- `checkpoints.jsonl` is written with `default=float` and may hold `Infinity` (box_hit returns `inf` when no bar spans the vehicle's height). `main()`'s `allow_nan=False` would then raise. This is unlikely at 1 m flight height.
- The stdbuf binding hashes fixed paths, not the PATH-resolved executable. That is correct on this host today (`command -v stdbuf` gives `/usr/bin/stdbuf`; no conda environment has one).
- Author's mutant kill check `mutants_v3/` (still running at 02:58): control OK; 26 of 30 mutants killed so far. `hash_unchecked`, `clock_not_withheld`, `envelope_off`, `order_swapped` and `skip_not_checked` were pending. I did not re-run them, except my equivalent of `envelope_off`, which survives (N8).

## Check of the requested guards

| Guard | Result |
| --- | --- |
| Control freshness | Correct and paired-tested. A stall is caught (log behind PX4 by more than 0.5 s). It cannot pass on PX4 silence (`now is None` fails). |
| Gap window | `[start−1, final+0.5]`, as the preregistration says. Paired tests. Narrower than the reading window (N2). |
| Takeoff anchor | Correct (`last_t_before_start`). The window is adequate on old ULogs (rise 0.10–0.12 s after liftoff). Broken by N1's type defect. |
| Contact-file SHA-256 | Bound in the ledger and verified in `analyze()`. Paired test. Other read files unverified (N9). |
| stdbuf binding | Both files hashed into the binding and the cross-member core. Matches the compiled-in libdir. No negative test (N8). |
| `envelope_problems` | It does not wrongly refuse the real envelope: `nice` and `taskset` exec in place, so the parent is `timeout`; RLIMIT 16 GiB equals the maximum, and the comparison is `>`; the fixed absolute `attempt01` path matches. It refuses relative or other directories and a non-`timeout` parent. The `main()` gate itself is not effectively tested (N8). |
| 900 / 1200 / 120 s | The 120 s kill margin covers the ~95 s worst-case cleanup. The 900 s budget plus 240 s plus 95 s exceeds 1200 s; this is fail-closed and practically unreachable (N7). |

## Tests

- Paired positive and negative controls exist for:
  - the parser and audit (in-window against boot gaps, coverage, order);
  - the floor control (125 against 124, non-floor, gap, short, foreign line, tail);
  - freshness;
  - each reading label;
  - the takeoff anchor;
  - velocities;
  - the approval gate;
  - the stop rules (RECORDER_LOSS, MEMBER_NOT_VALID, TIMING_GUARD, VOID, SOURCE_CHANGED, signal, BINDING_CHANGED ×2);
  - the recorder problems.
- Gaps are listed in N8.
- Portability: **22/22 OK pinned (cores 16–19, nice 10, 52.6 s), and 22/22 OK unpinned (affinity 0-3,6-23, nice 0, 51.3 s)**, both under `ulimit -v 8388608`.
- The envelope subprocess test is robust to an inherited hard RLIMIT below 16 GiB (test:792-794).

## Commands run

All Python ran under `CUDA_VISIBLE_DEVICES="" PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 timeout -k 10 900 nice -n 10 taskset -c 16,17,18,19 bash -c 'ulimit -v 8388608; <python> -B …'`. The one exception is the unpinned test run, which had no `nice` or `taskset`. The aerialgym Python was used unless "px4 env" is noted.

1. `sha256sum -c BYTES_UNDER_REVIEW.sha256`, at start and end.
2. `python -B tests/test_sitl_agq_geometry_diag.py -v`: `focused_tests_pinned.log`, and unpinned `focused_tests_unpinned.log`.
3. `scripts/envelope_chain_probe.py`: the process chain under `timeout … nice … taskset … python` and the real `envelope_problems` → `envelope_chain_probe.txt`.
4. `scripts/old_clearance_probe.py`: dev08 clearances and path differences around `t_c` → `old_clearance_probe.json`.
5. `scripts/old_ulog_probe.py` (px4 env): the real `ulog_series` on three dev08 ULogs, covering span, dropouts, sample gaps, rate windows and the ULog end after final → `old_ulog_probe.json`.
6. `scripts/liftoff_trace_probe.py` (px4 env): truth z around liftoff and the 1 cm lag → `liftoff_trace_probe.json`.
7. `scripts/numpy_bool_probe.py` (px4 env): N1 on real ULog data → `numpy_bool_probe.json`.
8. `scripts/reading_probes_r2.py`: Q1–Q7 with the test file's fake fixtures and the real `execute()`/`analyze()` → `reading_probes_r2.json`. `scripts/q4_post_episode_probe.py` → `q4_post_episode_probe.json`.
9. Round 1's `scripts/post_end_floor.py` (px4 env) re-run → `post_end_floor_rerun.txt`.
10. `scripts/mutants_r2.py`: six targeted mutants, run on copies in `mutants/` → `mutants/results.json`.
11. Read-only reads of the dev08 `attempt01` ledger and receipts, and `dev08_execute_20261105/root/execution_process_record.json`.

## Limits

- No live Gazebo or PX4 run. The subscriber's real output, queue behaviour and floor-contact density, and the real liftoff-versus-contact timing, are NOT_VERIFIED. The takeoff-window adequacy is inferred from truth z in old ULogs, without contact data.
- N1 was confirmed on real ULog bytes for `ulog_series`/`takeoff_check`, and end to end only with numpy-typed synthetic series in the fixture attempt. The full `--analyze` CLI was not run on a real attempt, because none exists.
- The full PUBLIC_CPU suite (`tools/run_tests.py`) was not run. Only the focused file was run, pinned and unpinned.
- The author's mutant run was incomplete at the end of this review. Its pending results are not judged, except `envelope_off` by an equivalent mutant.
- The sampling-error and exposure arguments (N6) use one old trace (env5 native) and the stated geometry. They are reasoning, not a measured rate of misclassification.
- The cores 6, 7, 12 and 13 were not used.
