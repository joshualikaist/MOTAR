# dev08 runtime stack (amendment2, README, step/attempt/runner deltas): independent review r4

Reviewer: independent reviewer, Claude Opus 5.5 (Agent model opus). Fresh context, read-only on every repository and
the PX4 tree. Date 2026-11-05, 19:31–19:45 KST. Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`,
branch `camonly/dev`, HEAD `9051cea`; the reviewed files are uncommitted (staged).

Snapshot: the 7 files of `SUBMITTED_SHA256SUMS` were copied to `review_r4/snapshot/`; 7/7 hashes OK at 19:31 and again
at 19:38, and the worktree bytes still equalled the snapshot at 19:38 (no moving target this round). The test files
`tests/test_sitl_dev08_{attempt,step}.py` and `tests/test_sitl_s3b_dev08_prefix.py` are byte-identical to the r3
snapshot (the two `.diff` files for them are empty).

## Verdict: **FAIL** (narrow: two MAJOR items, text plus three small code edits; no BLOCKER)

- Most of MJ1-r3 is repaired: (a), (c), (e)/link N8 and (f) hold against the code and the records.
- **MJ1-r4.** (b) is not reconciled honestly. The text relabels the 20 s outer `timeout -k` grace, which the parent
  plan and amendment1 fix, as the session's per-process SIGINT grace. Meanwhile l.6 still says no stop rule changes.
- **MJ2-r4.** The new termination sentences (l.217–221) are contradicted by the submitted code. Probes Q1, Q2/Q2b
  and Q3 show three windows:
  - a final.json that does not close the ledger;
  - no final.json at all, including after 42 recorded members;
  - an unrecorded member that ran but is not named.

  Each is a small code fix; once fixed, the text becomes true.
- With these two items and the MINOR text edits below done, I would expect PASS_WITH_LIMITS.
- No path produces a false COMPLETE: every window ends ABORTED, with no final.json, or with an unverifiable ledger.

## MJ1-r3 / link N8 repair table

| Item | Status | Evidence |
| --- | --- | --- |
| (a) bound vs approval | **REPAIRED** | l.132–133 "bounds the difference to the approval's `skip_max_s`"; l.166–168 "code default is 0.05 s … approval record's `skip_max_s`, accepted in 0.02–0.5 s, for both CLOCK_SKIP and RUNNER_LATE". Code: `SKIP_MAX_S = 0.05` (link :54); both checks use `self.skip_max_s` (link :406, :410); approval range check runner :404–406; forwarded runner :467 |
| (b) kill grace vs amendment1 | **NOT REPAIRED honestly** → MJ1-r4 | l.197–199 says amendment1's 20 s "is kept as each owned process's SIGINT grace inside the session". The parent prereg l.104–105 says "a 2400 s timeout per density and 5400 s overall, plus `timeout -k20`", and amendment1 l.150–151 carries it as "plus timeout kill grace 20s". The design review r1 l.90 reads it the same way. That value is the outer `timeout`'s kill grace; amendment2 changes it to 90 s (l.194) |
| (c) rl_games version | **REPAIRED** | l.111–113 "path, a hash over their .py files, and version where the package declares one (rl_games does not)"; runner :389 `getattr(mod, "__version__", None)` |
| (d) final.json exceptions | **PARTIAL** → MJ2-r4 | Repaired: the fault read now runs before `mkdir`; an unreadable kernel log or busy cores leave no directory (Q5, Q5b). `interrupted_member` is added (Q3b control names the member). Writes are masked (Q7: mask {1, 2, 15} inside, restored after, no-op off the main thread). Not repaired: the windows in MJ2-r4, which the new text says are closed |
| (e) truth-lag wording / link N8 | **REPAIRED** (one ambiguity, m4-b) | l.90–95, l.127–130, l.151–152 and README l.47–49, l.83–86 use the link review r3 N8 replacement text. The nominal per-phase offsets 0/−4/−8/−12/−16 ms, the receive-delay +4 ms flip in one phase, and "whole truth samples (20 ms)" match `dev08_link_20261105/review_r3/probe_px4_truth_r3.out` rows 1–60. The link module docstring (:25–35) was updated the same way |
| (f) README numbers without a record | **REPAIRED** (one new claim without a record, m4-d) | `dev07_records.json`: `dev07_70/ep02` compute_ms_max 218.4 at k_of_max 27, median 6.29. Param imports: 23 logs, 23 importing, 22 × (13, 16), and `dev07_smoke_70/ep00` (13, 6). Summary `ep02_70_k27_compute_ms` 218.4. It is bound by `make_validation.py`'s rglob over the external folder |
| (g) timeout margin vs setup before open | unchanged (NOTE n4-a) | l.195–196 |

Other numbers re-checked on this round's bytes:
- 463/500 diverge, 198 at k ≤ 2, k = 164, range 1–384: `tolerance_divergence.out` ✓.
- 3372 tests, 1 failure: `public_cpu_pre_fix.log` ✓.
- 34 files, 20 scenes, windows 208/600: `scene_specs_check.json` ✓.
- 599 and 63 commands: `real_bridge_identity.json` ✓.
- Seven step mutants killed, control not killed: `step_mutants/results.json` ✓.
- dev07 reports: steps 18–75 and RTF 0.9964–1.0037 ("18–75", "0.996–1.004") ✓.
- Assumption 7's "4–16 ms newer … 8 of 100" matches the gate stall rows at 30 and 45 ms ✓.

**amendment1 invariants.** Estimand, cohort, order and analysis are unchanged:
- `schedule()` is unchanged.
- `summarize_complete_pairs` is unchanged.
- `main_records` still requires COMPLETE, an unchanged fault count and no record problems.

The added stops put amendment1's existing abort conditions into practice (as r3 found). Two amendment1 runtime
bounds are changed, and l.6 does not say so:
- **Outer kill grace:** 20 s → 90 s.
- **Hard outer cap:** parent/amendment1 `timeout` at the 5400 s overall cap, + 20 s → `timeout 5700` + 90 s. The 2400 s
  and 5400 s budgets become between-member checks; l.210–211 states that part.

## Remaining text errors

| file:line | Current text | Correct statement |
| --- | --- | --- |
| amendment2:6 | "It changes no estimand, cohort, order, stop rule or analysis of amendment1." | "It changes no estimand, cohort, order or analysis of amendment1. Its added stops (HOST_NOT_IDLE, SOURCE_CHANGED, FAULT_CHECK_ERROR, non-finite record) implement amendment1's existing abort conditions. It changes two amendment1 runtime bounds, stated under *Proposed execution*: the outer timeout's kill grace (20 s → 90 s) and the outer timeout (5700 s; the 2400 s and 5400 s budgets are checked between members)." |
| amendment2:197–199 | "amendment1's "timeout kill grace 20 s" is kept as each owned process's SIGINT grace inside the session. The outer `timeout -k` is longer (90 s) so that this cleanup … ends before the runner is killed." | "The parent plan and amendment1 fix `timeout -k20`, a 20 s kill grace for the outer timeout. This amendment changes it to 90 s. With 20 s, `timeout` could SIGKILL the runner during the session cleanup (up to about 35 s), leaving owned processes and no final.json. Inside the session each owned process keeps a 20 s SIGINT-to-SIGKILL grace (`KILL_GRACE_S`). This is a changed runtime bound, not a reading of amendment1's." |
| amendment2:201–202 | "The runner's cores must be idle, at least 90 % over 1 s, when the attempt opens and before every member. Otherwise the unit stops (HOST_NOT_IDLE)." | "The runner's cores must be idle, at least 90 % in one of up to three consecutive 1 s samples (point samples, all recorded), when the attempt opens and before every member. Otherwise the attempt is refused at open, or the unit stops (HOST_NOT_IDLE). One further sample after each member is recorded and is not a stop." (code: `idle_check(tries=3)`, attempt :180–189; Q6) |
| amendment2:118 | "Each member records the core idleness measured before it." | "Each member records the core idleness measured before and after it." (runner :469–470 `host_idle_after`) |
| amendment2:217–221 | "These keep the attempt and write final.json listing the unrun members. A member interrupted … is named as `interrupted_member` … Ledger and final.json writes hold SIGTERM/SIGHUP/SIGINT until they are complete. … A SIGKILL of the runner leaves no final.json; the open leak marker and the journal remain." | Fix the code first (MJ2-r4). Then: "A ledger line with its chain update, and final.json with its fault read, are written with SIGTERM/SIGHUP/SIGINT held; a held signal is delivered afterwards. … A SIGKILL of the runner leaves no final.json; if a session was running, its open leak marker and journal remain." Without the code fix, the three windows of MJ2-r4 must be stated as limits instead |
| amendment2:129–130 | "In the +4 ms phase a receive delay can put the next pass's truth inside the gate's window." | "In the phase whose truth is 16 ms older than the position, a receive delay can put the next truth, sent 4 ms after the position, inside the gate's window." The cited probe labels phases by creation offset; there "+04" is the −8 ms phase and the affected one is "+16". The N8 text inherited this label clash (m4-b) |
| amendment2:158–160 | assumption 7 names only the newer-truth effect | Add: "A stall of 3 ms or more between a position and its same-pass truth makes the gate use the previous truth, 20 ms older, also silently." (link N12 asked for both effects; give no count, see n4-d) |
| README:70 | "`link_mutants/` … order-bound and settle mutants against the link tests: killed; unmutated control passes" | The folder holds only the 4 module files (control = link adcd73df) and 3.13 `.pyc` files. It has no results file. Add the run output (for example `link_mutants/results.json`), or drop "killed; unmutated control passes" |
| README:48–49 (optional) | "…4–16 ms before it, fixed per boot, and 4 ms after it under receive delay" | "…or 4–16 ms before it, fixed per boot; in one phase a receive delay can admit the truth sent 4 ms after it" |

## Code findings (the r4 deltas)

### MAJOR

**MJ2-r4. Termination recording does not do what amendment2 l.217–221 says** (probe `probes/probe_r4.py`, output
`probes/probe_r4.out`).
- **Q1: a signal during the ledger write.** The signal is held during the write, then delivered at unmask (inside
  `signals_held`'s `finally`), *before* `self.prev_sha`/`self.next` are updated (attempt :368–371).
  - Result: ledger 4 lines; final.json `recorded 3`, `unrun` starting with the recorded `03_main_70bars_env0_candidate`;
    `interrupted_member null`.
  - `read_ledger` → "final record does not close the ledger".
  - The hold cannot help here: without it, PEP 475 would finish the write anyway and the handler would raise at the
    same point. r3 m3-c asked for the hold around "the ledger write, the `prev_sha` update and the final write".
- **Q2 / Q2b: a signal during `_final`'s fault read.** `self.status` is set first (:396). The handler's
  `RunnerTerminated` is a BaseException, so `except Exception` (:399) misses it. The write is never reached, and the
  runner's later `abort` is a no-op.
  - After a MEMBER_NOT_VALID abort: **no final.json** (3 lines).
  - At COMPLETE: **no final.json after 42 recorded lines**.
  - `journalctl -k -b 0` takes about 0.01 s here (547 kB), so the window is short but real. Users with a slow journal
    get a longer one.
- **Q3: a signal during the post-member idle sample.** `current = None` (runner :468) runs before `rec.host_check()`
  (1–3 s per member), `_binding_core`, `git_state` and `append`.
  - Result: member 1 ran (`run` was called), but final.json has `interrupted_member null`; it is still in `unrun`.
  - That is about 42–126 s of exposure per unit.
  - Control Q3b, a signal inside `run()`, names the member correctly.

**Fix** (small; then the l.217–221 text is true):
- **Attempt `append`.** Hold signals over the write *and* the chain update:
  `with signals_held(): (write line; self.prev_sha = …; self.next += 1)`.
- **Attempt `_final`.** Put the whole body under `with signals_held():`, from `self.status = status` through the write.
  The child journalctl inherits the mask, which is harmless.
- **Runner.** Track the member, not its name: `current = member` before `run`. In `except`, use
  `interrupted_member=current["name"] if current is not None and rec.next == current["index"] else None`.
  This names exactly the member that ran but was not recorded, including one whose append was interrupted. Setting
  `current = None` after `append` would instead name a member that *was* recorded, if the held signal is delivered
  at the end of append.
- **Tests.** Add tests for Q1–Q3 (m4-e).

### MINOR

- **m4-e. No r4 delta is tested.** The three test files are byte-identical to r3. The following new behaviours have
  no test: `signals_held`, faults-before-mkdir, `idle_check(tries=3)` and its `samples`, `host_idle_after`,
  `interrupted_member`, and numpy non-finite handling. The 14 attempt and 10 runner tests still pass; they would not
  catch a reversal of any of these. My probes Q5–Q7 confirm the parts that work.
- **m4-f. Residual of m3-d** (Q4).
  - **Fixed.** np.float32/np.float64 NaN and inf are now flagged and nulled.
  - **Still silently stringified by `default=str`, with no record problem:**
    - finite np.float32 → `"0.5"`;
    - np.int64 → `"5"`;
    - np.bool_ → `"True"`;
    - an ndarray holding NaN → `"[ 1. nan]"`, unflagged.
  - Current producers emit Python types, so the risk is low. A `default` hook that records a problem (or converts
    numpy scalars with `.item()`) closes it.
  - `_final` uses `default=str` with `allow_nan=True`.
- **m4-g. Idle-sample timing.** The post-member sample runs after the session cleanup, so it cannot attribute load
  during a member (RUNNER_LATE). It measures the cores just after. Up to 6 s per member (3 + 3 tries) is not in the
  duration estimate (l.222–226). This is text-level only.

### NOTE

- **n4-a (r3 g).** The 5700 s outer timeout counts from process start. The 5400 s budget counts from `rec.open()`.
  The margin must also cover the setup before open (scene inputs, bridge, Policy F, warm-up).
- **n4-b.** `signals_held` blocks SIGINT, but the runner installs handlers only for SIGTERM and SIGHUP. SIGINT
  arrives as KeyboardInterrupt, which follows the same RUNNER_EXCEPTION path. That is consistent with the text.
- **n4-c.** r3 m3-b test gaps and the r2 step/attempt survivors are carried unchanged (test bytes unchanged).
- **n4-d.** There is a discrepancy in the link record, which belongs to that unit's owner.
  - The link review r3 text and the docstring of `debug_stall_phase12.py` give "8 of 100" for the previous-truth
    (−20 ms) effect at phase +12.
  - `probe_px4_truth_r3.out` rows 102–133 print `{-20.0: 92, 0.0: 8}` for those rows.
  - Do not quote a count for that effect in amendment2 until the link owner settles it.

## What I ran

All CPU-only: `PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES="" nice -n 10 taskset -c 6,7,12,13 timeout -k 10`, `-B`.
- No PX4, Gazebo or GPU was used.
- The only signals were SIGTERMs this probe process sent to itself.
- Nothing was written in a repository or the PX4 tree; only read-only git was used (`status`, `log`).

Runs and checks:
- **Snapshot.** 7/7 hashes OK (19:31, 19:38); the worktree equalled the snapshot at 19:38.
- **Unit tests** (worktree = snapshot bytes):
  - `test_sitl_dev08_attempt.py`: 14 OK (0.44 s), `unit_test_sitl_dev08_attempt.out`.
  - `test_sitl_s3b_dev08_prefix.py`: 10 OK (22.8 s), `unit_test_sitl_s3b_dev08_prefix.out`.
- **`probes/probe_r4.py`** → `probes/probe_r4.out`, using the real AttemptRecorder and execute() with the test
  fixture's fakes:
  - Q1: ledger-write signal;
  - Q2/Q2b: `_final` fault-read signal, after an abort and at COMPLETE;
  - Q3/Q3b: post-member sample signal against an in-run signal;
  - Q4: `_finite_json` numpy cases;
  - Q5/Q5b: open with an unreadable kernel log or busy cores;
  - Q6: idle_check retries and the 0.90 boundary;
  - Q7: signal mask in the main thread and off it.
- **`journalctl -k -b 0` duration**, ×3: 0.01 s.
- **Read-only checks.**
  - Parent prereg l.95–112 and amendment1 l.141–159.
  - Design review r1 l.80–100.
  - Link review r3 (verdict, N8, N12, probe rows 1–135).
  - Session code: `KILL_GRACE_S`, leak-marker lifecycle :516–548 and :806–819.
  - Link :1–80 and :282–411.
  - Runner :360–499, attempt (whole file).
  - The external records: `dev07_records.{py,json}`, `tolerance_divergence.out`, `scene_specs_check.json`,
    `real_bridge_identity.json`, `public_cpu_pre_fix.log`, `step_mutants/results.json`, `link_mutants/`,
    `make_validation.py`.
  - The dev07 s3b reports.
