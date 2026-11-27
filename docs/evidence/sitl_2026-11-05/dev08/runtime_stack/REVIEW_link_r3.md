# REVIEW r3: dev08 parameter/telemetry boundary (`tools/sitl_dev08_link.py`, `tests/test_sitl_dev08_link.py`, truth-model wording)

- **Reviewer:** independent reviewer, Claude Opus 5.5 (Agent model opus). Fresh context, read-only.
- **Date:** 2026-11-05. Round 3. Earlier rounds: `../review_r1/REVIEW_r1.md`, `../review_r2/REVIEW_r2.md`.
- **Worktree:** `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch camonly/dev, HEAD 9051cea. All four
  files are untracked.
- **Reviewed bytes.** The four files were copied to `snapshot/` (relative paths kept) and match
  `SUBMITTED_SHA256SUMS`:
  - `tools/sitl_dev08_link.py` aa2551a7…
  - `tests/test_sitl_dev08_link.py` db7a04e5…
  - amendment2 9ecd89f0…
  - runtime_stack `README.md` e08ce40a…
- **The worktree changed during the review.** Another session edited three of the four files, so their hashes now
  differ from the submitted ones; only `tools/sitl_dev08_link.py` is unchanged.
  - Current hashes: the test file b409bf1a…, amendment2 10866de9…, the README 469caf63….
  - One later test edit is relevant to N9: it widens `burst_s` to 0.02 in the real-clock test.
  - None of the later edits touch the truth-model sentences of N8.
  - This review covers the snapshot only. I did not review the later edits.
- **PX4:** `/home/fair/workspaces/px4_ws/PX4-Autopilot` at v1.14.3 (1dacb4c). I re-read
  `mavlink_stream.cpp:47-130`, `mavlink_main.cpp:1142-1183, 1269-1334, 2159-2169, 2207-2219, 2402-2403, 1446-1451`,
  `streams/HIL_STATE_QUATERNION.hpp:60-105`, `streams/LOCAL_POSITION_NED.hpp:57-80` and
  `ekf2/EKF/imu_down_sampler.cpp`. They agree with round 2's assumption table, A1–A20.

## Verdict: PASS_WITH_LIMITS, on two conditions

**Code.** The code changes are correct, and I found no new code defect.
- BURST_S = 0.003 is below one 4 ms PX4 pass.
- The new order bound ("arriving before the next position message", `:335-336`) is right: it only removes
  candidates that cannot belong to the selected position's pass.
- At real-time factor (RTF) about 1, none of the 10 stream phases fails. This holds without jitter (200 checkpoints
  each) and in 150 jittered runs (600 checkpoints per row). There were zero errors.
- With the 3 ms bound, the gate now selects the same truth sample as the historical runner in three of the five
  phases, every time.
- Round-2 survivors S1, S3, S4, T3, T3c, Q1, Q3, F5, P2 and R4 are all killed.

**Conditions.** Both are about description or tests, not the gate's behaviour.

1. **N8 (MAJOR, wording).** N1's core is repaired.
   - The text no longer claims same-burst or same-cycle pairing.
   - The false claim "a violation shows as STALE_TRUTH" is gone.
   - The documents now say that the receipt age is bounded and the PX4-time offset is not.

   Three quantitative statements are still wrong for the submitted gate:
   - **"−12 to +4 ms".** This is round 2's result for the old 5 ms bound. With 3 ms the range is −16 to 0 ms, and
     +4 ms only under receive delay.
   - **"An adjacent 4 ms PX4 cycle".** In three phases out of five the truth used is 8, 12 or 16 ms older than the
     position's pass.
   - **"They can differ by one PX4 cycle"** (the gate and the historical runner). Distinct truth samples are 20 ms
     apart. So whenever the two choose differently, they differ by one whole 20 ms sample.

   These sentences are in amendment2, which becomes immutable when it is committed. Fix them first.
2. **N9 (MAJOR, test robustness).** The new real-clock test `test_same_burst_truth_is_awaited` failed the
   repository command on my first run. Another session's `tools/run_tests.py` was running on the same four cores at
   the time.
   - It relies on a 1 ms wall-clock margin: a 2 ms `sleep` against the 3 ms window.
   - It is the only test that kills S1, S3, S4 and S5.
   - Replace it with a fake-clock version, or widen its bound as the later worktree edit does. In either case keep a
     fake-clock test that kills S1–S5.

There are also three MINOR findings (N10–N12) and some notes.

## Repair table (round-2 findings)

| # | R2 finding | Status | Evidence |
| --- | --- | --- | --- |
| N1 | MAJOR: "same burst / same PX4 cycle" contradicted; "violation shows as STALE_TRUTH" false; 5 ms admits the next pass | **Core REPAIRED; numbers wrong (N8)** | Module `:25-29`, amendment2 l.84-88, l.111-114 and l.132-134, README l.47-49 and l.76-78 drop the pairing claim and the STALE_TRUTH claim, and say the receipt age is bounded, not the PX4 offset. Amendment2 assumption 3 asks the smoke to report the skew distribution. The optional code change was made (BURST_S 0.003, `:55`) and is pinned at 2 ms (inside) and 4 ms (outside) by `tests:419-424`. Residual errors are listed in N8. The `_settle` docstring `:344-345` still says "same-burst" |
| N2 | MINOR: burst wait and BURST_S unpinned | **Mostly REPAIRED** | KILLED: S1, S3, S4, S5 (only by the flaky real-clock test, N9) and T3/T3c/T3d/T3e/T3h/T3i/T4. BURST_S is pinned to (0.002, 0.004]: T3f (0.0039) and T3g (0.0021) survive, which is consistent with "below one 4 ms pass". **S2 (no settle in `align`) still survives** (N11) |
| N3 | MINOR: NaN truth quaternion untested | **REPAIRED** | `tests:470-473`; Q1 and Q3 KILLED |
| N4 | MINOR: rx batching admits later truths | **PARTIAL** (N12) | The order bound `:335-336` caps a batched truth below the next position. In the probe, a 45 ms stall gives +8/+12/+16/0/+4 ms where the mutant O1 gives +28/+12/+16/+20/+24 ms. At 30–45 ms stalls the gate still silently takes truths 4–16 ms newer than the position in 8 of 100 checkpoints. The bound is **untested** (O1, O3 and O6 survive, N10). Receiver promptness is named in the module (`:29`) but is **not** among amendment2's live assumptions |
| N5 | MINOR: F5, P2, R4 unpinned | **REPAIRED** | `tests:180-188` (one-ulp booted value, last line wins) and `tests:190-198` (failure 0.1 s into a 2 s wait gives RX_ERROR in under 1 s); F5, P2 and R4 KILLED |
| N6 | NOTE: diagnostics | **REPAIRED** | `inbox_backlog` was removed and `newest_pos_t` added. `positions_near` keeps the last 20 (`:352-356`). Probe Q3: RUNNER_LATE now lists 1.12–1.50 with the newest at 1.50, and CLOCK_STALL lists up to the newest. Not pinned by tests (D2–D6 survive), which is acceptable for diagnostics |
| N7 | NOTES | **Unchanged** (acceptable) | Malformed message is still a plain KeyError, not latched (`:304`, probe Q1). A second `align` still reports ALIGN_TWICE (probe Q2). The `SKIP_MAX_S` comment `:52` still says "≤ 0.02 s late" (dev07: 0.024, at most 0.040). The RTF floor is still not recorded in amendment2 (measured below: about 0.54). The equivalent mutants are unchanged (T8, C3, and now O2) |

## Truth model against PX4 scheduling and the code (task 1)

### Probe (`probe_px4_truth_r3.py`, `.out`, `.json`; `probe_rtf_r3.py`, `.out`)

The probe is round 2's PX4-shaped generator, unchanged, pointed at the snapshot. It follows MavlinkStream::update on
the 4 ms lockstep grid, uses real pymavlink encode/decode, and sets time_usec = 0. Additions in round 3:
- All 10 truth-stream creation phases.
- Two receive-jitter models:
  - **iid**, round 2's: an independent delay per message, then a sort;
  - **fifo**: one receiver thread, order kept, head-of-line blocking.
- An emulation of the historical selection (`sitl_s3b_closed_loop.py:265-276`): poll every 2.08 ms, then read the
  latest received truth.
- Mutant O1 next to the gate in the rx-stall rows.
- Offsets are reported in PX4 pass time: (truth's send pass) − (position's send pass).

**Nominal (RTF 1, no receive delay).** The truth used is fixed per boot; the 10 phases give 5 distinct values.

| Phase | Truth used, relative to the position's pass | Gate vs historical |
| --- | --- | --- |
| +12 | **0 ms**, the same pass | equal in 195 of 200; in 5 the historical poll fell between the position and its truth, so it took the previous truth (+20 ms difference) |
| +08 | −4 ms, the adjacent earlier pass | equal in 200 of 200 |
| +04 | −8 ms | equal in 200 of 200 |
| +00 | −12 ms | equal in 200 of 200 |
| +16 | **−16 ms**; the next truth is +4 ms and is now outside 3 ms | equal in 200 of 200 |

So, in answer to the question how often an adjacent-cycle truth is chosen now:
- **Nominal:** an adjacent pass (−4 ms) in one phase in five. The position's own pass in one in five. Two to four
  passes earlier (−8, −12 or −16 ms) in three in five. A later pass never.
- **Under receive delay.** In the +16 phase, a delay of about 1 ms or more on the position admits the +4 ms truth,
  so the choice flips within an episode by one whole truth sample (20 ms):

  | Maximum receive delay | +4 ms truth chosen (iid and fifo identical) |
  | --- | --- |
  | 0.5 ms | 0 of 600 |
  | 1.0 ms | 0 of 600 |
  | 1.5 ms | 39 of 600 (6.5 %) |
  | 2.0 ms | 84 of 600 (14 %) |
  | 3.0 ms | 152 of 600 (25 %) |

  The other phases do not change.
- **Gate against historical in the same-pass phase.** The two differ by 20 ms in 22–117 of 600 checkpoints (iid)
  and 39–187 of 600 (fifo), rising with delay. The gate consistently uses the same-pass truth; the historical poll
  sometimes misses it.

**STALE_TRUTH at RTF about 1: none.**
- 10 phases × 200 checkpoints and 150 jittered runs of 200 checkpoints, all served, zero errors.
- The worst receipt age is 16 ms (phase +16) of the 30 ms allowed.

**RTF floor** (`probe_rtf_r3.out`):
- At RTF 0.53 and below, the −16 ms phase fails at alignment with STALE_TRUTH, because 16 ms / 0.53 > 30 ms. The
  other four phases pass down to RTF 0.4. This is the same floor as round 2.
- The main probe's apparent failures at "RTF 0.6/0.5/0.4 phase +12" are artifacts. Its pre-alignment feed cut fell
  between a position and its same-pass truth; a live 3 ms settle would include that truth. The corrected re-run in
  `probe_rtf_r3.out` serves 100/100 in that phase at every RTF.

**rx stalls.** A stall that begins between a position and its same-pass truth stamps the truth more than 3 ms late.
The gate then silently uses the previous truth, 20 ms older ("stall 0.005/0.015/0.030/0.045 phase+12": 8 of 100).
With 5 ms this needed a 5 ms stall; with 3 ms a 3 ms stall is enough. For later truths, see N4 in the repair table.

### Wording, statement by statement

| Place | Statement | Accurate? |
| --- | --- | --- |
| Module `:20-24` | Selection rules: age, BURST_S, before the next position, new | **Yes**, matches `:331-337, :414-415` |
| Module `:25-26` | "each MAVLink stream keeps its own schedule, so the truth-to-position phase is fixed per boot" | **Yes** for send passes (`mavlink_stream.cpp:108-116`; `configure_stream` creates HSQ at its own time, `:1173-1176`). The position's `time_boot_ms` is the EKF publication time, which can precede its send pass, so the PX4-sample offset can vary within an episode. That is consistent with "the PX4-time offset is not [bounded]" |
| Module `:27` | "(review r2 probe: -12 to +4 ms in 4 ms PX4 cycles)" | **No, stale.** That was the 5 ms bound. With 3 ms: −16 to 0 ms, and +4 ms only under receive delay (N8) |
| Module `:27-28` | "A truth from an adjacent cycle can be selected without an error" | **Understated.** Up to four passes (16 ms) earlier, in 3 of 5 phases (N8) |
| Module `:28-29` | receipt age bounded and recorded; PX4 offset not; RTF about 1 and prompt receiver assumed | **Yes** |
| Module `:344-345` | `_settle`: "so a same-burst truth sample is in" | Leftover term ("same burst" is defined nowhere now). Use "a truth sent in the position's PX4 pass" |
| Module `:378-379` | `truth_time_model` record string | Omits the order bound (NOTE) |
| Tests `:317-322`, `:251`, `:468` | "same burst" names and comments | Describe the synthetic fixture, which really is one burst. Acceptable; the docstring at `:318` was corrected |
| Amendment2 l.83-85 | the window and "before the next position message" | **Yes** |
| Amendment2 l.85-88 | "fixed per boot; review r2 measured -12 to +4 ms … an adjacent 4 ms PX4 cycle can therefore be selected" | **No** (as in the module, N8) |
| Amendment2 l.111-114, Differences | "the two coincide … With a per-boot stream phase they can differ by one PX4 cycle" | **No.** They coincide in 3 of 5 phases. Where they differ (the same-pass phase: historical poll early; the +16 phase: receive delay), they differ by **one truth sample = 20 ms** (N8) |
| Amendment2 l.132-134, assumption 3 | window; ">0.03 s old shows as STALE_TRUTH"; "a truth from a neighbouring PX4 cycle inside the window is accepted silently" | The first two are **yes**. The third is **understated** (any pass, up to 16 ms earlier; +4 ms under delay) |
| Amendment2 l.128-140, assumption list | — | **Missing:** receiver-thread promptness (N4 fix part 2; N12) |
| README l.46-49 | the window and "a truth from a neighbouring 4 ms PX4 cycle can be chosen silently" | **Understated** (N8) |
| README l.76-78 | "the two can differ by one PX4 cycle" | **No.** One sample of a 50 Hz stream, 20 ms (N8) |

## New findings

### N8. MAJOR (wording, condition): wrong numbers for the truth offset in the module, amendment2 and the README

**Where.**
- `tools/sitl_dev08_link.py:25-29` and `:344-345`.
- Amendment2 l.85-88, l.113-114 and l.133-134.
- README l.47-49 and l.76-78.

**Scenario.** A reader of the preregistration amendment would conclude two things, both wrong for the submitted
gate:
- that the truth used is at most one 4 ms pass from the position;
- that the gate and the historical runner differ by at most 4 ms.

What the probe shows instead:
- The truth used trails the position's pass by 0, 4, 8, 12 or 16 ms, fixed per boot. Three phases in five are
  8–16 ms.
- At 3.6 m/s, the configured MPC_XY_VEL_MAX, a 16 ms lag is 5.8 cm of truth position.
- Any difference from the historical choice is a whole truth sample, 20 ms.

The behaviour is sound and both arms share it. Only the description is wrong.

**Fix.** Use replacement text along these lines:
- **Module `:25-29`, and amendment2 l.85-88 in short form.** "PX4 does not send the streams together. Each MAVLink
  stream keeps its own 20 ms schedule, and the truth stream's phase relative to the position stream is fixed per
  boot by when its stream-interval command is handled. With BURST_S below one 4 ms PX4 pass, the truth used was sent
  in the position's own pass or 4, 8, 12 or 16 ms before it, by that phase (review r3 probe). A receive delay of
  about 1 ms or more on the position can instead admit the truth sent one pass (4 ms) after it. In that phase the
  choice can then switch by one truth sample (20 ms) within an episode. Which pass the truth came from is not
  checked: its receipt age is bounded and recorded (truth_recv_skew_s), the PX4-time offset is not."
- **Amendment2 l.113-114 and README l.76-78.** "With a per-boot stream phase they can differ, always by whole
  samples (20 ms). In the same-pass phase, the 2 ms poll can read the position before its truth has arrived and use
  the previous truth; the gate's 3 ms wait avoids that. In the +4 ms phase, a receive delay can put the next pass's
  truth inside the gate's window. In the other three phases they coincide."
- **Assumption 3.** Replace "from a neighbouring PX4 cycle" with "whatever PX4 pass sent it (expected: the
  position's pass or up to 16 ms earlier; +4 ms under receive delay)".
- **`_settle` docstring.** Replace "same-burst" with "sent in the position's PX4 pass".

### N9. MAJOR (test robustness, condition): `test_same_burst_truth_is_awaited` is timing-fragile and failed the repository command

**Where.** `tests/test_sitl_dev08_link.py:435-468`: a 2 ms `time.sleep` at `:453` against BURST_S = 3 ms, and the
assertion "≥ 8 of 10 positive skews" at `:468`.

**Observed.**
- **Prescribed command, first run.** `FAILED (failures=1)`, 1 of 10 skews positive (`unittest_run.out`). The other
  nine were −0.016 or −0.012: the truth arrived 4 ms after its position, so the gate correctly took the previous one.
  At the time, another session's `python -B tools/run_tests.py` (pid 2184361, nice 10, affinity 6,7,12,13) was
  using the same cores.
- **Idle host.** 30 of 30 OK.
- **With my own busy loops on the same cores.** 1 failure in 80 runs:
  - 4 loops: 9/10 OK in one batch and 30/30 in another;
  - 2 loops: 30/30;
  - 8 loops: 10/10.

  The full file also passed 5 of 5 runs while the probe ran.
- **Mechanism.** The kernel has CONFIG_HZ=250. Under same-nice contention a woken thread can wait for the next 4 ms
  tick, so the 2 ms sleep becomes about 4 ms, more than BURST_S.

**Why it matters.**
- This is the only test that kills S1, S3, S4 and S5 (`mutate_r3.out`).
- Its spurious failures would block or confuse the PUBLIC_CPU and `validation.json` evidence.
- There is a risk that someone "fixes" it by changing production bounds.

**Fix.** Use the fake receive clock, with a thread that ingests the same-pass truth (fake recv +0.2 ms) 10 ms of
wall time after `checkpoint()` and `align()` begin waiting. The settle runs on the wall cap there, so there is no
margin to lose. `survivors_r3.py: s_settle_checkpoint` kills S1, S3 and S4; `s_settle_align` kills S2. The later
worktree edit (`burst_s=0.02` in this test) removes the fragility but is unreviewed. Keep or add a fake-clock test
in either case.

### N10. MINOR: the new order bound is untested; the test that names it is vacuous

**Where.** `tests:425-433`. The "late truth for t0" is put at receipt t0 + 0.0215. That is already outside
[t0 − 0.03, t0 + 0.003] by BURST_S alone, so the "before the next position message" clause (`:335-336`) is never the
reason it is excluded. Survivors: O1 (bound removed), O6 (bound replaced by a no-op) and O3 (bound = newest
position).

**Scenario** (`survivors_r3.out`):
- **Batched receiver.** The next position is stamped +0.5 ms and its truth +1.0 ms after the selected position. The
  submitted gate picks the selected position's truth (seq + 2); O1 and O6 pick the next pass's truth (seq + 5).
- **Two later positions batched.** The original gives seq + 2, O3 gives seq + 4 and O1 gives seq + 6.

**Fix.** Use the first scenario as the test (fake clock, three `feed.put` calls with explicit `recv`).

### N11. MINOR (residual N2): the settle in `align()` is unpinned (S2 survives)

**Scenario** (`survivors_r3.out`).
- The truth stream pauses 0.5 s and resumes with the sample that arrives 10 ms of wall time after `align()` starts
  waiting. The original accepts; S2 raises STALE_TRUTH.
- With earlier truths present, S2 sees the previous truth rather than the same-pass one.

**Impact.** `align()` uses the truth only for the existence check, so the effect is a false STALE_TRUTH at
alignment. That fails closed.

**Fix.** Add the fake-clock `s_settle_align` case to the N9 replacement.

### N12. MINOR (residual N4): receiver promptness is not a stated live assumption, and later truths are still admitted under stalls

The order bound helps: at a 45 ms stall the newest truth taken falls from +28 to +8 ms. But 30–45 ms stalls still
admit truths 4–16 ms newer than the position, silently, in 8 of 100 checkpoints. A ≥ 3 ms stall between a position
and its truth makes the gate take the previous truth, 20 ms older.

Amendment2's live assumptions (l.128-138) do not mention the receiver thread. Only the module docstring does
(`:29`). The integration runs a watchdog thread that samples RSS every 0.25 s, which is one plausible source of
GIL-induced stamping delays.

**Fix.** Add a live assumption: "the receiver thread stamps messages within about 1–3 ms of arrival". Name the two
silent effects above. Optionally have the smoke report the per-checkpoint skew histogram per phase; it is already
recorded.

## Notes

- **`truth_time_model` record string** (`:378-379`). It reads "receipt: latest within [-0.03, +0.003] s of the
  position sample" and omits "before the next position message". Add it so the ledger states the full rule.
- **Equivalent or acceptable survivors** (`mutate_r3.out`). All of these are acceptable:
  - O2 (`<=` in the order bound) is equivalent: a truth's seq is never a position's seq.
  - T3f and T3g give a BURST_S pin of (2, 4] ms, which is acceptable.
  - S6 (a 0.5 s wall cap) is unobservable live, because the live receive clock is the wall clock.
  - D2–D6 (diagnostics) are acceptable.
  - T6/T6b (TRUTH_AGE_S ± 0.009) and T8/C3 are as in round 2.
- **Regressions:** none. Every round-1 and round-2 kill re-run here is still KILLED; I re-ran most of them, not all.
  T1 (no upper truth bound) is now killed only by `test_burst_window_bound`, because the order bound hides it in the
  other tests.
- **Fixed per boot is a pass-time statement.** PX4's LOCAL_POSITION_NED `time_boot_ms` is `lpos.timestamp`
  (`LOCAL_POSITION_NED.hpp:64-67`), the EKF publication time. The EKF runs every `round(10 ms / 4 ms)` IMU samples
  (`imu_down_sampler.cpp` reset), and the stream sends the latest value at its own pass. So the PX4-time gap between
  the truth and the position sample can vary within an episode even though the pass phase is fixed. The probe's EKF
  model hides this, because its EKF updates fall exactly on the sends. The documents already say the PX4-time
  offset is not bounded, so nothing needs to change unless N8's text quotes sample-time offsets.
- **Tests are otherwise non-vacuous and robust.**
  - Every new rejection test has a positive control: `tests:180-188`, `:190-198`, `:419-424` and `:470-473`.
  - All waits are bounded.
  - `threading.Timer` in `:192` is a short non-daemon timer that ends at 0.1 s.
  - The file still does not touch `sys.modules`, `os.environ` or `sys.path`.
  - The only fragile test is N9.

## Contract criteria (amendment1 row "Parameter/telemetry boundary")

| Criterion | Status |
| --- | --- |
| Bounded, source-matched echo/readback | Met |
| Wrong/missing/stale echo fails; actual fresh native yaw values saved | Met. The fresh check is now exact (F5 killed), and the airframe parse is last-wins (P2 killed) |
| Coherent finite state snapshots | Met with limits. Finite now includes the truth quaternion (Q1/Q3 killed). Coherence is a receipt window: the truth used is 0–16 ms before the position's pass, fixed per boot, or +4 ms under receive delay. It is recorded per checkpoint and described with wrong numbers (N8) |
| All freshness/skip/stall checks accept valid fixtures and reject mutants | Met except for the order bound (N10) and the align settle (N11). The burst-wait kills depend on a fragile test (N9) |
| Failures abort, never an outcome | Met (unchanged) |
| No new ground-truth policy input; 10 Hz simulator pacing | Met (unchanged; K1/K2 killed) |

## What I ran (all under `/home/fair/motar_tmp/dev08_link_20261105/review_r3/`)

All runs used `CUDA_VISIBLE_DEVICES=""`, `PYTHONNOUSERSITE=1`, `nice -n 10`, `taskset -c 6,7,12,13` and a timeout.

What I did not do:
- open a socket, or start PX4 or Gazebo;
- use the GPU;
- write to any repository or the PX4 tree;
- signal any process I did not start. The busy loops were my own, under `timeout`, and were killed by their script.

Git use was `status` only.

- **Snapshot.** `snapshot/` holds the four submitted files; `sha256sum -c SUBMITTED_SHA256SUMS` gives OK.
  `reviewed_sha256.txt` lists the snapshot's code and test hashes. A re-check of the worktree at the end showed
  three files changed by another session (see the header).
- **`unittest_run.out`.** The prescribed command, first run: 37 tests, FAILED (failures=1, N9), under the other
  session's concurrent `tools/run_tests.py` on the same cores.
- **`flaky/`.** The real-clock test alone, repeated:
  - idle: 10 + 20 OK;
  - my own busy loops: 4 loops 9/10 and 30/30, 2 loops 30/30, 8 loops 10/10;
  - the full file during the probe: 5/5 OK.
- **`mutate_r3.py`, `mutate_r3.out`, `mutants_r3.json`, `mut/*`.** 59 single-site mutants plus an unmutated control
  (CONTROL_PASS), against the snapshot test file.
  - A flake guard re-runs a mutant up to 3 times when only the real-clock test fails. S1, S3, S4 and S5 failed all
    four runs.
  - **42 killed** (S7 by timeout) and **17 survived**:
    - real gaps: O1, O3, O6 (N10) and S2 (N11);
    - acceptable: T3f, T3g, D2–D6, S6, T6, T6b;
    - equivalent: O2, T8, C3.
- **`survivors_r3.py`, `.out`, `.json`.** For each survivor class, a fake-clock scenario on which the original and
  the mutant differ, with positive controls.
- **`probe_px4_truth_r3.py`, `.out`, `.json`.** Round 2's PX4-shaped probe on the snapshot. Rows:
  - 10 phases with the historical emulation;
  - iid and fifo receive delay at 0.5, 1, 1.5, 2 and 3 ms × 5 phases × 3 seeds;
  - RTF 0.9 to 0.4;
  - rx stalls of 5, 15, 30 and 45 ms, the gate against O1.

  `debug_stall_phase12.py` explains the 5 ms stall row.
- **`probe_rtf_r3.py`, `.out`, `.json`.** The RTF rows with the alignment feed artifact removed. The floor is about
  RTF 0.54, in one phase in five.
- **`probe_semantics_r3.py`, `.out`.** Q1 malformed message, Q2 align after a latched error, Q3/Q3b diagnostics
  (N6 repaired), Q4 the recorded `truth_time_model`.
- **Read-only source reading.** The PX4 files listed in the header, the historical runner
  (`tools/sitl_s3b_closed_loop.py:240-276`), the integration (`tools/sitl_s3b_dev08_prefix.py:227-237`, which
  records `truth_recv_skew_s` per checkpoint and does not use the removed `inbox_backlog`), amendment1 l.60-70 and
  l.126, and AGENTS.md.
