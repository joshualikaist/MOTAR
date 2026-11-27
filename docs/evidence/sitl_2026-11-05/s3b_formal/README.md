# S3b formal v1 — preparation record (2026-11-06)

**Status:** attempt01 ran on 2026-11-06 and reads **FEASIBLE_IN_SITL** ([Result](#result-attempt01-2026-11-06)); results review r1: PASS_WITH_LIMITS. Read the
[preregistration](../../../plans/prereg_sitl_s3b_formal_v1_2026-11-06.md) first. Runner `tools/sitl_s3b_formal.py`,
tests `tests/test_sitl_s3b_formal.py`.

## What the unit does

- **Scenes:** Frozen Policy F flies PX4 v1.14.3 SITL (navrl_quad) on the G8 r2 scenes whose zero-action target ran all
  600 actions.

| Block | Density | Scenes | Isaac on these scenes | Role |
| --- | --- | --- | --- | --- |
| A | 70 bars | all 122 eligible (excluded envs 5, 14, 19, 42, 45, 63) | 121 capture, 1 crash | primary: FEASIBLE_IN_SITL if SITL crash ≤ Isaac crash + 5 pp |
| B | 205 bars | first 30 of 124 eligible | — | reported |
| C | 70 bars | first 10 of block A, a second boot | — | reported (repeatability) |

- **Loop:** the dev07 development loop, unchanged in what it computes. A normalized diff in review r3 shows only
  additions.
- **Added for a formal run:**
  - an owned fresh-rootfs session per episode;
  - a parameter read-back;
  - a 0.15 s lateness bound and a 2 s bounded step wait;
  - a per-episode runtime binding of the PX4/Gazebo stack (BINDING_CHANGED);
  - an idle check per episode (HOST_BUSY);
  - launch refusals: sources equal to HEAD, Policy F `f7022139…`, a clean external replay module, no inherited
    simulator variables, command yaw `truth`, single use;
  - `final.json` on every exit path, including SIGTERM.

## Independent reviews (Claude Opus 5.5, fresh context each round)

- [r1](REVIEW_r1.md): **FAIL**.
  - BLOCKER: the MAVLink socket was never closed, so the second episode could not start.
  - MAJOR M1: an unbounded step wait.
  - MAJOR M2: sources were not compared with HEAD, and F was not pinned.
  - MAJOR M3: no live-path tests.
  - All were repaired.
- [r2](REVIEW_r2.md): **FAIL**.
  - MAJOR-1: the PX4/Gazebo stack under test was not bound.
  - MINOR-1 to MINOR-4: prereg wording, a missing `final.json` on some paths, inherited environment, and idleness
    checked once only.
  - All were repaired.
- [r3](REVIEW_r3.md): **PASS_WITH_LIMITS**, no BLOCKER, MAJOR or MINOR. It verified:
  - the real-plan runtime binding: 61 files, 40 plugins, none missing, equal to AGQ attempt01's;
  - SIGTERM and exception paths in the real `launch()`;
  - no false environment refusal.
- **Open LOWs from r3:**
  - L1: the `empty.world` template is not in the compared binding. No unit edits it.
  - L2: a timeout during block B or C voids a complete block A, as preregistered.
  - L3: lateness p99 and the read-back values are reported per episode only.
  - L4: no separate negative test for the command-yaw refusal; the `S3B_` refusal covers it.

## Validation

[`validation.json`](validation.json) has the details.
- `tests/test_sitl_s3b_formal.py`: 33 OK, about 120 s. This includes fake-PX4 tests of:
  - two consecutive episodes;
  - a stalled clock, a dead process and a time jump (LATE);
  - PARAM_VALUE and INT32 parsing;
  - every stop rule, BINDING_CHANGED, HOST_BUSY, a tree change and STOPPED_AFTER_A;
  - every launch refusal;
  - a missing `final.json`.
- `--check`: inputs pinned (24 files), schedule 122/30/10, environment OK.
- PUBLIC_CPU (one thread, on the staged bytes): PASS, 3538 run, 0 failures, 0 errors, 10 skipped; kernel faults
  0 → 0.

## Result: attempt01 (2026-11-06)

**Reading: FEASIBLE_IN_SITL** (prereg rule: block-A SITL crash ≤ Isaac crash + 5 pp).
Results review r1: PASS_WITH_LIMITS.

- **Approval.** The user approved this run in chat at about 16:35 KST, and with it the decision to enter S3b while S3a
  reads NOT_EQUIVALENT.
  - The approval was given for commit `c22a91b`. The run's tree was `0d44972`, which adds only the S3c/S4 files,
    `EXPERIMENTS.md` and `WORKLOG.md`.
  - `git diff c22a91b..0d44972` is empty for every S3b and production source. The approval file records this.
- **Execution.**
  - Ran once, 16:50–18:50 KST, on cores 6,7,12,13 at nice 10 with one thread and CUDA hidden.
  - COMPLETE, 162 of 162 episodes VALID. Kernel faults 0 → 0, and the tree was unchanged.
  - The runtime binding of the PX4/Gazebo stack was identical in every episode.
- **Records.**
  - [`execution_record.json`](execution_record.json): bindings, per-block numbers, the per-scene table and the outcome
    transitions. It was generated from the raw files by `~/motar_tmp/s3b_formal_20261106/make_results.py`.
  - [`reading_attempt01.json`](reading_attempt01.json): a byte copy of the fixed analyzer's output.
  - The raw attempt stays outside Git in `~/motar_tmp/s3b_formal_20261106/attempt01/`.

| Block | Scenes (VALID) | SITL capture / crash | Isaac, same scenes | Δcrash (pp) | Same outcome |
| --- | ---: | --- | --- | ---: | ---: |
| A, 70 bars (rule) | 122 (122) | 122 / 0 | 121 / 1 | −0.82 | 121 |
| B, 205 bars (reported) | 30 (30) | 27 / 3 | 26 / 4 | −3.33 | 25 |
| C, 70 bars, second boot (reported) | 10 (10) | 10 / 0 | 10 / 0 | 0.00 | 10; 10 of 10 repeat block A |

- **Block A.**
  - The SITL crash rate is 0 %, with a Clopper-Pearson 95 % interval of 0–2.98 %.
  - The worst case that counts INVALID episodes as crashes is the same, since none were INVALID.
  - The one scene where Isaac crashed (env 75) was captured in SITL at step 72.
- **Block B.** SITL crashed in env 10 (bar; Isaac also crashed), env 14 (bar; Isaac captured) and env 18 (altitude;
  Isaac captured). Isaac's other three crashes were captured in SITL.
- **Block C.** In all 10 second boots the outcome repeated. Unlike the AGQ members at 205 bars, these 70-bar scenes
  did not flip between boots.
- **Timing and state.**
  - Lateness was at most 0.024 s; the truth sample's age at most 0.019 s.
  - Real-time factor 0.995–1.006; compute at most 41 ms (the first step), otherwise under 11 ms. No parameter problems.
  - PX4's EKF-versus-truth horizontal error, per-episode maximum: median 0.21 m in every block. The largest values
    were 0.54 m (one block-A episode) and 1.12 m (block B; two episodes above 0.5 m).
- **Median capture time in SITL:** 38.5 steps (A) and 45 steps (B).

**What follows:**
- Frozen Policy F, unchanged, flew this PX4 v1.14.3 stack (EKF, controllers, offboard interface, real latency) at
  70 bars within the plan's crash margin, under the disclosed approximations.
- This is the S3b prerequisite the plan names for S5.

**What does not follow:**
- real-flight readiness;
- S3a equivalence;
- Gazebo-sensor realism (S4);
- a heavier vehicle or extra latency (S3c);
- a claim at 205 bars;
- target reactivity.

The comparison is of rates on the same scenes:
- the replayed target does not react;
- the sensors are MOTAR's own, rendered from truth;
- the PX4 velocity loop is not matched to G9;
- dev07 had already flown 9 of the block-A scenes and 10 of the block-B scenes.

**Likely uncounted contacts** (results review r1, MAJOR-1, disclosure):
- **The rule:** a crash is MOTAR's geometric rule, a 0.14 m footprint against the bar boxes. The Gazebo contact
  cross-check of the plan is not implemented (prereg, *Known limits*).
- **The gap:** navrl_quad's rotor collision cylinders reach about 0.31 m from the centre. So a rotor can touch a bar
  without a MOTAR crash.
- **The evidence:** five episodes show disturbances consistent with contact, each within about 3 steps of a bar
  clearance of 0.18–0.29 m:
  - altitude dips to 0.49 m (A env 82) and 0.275 m (B env 8);
  - EKF speeds of 4.7–9.4 m/s against the 3.6 m/s cap (A env 50, B env 8, B env 18).
- **B env 18:** its altitude crash is such a fall. The clearance was 0.182 m, then z went from 1.03 to 0.06 m, and
  `px4.log` shows "Attitude failure (roll)" and "Failsafe".
- **The direction of the bias:** it can only add SITL crashes. Counting both block-A cases as crashes gives
  Δcrash +0.82 pp, still inside the 5 pp margin. This is inferred; no contact sensor ran.
- **Other logs:** `px4.log` of B env 10, 14 and 16 also shows "Attitude failure (roll)", probably after the episode
  ended (the log has no timestamps).

**Definitions and scope of the reported blocks:**
- `min_clearance_m` in the per-scene table is the distance from the vehicle centre to the nearest bar box within the
  bars' height. The crash threshold is 0.14 m, not 0.
- Block C repeated only 70-bar captures, so it says nothing about whether crash outcomes repeat between boots.

## Results review r1 (Claude Opus 5.5, fresh context)

[RESULTS_REVIEW_r1.md](RESULTS_REVIEW_r1.md) is a copy of
`~/motar_tmp/s3b_formal_20261106/results_review_r1/REVIEW.md`. Verdict **PASS_WITH_LIMITS**, no BLOCKER.
- **Checks:**
  - authorization, the approval-commit note (all 33 pinned sources identical at `c22a91b` and `0d44972`) and the
    envelope;
  - the ledger in schedule order, validity recomputed (0 mismatches) and the binding core identical in 162 rows;
  - every number recomputed independently, exactly matching `reading.json`;
  - physical plausibility: the bars match the Isaac layouts within 5e-5 m, and the capture and clearance values
    reproduce.
- **Findings:**
  - MAJOR-1: disclosed above;
  - MINOR-1 (the block-B dev07 overlap), MINOR-2 (the clearance definition) and MINOR-3 (the attitude-failure lines):
    repaired above;
  - LOW-1: the binding environment was checked by hand; only `GAZEBO_LOG_PATH` differs;
  - LOW-2: noted above.
