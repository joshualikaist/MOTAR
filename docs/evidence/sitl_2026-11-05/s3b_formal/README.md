# S3b formal v1 — preparation record (2026-11-06)

**Status:** preregistered and reviewed, **NOT_RUN**. Read the
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
