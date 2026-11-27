# Addendum r1a: doc fix check (unit B, review r1)

- **Reviewer:** Claude Opus 5.5 (`claude-opus-5-5`), same reviewer as r1, read-only on the worktree. 2026-11-18.
- **Scope:** the README-only fixes for r1 P2-1, P2-2 and the P3 wording items, in
  `ros2/motar_camera_policy/README.md`.

## What changed

- **Only the README changed.** I rebuilt the README version reviewed in r1 by applying the r1 diff to the preB copy.
  Its SHA-256 is `fff4abfa…`, matching the implementer's record. The current README is `2321296e…`. The diff
  (`r1a/readme_r1a.diff`) adds exactly two blocks, at README lines 367-372 and 418-434.
- **Every other unit B file is unchanged.** `logs/unitB_changed_files.sha256` reports OK for all other 10 files.
  `git status` is identical to the r1 status. The same 10 files differ from preB as at r1.
- **Both pinned prefixes are intact.**
  - ROS README, first 16865 characters: `4caab165…`, OK.
  - Evidence README, first 16007 characters: `5b5ce24b…`, OK.

## Results

| Item | Result | Evidence |
| --- | --- | --- |
| Pinned prefix unchanged | **PASS** | Prefix SHA-256 values above. `test_camera_ros2_package`, `test_camera_runtime_unit_b` and `OutdoorRecords`: `Ran 40 tests OK` (`r1a/tests_r1a.log`). `check_public_docs`: exit 0. |
| (1) "*Supersedes … P2-2*" paragraph | **PASS** | See (1) below. |
| (2a) Cadence "every 7th tick (a refusal, 5 re-entry holds, a command)" | **FAIL (understated)** | See (2a) below. |
| (2a') Operator advice to watch `refusal`, `consecutive_refusals`, `hold_episode_s` | PASS, with a note | See (2a') below. |
| (2b) Gated-time statement | **FAIL (overstated)** | See (2b) below. |
| (2c) Clock statement | **PASS** | See (2c) below. |
| (2d) Replay duplicate-stamp statement | **PASS**, with a nit | See (2d) below. |

### (1) The supersedes paragraph

- **Unconditional encoding.** It says the gateway writes the encoding on every deployment. Correct:
  `odometry_covariances` is unconditional (`camera_ros2_px4_node.py:319-324`, `:470-472`).
- **Slot contents.** Slot 0 = -20261118.0 and slots 1-3 = the fields, whenever forwarded. Correct.
- **Who reads it.** "Only a policy node whose binding declares the encoding … reads them" is a true necessary
  condition. The paragraph below it adds the outdoor-profile condition (`camera_ros2_session.py:141`).

### (2a) The cadence statement

- **The 7-tick cycle is only the densest case.** A policy command ends the episode, so any cycle with one command per
  at most 31 ticks never latches: up to 25 refusals, then 5 re-entry holds (30 holds, 0.0-2.9 s), then one command.
- **Probe** (`r1a/r1a_probe.json`, 60 s each):

  | Refusals per cycle | Cycle length | Zero-hold share | Latch |
  | --- | --- | --- | --- |
  | 1 | 7 | 85.8 % | none |
  | 10 | 16 | 93.8 % | none |
  | 25 | 31 | **96.8 %** | none (longest episode 2.9 s) |
  | 26 | 32 | — | `HoldTimeout` at tick 30 |

- **My own r1 P2-1 text had the same imprecision.** I am correcting it here.
- **Suggested wording:** "A policy command ends the episode, so any refusal pattern that lets one policy command
  through before the hold reaches the limit (at most 25 consecutive refused ticks, then 5 re-entry holds and a
  command; the densest is every 7th tick) never latches. In the review probe this held the vehicle at zero for up to
  97 % of ticks for 60 s, with no fault."

### (2a') The operator advice

- The advice is valid, but `refusal` and `consecutive_refusals` reset to None and 0 on every accepted frame, re-entry
  holds included (`camera_ros2_session.py:387-388`).
- In a sparse pattern, the 1 Hz diagnostics often show `refusal: null` and `consecutive_refusals` of 0 or 1.
- `state` (`HOLD_REENTRY` or `HOLD_FRAME_REFUSED`) and `hold_episode_s` are the better signals. Suggest naming
  `state` first.

### (2b) The gated-time statement

- **What is correct.** "A `WAITING_ALTITUDE` tick does not end the episode" and "time spent gated still counts" are
  right.
- **What is wrong.** "After a landing, or any long gated stretch, a re-climb in the same session latches at its first
  hold" is true only if an episode was open when gating began, meaning the last proposal before gating was a zero
  hold.
- **Probe d:** after policy commands (no open episode), 600 gated ticks (60 s) and then a re-climb gave 5 re-entry
  holds, then commands. There was no fault, and the status was `RUNNING_PROPOSALS`.
- **The r1 probe (case b), with an open episode,** latched at `episode_s` 60.3.
- **Suggested wording:** "If an episode is open when the gated stretch begins (the last proposal was a zero hold,
  e.g. refusals during the descent), a re-climb in the same session latches at its first hold; reset the session
  before re-arming." The pre-OFFBOARD latch sentence is correct.

### (2c) The clock statement

- It says the checks catch jumps, pauses and a wrong period, but not a uniformly slow ROS clock, and gives
  "about 1.33-1.5×". This matches the r1 analysis (P3-2).

### (2d) The replay duplicate-stamp statement

- `RefusalHoldEpisode.hold` raises on a non-increasing capture (`camera_policy_runtime.py:146-147`). The NPZ `replay`
  calls it on held lines (`:1386`). The case is new only for consecutive held frames after a refusal, because
  `reset()` clears `_last_stamps`.
- **Nit:** a regressing stamp also stops it, not only a duplicate. The sentence "Recordings must carry strictly
  increasing stamps" covers this.

## Verdict for r1a

- **Fixed:** P2-2 and the clock and replay P3 items.
- **Not yet accurate:** two items, (2a) and (2b). Both err on the cautious side for safety, but they are factually
  inaccurate, and (2a) understates the bound's gap.
- **Status:** P2-1 is documented but understated, so it stays open until the text is corrected. No code change was
  needed or made.
- **Overall:** unit B stays **PASS_WITH_LIMITS**, with P2-1 open on wording.

## Commands (env as in r1: `CUDA_VISIBLE_DEVICES=""`, `PYTHONNOUSERSITE=1`, `PYTHONDONTWRITEBYTECODE=1`, `taskset -c 0-3,6-23`, `python -B`)

- **Rebuild the reviewed README:** `patch` of the r1 README diff onto the preB copy → `fff4abfa…`. Then
  `diff -u reviewed current` → `r1a/readme_r1a.diff` (41 lines, 2 hunks, additions only).
- **Hash checks:** `sha256sum -c logs/unitB_changed_files.sha256` gives only `README.md: FAILED` (expected). The
  prefix SHA-256 checks pass.
- **Tests:** `python -B -m unittest test_camera_ros2_package test_camera_runtime_unit_b
  test_camera_runtime_outdoor_r1.OutdoorRecords` → `Ran 40 tests in 1.574s OK`.
- **Docs check:** `python -B tools/check_public_docs.py` → exit 0.
- **Probe:** `r1a/r1a_probe.py` → `r1a/r1a_probe.json` (cases c and d above).

## Addendum r1b (re-check of the two r1a FAIL items, 2026-11-18, same reviewer, read-only)

Checked README `e062bfec…` (the pinned prefix `4caab165…` still OK; other unit B files unchanged; README tests `Ran 40 OK` in `r1a/tests_r1b.log`; `check_public_docs` exit 0).

- **(2a) "Sparse commands defeat the bound": PASS.** It names the general condition (one command before the limit), at most 25 consecutive refusals plus 5 re-entry holds plus a command, the 7-tick densest case, "up to 97 %" (96.8 %), and 26 refusals latching at 3.0 s. All of these match `r1a/r1a_probe.json` case c.
- **(2a') Operator advice: PASS.** It now names `state` and `hold_episode_s` first, and correctly notes that `refusal` and `consecutive_refusals` reset on every accepted frame (`camera_ros2_session.py:389`).
- **(2b) "Gated time counts": PASS.** The latch-on-re-climb is now conditional on an episode being open when gating begins. This matches probe d (no open episode: no latch) and r1 probe b (open episode: latch at 60.3 s).
- **(2c) Clock and (2d) Replay stamps: PASS.** Both are unchanged since r1a.
- **Result:** P2-1 is closed as documented. Unit B stays **PASS_WITH_LIMITS**, with no open P0, P1 or P2. The remaining open items are the P3 hardening ones (P3-3, P3-4, P3-5, P3-7, P3-8), plus NOT_TESTED for rclpy, DDS, PX4 and the vehicle.
- **Observation, outside this check:** since r1, 11 new untracked files have appeared in the worktree under `docs/evidence/sitl_2026-11-05/camera_runtime/` (`OUTDOOR_*.md`, `ROS2_FIX_*.md`, including `OUTDOOR_UNIT_B_r1.md`). I did not review them.
