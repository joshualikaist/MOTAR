# H-R3 pilot HR3-P01 — independent review — 2026-10-18

## Reviewer

- **Model:** a read-only Claude subagent on `claude-fable-5-1` (Fable 5.1), prompted in neutral
  sensor-fusion terms. It completed on the first attempt.
- **Scope:** HEAD `b6aba18`; source `5c8afde`; preregistration `5f8719f`. It had no write access to
  the repository or the raw data, and used no GPU.
- **Independent recomputation:** it recomputed the results from the four raw `h_r3_trace.npz`
  files with its own NumPy script, without importing repository code. The script and its outputs
  are archived unchanged in [`review/`](review/).
- **Raw manifest:** verified 21/21 (`ae2633a2…`).
- **Recommendation: ACCEPT_WITH_CORRECTIONS.**

This review is independent of the author's self-checks. Those are the tests, the byte-for-byte
analysis re-run and `h_r3_posthoc.json`, which are recorded separately in
[`REPORT.md`](REPORT.md).

## Verdicts on the five questions

| # | Question | Verdict |
| --- | --- | --- |
| 1 | Arm matching | **PASS** |
| 2 | Online ground-truth leakage | **PASS** |
| 3 | Primary-metric denominator | **PASS** |
| 4 | Confounding of the translation effect | **CONCERN** (disclosed, but one report number misleads) |
| 5 | Hidden collisions or natural recovery | **CONCERN** (one mis-attribution, outside the primary population) |

### 1. Arm matching — PASS

- **Counts reproduced:** 125 triggered, 107 matched and 18 unmatched (9 t0 / 9 prefix). The
  unmatched events are 16 FOV_EXIT and 2 FULL, and 3 envs never triggered. Matched t0 runs from 2
  to 10.
- **Coupling onset:** step 10 for every scripted arm. The envs that differ at step 10 are exactly
  the 21 not yet branched: 18 unmatched plus 3 that never triggered.
- **Resets around the onset:** B0 resets env 79 at step 10, and no scripted arm resets anything at
  step 10. This supports the reset-driven explanation that the report leaves unverified.
- **One controller for B1/B2/B3:** they are one function with identical inputs. The only B3-specific
  term is `motar_eval/h_r3_arms.py:82-84`.
  - On same-state control steps, B3 = B2 in 140/140 steps with offset 0 and in 0/55 with a nonzero
    offset.
  - B1 = B2 in xy on 108/108 steps, and B1's yaw is exactly 0.
  - Compute and environment configuration are identical.
- **Sensitivity check:** the two unmatched FULL events (envs 89 and 111), each evaluated at its own
  t0, are concordant between B2 and B3. Excluding them cannot bias B3−B2.

### 2. Online ground-truth leakage — PASS

- **Action path inputs:** own pose and orientation, the KF tracker state, and
  `_lidar_distance_m()` (= `depth_range_pixels × max_range`).
- **Trigger and window end:** these use `_camera_visible_now`. That flag is the detector's
  `visible` output (segmenter score with a minimum-pixel threshold), not the renderer mask.
- **Ground truth stays in the evaluator:** `task.target_position` and the E-L0 outputs are read
  only after the step, for the digest and trace, and never feed back.
- **The shadow B3 call has no side effects:** unbranched envs stay digest-identical through step 9.
- **No arm-specific inputs:** B3 receives nothing that B1/B2 do not.

### 3. Primary denominator — PASS

- **Recomputed independently:**
  - n = 22 (FULL 18, CENTRE_ONLY 4, PARTIAL 0), with the same class at t0−1 in every arm;
  - reobserved: B0 18, B1 7, B2 9, B3 17;
  - B3−B2: 9/1 discordant, McNemar p = 0.0215, bootstrap [+13.6, +59.1];
  - all six contrasts, the outcome patterns and every secondary in the primary table match.
- **Terminations in primary windows:** only B0 env 76, a crash after reobservation, which is counted
  correctly.

### 4. Confounding — CONCERN

- **Ruled out by the data:**
  - noise as initiator: in all 10 discordant events the action differs first;
  - pre-branch belief error: identical by matching, 0.05–0.28 m, with no pattern;
  - the target trajectory;
  - **total motion under arm control:** 1.65 m for B2 and 1.66 m for B3. In the nine `001` events
    it is 1.81 m for B2 and 1.60 m for B3.
- **Misleading report number:** the report's "B3 travels 0.39 m more" uses window translation. That
  includes the Policy-F steps after reobservation, so it measures policy resumption rather than the
  manoeuvre.
- **Still open:**
  - (a) the commanded-speed profile. Under control the means are 0.83 m/s (B2) and 1.22 m/s (B3).
    B2 is stalled by the shared safety filter behind the bar in envs 73, 76 and 84 (minimum
    0.01/0.05/0.08 m/s). "B2 stuck behind the bar" is therefore not separated from "B3 chose a clear
    viewpoint";
  - (b) noise realisations differ inside every primary window;
  - (c) selection: every primary event has its first detection at step 0 and t0 between 2 and 9;
  - (d) the slow 1.0 m/s scripted baseline;
  - (e) a single seed.

### 5. Hidden collisions or natural recovery — CONCERN

- **No hidden collisions:** the collision counts were reproduced (B0 5 / B1 2 / B2 2 / B3 3 over the
  107 matched events; 1 under control, B1 env 34). Natural recovery is disclosed.
- **Defect:** `tools/h_r3_analysis.py:74-82` tests for reobservation before termination *on the same
  index*. That index holds the post-reset observation.
  - **Effect:** B1 env 34 (FOV_EXIT) crashed under arm control at index 13. The new episode's camera
    flag, with 4 px, was counted as "reobs 0.9 s", and the launcher's `end_reason` reads
    "reobserved" for the same reason.
  - **Corrected FOV_EXIT:** B1 32/83; B2−B1 50/0, +60.2 pp.
  - **Scope:** the primary result is unaffected, and no test covered the case.

### Numbers the reviewer could not reproduce from the evidence

- The smoke "1.4 mm" and "process-id mismatch": the smoke records were then only in session
  scratch.
- "Byte-for-byte reproduction" and "same 5c8afde tool": the JSON carried no tool binding.

## Corrections and how they were resolved

The corrections are append-only. `tools/h_r3_analysis.py`, `h_r3_analysis.json`, the
preregistration and the raw data are unchanged. The new tool
[`tools/h_r3_review_corrections.py`](../../../tools/h_r3_review_corrections.py) is NumPy only, and
its output is [`h_r3_review_corrections.json`](h_r3_review_corrections.json).

| # | Reviewer correction | Resolution |
| --- | --- | --- |
| 1 | Same-index terminal rule | **Recount under the preregistered rule:** the observation at a terminal step cannot count as a reobservation. One outcome changes: B1 env 34, FOV_EXIT, from reobserved to not. FOV_EXIT becomes B1 32/83, with B2−B1 50/0, +60.2 pp. Primary unchanged. **Test added.** The launcher's `end_reason` caveat is recorded in the REPORT addendum. |
| 2 | Window translation includes Policy-F steps | **New figures under arm control:** translation B2 1.65 / B3 1.66 m (in `001`: 1.81 / 1.60 m); mean commanded speed 0.83 / 1.22 m/s; B2 stalled (< 0.1 m/s) in envs 73, 76 and 84. The REPORT's "0.39 m more" is superseded in the addendum. |
| 3 | Selection | **Recorded:** the matched set equals the envs branched at or before the onset step 10 (`true`). The onset reset is B0 env 79. All primary events have their first detection at step 0, with t0 between 2 and 9. |
| 4 | Smoke records and binding | **Preserved:** both smoke runs as new raw in `../aerial_gym_simulator/results/h_r3_pilot_v1/smoke/`, verified 5/5 and 17/17, with records in `run_records/smoke/`. The 1.4 mm figure is recomputed from the preserved smoke traces: 0.00136 m at step 14, target 0.0 m. **Binding:** the analysis tool's SHA-256 `42338457…` equals the `5c8afde` blob, and the re-run is byte-identical (`true`). |

After these corrections, the reviewer judged the exploratory label, the PROVISIONAL status and the
confirmatory design (sham-lateral and speed-matched B2 arms) appropriate. The corrections
themselves were checked by the author's tests and by recomputation, **not** by a second
independent review.
