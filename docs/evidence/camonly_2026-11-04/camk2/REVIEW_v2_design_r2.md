# Review r2 (focused): CAM-K2 v2 repairs (2026-11-08)

- **Reviewer:** Claude Opus 5.5 (fresh context). Read-only; this is the only file I wrote.
- **Time:** 19:13 to 19:18 KST. I started after the 18:55 report time, so this report is late.
- **Worktree:** `MOTAR-realveh` at base `ab2eeb5`, with the uncommitted repairs:
  - the diff of `navrl_vehicle_rand.py`;
  - `tools/camk2v2_train.py`, `train_navrl_camk2v2.sh`, `tests/test_camk2v2_train.py` and the v2 prereg.
  - I also read the parts of `tests/test_realveh.py`, `tools/realveh_entry.py`, `tools/camk2e_run.py` and
    `motor_model.py` that bear on the four checks.
- **Scope:** only checks (1)–(4) from the brief. Round 1's other findings and CAM-K2-E were not re-reviewed.
- **Test run:** the one authorised command (`test_camk2v2_train.py`, CPU, cores 6–7): **38 tests, OK, none skipped,
  0.84 s.** I did not run `tests/test_realveh.py`. Its `TestRaisedLimit` (the real `MotorModel` clamp and the counters
  with the real clamp) is checked here by reading only.

## Verdict: PASS_WITH_LIMITS

M-1, m-1, m-2 and m-4 are repaired as asked. There is no MAJOR finding. One MINOR is prereg wording only, to fix
before the commit. Three LOW findings are optional.

## (1) M-1: counters and live read-back

**They are real.**
- `raise_counters` reads `mm.max_thrust`, the attribute itself (`navrl_vehicle_rand.py:384`). The clamp at
  `motor_model.py:93` and the reset draw at `:152` use that same attribute. So the read-back is the clamp that acts,
  not a copy of `cap64`.
- The record's `live_max_thrust_per_motor_n` (`:201`) is read after the swap.
- `_count_commands` (`:361-375`) counts the reference before both `torch_minimum` and the motor model's clamp.
- None of the per-step updates synchronises the device:
  - `numel` is a Python int;
  - the two counts are in-place additions to 0-d device tensors;
  - the running maximum stays on the device.
  - `.item()` and `float()` run only at the log points.
- No `torch.inference_mode` exists in `aerial_gym/` or `rl_games`. So the in-place `+=` on the counters cannot hit
  the inference-tensor error.

**A valid run cannot be failed by them.**
- **Initial draw.** The draw inside `initialize()` logs no "reset" line: `_account` returns while `_veh_rand_record` is
  None (`:289-290`). That draw runs before `_veh_rand_limit_raised` is set, so no reset line can lack the counters.
- **Reset before the first update.** It carries `cmd_elements` 0, `cmd_max_n` null and a live read-back. It passes
  `0 <= at <= above <= n` (`camk2v2_train.py:252-253`). The fixture does exactly this: `real_telemetry` redraws before
  its first update, uses the real module and is the positive case.
- **All counts legitimately zero.** This passes; the zero-share case is the fixture default, at 1.0 N.
- **Ordering.** Only one line must have `cmd_elements > 0`, and update line 1 always has it. The counters only grow,
  so `max(lines, key=cmd_elements)` is the latest state, whatever the order of resets and updates.
- **Quiet startup.** The stream is bound at `install()` (`:312`), and `realveh_entry` runs install before rl_games
  builds anything. The launcher's stdout and stderr both go to `launcher.log` (`camk2v2_train.py:622-624`), which is
  the only text checked (`:571`). The quiet case is tested (`test_camk2v2_train.py:616`).
- **Numerics.** The live float32 limit against the float64 cap differs by about 4e-8 relative, below `LIVE_RTOL` 1e-6.
  The JSON fields are ints, which the `isinstance(..., int)` checks need. The log keys do not collide with
  `log()`'s own fields.

**An invalid run cannot pass through them.**
- **Raise not in effect.** If the swap did not happen, or something later replaced `max_thrust`, the read-back reads
  2.0 against 2.4525. That fails the record (`:225-227`) and every line (`:247-250`); the "live max at 2 N" negative is
  tested.
- **Missing counters.** Missing counters, inconsistent counts and "never counted" each fail, one negative each
  (`test_camk2v2_train.py:657-690`), with real-module positives at 1.0 N and 3.0 N.
- **Comparison base.** The per-line check compares against the init line's `cap_per_motor_n` (cap64). That is
  consistent, because the record check ties cap64 to 4.0·m·g/4 with m in [0.249, 0.251] and g 9.81.
- **What stays.** `limit != cap` in the record check (`:221-222`) still compares two copies of cap64, as round 1
  noted. That is now harmless, because the live read-back carries the proof.

## (2) Optional LIMIT_MODE: no silent raise found

- `initialize()` (`:171-186`) gives "raise" only when `requested == "raise"` and the TW is above the robot's own T/W
  in every env.
  - Absent or "lower" with a TW above it: refuses, with v1's message plus a hint.
  - "raise" with a TW below it: refuses.
  - Envs that straddle the robot's T/W: refuses.
  - An unknown value: refuses in `settings_from_env` (`:139-143`).
  - An empty string counts as absent.
- **v2.**
  - The launcher exports `NAVRL_VEH_RAND_LIMIT_MODE=raise` (`train_navrl_camk2v2.sh:127`).
  - `expected_settings` includes it (`camk2v2_train.py:119-121`). The wrapper compares it three times: the receipt's
    settings, the receipt's `vehicle_rand.settings` and the record's settings (`:159-170, 205`). The install line is a
    fourth (`:350`). The record and the init line must also show mode "raise" (`:221, 353`).
  - So a v2 run without the key fails twice: at init (refusal), and again after the run (settings mismatch).
- **v1.** Its settings carry no key, so TW 2.25 runs "lower" exactly as committed.
  - `git diff ab2eeb5` is empty for `tools/camk2_train.py`, `train_navrl_camk2.sh` and `tests/test_camk2_train.py`.
  - `test_camk2v2_train.py:129` asserts that v1 has no key.
- **OFF.** If the key is set while `NAVRL_VEH_RAND` is off, `realveh_entry.py:122-124` refuses, because the key has
  the `NAVRL_VEH_RAND_` prefix.
- **CAM-K2-E.** `camk2e_run.treatment_env` sets the key and `expected_settings` requires it (`camk2e_run.py:112-128`).

## (3) T/W 4.0 numbers against the code and the prereg

I recomputed every figure, and each matches the code (`VEH_RAND` TW 4.0, tag `tw40`, launcher 4.0) and the prereg:

| Quantity | Value |
| --- | --- |
| Limit, 4.0 × 0.25 × 9.81 / 4 | 2.4525 N |
| Per motor, 4.0 × 1.1 kg / 4 | 1.1 kgf = 10.79 N |
| Thrust acceleration | 39.24 m/s² (own: 32.0) |
| Hover fraction | 0.25 |
| Roll, 2·0.13·T / 8.45e-4 | 754.6 rad/s² (own: 615.4) |
| Real frame, L = 0.0795 m | 686.7 / 286.1 rad/s² at Ixx 2.5e-3 / 6e-3; ratio 1.10–2.64× |
| Yaw | 29.02 (own: 23.67); real 19.6–61.7, given as "about 20–62" |
| Effective T/W | 3.4–4.6 per env; 3.23–4.83 per motor |
| Worst case | 3.23 > the 1.8 floor |
| Weakest-env hover | 0.759 N |
| Tilt headroom | 2.59 g / 1.82 g |
| Source point | 4 × 1.084 / 1.1 = 3.94 |
| Identity | 8 / (0.25 × float32 9.81) = 3.261977434, and "3.2620" is 6.9e-6 away |

- The opposite-direction corrections that lead from 3.94 to 4.0 are stated (`prereg :44-48`), and the value is
  labelled INFERRED.
- No stale 4.2, 2.575 or `tw42` remains in the v2 code or tests. The prereg keeps 4.2 only as "the earlier
  estimate".
- The prereg's telemetry paragraph (`:155-168`) describes the code exactly:
  - the same keys;
  - the same four TRAINING_FAIL conditions;
  - shares that are descriptive, not gates;
  - no counters in "lower" and "none";
  - logging at the first update and every 100,000th (`UPDATE_LOG_EVERY = 100000`).

## (4) Tests

`test_camk2v2_train.py`: 38 OK. The new M-1 test has real-module positives and five negatives, all non-vacuous. The
m-1, m-4, L-3 and v1-refusal tests are present. `test_realveh.py:847-868` (counters with the real clamp, an
unsaturated control and no counters in lower, none and OFF) is non-vacuous by reading, but I did not run it here.

## Findings

**r2-m1 (MINOR, prereg wording only): the inert-raise case has no stated threshold or claim consequence.**
- `prereg :146-147` says that if the commands rarely reach 2 N, "the raised limit changes little". The claim text
  (`:401-404`) names "the per-motor thrust limit raised to T/W 4.0" whatever share is measured.
- The share is now recorded, so the rule for reading it should be fixed before the run. That is the gate-design
  checklist item "derive criteria from the claim".
- **Fix:** add one sentence before the commit. For example: if `share_above_own_max` < 1 % (the number to be chosen
  before the run), the report must say that the raise was nearly inert during training, and that no result may be
  attributed to the higher T/W.
- No code change is needed.

**r2-L1 (LOW): "raise" requested but "none" selected passes silently.**
- `LIMIT_MODE=raise` with a TW equal to the robot's own T/W selects "none" with no refusal (`navrl_vehicle_rand.py:172-173`).
- This is not a silent raise: nothing is raised, and v2's record check requires "raise".
- **Fix (optional):** refuse when the requested mode is "raise" but the mode chosen is "none".

**r2-L2 (LOW): the shares stop at the last log point.**
- They cover commands up to the last reset or update line (every 1,000th reset, every 100,000th update). The tail
  after it is not counted.
- The prereg discloses this (`:166`, "up to the last log line"). No fix is needed for a descriptive record.

**r2-L3 (LOW): tests not run here.**
- `tests/test_realveh.py` (`TestRaisedLimit`, the real clamp and the counters) was outside the authorised command.
- **Fix:** it must pass in the implementer's full CPU run or PUBLIC_CPU before the commit.

## One line per finding

- r2-m1 MINOR: `prereg :146-147, 401-404` has no pre-stated share threshold and no claim consequence if the raise is
  nearly inert. Add one sentence before the commit; no code change.
- r2-L1 LOW: `LIMIT_MODE=raise` with a TW equal to the robot's own T/W selects "none" without refusing
  (`navrl_vehicle_rand.py:172-173`). This is not a raise. Optionally refuse.
- r2-L2 LOW: the shares cover only up to the last log line. This is disclosed (`prereg :166`); no fix.
- r2-L3 LOW: `tests/test_realveh.py` was not run in this review. It must pass in the full CPU run or PUBLIC_CPU
  before the commit.
