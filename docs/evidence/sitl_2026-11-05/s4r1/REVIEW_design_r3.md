# S4R-1 v1: independent re-review r3 (repairs to r2), fresh context

- **Reviewer:** Claude Code subagent, Opus 5.5 (`claude-opus-5-5`), fresh context, read-only. Date: 2026-11-10.
- **Scope:** worktree `src/MOTAR-camonly` (`camonly/dev`, HEAD `fb455c4`); untracked unit files
  `tools/sitl_s4r1.py` (1,460 lines), `tests/test_sitl_s4r1.py` (976), `docs/plans/prereg_sitl_s4r1_v1_2026-11-10.md`
  (384). Read against `s4r1_review_r2/REVIEW_design_r2.md` and `REPAIR_r2.md`, plus `sitl_s3c.launch_unit` and
  `sitl_s4f1.gzip_ulogs/verify_ulog_record`.
- **Not done:** no PX4, Gazebo, GPU, `--launch`, `--smoke` or network. Only this file was written.

## What I ran

1. Requested unit tests (cores 10,11, nice 10, CUDA hidden, 900 s timeout): **34 tests, OK** (4.05 s).
2. Display preflight on this host, read-only (`display_problems(os.environ)` and `display_reachable`, imported from
   the tool; xdpyinfo only, no directory created): this shell has `DISPLAY=:1`,
   `XAUTHORITY=/run/user/1000/gdm/Xauthority` (mode 0700, owner fair, readable), `/usr/bin/xdpyinfo` present.
   Result: `display_problems -> []`, `display_reachable -> (True, 'xdpyinfo returncode 0')`. A normal launch from the
   user's desktop shell passes this check.
3. `df -B1 ~/motar_tmp`: 6,300,479,488 B free (6.30 GB).

## Disposition of r2 findings

| r2 | Status | Evidence |
| --- | --- | --- |
| MAJOR-1 non-finite | **Repaired** | `diff_stats` (T:1063) takes max over entries finite in both, counts finite/±inf/NaN, +inf/−inf and inf/NaN mismatches; equal inf/NaN not counted; None only on shape mismatch. `_payload_diff` marks incomparable only for a shape mismatch or missing array; the string marker is gone, so `_stats` no longer drops values. Hover packet uses the same function (max 0.0, mismatch 1, test t:785). Per-stage `nonfinite_mismatch_n/_steps/first_k/at_fixed_steps`, `incomparable_steps/_k/_at_fixed_steps`. Prereg §6 states the rule identically, including that a non-finite-only mismatch makes the step differ exactly but not "above tolerance". Tests cover flip, identical pattern, true shape mismatch, hover, and summary keeps n=1/max 0.25. |
| MINOR-1 question 2 | **Repaired; counting well-defined** | 6 boots/scene (3 OFF, 3 ON) give C(3,2)+C(3,2)=6 within pairs and 3×3=9 between pairs; `pair_kind` classifies by `contact`, so the split is unambiguous and `analyze` only pairs same-env boots. `question2` pools within_off+within_on (6) as min–max; counts between (9) below/inside(bounds inclusive)/above; same at each fixed step over pairs that reached it (NOT_AVAILABLE if no within or no between value); secondary = observation `first_k_above_tol`, None→inf. Prereg §6 (P:273ff) and §7 state exactly this; `analyze` limits repeat it. Positive control checks within_n 6, between_n 9, 5/3/1, range [0.01,0.02], step 0 and 10 readings, secondary, and `reading.json`. JSON of an inf bound is safe (`jsonable` → `{'nonfinite': 'inf'}`). |
| MINOR-2 est-pos stats | **Repaired** | `est_pos_dist_mean_m`, `est_pos_dist_at_fixed_steps_m` in `spread_summary`; listed in §6; tested (n 9; step 10 n 0). |
| MINOR-3 stamp offset | **Repaired** | §6 "Step-0 reference" discloses the carry and why no second anchor; `frame_age` and `sim_now_increment` reference-free readings; stamp µs max and `max_abs_target_count` separated; `absolute_t0_difference_s` summarised; `stamp_reference` text updated. Test with a 4 ms late step 0 shows carry in relative stamps, none in frame ages, increment 4000 µs at k 1. |
| MINOR-4 X preflight | **Repaired** | `display_problems` requires DISPLAY `:1`, XAUTHORITY set to a readable file, then `xdpyinfo -display :1` (10 s, minimal env with inherited DISPLAY/XAUTHORITY); OSError/timeout → refusal. It is inside `runtime_problems`, which `launch()` evaluates first; `S3C.launch_unit` raises `SystemExit("refused…")` before `attempt_dir.mkdir(parents=True)` (sitl_s3c.py:107-109). So a refusal creates no smoke directory and does not consume the single-use smoke. Live check on this host passes (item 2). Tests inject the probe, so PUBLIC_CPU does not depend on a live X server. §8 states XAUTHORITY is inherited and checked. |
| MINOR-5 long boot | **Repaired as disclosure + CPU test** | `test_long_boot_600_steps_…` drives a synthetic 600-step boot through real persistence, `checked_gzip` with a stub extractor, timeline, byte counter (single writer equals bytes on disk), `member_problems`, fixed step 599 comparison and a mid-persist OUTPUT_CAP refusal keeping the physical outcome. Disclosed in §3 (coverage limit, 0.72 s / 138 MB extractor timing), §7 may-not, `analyze` limits and `SCOPE` ("a short boot"). The live 600-step path remains first exercised in attempt01; that is an accepted, disclosed limit. |
| LOW limits | Repaired | 180 s vs 240 s and 8 GiB vs 12 GiB disclosed in §5. |
| LOW disk floor | Repaired, consistent | `LAUNCH_FREE_BYTES = 4<<30` (4.29 GB), checked at both `--smoke` and `--launch`. Budget: 6.30 GB now − 0.23 GB smoke worst case = 6.07 GB at attempt launch (1.78 GB above the floor); − 2.7 GB attempt worst case = 3.37 GB at the end, above the 1 GiB write floor. Expected (≈1.5 GB) leaves ≈4.5 GB. The floor is therefore consistent with smoke + 12 boots as long as nothing else consumes > ~1.7 GB on `/` before launch (shared 95 %-full disk; re-check `df` before each command). |
| LOW SCOPE | Repaired | `SCOPE` quoted verbatim in §8; `test_prereg_quotes_scope_and_names_xauthority` asserts `R.SCOPE in prereg`; the wording now matches (wait_heartbeat NOT_RECORDED; "first recorded divergence per stage and within/between-condition spread"; DISPLAY/XAUTHORITY; 4 GiB). |
| LOW others | Repaired | all `NATIVE_BOUND` required; seed binding and contact audit wrapped; state-row keys; review table. |

## New findings (none above LOW)

- **LOW-1 (wording):** §7 says "k of 9 … (and at the fixed steps …)". At a fixed step the denominator is the number of
  between pairs that reached it (code reports `between_n`), not 9; and in env 0 (≈45-step boots) most fixed steps
  ≥ 50 will be NOT_AVAILABLE. Report the actual `between_n`/`within_n` with every fixed-step reading. No code change.
- **LOW-2 (expected degeneracy, secondary):** the observation stage almost surely differs above 1e-6 at k = 0 (start
  states differ by cm), so the secondary reading will likely be "9 of 9 inside [0, 0]" (as in the positive control).
  It is informative only if some within pair stays equal past k 0. Do not over-read it.
- **LOW-3 (wording):** question 2 (§2) says "variation within one condition"; the reading pools both conditions'
  within pairs. §6 defines the pool explicitly, so this is consistent but the per-condition ranges are only in the
  secondary summaries (`within_off`, `within_on`).
- **LOW-4:** primary `truth_pos_dist_max_m` is over each pair's common steps, so a pair containing a short boot has a
  shorter window; `common_steps` is reported per pair. Disclosed in §6 wording; mention it when reporting env 0.
- **LOW-5:** five lines > 120 characters remain (T:1252, T:1262, t:587, t:595, t:624); harmless unless a style check
  in PUBLIC_CPU applies.
- **Note:** `DISPLAY :1` is hard-coded; after a reboot that renumbers displays the tool refuses (fail-closed, before
  any directory exists), and a prereg amendment would be needed. Acceptable.

## Must be fixed before commit

No code or design defect must be fixed. Process items that must precede the commit (the prereg is immutable after
it): update the prereg's "Re-review of the r2 repairs: pending" line to cite this r3 review and verdict; copy
REVIEW_r1/REPAIR_r1/REVIEW_r2/REPAIR_r2/REVIEW_r3 into the unit's evidence folder; add the `docs/EXPERIMENTS.md`
family entry (not present now; `test_experiment_index` needs it); run full PUBLIC_CPU (not run here); WORKLOG/Beads.
LOW-1 wording may optionally be folded into §7 in the same pre-commit edit.

VERDICT: PASS_WITH_LIMITS
