# CAM-S3-E review r1 — commit 8fbf950 (camonly/s2b), read-only, fresh context

Reviewer: Claude Opus 5.5 (`claude-opus-5-5`) subagent, 2026-11-27. No GPU, no repository writes. Probes in this folder
(`probe_envs.py`, `probe_analysis.py`, `ckpt_keys.py`, `ckpt_diff.py`, `check_review.json`). Text saved by the main
session from the reviewer's final message (its file write was refused).

## Bottom line
No P1 in the code: arm environments, rule liveness, identity, allowlist, launcher copy and analysis mechanics are
correct. One P1 on authorization (needs one recorded user confirmation, no code). Four P2 on the fixed decision rules
and the budget, which can only be fixed before the GPU run (dated amendment + new commit).

## Verified
- tests/test_cams3_eval.py 26/26 OK (python -B); with test_camref, test_camref_stage2, test_cams3, test_realveh 150/150 OK.
- Runner check mode at 8fbf950: exit 0, preflight_problems empty; GPU-stage state only "no approval file"; idle-host
  passed; scope SHA-256 3bfb10f1...
- Full suite 9 F / 11 E; vs ~/motar_tmp/cams3_20261127/tests_full_pre.log exactly as the WORKLOG says (four gone, three
  new MKL_NUM_THREADS errors).
- Arm environments (probe_envs.py, seeds 4501-4503) differ from S0ID's in exactly the intended keys: S0T only
  NAVRL_BAR_POOL; S0M the nine measured-depth keys (holes 0.02 -> 0, mask seed 10*seed+6); W = S0M keys + sector k 24 +
  yaw 1.0472 -> 1.5708 + NAVRL_CAMREF_POLICY S3W; WT = W + thin pool.
- W vs S3-W training env: same differing keys as S0ID vs S2b-FT training env, plus per-cell mask seed and P9 path
  (P9 SHA-256 equal).
- Checkpoint state: S2b-FT vs S3-W differ only in cfg_bar_pool, cfg_yaw_rate_max and density/step counters, so the
  expected ARENA lines appear exactly in S0T and W; no yaw CONFIG MISMATCH in any arm.
- No simulation code changed after cf11845 (realveh_entry.py, navrl_cams3.py, env_object_config.py, thin assets equal).
- Training-log telemetry matches what the smoke controls require (install_measured false near off, meas_first
  library_frames 1849 invalid 0.10, sector_first k 24 over 13 bins).
- Positive/negative controls present for every check (measured depth S0M/W/WT vs S0T; sector W/WT vs S0T/S0M; yaw
  1.5708 W/WT vs 1.0472 S0T/S0M; ARENA line S0T/W vs S0M/WT).
- Launcher copy: 4-line header + policy check (2 lines -> 3); stage-1 launcher still 51f949d0.
- S0 references load at pins; reference_problems() empty; pooled capture 0.704, crash 0.285.
- S0ID identity valid. Allowlisted off-path files are not imported by aerial_gym, realveh_entry, camref_entry or wrappers.
- Budget: 2.0 h cap correct against ~2.4 h remaining (S3-W used 1 + 3 min smoke + 3.53 h run).

## P1
**P1-1 Authorization record does not support this GPU scope and contradicts the package's previous record.**
Prereg lines 5-20 (13-14); WORKLOG.md:27409-27411 (7a2e9c8 "Next" says the eval prereg "needs the user's approval of the
preregistration"); prereg_camonly_s3w_2026-11-27.md:23-25, 147-150; AGENTS.md 28-32, 41-42, 63-64. The reply
"우리 원래 하던거부터 진행하자" answered "평가 사전등록을 작성해서 리뷰를 받을까요?" and at most approves writing and
reviewing. The user has not seen the five-arm design (incl. S0M; S2b-FT at 60 deg/s rather than the "vehicle's 90 deg/s"
sketched in the S3-W prereg). Fix: get the user's explicit go for this scope (scope SHA 3bfb10f128df..., ~1.4 h
expected, 2.0 h cap) and record their words, or their explicit confirmation that the 6 h approval covers it; note it in
a dated amendment or the WORKLOG.

## P2
**P2-1 "Helps, not enough" pointer reads crash alone** (camref_stage3_analyse.py:91; prereg 143). probe case 2: W 53 %
capture / 21 % crash vs S0 65 / 29 -> crash -8.0 pp MATERIAL and capture -12.0 pp MATERIAL, pointer HELPS_NOT_ENOUGH.
A more passive policy reads as improvement. Fix: require W - S0 capture not MATERIAL negative, else e.g.
CRASH_DOWN_CAPTURE_DOWN -> structural; add the control.

**P2-2 W - S0 confounds the sensor model; fixed pointer ignores S0M** (prereg 37, 54, 140-146; analyse 45-46, 83-96).
probe case 3: S0M 40 %, W 27 % -> W - S0 -2.1 pp NO_DETECTED_CHANGE -> STRUCTURAL_NEXT, while W - S0M = -13.1 pp. Reverse
error possible. Departure from the S3-W prereg's "with the vehicle's 90 deg/s" not acknowledged. Fix: when S0M - S0
crash is MATERIAL, the pointer carries W - S0M with its interval and STRUCTURAL_NEXT is marked "sensor-model
confounded" (or W - S0M co-primary Q2); state and justify the 60 deg/s departure (each policy its own action scale).

**P2-3 BAR_MET on point estimates is more lenient than stage 1's rule** (analyse 67-69; prereg 129, 187; stage-1 R3
128-132). probe case 1: crash 19.6 %, capture 70.5 %, timeout 9.9 % -> BAR_MET / RUNTIME_NEXT though one-sided Wilson upper
20.8 % (stage 1 bar_label UNRESOLVED). Fix: report A.bar_label for all crashes and bar-contact crashes next to Q1;
qualify RUNTIME_NEXT when not BELOW_20, or rename BAR_MET_POINT.

**P2-4 2.0 h cap is per output parent, not per unit** (camref_eval_run.py:1325, 1352-1355; camref_stage3_run.py:50-51).
A rerun under a new parent starts with a full 2.0 h; an exception writes no report. Fix: unit-wide ledger
min(sub-cap, 2.4 h - sum wall_s of every *_report.json under UNIT_DIR + orphan cell dirs), or state that any rerun needs
the user's approval with the remaining hours named.

## P3
- P3-1 analysis does not refuse VOID_HOST_FAULT eval/identity verdicts (analyse 146-164).
- P3-2 per_cell summarises incomplete blocks (analyse 79); restrict to blocks.
- P3-3 use_of_result returns STRUCTURAL_NEXT for NOT_RUN (analyse 83-96; test asserts it); return NOT_RUN.
- P3-4 sector liveness checked in smoke only (camref_eval_run.py:949); check sector_first per eval cell (W/WT only).
- P3-5 tests/test_cams3_eval.py:41-49 _old_runner rmdir fails without -B (pycache); use TemporaryDirectory/rmtree.
- P3-6 prereg line 18 "the CPU suite passes" overstates (9 F / 11 E baseline); reword.
- P3-7 overclaims: line 37 "under one protocol" -> "one task protocol, each in its own runtime"; line 143 "the CAM-S3
  changes lower the crash" -> "W crashes less than S0 (yaw, arena, masks, sector rule and 3,000 epochs confounded)".
- P3-8 no filter intervention share reported (user rule); report camsafe and governor intervention shares W vs S0, S0M.
- P3-9 analysis binds only its own file to the eval commit; also bind camref_eval_analyse.py and camref_stage3_run.py.
- P3-10 stale launcher comments lines 11, 16, 119.

## Verdict
NO_GO as committed, solely on P1-1 (authorization); code GO_WITH_FIXES. Fix P2-1..P2-3 before data in a dated amendment +
code with new analysis controls; P2-4 by ledger or stated rerun rule; P3 fix or record; re-review the fix diff only.
Model: Claude Opus 5.5 (claude-opus-5-5).
