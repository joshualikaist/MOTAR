# CAM-S3 S3-W package — independent review r2

- Reviewer: Claude Code subagent, model Opus 5.5 (claude-opus-5-5), fresh context, read-only on the repository.
- Date: 2026-11-27, ~03:10-03:40 KST. The system clock read 03:11 KST during this review.
- Scope: the uncommitted working tree of `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly-s2b` (branch
  `camonly/s2b`, base `da5f07b`), compared with r1 (`~/motar_tmp/cams3_20261127/review_r1/REVIEW.md`). Files changed
  after r1 (by mtime): `navrl_cams3.py` (03:07), `train_navrl_cams3.sh` (03:07), `tools/cams3_train.py` (03:07),
  `tests/test_cams3.py` (03:07), the preregistration (03:08), and the WORKLOG addendum.
- Severity: P1 = must fix before a GPU run, as in r1.

## Verdict: NO_GO

The technical part is GO. P1-1 is resolved in code on every path, the P3-5 cache is correct and leaves no per-step
host sync, the P2-4 launch rule is correctly implemented and calibrated, and I found no new technical P1.

One P1 remains, and it is an authorization gap, not a code defect (P1-A). The user's conditional approval names
false near (R5b) in the S3-W scope. The run now trains without it, and the preregistration does not say that the run
departs from the approved scope. AGENTS.md:41-42 says that anything the approval does not name needs a new approval.

How this becomes GO:
1. The user explicitly accepts S3-W without trained false near.
2. That acceptance is recorded, and the preregistration's Authorization section states the departure.
3. The P2 and P3 wording fixes below are made before the commit.

No further code change is needed. A re-review only has to confirm the acceptance record and the text edits.

## Findings

### P1-A — The run departs from the approved scope (false near R5b), and this is unstated (authorization)

- `~/motar_tmp/cams3_20261127/user_conditional_approval.json` `scope`: "measured depth masks + 20-row top band (R5),
  **false-near (R5b)**, sector reduction robust to outliers (R27b) identical in training and runtime".
- The preregistration repeats that scope at `prereg:15` without qualification. It switches false near OFF at
  `prereg:60-67` and `train_navrl_cams3.sh:158`. The text never says that the trained configuration is therefore not
  the approved one. The Review section (`prereg:171`) treats the change as a P1 fix.
- AGENTS.md:28-29 says "Never infer approval from an earlier approval". AGENTS.md:41-42 says "Approvals are single-use
  and name their scope ... anything it does not name needs a new approval".
- r1 already applied this standard. Its fix option 3, which departed from the approved "robust to outliers" wording,
  "needs the user's explicit acceptance". Dropping a named treatment component departs from the approved scope at
  least as much.
- The approval's condition (2), "P1 fixed then re-review GO", lets the author fix defects. It does not by itself
  approve a smaller treatment. This is a reserved decision for the user, not for the author or a reviewer.
- Fix: one explicit user acceptance, recorded next to the approval file, plus a sentence in the Authorization section
  (`prereg:12-17`), for example: "Departure: false near (R5b) is implemented but not trained (review r1 P1-1); accepted by
  the user on <date/time>, record <path>".

### Technical answer to P1-1 — resolved

Verified on every path:
- **Launcher.** `train_navrl_cams3.sh:158` reads `export NAVRL_DEPTH_MEAS_FALSE_NEAR=0     # review r1 ...`. Bash
  treats `#` after whitespace as a comment. Checked with bash: the exported value is exactly `0` (`od -c` shows `=0\n`).
- **Settings.** `navrl_cams3.meas_settings_from_env` (`navrl_cams3.py:120-123`) strips the value, accepts `"0"` and
  stores int 0. `realveh_entry.settings` passes it through (`realveh_entry.py:177-190`). `apply_measured` skips the
  whole false-near branch when the value is 0 (`navrl_cams3.py:225`). No segmenter call or RNG draws are made then.
- **No overrides.** Neither `train_navrl_v2_search.sh`, `train_navrl.sh` nor `realveh_python` touches
  `NAVRL_DEPTH_MEAS*`. The driver's `clean_env` strips any inherited `NAVRL_*` before the launcher sets it.
- **Receipt.** `cams3_train.expected_env` expects `"NAVRL_DEPTH_MEAS_FALSE_NEAR": "0"` (`cams3_train.py:238`).
  `receipt.env_navrl` is the literal `NAVRL_*` environment (`realveh_entry.py:293`). Any other value stops the run
  (TRAINING_FAIL).
- **Tests.** `tests/test_cams3.py:502-504` checks the launcher line and `expected_env`. The off branch is tested to
  change nothing outside the masks (`:368-369`).

Reviewer re-run of r1's `false_near_vs_k.py` idea with false near OFF. Script: `scripts/r2_checks.py`, output
`scripts/r2_checks.out`. The scene is a 40x24 render bilinearly upsampled to 312x234, 64 envs, with and without the
real mask library (`mask_library_v1.npz`, random frames, plus the 10-row top band). Median of the bin's reading:

| bar side, bearing | 1 m | 2 m | 3 m | 5 m | 7 m | 9 m |
|---|---|---|---|---|---|---|
| 0.20 m on-axis, k=24 | 1.000 | 2.001 | 3.001 | 5.002 | no render hit | no render hit |
| 0.20 m 1.5 deg, k=24 | 1.000 | 2.001 | 3.002 | 5.003 | 7.163 (rendered 7.156) | no render hit |
| 0.20 m 1.5 deg, native sampled column | 1.000 | 2.001 | 6.052 | 7.18 | 8.309 | — |
| 0.25 m 1.5 deg, k=24 | 1.000 | 2.001 | 3.001 | 5.003 | 7.163 | 9.063 (rendered 9.054) |

- Seen share: k = 24 is identical to k = 1 in every case (1.0 whenever a render ray hits the bar).
- Range bias of k = 24 against the nearest rendered return is ≤ +0.009 m.
- With the masks and top band, every median and seen share is the same as without them.
- So without false near, k = 24 behaves as the preregistration says. The k rationale (`prereg:73-79`) matches this,
  including "a 0.20 m bar is hit within about 7 m": it is missed on-axis at 7 m and kept 1.5 deg off-axis.

The rationale is sound in its main direction. Untrained real false near only shortens existing returns. r1 showed
that it cannot create an obstacle in empty space, so in flight obstacles look nearer than trained, which is the
conservative direction for collisions. Two caveats are not stated (P3-B):
- A policy that never saw near outliers is out of distribution when a whole textureless surface reads at 0.3-0.6 of
  its range. The effects are phantom-like evasions, governor speed caps and capture loss, not only "more caution".
- At ≤ about 1.05 m, a 0.3x shortening falls below MinZ 0.315 m. The real sensor then reports no return, so near
  pixels vanish. That is the non-conservative direction, though k = 24 over thousands of pixels makes it matter only
  when most of the bin vanishes.

The remaining R27b claim is now honestly scoped: "in flight, k = 24 ignores up to 23 isolated near pixels per bin
... it does not remove a near bias spread over a whole surface" (`prereg:77-79`). One addition belongs there: training
itself contains no outliers (holes = 0, false near off, the depth model adds only disparity noise). In training,
k = 24 differs from k = 1 only by ignoring genuine features smaller than 24 pixels per bin, and none of those occurred
in my thin-bar check. The robustness is a flight-side property that S3-W does not exercise.

### P3-5 response — correct, no remaining per-step host sync

- `sector_columns` (`navrl_cams3.py:299-335`) builds once per perception instance and device. It returns the 6-tuple
  `(cam_u, columns, valid, flat_columns, flat_bins, min_columns)`. Its only caller is `sector_values` (`:342`); the
  replacement reads `[3:5]` and `[2]` (`:376`, `:382`). The cache is reused on every later call (checked: identity
  on the second call).
- Device key (`:305`): `torch.device("cuda") != torch.device("cuda:0")`. The key is always `depth_no_target.device`,
  which is indexed, so the comparison is between indexed devices and is correct. Passing an unindexed `"cuda"`
  directly would only rebuild the cache, never use a wrong one.
- Flat indices with the real geometry (13 bins; 28, 25, 23, 22, 22, 21, 20, 21, 22, 22, 23, 25, 28 columns):
  - 302 flat columns, 0 duplicates. `index_put` is therefore deterministic on CUDA.
  - No gap inside the in-FOV span.
  - `min_columns` = 20.
- The cached gather and write equals a per-bin reference implementation exactly (`torch.equal` on random frames):
  `sector_values` against a per-bin `kthvalue`, and the flat write against a per-bin column write
  (`scripts/r2_checks.out`, block A).
- Remaining syncs per call:
  - `torch.equal` only for the first 50 calls (`:373`);
  - the `.tolist()` of the `sector_first` log only on call 1;
  - rule 1's statistics `int(...)` only for the first 200 calls (`:240-265`).
  After that, rule 1 and rule 2 issue no host syncs: gathers, `torch.where`, `kthvalue` and long-index `index_put`.
  The native fusion's own boolean-mask `scan[:, :, inside]` sync (`navrl_perception.py:1920`) is pre-existing.
- `tests/test_cams3.py` still covers rule 2 through the cached path. Each chain calls `observe` four times on one
  perception (thin, wide, target, grey) plus the guarded direct calls (`:125-175`), and asserts
  `p._cams3_sector_cols[2]`/`[0]` (`:151-152`). The cache's device branch is CPU-only in the tests (no CUDA), which is
  acceptable.

### P2-4 response — correct and calibrated

- **Regex.** `STEP_TIME` (`cams3_train.py:111`) on the real S2b-FT smoke log
  (`~/motar_tmp/cams2b_20261124/train_ft_smoke_134159/launcher.log`) matches 25 of 25 epoch blocks, with no
  duplicates. `speed_projection` gives a median of 4.0156 s over epochs 6-25 (20 values), projected 3.708 h,
  `within_cap` true. That equals the preregistration's "4.016 s -> 3.71 h". On the full S2b-FT log it matches 3,000
  of 3,000 blocks.
- **Factor 1.108.** From `train_ft/run_report.json`, started 13:44:30 and ended 17:08:45, so the wall time is
  3.4042 h. The sum of logged step times is 3.0697 h. The ratio is 1.109, and 3.40 / 3.07 = 1.108, which is correct.
  The rule applies the factor to a median rather than a mean: the run's wall / (median x 3000) is 1.113, 0.4 % higher.
  That is negligible against the smoke's own overestimate: the S2b-FT smoke median of 4.016 s was 9.4 % above that
  run's median of 3.670 s. The rule is conservative.
- **Threshold.** 4.3 h ↔ 4.657 s, matching the preregistration's "≤ 4.66 s".
- **Recording.** The report records `speed` and `gpu_memory_peak_mib_polled` for smoke and run alike
  (`cams3_train.py:581`). The test `Speed` (`tests/test_cams3.py:508-519`) covers within, outside and too-short cases.
- Minor (P3-C): `gpu_memory_used_mib` (`cams3_train.py:198-204`) calls `nvidia-smi` in the watcher thread without a
  timeout. A hang would stall the receipt and start checks and the stop rule, and the post-run re-check only labels the
  run afterwards. An exception other than ValueError/IndexError would end the watcher (recorded as
  `watcher_error`), with the same effect. Preflight guarantees that `nvidia-smi` exists, so only a hang is plausible.
  Suggest `timeout=10` and catching `(OSError, subprocess.SubprocessError)`. The launch rule itself is a manual gate:
  no code makes `--run w` refuse without a `within_cap` smoke. It is preregistered, so it is acceptable, but the
  decision should be recorded.

### Other r1 items

| r1 | Status |
|---|---|
| P2-1 k rationale | Addressed (`prereg:73-79`). The rationale now reaches only reachable cases and is confirmed by the table above. |
| P2-2 false-near grid | Stated ("per pixel of the 624 x 468 recording", `prereg:60`) and moot while OFF. |
| P2-3 consumers | Addressed (`prereg:81-85`): governor scan, detector depth gate, P9 on top. |
| P2-4 launch rule | Addressed (above). |
| P3-1 slits | Stated in Limits. |
| P3-2 time / key count | Key count fixed ("eight", `prereg:107`). **Time still wrong**: see P3-A. |
| P3-3 runtime contract | Stated in Limits. |
| P3-4 receipt-only settings | Stated in Limits. |
| P3-5 cache | Addressed (above). |

### P2-B — The experiment index still says false near is trained

`docs/EXPERIMENTS.md:830`: "measured OAK-D invalid masks + top band + false near replacing iid holes". This
contradicts the preregistration and the WORKLOG addendum. EXPERIMENTS.md is the record index, so fix the line before
the commit. For example: "... + top band (false near implemented, OFF) replacing iid holes".

### P3-A — The revision timestamp is in the future

`prereg:3` says "revised after review r1 (~04:30 KST)". The file's mtime is 03:08:08 KST, and the system clock read
03:11 KST while I reviewed it. This is the same class of error as r1 P3-2. A committed preregistration is frozen, so
correct it before the commit (≈03:08 KST).

### P3-B — Stale or contradictory wording around false near

- `prereg:151-152` (Limits) still calls the constant-beyond-3-m profile "conservative: more near outliers than
  measured at range". That is the direction argument r1 P1-1 refuted and the author accepted. The bullet now
  describes an unused model, so qualify it ("if switched on") or drop "conservative".
- `prereg:66` "the conservative direction": add the two caveats above, the out-of-distribution response and
  below-MinZ dropout at ≤ ~1 m.
- `train_navrl_cams3.sh:6-7` header comment: "rule 1: recorded OAK-D invalid masks, top band, false near" should say
  that false near is off.
- The `install_measured` log line (`navrl_cams3.py:294`) prints the false-near profile even when it is off. The
  settings field shows 0, so a reader can tell, but `false_near_active: bool(...)` would avoid misreading.
- WORKLOG addendum: "18 tests"; there are now 19.

### P3-D — GPU budget arithmetic

`prereg:19` says the 6 h budget "leaves at most 1.5 h" for the evaluation. With the 4.5 h wall cap plus the smoke
(about 0.1 h, cap 0.5 h), the remainder is about 1.4 h or less if the run reaches its cap. The launch rule makes a
run longer than about 4.3 h unlikely. State it as "about 1.4 h if the run hits the cap".

## New P1 search — none found (technical)

I checked the changed code paths:
- sector cache, flat write and `[3:5]` slicing;
- false-near switch parsing;
- receipt and start checks;
- speed and memory polling in the watcher;
- the launcher's exports and the order of its unsets;
- `cams3_python`'s guards;
- the unchanged `realveh_entry.py` install order (inner measured wrapper, then the depth model, then the ablation,
  then sector, then camsafe).

No defect reaches P1. The unchanged parts (mask builder, thin pool, `env_object_config` ratio) match r1's verification.

## What I ran (CPU only; `CUDA_VISIBLE_DEVICES=""`, `nice -n 10 taskset -c 6,7,12,13`, `PYTHONNOUSERSITE=1`)

- `python -B -m unittest -v test_cams3` (from `tests/`): **19 tests OK** in 1.9 s. Log: `test_cams3_run.log`.
- `scripts/r2_checks.py` → `scripts/r2_checks.out`: cache correctness (block A), and thin-bar retention and bias with
  false near OFF, with and without the real masks (block B).
- `speed_check.py` → `speed_check.out`: regex against the S2b-FT smoke and run logs, and the 1.108 factor.
- Bash parse of the commented export line.
- Read-only: `git status` and `git diff`, the preregistration, the approval JSON, AGENTS.md, the S2b-FT run report and
  logs. Not run: the full suite (the lead's), any GPU code, the smoke.
