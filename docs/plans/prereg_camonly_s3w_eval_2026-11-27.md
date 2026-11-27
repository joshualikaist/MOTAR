# CAM-S3-E — S3-W against S2b-FT at P_S, each in its own runtime (preregistration)

- **Status:** preregistration, committed before any CAM-S3-E GPU cell (2026-11-27).
- **Branch:** camonly/s2b (no new branch). Unit folder `~/motar_tmp/cams3e_20261127/`.

## Authorization
- The user's conditional GPU approval of 2026-11-27 for CAM-S3 ("오케이 너가 말하는대로 할게"; record
  `~/motar_tmp/cams3_20261127/user_conditional_approval.json`) covers S3-W training **and its evaluation**, at most 6 h of
  GPU in total.
  - The user accepted training without false near ("오측정 없이 S3-W 진행",
    `docs/evidence/camonly_2026-11-04/cams3/user_acceptance_false_near_off.json`).
  - S3-W used about 3.6 h (smokes 4 min, run 3.53 h), so about 2.4 h remain. This unit's wall cap is 2.0 h.
- After the S3-W result, the assistant asked whether to write this evaluation's preregistration and have it reviewed
  ("평가 사전등록을 작성해서 리뷰를 받을까요?"). The user answered "우리 원래 하던거부터 진행하자" (2026-11-27).
- The GPU stages start only when all of these hold:
  - this file is committed;
  - an independent Opus review leaves no open P1;
  - the CPU suite passes;
  - the host check passes;
  - `~/motar_tmp/cams3e_20261127/execution_approval.json` names this scope (`tools/camref_stage3_run.py scope`) and HEAD.
- Not approved: S3-M GPU, any new training, push, merge.

## Question
S3-W (`docs/plans/prereg_camonly_s3w_2026-11-27.md`, TRAINED 2026-11-27, commit 7a2e9c8) is S2b-FT ep15000 continued
for 3,000 epochs with:
- 90 deg/s;
- thin bars;
- the measured OAK-D invalid masks;
- the camera sector reduction (k = 24).

Its training-rollout rates were flat (capture 0.61, crash 0.29-0.30). The step from S2b-FT's 0.51 / 0.40 appeared in the
first window, so it is the arena and observation, not learning.

The questions are:
1. Does S3-W, run deterministically at P_S in its own runtime, meet the user's bar: crash ≤ 20 %, capture ≥ 70 %,
   timeout ≤ 10 %?
2. Does it crash less than S2b-FT under one protocol?

This is a simulator measurement. It makes no flight claim.

## Cells
All cells use 205 bars, seeds 4501-4503, quota 8 episodes per env at 128 envs, and count the first 8 per env (1,024
episodes per cell). The protocol is P_S exactly as CAM-REF stage 1's S0 (`docs/plans/prereg_camonly_ref_matched_protocol_2026-11-26.md`):
- the CAM-DX1 V312 cell construction;
- the visible start (±28.35 deg, goal ≤ 12 m);
- the legacy mixed target motion with the D8 same-drone target;
- the CAM-S2 capture / contact rule (1 m, ≤ 1 m/s closing, 0.40 m contact);
- delay U{1,2};
- vehicle and gain randomisation;
- altitude 1.5 m, mount 0.08 m;
- governor FREE 2.5 m/s;
- camsafe, P9.

Each policy runs in its own trained runtime.

| Arm | Policy | Yaw scale | Depth | Sector rule | Arena |
| --- | --- | --- | --- | --- | --- |
| S0 (reference, not rerun) | S2b-FT ep15000 `f2c80fcc…` | 1.0472 | iid holes 0.02 | off (13 sampled columns) | bars_h3 |
| S0T | S2b-FT | 1.0472 | iid holes 0.02 | off | bars_h3_thin |
| S0M | S2b-FT | 1.0472 | measured masks + 10-row top band | off | bars_h3 |
| W | S3-W ep18000 `2626a4b2…` | 1.5708 | measured masks + 10-row top band | k = 24 | bars_h3 |
| WT | S3-W | 1.5708 | measured masks + 10-row top band | k = 24 | bars_h3_thin |

- **Measured masks** are navrl_cams3 rule 1 as S3-W trained it:
  - library `~/motar_tmp/cams3_20261127/mask_library_v1.npz`, SHA-256 `55c78113…`;
  - TOP_ROWS 10, ADVANCE_P 0.5, JUMP_P 0.01, false near 0, `NAVRL_DEPTH_HOLE_P=0`;
  - mask-pointer seed 10 × seed + 6, so the arms of a block share it.
- **S0** is the CAM-DX1 V312 205 / 4501-4503 cells (SHA-256 pinned in `tools/camref_eval_run.py` `REF_SHA`), as stage 1
  read it.
- **S0M** is the sensor-matched control: S2b-FT under the same mask model as W, without the sector rule. S2b-FT's
  runtime reads the sampled columns.
- **W** runs on the standard arena, **WT** on S3-W's training arena. W's environment is S0's plus exactly
  `s3_rung_delta`. Preflight binds those keys to S3-W's training receipt (SHA-256 `3ae14489…`). That receipt differs
  from S2b-FT's training environment in exactly the CAM-S3 keys, the receipt path and the P9 file path; the P9 SHA-256
  is equal. W keeps S0's P9 file path.

**Stages.** The chain and guards are stage 1's (`tools/camref_eval_run.py`), with this unit's constants installed by
`tools/camref_stage3_run.py`.
- **Smoke** (never counted): S0T, S0M, W, WT at 205 / 4501, quota 1. SMOKE_OK needs every cell OK with ≥ 1 finished
  episode. Every cell also passes the per-cell checks:
  - realveh receipt with the rung's cams3 installs and settings and the committed `navrl_cams3.py` bytes;
  - cams2 / camsafe / inview telemetry;
  - snapshots;
  - checkpoint-contract lines.

  In addition, each check below is a positive control in some arms and a negative control in the others:

  | Check | Present in | Absent in |
  | --- | --- | --- |
  | `[cams3] install_measured` (false near off) and `meas_first` over the 1,849-frame library, invalid share in (0, 1) | S0M, W, WT | S0T |
  | `[cams3] install_sector` and `sector_first` with k 24 over 13 bins | W, WT | S0T, S0M |

  - Yaw: the snapshot reads NAVRL_YAW_RATE_MAX 1.5708 in W / WT and 1.0472 in S0T / S0M. No cell has a NAVRL_YAW_RATE_MAX
    CONFIG MISMATCH line; the task prints one when the running scale differs from the checkpoint's.
  - ARENA line: S0T shows `checkpoint=bars_h3 running=bars_h3_thin`, W shows `checkpoint=bars_h3_thin running=bars_h3`,
    and S0M and WT show none. The snapshot pool is the rung's.
  - The rendered step-0 bearing is within ±28.35 deg in every cell.
- **Identity:** S0ID reruns S0 at 205 / 4501 on this commit. Its per-episode outcomes (first 8 per env) must equal
  CAM-DX1 V312 205 / 4501's; any difference is IDENTITY_FAIL and the chain stops. This shows that the code changed since
  stage 1 leaves S0 unchanged: the CAM-S3 module, the bars_h3_thin assets and the object-config height ratio of the new
  pool, the entry's S3W policy, and the runner's stage-3 rungs.
- **Eval:** S0T, S0M, W, WT × 4501-4503, block-major by seed (12 cells).
- **Wall cap 2.0 h** shared by smoke (sub-cap 0.4 h), identity (0.3 h) and eval (2.0 h). Stage 1's projection /
  BUDGET_SKIPPED rule applies: a cell whose projected wall time does not fit is skipped, never started. Expected about
  1.4 h: stage 1's S2b-FT cells took 3.9-5.5 min each (S0ID 327 s, S3 / S6 235-306 s, S6 smoke 130 s), and the
  sector rule adds GPU work per step.
- **Guards** (stage 1's, unchanged):
  - the approval file;
  - a clean tree on branch camonly/s2b;
  - committed files;
  - the launcher copy at its pin;
  - an idle host with one RTX 3070 and no other GPU compute;
  - host fault count, boot id and online CPUs unchanged per cell and stage;
  - the per-cell environment, receipt and telemetry checks.

  Added by this stage: S3-W's training checks (`s3w_problems`):
  - run report TRAINED at epoch 18000 on cf11845, with its final checkpoint, receipt and run report at their SHA-256
    pins;
  - the environment difference above;
  - the mask library and `navrl_cams3.py` bytes equal to training's.

## Readings (`tools/camref_stage3_analyse.py`)
Stage 1's machinery (`tools/camref_eval_analyse.py`) is reused: counts, crash classes from navrl_cams2diag (bar contact,
target contact, out of bounds), the paired seed-block differences and their intervals. The episode bootstrap draws
10,000 resamples from numpy `default_rng(4711)`, with cells in the order S0, S0T, S0M, W, WT × seeds. Only complete
seed blocks are read; with none complete the verdict is NOT_RUN.

- **Q1 (primary, labelled):** W against the bar, on the episode-weighted pooled rates of the complete blocks.
  - **BAR_MET** iff crash ≤ 20 %, capture ≥ 70 % and timeout ≤ 10 % (point estimates); otherwise **BAR_NOT_MET**.
  - Reported with the label: the bootstrap intervals, one-sided 95 % Wilson bounds of crash and bar-contact crash, and
    the crash classes.
- **Q2 (labelled):** W − S0 (bars_h3) and WT − S0T (bars_h3_thin), on bar-contact crash, crash (all) and capture.
  - MATERIAL: |difference| ≥ 5 pp and the bootstrap 95 % interval excludes 0.
  - SMALL: the interval excludes 0 and |difference| < 5 pp.
  - NO_DETECTED_CHANGE: otherwise.
- **Q3 (reported, unlabelled):**
  - S0M − S0: the mask model on S2b-FT.
  - W − S0M: policy + sector rule + 90 deg/s under the same sensor model.
  - WT, S0, S0T and S0M against the bar.
- **Use of the result (fixed):**
  - Q1 BAR_MET → the runtime unit is next (rule 2 + 90 deg/s in the flight runtime, pin `navrl_cams3.py` for the new
    lineage), followed by the Orin timing of the sector rule.
  - BAR_NOT_MET with Q2 W − S0 crash MATERIAL and negative → the CAM-S3 changes lower the crash, but not to the bar.
  - Otherwise → fine-tuning this lineage stops. The next design is structural (memory S3-M, or the learning rate), and
    the user chooses it.
- **Analysis controls** (run before the eval is read; a miss reads CONTROL_FAIL):

  | Synthetic case | Required result |
  | --- | --- |
  | Every cell equal (29 % crash) | Q2 all NO_DETECTED_CHANGE; pointer "structural" |
  | W crash −8 pp in every block (21 % crash) | W − S0 crash MATERIAL negative, WT − S0T NO_DETECTED_CHANGE; pointer "helps, not enough" |
  | W at 18 / 73 / 9 % (crash / capture / timeout) | BAR_MET, pointer "runtime next" |
  | W at 22 / 70 / 8 % | BAR_NOT_MET |
  | W at 17 / 71 / 12 % | BAR_NOT_MET |

## Code (committed with this file)
- `tools/camref_eval_run.py`
  - Stage-3 rungs: `S3_RUNGS`, `s3_rung_delta`, `meas_delta`, `CAMS3_POINT` / `cams3_problems`,
    `cams3_receipt_problems`, `s3_smoke_controls`, `MISMATCH_ARENA`.
  - The S3W policy (`POLICY_SHA`, `CAMERA_POLICIES`). Every check that applied to S2B applies to S3W.
  - The camera-rung sets for visible start, cams2 and the recorder now include the new rungs.
  - Every earlier arm is unchanged. `tests/test_cams3_eval.py` checks, for every arm and smoke arm of the runner at
    7a2e9c8, the rung deltas, built environments, environment checks, installs, telemetry expectations, mismatch
    allowances, launcher values and protocol points against that runner loaded from git.
- `tools/camref_entry.py`
  - Policy S3W (checkpoint `2626a4b2…`, realveh_entry delegate), which requires S2B's perception at 1.5708 rad/s,
    measured depth with false near 0 and holes 0, and sector k 24.
  - S2B now refuses the sector key; F and K1 refuse every CAM-S3 key.
- `aerial_gym/rl_training/rl_games/eval_navrl_cams3e_arms.sh`: a copy of the stage-1 launcher, which stays unchanged at
  its pin `51f949d0…`. Its only change is that the policy check also accepts S3W.
- `tools/camref_stage3_run.py`: the unit's constants, S3-W's checkpoint and training checks, and its code allowlist.
  The allowlist adds the CAM-S3 simulation files, the launcher copy, CAM-S3's training-only files, two flight-runtime
  files and the test runner (none of these is imported by the evaluation path).
- `tools/camref_stage3_analyse.py`, `tests/test_cams3_eval.py`.

## Limits
- **One training seed.** S3-W is a continuation of the S2b lineage, not an independent replicate. Several changes are
  confounded in W − S0: yaw, thin-bar training, the masks, the sector rule and 3,000 more epochs. S0M and WT / S0T
  separate only the mask model and the arena.
- **Stochastic vs deterministic.** Training rollouts were stochastic (fixed action std 0.35 / 0.08). This evaluation
  runs the policy's deterministic action, as every earlier evaluation did. Training-rollout and evaluation rates are not
  comparable (S2b-FT capture / crash: last training window 0.510 / 0.399; evaluation S0, CAM-REF stage 1, 0.704 / 0.285).
- **The measured masks are indoor only:** six handheld lab scenes from 2026-11-26. No outdoor, flight-vibration or
  forest masks. The masks are placed independently of the simulated geometry.
- **False near** is not modelled in any arm. Real false near on textureless surfaces is not represented.
- **Thin bars** are vertical only. A 0.20 m bar is hit by a render ray only within about 7 m.
- **Thresholds.** The bar reads point estimates. The 5 pp MATERIAL threshold is stage 1's.
- **S0 is not rerun.** Seeds 4502-4503 are CAM-DX1's cells, and identity is checked at 4501 only (as in stage 1).
- **Diagnostic yaw reference.** The cams2diag recorder's yaw reference (0.7854 rad/s, counts only) is S0's in every
  arm.
- **Not flyable.** S3-W cannot fly until the flight runtime installs rule 2 and accepts 1.5708 rad/s (S3-W preregistration,
  Limits).

## Review
Independent review (Claude Opus 5.5, fresh context, read-only) before any GPU stage; the report goes to
`~/motar_tmp/cams3e_20261127/review_r1/`. Findings and fixes are recorded in the WORKLOG and copied as
`docs/evidence/camonly_2026-11-04/cams3/REVIEW_E_r1.md`; this file is not edited after commit (amendments go in a
new, dated file).
