# Preregistration: CAM-S3 S3-W — S2b-FT continued with 90 deg/s, measured OAK-D depth, a sector reduction and thin bars (training only)

Written 2026-11-27 02:53 KST (draft), revised after reviews r1 (03:05) and r2 (03:15 KST), before any S3-W run. Branch `camonly/s2b` (no new branch, user 2026-11-26), base
`da5f07b`. Family `camera-only`. Predecessor: CAM-S2b (`docs/plans/prereg_camonly_s2b_2026-11-24.md`), whose S2b-FT
run (TRAINED, `~/motar_tmp/cams2b_20261124/train_ft/run_report.json`) is the source.

## Authorization (the user's words, 2026-11-27)

- The plan (2026-11-27 ~01:30, "CAM-S3"): stage 0 on the CPU, then S3-W (warm start) and S3-M (memory) later.
  Approved: "추천하는 방식대로 진행하자". Delay stays U{1,2}.
- The conditional GPU approval proposed at ~02:35 and accepted with "오케이 너가 말하는대로 할게": overnight, finish
  stage 0 and start S3-W training only if ALL hold: (1) this preregistration committed; (2) an independent Opus review
  with no open P1 (a P1 is fixed and re-reviewed to GO); (3) the full CPU test suite and the training smoke pass;
  (4) the host check (machine-check count, idle GPU). Scope named there: warm start from S2b-FT, yaw 90 deg/s, delay
  U{1,2}, measured masks + 20-row top band (R5), false near (R5b), an outlier-robust sector reduction shared by
  training and runtime (R27b), part of the 205 bars thin; GPU at most 6 h including evaluation. Not approved: S3-M on
  the GPU, push, merge. Record: `~/motar_tmp/cams3_20261127/user_conditional_approval.json`.
- **Departure from the approved scope, pending the user (review r2 P1-A):** this run trains WITHOUT false near (R5b),
  which the approval names (Rule 1, review r1 P1-1). Neither the smoke nor the run may start until the user has
  explicitly accepted S3-W without false near; that acceptance is recorded, with the user's words, in
  `~/motar_tmp/cams3_20261127/user_acceptance_false_near_off.json`, and the driver's preflight refuses without that file.
  If the user instead wants false near trained, this document is not used; a new preregistration follows.
- This document covers one development smoke (repeatable until one completes) and ONE S3-W run. The evaluation of S3-W
  is NOT in this document; it needs its own preregistration (the GPU budget above leaves about 1.4 h for it after a
  smoke and a run at its 4.5 h cap).

## Why (evidence on 2026-11-26/27)

- CAM-REF stage 1: the camera-only S2b-FT crashes into bars in about 21 % of episodes at every protocol point, which
  is above the 20 % bar. Stage 2: delay (+9.9 pp) and the 60 deg/s clip (+5.2 pp) carry Policy F's loss from P_T to
  P_S.
- The vehicle answer (2026-11-25) and the Orin latency report (2026-11-27, NOT_VERIFIED vehicle-side): capture to
  result p95 185 ms ≤ 0.19 s, so 90 deg/s is allowed. Delay U{1,2} (0.1-0.2 s) still covers it.
- The Jetson handheld recordings (2026-11-26, RULES.md amendment): invalid depth is clustered (68-91 % of invalid
  pixels in blobs > 1,000 px), the top 20 rows are invalid in every frame, and textureless surfaces give false near
  depth. Training used 2 % iid holes and no false near.
- Column sampling (CPU geometry, `~/motar_tmp/cams3_20261127/column_gap.py`): the camera-only obstacle scan reads one
  pixel column per 5-degree bin (13 of 312 columns; `navrl_perception.py:1909-1921`). In-FOV obstacles seen by those
  columns: simulated 0.4-0.8 m bars 100 / 99.4 / 83 % at 0-3 / 3-6 / 6-10 m; 0.25 m trunks 99.7 / 65.1 / 36.7 %.

## What changes (everything else is S2b-FT's run)

| Change | Value | Where |
| --- | --- | --- |
| Warm start | S2b-FT `last_gen_ppo_ep_15000_rew_51.018074.pth`, SHA-256 `f2c80fcc2b2eaeb24e0b05d2284e74594a7ed8e7953f6b17b7dce49b09f2bbab` (run `ppo_261125_1344_navrl_v2-cams2b-ft-yaw60-camonly-s1`, read-only; copied to `~/motar_tmp/cams3_20261127/source/`) | `tools/cams3_train.py`, launcher pin |
| Yaw-rate ceiling | `NAVRL_YAW_RATE_MAX` 1.0472 -> **1.5708** rad/s (90 deg/s), action scale and Lee clamp together (the S2b table of consumers applies unchanged) | `tools/cams3_python` |
| Bar pool | `NAVRL_BAR_POOL` bars_h3 -> **bars_h3_thin**: the 40 bars_h3 files (byte-identical) + 14 thin 3.0 m bars, sides U(0.20, 0.30) m (numpy seed 4999); each bar picks a file uniformly (`asset_loader.py:96`), so about 26 % of bars are thin. Same count (205 final), same placement rule (navrl_band). `env_object_config.py` gives the new pool the bars_h3 height ratio. | `resources/models/environment_assets/bars_h3_thin/`, `tools/cams3_python` |
| Depth holes | `NAVRL_DEPTH_HOLE_P` 0.02 -> **0** (replaced by the measured masks) | launcher |
| Measured depth | `NAVRL_DEPTH_MEAS=1` (navrl_cams3 rule 1): recorded invalid masks and a 10-row top band (20 rows of the 468-row recording); false near implemented but **OFF** (`NAVRL_DEPTH_MEAS_FALSE_NEAR=0`, review r1 P1-1) | `aerial_gym/task/navrl_task/navrl_cams3.py` |
| Sector reduction | `NAVRL_CAMS3_SECTOR_K=24` (navrl_cams3 rule 2) | same |
| Epochs | 15,001-18,000 (3,000), LR 7.5e-6 (the checkpoint's), Adam moments kept, seed 1 | driver |

Rule 1 (training only). The mask library `~/motar_tmp/cams3_20261127/mask_library_v1.npz`, SHA-256
`55c781136c3ce26635cab61d11c2e5fe4cbdcd28a7433cb6c315f858c603a351`, built by `tools/cams3_build_mask_library.py` from
the six recorded scenes (1,984 frames; inputs SHA-256-recorded in the file): 624 x 468 -> 312 x 234 with "invalid only
if the whole 2 x 2 block is invalid" (the runtime's min-valid projection keeps a pixel when any return lands in it);
frames whose invalid share below the band exceeds 0.5 dropped (135; the camera held against a near surface, which the
simulator produces from its own geometry through MinZ). Kept 1,849 frames; invalid share 14.7 % overall, 10.9 % below
the band; per scene below-band median 1.7-27.7 %. Each env holds a pointer into the library that advances one recorded
frame with probability 0.5 per step (5 fps recording, 10 Hz control) and jumps to a uniform frame with probability 0.01
per step or at its scene's end; seed 4943. Masked and top-band pixels read as no return (`max_camera_range`), the
depth model's encoding. The renderer's no-hit pixels pass through.

False near (R5b) is implemented (switch `NAVRL_DEPTH_MEAS_FALSE_NEAR`: each valid non-target return shortened with
probability P20(z), by U(0.3, 0.6) with probability P40(z), else U(0.6, 0.8); P40 = 0.29 / 0.024 / 0.011 and
P20 = 0.51 / 0.18 / 0.14 at z = 1 / 2 / 3 m, from one textureless grey wall, per pixel of the 624 x 468 recording) but is
**OFF in S3-W**. Review r1 P1-1 measured on the CPU (reviewer script `~/motar_tmp/cams3_20261127/review_r1/scripts/
false_near_vs_k.py`) that this iid model, applied to every simulated surface, shortens every bar inside 7 m to
0.33-0.61 of its range even after the k = 24 reduction (bars at 1 / 2 / 3 / 5 / 7 m read 0.33 / 0.75 / 1.35 / 2.70 /
4.27 m). A policy trained on that learns that an observed 1.3 m is a true 3 m, which is the unsafe direction when the
real surfaces (bark, the target drone) are textured and read nearly true. With false near OFF in training, real false
near can only make obstacles look nearer than trained, the conservative direction, with two caveats (review r2): a
policy that never saw false near may react to it in untrained ways, and a return shortened below MinZ (0.315 m, about 1 m
true range at the measured factors) vanishes instead of looking nearer. Training contains no near outliers at all, so
rule 2's k = 24 is exercised in training only by masked and blended pixels. Recorded as an open model item:
false near needs a texture-dependent, spatially correlated model (the recorded artefacts are horizontal stripes on
textureless surfaces) before it is trained on.

Rule 2 (training and runtime). For each 5-degree bin inside the FOV (13 bins), the value at the sampled column is
replaced by the 24th-smallest range over every pixel of every column whose pinhole bearing atan((cx - u)/fx) lies in
the bin (20-28 columns x 234 rows), on the same target-carved frame (`depth_no_target`), and written into every column
of that bin. Choice of k = 24 (review r1 P2-1 corrected): the thinnest simulated obstacle, a 0.20 m bar, is hit by a
render ray at every range up to about 7 m (1.65 deg ray spacing); at 7 m its 3 m height spans about 108 of 234 rows, so
after the bilinear upsampling it covers far more than 24 pixels at or near its range even with half of them masked.
Beyond 7 m a thin bar can fall between render rays (Limits), and at 10 m everything reads as no return anyway. In
flight, k = 24 ignores up to 23 isolated near pixels per bin (projection speckle, flying pixels at edges); it does not
remove a near bias spread over a whole surface (review r1 P1-1). Rows are not separated (the original column minimum
also takes every row), so the floor / ceiling handling is unchanged.

Consumers (review r1 P2-3): the riskcap speed governor reads the same fused scan
(`navrl_sensor_ablation.governor_scan`), so it sees the sector-reduced, masked depth too. A masked target pixel reads
as no return and fails the detector's depth gate (`navrl_perception.py:1601`), on top of the P9 detection-error
injector: target detections drop where the recordings had invalid depth, as they would in flight.

Both rules are installed at run time from `navrl_cams3.py` because `navrl_perception.py` and `navrl_depth_model.py` are
byte-pinned by the CAM-S2 flight-runtime contract (`TRAINING_SOURCE_PINS`); neither file changes, so S2b-FT's flight
runtime keeps working. Rule 2 replaces the module function `camera_no_return_to_lidar_range`, reads `depth_no_target`
from its caller's frame, refuses any caller other than `_fuse_static_and_extract_obstacles`, and checks for its first
50 calls that the column minima it receives equal `depth_no_target.amin(dim=1)`.

## Warm-start contract

`set_env_state` will warn (not refuse): `CONFIG MISMATCH | NAVRL_YAW_RATE_MAX: checkpoint trained with 1.047, running
with 1.571`; `ARENA MISMATCH | NAVRL_BAR_POOL: checkpoint=bars_h3 running=bars_h3_thin`; both reset the density evidence
(`NAVRL_RESET_DENSITY_WINDOW=1`, `NAVRL_DENSITY_RESUME_WARMUP=1000`, gate_not_before_step = 480000 + 32000). The
checkpoint restores 205 bars (the final density), task_steps 480000 and the k 20..28 window; no promotion is expected.
The observation layout (898-D) is unchanged; its values change (sector reduction, measured depth), which the policy
must adapt to. The driver stops the run (TRAINING_FAIL) unless the restore line, both warnings, the reset line, the
four `[cams3]` lines (install_measured, install_sector, meas_first, sector_first with k = 24) and first epoch 15,001
appear before the first epoch block is accepted.

## Receipt contract

The reference is S2b-FT's receipt `~/motar_tmp/cams2b_20261124/train_ft/receipt.json` (SHA-256
`3411d59cc1656a2b94cae97843ecd22c8d5d2d75ebe362def0b3887132f8365f`). The new receipt's `env_navrl` must equal it except
exactly: `NAVRL_YAW_RATE_MAX`, `NAVRL_BAR_POOL`, `NAVRL_DEPTH_HOLE_P`, the eight `NAVRL_DEPTH_MEAS*` keys (the switch
and seven settings, `NAVRL_DEPTH_MEAS_FALSE_NEAR=0`),
`NAVRL_CAMS3_SECTOR_K`, `NAVRL_REALVEH_RECEIPT` and `NAVRL_P9_ERROR_MODEL` (same file, this tree's path). The receipt
must carry the cams2, camsafe, cams3_meas and cams3_sector install records and no evaluation-only wrapper.

## Driver, stop rule and labels

`python3 tools/cams3_train.py --run w [--smoke]`; launcher `aerial_gym/rl_training/rl_games/train_navrl_cams3.sh`;
wrapper `tools/cams3_python`; unit folder `~/motar_tmp/cams3_20261127/{train_w, train_w_smoke_HHMMSS}`. It is
`tools/cams2b_train.py` (run ft) with these values; preflight, run-time checks, report and labels are S2b's: branch
`camonly/s2b`; this file committed at HEAD; no changes under `aerial_gym/`, `tools/`, `resources/`, `docs/plans/`; no
CUDA process except RustDesk; free disk ≥ 1.0 + 0.25 GB; reference receipt, source checkpoint and mask library at their
pinned SHA-256; runner cores 0-3,8-11,14-23; wall cap **4.5 h** (`timeout -k 120`; S2b-FT took 3.4 h for 3,000 epochs,
the two rules add GPU work per step, which the smoke measures). Smoke: 25 epochs, development only.

- Launch rule (review r1 P2-4): the run is started only if the smoke's projection reads `within_cap`: median logged
  step time over smoke epochs 6-25 x 3,000 epochs x 1.108 (S2b-FT's 3.40 h wall / 3.07 h of logged step time) ≤ 4.3 h,
  i.e. a median step time ≤ 4.66 s (S2b-FT's own smoke read 4.016 s -> 3.71 h). Recorded with the polled peak GPU memory
  (`speed`, `gpu_memory_peak_mib_polled` in the smoke's run_report.json). A smoke outside the rule ends the night with no
  run and a report.

- Stop rule: at epoch **16,000**, if the episode-weighted training-rollout capture of epochs 15,801-16,000 is below
  **0.30**, the run stops: STOPPED_NO_LEARNING. (S2b-FT's rollout capture at 205 bars was 0.503.)
- Labels: TRAINED, STOPPED_NO_LEARNING, WALL_CAP, TRAINING_FAIL (including the inherited same-density capture guard),
  VOID_HOST_FAULT; smoke SMOKE_COMPLETED / SMOKE_FAIL / VOID_HOST_FAULT.
- Recorded: `run_report.json` (as S2b), the `[cams3]` statistics lines (mask-invalid share, false-near share, jump and
  advance rates), the smoke's seconds per epoch.

Training-rollout rates are not evaluation results: the arena (26 % thin bars, fewer blocked metres) and the observation
differ from every earlier evaluation.

## What is compared later (its own preregistration)

S3-W final vs S2b-FT ep15000 at 205 bars under one protocol, both on the standard bars_h3 arena and on bars_h3_thin,
with the vehicle's 90 deg/s and delay U{1,2}; bar crash against the 20 % bar, capture against 70 %. Not in this file.

## Limits

- One seed; a continuation of the S2b lineage, not an independent replicate.
- Several changes at once (yaw, thin bars, two depth rules, sector rule): an improvement or a loss cannot be attributed
  to one of them. The user accepted this (2026-11-27 plan); factor isolation would need separate runs.
- The masks come from six indoor handheld scenes (no outdoor, no flight vibration); outdoor masks join the library later.
  They are placed on simulated scenes independently of the simulated geometry (an invalid blob can hide a bar that a
  real sensor would have seen, or the reverse).
- False near is implemented but not trained (Rule 1). Its implemented model (iid per pixel, one grey wall up to 3 m,
  constant beyond) is not a validated model of the real artefacts (horizontal stripes on textureless surfaces).
- The recording's colour-aligned field of view is taken as the trained 66.7 x 52.6 deg grid (not re-measured).
- Thin bars are vertical only; horizontal branches (R19) are not modelled. The render stays 40 x 24 rays (R20 not
  done): a 0.20 m bar is hit by a render ray only within about 7 m (1.65 deg spacing).
- False near is not trained (see Rule 1); the training observation is optimistic on textureless surfaces.
- Thin bars and the placement rule (review r1 P3-1): navrl_band's "touching means overlapping" assumes 0.4-0.8 m bars;
  with thin bars the reviewer's re-implementation found about 0.5 impassable slit per 205-bar layout (at most 0.07 m
  wide), none with bars_h3. Such a pair acts as one wider obstacle.
- An S3-W policy is not flyable until the flight runtime installs rule 2 and pins `navrl_cams3.py` for the new lineage
  (a later runtime unit; the rule's code is the same function on the CPU). Today the CAM-S2 contract refuses an S3-W
  receipt only because 1.5708 rad/s is not a known yaw variant, and it does not yet refuse or apply
  `NAVRL_CAMS3_SECTOR_K` (review r1 P3-3).
- The checkpoint's env_state does not record the measured-depth or sector settings; only the receipt does. The S3-W
  evaluation preregistration must bind the receipt and install both rules (review r1 P3-4).

## Review

Independent read-only review r1 (Claude Opus 5.5, fresh context, 2026-11-27; `~/motar_tmp/cams3_20261127/review_r1/
REVIEW.md`): **NO_GO** with one P1. P1-1 (the iid false-near model defeats the robust reduction and compresses every
range) is answered by switching false near OFF in S3-W (above). P2-1 (k rationale), P2-2 (pixel grid of the false-near
rates; moot while OFF, stated), P2-3 (consumers), P2-4 (launch rule), P3-1 (slits), P3-2 (time, key count), P3-3,
P3-4 (limits) and P3-5 (per-step host synchronisation in rule 2: index tensors now cached) are addressed in this file
and the code. Re-review r2 before commit.
