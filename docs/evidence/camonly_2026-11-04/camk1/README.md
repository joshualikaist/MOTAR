# CAM-K1 v1 — camera-only continuation training: preparation record (2026-11-06/07) and training runs (2026-11-07)

**Status:** seed 1 **TRAINED**, seed 2 **TRAINED** (2026-11-07). This is a training record only: no capture,
crash or transfer claim follows until CAM-K1-E. Read the
[preregistration](../../../plans/prereg_camonly_k1_train_v1_2026-11-06.md) and decision
[D-2026-11-06-1](../../../DECISIONS.md) first.

## What it does

This is a new policy lineage, continued from Frozen Policy F (ep25000, `f7022139…`). F stays frozen. The lineage keeps
GR-1 `ctrl`'s training contract (205 bars, LR 5e-6, the riskcap parameters, 128 envs) and changes only the
following:
- **Camera only:** `NAVRL_SENSOR_ABLATION=no_lidar`, the C-study switch, so neither perception nor the governor sees
  the LiDAR. The reward's `r_ss` keeps reading the simulator's LiDAR, which the observation contract allows.
- **Camera FOV:** 66° × 54°, the user's OAK-D Pro autofocus RGB camera. F's camera is 87° × 58°.
- **Reward weights:** `yaw_align_weight` 0.6 (F 0.3) and `visibility_bonus` 0.03 (F 0.02).
- **Length:** 3,000 epochs (25001..28000). A 2-epoch development smoke runs first, then seed 1 and seed 2.

The overrides are installed at runtime by `tools/camtrain_entry.py` through `tools/camtrain_python` for `runner.py`
only. The pinned files are unchanged.

## Independent review r1 (Claude Opus 5.5, fresh context)

[REVIEW_r1.md](REVIEW_r1.md) is a copy of `~/motar_tmp/camk1_20261106/review_r1/REVIEW.md`. It covered two parts.
- **Part 1 — the S3c/S4 runner repair** (dataman removal, the RustDesk allowance): **PASS**.
  - It is recorded in [the S3c/S4 amendment](../../../plans/prereg_sitl_s3c_s4_amendment1_2026-11-07.md).
  - LOW P1-3, a GPU check that would pass when `nvidia-smi` fails, is repaired.
- **Part 2 — CAM-K1:** the code and the preregistration **PASS_WITH_LIMITS**.
- **What the review verified:**
  - the override reaches the task: the registry passes the configuration class by reference, and every consumer reads
    the FOV when it is built;
  - the ablation behaves in training as in the C-study evaluation;
  - the launcher keeps GR-1 `ctrl`'s contract in both diff directions;
  - both preflights print PREFLIGHT PASS.
- **Findings repaired before commit:**
  - **MAJOR K-1:** the chain never ran the smoke. The smoke now runs before the SITL chain, and the chain refuses
    without SMOKE_OK.
  - **K-2:** one commit holds everything, and the approvals name it.
  - **K-3:** the run must show the task's own `hfov=66deg` startup line.
  - **K-4:** the smoke must write `ep_25002`.
  - **K-5:** the loiter argument is corrected with discounting.
  - **K-6:** the disclosures are added.
  - **K-7:** the index entry and D-2026-11-06-1 are added.
  - **K-8:** no training after a SITL host fault.
  - **K-9:** a fixed 24 h deadline.
  - **K-10:** inherited treatment variables are stripped.
  - **K-11:** a disk guard.
- **Open:** LOW K-12, a one-direction test of the contract. The reviewer's two-way diff was clean.

## Validation

- `tests/test_camk1_train.py`: 16 OK.
- Launcher preflight (`NAVRL_PREFLIGHT_ONLY=1`), seed 1 and smoke: `[v2-search] PREFLIGHT PASS`.
- PUBLIC_CPU (one thread, staged bytes): PASS, 3557 run, 0 failures, 0 errors, 10 skipped; kernel faults 0 → 0.
  A first run failed only the C0 import guard on the new opt-in entry. The guard's allow-list now names
  `tools/camtrain_entry.py`, with a positive control. Details in [`validation.json`](validation.json).

## Training runs (2026-11-07)

**Records** in this folder:
- [`train_report_s1.json`](train_report_s1.json) and [`train_report_s2.json`](train_report_s2.json): byte copies of
  each seed's report. They hold the verdict, the host before and after, the run folder, the checkpoint SHA-256 and
  copy, and the camtrain receipt.
- [`training_curve.json`](training_curve.json): per-100-epoch training-rollout statistics parsed from each seed's
  launcher log. GR-1 `ctrl` seed 1 (camera + LiDAR, 87°, F's weights, epochs 25001..26000) is included as a
  reference.
  - These are rollout statistics under PPO's exploration noise, as the runner prints them. They are not an
    evaluation.
  - Built by [`make_camk1_record.py`](make_camk1_record.py).

| Seed | Verdict | Wall time | Checkpoint (`last_gen_ppo_ep_28000_*`) SHA-256 | Host faults before → after |
| --- | --- | --- | --- | --- |
| 1 | TRAINED | 14,499 s (13:46:12–17:47:51) | `d7bf51a89b0e8124687e618cf8d95735713d36af2eb34fc5b8819efb14715beb` (`..._rew_-17.796482.pth`) | 0 → 0 |
| 2 | TRAINED | 14,364 s (17:47:51–21:47:15) | `5128bf2e0461fb45c6b8c4463d4d0a950bb88c0d28923c41d02d99da1102a4b7` (`..._rew_-10.653644.pth`) | 0 → 0 |

**Training-rollout statistics** (descriptive, `training_curve.json`):

| Window | Capture % | Crash % | Mean reward | Mean episode length |
| --- | --- | --- | --- | --- |
| seed 1, 25001–25100 | 8.45 | 91.55 | −49.85 | 62.5 |
| seed 1, 27901–28000 | 26.29 | 73.70 | −15.72 | 60.9 |
| seed 2, 25001–25100 | 7.79 | 92.20 | −50.57 | 61.0 |
| seed 2, 27901–28000 | 25.55 | 74.45 | −15.51 | 62.3 |
| GR-1 `ctrl` s1, 25001–25100 (reference) | 80.72 | 16.32 | +48.11 | 139.3 |
| GR-1 `ctrl` s1, 25901–26000 (reference) | 79.84 | 17.23 | +39.99 | 148.4 |

**What the curves show** (descriptive):
- Both seeds started near 8 % capture and 92 % crash. GR-1 `ctrl`, with the LiDAR and the 87° camera, stayed near
  80 % capture.
- In the last 100 epochs, capture reached 26.3 % (seed 1) and 25.6 % (seed 2); crash fell to 73.7 % and 74.5 %.
- Over the last 500 epochs, capture ranged 23.1–26.3 % (seed 1) and 25.6–26.1 % (seed 2). Whether more epochs would
  raise it is NOT_TESTED.
- Mean episode length first fell, to 43.9 and 43.0 steps around epoch 25600, then returned to about 61–62 steps.
- Mean reward rose from about −50 to about −15.5 and stayed negative; GR-1 `ctrl` ran at +40 to +48.
- Timeouts stayed at or below 0.03 %, so almost every episode ended in a capture or a crash.

**Execution:**
- **Chain:** `~/motar_tmp/camk1_20261106/run_camk1_chain.sh` at `5e58984`. Smoke SMOKE_OK at 11:32; seed 1 started
  at 13:46:12, after the SITL chain had ended (13:35:44).
- **Incident:** the chain's SITL-end check (`pgrep -f`) matched the parent launcher shell, whose command text named
  the SITL chain. That held the start for 8 minutes, until I ended that shell (my own process) at 13:44. The training
  itself is not affected.
- **Seeds:** seed 1 ran 13:46:12–17:47:51 and seed 2 17:47:51–21:47:15. Both preflights, receipts and FOV log checks
  passed, with no problems listed. Each took about 4.0 h, against the 3.3 h I had projected from the smoke.
- **Concurrency:** the runner was not pinned to cores. While the seeds trained, only reads and drafts ran outside the
  repository, plus:
  - one read-only Opus results review of S3c/S4 (13:53–14:35, `nice` 19);
  - my record-building scripts, a few seconds each at `nice` 10–19;
  - a free-disk logger, one `df` every 5 minutes.
- **Disk:** I first budgeted the checkpoints from GR-1's 1,000-epoch run. A 3,000-epoch seed writes about 0.54 GB, so
  seed 2's 2.0 GB preflight guard would have failed. The user approved deleting Chrome's disk caches at 14:47 (0.96 GB
  freed). Seed 2's preflight passed with 2.37 GB free; about 1.8 GB (1,722 MiB) was free at the end.
- **Kept outside Git:** the checkpoint copies and launcher logs in `~/motar_tmp/camk1_20261106/train_s{1,2}/`, and
  the run folders in the worktree's ignored `runs/`.
