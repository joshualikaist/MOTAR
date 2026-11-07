# CAM-K1 v1 — camera-only continuation training: preparation record (2026-11-06/07)

**Status:** preregistered, reviewed and approved (user, 2026-11-07), **NOT_RUN**. Read the
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
