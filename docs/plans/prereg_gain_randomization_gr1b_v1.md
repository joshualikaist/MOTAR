# Preregistration: control-gain randomization, GR-1b — GR-1 with a second training seed

Fixed 2026-11-05, before any GR-1b training or evaluation. Branch `gainrand/dev`. Decision D-2026-11-04-1 (new PPO
allowed again). User approval of the GPU session (2026-11-05: "이거 하고 gpu 세션 바로 시작하자"), whose queue
(Beads `motar-g9a`, G5) names GR-1b.

**Entry condition.** GR-1 ([`prereg_gain_randomization_gr1_v1.md`](prereg_gain_randomization_gr1_v1.md) and its
amendment 1) read **GR1_SUPPORTED** on one training seed per arm; its independent review was PASS (`6b3ac27`).
- Amendment 1 fixed this step in advance: if the reading is GR1_SUPPORTED, a confirmation with a second training seed
  per arm is proposed as a new preregistration.
- The review's limits:
  - M2: the intervals cover evaluation noise only. `ctrl` drifted from F by +0.54 pp crash at S40, of the order of
    P1's 0.71 pp margin.
  - M1: the DiD effect is unresolved on a relative scale.

## Training (two arms, `tools/gr1b_train.py` → `aerial_gym/rl_training/rl_games/train_navrl_gr1b.sh`)

- **Arms:** `rand40` and `ctrl`.
- **What is copied from GR-1:**
  - `train_navrl_gr1b.sh` is a copy of `train_navrl_gr1.sh`. Only these change: `SEED=2`,
    `NAVRL_GAIN_RAND_SEED=2` (rand40), the run tag suffix `-s2`, the session-log prefix `gr1b_`, and the messages.
  - `tools/gr1b_train.py` is a copy of `tools/gr1_train.py` with the prereg path, launcher, tag and log glob changed.
  - Both diffs are in the commit message.
- **Contract (unchanged):** Policy F `f7022139…` continued for 1,000 epochs (25001 → 26000) at fixed 205 bars,
  curriculum off, LR 5e-6, riskcap governor parameters from the validated screen summary, main profile, 128 envs.
  `rand40` uses scale 0.4.
- **Order:** `rand40`, then `ctrl`, one after the other, on one commit, each with a 3 h cap.
- **Checks (`gr1b_train.py`, as GR-1):**
  - fail-closed preflight;
  - host state unchanged;
  - exactly one new run folder with the `-s2` tag;
  - `last_gen_ppo_ep_26000_*.pth` copied to the output with its SHA-256;
  - the tree is still clean.

## Evaluation (`tools/gr_run.py`, unchanged)

- **`gr1repro` on the GR-1b commit:** Policy F reruns GR-0's S40 / 205 / 4501 cell through both launchers. It must
  read GR1_REPRO_PASS; otherwise the unit stops.
- **`gr1eval` for each trained arm** (`--train-report` of the GR-1b training): GR-0's grid. That is E2, A3, 70/115/160/
  205 bars, seeds 4501-4503, levels S00 / S20 / S40, quota 16 and 36 cells per arm. The checkpoint SHA is bound in
  every receipt.
- **Outputs:** `results/gain_randomization_gr1b_v1/{train_rand40_s2,train_ctrl_s2,repro,eval_rand40_s2,eval_ctrl_s2}`.

## Analysis (fixed here)

- **Pair reading:** `tools/gr1_analyse.py`, unchanged (GR-1 amendment 1's rules and bootstrap), on
  (GR-0, `eval_rand40_s2`, `eval_ctrl_s2`). It gives the seed-2 pair's own reading: GR1_SUPPORTED,
  GR1_NOT_SUPPORTED, GR1_MATERIAL_LOSS or INCONCLUSIVE.
- **Confirmation reading:**

  | Reading | Rule |
  | --- | --- |
  | **CONFIRMED** | the seed-2 pair reads GR1_SUPPORTED |
  | **NOT_CONFIRMED** | the seed-2 pair reads GR1_NOT_SUPPORTED or GR1_MATERIAL_LOSS |
  | **MIXED** | otherwise (INCONCLUSIVE) |

- **Reported, not gated:**
  - P1, P2, P3 and DiD for each pair, their mean over the two pairs, and the between-pair difference. Two runs per
    arm give no useful training-variance interval, so none is claimed.
  - The log-ratio DiD (the GR-1 review's relative scale).
  - `ctrl_s2 − F` and `rand40_s2 − F` at each level.
  - Crash causes.
- **Labels:** they stay as computed until an independent results review (Opus 5.5).

## Outcomes and what follows

| Reading | Next |
| --- | --- |
| CONFIRMED | `rand40` (both seeds) is the candidate controller-robust policy for GR-2 (platform randomization and the Gazebo / PX4 transfer), by a new decision |
| NOT_CONFIRMED | GR-1 stays as recorded, labelled "not reproduced with a second seed"; no further gain-randomization PPO variants (GR-1 amendment 1's rule) |
| MIXED | both are reported; a third seed only by a new decision |

## Limitations fixed in advance

- **Two training seeds per arm** are a replication check, not a training-variance estimate.
- **The F-relative contrasts use GR-0's cells:**
  - 23 of them ran alongside CPU work on 2026-11-04 (`load_overlap/`).
  - Read them with the T-LOAD reading (`camonly/dev`, `docs/plans/prereg_tload_v1.md`).
  - The pair contrasts P1-P3 do not involve F.
- **Everything else** is GR-1's limitation list: gains as a proxy for platform variation, a narrow training setup,
  the 898-D actor unchanged.

## Concurrent work during this unit

D-2026-11-05-1:
- **During training:** CPU work on cores 6,7,12,13 at `nice -n 10`, capped, with no CUDA and no writes to this
  worktree or the main checkout. Its windows are recorded in the WORKLOG.
- **During evaluation:** CPU work only if T-LOAD read LOAD_INVARIANT, under the same conditions.

## Cost

| Step | GPU |
| --- | --- |
| Training | about 70 min per arm |
| Reproduction | about 5 min |
| Evaluation | about 90 min per arm |
| Total | about 5.5 h |
