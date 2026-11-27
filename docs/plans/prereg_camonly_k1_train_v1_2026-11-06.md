# CAM-K1 v1 — camera-only continuation training of Policy F (preregistration, 2026-11-06)

**State:** preregistered, **NOT_RUN**. Training only; this file claims no performance result. Code:
- launcher `aerial_gym/rl_training/rl_games/train_navrl_camk1.sh`;
- wrapper `tools/camk1_train.py`;
- entry `tools/camtrain_entry.py` and `tools/camtrain_python`;
- tests `tests/test_camk1_train.py`.

Plan: `~/motar_tmp/camonly_plan_20261106/PLAN.md` (Claude Fable 5.1, 2026-11-06; a planning draft, not a
preregistration). Amendments go in a new dated file.

## Why

- The user decided on 2026-11-06 that the real vehicle carries no LiDAR. Its sensors are a Luxonis OAK-D Pro: a
  stereo depth pair and an autofocus RGB camera, IMX378, HFOV 66° and VFOV 54°.
- The user fixed the simulated camera to the RGB camera's field of view ("use this as the reference"), and asked for
  a larger reward for keeping things in the camera's view.
- Frozen F without LiDAR crashed 4.46 % → 44.4 % (C2, `camera-only` family). The crashes are on the blind side, on
  bars outside the FOV (G3 BLIND_SIDE_SUPPORTED).
- This unit starts a new policy lineage. **Policy F stays frozen and is never replaced**
  (AGENTS.md; D-2026-11-04-1).

## Fixed treatment

GR-1 `ctrl`'s continuation contract is kept: fixed 205 bars, curriculum off, LR 5e-6, the riskcap governor
parameters from the same validated screen summary, the main profile, 128 environments. A test asserts that every
`export` and `unset` of `train_navrl_gr1.sh` is present unchanged, apart from the lines below.

| Item | Policy F / GR-1 ctrl | CAM-K1 |
| --- | --- | --- |
| Start | — / F ep25000 (`f7022139…`) | F ep25000 (`f7022139…`) |
| Epochs | 24001..25000 / 25001..26000 | **25001..28000** (3,000) |
| Sensors seen by the actor and the governor | RGB-D + LiDAR | **camera only**: `NAVRL_SENSOR_ABLATION=no_lidar` (C-study switch, reviewed in C0–C2) |
| Camera FOV (detector and obstacle depth) | 87° × 58° | **66° × 54°** (OAK-D Pro autofocus RGB) |
| `yaw_align_weight` (crab penalty, ≤ 0) | 0.3 | **0.6** |
| `visibility_bonus` (per step while the target is seen) | 0.02 | **0.03** |
| Training seed | 1 | **1, then 2** (two runs, chained) |

- **Reward.** The reward's static-safety term `r_ss` keeps reading the simulator's LiDAR. That is privileged simulator
  state used only in the reward, which the observation contract allows. No other reward or termination term reads
  the LiDAR (`navrl_task.py` 6019–6319).
- **Why these two weights.**
  - The yaw-alignment term is the existing speed-gated penalty for moving off the nose direction. Doubling it is
    intended to push the motion into the camera's view. It is ≤ 0, so it opens no reward for standing still.
  - The visibility bonus is positive per step. It is raised by only 1.5×, because a positive bonus can pay for
    loitering with the target in view instead of capturing it.
    - At 0.03 it stays below the per-step time cost of −0.05, so loitering in view still nets −0.02 per step (F:
      −0.03).
    - With γ = 0.99 the bonus is worth at most about 3 over an episode, against the capture bonus of 30. Review r1
      K-5 corrected an earlier undiscounted argument.

## Runs, checks and labels

- **Development smoke first:** `tools/camk1_train.py 1 --smoke` runs epochs 25001..25002 under the same contract, with
  its own run tag `…-smoke`. It is not a result.
  - It reads **SMOKE_OK** when the training starts and ends cleanly with the camtrain receipt verified and the host
    unchanged.
  - Seeds 1 and 2 refuse unless the smoke read SMOKE_OK at the same HEAD. A failed smoke stops the unit for repair.
- **Order:** seed 1, then seed 2, each once, through `tools/camk1_train.py {1,2}`.
  - The chain waits until the day's SITL units (S3c, S4) have ended. It starts seed 2 only if seed 1 reads
    TRAINED.
- **Preflight refusals:**
  - the approval does not name this SCOPE and HEAD;
  - not branch `camonly/dev`;
  - the tracked or untracked tree is not clean;
  - this preregistration is not committed;
  - not exactly one RTX 3070;
  - any GPU compute process other than the user's remote desktop;
  - a NavRL runner, PX4 SITL or Gazebo process;
  - the Policy F SHA does not match;
  - an existing output directory.
- **During the run:** `timeout -k 120` at 5 h.
- **Labels:**
  - **TRAINED**: exit 0; the host's fault count, boot id and online CPUs are unchanged; exactly one new run folder
    `…v2-camk1-ep25000-205bars-main-riskcap-camonly-s{seed}`; `last_gen_ppo_ep_28000_*.pth` exists; the camtrain
    receipt shows `no_lidar` and the four values above; the tree is unchanged.
  - **VOID_HOST_FAULT**: the host changed. The run is not evidence.
  - **TRAINING_FAIL** or **PREFLIGHT_FAIL**: otherwise.
- **Outputs:** `~/motar_tmp/camk1_20261106/train_s{seed}/`. These hold the report, the launcher log, the camtrain
  receipt, a copy of the checkpoint with its SHA-256, and the session log. The raw run folder stays in the
  worktree's `runs/`.

## What does not follow

- No capture, crash or transfer claim. The evaluation (CAM-K1-E: matched against Frozen F and against camera-only F
  under the same 66° camera) needs its own preregistration, with thresholds fixed before it runs.

## Disclosed limits of this first camera-only run

- **Vehicle:** navrl_quad (0.25 kg, T/W 3.26). The real vehicle is about 0.5 kg with small motors (the user,
  2026-11-06). A 0.5 kg nominal model and motor randomization come next, warm-started from this lineage.
- **Camera model:** one camera shared by RGB and depth at 66° × 54°. The real depth pair is 80° × 55°, so this is
  conservative for obstacles. Not modelled yet:
  - stereo minimum depth (0.4 m with extended disparity);
  - depth noise growing with z²;
  - holes;
  - the separate RGB and depth poses;
  - the 40 × 24 obstacle-depth upsampling (EQ-1D).
- **Detector:** still F's colour rule. On real images it fails, and in Gazebo shading it passed 4 of 35 pixels (S4
  component smoke).
- **Seeds:** a single training seed is not evidence of an effect (GR-1b NOT_CONFIRMED). Two seeds show only the seed
  spread; they do not establish an effect either.
- **Under `no_lidar`:**
  - The visibility bonus is paid on camera-only visibility. F's 0.02 was paid on camera-or-LiDAR visibility over
    360°.
  - Commands outside the camera's view are capped at the governor's 2.0 m/s floor, and the 66° FOV widens that
    sector.
  - F's critic starts on states that have lost the LiDAR.
- **Camera resolution:** 160 × 90 pixels over 66° × 54° gives non-square pixels and finer horizontal resolution than
  F's 87°:
  - fx is about 123 px/rad (F: 84.3);
  - the 0.30 m target spans about 1.85 px at 20 m (F: 1.27 px);
  - so detection range improves over F's camera, unlike the real narrower optics would suggest.
- **Post-run checks** (review r1):
  - the task's own startup line `hfov=66deg` must appear in the logs, so the FOV override is shown to reach the task;
  - the smoke must write its final `last_gen_ppo_ep_25002_*.pth`;
  - the wrapper strips inherited `NAVRL_*`, `CAMK1_*`, `CAMTRAIN_*` and `S3B_*` variables;
  - the wrapper refuses with less than 2 GB free disk.
- **Fixed density:** 205 bars throughout, as F's last stage.
