# CAM-K1-E v1 — matched evaluation of the CAM-K1 camera-only lineage (preregistration, 2026-11-08)

Training record: [CAM-K1](../evidence/camonly_2026-11-04/camk1/README.md) (both seeds TRAINED, `814c28a`). Decision
[D-2026-11-06-1](../DECISIONS.md): the real vehicle has no LiDAR; its camera is a Luxonis OAK-D Pro (RGB 66° × 54°).
CAM-K1's training preregistration left the evaluation to this file: "matched against Frozen F and against camera-only F
under the same 66° camera ... with thresholds fixed before it runs".

**Approval.** The user, 2026-11-08 at 12:07 KST, answered "예, 오늘 전부 (권장)" to: "오늘 GPU 평가, PPO 학습, SITL
실행을 단위마다 따로 묻지 않고 진행해도 될까요? 각 단위는 사전등록을 커밋하고, Opus 검토와 실행 전 점검을 통과한 경우에만
돌리겠습니다." The option named this evaluation ("CAM-K1-E 평가"). The runner refuses without
`~/motar_tmp/camk1e_20261108/execution_approval.json` naming the runner's SCOPE and HEAD.

## Question

Does continuation training for the camera alone (CAM-K1) reduce the crash rate of Policy F once the LiDAR is gone, at
the real camera's 66° field of view? And how far is it then from Policy F with camera + LiDAR?

## Arms

All arms run Policy F's evaluation contract on GR-0's S00 grid, the condition of C2 (E2 moving target, A3 placement,
70/115/160/205 bars × seeds 4501–4503, quota 16 episodes per environment, 128 environments, nominal gains).

| Key | Policy | Sensors | Camera | Source |
| --- | --- | --- | --- | --- |
| A | Frozen F (`f7022139…`) | camera + LiDAR | 87° × 58° | GR-0's S00 cells (existing; C1 identity) |
| C87 | Frozen F | `no_lidar` | 87° × 58° | C2's `no_lidar` cells (existing) |
| F66 | Frozen F | `no_lidar` | 66° × 54° | new, 12 cells |
| K1s1 | CAM-K1 seed 1 (`d7bf51a8…`) | `no_lidar` | 66° × 54° | new, 12 cells |
| K1s2 | CAM-K1 seed 2 (`5128bf2e…`) | `no_lidar` | 66° × 54° | new, 12 cells |

- The new arms run through `tools/camtrain_python`, CAM-K1's committed entry point. It installs `no_lidar` and four
  values: the camera FOV and the two reward weights (0.6, 0.03). The reward weights enter only the reward
  (`navrl_task.py` lines 6059 and 6068), not an episode's outcome, so F66 and the K1 arms differ in the checkpoint
  only.
- CAM-K1 checkpoints come from their TRAINED train reports, as copies whose SHA-256 equals the run's.

## Runner and launcher

- **Runner:** `tools/camk1e_run.py`, built on `tools/gr_run.py`'s cell environment, checks and launcher call (as C2's
  runner).
- **Launcher:** `aerial_gym/rl_training/rl_games/eval_navrl_camk1e_arms.sh`, pinned by SHA-256 in the runner. It is
  the pinned `eval_navrl_target_behavior_arms.sh` with four marked changes (`[camk1e]`):
  1. it requires the condition, the entry point `CAMEVAL_PY` and absolute interpreter paths;
  2. it takes the checkpoint and its expected SHA-256 from the caller, as the GR-1 copy does;
  3. each cell gets its own camtrain receipt;
  4. only the `play_navrl.sh` call runs through the entry point.
- **Cell checks.** A cell is valid only if it passes all of these:
  - GR-0's checks (receipt OK on this commit with a clean tree, the arm's checkpoint SHA, no `NAVRL_GAIN_RAND*`, the
    distance backend, the ledger quota, import origins inside the tree, no recorder violations, host unchanged);
  - the camtrain receipt names `no_lidar`, a 72 × 4 LiDAR, the committed ablation module and entry point, and
    exactly the arm's four values;
  - the receipt's NAVRL snapshot names the same condition and values;
  - the recorder summary shows the camera the simulator built (`camera.hfov_deg`, `camera.vfov_deg`), and the cell
    log its FOV line (`hfov=66deg`, or `hfov=87deg` for the identity cell);
  - the receipt's task and perception paths are absolute and inside the tree.
- **Preflight refusals:**
  - the approval does not name SCOPE and HEAD, or the commit argument is shorter than 7 digits;
  - less than 1 GB free disk;
  - not branch `camonly/dev`, or a dirty tree;
  - this file, the launcher, the ablation module, the entry point or the runner not committed;
  - the launcher not at its pinned SHA;
  - Policy F's or a CAM-K1 checkpoint's SHA wrong (the CAM-K1 SHA-256s are pinned in the runner);
  - not exactly one RTX 3070, or another GPU compute process except the remote desktop;
  - import origins outside the tree;
  - an existing output folder.
- **Host faults:** a rise in the kernel fault count voids the run.
- **Outputs:** `~/motar_tmp/camk1e_20261108/raw/identity_01/` and `.../raw/eval_01/`, outside every repository.

## Stage 1: identity, positive and negative controls and a checkpoint smoke (3 cells, 205 bars, seed 4501)

| Cell | Requirement |
| --- | --- |
| `F87_identity`: Policy F, `no_lidar`, F's own values (87° × 58°, 0.3, 0.02), through this launcher and entry point | `result.json` (minus the per-run nonce and the checkpoint path) and `episodes.jsonl` equal C2's `no_lidar` cell exactly |
| `F66_negative`: Policy F, `no_lidar`, 66° × 54°, 0.6, 0.03 | `episodes.jsonl` differs from C2's `no_lidar` cell: the override takes effect in the simulator |
| `K1s1_smoke`: CAM-K1 seed 1, as F66 | valid, and its `episodes.jsonl` differs from `F66_negative`'s: the CAM-K1 checkpoint path takes effect |

- **Reading:** IDENTITY_PASS if all three hold; IDENTITY_FAIL otherwise, and stage 2 does not run.
- **What a pass admits:** C2's `no_lidar` cells as arm C87, and GR-0's S00 cells as arm A (through C1's identity),
  without rerunning them. The design review found the simulation code unchanged since C2's commit `5522be0`.
- This stage is the GPU smoke of every changed runner path: the launcher copy, the entry point at both value sets,
  and F's and a CAM-K1 checkpoint.
- **Confound, disclosed (review r1 m3):** `F66_negative` changes the FOV and the two reward weights together. The
  weights enter only the reward (`navrl_task.py` 6059 and 6068; verified by the review), so the difference is the
  FOV's.

## Stage 2: eval (36 cells)

- **Order:** density, then seed, then arm (F66, K1s1, K1s2).
- **Needs** exactly one identity report in the same parent folder, an IDENTITY_PASS on the same commit, no earlier
  eval there, the same allowlisted runner environment and the same C2 folder (review r1 m11).
- **Cap:** 3 h, checked between cells, so the stage may overrun by up to one cell (a 30-minute timeout).
- **Free disk** at least 1 GB at preflight; about 0.31 GB is written (review r1).

**Reruns (review r1 m11).** A failed identity, or an INCOMPLETE or VOID eval, stays recorded in its folder. A rerun
needs a new parent folder and a dated amendment that says why. The analysis reads one identity folder and the one eval
folder that named it.

## Analysis (fixed): `tools/camk1e_analyse.py`

**Bindings first.** Any failure reads EVAL_BINDING_FAIL:
- IDENTITY_PASS on the eval commit, and it is the identity report the eval named;
- eval COMPLETE with 36 cells;
- one runner environment and C2 folder in both stages;
- every eval cell's camtrain receipt, recorder camera FOV (66 × 54) and checkpoint SHA as the run report names them;
- every eval cell's result condition equals C87's at the same block, and its NAVRL snapshot differs from C87's only in
  the expected keys (the run's nonce, file paths and run id, the checkpoint SHA and the four entry-point values)
  (review r1 M4);
- complete grids;
- in every block, F66 differs from C87, each CAM-K1 seed from F66, and the two seeds from each other;
- the identity stage's F66 and K1s1 cells equal the eval's (205 bars, seed 4501);
- GR-0's S00 and S20 cells and C2's `no_lidar` cells at their committed manifests' SHA-256;
- the analysis script's bytes equal its bytes at the eval commit (review r1 m6).

**Quantities.**
- Rates are quota rates per block; the 12 (density, seed) pairs are the blocks.
- **Intervals (review r1 M2).** The four densities are fixed, not sampled, so the intervals resample the three seeds
  within each density:
  - a stratified percentile bootstrap (10,000 resamples, seed 20,261,108);
  - a stratified t-interval (8 df) as a sensitivity check.
  - Both describe these 12 blocks; they do not generalise to other densities.
- Contrasts are in pp.

**Gates,** on the crash rate, per CAM-K1 seed s. A label needs both intervals to agree.

| Gate | Contrast | Labels, in order |
| --- | --- | --- |
| R (recovery at the same camera) | K1s − F66 | **UNRESOLVED**: the two intervals disagree on whether 0 is excluded, or on the side. **MATERIAL_RECOVERY**: both upper ends < −10 pp. **SMALL_RECOVERY**: both upper ends < 0 and the mean ≤ −2 pp. **WORSE**: both lower ends > 0 and the mean ≥ +2 pp. **NEGLIGIBLE**: both intervals inside (−2, +2) pp. **NO_DETECTED_RECOVERY**: otherwise. |
| P (closeness to camera + LiDAR) | K1s − A | **NO_MATERIAL_LOSS**: both upper ends < +3 pp. **MATERIAL_LOSS**: both lower ends > +3 pp. **LOSS_INCONCLUSIVE**: otherwise. |

- **Timeouts (review r1 m9).** A crash reduction with timeouts up by 2 pp or more (the timeout contrast's mean) reads
  **CRASH_TO_TIMEOUT** in place of MATERIAL_RECOVERY or SMALL_RECOVERY. "Recovery" may be said only with one of those
  two labels.
- **Lineage reading** per gate: the label both seeds share, else **SEEDS_DISAGREE**.
- **Why these bands.**
  - Removing the LiDAR raised F's crash rate by about 40 pp at 87° (C2: 4.46 % → 44.4 %). A recovery smaller than
    10 pp would leave most of that loss.
  - Gate P's 3 pp is about two thirds of A's crash rate (review r1 M3). On existing data, GR-0's ±40 % gains (+3.7 pp)
    read MATERIAL_LOSS under it, as GR-0 labelled them GAIN_SENSITIVE, and its ±20 % gains (+0.6 pp) read
    NO_MATERIAL_LOSS.
  - The 2 pp floor keeps negligible differences out of SMALL_RECOVERY and WORSE (review r1 m7).
- **Controls at claim scale, on existing data (review r1 m8).** Every control must read as required. Otherwise the
  reading is CONTROL_FAIL, and no gate label is reported.

  | Control | Required | Computed on 2026-11-08 before this commit, with this script |
  | --- | --- | --- |
  | gate P on C87 − A | MATERIAL_LOSS | +39.95 pp [39.64, 40.22]; t [39.53, 40.36] |
  | gate R on A − C87 | MATERIAL_RECOVERY | −39.95 pp [−40.22, −39.65]; t [−40.36, −39.53] |
  | gate P on S20 − A | NO_MATERIAL_LOSS | +0.61 pp [0.28, 0.96]; t [0.11, 1.11] |
  | gate R on S20 − A | not MATERIAL_RECOVERY or SMALL_RECOVERY | NEGLIGIBLE |

  These controls use no CAM-K1-E data. They show that the gates separate known-bad from known-good differences on
  this grid.

**Reported, not gated:**
- the same contrasts for capture and timeout (capture labelled with C2's scheme, band 3 pp);
- F66 − C87: narrowing camera-only F's view from 87° to 66°;
- F66 − A, K1s1 − K1s2 and C87 − A;
- pooled and per-density rates;
- crash causes;
- the speed governor's telemetry.

## What may be said, and what may not

- **May be said:** "On this grid, in simulation, CAM-K1 seed s changes the crash rate by X pp [CI] against camera-only
  F at the same 66° camera, and by Y pp [CI] against F with camera + LiDAR at 87°", with the gate labels.
  - Gate P's NO_MATERIAL_LOSS reads "within 3 pp of F with camera + LiDAR on this grid". The words "parity",
    "equivalent" and "matches F" are not used (review r1 M3).
- **May not be said:**
  - anything about the real vehicle, real images, the OAK-D Pro's depth or real flight;
  - that the LiDAR is unnecessary;
  - that more training would or would not help (not tested);
  - an effect of training from one seed alone;
  - that camera-only training alone caused R. K1 − F66 measures CAM-K1's whole recipe (review r1 m10): `no_lidar`
    training, the 66° FOV, the two reward weights and 3,000 more epochs.

## Disclosed limits

- **Reference A** is F with camera + LiDAR at the 87° camera, not at 66° (review r1 m10).
- **Seeds:** two training seeds; the intervals are over evaluation blocks, not training seeds.
- **Vehicle:** navrl_quad (0.25 kg, T/W 3.26), not the user's 0.5 kg vehicle.
- **Camera model:** one camera for RGB and depth at 66° × 54°, Euclidean ray depth, no stereo minimum depth, noise or
  holes. The detector is F's colour rule on a red sphere.
- **The speed governor:** under `no_lidar` its camera corridor narrows with the FOV (review r1 m10).
- **Training against evaluation:** CAM-K1 trained at a fixed 205 bars; the grid also has 70, 115 and 160.
- **Shared F arms:** A and C87 are cells from earlier commits, admitted through C1's identity and stage 1.
- **Concurrent CPU work during the eval (review r1 l3):** allowed under AGENTS.md's T-LOAD rule (LOAD_INVARIANT for
  per-episode outcome files): `nice -n 10`, cores 6 and 7, a memory cap, no CUDA, with the window recorded. No
  wall-clock quantity of this unit is used.

## Controls of the machinery, run before launch (CPU)

`tests/test_camk1e.py` (18 tests, no simulator):
- **Launcher copy:** reconstructed into the pinned base exactly, and the runner pins its SHA.
- **Approval:** a valid approval passes; a wrong scope, wrong commit or empty field fails.
- **Receipt:** valid cells pass at both value sets, and CAM-K1 seed 1's real receipt passes from another working
  directory (review r1 M1). These all fail:
  - a wrong override, a wrong condition, a missing FOV log line or recorder FOV;
  - a wrong module SHA, the wrong values;
  - a missing, relative or empty task path;
  - a missing receipt.
- **Checkpoints:** TRAINED reports with the pinned SHA pass, and the pins equal the real train reports. A smoke, a
  failed training, a copy-SHA mismatch and an unpinned SHA fail.
- **Gates:** labels on fixed intervals, including the floor, CRASH_TO_TIMEOUT and a null contrast. The stratified
  intervals are not widened by a fixed density effect.
- **The commit check** passes for a committed file and fails for other bytes or an unknown commit.
- **Runner end to end, with the simulator call patched:** identity, then eval COMPLETE with 36 cells in the fixed
  order. These are refused or fail:
  - eval without an identity pass (PREFLIGHT_FAIL);
  - eval after a failed identity in the same parent (PREFLIGHT_FAIL);
  - an F87 cell that differs (IDENTITY_FAIL);
  - an F66 cell that equals C2's (IDENTITY_FAIL);
  - a wrong FOV in a K1s2 cell, which stops the eval.
- **Analysis end to end, on synthetic eval folders built from C2's real cells:**
  - a known recovery reads MATERIAL_RECOVERY, with all four controls as required;
  - one unrecovered seed gives SEEDS_DISAGREE;
  - EVAL_BINDING_FAIL for each of: a K1 arm identical to F66, a wrong override, an unexpected snapshot key, and a
    changed result condition.

## Review

- **Design review r1** (Claude Opus 5.5, fresh context, before this file was committed): **AMEND_FIRST**, with no
  BLOCKER, four MAJOR, eleven MINOR and five LOW findings (`docs/evidence/camonly_2026-11-04/camk1e/REVIEW_design_r1.md`).
  - Repaired in this file and the code before the commit:
    - M1 (receipt keys);
    - M2 (stratified intervals);
    - M3 (gate P band 3 pp and wording);
    - M4 (condition and snapshot bindings);
    - m1 (K1s1 smoke cell), m2 (recorder FOV), m3 (disclosed), m4 (more failing tests), m5 (pinned checkpoint
      SHA), m6 (script and identity binding), m7 (2 pp floor), m8 (controls that must pass), m9 (timeouts),
      m10 (wording), m11 (rerun rules);
    - l1 (cap recorded), l2 (free disk), l3 (concurrency), l4 (commit length), l5 (fault pattern in the chain
      script).
  - The repaired files were not reviewed again before launch. Stage 1 is the execution check.
- An independent review of the results follows. Until it ends, the readings are CONDITIONAL.

The results go to `docs/evidence/camonly_2026-11-04/camk1e/`.
