# CAM-K2 v3 — CAM-K2 v2 retrained with an invalid-depth validity rule (dense invalid blob → surface at MinZ) (preregistration, drafted 2026-11-08, committed 2026-11-09)

**State:** **NOT_RUN** at its commit. Independent review: see *Review before the commit* at the end. Its repository path is
`docs/plans/prereg_camonly_k2_train_v3_2026-11-08.md`, which `tools/camk2v3_train.py` names and refuses to run without.
- It is never edited after the commit. Amendments go in a new dated file.
- The v3 files were written on 2026-11-08 between about 23:20 and 23:45 KST, while MOTAR-camonly was frozen for the
  CAM-K2 v2 evaluation. They live outside the repository in `~/motar_tmp/camk2v3_20261108/wip/` (the same relative
  paths), and were first tested in a tree copy `~/motar_tmp/camk2v3_20261108/tree` (since removed for disk; `git archive` of HEAD `ceab4a0` plus
  these files). No GPU was used.
- This is training only. It claims no performance result.
- **CAM-K2 v2 is kept as recorded.** Its preregistration, code, checkpoints and evaluation stay unchanged. v3 is a new
  lineage branch from the same CAM-K1 sources; it does not replace v2.

Code (all new except the entry):
- module `aerial_gym/task/navrl_task/navrl_depth_validity.py` (new; opt-in, default OFF);
- entry `tools/realveh_entry.py` (**minimal extension**: installs the module only when `NAVRL_DEPTH_VALIDITY` is set;
  unset, its settings, receipt keys and install order are v2's);
- wrapper `tools/camk2v3_train.py` (new; a copy of `tools/camk2v2_train.py` with v3's constants and checks);
- launcher `aerial_gym/rl_training/rl_games/train_navrl_camk2v3.sh` (new; a copy of v2's with three added exports);
- tests `tests/test_depth_validity.py` and `tests/test_camk2v3_train.py` (new).
Unchanged: `navrl_depth_model.py`, `navrl_vehicle_rand.py`, `tools/camk2v2_train.py`, `train_navrl_camk2v2.sh`,
`tools/camtrain_entry.py`, the pinned `navrl_task.py`, `navrl_perception.py`, `navrl_detector.py`,
`kroc_eval_adapter.py`, and every committed preregistration.

## Why

- **Training logs (exploratory, WORKLOG 2026-11-08, `ceab4a0`).** The share of crash steps whose hit bar is among the
  obstacle tokens (`barprobe v2` `hit_token`, means of the last 5 windows,
  `~/motar_tmp/camk2_depthprobe_20261108/contract_r1/REPORT.md`) was 0.68 / 0.72 for CAM-K1 v1 seeds 1 / 2, and 0.17 /
  0.19 for CAM-K2 v2. The drop is already present in v2's first windows, when the OAK-D depth model is switched on.
- **CPU probe (exploratory, `~/motar_tmp/camk2_depthprobe_20261108/probe_r1/REPORT.md`).** Through the production depth
  model → column minimum → scan → token path, on 50 synthetic bar scenes: the hit bar is associated in 50/50 OFF, 26/50
  ON, 50/50 with MinZ off. A bar face closer than MinZ 0.4 m is marked invalid, invalid is encoded as 20 m ("no
  return"), and the bar disappears before any token selection. The disclosed "invalid = no return" convention of v2 is
  therefore an optimistic sensor: it turns the obstacle about to be hit into free space.
- **Contract audit (`contract_r1/REPORT.md`).** `hit_fov` is the 240° token-sector inclusion, not visibility in the
  66° camera FOV; the governor's logged contact clearance is the previous requested-corridor clearance, not a
  same-time surface distance. Neither is used here as an estimate of perception error.
- **The user's choice** (2026-11-08 about 23:25 KST, answering an AskUserQuestion): "규칙 고쳐서 v3 재학습 (권장)"
  (fix the rule and retrain as v3, recommended). That option included GPU approval for this unit. The approval file
  and the commit still come first (below), and the coordinator deferred the run on 2026-11-08 (v2 evaluation first).
- **CAM-K2-E has since run** (`d2a018b`, results review r1 PASS): under the treatments v2 crashes 3.80 / 4.74 pp less
  than CAM-K1 (gate A SMALL_ADAPTATION, read as "lower crash under the treatments"), with no material loss on F's
  vehicle and an interaction of −0.15 / −0.69 pp (`adaptation_supported` false). It did not test whether the near-bar
  depth gap changes the crash rate. v3's evaluation would need its own preregistration and approval.
- **The user's run approval** (2026-11-09 about 12:13 KST): "v3 재학습하면 무슨 문제가 있어? 재학습해" (what is the
  problem with retraining v3? retrain). Before the approval took effect, the coordinator listed the problems: the
  too-far/textureless invalid case below (phantom near walls on a real OAK-D), that a crash effect of the depth gap
  is NOT_TESTED, about 3 h of GPU, and the disk floor. The approval covers the smoke and seeds 1 and 2, once each.
  The user also approved deleting CAM-K2 v2's intermediate checkpoints for disk (38 files, list in
  `~/motar_tmp/camk2v2_20261108/deleted_intermediate_ckpts_20261109.txt`; the ep29000 checkpoints are kept).

## The rule

`NAVRL_DEPTH_VALIDITY=blob_near`, `NAVRL_DEPTH_VALIDITY_WINDOW=5`, `NAVRL_DEPTH_VALIDITY_DENSITY=0.5`.
Per frame, on the depth frame the depth model just produced:
1. **Invalid** is the depth model's own mask of this frame (`perception._depth_model_invalid`: z < MinZ, z > MaxZ or a
   hole; the renderer's no-hit pixels are never invalid). The 20 m value is not used to detect invalid pixels: a
   valid pixel whose noised range reaches the far plane is clamped to exactly 20 m by the depth model, so equality
   would be unsafe. A freshness check refuses a mask that is not this frame's (the depth model's call counter must
   advance by exactly one per call). The rule's telemetry checks that the mask and the 20 m encoding agree.
2. **Density** is the share of invalid pixels in the 5 × 5 window around each pixel (stride 1, same padding, pixels
   outside the image not counted).
3. **Invalid pixels with density ≥ 0.5 become a surface at MinZ**: optical z = 0.4 m, converted to the stored ray
   range with the depth model's own per-pixel cosine (range = 0.4 / cos). Isolated invalid pixels stay "no return".
   Every other pixel passes through bit for bit.

It uses only the sensor's own invalid mask (a real OAK-D reports invalid as depth 0), no simulator-only quantity and no
random number. It is installed by the entry just **before** the depth model, because the later-installed wrapper runs
first: the rule must be the inner wrapper to run on the depth model's output. The depth model and the LiDAR ablation
still bind observe's original signature (functools.wraps).

## Fixed treatment

Everything is CAM-K2 v2's, plus the rule:

| Setting | Value |
| --- | --- |
| Source | CAM-K1 v1 seed s, `last_gen_ppo_ep_28000_*.pth`, SHA-256 pinned per seed (as v2) |
| Epochs | 28001..29000 (smoke 28001..28002), LR 5e-6, fixed 205 bars, main profile, 128 envs |
| Sensors | `no_lidar`; FOV 66 × 54°; yaw_align_weight 0.6; visibility_bonus 0.03 |
| Vehicle | navrl_quad identity; per-motor limit raised to T/W 4.0 × m × g / 4; TW_SCALE 0.15; MOTOR_IMBAL 0.05; τ 0.02–0.04 s; seeds 4911 / 4912 |
| Depth model | MinZ 0.4 m, MaxZ 20 m, σ_d 0.15 px, f·B 66.1875 px m, holes 0.02; seeds 4921 / 4922 |
| Validity rule (new) | blob_near, window 5 px, density 0.5 |
| Run tag | `v2-camk2v3-ep28000-205bars-main-riskcap-camonly-tw40-s{seed}` |
| Unit directory | `~/motar_tmp/camk2v3_20261108/` (approval, outputs, reports) |
| Wall cap | 2.5 h per seed (`timeout -k 120`) |

## Runs, checks and labels

- **Development smoke first** (`tools/camk2v3_train.py 1 --smoke`, epochs 28001..28002, tag `…-smoke`): not a result.
  SMOKE_OK when every post-run check passes. Seeds refuse unless the smoke read SMOKE_OK at the same HEAD and
  1,000 epochs × the smoke's last step time × 1.25 ≤ 2.5 h.
- **Order:** seed 1, then seed 2, each once. Seed 2 starts only if seed 1 reads TRAINED. The tree stays frozen from
  the commit until seed 2 has ended. This order is enforced by the chain script
  `~/motar_tmp/camk2v3_20261108/run_camk2v3_chain.sh` (as in v2), not by the wrapper (review r1 m-5).
- **Preflight refusals:** v2's list, with the approval `~/motar_tmp/camk2v3_20261108/execution_approval.json` naming
  `tools/camk2v3_train.py`'s SCOPE and HEAD.
  - **Disk floor 0.6 GB** (v2: 1.0 GB; review r1 M-2). From v2's real folders the smoke wrote about 0.045 GB and each
    seed about 0.22 GB. With 1.32 GB free at the commit, a 1.0 GB floor would leave seed 2's preflight about 60 MB
    above it, so a normal run could be refused. 0.6 GB keeps one seed plus about 0.35 GB of headroom for logs and
    caches; about 0.84 GB is expected free at the end.
- **Entry refusals added for v3:** `NAVRL_DEPTH_VALIDITY` other than unset or `blob_near`; the rule without
  `NAVRL_DEPTH_MODEL=oakd`; a window that is not an odd integer in [3, 31]; a density outside (0, 1]; a rule parameter
  set while the rule is off. Run time: the rule installed after the depth model or the ablation, or twice; a missing
  or stale depth-model mask; MinZ 0.
- **TRAINED** needs every v2 check (exit 0; host unchanged; one run folder with the tag; the final checkpoint; the
  realveh receipt with every setting and the committed SHA-256 of every hashed file; the vehicle-randomization record;
  the vehicle-randomization and depth-model telemetry; the checkpoint's navrl_quad identity; the unchanged source
  copy; the `hfov=66deg` line; step-time lines; the tracked tree unchanged), and in addition:
  - the receipt's `settings.depth_validity` and `depth_validity.settings` equal the table, its
    `depth_validity_sha256` equals the module committed at HEAD, and its environment has `NAVRL_DEPTH_VALIDITY=blob_near`,
    window `5` and density `0.5`;
  - the rule's telemetry: one install line with the settings; a first-frame line; a summary line after 50 frames;
    on both, pixels > 0, 0 ≤ near ≤ invalid ≤ pixels, and `sentinel_mismatch` 0; on the summary line also
    near > 0, so the rule must have acted (review r1 M-1; v2's summaries had about 3.7 % of eligible pixels below MinZ, while
    the first frame may have none).
  Otherwise TRAINING_FAIL, or VOID_HOST_FAULT if the host changed.
- **Preregistered training diagnostic (labelled, does not change TRAINED / TRAINING_FAIL):** the mean `hit_token` of
  the last 5 `NavRL barprobe v2` lines in the launcher log. **DIAG_OK** if ≥ 0.5, **DIAG_LOW** otherwise,
  **DIAG_NOT_RECORDED** with fewer than 5 lines (the smoke). It is reported with the references CAM-K2 v2 0.17 / 0.19
  and CAM-K1 v1 0.68 / 0.72 (4 dp: 0.1668 / 0.1916 and 0.6768 / 0.7222). DIAG_LOW would be consistent with the rule not
  restoring hit-bar tokens in the training population; it would not by itself establish a cause (review r1 m-4). It is a training-rollout statistic, not an evaluation result.
  - `hit_fov` (logged next to it) is the 240° token-sector inclusion, not camera visibility; it is not used.
  - The governor's contact clearance is the previous requested-corridor clearance; it is not used.

## CPU evidence before the run (exploratory)

`~/motar_tmp/camk2v3_20261108/probe/REPORT.md` (probe_r1's scenes and path, plus 20 hole-only frames; the protocol
was fixed before the run):
- hit bar associated in 50/50 valid scenes with the rule, for both depth seeds (v2 depth model: 26/50; MinZ off: 50/50);
- negative control: on the 49 frames without any sub-MinZ pixel (20 of them hole-only), about 6,100 hole pixels per
  seed, 0 turned near and 0 frames whose tokens differ from v2's;
- the near bar is reported at MinZ (minimum token range 0.40 m), not at its true distance (MinZ off: 0.02 m).

## Tests (CPU, tree copy, 2026-11-08; superseded by the worktree runs in *Review before the commit*)

- `tests/test_depth_validity.py`: 17 tests, OK. `tests/test_camk2v3_train.py`: 18 tests, OK.
  `tests/test_realveh.py` (with the extended entry): 48 tests, OK.
- `tests/test_camk2v2_train.py`: 31 of 38 pass; the 7 failures are the tree copy having no git history and no
  `results/` directory. The same 7 fail with HEAD's entry restored in the copy, so they are environmental.
- The PUBLIC_CPU suite ran in the worktree before the commit; see *Review before the commit*.

## Disclosed limits

- **Too-far invalid.** A real OAK-D also reports 0 (invalid) for regions too far, textureless or overexposed. This
  rule would call a dense blob of those near. In simulation that case is **NOT_TESTED**: obstacle depth is capped at
  10 m and the renderer's no-hit pixels pass through as no return; they are never invalid. A deployment must separate
  too-far invalid from too-near invalid (for example by disparity confidence, or by the range of the valid border)
  before using this rule. Sim-to-real validity of the rule is NOT_TESTED.
  - The same holds for other real invalid bands (review r1 m-2): occlusion shadows beside a near edge, pixels
    rejected by the left-right consistency check, and specular or overexposed patches. Any such band at least about
    3 px wide (dense in a 5 × 5 window) becomes a phantom surface at MinZ.
  - The coordinator told the user this before the run approval took effect (2026-11-09, *Why*).
- **Detector mask** (review r1 m-3). The rule's output also feeds the colour-rule detector's `depth < max_range` mask.
  It uses no ground truth; it can only matter when the target is within 0.4 m.
- **Conservative distance.** A near bar is reported at MinZ, not at its true distance. A bar touching the vehicle reads
  the same as one at 0.4 m.
- **Small dilation.** Holes adjacent to a near blob are turned near as well; blob borders below the density threshold
  stay no return.
- **Mixed edges.** probe_r1 found five scenes where bilinear edges between a near bar and the 10 m background survive
  as inflated valid ranges. With the rule the bar core is at MinZ, so the column minimum takes it; the inflated edge
  pixels themselves are unchanged.
- **Bundled treatments.** As in v2, the vehicle, motor randomization, depth model and now the rule are trained
  together, without ON/OFF arms; no effect is attributed to any one of them. v3 vs v2 differs by the rule only, but
  is a separate training run, not a paired ablation (one seed pair, training rollouts).
- The probe uses synthetic analytic frames; production Warp renders and recorded crash frames are NOT_TESTED.

## Review before the commit

- **Design review r1** (Claude Opus 5.5, fresh context, read-only): **AMEND_FIRST**,
  `docs/evidence/camonly_2026-11-04/camk2/REVIEW_v3_design_r1.md`. The rule, its install order and the rule-unset
  path were found correct. Findings and repairs:
  - B-1 BLOCKER: `test_camk2v3_train`'s path test failed in the single-process suite → uses v2's import-time flag.
  - M-1 MAJOR: the telemetry check accepted a rule that never acted → near > 0 on the summary line, with a negative
    test (`rule_never_acted`).
  - M-2 MAJOR: the 1.0 GB disk floor would probably refuse seed 2 → 0.6 GB, derived under *Runs, checks and labels*.
  - m-1 index bullet order; m-2 other real invalid bands; m-3 detector mask; m-4 DIAG_LOW wording; m-5 order
    enforced by the chain script; LOW docstring and "draft" headers: all fixed in this text or the files.
  - LOW, environmental: the suite needs `OMP_NUM_THREADS=1 MKL_NUM_THREADS=1` for three SITL tests, and two
    `test_boia_planner` tests need 3 GB free.
- **Design review r2** (Claude Opus 5.5, focused, read-only): **PASS_WITH_LIMITS**,
  `docs/evidence/camonly_2026-11-04/camk2/REVIEW_v3_design_r2.md`. Every r1 repair verified; no new defect. M-1:
  `near_pixels` on the summary line is cumulative over 50 calls × 128 envs, and v2's real logs give a sub-MinZ share of
  0.037–0.038 (about 1.7 M pixels), with 0 on the first frame. M-2: per seed about 0.219 GB, smoke 0.045 GB; seed 2's
  preflight should see about 1.05 GB. N-1 (this section) and its LOWs were fixed before the commit.
- **Module hash.** The probe ran `navrl_depth_validity.py` at SHA-256 `8be04135…`; the committed file is
  `f06a8897…`. The difference is docstring only (review r1 LOW, "draft" header).
- **Chain script** `~/motar_tmp/camk2v3_20261108/run_camk2v3_chain.sh`: SHA-256 `4059cdee767bc5d16e5aeed8941fc7f5ace91583effc4869f5ec99867817d56c`.
- **PUBLIC_CPU** (2026-11-09 12:37–12:49, `OMP_NUM_THREADS=1 MKL_NUM_THREADS=1`, cores 6,7,12,13, nice 10, faults
  0 → 0): 3,810 run, 0 failures, 2 errors, 10 skipped. The 2 errors are the known `test_boia_planner` 3 GB disk
  floor (also at `1237f9f`). Log: `~/motar_tmp/camk2v3_20261108/public_cpu_precommit.log`. An earlier run without
  the thread variables had 3 more environmental SITL errors and the B-1 failure
  (`public_cpu_precommit_r0_noOMP.log`).
