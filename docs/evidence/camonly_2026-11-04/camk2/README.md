# CAM-K2 v1 — CAM-K1 continued under a T/W 2.25 thrust cap, motor randomization and an OAK-D Pro depth model (2026-11-08)

**Status:** preregistered, design reviews r1 and r2 AMEND_FIRST and r3 PASS_WITH_LIMITS with every finding repaired,
approved by the user (2026-11-08 12:07 KST), **NOT_RUN**. Read the
[preregistration](../../../plans/prereg_camonly_k2_train_v1_2026-11-08.md) first.

- [`REVIEW_design_r1.md`](REVIEW_design_r1.md), [`REVIEW_design_r2.md`](REVIEW_design_r2.md) and
  [`REVIEW_design_r3.md`](REVIEW_design_r3.md): the design reviews (Claude Opus 5.5, fresh context). Their probes
  stay in `~/motar_tmp/camk2_20261108/review_design_r{1,2,3}/`.
- Code:
  - `tools/camk2_train.py`, `aerial_gym/rl_training/rl_games/train_navrl_camk2.sh`;
  - `tools/realveh_entry.py`, `tools/realveh_python`;
  - `aerial_gym/task/navrl_task/navrl_vehicle_rand.py`, `aerial_gym/task/navrl_task/navrl_depth_model.py`;
  - tests `tests/test_realveh.py`, `tests/test_camk2_train.py` and the amended `tests/test_navrl_sensor_ablation.py`.
- Option A documentation only: `resources/robots/quad/quad_navrl_rv05.urdf`,
  `aerial_gym/config/robot_config/navrl_rv05_quad_config.py`. CAM-K2 does not use them. They are the oracle of the
  equivalence test.
- The implementer's notes and design survey are in `~/motar_tmp/realveh_design_20261108/`.

## v1 withdrawn before running; v2 (2026-11-08)

- **v1 is WITHDRAWN_BEFORE_RUN.** The user relayed the real vehicle's specs: 1.1 kg, 5x4.3x3 props, T-Motor Velox
  2550KV (D-2026-11-08-1). v1's T/W 2.25 cap for a 0.5 kg placeholder points the wrong way: the real T/W is about 4.0,
  and navrl_quad's own is 3.26.
  - v1's code is unchanged.
  - It cannot run: there is no approval, and `WITHDRAWN.md` markers occupy its single-use output folders
    (`~/motar_tmp/camk2_20261108/train_{smoke,s1,s2}/`).
  - Its tests are unchanged and pass.
- **v2:** [preregistration](../../../plans/prereg_camonly_k2_train_v2_2026-11-08.md); wrapper
  `tools/camk2v2_train.py`; launcher `train_navrl_camk2v2.sh`; tests `tests/test_camk2v2_train.py`.
  - `navrl_vehicle_rand.py` gained an opt-in raise mode (`NAVRL_VEH_RAND_LIMIT_MODE=raise`), with on-device counters
    and a live read-back of the raised limit.
- **Reviews** (Claude Opus 5.5): [`REVIEW_v2_design_r1.md`](REVIEW_v2_design_r1.md) read AMEND_FIRST (M-1: the
  raised limit was never measured; m-1..m-5; L-1..L-5). All were repaired. Round 2 is in
  [`REVIEW_v2_design_r2.md`](REVIEW_v2_design_r2.md).

## CAM-K2 v2 training runs (2026-11-08)

**Records** in this folder (from `make_camk2v2_record.py`, copied here):
- `train_report_v2_s1.json`, `train_report_v2_s2.json`: byte copies of each seed's report.
- `manifest_v2_s1.json`, `manifest_v2_s2.json`: the checkpoint sidecar manifests.
- `training_curve_v2.json`: per-100-epoch training-rollout statistics. They are under exploration noise and are not
  an evaluation.

| Seed | Verdict | Wall time | Checkpoint SHA-256 | Commands above 2 N | Commands at the raised limit | Host faults |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | TRAINED | 4,816 s (19:27:55–20:48:11) | `769a26f4…de04e8a` | 5.95 % | 4.17 % | 0 → 0 |
| 2 | TRAINED | 4,639 s (20:48:12–22:05:31) | `e76eb048…88321db6` | 5.25 % | 3.61 % | 0 → 0 |

- **The raise was not inert:** commands above 2 N were 5.95 / 5.25 % of motor commands in the two seeds, above the
  preregistered 1 % floor.
- **Crash causes** (training rollouts): bar contact in 99.9 % of crashes in seed 1's last 10 windows, 99.95 % in seed 2's.
- **Training rollouts** (capture %, first → last 100 epochs, under the treatments): seed 1 24.78 → 28.43; seed 2 25.47 → 28.61.
  CAM-K1 ended at 26.29 / 25.55 without the treatments. These are not comparable as an evaluation.
- **Smoke:** SMOKE_OK at 19:27:55 (44 s), with the limit read back live as 2.4525 N.
- **Concurrency:** during training only light reads and writes ran outside the repository. The S5 agent wrote files
  in `~/motar_tmp`; there were no tests, simulators or GPU jobs. The runner was not pinned.
- **Next:** CAM-K2-E. Its files are in `~/motar_tmp/realveh_wip_20261108_1927/`; fill the K2 SHA pins from these
  reports.

## CAM-K2 v3 training runs (2026-11-09)

[Preregistration](../../../plans/prereg_camonly_k2_train_v3_2026-11-08.md) at `d85589d`. Design reviews:
[`REVIEW_v3_design_r1.md`](REVIEW_v3_design_r1.md) (AMEND_FIRST, repaired) and
[`REVIEW_v3_design_r2.md`](REVIEW_v3_design_r2.md) (PASS_WITH_LIMITS). The user approved the run on 2026-11-09 at
about 12:13 KST. v3 is CAM-K2 v2 with the depth-validity rule `blob_near`: an invalid pixel in a 5 × 5 window that
is at least half invalid becomes a surface at MinZ.

**Records** (from [`make_camk2v3_record.py`](make_camk2v3_record.py)):
- `train_report_v3_s1.json` and `train_report_v3_s2.json`;
- `manifest_v3_s1.json` and `manifest_v3_s2.json`;
- `training_curve_v3.json`.

| Seed | Verdict | Wall time | Checkpoint SHA-256 | Commands above 2 N | Commands at the raised limit | Host faults |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | TRAINED | 4,765 s (12:51:13–14:10:38) | `888e8056…99fafc8e1`[^v3s1] | 6.19 % | 4.37 % | 0 → 0 |
| 2 | TRAINED | 4,676 s (14:10:38–15:28:34) | `203deea4…f9492e4ea2`[^v3s2] | 5.42 % | 3.75 % | 0 → 0 |

[^v3s1]: Full SHA-256 `888e8056b2a6768318e6d9fa4136c5ed01f68b7480954f7f339801b99fafc8e1`,
    `last_gen_ppo_ep_29000_rew_-7.648726.pth`.
[^v3s2]: Full SHA-256 `203deea459ef1d937239281c46f07d96e797839708337666de1ecdf9492e4ea2`,
    `last_gen_ppo_ep_29000_rew_-4.657931.pth`.

- **Smoke:** SMOKE_OK at 12:51:13, after 45 s.
- **The rule acted.** Its summary line (50 calls × 128 frames) turned 1,982,532 / 1,883,036 pixels near. That is
  2.15 / 2.04 % of all pixels and 69.2 / 68.0 % of invalid pixels. `sentinel_mismatch` was 0 in both seeds, and the
  first frame had 0 near pixels.
- **Preregistered diagnostic:** **DIAG_OK** for both seeds. The `hit_token` mean over the last 5 windows was
  0.6958 / 0.6898. References: CAM-K2 v2 0.1668 / 0.1916; CAM-K1 v1 0.6768 / 0.7222.
  - This is consistent with the rule restoring hit-bar tokens in the training population, to about CAM-K1's level.
  - It is a training-rollout statistic, not an evaluation result, and it does not by itself establish a cause.
- **Training rollouts** (capture %, first → last 100 epochs; under exploration noise, not an evaluation):

  | | Seed 1 | Seed 2 |
  | --- | --- | --- |
  | v3 | 26.80 → 26.73 | 25.02 → 28.51 |
  | v2 | 24.78 → 28.43 | 25.47 → 28.61 |

  Crashes were bar contact in 99.9 % of cases over the last 10 windows, as in v2.
- **Concurrency:** only reads and small writes to `~/motar_tmp` ran during training. There were no tests,
  simulators or GPU jobs, and the tree stayed frozen.
- **Disk:** 1.31 GB free at chain start, 0.82 GB at the end.
- **Not shown:** that v3 crashes less than v2 under evaluation, which needs its own preregistered evaluation, or
  anything about a real OAK-D. The phantom-surface risk from sky, textureless, shadow and specular invalid bands is
  NOT_TESTED.
