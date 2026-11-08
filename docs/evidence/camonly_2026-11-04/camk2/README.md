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
