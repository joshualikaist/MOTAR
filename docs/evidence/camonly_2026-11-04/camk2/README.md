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
