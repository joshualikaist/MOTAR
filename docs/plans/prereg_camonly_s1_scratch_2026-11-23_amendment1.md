# CAM-S1 v1 amendment 1 (2026-11-23, before the training run)

Amends `docs/plans/prereg_camonly_s1_scratch_2026-11-23.md` (commit eb37f38). No setting of the training run changes.

## What happened

Development smoke01 (2 epochs, `~/motar_tmp/cams1_20261123/train_s1_smoke/`) installed every module as preregistered
(receipt: camera only, FOV 66 x 54, P9 on, camsafe blind 1.0 m/s / weight 0.5 / collision -40 / static-safety range
4 m, governor FREE 2.5, gates `70:0.284,85:0.266,100:0.249,115:0.242`, vehicle and depth treatments) and ran both
epochs, then exited 1 (TRAINING_FAIL, host faults 1 -> 1):

1. `early_stop_a2c_agent.py:1678` raised `UnboundLocalError: checkpoint_name`. The trainer sets that name only after an
   episode has finished; a policy from random initialisation finishes none in 2 x 32 steps (600-step limit). This is
   existing trainer behaviour for runs in which no episode ended, not a CAM-S1 code path; the run itself ends episodes
   within its first ~19 epochs.
2. No `[camsafe]` line reached the log: the report used `print`, and rl_games swaps `sys.stdout` / `sys.stderr` during
   its quiet startup. `navrl_depth_model.py` binds `sys.stderr` at install for the same reason.

## Changes

- `navrl_camsafe.py`: the report stream is bound at install (stderr), as in `navrl_depth_model.py`. No change to the
  rule, the penalty, the collision penalty or the static-safety computation (tests/test_camsafe.py unchanged, 15 OK).
- `tools/cams1_train.py`: the smoke runs 25 epochs (800 steps, more than one episode limit) in a new folder
  `train_s1_smoke02`. The scope "one development smoke" becomes "development smokes until one completes"; smokes are
  not results.
- Smoke02 must show: exit 0, at least one finished episode, two `[camsafe]` lines with capped_frac and penalised_frac
  above 0 (non-vacuous), a P9 install line, and the measured s/epoch.
