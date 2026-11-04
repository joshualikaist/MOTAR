# Amendment 2 to `prereg_sitl_reference_g8g9_v1.md` (2026-11-05, before any G8 or G9 run)

## What changes (G8 only)

After the two Policy F cells, G8 runs the same two cells (70 and 205 bars, seed 4501, quota 1) with the pursuer held
at zero action.
- **Hook:** `tools/zeroaction_hook.py`, loaded by `tools/zeroaction_site/sitecustomize.py` when `MOTAR_ACTION_ZERO=1`.
  It runs the policy and returns zeros.
- **Effect:** the pursuer hovers (zero velocity and yaw rate; the altitude PI holds 1.0 m), so every episode runs to
  its 600-step timeout. These cells record the occlusion trace only.

**New check (target independence):**
- **Measure:** for every env's first episode, the target positions (`t_pos`) of the zero-action cell must equal those
  of the F-flying cell, exactly, on their common frames.
- **Readings:**
  - **DUMP_OK** now also requires this.
  - **TARGET_DEPENDS_ON_PURSUER** if it fails, with both cells OK.

## Why

The E2 target does not read the pursuer's state (`target_motion.resolve_target_behavior_contract`:
`uses_privileged_pursuer_gt: False`). Its trajectory in a scene should therefore not depend on how the pursuer flies.
- **What S3b needs:** S3b (SITL) replays the target. Its development runs used trajectories that end when the Isaac
  episode ended, about 3-6 s, and SITL captures take longer.
- **What the zero-action cells give:** the full 60 s trajectory of the same scenes. The check shows the replay is
  exact, not an approximation.

## What does not change

- the F cells, their recorders and the noninterference gate;
- G9.

## Checks

`tests/test_zeroaction_hook.py`, 2 tests:
- zeros of the same shape, with the policy still called;
- the install flag is required.

## Cost

Two more cells of 600 steps each, about 5 min of GPU.
