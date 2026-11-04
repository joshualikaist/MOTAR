# Amendment 1 to `prereg_sitl_reference_g8g9_v1.md` (2026-11-05, before any G8 or G9 run)

## What changes (G8 only)

G8 adds a third evaluation-only recorder, the opt-in state recorder hook.
- **Files:** `tools/staterec_hook.py`, loaded by `tools/staterec_site/sitecustomize.py` when `MOTAR_STATE_RECORD` is
  set. `tools/sitl_g8_ref_dump.py` puts the site directory on `PYTHONPATH` for G8 cells only.
- **Mechanism:** at every action request it reads, for every env, the tensors the observation was built from. It
  then returns the policy's action unchanged.
- **Recorded:**
  - episode step (`sim_steps`);
  - robot position and world velocity;
  - vehicle and full orientation;
  - body angular velocity;
  - target position and orientation (passed to the renderer);
  - the arena bounds (passed to perception);
  - the task's `prev_action`, as fed to perception;
  - `perception.step_count`;
  - the policy action.
- **Output:** `state_record_<bars>bars_seed<seed>.npz` per cell, beside the observation dump. The runner requires it
  and records its SHA-256.

## Why

A read-only map of the observation path (2026-11-05) found that the dump and the occlusion trace together lack
three quantities an exact bridge check needs:
- the body yaw rate (a robot-slice input);
- `prev_action` (four robot-slice entries);
- the global history phase (`step_count % 5`).

Deriving them by differencing would make the S3a gate test an approximation, not the bridge.

## What does not change

- the cells and the quota;
- the other two recorders;
- the noninterference gate and the readings;
- G9.

The new hook only reads. If it changed outcomes, the noninterference gate would show it (RECORDER_INTERFERES).

## Checks

- `tests/test_staterec_hook.py`, 3 tests:
  - the action is returned unchanged;
  - one row per call, with copies;
  - the file is written once and never overwritten;
  - the path is validated.
- The G8 comparison smoke of v1 is unchanged.
