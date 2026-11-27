# Preregistration: SITL reference runs G8 and G9 (v1, 2026-11-05)

Branch `camonly/dev`, which hosts the SITL unit tonight because G8 reuses the camonly cell path.

User approval:
- the GPU session, 2026-11-05: "이거 하고 gpu 세션 바로 시작하자";
- the direction: real-vehicle flight after a Gazebo/PX4 SITL check ("실제 기체 비행인데, 그 전에 이게 가능한지 gazebo px4
  sitl 로 한번 다시 검증해보려고 하는거야").

The plan is `docs/plans/sitl_validation_plan_v1.md`. These two short runs are references for its stages, not claims.

## G8: reference dump for the bridge's observation-equivalence check (`tools/sitl_g8_ref_dump.py`)

**Cells:**
- Policy F through the unchanged camonly cell path, condition `full` (native F);
- GR-0's grid condition (E2, A3, 128 envs);
- 70 and 205 bars, seed 4501, quota 1.

**Recorders** (evaluation-only, reading detached tensors):
- `NAVRL_OBS_DUMP`: stride 1, cap 80,000 rows, so it never decimates;
- `NAVRL_KROC2027_OCCLUSION_TRACE=1`.

**Reading:**
- **DUMP_OK:** both cells OK, and every env's first-episode row (outcome, completion step, minimum relative
  distance) equals the untraced GR-0 S00 cell's.
- **RECORDER_INTERFERES:** otherwise, with both cells OK.
- **INCOMPLETE / VOID_HOST_FAULT:** as the runner defines them.

**CPU smoke:** the comparison gives 0 mismatches for C1 `full` against GR-0 S00, and 128 for `no_lidar` against
GR-0 S00, a negative control.

## G9: MOTAR's own velocity response in Isaac Gym (`tools/sitl_g9_lee_steps.py`)

**Simulation:**
- F's simulation built as the task builds it: `base_sim`, `navrl_bars_env` with 10 physics steps per 0.1 s,
  `navrl_quad`, `lee_velocity_control_navrl`, midpoint gains;
- 4 envs, 1 bar;
- no policy and no governor.

**Command:** the same vehicle-frame commands as the SITL tests A3/A4/A5, held for 10 physics steps. Altitude comes
from the task's altitude PI.

**Output:** per-env step metrics (10-90 % rise, onset, overshoot, 5 % settling, final mean) and their median over
envs. Reported only.

**Use:** the target response for tuning PX4's velocity loop on navrl_quad (S2). The expected first-order τ is about
0.4 s; that is a prediction, not a gate.

**First run:** this is the script's first GPU run. A failure is recorded and fixed in a new attempt.

## Concurrent work

These are evaluations: G8's outcome files are covered by T-LOAD; G9's metrics are in sim time (physics steps). The
SITL runs (real time) do not run alongside them.

## Cost

| Run | GPU |
| --- | --- |
| G8 | 2 cells with quota 1, about 5-8 min |
| G9 | about 2 min |
