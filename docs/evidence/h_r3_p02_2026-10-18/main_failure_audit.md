# H-R3 P02 main integrity failure: independent read-only localization

Source HEAD: `d64cf84185018ab720040f266a4a481929cfa62b`. No repository edits or GPU execution.
Raw: `/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/h_r3_p02_v1/HR3-P02`. All step indices below are zero based.

| Seed | Arm versus B0 | Earliest genuine prebranch state/sensor mismatch | First prebranch action/velocity difference |
|---|---|---|---|
| 4601 | B1 | step11 / env1 | action [12, 22]; velocity [12, 22] |
| 4601 | B2 | step11 / env1 | action [12, 22]; velocity [12, 22] |
| 4601 | B3 | none | action None; velocity None |
| 4601 | B4 | step11 / env1 | action [12, 22]; velocity [12, 22] |
| 4601 | B5 | step11 / env1 | action [12, 22]; velocity [12, 22] |
| 4602 | B1 | step7 / env2 | action [8, 14]; velocity [8, 14] |
| 4602 | B2 | step7 / env2 | action [8, 14]; velocity [8, 14] |
| 4602 | B3 | step7 / env2 | action [8, 14]; velocity [8, 14] |
| 4602 | B4 | step7 / env2 | action [8, 14]; velocity [8, 14] |
| 4602 | B5 | step7 / env2 | action [8, 14]; velocity [8, 14] |

Seed4602 first onset: B0 resets env56 at step7; B1–B5 do not. Env2 has t0=18 in both B0/B1. Seed4601 first onset: B0/B3 reset envs120/123 at step11; B1/B2/B4/B5 do not. Env1 has t0=15. Immediately before these steps reset sets are empty.

At both earliest failing rows, all separately recorded numeric before/actual/root positions, velocity, action, command, target, belief position and RNG probes are bitwise equal. Only full-state and combined raw-sensor hashes differ initially. The first prebranch action/velocity differences follow one step later (4602: step8/env14;4601:step12/env22). The JSON retains exact values, per-field first locations and surrounding reset sets.

`warp_env_manager.py:40–48` is the strongest verified source coupling: its reset method ignores env_ids when rewriting the entire shared vertex buffer, and uses env_ids only for mesh refit. Recomputing all static mesh vertices from current actor poses after physics can change unrelated ray casts, for example after actor quaternion normalization. This is consistent with the chronology, not a proved numerical cause: mesh vertices/static actor poses were NOT_RECORDED. Full root setter outside reset and unrecorded orientation/derived-state changes also cannot be conclusively excluded retrospectively.

Combined raw-sensor hashes cannot distinguish RGB, depth or LiDAR as the first changed modality. Orientation, angular velocity, observation rows and histories were hashed but not independently retained numerically. RNG/target differences occurring much later follow divergent outcomes/reset schedules and do not explain onset.

Keep the main run INTEGRITY_FAIL. No salvage exclusions. A future separately approved diagnostic must inspect global mesh writes and snapshot per-modality raw products, static actor/root orientations, mesh vertices and all derived kinematics around the first differing reset, at N128. N16 smoke PASS was insufficient.

Git Traceability: read-only at source HEAD above; only `/tmp/motar_hr3_p02_20261018/main_failure_audit.py`, `.json`, `.md` written. No repository changes, commits, GPU work, push or merge; raw/checkpoints unchanged.

The SMOKE02 blind spot is observed directly: both N16 runs first reset env4 at step27. At forced step40 baseline naturally resets [5,6], while forced resets [5,6,15]. Both therefore execute the global Warp refresh at the same times around the injection; this does not exercise reset-versus-no-reset geometry refresh before the first natural reset.

Future tests, PLANNED and NOT_EXECUTED: (1) CPU independent per-env geometry oracle with actor-to-vertex maps and fake refit counters; perturb an unselected scene input pose by a tiny increment/quaternion normalization, reset another scene, and require unselected geometry bytes to remain unchanged, selected transforms correct, and order/split invariance. This tests isolation, not the unrecorded historical mechanism. (2) Separately preregistered real N128 B0 forced-reset pair at an early step with no natural reset in the reference, preferably before its first natural reset; retain before/after static actor roots, vertex arrays, derived kinematics, root orientations/angular velocities and separate raw sensor modalities, then require all untouched env future state/sensors/probes byte-identical. No implementation or GPU diagnostic was performed in this audit.
