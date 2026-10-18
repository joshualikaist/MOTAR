# P02 amendment2 — reset transaction repair after failed mechanics smoke

2026-10-18, before any two-seed pilot or arm-effect measurement. SMOKE01 at source125690c
is FAIL and remains immutable with its report/manifest under docs/evidence/h_r3_p02_2026-10-18/.
Two B0 mechanics processes completed. The runner stopped before B1–B5 or main.

Observed: normal B0 resets env0/env8 at98 and env15 at99; forced B0 resets env15 at40 and not99.
Unaffected env8 first differs in state/raw-sensor hash at99; action differs at100. Exogenous
probe and unaffected target trajectories remain identical. Full actor-root writes on unrelated
reset are a plausible cause; the fresh forced-reset test is the deciding independent check.

The P02-only instance wrapper will suppress root writes inside serialized manager/task resets,
collect their reset envs, then issue one indexed actor-root setter at the end of the task-reset
batch, for all actors in only those environments. DOF profiles are rejected. Original writes
outside a reset transaction are unchanged. This also follows the installed Isaac Gym tensor
setter guidance (one call of each setter between simulate calls). The five derived-buffer
protections of amendment1 remain. No production implementation or physical model is edited.

The forced-reset diagnostic still resets env15 at logical step40, now by adding env15 to the
ordinary termination/reset transaction before its selection, after physical terminal telemetry
has been captured. This avoids a second setter after the completed env step. Other environments
receive no injected termination, extra perception update or noise draw.

Fresh mechanics output: HR3-P02-SMOKE02 under the original output root. SMOKE01 is never reused,
renamed or overwritten. Seeds, N, steps, all eight processes, arms, fixed control window, metrics,
exact state/sensor/action/probe equality and safety gates are unchanged. No additional pilot seed
or event is authorized. Main still requires committed PASS mechanics evidence and identical code
hashes. A failed fresh equality gate blocks main; it is not grounds to relax tolerances.
