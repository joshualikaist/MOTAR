# Amendment 3: G9 one-bar scene correction before attempt 3 (2026-11-05)

Amends only G9 in `prereg_sitl_reference_g8g9_v1.md`. G8 and all historical attempts remain unchanged.
The user explicitly resumed through G7 on 2026-11-05: "오케이 G7 까지 얼른 다 진행하자".

## Reason and source

Attempt 1 lost its report to a locale-decoding failure. Attempt 2 produced metrics, but two envs tipped over;
only env 1 was used as an exploratory reference. Its robot and obstacles retained random placement.
The prepared placement-only repair was independently audited before execution. The audit found that raw
SimBuilder loads `NAVRL_MAX_BARS` assets (default 150), whereas `NAVRL_NUM_BARS=1` controls the task's active
density only. Historical actual counts were not recorded, so the older attempts are not relabelled as measured
150-bar runs. They are not the qualified one-bar response reference.

Use a new additive runner, `tools/sitl_g9_lee_steps_v2.py` (SHA-256
`a41a4e1aadacabc023f37ae9b1efddbf01b212b18eeff559b53c35e87c2441a1`). Preserve `tools/sitl_g9_lee_steps.py`.
The independent preparation audits and CPU smoke are connected in `docs/evidence/sitl_2026-11-05/g9/`.

## Fixed changes

- Pin the 40 × 40 × 3 m arena, `bars_h3`, `NAVRL_NUM_BARS=1`, **`NAVRL_MAX_BARS=1`**, legacy virtual target,
  and no distractors. Reject inherited conflicts before importing the simulation, and require the actual obstacle
  tensor and asset counts to equal one.
- Start every robot at `(bounds_min.x + 5, bounds_min.y + 5, 1)` with identity orientation and zero velocities.
  Move its one bar to `(bounds_max.x − 2, bounds_min.y + 2)`; retain the bar's configured height. Submit PhysX state
  and refit Warp after relocation.
- Keep the controller, midpoint gains, altitude PI, schedule, four envs, physics/action dt and metric definitions
  unchanged. No policy or governor is used.
- Record effective scene settings, actual asset count, initial/final robot position, bar position, minimum bar
  center XY distance, peak robot contact force, whole-run altitude extrema and maximum tilt, and finite-sample
  checks for all four envs. These are diagnostics, not a new numerical acceptance gate.
- Require the explicit approved full commit and clean source tree. Journal retrieval failures propagate.
  Write the report exclusively; preserve an INVALID report and return failure if host faults increase, source
  identity/cleanliness changes, or samples are nonfinite.

## Output, interpretation and stop rules

Attempt 3 writes to a fresh `results/sitl_ref_v1_r2/g9_attempt3/g9_lee_steps.json`. Its launch log, host-state and
resource records are kept separately under `~/motar_tmp/g9_resume_20261105/`. No older output is overwritten.
Use only the installed, committed runner with `PYTHONPATH` resolving inside `MOTAR-camonly`. Run alone on the GPU,
with a memory cap, disjoint-to-offline CPU affinity, bounded runtime and host-fault checks before/after.

G9 remains **reported only**. `RECORDED_REVIEW_REQUIRED` is not a scientific PASS. Report all four envs, including
any contact, tip, incomplete response or unusable metric; do not select a clean env afterwards to claim a valid
four-env reference. A flawed attempt is retained, and any further repair is recorded before a new output.
Host faults void the attempt and stop the queue. Independent review precedes a positive interpretation.

The legacy sampling alignment is retained: world velocity is refreshed after a physics step while vehicle
orientation/body angular velocity were refreshed before it. The within-step uncertainty is one physics dt
(0.01 s); no exact PX4 latency-equivalence claim follows from this reference.

## Checks and order

Before launch: parse the installed source, check its reviewed hash, exercise the accepted scene and rejected
inherited scene settings and journal failures without importing Isaac/CUDA, and commit this amendment/index.
The run itself is the first GPU smoke of the corrected path; do not treat a CPU smoke as simulation validation.
After GR-1b analysis/review is recorded: G9 attempt 3, then the already-preregistered G7 identity cell and six
treatment cells. A G7 identity failure stops that unit. No new PPO or checkpoint promotion is included.
