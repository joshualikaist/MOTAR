# Preregistration — recovery-v2 lower-1.25 NO_CONNECTOR geometry forensics

Frozen after the verified `FAIL_ROUTE_MECHANISM` lower-1.25 32-cell gate and the CPU packed
diagnosis. Evaluation-only. No 32-cell rerun, controller change, PPO, 1.5 m/s claim, env-count
change, or `0.45 m` retune is authorized.

## Question

When recovery-v2 latches `NO_CONNECTOR` from BRAKE (in-phase or same-interval), does a
radius-3 exact hard-safe anchor already exist at that state? Separately, when CONNECT is
soft-free and still latches, what is the resume-replan status?

The packed receipt can classify the latch. It cannot recompute the 7×7 hard-envelope connector,
because bar poses are not in the npz.

## Frozen probe

- Source bytes: the verified gate commit `2b151d9a4c4fe078ecc027152e5642fa857a2e2f` and the
  bound heading-rest braking receipt. Do not mix later runtime edits.
- Seed 827, 32 env, 300 steps, gain 2.5, `baseline_1p25`.
- Cells: the four 70-bar recovery-v2 speeds `{0.6, 0.9, 1.2, 1.25}` only. Density knots 150/205/300
  are out of scope for this isolation probe.
- Observer attaches at evaluation time. It must not change target commands, planner choices,
  observations, reward, termination, or the 32-cell evaluator.
- Output directory must not be the 32-cell gate path. VOID/incomplete siblings cannot be
  combined with a later attempt.

## Gates (descriptive; cannot pass the 32-cell mechanism)

Report, with Wilson 95% lower bounds where n≥20:

1. Fraction of BRAKE-origin `NO_CONNECTOR` entries (packed classes
   `brake_no_anchor_likely` and `same_interval_brake_no_anchor_likely`) for which at least one
   radius-3 exact hard-safe connector exists.
2. Fraction of those entries that are still hard-free / soft-unsafe at latch.
3. Resume-replan status counts for CONNECT-origin latches with positive soft margin.

If (1) is high, the latch is a search/budget failure, not missing hard-free space. If (1) is
low, the 7×7 neighbourhood is empty at the moment BRAKE needs it. Neither outcome authorizes
changing `0.45 m`, gain, or the 32-cell thresholds.

## Claim boundaries

A completed probe cannot be read as a recovery-v2 gate pass, a 1.5 m/s result, training
authorization, or hardware validation.
