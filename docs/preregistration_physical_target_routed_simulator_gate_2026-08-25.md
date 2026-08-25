# Preregistration — routed physical-target simulator gate

Frozen: 2026-08-25, before any routed PhysX cell was executed.

## Question and claim boundary

This is an engineering evaluation of the new target-motion environment, not PPO training and not
hardware validation. It asks whether a deterministic global route removes unreachable random
waypoints without introducing contacts, invalid rigid-body states, tracking failure, or persistent
zero-command stalls.

The only manipulated factor in the matched comparison is route mode:

- `route_off`: physical ref5in target, waypoint pattern, existing bounded local planner;
- `global_astar_v1`: the same target and waypoint task with exact inflated-AABB global routes.

The pursuer policy is not loaded and receives no route information. Simulator ground-truth bar
geometry is used only by the environment controller that moves the target. Results cannot support
a real-hardware claim, a pursuer-policy improvement claim, or arena-wide connectivity at 300 bars.

## Frozen grid

- seed: `827`;
- route arms: `off`, `global_astar_v1`;
- densities: `70, 150, 205, 300` bars;
- exact target speeds: `0.6, 0.9, 1.2, 1.5 m/s`;
- `32` environments per cell;
- `300` RL intervals per cell, first `20` excluded from tracking metrics;
- physics `dt=0.01 s`, `10` physics steps per RL interval (`10 Hz` command update);
- ref5in target, `40 x 40 x 3 m`, `bars_h3`, `navrl_band`, waypoint pattern;
- route geometry: actual per-asset axis-aligned bar AABBs, all-orientation target support,
  `0.45 m` tracking reserve, `0.25 m` grid, exact continuous AABB line-of-sight checks;
- each route arm and speed is created in a fresh process. Density cells share only that process and
  report counter deltas per cell.

The fixed grid is not expanded, narrowed, or retuned after seeing results. A failed cell is retained.

## Existing physical-controller gates

Each cell is evaluated against the previously used limits, unchanged:

| metric | gate |
|---|---:|
| tracking RMSE | `<= 0.35 m/s` |
| realized/requested mean speed ratio | `>= 0.80` |
| target contact-step fraction | `<= 0.01` |
| immediate local-step infeasible fraction | `<= 0.01` |
| motor saturation fraction | `<= 0.15` |
| maximum tilt | `<= 60 deg` |
| finite but invalid OBB state fraction | `0` |

The historical `planner_infeasible_fraction` field is interpreted only as an immediate local-step
failure, never as proof that no global route exists.

## Routed-mechanism quality gates

These gates apply to `global_astar_v1` and are checked before interpreting speed performance:

1. source/config receipt and all 32 cell records must be present; a missing or crashed arm is
   `VOID_EXECUTION`;
2. route plan successes divided by attempts must be at least `0.99` at 70 bars;
3. fail-closed fallback intervals divided by all commanded intervals must be at most `0.01` at
   70 bars;
4. at `70 bars x 0.6 m/s`, completed global goals per environment must be at least `0.5` during
   the 30 s run; this rejects a planner that succeeds once and then silently parks;
5. routed planning wall time, including device-to-host geometry transfer, is reported. It is an
   engineering throughput measurement, not a policy metric; no post-hoc latency threshold is added
   to this simulator gate because the separate CPU preregistration already froze that threshold.

High-density route failures and fallback are reported, not hidden by selecting only large connected
components. The 70-bar mechanism gate is the minimum viable curriculum-start contract.

## Verdicts

- `PASS_ROUTE_MECHANISM`: all routed-mechanism gates pass.
- `PASS_FULL_1P5_CONTRACT`: route mechanism passes and every routed `1.5 m/s` density cell passes
  all existing physical-controller gates.
- `PASS_DENSITY_CONDITIONED_ENVELOPE`: route mechanism passes and every density has at least one
  preregistered passing speed; the highest passing speed per density is reported without interpolation.
- `BLOCKED_PHYSICAL_TRAINING`: route mechanism fails, any density has no passing registered speed,
  or execution integrity is void.

Even `PASS_FULL_1P5_CONTRACT` authorizes only a separately preregistered short fresh PPO smoke. It
does not authorize checkpoint reuse or a long training run.

## Matched-arm interpretation

For every density-speed cell, route-on minus route-off deltas are reported for speed ratio, tracking
RMSE, contact, immediate infeasibility, and invalid state. With one seed these are descriptive
mechanism checks, not confidence-bounded causal effects. No claim requires a favorable delta; safety
and route gates determine the verdict.
