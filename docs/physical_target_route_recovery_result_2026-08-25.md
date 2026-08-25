# Physical-target route recovery result — seed 827

The frozen evaluation-only recovery forensics completed all 8 route-on cells and verified the
receipt. It supports the descriptive verdict `RECOVERY_DOMINANT`: the repeated local recovery
state, rather than the initial route plan, dominates the observed failure population.

| Diagnostic | Result |
|---|---:|
| Receipt / cells | `8/8 verified` |
| Local invalidations → attributed fallback intervals | `358 → 35,666` (`99.6257×`) |
| Fallback age | median `101`, p90 `219`, max `287` steps |
| Unique local origins | `n=200` |
| Hard-free / soft-unsafe | `97.0%` (Wilson 95% lower `93.61%`) |
| Exact hard-safe anchor connector | `96.5%` (Wilson 95% lower `92.95%`) |
| Replan statuses | `unsafe_start 3774`, `ok 101`, `no_path 82`, `unsafe_goal 79` |
| Initial-plan statuses | `ok 349`, `unsafe_start 17`, `no_connected_goal 6` |
| Rounded-local vs square-route disagreement | `1,832` events |

The evidence is consistent with a recovery deadlock: after a local step invalidation, the target
can remain hard-free while outside the route planner's soft envelope, causing repeated
`unsafe_start` replans and fail-closed fallback. This does not show that the arena lacks a global
route, and it does not authorize reducing the frozen `0.45 m` reserve or tuning the planner.
The 300-bar `no_path` population remains a separate topology limitation.

This was an evaluation-only diagnostic using the neutral pursuer command. It changed no target
commands, planner decisions, reward, observations, termination, PPO, or original attempt2
artifacts/evaluator. Physical PPO remains blocked until a separately preregistered recovery
change passes the unchanged routed gate.

Evidence and contracts:

- [forensic summary](../results/navrl_physical_target_route_recovery_forensics_seed827/summary.md)
- [forensic receipt](../results/navrl_physical_target_route_recovery_forensics_seed827/receipt.json)
- [forensic preregistration](preregistration_physical_target_route_recovery_forensics_2026-08-25.md)
- [original attempt2 gate](../results/navrl_physical_target_routed_gate_seed827_attempt2/summary.md)
