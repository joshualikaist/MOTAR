# 14 — G4 P0 source-audit amendment (2026-10-20)

Authority: the user's CPU-only audit/fix request, base `13d5f195399bf1a7f1cdf9ddeccd3338d8e34c0f`.
This explicitly supersedes conflicting G4 implementation assumptions below; it does not rewrite
07–09, historical experiments, preregistrations, or U1–U7 receipts.

## Corrected assumptions and implementation

1. A belief gate is not identity evidence. At zero covariance its old exclusion radius was
   `sqrt(9.21)*.15 = .45522 m`, not .15 m. `exclude_target` now preserves every valid
   first hit, including ambiguous/true target returns. No semantic GT is used. Self-blocking
   visibility and safe target-approach handling remain BLOCKED; rays must not be filled to far.
2. LiDAR slant ranges are projected by each row's elevation before taking horizontal minima.
   A 5 m return at20 degrees permits at most4.698463 m horizontally before margin. Adjacent
   sector minima remain an interpolation assumption, not a physical 3-D free-volume proof.
3. Camera LOS starts at the calibrated optical XY origin, consistently with the FOV transform.
   Roll/pitch and full3-D silhouette/ray visibility remain unqualified.
4. The hard sensed-point radius has a non-shrinkable physical floor of
   `sqrt(.14²+.14²+.04²)=.201990099 m` from the frozen collision box. The .198 m legacy
   soft prior bounds only its level XY projection. Neither is a measured uncertainty allowance.
   Centre-segment known-free containment still does not certify whole-body containment.
5. Stopping feasibility uses the larger of the empirical total and an upper bound on the
   planner's own first-order zero-command response, with reaction added once to each model.
   Low-speed table extrapolation alone underbounded that response at .6 and1.2 m/s.
   `dynamics.braking_distance`, production governor, controller and empirical knots are unchanged.
6. Invalid free contexts fail closed rather than clipping infinite radii into finite free space;
   invalid physical/dynamic parameters are rejected.

## Governor wording correction (M2 and risk R4)

The riskcap **cap** has a2 m/s floor. Executed command speed is `min(requested_speed, cap)`.
At .1 m clearance requests0, .6 and1.2 m/s remain0, .6 and1.2; requests at least2 are
capped at2. Earlier M2 statements “never brakes below it” and “still executes2” without
conditioning on the request are incorrect. The code arithmetic was already correct.
Negative stopping-margin telemetry is not itself a lower-speed command, nor a hard feasibility
filter. A zero command passes zero to Lee velocity control; actual velocity decays over time.
No claim that every candidate failure implies physically unavoidable collision is justified.

## U4 interpretation retained

At exact nominal2.00/2.50/3.52 m/s, dt_sub=.01 s and tau=.4 s, the surrogate's zero-command
asymptotic travel is .780/.975/1.3728 m, versus empirical design knots .82/1.05/1.57 m.
Signed errors are -4.878/-7.143/-12.561 percent. The last misses the10 percent goal and is
LIMITED_VALIDITY; all remain within the user's25 percent CPU-continuation gate. This permission
is not plant qualification. Historical Lee measurements started near, not exactly at, these
nominal speeds; the detailed audit separates the measured speeds and physical timesteps.

## Current gate and scope

G4 U1–U7 CPU implementation is complete; the independent audit found unresolved P0 limitations.
**G5 HOLD** until target-return/foreground evidence and physically justified free-volume/body
containment have an approved resolution and independent regression evidence. Any plant/sensor
integration also requires calibration, per-env RNG/timestamp/identity contracts and explicit approval.
No GPU, raw replay/calibration, production change, PPO, push or merge is authorized in this audit.
RGB-D Camera + LiDAR, Frozen Policy F and actor898 remain unchanged. Disk stop is3GB.

Full file/function inventory, issue tables, independent counterexamples, model comparisons and
CPU receipts: [G4 precise source audit](../../evidence/boia_g4_p0_2026-10-20/AUDIT_REPORT.md).
