# NavRL joint speed-allocation diagnosis — preregistration

This is an evaluation-only, single-cell diagnostic of the frozen ep25000+riskcap candidate.  It
does not alter or search riskcap parameters, and it does not train PPO.

## Frozen condition

- 205 bars, deterministic original action, seed 379, 4,097 held-out episodes
- checkpoint SHA-256 `f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40`
- riskcap and braking constants are identical to the completed campaign
- source bytes, checkpoint bytes, evaluator, Python environment and result are receipt-bound by
  `eval_navrl_v2_density_sweep.sh`

## Measurements

Every action-selection step records actual horizontal speed, requested and executed command,
actor-safe directional minimum LiDAR clearance, braking distance/margin, normalized XY action delta, and
requested/realized heading-rate and curvature proxies.  Steps are attributed after termination to
capture/crash/timeout and binned by decision-time actual stopping margin:

`<0`, `0–0.5`, `0.5–1.5`, `>=1.5 m`.

The last 1.0 s before every cause-attributed bar contact is aggregated separately.  Heading rate is
a finite difference of command or actual velocity bearing, and curvature is heading-rate divided
by speed only when both adjacent speeds are at least 0.25 m/s.  They are explicitly proxies, not a
planner's required curvature or a reconstructed continuous path.

## Fixed gates

Quality requires at least 100 bar-contact episodes, 500 pre-contact steps and 1,000 capture steps.
Conditional on quality, the descriptive association gate passes only when:

1. at least 50% of the last-1-s bar-contact steps have negative actual stopping margin; and
2. that rate exceeds the capture-episode step rate by at least 10 percentage points.

A pass supports only an association between unsafe speed margin and contact.  It does **not** show
that speed caused the contact, identify a controller change, or authorize post-hoc riskcap tuning.
Failure leaves the adaptive-speed hypothesis unconfirmed; proxy distributions remain descriptive.
