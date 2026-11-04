# Amendment 1 to the direction-bound repair: gate D2 counts only unexcused disagreements, by direction

Dated 2026-11-04. Amends [`prereg_cplanner_dirbound_v1.md`](prereg_cplanner_dirbound_v1.md); that file is
unchanged. This is the one repair round the charter allows for an in-scope local defect (§5 stop rule).

## What happened

The gate ran from the frozen copy of `8e2b77d` (`~/motar_tmp/cplanner_dirbound_20261104/run/`). Verdict **FAIL**,
which stays recorded:

- D0 (seven R2' suites, eps = 0): PASS.
- D1 (2,000 scenes): PASS. 0 FALSE_CERT inside the bound; 46 FALSE_CERT at 100x the bound (power control).
- D3: FILTER_ARM_RETAINED. Certifiable rate 0.8162, 0 FALSE_CERT.
- **D2 (V1-v2): FAIL** on 1,728,000 beams, through K1, K3 and the range tolerance:
  - Trace index 14571 of `A/F_SHADOW/E2_obstacle_aware__205bars__seed4301`: in the beams of column 54 (rows 0-3)
    the kernel reported HIT at 1.51-1.61 m. The float64 oracle along the same ray finds nothing within 12 m (rows 0,
    1, 3) or a bar at 25.8 m (row 2).
  - All four hit points lie on the vertical corner edge of bar 167, within 1e-6 m of its surface
    (`dirbound/v1v2_failure_frame14571.txt`, post-hoc).
  - The float32 ray touches the corner and the float64 ray passes just outside it. This is corner grazing in the
    **conservative** direction: a declared obstacle where the recorded layout is free.
- Along the manifest, 2 more disagreements of the M2 kind appeared (trace index 5575 of the 160-bar cell). Each was
  4.7e-7 m deep, within eps.
- K2 held: the largest lateral offset was 2.96e-6 m.

## Why the rule changes

The preregistered K1 / K3 / range rules demanded agreement in both directions. Soundness under the declared bound
needs only that declarations are not **more permissive** than the scene beyond the bound.

- A conservative declaration only shortens free evidence, adds a surface or ambiguates a beam (the core's `_step`,
  `_certify_one`). It cannot make a certificate unsound.
- A conservative disagreement far from any surface would still point to a replay defect (a phantom object). So it
  is accepted only when it is grazing.

## Amended gate D2-A1 (`cplanner/tests/v1v2a_kernelray.py`; the original D2 script is unchanged)

Every disagreement between a declaration and the oracle along a ray is classified:

- **PERMISSIVE.** NO_HIT while the oracle hits within 12 m (outside V1's 0.1 mm range band), or a HIT range 12 v that
  overshoots the oracle distance by more than 0.5 mm. Allowed only if the ray is at most LIMIT deep inside the object
  it misses. An overshoot is never allowed.
- **CONSERVATIVE.** NEAR_CLIPPED while the oracle is beyond 0.2 m (outside the band), or a HIT whose oracle distance
  is longer than 12 v + 0.5 mm, or infinite. Allowed only if the declared point lies within LIMIT of an object
  surface. For NEAR_CLIPPED the declared point is the nearest point of the first 0.2 m.

LIMIT is `DELTA_GEOM` = 1e-5 m along the kernel's ray, and eps = 1e-3 m along the manifest; both are as
preregistered.

**PASS** iff all of these hold:
- the run completes;
- along the kernel ray, 0 unexcused disagreements and 0 adapter HIT ranges beyond the oracle distance;
- K2 unchanged (lateral <= eps / 10);
- along the manifest, 0 unexcused disagreements;
- the largest permissive overshoot is <= 0.5 mm;
- the adapter mapping is exact;
- V1's positive controls hold (HIT, NO_HIT and target hits > 0);
- **rule controls** in every frame:
  - one real bar HIT at least 1 cm deep, rewritten as NO_HIT, must be flagged PERMISSIVE and unexcused;
  - one empty NO_HIT beam given a phantom HIT at 3 m must be flagged CONSERVATIVE and unexcused;
  - both control counts must be > 0, with every control flagged.

**Sample.** 1,000 frames per V1 cell from a **fresh** sampling seed 20,261,110, run twice; the two runs must agree
after removing timing keys. The same rule is also run on the first run's D2 sample (seed 20,261,109) and reported,
not gated, because that sample informed this amendment.

**Carry-over.** `cplanner/tools/run_dirbound_a1.py` refuses unless `cplanner/src/`, `cplanner/native/` and
`cplanner/motar/` in its frozen copy are byte-identical to the first run's frozen copy (`8e2b77d`). Then D0, D1 and
D3 of the first run stand for the repaired configuration.

**Host and verdict.**
- Two workers keep the load low beside a GPU evaluation. Host, input and online-CPU checks are as in the first run.
- If D2-A1 passes and everything carries over, the repair is **CONDITIONAL**, decided by an independent review.
- If D2-A1 fails, the repair is **FAIL** and goes back to the user (no further round).

## Development on the uncommitted tree (not evidence)

| Check | Result |
| --- | --- |
| trace index 14571 (the D2 failure) | 4 CONSERVATIVE disagreements along the kernel ray and 4 along the manifest, all within 1.03e-6 m of the bar surface (excused) |
| trace index 5575 (160 bars) | 2 PERMISSIVE manifest disagreements, 4.7e-7 m deep (excused, <= eps); none along the kernel ray |
| trace index 10752 (the M2 V1 violation) | 1 PERMISSIVE manifest disagreement, 1.7e-8 m deep; none along the kernel ray |
| 2 cells x 30 frames, seed 20,261,111 | PASS; rule controls 60/60 permissive and 60/60 conservative flagged |
