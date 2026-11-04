# Preregistration: C-Planner direction-bound gate D1-F — full-bound adversary (review I1)

Fixed 2026-11-04 before the gate run. Follows the direction-bound repair
([`prereg_cplanner_dirbound_v1.md`](prereg_cplanner_dirbound_v1.md), amendment 1). The repair is PASS after its
independent review ([`../evidence/cplanner_2026-11-04/dirbound/REVIEW_r1.md`](../evidence/cplanner_2026-11-04/dirbound/REVIEW_r1.md)).
CPU only. Beads `motar-3p3.36`.

**Why.** Review finding I1 showed that D1 has almost no power at the scale of eps:
- its cone moves rays by at most 0.53 eps at bar range, with no origin offset;
- its aimed candidates rarely land near R;
- the unrepaired eps = 0 configuration also gave 0 FALSE_CERT.

So D1 is not evidence of eps-scale soundness. The reviewer's own probe P1 was: a full-bound adversary
with 81 rays per beam, heights near 0 and 3 m, and candidates within 1.5 mm of R. P1 gave 0 FALSE_CERT on the
repaired configuration and 1,577 on the unrepaired one. D1-F makes that construction a preregistered gate. It
tests the unchanged certifier of `8e2b77d`; nothing in `cplanner/src/`, `native/` or `motar/` changes.

## Gate D1-F (`cplanner/tests/check_direction_bound_full.py`)

Ground truth and casting are local to the test and share no code with `src/`:
- a slab caster for axis-aligned 0-3 m boxes, with no ground plane;
- an exact 2-D segment-rectangle distance.

**Adversary.** For every beam with manifest direction d and declared origin o, the probed rays are
o + e0 + t w, with w = normalize(12 d + e12 - e0).
- e0 and e12 range over 0, ±a u, ±a v, ±a (u+v)/√2 and ±a (u−v)/√2, with a = 0.99 eps.
- That gives 81 rays per beam, each within a of o + t d for t in [0, 12]. The measured maximum is reported and
  gated (`adversary_within_bound`: ≤ eps).
- The declaration is the most permissive result over the family: NO_HIT if any ray misses within 12 m, otherwise
  the largest distance as the HIT range (no adapter margin), and NEAR_CLIPPED below 0.2 m.

**Scenes.** 2,000 scenes, with seed0 20,261,112 and 6 workers, run twice.
- 1-6 catalogue bars, 0.8-4 m from the vehicle.
- Vehicle heights: 40 % in [0.3, 2.7] m, 20 % in [2.95, 2.999], 10 % in [2.999, 2.99999], 20 % in [0.001, 0.05]
  and 10 % in [0.00001, 0.001] m.
- Candidates: 4 random segments, plus up to 12 segments aimed by bisection so that their exact clearance to a
  random bar is R − delta, with delta ~ U(−0.3 mm, 1.5 mm).

**PASS** iff all of these hold (both runs, identical after removing timing):

| Check | Requirement |
| --- | --- |
| `completed`, `all_steps_accepted` | every scene, every configuration |
| `adversary_within_bound` | maximum family offset ≤ eps |
| `DIR_zero_false_cert` | `MOTAR-72x4-RUNTIME-DIR-NATIVE` (repaired): 0 FALSE_CERT on the adversarial scans |
| `DIR_nonvacuous_certified` | the repaired configuration certifies > 0 segments |
| `ZERO_detected_at_eps_scale` | `MOTAR-72x4-RUNTIME-NATIVE` (eps = 0) on the same scans: FALSE_CERT > 0 |
| `ZERO_detected_at_ordinary_heights` | that is also true at heights in [0.3, 2.7] m |
| `EXACT_zero_false_cert`, `EXACT_nonvacuous_certified` | manifest-only scans: 0 FALSE_CERT and > 0 certified for both |
| `VERB_identical` | on every 20th scene, `MOTAR-72x4-RUNTIME-DIR` (verbatim Python) gives results identical to the native configuration |

FALSE_CERT means a STATIC_BAR_CERTIFIED segment whose exact clearance to some bar is ≤ R.

**Driver.** `cplanner/tools/run_dirbound_d1f.py` runs from a frozen `git archive` copy.
- It refuses unless `src/`, `native/` and `motar/` are byte-identical to the repair's frozen copy (`8e2b77d`).
- It builds the native library in the output directory.
- Host rules: kernel fault record, boot id and online CPU list checked before and after each command;
  6 workers beside the GR-1 GPU training. Inputs are bound before and after.

**Verdict.** PASS on both runs gives **CONDITIONAL**, decided by an independent review. If the review passes, D1-F
replaces D1 as the eps-scale evidence of the repair. Otherwise the result is **FAIL**, which would be a finding
against the repair itself and goes back to the user.

## Limitations

- The adversary samples 81 rays per beam, not the whole tube of radius a.
- Scenes are synthetic, with axis-aligned catalogue bars, the charter §4 scope.

## Disclosed development (not evidence)

D1-F dev on 120 scenes, seed0 20,261,199, 6 workers, on the uncommitted tree: PASS.

| Configuration | FALSE_CERT | Certified |
| --- | --- | --- |
| repaired | 0 | 296 |
| eps = 0 | 114 (7 at ordinary heights) | – |
| exact scans, repaired / eps = 0 | 0 / 0 | – |

The verbatim configuration was identical on 6 scenes.
