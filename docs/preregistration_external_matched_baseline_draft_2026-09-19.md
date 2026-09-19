# DRAFT preregistration — first external matched baseline — 2026-09-19

**Status: DRAFT. NOT EXECUTED. NOT AUTHORISED.** No external system has been installed, built or
run. This document fixes what a fair comparison would require, so that the decision to run it is
made against a written contract rather than improvised afterwards.

Candidate: **Elastic Tracker** ([selection audit](external_baseline_selection_2026-09-19.md)).
Secondary architectural reference: NavRL.

## 1. The honest headline

This comparison would be **`PARTIALLY_MATCHED`** by construction. It cannot be Class A in the strict
sense, and calling it Class A later would be wrong. The unmatched dimensions are named in §4 before
any result exists, so that they cannot be quietly dropped once numbers are available.

## 2. Question

Under one fixed arena, one fixed target trajectory family and one fixed success definition, how does
a learned sensor-only pursuit policy compare with a competent classical visibility-aware tracker?

This is the question C1–C7 cannot answer: MOTAR currently has no evidence that the learned policy
earns its keep against a non-learned tracker.

## 3. Matched quantities — fixed before execution

| quantity | value |
|---|---|
| arena | 40 × 40 × 3 m, MOTAR bar field |
| obstacle layouts | **replayed from MOTAR's frozen seeds**, not regenerated |
| densities | 70 / 115 / 160 / 205 bars |
| start states | replayed per episode from MOTAR's frozen initial conditions |
| target initial state | replayed |
| target trajectory | **replayed from the frozen TM-E2 arm**, identical sample path per episode |
| target-motion arm | E2 obstacle-aware (single arm; see §6) |
| close-approach radius | `success_radius = 0.5 m`, swept between steps |
| timeout | 600 steps × 0.1 s = 60 s |
| evaluation seeds | new, unused, fixed before execution |
| episodes per cell | 2048, with the counter **capped**, not compared (see §7) |
| primary outcome | close-approach rate (capture_rate) |
| guard outcomes | crash rate, timeout rate |

Replaying layouts and target paths, rather than regenerating them from a shared seed, is deliberate:
the two systems do not share a random number generator, and "same seed" would not mean "same world".

## 4. Unmatched dimensions — declared in advance

| dimension | MOTAR | Elastic Tracker | why it cannot be equalised |
|---|---|---|---|
| action space | velocity + yaw-rate command | trajectory, tracked by its own controller | the trajectory interface is the method |
| vehicle model | Isaac Gym quadrotor, max 2.5 m/s | its own model, chaser max 3.0 m/s / 6.0 m/s² | forcing MOTAR's limits would change its feasible set |
| target sensing | sensor-only camera evidence | depth map + separate target camera | its perception front-end is not MOTAR's |
| simulator | Isaac Gym | its ROS stack | neither runs inside the other |

**Speed limits must be reported, not silently equalised.** If the chaser limit is reduced to 2.5 m/s
to match MOTAR, that is a modification of the baseline and is declared as such.

## 5. Decision rule — fixed before execution

1. Verify the replayed layouts and target paths are byte-identical to the frozen MOTAR inputs. If
   not, record `REPLAY_MISMATCH` and stop.
2. Verify the success and timeout definitions evaluate identically on both sides on a synthetic
   check set. If not, record `TERMINATION_MISMATCH` and stop.
3. Only then compare outcomes, seed-paired, with the same BCa95 estimator used for the frozen result.
4. Report the result as `PARTIALLY_MATCHED` with §4 attached. **It does not become Class A.**
5. No claim extends to real flight, deployment or a general superiority statement in either direction.

## 6. Scope limits fixed now

* **One target arm (E2).** Running all four would multiply cost without changing the question, and
  E2 is the arm where MOTAR's frozen policy is strongest — deliberately not the arm most favourable
  to a contrast in MOTAR's direction.
* **No retraining of MOTAR.** The frozen checkpoint is used as-is.
* **No tuning of the baseline against results.** Its released configuration is used; any change is
  declared before execution and recorded in the receipt.
* **Licensing must be resolved first.** Elastic Tracker's license is `NEEDS_CONFIRMATION`; the port
  does not start until that is answered.

## 7. Lessons already applied from the frozen campaign

The target-motion evaluation produced three process failures. This protocol fixes them in advance
rather than rediscovering them:

* **Cap the episode counter, do not compare it.** The frozen run overshot 2048 because the stop
  condition was `total >= target` evaluated per vectorised step. The matched run caps at exactly
  2048 or records the overshoot as a declared deviation.
* **Export per-episode records.** The frozen run's aggregate-only export made a post-hoc 2048 subset
  impossible and left `min_relative_distance_m` `NOT_RECORDED`. This protocol requires per-episode
  rows with an episode id, including per-episode minimum relative distance.
* **Fix the decision threshold in this document or have none.** The frozen run's 5 pp materiality
  rule was never preregistered and had to be withdrawn. Any threshold used here is written above, or
  the result is reported as an effect size with an interval and no verdict vocabulary.

## 8. Seed count

The frozen campaign used n = 3 evaluation seeds, which left exact permutation inference without
resolution. If this comparison is run, **n ≥ 5** is fixed here, before any result exists.
