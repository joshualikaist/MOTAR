# H-R3 pilot v1 — active translation for reobservation after occlusion (exploratory) — 2026-10-18

**Status: EXPLORATORY PILOT, COMPLETED. H-R3 stays PROVISIONAL.** One seed. The sample size is
not justified by power, and no confirmatory claim is made. No physical-safety claim is made either,
because P-L1 and G-L1 are not done. All numbers below come from
[`h_r3_analysis.json`](h_r3_analysis.json) and [`h_r3_posthoc.json`](h_r3_posthoc.json).

## What ran

- **Preregistration:** [`docs/plans/h_r3_pilot_v1.md`](../../plans/h_r3_pilot_v1.md) and `.json`,
  committed in `5f8719f` before any H-R3 code.
- **Source:** `5c8afde`, frozen during the run. Analysis was run with the same `5c8afde` tool.
  The post-hoc diagnostic tool is new in the result commit.
- **Run `HR3-P01_seed4502`:**
  - moving E2 `direct_l2_v1` receipt, density 160, legacy sensor lineage (off);
  - N = 128, 600 steps (0.1 s), seed 4502;
  - one process per arm (B0–B3), on an RTX 3070.
- **Process outcomes:** each process returned rc=0 in about 104 s, with a peak of 5,414 MiB under a
  7,020 MiB cap. GPU wall time was 416.5 s in total.
- **Disk:** 18.5 GB free afterwards.
- **Seed 4503 not run.** It was required only if there were fewer than 20 matched geometric events;
  there were 22.
- **Smoke (N16, 300 steps, session scratch):**
  - the first attempt stopped closed on a process-id mismatch;
  - the second attempt passed the 30% collision gate (B0/B2/B3 1/15, B1 0/15), so the intervention
    was allowed;
  - its analysis is in [`smoke_n16_analysis.json`](smoke_n16_analysis.json).

## Matching validity

- **Approach.** `get_env_state`/`set_env_state` restore only the curriculum state, so no snapshot
  fork was used. Each arm is a deterministic replay with the same seed.
- **Matched events.** An event is matched when two conditions hold:
  - t0 is identical in all four arms;
  - the per-env, per-step digest (actor observation, pursuer, target, tracker and camera flag) is
    identical for every step before t0.
- **Counts.** 125 of 128 envs triggered:
  - **107 events matched**;
  - **18 unmatched**: 9 differ in t0 and 9 in their prefix digest. By B0's class at t0 they are
    16 FOV_EXIT and 2 FULL;
  - 3 envs never triggered.

  The unmatched events are reported only, never analysed.
- **Cross-env coupling.** From step 10, any env that has not yet branched differs between B0 and
  every scripted arm.
  - **Cause, as observed in the smoke:** an arm-dependent reset in another env changed a
    not-yet-branched env's tracker by 1.4 mm. Its target did not change. This is consistent with a
    shared random stream that resets advance; the mechanism itself is not verified.
  - **Consequence:** the matched events are the early ones (t0 2–10), and no primary window
    completes before the onset. Inside the window the arms therefore do **not** share
    noise realisations.
- **Source of the B2/B3 divergence (post-hoc).**
  - In 20 of 22 primary events, B3's and B2's digests first differ at the very step at which their
    actions first differ.
  - In all 10 B2/B3-discordant events the divergence starts with B3's lateral command; in none
    does the noise stream diverge first.
  - The other 2 events diverge through the noise stream first, and every arm reobserves in both.
- **Target independence.** Before termination, target trajectories inside the window are identical
  across arms: maximum difference 0.0 m, over 107 events.

## Primary result (matched geometric occlusion, n = 22: FULL 18, CENTRE_ONLY 4, PARTIAL 0)

Primary metric: identity-correct camera reobservation within 2.0 s. A reobservation counts only
when the camera detects *and* E-L0 renderer pixels are ≥ 1. Crashes, captures and resets count as
failures.

Column definitions:
- **5 s:** W plus 3 s.
- **Δ distance:** the change in pursuer–target distance over the window, stopping before
  termination.

| Arm | Reobserved | Rate [Wilson 95%] | Median time (s) | Mean time, censored at 2 s | Collision in 2 s | Collision under arm control | Collision-free reobs | Capture by 5 s | Crash by 5 s | Δ distance (m) | Translation (m) | FOV retention | Camera / LiDAR-only fraction |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| B0 Policy F | 18/22 | 81.8% [61.5%, 92.7%] | 0.9 | 1.18 | 1 | 0 | 18 | 11 | 1 | -4.25 | 5.00 | 0.91 | 0.34 / 0.03 |
| B1 prediction | 7/22 | 31.8% [16.4%, 52.7%] | 0.9 | 1.67 | 0 | 0 | 7 | 6 | 1 | -2.02 | 2.47 | 0.83 | 0.11 / 0.05 |
| B2 + yaw | 9/22 | 40.9% [23.3%, 61.3%] | 0.9 | 1.55 | 0 | 0 | 9 | 4 | 1 | -2.11 | 2.58 | 0.95 | 0.20 / 0.02 |
| B3 + translation | 17/22 | 77.3% [56.6%, 89.9%] | 1.0 | 1.29 | 0 | 0 | 17 | 6 | 0 | -2.32 | 2.97 | 0.95 | 0.31 / 0.03 |

| Contrast | Rate difference [paired bootstrap 95%] | Discordant (first only / second only) | Exact McNemar p | Censored mean time difference, s [95%] |
|---|---|---|---|---|
| **B3−B2** | **+36.4 pp [+13.6, +59.1]** | **9 / 1** | **0.021** | **−0.27 [−0.50, −0.06]** |
| B3−B1 | +45.5 pp [+22.7, +68.2] | 11 / 1 | 0.006 | −0.39 [−0.64, −0.15] |
| B2−B1 | +9.1 pp [+0.0, +22.7] | 2 / 0 | 0.500 | −0.12 [−0.30, +0.02] |
| B1−B0 | −50.0 pp [−77.3, −22.7] | 2 / 13 | 0.007 | +0.49 [+0.20, +0.78] |
| B2−B0 | −40.9 pp [−68.2, −13.6] | 2 / 11 | 0.022 | +0.37 [+0.11, +0.64] |
| B3−B0 | −4.5 pp [−27.3, +18.2] | 3 / 4 | 1.000 | +0.10 [−0.09, +0.31] |

The p-values are exploratory and are not corrected for six contrasts. Even so, B3−B2 has a
bootstrap interval that excludes 0.

**Attribution within the scripted family.** Each event is classified by which of B1, B2 and B3
reobserve it (pattern written as B1B2B3):

| Pattern | Events | Attributed to |
| --- | --- | --- |
| `111` | 6 | Natural unocclusion; reobserved with no manoeuvre |
| `011` | 2 | Yaw |
| `001` | 9 | Translation only |
| `110` | 1 | Translation lost one: env 50, offset +1.5 m |
| `000` | 4 | None |

Of B3's 17 reobservations, 6 also occur in B1 and 2 more appear with yaw. The other 9 occur only
with translation. In the 4 primary events that every scripted arm misses, Policy F reobserves
4 times (envs 8, 56, 61, 86).

## Trade-offs: collision, approach, capture

- **Collisions.** Under scripted control, the primary events had 0 collisions in every arm. B0 had
  1 collision within 2 s (env 76, after reobservation).
- **All 107 matched events.** Collisions within 2 s: B0 5, B1 2, B2 2, B3 3. Under arm control:
  B1 1 (env 34, FOV_EXIT) and 0 for the other arms. This is no physical-safety claim.
- **Approach and capture.** Every scripted arm approaches less than Policy F: Δ distance −2.0 to
  −2.3 m against −4.25 m. They also capture less often by 5 s: B3 6/22 and B2 4/22, against
  B0 11/22. B3 travels 0.39 m more than B2 in the window: 2.97 m against 2.58 m.
- **Speed.** All arms share the riskcap governor, `MAX_VELOCITY` 2.5 and `YAW_RATE_MAX` 3.0, and
  the crash rule is unchanged. The scripted pursuit speed is 1.0 m/s; B3 adds a lateral speed of
  up to 1.5 m/s. B3's median maximum commanded speed was 1.80 m/s, against 1.00 m/s for B2.

## Separate classes (not pooled)

- **FOV_EXIT (n = 83).**
  - B0 82, B1 33, B2 82 and B3 81 reobserved.
  - Yaw is what matters here: B2−B1 is +59.0 pp, with 49 / 0 discordant pairs.
  - Translation adds nothing: B3−B2 is −1.2 pp, with 0 / 1 discordant pairs.
- **VISIBLE detector misses (n = 2).** All arms reobserve after 0.1 s.
- **RANGE_EXIT, SCALE_LOSS and PARTIAL:** 0 matched events.

## Compute (controller section, per step, all envs)

| Arm | Mean (ms) | P95 | P99 |
|---|---|---|---|
| B0 (shadow B3 evaluation only) | 0.88 | 0.94 | 0.98 |
| B1 | 1.64 | 1.72 | 1.90 |
| B2 | 1.67 | 1.74 | 1.88 |
| B3 | 1.66 | 1.73 | 1.83 |

## Key cases (reproducible scene IDs: run `HR3-P01`, seed 4502, N128, env index)

| Env | t0 | Class | Shadow offset at t0 (m) | Outcomes |
|---|---|---|---|---|
| 78 | 4 | FULL | +0.00 | Only B3 reobserves, at 1.8 s; B0 misses too |
| 84 | 4 | FULL | +0.50 | Only B3 reobserves, at 1.1 s; B0 misses too |
| 17, 19, 48, 73, 76, 80, 120 | 2–6 | FULL | 0 to ±0.5 | B3 and B0 reobserve; B1 and B2 miss |
| 50 | 6 | FULL | +1.50 | B1 and B2 reobserve at 1.3 s; B3 misses (translation lost it), and B0 misses |
| 8, 86 | 3, 2 | CENTRE_ONLY | +0.5, −0.5 | Only B0 reobserves |
| 56, 61 | 6 | FULL | +0.25, −0.5 | Only B0 reobserves |

The per-event traces are in `h_r3_analysis.json` (`events`) and the raw run.

## Verified and not verified

**Verified**, by the preregistered rules and raw records:
- The pre-branch state is identical across the four arms for 107 events.
- The arms are matched in inputs, belief, safety filter, governor and compute:
  - B1/B2/B3 run one controller with switches;
  - on the steps where both arms are in control from an identical state, B3's action equals B2's
    in 140 of 140 steps with offset 0 and in 0 of 55 steps with a nonzero offset;
  - compute is 1.64–1.67 ms for all three.
- No online GT input:
  - the scripted arms read only own pose, the tracker belief and raw LiDAR ranges
    (`depth_range_pixels`), checked by a signature and import firewall test;
  - the shared detector is appearance-based, according to the perception module's contract;
  - E-L0 is used only by the evaluator.
- The primary denominator includes every matched geometric event. 18 unmatched events are reported.
- The target is independent of the arm.
- Policy F, the RMS normaliser and existing raw records are unchanged.
- The raw manifest verifies 21 of 21 files.
- The analysis reproduces byte for byte.

**Not verified:**
- that the advantage comes from *viewpoint selection* rather than from more lateral or total
  motion. No sham-lateral or speed-matched B2 arm was run;
- generalisation beyond seed 4502, density 160, early first-occlusion events (t0 ≤ 10) and this
  scripted B3;
- the exact mechanism of cross-env coupling;
- that post-branch noise realisations are equal across arms. They are **not** equal after step 10;
- any physical safety claim;
- whether Policy F's similar rate (B3−B0 −4.5 pp) reflects the same mechanism.

## Limits

- **Legacy sensors.** Policy F and the shared tracker run on the legacy sensors, with the A1/A2
  fusion errors, the A7 camera bias and the B-carve retention issue. The S-L1D belief error is not
  removed.
- **Lineages not used.** S-L1C/D are not used, and no D-v1 class is used as a planner signal.
- **Scripted baselines are slow.** The scripted family pursues at 1.0 m/s, against up to 2.5 m/s for
  Policy F. That makes B1/B2 weak references, and B3's benefit may partly reflect this slow
  baseline.
- **Coupling.** Cross-env coupling restricts the matched population to early events.
- **Sample.** 22 primary events from one seed.

## Recommendation for a confirmatory experiment (not run)

- **Primary contrast:** B3 versus B2 on matched geometric-occlusion events, using the same
  preregistered metric.
- **Sample size:** exact McNemar, two-sided α = 0.05, 90% power, normal approximation (Connor 1987).
  - At the pilot estimate (p10 0.41, p01 0.045): **32 events**.
  - At half that effect with the same discordant total: **141 events**. This is recommended,
    because single-seed pilot effects are usually inflated.
  - At the observed yield of 22 matched primary events per 128-env seed, that is about 7 seeds.
    Use **8 new independent seeds** (not 4502 or 4503), preregistered before running.
- **Add two arms that separate viewpoint selection from extra motion, with Holm correction across
  the B3 contrasts:**
  - **B3-sham:** the same lateral magnitude, but with the offset mirrored and not chosen by line
    of sight;
  - **B2-fast:** pursuit speed matched to B3's commanded speed.
- **Keep:** the digest-matched rule, the reported coupling onset, the 30% smoke collision gate and
  the separate FOV_EXIT population.

## Files

| File | Content |
| --- | --- |
| `h_r3_analysis.json` | Committed-tool analysis, reproduced byte for byte |
| `h_r3_posthoc.json` | B2/B3 divergence source and confirmatory planning (post hoc) |
| `smoke_n16_analysis.json` | Smoke |
| `run_records/` | Run report, raw manifest, per-arm meta and completion records |
| `preservation.json` | Policy F, RMS, raw manifest and main-checkout WIP/stash state |

Raw data: `../aerial_gym_simulator/results/h_r3_pilot_v1/HR3-P01_seed4502` (21 files, 17 MB;
`RAW_MANIFEST.json` sha256 `ae2633a28859b91c6ad0499d0835872a430b8a3a97c946cfca5cff885a3ecfcf`).
