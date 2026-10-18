# Independent Fable 5.1 focused read-only review

Reviewed result commit: `b98b400`. First broad attempt timed out without a verdict; this is the completed focused follow-up. No CPU/GPU test or raw NPZ verification was performed by this focused reviewer. Source-checked facts and report-only claims are distinguished in the verbatim text. See REVIEW_RESPONSE.md for corrections/qualifications.

**Verdict: NO-GO for confirmatory readiness is justified and, on the supplied sources, over-determined.** H-R3 stays PROVISIONAL. The NO-GO is not a rejection of the H-R3 science; it reflects invalid matched data plus design limits that would block confirmation even with clean data.

## What I source-checked (facts) vs. what is report-only

**Independently verified from the supplied code:**

- **Prefix logic is correct.** The launcher records state and sensor digests after each step; the trigger sets t0 to the next step. The analysis compares digests over `[:t0]`, so the last compared row is the pre-branch state. Terminal-first handling excludes the terminal index for the spatial primary and includes it only for the literal P01 variant, matching the plan.
- **Exposure bookkeeping matches the plan.** Active steps run t0 through t0+19, ending early only on reset, crash or capture. The analysis reconstructs exactly this and flags deviations.
- **B4 and B5 nominal budgets are exactly equal to B3 by construction.** Pursuit is along u and lateral along the perpendicular, so B3 and B4 have norm sqrt(p²+l²), which is B5's scalar. The launcher aborts on a 1e-6 mismatch and the analysis re-derives it from shadow stages.
- **GT firewall holds within the supplied code.** The controller interface in `evaluate_actions` takes only pose, quaternion, tracker belief and LiDAR. The recorder wraps `_detect_rgbd`, reads GT only for scoring, and returns the detector output untouched. The 3x3 pixel-center enumeration correctly covers continuous Chebyshev distance 1.
- **Report arithmetic is internally consistent.** Per-seed pp differences, equal-seed means, arm totals (34/37/43/32 of 56), class sums (31+25=56) and the 246 total triggers all reconcile.
- **Warp global rewrite is real.** `warp_env_manager.py:44-48` rewrites all envs' vertices from the current root tensor while `env_ids` only selects which meshes refit. `env_manager.py:328` calls it from every reset, including the RNG wrapper's per-env serialized resets.

**Report-only, not verifiable here:** raw digests, the step 7/env 2 and step 11/env 1 localizations, the smoke reset chronology, preservation counts, the 96 CPU tests (passed per `cpu_validation.json`; I did not run them), and checkpoint hash bindings.

## Ranked findings

1. **The matching failure is a design-level cross-env coupling, not a seed-specific glitch.** `tools/h_r3_p02_rng.py` isolates RNG and restores other envs' derived kinematics, but nothing preserves other envs' Warp vertex slices. Any arm-dependent reset schedule difference, which scripted control produces whenever it avoids or causes a crash, triggers the global vertex rewrite in one arm and not another. The audit's chronology fits this exactly: in seed 4602 B0 alone resets env 56 at step 7 and all five other arms diverge at step 7; in seed 4601 B0 and B3 share a reset at step 11 and B3 alone stays matched. This is consistent but NOT_PROVEN, as the audit says. Static poses and vertices were not snapshotted.

2. **The 42 matched events are also contaminated inside the window.** The same mechanism acts after t0. Other envs' resets alter a matched env's ray casts differently across arms during the 20-step window, which the `target_pos`/`rng_probe` noninterference check cannot see. This independently supports the instruction not to salvage them.

3. **The RNG probe is a weak noninterference witness.** `probe()` draws from a scope keyed only on seed, step, env and episode counter. Equal probes prove equal episode counters, not equal live RNG consumption. The metadata labels it synthetic, but the analysis key name "target/RNG noninterference" overstates it.

4. **SMOKE02 could not have detected the coupling.** The forced reset at step 40 coincided with natural resets of envs 5 and 6 in the baseline, so both processes ran the global rewrite at the same step. The gate passed vacuously for this failure mode. Report-only, but the launcher's injection design makes this blind spot plausible: it joins the ordinary reset transaction rather than forcing an isolated reset.

5. **B5 did not achieve effort equivalence.** Control-only command squared-speed differences of 19 to 70 percent and translation differences of 9 to 15 percent exceed the preregistered 10 percent diagnostic. Per plan, movement and speed effects cannot be ruled out. This is a second NO-GO condition independent of matching.

6. **B4's sham design halves the B3−B4 contrast by construction.** The SHA sign matches the informed side in roughly half of envs, where B4 equals B3 exactly. The planning grid in `tools/h_r3_p02_planning.py` does not model this dilution. A 15 pp B3−B4 margin implies roughly 30 pp between informed and mirrored sides. Also, the nominal B4 > B3 result in both seeds is uninterpretable on invalid data and must not be read either way.

7. **Three task-level full root writes exist outside the reset path** at `navrl_task.py:4845`, `4849` and `4936`. They pass through `_write_selected_roots` as full writes when no reset transaction is open. If any is conditional on per-env state, it is a second coupling candidate. Not excluded by the audit, not checked here.

8. **Zero-denominator and two-cluster handling is correct.** Empty classes yield None rather than crashes; PARTIAL, NONE, RANGE_EXIT and SCALE_LOSS are reported as zero. Sign-flip minimum p of 0.5 and the per-seed range are honestly labeled. The planning normal approximation reproduces: 15 pp, discordance 0.4 gives about 184 paired events, and with 20 per seed and ICC 0.1 about 27 seeds, so 32 proposed is consistent.

## Residual unknowns

- Whether `detector.target_mask` is occlusion-aware visible support or a projected full-target mask. Set at `navrl_detector.py:422` and `navrl_dynamic_mesh_treatment.py:245`; I did not read these. If projected, the "visible target pixel" rule is weaker than preregistered.
- Pixel-center convention (integer vs. half-integer) for the intrinsics used in the centroid rule.
- Whether the moving target mesh is also refreshed through the global vertex path or only via `_sync_target_to_sensor`.
- First changed sensor modality at onset, since RGB, depth and LiDAR share one digest.

## Minimum next validation steps (no new sensor, Planner or PPO work)

1. **CPU geometry oracle test**, as the audit plans: fake refit counters, perturb an unselected env's static pose by one float ulp or a quaternion renormalization, reset another env, require unselected vertex bytes unchanged.
2. **Opt-in evaluation-only wrapper** around `warp_env.reset_idx` that saves and restores unselected envs' vertex slices, mirroring `_preserve_other_kinematics`. This is a P02-style module, not a production edit.
3. **Preregistered N128 B0 forced-reset pair** at a step with no natural reset in the reference, snapshotting per-modality raw sensors, static roots, vertices and derived kinematics. Require byte-identical futures for untouched envs. Separate the sensor digest into three modalities.
4. **Audit the three task-level `write_to_sim` call sites** for per-env conditionality.
5. **Confirm the target_mask semantics** before relying on the spatial identity rule in any future preregistration.
6. **Revise the planning grid** to model B4's structural 50 percent dilution and the realized-effort imbalance before fixing any sample size.

Only after steps 1 to 4 pass at N128 should a fresh exploratory preregistration be considered. Nothing here promotes H-R3 or rejects it; the data cannot speak to the hypothesis.
