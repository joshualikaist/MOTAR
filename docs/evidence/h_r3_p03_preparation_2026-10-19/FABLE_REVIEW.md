# Fable5.1 one-shot implementation review

Completed read-only source review before GPU; CPU tests and simulator were NOT executed by Fable. The reviewed snapshot hashes and post-review closure are in review_receipt.json. GPU remains NOT_TESTED at review time. Verbatim result:

**Verdict: CONDITIONAL (GO_SOURCE_FOR_REGISTERED_QUALIFICATION once two zero-GPU items are closed).** No BLOCKER found in the reviewed snapshot. One MEDIUM spec deviation in the arms module has a one-line remedy and should be fixed before commit. Two launcher/runner contracts could not be verified from source and should be probed on CPU before the first GPU process. GPU status remains NOT_TESTED. I did not run the 51 CPU tests.

## What I verified against external contracts

I read the inherited isolation base and grepped the P02 RNG wrapper and P02 analysis constants. Everything else is from the exact text supplied.

- **Selected-only correction faithfully integrated.** The runtime reset loop is byte-for-byte the base selected branch (`tools/h_r3_env_isolation.py:142-149`): same actor-id mapping, same quaternion/position slices, same transform binding, refit of selected meshes only. Runtime adds the expected-vertex update and replaces the base lifetime check, so no full vertex host copy or SHA runs per frame. Audit mode still runs the base before/after host snapshots per reset, as registered. The base validates only before writing, so the device guard never sees stale expected vertices in audit mode. MRO for `AuditIsolation._validate_lifetime` resolves to base then device; correct.
- **600-step gates.** Every frame carries separate RGB/depth/LiDAR digests plus state, root, tracker and history digests. Reset, evaluator and audit comparisons run over all frames with only env 127 excluded in the forced pair. Numeric windows match the plan for both 16 and 600 steps and are cross-bound to the archived per-frame sensor SHAs. Non-triggered environments must match B0 over the whole horizon in every arm.
- **RNG wrapper fields exist.** `logical_step` starts at -1, `_reset_env_id`, `_pending_root_envs`, `_flush_selected_roots`, `probe`, `metadata`, `step`, `action` all exist. The wrapper replaces `write_to_sim` at construction, so the launcher's wrapper correctly sits on top of it and the queued/full/indexed_flush classification mirrors the wrapper's own branch.
- **B4 sham and frozen magnitude.** Sign is a pure SHA of seed, env, episode, event and decision index. Magnitude is taken from the pre-loss proposal computed before `env_step`, archived with a digest at trigger, and only validated, never re-estimated, online. The `nextafter` loop can only reduce the reference. Prefix constancy and norm consistency are enforced on every call.
- **B5 and balance diagnostics.** Realized-effort imbalance or missing exposure yields INCONCLUSIVE; even a pass is NOT_ESTABLISHED. Mechanical energy is labelled NOT_RECORDED. Safety stages from frozen P02 are unchanged and attenuation is only recorded.
- **Firewall and frozen F.** The control input tuple is own pose, vehicle quaternion, tracker belief and LiDAR only, AST-tested. GT silhouette, identity, target position and geometry snapshots are post-step evaluation only. Checkpoint SHA, RMS start/end, training flag, 898 obs and vision mode are all gated.
- **Outcome rules.** Terminal-first via P02 `terminal` (reset, crashed, captured, timeout). Class denominators and FOV_EXIT separation follow P02 constants. No historical raw is loaded, and any prefix mismatch is a global failure.

## Ranked findings

1. **MEDIUM, fix before commit. Direction-invalid abstention is not enforced in code.** In `_candidates`, direction is `rel / clamp(dist, 1e-6)`. For 0 < dist < 1e-6 the direction is nonzero while `direction_valid` is False, so B3/B4/B5 nominal commands are nonzero. The launcher then raises its hard "direction-abstention violation" error and kills the process with no retry, and the analysis would also fail. The test only covers exactly coincident positions. Smallest remedy: zero the direction (and therefore normal) where `direction_valid` is False inside `_candidates`. Probability is very low, but the consequence is a wasted registered run.

2. **MEDIUM, zero-GPU probe. Runner binding keys from base preflight are unverified.** `validate` reads `checks['head']`, `checks['policy_f_sha256']` and `checks['rms_digest']`. The runner test fabricates these. If `base.preflight` does not emit `rms_digest`, the first process completes and then raises KeyError, burning the fixed run_id since the output root is created with `exist_ok=False`. Run the `preflight` stage and confirm those three keys appear in the printed checks before `smoke`.

3. **MEDIUM, zero-GPU probe. `process_env` compatibility with the P03 spec.** `specs()` injects seed, num_envs, config, arm, batch and lineage into a P02-shaped env builder with a P02 plan. Call `b.process_env(envp, specs(plan)[0], tmp)` on CPU and confirm it returns NAVRL_SEED 4801, 160 bars and lineage off. Also confirm `rng.episodes` becomes non-negative for all environments after the initial reset, since `sham_signs` rejects the initial -1 and would kill step 0.

4. **LOW. Nominal budget tolerance is a hard kill.** The 1e-6 m/s check is preregistered and the float32 worst-case error at maximum budget is roughly 9e-7, so it should hold, but a breach stops the process rather than being logged. Tests pass at 3e-7. No change recommended; noted so a failure here is read as numeric, not scientific.

5. **LOW, design observation.** B4 re-hashes per decision index, so its sign can flip every step. That is registered, but it makes B3-B4 realized balance likely IMBALANCED, which the analysis will correctly report as INCONCLUSIVE.

6. **LOW, registered risk.** A natural reset at step 2 in the baseline, or in the forced run, stops the qualification by design. Seed 4801 has not been observed, so this is a possible spurious stop, not a bug.

## Not verified (limitations)

- Whether the initial reset produces an indexed root flush covering all 128 environments rather than only a full write. The chronology gate and the test fixture assume it does. The first 16-step process will reveal it at low cost.
- Line 27 of the P02 `CORE` tuple was truncated in my grep; the launcher appears to record every P02 key but I could not confirm the tail.
- The hard-coded LiDAR elevations, `_lidar_distance_m` layout, `check_origins` keys, `step_dt == 0.1` exactness and the `identity.latest` key set are inherited P02 contracts I did not re-read.
- No audit of production sim code, no simulator, no GPU. Peak memory, audit-mode wall time and cross-environment physics independence over 600 steps are exactly what the qualification is designed to establish and are NOT_TESTED.

One qualification cluster gives no efficacy confirmation, and the 10% realized margin is descriptive only.
