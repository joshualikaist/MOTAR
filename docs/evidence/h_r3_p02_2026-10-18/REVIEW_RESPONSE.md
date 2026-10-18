# Response and source qualification of the focused Fable review

The independent NO-GO verdict is accepted. H-R3 stays PROVISIONAL; no confirmation or extra GPU,
seed, production repair or controller tuning follows. The original review is retained verbatim.
The first broad review timed out900s with no verdict; the focused same-model follow-up completed.
Its scope is source/arithmetic review, not an independent execution of96CPU tests or raw NPZ hashes.
Root's tests/preservation and the separate read-only raw localization provide those other records.

## Accepted blockers and limitations

- Actual-N128 all-arm prefix failure blocks causal inference. Even42 primary candidates with matched
  prefixes are not salvaged: postbranch sensor/geometry isolation is not established for them.
  This is an unresolved contamination risk, not proof that every one of42 was numerically altered.
- WarpEnv.reset_idx really writes the global vertex buffer and refits selected meshes. Its role in
  observed failures remains a source-consistent hypothesis, NOT_PROVEN. Modality/vertex snapshots
  are missing. The N16 force40 diagnostic shared natural resets and could miss this mechanism.
- Synthetic RNG probes alone cannot demonstrate equality of active sensor-noise consumption.
  They are counter/seed consistency evidence. This profile has measurement perturbations disabled;
  no robustness claim about stochastic real sensors is made. Historical analysis key names remain.
- Nominal budget parity does not establish realized effort parity. Per the original registration,
  >10% differences prohibit a pure-viewpoint interpretation; this is not a newly invented hard
  stop rule for the exploratory pilot. The already-preregistered prefix FAIL supplies NO-GO.
- Two clusters and zero PARTIAL events do not establish population effects or cross-class claims.
  The planning grid is HOLD, not an executable32-seed recommendation.

## Reviewer statements requiring correction/qualification

1. Item7's cited navrl_task.py4845/4849/4936 calls are inside reset_idx (begins4701), not outside
   reset. FixedNRandomness._reset_task invokes this method inside _preserve_other_kinematics,
   with _reset_env_id set; _write_selected_roots queues them. DOF/physical-target profiles are
   rejected. These cited lines are not an independently demonstrated remaining full-root bypass.
   Actual writes outside the transaction can still be audited as part of future qualification.
2. Item6 does not imply an exact50% episode-level effect dilution or automatic4x sample multiplier.
   Conditional on a fixed same-state nonzero informed side, a symmetric sham sign agrees half
   the time in expectation. B4 sign is fixed perenv while informed side/magnitude and subsequent
   trajectories can vary, so equality for a full trajectory or a guaranteed factor2 contrast is
   not established. Future power simulation must model this mixture and arm dependence explicitly.
3. The target_mask residual is resolved narrowly by source contract and existing receipts:
   navrl_detector.py:_render_target_camera_kernel (roughly77–132) starts mask0, intersects target,
   queries mesh up to its first-hit distance, and sets mask1 only if unblocked. D8 is OFF in all12
   existing bulk-eval condition records (review_source_checks.json). This is visible analytic support,
   not an unoccluded whole silhouette. It does not prove geometric numerical fidelity under the
   unresolved shared-mesh bug or realistic appearance/identity persistence.
4. Integer pixel centers are explicit in detector construction: torch.arange rows/cols,
   cx=(W−1)/2,cy=(H−1)/2 and corresponding rays. P02 scorer uses the same integer convention.
   No production change or extra GPU test was made to establish this source consistency.

## Next unit — plan only

See NEXT_VALIDATION_PLAN.md: independent CPU scene/vertex/ray oracle, then a preregistered actual-N128
unrelated-reset-vs-no-reset mechanics canary with separate sensor modalities and numerical snapshots.
All measured source/results remain preserved. New GPU/exploration/confirmation waits for user review.
