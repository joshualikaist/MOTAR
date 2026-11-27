# S-L1C v2 final disposition

**GO — V2-A, limited to the preregistered precision-limited simulator scope.**
This supersedes the pre-review status in the original result section. It is not a qualification
of real sensor realism, overall tracking reliability, Recovery, or D-on-C-v2 validity.

## Evidence and decision

- Base `aed6affbd2567feee79e658fbe5f07510b039f36`; GPU source
  `e45794e876b15f71b5f38c65702fefadc368f93a`; result commit
  `d063e7d3c6b2eed0243d1375f53b986ed0ff1d3b`.
- [CPU selection](CPU_SELECTION.md): V2-A alone passes all candidate gates. The exact declared
  sets/thresholds were committed before measurement. No post-result tolerance relaxation.
- [GPU analysis](s_l1c_v2_analysis.json): all nine gates PASS. Near-range genuine recall
  140/288 → 264/288, no paired true loss, no new false association; original EQ-1D negatives,
  new bars and foreground negatives remain zero. Covariance, identity/expiry, old-mode
  regression, E-L0, GT firewall and integrity PASS.
- [Independent Fable 5.1 review](FABLE_REVIEW.md): **SUBSTANTIATED_GO**, no blocker. The CLI
  metadata verifies canonical model `claude-fable-5-1`; no model substitution. The initial old-CLI
  API400 was resolved by the tool's updater before retry; it was not a completed review.
- Fable reports targeted 22/22 and full PUBLIC_CPU 2,593 tests, 0 failures/errors, 10 skipped,
  5 excluded, plus its own sphere-intersection truth, ledger recount, raw hashes, covariance,
  clock and firewall recomputation. Its analysis output is byte-identical to the committed one.
- [Final preservation check](final_preservation.json) records historical source/raw, both Policy F
  copies, main WIP/stash and new CPU/GPU/review manifests. No production change since GPU source.

## Review disposition and disclosure

1. **Medium disclosure gap — RESOLVED by append-only result addendum.** The predeclared ±1°
   angular-error diagnostic is now explicit: v1 312/864 (36.11%), v2 504/864 (58.33%), 648/1,152
   v2 abstentions, zero false associations and newly accepted maximum centre error 5.159mm.
   This diagnostic is not qualified, and the GPU 91.67% headline assumes exact angular prediction.
2. **Low negative-coverage limit — OPEN LIMIT, not a failed preregistered gate.** Vertical box
   faces/EQ-1D tested; non-vertical surfaces, top edges, ground and different objects in adjacent
   rows remain NOT_TESTED. GPU prior-1.0 probes test a wider adaptive gate but do not qualify
   every scene/noise distribution. Preserve inherited CPU 135 far-bar false acceptances.
3. **Low closed-loop interpretation limit — DISCLOSED.** Post-review E-L0 recount labels all
   saved closed-loop LiDAR associations correct (static 419/419 each; moving v1 362/362,
   v2 431/431); this is evaluator semantic provenance, not a new independent per-frame ray study.
   Moving trajectories diverge and completed outcomes are v1 capture96/crash4 versus v2
   capture89/crash5 in a fixed 600-step run. No performance gain is claimed. This post-review
   descriptive recount is separate from the review and unchanged selection gates.
4. **Informational:** the bar/foreground row-tie convention and inherited accepted-return
   maximum range bias 0.168693m remain as recorded. Camera centre-range bias is not corrected.
5. **Telemetry clarification:** the reference-derived reason split is labelled reconstructed;
   actual production reason codes are NOT_RECORDED. Saved validity decisions are independently
   matched. No missing online telemetry is silently replaced with a proxy.

## Operational contract and next boundary

`NAVRL_SENSOR_LINEAGE=s_l1c_v2` opts into cumulative A+B+C-v2. Default OFF and A/B/C/D-v1 remain
unchanged. D is disabled on C-v2; only API/non-mutation compatibility is checked. Keep Frozen
Policy F, RGB-D + LiDAR, actor898, camera-only identity reset and fused 5s expiry unchanged.

The 1.16m inward-biased predictions still abstain; fixed production noise bound is 0.1mm and is
not a calibrated hardware model. ±1cm/±3cm offline experiments with declared higher bounds do
not imply fixed-parameter hardware robustness. Existing v1 range bias and false associations
are preserved, not solved.

Only [future validation designs](../../plans/post_s_l1c_v2_validation_design.md) are prepared for
D empirical belief-error stress, B target-return carve retention and camera centre-range bias.
No B/D production change, P-L1, G-L1, PPO, Planner, H-R confirmation or extra GPU is included.

The authorized ordinary push is restricted to `codex/mt-p1` after exact-HEAD unpublished-history,
security, WIP/raw/checkpoint/symlink/local-path and remote-target checks. Main merge/push,
force-push and history rewrite remain prohibited. Git merge/symlinks do not publish raw data.
