# G4-R CPU validation contract — 2026-10-20

Authority: the user's G4-R instruction, baseline cdfaffd28534cd37693a6cd678de9766bef4bd25.
This is an additive, opt-in nominal-model implementation and design package. No local Git commit,
push, merge, production edit, simulator/GPU run, training or legacy test edit is authorized.

## Evidence and interpretation

- Original A01/B01/B02/C01/D01 corrections and their historical receipts are preserved.
- B03 sparse coverage and B04 target/foreground identity remain BLOCKED.
- C02 may receive a nominal planar-model result only. No result establishes a physical3-D
  free volume, calibrated error/latency bounds, real rotor envelope or actual Lee response.
- MODEL_SAFE is limited to the explicit model. Unknown-space penetration returns UNKNOWN;
  resolved measured-point penetration returns MODEL_UNSAFE; numerical contact ambiguity returns
  UNKNOWN; malformed/nonfinite inputs return INVALID.
- PHYSICAL_QUALIFICATION_NOT_AVAILABLE is invariant, even when all supplied scalar bounds are zero.
- Missing bounds are None/provenance, not measured zeros. Nonzero unsupported latency or scan-frame
  attitude error must abstain rather than silently disappear.

## Required comparisons

1. Preserve every legacy BOIA function/class definition and all10 tracked BOIA test files.
   No online planner/arbitration integration: new APIs must have no caller in the existing loop.
2. Independent arc/step analytic oracles for whole capsule containment; no radius-only shrink or
   corner/endpoints-only proof. Uncertain numerical tangency must abstain, not loosen the gate.
3. Include centre/body unknown penetration, radial steps, nonconvex free sets, uniform open fields,
   low/high-speed .55m gap, zero speed, unknown-only fields and invalid-row isolation.
4. Reproduce the declared(2,0) to(0,2) response with obstacle(.97,.2),yaw−43.34970056deg:
   legacy full-horizon hard/stop PASS, new first-cycle fallback NOT MODEL_SAFE. Compare the response
   with independent scalar equations, not the same implementation helper.
5. Check the complete first .1s swept segments and reaction/zero-input braking, including an
   analytical infinite-tail remainder. A finite iteration cap or speed tolerance is not stopping.
6. Preserve exact F candidate/action on the legacy path, GT firewall, no input mutation, local
   reproducibility and global RNG/default dtype/TF32 flags.
7. New math/fixture tests passing does not qualify measured false-alarm, actual calibration,
   capture/reacquisition, GPU deadline/VRAM or real flight.

## Execution/resource discipline

- Pinned aerialgym Python with PYTHONNOUSERSITE=1, CUDA_VISIBLE_DEVICES='', python-B.
- Focused new/BOIA tests first. One PUBLIC_CPU pass may follow after source/test freeze; historical
  runtime155s (~3 minutes),2899tests,4pre-existing DetectReduction errors,0assertion failures.
  Record whole-suite FAIL if those errors remain; no waiver or unrelated source fix.
- Before broad testing check concurrent evaluation processes and free disk. Disk stop3GB;
  baseline free~9.66GB. No original data deletion to satisfy this floor.
- Small standalone timing separates geometry/next-cycle verifier from the legacy planner.
  Do not infer GPU speed or a worst-case bound from one cold CPU observation.

## Review boundaries

New algorithm, hypothesis-set calibration, evidence packet, robust intervention and F0–F5 redesign
are DESIGN ONLY. They do not authorize editing access/planner/arbitration, reading online GT,
executing unsafe ablations or changing frozen07–09/preregistration/Raw. Current source and tests
remain uncommitted for user review by explicit instruction; WORKLOG will record this exception.

## Post-review extension, before final source freeze

The initial188-test BOIA pass and2927-test PUBLIC_CPU run preceded the focused timing-contract
review. That review found no input check tying the first Rollout slice to the assumed.1s grid.
The final opt-in path additionally checks semi-implicit `delta_position=delta*velocity_end`,
with only dtype/coordinate-scale arithmetic tolerance. A mismatched-prefix negative and healthy
same-batch control are added. This validates consistency with an assumed grid, not measured
timestamps or plant dynamics; stationary data cannot identify elapsed time. Preserve the earlier
transcripts/source SHA as pre-guard evidence and run the complete BOIA suite on the final source.
The unrelated public-suite errors are not fixed or waived, and the earlier broad run is not
retroactively attributed to the final guard implementation. No existing test is changed.
