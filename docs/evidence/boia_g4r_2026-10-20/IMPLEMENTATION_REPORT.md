# BOIA G4-R: nominal C02 correction and mission-arbitration design

Date: 2026-10-20. **CPU MODEL ONLY; G5 HOLD.** This is a new supplement; prior
preregistrations, source receipts and Raw are preserved. The current instruction
explicitly says **“로컬 Git commit도 수행하지 말고”**. This overrides the standing
local-commit rule for this package. No staging, commit, push or merge is authorized.

## A. Current state and evidence

Branch `codex/mt-p1`; base/current HEAD `cdfaffd28534cd37693a6cd678de9766bef4bd25`.
The source is an **uncommitted patch**, not a new source commit. Exact file hashes,
test results, commands and preserved-file checks are in [validation.json](validation.json),
[integrity.json](integrity.json) and [EVIDENCE_MANIFEST.json](EVIDENCE_MANIFEST.json).
See [HANDOFF](HANDOFF.md) for every outstanding path and session continuation.
The exact three-file source patch is [C02_SOURCE_DIFF.patch](C02_SOURCE_DIFF.patch);
the new tests and documents are separately listed additions, not hidden in that patch.

Only three BOIA source files change: `types.py`, `geometry.py`, `safety.py`.
Two new test modules and this evidence/design package are added. WORKLOG is
append-only. All existing BOIA functions/classes and all ten previous BOIA test
files are preserved. The other twelve BOIA modules, production, controller,
governor, policy, tracker, simulator and checkpoint are unchanged.

The whole BOIA module set was source-reviewed, including the actual planner,
arbitration, fallback and terminal call paths. Reviews of the large files were
delegated as required by AGENTS.md. This package does not replace the earlier
[file/function audit](../boia_g4_p0_2026-10-20/AUDIT_REPORT.md). The new API has no
caller in the existing planner or arbitration. Static source preservation plus
the unchanged legacy tests support default CPU noninterference. No new physical
actor/action/trajectory replay was run; it must not be inferred from a checksum.

Executed commands are recorded in validation.json. They include focused BOIA
unittest discovery, PUBLIC_CPU, independent scalar/oracle receipts, an adversarial
finite witness check, public-document checks, source AST/hash comparison and
`git diff --check`. No GPU/Isaac/learning command was launched. Full Raw payload
rehash is NOT_RUN; external Raw backup is still absent.

| Check | Source scope | Result |
| --- | --- | --- |
| Complete BOIA | Final C02 source, including prefix-time consistency guard | **189/189 PASS**,13.384 s;160 old plus29 new tests. |
| PUBLIC_CPU | Source before the final timing guard; exact hashes retained | 2927 run,0 assertion failures,4 errors,10 skipped,5 excluded;156.488 s. **FAIL**. |
| Earlier BOIA | Same pre-guard source as broad run | 188/188 PASS,13.521 s. |
| Independent finite witnesses | Frozen capsule geometry | 96 queries,44 MODEL_SAFE,92,928 witness points,0 false-safe witnesses. Not a proof by sampling. |

All four broad errors have the same test IDs as the prior base receipt:
`test_navrl_detect_resolution.DetectReduction`. No related source/test is changed
or the failures waived. The broad test is not attributed to the final timing
guard source; final verification of that opt-in change uses all189 BOIA tests.

## B. C02 implementation

| File / new API | Existing problem | Additive change | Independent evidence | Status |
| --- | --- | --- | --- | --- |
| `types.py`: `FreeBoundary2D`, `SweepBounds`, result/status records | Boolean tests hid unknown coverage and missing physical assumptions. | Explicit MODEL_SAFE/MODEL_UNSAFE/UNKNOWN/INVALID; five missing error declarations remain None. Physical qualification is always unavailable. | Missing-bound, invalid shape/device, unsupported latency/attitude and batch tests. | PASS, explicit nominal contract only. |
| `geometry.py`: `capsule_in_free_model` and three analytic helpers | Centre segments or a finite number of body corners can be free while part of the body is unknown. | Test an entire disc-swept segment against outer circular arcs and genuine radial steps in the declared planar interpolation. Uniform internal sector rays are not obstacles. | Trigonometric wedge/step, nonconvex front-face, disk convexity, analytic arc/segment and transformation oracles. | PASS in declared planar model. |
| `safety.py`: `screen_next_cycle` | Full-horizon hard/terminal-stop checks do not imply backup from the executed first 0.1 s. | Retain old checks, then check every first-interval capsule and complete prefix-end backup. No default hook. | Exact user counterexample and scalar semi-implicit recurrence. | PASS_CPU_MODEL; not physical qualification. |
| `safety.py`: `_c02_backup` | A finite sample cap could omit remaining braking motion. | Reaction coast once, 64 resolved zero-input samples, then the complete continuous tail bound; max with the empirical total envelope. | Geometric series and acceleration-saturated analytic distance. Nonzero speed after sample64 is retained. | PASS_CPU_MODEL, plant validity LIMITED. |
| `safety.py`: `_c02_point_check`, local verification precision | float32 can round the body floor down or turn near contact into a nominal certificate. | New verification locally float64; one-sided ambiguity band; outward-rounded backup endpoint. Legacy comparison unchanged. | Represented float32 floor-rounddown counterexample and numerical-contact tests. | PASS_CPU_MODEL; numerical band is not a sensor margin. |

The effective radius is at least `sqrt(.14²+.14²+.04²)=.20199009876724155 m`,
enclosing the declared .28×.28×.08 m collision box at any roll/pitch/yaw. This is
not the actual rotor envelope. Prefix inflation retains `max(endpoint speeds)*t_det`
with `t_det=.09 s`; backup includes `t_react=.1 s` once. These are separate existing
model allowances, not measured latency bounds. Declared position/tracking/sensor
allowances add conservatively. Missing values are reported and never confer
physical qualification. Nonzero latency/scan-attitude bounds cannot currently be
propagated and yield UNKNOWN. Vertical motion also yields UNKNOWN.

The boundary is the explicitly named PLANAR_SECTOR_INTERPOLATION_V1. It is **not**
a new claim that sparse LiDAR certifies free volume. A scan timestamp is provenance
only; this API does not certify ageing. A supplied Rollout is a nominal model
object, not measured flight. The next interval uses the declared .1 s grid and
checks semi-implicit position/velocity consistency; actual clock calibration and
command-to-plant consistency are not established by that check.

### Mandatory next-cycle counterexample

Use `v0=(2,0)`, world command `(0,2)`, tau=.4 s, substep=.01 s,
point `(.97,.2)` and yaw `atan2(.2,.97)-55°`. The raw-derived boundary uses four
rows and bin25 in row2. The old full-horizon hard/terminal-stop results are both
True. New prefix is True, backup is False, result **MODEL_UNSAFE**. The independent
zero-input asymptotic limit without reaction is `(.78,.2)`; in that yawed box the
obstacle offset has both body coordinates below .14 m. This is a real mathematical
collision counterexample, not just rejection due to an enlarged empirical tail.
See [model_receipt.json](model_receipt.json) for exact values and source bindings.
It is not an actual Frozen F command or a simulator incident.

### Required case ledger

Every row is a CPU model test, not a flight claim. Exact unittest IDs/results are
in [validation.json](validation.json) and the unchanged captured log.

| Requirement | Mathematical input / independent oracle | Expected and actual | Limitation |
| --- | --- | --- | --- |
| 1. Center-only false-safe | Centre `(1,0)` near a .5 m notch; distance to 2.5° sector face is `sin(2.5°)<r`. | Legacy passes; whole-body UNKNOWN. PASS. | Assumed planar set. |
| 2. Sector penetration | Same wedge and genuine radial-step cases; analytic point-line/segment distance. | Penetrating/tangent capsule cannot be MODEL_SAFE; uniform disk internal rays remain free. PASS. | Finite-precision ambiguity abstains. |
| 3. Nonconvex penetration | Centre at(.6,0), .65 m notch; all four box corners outside narrow angle while front-face centre(.74,0) crosses it. | UNKNOWN despite free centre/corners. PASS. | Does not reconstruct 3-D surfaces. |
| 4. .55 m gap | Synthetic parallel sensed points y=±.275; `r+.09v`=.2559901 at.6 m/s and .3819901 at2 m/s. | Low MODEL_SAFE; high MODEL_UNSAFE. Adding .03 m position allowance rejects low. PASS. | Point-wall fixture plus assumed surrounding free set, not a real traversable-gap certificate. |
| 5. Horizon/next cycle | User's exact orthogonal velocity-command fixture above; scalar geometric-series U4 oracle. | Old True/True, prefix True, backup False, MODEL_UNSAFE. PASS. | Surrogate dynamics only. |
| 6. Curved response | Rotating world command; complex geometric-series position/velocity formula. | Backup starts with actual curved-prefix endpoint velocity and covers the zero-input continuation. PASS. | Zero XY command is direction-preserving only in this surrogate. |
| 7. Zero speed | Unit disk and centres x=.79/.81; compare `x+r` to1. | Safe at.79, UNKNOWN at.81; zero backup distance. PASS. | A zero-speed model state is different from issuing zero command while moving. |
| 8. Unknown only | Every radial extent zero, nonzero body radius. | UNKNOWN, never MODEL_SAFE. PASS. | No physical free-space inference. |
| 9. Missing bounds | Default five None fields; separately supply unsupported latency/attitude. | Missing list retained; physical unavailable; nonzero unsupported fields UNKNOWN. PASS. | No unmeasured bound replaced by a certified zero. |
| 10. Batch independence | Healthy and NaN/overflow/invalid/mismatched-prefix rows in same batch; compare independent row result. | Invalid rows isolated; healthy results unchanged. PASS. | Does not requalify simulator reset/RNG isolation. |
| 11. Policy F preservation | Old function source hashes and existing caller/tests; old/new API calls compared before/after. | Old functions/tests and checkpoint hash preserved. PASS for CPU/static scope. | Full actor/trajectory GPU replay NOT_TESTED. |
| 12. Repeatability | Identical calls in float32/64, Python/Torch RNG and TF32 state snapshots. | Identical deterministic outputs, no global RNG/settings mutation. PASS. | Not a statistical or physical guarantee. |

Extra regressions cover the infinite tail after sample64, floating floor rounding,
malformed shape/device, unsupported vertical motion and prefix grid mismatch.
The supplemental finite falsification tried96 capsules;44 accepted capsules had
92,928 independently sampled boundary witnesses with no false-safe witness. This
is supporting negative-search evidence, **not** the continuum containment proof.

### Computational observation

Standalone verifier, one CPU thread, float64, H10/S10,72 boundary bins,64 backup
samples, **zero sensed obstacle points**; inputs prepared before timing:

| N | K | Observed seconds, one call | Interpretation |
| --- | --- | --- | --- |
| 1 | 1 | .007870 | Includes the retained legacy screen and new verifier. |
| 16 | 20 | .461017 | All320 nominal candidates accepted in an open synthetic field. |
| 128 | 20 | 3.863113 | All2560 nominal candidates accepted; far beyond a.1 s decision period. |

These are one-shot observations, not P95/P99, WCET or a full planner/trace timing.
They do not include new arbitration and do not represent difficult dense-point
scenes. No GPU, VRAM or real-time claim follows. Even this limited CPU workload
does not meet the historical CPU5 ms wording; performance/integration is unresolved.

### Stopping models remain distinct

| Speed m/s | U4 zero-input limit, no reaction m | Empirical base m | U4 signed error vs empirical | Status |
| --- | --- | --- | --- | --- |
| 2.00 | .7800 | .82 | -4.878% | Surrogate only. |
| 2.50 | .9750 | 1.05 | -7.143% | Surrogate only. |
| 3.52 | 1.3728 | 1.57 | -12.561% | LIMITED_VALIDITY; user's≤25% continuation permission, not≤10% target. |

U4 is semi-implicit with q=.975 and tail `.01*q/(1-q)*v=.39v`.
The C02 envelope takes `max(empirical_with_reaction, reaction+tau*v+
relu(v-a*tau)^2/(2a))`; reaction is not added again after that maximum. Numerical
outward rounding is labelled separately. Riskcap telemetry/governor and actual
Lee/UAV dynamics are different models and unchanged. Actual stop/latency/plant
bounds are NOT_TESTED here. Riskcap's2 m/s value is a cap floor, not an executed
minimum speed. NO_CERTIFIED_BACKUP does not prove inevitable collision.

## C. Remaining P0

| ID | Status | Remaining risk | Resolution principle / next gate |
| --- | --- | --- | --- |
| B03 | BLOCKED | Finite rays do not certify continuous free volume between beams, vertically or between scans. | Approve an explicit measured beam/feature/pose/age support contract or retain UNKNOWN. Independent geometric indistinguishability tests first. |
| B04 | BLOCKED | Target surface vs foreground or false identity may be observationally indistinguishable. Keeping all hits conservatively self-blocks forecasts. | Measured camera-depth/association/timestamp packet; retain competing hypotheses. Never delete a hit using GT or belief alone. |
| C02 | PASS_CPU_MODEL / PHYSICAL_QUALIFICATION_NOT_AVAILABLE | New opt-in nominal containment and next-cycle checks pass; absent3-D free volume, plant/control/latency bounds, rotor geometry and execution integration. Legacy path intentionally retains its documented limitations. | Review this diff, then separately qualify observation/plant bounds and executor contract; no automatic G5 entry. |

The centre6 m/radius.15 m ideal on-axis target first hit is5.85 m. The mandatory
5.8 m return remains an ambiguous/foreground/noise/calibration fixture; it is not
silently relabelled as the ideal target. Both must preserve collision evidence.

## D. New BOIA algorithm proposal: design only

The complete implementable mathematical specification is [ALGORITHM_PROPOSAL](../../research/occlusion_algorithm_v1/g4r_2026-10-20/ALGORITHM_PROPOSAL.md).
This crosswalk identifies the twelve requested elements without representing them
as tested/implemented:

| Requested element | Proposed specification |
| --- | --- |
| 1. Name/core | Evidence-aware mission arbitration with supported intervention-opportunity windows. |
| 2. Change | Scene/identity hypothesis set, separate safety and utility evidence gates, calibrated paired benefit. |
| 3. Problem | Improve eventual collision-free capture over continued Frozen F; visibility is intermediate. |
| 4. Objective | Bounded finite-window mission loss; paired improvement under the same scene/path, then held-out continuation-error calibration. No invented capture probability. |
| 5. Constraints | Supported sensor schema, preserved first hits, prefix+complete backup, missing bounds explicit, no GT online. |
| 6. Intervention | Candidate in safety/evidence sets and simultaneous conservative net paired bound above switching threshold; supported immediate timing option. |
| 7. Abstention | Unidentifiable scene, unknown forecast set/error/calibration or stale/unqualified packet. ABSTAIN executes F only if F independently passes safety. |
| 8. Backup | Revalidate retained backup first; safety override independent of target utility; no backup is an explicit emergency. |
| 9. Pseudocode | Proposal§7, including terminal-first handling, per-env paths, WAIT options, safety precedence and hand-back state. |
| 10. Complexity | Forecast O(NJBKMHP), rollout O(NJKHS), capsule O(NJKLTbackup); chunking saves memory, not work. No10Hz claim. |
| 11. Novelty | At most three conditional candidates; robust baseline regret itself ALREADY_KNOWN. |
| 12. Failures | Unknown space, wrong association, missing calibration, target manoeuvres, no safe gap, empty opportunity set, CPU deadline, harmful endgame delay. |

Crucially, holding F's present action is not predicting the closed-loop policy.
Calibration must compare a one-step intervention followed by the SAME Frozen F
continuation against F, preserving actor action history. Current OnlineInputs
lacks the required measured evidence; the design proposes an explicit whitelist
contract rather than a GT shortcut. Continuous uncertainty needs exhaustive
scenarios or a justified set-remainder bound. A finite scenario grid alone is
not a confidence-certified robust minimum. Unknown calibration implies refusal.

[NOVELTY_REVIEW](../../research/occlusion_algorithm_v1/g4r_2026-10-20/NOVELTY_REVIEW.md)
compares all fourteen requested groups plus the direct2016 robust-baseline-regret
prior. The paired `inf_theta[J_F(theta)-J_g(theta)]` expression is already known.
No first-method, SOTA, safe-improvement theorem or verified novelty claim is made.

## E. Experiment plan: no execution authority

[EXPERIMENT_DRAFT](../../research/occlusion_algorithm_v1/g4r_2026-10-20/EXPERIMENT_DRAFT.md)
defines F0–F5 by actual removed mechanisms, not weights alone; evidence×robust
2×2, immediate/predictive timing, ambiguity and latency stresses; mission,
occlusion, reliability, safety and system endpoints; independent calibration/test
seeds, seed-cluster inference, multiplicity and competing risks. All bar counts,
motion/start conditions and placement lineages remain separate and explicit.

Literal F0 cannot silently receive the new safety gate and still be unchanged F0;
that would be F1. Initially compare method layers with the same qualified safety
gate. Unsafe backup-OFF arms stay CPU/offline. No fixed new sample size or GPU
execution is approved. The existing5 ms CPU-vs-GPU wording conflict is flagged;
neither the old CPU timing nor the new standalone C02 timing establishes it.

If evidence refusal eliminates intervention, report insufficient coverage rather
than weakening bounds. If predictive visibility does not improve capture or adds
harm, reject/narrow that mechanism. No retrospective adjustment on test seeds.

## F. Final verdict

1. **C02 CPU counterexamples resolved?** Yes, in the explicit opt-in planar/U4
   model, including the user next-cycle case. Full C02 is not physically resolved.
2. **What improved?** Whole-disc containment instead of centre/corners, and a
   complete backup from the executed prefix endpoint rather than only horizon end.
3. **B03/B04 risk?** Unobserved continuous volume and ambiguous target/foreground
   identity remain; they prevent a physical certificate and reliable utility.
4. **Distinct publishable principle established now?** No. There is a precise
   candidate design, but neither novelty nor mission benefit has been established.
5. **How distinguished?** Sensor-identifiability/benefit calibration and supported
   timing over Frozen F are candidate specifics; generic robust regret/filtering
   are explicitly reused prior art.
6. **What must be implemented to test it?** Approved measured-evidence schema,
   independent hypothesis/paired-bound/timing kernels and matched continuation
   calibration; only then a separately reviewed integration.
7. **Next implementation?** Review this uncommitted C02 diff; then a separately
   scoped CPU evidence-contract/B03–B04 feasibility package and benefit-oracle
   kernels. No online arbitrator or GPU auto-start.
8. **G5?** HOLD. Nominal geometry is not3-D sensor/plant qualification; broader
   PUBLIC_CPU errors, calibration and real-time constraints remain explicit.

**Executed:** CPU C02 APIs/tests and static preservation checks. **Failure:** broad
PUBLIC_CPU retains pre-existing DetectReduction errors (see validation.json);
the aggregate gate is not PASS. **NOT_TESTED:** actual flight/physics, GPU,
end-to-end capture benefit, calibrated evidence/plant errors, full Raw payload
rehash. **Design only:** new arbitrator, novelty hypotheses and F0–F5 experiments.

## Git Traceability

- Branch: `codex/mt-p1`
- Base SHA / unchanged HEAD: `cdfaffd28534cd37693a6cd678de9766bef4bd25`
- New Commit SHA: **NONE — explicitly prohibited in this instruction.**
- Changed Files: exact full list in HANDOFF.md and validation.json.
- Tests: CPU receipts only; detailed counts/statuses in validation.json.
- Working Tree: approved uncommitted G4-R changes; protected untracked symlinks remain.
- Remote Push: NO
- Main Merge: NO
- Raw/Checkpoint Integrity: preserved scope/path checks and checkpoint hash; Raw
  full rehash NOT_RUN, external backup NOT_CREATED.
