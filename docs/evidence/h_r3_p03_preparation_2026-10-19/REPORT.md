# H-R3 P03 preparation — qualification result

## 1. Research state

**GO_QUALIFICATION_ONLY**, RunID **HR3-P03-Q01**, source **0e8790109a26abdb6d432b50cb7a6b31e9273e9c**.
All registered processes completed and every mechanics/control gate passed. H-R3 remains
**PROVISIONAL / CONFIRMATION HOLD**. **Pure viewpoint attribution: INCONCLUSIVE** because
realized motion is imbalanced. This is one-seed engineering qualification, not a P03 efficacy
pilot or confirmatory experiment. P01 exploratory, P02 NO-GO and ISO-Q02 records are unchanged.

Base: ce3d10aacad9358bc888a53edb6c5e6cf4a904cc. Preregistration commit:
e5d4c7915a965efd140af59125bbb79e83fec667. The user explicitly authorized this bounded GPU unit;
that instruction superseded the stale AGENTS note that no GPU was currently authorized.

## 2. Changed code and reasons

All executable changes are new opt-in evaluation modules; no production runtime source changed.
The complete base-to-result inventory is [CHANGED_FILES.md](CHANGED_FILES.md).

| New file | Main functions/classes | Purpose |
| --- | --- | --- |
| tools/h_r3_p03_isolation.py | RuntimeIsolation.reset_idx, AuditIsolation, _DeviceGuards, attach | Selected-only transform/refit; exact device guards; audit/runtime telemetry |
| motar_eval/h_r3_p03_arms.py | freeze_schedule, sham_signs, evaluate_actions, _candidates | Common pre-loss budget, independent sham, nominal-speed reference |
| tools/h_r3_p03_launcher.py | main | N128 evaluation, split sensor/state digests, numeric windows, control/terminal telemetry |
| tools/h_r3_p03_run.py | preflight, validate, next_gate, run | Immutable-source, bounded stop-on-failure qualification matrix |
| tools/h_r3_p03_analysis.py | load_process, matched_prefix, compare_noninterference, analyse | Fail-closed inventory/chronology/control checks and registered paired analysis |
| tests/test_h_r3_p03_{isolation,arms,analysis,runner}.py | focused test cases | Independent geometry/control/parser/runner regression |
| summarize_results.py (this directory) | main | Post-run CPU-only raw verification, compact receipt and explicitly secondary follow-up |

The preregistration files, CPU/review evidence, compact results and appended WORKLOG are also
new/current-scope records. Historical source, manifests, preregistrations and raw were not rewritten.

## 3. ISO-Q02 integration and 600-step qualification

Both modes use the same selected-slice transform and selected mesh refit as ISO-Q02. Runtime
compares whole vertex/reference/topology bits on-device, including signed zero; it does not
replace integrity with sampling or a lossy fingerprint. Roots, mesh identities, mapping, storage
and refit chronology are guarded. Audit retains detailed CPU reset telemetry. Runtime still has
device comparison/scalar synchronization costs; it is not a zero-overhead path.

| Registered comparison | Scope | Result |
| --- | --- | --- |
| Short normal vs forced env127 at step2 | 127 untouched envs, initial +16 steps | Exact PASS |
| Long normal vs forced env127 at step2 | 127 untouched envs, initial +600 steps | Exact PASS |
| Evaluator ON vs OFF | All128, initial +600 steps | Exact PASS |
| Audit vs runtime | All128, initial +600 steps | Exact PASS |
| B0–B5 common prefix | All128 initial; all121 triggers; no-trigger histories through horizon | Exact PASS |
| Shared pre-loss schedules / B4 hash / nominal norms | Frozen archives and all active decisions | PASS |

Baseline step2 natural reset set was empty; forced reset set was exactly [127] in both pairs.
Selected environment reset execution, indexed root flushes and selected geometry/refit chronology
passed their own gates; env127 is excluded only from the unrelated-env equality comparison.
RGB, depth and LiDAR have separate SHA-256 every frame; roots, tracker/actor history and actions
are compared separately. Registered numeric windows also match exactly: initial plus 12 long-run
frames (0–4,38–42,598–599), and initial plus seven short-run frames (0–4,14–15).
The long B0 pair performed 4174 exact guards and recorded 1779 geometry events per process.
This qualifies the actual N128/seed4801/moving160/600-step profile, not arbitrary future scenes
or a retrospective proof of the cause of every historical P02 mismatch.

## 4. CPU and independent review

Focused CPU **52/52 PASS** after review correction (0.263s), including actual Warp CPU geometry/
ray checks, reset subset/order/topology mutation cases, controls, terminal-first scoring,
malformed inventory, prefix mismatch, and supervisor contracts. [CPU receipt](cpu_validation.json)
and [test log](cpu_tests.txt) bind the exact source hashes.

One actual Fable5.1 read-only source review completed **CONDITIONAL**, no BLOCKER/HIGH.
Its MEDIUM small-positive-distance abstention finding was fixed before GPU; regressions cover
1e-8,5e-7 and the 1e-6 boundary, with independent eight-case CPU checks. Two preflight/env/RNG
contract probes also closed before launch. See [verbatim review](FABLE_REVIEW.md),
[closure](REVIEW_RESPONSE.md), and [receipt](review_receipt.json).
Fable did not execute CPU/GPU or re-review the patch/results; closure is evidenced by CPU/source
checks and subsequent qualification. Direction-invalid abstention means zero horizontal commands;
unchanged yaw remains active. The frozen preregistration's shorthand “all nominal commands” must
not be interpreted as all-action-zero or hover.

## 5. GPU time, memory, storage and limits

Fresh seed4801, N128, moving E2_direct_l2_v1,160bars,0.1s control timestep; sensor mode unchanged
from P02/ISO, RGB-D+LiDAR, Frozen Policy F/898-D. No historical outcomes were reused.
The two 16-step checks preceded nine 600-step checks. No extra process, retry, large pilot or
confirmation was launched. Source/WORKLOG remained frozen during all live processes.

| Process | Steps | Process wall s | Loop wall s | Peak device MiB | Numeric snapshots |
| --- | --- | --- | --- | --- | --- |
| short_normal | 16 | 31.611 | 7.583 | 5398 | 8 |
| short_forced | 16 | 31.122 | 8.472 | 5398 | 8 |
| long_B0 | 600 | 225.622 | 202.465 | 5398 | 13 |
| long_forced | 600 | 220.481 | 196.206 | 5398 | 13 |
| long_noeval | 600 | 212.281 | 187.994 | 5398 | 13 |
| long_audit | 600 | 283.484 | 250.453 | 5398 | 13 |
| long_B1 | 600 | 210.754 | 187.748 | 5398 | 13 |
| long_B2 | 600 | 215.297 | 190.082 | 5398 | 13 |
| long_B3 | 600 | 220.241 | 195.700 | 5398 | 13 |
| long_B4 | 600 | 215.056 | 191.456 | 5398 | 13 |
| long_B5 | 600 | 220.175 | 196.813 | 5398 | 13 |

GPU-process wall total **2086.125s (34.77min,
0.5795h)**; supervisor total **2119.587s** against registered3600s.
These are elapsed GPU-process times, not integrated accelerator utilization. Peak sampled whole-device
VRAM **5398/8192MiB**, including existing background/display use. Disk after run:
**13.900 decimal GB**; final raw bytes **1,735,489,055**.
8GB disk minimum,512MiB reserve,5GB output and individual wall caps did not trigger.

For one matched B0 pair, runtime wall was 20.41% lower than
audit; process-directory bytes were 25.65% lower
(168,943,517 vs 227,215,996).
Both sampled the same peak VRAM. This is a descriptive pair, not a general performance benchmark.

## 6. B4/B5 changes and observed reobservation

B3/B4/B5 share a constant20-step magnitude schedule frozen from the last valid common pre-loss
sensor/belief state, independent of subsequent arm trajectories. Eligible nonzero lateral candidates
set the magnitude; if none exists it stays zero. B3 scores the current side; B4 uses a pure hash of
seed/env/episode/event/decision. B5 pursues the belief at the same pre-safety nominal norm.
No arm reads another arm's future state. Existing safety projection, yaw rules and governor remain.
Budget parity is explicitly nominal; actual motion is measured independently. Of26 geometry events,
24 had nonzero lateral budget and two zero-lateral events remained in the denominator.

There were121 first-loss triggers, all121 matched: FULL21, CENTRE_ONLY5, PARTIAL0 and FOV_EXIT95.
All26 geometry events also had spatially confirmed target identity before loss (diagnostic only;
not a post-hoc inclusion rule). Classes with zero events remain NOT_TESTED.
Primary is spatially identity-correct reobservation within2s, before any terminal including reset.
The P01 metric is recomputed on these new data only, never reused as confirmation.

| Arm | Spatial ID-correct /26 | Rate | Success latency (s) | P01 literal | P01 terminal-corrected | Tracker fresh /26 |
| --- | --- | --- | --- | --- | --- | --- |
| B0 | 21 | 80.77% | 1.052 | 23 | 21 | 24 |
| B1 | 13 | 50.00% | 1.231 | 13 | 13 | 13 |
| B2 | 14 | 53.85% | 1.114 | 14 | 14 | 14 |
| B3 | 22 | 84.62% | 0.950 | 22 | 22 | 24 |
| B4 | 18 | 69.23% | 1.011 | 18 | 18 | 19 |
| B5 | 13 | 50.00% | 0.992 | 13 | 13 | 14 |

Latency is conditional on success. Tracker “fresh” means active and age==0, not persistent identity;
the different counts demonstrate why it cannot replace the spatial metric.

| Class | Denominator | B0 | B1 | B2 | B3 | B4 | B5 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| FULL | 21 | 17 | 10 | 11 | 17 | 14 | 11 |
| CENTRE_ONLY | 5 | 4 | 3 | 3 | 5 | 4 | 2 |
| PARTIAL | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| FOV_EXIT (separate) | 95 | 91 | 34 | 91 | 89 | 90 | 90 |

Entries in arm columns are successful reobservations; PARTIAL has no measurable effect here.
FOV_EXIT is excluded from the geometry primary population and contrasts.

| Contrast | Difference (pp) | Discordant + / − | Population inference |
| --- | --- | --- | --- |
| B3-B2 | +30.77 | 9 / 1 | NOT_ESTIMATED |
| B3-B4 | +15.38 | 5 / 1 | NOT_ESTIMATED |
| B3-B5 | +34.62 | 9 / 0 | NOT_ESTIMATED |

These are descriptive paired differences within one seed, with no population p-value or CI.
No seed-cluster heterogeneity can be estimated from this qualification. No H-R3 confirmation follows.

## 7. Realized speed/path/effort and terminal diagnostics

Means below are over26 geometry scenes, each truncated at its first terminal. Scripted arms have
the full20-step window; B0 averages1.904s because two early terminals occur. Speeds are horizontal,
path is XY, acceleration is3-D. v² integrals are m²/s, acceleration² integrals are m²/s³.
Post-point/cap is before the separately recorded governor; command is after that stage.

| Arm | XY path m | Actual XY m/s | Command XY m/s | Post-point/cap m/s | Actual v² integral | Command v² integral | Acceleration² integral | Point attenuation | Stall s | Nearest LiDAR m |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| B0 | 4.974 | 2.605 | 2.932 | 3.283 | 14.227 | 17.480 | 14.567 | 0.000 | 0.000 | 0.682 |
| B1 | 1.943 | 0.958 | 0.747 | 0.747 | 2.389 | 1.348 | 5.286 | 0.253 | 0.065 | 0.981 |
| B2 | 1.973 | 0.973 | 0.754 | 0.754 | 2.452 | 1.365 | 6.490 | 0.246 | 0.054 | 0.971 |
| B3 | 2.617 | 1.305 | 1.418 | 1.418 | 3.895 | 4.448 | 13.171 | 0.128 | 0.004 | 0.934 |
| B4 | 2.161 | 1.070 | 1.357 | 1.357 | 2.765 | 4.164 | 9.785 | 0.164 | 0.031 | 1.010 |
| B5 | 2.706 | 1.351 | 1.264 | 1.264 | 4.344 | 3.895 | 7.158 | 0.223 | 0.038 | 0.865 |

“Nearest LiDAR” includes target returns and is not independent obstacle-only clearance.
True obstacle-only minimum clearance and mechanical actuator energy are **NOT_RECORDED**.
Squared speed/command and acceleration are named kinematic proxies, never substituted for energy.
Per-scene 3-D path, angular velocity, all safety stages and tracker timing remain in raw/analysis.

Registered descriptive imbalance = mean over paired scenes of
abs(a−b)/((abs(a)+abs(b))/2); both-zero pairs count as equal. The predeclared10% margin is
an engineering diagnostic, not a formal statistical equivalence test or direct-effect identification.

| Contrast | XY path | Actual XY speed | Actual v² integral | Command v² integral | Acceleration² integral | Attribution |
| --- | --- | --- | --- | --- | --- | --- |
| B3-B2 | 29.51% | 30.96% | 45.68% | 101.94% | 61.89% | INCONCLUSIVE |
| B3-B4 | 20.25% | 20.89% | 34.06% | 18.35% | 32.37% | INCONCLUSIVE |
| B3-B5 | 20.59% | 21.34% | 31.60% | 39.01% | 55.68% | INCONCLUSIVE |

All comparisons fail the realized-balance diagnostic. B3/B5 group mean speed/path look close,
but opposite scene-level differences cancel in those means; matched scene imbalance is about21%.
B4's decision-by-decision sham may alter actuation smoothness. No safety override or post-treatment
matching/exclusion was used. Even a future diagnostic pass would not alone identify a controlled
direct effect of viewpoint independent of motion.

| Arm | 2s task crash / contact | 5s capture / task crash | Episode capture / task crash | Episode contact | Timeout / censored |
| --- | --- | --- | --- | --- | --- |
| B0 | 2 / 2 | 13 / 5 | 21 / 5 | 5 | 0 / 0 |
| B1 | 0 / 0 | 4 / 3 | 22 / 4 | 4 | 0 / 0 |
| B2 | 0 / 0 | 4 / 2 | 21 / 5 | 5 | 0 / 0 |
| B3 | 0 / 0 | 4 / 1 | 24 / 2 | 2 | 0 / 0 |
| B4 | 0 / 0 | 3 / 0 | 25 / 1 | 1 | 0 / 0 |
| B5 | 0 / 0 | 4 / 1 | 24 / 2 | 2 | 0 / 0 |

All denominators are26. Task crash can include non-contact termination; actual contact is separately
reported. Here contact counts coincide with task crash counts in these windows. Within2s capture
and timeout are zero for all arms. The first5s flags are registered secondary metrics. Whole-episode
follow-up is an explicitly **post-hoc descriptive** read of the already-saved trace, through the
first original-episode terminal, never a later reset episode. No terminal flags overlapped in this
sample, and no episode was censored. Intervention ends after2s, then Frozen F resumes; the eventual
capture counts are therefore intervention-plus-handback outcomes, not continued scripted control.
Safe abort was not recorded. B4 has more final captures than B3 despite fewer early reobservations;
no overall task superiority is established. See [follow-up](secondary_followup.json).

Four B3 failures are retained without retuning:

| B3 non-reobservation env | Class | Common pre-loss lateral available | 2s terminal |
| --- | --- | --- | --- |
| 45 | FULL | True | None |
| 73 | FULL | True | None |
| 101 | FULL | False | None |
| 105 | FULL | True | None |

## 8. Residual risks and H-R3 status

- **INCONCLUSIVE:** pure-viewpoint attribution and population efficacy. One seed,26 geometry
  events, five centre-only cases and no partial cases do not establish generality.
- **Known measurement limits:** raw LiDAR is a clearance proxy; energy and safe abort were not
  recorded; tracker freshness is not semantic identity. Spatial evaluator overlap is an analytic
  silhouette association rule, not validation of real-camera perception realism.
- **Profile-limited mechanics:** exact600-step tests now extend the previous40-step scope only
  to this configuration. Future seeds/runs still need their own prefix/integrity gates.
- **Control-design limitation:** per-decision sham dynamics and direction-dependent safety remain
  part of a total direction-policy contrast. Safety and realized motion were not experimentally
  eliminated as mediators/confounders of a pure-viewpoint interpretation.
- **GT firewall:** online controls accept RGB-D+LiDAR-derived belief, own state and permitted
  sensor information; evaluator geometry/identity stays outside control. Source-bound tests and
  evaluator OFF/ON exact parity pass; this is not a new real-flight or sensor-fidelity claim.

## 9. Next P03 proposal — no execution

See [NEXT_P03_PLAN.md](NEXT_P03_PLAN.md). First settle whether the estimand is the total effect of
direction policy under a common nominal budget or a narrower mechanism claim requiring further
control design. An optional later exploratory pilot is seeds4802/4803, B0/B2/B3/B4/B5, moving160,
N128,600steps: ten processes, one-hour proposed aggregate cap, roughly35–45min predicted process
wall from the current profile. This is **PLANNED / NOT_EXECUTED / USER_REVIEW_REQUIRED**.
Do not launch it or a large confirmation from this result. P01/P02/qualification cases cannot enter
the future confirmatory sample. Sample-size sensitivity is assumption-only, not powered from the
favourable qualification difference; independent seed clusters and Holm correction are required.

## 10. Provenance, preservation and handoff

- Branch codex/mt-p1; base ce3d10aacad9358bc888a53edb6c5e6cf4a904cc; source 0e8790109a26abdb6d432b50cb7a6b31e9273e9c.
- Source and preregistration were committed before GPU; result commit is the commit containing
  this report. No push or main merge. Last verified remote ce3d10a remains the pre-task state;
  local-only changes are not visible on GitHub.
- Raw: `/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/h_r3_p03_qualification_v1/HR3-P03-Q01`. Compact manifests/evidence are tracked; raw is not committed or moved.
- New raw inventory fully verified: 221 members, 1,735,455,078 payload bytes.
  Manifest SHA256 `50401c0bbf0211758d4cf466f3465fa00e977f10ce19b220c59017e655bb2099`; analysis SHA256 `d01983f43c8b9064376caed24f7d5319fc0840e4a8c29f20ab876949caea7ae8`.
- Historical preservation scope:6035 original tracked paths (WORKLOG append-only),12 prior raw
  manifest SHA checks plus463 member size/mtime checks. Historical payload full hashing was not
  repeated. FrozenF and RMS full hashes, main cb644bbe, its three WIP files, stash and two protected
  untracked symlinks are preserved. See [preservation receipt](preservation.json).
- FrozenF SHA256 f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40;
  RMS SHA256 642fc9f7f86b068e816e156fea7a34782bbc525966650f73cd85aae8cc18387f.
- No owned GPU/review process remains. Session started01:03KST Oct19; actual quota deadline is
  unavailable. Conservative wrap-up05:03KST remains. This authorized unit finishes with a local
  result commit; a later session begins by reviewing the plan, not automatically launching GPU.

Reproduce analysis into a new scratch file (never overwrite raw):

```bash
PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES='' /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/h_r3_p03_analysis.py --run ../aerial_gym_simulator/results/h_r3_p03_qualification_v1/HR3-P03-Q01 --out /tmp/hr3-p03-reanalysis-new.json
PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES='' /home/fair/miniconda3/envs/aerialgym/bin/python -B docs/evidence/h_r3_p03_preparation_2026-10-19/summarize_results.py --raw ../aerial_gym_simulator/results/h_r3_p03_qualification_v1/HR3-P03-Q01 --out /tmp/hr3-p03-compact-new
```

Original bounded launch command at the frozen source (historical reproduction only; not an
instruction or authorization to reuse RunID/output): `python -B tools/h_r3_p03_run.py smoke
--approved-commit 0e8790109a26abdb6d432b50cb7a6b31e9273e9c`. Executing it now must fail its source/output guards.
