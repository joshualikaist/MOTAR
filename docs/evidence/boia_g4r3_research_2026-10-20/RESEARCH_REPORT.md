# G4-R3 final review: C02 preserved; independent CPU research only

Authority: user's G4-R3 attachment. Branch `codex/mt-p1`; original base
`cdfaffd28534cd37693a6cd678de9766bef4bd25`. PART A/B completed before PART C.
C02 local commit **8689efcd066d64e7b49cecf011e3bd8675380cdb**.
PART C remains **UNCOMMITTED by explicit instruction**, including this report.
No GPU, Isaac, learning, online connection, production/controller/governor edit,
push, merge, reset, stash operation or data deletion. RGB-D+LiDAR/actor898 retained.

## A. Candidate independence fix

| File/function | Defect/counterexample | Actual correction / independent check | Result |
| --- | --- | --- | --- |
| `motar_eval/boia/safety.py::screen_next_cycle` | One NaN candidate contaminated env-wide finite reduction. A:0 healthy of19; B healthy counts[20,0,0]. | Candidate(N,K) validation; pack eligible(env,candidate) as K1 pseudo-envs; gather shared scans only by env; scatter status/metrics/full traces. | A:19/19 healthy; onlyc19 INVALID. B:[20,19,18]. |
| Same, timing/planar fields | One time-grid mismatch or unsupported vertical motion blocked healthy siblings. | Local finite/step/time/planar masks, separate common scan/schema errors. | Required C/D healthy19/19; invalid or UNKNOWN isolated. |
| Same, binding/scatter | Singleton healthy became INVALID in mixed batch; wrong yaw/scan gather could hide this. | E/F exact permutations, singleton-vs-batch numerics, asymmetric env-specific radial notch/yaw fixture, used-field Inf/common faults. |13 new tests PASS; archived exact pre-fix replay retained. |

Pure permutation alone was already invariant before; no permutation-only failure
is invented. All six existing safety helpers and the prior29 C02 tests remained
byte-identical. Whole-body circumsphere0.20199009876724155m, actual first0.1s
prefix, one reaction coast, zero-command model,64 backup samples plus analytical
remainder and outward numerical bounds remain. The required orthogonal-motion
counterexample still rejects the candidate. `MODEL_SAFE` qualifies only the
declared CPU model, never continuous3D free space or real UAV safety.

Full before/after tables, replay, source diff and staged review:
[C02 report](../boia_g4r3_c02_2026-10-20/C02_REPORT.md).

## B. Final regression verification

| Frozen source scope | Count / result | Interpretation |
| --- | --- | --- |
| C02 final source, before PART C |160 existing+29 priorC02+13 new=202;0failures/0errors;15.089s;exit0 | BOIA PASS. |
| Same source, PUBLIC_CPU |2941;0failures/4errors;10skips/5excluded;160.451s;exit1 | **FAIL**. Four pre-existing DetectReduction errors match IDs, exception chains and terminal frames; no new failure/error. |
| New PART C focused |30/30 PASS; independent reviewer also30/30 | Exact synthetic/declaration-contract checks only. |
| Final source including PART C |232=202+30;0failures/0errors;15.435s;exit0 | Full BOIA-pattern regression PASS. No new whole-PUBLIC_CPU run is claimed for PART C. |
| Root independent receipt |17 checks PASS; includes two explicit decision-rule contrasts | Not mission measurements; all positive navigation-selection fixtures EXACT_SYNTHETIC. |

Pinned prefix used for test/oracle commands:

```bash
PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 TMPDIR=/tmp/codex_occ /home/fair/miniconda3/envs/aerialgym/bin/python -B
```

Arguments after that prefix:

```text
-m unittest discover -s tests -p 'test_boia*.py' -v
docs/evidence/boia_g4r3_c02_2026-10-20/run_public_cpu_with_results.py --output /tmp/codex_occ/g4r3_public_cpu_results.json
docs/evidence/boia_g4r3_research_2026-10-20/run_independent_oracles.py --output docs/evidence/boia_g4r3_research_2026-10-20/independent_oracle.json
tools/check_public_docs.py
```

PUBLIC runner used timeout600s. It records per-method results through a result-only
observer, without changing discovery, ordering or assertions. Full exact command,
environment/source SHA maps and old-error comparison live in the committed
[C02 validation](../boia_g4r3_c02_2026-10-20/validation.json).
Current source/test SHA maps, final command/exit and log are in
[research validation](validation.json) and [full BOIA log](boia_final_tests.txt).
Safety source SHA256 is
`b2638ca46324be01858de38144b4a259b425c33970cfb22dc8c58283d3363b1c`.
Existing tests were neither removed nor relaxed.

## C. Local commit and exact scope

Base `cdfaffd28534cd37693a6cd678de9766bef4bd25` →
**8689efcd066d64e7b49cecf011e3bd8675380cdb**; one local C02 commit,39 paths.
Exact list: [COMMIT_SCOPE.json](../boia_g4r3_c02_2026-10-20/COMMIT_SCOPE.json).
Three BOIA source files, three C02 tests, selected prior/new compact C02 evidence
and WORKLOG were explicitly staged. No raw, checkpoint, protected symlink or
production file was staged. Prior mixed summaries/proposals/manifests remain
untracked and unchanged. PART C source/tests/evidence plus its WORKLOG append are
unstaged/uncommitted. [integrity.json](integrity.json) and
[uncommitted_paths.json](uncommitted_paths.json) record the exact final state.

Staged source/tests/Markdown/JSON whitespace review passed. Full staged
`diff --check` had exit2 solely on preserved console-output and unified-patch
context whitespace, explicitly recorded rather than silently called clean.
Credential-pattern scan passed within its pattern scope. No push/merge occurred.
Main WIP3, main HEAD, stash, frozen checkpoint hash, symlinks and earlier evidence
hashes were checked. Full raw rehash NOT_RUN; external backup NOT_CREATED.

## D. Implemented new CPU research module

Five new pure-stdlib modules,484 source lines under `motar_eval/boia_research/`:

| Module / function | Implemented contract |
| --- | --- |
| `types.py` | Immutable measured-evidence/provenance/interval/loss/error-budget/option/state/result records; no task handles or executable actions. |
| `evidence.py::build_hypotheses`, `validate_probability` | Preserve first-hit intervals and nonexclusive TARGET_SURFACE/FOREGROUND_OCCLUDER/FALSE_ASSOCIATION/UNKNOWN interpretations. Timestamp/frame/ray/provenance gaps refuse utility. No GT resolution or carving. |
| `benefit.py::paired_bound` | Exact env/episode/decision/scenario/path pairing; worst scenario mean(F-loss minus option-loss) less separate MC,coverage,calibration,continuation,switching allowances. Unsupported/missing proof refuses. |
| `decision.py::opportunity_window` | Finite candidate/wait grid with entire cumulative waiting-interval safety; report supported times, first connected current window and finite-grid boundary/censoring. |
| `decision.py::decide`, `initial_state` | KEEP_F/INTERVENE/ABSTAIN/SAFETY_OVERRIDE/NO_CERTIFIED_BACKUP, reason codes, safety-first guards, endgame, owner→F hysteresis and monotonic decision IDs. No mutable global state. |

The exact utility rule maximizes supported conservative gain over safe finite
`(candidate,wait)` pairs, not just gain of current actions. Immediate winner selects
INTERVENE; a future winner returns KEEP_F/WAIT and must be recomputed later. Current
window gaps are not bridged. `closing` is **diagnostic**, not a separate overriding
trigger. Unsupported utility cannot authorize unsafe F; no current backup returns
NO_CERTIFIED_BACKUP, not successful stop or inevitable physical collision.

Independent scalar receipts: paired worlds(10,9),(1,0) give gain1, while invalid
cross-world `min(F)-max(g)` gives−8. Adding world(0,2) yields−2. Gain.4 minus switching
cost.5 yields−.1 and KEEP_F. Supported now.4/later.9 with safe wait yields WAIT;
immediate-only yields INTERVENE. Unsafe waiting blocks later benefit. Before-loss
positive benefit can INTERVENE while an actual-loss-only trigger keeps F.

All positive action-selection demonstrations use exact synthetic loss tables.
One test-only HELD_OUT_DECLARATION fixture checks the conditional Hoeffding
arithmetic; it is not actual held-out data. Real artifact verification, sensor→loss
generation, calibration, scenario coverage, actual F continuation and collision
certification are **not implemented/qualified**. Supplied booleans/tags and matching
IDs cannot establish these facts. Focused AST/schema/import tests are not a
universal information-flow proof.

Per-option paired bound costs O(B*M) time/storage; selection currently
O(O²+O*J+OlogO) for O options,J waits, not linear. For a prebuilt2-option/3-wait case,
1000 repeated CPU kernel calls: median9.536µs,P95=10.354µs,P99=14.232µs,max47.021µs.
These exclude perception, trajectory/safety/forecast generation, transfers,
tracing/SHA and actual controller; no WCET/full-planner/deadline claim.
See [module specification](CPU_MODULE_SPEC.md), [oracle protocol](ORACLE_PROTOCOL.md)
and [independent receipt](independent_oracle.json).

Independent read-only review:30 focused PASS plus stale-proof, gap and option-
isolation probes; no blocker within declared typed CPU scope. Reviewer SHA map
matches the final source/test bytes. [Review record](INDEPENDENT_REVIEW.md).

## E. Research novelty

[Required13-group comparison](NOVELTY_AND_EVALUATION.md) covers FASTER,BPOD,
Eva-Tracker,Elastic Tracker,UAST,YOPOv2-Tracker,NavRL++,PRIMER,risk-averseMPC,
predictive safety filters,runtime assurance,context-POMDP tracking and robust
baseline regret, with links, reading boundaries and proposed mechanism ablations.

| Mechanism | Closest overlap | Candidate distinction / necessary test | Status |
| --- | --- | --- | --- |
| Backup and safety mediation | FASTER / safety filters / runtime assurance | Qualification, not novelty; common mandatory safety in every comparison. | ALREADY_KNOWN |
| Uncertainty/paired robust gain | Robust baseline regret / risk-averse MPC | Concrete measured-evidence/continuation construction needs validation. | Formula ALREADY_KNOWN; construction NOT_VERIFIED |
| Evidence-supported finite timing | Active perception / tracking / predictive control | Same inputs/safety/options, immediate-only vs wait-grid vs loss-reactive mission contrasts. | CANDIDATE; novelty NOT_VERIFIED |
| Closing-window flag | Finite predictive option timing | Diagnostic only; no independent closing-mechanism or capture gain shown. | NOT_VERIFIED |

No component has VERIFIED novelty. Do not present a synthetic WAIT-vs-threshold
contrast as a new control principle. The research contribution must survive
matched mission comparisons and deeper prior-art review; otherwise narrow or
withdraw it. F0–F5 mechanisms, outcome accounting and rejection rules are in the
comparison document. No future sample size or synthetic capture rate is invented.

## F. Remaining blockers / exact next unit

| Blocker | Status / next required evidence |
| --- | --- |
| B03 sparse LiDAR volume coverage | BLOCKED; finite beams do not certify continuous free volume. |
| B04 target/foreground association | BLOCKED; indistinguishable first hits retained, not silently erased. |
| C02 physical qualification | NOT_AVAILABLE; declared capsule/plant/scan model only. |
| Actual dynamics and controller tracking error | NOT_TESTED; zero command is not immediate stop. |
| Sensor timing / SE(3) / association interval calibration | NOT_QUALIFIED for the new research adapter. |
| CPU/GPU deadline | NOT_QUALIFIED; tiny selector timings cannot replace full-planner/plant timing. |
| Detector likelihood / coverage / error budgets | Actual artifacts unavailable; declarations are not calibration. |
| Actual episode capture benefit | NOT_TESTED; no GPU/Isaac/flight/episode experiment. |
| Frozen F continuation mismatch | NOT_BOUNDED empirically; supplied losses are not closed-loop F value. |
| G5 | **HOLD**, no online or GPU progression. |

Next approval request is concrete: review this pure CPU diff, then permit its
separate local commit and a CPU-only **measured-evidence and F-continuation
qualification design** (Bead `motar-3p3.19`). Define actual artifact/adapter inputs,
held-out splits, coverage/refusal and independently checked loss/continuation
estimands before any calibration implementation or online hookup. Existing
`motar-3p3.13`/`.15` blockers and `motar-6nn.5` remain open. No automatic GPU follows.

## G. Eleven explicit verdicts

1. Candidate independence fixed: **YES for declared CPU contract**, A–F and mapping controls pass.
2. New false-safe: **none observed in checked model regressions**; universal/physical absence unproven.
3. C02/full BOIA regression: **PASS**,202 committed-scope and232 including research.
4. C02 final PUBLIC_CPU new errors: **NONE detected**, exact old4 remain; aggregate FAIL.
5. C02 local commit: **YES**,8689efcd066d64e7b49cecf011e3bd8675380cdb.
6. New CPU research module: **YES**, five modules and30 tests; uncommitted by instruction.
7. Difference from threshold switching: supported robust argmax over candidate AND safe wait time, with refusal/identity/freshness/continuation budgets; not only a current gain threshold.
8. Window effect: **CPU decision difference demonstrated**; actual mission gain NOT_TESTED; closing flag itself diagnostic.
9. Paper novelty: **CANDIDATE / NOT_VERIFIED**; generic safety/regret/MPC principles ALREADY_KNOWN, none VERIFIED.
10. Priority: actual measured-evidence / mission-loss / Frozen-F-continuation CPU qualification design, approval required.
11. G5: **HOLD**; all scope restrictions remain.

## Git Traceability

- Branch: `codex/mt-p1`
- Base SHA: `cdfaffd28534cd37693a6cd678de9766bef4bd25`
- New Commit SHA: `8689efcd066d64e7b49cecf011e3bd8675380cdb` (C02 only)
- Changed Files: exact39 committed paths in COMMIT_SCOPE; PART C exact uncommitted paths in uncommitted_paths.json.
- Tests: C02 BOIA202PASS; PUBLIC2941/4existingerrors FAIL; research30PASS; finalBOIA232PASS; independent17checksPASS.
- Working Tree: PART C intentionally uncommitted; preserved earlier untracked evidence/proposals/symlinks; index empty.
- Remote Push: NO
- Main Merge: NO
- Raw/Checkpoint Integrity: checkpoint/mainWIP/stash/symlink/old-evidence hashes checked; fullRaw rehash NOT_RUN; external backup NOT_CREATED.
