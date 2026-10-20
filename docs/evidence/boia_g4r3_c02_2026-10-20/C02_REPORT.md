# G4-R3 PART A/B: independent candidates and final C02 CPU qualification

Base `cdfaffd28534cd37693a6cd678de9766bef4bd25`, branch `codex/mt-p1`,2026-10-20.
**CPU MODEL ONLY; physical qualification unavailable; G5 HOLD.** This is a new
record, not a rewrite of G4-R evidence. The user now permits a conditional local
commit of C02 only. PART C starts only after that successful commit and remains
uncommitted and disconnected from production.

## A. Candidate independence

The defect was confirmed as false-block/candidate-set contamination. Pre-fix
`screen_next_cycle` reduced finite, step, timing and planar checks across K into
one environment mask. A malformed sibling then prevented every candidate in that
env from reaching either legacy or new geometry checks. It was not a demonstrated
false-safe or actual controller incident; no online caller uses this API.

| File/function | Defect | Independent counterexample | Actual correction | Verification |
| --- | --- | --- | --- | --- |
| `safety.py::screen_next_cycle` validation | Candidate values reduced to(N). | One NaN candidate19 of20: healthy singleton SAFE but its batch status INVALID. | Candidate masks and time residuals retain(N,K); shared scan mask stays(N). | A–D,F and consumed-field/schema regressions. |
| Same, legacy adapter | Sending a malformed whole env to `screen` could trigger global assertions. | N3K20 with faults(1,5),(2,2),(2,11). | Pack only eligible(env,candidate) pairs as pseudo-envs with K1; gather scans byenv, states byboth indices. | Correct membership plus asymmetric notch/yaw and common-invalid scenes. |
| Same, output adapter | A flat live batch alone would hide candidate provenance. | Reorder both axes; compare healthy candidates to isolated calls. | Scatter outputs and full backup traces into(N,K); explicit live_pair map for compact traces. | Exact numerical equality, zero tolerance/equal-NaN. |

Define `V[e,c]=scan_valid[e] AND finite[e,c] AND step[e,c] AND time[e,c]
AND planar[e,c]`. Packing uses only this predicate. A common invalid scan may
reject its env; a malformed rectangular tensor/configuration is globally INVALID.
Neither is confused with a single invalid candidate value. Unsupported but
kinematically consistent vertical motion is UNKNOWN for that candidate alone.

Every consumed tensor (position,velocity,psi,step endpoints,governed commands and
initial_pos/initial_vel) is validated. Unused optional rollout diagnostics are
explicitly NOT_SAFETY_EVIDENCE. Compatibility timing summaries retain(N) for old
K1 consumers but are marked ENVIRONMENT_AGGREGATE/NOT_USED_FOR_ELIGIBILITY. The
canonical `candidate_validation` preserves both axes. Unevaluated backup entries
are NaN/False, never an invented zero stopping distance or successful backup.

### Actual before/after

| Case | Healthy model-safe counts before | Required/observed after | Other result |
| --- | --- | --- | --- |
| A,one NaN among20 | [0] | [19] | Bad candidate alone INVALID. |
| B,three faults in3×20 | [20,0,0] | [20,19,18] | Exactly3 INVALID. |
| C,one inconsistent prefix | [0] | [19] | Bad candidate INVALID; peers' timing/prefix/backup unchanged. |
| D,one supported-time-grid vertical state | [0] | [19] | Bad candidate UNKNOWN; peers evaluated. |
| E,permutation of B | [20,0,0] | [20,19,18] | Pure permutation was already invariant before; membership and all restored metrics now pass. |
| F,single vs batch | Singleton SAFE / batch INVALID | Exact equality for healthy candidates | Includes clearance,distance,flags and backup trace. |

The expected healthy status is grounded in the open-room/clear-point scalar
fixtures. Singleton comparison is an independence oracle, not the implementation
being used as its own collision oracle. The extra binding-notch/yaw fixture
produces independently expected [SAFE,UNKNOWN,SAFE] env classes and detects
incorrect scan gathering. Archived source and fixtures permit separate replay:
[pre-fix source manifest](candidate_independence_prefix_source_manifest.json),
[replay script](replay_candidate_independence_prefix.py),
[before receipt](candidate_independence_prefix.json),
[after receipt](candidate_independence_after.json).

## B. Regression and source binding

All160 previous BOIA tests,29 previous C02 tests and13 new candidate tests pass:
**202 PASS,0 failures,0 errors,15.089s**. See [boia_final_tests.txt](boia_final_tests.txt).
No old test is changed. The13 new methods cover A–F, common scan/schema faults,
used diagnostics/step fields, asymmetric frame binding,float32 and repeatability.

Final PUBLIC_CPU results, exact per-method outcomes, command/env/source hashes,
exit code and comparison with all four original exception causes are in
[validation.json](validation.json) and [public_error_comparison.json](public_error_comparison.json).
Final count: **2941 run,0 assertion failures,4 pre-existing errors,10 skipped,
5 excluded,160.451s,exit1; aggregate FAIL**. All four IDs, exception types/messages
and terminal call frames match exactly. Only the narrower conditional C02 commit
gate passes; neither PUBLIC_CPU nor G5 is relabelled PASS.
The result observer preserves the original runner's discovery/order/exclusions
and assertions; it changes only the recording result class in an isolated loaded
runner. The global unittest implementation and production source are not patched.
An aggregate FAIL is retained if the known four errors remain.

| New method family | Independently checked condition |
| --- | --- |
| case_a | One position/velocity NaN cannot suppress19 valid candidates. |
| case_b | Exactly3 malformed candidates in3×20 are isolated. |
| case_c | Time residual and consistency remain candidate-shaped. |
| case_d | A time-consistent vertical candidate is UNKNOWN alone. |
| case_e | Candidate and env permutations preserve restored flags,numbers and traces. |
| case_f | Healthy singleton and mixed-batch outputs are exactly equal. |
| common_scan_invalid | A NaN shared scan invalidates only its own env. |
| candidate_diagnostics_and_step | Initial-state/step corruption remains local. |
| consumed_fields | Inf in every consumed field and step-yaw mismatch remain local. |
| asymmetric_scan_binding | Same notch rotated behind the UAV does not inherit another env's UNKNOWN. |
| structural_schema | Malformed shared mask/diagnostic schema returns global INVALID. |
| float32 | Local malformed candidate does not change healthy float32 outputs. |
| repeatability | No input/global RNG mutation; identical repeated output. |

Safety source SHA256:
`b2638ca46324be01858de38144b4a259b425c33970cfb22dc8c58283d3363b1c`.
New candidate-test SHA256:
`4bd8fc65cac3389bd57437caecaf263be6ac350aba3795e5402d97f44f42bbbe`.
Every other C02 helper in safety is byte-identical to the pre-fix snapshot,
including `screen`, `_stopping_envelope`, point checks and complete backup.
Geometry/types and the prior29 tests are unchanged from G4-R.

The body floor(.201990099m), first.1s whole capsule, reaction once,64samples+
analytic remainder, empirical maximum and numerical outward rounding remain.
The `(2,0)->(0,2)` obstacle(.97,.2) counterexample is still rejected by the new
next-cycle backup despite legacy horizon PASS. High-speed12.56% surrogate/table
mismatch remains LIMITED_VALIDITY. No measured physical error bound is invented.

Independent read-only review found no blocking defect. It also tested all-invalid
2×3 and mixed shared-invalid/vertical/NaN/healthy states: the latter returned
`[[INVALID,INVALID,INVALID],[UNKNOWN,INVALID,MODEL_SAFE]]` with only pair(1,2)
evaluated. This is finite/model evidence, not a universal false-safe theorem.

## C. Local commit scope and preservation

[COMMIT_SCOPE.json](COMMIT_SCOPE.json) lists every staged path. C02 source/tests,
compact previous C02 receipts and new A/B evidence only. Protected symlinks,Raw,
checkpoint,production,previous algorithm-only proposals and new PART C modules
are excluded. Existing mixed G4-R report/handoff/manifest remain unchanged and
untracked; their hashes are retained in preflight. This avoids rewriting them
or committing unrelated design work merely to obtain a clean tree.

The staged diff is reviewed only after final verification. The full successful
commit SHA is reported to the user after Git creates it; a commit cannot embed
its own SHA. No push or merge is authorized. See [integrity.json](integrity.json)
for checkpoint/mainWIP/stash/symlink/legacy-source checks. Raw full payload rehash
is NOT_RUN and external backup remains NOT_CREATED.

Staged whitespace review has warnings only in the two verbatim PUBLIC_CPU text
transcripts and two unified-diff artifacts (including required blank context
lines). Their original bytes are preserved. Source/tests/Markdown/WORKLOG pass
the whitespace check; these artifact warnings are not hidden as a clean full
`git diff --cached --check`. The two small new test.log transcripts are explicitly
included despite the generic log ignore rule; no Raw payload is force-added.

## D. Research handoff and remaining limits

PART C is a separate CPU research unit, not code in this commit. It must return
candidate identifiers only, have no online callers and label synthetic exactness,
caller-declared safety and unavailable calibration explicitly. A successful A/B
commit is a prerequisite, not evidence of mission benefit or verified novelty.

Remaining blockers: B03 sparse physical coverage; B04 identity/foreground
ambiguity; actual3-D C02 qualification/rotor footprint; Lee/plant tracking and
latency errors; sensor timing/extrinsics; CPU/GPU deadlines; detector likelihood
calibration; Frozen F continuation mismatch; actual episode capture performance.
These are NOT_TESTED/BLOCKED, not resolved by candidate isolation. **G5 HOLD.**

## Git Traceability

- Branch: `codex/mt-p1`
- Base SHA: `cdfaffd28534cd37693a6cd678de9766bef4bd25`
- New Commit SHA: reported after successful conditional Git commit.
- Changed Files: COMMIT_SCOPE.json; excluded WIP remains listed separately.
- Tests: BOIA202 PASS; final PUBLIC_CPU status in validation.json.
- Working Tree: C02 unit eligible only after exact error/preservation checks;
  unrelated untracked design records and protected symlinks retained.
- Remote Push: NO
- Main Merge: NO
- Raw/Checkpoint Integrity: scoped checks/hash; Raw full rehash NOT_RUN.
