# G4-R3 C02 candidate-independence validation and commit gate

Authority: user G4-R3 instruction,2026-10-20; base
`cdfaffd28534cd37693a6cd678de9766bef4bd25`, branch `codex/mt-p1`.
This protocol precedes the final root verification. No GPU, Isaac, training,
production/controller/governor edit, online integration, push or merge.

## Hypothesis and negative control

Candidate-local failures must not erase healthy candidates in the same env.
Common scan/pose/configuration corruption is a different failure scope. Preserve
the pre-fix source (`0041a40ddb599c400c2e5d0ac7c7a0517d6ffbefa8a7f019c1905a6ab6c883f8`)
and isolated replay before changing the opt-in checker. Baseline HEAD has no C02
API, so replaying that commit alone is not the pre-fix experiment.

Required A–F: one NaN among20; three malformed among3×20; a single inconsistent
prefix; a single kinematically consistent vertical candidate; candidate/env
permutations; and exact healthy singleton-versus-batch results. Expected healthy
counts are19 for A/C/D/F and[20,19,18] for B. E must preserve the restored result,
including numerical diagnostics; permutation invariance alone does not prove
candidate validity. Also test common scan invalidity, used initial diagnostics,
step endpoints, malformed schema, float32/64, RNG/input preservation and correct
per-env scan association in a binding asymmetric scene.

No radius, containment, prefix duration, braking remainder or numerical tolerance
is weakened. Legacy `screen`, `_stopping_envelope`, geometry/types and all previous
tests remain unchanged relative to the reviewed G4-R files. Only the explicit
opt-in candidate adapter/validation changes; no existing caller activates it.

## Final verification

1. Freeze the new source/test hashes. Run the complete BOIA suite: all160 old,
   all29 previous C02 tests and new candidate tests must pass.
2. Check concurrent workloads/disk before PUBLIC_CPU (~3 minutes, previously
   156.5s). Pinned Python, PYTHONNOUSERSITE=1, CUDA_VISIBLE_DEVICES='', single CPU
   thread. Stop additional work below3GB.
3. Run the unchanged PUBLIC_CPU discovery/order/profile through the result-only
   observer supplied here. It does not patch global unittest or any assertion;
   it records method-level outcomes and exception chains. Preserve command,
   interpreter/env, source/runner SHA, logs, every result and exit code.
4. PUBLIC_CPU can remain aggregate FAIL ONLY with the same four pre-existing
   DetectReduction IDs, exception types/messages and terminal call paths as the
   earlier receipt. No new assertion failure/error is allowed for the commit
   gate. Different causes with the same count do not pass. This is not a waiver
   of the public-suite failure and does not release G5.
5. Verify frozen source/tests; original evidence bytes; main HEAD/WIP; stash;
   Policy F hash; protected symlinks; production/controller/governor. Raw full
   payload rehash remains NOT_RUN, external backup NOT_CREATED.

## Conditional local commit

Only after all conditions above, review actual diff and explicitly stage approved
C02 source/tests and compact C02 evidence. Never stage Raw/checkpoints/symlinks or
use add-A. Previous mixed design summaries and algorithm-only proposal files may
remain untracked; this commit is a C02 unit, not blanket authorization to commit
all prior untracked work. Preserve those files unchanged, with their hashes and
scope exclusions recorded. A new C02 report is self-contained in the committed
scope rather than rewriting the old mixed reports.

Record full commit SHA and exact committed file list after success. PART C starts
only after that successful commit. New PART C code/tests/reports stay UNCOMMITTED,
with no online callers. B03/B04/physical C02 remain unqualified and G5 HOLD.
