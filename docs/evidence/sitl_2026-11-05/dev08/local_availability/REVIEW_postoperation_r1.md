# Independent post-operation review — historical asset availability

**Verdict: PASS for the19-file local availability materialization and its CPU verification.**
No open integrity finding. The restored files exactly reproduce their preexisting canonical
bindings; source/test code and historical verdicts remain unchanged. This is no scientific gate,
SITL/GPU execution approval, new policy or full raw-dataset restoration claim.

Reviewer: independent Codex subagent `/root/dev08_prefix_review`; Codex/GPT-6 family stated by
the environment, exact backend NOT_RECORDED. Worktree MOTAR-camonly, branch camonly/dev,
source HEAD `aa047269fdbeee4e65c8e99c6b7f29e817ee0b89`. The separate pre-operation review
REPORT.md remains unchanged; its two repaired receipt findings and initial source are preserved.
No apply was performed by this reviewer, no tracked files were changed and no checkpoint was
deserialized by the materializer or this review.

## Actual operation and integrity

- Read actual materialization_receipt.json and all38 progress.jsonl events. The receipt is
  COPIED_VERIFIED with19 copies and no active/failed spec. Each planned file has exactly one
  ordered ATTEMPT_STARTED and COPIED_VERIFIED event; completed events equal receipt entries,
  expected spec fields match, actual source/target bytes+hashes match and timestamps fit the
  operation interval 2026-11-05T07:42:22.844437+00:00–2026-11-05T07:42:23.183104+00:00.
- Independently rehashed all19 source and destination payloads,54,147,345bytes per set. All
  hashes/sizes match the reviewed canonical bindings. Each target is a regular file with
  link count1, distinct device/inode identity from its source, within camonly and already
  ignored. This is a byte copy with no symlink/hardlink alias. Five copied F snapshots and the
  distinct historical P1c ep900 payload retain their original hashes; no policy identity changes.
- The canonical results/MANIFEST.json hash remains
  `bb595589dcf7141eb7295cb876940426b6b88c1526b6fba87210282da49c8783`. All453 unique asserted paths now exist inside the
  worktree. All245 asserted file hashes independently match, with zero drift. The initial
  absence and matching-original evidence remains in the preparation audit.
- The focused three-module log records145 tests,0 failures/0 errors, OK. The actual D1 test
  explicitly supplies REF5IN_D1_PREFLIGHT_ONLY=1. Focused test source bytes equal the review
  base. These are observed parent-run results, not an independent second test execution.
- Actual postcopy refusal log shows an occupied first destination being rejected. Receipt,
  journal and all19 source/target hashes remain unchanged after that refusal and the full
  test run. Source/test code under tools/tests/aerial_gym has no diff from the review base.
  Git was independently clean at the earlier post-operation checkpoint; parent documentation
  records/commit are a separate subsequent step.

## Full CPU and host result

Actual full PUBLIC_CPU log ends with3298 tests in221.701s, **PASS:0 failures,0 errors,
10 skips and5 GPU-dependent exclusions**. The10 skips/exclusions remain explicit; this does
not claim GPU-required validation. The earlier3298-test2failure/2error log and its historical
record remain unchanged. Restoring availability has not relabeled that earlier attempt.

Actual host_before.json and host_after.json both record fault_count0. Full-suite summary,
counts and elapsed value were parsed from the actual final log, and both host snapshots were
read. Their hashes bind this review. No full-suite rerun was needed.

## Verification scope and resources

Detailed post-operation spec/inode/journal/manifest checks are in post_operation_checks.json;
final logs and stable hashes are in final_validation_checks.json. The initial post-operation
read-only window was 2026-11-05T07:44:25.147122+00:00–2026-11-05T07:44:25.271766+00:00, overlapping the parent CPU
suite on core6/nice19 with CUDA hidden/PYTHONNOUSERSITE=1,1GiB address-space/30sCPU/60s outer
bounds. Final log/hash checks ran 2026-11-05T07:49:39.438754+00:00–2026-11-05T07:49:39.498394+00:00 under
core6/nice19,512MiB/15sCPU/30s outer bounds. No timing-performance inference follows.

The operation restores only the exact19 asserted/test-bound assets. Files referenced inside
historical receipts may have additional availability limits. Runtime observed-prefix design
remains unimplemented/NOT_RUN, strict600 remains blocked, and S3a/formal S3b/real-sensor/flight
qualification remain outside this availability result.

## Evidence bindings

| Actual evidence | SHA-256 |
| --- | --- |
| `public_cpu.log` | `8f93c16e1894bffbcd5ac695144c3143ddb157a08cde61452f062464c7e2352d` |
| `focused.log` | `ca18d1c86619a4dd5e58b8f2a2d77d0952368ea469cedcd0f4fe262f2a6f5f44` |
| `host_before.json` | `84659e1c2578c385abdf65ab2cc6420092fc80faa5f67c7dc920aacab71e0be0` |
| `host_after.json` | `84659e1c2578c385abdf65ab2cc6420092fc80faa5f67c7dc920aacab71e0be0` |
| `occupied_refusal.log` | `5c15f31b58fcf6a5cecef12f6afe2f340c0b1f3e9d54b816909fe15726099a7c` |
| `materialization_receipt.json` | `fbebfb55bd7ef47ec4bdacb81d23ee8d0ff73b4cff00639747d09d0a6c6aa7a7` |
| `progress.jsonl` | `5fe29522a100cad4aa731ccd8c30e8223f6dd927de3c2507650ad763acbc9323` |

Reviewed applied script SHA-256:
`bb9b1aea4dfb907f30c1cf355fe248a26a467273eabca319e0814ba0d8e725a7`.

## Git Traceability

Branch camonly/dev; Base SHA aa047269fdbeee4e65c8e99c6b7f29e817ee0b89;
New Commit SHA NONE by reviewer; Changed Files reviewer external audit only, parent created19
previously absent ignored files; Tests focused145/145 and actual PUBLIC_CPU3298 PASS with
10skips/5exclusions; Working Tree independently clean after operation before parent records;
Remote Push NO; Main Merge NO; Raw/Checkpoint Integrity all19 originals remain hash-identical,
all19 targets match canonical identities without links, manifest unchanged, no source/test
weakening or materializer/reviewer checkpoint deserialization.
