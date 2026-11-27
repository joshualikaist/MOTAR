# Independent pre-operation source review r1 — historical asset availability

**Verdict: PASS_WITH_LIMITS; scoped readiness for the reviewed19-file exclusive materialization.**
Final script SHA-256 `bb9b1aea4dfb907f30c1cf355fe248a26a467273eabca319e0814ba0d8e725a7` is acceptable for restoring the exact absent,
already-ignored camonly files from their matching main originals. No open source blocker remains.
This verdict concerns the concrete local copy operation only. It is not a successful operation
record, a PUBLIC_CPU PASS, authorization for training/evaluation, or a checkpoint/policy change.
The parent must inspect actual receipts/source+destination hashes and test results afterward.

Reviewer: independent Codex subagent `/root/dev08_prefix_review`. The environment identifies
Codex as based on GPT-6; exact backend NOT_RECORDED. Root worktree is
`/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, clean reviewed HEAD
`aa047269fdbeee4e65c8e99c6b7f29e817ee0b89`. Shared root clock was read and retained; deadline
11:47:14Z, wrap10:47:14Z, finish11:37:14Z on2026-11-05. No reviewer clock or Beads mutation.

## Findings and repairs

- **MEDIUM-1, repaired — incomplete active-file failure accounting.** The initial script only
  journaled fully verified copies. A mid-write/hash failure preserved its path on disk but did
  not identify it in copied/progress, and the generic verification error omitted the path.
  The final script fsyncs an ATTEMPT_STARTED event with exact spec/time before parent/target
  creation, retains active_spec, and logs actual size/hash and timestamps on success or
  failure. Partial files remain intact; no silent cleanup/replacement occurs.
- **MEDIUM-2, repaired — appending another invocation's existing progress file.** After the
  first repair, failure to exclusively create progress.jsonl still entered an exception branch
  opening that existing file for append. Final code sets progress_owned only after its own
  successful exclusive create and appends failure events only when that ownership holds.
  Otherwise failure is recorded only in its newly created operational receipt.

Initial source SHA-256 was
`e1ece1321d91eafb9b514f00fca3c48a331506d6ac8f094024cd371962fdddad`; the initial snapshot is preserved under
`initial_snapshot/`. Both repairs were inspected in actual final source. Dynamic copy/write-
failure controls were NOT_RUN because this review was explicitly prohibited from invoking
--apply. Actual post-operation evidence remains necessary.

## Independent source and input checks

- Read PLAN.md, availability.json, destination_checks.json, the audit inventory implementation,
  materialize_missing.py, current AGENTS/WORKLOG/shared handoff and motar-lp5.6. Confirmed current
  HEAD and clean Git status. The full PUBLIC_CPU had ended before this review; no apply was run.
- Independently hashed all19 selected main source files as opaque bytes and checked their
  exact byte counts. Every destination is absent, contained in camonly and already ignored;
  all existing ancestor components are directories without symlinks. Exactly five parent
  directories are missing. Total source bytes54,147,345; six files are checkpoints.
- Independently rederived the eight result-file bindings from the actual camonly canonical
  manifest. Rechecked all five stopcap receipts' source/snapshot F identity, corresponding
  result JSON hashes and log/snapshot expected hashes. Rechecked P1c's checkpoint hash from
  both the actual committed launcher and summary. All19 copy specs match those bindings.
  No checkpoint was deserialized, substituted, trained or promoted.
- Source opens are read-only. Destination copies use exclusive xb creation, so an occupied
  leaf, including a symlink, refuses rather than overwrites. No hardlink/symlink, deletion,
  replacement or source-write operation exists. Existing directory creation is limited to
  the pinned per-file parents. Containment and occupancy are checked again immediately
  before creation and parent containment is checked after mkdir.
- Preflight pins the audit's exact hash, clean camonly HEAD, canonical manifest hash,19-file
  count/aggregate bytes, exact main-to-camonly relative mappings, source regularity/hash/size,
  uniqueness/absence and existing ignore rules. It refuses named PX4/gzserver and run_tests.py
  processes. The parent must retain the stated idle/no-freeze precondition through this short
  operation; a preflight is not permission to start a concurrent job.
- Per-file data are flushed/fsynced, target/source size+hash are rechecked, durable progress is
  appended, and all19 source/target hashes plus canonical manifest are checked again at end.
  Exceptions retain partial copies and failure receipts. Existing receipt/progress paths are
  not repurposed as a silent resume: the concrete script refuses occupied destinations and
  does not implement PLAN's optional VERIFIED_EXISTING resume behavior.
- The default read-only preflight accepted the real19-file set. Independent negative controls
  rejected changed audit/HEAD pins, changed manifest/source hashes, an occupied destination
  and an escaping destination. `initial_checks.json` and `final_checks.json` bind these probes.
  No --apply path was invoked, no repo files were created and no source payload was written.

## Scope and pending verification

The plan materializes matching missing local copies; it neither fabricates a historical result
nor reinterprets a negative verdict. It restores only the test/asserted subset, not necessarily
all paths referenced inside copied receipts/source manifests. It needs no .gitignore/exclude
change, force-add, source/test weakening, push or merge. Existing raw/checkpoint originals in
main and mt-p1 remain read-only. The five copied F snapshots retain the recorded F hash and
P1c retains its distinct historical ep900 hash; no policy identity changes.

After apply, independently inspect the19 completed receipts, exact source/target hashes,
nonalias regular-file identities, unchanged canonical manifest/bindings/source tree and host
fault counters. Then inspect bounded focused tests and the broader PUBLIC_CPU rerun. The
pre-operation3298-test run's2failures/2errors remain historical evidence; no global PASS is
claimed in this report. D1 verification must use the existing PREFLIGHT_ONLY test path and
must not launch training or deserialize a checkpoint.

## Evidence hashes and resources

| Input | SHA-256 |
| --- | --- |
| `audit/PLAN.md` | `08e5d9bf189e6a1a2369af0ba9d5f3500a63ba5691b5fe51780e555e96ce048b` |
| `audit/availability.json` | `dce42883849fcb66531196dd28be488176096c8a415344b6d8ef920a016d5952` |
| `audit/destination_checks.json` | `1ebbef0cd4700a5685529b05f4e8b3dac31bcb2e9f80aeffdf0b4b938083fdfb` |
| `audit/audit_availability.py` | `efff28172d5fe9c7965858f7cf523825d07b719a2f4509a7327230afe241c9ef` |
| `materialize_missing.py` final | `bb9b1aea4dfb907f30c1cf355fe248a26a467273eabca319e0814ba0d8e725a7` |

Read-only initial validation ran 2026-11-05T07:40:42.007101+00:00 to
2026-11-05T07:40:42.195701+00:00; final validation ran 2026-11-05T07:42:00.071160+00:00 to
2026-11-05T07:42:00.244026+00:00. Both used core6/nice19, CUDA hidden, PYTHONNOUSERSITE=1,
1GiB address-space/30sCPU/60s outer bounds. Report writing used256MiB/15s. No timing or
performance inference follows. Output is confined to this external review directory.

## Git Traceability

Branch camonly/dev; Base SHA aa047269fdbeee4e65c8e99c6b7f29e817ee0b89;
New Commit SHA NONE; Changed Files reviewer external audit files only; Tests real preflight
and positive/rejection input probes PASS, apply/failure-injection NOT_RUN; Working Tree clean
at review with no reviewer tracked mutations; Remote Push NO; Main Merge NO;
Raw/Checkpoint Integrity all19 main originals match existing canonical bindings and were
opened read-only, all19 camonly destinations absent at review, checkpoint deserialization NO.
