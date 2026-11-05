# Local historical-asset availability repair — 2026-11-05

Beads `motar-lp5.6`. This related-issue unit restores absent local bytes for existing tests;
it does not regenerate evidence, replace a policy/checkpoint, move original files or run an
experiment. The earlier full-suite failures remain in their original preparation records/logs.

Read-only audit found main originals matching every required historical canonical digest.
Only19 absent ignored destinations were created,54,147,345bytes in total:

- Eight manifest-bound receipt/source-manifest/summary files under three physical-target
  result directories.
- Five stopcap logs and five receipt-bound Frozen-F checkpoint snapshots.
- The exact historical P1c ep900 checkpoint, independently bound by its launcher and summary.

All were streamed to exclusively created regular files. No existing destination was
overwritten, no symlink/hardlink was introduced, and original main/mt-p1 files were preserved.
The copied checkpoints are existing historical identities, not newly trained/replacement
models. No checkpoint deserialization occurred in the audit/copy operation. Existing ignore
rules cover all19 destinations; copied raw/checkpoint bytes are not staged or committed.

[operation.json](operation.json) gives the exact relative destinations, byte counts/digests,
canonical binding/audit/source/receipt paths and SHA-256 values. The original manifest and
test/source files were not edited. Independent postoperation verification checks all19
source/destination pairs, separate regular inodes/nlink1, all453 canonical path assertions and
245 recorded hashes, with no drift.

Existing focused tests145/145 PASS. Full PUBLIC_CPU3298 PASS,0 failures/0 errors,10 skipped,
5 GPU exclusions; host faults0→0. Seven previously skipped availability-dependent cases now
run. The postcopy dry-run rejects the occupied destinations and leaves completed receipts and
files unchanged. Independent pre-operation and postoperation reports are retained alongside
this record. Full logs and the exclusive-copy script remain at
`/home/fair/motar_tmp/dev08_asset_availability_20261105/`, with committed digest bindings.

This resolves the local validation failure; it does not supply missing target states or
implement the live runner. Dev08 strict600 stays BLOCKED, observed-prefix measurement NOT_RUN,
runtime NOT_IMPLEMENTED. The next implementation unit is `motar-5md`: isolated owned session,
parameter/telemetry boundary, native command-prefix identity and guarded attempt integration,
followed by independently reviewed source and a newly scoped measurement decision.
