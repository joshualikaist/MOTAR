# Dev08 observed-prefix input/runtime design — 2026-11-05

Protocol: [amendment1](../../../../plans/prereg_sitl_dev08_candidate_gains_v1_amendment1_2026-11-05.md).
This user-authorized unit progresses the related issue/design and prepares the next implementation
stack. It introduces no simulator/GPU measurement or later gate. The original dev08 strict600
plan and its BLOCKED_INCOMPLETE_TARGET_TRACE state remain unchanged.

The offline [contract tool](../../../../../tools/sitl_dev08_prefix_contract.py) verifies the
34 pinned input/source/asset files, actual selected raw frames, finite contiguous prefixes,
seed/density/F identity and full/zero common target positions. It defaults to strict600;
observed-prefix-v1 requires explicit selection. [prefix_preflight.json](prefix_preflight.json)
is actual read-only input verification, not a rollout. Live runtime is NOT_IMPLEMENTED,
measurement NOT_RUN, execution_supported/launch_ready=false. Layout geometry/live timing
qualification remains outside this preflight.

The208-position path supports checks0..207 (nominal20.7s); the other nineteen support0..599
(59.9s). A true post-action600 endpoint would need frame600, hence601 positions. The parent
report's original fields and dev07 results are preserved. All historical physical terminal
checks lie within these prefixes.

Pure contracts implement capture-before-crash-before-censor precedence, no command at a
terminal/final checkpoint, exact fixed40 main ledger membership (two arms×20 scenes), fixed
seed/episode/window identities, operational/time/count sanity checks, and observed terminal
counts/paired summaries with full-denominator unknown bounds. CENSORED_TRACE_END is neither
timeout nor a harness failure. The ledger tests are synthetic controls, not measured candidate
outcomes; no candidate result exists. Status labels/scalar endpoints do not establish live
per-step freshness or guard operation.

The design proposes two native smoke episodes plus20 fresh native/candidate pairs (42 total),
subject to completed reviewed runtime and a new scoped execution approval. The new launcher
must own its processes and per-episode external rootfs. Existing PX4 sitl_run.sh has a global
pkill and shared build/rootfs, so replacing Python cleanup alone is insufficient. Installed
build/etc rcS/airframe and parameter-state provenance must be bound too; no installed asset,
historical launcher or rangefinder-mismatched jinja was changed.

[validation.json](validation.json) binds the source/design probe, actual preflight, CPU checks
and independent review. Global PUBLIC_CPU availability limitations remain explicit; missing
historical raw/checkpoints are not reconstructed or substituted. Persistent external audit
root: `/home/fair/motar_tmp/dev08_runtime_design_20261105/`.

Next implementation sequence: isolated owned session/launcher and packet/parameter boundary;
native nonterminal command-prefix identity; attempt/source/host/resource integration; reviewed
bounded execution proposal. The amendment fixes the interfaces, stop rules and remaining
S3a/formal-S3b/S3c/S4/S5 qualification sequence. Task state and follow-ups live in Beads.
