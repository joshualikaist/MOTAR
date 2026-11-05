# Dev08 preparation — 2026-11-05

Scope: handoff and candidate-gain configuration preparation, not a measurement. Protocol:
[`prereg_sitl_dev08_candidate_gains_v1.md`](../../../plans/prereg_sitl_dev08_candidate_gains_v1.md).
Beads: completed-scope issue `motar-mx0`; runtime/input follow-up `motar-q8j`;
historical local-asset availability `motar-lp5.6`.

The additive [`tool`](../../../../tools/sitl_s3b_dev08.py) defaults to native configuration;
`--arm candidate` changes only MPC_XY_VEL_P_ACC to2.0, MC_YAWRATE_P to0.2 and MC_YAWRATE_I to0.1.
It parses pinned historical source/reports as data, never imports the runner or checkpoint,
and supports no execution. `runtime_status=NOT_IMPLEMENTED`, `measurement_status=NOT_RUN`.
Frozen F, the historical runner/bridge, PX4 changes and all historical raw/results are preserved.

## Input finding and scope limits

[`input_inventory.json`](input_inventory.json) binds four raw trace cells and source/platform
assets; it is copied byte-for-byte from the read-only external final inventory. The selected
70bars/env5/episode0 zero-action trajectory has208 frames because its ledger reports capture
at208. Dev07 captured at35 before exhaustion, so its historical outcome is preserved.
The strict600-frame requirement blocks the proposed unchanged20-member cohort. Do not pad,
drop, substitute or regenerate under the present preparation authorization.

[`candidate_configuration.json`](candidate_configuration.json) is deterministic static output
from the new tool. Its `trace_payload_verified=false` means the tool itself reads only source
and committed reports; the separate independent raw audit checks payloads. All selected
full/zero target positions agree on their common frames; G8's broader
TARGET_DEPENDS_ON_PURSUER result remains. S3a is NOT_EQUIVALENT and dev07 is development only.
The live SDF/airframe match dev07, while the jinja lacks the rangefinder addition; no assets
were regenerated or edited.

## Validation

[`validation.json`](validation.json) binds commands, logs and actual outcomes. The new focused
CPU tests pass, including positive controls, four historical hash-drift rejections, native
default/copy behavior, the three-gain delta, unchanged10Hz/ODE100, explicit input blocker,
no runtime execution CLI, and no shared environment/import-path mutation.

The shared PUBLIC_CPU run is **FAIL**, not waived:3285 tests,2 failures,4 errors,17 skipped,
5 excluded. Two errors were an absent `/tmp/codex_occ` fixture parent; after creating only that
empty directory, both controls pass. The other four tests reproduce2 failures/2 errors caused
by unavailable historical stopcap log, braking result and ref5in P1c checkpoint paths. Their
tests/analysis source match base1fe9afa. No raw/checkpoint was fabricated, restored or replaced,
and no guard was weakened. The availability issue remains open. Host faults were0→0.

Full-suite logs and baseline/source-availability proof remain in
`/home/fair/motar_tmp/dev08_preparation_20261105/`, with SHA-256 bindings in validation.json.
The full suite included the new test module before the final explicit runtime-status metadata
and plan wording repair; focused checks validate the final source afterwards. The shared
suite is not represented as green. Preparation review is recorded in the numbered report
alongside this record and applies only to this static preparation scope.

## Continue

First open `motar-q8j` and the protocol's input-blocker/runtime sections. Resolve the208-frame
input design without altering original evidence, then prepare and review an additive runtime.
The resulting concrete execution proposal needs newly scoped dev08 approval. No SITL/GPU,
later gate, push, main merge or real-flight operation is authorized by this preparation.
