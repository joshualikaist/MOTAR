# R-P1a implementation and CPU verification — 2026-10-16

Status: **IMPLEMENTED; CPU verification only. GPU runtime validation NOT_TESTED.**
Base: `c038ba50193fb6401a473d8eb9ff4be097883e0e`, branch `codex/mt-p1`.
The user's subsequent “승인할게. 이어서 진행해.” approved the implementation and CPU scope
of [the proposal](../plans/r_p1a_observability_v1.md). That proposal and its unapplied design
draft remain unchanged as provenance; this report supersedes their pre-approval status.

## Actual changes

| File | Change |
| --- | --- |
| `motar_eval/recovery_events.py` | Immutable scalar frames and independent post-acquisition event intervals, expiry transitions, prefix hashes and censoring |
| `aerial_gym/task/navrl_task/kroc_eval_adapter.py` | Default-OFF flag, detached staging, consume-before-action, reset/terminal/shutdown handling, separate sidecars |
| `aerial_gym/task/navrl_task/recovery_visibility_probe.py` | Lazy actual scene-mesh centre-ray query with evaluator-owned buffers; unsupported geometry stays UNKNOWN |
| `tools/recovery_observability.py` | CPU trace replay, provenance checks, interval summaries and existing episode-outcome joins; no launcher |
| `tests/test_recovery_events.py` | Scalar timing, prior acquisition, independent causes, expiry, censoring, prefix and alignment contracts |
| `tests/test_recovery_observability_contract.py` | CPU adapter stand-ins, byte-identical existing outputs, input/RNG preservation, ledger joins and malformed-trace rejection |
| `tests/test_recovery_visibility_geometry.py` | Independent geometry fixtures and an actual Warp CPU mesh query |
| `WORKLOG.md` | Append-only implementation record |

The [actual review diff](../evidence/r_p1a_cpu_2026-10-16/implementation.diff) contains these
runtime/test changes, this report and the new WORKLOG entry. Earlier uncommitted audit/proposal
files and the untracked raw-results symlink are excluded from that patch.

## Measurement contract

`NAVRL_KROC2027_RECOVERY_EVENTS=1` requires the existing recorder flag and a fresh cell output
directory. It writes `recovery_events.jsonl` and `recovery_summary.json` separately from existing
KROC outputs. OFF imports no new probe or event core. ON records source-file hashes as well as
the actual source HEAD, checkpoint declaration, dynamics/model, distance backend and episode key.
Unknown or unsupported lineage cannot pass the CPU provenance validator as a valid direct lineage.

A recovery interval requires an earlier camera-valid, fused-valid, active track. Initial hidden
search is ineligible. Camera loss, fused loss, track inactivity/expiry, centre FOV exit, range
exit and mesh LOS are separate, potentially overlapping signals. The actual tracker age and
strict `age > memory_s` transition determine expiry; no evaluator timer changes tracker state.
Shutdown/reset censor intervals; capture/crash/timeout remain the task's existing outcomes.

Physical time is the count of completed physics actions times `step_dt`: the reset observation
is frame 0, consumed before its action. Terminal intervals close after the last completed action,
before reset. Sensor capture time remains unrecorded. A combined primary occlusion candidate
requires verified zero configured detection latency, zero latency steps and no empirical latency
injector. Otherwise current-render LOS is retained independently and the combined candidate is
UNKNOWN. This does not invent a hardware capture timestamp.

The probe checks the actual fixed-bar scene configuration, camera pose/FOV, mesh IDs and rendered
virtual target state. It never rerenders the detector. Physical/oriented/dynamic targets,
decoupled cameras or unsupported scenes produce UNKNOWN. Centre-ray blockage and visible pixel
counts do not establish full silhouette occlusion or the cause of a camera rejection.

## CPU evidence and preservation

Counts, commands, source hashes and checks are in [validation.json](../evidence/r_p1a_cpu_2026-10-16/validation.json),
with [scoped](../evidence/r_p1a_cpu_2026-10-16/scoped_cpu.log) and
[PUBLIC_CPU](../evidence/r_p1a_cpu_2026-10-16/public_cpu.log) logs. CUDA was hidden; no simulator
rollout or policy execution was performed. The full suite used the existing Matplotlib font cache
copied to `/tmp`; no assertions were changed. Stand-ins verify both stationary and moving event
chronology and 2/5/10/15-second arithmetic, not natural-occlusion feasibility.

Existing KROC episode/summary bytes match with recovery OFF versus ON in CPU fixtures. Python,
NumPy and torch RNG states, supplied task/actor/tracker tensors and frozen source files are
preserved. Actual GPU trajectory noninterference remains untested.

All 131 canary and 452 main-run manifest members retain their SHA-256 values. Policy F matches
the frozen checkpoint SHA. Main HEAD, branch HEAD, user WIP, stash, preregistrations, earlier audit
records and raw symlink are preserved; nothing is staged or committed. Existing backup archives
remain on the same filesystem: external backup is still incomplete. No deletion was performed.

## Remaining work and approval boundary

- GPU camera/mesh parity and matched ON/OFF trajectories: **NOT_TESTED**; require a new committed
  preregistration and explicit approval for the particular GPU run.
- Matched scene/pre-loss actor/action prefixes and natural 2/5/10/15-second occlusions:
  **NOT_TESTED**; R-P1b remains separate. Initial 4–12 m is still a feasibility candidate.
- Sensor capture timestamp, full silhouette occlusion, sampled dropout versus rejection,
  persistent physical target identity and an explicit safe-abort reason: **NOT_RECORDED**.
- No belief/prediction, scenario generator, Recovery Planner, PPO, commit, merge or push was
  included. Existing A/B results remain initial-view evidence, not long-occlusion recovery evidence.

The approved R-P1a implementation/CPU package is complete. Commit review and each subsequent
research package remain separate; this report does not authorize any GPU run.

## Final verification addendum (2026-10-16, after the interrupted session)

The text above is kept as written. This addendum records what the final check found.

- `validation.json` (20:33:59) predates the last source change. `tests/test_recovery_observability_contract.py`
  was edited at 20:34:31 to add `test_unsupported_motion_cannot_be_mislabeled_as_valid_direct_lineage`.
  That record therefore lists a stale SHA-256 for the test file (`2a933c13…`, now `f1c81b2f…`) and the
  earlier counts (scoped 37, PUBLIC_CPU 2,318). It is kept unchanged as history.
  [`validation_final.json`](../evidence/r_p1a_cpu_2026-10-16/validation_final.json) supersedes it.
- The final runs after the last source change both pass: scoped 38/38, and PUBLIC_CPU 2,319 run with 0 failures,
  0 errors, 10 skipped and 5 GPU-only exclusions (`scoped_cpu_final.log`, `public_cpu_final.log`). Re-running the
  same two recorded commands unchanged gave identical counts and the same scoped test list
  (`*_reverify.log`). The PUBLIC_CPU command depends on `/tmp/mt_p1_mpl_cache` (a copy of the host
  `fontlist-v330.json`, SHA-256 `0c77c752…`), which does not survive a reboot.
- The review diff `implementation.diff` named above did not exist when the session stopped. It was generated
  afterwards from the final tree, with the same scope as stated above.
- The table above lists a WORKLOG implementation record, but none had been written. It is appended now.
