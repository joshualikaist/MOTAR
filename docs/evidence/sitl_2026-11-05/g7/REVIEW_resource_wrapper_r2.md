# Resource wrapper source review r2

Reviewer: Codex `/root/g7_guard_review`; model GPT-6 (Codex; exact serving variant not exposed). Read current external wrapper and supplied cap_validation_r2.json only; no tests or GPU launched.

Verdict: **HOLD — one normal-exit cleanup race remains.** The r1 frozen-set/ancestry defects are substantially repaired: PR_SET_CHILD_SUBREAPER is enabled before Popen, discovery follows the wrapper's own descendants, and cleanup refreshes ownership repeatedly through TERM/KILL while checking PID start times. Probe source_sha256 matches actual wrapper bytes. Supplied probes show the intended outcomes and zero live descendants, including a late-fork TERM handler.

Remaining blocking finding (medium): the snapshot at lines 70–75 precedes child.poll at line 85. A child can fork a descendant and exit between these operations. The old snapshot has no descendant, poll sees successful exit, `reason` stays None, and the finally condition at line 96 skips cleanup because the child has exited. The return can record `live_descendants_after_cleanup > 0` while CLI line 133 still exits zero, since it does not test that count. This violates the wrapper's fail-closed descendant guarantee. Always perform a fresh ownership/cleanup check when the main child exits (ideally cleanup unconditionally in finally), and force a non-success reason/exit whenever any live descendants remain. Validate a normal parent that spawns a child and exits successfully, rather than only timeout-driven cleanup.

Non-blocking limits retained:

- Single-process memory-negative still uses 2 MiB, below ordinary Python baseline, and descendant-negative uses an allocation whose child alone can exceed the 24 MiB aggregate limit. They demonstrate rejection but not the same-threshold valid case or a sum-only threshold crossing; a same-limit positive plus two individually-under-limit processes whose sum exceeds the limit would improve evidence.
- CLI catches Exception, not KeyboardInterrupt/SystemExit; run's BaseException path cleans up but can omit a receipt for those exceptions. External SIGTERM/SIGKILL of the wrapper is outside its current graceful-shutdown handling. Keep operational launcher cancellation behavior explicit.
- A 0.25-second sampled RSS watchdog permits transient overshoot; shared pages are counted separately. It remains an operational bound, not a kernel hard cap. Dead adopted grandchildren absent from `known` may remain zombies until wrapper exit; owned() correctly excludes them from live compute.

Bindings:

- run_capped.py SHA-256: `1133b8b591700dde6dd9ccbfd19a0a9d0d840d2933724a2c092d4ab2ef21cbb0`
- cap_validation_r2.json SHA-256: `81b93e3fd434517c820ee7f13290b2ea19a3000833afd691368f502ec11e97f6`

Git Traceability: repository unchanged; no commit; tests NOT_RUN by reviewer; push NO; merge NO; raw/checkpoint bytes untouched. Only this external report created.
