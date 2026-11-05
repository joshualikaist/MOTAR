# Resource wrapper source review r3

Reviewer: Codex `/root/g7_guard_review`; model GPT-6 (Codex; exact serving variant not exposed). Read the latest external wrapper and cap_validation_r3.json. No tests, GPU, repository edits, or Beads actions performed by reviewer.

Verdict: **PASS_WITH_LIMITS for the standalone, sampled aggregate-RSS/time watchdog.** The blocking findings from r1 and r2 are repaired in these bytes. This is an operational bound, not a kernel memory hard cap, and is not scientific-results evidence.

The finalizer now always refreshes owned descendants, marks unexpected surviving descendants as LINGERING_DESCENDANTS, and attempts cleanup. A final fresh ownership count forces CLEANUP_FAILED when nonzero; CLI success additionally requires child exit 0, no stop reason, and zero live descendants. Thus the r2 success-with-live-child path no longer succeeds. Subreaper adoption and repeated refreshed TERM/KILL discovery cover the earlier frozen-PID and orphaned-subtree problems. PID start-time checks protect each signal attempt against ordinary PID reuse. Launch/ordinary-monitor exceptions produce a non-success external receipt.

The supplied source hash matches current bytes. All six supplied probes have nonzero samples and zero live descendants afterward. The valid control completes with exit 0 and no stop reason; memory and elapsed-time controls reject; descendant RSS rejection observes two processes; late-fork cleanup rejects despite main child exit 0; successful-parent-with-child records LINGERING_DESCENDANTS despite parent exit 0 and cleans the child. The CLI condition makes both latter cases unsuccessful wrapper executions. These are author-supplied probe records inspected by reviewer, not tests rerun independently.

Limits retained:

- Declared 0.25-second polling can overshoot the 12 GiB threshold and miss transient RSS peaks. Shared RSS pages are counted per process. The watchdog itself must remain alive and scheduled.
- This review assumes the declared standalone wrapper process has only this job's children. owned() deliberately includes all its descendants, so it is not appropriate to import run() into a host process managing unrelated jobs.
- Supplied memory probes establish rejection but do not include a same-threshold allocation positive/negative pair or a sum-only crossing where each process individually remains below the limit. The source explicitly sums all discovered live RSS; stronger threshold controls would improve probe coverage without changing this source verdict.
- External SIGTERM/SIGKILL of the wrapper has no graceful handler here. KeyboardInterrupt/SystemExit clean up through run's finally but do not reach the CLI Exception receipt handler. Cancellation/forced wrapper termination therefore remains outside the validated receipt guarantee.
- processes_observed counts PIDs seen during monitoring, including zombies or earlier adopted children when probes share a process; it is not a count of live worker processes. This does not change the RSS calculation, which excludes zombies.

Bindings:

- run_capped.py SHA-256: `75e18306bb105a30f78dc9b67fe1f6db7c6e2df90896d08409ffb940afddcdf0`
- cap_validation_r3.json SHA-256: `a80a98f5e821f45e51c0c35affd8d1e294a3be604fd58741b44889331be1b2ee`

Git Traceability: repository unchanged; no commit; tests NOT_RUN by reviewer; push NO; merge NO; raw/checkpoint bytes untouched. Only this external report created. Prior HOLD reviews remain valid records of the earlier source versions.
