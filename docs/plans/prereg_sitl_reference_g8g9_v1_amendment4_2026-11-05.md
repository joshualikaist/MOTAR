# Amendment 4: enforce the resource bound before the G9 confirmation and G7 (2026-11-05)

Operational correction within the user's instruction to finish through G7. Measurement code, controller,
commands, metric definitions, four-env scene and all G7 scientific gates stay unchanged.

## Observed deviation, retained

G9 attempt 3 completed with one actual bar, four finite responses, zero measured contacts, and no host faults.
Its user-systemd scope accepted `MemoryMax=12G`, but inspection showed this cgroup-v1 host does not delegate
the memory controller to that scope. A displayed systemd property did not establish an enforced limit. Do not
claim that run met the preregistered memory bound. Preserve its JSON, log and launch records as operational-
deviation evidence; do not rewrite it or silently treat the property as a kernel cap.

## Fix, before the next measurements

- Use the unchanged committed `sitl_g9_lee_steps_v2.py` in a new **attempt 4** output:
  `results/sitl_ref_v1_r2/g9_attempt4/g9_lee_steps.json`.
- Launch G9 and then G7 through `~/motar_tmp/g7_20261105/run_capped.py`. It monitors aggregate resident memory
  of the child process tree, including descendants which start separate sessions, and stops the observed tree
  if RSS exceeds **12 GiB** or elapsed time exceeds its declared bound. Record every observation's peak summary,
  interval, process count, command, exit and any stop reason in an exclusive report.
- The interval is **0.25 s**. This is a process-tree RSS watchdog, not a kernel hard memory cap. Shared pages
  count in each process, so its aggregate is conservative; short transients between observations are unmeasured.
  The cgroup settings are not used as evidence of enforcement.
- Keep the existing affinity, no other CUDA job, clean approved HEAD, worktree import-origin guard, exclusive
  outputs, and before/after host-fault/boot/topology checks. G9 is bounded to 1,200 s (plus outer cleanup);
  G7 retains its existing budget and per-cell limits, with an outer 7,000 s bound. No tracked edits while running.
- CPU checks of the watchdog must accept a normal bounded child and reject real RSS-over-limit, descendant-
  RSS-over-limit and timeout controls. Record independent source review and exact wrapper bytes with evidence.

The guard changes no actions, RNG, simulator or outcome logic. If it stops a run, preserve that attempt and do
not claim completion. Host faults still void the run and stop the queue. Reported G9 diagnostics and the fixed
G7 identity/sidecar/analysis rules remain exactly as registered. No new PPO training or checkpoint substitution.
