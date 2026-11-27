---
name: session-handoff
description: >-
  Track observed account usage and leave durable continuation before exhaustion for Codex, Claude Code,
  Cursor and delegated workers. Never infer quota from elapsed wall-clock time.
---

# Session handoff — observed usage, shared continuation

Use AGENTS.md **Session usage and handoff** as the canonical policy. The user's 2026-11-11 clarification replaces
wall-clock five-hour scheduling with usage-based closeout. Preserve historical handoffs and existing approvals.

## Begin and check

1. Read the latest WORKLOG/Beads and `~/motar_tmp/agent_sessions/latest_handoff.json`, verifying the worktree.
2. Start/reuse `python3 tools/agent_session.py start --agent <agent>`. After handoff, `--new-session` creates a
   successor activity record while retaining quota observations. `--new-window` requires an explicitly confirmed
   new quota window; reconnects, new tasks and account logins do not prove a reset.
3. Record actual provider-visible or user-reported usage with `usage --remaining-percent <observed-percent>
   --source <source>`. Without a signal, keep `USAGE_UNKNOWN`. The helper cannot read hidden account quota.
4. Check `status` at turn/unit boundaries and before long calls. A last observation is a snapshot, not live usage;
   never treat an old reading as a fresh measurement. Known cost estimates may use `check --usage-percent <cost>
   --verification-percent <cost>`. Missing observations/estimates give UNKNOWN, never a budget PASS.
5. Do not convert elapsed time, token/context capacity, login changes or a five-hour countdown into remaining quota.
   A separately explicit current user wall-clock deadline may use `--deadline`; there is no automatic deadline.

## Closeout margin

The prior margins are retained as usage fractions: **20% remaining** starts WRAP_UP and **3⅓% remaining** stops
work for handoff. They are policy thresholds, not quota measurements or a claim about provider window size.
An observed closeout/exhaustion warning can trigger earlier. Unknown quota alone does not stop authorized work.
During WRAP_UP, start no long implementation/research unit, training/evaluation or expensive review; finish only
in-flight authorized work that fits with validation and records. Keep continuation durable throughout.
Workers and changed worktrees inherit the root usage state and limits; use `--worktree <original-root-worktree>`.
The helper records supplied observations; it is neither a quota API nor an alarm or process killer.

## Preserve actual state

- Verified authorized work is committed locally in named logical units; push/merge still need explicit approval.
- Keep WORKLOG and Beads current when editable. During GPU/guarded-run freeze, do not edit any tracked file,
  even docs or new tools: record only outside the repositories and name that path in the handoff.
- Do not automatically revert, reset, delete, or kill a job to make the ending look clean. Preserve incomplete
  paths, failures and evidence. Owned jobs continuing under existing approval require PID/owner, command,
  log/output, cap, actual state and exact safe next action; do not stop another agent's process.
- All scoped work is labelled verified, incomplete or not run as observed. Quota exhaustion is not proof the
  objective is complete. Secrets, account tokens and provider-only memory never serve as continuation storage.
- No Markdown todo list or tracked HANDOFF.md: Beads tracks tasks; WORKLOG and the shared external handoff carry
  the continuation. Agent-session metadata is not a second project task tracker.

## Publish the handoff

Write an external JSON with `completed`, `remaining`, `running_processes`, `pending_approvals`, `validation`
(lists) and one concrete `next_action` (string: exact file/command and required gate). Then:

```bash
python3 tools/agent_session.py close --handoff /absolute/path/summary.json
```

The helper adds worktree/branch/base/HEAD/dirty state and observed usage, preserves a session-specific handoff, and updates
both the per-worktree and account-neutral latest pointer. Record it in WORKLOG/Beads when the freeze ends.

Final user message: what finished and the fixed verdicts/limits; commits and uncommitted paths; any continuing
owned job; **one** resume action; pending approvals/contracts that the next account must preserve. Include Git
Traceability. Next account reads these disk records before beginning the recorded action.
