---
name: session-handoff
description: >-
  Close out every five-hour agent session before the account deadline and leave an explicit on-disk continuation
  for the next account. Applies to Codex, Claude Code, Cursor and delegated workers, without another reminder.
---

# Session handoff — all agents, shared deadline, durable continuation

Use AGENTS.md **Session deadline and handoff** as the canonical policy. The user's standing instruction is to
finish and hand off before the five-hour session expires, so the next account can immediately continue.

## Begin and check

1. Read the latest WORKLOG/Beads and `~/motar_tmp/agent_sessions/latest_handoff.json`, verifying the worktree.
2. Start/reuse `python3 tools/agent_session.py start --agent <agent>`. Supply the actual visible/user deadline
   with `--deadline <ISO8601-with-timezone>` and any known earlier start with `--started-at`.
3. Check `status` at turn/unit boundaries and at least every10min while active. Use `check --minutes <work>
   --verification-minutes <tests-review-records>` before a new unit. A clock check does not authorize a run.
4. Existing clocks survive restarts and compaction. Parent/worker agents share the root deadline. Use a new
   window only when the old session is handed off and a new account/window is established; do not infer one
   from a reconnect or a long-lived chat's creation date. Unknown quota timing stays unknown.

## Closeout budget

Reserve deadline−60min onward for closeout: finish only an in-flight authorized unit that fits with its tests,
review, records and local commit. No new implementation/research, long evaluation or expensive review starts
there. By deadline−10min, leave the handoff and stop. Visible imminent quota exhaustion advances this schedule.
The helper is an on-demand clock, not a background alarm, automatic process killer or quota API.
Changing worktrees does not reset the root clock: use `--worktree <original-root-worktree>` for its checks, and
record all affected worktree/branch/HEAD values in the handoff (the automatic Git snapshot covers the root only).

## Preserve actual state

- Verified authorized work is committed locally in named logical units; push/merge still need explicit approval.
- Keep WORKLOG and Beads current when editable. During GPU/guarded-run freeze, do not edit any tracked file,
  even docs or new tools: record only outside the repositories and name that path in the handoff.
- Do not automatically revert, reset, delete, or kill a job to make the ending look clean. Preserve incomplete
  paths, failures and evidence. Owned jobs continuing under existing approval require PID/owner, command,
  log/output, cap, actual state and exact safe next action; do not stop another agent's process.
- All scoped work is labelled verified, incomplete or not run as observed. Session time ending is not proof the
  objective is complete. Secrets, account tokens and provider-only memory never serve as continuation storage.
- No Markdown todo list or tracked HANDOFF.md: Beads tracks tasks; WORKLOG and the shared external handoff carry
  the continuation. Agent-session metadata is not a second project task tracker.

## Publish the handoff

Write an external JSON with `completed`, `remaining`, `running_processes`, `pending_approvals`, `validation`
(lists) and one concrete `next_action` (string: exact file/command and required gate). Then:

```bash
python3 tools/agent_session.py close --handoff /absolute/path/summary.json
```

The helper adds worktree/branch/base/HEAD/dirty state and clock, preserves a session-specific handoff, and updates
both the per-worktree and account-neutral latest pointer. Record it in WORKLOG/Beads when the freeze ends.

Final user message: what finished and the fixed verdicts/limits; commits and uncommitted paths; any continuing
owned job; **one** resume action; pending approvals/contracts that the next account must preserve. Include Git
Traceability. Next account reads these disk records before beginning the recorded action.
