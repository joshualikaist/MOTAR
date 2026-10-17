# MOTAR Git Commit & Research Traceability Protocol

User directive adopted on 2026-10-17. Applies to all AI research agents, including Claude, Astra
and Codex, and to all subsequent authorized MOTAR work. The canonical entry point is
[`AGENTS.md`](../AGENTS.md); `CLAUDE.md` imports it. This policy supersedes older instructions to
wait for separate approval before each local commit of completed authorized work. It grants no
new implementation, GPU, training, push or merge authorization.

## Completion requires a local commit

When an approved research unit is complete and verified, record it in a local Git commit before
reporting completion. This includes simulation/dynamics, sensors/camera/rendering, RL/controller
implementation, experiment designs/preregistrations, CPU/GPU tests and validation reports, paper
hypotheses/analysis, environment qualification, WORKLOG and research progress records.

Keep independent units in separate commits where practical. The required sequence is:

1. Carry out the approved work.
2. Run the appropriate tests and verify the actual result and its evidence.
3. Review the actual Git diff against the unit's base commit, including new files.
4. Stage only approved files, naming each path explicitly.
5. Make a local commit.
6. Report the commit SHA, actual changed paths, verification and working-tree state.

Do not silently leave finished verified work uncommitted. If a conflict, failing check, hook,
missing identity or other condition prevents a commit, report the concrete reason and all
remaining uncommitted paths. Do not delete, reset or hide work to produce a clean status.
Incomplete or unverified work must not be reported as completed.

For GPU experiments, commit source and preregistration before measurement. Keep source HEAD and
all tracked files, including WORKLOG, docs and tests, unchanged during execution. Write only to
the designated new run output or scratch. After all owned experiment processes have ended,
validate and separately commit the result report. This protocol does not authorize a new GPU run.

## Read-only audits and evidence labels

Read-only means preserving existing execution source, historical results and raw data. An approved
audit may create new reports, analysis outputs, evidence documents and appended WORKLOG entries
and must locally commit those completed verified records. It must not modify historical evidence
to make the audit pass.

Use PASS, FAIL, NOT_TESTED and LIMITED_VALIDITY accurately for audit gates, with the measured
scope and limitation. Preserve original experiment verdicts such as INCONCLUSIVE or WITHDRAWN;
do not relabel them as audit PASS. A plausible but unverified explanation remains a hypothesis.

## Protect other work and frozen records

- Prefer an isolated short-lived worktree/research branch where practical. Record the branch and
  base SHA before editing. Main merge is separately authorized, never automatic cleanup.
- Do not modify, stage or commit user WIP, an existing stash, another agent's unfinished work,
  frozen Policy F or an unauthorized checkpoint, historical raw, original results or manifests.
- Stage explicit paths. Never use `git add .`, `git add -A` or a broad path that includes unrelated
  files. Review the staged diff and exact staged path list before committing.
- On conflict with existing user changes, stop and report. Do not force-reset, stash or overwrite
  the conflicting work. Preserve all existing path/hash contracts and preregistrations.

## Local history and GitHub access

A local commit is not visible through GitHub. Report whether a remote push actually completed;
provide a GitHub commit/branch URL only after a successful authorized push. If remote review is
needed, propose pushing the named dedicated research branch and wait for explicit authorization.
Main merge, GitHub push and branch/worktree deletion are not implied by completing a local unit.
If the user later authorizes automatic push of a particular branch, apply it only within that
stated scope. Preserve current remote state otherwise.

## Research data record

Do not indiscriminately add large raw outputs, checkpoints or GPU dumps to Git. Track compact
text evidence and copied manifests separately from external raw. Record:

- Run ID, source commit SHA, preregistration version and its committed path.
- Actual raw root, RAW_MANIFEST path and SHA-256 bindings, including checkpoint identity.
- Analysis result, tests, interpretation and material limitations.
- Historical execution command and a CPU reanalysis command where available.
- GPU wall time, hardware/runtime information actually recorded, and output preservation status.

Never invent an unrecorded environment detail. A symlink or a branch merge does not cause
untracked raw to enter Git. Do not stage secrets, keys, tokens, login details or personal data.

## Session limit and handoff

At the beginning of a long session, record its known start and deadline and a wrap-up time.
For a five-hour session reserve roughly the final hour for checks, records, local commits and
handoff. Check the clock after each research unit and before starting a long run. If the true
service quota deadline is not exposed, record that it is unknown; use a conservative stopping
point rather than claiming precise access to the service's remaining time.

Finish the largest authorized unit that fits before wrap-up, leaving enough time for its tests
and commit. Do not start a unit that cannot fit. Before the limit, report and persist:

- Completed units, tests, decisions, branch, base and resulting commit SHAs.
- Incomplete work, unresolved problems, pending approvals and exact uncommitted paths.
- Any owned process still running: PID/session, log/output, cap, status and safe continuation.
- One concrete first action for the next session, with a command/file and completion gate.

Never falsely report a running experiment as complete. Do not edit tracked files while its
freeze is active merely to write a handoff; use scratch and tell the user where it is. If no
process remains, append the verified handoff to WORKLOG and include it in the local commit.

The repository's [session-handoff skill](../.claude/skills/session-handoff/SKILL.md) supports this
workflow. Its older push, revert and concurrent-edit suggestions do not override this user
directive: no unauthorized push, no destruction of WIP, and no tracked edits during GPU runs.

## Required final block

End every work report with the following block, using actual values. List multiple commits and
their bases when several independent units were completed. A tracked-clean tree may still
contain preserved untracked raw/symlinks; disclose those paths.

**Git Traceability**

- Branch:
- Base SHA:
- New Commit SHA:
- Changed Files:
- Tests:
- Working Tree:
- Remote Push: YES / NO
- Main Merge: YES / NO
- Raw/Checkpoint Integrity:
