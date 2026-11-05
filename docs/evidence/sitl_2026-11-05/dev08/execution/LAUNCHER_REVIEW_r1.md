# Dev08 external launcher: independent follow-up r1

Reviewer: Codex fresh-context delegated reviewer `/root/dev08_execution_source_review`.
Model: GPT-6 family per supplied Codex role; exact runtime variant **NOT_RECORDED**.
Date: 2026-11-05. Root clock retained, finish-by18:35:22 UTC; never reset/closed.
Source: `/home/fair/motar_tmp/dev08_execute_20261105/root/launch_authorized.py`.
Exact bytes are copied to `launch_authorized_snapshot.py` and SHA-bound in `probe_results.json`/`SHA256SUMS.json`.

## Verdict: PASS_WITH_LIMITS; no concrete launch blocker found

The inspected launcher matches amendment5's external-launch responsibilities:

- No launch occurs without exact argv `--launch`. Preparation checks approval's exact scope/float0.15, matching
  HEAD, completely clean `git status --porcelain`, and absent attempt output. A missing final approval refuses
  before any launch. The actual execution approval does not yet exist and must be written after the source
  commit with that exact reviewed clean HEAD.
- Argv uses timeout TERM5800/kill90, nice10, taskset6,7,12,13 and the named aerialgym Python `-B` CLI invocation.
  Environment hides CUDA, fixes PYTHONNOUSERSITE/OMP/MKL1 and worktree PYTHONPATH, and disables bytecode writes.
- The Popen preexec callback sets both soft/hard RLIMIT_AS to16GiB before the timeout exec. The ordinary exec/fork
  chain retains this cap for timeout/nice/taskset/Python and owned descendants. Existing12GiB sampled aggregate
  RSS/240s member/2400s density/5400s overall guards remain inside the already-reviewed runner/session stack.
- An exclusive pending state file plus exclusive once-only log and existing-state refusal prevent automatic
  repeat. STARTING is recorded before Popen, then RUNNING with timeout PID, `/proc` stat/limits (or explicit
  NOT_RECORDED if already exited). ENDED includes exit/fault/source state; success requires clean matching HEAD,
  unchanged fault count and child0. There is no retry path.

The parent reports focused136 PASS and fullPUBLIC_CPU3434 PASS in292.366s,10skips, faults0→0. Those are parent
validation, not repeated independent runs in this follow-up. The parent still owns the final commit/blob/approval
binding, actual idle-host preflight, launcher invocation and post-run review. Source review r1's limits remain.

## Probe and practical limits

`probe.py` compiles the actual launcher AST, replacing only four fixed filesystem-location assignments with new
audit fixture paths. It mocks subprocess/git/journal/Popen and resource.setrlimit; **no real Popen or CLI runs**.
Actual argv/environment/control flow are retained and checked. Positive default preparation creates no record;
wrong commit/bound/existing attempt refuse; one mocked launch records STARTING/RUNNING/ENDED, requests the exact
16GiB AS pair, records the missing-/proc fallback, and a second launch refuses without overwriting any record.
Source bytes are equal before/after. Probe command0; results in `probe_results.json`.

This mock validates requested arguments and branches, not a measured live AS limit. The actual `/proc` receipt
must confirm the inherited cap after launch. Three nonblocking operational limits are explicit:

- The launcher uses assertions for several refusal checks. Invoke it with ordinary non-optimized Python;
  `python -O`/PYTHONOPTIMIZE would remove those checks. The inner production CLI/recorder still have their own
  explicit guards. No optimized invocation was proposed or tested.
- Killing/interruption of the external supervisor, failure to read the final journal/Git, or an I/O failure can
  leave execution_state at STARTING/RUNNING even after the detached timeout process ends. Preserve and reconcile
  that state with the timeout PID/start identity, runner ledger/final and owned-session receipts; do not retry.
- Wrapper `faults()` accepts empty successful journal stdout as0. The inner recorder independently refuses an
  empty/unreadable kernel log; wrapper counts should not substitute for that runner validation.

Probe command (all reviewer CPU work is now ended):

```bash
ulimit -v 8388608
export CUDA_VISIBLE_DEVICES='' PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
timeout -k 5 30 nice -n 10 taskset -c 16-19 /home/fair/miniconda3/envs/aerialgym/bin/python -B \
  /home/fair/motar_tmp/dev08_execute_20261105/launcher_review_r1/probe.py
```

No PX4/Gazebo/SITL/GPU, actual CLI, real policy/checkpoint, live session, repository/Beads/Git mutation or remote
operation. New audit outputs only; previous bound source-review files remain unchanged.

Git Traceability: Branch camonly/dev; Base/observed HEAD cbf5527d1d42fe96b1a9b49b0e52afc72543c7ba; New Commit NONE;
Changed repository files NONE by reviewer (parent's submitted changes remain); Tests mocked launcher probe PASS;
Working Tree parent's ongoing source/record changes; Remote Push NO; Main Merge NO; Raw/Checkpoint Integrity
preserved, no writes/deserialization.
