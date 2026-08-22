---
name: navrl
description: >-
  Run one NavRL sensor-only research iteration end to end — pick a training config, launch it,
  wait, evaluate the density curve, diagnose, and record — DELEGATING the heavy steps (CSV
  parsing, eval sweeps, code/log investigation) to subagents so the main context (and token
  spend) stays small. Invoke when the user wants to run the next cycle, "다음 학습", "iterate",
  "run the sweep", "분석해줘", or asks how the training is going.
---

# NavRL sensor-only research loop

You are driving the MOTAR / NavRL "Phase 3 vision pivot": a **sensor-only** UAV interception policy
(no ground-truth target in the actor observation) trained with a **density curriculum**. This skill
is the repeatable loop. **Token rule: you (the main loop) orchestrate and record; you DELEGATE every
step that reads many files, parses CSVs, or scans logs to a subagent via the `Agent` tool, and keep
only its short structured report.** Never read `runs/**/epoch_metrics.csv`, `nn/*.pth`, or long
training logs into the main context — that is what subagents are for.

Working dir for all training/eval: `aerial_gym/rl_training/rl_games`. Env: conda `aerialgym`, always
`PYTHONNOUSERSITE=1`. Sensor-only mode = `NAVRL_VISION=1`.

## TWO DISCIPLINES, ONE NAME — know which one you are paying for

`docs/discipline_review_2026-08-22.md` found that "preregistration" here means two different things
with very different track records. Keep them apart; they cost differently and fail differently.

**(A) Claim protection** — freeze the gate, the metric, the seed, and the verdict rule BEFORE
measuring, so the answer cannot be steered afterwards. This is cheap. It should be a short block:
question, arms, primary metric, thresholds, what would falsify it, and what the result does NOT
authorise. It does not need 150 lines of prose. Write it, freeze it, commit it before the machinery
that runs it exists.

**(B) Machinery integrity** — receipts, source manifests, checkpoint SHA pinning, runtime-clean
gates, import-origin enforcement, fail-closed guards, equivalence proofs. This is expensive and it
is what has actually saved this project: all five VOIDed runs (`governor_off`, `dirty_runtime`,
`export_guard_bug`, `source_drift`, `guard`) were caught by (B), not by (A). With 2,000+ episodes a
cell, noise is not the adversary — silent machinery failure is. **Do not economise here.**

The failure mode to avoid is paying (B)-sized ceremony for an (A)-sized decision.

## CHEAP CALCULATION BEFORE EXPENSIVE MACHINERY

Before preregistering an experiment, write down: **can this result be predicted by calculation, and
if so what is the prediction?** Put the answer in the preregistration.

- If calculation gives a confident prediction, ask whether the experiment is still the highest-value
  next step. Often a 30-minute derivation replaces hours of GPU and days of tooling.
- If you run it anyway, a matching result confirms the model and a mismatch is a real discovery.
  Either way you learn more than from an unanchored measurement.

This is not hypothetical. On 2026-08-22 the three most valuable findings of the day — that 28 m
detection is geometrically impossible in-sim, that `min_target_pixels` is an area rather than a
diameter, and that the 20 m clip rather than the pixel threshold is what binds — all came from
reading code and doing arithmetic with the GPU idle. The last one was predictable before the
experiment that measured it.

## PROHIBITIONS EXPIRE — every one carries a reason and a review trigger

When you forbid something, write **why** and **what would make it reviewable again**. A prohibition
whose cause has been fixed and which nobody has re-examined is not discipline, it is sediment.

The live example: post-hoc riskcap tuning was banned because the speed-governor data was
contaminated. The contamination was found and fixed long ago. Nobody asked whether the ban still
applied, and as a consequence `NAVRL_MAX_VELOCITY` (2.5), `NAVRL_MAX_TILT_DEG` (45) and
`NAVRL_YAW_RATE_MAX` (2.5) have **never once been ablated** — zero experiments, and `3.5355` is not
an optimum but the geometric consequence of `2.5 x sqrt(2)`.

Distinguish two things that look alike:
- **Forbidden** — doing it would corrupt a claim or destroy provenance. Example: editing
  `aerial_gym/config/robot_config/**` or `resources/robots/**`, which are byte-frozen against every
  existing checkpoint.
- **Expensive and unexplored** — legitimate, just costly. Example: the speed/tilt/yaw ceilings.
  `max_velocity` is the observation-normalisation denominator, so changing it breaks the checkpoint
  contract and forces a retrain. That is a price, not a prohibition. Say so, and price it.

Never let the second quietly become the first.

## PLAN SYNC RULE — non-negotiable, and the one most often forgotten

**When the plan changes, update the planning documents in the same commit as the work that changed
it.** A WORKLOG entry records what happened; it does NOT tell the next session what to do. That is
`VERIFICATION.md`'s job (execution authority: gates, verdicts, the next experiment) and
`RESEARCH_PLAN.md`'s job (charter: hypotheses and method).

The failure this prevents is specific and has happened: WORKLOG grows six honest entries in a day
while `VERIFICATION.md` still carries a stale `기준일` and still names an experiment that the day's
findings superseded. The next session reads the authority document, believes it, and runs the wrong
thing. WORKLOG being correct does not save you — nobody reads 9,000 lines to find out that the plan
moved.

Update `VERIFICATION.md` whenever ANY of these change:
- the next experiment, or the reason for it
- a gate, threshold, or the conditions that unblock a blocked stage
- what the current bottleneck is believed to be
- a prerequisite being satisfied or invalidated
- the `기준일` — bump it every time you touch the file

Update `RESEARCH_PLAN.md` when the hypothesis or the method changes, not when a number lands.

**What you may NOT do while syncing**: change a recorded verdict. P2 / D1 / P3 status lines and any
frozen result's judgement are historical facts. If new evidence reinterprets an old result, record
the reinterpretation as a LIMITATION next to it — never by editing the verdict. A plan sync that
quietly softens a FAIL is the worst possible outcome of this rule.

Self-check before you end a session: `git log --oneline -5` and ask, for each commit, "would a fresh
session reading only VERIFICATION.md do the right next thing?" If not, the sync is missing.

## WORKLOG RULE — non-negotiable

**Every piece of work ends with a `WORKLOG.md` entry. No exceptions, no "I'll add it later".**

This applies to *each* of these, not just a full loop iteration:
- a training run (launched, finished, died, or was killed — record which, and why)
- an eval / sweep (record the actual numbers, not "it improved")
- a code change (what changed, what it was measured against)
- a diagnosis, **including one that turned out wrong** — a refuted hypothesis is a result and stops
  the next session from re-running it

Entry format (newest at the BOTTOM of `WORKLOG.md`):
- dated `## YYYY-MM-DD — <one-line headline>`
- the measured numbers, in a table when there is more than one cell
- the decision it led to, and the next concrete step
- run folder / checkpoint / `results/*.csv` paths so the numbers can be re-derived
- when a claim was **disproved**, say so explicitly ("hypothesis X refuted: measured Y")

Write the entry **before** asking the user to review a diff, so the docs and the code land in the
same commit. If a session is running out of budget, the WORKLOG entry is the LAST thing to cut —
it is what makes the next session cheap. `CRASH_TUNING_LOG.md` gets the long-form mechanism writeup;
`WORKLOG.md` always gets at least the dated summary.

## Hard-won rules (do not relearn these)

- **Evaluate a curriculum run with `last_gen_ppo_ep_XXXX.pth`, NOT `gen_ppo.pth`.** `gen_ppo.pth`
  is the *best-reward* checkpoint, and in a density curriculum reward is highest at LOW density, so
  `gen_ppo.pth` is a sparse-density policy — evaluating it at high density gives garbage (~15 %).
  The `last_*` checkpoint is the actual high-density policy and generalizes DOWN well.
- **Warm-start works now**: `--checkpoint <path> --max_epochs <N>` resumes and extends. `runner.py`
  auto-strips the torch.compile `_orig_mod.` prefix from the critic and honors `--max_epochs`.
- **1650 Ti (4 GB, N=128) trains ~10-15 pt WEAKER than the 3070 (N=256)** — never mix their training
  numbers in one curve. Use the 1650 Ti for EVAL sweeps (eval is N-independent), not paper numbers.
- **Curriculum knobs are all env vars** (defaults were tuned for the old GT-LiDAR task):
  density `NAVRL_DENSITY_CURRICULUM/_START/_FINAL/_STEP/_THRESHOLD/_WARMUP/_CHECK_EPS`,
  goal-distance `NAVRL_K_FINAL/_MIN_FINAL/_K_WARMUP`, plus `NAVRL_MAX_BARS/_NUM_BARS`.
  Sensor-only wants `NAVRL_DENSITY_THRESHOLD=0.6` (not 0.8) or it stalls at high density.

## The loop

### 0. Status (direct, tiny)
Check GPU + newest run: `nvidia-smi --query-gpu=memory.used --format=csv,noheader`,
`ls -dt runs/*/ | head -3`, and the current WORKLOG tail. If a run is training, don't launch another
(8 GB fits one N=256 run). Report where things stand in ~3 lines.

### 1. Choose + launch training (direct)
Pick from these templates and state which and why:
- Density curriculum (main, one command = goal-ramp-then-density via warmup):
  `NAVRL_VISION=1 NAVRL_DENSITY_CURRICULUM=1 NAVRL_DENSITY_WARMUP=6000 NAVRL_DENSITY_THRESHOLD=0.6 ./train_navrl.sh --seed 1 --max_epochs 12000`
- Fixed density (baseline / anchor): `NAVRL_VISION=1 NAVRL_NUM_BARS=<D> NAVRL_MAX_BARS=<D> ./train_navrl.sh --seed <s>`
- Moving target (RQ2, the heatmap's speed axis): add `NAVRL_TARGET_SPEED_FINAL=1.5`.
- 4 GB machine: add `GPU4GB=1` (auto N=128, base_sim_4gb).
Launch in the background (nohup / `run_in_background`), note the pid + run folder.

### 2. Wait (Monitor, not polling)
Arm a `Monitor` or a background `until` loop that fires once on completion / error — do NOT tail
the log yourself. Widen the grep to catch `Traceback|Error|Killed|MAX EPOCHS|density curriculum`.

### 3. Evaluate + analyze — **DELEGATE to a subagent**
Spawn one `Agent` (general-purpose) with a prompt like: *"Read-only. Eval the run
runs/<name> at densities 25/50/75/110/130/150 using its `last_gen_ppo_ep_*.pth` via
`NAVRL_VISION=1 NAVRL_NUM_BARS=<D> NAVRL_MAX_BARS=150 NUM_ENVS=256 HEADLESS=True
PLAY_GAMES_NUM=2500 ./play_navrl.sh <last_ckpt>` (grep 'NavRL progress' for captured/crash/timeout).
ALSO parse aerial_run/epoch_metrics.csv for crash-vs-density and any promotion sawtooth. Return a
compact table + one-paragraph diagnosis + next-config suggestion. Cite file:line."* Keep only its
report; do not pull the raw numbers into your context.

### 4. Record + surface (direct, small) — MANDATORY, see the WORKLOG RULE above
Append the dated WORKLOG entry (numbers + decision + paths), save the curve to `results/<name>.csv`,
and if a figure/status page exists, redeploy the Artifact with the new point. Commit docs/results
only after the user reviews the diff (their standing preference — never autonomous commit/push).
Do not report a step as finished until its WORKLOG entry exists.

## Running subagents well (token-efficient)
- **Delegate reads, keep conclusions.** One agent that returns a 15-line report replaces hundreds of
  lines of CSV/log in your context. Launch several *independent* agents in ONE message so they run in
  parallel; wait for the completion notifications rather than polling.
- **Give each agent a tight, self-contained prompt**: exact paths, exact commands, "read-only", and
  the precise structured output you want back. Vague prompts waste tokens.
- **Never** read a subagent's `*.output` transcript from the shell — it's the full JSONL and will
  overflow context. Use its completion report only.
- For a heavier, deterministic multi-phase sweep (many densities × seeds × verify), the user can opt
  into the `Workflow` tool ("use a workflow" / "ultracode") instead of hand-spawning agents.
- Continue an existing agent with `SendMessage` (keeps its context) instead of spawning a fresh one
  when you're iterating on the same investigation.
