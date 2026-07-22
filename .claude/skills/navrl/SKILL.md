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

### 4. Record + surface (direct, small)
Append a dated WORKLOG entry (numbers + decision), save the curve to `results/<name>.csv`, and if a
figure/status page exists, redeploy the Artifact with the new point. Commit docs/results only after
the user reviews the diff (their standing preference — never autonomous commit/push).

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
