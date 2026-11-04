# Independent review: GR-0, Policy F under randomized controller gains

- **Reviewer:** Claude Opus 5.5 (`claude-opus-5-5`), fresh context, independent of the author. 2026-11-04, 13:26–13:45 KST.
- **Scope:** the code at commit `c56be07` (`git diff 62e59c0 c56be07`) and the helpers it reuses, the preregistration
  and amendment 1, D-2026-11-04-1, and the raw run files bound by `SHA256_MANIFEST.json`. The author's WORKLOG,
  summaries and evidence folders were not used. The author's `analysis/gr0_crash_causes_exploratory.json` was opened
  only after my own crash-cause numbers were computed, to cross-check them (finding L5).
- **Host:** kernel fault count (`journalctl -k -b 0 --no-pager | grep -cE "Machine check|segfault|general protection"`)
  was **0 before** (13:26) and **0 after** (13:40). Online CPUs stayed `0-3,6-23` (cpu4-5 offline, as during the runs).
  No GPU process was started. Every probe ran on the CPU (`taskset -c 0-3,8-11,14-23`, `CUDA_VISIBLE_DEVICES=`,
  `PYTHONNOUSERSITE=1`, aerialgym python `-B`), in child processes with path-only package stubs. At most one probe ran
  at a time.
- **Writes:** only inside `review/audit/`. That includes `trees/`, a `git archive` export of the controller, config,
  utils and tests files at `c56be07` and `62e59c0`. The worktree, the main checkout's `results/` and every other path
  were only read.

## Verdict: **PASS**

The identity stage and the GR-0 readings stand as computed:
- **IDENTITY_PASS (identity_02):** OFF = the base tree in every record file, and scale 0 = OFF. s40 differs from OFF,
  starting with every environment's first episode.
- **S20 = GAIN_ROBUST:** Δcapture −0.62 pp, 95 % CI [−1.01, −0.26]; Δcrash +0.61 pp, CI [+0.25, +0.98].
- **S40 = GAIN_SENSITIVE:** Δcapture −3.69 pp, CI [−4.36, −3.03]; Δcrash +3.69 pp, CI [+3.03, +4.37].
- **Next step:** under the preregistered rule, GR-1 is motivated.

All 36 GR-0 cells and the 4 identity_02 cells are valid. I reproduced the analysis exactly from the ledgers. I found no
blocking finding: nothing changes a reading, invalidates the OFF identity, or shows that the gains did not vary as
preregistered. The two medium findings limit what the S40 label can be taken to mean (Q5). They do not change the
label.

## Findings by severity

### Blocking

None.

### Medium (they limit interpretation; no reading changes)

**M1. The drawn gains are not recorded per episode.** Neither `episodes.jsonl` nor `kroc2027_episodes.jsonl` nor
`result.json` holds the K_vel / K_rot / K_angvel values an episode flew with. The drawing code is
`base_lee_controller.py:114-134`; the runner's records are built at `gr_run.py:106-148`.
- The preregistration did not require this record, so it is not a deviation.
- My evidence that the drawn gains reached the GPU runs is therefore indirect:
  - the code path (Q1);
  - the CPU probes;
  - s00 = off exactly;
  - s40 differs from off in 128/128 first episodes, although their initial states are identical;
  - the dose response S00 → S20 → S40.
- GR-0 cannot say which element or which end of the range (−40 % or +40 %) drives the crashes. It cannot give a
  dose-response curve inside S40.
- The CUDA Philox stream also cannot be replayed on the CPU to reconstruct the gains.
- Recommendation: record the gains per episode in any GR-1 evaluation.

**M2. The gate reading hides a change in what kind of crash happens.** These numbers are exploratory, from
`result.json` `crash_causes` (the aggregate window of each cell, about the first 2,048 completions), not from the
quota rows. Probes: `audit/recompute_analysis.json`, `audit/explore_crashes.json`.
- Floor strikes ("below") rise about 18×: 0.10 (S00), 0.29 (S20), 1.82 (S40) per 100 episodes. The increase is roughly
  the same at every density (S40: 1.66 / 1.63 / 1.84 / 2.15 per 100 at 70 / 115 / 160 / 205 bars).
  - The cells' `crashdiag` lines put S40 floor strikes 8–16 steps into the episode, at 41–64° tilt. The commanded
    tilt limit is 45°.
- Bar contacts rise with density: +0.63, +1.59, +3.09, +2.68 per 100 episodes.
- At 70 bars, 42–50 % of S40 crashes are floor strikes, against 0–4 % at S00.
- S40 crashes come earlier: median 30 actions against 41 at S00. 467 S40 quota crashes end within 10 actions, against
  98 at S00.
- What this means for GR-1:
  - About half of the S40 crash increase is a low-level attitude and altitude failure.
  - The policy has only partial authority over it. Its z output is overwritten by the task's altitude PI
    (`condition.policy_z_output_overwritten_by_altitude_pi = True`, `navrl_task.py:9500`), and it does not observe the
    gains.
  - Retraining the policy (GR-1) may not remove this part. A change on the controller side is an alternative worth
    weighing.

### Low

**L1. The runner's per-cell check is weaker than the KROC2027 helper's** (`gr_run.py:106-148`).
- It does not check that the recorder covers every ledger episode, the motion gate, swept clearance, the raw-file
  SHA-256 bindings, or the cell contract (seed, density, view, arm label, protocol SHA).
- The "recorder violations empty" test (`gr_run.py:140-142`) never fires. No cell writes `kroc2027_violations.jsonl`.
  In the default `stop` mode a violation raises instead, as identity_01 shows. The cell then fails closed through the
  missing `result.json`, so the runner's net behaviour is still fail-closed.
- My stricter audit (`audit/audit_cells.py`) adds every missing check. It found **0 problems in all 40 cells**.

**L2. Amendment 1 reinterprets the stop rule without a recorded user approval.**
- The preregistration says IDENTITY_FAIL "stops the unit" (`prereg_gain_randomization_gr0_v1.md:71`). The amendment
  limits that rule to a completed comparison and allows one rerun (`…amendment1_2026-11-04.md:31-35`).
- I judge the amendment justified on the evidence:
  - All four identity_01 cells, including the base-tree reference `ref`, stopped at step 1 with the same swept-clearance
    violation: target clearance 0 in 3 of 128 environments, first violation in env 50 at (37.50, 18.40).
  - No comparison was made.
  - The amendment was committed at 11:54:28, before identity_02 started (11:54:37) and before any GR-0 cell.
  - It changes no level, seed, quota, analysis, threshold or rule.
- The user should still be told that this rule was reinterpreted by the author.

**L3. The gain seed equals the cell seed.** The runner passes the cell seed (`gr_run.py:81,102`), which is also the
simulator seed. The generator is built at `base_lee_controller.py:118-121`.
- Both streams are Philox on `cuda:0` with the same key.
- S20 and S40 in a block consume the same u stream (common random numbers), so they are not independent replications.
- I saw no harmful coupling: first-episode initial state and scene hashes are identical across S00 / S20 / S40 in
  36 × 128 environments, and s00 = off exactly.
- A fixed offset on the gain seed would remove the theoretical coupling.

**L4. The unit tests check only the CPU global RNG** (`tests/test_gain_randomization.py:59-65`). The CUDA generator
path is covered only empirically, by s00 = off over the whole identity run (456 rows) and by the matching
first-episode hashes.

**L5. The author's exploratory crash file sums shares across cells.** In `motar_tmp/.../analysis/gr0_crash_causes_exploratory.json`
the counts match mine exactly, but each `*_share` field is the sum of 12 per-cell shares, not a share. For example,
S00 `bar_contact_share` = 11.66, while 1076/1106 = 97.3 %. The file is not in the manifest and is not a gate input,
but it should not be quoted as is.

### Info

- **I1. Cosmetic issues:**
  - extra blank lines at `lee_controller_config_navrl.py:11-13`;
  - `identity_compare` drops `checkpoint`, which holds the checkpoint SHA, not the absolute path the preregistration
    mentions (`gr_run.py:196`); this is harmless, because the receipt SHA is checked;
  - the pre-existing upstream `reset_idx(None)` refers to a non-existent `self.K_rot_tensor`
    (`base_lee_controller.py:89`); no GR path calls it.
- **I2. The worktree has new staged work.** It now holds staged, uncommitted non-documentation files
  (`eval_navrl_gr1_arms.sh`, `train_navrl_gr1.sh`) beyond `c56be07`. These do not affect the reviewed runs: every
  receipt records `git_dirty_files = 0` at `c56be07`.
- **I3. The amendment's "B1-R Policy F condition" holds for the settings that affect the run.**
  - GR-0 S00 and B1-R `F_SHADOW` env snapshots differ only in quota, run id, seed, nonce and output path.
  - B1-R additionally set telemetry variables (occlusion trace, packets, recovery events).
  - S00 quota rates agree with B1-R F within ≤ 0.7 pp at every density. The seeds differ (4501–4503 against
    4301–4307), so this is context only.

## Answers

**Q1 — Switch.**
- *OFF:*
  - The `control` class is attribute-for-attribute identical to `62e59c0`, with the variable unset, `"0"` or `" 0 "`
    (probe `offcfg_*`, every `dir()` attribute's repr compared). Only the module-level names `GAIN_RAND` and
    `_gain_rand_from_env` are added.
  - `randomize_params` returns before the new branch (`base_lee_controller.py:93-97`).
  - `_gain_rand_condition()` returns `{}` (`navrl_task.py:3021-3031`), so neither the env state (`:3473`) nor the
    bulk-evaluation condition (`:9560`) changes.
  - The diff touches nothing else under `aerial_gym/`, `motar_eval/`, the launcher, the player or the protocol.
  - Every identity_02 record file of `off` equals `ref` (below).
- *ON:*
  - `NAVRL_GAIN_RAND=1` requires a scale in [0, 1) and a seed ≥ 0, and refuses anything else (5/5 unit tests pass at
    `c56be07`).
  - At every robot reset the path is: env manager `reset_idx` (`env_manager.py:316-329`, `:409-420`) →
    `robot_manager.reset_idx` (`robot_manager.py:477-478`) → `BaseMultirotor.reset_idx` (`base_multirotor.py:177-201`;
    `navrl_quad` is registered as `BaseMultirotor`) → `controller.randomize_params(env_ids)`. Each K_vel, K_rot and
    K_angvel element of the reset envs becomes nominal × (1 + s(2u − 1)), with u from a dedicated generator seeded once.
    The values are written in place into the tensors that `compute_acceleration` and `compute_body_torque` read
    (`:146-149`, `:165-167`). `update` runs every physics step (`base_multirotor.py:296-305`).
  - My CPU probe on a real `LeeVelocityController` (`audit/probe_controller.json`):
    - bounds [1 − s, 1 + s]: s = 0.4 gives min 0.600001, max 1.399999;
    - mean 1.0003, and variance matching s²/3 (0.05344 against 0.05333);
    - per-element KS and χ² consistent with uniform at 4 seeds × 102,400 draws (`audit/probe_uniformity.json`);
    - element cross-correlation ≤ 0.015, and reset-to-reset correlation −0.003;
    - non-reset envs keep their gains, and K_pos is untouched;
    - the writes are in place (same storage), and the global CPU RNG is untouched;
    - scale 0 is bitwise nominal;
    - after a reset, `update()` gives accel = K_vel_current × v exactly, and the torque equals the Lee formula on the
      current gains.
  - On the GPU: s00 = off exactly, and s40 ≠ off in 128/128 first episodes with identical initial states.
- *Anything else ON:* only `randomize_params = True`, `gain_rand_scale` and `gain_rand_seed` on the class, plus three
  `cfg_gain_rand*` keys in the condition and env state. The identity diff confirms nothing else differs.
- The target's controller (`PhysicalTargetController`) is not a Lee controller and is unaffected.

**Q2 — Identity.** These are my own comparisons of every file (`audit/audit_identity.json`).
- *ref vs off:*
  - `episodes.jsonl` (456 rows), `kroc2027_episodes.jsonl` (456 rows, all per-episode telemetry and hashes) and
    `episode_ledger_summary.json` are identical.
  - `result.json` differs only in `evaluation_nonce`.
  - `kroc2027_summary.json` differs only in `finished_at` and the import-origin paths, each inside its own tree.
  - `receipt.json` differs only in nonce, timestamps, commit, output path and result SHA.
- *off vs s00:* identical except the three `cfg_gain_rand*` keys, the three `NAVRL_GAIN_RAND*` variables, nonce,
  timestamps and duration.
- *s40:* all 452 shared episodes differ in outcome, step or distance, including all 128 first episodes. Quota counts
  are 224/32 against 240/16.
- *Cross-run check:* the identity off / s00 / s40 quota rows equal, field for field (ledger and recorder), the first two
  quota episodes per environment of the GR-0 160-bar seed-4501 S00 / S00 / S40 cells (256/256 each). This links the
  GR-0 OFF stream itself to the base-tree reference.
- *Amendment 1:* justified (L2). identity_02 is like for like: all four cells use the same A3 placement, and the env
  snapshots differ only in the gain variables and run-specific fields.

**Q3 — Runs.**
- `gr0_run_report.json`: verdict COMPLETE, 36/36 records OK, no preflight problems, order density → seed → level as
  preregistered, budget not exhausted (11:58–13:25).
- The identity link names identity_02 on the same commit.
- The host record is unchanged after every cell: fault count 0, the same boot id, online CPUs `0-3,6-23`.
- My independent per-cell audit (`audit/audit_cells.py`) found no problem in any of the 40 cells. It checked:
  - receipt: status OK, return code 0, commit, 0 dirty files, checkpoint `f7022139…` in receipt and result;
  - SHA-256 of `result.json`, the ledger rows and the recorder rows against the records that bind them;
  - gain variables and `cfg_gain_rand*` equal to the level, and absent when OFF;
  - backend `direct_l2_v1`, view `visible` at 10 m, arm label `A3`, run id, quota, protocol SHA, seed, density,
    governor `riskcap`;
  - ledger COMPLETE, 128 × 16 quota rows that are exactly episodes 0–15 of each env, contiguous indices, unique ids,
    consistent outcome flags;
  - recorder: covers every ledger episode, outcomes agree, motion gate PASS, 0 swept-clearance and wall violations,
    0 initial-view fail-closed, every episode initially in view, line of sight clear, range 10 m ± 3 µm, import origins
    inside the tree (the `ref` cells inside `MOTAR-mt-p1`);
  - no Traceback in any `cell.log`;
  - the other 66 `NAVRL_*` variables are identical in all 40 cells;
  - condition fields are identical within each block apart from `cfg_gain_rand*` and the nonce.
- The cell logs contain the same 34 warning classes at every level.
- All 303 raw files and 4 `motar_tmp` files match the manifest, with no unlisted files (`audit/verify_manifest.out`).
- No cell should be excluded.

**Q4 — Analysis.**
- My own estimator (`audit/recompute_analysis.py`) reproduces `run_copy/gr0_analysis.json` exactly from the ledgers:
  - 12 blocks, 2,048 quota episodes per cell;
  - pooled and per-density rates;
  - block deltas;
  - point estimates;
  - percentile CIs (an exact replay of the tool's RNG order with seed 20,261,110; indices 250 and 9,749 of 10,000);
  - both readings.
- The readings are stable under:
  - 5 other bootstrap seeds;
  - t-intervals (S20 capture [−1.06, −0.19], crash [+0.18, +1.04]; S40 capture [−4.48, −2.91], crash [+2.91, +4.48]);
  - leave-one-density-out (S40 Δcapture −3.34 to −4.16, always GAIN_SENSITIVE; S20 always GAIN_ROBUST).
- S40 is negative in 12/12 blocks; S20 in 10/12 (sign test p = 0.039).
- The S40 label holds through both preregistered criteria. The capture criterion (−3.69 ≤ −3, upper bound −3.03 < 0)
  is narrow. The crash criterion (+3.69 ≥ 2, lower bound +3.03 > 0) has a wide margin.
- Timeouts are about 0, so Δcrash ≈ −Δcapture.

**Q5 — Interpretation limits.**
- *Airframe generality: cannot be concluded either way.* Only Lee gains vary; mass, inertia, motor and drag stay
  nominal, and the elements are drawn independently.
  - A mass error is not equivalent to a velocity-gain change: it also biases the gravity feedforward, which this run
    does not emulate.
  - GR-0 measures F's sensitivity to controller mistuning on `navrl_quad`, nothing more.
- *GAIN_ROBUST ≠ no effect.* S20 shows a small degradation that the data can detect (CI excludes 0). It falls inside
  the ±3 / ±2 pp tolerance.
- *The motivation for GR-1 rests on S40.* That range (±40 %) exceeds the upstream Aerial Gym ranges (±20 % K_vel and
  K_rot, about ±33 % K_angvel). Over the realistic range the effect is about 0.6 pp. The response is strongly
  nonlinear: −0.6 pp at S20 against −3.7 pp at S40.
- *Crash mechanism:* see M2. Part of the S40 effect is early, high-tilt floor strikes, roughly independent of density.
  The rest is extra bar contacts that grow with density.
- *Density pattern:* the S40 Δcapture is −2.29 / −2.96 / −4.75 / −4.77 pp at 70 / 115 / 160 / 205 bars. The S20 pattern
  is flat (−0.44 to −0.93 pp).
- *Governor interplay:* the riskcap statistics do not move across levels (window means):
  - intervention rate 27.7 / 27.8 / 27.8 %;
  - executed speed 2.95 m/s;
  - executed negative-stopping-margin rate 14.2 / 14.3 / 14.2 %.
  The governor's fixed braking model (2.96 m/s², 0.1 s reaction) does not adapt to the drawn gains. GR-0 cannot say
  whether the density-dependent bar-contact increase comes from that mismatch.
- *Paired first episodes* (identical initial states, 1,536 per level):
  - S40 flips 81 capture → crash against 35 crash → capture;
  - S20 flips 33 against 33.
  Single-episode outcomes are chaotic, so only the pooled rates carry the effect.
- *Scope:* one target arm (E2) with the A3 visible start at 10 m, i.e. short pursuits (median crash at about 30–41
  actions); `navrl_quad` only; simulation only; the per-episode gains are unknown (M1).

## What I verified and how (all outputs in `review/audit/`)

| Check | Method | Result |
| --- | --- | --- |
| Raw-file binding | `verify_manifest.py`: SHA-256 and byte count of every manifest entry; walk the raw directory for unlisted files | 307/307 match, 0 unlisted |
| Unit's tests at `c56be07` | `trees/c56be07/tests/test_gain_randomization.py` (exported tree) | 5/5 OK (`unit_tests_c56be07.out`) |
| OFF config equality against `62e59c0`; ON draw distribution; gains reach the controller | `probe_controller.py`, `probe_uniformity.py` (child processes, path stubs, CPU) | all true (see Q1) |
| Code path | `git show c56be07:` of the config, `base_lee_controller`, `velocity_control`, `navrl_task` (reset, conditions), `env_manager`, `robot_manager`, `base_multirotor`, the registries, `kroc2027_eval` helpers, `gr_run`, `gr_analyse`; diff stat of `62e59c0..c56be07` | as described in Q1 |
| identity_01 cause | its four `kroc2027_summary.json` files and `cell.log` | the same swept-clearance stop in all four, including `ref` |
| Identity, every file | `audit_identity.py` | Q2 |
| Cell validity, 40 cells | `audit_cells.py` | 0 problems |
| Analysis recomputation and sensitivity | `recompute_analysis.py` | exact match; readings stable |
| Exploratory mechanism | `recompute_analysis.py`, `explore_crashes.py`, `crashdiag` lines in `cell.log` | M2 |
| Context | `compare_b1r.py` | I3 |
| Host | `journalctl` count before and after; `/sys/devices/system/cpu/online` | 0 → 0; `0-3,6-23` unchanged |
