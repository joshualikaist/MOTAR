# CAM-S3-E results review r1 (161c08e over e73c573), read-only, fresh context

Reviewer: Claude Opus 5.5 (`claude-opus-5-5`), 2026-11-28. CPU only (`CUDA_VISIBLE_DEVICES=""`, nice 10, cores 6,7,12,13).
Probes in this folder: `probe_recompute.py` -> `recompute.json`; `probe_boot_diff.py`; `probe_liveness.py` -> `liveness.json`;
production-analysis reruns `rerun_aerialgym.json` (aerialgym env, numpy 1.23.0) and `rerun_sys.json` (miniconda base, numpy 2.5.1).
No repository file was written (git status clean).

**Verdict: PASS_WITH_LIMITS.** No P1. One P2 and five P3. The status labels and the pointer (STRUCTURAL_NEXT) do not change.

## 1. Provenance: PASS
- All three stage reports and all 17 cell receipts show commit e73c573, `git_dirty_files` 0, scope SHA 63fb7f7e (the same value
  `camref_stage3_run.py scope` gives now), aerial_gym imported from the s2b worktree, and the aerialgym interpreter.
- The user's words were recorded at 22:40. The approval file was written at 22:58:19 and smoke started at 22:58:40. The
  commit and SHA were filled in after the user spoke, as the approved text said they would be (see P3-5).
- S0ID matches V312/4501 on every one of the 1,024 episodes: outcomes differ 0, crash classes differ 0, counts 743/273/8.
  The pinned S0 reference SHAs match.
- Host faults were 0 -> 0 at every stage. Boot id and online CPUs (0-23) did not change.
- GPU wall: 383 + 329 + 3,510 s = 1.17 h. That is under the 2.0 h cap and under the 2.4 h ledger (eval budget 1.80 h,
  `earlier_stages_h` 0.198). All outputs are under the unit folder.
- The evidence copies are byte-identical to the raw reports. The analysis JSON is identical to its committed copy.

## 2. Recompute from per-episode files: numbers match, one label depends on the interpreter
- My counts come only from `episodes.jsonl`, using the first 8 episodes per env, with 128 envs x 8 in every cell. Crash
  classes come from a join with `cams2diag_episodes.jsonl`.
  - These counts equal the analysis for every per-seed rate, every pooled rate, every Wilson bound and label, and the
    Q1 result, BAR_NOT_MET_POINT.
  - W: crash 25.39 % [24.12, 26.70] ABOVE_20, bar contact 15.72 % BELOW_20.
  - Q2 means: W-S0 crash -3.125, W-S0M -3.711, WT-S0T -5.469.
  - Statuses: NO_IMPROVEMENT / NO_IMPROVEMENT / IMPROVES. Pointer: STRUCTURAL_NEXT.
- **P2-1 (WORKLOG.md:27516; eval_analysis.json Q2.W-S0.capture).** The committed analysis was run with
  `/home/fair/miniconda3/bin/python3` (numpy 2.5.1), not the project's aerialgym env (numpy 1.23.0). `rerun_sys.json`
  reproduces the committed JSON exactly. The draws of `default_rng(4711).multinomial` differ between the two numpy
  versions.
  - Under the aerialgym env, the co-primary label W-S0 capture changes: [-0.03, +4.43] NO_DETECTED_CHANGE, against
    [+0.03, +4.39] SMALL in the committed run.
  - Every other label is the same under both interpreters, and so are the statuses and the pointer.
  - The record does not name the interpreter. "[0.0, 4.4] SMALL" hides that the interval's lower end is within 0.03 pp
    of 0.
  - Fix: record the interpreter and numpy version, and say that this one label depends on the interpreter
    (SMALL / NO_DETECTED_CHANGE) with no effect on status or pointer.

## 3. Liveness: PASS (all 12 eval cells, 4 smoke cells and S0ID)
- **Measured depth** (`install_measured`, false near off, TOP_ROWS 10, `meas_first` over 1,849 frames, invalid share
  0.12-0.14):
  - present in S0M, W and WT;
  - absent in S0T and S0ID, which have holes 0.02.
  - S0M and W show identical invalid shares in each seed, so they share the mask pointer.
- **Sector rule** (`sector_first`, k 24 over 13 bins): present in W and WT only.
- **Yaw scale:** the snapshot reads 1.5708 for W and WT, and 1.0472 for the others. No cell has a YAW CONFIG MISMATCH.
  The only CONFIG MISMATCH in any cell is NAVRL_P9_SEED, which the V312 reference shows as well.
- **ARENA lines:** S0T reads `checkpoint=bars_h3 running=bars_h3_thin` and W reads `bars_h3_thin->bars_h3`. S0M and WT
  have none. The pools match each arm.
- **Checkpoint SHAs:** f2c80fcc (S2B) and 2626a4b2 (S3W) in every receipt.

## 4. Records
- All numbers in the WORKLOG table, Q1, Q2, Q3, the intervention shares, the 37 % (9.41 / 25.39), the walls and the
  identity counts match the machine records (apart from P2-1).
- The "results review NOT_RUN / CONDITIONAL" wording is honest, and the open findings from review r2 are listed.
- **P3-1 (WORKLOG.md:27523-27524):** "about half of the gain went to target contacts" suggests that individual episodes
  switched from one crash type to the other. The two policies' episodes are not paired. Suggested wording: "the
  target-contact rise (+3.4 pp) offsets about half of the bar-contact fall (-6.6 pp)".
- **P3-2 (WORKLOG.md:27526-27527):** "the 90 deg/s policy meets the target more often" ties the effect to one factor.
  The yaw scale, the masks, the sector rule, thin-bar training and 3,000 epochs are confounded (prereg Limits). Say
  "S3-W".
- **P3-3 (WORKLOG.md:27521):** "sensor model alone changes little". S0M-S0 is unlabelled, with an interval of
  [-1.7, +2.9] pp. Say "no label; the interval includes 0".
- **P3-4 (WORKLOG.md:27520, 27536):**
  - WT-S0T IMPROVES compares S3-W on its own training arena with S2b-FT off its training arena (ARENA mismatch). The
    record should say so beside the label.
  - The "Next" line adds "target-approach terms / capture geometry", which the fixed pointer does not include
    (S3-M or the learning rate). Mark it as a suggestion, not the pointer.
  - "last log window" should read "running share from the last log line" (amendment P3-8).
- **P3-5 (procedural; execution_approval.json):** the user approved a paraphrased scope at 22:40. The commit e73c573
  (22:49) and the scope SHA came afterwards. This matches the approved wording ("re-review before launch"), but the user
  never saw the SHA itself.

## 5. Other
Nothing else makes these results non-evidence:
- complete blocks only;
- no OK cell missing;
- `actual_episodes` 1,024-1,026, with exactly 1,024 counted;
- no code change in 161c08e;
- the analysis controls give 9/9 OK under both interpreters.
