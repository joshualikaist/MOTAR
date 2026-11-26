# CAM-REF stage 2: independent review r1 (read-only)

- Reviewer: Claude Code subagent, model Opus 5.5 (claude-opus-5-5), fresh context, 2026-11-26.
- Object: worktree MOTAR-camonly-s2b, branch camonly/s2b, HEAD 1614b15 plus uncommitted changes:
  M tools/camref_eval_run.py; ?? docs/plans/prereg_camonly_ref_stage2_2026-11-26.md, tools/camref_stage2_run.py,
  tools/camref_stage2_analyse.py, tests/test_camref_stage2.py. Stage-1 runner reference: `git show 358c363:tools/camref_eval_run.py`.
- Tests run (CPU, nice/taskset 6,7,12,13, CUDA_VISIBLE_DEVICES=""):
  `python -B -m unittest test_camref_stage2 test_camref` -> Ran 53, OK. The full PUBLIC_CPU suite was not run.
- My own CPU checks (scratch script, nothing in the repo written): for all 27 stage-1 output cells in
  ~/motar_tmp/camref_20261126/gpu1/{smoke,identity,eval}, new vs 358c363 `smoke_cell_controls` and
  `condition_problems` -> 0 differences. Old and new forms of the receipt telemetry map, the FREE expectation and the
  smoke-control gate, compared for every arm in ARMS+ARM_BASE -> they differ only for F4D/F4R/F4Y/F4V/F4A(+_smoke).
  Then I applied each new arm's smoke controls and result-condition check to stage-1 F4 and F8 cells (as wrongly built
  cells). The results are under Q2.

## Q1. Stage-1 cells unchanged? Yes.
- rung_delta: F4/F8 now go through `F_FACTORS` (camref_eval_run.py:197-224, 240-241). f_factor_delta gives F8 the same
  keys and module seeds as the old f8_extra; key order is kept too. K1S still uses f8_extra (:242-243). Test
  test_camref_stage2.py:47-53 checks equality with 358c363 for every stage-1 rung and seed.
- camref_rules_on / visible_start (:377-383) give the same sets as the old literal lists for stage-1 rungs.
  expected_installed (:776-790) keeps the `pol == "F"` guard, so K1S still gets no vehicle_rand/yawclip from
  camref_entry. The tests check expected_installed, visible_start and mismatch_allow against OLD (:55-63).
- condition_problems FREE (:682): `"free" in factors(arm)` matches the old rung set (F8, K1S) for stage-1 arms. My check
  on the stage-1 cells shows the same output.
- receipt telemetry map (:844-847): the same for every stage-1 arm (my enumeration). The realveh block (:819-840) is
  unchanged in the diff (K1S/S0 vehicle randomisation, K1S yawclip, cams2 config_after for S0).
- smoke_cell_controls (:893-917): F8_smoke and K1S_smoke still give exactly the keys
  {delay_draws_1_and_2, capture_rule_acted, veh_rand_init, yawclip_commands, start_bearings} with the same values on the
  stage-1 cells. S6 is unchanged. smoke_verdict and the identity and eval logic are not touched by the diff.
- preflight now receives UNIT_DIR from main (:1240). The stage-1 behaviour is the same (the default was the same value).

## Q2. New rungs = F4 + one factor; non-vacuous? Mostly yes.
- test :72-85 checks, per seed, that each rung's extra keys equal f_factor_delta of its own factor and nothing else,
  that no F4 key is dropped, that the union of the five extras equals f8_extra, and that F4 plus the union equals the F8
  delta. The module seeds are the F8 ones (stage-1 F8 receipt: delay seed 45014, vehicle-randomisation seed 45011 for
  4501). RUNG_PROTOCOL entries (:461-465) are consistent. The test (:100-106) shows that each new environment fails
  every other rung's protocol, so protocol mismatches are detected.
- Wrongly built cells: an F4 cell checked as F4D, F4R or F4Y fails its positive control. An F8 cell checked as any
  single-factor arm fails its negative controls, and fails the FREE result condition except for F4V. An F4 cell checked
  as F4V fails `speed_governor_free_mps`, so FREE has a real positive and negative control.
- **The gaps are F4A and the gain half of F4R (P2-1).** An F4 cell checked as F4A passes everything: smoke controls
  ALL_OK and no result-condition problem. Nothing after launch shows that the geometry acted. The only checks are the
  launch environment and the two NAVRL snapshots, which show that the keys reached the process, not that the module
  applied them. A usable discriminator is already recorded: camref_receipt.json `camref_rules.config_after`. It reads
  flight_altitude 1.0 / camera_translation[0] 0.10 in stage-1 F4 and 1.5 / 0.08 in F8. Gain randomisation (F4R) has no
  telemetry at all; it is read at import in lee_controller_config_navrl.py:37. The `[veh-rand]` init line covers only the
  vehicle half. Stage 1's F8 had the same two gaps, and the prereg says F4A is "checked by the cell environment and
  protocol checks", so this is consistent with the prereg but weaker than the per-factor control the stage-2 design relies on.

## Q3. Monkeypatch redirection: complete for main(); small leak risk.
- install() (camref_stage2_run.py:77-85) sets PREREG, UNIT, SCOPE, UNIT_DIR, REF_DIRS/REF_SHA (merged), the plans,
  IDENTITY_REF, EVAL_ARMS, the wall caps, SMOKE_CONTROLS, COMMITTED_FILES (which keeps the stage-1 prereg and the
  `R.NEW_FILES` entries) and NEW_FILES. Every reader looks these up at call time:
  - scope_digest (:398) and the approval check (:1189-1192, which passes `scope=SCOPE` to R1.approval_problems
    explicitly, so the R1 default is not used);
  - eval_plan, plan_env_problems and smoke_verdict / identity_verdict;
  - stage_budget_h (STAGE_SUB_CAP_H and MAX_HOURS_TOTAL) and the report fields (unit, prereg, scope).
- Default arguments bound at def time: `preflight(unit_dir=UNIT_DIR)` was one; main now passes it (:1240).
  The remaining defaults are ROOT, FIRST_N and K1_TRAIN_REPORT, which do not change between stages. No captured copies of
  the patched constants were found (COMMITTED_FILES is built at import but is reassigned whole).
- Approval file: ~/motar_tmp/camref2_20261126/execution_approval.json, as the prereg says. Reference pins: the six
  F4/F8 cells, 3 files each. The test (:145-148) checks that they are at their pins; I re-read stage-1 receipts and
  config.
- Budget: stage-1 F cells took 2.9-5.9 min, smoke 1.6 min and identity 1.7 min. 15 cells plus 2 identity cells plus 5
  smoke cells comes to about 1.3-1.6 h against the 2.0 h cap. The projection takes the identity F cells (quota 8) as
  earlier reports, as stage 1 did.
- Leaks: the tests use `installed()`, which restores everything (:92-101; the test at :120-130 checks this).
  However, camref_stage2_analyse.load_eval (:203) calls `S.install()` without restoring. No test calls it today, but a
  later test that did would leak stage-2 constants into the rest of the PUBLIC_CPU process (P3-1). The test module
  execs the 358c363 runner from a mkdtemp file at import and never deletes that file (P3-2).

## Q4. Analyser: matches the prereg.
- Q1 weights {X:+1, F4:-1} and Q2 {F8:+1, F4:-1} (analyse.py:150-151). Labels are computed on
  crash_bar, crash and capture (:144) using stage 1's label() (5 pp, CI excludes 0). Target and out-of-bounds crash are
  reported through METRICS.
- Interaction: (F8-F4) - Σ(X-F4) = F8 + 4·F4 - ΣX. The code sets the F4 weight to -1+5 = 4 (:152-154): correct. Its
  bootstrap interval comes from the same per-resample draws. The test (:156-164) shows ≈0 interaction for additive effects.
- Bootstrap: default_rng(4611), 10,000 resamples, order F4, F4D..F4A, F8 × seeds (:143,149), as preregistered.
- Carriers = bar MATERIAL with mean > 0, largest first (:160-166). The no-carrier rule (:172-173) is as preregistered.
- Controls (:188-199): the all-equal case and F4Y +8 pp -> [F4Y]. Both run before the eval is read; a miss gives
  CONTROL_FAIL (:237-241).
- Complete-block rule: all five factor arms per seed; none -> NOT_RUN, with no fallback (:221, 244-245). F4/F8 come from
  the pinned stage-1 cells.
- Binding checks (:206-214): identity PASS, same commit, eval report names the identity report, unit and scope,
  references at their pins, script equal to the eval commit. These mirror stage 1, plus the unit/scope check.

## Q5. Prereg ambiguities and gaps
- The prereg's use-of-result consequences for F4Y and F4D ("supports testing a higher cap", "latency work prioritised")
  are not emitted by use_of_result; only the carrier list and the text are (P3-3). The case "no carrier and F8-F4 not
  MATERIAL" is not defined in the prereg; the code reads "no preregistered pointer applies" (acceptable; record it).
- The prereg does not say whether a negative MATERIAL factor (X < F4) is reported specially. It is only labelled. Fine,
  but state it in the report.
- The smoke definition in the prereg relies on the environment checks for F4A, and has no gain-randomisation control
  (see P2-1). Any added control must go into the prereg before commit, because the prereg is still uncommitted.
- Identity covers seed 4501 only. The F4/F8 references for 4502/4503 rest on the stage-1 runs at 358c363 plus the
  refactor-equality tests. This is acceptable as preregistered; list it under Limits (P3-4).
- The camref_eval_run.py module docstring still describes stage 1 only (2.5 h, 21 cells) (P3-5).

## Findings
- P1: none.
- P2-1: F4A (geometry) and F4R's gain randomisation have no positive or negative control after launch. An F4 cell
  labelled F4A passes every check. Fix before the GPU run, while the prereg is still uncommitted: in receipt_problems,
  check `camref_rules.config_after` (flight_altitude, camera_translation[0]) = (1.5, 0.08) iff "geom" in
  factors(arm), else (1.0, 0.10), for F/K1 arms with camref_rules on. Add it to the smoke controls (F4A positive, the
  other four negative) and the prereg. For gain, either find a recorded trace of the gain draw, or record its absence as
  a limit in the prereg ("gain randomisation is verified only by the environment and snapshots, as in stage 1's F8").
- P3-1: analyse.load_eval calls install() permanently; use `with S.installed():` or document that it is a CLI-only call.
- P3-2: the test leaves a mkdtemp copy of the stage-1 runner behind; clean it up.
- P3-3: use_of_result does not emit the prereg's F4Y/F4D consequence lines.
- P3-4: put identity = seed 4501 only (F4/F8 references at 4502/4503 not reproduced) under Limits.
- P3-5: update the module docstring of camref_eval_run.py, or point it to stage 2.

VERDICT: GO_WITH_FIXES (P2-1: add the geometry config_after control, or record both control gaps as preregistered limits,
before the prereg is committed)
