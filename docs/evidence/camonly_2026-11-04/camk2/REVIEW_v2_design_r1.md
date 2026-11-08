# Design review r1: CAM-K2 v2 (raised per-motor limit) and CAM-K2-E v1 (2026-11-08)

- **Reviewer:** Claude Opus 5.5 (fresh context), read-only. This file is the only file written.
- **Time:** 16:47 to about 17:10 KST, 2026-11-08.
- **Host rule:** S4F-1's real-time SITL run needed an idle host. I ran no test, no import and no project Python. I
  used only cat, sed, grep, diff, git, ls, sha256sum and date. One slip: a bare `python3 -c "print(1)"`, with no
  import and output sent to /dev/null, in the command that listed CAM-K1-E's evidence folder (about 16:58). It ran
  for well under a second.
- **Worktree:** `MOTAR-realveh`, branch `realveh/dev` at `ab2eeb5`, with these uncommitted changes:
  - modified: `navrl_vehicle_rand.py`, `tests/test_realveh.py`, `tests/test_camk2_train.py`;
  - untracked: `tools/camk2v2_train.py`, `train_navrl_camk2v2.sh`, `tests/test_camk2v2_train.py`,
    `tools/camk2e_{run,analyse}.py`, `eval_navrl_camk2e_arms.sh`, `tests/test_camk2e.py`. `tools/sitl_s5_safety.py`
    and its test are also untracked; they belong to another unit and are outside this review.
- **What I read:**
  - the diff of the three modified files against `ab2eeb5`, and every untracked file under review;
  - `motor_model.py`, `control_allocation.py`, `base_multirotor.py`, `velocity_control.py` and the motor block of
    `base_quad_config.py`;
  - `realveh_entry.py` (settings part), v1's `camk2_train.py` (preflight and main), and a `diff` of v1 against v2 for
    both the wrapper and the launcher;
  - both draft preregistrations, `IMPLEMENTATION.md` Part 6, CAM-K2-E's `IMPLEMENTATION.md`, CAM-K1-E's
    `analysis.json`, and the three WITHDRAWN markers (one SHA-256, `b04d95f1…`, for all three).

## Verdicts

| Unit | Verdict | Why |
| --- | --- | --- |
| **CAM-K2 v2** (raise mode, wrapper, launcher, tests, prereg) | **AMEND_FIRST** | The mechanism is correct and bounded, and OFF and identity are unchanged. One MAJOR: the run never records whether the raised limit acts, and that cannot be measured after the run. Five MINOR. |
| **CAM-K2-E v1** (runner, analysis, launcher, tests, prereg) | **AMEND_FIRST** | The bindings, the identity stage, the matched seeds and the tests are sound. Two MAJOR: (1) gate A's "adaptation" is confounded with 1,000 more epochs of training; (2) the controls never show that a passing label can be reached, and gate N has no control. Five MINOR. |

Both units are desk-checked only: none of their tests has run. Every verdict here assumes the CPU tests and PUBLIC_CPU
then pass as written.

---

## Unit 1: CAM-K2 v2

### (a) The raised limit: checked and sound, with the open points below

I traced every run-time use of the per-motor maximum:
- **Reference clamp.** `MotorModel.update_motor_thrusts` clamps the reference to `self.max_thrust`
  (`motor_model.py:93`). This is the only clamp at run time. The module replaces that attribute after init
  (`navrl_vehicle_rand.py:163-166`), so the raised clamp acts on every later update.
  - The wrapper's own `min(ref, limit)` (`:295-297`) is then redundant, and harmless.
- **Reset state.** `MotorModel.reset_idx` draws `torch_rand_float_tensor(self.min_thrust, self.max_thrust)`
  (`motor_model.py:151-153`), so after the swap it draws in [0, limit] itself.
  - `redraw` correctly skips its rescale once `_veh_rand_limit_raised` is set (`:232`).
  - The construction-time state is rescaled once, from [0, 2] to [0, limit], by the `redraw` call at `:162`. That call
    runs before the swap, so `hi_t` is still the robot's own 2 N.
  - `rand_like` on a tensor of the same shape keeps the number of global-RNG draws unchanged.
- **Allocation and controller.** `ControlAllocator` uses a plain pseudo-inverse with no actuator limits
  (`control_allocation.py:48-50, 85-93`). The Lee velocity and attitude code never reads `max_thrust`. A grep of
  `aerial_gym/` finds no other run-time consumer of `max_thrust` or `current_motor_thrust`; `enjoy.py` reads only the
  config.
- **Cannot exceed the cap.** The reference is clamped to the limit.
  - The rpm-space lag (`use_rps=True`, discrete mixing 1/(dt+τ), the RK4 form) has a per-step gain below 1 for
    h = dt/(dt+τ) ≤ 0.67, which covers τ ≥ 0.005 s at dt 0.01. So the motor state cannot overshoot the reference.
  - The rate limit (`max_thrust_rate` 1e5) does not bind.
  - Delivered thrust is at most limit × (1+s_tw)(1+s_m) = 2.575 × 1.2075 = 3.11 N, which is the design.
  - `MAX_TW_NOMINAL` 6 is enforced both in the settings (`:75`) and at init (`:143-144`).
- **OFF and identity.**
  - OFF: nothing imports the module.
  - The "none" band, |tw − own| ≤ own × 1e-6, is the same band v1 used. The factor is exactly 1.0 and τ is rewritten
    with the same float32 value. `test_identity_settings_are_bitwise_off` uses `repr(OWN_TW)`. For the value printed
    in the docstring, see m-4.
- **The CPU tests use the real `ControlAllocator`, `MotorModel` and `BaseMultirotor.reset_idx`**
  (`test_realveh.py:101-120`). Only init is a stand-in, and that is where production builds the allocator too.
  `TestRaisedLimit` saturates at 60 N, checks the limit, and has an OFF control at 2 N. It is non-vacuous.
- **Equivalence arithmetic: re-derived and correct.**
  - Roll: 792.35, 615.38, 450.67 and 300.45 rad/s², ratios 1.758 and 2.637.
  - Yaw: 30.47 and 23.67 rad/s², and the 20.6–64.7 range.
  - Hover fractions 0.2381 and 0.3066.
  - Effective T/W: 3.57–4.83 per env, 3.39–5.07 per motor, worst case 3.39. Hover in the weakest env needs 0.759 N.
  - Because the hover fractions are matched, the sim-to-real roll ratio does not depend on the operating point; the
    "two motors at the limit" formula is a fair way to compare. The prereg could say so.
- **Is the raise a sound stand-in for the 1.1 kg vehicle?** For thrust-to-weight and hover fraction, yes. Two
  qualifications:
  - The prereg already discloses the 45° tilt limit (`lee_controller_config_navrl.py:74`; `velocity_control.py:46-53`):
    lateral acceleration at level flight is at most 1 g, both at 3.26 and at 4.2. The raise therefore acts only when a
    per-motor reference exceeds 2 N, and nobody knows how often that happens (M-1).
  - Drag per unit mass is navrl_quad's (L-2).

### (b) The shared module and v1

- **v1's "lower" path is behaviourally unchanged:** the same cap, rescale, draws, "none" band and update wrapper.
  Four things do change, all listed in the prereg (`PREREG_V2_DRAFT.md:298-317`):
  - v1's run-time refusal of a TW_NOMINAL above the robot's own T/W becomes a silent raise;
  - the TW bound goes from 10 to 6;
  - the record gains two keys and the init line gains `mode`;
  - one committed test assertion is replaced.

  The first is a real change in behaviour (m-1).
- **v1 cannot run through its wrapper.** It refuses for three reasons:
  - there is no approval file; none exists, and the approval rule is the real guard;
  - the three WITHDRAWN markers occupy its single-use folders (`camk2_train.py:450-451`). `main` only prints a
    PREFLIGHT_FAIL, so the markers are never mixed with run files;
  - the seeds need a SMOKE_OK that cannot be produced.

  The markers are honest: they say what they are, why, and the replacement. They are a sound second layer.
- **Better or extra measures:**
  - Commit the WITHDRAWN_BEFORE_RUN status in the v2 commit: the `docs/EXPERIMENTS.md` family status and the v2
    prereg's "How v1 refuses". The markers live outside Git.
  - Optionally, add a committed refusal in v1's wrapper with the reason. That changes v1's code bytes, but not its
    preregistration, and it would make the refusal message truthful: today it reads "output exists (single use)",
    not "withdrawn".
  - Running v1's launcher directly, with `CAMK2_SOURCE_CKPT` set, skips the wrapper and the markers. That is true of
    every unit, so the approval rule is the governance (L-5).

### Findings, CAM-K2 v2

**M-1 (MAJOR): whether the raised limit acts is never recorded, and the live clamp is not read back.**
- `PREREG_V2_DRAFT.md:125-126, 352-353` calls the share of motor references above navrl_quad's 2 N
  **NOT_RECORDED**. The prereg itself says that if it is small, "the unit mainly tests the randomization and the depth
  model". So the unit's headline treatment ("the real vehicle's T/W") could be inert, and the run would not show it.
  This is the "measure coverage" item of the gate-design checklist, and it cannot be recovered after the training has
  run.
- Separately, nothing in the run's evidence shows that the swap took effect on the GPU:
  - the record's `thrust_limit_per_motor_n` is `cap64` (`navrl_vehicle_rand.py:172`), not a read-back of
    `mm.max_thrust`;
  - the update line logs only `cap_applied` (`:299-301`);
  - the wrapper's check `limit != cap` (`camk2v2_train.py:215`) compares two copies of `cap64`, so it is tautological
    for the swap.
- **Fix** (additive; no change to the dynamics):
  1. In the update wrapper's raise mode, accumulate on the device, with no per-step sync:
     - the count of references above `_veh_rand_own_max_thrust`;
     - the count at or above the limit × (1 − 1e-6);
     - the maximum reference.
  2. Emit them with `.item()` only at the existing log points: the first update, every `UPDATE_LOG_EVERY`, and the
     reset lines. In the first update line, also log `float(self.max_thrust.min())` and `float(self.max_thrust.max())`.
  3. `camk2v2_train.telemetry_problems` and `camk2e_run.telemetry_problems` require that live maximum to equal the
     limit.
  4. Report the shares as descriptive records in the train report. CAM-K2-E reports them per arm.
  5. Extend `TestRaisedLimit`: counts > 0 in the saturating case; 0 in an unsaturated control; no counters in modes
     "lower" and "none".

**m-1 (MINOR): mode selection is implicit, which removes v1's run-time refusal.**
- At `ab2eeb5`, a TW_NOMINAL above the robot's own T/W refused at init. Now it raises the limit silently
  (`navrl_vehicle_rand.py:143-153`).
- To fit this, the committed v1 test `assertRaises(RuntimeError): good_record(1, mass=0.5)` was replaced
  (`tests/test_camk2_train.py:487-489`), and the run-time refusal test was rewritten (`tests/test_realveh.py:751-763`).
- For v1 the effect is nil, since v1 is withdrawn. For any later unit, a wrong mass or a mistyped TW now flips the
  direction, and only the post-run record check catches it, after the GPU hours are spent.
- **Fix:** make the raise opt-in.
  - Add an optional `NAVRL_VEH_RAND_ALLOW_RAISE=1`. When it is absent, a raise refuses exactly as in v1. It is set only
    by `train_navrl_camk2v2.sh` and by CAM-K2-E's `treatment_env`, and it appears in the receipt settings.
  - Restore the v1 test.
  - If you keep the implicit mode instead, say in the prereg that the run-time refusal is replaced by the post-run
    record check.

**m-2 (MINOR): T/W 4.2 is not tied to its source.**
- `PREREG_V2_DRAFT.md:40-44` says only "public thrust data … about 1.1–1.2 kgf". The relayed data point (LIGPOWER
  V2306 V2 2400 KV, T5143S, 4S: 1,084 g at 100 %) gives 4 × 1.084 / 1.1 = **3.94**. Reaching 4.2 needs about +6.5 %
  from 2550 KV.
- Static bench thrust at full pack voltage overstates what is available in flight, because the voltage sags. It is
  also unknown whether 1.1 kg includes the battery.
- **Fix:** cite the data point, show the step from 3.94 to 4.2, and state which way the estimate is biased. Note that
  the ±15 % env factor (3.57–4.83) covers 3.94. "Numbers travel with their source" applies.

**m-3 (MINOR): the frame inertia behind the rotational ratio has no stated basis.**
- `PREREG_V2_DRAFT.md:89-100, 332, 346-349` gives a ratio of 1.8–2.6×, from Ixx 0.004–0.006 kg m². The arithmetic is
  correct; the inertia range is the weak point.
- My own rough component build-up (INFERRED) gives about 2.2–2.5e-3 kg m²:
  - 4 motors of about 37 g at y = 0.0795 m: about 0.9e-3;
  - a battery of about 0.2 kg mounted on top: about 0.35e-3;
  - a central stack of about 0.55 kg: about 0.6–0.8e-3;
  - arms and the OAK-D: about 0.3e-3.
- Scaling a typical 0.75 kg 5-inch research frame (Ixx about 2.5e-3, from memory, not checked) to 1.1 kg and 112 mm
  arms gives about 3e-3. At Ixx 0.0025–0.003, the ratio is about 1.1–1.3×.
- **Fix:** state the basis, widen the range to about 0.0025–0.006 (ratio about 1.1–2.6×), and keep it INFERRED. The
  sentence "attitude saturation is rarer in the sim" still holds across that range.

**m-4 (MINOR): the identity value "3.2620" does not give identity.**
- The docstring (`navrl_vehicle_rand.py:52`) and the prereg (`PREREG_V2_DRAFT.md:83`) say "3.2620, within relative
  1e-6". But 8 / (0.25 × 9.81) = 3.2619776, and 3.2620 is 6.9e-6 away in relative terms, which is above `OWN_TW_RTOL`
  1e-6 (`:79`).
- So TW_NOMINAL=3.2620 selects "raise": the limit becomes 2.0000137 N and the init rescale multiplies the motor state
  by 1.0000068. That run is not bitwise OFF. The test passes only because it uses `repr(OWN_TW)`.
- **Fix:** either give the exact 8/(m g), or widen the "none" band to about 1e-4 relative. That changes nothing at
  2.25 or at 4.2.

**m-5 (MINOR): `test_v1_cannot_run` never runs v1's preflight.**
- `tests/test_camk2v2_train.py:806-818` asserts a source substring and that the markers exist, and it skips when the
  folder is missing.
- **Fix:**
  - call `camk2_train.main(["1", "--smoke"])` with the GPU and process probes stubbed, and assert PREFLIGHT_FAIL with
    "output exists (single use)";
  - add a positive control: with a temporary `UNIT_DIR` that has no marker, that problem is absent;
  - make a missing folder fail rather than skip on the maintainer's host.

**L-1 (LOW).** `test_translational_capacity_equals_a_1p1_kg_vehicle_at_tw_4p2` (`test_realveh.py:828-837`) is
arithmetic on the definition `limit = 4.2 m g / 4`. It shows the record uses the controller's mass, nothing more.
Describe it that way in the prereg (`:106-108`).

**L-2 (LOW).** Drag and frontal area per unit mass are navrl_quad's (PhysX damping), not those of a 1.1 kg airframe.
Add this to "Disclosed limits". Reword "translational capacity matches" as "thrust-to-weight (maximum thrust
acceleration and hover fraction) matches".

**L-3 (LOW).** `initialize()` is not idempotent. A second call after the swap would read the raised tensor as the
robot's own maximum and select "none". Production calls it once, and the record's mode "x" would fail anyway, but add
a guard that refuses when `_veh_rand_ready` is already set.

**L-4 (LOW).** Some text is stale:
- the `redraw` docstring (`:203`) still says it always rescales;
- `cap_applied` and `cap_per_motor_n` read as a cap in raise mode;
- CAM-K2-E's `IMPLEMENTATION.md`, pre-update "To do 1", still points at `~/motar_tmp/camk2_20261108/train_s{1,2}`
  (v1's folders).

**L-5 (LOW).** Both launchers run if invoked directly with the `*_SOURCE_CKPT` variable set, which skips the wrapper,
the markers and the approval check. Say in "How v1 refuses" that only the wrapper path is guarded.

---

## Unit 2: CAM-K2-E v1

### (c) What holds

- **Matched module seeds.** 10e+1 and 10e+2 (45011/45012, …) are a function of the cell seed only, the same in every
  arm, and distinct from training's 4911/4912 and 4921/4922. The prereg honestly states that common random numbers
  hold only until the first termination.
- **Identity stage.** It has three parts:
  - an OFF cell through the realveh entry must equal CAM-K1-E's camtrain cell (result minus nonce and path, plus
    episodes). This satisfies "OFF reproduces native" before ON is measured;
  - an ON negative control;
  - a K2 smoke that must differ from it.

  The analysis also binds both identity ON cells to the eval's (205 bars, seed 4501), which is a reproducibility check
  of the ON path.
- **Bindings** (`camk2e_analyse.py:139-187, 215-234`): the commit, completeness and order, the per-cell receipt checks
  re-run with the run report's committed SHA-256, the result condition and the allowed snapshot differences against
  CAM-K1-E's cell of the same block, cells-differ checks, CAM-K1-E's manifest, the checkpoint pins and the script's
  bytes. They are complete for the claim as now written. The K1s1/K1s2 reference mapping (`:168`) is correct.
- **v2 only.** The runner imports `camk2v2_train`, refuses v1's unit, prereg or treatments (`camk2e_run.py:150-176`),
  and requires mode "raise" in both the init line and the record.
- **Tests, by reading, are non-vacuous:**
  - every negative receipt, checkpoint and approval case has a positive control (`test_camk2e.py:237-262, 301-327`);
  - the gate tests cover the floor, CRASH_TO_TIMEOUT and a null;
  - the df 4 and df 8 values are checked;
  - the runner and analysis smokes include known-bad runs.
- **Budget arithmetic.** 63 × 157 s = 2.75 h. The eval stage's 60 cells take about 2.6 h against the 3.5 h cap, a
  margin of about 34 %.
- **The expected control numbers** match CAM-K1-E's `analysis.json`:
  - F66 − K1s1 is +37.67 pp with t [36.45, 38.89];
  - K1s1 − K1s2 is −0.92 pp with t [−1.78, −0.07] and bootstrap [−1.53, −0.33]. By `gate_a` that is NEGLIGIBLE, as
    the prereg says.

### Findings, CAM-K2-E

**E-M1 (MAJOR): gate A does not identify "adaptation".**
- K2s is K1s plus 1,000 epochs at LR 5e-6, all trained under the treatments. So K2s − K1s under the treatments
  (`PREREG_DRAFT.md:58, 220`) mixes adaptation to the treatments with plain continued training.
- Gate N cannot separate them. It only bounds a nominal loss (upper end < +3 pp, `:221`); a nominal improvement of the
  same size would read NO_MATERIAL_LOSS.
- The prereg admits the bundling (`:265-266`), yet allows the word "adaptation … to these simulated treatments"
  (`:258-259`).
- **Fix:**
  1. Add an interaction contrast on the 6 nominal blocks (df 4): I_s = (K2s − K1s)_ON − (K2s_nom − K1E_K1s)_OFF, with
     weights {K2s: +1, K1s: −1, K2s_nom: −1, K1E_K1s: +1}. The existing `contrast()` takes these weights as they are.
  2. Allow "adaptation" only when gate A reads MATERIAL or SMALL **and** both of I_s's interval upper ends are below
     0 (or below −2 pp). Otherwise, read gate A as "lower crash under the treatments", and consider renaming its
     labels CRASH_REDUCTION_*.
  3. Report K2s_nom − K1E_K1s with a two-sided label.
  4. Add a synthetic analysis test: a uniform improvement, equal ON and OFF, must not read as adaptation.

**E-M2 (MAJOR): no control shows that a passing label can be reached, and gate N has no control.**
- `camk2e_analyse.py:124-136` and `PREREG_DRAFT.md:232-238` hold two controls:
  - WORSE on a contrast of +37.7 pp, 7.5× the band;
  - a null that only has to avoid MATERIAL.

  Neither shows that MATERIAL_ADAPTATION or NO_MATERIAL_LOSS can actually be read on these block counts.
- **Fix:** add these controls on CAM-K1-E's data, with no new GPU work:
  1. K1E_K1s1 − K1E_F66 (−37.7 pp, timeouts about 0) must read **MATERIAL_ADAPTATION**;
  2. gate N on the 6 nominal blocks: K1E_K1s2 − K1E_K1s1 must read **NO_MATERIAL_LOSS**. This is the non-vacuity of
     N's passing label at df 4;
  3. gate N on K1E_F66 − K1E_K1s1 must read **MATERIAL_LOSS**.
- No claim-scale (about 5 pp) known-bad contrast exists in the current data. Record the null control's interval
  half-widths (about 0.9 pp at 12 blocks, more at 6) as the resolution, and disclose that gap.

**E-m1 (MINOR): "treatments OFF" is no longer the easier condition.**
- For v2, OFF lowers T/W from 4.2 to 3.26 and fixes τ at 0.04 s, the slow end of the trained range. It also removes
  the depth noise.
- So gate N's "nominal loss" (`:59, 221`), and the OFF arm of the E-M1 interaction, include a vehicle weaker than the
  one trained on. v1's framing, where OFF was strictly easier (more thrust), carried over without being re-examined.
- **Fix:** say in Question 2, in gate N and in the "may be said" text that "nominal" means F's vehicle (T/W 3.26,
  τ 0.04 s, exact depth).

**E-m2 (MINOR): the budget has no rate gate, and `--max-hours` is free.**
- `camk2e_run.py:414, 455`; `PREREG_DRAFT.md:175-178`.
- 50 ON cells carry an unmeasured depth-model cost. An overhead above about 34 % hits the 3.5 h cap and leaves the
  eval INCOMPLETE, which loses it.
- **Fix:**
  - refuse `--max-hours` other than 3.5, as the training wrapper does;
  - at eval preflight, project from the identity report's cell wall times (2 ON, 1 OFF): refuse unless
    48·t_on + 12·t_off ≤ 3.5 h / 1.1;
  - before the commit, re-estimate the budget from CAM-K2 v2's smoke step time.

**E-m3 (MINOR): the T-LOAD coverage argument is missing.**
- `PREREG_DRAFT.md:304-306` allows concurrent CPU work under T-LOAD. T-LOAD covered the camtrain path. The realveh
  modules add CPU-side draws at every reset and a depth model, and AGENTS.md keeps an idle host for paths T-LOAD did
  not cover.
- The draws are independent of load by construction, since they involve no timing. The ON-cell binding between the
  identity stage and the eval is an empirical check on two cells.
- **Fix:** state that argument, or keep the host idle, or allow only the disjoint cores.

**E-m4 (MINOR): the approval wording.**
- `PREREG_DRAFT.md:46-47, 382` leaves open whether the user's 12:07 KST answer covers this unit. That answer predates
  CAM-K2 v2 (16:10) and this v2 scope.
- Under AGENTS.md (approvals name their scope; never infer approval from an earlier one), it cannot cover this unit.
- **Fix:** say that a new approval naming SCOPE and HEAD is required. The same holds for the CAM-K2 v2 training run.

**E-m5 (MINOR): the commit order.**
- `test_camk2_pins_are_filled` (`tests/test_camk2e.py:265-268`) fails by design.
- PUBLIC_CPU finds untracked tests from the filesystem (`tools/run_tests.py:106-111`).
- v2's preflight requires a clean tree, untracked files included.
- So CAM-K2-E's four files must stay out of the worktree where v2's PUBLIC_CPU and training run, until the pins are
  filled. Part 6's test command includes `tests.test_camk2e` and will show that one failure.
- **Fix:** write this order into both preregs' "To do".

**E-L1 (LOW).** The identity stage tests one CAM-K1-E cell, K1s1 at 205 bars and seed 4501. The K1s2 cells and the
other blocks are admitted by extension; the snapshot and condition bindings reduce the risk. Disclose it.

**E-L2 (LOW).** The ON path's determinism is checked only after the whole eval: the 205/4501 block is about cell 49
of 60. Optionally, run `K1s1_on_negative` twice in the identity stage (+1 cell, about 3 min), so a nondeterministic
ON path fails before 2.6 GPU-hours are spent.

**E-L3 (LOW).** The stratified percentile bootstrap with 3 seeds per stratum has at most 10 distinct resample means
per stratum, and 100 combinations for gate N's 2 strata, so its interval ends are coarse; the t-interval carries
gate N. This is inherited from CAM-K1-E. Disclose it.

---

## (d) Prereg wording, summary

The v2 prereg is careful. It discloses the bundling, PX4 against Lee, the 45° tilt bound, the reset state, the
untested altitude-sag transient and the not-yet-run tests. Changes:
- add the measured share of motor references above 2 N (M-1);
- give the source of the T/W and inertia figures (m-2, m-3);
- use the exact identity value (m-4);
- add the drag limit (L-2).

The CAM-K2-E prereg needs:
- the interaction rule before the word "adaptation" (E-M1);
- the positive-label controls (E-M2);
- F's vehicle as the meaning of "nominal" (E-m1);
- the new approval (E-m4).

## One line per finding

- V2 M-1 MAJOR: whether the raise acts (references above 2 N) is NOT_RECORDED and unrecoverable after the run; the
  live `max_thrust` is never read back (`navrl_vehicle_rand.py:172, 299-301`; `camk2v2_train.py:215` is
  tautological). Add on-device counters and a live read-back to the telemetry.
- V2 m-1 MINOR: implicit mode selection turned v1's run-time refusal into a silent raise, and a committed v1 test was
  weakened (`:143-153`; `test_camk2_train.py:487-489`). Make the raise opt-in.
- V2 m-2 MINOR: T/W 4.2 is uncited. The relayed LIGPOWER point gives 3.94 at 2400 KV, and a bench figure overstates
  flight. Cite the point and show the step to 4.2.
- V2 m-3 MINOR: Ixx 0.004–0.006 has no basis; a build-up suggests about 2.5–3e-3, so the ratio is about 1.1–2.6×.
  State the basis and widen the range.
- V2 m-4 MINOR: "3.2620" is 6.9e-6 from 8/(m g), outside the 1e-6 band. It selects raise and is not bitwise OFF.
  Give the exact value or widen the band.
- V2 m-5 MINOR: `test_v1_cannot_run` greps text and skips. Exercise v1's preflight, with a positive control.
- V2 L-1 LOW: the translational-capacity test is arithmetic on the limit's definition. Describe it as such.
- V2 L-2 LOW: drag per mass is not matched. Disclose it, and reword "translational capacity".
- V2 L-3 LOW: `initialize()` is not idempotent after the swap. Add a guard.
- V2 L-4 LOW: stale `redraw` docstring and "cap" names in raise mode; CAM-K2-E's IMPLEMENTATION "To do 1" points at
  v1's folders.
- V2 L-5 LOW: running a launcher directly skips the wrapper and the markers. Say that only the wrapper path is
  guarded.
- E M-1 MAJOR: gate A confounds adaptation with 1,000 more epochs. Add the interaction
  (K2−K1)_ON − (K2_nom−K1E)_OFF on 6 blocks and gate the word "adaptation" on it.
- E M-2 MAJOR: no control reaches MATERIAL_ADAPTATION or NO_MATERIAL_LOSS, and gate N has no control. Add K1s1−F66
  (MATERIAL_ADAPTATION), N on K1s2−K1s1 (NO_MATERIAL_LOSS) and N on F66−K1s1 (MATERIAL_LOSS).
- E m-1 MINOR: in v2, OFF is F's weaker vehicle (T/W 3.26, τ 0.04 s), not an easier condition. Reword gate N.
- E m-2 MINOR: no eval rate gate, and `--max-hours` is free (`camk2e_run.py:414`). Project from the identity cells and
  fix the cap at 3.5 h.
- E m-3 MINOR: concurrent CPU work under T-LOAD lacks a coverage argument for the realveh modules. State it, or keep
  the host idle.
- E m-4 MINOR: the 12:07 answer predates v2 and cannot cover this unit. Require a new scoped approval.
- E m-5 MINOR: the by-design failing pin test plus filesystem test discovery means CAM-K2-E's files must stay out of
  v2's tree until the pins are filled. Write the order down.
- E L-1 LOW: the identity stage covers one K1s1 cell only. Disclose it.
- E L-2 LOW: ON determinism is checked only after the full eval. Optionally repeat the ON cell in the identity stage.
- E L-3 LOW: the 3-per-stratum bootstrap has coarse interval ends. Disclose it.
