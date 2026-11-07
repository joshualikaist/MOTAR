# S3c v1 and S4 v1 — preparation record (2026-11-06) and attempt01 results (2026-11-07)

**Status:** both attempt01 runs finished on 2026-11-07 (COMPLETE, 0 integrity problems). Neither unit has a pass or
fail label; see [Results](#results-attempt01-2026-11-07). Results review r1: S3c PASS_WITH_LIMITS, S4
PASS_WITH_LIMITS with its two MAJOR findings repaired below. Read the preregistrations first:
- [S3c v1](../../../plans/prereg_sitl_s3c_v1_2026-11-06.md): PX4's iris, and a one-step command delay;
- [S4 v1](../../../plans/prereg_sitl_s4_v1_2026-11-06.md): Gazebo's own LiDAR, RGB and depth, with ground-filtered and
  raw arms.

Both run [S3b formal v1](../s3b_formal/README.md)'s episode and unit loop. Each has a development smoke01 (one
episode) that must be COMPLETE and VALID, at the same HEAD and source bytes, before its attempt01.

## S4 component smoke (development, before preregistration)

Record: [`s4_component_smoke.json`](s4_component_smoke.json). Its script and output are bound in
[`validation.json`](validation.json). The smoke ran gzserver only, without PX4 or a policy, with OpenGL on display :1:
a sensor rig at 1 m, a bar 3 m ahead and one 2 m to the left, and the target sphere.
- **LiDAR order:** verified. The front bar reads 2.800 m in MOTAR column 36 at 0° elevation, 2.843 m at ±10° and
  2.979 m at +20°. The left bar reads 1.800 m in column 18.
- **Depth:** 2.700 m on the front bar.
- **Target:** a moved target appears in the next frame.
- **Found:**
  - At 4.1 m, F's colour rule passes 4 of the 35 reddish target pixels, because Gazebo's shading darkens most of the
    sphere below the 0.55 threshold.
  - The −10° LiDAR row hits the ground at about 5.5 m.
  - MOTAR's camera fusion takes each column's minimum depth over all rows, so a floor in the lower image rows reads
    as obstacles about 2 m ahead. That is why S4 has a ground-filtered arm.

## Independent reviews (Claude Opus 5.5, fresh context)

- [r1](REVIEW_r1.md): **S3c PASS_WITH_LIMITS, S4 FAIL.**
  - **S4 BLOCKER:** `navrl_quad_s4` kept navrl_quad's visuals. Ray casts put the iris body mesh and the propeller discs
    in the LiDAR beams at 0.20–0.31 m and in the camera at 0.05–0.15 m. The smoke rig had no vehicle, so it could not
    show this.
  - **S4 MAJOR:** recall used fused visibility, which includes LiDAR association.
  - **Both, MAJOR:** the analyzers lacked S3b's integrity checks and timing.
  - **Repairs:**
    - visuals removed, with a vacuity check;
    - a 0.35 m hover guard;
    - camera-only recall, with LiDAR-only counted separately;
    - the shadow depth clamp, frame age in lateness, −inf → 0;
    - render-thread RGB;
    - port, GPU and plugin-age refusals;
    - the target height offset;
    - analyzers with integrity, timing and a SHA-bound S3b reference;
    - an iris-footprint report;
    - smoke gates;
    - wording.
- [r2](REVIEW_r2.md): **S3c PASS_WITH_LIMITS, S4 PASS_WITH_LIMITS.** No BLOCKER or MAJOR.
  - **S4 verification:** from the Gazebo 11 source, collisions are not rendered by cameras or gpu_ray. The two included
    sensor visuals (rangefinder, GPS) are inside the 0.2 m near clip and behind the camera.
  - **Repaired after r2:**
    - s1: RGB and depth are stamped with the rendering scene's simulation time;
    - s2: the smoke must match HEAD and the source bytes;
    - s3: `--analyze` refuses a smoke;
    - the chain launcher stops on a host fault, builds the plugin in a clean directory, and has 3 h timeouts.
  - **Open LOWs:**
    - the Gazebo–PX4 clock offset is not computed;
    - steps with RGB older than the target update are not summarised;
    - no test feeds a near depth value into the hover guard.

## PX4-tree additions (outside every repository)

- `navrl_iris` model and airframe 10091 (S3c).
- `navrl_quad_s4` model (visuals removed) and airframe 10092, which is 10090's file with only its @name changed (S4).
- The PX4 binary's SHA-256 is unchanged. All SHA-256s are in [`validation.json`](validation.json).

## Results (attempt01, 2026-11-07)

### Records and sources

All files are in this folder.
- [`execution_record.json`](execution_record.json) holds:
  - bindings, the smokes and the chain times (`chain`);
  - a per-scene table for each unit;
  - a SHA-256 manifest of every `steps.npy` and `sensor_steps.npy`, and of the `px4.log` files quoted below
    (`raw_manifest_sha256`);
  - a block `descriptive_not_preregistered`. That block is outside both preregistrations' reported lists. It is
    exploratory and only says what to look at next.
- [`reading_s3c_attempt01.json`](reading_s3c_attempt01.json) and [`reading_s4_attempt01.json`](reading_s4_attempt01.json):
  byte copies of each runner's fixed `--analyze` output.
- [`make_results.py`](make_results.py): the script that built them. Its SHA-256 is in the record.

**Where the numbers come from:**
- Measured numbers come from these files.
- Design values come from the preregistrations: vehicle masses and T/W, the 0.14 m crash rule, the 0.235 m iris half
  span and the 0.10 m ground rule.
- The rotor reaches come from the model SDFs, as the record's `ROTOR_REACH_M` comments say.
- The real vehicle's description comes from D-2026-11-06-1.

Raw data stays in `~/motar_tmp/s3c_20261106/attempt01/` and `~/motar_tmp/s4_20261106/attempt01/`.

### Execution

- **Commit and host:** both units ran at HEAD `5e58984`, with a clean tree before and after. They used
  `CUDA_VISIBLE_DEVICES=""`, one thread, cores 6, 7, 12 and 13, and `nice` 10. Kernel faults were 0 before and 0
  after.
- **Chain** (record `chain`):
  - S3c smoke01, then attempt01 (11:33:57–12:33:10);
  - the S4 plugin built in a clean directory (`9b3109af…` from source `661a4d71…`);
  - S4 smoke01, then attempt01 (12:34:12–13:35:44).
- **Smokes:**
  - S3c smoke01 (iris, env 0): COMPLETE and VALID, a capture in 35 steps.
  - S4 smoke01 (ground, env 0): COMPLETE and VALID, a bar crash at step 79. The smoke gate requires COMPLETE and
    VALID, not a capture. attempt01 flew the same scene to a bar crash at step 54.
- **Concurrency:** the evidence is limited to three facts:
  - each episode's idle check, on cores 6, 7, 12 and 13 only;
  - S4's GPU check at launch;
  - CAM-K1 training starting at 13:46:12, after the chain ended (its chain log).
- **Timeout (deviation, disclosed):** both preregistrations fix `timeout -k 30 7200`. The chain used 10,800 s, as
  noted in the r2 repairs and `validation.json` but in no amendment. It had no effect: the attempts took 59 and
  61 minutes.
- **Approval provenance (disclosed):**
  - The user approved S3c and S4 on 2026-11-06 at 16:42 KST: "after the review ends PASS or PASS_WITH_LIMITS and is
    committed, run with that commit after S3b". The units were then at `0d44972`.
  - At 17:11 I told the user that the dataman repair would be added. The user answered "오케이 다음 내용
    진행해줘" ("OK, carry on").
  - Amendment 1 changed the runners in exactly two ways: dataman removal, and RustDesk allowed in S4's GPU check with
    a refusal when `nvidia-smi` fails. Its review read PASS, and it was committed in `5e58984`.
  - I then re-issued both approval files for `5e58984` and left `approved_at` at 2026-11-06. The SCOPE strings are
    unchanged.
  - No separate user confirmation of the re-issued commit was asked for.

### S3c v1: PX4's iris, and a one-step command delay (40 scenes each, 70 bars)

| Arm | VALID | Capture | Crash (95 % CI) | Δcapture vs Isaac | Δcrash vs Isaac | Same outcome as Isaac | Same outcome as S3b |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `iris` (navrl_iris, no delay) | 40/40 | 40 (100 %) | 0 (0–8.8 %) | 0.0 pp | 0.0 pp | 40/40 | 40/40 |
| `delay1` (navrl_quad, one-step delay) | 40/40 | 37 (92.5 %) | 3 (1.6–20.4 %) | −7.5 pp | +7.5 pp | 37/40 | 37/40 |

- **References:** Isaac captured all 40 scenes. So did S3b attempt01 (its ledger bound by SHA-256 `97b94dfd…`,
  FEASIBLE_IN_SITL).
- **Iris footprint, as preregistered:** no step came within the 0.235 m half span; 40 episodes, none missing.
- **Iris rotor reach** (descriptive, inferred contact risk):
  - navrl_iris's rotor discs reach about 0.384 m from the centre.
  - 4 iris episodes came within that distance: envs 1, 9, 31 and 40, all captures.
  - So "no step within 0.235 m" does not mean that iris's rotors stayed clear.
  - In `delay1`, 2 non-crash episodes came within navrl_quad's 0.31 m (envs 21 and 41).
- **Median capture time:** 36.5 steps (`iris`) and 39 steps (`delay1`). On the scenes both captured, S3b took 34.5
  and 34 steps.
- **The three `delay1` crashes:** env 0 (bar, after 44 steps), env 40 (bar, after 47 steps) and env 29 (altitude,
  after 100 steps). Steps are counted from 1 here; `k` below is the 0-based step index.
  - **Env 29:**
    - The bar clearance fell to 0.146 m at k = 49, just above the 0.14 m crash rule.
    - PX4's EKF-versus-truth horizontal error passed 1 m at k = 58 and reached 14.3 m.
    - `px4.log` shows "Navigation failure!" and "Failsafe activated". The vehicle then descended to 0.10 m.
    - This is the pattern S3b's results review r1 called a likely uncounted rotor contact. It is inferred, since no
      contact sensor ran.
    - The 14.3 m in `reading.json`'s `delay1` `ekf_xy_err_max_m` is this episode.
  - **Envs 0 and 40:** `px4.log` shows "Attitude failure (roll)" and "Failsafe activated". The log has no
    timestamps.
- **Timing and state:**
  - Lateness at most 0.024 s, truth age at most 0.019 s, real-time factor 0.996–1.005.
  - Compute at most 46 ms (`iris`) and 12 ms (`delay1`). No parameter problems.
  - The per-episode EKF maximum had a median of 0.20 m (`iris`) and 0.23 m (`delay1`). One episode in each arm was
    above 0.5 m: 0.54 m (`iris` env 40, a capture) and the 14.3 m above.

**What follows:**
- **The iris arm:** on these 40 scenes, PX4's own iris left every outcome unchanged, with S3b's velocity loop. Iris
  is a heavier vehicle with less thrust margin: 1.5 kg and T/W about 1.9, against navrl_quad's 0.25 kg and 3.26
  (prereg).
- **The delay arm:** with every command one 0.1 s control step late, 3 of the 40 scenes S3b captured ended in
  crashes, one boot per scene.

**What does not follow:**
- **No label.** S3c has no margin, so neither "acceptable" nor "not acceptable" is claimed.
- **Small counts.** The crash intervals are wide, and the delay arm used navrl_quad, not iris.
- **Boot-to-boot variation.** Only S3b block C (10 of 10 repeats) bears on it.
- **The real vehicle.** D-2026-11-06-1 describes it as about 0.5 kg with an OAK-D Pro. That supersedes the S3c
  prereg's ref5in description, and neither model is that vehicle.

### S4 v1: Gazebo's own LiDAR, RGB and depth (70 bars)

| Arm | VALID | Capture | Crash (95 % CI) | Timeout | Δcapture vs Isaac | Δcrash vs Isaac | Same outcome as Isaac |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `ground` (preregistered 0.10 m ground filter applied) | 39/40 | 20 (51.3 %) | 17 (27.8–60.4 %) | 2 | −48.7 pp | +43.6 pp | 20/39 |
| `raw` (no ground filter) | 19/20 | 5 (26.3 %) | 11 (33.5–79.8 %) | 3 | −73.7 pp | +57.9 pp | 5/19 |

- **References:** Isaac captured every VALID scene, and so did S3b attempt01: 39 of 39 for `ground`, 19 of 19 for
  `raw`.
- **INVALID:** one LATE episode per arm, against the 0.15 s bound with the frame age counted.
  - `ground` env 37 reached 0.160 s, an episode that captured.
  - `raw` env 20 reached 0.152 s, an episode that timed out.
  - The LiDAR frame was the late one (review r1).
  - Neither stop rule triggered: 2 of 60 INVALID, never two in a row.
- **Crash types:** all 17 `ground` crashes hit a bar. In `raw`, 10 hit a bar and 1 was an altitude crash.
- **Camera recall in Gazebo** (prereg definition):
  - The colour rule fired on 63.4 % (`ground`, 1771 of 2795) and 47.9 % (`raw`, 541 of 1130) of the steps where
    MOTAR's shadow renderer shows at least 2 target pixels.
  - The shadow's camera is yaw-level while Gazebo's tilts with the body, so the recall mixes detection with mount
    geometry.
  - LiDAR-only detections on those steps: 42 and 16.
- **Target pixels on those steps, median over episodes:** the colour rule passed 5 Gazebo pixels (`ground`) and 1
  (`raw`), where the shadow had 26 and 10.
- **LiDAR and depth against the shadow:**
  - The per-step median over all beams is 0.0 m in both arms, and the depth median 0.0 m (`ground`) and 0.015 m
    (`raw`). Those medians are dominated by beams with no return in either sensor.
  - The per-step maximum over beams had a median of 10.3 m (`ground`) and 10.4 m (`raw`) (descriptive), so on most
    steps at least one beam disagrees.
  - These statistics cannot separate mount tilt, frame age and real sensor disagreement. Attitude was not logged.
- **Frame age:** median 0.028 s; maximum 0.144 s over the VALID episodes, which is also the arms' maximum lateness.
- **Clock and frame order** (descriptive):
  - Gazebo's simulation time minus PX4's state time was 0.00–0.02 s.
  - After step 0, no RGB frame was older than the latest target move. At step 0, the first frame predates the
    target's placement in 3 `ground` and 2 `raw` episodes.
  - This closes review r2's two open LOWs.
- **Timing and state:**
  - Real-time factor 0.998–1.003; compute at most 42 ms; no parameter problems; 0 hover-guard failures.
  - The per-episode EKF maximum had a median of 0.30 m (`ground`, 5 episodes above 0.5 m) and 0.24 m (`raw`, 1).
  - The largest values were 2.59 m (`ground` env 28) and 6.42 m (`raw` env 0), both crashes.

**The ground filter left some ground in** (results review r1 MAJOR-2; descriptive, inferred):
- **At hover (step 0):** 11 of 39 `ground` episodes show a LiDAR maximum difference of 6.0–6.6 m. That is the
  signature the `raw` arm shows in 18 of 19 episodes: a ground hit at about 5.5–5.9 m on the −10° row (review r1).
- **Likely cause** (inferred): the 0.10 m rule has almost no margin there, and the component smoke measured the −10°
  row's ground hit short of flat-ground geometry.
- **In flight:**
  - 1278 of the 7945 `ground` steps removed no depth pixel as ground.
  - 708 steps, in 38 of 39 episodes, had a per-step depth median difference above 1 m.
  - Body tilt can cause both, so how well the filter worked in flight is NOT_RECORDED.

**Descriptive, not preregistered** (`execution_record.json`, `descriptive_not_preregistered`):
- **The same 19 scenes in both arms:** `ground` captured 11 and `raw` 5. Every `raw` capture was also a `ground`
  capture. Of the other 6 `ground` captures, `raw` crashed in 4 and timed out in 2.
- **Episodes in which the colour rule never fired** while the shadow had the target in view:
  - 5 of 39 in `ground` (3 crashes, 2 timeouts) and 3 of 19 in `raw` (2 crashes, 1 timeout);
  - median shadow target size 4 px and 3 px, a distant target;
  - none of them captured.
- **Detection and crashes:**
  - 14 of the 17 `ground` crashes (9 of 11 `raw`) were in episodes with at least one camera-detection step. That
    does not show the target was detected at the crash.
  - In the last 10 steps before the crash, 9 of 17 (8 of 11) had a detection.
- **Slower captures:** on the 20 scenes captured in both, `ground` took a median of 179.5 steps (10 Hz), against
  33.5 in S3b. On 5 such scenes, `raw` took 81 against 30.
- **Crash timing:** `ground` crashed after 21 to 381 steps (median nearest target distance 2.04 m); `raw` after 38
  to 480 (3.97 m).

**A candidate perception mechanism** (results review r1 MAJOR-1; inferred from the code, NOT_TESTED):
- F's camera-to-scan fusion blanks only the target pixels the colour rule passes. In `navrl_perception.py`, it
  replaces the depth under `_detect_rgbd`'s mask with no-return. `target_mask_backfill` is off, as in Frozen F.
- MOTAR's renderer paints the target flat, so all of it passes. On Gazebo's shaded sphere about 5 of 26 pixels pass.
- So the rest of the target would enter the static scan as an obstacle at the target's range, even on steps where the
  target is detected.
- That would fit the slower captures and the dodging near the target. It is not measured here.

**What follows:**
- **The capture rate fell.** Replacing MOTAR's idealised renderer with Gazebo's sensors lowered Frozen Policy F's
  capture rate on these 70-bar scenes:
  - from 100 % in Isaac and S3b;
  - to 51.3 % with the preregistered ground filter;
  - to 26.3 % without it.
- **Several things change together** in this swap:
  - the body-fixed, tilting mount;
  - the target's shading, which also changes how much of it the colour rule blanks from the obstacle scan;
  - the field of view and square pixels;
  - frame timing.
  - The depth range beyond the 10 m clamp probably does not reach the scan: columns at 10 m or more map to no-return.
    That is inferred.
- **Only the ground is separated, by the two arms.** On the 19 matched scenes the filter arm captured 11 against 5.
- **Inputs to S5** (prereg):
  - An onboard ground filter is needed. The raw arm is worse, and the preregistered 0.10 m filter left some ground
    returns.
  - A gap remains with the preregistered ground filter applied.
  - The camera path needs a detector whose mask covers the whole target, or a backfill. Otherwise the undetected
    target pixels may act as an obstacle (NOT_TESTED).

**What does not follow:**
- **Which factor causes the remaining crashes.** S4 did not separate the mount, the target appearance and its partial
  blanking, the timing, or the residual ground. A factorial follow-up with its own preregistration would be needed.
- **Real-camera detection.** The target here is a red sphere under F's colour rule. The real target is a drone, and
  its detector lineage (YOLOv5s on NPS and Det-Fly) is not connected.
- **The real vehicle and camera.** The OAK-D Pro's 66° × 54° colour camera, its stereo depth and the 0.5 kg vehicle
  (D-2026-11-06-1) were not modelled.
- **Camera-only flight.** CAM-K1 is a separate unit.
- **Real flight.** No claim is made.
- **Uncounted contacts** (inferred, as in S3b):
  - The 0.14 m crash rule can miss rotor contacts.
  - Two crash episodes show S3b's signature before the counted crash:
    - `ground` env 28: EKF error above 1 m from k = 131, the crash after 179 steps;
    - `raw` env 0: EKF error above 1 m from k = 105, true height down to 0.39 m, EKF error up to 6.42 m, the crash
      after 124 steps.
  - In `ground` envs 18 and 41, the EKF rise coincides with the final approach to the crash.
  - Non-crash episodes within navrl_quad's 0.31 m rotor reach: 5 in `ground` (envs 3, 17, 24, 25, 35) and 2 in
    `raw` (envs 7, 17).

## Results review r1 (Claude Opus 5.5, fresh context)

[RESULTS_REVIEW_r1.md](RESULTS_REVIEW_r1.md) is a copy of
`~/motar_tmp/s3c_s4_results_20261107/results_review_r1/REVIEW.md`. Its probes stay in the `probes/` folder next to
it.

**Verdict:**
- **S3c PASS_WITH_LIMITS.**
- **S4 PASS_WITH_LIMITS, provided MAJOR-1 and MAJOR-2 are fixed before install.** Both are fixed above.
- **No BLOCKER.**

**Checks:**
- authorization and envelope;
- the ledger in schedule order, with validity recomputed (0 mismatches) and one binding core per arm;
- every number of both readings and the draft recomputed independently (0 mismatches);
- the staged files are byte copies with the right columns;
- harness plausibility: hover LiDAR within 0.03–0.07 m in 4 episodes, clocks within 0.02 s, no self-occlusion at the
  start.

**Findings, all repaired above:**
- **MAJOR-1:** the detection split was over-read. Repaired with the definition, the last-10-step counts and the
  partial-blanking mechanism.
- **MAJOR-2:** "ground removed" overstated the arm. Repaired with the arm label and the residual-ground disclosure.
- **MINOR-1:** the medians misled; the per-step maximum is added.
- **MINOR-2:** the evaluative detector claim is replaced by the numbers.
- **MINOR-3:** the matched ground/raw scenes are added.
- **MINOR-4:** the contact evidence is added.
- **MINOR-5:** iris's 0.384 m rotor reach is added.
- **MINOR-6:** the approval provenance is disclosed.
- **MINOR-7:** the timeout deviation is disclosed.
- **MINOR-8:** sources are cited, and the NPY/log SHA manifest and chain times are added.
- **LOW-1 to LOW-6:**
  - causal shorthand and intervals;
  - the INVALID outcomes;
  - the r2 LOWs closed;
  - the 10 m clamp factor;
  - the script copied;
  - the concurrency scope.

The repaired text was not reviewed again.
