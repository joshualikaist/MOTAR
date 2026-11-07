# S3c v1 and S4 v1 attempt01: results review r1

**Reviewer:** fresh-context Claude Code subagent, **Claude Opus 5.5**, 2026-11-07 13:53–14:35 KST.
**Mode:** read-only. I wrote only into this folder. I ran no tests, CUDA, Gazebo, PX4, rendering or `--analyze`.
Probes are short NumPy/JSON reads, run with `CUDA_VISIBLE_DEVICES="" OMP_NUM_THREADS=1 nice -n 19`, under one
CPU-minute in total. They are in `probes/`, each with its `*_stdout.txt`. I wrote my own recompute and imported no
runner or analyzer code.

## Verdict

| Record | Verdict | Why |
| --- | --- | --- |
| **S3c v1** | **PASS_WITH_LIMITS** | Every number recomputes exactly. Integrity is clean. The wording stays inside the prereg except for MINOR items (iris rotor reach, approval provenance, timeout, sources). |
| **S4 v1** | **PASS_WITH_LIMITS**, provided MAJOR-1 and MAJOR-2 are fixed before install | Every number recomputes exactly, integrity is clean, and I found no sign of a gross harness fault. Two passages over-read the descriptive split and the ground filter. |

There is no BLOCKER. Neither unit has, or should get, a pass/fail label. The draft adds none.

## What I read

- **Preregistrations:** S3c v1, S4 v1, amendment 1, and S3b formal v1 (reference).
- **Code:**
  - runners: `tools/sitl_s3c.py`, `tools/sitl_s4.py`, `tools/sitl_s3b_formal.py`. In `sitl_s3b_formal.py` I read `run_unit`, `episode_valid`, `_fly` (steps.npy columns), the analysis functions and `remove_dataman`;
  - S4 pieces: `tools/sitl_s4_gazebo.py` (conversions, ground removal, shadow) and `tools/sitl_s4/motar_s4_bridge.cc`;
  - helpers: `tools/sitl_s4_model.py` (sensor poses) and `tools/sitl_dev08_attempt.kernel_faults`;
  - perception: the camera-fusion path of `aerial_gym/task/navrl_task/navrl_perception.py` (`_detect_rgbd`, `_fuse_static_and_extract_obstacles`, `camera_no_return_to_lidar_range`, the mask backfill), and `navrl_detector.render_raw_rgbd`.
- **Prior evidence:** `docs/evidence/sitl_2026-11-05/s3c_s4/` (README, REVIEW_r1/r2, validation.json, s4_component_smoke.json) and `s3b_formal/` (README, RESULTS_REVIEW_r1).
- **Approvals:** both `execution_approval.json`.
- **Chain:** `run_s3c_s4.sh`, `chain.log`, the analyze logs and `HANDOFF_running_s3c_s4_camk1.md`.
- **Raw records:** `attempt.json`, `final.json`, `episodes.jsonl`, `reading.json`, and every VALID episode's `steps.npy`/`sensor_steps.npy`. I read `px4.log` for S3c delay1 envs 0, 29 and 40.
- **Draft:** `README_results_draft.md`, `make_results.py` and `staged/*`.
- **Rules:** `AGENTS.md` (research rules, "Numbers travel with their source", wording limits, host-fault rule) and the 5e58984 WORKLOG entry.
- **Commit diff:** `git diff 0d44972 5e58984` for the S3c/S4 runner sources.

## Checks

### 1. Authorization and envelope: PASS (S3c), PASS (S4), with MINOR-6 and MINOR-7

- **Approvals:**
  - Both approvals name commit `5e58984c…`.
  - The approval `scope` strings equal each runner's `SCOPE` and each `attempt.json.scope`, for both smoke01 and attempt01.
  - The approval file SHA-256s (`abef8497…` S3c, `6ed39b14…` S4) equal `attempt.json.approval_sha256`.
- **Git state:**
  - `attempt.git` = `final.git_after` = `{head 5e58984, dirty ""}` in all four records.
  - HEAD is still `5e58984` and the tree is clean now.
- **Order and retries:**
  - `chain.log` shows the order S3c smoke01 (11:33:08–11:33:56), attempt01 (11:33:57–12:33:10), the plugin build (`9b3109af…` from `661a4d71…`), S4 smoke01 (12:33:19–12:34:12), then attempt01 (12:34:12–13:35:44).
  - Each unit has exactly smoke01 and attempt01, so there were no retries.
  - Each smoke matches its attempt's HEAD and source bytes. S4's smoke and attempt bind the same plugin, plugin source, model SDF and display (:1).
- **Environment:** `CUDA_VISIBLE_DEVICES=""`, OMP/MKL 1, affinity [6, 7, 12, 13], nice 10. No `NAVRL_/PX4_/S3B_/GAZEBO_` keys were inherited.
- **Kernel faults:** 0 → 0 in all four finals and in every ledger row. The current boot's journal has 0 lines matching the fault patterns.
- **Schedule:** both units' schedules equal my own rebuild from the zero-action ledger. There are 122 eligible 70-bar scenes; S3c ran iris ×40 then delay1 ×40; S4 ran ground ×40 then raw ×20.
- **Amendment 1:** `git diff 0d44972 5e58984` changes the runners only by `remove_dataman` and the S4 GPU allow-list with the nvidia-smi rc check. `dataman_removed_bytes` = 7,866,640 in all 140 rows.
- **Deviations not in the draft:** MINOR-6 (approval re-issue) and MINOR-7 (timeout 10800 s, against the prereg's 7200 s).

### 2. Ledger integrity: PASS

- **Order:** rows are in schedule order with index 0..N−1 (80 and 60).
- **`isaac_outcome`:** equals the Isaac ledger in every row.
- **Validity, recomputed with 0 mismatches.** The rule is: the outcome is in {capture, crash, timeout}; every read-back is within 1e-6 of the arm's PARAMS (iris: the 3 velocity-loop values; delay1 and S4: S3b's 10); late_max ≤ 0.15 s; and the binding core is checked within each arm.
- **Extra read-backs** are constant in all rows: YAWRATE_P 0.11, HGT_REF 2, XY_VEL_MAX 3.6.
- **Binding:** there is one distinct binding core per arm (61 files in S3c, 59 in S4 including `s4_plugin`).
- **Host idle:** `host_idle_before.ok` in all 140 rows (minimum core idle 0.94). There were no HOST_BUSY episodes, no errors, and the receiver thread joined in every row.
- **Stop rules, none triggered:**
  - S3c: 0 INVALID.
  - S4: 2 INVALID of 60 (ground env 37, LATE 0.160 s; raw env 20, LATE 0.152 s). The maximum consecutive INVALID run is 1, and episode 0 is VALID.
  - The cause of each LATE was the LiDAR frame's age, not the state sample's lateness.
- **S4 lateness:** recomputed as max(state lateness, frame age), with frame age = `sensor_steps[:,15] − min(sensor_steps[:,1:4])`. `late_max_s` equals the frame-age maximum wherever that is the larger term.
- **INVALID handling:** both INVALID episodes are excluded from the VALID denominators, as the prereg says. Their outcomes (a ground capture and a raw timeout) are not stated in the draft (LOW-2).

### 3. Independent recompute: PASS (0 mismatches)

My `probes/recompute.py` reproduced the following from the raw files:
- every value in both `reading.json` files: counts, rates, Δ against Isaac, agreement, the S3b matched table, timing maxima, the iris footprint (40 episodes, 0 within 0.235 m, 0 missing), recall 1771/2795 = 0.6336 and 541/1130 = 0.4788, LiDAR-only counts 42 and 16, the median blocks, frame age max 0.144, and hover-guard failures 0;
- every per-episode `sensor` field, from `sensor_steps.npy`;
- every number the draft quotes: capture-step medians, the both-captured medians, crash steps and reasons, crash target-distance medians, EKF medians, counts above 0.5 m and top values, the zero-detection split, the smoke results, the delay1 env 29 numbers (0.146 m at k = 49, EKF above 1 m at k = 58, 14.297 m) and the `px4.log` lines for envs 0, 29 and 40.

The S3b reference ledger hashes to `97b94dfd…`, as recorded. The analyzer's "median over episodes" is the upper median; it equals the true median for every quoted value.

### 4. `make_results.py` and the staged files: PASS, with MINOR-8

- **Columns:** they match `_fly`'s `steps_log` layout: 0 k, 1 PX4 time, 2–4 EKF position, 5–7 velocity, 8 yaw, 9–12 action, 13–15 governor, 16–18 target, 19 distance, 20 clearance, 21 compute, 22 visible, 23–25 true position, 26 true yaw.
  - `CLEAR_COL = 20`: correct.
  - EKF against truth, `s[:,2:4] − s[:,23:25]`: correct; it equals `ekf_vs_truth_xy_err_m.max` to 1e-9 in every episode.
  - `z_true = s[:,25]`: correct.
- **Byte copies:** `cmp` confirms both staged readings are byte copies.
- **The staged `execution_record.json` agrees:**
  - the raw SHA-256s (attempt, final, episodes and reading of both units, and the smoke finals), the script SHA (`286d5177…`, equal to the current file) and the S3b record SHA (`974a3829…`) all match;
  - `per_scene` has 0 field mismatches against the ledgers;
  - the whole `descriptive_not_preregistered` block equals my recompute.
- **Gap:** the NPY and log files behind the per-scene table and the descriptive block are not SHA-bound (MINOR-8).

### 5. Wording against the preregistrations: LIMITED_VALIDITY (S4: MAJOR-1, MAJOR-2, MINOR-1 to MINOR-4; S3c: MINOR-5, LOW-1)

- **Labels:** no pass/fail label is added. The "descriptive, not preregistered" block is labelled exploratory. Real-flight, real-camera and real-vehicle claims are explicitly disclaimed.
- **Mechanisms:** the uncounted-contact passages are marked as inferred.
- **S3c:** within the prereg. "turned 3 of the 40 captures into crashes" is causal shorthand (LOW-1).
- **S4:** the two "Inputs to S5" phrases and the descriptive conclusion go beyond the data (MAJOR-1, MAJOR-2, MINOR-2).
- **Causal attribution:** S4 does not attribute the loss to a single factor ("Which factor causes the remaining crashes" is correctly listed as not following).

### 6. Physical plausibility of the S4 drop: LIMITED_VALIDITY

**Verified (no sign of a gross harness fault):**
- **LiDAR beam mapping at hover.**
  - At step 0 the per-step max |Gazebo − shadow| over all 288 beams is 0.03–0.07 m in 4 of 39 ground episodes, under 2.1 m in 15 of 39, and below 1.2 m in 10, in 70-bar scenes.
  - A mirrored or shifted mapping could not give that.
  - A cluster of 2.0–2.09 m in 5 episodes is consistent with the target sphere's size difference (0.20 m shadow against 0.15 m Gazebo) at its 10 m start distance.
- **Clocks.** The plugin's Gazebo `sim_now` minus PX4's state time is 0.000–0.020 s (median 0.004) in both arms. PX4 lockstep time and Gazebo time agree, which closes review r2's open LOW on the clock offset. The frame ages are therefore real.
- **Target and frame order.** In 0 of 12,385 steps was the RGB frame older than the latest target move, which closes r2's other open LOW.
- **Frame age.** Median 0.028 s. Ages above 0.1 s occur on 13 of 7,945 ground and 15 of 3,828 raw steps, and the LiDAR is always the oldest frame.
- **Self-occlusion at the start.** The hover-guard minima over all episodes are LiDAR 0.72 m and depth 0.77 m (ground), and 0.72 m and 1.48 m (raw). Nothing came near 0.35 m, so the vehicle did not see itself at the start.
- **Ground removal at hover.** It removed 48–69 LiDAR beams and 3,437–6,347 depth pixels per frame, and the depth median difference is 0.0 in 37 of 39 frames. See MAJOR-2 for what is not removed.
- **Real-time factor, compute and EKF** are in S3b's range, except in crash and contact episodes.
- **Crash traces.**
  - Ground env 0 and env 2 flew into a bar over about 10 steps at a commanded speed of about 2 m/s, with the target outside the shadow camera's view the whole time (`probes/s4_traces_stdout.txt`).
  - Ground env 28 and raw env 0 show the S3b-style contact signature well before the counted crash (MINOR-4).

**Not verifiable from the logged data:**
- **In-flight sensor agreement per beam or pixel.** Only the per-step median and max of |Δ| are logged.
  - The max is above 3 m on 99.8 % of ground-arm steps (median 10.1 m).
  - Approaching a bar, it tracks 12 m minus the nearest bar's range. For example, ground env 0 at k = 46–53: 11.12 → 12.00 m, while the clearance fell from 0.75 to 0.13 m.
  - So on almost every step at least one beam disagrees about the nearest bar. In which direction is not logged.
  - Frame age and body tilt at bar edges are a sufficient explanation, but that is inferred. A missed near bar cannot be excluded from these statistics, though the median of 0 shows most beams agree.
- **Attitude.** It is not in `steps.npy`, so the size of the mount tilt is unknown.
- **What the governor saw** was not logged.
- **The target-as-obstacle mechanism** in MAJOR-1 is NOT_TESTED.

### 7. Missing disclosures

These are MINOR-1 to MINOR-8 and LOW-1 to LOW-6 below.

## Findings

### MAJOR-1 (S4): the descriptive detection split is over-read, and a perception mechanism contradicts the reading

- **Draft text:**
  - "So a lost target accounts for some of the drop, but most crashes happened with the target detected";
  - in *Inputs to S5*: "a large part of the gap remains after the ground is removed, **with the target detected**".
- **What the split measures:** whether an episode had **at least one** camera-detection step among the shadow-visible steps (14 of 17 ground crashes, 9 of 11 raw).
  - It does not say the target was detected at or near the crash.
  - Only 9 of 17 ground crashes and 8 of 11 raw crashes had any camera detection in their last 10 steps (`probes/s4_traces_stdout.txt`).
  - "Accounts for" is causal wording for a descriptive split.
- **Code-level mechanism (inferred, NOT_TESTED):**
  - F's camera-to-scan fusion blanks only the pixels the colour rule passes. In `navrl_perception.py`, `depth_no_target = where(target_pixels, lidar_max, raw_depth)` uses the `_detect_rgbd` mask, and `target_mask_backfill` defaults to False and is not set in the G8 receipt. This is the historical path the code labels as Frozen F's.
  - MOTAR's renderer paints the target into depth as a flat colour that passes completely (`render_raw_rgbd`).
  - On Gazebo's shaded sphere, a median of 5 of 26 pixels passed. The remaining target pixels would enter the static scan as an obstacle at the target's range, even on "detected" steps. The code's own comment at the backfill names this "phantom obstacle" effect.
  - This is a candidate contributor to the slower captures (179.5 against 33.5 steps) and to dodging near the target. "With the target detected" would steer S5 away from it.
- **Fix:**
  - Replace both phrases with the definition: "14 of the 17 `ground` crashes (9 of 11 `raw`) were in episodes with at least one camera-detection step; this does not show the target was detected at the crash. In the last 10 steps before the crash, 9 of 17 (8 of 11) had a detection."
  - Add the partial-blanking mechanism as inferred from the code and NOT_TESTED.
  - Drop "with the target detected" from the S5 bullet.
  - Put the last-10-steps counts in the descriptive block, so that the numbers have a record.

### MAJOR-2 (S4): "with ground returns removed" overstates the `ground` arm

- **Step-0 evidence (hover, `probes/s4_step0_stdout.txt`):**
  - In 11 of 39 ground episodes, the max |LiDAR − ground-free shadow| is 6.23–6.54 m.
  - That is the raw arm's ground-return signature: 6.07–6.36 m in 18 of 19 raw episodes, a ground hit at about 5.5–5.9 m on the −10° row.
  - The likely cause is that the 0.10 m rule has almost no margin. A Gazebo return at about 5.7 m computes a hit height of about 0.09–0.10 m from the assumed height (truth + 0.09 m). The component smoke also measured the −10° row's ground hit short of flat-ground geometry: 5.48 m against 5.76 m at a 1 m rig height. This is inferred.
- **In flight:**
  - 16 % of ground-arm steps removed 0 depth pixels as ground, and 8.8 % (in 38 of 39 episodes) had a per-step median |depth − shadow| above 1 m.
  - Body tilt can cause both, so how well the filter worked in flight is NOT_RECORDED.
- **Fix:**
  - Label the arm "preregistered 0.10 m ground filter applied", not "ground removed".
  - Disclose the step-0 residual (11 of 39) as inferred.
  - Reword "a large part of the gap remains after the ground is removed" to "…with the preregistered ground filter applied; the filter left some ground returns (see above)".

### MINOR-1 (S4): the "LiDAR and depth difference 0.0 m" medians mislead

- The per-step median over 288 beams is dominated by beams that are no-return in both sensors.
- The per-step max is 9–12 m on nearly every step (see check 6).
- **Fix:** report the max as well (descriptive, with a record), and state that these statistics cannot separate mount tilt or frame age from sensor disagreement.
- **For a later unit:** log per-step counts of beams and pixels with |Δ| > 0.5 m, the minimum Gazebo range, and the true attitude.

### MINOR-2 (S4): "the colour rule is not a usable detector on Gazebo's shaded sphere"

- This is an evaluative claim with no preregistered criterion.
- **Fix:** give the numbers instead: "fired on 63.4 % (`ground`) and 47.9 % (`raw`) of the steps where MOTAR's renderer would show ≥ 2 target pixels, with a median of 5 of 26 (1 of 10) pixels passing".

### MINOR-3 (S4): use the matched-scene ground/raw comparison

- On the 19 scenes VALID in both arms, `ground` captured 11 and `raw` captured 5.
- Every raw capture was also a ground capture. Of the 6 other ground captures, 4 were raw crashes and 2 raw timeouts.
- **Fix:** add this to `descriptive_not_preregistered` and quote it in place of "51.3 % against 26.3 %, on different scene counts", or beside it.

### MINOR-4 (S4): the uncounted-contact sentence needs its evidence

- **Two episodes show the S3b-type signature:**
  - Ground env 28: clearance 0.25 m at k = 126; the EKF error rose from 0.04 m to 1.16 m by k = 131; the counted crash came at k = 178.
  - Raw env 0: clearance 0.21 m at k = 98; true z fell from 0.87 to 0.39 m and the EKF error reached 6.4 m; the crash came at k = 123.
- **Two do not:** in envs 18 and 41, the EKF rise coincides with the final approach to the crash.
- **Non-crash episodes within navrl_quad's 0.31 m rotor reach:** ground 5 (4 captures, 1 timeout) and raw 2 (`probes/rotor_reach_stdout.txt`).
- **Fix:** state these and keep "inferred".

### MINOR-5 (S3c): iris's rotor reach is larger than the reported 0.235 m half span

- In `navrl_iris.sdf`, the rotor collision cylinders have radius 0.128 m at (±0.13, ±0.22/0.20), which reach about 0.384 m.
- 4 of 40 iris episodes came within that distance: envs 1, 9, 31 and 40, minimum 0.286 m, all captures.
- In env 40 there was no disturbance at the closest pass, which was at the capture step (`probes/iris_reach_stdout.txt`).
- The prereg's 0.235 m report is correct as preregistered, but "no step came within 0.235 m" reads as "iris stayed clear".
- **Fix:** add the 0.384 m disc reach and the count of 4, as descriptive and inferred, consistent with the S3b MAJOR-1 disclosure. delay1 also has 2 non-crash episodes within 0.31 m.

### MINOR-6 (both): approval provenance is not disclosed

- **What happened:**
  - The user approved S3c and S4 at about 16:35 on 2026-11-06, for the reviewed units at `0d44972`.
  - The approval files were re-issued for `5e58984`, which carries amendment 1 (runner code changes: dataman removal, and an S4 GPU refusal relaxed to allow RustDesk). `approved_at` was left at 2026-11-06T16:35.
  - The WORKLOG lists the re-issue under "Approvals (user)". The handoff records the user's 2026-11-07 schedule instruction.
- **What is missing:** the draft is silent, whereas the S3b README disclosed its own approval-commit mismatch.
- **Fix:** one bullet stating the re-issue, the verified two-change diff, and the 11:10 instruction. I could not verify a separate user confirmation of the re-issue in the files.

### MINOR-7 (both): the timeout envelope deviated from the prereg

- Both preregs fix `timeout -k 30 7200`. The chain used 10,800 s.
- This is disclosed in the prep README (r2 repairs) and in `validation.json`, but not in an amendment or the draft. It had no effect: the runs took 59 and 61 minutes.
- **Fix:** disclose it in the Execution bullet.

### MINOR-8 (both): "Numbers travel with their source", and binding

- **Header claim:** the draft header says every number comes from the three record files. It does not:
  - design values (1.5 kg, T/W 1.9, 0.25 kg, 3.26, 0.14 m, 0.235 m) come from the preregs;
  - 0.5 kg and the OAK-D Pro's 66° × 54° come from D-2026-11-06-1, which also supersedes the S3c prereg's "ref5in 1.20 kg" description of the real vehicle;
  - the chain times come from `chain.log`, the `px4.log` text from logs, and "about 0.7 GB" from `du` (679 MB and 706 MB).
- **Binding claim:** "The record binds them by SHA-256" covers only four files per unit, but `per_scene` and the descriptive block come from the `steps.npy`/`sensor_steps.npy` files.
- **Fix:**
  - cite sources for the design values;
  - add to `execution_record.json` a per-episode SHA-256 manifest of `steps.npy` and `sensor_steps.npy` (about 280 small files), and of the `px4.log` files quoted;
  - put the chain times into the record;
  - copy `make_results.py` beside the record (LOW-5).

### LOW items

- **LOW-1 (S3c).** "turned 3 of the 40 captures into crashes" uses one boot per scene. Suggest: "with the delay, 3 of the 40 scenes S3b captured ended in crashes". S3b block C's 10/10 repeat is the only evidence on boot-to-boot variation. A Clopper-Pearson interval in the record would quantify "wide interval": 3/40 gives 1.6–20.4 %, and 0/40 gives 0–8.8 %.
- **LOW-2 (S4).** State the INVALID episodes' outcomes (ground env 37 capture; raw env 20 timeout) and that the LiDAR frame was the late one.
- **LOW-3 (S4).** Record the two r2 LOWs closed in check 6: the clock offset is 0–0.02 s, and 0 RGB frames predated the target move.
- **LOW-4 (S4).** The factor "obstacle depth without the 10 m clamp" probably does not reach the static scan, because `camera_no_return_to_lidar_range` maps columns ≥ 10 m (`camera_obstacle_max_range`) to no-return. This is inferred. Say so or drop it.
- **LOW-5.** `make_results.py` lives outside the repository. Copy it into the evidence folder so its recorded SHA can be checked.
- **LOW-6.** "No other job ran on the host" is supported by:
  - the per-episode idle check, which covers cores 6, 7, 12 and 13 only;
  - S4's GPU check, which ran once, at launch;
  - the handoff's CAM-K1 start time (13:46:12, after the chain ended at 13:35:44).

  Scope the sentence to this evidence.
- **Operational, not a record issue.** `/` has 2.4 GB free (98 %) while CAM-K1 trains.

## Must be in README and WORKLOG before install

1. MAJOR-1 and MAJOR-2: rewording, and the residual-ground and partial-blanking disclosures, marked inferred and NOT_TESTED where they are.
2. MINOR-3, MINOR-4 and MINOR-5: matched ground/raw scenes, the contact evidence and iris's rotor reach. Each needs a machine-readable record first.
3. MINOR-6 and MINOR-7: approval re-issue and timeout.
4. MINOR-8: source attribution, and an NPY/log SHA manifest in `execution_record.json`.
