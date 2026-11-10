# S4R-1 v1: Gazebo sensor repeatability diagnostic (preregistration, 2026-11-10; NOT_RUN)

- **Status:** NOT_RUN at its commit. The independent review is recorded under *Review before the commit* at the end.
  This file is never edited after the commit; amendments go in a new dated file.
- **Code:** `tools/sitl_s4r1.py` (recorder, launch, analysis) and `tests/test_sitl_s4r1.py`.
- **Origin:** Codex prepared the code on 2026-11-10 in `~/motar_tmp/sitl_next_20261110/repeatability_prepare_r1/`,
  with a design draft (`DESIGN_DRAFT.md`). Claude Code wrote this text from the draft and the code's fixed constants.
- **Beads:** motar-783.11. The older motar-15g (dev08 replayed-sensor request) is not satisfied here.
- **User, 2026-11-10:**
  - asked to defer OAK-D;
  - asked to audit the existing data first, then plan and advance S5 and Gazebo repeatability;
  - later: "앞서서 코덱스가 친 명령을 보면서 어떤거를 해야되는지 짐작해서 동일하게 진행해줄 수 있어?".

  **Execution needs its own scoped approval** because it uses GPU rendering.
- **Why the LiDAR stack** (the user asked on 2026-11-10, "라이다 우리 못쓰잖아. 카메라만 쓰는거아니야?"):
  - The real vehicle is camera-only (D-2026-11-06-1). This unit qualifies the simulator, not the vehicle. It reruns the
    only SITL-ready stack, S4F-1's Policy F with camera and simulated LiDAR, where non-repeating scene outcomes were
    observed.
  - A camera-only policy has no SITL bridge yet (motar-783.4). Its comparisons come after this qualification.
  - The user chose to commit the unit as designed.

## 1. Why

- S4F-1 v1 (`8631059`) kept its original reading. Its negative reproducibility finding stands too:
  - In scene-matched comparison across runs, outcomes agreed in 17 of 39 cases. Chance gives about 17.1, and kappa
    is about zero.
  - Fixed-grid rates remain usable. Scene-level inference across runs does not.
- **The existing-data audit** (2026-11-10, `docs/evidence/sitl_2026-11-05/data_utility/`) found that the existing
  data:
  - **record:** sampled trajectory, requested action, delivered frame stamps and ages, and visibility (from
    `steps.npy` / `sensor_steps.npy`), plus high-rate truth and control state (from the ULogs);
  - **do not record:** raw rendered arrays, actor observations, the exact receive/send timeline or the target-applied
    frame counter.

  So the first point where two boots diverge between sensor and action cannot be reconstructed after the fact. This
  unit records exactly those missing streams, and only in a bounded diagnostic. It does not repeat any earlier outcome
  evaluation.

## 2. Fixed question

On the unchanged S4F-1 `lit_rendered` / ground stack:
1. At which recorded stage do fresh boots of the same scene first differ?
2. Does turning on the existing Gazebo world-contact subscriber coincide with a change larger than the variation within
   one condition?

**This unit does not compare** policies, vehicle models, gains, sensor fields of view or floor filtering. Frozen
Policy F stays frozen, and no OAK-D input is used or qualified.

## 3. Fixed design

- **Scenes:** env 0 and env 1, the first two eligible G8 r2 70-bar scenes in input order (`F1.scene_envs()[:2]`). If
  they are not `[0, 1]`, the tool refuses.
- **Treatment:** the world-contact subscriber, OFF or ON. Every new recorder is identical in both conditions. The ON
  condition is the whole contact path, not only the subscriber process (review r1 MINOR-1):
  - the owned `stdbuf -oL gz topic -u -e` subscriber process (250 Hz contact messages, logged to `logs/contacts.log`);
  - after the parameter read-back and before take-off, one subscriber liveness check and the floor-contact positive
    control (`floor_control`, which parses up to an 8 MiB tail of the log), plus a freshness rule: the control
    window's last message must lie within 0.5 s of PX4's newest LOCAL_POSITION_NED time (AGQ `session_control`'s rule);
  - the subscriber added to the owned-session liveness check (one combined check call, not a second one).
- **Schedule:** 3 repeats × 2 scenes × 2 conditions = **12 fresh boots**, each with a fresh rootfs and an owned session.
  - Interleaved by repeat, then scene.
  - Condition order alternates with (repeat + scene) parity: (OFF, ON) when even, (ON, OFF) when odd.
  - No retries, no replacements. `--schedule` prints the fixed list.
  - Each ledger row's `arm` is the condition (`contact_off` / `contact_on`); the historical cell name stays in `cell`
    (`lit_rendered`) and S4's arm in `s4_arm` (`ground`) (review r1 BLOCKER-1).
- **Fixed instrumentation smoke (before attempt01, not a result):** one boot of the schedule's first env 0 contact-ON
  item (repeat 0) into its own directory `~/motar_tmp/s4r1_20261110/smoke01_contact_on_env0/`, through the same code
  path (`--smoke`, `S3C.launch_unit`'s smoke mechanism). attempt01 refuses unless the smoke is COMPLETE with its one
  boot VALID at the same HEAD and source bytes (`S3C.smoke_problems`), and unless `instrumentation_smoke_problems`
  finds the boot is env 0 / contact ON with no recording problem, a recording manifest whose bytes still match, a
  verified ULog gzip, the contact log bound and the contact control passed, the Gazebo seed 4501 in the launched
  `gzserver` argv, the owned `px4-logger stop` returning 0, and `packets == steps + 1`. The smoke has no reading
  (`--analyze` refuses it). It runs once; if it fails, the unit stops and is amended, with no retry.
  **Coverage limit (review r2 MINOR-5):** env 0 ended after 43–46 steps in S4F-1, so the smoke is a short boot. The
  600-step path is therefore first exercised live inside attempt01: 600-step persistence, the worst-case disk per boot,
  pyulog extraction of a long ULog under the 60 s extractor timeout, the long contact audit, and recorder memory and
  garbage collection over a long boot. Before execution these are covered only on CPU: a test drives a synthetic
  600-step boot through the real persistence, checked-ULog (stub extractor), timeline, byte-counter and output-cap
  paths and compares two such boots at every fixed step up to 599; the real extractor took 0.72 s wall and 138 MB RSS
  on S4F-1's longest `lit_rendered` ULog (20.5 MB, attempt01 ep006; CPU only, 2026-11-10). A failure on any of these in
  attempt01 is handled by the stop rules of §5 (recording-invalid boot, unit stops, no retry), not by a rerun.
- **Stack:** the same as S4F-1 `lit_rendered`:
  - lit target, rendered floor, 0.10 m ground filter;
  - 87° body-fixed RGB / depth / `gpu_ray` mount;
  - `navrl_quad_s4`, the pinned historical plugin, airframe and bridge;
  - Frozen Policy F;
  - `S4F1.run_episode`, reused unchanged through owned-session, link/client and bridge/policy proxies.
- **Gazebo seed:** fixed at 4501 and passed explicitly at launch (`gzserver <world> --seed 4501`). The seed recorded in
  each row is read back from the owned session's receipt (`session.json`, the `gzserver` argv actually launched). This
  removes an unspecified seed as one source of difference. **It may not be claimed that this reproduces the old runs'
  random stream.** Whether `--seed` also controls the noise engines of PX4 SITL's sensor plugins is NOT_TESTED. The
  option order (after the world path) is first exercised live by the smoke.
- **Initial-condition binding:** within each scene, the scene binding and world SHA-256 must be the same in every boot.
  A change marks the boot `HARNESS_INVALID` (`SCENE_INITIAL_CONDITION_BINDING_CHANGED`). Actual start-state differences
  are preserved, not reset away.

## 4. Recording contract (additive)

- **Sensor packets:** every delivered packet, hover read included (packet 0 is the hover read; step k uses packet
  k + 1). Each packet record keeps:
  - its header as the pinned plugin sends it: `sim_now`, `lidar_t`, `rgb_t`, `depth_t`, `target_t` (the time the
    plugin applied the last target pose) and `target_count` (its applied-target counter);
  - monotonic request and return times, and the phase at the read;
  - `link_state_at_return`: the receiver thread's latest `pos`, `att`, `truth` and `hb` tuples at the packet's return
    (references to the receiver's immutable tuples, no extra reads or events);
  - raw payload signatures (shape, dtype, SHA-256 of the exact arrays) and the exact arrays in a compressed
    per-packet `.npz`.

  **Frame counters produced by the sensor itself are NOT_RECORDED by the pinned plugin.** The source-frame signature is
  the timestamp plus the payload digest, and the client packet ID is the received packet index. A packet count is not
  called a sensor production count.
- **Steps** (every control step): the state dictionary passed to the observation (estimated and rendered pose,
  velocity, quaternions, target position, bounds), the processed LiDAR / RGB / depth passed to perception, the
  observation, the raw actor output, the actor action, the governed command, yaw rate and previous action, the
  monotonic times of the observation and command, and `target_send_next`: the next step's target request sent after
  the command (xyz, plugin acknowledgement time, monotonic request and return).
- **MAVLink:** every arrival through `recv_match` and every `*_send`, with monotonic time, source IDs and sequence,
  fields and per-type counts, including COMMAND_ACK and STATUSTEXT. Messages consumed inside pymavlink's
  `wait_heartbeat` are NOT_RECORDED. A sent SET_POSITION_TARGET is not an accepted command. Accepted-setpoint
  telemetry comes only from the ULog. If it is absent there it is NOT_RECORDED, and no accepted-command causal
  conclusion is drawn.
- **Events** (`events.jsonl.gz`, gzip level 1, mtime 0): one sequence number across threads, phase markers, frame and
  target requests, policy calls, MAVLink sends and arrivals, the launch plan and the subscriber start.
- **Phase markers:** `boot`, `initialization_takeoff_hover` (link created), `preflight_parameters_verified` (after
  the read-back; arming, take-off and hover follow in the same phase), `control` (first observation),
  `post_terminal_landing` (the AUTO/LAND command) and `shutdown_complete` (marked after the session teardown). Only the
  control phase enters the divergence and outcome analysis.
- **Recorder overhead:** per proxy site (frames, control step, command, MAVLink receive), the call count, sum and
  maximum of `perf_counter` time, in `summary.json`. Reported, not a validity rule; the LATE rule (§5) bounds its effect
  on the loop.
- **Contacts:**
  - with the subscriber OFF, contacts are `NOT_RECORDED` (reason "subscriber OFF"), not zero;
  - with it ON, the floor-contact positive control (§3) must pass, and after the flight AGQ's whole-log
    `contact_audit` runs over the control window (1.0 s before the first to 0.5 s after the last control step);
  - the raw `logs/contacts.log` is kept and bound by SHA-256; the audit summary is `repeatability/contacts.json`.
- **ULog:** strict framing (header, record lengths to EOF, known subscriptions), the six required topics
  (`ULOG_TOPICS[:6]`) with at least two samples each and covering the control span (first sample no later than the
  first control step, last no earlier than 0.2 s before the last), the logger stopped by its owner before teardown,
  then a pyulog extraction of nine topics (PX4 env Python, `ulog_reconstruction.json.gz`), then S4F-1's verified gzip.
  Any failure keeps the native `.ulg` untouched and marks the boot recording-invalid; it never discards the physical
  record (review r1 MINOR-5).
- **Canonical timeline per step** (`timeline.json`): PX4 sim time, elapsed sim and monotonic time, the chosen packet
  and its header, stamps relative to the boot's step-0 packet `sim_now`, frame ages, packet-to-observation delay,
  estimated and truth position, actor action, governed command, the requested setpoint send, `target_send_next`, and the
  ULog samples bracketing the step for the controller setpoint, trajectory setpoint and truth attitude, with the
  bracket's alignment uncertainty. Per-packet acceptance is NOT_RECORDED. **The per-step nearest contact event is
  NOT_RECORDED**: the field says so; ON contact evidence is the whole-log audit in `contacts.json`.

## 5. Validity and stop rules (fixed)

- **A boot is operationally invalid** (S3b/S4's `SF.episode_valid` and `run_unit`, unchanged) when:
  - its outcome is not capture, crash or timeout (for example `HARNESS_INVALID` from S4's start-state or hover-guard
    check, a stalled PX4 clock, a missing sensor payload, a process exit or a watchdog abort, or `ERROR`);
  - the parameter read-back mismatches (`PARAM_MISMATCH`);
  - `late_max_s` is missing or above 0.15 s (`LATE`; the larger of state lateness and frame age per step);
  - its runtime binding differs from the first valid boot of the same condition (`BINDING_CHANGED`);
  - the cores were still busy after the inherited single 30 s wait (`HOST_BUSY`, see below);
  - the scene binding or world SHA-256 changed within its scene (`SCENE_INITIAL_CONDITION_BINDING_CHANGED`).
- **A boot is recording-invalid** (outcome replaced by `HARNESS_INVALID`, the physical outcome kept in
  `physical_outcome_before_recording_check`) on any of these:
  - a missing MAVLink stream (LOCAL_POSITION_NED, ATTITUDE, HIL_STATE_QUATERNION);
  - a ULog integrity failure (§4: count, framing, required topics, control coverage, parse, gzip verification);
  - step and packet counts that disagree (`steps == n`, `packets == n + 1`);
  - a missing control phase;
  - an actor record missing `obs`, `action`, `actor_output`, `governed` or `prev_action`;
  - ON: a failed contact control, or any `CONTACT_AUDIT` problem (contact log missing, parse or continuity gaps,
    missing steps or non-increasing times in the window, a partial tail, no control steps);
  - OFF: a contact record other than `NOT_RECORDED`;
  - the owned logger stop not returning 0 (`LOGGER_STOP`);
  - the Gazebo seed in the launched `gzserver` argv not 4501 (`SEED_NOT_BOUND`);
  - a write failure or output/disk limit while persisting (`RECORDING_WRITE`), or `DISK_LOW` / `OUTPUT_CAP` at the
    boundary after the boot.
- **The unit stops at the first operationally or recording-invalid boot** (`invalid_abort=0`), on a host fault, on a
  resource or recorder loss, or when the GPU is busy. There is no retry. **Waits:** the GPU check before each boot has
  no wait (a listed job raises and the unit stops). The CPU-core idle check is `SF.run_unit`'s, inherited unchanged:
  if the runner cores are busy it waits once for 30 s, then records `HOST_BUSY` without running, which stops the unit.
- **Bounds:**
  - 180 s per boot (owned-session deadline). S4F-1 ran with 240 s; the tighter value is disclosed (review r2 LOW):
    S4F-1's longest `lit_rendered` boot took 101.0 s;
  - **1,800 s outer limit** for attempt01 and **600 s** for the smoke, through a direct `timeout -k 60` parent whose
    exact argv the tool checks;
  - memory: the owned session's RSS watchdog, 8 GiB over all owned processes plus the runner (S4/S4F-1's mechanism;
    S4F-1 ran it with a 12 GiB cap, so this value is tighter; S4F-1 sessions peaked at 1.14 GiB). **No
    `RLIMIT_AS`:** the tool refuses a finite address-space limit, because it would be inherited by `gzserver`'s OpenGL rendering (Xorg's virtual size is 25.4 GB here) and PX4 (review r1
    MAJOR-3);
  - output cap 3 GiB per output directory (attempt01; the smoke has its own directory and cap);
  - launch refused below 4 GiB free disk (raised from 3.5 GiB by review r2 LOW: 4 GiB minus the 2.7 GB worst case
    leaves about 1.6 GB, above the 1 GiB write floor); a new boot refused, and a write refused, below 1 GiB free.

  **How limits are checked (review r1 MAJOR-2):** never inside the control loop by a tree walk. In flight, the
  session check adds at most one `statvfs` free-disk check per second; the frame read adds no check. The full-tree
  output size is measured only at boot boundaries (before each boot, after the flight before persisting, and after
  each boot); persistence then uses an incremental byte counter updated by the recorder's own writes, plus `statvfs`.
- **Time:** actual S4F-1 `lit_rendered` boots took 42.4–101.0 s (median 50.9 s, 2,327 s for 40). Scene-specific
  (review r2 LOW): env 0 about 44 s and 45 steps, env 1 about 101 s and 600 steps (timeout). Twelve boots (six of
  each) take about 15–17 min including persisting (estimated ≤ 20 s per 600-step boot), about 24 min if every boot
  took the observed maximum, within the 1,800 s outer limit. The smoke (env 0) takes about 1–3 min. A boot that
  reaches the 180 s deadline is invalid and stops the unit.
- **Disk:** per 600-step boot (worst case, every step recorded): processed per-step arrays and observation about
  235 KB per step (141 MB), raw packets 103,104 B each (62 MB), both uncompressed upper bounds before `.npz`
  compression; the ULog gzip about 7 MB (S4F-1's longest `lit_rendered` ULog, 20.5 MB raw); the ULog reconstruction
  about 3.6 MB gzipped (20.8 MB as JSON, measured on that ULog); events, timeline and JSON records a few MB. **Worst
  case about 225 MB per boot, 2.7 GB for attempt01 (under the 3 GiB cap) and 0.23 GB for the smoke.** Expected at
  S4F-1's mean of 182 steps: about 70 MB per boot; with the scene-specific step counts (six env-1 boots near 600
  steps, six env-0 boots near 45) about 1.5 GB in all (review r2 LOW). Free disk under `~/motar_tmp` was 6.31 GB
  (`df`, 2026-11-10); the 4 GiB launch floor and the 1 GiB minimum apply.
- **Host:** idle (no concurrent timing or GPU job), with the kernel fault count checked before and after each boot and
  for the unit.

## 6. Analysis (fixed, descriptive only)

`--analyze` reports **REPORTED_DESCRIPTIVE** only if every integrity check passes:
- the attempt is not a smoke and its status is COMPLETE;
- exactly 12 boots, in the scheduled order with the scheduled `arm`, env, repeat and condition;
- every boot valid with no recording problems;
- the same initial-condition binding within each scene and the same runtime resource binding in every boot;
- per boot: the recording manifest's bytes and SHA-256 still match (the `repeatability/` files plus `steps.npy`,
  `sensor_steps.npy`, `session.json`, `ulog_integrity.json`, `ulog_reconstruction.json.gz` and, ON, `logs/contacts.log`);
  the ULog gzip still decompresses to its recorded original (`F1.verify_ulog_record`); the seed read from `session.json`
  is 4501; the logger stop returned 0; contact semantics as in §5; `packets == steps + 1`;
- no kernel-fault rise and no tree change during the run.

Anything else reads **INCOMPLETE_INVALID**, with no comparisons.

**Pairs:** every pair of boots of the same scene (15 per scene: 3 within OFF, 3 within ON, 9 between conditions).
For each pair and each stage, at the same control step index k over the common steps:

| Stage | What is compared |
| --- | --- |
| sensor stamp (relative) | `sim_now`, `lidar_t`, `rgb_t`, `depth_t`, `target_t` minus the boot's own step-0 packet `sim_now`, in integer microseconds, and `target_count` minus step 0's (the microsecond maximum and the count maximum are reported apart) |
| sensor payload | the chosen packet's raw ranges, RGB and depth |
| state | the estimated position, velocity and quaternion and the rendered position and quaternion passed to the observation (keys `pos`, `vel`, `vquat`, `render_pos`, `render_vquat`; no rendered velocity) |
| perception input | the processed LiDAR, RGB and depth passed to perception |
| observation | the actor's observation |
| actor | the raw actor output |
| governed request | governed command, action and previous action |

- **Readings per stage:** the first step that differs exactly (SHA-256 of the arrays; exact relative stamps), the
  maximum absolute difference over the common steps, the first step whose maximum absolute difference exceeds a fixed
  reporting tolerance (relative stamps 0 µs and 0 counts; state 1e-3 in its units; every other stage 1e-6), and the
  maximum absolute difference at the fixed steps 0, 10, 25, 50, 100, 200, 300, 400, 500 and 599 where both boots
  reached them.
- **Non-finite entries (review r2 MAJOR-1):** raw LiDAR ranges and raw depth hold `+inf` / NaN (no hit), and two
  boots a few centimetres apart flip some beams between hit and no hit. For each differing array the maximum absolute
  difference is taken **only over entries finite in both boots**. Entries whose non-finite pattern differs (finite
  against ±inf or NaN, +inf against −inf, inf against NaN) are **counted separately** per stage and step:
  `nonfinite_mismatch_n` (total entries), `nonfinite_mismatch_steps`, `first_k_nonfinite_mismatch` and the count at
  each fixed step. Equal non-finite entries (same-sign inf, NaN with NaN) are equal and not counted. The first step
  above tolerance uses the finite maximum only; a non-finite mismatch still makes the step differ exactly. A step is
  **incomparable** only for a genuine shape mismatch or a missing array (`incomparable_steps`, `incomparable_k`, the
  fixed steps where it occurs); the stage's maximum is then taken over its comparable arrays and steps. The hover
  packet follows the same rule.
- **Step-0 reference (review r2 MINOR-3):** relative stamps subtract each boot's own step-0 packet `sim_now`, so any
  step-0 offset between two boots (for example step 0 served one 4 ms physics step later in one boot) is carried into
  every later step of the relative-stamp stage. Two **reference-free** readings, which do not carry it, are reported
  beside it: the frame ages (`sim_now` minus `lidar_t`, `rgb_t`, `depth_t`, `target_t` of the same packet, integer
  µs) and the per-step increment of `sim_now` (step k's packet minus step k−1's, k ≥ 1), each with its first differing
  step and maximum absolute difference. The step-0 offsets themselves are the absolute offsets below. A second
  relative reading anchored at a "first common control-phase stamp" is not used: the two boots' absolute sim clocks
  differ by seconds at control start (take-off and hover take different times), so no stamp value is common to both
  and such an anchor would be the step-0 anchor again.
- **Absolute offsets:** each boot's step-0 `sim_now` and their difference, step 0's `target_count`, and the step-0
  difference of every absolute header field. These are reported, not used for ordering.
- **Pre-control:** the hover packet (payload exact equality and maximum absolute difference), the start checks and their
  absolute differences (truth and EKF altitude, EKF–truth horizontal error, distance from spawn), and the hover guards.
- **Tie rule (fixed):** the earliest step is the smallest first-exact-difference step over the stages. If one stage
  holds it, it is reported as `SINGLE_EARLIEST`; if several, `TIED`; if all seven, `ALL_TIED`; if none differs,
  `NO_RECORDED_DIFFERENCE`. **Tied stages are not ordered**, and a difference at k = 0 is uninformative about order
  within the control phase. The pipeline order of the table is not used to break ties.
- **Trajectory and start-state spread (question 2):** for each pair, from `steps.npy`, the same-index distance between
  the truth positions and between the estimated positions: its maximum, its mean, and its value at the fixed steps; and
  at step 0, the truth and estimated position distances, the estimated velocity difference and the truth yaw
  difference. Per scene and pair kind (within OFF, within ON, between conditions): the pair count and the median,
  minimum and maximum of each of these (truth and estimated position distance maximum, mean and value at each fixed
  step; the four step-0 quantities), of the start-check differences, of every stage's first-exact step,
  first-above-tolerance step, maximum absolute difference, non-finite mismatch count, first non-finite-mismatch step
  and incomparable-step count, of the stamp-stage target-count maximum, of the two reference-free readings and of the
  absolute step-0 offset; the counts of the tie readings; and outcome agreement.
- **Question 2 is read descriptively, with fixed statistics (review r2 MINOR-1):**
  - **Primary per-pair statistic:** `truth_pos_dist_max_m`, the maximum same-index truth-position distance over the
    pair's common steps.
  - **Within range:** per scene, the minimum–maximum of the primary statistic over the **pooled 6 within-condition
    pairs** (3 within OFF and 3 within ON).
  - **Reading:** per scene, how many of the 9 between-condition pairs lie **below**, **inside** (bounds included) or
    **above** that range, written "k of 9 inside (b below, a above)". Both scenes are reported separately and are not
    combined; mixed results between scenes are stated as they are.
  - **Same reading at each fixed step:** the truth-position distance at step 0, 10, 25, …, 599, using only the pairs
    that reached that step (NOT_AVAILABLE when no within or no between pair did).
  - **Secondary statistic:** the observation stage's first step above tolerance; a pair with no such step counts as
    later than every common step (inf). Same range reading.
  - Everything else in this section is secondary description. No significance or equivalence gate is used (n = 3 per
    scene and condition). Exact equality and the tolerances are diagnostic thresholds, not statistical criteria.

The total output size is also reported.

## 7. What may and may not be said

- **May:** "In these 12 instrumented boots, boots of the same scene first differed at stage X (step k), or at tied
  stages X, Y (step k), within the same condition and between conditions", with the pair counts; the reported maximum
  differences and trajectory distances; and, per scene, "k of 9 between-condition pairs lay inside the range of the
  6 within-condition pairs" for the primary statistic (and at the fixed steps, and for the secondary statistic), with
  the pair counts and outcome agreement.
- **May not:**
  - determinism or equivalence;
  - a causal effect of the contact subscriber; ON also includes the floor control and the subscriber check (§3);
  - that the first recorded divergence is the causal initiator, because recording resolution and arrival alignment
    limit ordering, and tied stages are not ordered;
  - that the instrumented boots behave like the historical uninstrumented ones: the recorder adds per-packet array
    copies, per-message MAVLink logging in the receive thread and one throttled `statvfs` per second, identical in
    both conditions (reported as recorder overhead, bounded only through the LATE rule);
  - that `--seed` controls the PX4 sensor-plugin noise (NOT_TESTED);
  - that the 600-step path was tested live before attempt01 (the smoke is a short boot, §3);
  - anything about camera-only policies, OAK-D, the new vehicle, S5 or real flight.

## 8. Before execution

- This preregistration, the code and the tests are committed.
- An independent review of the design and source has passed.
- PUBLIC_CPU has passed.
- The user gives a scoped approval naming `SCOPE` (smoke and attempt01) and HEAD for GPU rendering. The approval file
  is `~/motar_tmp/s4r1_20261110/execution_approval.json`. `SCOPE`, quoted exactly from the tool (a test binds the two):

  ```
  S4R-1 v1: one fixed instrumentation smoke smoke01_contact_on_env0 (env0, contact subscriber ON, not a result, 600s outer, a short boot) then attempt01: 12 fresh Gazebo/PX4 boots, G8 r2 first two eligible 70-bar scenes env0/env1, 3 repeats per scene with world-contact subscriber OFF/ON, fixed alternating order; S4F1 lit_rendered ground arm, Frozen Policy F, navrl_quad_s4, body-fixed 87deg RGB/depth/gpu_ray, pinned historical bridge, explicit new Gazebo seed4501; MAVLink recv_match arrivals and sends recorded (wait_heartbeat NOT_RECORDED), all delivered raw frames and actor observations recorded; no retries, stop on first operational/recording invalid member, 180s per boot, 1800s outer; owned-session RSS cap 8GiB, no RLIMIT_AS; DISPLAY :1 with a readable inherited XAUTHORITY; 3GiB maximum output, 4GiB launch free and 1GiB minimum free disk; descriptive only: first recorded divergence per stage and within/between-condition spread, no gate
  ```
- The S4F-1 component smoke result, plugin pins and world-model pins still pass (the tool refuses otherwise).
- **Exact commands**, from a shell with no inherited `NAVRL_*`, `PX4_*`, `S3B_*` or `GAZEBO_*` variable. `taskset`,
  `nice` and `timeout` each exec the next, so `timeout` is the tool's direct parent. The tool refuses unless that
  parent's argv is exactly `timeout -k 60 <1800 | 600> /home/fair/miniconda3/envs/aerialgym/bin/python -B
  tools/sitl_s4r1.py <--launch | --smoke>`, the interpreter is that absolute path, the working directory and
  `PYTHONPATH` are the worktree, `aerial_gym` resolves inside it, `DISPLAY` is `:1`, and no address-space limit is
  set. **`XAUTHORITY` is not written in the commands: it is inherited from the user's desktop session** (on
  2026-11-10 `/run/user/1000/gdm/Xauthority`; `:1` is the user's vt2 X server, `:0` belongs to gdm). Before any
  directory exists the tool refuses unless `XAUTHORITY` is set to a readable file and `xdpyinfo -display :1` (10 s
  timeout) returns 0 with that `DISPLAY` and `XAUTHORITY` (review r2 MINOR-4); `SF.launch_problems` also refuses a CUDA device, user site, more than one thread, an affinity outside
  {6, 7, 12, 13} or nice below 10.
  1. the instrumentation smoke:

     ```
     cd /home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly && CUDA_VISIBLE_DEVICES="" PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly DISPLAY=:1 taskset -c 6,7,12,13 nice -n 10 timeout -k 60 600 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/sitl_s4r1.py --smoke
     ```

  2. only if the smoke is COMPLETE and VALID and passes `instrumentation_smoke_problems`, attempt01:

     ```
     cd /home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly && CUDA_VISIBLE_DEVICES="" PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly DISPLAY=:1 taskset -c 6,7,12,13 nice -n 10 timeout -k 60 1800 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/sitl_s4r1.py --launch
     ```

  3. `--analyze` (CPU only, after the run).

## Review before the commit

- **Design review r1** (`~/motar_tmp/sitl_next_20261110/s4r1_review_r1/REVIEW_design_r1.md`, Claude Opus 5.5, fresh
  context, 2026-11-10): verdict **AMEND_FIRST** (1 BLOCKER, 4 MAJOR, 7 MINOR, 6 LOW). Repairs before the commit
  (detail in `REPAIR_r1.md` beside the review):

| Finding | Repair |
| --- | --- |
| BLOCKER-1: F1 overwrote the ledger `arm` with the cell, so `--analyze` could never read REPORTED_DESCRIPTIVE | `finish_episode` sets `arm` to the condition, keeps `cell` and `s4_arm`; a positive control builds a complete 12-boot attempt through `SF.run_unit`'s real `dict(item, **res)` path and reads REPORTED_DESCRIPTIVE; negatives: wrong arm, missing boot, changed bytes, changed ULog gzip, changed contact log, lost seed, smoke attempt |
| MAJOR-1: absolute stamps and exact hashes made the divergence analysis degenerate; no magnitudes, no tie rule | relative stamps (µs) with absolute offsets reported; per-stage first exact step, max-abs, first step above a fixed tolerance, fixed-step values; hover packet and start checks; fixed tie rule; per-scene within-OFF / within-ON / between trajectory and start-state spread (§6) |
| MAJOR-2: a whole-tree walk inside the control loop, per-read state events, a re-walk per written file | no check in the frame read; one session check plus a ≤ 1 Hz `statvfs`; tree walks only at boot boundaries; incremental byte counter for writes; state reads no longer logged (a per-packet state snapshot instead); recorder overhead reported; a test asserts no tree walk during the per-step check |
| MAJOR-3: `ulimit -v` reached `gzserver` and PX4; no smoke | finite `RLIMIT_AS` refused; the owned-session 8 GiB RSS watchdog bounds memory; the fixed instrumentation smoke (§3) gates attempt01 |
| MAJOR-4: the preregistered command failed the tool's own refusals; loose parent check | exact commands above; exact parent-argv, interpreter, cwd, `PYTHONPATH`, `aerial_gym` and `DISPLAY` checks with tests; a test binds this file's commands to the tool's |
| MINOR-1 | the ON treatment is stated in full (§3); the subscriber check is folded into one session check; AGQ's freshness rule added |
| MINOR-2 | "operationally invalid" and the ULog coverage rule defined (§4, §5) |
| MINOR-3 | the inherited 30 s core-busy wait disclosed; the GPU check has no wait (§5) |
| MINOR-4 | recording contract corrected: packet state snapshot implemented, the target send recorded per step, the per-step contact field stated NOT_RECORDED, header keys and phase names as implemented (§4) |
| MINOR-5 | ULog check failures are returned, never raised; the physical record and native ULog are kept and the boot is recording-invalid |
| MINOR-6 | `--analyze` verifies the ULog gzip, the contact log and the session receipt's seed |
| MINOR-7 | disk estimate updated with processed arrays, ULog reconstruction and events (gzip-compressed) (§5) |
| LOW | one event sequence across threads; `wait_heartbeat` messages and the seed's effect on sensor noise stated NOT_RECORDED / NOT_TESTED; time estimate includes persisting. Not done here: hashing `gz`/`stdbuf`, the data-audit README binding and the `docs/EXPERIMENTS.md` entry (at the commit) |

- **Design review r2** (`~/motar_tmp/sitl_next_20261110/s4r1_review_r2/REVIEW_design_r2.md`, Claude Opus 5.5, fresh
  context, 2026-11-10), a re-review of the r1 repairs: BLOCKER-1 and MAJOR-2/3/4 repaired, r1 MAJOR-1 mostly
  repaired; verdict **AMEND_FIRST** (1 MAJOR, 5 MINOR, LOW). Repairs before the commit (detail in `REPAIR_r2.md`
  beside the review):

| Finding | Repair |
| --- | --- |
| MAJOR-1: a finite-against-inf entry made the whole stage "incomparable" and erased the raw-sensor and hover magnitudes | max-abs over entries finite in both boots; non-finite pattern mismatches counted separately per stage and step; incomparable only for a shape mismatch or missing array; same for the hover packet; rule in §6; tests with inf/NaN flips, identical patterns and a true shape mismatch |
| MINOR-1: question 2 had no fixed statistic or within-range definition | primary `truth_pos_dist_max_m`, pooled 6 within pairs per scene, "k of 9 inside" with below/above, the same at each fixed step, secondary observation first-above-tolerance step; computed in `spread_summary` (§6) and tested |
| MINOR-2: summary lacked the estimated-position mean and fixed-step distances | added to `spread_summary`, tested |
| MINOR-3: the step-0 reference carries a boot-level offset into every step | disclosed in §6; reference-free frame ages and `sim_now` increments added; stamp µs and target counts reported apart; the "common control-phase anchor" explained as unavailable; tested with a 4 ms step-0 offset |
| MINOR-4: `XAUTHORITY` unchecked, so the single-use smoke could fail at rendering | `XAUTHORITY` set and readable, and `xdpyinfo -display :1` returning 0, checked before any directory exists; §8 states it is inherited; tested positive and negative |
| MINOR-5: the smoke does not exercise the 600-step path | a CPU test drives a synthetic 600-step boot through persistence, checked ULog, timeline, byte counter, output cap and the fixed step 599; the real extractor timed on the longest S4F-1 ULog; the limit disclosed in §3, §5 and §7 |
| LOW | 180 s / 8 GiB disclosed against S4F-1's 240 s / 12 GiB; launch floor raised to 4 GiB; scene-specific time and disk estimates; `SCOPE` wording aligned and quoted in §8 with a test; every `NATIVE_BOUND` file required; seed binding and contact audit wrapped so a row is never lost; state-stage keys stated. Not done here: the evidence-folder copies, the `docs/EXPERIMENTS.md` entry and full PUBLIC_CPU (at the commit); a test of the real `private_module` attribute overrides |

- **Design review r3** (Claude Opus 5.5, focused, 2026-11-10): **PASS_WITH_LIMITS**. Every r2 finding was repaired
  with no new defect above LOW, and no code fix is required before the commit.
  - On this host the display preflight passes, and it runs before any directory is created.
  - Disk: 6.30 GB free; about 3.37 GB is expected after the smoke and attempt01 worst cases. Re-check `df` before each
    launch.
  - **Open LOWs:** §7's "k of 9" wording at fixed steps; the secondary statistic will likely read [0, 0]; pooled vs
    per-condition wording; common-step windows; line lengths.
  - Review files: `docs/evidence/sitl_2026-11-05/s4r1/`.
- **CPU tests:** `tests/test_sitl_s4r1.py`, run on 2026-11-10 after each repair round (cores 14–15, `nice 10`, CUDA
  hidden); the counts and results are in `REPAIR_r1.md` and `REPAIR_r2.md`.
