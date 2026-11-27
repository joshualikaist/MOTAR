# S4R-1 v1: independent design and source review r2 (repairs to r1), fresh context

- **Reviewer:** Claude Code subagent, Opus 5.5 (`claude-opus-5-5`), fresh context, read-only. Date: 2026-11-10.
- **Scope:** the worktree `src/MOTAR-camonly` (`camonly/dev`). The unit's three files are untracked:
  - `tools/sitl_s4r1.py` (1,289 lines);
  - `tests/test_sitl_s4r1.py` (708);
  - `docs/plans/prereg_sitl_s4r1_v1_2026-11-10.md` (315).

  I read them against r1 (`s4r1_review_r1/REVIEW_design_r1.md`), `REPAIR_r1.md`, the pre-repair originals (diffed),
  and the shared runners they call: `sitl_s3b_formal` (`run_unit`, `launch_problems`, `episode_valid`), `sitl_s3c`
  (`launch_unit`, `smoke_problems`, `arm_integrity_problems`), `sitl_s4f1.run_episode` / `cell_os`, `sitl_s4._fly`,
  `sitl_owned_session` (`OwnedSession`, its receipt, `validate_argv`, `runtime_binding`), `sitl_agq_geometry_diag`
  (`floor_control`, `contact_audit`, `session_control`, `_px4_now`), `sitl_dev08_attempt` (`git_state`,
  `environment_problems`, `idle_check`) and `sitl_s4_gazebo` (`motar_lidar`, `motar_rgbd`).
- **Not touched:** the S5 files. No PX4, Gazebo, GPU, `--launch`, `--smoke` or network.

## What I ran (CPU only; cores 14 and 15, `nice 10`, CUDA hidden)

1. **Requested unit tests:** `unittest discover -s tests -p test_sitl_s4r1.py`. 27 tests ran in 1.36 s; result OK.
2. **The prereg §8 argv shape, simulated without launching anything.** I put a probe script at
   `<scratch>/tools/sitl_s4r1.py` and ran it exactly as
   `env -i HOME PATH XAUTHORITY CUDA_VISIBLE_DEVICES="" PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
   PYTHONPATH=<worktree> DISPLAY=:1 taskset … nice -n 10 timeout -k 60 {600|1800}
   /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/sitl_s4r1.py {--smoke|--launch}`. The probe imports the
   real module by path and calls its checks.
   - The parent read from `/proc` is `timeout -k 60 600|1800 /home/…/python -B tools/sitl_s4r1.py --smoke|--launch`,
     so `taskset`, `nice` and `timeout` each exec the next.
   - `sys.executable` is the absolute path. It is not resolved through the `python -> python3.8` symlink.
   - `runtime_problems(mode, cwd=ROOT)` returned `[]` for both modes. This includes the RLIMIT_AS, parent, PYTHONPATH,
     DISPLAY, interpreter and `aerial_gym` checks. The only problem with the real cwd was the working directory,
     because the probe sat in the scratchpad.
   - `AT.environment_problems` refused only the affinity {14, 15}. I chose those cores deliberately; the prereg's
     6, 7, 12, 13 pass the rule. `SF.env_problems` returned `[]`.
   - This shell has no `NAVRL_*`, `PX4_*`, `S3B_*` or `GAZEBO_*` variable, and `ulimit -v` is unlimited.
3. **DISPLAY in the earlier runs:**
   - Every S4F-1 record (`attempt01`, the four `smoke01_*`) and every S4 record (`attempt01`, `smoke01`) has
     `"display": ":1"`. The S4F-1 attempt also ran at affinity [6, 7, 12, 13] and nice 10.
   - `run_s4f1.sh` exported `DISPLAY="${DISPLAY:-:1}"`, inherited `XAUTHORITY`, and set no `ulimit -v`.
   - Two Xorg servers run now: vt1 belongs to gdm (uid 126) and vt2 to user 1000. `:1` is the user's.
4. **Historical session limits** (from the 160 S4F-1 `session.json` receipts): `deadline_s` was 240 and
   `rss_cap_bytes` was 12 GiB; peak RSS was at most 1.14 GiB.
5. **Recorder cost microbenchmark** (the real `Tap` and proxies, 600 steps with realistic array sizes, 35,000 MAVLink
   messages):
   - 0.117 ms per control step (frames, control_step and set_target together);
   - 6.1 µs per received message;
   - a full `gc.collect()` with 167 k objects took 10 ms.

   AGQ's recorded boot had 16,896 MAVLink arrivals, so these costs are small next to the 0.15 s LATE bound.
6. **A raw LiDAR array with a +inf (no-hit) beam against a finite beam, through `compare_members`:**
   - `sensor_payload` reads `first_k_above_tol 0`, `max_abs 'INCOMPARABLE_SHAPE_OR_MISSING'` and `incomparable_k 0`;
   - `hover_packet.max_abs` reads `inf`;
   - `spread_summary` drops the stage's max-abs to `{'n': 0}`.

   See MAJOR-1 below.

## Disposition of r1 findings

| r1 | Status | Evidence |
| --- | --- | --- |
| BLOCKER-1 | **Repaired** | `finish_episode` sets `arm` to the condition after F1 has set `arm` to the cell. `SF.run_unit` writes `dict(item, …, **res)`; no key of `res` collides with run_unit's explicit keywords, as the historical F1 rows show. `run_unit` keys BINDING_CHANGED by `item['arm']`, that is, per condition. `runtime_binding` does not depend on the condition, so `analyze`'s all-boots binding check fits a valid run. A positive control of `analyze` runs through the real `run_unit` and reads REPORTED_DESCRIPTIVE with 30 pairs; its negatives are non-vacuous. |
| MAJOR-1 | **Mostly repaired**; new defects MAJOR-1 and MINOR-1/2/3 below | Relative stamps, per-stage max-abs, first step above tolerance, fixed steps, the tie rule, hover/start checks and within/between spread are implemented. The tolerances and the tie rule are stated identically in prereg §6. The magnitude reading is degenerate for raw sensor arrays; question 2 has no fixed primary statistic. |
| MAJOR-2 | **Repaired** | `frames()` makes no session check. `Session.check` makes one inherited check plus at most one `statvfs` per second, and never walks a tree; the test patches `rglob`, `os.walk` and `output_size` to raise. Tree walks happen only at boot boundaries and after the flight (the session is already closed then). `state_read` events are gone. Overhead is recorded. The measured per-step cost is tiny (item 5 above). |
| MAJOR-3 | **Repaired** | A finite RLIMIT_AS is refused. Like S4/S4F-1, the run has no `ulimit -v` and relies on the owned-session RSS watchdog. The smoke is wired: `--smoke` → `launch_unit(smoke=True)` into its own single-use directory. attempt01 adds `instrumentation_smoke_problems` and passes `smoke_dir`, so `S3C.smoke_problems` requires COMPLETE, VALID, the same HEAD and the same source bytes. Both gates are tested positive and negative. Scope, 600 s, time (1–3 min) and disk (0.23 GB) are in §3 and §5. |
| MAJOR-4 | **Repaired** | The exact commands are bound to `launch_command()` by a test. The parent argv is compared exactly. The simulation in item 2 shows the §8 argv is accepted. DISPLAY `:1` matches every earlier S4 and S4F-1 record. See MINOR-4 for XAUTHORITY. |
| MINOR-1 to 7 | Repaired or disclosed | The ON treatment is stated in full. AGQ's freshness rule is reused; it uses the same clock source, `time_boot_ms`. "Operationally invalid" is defined. The 30 s wait is disclosed. The recording contract matches the code. `checked_gzip` never raises. `analyze` verifies the ULog gzip, the contact log and the seed read from `session.json`; the `launches[role=gzserver].argv` field exists in the receipt. The disk estimate is updated. |
| LOW | Partly done, as REPAIR_r1 says | — |

## Findings

### MAJOR-1: Non-finite mismatches are labelled "incomparable shape" and wipe out the sensor-payload and hover magnitudes

- **Where:** `max_abs` maps a finite-against-±inf entry to `inf` (`tools/sitl_s4r1.py:1022-1026`). `compare_members`
  then treats any `inf` as INCOMPARABLE: it sets `incomparable_k` and `max_abs = 'INCOMPARABLE_SHAPE_OR_MISSING'`
  (`:1128-1136`). `_stats` silently drops the string (`:1155-1159`). The hover packet reports a bare `inf`
  instead (`:1145-1147`).
- **Why it happens in practice:** the raw `gpu_ray` ranges and raw depth hold non-finite values. `motar_lidar`
  maps `+inf`/NaN to LIDAR_MAX and flags `-inf`; `motar_rgbd` handles `~isfinite`
  (`tools/sitl_s4_gazebo.py:75-76, 98`). Two boots whose poses differ by centimetres will flip some beams between
  no-hit and hit.
- **Result:** for essentially every pair, the `sensor_payload` stage will read "incomparable" at k = 0. Its max-abs,
  fixed-step values and per-scene summary will be empty (`n = 0`), so the raw-sensor magnitude, the stage that
  question 1 is most about, is lost. Item 6 above reproduces this.
- **The prereg leaves the case open:** it states "equal non-finite entries count as equal; a shape mismatch is
  reported as incomparable" (§6) but says nothing about a finite-against-non-finite mismatch, so prereg and code are
  not stated identically.
- **Fix:**
  - In `max_abs`, return the max over entries finite in both arrays, plus a separate count of non-finite pattern
    mismatches (`nonfinite_mismatch_n`) per stage and step.
  - Reserve INCOMPARABLE for a shape mismatch or a missing key.
  - Apply the same rule to the hover packet.
  - State the rule in §6, and add a test with an inf/finite flip in raw ranges and depth.

### MINOR-1: Question 2's reading has no fixed primary statistic or within-range definition

- **The wording:** §6 says "whether the between-condition values lie inside or outside the range of the
  within-condition values".
- **What is undefined:** with about 40 summary statistics per scene, the wording does not say:
  - which statistic answers question 2;
  - whether the "within range" pools within-OFF and within-ON (6 pairs per scene) or takes each separately;
  - how mixed results are stated.
- **The code does not compute it:** the inside/outside reading is left to post-hoc choice. That is analysis freedom
  after measurement, and the preregistration cannot be changed afterwards.
- **Fix:**
  - Fix one primary per-pair statistic, for example `truth_pos_dist_max_m` over the common steps, and one secondary,
    for example the observation stage's `first_k_above_tol`.
  - Define the within range as the min–max of the pooled 6 within-condition pairs per scene.
  - Compute `between_inside_within_range: k of 9` in `spread_summary`, with a test.
  - Report everything else as secondary description.

### MINOR-2: `spread_summary` omits statistics that §6 lists

- **What is missing:** §6 asks for the median, minimum and maximum of "each of these", which includes the
  estimated-position distance mean and the estimated-position distance at the fixed steps. `spread_summary` reports
  only `est_pos_dist_max_m` (`:1181`). The per-pair values exist, so only the summary lacks them.
- **Fix:** either add the two statistics to the summary or narrow the prereg text.

### MINOR-3: Stamps relative to step 0 carry one boot-level offset into every step

- **The mechanism:** the relative stamps subtract the boot's own step-0 `sim_now` (`stamp_vector`, `:1029`). If two
  boots served step 0 one physics step apart (4 ms of jitter in `wait_for_step`), every later step reads as
  different, even when the per-step frame ages agree.
- **The design is well-defined but undisclosed:** r1 suggested a per-step reference (`sim_start + k·DT`, which is not
  recorded). The prereg does not explain the consequence of the choice made instead.
- **Fix (descriptive, cheap):**
  - Also report reference-free per-step quantities: frame ages (`sim_now − lidar_t / rgb_t / depth_t / target_t`)
    and the per-step increment of `sim_now`.
  - Say in §6 that the relative-stamp stage includes the step-0 offset.
  - Note that `max_abs` for this stage mixes µs and counts. Report the two separately.

### MINOR-4: Rendering preconditions that are not checked could make the single-use smoke fail for the wrong reason

- **The smoke is not retryable:** "if it fails, the unit stops and is amended" (§3).
- **What is not checked:** the tool checks `DISPLAY == ':1'` but does not check that the display is reachable with
  the launching shell's `XAUTHORITY`. That variable is inherited, appears in no command, and is not checked.
- **Why it matters:** earlier runs worked because they were launched from the user's GUI session, whose
  `environment_keys` contain `XAUTHORITY`. From a shell without `XAUTHORITY`, or after a reboot that renumbers the
  displays (`:1` is the user's vt2 server today, and `:0` belongs to gdm), gzserver cannot render.
- **The precedent:** S4/S4F-1 required only a non-empty `DISPLAY`, so the hard-coded `:1` is new strictness.
- **Fix:**
  - Add a preflight that opens the X display with the inherited authority and refuses before any directory exists,
    for example `xdpyinfo -display :1` with return code 0, or an Xlib connect.
  - Either name `XAUTHORITY` in the §8 command text or state that it is inherited from the user's desktop session.

### MINOR-5: The smoke does not exercise the long-boot path

- **The smoke boot is short:** env 0 captured at 43 steps in S4F-1 attempt01, and at 46 in S4F-1's own smoke.
- **What reaches attempt01 untested:** env 1 timed out at 600 steps (101 s). The following are therefore first
  exercised inside the single-use attempt01:
  - 600-step persistence (the "≤ 20 s" estimate);
  - worst-case disk (about 225 MB per boot);
  - the 20 MB ULog reconstruction under the 60 s extractor timeout;
  - the long contact audit;
  - recorder memory and GC over a long boot.

  My microbenchmark makes LATE trouble unlikely, but none of these paths is exercised live.
- **Fix:** either make the smoke env 1 contact ON (schedule index 2; about 100 s more, still under 600 s), or state
  the limit in §3 and §7.

### LOW

- **Session limits differ from S4/S4F-1:** they ran with `deadline_s` 240 and `rss_cap` 12 GiB; S4R-1 uses 180 s and
  8 GiB. "S4/S4F-1's mechanism" (§5) is accurate for the mechanism but not the values. Disclose the tighter values
  (peak was 1.14 GiB and the maximum boot 101 s, so both are safe).
- **Disk margin is thin:** the launch floor of 3.5 GiB minus the 2.7 GB worst case leaves about 1.06 GB, just under
  the 1 GiB (1.074 GB) write floor. Raise the floor to about 4 GiB, or note it.
- **Scene-specific estimates:** env 0 is about 44 s and about 45 steps; env 1 is about 101 s and 600 steps. Expected
  wall time is about 15–17 min, not 12, and expected disk is about 1.5 GB, not 0.9 GB. Both remain within the bounds.
- **SCOPE against the prereg:** the prereg does not quote `SCOPE` (consistent with S4/S4F-1). Two of `SCOPE`'s
  phrases are broader than the prereg:
  - "all boot/control/landing telemetry retained", while `wait_heartbeat` messages are NOT_RECORDED;
  - "descriptive first-recorded divergence only", while §6 also reads question 2's spread.

  Align the wording, and optionally quote `SCOPE` in §8 and bind it with a test, as the launch commands are.
- **Silent manifest omission:** `NATIVE_BOUND` files are added to the manifest only if they exist (`:799-802`), and
  `member_problems` requires only four of them. A valid boot implies the others exist, but require all of
  `NATIVE_BOUND` explicitly.
- **Lost physical record:** `contact_audit` and `seed_binding` run outside the `try` in `finish_episode`. An
  unexpected exception there leaves no ledger row (ABORTED). AGQ's `contact_audit` is defensive, so the risk is low.
- **Prereg wording:** "estimated and rendered position, velocity and quaternion" (§6 state row) implies a rendered
  velocity, which is not compared. The stage's keys are `pos`, `vel`, `vquat`, `render_pos` and `render_vquat`.
- **Before the commit:**
  - update the prereg's review section, which currently says "re-review pending";
  - copy REVIEW_r1, REPAIR_r1 and this review into the unit's evidence folder;
  - add the EXPERIMENTS.md family entry, without which `test_experiment_index` fails;
  - run the full PUBLIC_CPU.
- **Test gap:** `test_run_episode_wiring_overrides_f1_arm` stubs `private_module`, so the real attribute overrides
  (`s4.G`, `f1.S4`, `f1.gzip_ulogs`) are not exercised. I confirmed them by reading: `sitl_s4._fly` uses
  `G.gazebo_episode_class`, `F1.run_episode` uses `S4.run_episode` and `gzip_ulogs`, and `cell_os` copies the proxy
  namespace, so the seed and launch wrappers survive.

## Answers to the specific questions

- **Arm labelling on a real ledger path:** correct, through `F1.run_episode` → `finish_episode` → `SF.run_unit` →
  `S3C.arm_integrity_problems`. The positive control is real.
- **Comparison readings:**
  - Well-defined and stated identically: relative stamps, tolerances (0 / 1e-3 / 1e-6), the fixed steps, the tie
    rule (SINGLE_EARLIEST / TIED / ALL_TIED / NO_RECORDED_DIFFERENCE, with the k = 0 note) and the pair counts
    (3/3/9 per scene).
  - Not: the non-finite case (MAJOR-1), question 2's primary reading (MINOR-1), the summary contents (MINOR-2) and
    the stamp-reference disclosure (MINOR-3).
- **Control loop:** no tree walk. The per-step cost is about 0.1 ms; the per-message cost is about 6 µs.
- **Memory:** matches S4/S4F-1: the RSS watchdog and no RLIMIT_AS. Only a finite address-space limit is refused,
  which is the right refusal. The limits are tighter than S4F-1's (LOW).
- **Smoke:** wired as described. attempt01 refuses without a passing smoke at the same HEAD and source bytes. Scope,
  time and disk are in the prereg. Its coverage is limited (MINOR-5).
- **§8 commands:** accepted by the tool's checks in simulation. `DISPLAY=:1` is what S4 and S4F-1 used. A normal
  launch from the user's desktop shell would not be refused for a wrong reason. A shell without `XAUTHORITY` would
  pass the checks and then fail at rendering (MINOR-4).

VERDICT: AMEND_FIRST
