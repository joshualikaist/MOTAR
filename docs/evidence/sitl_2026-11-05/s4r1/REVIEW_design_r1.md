# S4R-1 v1 — independent design/source review r1 (fresh context)

- **Reviewer:** Claude Code subagent, model Opus 5.5 (`claude-opus-5-5`), fresh context, read-only.
- **Date:** 2026-11-10. Worktree `src/MOTAR-camonly`, branch `camonly/dev`, HEAD `504eb4e`.
- **Reviewed (untracked):** `docs/plans/prereg_sitl_s4r1_v1_2026-11-10.md` (178 lines), `tools/sitl_s4r1.py` (817),
  `tests/test_sitl_s4r1.py` (241). The tool and the test match the copies in
  `~/motar_tmp/sitl_next_20261110/repeatability_prepare_r1/` byte for byte (`diff` is empty). I also read
  `DESIGN_DRAFT.md`, the data audit `docs/evidence/sitl_2026-11-05/data_utility/README.md` and the committed runners it
  builds on: `sitl_s4f1.py`, `sitl_s4.py`, `sitl_s4_gazebo.py`, `sitl_s3b_formal.py`, `sitl_s3b_closed_loop.py`,
  `sitl_s3c.py`, `sitl_owned_session.py`, `sitl_agq_geometry_diag.py`, `sitl_dev08_attempt.py`, `sitl_bridge.py` and
  `tools/sitl_s4/motar_s4_bridge.cc`.
- **What I ran (CPU only; no PX4, Gazebo, GPU or `--launch`):**
  - the requested unit tests: 14 run, OK, exit 0 (cores 14 and 15, `nice 10`, CUDA hidden);
  - `ulog_framing` on `parser_control.ulg`: no problems, all six required topics present;
  - `--extract-ulog` under the PX4 env's Python 3.8: exit 0 in 0.41 s. Its output went to the scratchpad and was
    deleted;
  - a timing probe of `resource_problems` over the S4F-1 attempt tree;
  - an in-memory check of `S3C.arm_integrity_problems` on ledger rows built the way `SF.run_unit` builds them;
  - `nvidia-smi --query-compute-apps` (empty) and `df` (5.15 GiB free under `~/motar_tmp`);
  - `/proc/<Xorg>/status` (VmSize 25.4 GB).

## Findings

### BLOCKER-1 — `--analyze` always reads INCOMPLETE_INVALID: the ledger's `arm` is overwritten

- **Where it happens:**
  - `schedule()` sets `arm` to `contact_on` or `contact_off` (`tools/sitl_s4r1.py:101`).
  - `F1.run_episode` overwrites the result's `arm` with the cell: `res.update(arm=cell, ...)` (`tools/sitl_s4f1.py:342`).
    S4R-1's `run_episode` keeps that value.
  - `SF.run_unit` writes `rec = dict(item, ..., **res)` (`tools/sitl_s3b_formal.py:602`). The keyword arguments win,
    so every ledger row gets `arm = 'lit_rendered'`.
  - `analyze()` calls `S3C.arm_integrity_problems` (`tools/sitl_s4r1.py:745`). That function compares the
    `(arm, env)` of each row with the schedule's (`tools/sitl_s3c.py:158`).
- **Reproduced in memory:** the check returns `["ledger rows are not the schedule's prefix in order"]`. A complete,
  valid run of 12 boots can therefore never read REPORTED_DESCRIPTIVE.
- **Fix:**
  - In `run_episode`, set `result['arm'] = 'contact_on' if contact else 'contact_off'` after the F1 call. `cell` and
    `s4_arm` keep the cell.
  - Add a positive-control test of `analyze()` on a synthetic 12-row attempt, built through the real `dict(item, **res)`
    path. Add negatives for a wrong order, a byte change and a missing manifest.

### MAJOR-1 — The preregistered divergence analysis is predictably degenerate and cannot answer question 2

- **Stamps compared as absolute times:** `compare_members` compares the whole packet header for equality
  (`tools/sitl_s4r1.py:732`). The header holds absolute simulator times (`sim_now`, `lidar_t`, `rgb_t`, `depth_t`,
  `target_t`). The control start time (`sim_start = L.pos[0]`, `sitl_s4.py:211`) depends on wall-paced boot, parameter
  and hold loops (`sitl_s4.py:150-190`). So `sensor_stamp` will differ at k = 0 in essentially every pair.
- **State and observation compared by exact SHA-256:** state is EKF-derived (`sitl_s4r1.py:723-727`). Fresh boots with
  wall-paced, asynchronous take-off almost certainly differ in the last bit at k = 0, so `state`, `observation`,
  `actor` and `governed_request` will all report k = 0 as well.
- **No tie rule:** ties within a step have no rule, and no pre-control stage is compared. The permitted sentence
  "first differed at stage X (step k)" (prereg §7) presumes a unique stage, which this analysis will not produce.
- **Question 2 is not measured:** it asks for a change "larger than the variation within one condition". The analysis
  reports only first-k and outcome agreement. It has no magnitude: no max-abs differences and no within- versus
  between-condition spread. The design draft required these ("exact equality and max absolute differences ...
  within-condition versus between-condition trajectory spread", "start pose/velocity/attitude spread", "ties/unresolved
  ordering"). They were dropped in the code and the prereg.
- **Fix:** before the commit, preregister:
  - stamps relative to `sim_start + k*DT` (frame age), and `target_count` / `target_t − sim_start`;
  - a pre-control stage comparison: the `start_check`, hover packet and hover guard;
  - per-stage, per-step max-abs-difference curves with a fixed reporting tolerance, alongside exact equality;
  - an explicit tie and unresolved-order rule;
  - a descriptive within- versus between-condition spread statistic (for example the median per-step trajectory
    distance), with pair counts.

### MAJOR-2 — The recorder adds control-loop work that grows with the boot index (not OFF-identical, and confounded with repeat)

- **New work in the loop:**
  - `Client.frames()` now calls `tap.session.check()` on every control frame (`sitl_s4r1.py:263-264`); the historical
    loop does not (`sitl_s4.py:216`).
  - The overridden `Session.check` runs `resource_problems`, which calls `output_size(attempt_dir)`. That is a full
    `rglob` plus `stat` of the whole attempt tree (`sitl_s4r1.py:106-120, 336-343`). It also runs inside
    `SF.wait_for_step` every 0.1 s (`sitl_s3b_formal.py:251`).
- **Measured cost:** about 6.5 µs per entry (22.8 ms for 3,524 entries). Each 600-step boot adds about 1,210 npz and
  json files. By the last boots (about 6–7 k entries) the cost is about 40–45 ms per call, twice per 0.1 s step, at a
  real-time factor of 1.000 (S4F-1 lit_rendered).
- **Effect:**
  - It shifts the phase between the state sample and the frame request, and adds state lateness. Historical `late_max`
    reached 0.128 s against the 0.15 s bound, so later boots risk LATE and a unit stop.
  - Within-condition pairs span different repeats (different overhead), while between-condition same-repeat pairs are
    adjacent. The comparison is therefore biased toward "no subscriber effect beyond within-condition variation".
- **Smaller additions:**
  - per-message `to_dict` and `jsonable` in the MAVLink receive thread (`sitl_s4r1.py:200-208`);
  - a `state_read` event on every `L.pos` read in the 2 ms wait loop (`sitl_s4r1.py:216-220`);
  - `dump_tap` re-walks the tree once per written file (`sitl_s4r1.py:387, 395`). That costs minutes of wall time
    against the 1,800 s outer limit.
- **Fix:**
  - Remove the check from `frames()`.
  - Keep the resource checks out of the control path: a running byte counter plus `statvfs`, checked between boots and
    at most every few seconds.
  - Record per-step recorder overhead (`perf_counter` around the proxies).
  - Preregister an overhead bound as a validity rule, and state the residual intrusion in §7.

### MAJOR-3 — An inherited RLIMIT_AS of 8 GiB is untested for GPU-rendering `gzserver`, and the prereg has no smoke

- **The limit reaches every child:** the launch is `ulimit -v 8388608` (prereg l.170, enforced by
  `runtime_problems`, `sitl_s4r1.py:662-664`). It is inherited by `gzserver` (NVIDIA OpenGL), PX4, `gz topic` and
  `px4-logger`.
- **Evidence it may not fit:**
  - On this host `Xorg` has VmSize 25.4 GB.
  - The S3a prereg (l.130) deliberately set no RLIMIT_AS because the driver reserves address space.
  - S4 and S4F-1 ran without `ulimit -v` (`~/motar_tmp/s4f1_20261108/run_s4f1.sh`).
  - Python with torch and warp alone measured 3.4 GB VmSize.
- **Other parts also untested live on this stack:**
  - the owned `px4-logger stop` (`sitl_s4r1.py:439-462`);
  - `gzserver ... world --seed 4501`, with the option after the positional argument (`:347`);
  - 250 Hz contact echo under rendering;
  - strict ULog framing.
- **The smoke was dropped:** the draft requires "an explicitly scoped fixed instrumentation smoke", and S3c, S4 and
  S4F-1 required a smoke so that a first flight does not consume the single-use attempt. The prereg and the code drop
  it: `launch_unit` is called without `smoke_dir` (`sitl_s4r1.py:704`).
- **Fix:** either drop RLIMIT_AS and rely on the session RSS cap (8 GiB, which includes self; `:320`) as S3a did, or
  apply the cap to the Python process only. Add a preregistered one-boot development smoke per condition (OFF and ON)
  in its own directory, required COMPLETE and VALID at the same HEAD before `attempt01`.

### MAJOR-4 — The preregistered launch command fails the tool's own refusals

- **What is missing:** prereg l.170 omits `CUDA_VISIBLE_DEVICES=""`, `PYTHONNOUSERSITE=1`, `OMP/MKL_NUM_THREADS=1`,
  the affinity ⊆ {6, 7, 12, 13} and `nice ≥ 10`. `AT.environment_problems` (`sitl_dev08_attempt.py:256-268`) refuses
  without them.
- **Also missing:**
  - `DISPLAY`, which `S4.s4_extra_problems` requires;
  - `PYTHONPATH` set to the worktree (AGENTS.md);
  - the absolute aerialgym interpreter (a bare `python` is PATH-dependent);
  - the working directory.
- **Consequence:** run literally, the command refuses. Run otherwise, it deviates from the preregistration.
- **Fix:** write the exact command, for example `bash -c 'ulimit ...; exec timeout -k 60 1800 taskset -c 6,7,12,13
  nice -n 10 env CUDA_VISIBLE_DEVICES= ... /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/sitl_s4r1.py
  --launch'`, run from the worktree. Confirm that `timeout` stays the direct parent (taskset, nice and env each exec).
- **Tighten the parent check:** it accepts any argument equal to `1800` (`sitl_s4r1.py:669`). Parse the command line
  as `timeout -k 60 1800 ...`.

### MINOR

1. **The treatment is not only the subscriber.**
   - ON also runs `floor_control`, which parses an 8 MiB tail, between the parameter read-back and take-off
     (`sitl_s4r1.py:227-232`). That shifts the wall-paced phase.
   - ON adds a second `super().check` per check (`:341-342`).
   - Either state these in §2–3 as part of the treatment or move them off the flight path.
   - The AGQ freshness test of the control window is not reused (`sitl_agq_geometry_diag.py:542` `session_control`
     checks the last message within 0.5 s of PX4 time). A stalled subscriber whose process is still alive passes.
2. **The prereg does not define "operationally invalid".**
   - State `SF.episode_valid` (outcome, `PARAM_MISMATCH`, `late_max_s` ≤ 0.15 s), the start-state and hover guards,
     `BINDING_CHANGED`, the ON-only `CONTACT_AUDIT` problems (`:629-635`) and `RECORDING_WRITE`.
   - The ULog coverage rule (`:537-544`) is not described either.
3. **`run_unit` still waits 30 s once for busy cores** (`sitl_s3b_formal.py:585-588`). This contradicts "no wait"
   (prereg §5) unless the text scopes "no wait" to the GPU.
4. **The recording contract overstates the code.**
   - Packet records hold no state snapshot (`:272-279`); the prereg §4 says each packet keeps the latest estimate,
     truth, attitude and heartbeat. They are only reconstructable from the `state_read` event order.
   - The step records do not hold the target send (it is an event, `:253-256`).
   - The timeline's contact field is the placeholder `POSTHOC_CONTACT_LOG`, not the nearest contact event (`:598`).
   - The header keys are `target_t` / `target_count`, not `target_applied_*`.
   - The phase list differs: `preflight_parameters_verified` covers arming, take-off and hover; shutdown is marked only
     after teardown.
5. **An exception in `checked_gzip`** (ULog count, budget, `np.load`) discards the whole physical record. The
   `except` in `run_episode` replaces `result` (`:621-622`). Return the problem instead of raising.
6. **`analyze()` does not verify everything it relies on:**
   - the ULog `.gz` against its record (`F1.verify_ulog_record`);
   - the ON `contacts.log`;
   - the seed from the actual `gzserver` argv. `gazebo_seed` is a constant (`:655`); bind it from `session.json`.
7. **The size estimate is out of date.** The ~744 MB draft figure excludes the float32 processed RGB, depth and obs
   (about 235 KB per step, about 1.7 GB raw worst case) and uncompressed `events.jsonl`. The worst case approaches the
   3 GiB cap with 5.15 GiB free. Preregister an expected size, and compress the event log.

### LOW

- Event indexes (`len(self.events)`) can repeat across the receive and main threads (`:143`).
- MAVLink messages consumed by `wait_heartbeat` are not logged.
- The `gz` and `stdbuf` executables are not hashed (AGQ bound them, `STDBUF_FILES`).
- Wall time quoted at "10–20 min" omits the dump and compression time.
- The untracked data-audit README is the prereg's motivation: commit it with the unit or bind it by SHA-256, and add
  the unit to `docs/EXPERIMENTS.md`.
- PX4 SITL sensor plugins may seed their own noise engines. That `--seed` controls them is NOT_TESTED: say so.

## Item answers

1. **Prereg against code:**
   - These match: scenes and refusal (`:92-95`); schedule, order and parity (`:97-102`); bounds of 180 s, 1,800 s,
     3 GiB, 3.5 GiB, 1 GiB and 8 GiB (`:36-41`, `:320`, `:662-684`); `invalid_abort=0`; GPU busy raising without a wait
     (`:692-694`); the scene and world binding (`:697-701`); the recording-invalid list (`:415-435`); the readings and
     integrity list; the stage table (`:723-733`); the approval path; the timeout-parent check.
   - `GAZEBO_SEED` is passed in the `gzserver` argv (`:347`) and recorded in `events.jsonl` (`launch_plan`) and in the
     session receipt's process record. It is not verified live, and the row stores a constant.
   - Mismatches: MAJOR-4, MINOR-1 to MINOR-4, the RSS cap missing from the prereg, and the smoke the draft required.
2. **Coverage of the audit's gaps:**
   - Captured: raw rendered arrays (an exact copy of each payload; per-packet npz plus SHA-256), actor observations and
     raw output, the MAVLink receive/send timeline with monotonic times, the plugin request/return timeline, and the
     target-applied counter.
   - The proxies are not non-intrusive (MAJOR-2). The subscriber is not the only OFF/ON difference (MINOR-1).
3. **Predictable invalidation:** RLIMIT_AS (MAJOR-3), LATE from recorder overhead (MAJOR-2) and the launch command
   (MAJOR-4).
   - Checked and fine: `packets == steps+1` holds for a normal boot; the required ULog topics are present
     (`parser_control.ulg` passes framing); `px4-logger` links to `px4`; RustDesk is allowed by path; disk and the
     output cap are adequate in the expected case.
   - The floor control is a non-vacuous positive control.
   - `analyze` would fail vacuously (BLOCKER-1).
4. **Tests:** 14 run, OK. The pairs are real for the schedule, the payload round trip, recording counts, compare
   null/corruption, the output cap, ULog truncation, logger ownership and parser isolation. Missing:
   - `analyze` (it would have caught BLOCKER-1);
   - the seed argv injection in `session_module`;
   - `runtime_problems` (RLIMIT and parent, positive and negative);
   - the ON `read_params` contact path (pass and fail);
   - `run_episode` end to end with fake OS, link and client (OFF `NOT_RECORDED`, ledger `arm`);
   - `checked_gzip` topic and coverage failures;
   - `canonical_timeline`.

   `test_episode_and_policy_off_identity` shows return passthrough only, not loop identity.
5. **Analysis:** BLOCKER-1 and MAJOR-1.
   - The may/may-not limits are appropriate (no determinism, equivalence, causal subscriber effect, causal initiator,
     historical-stack equivalence or transfer claim).
   - Add "a k = 0 tie across all stages is uninformative about order".

VERDICT: AMEND_FIRST
