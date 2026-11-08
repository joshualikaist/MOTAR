# S4F-1 v1 design review r2 (focused re-check of the r1 repairs)

- **Reviewer:** Claude Opus 5.5 (fresh context), independent design reviewer. Read-only on every repository. Wrote only
  to `~/motar_tmp/s4f1_20261108/review_design_r2/`.
- **Date:** 2026-11-08, 13:49–14:10 KST.
- **Nothing ran on the GPU.** No Gazebo, gzserver, PX4, Isaac Gym or CUDA was started, and `--check` was not run.
- **What I ran** (CPU cores 6,7, `nice -n 19`, `ulimit -v 8000000`, `CUDA_VISIBLE_DEVICES=""`):
  - the unit tests;
  - the component smoke's `--dry-run`. I read its code first: `main()` returns from `dry_run()` before `preflight()`
    or any `Popen` call, and it wrote nothing;
  - one numpy power probe;
  - `pyflakes`, `bash -n`, and a check for lines longer than 120 characters.
- **Worktree:** `MOTAR-realveh`, branch `realveh/dev`, HEAD `814c28a`.

| File reviewed | SHA-256 (all equal to IMPLEMENTATION.md) |
| --- | --- |
| `tools/sitl_s4f1.py` (986 lines, untracked) | `5c08b71a8087ed1c…` |
| `tests/test_sitl_s4f1.py` (1023 lines, untracked) | `420fb16201e4d15a…` |
| `docs/plans/prereg_sitl_s4f1_v1_2026-11-08.md` (295 lines, untracked) | `3cdd770f0b5547fa…` |
| `~/motar_tmp/s4f1_20261108/component_smoke.py` (426 lines) | `14925afb963c27e6…` |
| `~/motar_tmp/s4f1_20261108/run_s4f1.sh` (78 lines) | `558840b4aa966588…` |

**Results:**
- **Tests:** `test_sitl_s4f1` ran 53 tests: OK, 0 skipped, 26 s (`probes/tests_s4f1_reviewer_r2.log`). The skip-guarded
  tests on the host's real files all ran: the S4 plugin bytes, the world model files, the S3b and S4 ledgers, and the
  pinned smoke script.
- **Lint:** `pyflakes` is clean on all three Python files, no line is longer than 120 characters, and `bash -n` passes.
- **Dry run** (`probes/dryrun_r2.json`):
  - SHA-256s: runner `5c08b71a…`, script `14925afb…`, `runner_pins_this_script` true;
  - target mask 34 px;
  - bar foot 36 px (70 when dilated) and 0 LiDAR beams;
  - `lit_rendered`'s rig world equals S4's `smoke.world` (`1eddb2e8…`);
  - no lockstep in the probe;
  - C4 passes without a floor (0 removed outside the feet; 36 depth removed, ≤ 70);
  - C4 fails with a floor (68 + 6102 outside the feet);
  - C5 passes (68 / 6160).

## Verdict: **PASS_WITH_LIMITS**

- All three MAJOR findings are resolved, and so are seven of the eight MINOR findings and all eight LOW findings.
- MINOR-6 is only partly resolved. The binding of the bytes is right. The authorization part has a gap (new N1): the
  chain renders on the GPU in the component smoke before it checks any approval file.
- N1 is in `run_s4f1.sh`, which nothing pins. Fixing it does not change the SHA-256 of the prereg, the runner or the
  smoke script, so it can be done before the chain runs without re-pinning. It does not block committing the prereg.
- The other new findings are LOW.
- The prereg text matches the code on every point checked, apart from the wording nits N2 and N5.

## r1 findings

| r1 finding | Status | Evidence (r2) |
| --- | --- | --- |
| **MAJOR-1** plugin-age refusal; a naive rebuild would overwrite S4's `.so` | **RESOLVED** | See "MAJOR-1" below |
| **MAJOR-2** C4 counted bar-foot pixels | **RESOLVED** (offline) | See "MAJOR-2" below |
| **MAJOR-3** replication of S4 `ground` not checked | **RESOLVED** | See "MAJOR-3" below |
| MINOR-1 power arithmetic, "detectable" | RESOLVED | The prereg's splits and power table equal r1 probe P4's output, including the new 32-scene row (0.45 / 0.72) and +40 pp (0.98 / 1.00). "Lowers this slightly" understates the effect of Holm (N2) |
| MINOR-2 multiplicity | RESOLVED | `PRIMARY` (:110) is main A and main G on capture. `holm` (:853) is correct: `[0.01, 0.04]` → `[0.02, 0.04]`, and order is kept. `reached` is set only when the label is REPORTED (:871), and raw and Holm p are both reported |
| MINOR-3 attrition and ABORTED readings | RESOLVED | `reading_label` (:875): VOID when the attempt is not COMPLETE, the fault count rose, or an integrity check failed; INSUFFICIENT below 30 complete scenes; REPORTED otherwise. A VOID reading returns before any outcome block. `invalid_per_cell` is reported. Tests cover ABORTED, a fault rise, a wrong protocol, and 29 against 30 scenes. The threshold is 30 where r1 suggested 32; it is stated in the prereg |
| MINOR-4 compound factors, gap wording | RESOLVED | The "May not" section lists what A and G each change; item 8 says "includes at least" |
| MINOR-5 GPU checked at launch only | RESOLVED | See "MINOR-5" below |
| **MINOR-6** smoke authorization and binding | **PARTLY RESOLVED** | Binding is resolved (see "MINOR-6" below). The authorization gap is N1 |
| MINOR-7 manipulation checks | RESOLVED | `manipulation_checks` (:819): sensor-step columns 11 and 12 per step and at step 0, and the 5/6 ratio on shadow-visible steps; prereg item 3 |
| MINOR-8 rangefinder premise | RESOLVED | Stated in the prereg's "Cells". `smokes_problems` (:359) refuses \|EKF − truth\| > 0.05 m and also refuses a missing value (tested: 1.06, 0.94 and None refused; 0.049 accepted). Reported per cell |
| LOW-1 carry-over | RESOLVED | Wording is corrected: within scenes 10 each; across boundaries 19–20 against 10 |
| LOW-2 RedGlow | RESOLVED | |
| LOW-3 C3 naming | RESOLVED | Now called a "geometry sanity check" |
| LOW-4 ODE iterations | RESOLVED | `rest_world` swaps `<iters>10</iters>` for 100, and refuses if the tag is not found exactly once |
| LOW-5 ULog gaps | RESOLVED | Directory fsync before the unlink (:288-293); the analyzer decompresses each `.gz` (`ulog_summary` → `verify_ulog_record`), and problems are reported in item 7 |
| LOW-6 interaction null | RESOLVED | Stated in item 4 |
| LOW-7 S3b pin | RESOLVED | `S3B_LEDGER_SHA256 97b94dfd…` is enforced in `s3b_reference`, with a negative test and a real-file test |
| LOW-8 untested paths; no rebuild | RESOLVED | The DISK_LOW path is tested (ABORTED, no row, no episode run). The chain has no build step |

### MAJOR-1: S4's binary accepted by its pins, nothing rebuilt

**The pins:**
- `S4_PLUGIN_SHA256 9b3109af…` and `S4_PLUGIN_SOURCE_SHA256 661a4d71…` are in `sitl_s4f1.py:85-86`.
- On the host, the `.so` is `9b3109af…` and the `.cc` is `661a4d71…` in both MOTAR-realveh and MOTAR-camonly.

**How the age rule is dropped** (`plugin_pin_problems`, :392-406):
- It drops only lines that contain "is older than its source". That is exactly one S4 message (`sitl_s4.py:326`).
- If either pin fails, it keeps the age line and adds its own refusal.

**Nothing else is weakened:**
- "plugin not built" remains.
- So do DISPLAY, TCP 4590 and `nvidia-smi`.
- If S4's wording ever changed, the filter would no longer match and the launch would refuse, which is the safe side.

**Where it is used:**
- `launch_extra_problems` (:498), for every `--smoke` and `--launch`;
- `check()` (:515);
- the smoke's `preflight()` (`component_smoke.py:97-99`).

**Nothing is rebuilt:**
- No `cmake`, `make` or `touch` appears in the runner, the smoke or the chain.
- `S4.PLUGIN_DIR` is only read, through `GAZEBO_PLUGIN_PATH` and SHA-256s.
- In camonly the `.cc` mtime (11-06) is older than the `.so` (11-07), so the age rule would not fire there anyway. The
  pins still apply.

**Tests:** both directions on temporary files, plus the host's real files (the real-file test was not skipped).

### MAJOR-2: C4 excludes the bar feet

- **Mask:** `bar_foot_masks` (:187) is a ray–box test with hit z < 0.10 m. `floor_returns` (:233) counts what
  `remove_ground` removes outside the foot mask dilated by 1 px.
- **C4** (:346): 0 removed outside the feet, and removed depth ≤ the dilated mask (70).
- **Offline controls in the dry run:** C4 passes without a floor, fails with one, and C5 passes. The chain gates on all
  three (`run_s4f1.sh:53-57`).
- **Remaining inference:** the dilation assumes that Gazebo's depth pixels line up with MOTAR's pixel-centre rays
  within 1 px. That is INFERRED. S4's smoke colour centroid, u = 101.5, agrees with the mask's u = 101.1. A larger
  misalignment would make C4 FAIL, which stops the unit; it would not let a false PASS through.

### MAJOR-3: replication of S4's `ground` arm checked

**`replication`** (:725-756) checks:
- that the stack core of every VALID row equals S4 `ground`'s single core, which includes `s4_plugin`;
- that `lit_rendered`'s bound world equals S4's per env;
- that the world model files at launch equal today's.

**`record_problems`** (:759-790) checks, and VOIDs the reading on failure:
- one S4 world per scene, with `lit_rendered` equal to it;
- one stack core across the four cells.

**`WORLD_MODEL_SHA256`** (:91-101, 9 files) is:
- refused at launch;
- recorded in `attempt.json`;
- checked again in the reading (VOID on failure).

**Tests:** positive and negative controls. The real S4 ledger gives 1 core and 39 worlds, with `s4_plugin` in the core.
A small gap remains (N4).

### MINOR-5: per-episode GPU check

- `make_run_one` (:470-490) waits once for 30 s. Then it returns GPU_BUSY when a job is listed, or GPU_UNCHECKED when
  `nvidia-smi` fails.
- **How the stop rules see these rows:** `SF.episode_valid` makes such a row INVALID, with the outcome as its reason.
  So it counts toward the 16 cap and the 3-consecutive rule, and a first-episode GPU_BUSY aborts the attempt.
- The snapshot and `loadavg` are stored in every row. The cell smokes use the same `run_one`.
- The check runs only after the previous episode's session has exited, so the episode's own gzserver is not listed.
  (A lingering one would already be refused by the owned session's foreign-process check.)
- Tested, including that no episode runs after a refusal.

### MINOR-6: binding of the smoke and the runner

**The SHA-256s agree:**
- The runner pins `14925afb…`, which equals the file (tested, and confirmed by the dry run).
- The prereg states `14925afb…` and the runner's `5c08b71a…`; both equal today's files.
- There is no cycle: the runner holds no SHA-256 of the prereg or of the tests.

**The gate:** `component_smoke_problems` (:423) gates every launch on:
- the verdict PASS with C1–C9 true;
- the pinned script;
- the runner's current bytes.

It is tested with 5 negative controls and an unreadable file.

**Not resolved:** the authorization (N1).

## New findings

### N1, MINOR: the component smoke runs on the GPU with no on-disk approval gate; the cited approval is not on disk

**Evidence:**
- The chain runs the component smoke, which renders with OpenGL on the RTX 3070 for about 15 min, at step 2
  (`run_s4f1.sh:59-63`).
- The first `execution_approval.json` check comes later, in `S3C.launch_unit` → `SF.launch_problems`, at the first
  cell smoke (step 3).
- `component_smoke.py` itself checks no approval (`preflight`, :97-104).
- The prereg (:87) and the smoke's docstring cite "the user-approved S4 follow-up (approval 2026-11-08 12:07 KST)".
  No file under `~/motar_tmp/s4f1_20261108/`, `~/motar_tmp/sitl_next_20261108/` or `agent_sessions/` records that
  approval or its wording. `execution_approval.json` does not exist yet.
- AGENTS.md (:28-42, :63) requires the user's explicit approval for every GPU run. Approvals are single-use and name
  their scope.

**Risk:** a chain launched before the approval file exists spends a GPU slot on the component smoke, then refuses at
step 3.

**Fix (cheap: the chain is not SHA-pinned):**
- At the top of `run_s4f1.sh`, before step 1, stop unless this is empty:
  `SF.approval_problems(json.load(open("$U/execution_approval.json")), <HEAD>, scope=S4F1.SCOPE)`.
  `SCOPE` already names the component smoke as the precondition.
- Separately, quote the user's 12:07 approval wording in the unit directory or the WORKLOG at commit, and confirm it
  covers OpenGL GPU use for the component smoke. If it does not, the user's approval of the SCOPE file covers it.

### N2, LOW: Holm lowers primary power by more than "slightly"

**Evidence:** probe Q1 (`probes/q1_holm_power.py`, `.out.json`; r1 P4's model, 20 000 draws, G null). Power of main A,
raw at α = 0.05 against Holm over the two primary contrasts:

| True rise | Raw, α = 0.05 | Holm |
| --- | --- | --- |
| +14 pp | 0.31 | 0.23 |
| +20 pp | 0.60 | 0.50 |
| +25 pp | 0.83 | 0.75 |
| +30 pp | 0.95 | 0.92 |

**Where:** the prereg's Power section (:214-216) says "about 80 % power at +25 pp … Holm … lowers this slightly".

**Fix:** state "about 75 % at +25 pp under Holm when the other factor has no effect (83 % unadjusted)". This is a
prereg wording change only. Note that it changes the prereg's SHA-256, but nothing pins the prereg.

### N3, LOW: no test binds the prereg's stated SHA-256s to the files

**Evidence:** the prereg states the runner's and the smoke script's SHA-256s (:88-91). `test_pinned_script_is_the_prepared_one`
checks only the runner's pin against the script. If the runner changes at commit (IMPLEMENTATION open issue 1: a
`PREREG` date change), the prereg's stated runner SHA goes stale silently. The launch gate still binds the smoke to
the runner's current bytes, so this affects the record only.

**Fix:** a test that asserts `SF.sha256_file(runner)` and `COMPONENT_SMOKE_SHA256` both occur in the prereg text.
There is no cycle: the runner holds neither the prereg's nor the test's SHA-256.

### N4, LOW: `replication` can report SAME_STACK_AND_WORLDS without some of its comparisons

**Evidence:** `sitl_s4f1.py:748`. The launch-to-now model-file comparison is skipped when `world_models_now` is None.
`analyze` sets it to None when `world_model_files()` raises (:951-955). The world comparison is also vacuous when
`worlds_compared == 0`. In both cases the status still reads `SAME_STACK_AND_WORLDS`.

The launch-time model files are still checked against the pins as an integrity item, so this is a reporting gap only.

**Fix:** add `"models_compared": bool`, and use the status `INCOMPLETE` when the models were not compared or
`worlds_compared == 0`.

### N5, LOW: prereg wording on GPU_UNCHECKED

**Evidence:** the prereg (:133-134) reads as if a failing `nvidia-smi` gives GPU_UNCHECKED at once. The code waits 30 s
and re-checks first (:482-486).

**Fix:** "a listed job or a failing `nvidia-smi` gets one 30 s wait, then GPU_BUSY or GPU_UNCHECKED".

### N6, LOW (informational): Gazebo's own `/usr/share/gazebo-11/models` has another `ground_plane`

**Evidence:**
- `/usr/share/gazebo-11/models/` holds `sun` (byte-equal to PX4's, `d73ffc90…`) and `ground_plane`, whose
  `model.sdf` (`a43cdd93…`) differs from PX4's pinned one (`23d45725…`).
- The owned session's `GAZEBO_MODEL_PATH` is `":<PX4 models>"`, for two reasons:
  - `SF.env_problems` refuses inherited `GAZEBO_*` (`sitl_s3b_formal.py:93`);
  - `_append` prefixes ":" (`sitl_owned_session.py:898-899`).
- Gazebo 11 uses its compile-time default path only when the variable is empty. So PX4's `ground_plane` is the one
  loaded, as pinned (INFERRED from Gazebo's `SystemPaths`; not observed).
- Even if Gazebo's copy were loaded, its plane lies at z = 0, under the 200 m asphalt box whose top is at z = 0.05 m.
  It would not be visible to the cameras or to gpu_ray in the rendered cells.

**Fix:** none required. Optionally add one line to the prereg, or check the component smoke's `gzserver --verbose` log
for the model paths it resolved.

## Observations (not findings against S4F-1)

- **The realveh tree is no longer clean.** `MOTAR-realveh` now has one modified tracked file,
  `tests/test_navrl_sensor_ablation.py`. Its mtime is 13:51 KST, during this review, and the change adds
  `realveh_entry.py` to the ablation allow-list. It is another session's realveh/camk2 work, and no S4F-1 source
  imports or binds it. IMPLEMENTATION.md's "git diff --stat empty" no longer holds for that worktree.
- **The commit HEAD will differ from realveh's.** The chain defaults to MOTAR-camonly, which is clean on tracked files
  at HEAD `971ae67` (the CAM-K1-E preregistration on top of `814c28a`; none of S4F-1's bound sources change). So
  the S4F-1 commit will sit on a different HEAD than realveh's. The cell smokes and attempt01 bind to whatever HEAD
  they run at, which is fine.
- **CAM-K1-E would compete for the GPU.** If it is approved and run, the per-episode GPU check turns S4F-1 episodes
  into GPU_BUSY, and three in a row abort the attempt. The two runs must not overlap. The chain's own launch-time
  refusal covers the start.

## Probes (`probes/`)

| File | What |
| --- | --- |
| `tests_s4f1_reviewer_r2.log` | 53 tests OK, 0 skipped |
| `dryrun_r2.json` | the component smoke's offline dry run against MOTAR-realveh (started nothing, wrote nothing) |
| `q1_holm_power.py`, `q1_holm_power.out.json` | the effect of Holm on main-A power (N2) |

**Not done:** anything on the GPU, Gazebo or PX4; `--check`; the full PUBLIC_CPU suite. The real-path items stay as
listed in IMPLEMENTATION.md ("What only the component smoke or a real run can verify").
