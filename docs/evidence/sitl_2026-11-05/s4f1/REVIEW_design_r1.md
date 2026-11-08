# S4F-1 v1 design review r1 (prereg draft, runner, tests, component smoke)

- **Reviewer:** Claude Opus 5.5 (fresh context), independent design reviewer. Read-only on every repository. Wrote only
  to `~/motar_tmp/s4f1_20261108/review_design_r1/`.
- **Date:** 2026-11-08, 13:05–13:35 KST.
- **Nothing was started on the GPU.** No Gazebo, gzserver, PX4, Isaac Gym or CUDA was run. `--check` was not run.
  Only CPU unit tests and numpy probes were run (cores 6,7, `nice -n 19`, `ulimit -v 8000000`, `CUDA_VISIBLE_DEVICES=""`).
- **Worktree:** `MOTAR-realveh`, branch `realveh/dev`, HEAD `814c28a`. `git diff --stat` is empty, so S4's and S3c's
  tracked files are unchanged.

| File reviewed | SHA-256 (matches IMPLEMENTATION.md) |
| --- | --- |
| `tools/sitl_s4f1.py` (644 lines, untracked) | `04dc8bd21fde2681…` |
| `tests/test_sitl_s4f1.py` (735 lines, untracked) | `053a44a0f0530f2f…` |
| `~/motar_tmp/s4f1_20261108/PREREG_DRAFT.md` | `fbbc07c6f69bf4d2…` |
| `~/motar_tmp/s4f1_20261108/component_smoke.py` (not run) | `a33c18ec0d362ee8…` |

**Tests:** `test_sitl_s4f1` gives 34 tests, OK, 0 skipped, 25 s (`probes/tests_s4f1_reviewer.log`). `pyflakes` is
clean on all three Python files, and no line is longer than 120 characters. I did not run the full PUBLIC_CPU suite.

## Verdict: **AMEND_FIRST**

There is no BLOCKER and no observation-contract violation. The factor manipulations are right:
- each cell changes exactly its own strings;
- `lit_rendered` is S4's world byte for byte;
- the collision-only floor keeps the physics.

The ULog gzip is correct and fails safe.

Three MAJOR findings must be fixed before the draft is committed and before the component smoke runs, because the
smoke's thresholds and the analyzer may not move afterwards:
- **MAJOR-1:** as the tree stands, neither the component smoke nor any S4F-1 launch can start. S4's plugin-age refusal
  fires because the worktree checkout gave the `.cc` today's mtime. The obvious repair, a rebuild into S4's
  `PLUGIN_DIR`, would overwrite the `.so` that S4 attempt01's records bind.
- **MAJOR-2:** the component smoke's C4 fails by construction. The front bar's foot is 36 depth pixels below 0.10 m,
  and C4 allows 0.
- **MAJOR-3:** the analyzer does not check the claim that `lit_rendered` replicates S4's `ground` arm against S4
  attempt01's recorded stack and per-scene worlds.

The MINOR findings concern:
- the power statement, which has an arithmetic error and overstates detectability;
- multiplicity;
- attrition and ABORTED handling;
- wording;
- GPU and host idleness during the run;
- binding of the smoke script;
- manipulation checks in the real path;
- the rangefinder premise, which holds physically but is not stated or checked.

## Answers to the seven questions

1. **Does `lit_rendered` reproduce S4's `ground` arm? Yes, in code and in today's stack; not yet verified at run time.**
   - **World text:** `CellSession.write_world` returns S4's text unchanged for `lit_rendered` (`sitl_s4f1.py:115-130`,
     `:136-141`). This is tested on a fake world and on the real `empty.world`.
   - **Code path:** the episode is `S4.run_episode(..., "ground")`, called unchanged (`:293`).
   - **Stack today:** probe P3 re-hashed all 58 non-world files of S4 attempt01's runtime binding: 58 of 58 are equal
     today. That covers the PX4 binary, the airframe, the model SDF, `s4_plugin` `9b3109af…` and the Gazebo plugins.
   - **Plugin source:** its bytes equal S4's recorded `plugin_source_sha256` (`661a4d71…`).
   - **Gaps:** the analyzer never compares any of this with S4 attempt01 (MAJOR-3). The plugin-age refusal will force
     a rebuild that may change the bytes (MAJOR-1).
2. **Is each factor changed exactly and only as stated? Yes.**
   - **Material:** the material string is replaced once (`:125-126`).
   - **Ground:** the two includes are replaced by inline models with the same names, no pose, and the PX4 tree's
     collision elements (`:83-94`). This was tested against `model.sdf`.
   - **Which floor file:** Gazebo picks `asphalt_plane/model.sdf`, SDF 1.5, 200 × 200 × 0.1 m (`model.config` lists
     1.4 → `model-1_4.sdf`, 20 × 20, and 1.5 → `model.sdf`). The inline copy matches it.
   - **Physics:** contact, friction and bitmask are identical. Physics does not read visuals, so the resting height,
     the truth origin, the 0.14 m crash rule and the hover guard are unchanged.
   - **PX4 rangefinder:** `navrl_quad_s4.sdf:537-546` includes `model://lidar`, whose sensor is
     `<sensor name="laser" type="ray">` (`lidar/model.sdf:28`, `libgazebo_lidar_plugin.so`).
     - A Gazebo Classic `ray` sensor is a CPU `RaySensor` intersecting the physics engine's collision geometry
       (`MultiRayShape`). It is not rendered, so it still sees the collision-only asphalt top at z = 0.05 m.
     - Only `gpu_ray` and the cameras use the OGRE scene.
     - Airframe 10092 sets `EKF2_HGT_REF 2` and `EKF2_RNG_CTRL 2` (`:64-65`), so the EKF height reference is unchanged.
     - The prereg does not say any of this and nothing checks it (MINOR-8).
   - **The ground filter is independent of the EKF:** it uses truth height + 0.09 m (`sitl_s4.py:240`).
3. **Does emissive change what the colour rule sees? Very likely, and the smoke is the right test.**
   - **Mechanism:** SDF 1.5 has `<emissive>` (`/usr/share/sdformat9/1.5/material.sdf:44`), and Gazebo 11 maps it to the
     OGRE pass's self-illumination. With ambient, diffuse and specular at 0, the colour is flat (224, 20, 11), logit
     1.492, so it passes the 0.55 threshold.
   - **Edge pixels:** Gazebo 11 cameras use 4× MSAA by default (`1.7/camera.sdf:27`), so edge pixels blend with the
     background.
     - Probe P2 needs pixel coverage ≥ 0.52–0.62 to pass, depending on the background grey.
     - It predicts a pass fraction of 0.94 (32/34) on the 34-pixel mask for every background from 0.3 to 1.0, with no
       passes outside the mask (INFERRED).
     - So C1 ≥ 0.80 is a sensible threshold with a margin of about 5 pixels.
   - **The lit control:** C2 ≤ 0.50 is a sensible negative control. S4 recorded 4 of 35, about 0.11.
   - **C3:** sensible. Its mask-size range is fixed geometry (LOW-3).
   - **C4:** broken (MAJOR-2). C5 and C6 are sound positive controls; P1 reproduces C6's 2.80, 1.80 and 2.70 m.
   - **C7–C9:** sound, with a small caveat on the ODE iteration count (LOW-4). They do not cover the rangefinder.
4. **Analysis:**
   - **Correct as implemented:** the per-scene contrasts and the sign test (it matches scipy `binomtest`), the
     McNemar tests, the complete-scene pairing and the descriptive comparison with S4.
   - **The power statement is wrong in places** (MINOR-1).
   - **Multiplicity:** 14 uncorrected p-values, read through "reached the exact test" (MINOR-2).
   - **Attrition and ABORTED readings** are unspecified (MINOR-3).
   - **The interaction sign test** only approximates "no interaction" (LOW-6).
5. **ULog gzip: correct and fail-safe.**
   - The `.part` file is opened with `xb`, fsynced, decompressed and matched on SHA-256 and size. The original is
     re-hashed, the existing `.gz` is re-checked, then the part is renamed and the original unlinked (`:223-256`).
   - Every failure keeps the original and removes the part.
   - The header is deterministic: no FNAME, mtime 0, OS 255.
   - The session receipt (`session.json`) names no ULog path (checked on S4 ep000), so removing the original leaves
     no dangling reference.
   - Small gaps are listed under LOW-5.
6. **Preflight, idle, fault, disk and the observation contract:**
   - **Correct:** fault and tree stops per episode, the single-use directories, the approval naming SCOPE and HEAD,
     the disk floors (`DISK_LOW` → ABORTED through `run_unit`'s exception path) and the per-cell smoke gate.
   - **Idle check:** it covers the runner's cores {6, 7, 12, 13} only (`sitl_dev08_attempt.py:58,187-196`).
   - **GPU check:** at launch only (MINOR-5).
   - **Observation contract: PASS.**
     - No ground-truth target information enters the actor.
     - The target's truth trajectory only places the Gazebo model.
     - The shadow render is never fed to perception.
     - The geometric mask exists only in the smoke.
     - Vehicle truth attitude and height feed the ground filter, as preregistered in S4.
7. **Wording:** mostly sound. The two factors are compound treatments and the "remaining gap" sentence attributes too
   much (MINOR-4). The RedGlow analogy is loose (LOW-2).

## Findings

### MAJOR-1: the plugin-age refusal blocks the smoke and the launch; the naive repair damages S4's evidence

**Evidence:**
- `tools/sitl_s4/motar_s4_bridge.cc` has mtime 2026-11-08 12:24:51, the worktree checkout time. `AGENTS.md` and
  `sitl_s4.py` share it.
- `~/motar_tmp/s4_gazebo_20261106/build/libmotar_s4_bridge.so` has mtime 2026-11-07 12:33:19.
- `S4.s4_extra_problems()` refuses when `.so mtime < .cc mtime` (`sitl_s4.py:325-326`).
- That function is called by:
  - `S4F1.launch_extra_problems` (`sitl_s4f1.py:333`), for every `--smoke` and `--launch`;
  - `component_smoke.preflight` (`component_smoke.py:82`).
- The bytes are nevertheless S4's own:
  - source `661a4d71…` = S4 attempt01's `plugin_source_sha256`;
  - `.so` `9b3109af…` = S4 attempt01's bound `s4_plugin`.
- S4's chain rebuilt the plugin in `S4.PLUGIN_DIR` (the `.so` mtime is 12:33, one minute before S4's smoke).

**Risk:** the same chain step here would overwrite the binary that S4 attempt01's 60 ledger rows bind by path and
SHA-256. Unless the build is byte-reproducible, that would also break the replication claim (MAJOR-3). The prereg
lists the refusal (line 187) but not the procedure.

**Fix** (choose one and write it in the prereg):
- **(a), preferred:** pin both SHA-256s in `sitl_s4f1.py`: `S4_PLUGIN_SHA256 = 9b3109af…` and
  `S4_PLUGIN_SOURCE_SHA256 = 661a4d71…`.
  - In `launch_extra_problems` and in the component smoke's preflight, drop only S4's "older than its source" problem
    when both pins match, and refuse otherwise.
  - S4's file stays byte-unchanged, nothing is rebuilt, and `lit_rendered` runs S4's binary exactly.
  - Test both the drop and the refusal, with positive controls.
- **(b):** rebuild from the committed source into a new S4F-1 directory, never into S4's.
  - Point `S4.PLUGIN_DIR` (a module attribute) at it from `sitl_s4f1`, and record that in `attempt.json`.
  - Report whether the new `.so` equals `9b3109af…`. If it does not, disclose that `lit_rendered` runs a rebuilt
    binary of the same source.

Do not `touch` S4's evidence file.

### MAJOR-2: component-smoke C4 cannot pass; bar-foot depth pixels count as "floor"

**Evidence:**
- **Rig geometry:** `rig_world` puts the front bar's box at centre (3.0, 0, 1.5), size 0.4 × 0.4 × 3, so it spans
  z 0–3 m (`component_smoke.py:95-98`).
- **The camera sees the foot:** the camera at (0.10, 0, 1.03) with a 28.1° half vertical field of view sees the bar's
  foot below z = 0.10 m in image rows 74–76.
- **C4 counts it:** `remove_ground` counts every depth pixel whose hit point is below 0.10 m (`sitl_s4_gazebo.py:200-203`).
  C4 requires LiDAR + depth = 0 with the ground unrendered (`component_smoke.py:63,265-266`).
- **Probe P1** ray-casts the rig with no floor at all and feeds the runner's own `remove_ground`:
  `{"lidar_ground_removed": 0, "depth_ground_removed": 36}`. Its bar ranges match C6 (2.800, 1.800, 2.700 m).

**Consequence:** both unrendered cells fail C4 whether or not Gazebo draws the collision-only floor. The verdict
becomes FAIL, and under "thresholds are not moved after the smoke" the unit goes back to design for the wrong reason,
spending a GPU slot.

**Fix, before the smoke runs:**
- Compute the bar-foot mask geometrically, as P1 does: ray–box hits with hit z < 0.10 m.
- Define C4 as: LiDAR floor returns = 0, and depth pixels removed outside that mask (dilated by 1 px) = 0.
- Optionally also require the removed depth count ≤ the mask's 36.
- C5 is unaffected; its floor counts include these 36.
- Record the mask count in `result.json`.
- An alternative is to lift the bars off the floor, but then `lit_rendered`'s rig world is no longer byte-equal to
  S4's `smoke.world` (`1eddb2e8…`).

### MAJOR-3: the replication of S4's `ground` arm is claimed but not checked

**Evidence:**
- The prereg reads `lit_rendered` against attempt01 as "a reading of boot-to-boot and day-to-day variation"
  (`PREREG_DRAFT.md:40-41`; reported item 4, `:142-143`).
- `vs_s4_ground` compares outcomes only (`sitl_s4f1.py:510-519`).
- `record_problems` checks cross-cell binding equality within S4F-1 (`:522-542`), but never against S4 attempt01's
  binding.
- The S4 ledger holds both comparison points, and is already pinned by `522f716e…`:
  - one binding core across its 39 VALID `ground` rows;
  - a per-scene world SHA-256 for each of those 39 envs (P3).
- **Not bound:** the runtime binding covers the vehicle SDF's `model://` resources, but not the world's `model://sun`,
  `ground_plane` and `asphalt_plane`, including the asphalt textures (`sitl_owned_session.py:1023-1026`). These
  determine `lit_rendered`'s rendered appearance.

**Fix (analyzer and record, before commit):**
- In the reading, report under `integrity_problems` or a `replication` block:
  - **Binding core:** S4F-1's `binding_core` across all VALID rows equals S4 attempt01's `ground` core (it includes
    `s4_plugin`).
  - **Worlds:** for every env VALID in both, `lit_rendered`'s bound world SHA-256 equals S4 attempt01's world SHA-256
    for that env.
  - **One base world per scene:** within each scene, the four cells' `cell_world.s4_world_sha256` are equal and equal
    `lit_rendered`'s `world_sha256`.
- At launch, record in `attempt_extra` the SHA-256 of every file under the resolved `sun`, `ground_plane` and
  `asphalt_plane` model directories, and compare them with today's values in the reading.
- Then the prereg can say "same stack, same world bytes; differences are run-to-run". Add a test with a positive
  control (equal) and a negative control (one changed hash).

### MINOR-1: the power statement has an arithmetic error and overstates detectability

**Evidence (P4, using the runner's own `sign_test_p`):**
- 8 of 10 discordant gives p = 0.109. It needs 9 of 10 (p = 0.021). The other claimed splits are right: 10/12 0.039,
  13/16 0.021, 15/20 0.041.
- "With 24 or more discordant scenes it needs 30 pp" holds for 24–31 discordant scenes. From 32 on it needs 35 pp
  (23/32, 27/40).
- These numbers are minimum observed splits, not power. Simulated power at α = 0.05 uses independent Bernoulli
  outcomes, lit 0.513, and no scene heterogeneity (INFERRED):

| True rise | Simple effect (McNemar) | Main effect (sign test) |
| --- | --- | --- |
| +14 pp | 0.18 | 0.31 |
| +20 pp | 0.36 | 0.60 |
| +25 pp | 0.55 | 0.83 |
| +30 pp | 0.75 | 0.95 |

- With only 24 complete scenes, +25 pp gives 0.32 and 0.58.

**Fix:** correct the splits. State power, not "detectable": main effects have about 80 % power at +25 pp, simple effects
about 55 %; a +14 pp rise is detected less than a third of the time. Keep "a non-significant result is not evidence of
no effect".

### MINOR-2: multiplicity, and "reached the exact test"

**Evidence:** the draft reports 14 p-values uncorrected (`PREREG_DRAFT.md:140-141`), but "May" allows "whether each
factor's paired effect reached the exact test" (`:159-160`). That works as a significance label per factor.

**Fix:** name the primary contrasts now. For example: main A and main G on capture, two tests, Holm at 0.05. Then
"reached" refers only to those (Holm-adjusted p reported beside the raw p). Everything else (interaction, simple
effects, crash) is secondary and descriptive. Add the Holm values to the analyzer.

### MINOR-3: attrition, missing scenes and ABORTED attempts are not specified

**Evidence:**
- The paired analysis uses only scenes VALID in all four cells (`sitl_s4f1.py:444`).
- The cap allows 16 INVALID, so up to 16 of 40 scenes can drop. Power then falls to the 24-scene row above.
- INVALID can differ by cell: LATE depends on render cost, which the unrendered ground changes. The data would then be
  missing not at random.
- `reading_from` labels every reading "reported; no pass or fail label" (`:585`). This includes an ABORTED attempt and
  one with a kernel-fault rise, which AGENTS.md says "are not evidence".

**Fix (prereg + analyzer):**
- Fix a minimum number of complete scenes for the paired block to be read, for example ≥ 32. Below that, label it
  `PAIRED_LIMITED`.
- Report per-cell INVALID counts by reason (already in `block_summary`). Add a sensitivity: per-cell rates over each
  cell's own VALID scenes, which item 1 already gives.
- State that a fault rise voids the reading (`NOT_EVIDENCE`), and that any other ABORTED attempt is reported as
  INCOMPLETE and descriptive only.
- Optionally run the simple McNemar tests over scenes VALID in the two cells concerned.

### MINOR-4: wording; both factors are compound, and the "remaining gap" over-attributes

**Factor G** changes:
- the residual ground LiDAR returns that the 0.05 m margin lets through;
- ground depth pixels that pass the filter at tilt;
- the RGB background behind and around the target (asphalt → 0.7 grey, which also changes anti-aliased edge pixels);
- render cost, and so frame age.

The Question frames G as the filter margin (`:20-21`), but the design cannot attribute a G effect to the margin.

**Factor A** changes:
- the fraction of the target the colour rule passes, and so how much of it is blanked;
- also detection range, centroid and bearing.

The "may not" covers blanking only (`:162-164`).

**Item 6** (`:145-146`) says the remaining gap "is what mount, FOV/pixels, frame timing and the depth range still carry
together". The gap also includes:
- the LiDAR target radius (0.15 m in Gazebo against MOTAR's 0.20 m, per S4's table);
- anti-aliased target edges;
- the 0.10 m filter acting on bar feet (absent in S3b);
- run-to-run variation.

**Fix:**
- Add "may not: attribute a G effect to the filter's margin; G includes background, residual-ground and render-timing
  changes".
- Extend A's "may not" to detection range and bearing.
- Phrase item 6 as "includes at least …", consistent with, not causal.

### MINOR-5: idle host and GPU are checked only partly during a real-time acceptance-type run

**Evidence:**
- The per-episode idle check samples cores 6, 7, 12 and 13 only (`sitl_dev08_attempt.py:58,187-196`).
- The CUDA-job check runs once at launch (`sitl_s4.py:337-344`).
- The prereg promises an "idle host" and "no other GPU job may run" (`:192-194`).
- Gazebo renders on the shared RTX 3070. A CUDA evaluation started by another session mid-run, which the brief warns
  of today, would be seen only indirectly, as LATE.

**Fix:**
- Per episode, record the `nvidia-smi --query-compute-apps` snapshot and `/proc/loadavg` in the ledger row.
- Treat a listed non-RustDesk job like HOST_BUSY (one wait, then INVALID without running), or at least report it.
- State plainly in the prereg that the idle check covers the runner's cores only, as S4's README does.

### MINOR-6: the component smoke's authorization and binding

**Evidence:**
- The smoke renders on the GPU before the prereg is committed (`component_smoke.py:1-3,21-24`).
- AGENTS.md requires the user's explicit approval for any GPU use. S4 set a precedent with its pre-prereg smoke.
- The prereg names the script by path only, with no SHA-256 (`PREREG_DRAFT.md:69-71`).
- The smoke imports `sitl_s4f1` from the uncommitted worktree. It records `runner_sha256` and `script_sha256`, but no
  rule binds them to the committed bytes.

**Fix:**
- In the committed prereg, give the script's SHA-256 and the runner's SHA-256 at smoke time.
- Require that the committed `tools/sitl_s4f1.py` equals the runner the smoke used, or that the smoke is rerun.
- Name the smoke explicitly as needing the user's approval for GPU (OpenGL) use.
- After fixing MAJOR-1 and MAJOR-2, the script's SHA-256 changes; bind the final one.

### MINOR-7: no manipulation check in the real flight path

**Evidence:**
- The component smoke checks the factors on a static rig. The per-cell development smokes gate only on COMPLETE +
  VALID (`PREREG_DRAFT.md:93-95`).
- The reading summarises colour and shadow pixel medians per cell (`sensor_summary`).
- It does not summarise the per-step ground-removed counts already logged in `sensor_steps.npy` (columns 11 and 12;
  `sitl_s4.py:254`).

**Fix:**
- Report per cell the median and maximum LiDAR and depth ground-removed counts per step, and at hover (step 0).
- Expected: unrendered ≈ bar feet only; rendered ≫.
- Report the ratio of colour-rule to shadow pixels.
- Optionally, fix in the smoke gate that an unrendered smoke's hover-step LiDAR ground-removed count is 0.

### MINOR-8: the rangefinder premise holds but is neither stated nor checked

**Evidence:** see answer 2.
- The airframe comment (10092 `:61-63`) says that with the barometer the EKF "still lagged the truth by 0.1–0.15 m".
- S4 attempt01's `start_check.ekf_alt_m − truth_alt_m` was −0.006 to +0.020 m over its 40 `ground` episodes, and
  −0.013 to +0.020 m over its 20 `raw` episodes (probe P6).
- So the start check can tell rangefinder height (≤ 0.02 m) from a barometer fallback (≥ 0.1 m).
- Without a check, a dev smoke would stay VALID under the fallback: the start check allows ±0.3 m
  (`sitl_s4.py:194`).

**Fix:**
- Add one sentence to the prereg's unrendered-ground paragraph: the PX4 rangefinder is a physics `ray` sensor, so it
  still returns from the collision floor.
- Report `ekf_alt_m − truth_alt_m` at the start per cell in the reading.
- Fix a gate on the unrendered smokes, for example |difference| ≤ 0.05 m.
- Optionally add a component-smoke criterion: the `model://lidar` ray sensor on the 1 m rig reads about 0.95 m in
  both ground variants.

### LOW findings

1. **Carry-over claim overstated.**
   - Within scenes, every ordered pair occurs exactly 10 times (P5).
   - Across scene boundaries, four transitions repeat, so each cell has one predecessor 19–20 times and the others 10.
   - "The previous episode's cell fall[s] evenly on every cell (tested)" (`PREREG_DRAFT.md:62-63`) holds within
     scenes only, and only that is tested.
   - Each episode is a fresh boot, so the effect is negligible. Correct the sentence.
2. **RedGlow analogy.** `Gazebo/RedGlow` keeps ambient = diffuse = 1 0 0 and adds emissive (`gazebo.material:414-432`),
   so it is not emissive-only. Say "uses the emissive term; here ambient, diffuse and specular are zeroed so that only
   it remains".
3. **C3's mask-size range is not a measurement.** The 20–45 px range is a deterministic calculation (always 34). Call
   it a geometry sanity check.
4. **ODE iterations in the rest probe.** The rest probe uses `empty.world` with ODE `<iters>10</iters>`. S4's world uses
   100 (`SF.world_text`). The tolerance is wide enough, but using 100 would match the run.
5. **ULog gzip gaps.**
   - `analyze` re-hashes `.gz` files but does not decompress them. `verify_ulog_record` is unused there; calling it costs
     little.
   - There is no directory fsync between the rename and the unlink. The risk is negligible on ext4 with ordered
     journaling.
   - `wall_s` now includes the gzip; this is disclosed.
6. **The interaction sign test** checks sign symmetry of the per-scene contrast. With binary outcomes and unequal base
   rates it is only approximately the "no interaction on the risk-difference scale" null. Say so in one line.
7. **"SHA-bound" for S3b is a recorded SHA, not a pin.** `S3C.s3b_reference` records the S3b ledger's SHA but does not
   pin it (`sitl_s3c.py:173-181`), unlike S4's `522f716e…`. Pin it, or say "recorded".
8. **Untested paths, and pre-commit work.**
   - The `run_one` closure's DISK_LOW path, from raise to ABORTED with no ledger row, is not unit-tested.
   - The full PUBLIC_CPU suite, the `docs/EXPERIMENTS.md` family entry and the chain launcher are still to do (the
     implementer lists them).
   - The chain must follow MAJOR-1's procedure and must not rebuild into S4's `PLUGIN_DIR`.

## What is right (checked, not only read)

- **World text per cell:**
  - exact factor diffs, reversible to S4's text;
  - four distinct worlds;
  - refusals of foreign worlds, with positive controls;
  - the real `empty.world`;
  - collisions equal the PX4 model files;
  - S4's material string reproduced byte for byte.
- **Schedule:**
  - the real G8 first 40 scenes equal S4's `ground` schedule (test ran, not skipped);
  - the Williams square has every cell 10 times per position and the within-scene pairs balanced;
  - the design check detects a swap.
- **Run path:**
  - `run_unit` with every cell primary, per-cell binding reference, cap 16 (`> 16` aborts), 3 consecutive INVALID, a
    first INVALID episode, fault and tree stops;
  - smokes single-use, same HEAD and sources, own cell, attempt refused otherwise.
- **Analysis:**
  - `sign_test_p` matches scipy;
  - the contrasts are computed as stated;
  - malformed ledgers are refused without writing;
  - `reading.json` is write-once;
  - S4's ledger is pinned (`522f716e…` re-hashed: equal; 39 VALID, 20 captures).
- **ULog gzip:** as in answer 5. Each episode had one ULog in S4 (60/60).

## Probes (all CPU, in `probes/`)

| Probe | Output | Finding |
| --- | --- | --- |
| `p1_c4_bar_foot.py` | `p1_c4_bar_foot.out.json` | 36 bar-foot depth pixels removed with no floor (MAJOR-2) |
| `p2_c1_edge_margin.py` | `p2_c1_edge_margin.out.json` | expected emissive pass fraction 0.94 under 4× MSAA; C1 sensible |
| `p3_stack_vs_s4.py` | `p3_stack_vs_s4.out.json` | 58/58 S4 binding files unchanged today; 39 per-scene world SHAs available (MAJOR-3) |
| `p4_power.py` | `p4_power.out.json` | split error and simulated power (MINOR-1) |
| `p5_carryover.py` | `p5_carryover.out.json` | boundary carry-over imbalance (LOW-1) |
| `p6_start_alt.py` | `p6_start_alt.out.json` | S4 start EKF − truth altitude −0.013 to +0.020 m (MINOR-8) |
| unit tests | `tests_s4f1_reviewer.log` | 34 OK, 0 skipped |

**Not done:** anything on the GPU, Gazebo or PX4; `--check`; the full PUBLIC_CPU suite. Gazebo 11's emissive
rendering and the collision-only floor's invisibility are inferred from source and SDF defaults until the component
smoke measures them.
