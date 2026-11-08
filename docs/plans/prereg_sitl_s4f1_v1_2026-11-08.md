# S4F-1 v1 — target appearance × ground, a 2×2 factorial on S4's `ground` arm (preregistration, 2026-11-08)

**State:** preregistered, **NOT_RUN**. Runner `tools/sitl_s4f1.py`, tests `tests/test_sitl_s4f1.py`. Outside the
repository: the component smoke `~/motar_tmp/s4f1_20261108/component_smoke.py` and the chain
`~/motar_tmp/s4f1_20261108/run_s4f1.sh`. It runs [S4 v1](prereg_sitl_s4_v1_2026-11-06.md)'s episode unchanged
(`tools/sitl_s4.run_episode`, arm `ground`), with S4's [amendment 1](prereg_sitl_s3c_s4_amendment1_2026-11-07.md).
S4's files stay byte-unchanged. The design comes from the 2026-11-08 survey (`~/motar_tmp/sitl_next_20261108/SURVEY.md`
§Q1). Design review r1 (Claude Opus 5.5, fresh context) read AMEND_FIRST; its findings and their repairs are listed
[below](#design-review-r1-and-repairs). Amendments go in a new dated file.

## Question

S4 attempt01 (2026-11-07) replaced MOTAR's renderer with Gazebo's sensors. With the preregistered 0.10 m ground filter,
F's capture rate on the first 40 eligible 70-bar scenes fell from 100 % (Isaac, S3b) to 20/39 (51.3 %). Several things
changed at once; S4 separated only the ground filter. Two candidates are cheap to separate in Python, with no plugin
rebuild:
- **A, target appearance.** Gazebo shades the lit sphere, so F's colour rule passes only part of it (median 5 of the
  shadow's 26 pixels). F's camera-to-scan fusion blanks only the passing pixels, so the rest may enter the static scan
  as an obstacle at the target's range (S4 results, candidate mechanism, NOT_TESTED).
- **G, the rendered ground.** The 0.10 m world-z filter has a margin of only 0.05 m above the rendered asphalt top
  (z = 0.05 m), and 11 of 39 `ground` episodes kept the ground's LiDAR signature at hover (S4 results, inferred).

How do capture, crash and timeout change with each factor, and do the two interact? Both factors are compound
treatments (see "What may and may not be said").

The unit is **reported, not gated**: there is no GO/NO-GO label.

## Cells

All four cells are S4's `ground` arm: navrl_quad_s4 (visuals removed) with Gazebo's gpu_ray LiDAR, RGB and depth
cameras on a body-fixed mount, Frozen Policy F with its 87° camera, and the 0.10 m world-z ground filter **on in every
cell**. So factor G is whether the ground is rendered.

| Cell | A: target material | G: ground |
| --- | --- | --- |
| `lit_rendered` | S4's: ambient = diffuse = (0.88, 0.08, 0.045), specular 0 | S4's world: `ground_plane` + `asphalt_plane` |
| `emissive_rendered` | ambient = diffuse = specular = 0, `<emissive>` = (0.88, 0.08, 0.045) | S4's world |
| `lit_unrendered` | S4's | the two includes replaced by the same two models with their collisions only |
| `emissive_unrendered` | emissive | collisions only |

- **`lit_rendered` replicates S4's `ground` arm.** Its world text is byte-equal to S4's (tested), and the analyzer checks
  it against S4 attempt01's records (stack binding and per-scene world SHA-256; see "Reported" item 5).
- **Emissive:** the SDF 1.5 `<emissive>` element, mapped by Gazebo 11 to the material's self-illumination. Gazebo's own
  `Gazebo/RedGlow` uses the emissive term together with ambient and diffuse; here ambient, diffuse and specular are
  zeroed so that only the emissive term remains. The intended colour is flat (224, 20, 11), for which the colour rule
  gives sigmoid(1.49) = 0.82 ≥ 0.55. With Gazebo's 4× MSAA, edge pixels blend with the background; review r1 probe P2
  predicts 32 of the 34 mask pixels pass (INFERRED; the component smoke measures it).
- **Unrendered ground:** `ground_plane` (plane at z = 0, μ 100/50, bitmask 65535) and `asphalt_plane` (static box
  200 × 200 × 0.1 m at the origin, top at z = 0.05 m) keep their names, collisions and poses; only their `<visual>`
  elements are gone (tested against the PX4 tree's model files). Gazebo's cameras and gpu_ray render visuals only (S4
  review r2, from the Gazebo 11 source). The resting height, the truth origin's 0.09 m offset and the filter's height
  input are unchanged by construction; the component smoke measures the resting height.
- **Rangefinder premise.** The EKF height reference is the PX4 rangefinder (airframe 10092: `EKF2_HGT_REF 2`,
  `EKF2_RNG_CTRL 2`). navrl_quad_s4 includes `model://lidar`, a Gazebo `ray` sensor: a CPU sensor that intersects the
  physics collision geometry, not the rendered scene, so it still returns from the collision-only asphalt top. S4
  attempt01's start-check EKF − truth altitude was −0.013 to +0.020 m; a barometer fallback lagged by 0.1–0.15 m. Each
  cell's development smoke is gated on |EKF − truth altitude| ≤ 0.05 m at the start check, and the reading reports it
  per cell.
- **Everything else is S4's** and is not repeated here: the episode step for step, the hover guard (0.35 m), the shadow
  render, the frame-age lateness rule, the target's one-step-ahead placement with the 0.09 m offset, the start check,
  termination and the crash rule (0.14 m), parameter read-back, owned fresh-rootfs session, runtime binding (with the S4
  plugin's SHA-256), dataman removal and S4's launch refusals (with the plugin rule below).
- **S4's plugin, not rebuilt.** The runner and the component smoke accept only S4 attempt01's plugin bytes: `.so`
  `9b3109af0b737ad7a4975df21ba5231fdc35b5c4e4319dc0c95d9ff4a7806c92`, built from source
  `661a4d71623aba293cb21feee569837bd5866c7f2357125f432b759196f8e5fa` (pinned in the runner). When both match, only S4's
  "older than its source" rule is dropped (a worktree checkout gives the committed `.cc` a new mtime); otherwise the
  launch refuses. Nothing is rebuilt into S4's plugin directory, and no chain step builds anything.
- **The world's model files** (`sun`, `ground_plane`, `asphalt_plane`, including the asphalt texture; nine files,
  unchanged since 2026-11-04) are pinned by SHA-256 in the runner, refused at launch if they differ, recorded in
  `attempt.json`, and compared again in the reading. S4's runtime binding covers only the vehicle's resources.

## Scenes and order

- **Scenes:** the first 40 eligible 70-bar G8 r2 scenes (seed 4501), which is S4 `ground`'s schedule (tested against
  `tools/sitl_s4.schedule()`). It contains the 20 scenes of S4's `raw` arm. 160 episodes, one boot each.
- **Order:** scene by scene, ascending env. The four cells of scene *i* run in the order of row *i* mod 4 of a 4 × 4
  Williams Latin square, fixed in code (`LATIN_SQUARE`, indices into the cell list above):
  (0, 1, 3, 2), (1, 2, 0, 3), (2, 3, 1, 0), (3, 0, 2, 1).
  Each cell takes each position 10 times, and within a scene every ordered pair of different cells is adjacent 10
  times (tested). Across scene boundaries four transitions repeat, so each cell has one predecessor 19–20 times and the
  others 10 times (review r1 probe P5). Each episode is a fresh boot, so the imbalance is taken as negligible.

## Before the attempt

### Component smoke (development, gzserver only)

- **Authorization:** part of the user-approved S4 follow-up (approval 2026-11-08 12:07 KST), recorded with its wording
  in `execution_approval.json` naming this runner's SCOPE and the commit; the chain checks that file before the
  component smoke (review r2 N1). It renders with OpenGL on the GPU, so it runs only while no CUDA job is listed.
- **Bound bytes:** script SHA-256 `14925afb963c27e6487f4133bd8f5e7f911b91578ac8c5ce51b185fa96f15e40` (pinned in the
  runner as `COMPONENT_SMOKE_SHA256`); runner `tools/sitl_s4f1.py` SHA-256 at preregistration
  `4b079db92c53941a9290e27a2de619ddf918a78248bc2e62f7c7525ddf3ed9cc`. The smoke's `result.json` records the runner's
  SHA-256 at smoke time. Every launch (each cell smoke and attempt01) refuses unless the result is PASS on C1–C9, was
  produced by the pinned script, and by the runner's current bytes; so if the committed runner differs from the one the
  smoke used, the smoke is rerun.
- **What it does:** S4's component-smoke rig (the static sensor block at z = 1 m, a bar 3 m ahead and one 2 m to the
  left, the target at (4, −1, 1), 4.12 m away) in each cell's world (`lit_rendered`'s is S4's `smoke.world` byte for
  byte), and a resting-height probe (navrl_quad_s4 without `mavlink_interface` and without the MOTAR sensors, ODE
  iterations 100 as in S4's worlds) on each ground. Its `--dry-run` checks the pins, the worlds and the offline
  controls of C4/C5 without starting anything.
- **Pass thresholds, fixed before it runs:**

| Criterion | Measure | Pass |
| --- | --- | --- |
| C1 | colour-rule pass fraction over the target's geometric pixel mask (34 px at 4.12 m), both emissive cells | ≥ 0.80 |
| C2 | the same, both lit cells (S4's record: 4 of 35 reddish pixels) | ≤ 0.50 |
| C3 | geometry sanity check: mask 20–45 px; emissive pass centroid within 1.5 px of the mask's; ≤ 5 passes outside the mask dilated by 1 px | all |
| C4 | ground unrendered, sensors 1 m up: LiDAR beams and depth pixels removed as ground (hit point below 0.10 m) outside the bars' own feet (the geometric ray–box hits below 0.10 m, 36 depth pixels and 0 beams, dilated by 1 px); and removed depth ≤ the dilated foot mask | 0 |
| C5 | positive control, ground rendered: LiDAR floor returns ≥ 40 and depth ≥ 1000 (offline ray cast: 68 and 6160) | both |
| C6 | sensors work in every cell: LiDAR 2.80 m (col 36) and 1.80 m (col 18) at 0°, centre depth 2.70 m, ±0.05 m; frames newer than the target | all |
| C7 | resting body-centre z, ground rendered | 0.09 ± 0.01 m |
| C8 | resting z, unrendered minus rendered | ≤ 0.002 m |
| C9 | settled: two readings 1 s apart | ≤ 0.0005 m |

A FAIL stops the unit for an amendment (for example `<lighting>0</lighting>` for C1); thresholds are not moved after
the smoke. Its record is development evidence, not a measurement of this unit. The rangefinder is not measured by the
smoke (disclosed); the cell-smoke altitude gate covers it in the real path.

### Development smokes (one per cell)

- **smoke01_<cell>:** the first scene's episode of each cell, once each, with the same code, into
  `~/motar_tmp/s4f1_20261108/smoke01_<cell>` (`tools/sitl_s4f1.py --smoke <cell>`, four launches). S4's and S3c's launch
  path (`tools/sitl_s3c.launch_unit`) smokes only the first scheduled item; three of the four worlds would otherwise
  first fly in the attempt.
- **Gate:** attempt01 refuses unless all four smokes are COMPLETE with their one episode VALID, of their own cell, at
  the same HEAD and source bytes, with |EKF − truth altitude| ≤ 0.05 m at the start check. The smoke outcome itself
  (capture or not) is not gated and not read. A failed smoke stops the unit for repair and a new review.
- **Approval:** one approval file names the component smoke's precondition, the four smokes and attempt01 (the SCOPE).

## Validity and stop rules

As S4, through `tools/sitl_s3b_formal.run_unit` with every cell primary:
- **Per episode:** idle check of the runner's cores 6, 7, 12, 13 only (HOST_BUSY); a GPU-job check (`nvidia-smi
  --query-compute-apps`, S4's allowed remote desktop excepted) with the load average recorded in the ledger row — a
  listed job or a failing `nvidia-smi` gets one 30 s wait and a recheck, then GPU_BUSY or GPU_UNCHECKED without
  running; runtime binding
  against the first VALID episode of the same cell (BINDING_CHANGED); lateness bound 0.15 s including frame age (LATE);
  parameter read-back (PARAM_MISMATCH); hover guard (HARNESS_INVALID).
- **The unit stops (ABORTED)** on a kernel-fault rise, a tree change, an INVALID first episode, three consecutive
  INVALID episodes, more than 16 of the 160 INVALID (10 %; S4 used 10 of 60), or less than 0.5 GB free disk before an
  episode (DISK_LOW).
- Any exception, termination request or SIGTERM is recorded in `final.json`. **No retries.**

## Disk: ULog compression

- After each episode, after S4's dataman removal, each PX4 ULog (`rootfs/log/*/*.ulg`) is gzipped at level 1
  (deterministic bytes: no name, mtime 0).
- The original is deleted only after the finished `.gz` has been decompressed and matched the original's SHA-256 and
  size, the original was re-hashed unchanged, and the rename was made durable (directory fsync). An existing `.gz` is
  never overwritten. On any failure the original stays.
- Each episode record holds, per ULog: path, size, SHA-256, `.gz` path, size, SHA-256, `verified`, `original_removed`.
  The analyzer decompresses every `.gz` again and matches it to its record.
- `wall_s` per episode includes the gzip (disclosed).

## Reported (fixed analyzer: `tools/sitl_s4f1.py --analyze`)

`reading.json` is written once; a rerun with other bytes is refused. A development smoke has no reading.

**Label of the reading:**
- **VOID** if the attempt is not COMPLETE (ABORTED, terminated), the kernel-fault count rose, or an integrity check
  failed (ledger order, design, protocol, world records, the world's model files, the one-S4-world-per-scene check, one
  stack binding across cells). A VOID reading carries no outcome numbers; host faults are not evidence.
- **INSUFFICIENT** if fewer than 30 of the 40 scenes are VALID in all four cells. Everything below is reported
  descriptively and no primary contrast is read.
- **REPORTED** otherwise.

1. **Per cell:** capture, crash and timeout counts and rates with Clopper–Pearson 95 % intervals; INVALID counts by
   reason; Δ against Isaac and agreement on the same VALID scenes; the matched-scene table against S3b attempt01 (its
   ledger pinned, `97b94dfd…`); timing and state; the start EKF − truth altitude.
2. **Per cell, as S4 reports them:** camera recall (among steps where the shadow renderer has ≥ 2 target pixels, the
   fraction with `camera_visible`), LiDAR-only detections, median colour-rule and shadow pixel counts, frame age, LiDAR and
   depth differences from the shadow, hover-guard failures. The denominator uses MOTAR's yaw-level camera while Gazebo's
   tilts, as in S4.
3. **Manipulation checks per cell** (from `sensor_steps.npy`): LiDAR and depth returns removed as ground per step (median,
   maximum) and at hover (step 0), and the colour-rule to shadow pixel ratio. Expected: unrendered ≈ bar feet only,
   rendered much more; emissive ratio well above lit. Descriptive.
4. **Paired effects** over the scenes VALID in all four cells, for capture and for crash (y = 1 if the outcome occurred):
   - main effect of A: emissive − lit, averaged over G, per scene; main effect of G: unrendered − rendered, averaged
     over A, per scene; interaction A × G: (emissive − lit | unrendered) − (emissive − lit | rendered), per scene; each
     as the mean in pp with an exact two-sided sign test over the scenes with a non-zero contrast;
   - the four simple effects (A given each G, G given each A) with exact McNemar tests on the discordant pairs.
   - **Primary contrasts: the main effect of A and the main effect of G on capture**, two exact sign tests, Holm-adjusted
     over these two at α = 0.05 (raw and Holm p reported). "Reached" refers only to these two, and only in a REPORTED
     reading.
   - **Everything else is secondary and descriptive:** the interaction, the simple effects and all crash contrasts. The
     interaction's sign test checks the symmetry of the per-scene contrast; with binary outcomes and unequal base rates
     it only approximates "no interaction on the risk-difference scale".
5. **Replication of S4's `ground` arm:** S4F-1's stack binding (world excluded, S4 plugin included) equals S4 attempt01
   `ground`'s single binding core; on every env VALID in both, `lit_rendered`'s bound world SHA-256 equals S4's; the
   world's model files at launch equal today's. Status `SAME_STACK_AND_WORLDS`, `DIFFERS` (with the differing keys or
   envs), or `INCOMPLETE` when one of the comparisons could not be made (flags `stack_compared`, `models_compared`,
   `worlds_compared`). With `SAME_STACK_AND_WORLDS`, outcome differences between `lit_rendered` and S4 are run-to-run.
6. **`lit_rendered` against S4 attempt01's `ground` arm** on the scenes VALID in both: pairs, same outcome, captures in
   each, discordant counts and the outcome cross-table. Descriptive, no test (S4's ledger pinned, `522f716e…`).
7. **ULog:** episodes without a ULog, gzip failures, total and compressed bytes, decompression problems. **Host:** GPU_BUSY
   and GPU_UNCHECKED episodes per cell.
8. **The remaining gap** of `emissive_unrendered` against S3b (via item 1's matched table) includes at least the mount,
   FOV and pixels, frame timing, the depth range, the LiDAR target radius (0.15 m in Gazebo, 0.20 m in MOTAR),
   anti-aliased target edges, the ground filter acting on bar feet, and run-to-run variation; it is consistent with
   these, not attributed to any one.

## Power

40 paired scenes, exact two-sided tests at α = 0.05 (review r1 probe P4, using the runner's `sign_test_p`):
- **Minimum one-way splits** for p ≤ 0.05: 8 of 8, 9 of 10, 10 of 12, 13 of 16, 15 of 20 discordant scenes (a net 20–25
  pp of 40); 18 of 24 to 21 of 30 (30 pp); from 32 discordant scenes on, 23 of 32 to 27 of 40 (35 pp).
- **Simulated power** (independent Bernoulli outcomes, lit capture 0.513, no scene heterogeneity; INFERRED):

| True rise | Simple effect (McNemar) | Main effect (sign test) |
| --- | --- | --- |
| +14 pp | 0.18 | 0.31 |
| +20 pp | 0.36 | 0.60 |
| +25 pp | 0.55 | 0.83 |
| +30 pp | 0.75 | 0.95 |
| +40 pp | 0.98 | 1.00 |

- With only 32 complete scenes, +25 pp gives 0.45 and 0.72; with 24, 0.32 and 0.58.
- So, unadjusted, **the main effects have about 83 % power at +25 pp, the simple effects about 55 %**; a +14 pp rise
  (51 % → 65 %) is detected less than a third of the time.
- **Under Holm over the two primary contrasts** (review r2 probe Q1, the same model, the other factor without effect),
  the main effect of A has **about 75 % power at +25 pp** (83 % unadjusted), 50 % at +20 pp (60 %), 23 % at +14 pp
  (31 %) and 92 % at +30 pp (95 %).
- A non-significant result is not evidence of no effect.

## What may and may not be said

- **May:** how F's SITL outcomes change, on these 40 scenes and one boot each, when the target is unshaded and when the
  ground is not rendered, with the ground filter on throughout; whether each primary contrast reached its Holm-adjusted
  exact test in a REPORTED reading; the per-cell recall and manipulation checks.
- **May not:**
  - **attribute a G effect to the filter's margin.** G changes the residual ground LiDAR returns the 0.05 m margin lets
    through, the ground depth pixels that pass the filter at tilt, the RGB background behind and around the target (and
    so its anti-aliased edge pixels), and the render cost and so the frame age;
  - **attribute an A effect to blanking.** A changes the fraction of the target the colour rule passes, and so how much
    of it is blanked from the static scan, and also the detection range, the centroid and the bearing; the candidate
    mechanism stays "consistent with", NOT_TESTED as a cause;
  - anything about real-camera detection: the target is still a red sphere under F's colour rule, the real target is a
    drone, and its detector lineage (YOLOv5s on NPS and Det-Fly) is not connected;
  - that an onboard ground filter is unnecessary: an unrendered ground is a simulator-only condition;
  - mount, FOV, timing or depth-range effects: they are held at S4's values in every cell;
  - S3b acceptance, flight readiness or any real-flight claim.
- **No gate label.** The question has no claim with a margin attached (S4 had none either); 40 scenes have limited
  power; and the next unit (S4F-2) is chosen from which factor moves, which the reported effects show.

## Observation contract

Nothing from ground truth enters the actor observation. In particular, **full-target blanking from truth (repainting
the true target's pixels, survey option a′) is NOT included**: it would put the ground-truth target mask into F's
static scan, which the observation contract forbids, and is possible only as an ORACLE diagnostic with the user's
explicit exception. The 0.10 m filter uses the vehicle's true attitude and height, as in S4 (the stand-in for an
onboard estimate; not target information). The target's geometric pixel mask exists only in the component smoke, as a
measurement. Review r1 checked the observation contract: PASS.

## Execution envelope

- **Process:** CPU process as S4 (cores 6, 7, 12, 13; `nice -n 10`; one thread; CUDA hidden).
- **Refusals before anything is created**, in addition to S3b's (approval, clean tree, sources at HEAD, F's SHA-256,
  inputs, environment, idle cores, single-use directory) and S4's (TCP 4590 in use, a CUDA job listed by `nvidia-smi`
  other than RustDesk, no DISPLAY):
  - the S4 plugin or its source not S4 attempt01's pinned bytes (replacing S4's plugin-age rule);
  - the world's model files not their pinned bytes;
  - the component smoke not PASS on C1–C9, not the pinned script, or not this runner's bytes;
  - less than 1.5 GB free on the unit's disk;
  - for attempt01, the four cell smokes as above.
- **GPU rendering:** OpenGL on the RTX 3070 through display :1; no other GPU job may run (checked at launch and before
  every episode). Real-time SITL acceptance-type runs keep an idle host (D-2026-11-05-1); the idle check covers the
  runner's cores only, as in S4.
- **Host and tree:** frozen tree; kernel-fault count before and after every step (the chain stops on a rise).
- **Chain:** `~/motar_tmp/s4f1_20261108/run_s4f1.sh` (worktree configurable, default MOTAR-camonly): clean tree and
  committed files → **the approval check** (`execution_approval.json` must name this runner's SCOPE and HEAD, checked
  with `SF.approval_problems` before anything renders on the GPU; the chain prints the expected SCOPE; review r2 N1) →
  the smoke's dry run → component smoke (`timeout -k 30 1200`; stop unless C1–C9 PASS) → the four cell smokes (900 s each;
  stop unless COMPLETE and VALID) → attempt01 (`timeout -k 30 18000`) → `--analyze`. It logs faults, free disk, GPU
  processes and load before and after every step, and builds nothing.
- **Single use:** `~/motar_tmp/s4f1_20261108/component_smoke`, `smoke01_<cell>` (4) and `attempt01`.
- **Approval:** `~/motar_tmp/s4f1_20261108/execution_approval.json`, naming `tools/sitl_s4f1.py`'s `SCOPE` and the commit.
- **Duration** (from S4 attempt01's measured 39.7 s + 0.100 s × steps per episode): attempt01 about **2.2–2.7 h** (2.7 h
  at S4's mean of 60.4 s; 2.2 h if the new cells capture in about 40 steps); **worst case 4.4 h** if every episode times
  out. The component smoke takes about 15 min and the four cell smokes about 5–8 min (inferred).
- **Disk:** S4 used 12.3 MB per episode, 98.6 % of it ULog; gzip −1 kept 0.33 of it. 164 episodes: about 2.0 GB raw,
  **about 0.7 GB with the ULog gzip** (inferred from S4's sizes).

## Design review r1 and repairs

Review: `~/motar_tmp/s4f1_20261108/review_design_r1/REVIEW.md` (Claude Opus 5.5, fresh context, 2026-11-08; probes in
`probes/`). Verdict **AMEND_FIRST**: no BLOCKER, observation contract PASS, factor manipulations exact. Repairs, all
before this file is committed and before the component smoke runs:

| Finding | Repair |
| --- | --- |
| MAJOR-1: S4's plugin-age rule refuses S4's own binary after a checkout; a rebuild would overwrite the `.so` S4 attempt01 binds | Both SHA-256s pinned; only the age rule is dropped when both match, otherwise refused; nothing rebuilt; the chain has no build step. Tested both ways, and on the host's real files |
| MAJOR-2: C4 failed by construction (36 bar-foot depth pixels below 0.10 m) | C4 counts removals outside the geometric bar-foot mask (dilated 1 px) and caps removed depth at that mask; C5 unchanged. The smoke's dry run checks offline that C4 passes without a floor, fails with one, and that C5 passes with one |
| MAJOR-3: the replication of S4's `ground` arm was claimed but not checked | Reading item 5 (stack core vs S4's, per-scene world SHA-256 vs S4's); integrity: one S4 world per scene, `lit_rendered` equal to it; the world's model files pinned, recorded and re-compared. Positive and negative tests |
| MINOR-1: power arithmetic (8 of 10 → 9 of 10) and "detectable" | Splits corrected; simulated power stated |
| MINOR-2: 14 uncorrected p-values read as "reached" | Two primary contrasts (main A, main G on capture), Holm; the rest descriptive |
| MINOR-3: attrition and ABORTED readings unspecified | VOID / INSUFFICIENT (< 30 complete scenes) / REPORTED; INVALID per cell by reason |
| MINOR-4: compound factors, over-attributed gap | "May not" lists what each factor changes; item 8 says "includes at least" |
| MINOR-5: GPU checked at launch only; idle covers the runner's cores | Per-episode GPU-job check (one wait, then GPU_BUSY / GPU_UNCHECKED) with load recorded; the core-only idle scope stated |
| MINOR-6: smoke authorization and binding | Approval named; script SHA-256 pinned in the runner and stated here; the runner requires the smoke to have used its current bytes |
| MINOR-7: no manipulation check in the real path | Reading item 3 |
| MINOR-8: rangefinder premise unstated and unchecked | Stated in "Cells"; cell-smoke gate \|EKF − truth\| ≤ 0.05 m; reported per cell |
| LOW-1 carry-over wording; LOW-2 RedGlow; LOW-3 C3 naming; LOW-4 rest-probe ODE iterations; LOW-5 ULog decompression in the analyzer and directory fsync; LOW-7 S3b ledger pin; LOW-8 DISK_LOW path tested, chain written | Fixed |
| LOW-6 interaction test approximates the risk-difference null | Stated in item 4 |

Round-2 check (Claude Opus 5.5, `~/motar_tmp/s4f1_20261108/review_design_r2/REVIEW.md`): **PASS_WITH_LIMITS**. Its open
items, closed before the commit:

| Item | Repair |
| --- | --- |
| N1 (MINOR): the component smoke ran before any on-disk approval check | The chain stops before the component smoke unless `execution_approval.json` names the runner's SCOPE and HEAD (`SF.approval_problems`); it prints the expected SCOPE |
| N2: Holm's effect on power understated | "Power" states the Holm figures (+25 pp: 0.83 → 0.75; +20 pp: 0.60 → 0.50) |
| N3: no test binds the SHA-256s stated here | `test_prereg_binds_the_runner_and_the_script` |
| N4: `replication` could read SAME_STACK_AND_WORLDS with a comparison missing | `models_compared` and `stack_compared` flags; INCOMPLETE when the model files, the stack or no world could be compared |
| N5: GPU_UNCHECKED wording | "a listed job or a failing `nvidia-smi` gets one 30 s wait and a recheck" |
| N6 (informational): `/usr/share/gazebo-11/models` holds another `ground_plane` | Not used: the owned session's model path is the PX4 tree's only, and the binding pins that tree's files |
