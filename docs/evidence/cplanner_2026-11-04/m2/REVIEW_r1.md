# Independent review: C-Planner M2 gate run (V1 status mapping, G-alpha certifiability on replay)

- Reviewer: Claude Opus 5.5 (model id `claude-opus-5-5`), fresh context, read-only. I wrote only inside
  `review/audit/`.
- Date: 2026-11-04, 07:20-08:00 KST. Run under review: `review/run_copy/` (19 files, bound by
  `SHA256_MANIFEST.json`), frozen code `frozen/` (commit `a5b7022`).
- Kernel fault record (`journalctl -k -b 0 | grep -cE "Machine check|segfault|general protection"`): **0 before
  the first probe (07:19:57), 0 between probes, 0 after the last probe (07:56:36)**. Boot id
  `407ae422-...` is the run's boot id. cpu4-5 offline throughout.

## Verdict: **FAIL** (this confirms the recorded FAIL; it does not add a new reason)

The request defines PASS as "V1 PASS and the G-alpha reading stand as computed". V1 did not pass, and the failure
is real, so the verdict is FAIL. The one blocking item is the V1 rule-1 violation the run recorded (B1). I
reproduced it with an oracle I wrote myself and traced its cause (below). I found **no other blocking finding**:

- V1's verdict (FAIL) is computed as preregistered.
- The G-alpha reading **FILTER_ARM_RETAINED** (rate 0.8161, FALSE_CERT 0) is computed as preregistered.
  My own loop reproduces it exactly (all four group counts, rate 0.81606, FALSE_CERT 0). The closest any certified
  segment came to a bar was 0.4556 m.
- I found no way that FALSE_CERT is undercounted, and no difference that could move the reading out of the
  >= 60 % band.

Q6 in one line: the violation comes from a mismatch between beam directions, not from the replay or the adapter.
The kernel's own float32 ray truly misses the bar. The certifier's float64 manifest ray, 9.7e-8 rad away, clips the
bar's bottom edge. The certifier's input is therefore more permissive than the real scene, but only by a sliver
0.13 um long inside the bar footprint. For every beam of this kind I give a bound: such a beam can erode the 0.45 m
radius by at most about 3e-5 m (about 7e-6 m within the reach of the G-alpha candidates). The bound fails only if
the same packet holds a second near-tangent ray on the same bar, or a HIT beam within about 2 deg of grazing. The
argument does not cover those coincidences (details under Q6).

## Findings by severity

### Blocking

**B1. V1 rule 1 is violated once: a NO_HIT beam whose manifest ray hits a bar (the recorded FAIL).**
Where: `run_copy/v1.json` `stats.v_nohit_but_oracle_hit = 1`, rule applied at `cplanner/tests/v1_hitbit.py:129-132`
(prereg `prereg_cplanner_m2_v1.md:53-55`).
- The beam: cell `A/F_SHADOW/E2_obstacle_aware__205bars__seed4301`, frame record 10752 = env 0, episode 1,
  frame 28. Beam 269 = row 3 (-10 deg), column 53 (sensor azimuth -85 deg). Kernel raw value 1000, scan value 1.0,
  adapter status NO_HIT at 12 m. The oracle hits bar 59 at 6.0991207 m.
- Reproduction: `audit/probe_v1_violation.py` and `audit/probe_v1_full.py` (log `audit/probe_v1_full.log`, data
  `audit/v1_full.json`). My oracle is an axis-aligned slab test that shares no code with `v1_hitbit.oracle`. It
  gives the same single violation and the same 3 band cases on the full V1 sample (2,400 frames, 691,200 beams),
  and it agrees with `v1_hitbit.oracle` to 7.1e-15 m on every hit/miss decision.
- This finding can change nothing: the label is already FAIL. It is listed as blocking because it is what makes
  "V1 PASS" false.

### Important (not blocking; they inform the later decision)

**I1. Cause of B1: the certifier's manifest directions differ from the rays the sensor actually casts.**
- I extracted the kernel's exact float32 beam directions with a Warp kernel (`audit/dirkernel.py`). It repeats the
  two `wp.normalize` calls and the `wp.quat_rotate` of `warp_lidar_kernels.py:192-193`. An exact float64 cast
  along those directions gives **0 violations on all 691,200 beams** (`kernel_dir_viol = 0` in
  `audit/v1_full.json`). For beam 269 the kernel ray passes 3.26e-8 m *below* the bottom edge (z = 0) of bar 59's
  -x face, so it misses. The manifest ray (`registry.directions`, rotated by the header yaw) enters that face
  2.24e-8 m *above* the edge, at t = 6.0991207380 m. It leaves through the bottom face at t = 6.0991208671 m, a
  chord of 1.29e-7 m. The angle between the two rays is 9.7e-8 rad (the run reports a maximum of 1.41e-7 rad).
- The same violation appears with the production triangulation and with 95 parked boxes added to the mesh
  (`audit/probe_v1_mechanism.py`). So it is neither a replay artifact nor an adapter mapping error. The value-to-status
  mapping held on every beam (`adapter_status_mismatch = 0`).
- Consequence: an adapter that declares kernel values against the float64 manifest will produce such beams at about
  1 per 7e5 beams on this kind of scene. As preregistered, V1 has no band at geometric edges
  (`v1_hitbit.py:124`: bands only at 0.2 m and 12 m), so a rerun of the same design is expected to fail again at
  any comparable sample size. G-alpha replayed 12,636 frames x 288 = 3.6e6 beams, so it probably contained a few
  such beams. Its FALSE_CERT is measured against ground truth and is 0.

**I2. Q6 result: a bound for every beam of this kind, with stated exceptions** (conditions (iv), (v), (vi)). The
argument is given under Q6 below.

### Minor

**M1. The replay's box triangulation is not the production triangulation (undisclosed).**
`cplanner/motar/replay.py:61-62` hard-codes a face list. Production loads the URDF boxes through urdfpy
(`aerial_gym/assets/warp_asset.py:22-23`), which builds them with `trimesh.creation.box` (trimesh 4.11.5).
Production's faces are `[[1,3,0],[4,1,0],[0,3,2],[2,4,0],[1,7,3],[5,1,4],[5,7,1],[3,7,2],[6,4,2],[2,7,6],[6,5,4],[7,5,6]]`
(`audit/probe_trimesh_box.py`). The vertices and their order are the same. The diagonal differs on the +x, -y and
-z faces. The preregistration (`prereg_cplanner_m2_v1.md:15,32`) says "trimesh.creation.box, 12 triangles". I
re-cast the whole V1 sample with production's faces: **0 status differences on 691,200 beams**, HIT ranges differ
by at most 2.9e-6 m, and the same single violation appears. The effect is negligible, but the claim in the
fidelity notes is not literally true.

**M2. G-alpha's terminal zone uses the 2-D target distance** (`galpha_replay.py:122`). The preregistration says
"target centre more than 0.9 m away" (`prereg:66`) and does not say 2-D or 3-D. The 2-D distance is at most the
3-D one, so steps with 2-D distance <= 0.9 m < 3-D distance are excluded. On this sample the effect is nil: in my recomputation a 3-D criterion gives the same 6,089 decision steps and the
same rate.

**M3. Input binding gaps** (`run_m2.py:47-59`). The driver does not hash `frozen/docs/` (the preregistration and
charter it applies), the site-packages it depends on (warp 1.0.0, torch, numpy), or the Warp kernel cache
(`~/.cache/warp/1.0.0`, reused by the run according to the logs). I closed the first gap myself: all 100 files of
`frozen/` are byte-identical to `git show a5b7022:<path>`. The kernel cache is keyed by a content hash, so a stale
binary is unlikely. This is a provenance gap, not an error.

**M4. Common-mode inputs between the oracle and the replay.** The V1 oracle is independent of Warp, the mesh and
the adapter. It shares the trace reader, the layout arrays and the yaw function `replay.sensor_yaw`
(`v1_hitbit.py:106-110`), so an error in decoding the pose or the yaw would affect the kernel and the oracle alike.
V1 cannot detect that. I checked `sensor_yaw` against production by reading it (see Q1).

**M5. `v1.json` keeps band cases as a count only.** I listed them: all 3 are oracle hits beyond 12 m (12.000002,
12.000063 and 12.000089 m). In each case the 12 m free ray stops short of the surface, so none is permissive.

### Limitations (disclosed or inherent; recorded, not findings against the run)

- **L1. Differences between the GPU and CPU in float32.** In production the ray vectors are normalised by
  `torch.norm` on CUDA, the yaw is computed with CUDA `atan2/sin/cos`, and the kernel is compiled by nvrtc
  (contraction to FMA possible). All of these can differ by a few ulp from the CPU replay, which is the same order as
  the 1e-7 rad mismatch in I1. Whether the delivered GPU scan for beam 269 was NO_HIT cannot be decided from the
  recorded data. The preregistration discloses this (`prereg:81-84`).
- **L2. The pooled reading hides the approach phase.** Steps 0-2 m from the target (lower bound 0.9 m) are
  certifiable at 275/654 = 42 % (`galpha.json` `by_distance_bin`), below the 60 % threshold. The reading is pooled
  as preregistered. The per-arm-density rates are all >= 0.758.
- **L3. Timing under 12 concurrent workers:** certify (16 segments) took 597 ms at p50 and 1,379 ms at p95. The
  G-beta budget is 50 ms. Not gated here; G-beta is M3.
- **L4. Probe process count.** My two pool probes used 4 workers plus one idle coordinator process. Another
  session's probe (`cplanner_m3_20261104/review/audit`) was running on the host at the same time. This affects
  only wall time; the host fault count stayed 0.

## What I verified and how

### Q1. Replay fidelity

I read the production path in the worktree. `git diff 61a3c59 a5b7022` is empty for `aerial_gym/sensors`,
`env_manager`, `config`, `assets`, `utils/math.py`, `navrl_task.py`, `kroc_eval_adapter.py` and
`motar_eval/occlusion_trace.py`, so the B1-R source commit and the frozen commit agree. I compared it with the
replay.

| Item | Production (B1-R) | Replay | Difference / can it make the input more permissive? |
| --- | --- | --- | --- |
| Scene contents | Warp mesh per env from the asset list. `scene_contract = static_scene_mesh` (only `bars`, no physical target, `use_warp`). No ground plane (`navrl_bars_env.py:60`). Slots >= `num_obstacles_in_env` parked at -1000 (`asset_manager.py:121`). `num_obstacles_in_env = n_bars_active + _bar_offset` (`navrl_task.py:2169`); `_bar_offset = 0` (bounded target, 0 distractors) | Active bars of the recorded layout only | Parked boxes are >1 km away. BVH composition differs; adding 95 parked boxes changed nothing on the probe frame. **No** |
| Layout | `obstacle_position` + `asset_collision_half_extents` read after `asset_manager.reset_idx` and `warp_env.reset_idx` (`env_manager.py:325-330`), slots `[0, 205]` | Same arrays | Recorded halves equal the float32 `bars_h3` catalogue for every box of all 32 cells checked. Centre z = half z = 1.5. Identity quaternions. URDF visual = collision for all 40 files, identical in both checkouts (`audit/probe_catalog.py`). **No** |
| Mesh | urdfpy, then `trimesh.creation.box` vertices, f32(c) + f32(+-h) (identity `quat_apply` is exact) | Same vertices; own face list | M1 (triangulation). Measured 0 status changes. **No (measured)** |
| Sensor pose | `tf_apply(attach, robot_position, 0)` = `robot_position`. `attach = quat_from_euler_xyz(0,0,get_euler_xyz(q).yaw)`. `quat_mul` with the identity local/data-frame quaternions is exact in float32 (checked by hand: `utils/math.py:243-259`) | `p_pos`; `sensor_yaw` repeats the same float32 expressions in the same order (`replay.py:119-130`) | GPU vs CPU ulp only (L1). **At the 1e-7 rad level, either direction** |
| Timing of state | Step: `_advance_target`, physics, `_sync_target_to_sensor`, `post_reward_calculation_step` (reset, then render), task `reset_idx`, sync, render again, observation, `after_perception` stages `p_pos/p_quat/t_pos` (`navrl_task.py:5023-5175`, `kroc_eval_adapter.py:1227-1245`) | Frame k rendered from recorded k state | Consistent. **No** |
| Target | Analytic sphere r = 0.20 (`navrl_lidar_config.py:84`; `target_use_oriented_box` False because `NAVRL_TARGET_DYNAMICS=bounded`), centre = `target_position` | r = 0.20, `t_pos` | Semantic id 15 vs 50, and mesh "velocities" zero vs segmentation ids: label only, no effect on range. **No** |
| Rays | 72x4, HFOV [-175, 180], VFOV [-10, 20], torch float32 on CUDA then `torch.norm` (`warp_lidar.py:68-92`). Env snapshot `NAVRL_LIDAR_HBEAMS=72`, `VBEAMS=4`, `RANGE=12`, `VISION=1` | Same on the CPU | ulp-level (L1). **Same order as I1** |
| Range limits and normalisation | `>12 -> 12`, `<0.2 -> -12`, `/12`, noise off (`warp_sensor.py:220-265`; config `navrl_lidar_config.py:64-72,102-108`) | Same order, float32 | **No** |
| Manifest vs kernel direction | n/a | The certifier uses float64 manifest rays | **I1: yes, a sliver of <= ~1e-5 m (Q6)** |

### Q2. V1

- The mapping (`adapter.py:167-184`) is a pure function of the value. On all 691,200 beams the status equals the
  preregistered mapping (`adapter_status_mismatch 0`, which I recomputed). HIT error is at most 2.478e-4 m, below the
  5e-4 m tolerance. No declared HIT range exceeds the manifest distance. Positive controls: 291,027 HIT, 400,119
  NO_HIT, 4,936 target hits, 54 NEAR. All of these reproduce exactly in my re-cast.
- Independence: the oracle (`v1_hitbit.py:42-71`) shares no code with Warp, `replay.box_mesh` or the certifier, and
  my second oracle agrees with it. It shares inputs (M4). Its reference direction is the right one: it is what the
  certifier assumes, and that is why it caught I1.
- Tolerances: TOL_RANGE = 5e-4 m is half the 1 mm adapter margin, so a declared range stays at or below the true
  range whenever the error is <= 0.5 mm. The value is justified, and it is not vacuous: the measured maximum is
  2.5e-4 m, 5x the dev maximum. The 0.1 mm band applies only at 0.2 and 12 m. Its 3 cases are all beyond 12 m and
  benign (M5). Tuning on dev data is disclosed (`prereg:95-97`).
- V1 PASS requires `zero_violations`. The single violation makes it false, so FAIL is computed as preregistered
  (`v1_hitbit.py:199-207`). The duplicate is identical apart from `wall_s`.

### Q3. G-alpha

- I read the code against the preregistration. Every frame is ingested in order by one core. A decision step is
  `frame % 2 == 0`, an accepted packet, and 2-D target distance > 0.9 m (M2). There are 16 candidates (headings 0,
  +-20, +-40, +-70 and 180 deg; lengths 1 and 2 m) from the vehicle. A candidate is progressing if it ends >= 0.1 m
  closer. A step is certifiable if any progressing candidate is certified. FALSE_CERT counts every certified
  segment, progressing or not, whose exact 2-D distance to any bar of the full recorded layout is <= 0.45 m
  (`galpha_replay.py:110-152`). The thresholds and the order of the labels match the charter (§3) and the
  preregistration. Merging across workers is correct, and a worker exception gives FAIL_INCOMPLETE. All 12,636
  frames were accepted. The 3 incomplete decision steps count as not certifiable.
- `seg_rect_dist` agrees with my own segment-rectangle distance (closed-form segment-segment distance) on 200,000
  random cases, maximum difference 2.7e-15 m (`audit/probe_segdist.py`).
- I recomputed the gate independently (`audit/probe_galpha_indep.py`, output `audit/galpha_indep.json`). I wrote my
  own loop, candidate construction and distance function, reused the frozen replay, adapter and core, and
  reproduced the preregistered sampling rule. The result is identical to the run: F_SHADOW:160 841/744 (decision/certifiable), F_SHADOW:205 843/732,
  B-PP:160 2014/1526, B-PP:205 2391/1967. Segments and certified counts per group also match (13,456/8,943,
  13,488/7,630, 32,224/15,694, 38,256/17,732). Rate 4969/6089 = 0.81606, FALSE_CERT 0, all 12,636 frames
  accepted. Of the 49,999 certified segments, the closest to any bar (my distance, all bars) is 0.4556 m.
  Wall time 1,018 s with 4 workers.
- Inflation channels: (a) the intercept proxy is the true target position, which is optimistic for a planner; this
  is disclosed. (b) Any certified progressing candidate counts, including +-70 deg ones that gain only about 0.3 m;
  this is preregistered. (c) Terminal exclusion is preregistered, and the near-target bin is 42 % (L2). (d) Sampling
  is random with a fixed seed and pooled by steps; B-PP contributes 72 % of the steps, and every group is >= 0.758.
  None of these goes beyond the preregistered definitions. The margin to the 60 % boundary is 0.216.

### Q4. Driver and provenance

- Manifest: I recomputed all 19 SHA-256 values in `run_copy/` and they match. All 164 inputs bound in
  `inputs_before.json` still hash to their recorded values now (frozen tree, kernel, trace reader, 30 cells x 4
  files, interpreter).
- Preflight: all 30 cells have `NAVRL_BAR_POOL = bars_h3`, no missing files, and the kernel and reader exist
  (`preflight.json`).
- Host: the fault count is 0 before and after each of the 4 commands, the boot id is unchanged, and `host_valid` is
  true (`records.json`, `host_start.json`, `m2_summary.host_end`).
- Determinism: G-alpha main and duplicate are identical after removing `wall_s`/`timing_ms` (checked by me). For V1
  only `wall_s` differs.
- Fail-closed: `judge` (`run_m2.py:87-104`) and `complete` (`:165-167`) need both V1 runs to PASS. V1 FAIL leads to
  summary `FAIL` and driver exit 1. Both are consistent with the files.
- Import provenance: the editable install maps only `aerial_gym` to the main checkout. The replay loads the kernel by
  file path, and `motar_eval` resolves to `frozen/motar_eval` through `sys.path[0]`, so no main-checkout code is
  imported. The frozen copy is byte-identical to `a5b7022` (100 files).
- The outputs support `m2_summary.json`: V1 FAIL, G-alpha reading FILTER_ARM_RETAINED, rate 0.81606, false_cert 0,
  determinism true/true, inputs unchanged.

### Q5. What a gate reading would hide

L2 (approach phase at 42 %), L3 (timing), I1 (a rate of about 1.4e-6 per beam of evidence that is more permissive
than the scene, invisible to a FALSE_CERT count on 6,089 decision steps), and M2. Also: the dominant refusal is
`MAP_VETO_OBSERVED_HIT` (46,209 of 97,424 segments), so certifiability is limited mostly by stale occupancy in the
evidence map rather than by the coverage proof.

## Q6. The V1 violation: characterisation and argument

**What the beam is.** The vehicle is at (6.78925, 10.21423, 1.05910), with header yaw 2.50488 rad. Bar 59 has centre
(10.16593, 15.43914), half-sizes (0.24, 0.20145) and spans z 0-3. The -10 deg beam 269 reaches bar 59's -x face
(x = 9.92593) at horizontal range about 6.0 m, exactly where it also reaches z = 0. This is a graze of the bar's
**bottom edge**. Neighbouring beam 197 (0 deg row, same column) hits the same face at 6.0065 m (HIT, correct).
Beams 268 and 270 are NO_HIT with no oracle hit.

**What the certifier receives.** In `MOTAR-72x4-RUNTIME` the free ray of a NO_HIT beam is
`[0.2, min(12, hi)]` (`core.py:150-160`). `hi` is where the manifest ray leaves the qualified slab, `(0 - z)/d_z`
for a downward beam (`core.py:206-218`). Here `hi` = 6.0991208671 m. That is exactly where the manifest ray leaves
bar 59 through z = 0, because the bar's bottom is the slab's bottom. The 2-D free segment therefore enters bar 59's
footprint by 1.27e-7 m along the ray, or 6.6e-8 m normal to the -x face. In the real scene that sliver is occupied
(the bar fills it for z in [0, 2.2e-8]). **The certifier's input is more permissive than the real scene.** In this
frame it has no consequence for G-alpha: the bar is about 6 m away, beyond the reach of any 1-2 m candidate plus
0.45 m.

**Why it is not covered by the certifier's own margins.** NO_HIT free rays end at `max_range - range_error_bound`
= 12 m or at the slab boundary, with no shortening (`core.py:151`). Coverage needs a strict crossing with a 1e-9 m
margin (`coverage_b.py:28-30,62-63`). The only margin against direction error is the adapter's 1 mm HIT shortening
(`adapter.py:161,178`), and it protects HIT beams only.

**The kind, and a bound that holds for every beam of it.** Define the kind as any beam whose kernel ray is free
(or hits farther) while its manifest ray touches a bar earlier: a NO_HIT like this one, or the HIT analogue, which
V1 would show as a range mismatch (none occurred). Let alpha be the angle between the two rays: measured at most
1.41e-7 rad, and of the order of a few float32 ulp a priori. Let delta_k be the kernel's own intersection error
along its ray: 0 violations along the kernel's rays on 691,200 beams; float32 error of about 1e-6 m. At range
t <= 12 m the two rays are within eps = alpha t + delta_k (about 3e-6 m at 12 m, about 7e-7 m at 3 m) in 3-D.

1. *Intrusion bound.* Wherever the kernel ray is inside the slab it is outside every bar box, so its 2-D
   projection is outside every footprint. The manifest point at the same t is therefore within eps of a point
   outside the footprint. The two rays can be on opposite sides of z = 0 or z = 3 only during a parameter interval
   of length at most eps/|d_z|, which projects to at most eps/tan 10 deg in the plane. Hence every point of the
   certifier's free segment lies within **w <= eps (1 + 1/tan 10 deg) ~= 6.7 eps (<= ~2e-5 m at 12 m; <= ~5e-6 m
   within the 3 m that G-alpha candidates can reach)** of the boundary of any footprint it enters. For the 0 deg row,
   w <= eps; for the 20 deg row, w <= 3.7 eps. Observed instance: 6.6e-8 m.
2. *Effect on certificates (translated-bar argument).* Let B be a bar that such free rays enter. Each entry lies
   in the w-wide inner band along one face of B, or near one corner of B along two adjacent faces. Move B by
   u = w' (n1 [+ n2]), where w' > w and n1, n2 are the inward normals of the face or faces the intrusions follow.
   This keeps the catalogue shape, the axis alignment and the full height. Call the result Sigma'. In Sigma' the
   intruding rays are truly free: they lie within w of faces that moved by w' > w, and they are at least 2h - w
   from the opposite faces. Condition (v): every HIT declaration on B stays at or below its Sigma' range. This holds
   automatically when u points away from the sensor, as it does for the observed beam (u = +x; the sensor has
   x = 6.79 < 9.93). Otherwise it holds as long as |u| / cos(incidence) stays below the 0.75 mm or more of adapter
   margin that remains, which fails only for a HIT beam within about 2 deg of grazing a face that moves towards the
   sensor. A packet that is conservative for a §4 scene gives only sound
   certificates: shorter free rays and extra surfaces can only remove certificates (`core.py:12-19`; the coverage
   rule `coverage_b.py:42-73`). By that soundness, on which R3' already ruled, a certified segment keeps >= 0.45 m
   from B + u, hence **>= 0.45 - sqrt2 w (>= 0.45 - ~3e-5 m) from B** — on two conditions:
   - (iv) no other free ray of the same packet passes through (B + u) \ B, the <= sqrt2 w-wide strip just outside
     B's opposite face or faces, which from the sensor can only be reached by a ray passing B's silhouette corners
     at a distance of order w';
   - (vi) intrusions into the same bar do not follow opposite faces;

   and on condition (v) above.

   I checked that the coverage rule cannot certify while an in-reach catalogue hypothesis is crossed by no free
   ray. A covered cell's shrunk box lies inside every hypothesis footprint of the cell, the pruning test uses the
   expanded cell, and depth and resolution are bounded. So "B + u in reach and uncrossed" implies refusal.
3. *What this does and does not establish.* For every beam of this kind, any FALSE_CERT it causes is either a
   radius erosion of at most about 3e-5 m (exact boundary grazing at the 10 um scale, which charter §5 places out
   of scope), or it needs a second, independent near-tangency on the same bar in the same packet ((iv) or (vi)), or
   a HIT beam within about 2 deg of grazing ((v)). I could not exclude these cases by argument. The first is a
   coincidence of two ~1e-5 m tangencies, but the argument gives no bound in any of them. The argument also relies on the kernel being exact along its own rays up to delta_k,
   which V1 measured on a sample but did not prove.
4. *Relation to charter §4/§5.* §4 assumes "pose is exact; declared beam statuses are true". This beam is a
   violation of the second assumption along the manifest at the 1e-7 rad level, equivalent to an orientation error
   of that size. §5 places adapter false declarations outside the certifier's P0 scope, but V1 exists to qualify
   exactly these declarations, and its rule 1 has no band at geometric edges. So the FAIL is in scope for V1. For
   comparison only: the preregistration knowingly accepts a free-ray overrun of up to 1e-4 m at the 12 m end
   (band, `prereg:53-55,88-89`), about 800 times this instance and about 5 times the general bound w.

**Computed as preregistered?** V1: yes, FAIL by rule 1 (`zero_violations` false), every other check true. G-alpha:
yes, FILTER_ARM_RETAINED with rate 0.81606 and FALSE_CERT 0. My independent recomputation agrees on every count.
The FAIL label stands as recorded.

## Audit files

`audit/common.py` (read-only imports, Warp cache inside `audit/warpcache`), `probe_trimesh_box.py`,
`probe_v1_violation.py` (+ `v1_violation.json`), `probe_v1_mechanism.py`, `dirkernel.py`, `probe_v1_kdir.py`,
`probe_v1_full.py` (+ `.log`, `v1_full.json`), `probe_catalog.py`, `probe_galpha_indep.py` (+ `.log`,
`galpha_indep.json`), `probe_segdist.py` (+ `.log`).
