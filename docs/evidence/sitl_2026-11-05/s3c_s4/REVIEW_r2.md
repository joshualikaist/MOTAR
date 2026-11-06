# Review r2: S3c v1 and S4 v1

Reviewer: Claude Opus 5.5 (claude-opus-5-5), fresh context, read-only. Tree: `camonly/dev` e4364c1 plus the uncommitted
units; source SHA-256s in `probes/sources_r2.sha256`, unchanged at the end. Nothing was started; no unit directory or
approval file was created.

## Runs
- `tests/test_sitl_s3c_s4.py`: 16/16 OK in 120.7 s (`probes/tests_s3c_s4_r2.out`).
- `--check`, cores 6,7,12,13: both rc 0, with no input or environment problems.
  - S3c schedule: iris 40, delay1 40.
  - S4 schedule: ground 40, raw 20; plugin present; DISPLAY :1.

## S4

**B1 (self-occlusion): FIXED.**
- The regenerated SDF has 0 `<visual>` and regenerates byte for byte.
- Airframe 10092 is installed and equals 10090 apart from its `@name` line.
- **Collisions are not rendered** (gazebo-classic `gazebo11` source):
  - `Scene::ProcessVisualMsg` only queues collision visuals as pending children.
  - `Visual::ShowCollision` creates them only when `showCollisions` is true (default false), and gives them `GZ_VISIBILITY_GUI`.
  - `Camera::SetRenderTarget` and GpuLaser's first- and second-pass viewports mask with `ALL & ~(GUI|SELECTABLE)`.
  - `GpuRaySensor` sets the near clip to `RangeMin` (0.2 m).
  - This matters: if collisions were rendered, the front rotor discs would put 276 sample points inside the camera frustum and return LiDAR ranges up to 0.31 m (`probes/remaining_visuals_probe.json`).
- **The visuals that remain** (from `model://lidar` and `model://gps`):
  - they lie 0.005–0.073 m from the LiDAR origin, entirely inside the 0.2 m near clip;
  - they are 0.04–0.094 m behind the camera, so 0 frustum points;
  - with the near clip ignored, 0/288 beams and 0/2058 camera rays hit them (`probes/beam_cast_probe.json`).
- **Hover guard (0.35 m) is not vacuous:**
  - A valid start over ground only keeps LiDAR and depth minima of at least 1.70 and 1.32 m, even at 30° nose-down.
  - Every r1 self-return (0.05–0.31 m) would trip it.
  - Tests: 12 m passes; 0.22 m gives HARNESS_INVALID.

**M1 (recall): FIXED.**
- `camera_visible` is perception's camera-only flag (`navrl_perception.py:2395`). LiDAR-only detections are counted separately.
- On a synthetic ledger the analyzer gives recall 136/340 = 0.4, as constructed.

**r1 minors, all fixed:**
- the 10 m clamp;
- lateness = max(state lateness, frame age) ≤ 0.15 s;
- −inf → 0 (tested);
- RGB copied under a lock in the rendering-thread callback;
- refusals for port 4590, nvidia-smi compute apps, DISPLAY, and a plugin older than its source;
- target z + 0.09 m (tested);
- `truth_age_max_s` recorded.

**s1 MINOR: the frame age is still underestimated.**
- RGB and depth are stamped with `world_->SimTime()` when the callback runs. LiDAR uses `LastMeasurementTime`, the render scene's SimTime.
- gzserver is launched as `[gzserver, world]` with no `--lockstep`, so the render scene can lag physics. That lag is missing from the frame age, which weakens the 0.15 s bound.
- The prereg row says all three are "stamped with simulation time at the copy". That is wrong for LiDAR, and it does not say the RGB and depth figures are lower bounds.
- Fix: stamp each callback with `Camera()->GetScene()->SimTime()`, or disclose the figure as a lower bound.

**LOW:**
- `sim_now` is logged, but nothing computes the clock-domain offset (sim_now − PX4 time).
- Steps whose RGB frame predates the target update (`rgb_t < target_t`) are not summarised.
- The see-through below 0.2 m (near clip) is still undisclosed.
- No test feeds near depth into the hover guard.
- `--check` does not run `s4_extra_problems`.

## S3c
- **r1 m6/m7: FIXED.**
  - The wording says "estimates…informs…no decision rule" and lists what iris confounds; smoke01 is added.
- **Iris footprint:** column 20 is confirmed through the real `SF.run_episode`. It is the only column equal to an injected clearance sequence, and F's 4-D action gives the 27-column layout (`probes/column_probe.json`).

## Both

**M2 (analyzers): FIXED.** On synthetic ledgers (`probes/analyzer_probe.json`): integrity [] when clean, a swapped
row pair flagged; per-arm timing, BOUND S3b reference (SHA-256, reading label), matched pairs; S4 medians and frame age.

**Smoke gate:**
- **Code is shared:** `launch_unit` runs with `sched[:1]`, the same `run_one_fn`, approval and refusals.
- **Single use:** smoke01 is single use, and attempt01 refuses unless smoke01 is COMPLETE with one VALID episode.
- **SCOPE:** both SCOPE strings name smoke01 and attempt01.

**s2 MINOR: the smoke gate does not check which code the smoke ran.**
- `smoke_problems` reads only `final.json` and the ledger. It never compares smoke01's `git.head` or source SHAs with the attempt's.
- A smoke from older code would therefore unlock attempt01.

**s3 MINOR: a smoke can be read as a result.**
- `--analyze …/smoke01` is accepted. Its `reading.json` has no smoke marker and integrity [], so it looks like a COMPLETE one-episode attempt.
- Fix: refuse, or label the reading, when `development_smoke` is true.

**Wording (r1 m5): fixed:** "only the ground is separated", flight readiness under does-not-follow, recall tilt disclosed.

## Chain launcher (`run_chain.sh`, already updated: smoke, `smoke_ok`, rebuild)

It must:
1. **(MINOR) Stop on a host fault.** Check each unit's `final.json` and stop if `kernel_faults_after ≠ before` or the reason cites a fault rise.
   - Today the next unit runs anyway: its launch reads a fresh fault baseline.
   - That is against "host faults void a run".
2. **(LOW) Build the plugin in a clean directory (or `make -B`) and drop the `touch`.**
   - The `touch` makes the older-than-source refusal pass unconditionally.
   - An incremental `make` can be a no-op, so "rebuilt from the committed .cc" is not guaranteed.
   - Keep not rebuilding between the S4 smoke and attempt01. The per-arm binding includes the `.so`.
3. **(LOW) Consider longer timeouts.** 7200 s is about 1.7× the expected 70/60 min, and a timeout ABORTs and consumes attempt01.

Also: check that X :1 is reachable before S4. Correct as it stands: the chain uses `smoke_ok`, not `--smoke`'s exit
code (0 even on a failed smoke), and a refusal stops it.

## Verdicts
- **S3c v1: PASS_WITH_LIMITS.** Every r1 finding is fixed. Fix s2/s3 and chain item 1 before approval.
- **S4 v1: PASS_WITH_LIMITS.**
  - B1 and M1 are fixed, with source and geometry evidence.
  - The only empirical check of B1 is the smoke's hover guard.
  - Fix s1 (one line per callback, plus a plugin rebuild), s2/s3 and chain item 1 before approval.
