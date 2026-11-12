# CAM-T1 v1 results review r1

- **Reviewer:** Claude Code subagent, Opus 5.5 (`claude-opus-5-5`), fresh context. Read-only on the repository; wrote
  only to `~/motar_tmp/camt1_20261112/review_results_r1/`. No GPU work and no simulator. Usage state: USAGE_UNKNOWN.
- **Date:** 2026-11-12. **Unit:** CAM-T1 v1, worktree `MOTAR-camonly-t1`, branch `camonly/t1`, run commit `b3b9f0b`
  (HEAD, tree clean). Prereg: `docs/plans/prereg_camonly_t1_2026-11-12.md`.
- **Scope:** process validity, an independent recompute of Stage A and Stage B from raw data, interpretation limits,
  and anything suspicious.

## Verdict: ACCEPT_WITH_LIMITS

The run is valid as preregistered, and every stage reading follows from the raw data under the prereg's rules:
IDENTITY_PASS, NONINTERFERENCE_PASS, CONTROL_PASS, envelope COMPLETE, POSITIVE_CONTROL_PASS, Stage A DESCRIBED and
Stage B REPORTED. No VOID condition occurred.

My own code imports no repository module.
- **Stage B:** it reproduces every rate, contrast, interval and label exactly (max |difference| 1.3e-10). An
  independent RNG gives the same labels.
- **Stage A:** it reproduces the primary profile exactly on all 6,643 quota crash episodes, including every share,
  every per-density count and the cell-cluster intervals to about 0.001. The `eye_nearest_point` profile differs by 2
  of 6,589 episodes, which is immaterial. The DEFINITION_SENSITIVE guidance reading holds.

**Limits.** The results may be recorded only with the wording limits below. They concern the record's wording, not the
measurement:
- IN_VIEW is not "visible" (R1);
- the cap arms read SEEDS_DISAGREE, which is neither "worse" nor "no effect" (R2);
- the caps change only the governor's open-direction speed (R3);
- the cap20 exposure context is descriptive only (R4);
- d1 is one 100 ms step of command delay over a zero-latency baseline (R5).

## Findings

No P0 or P1 finding: nothing invalidates the run, and no number in the analysis outputs must be corrected.

| ID | Severity | Finding | Evidence |
| --- | --- | --- | --- |
| R1 | P2 | **IN_VIEW means "inside the geometric FOV", not "visible".** The record must carry the guidance reading DEFINITION_SENSITIVE and must not apply the IN_VIEW → "depth fidelity or speed" mapping. The primary IN_VIEW of 67.7 % comes mainly from wide bar footprints seen from about 0.17 m. | At the last frame L, the eye was below the depth model's MinZ (0.4 m) in **100 %** of the 6,589 VALID crashes (median eye distance 0.171 m, p90 0.250 m). Contacted bars have half-extents of 0.21–0.39 m (p10–p90; footprints about 0.4–0.8 m) and are 3 m tall. The median \|nearest-point bearing\| is 51° against the 33° half-FOV. Cross-tab of footprint × nearest-point: of 4,459 footprint-IN_VIEW crashes, 1,061 are BLIND and 1,368 LEFT_FOV under the nearest-point test; of 1,417 footprint-LEFT_FOV crashes, 1,023 are BLIND. The handoff's "High IN_VIEW → depth fidelity or speed" is the tempting over-read. |
| R2 | P2 | **The cap readings are SEEDS_DISAGREE: not "a lower cap makes it worse" and not "speed is not a lever".** The pooled cap20 label (WORSE) is descriptive and not read. cap20 s1's label NO_DETECTED_CHANGE sits on intervals wholly above 0; it fails the 2 pp floor (mean +1.74 pp), so the record should cite the intervals beside the label. Neither of the guidance's speed branches (REDUCTION, NEGLIGIBLE) is reached, so no speed direction is named. | gate_d as preregistered: cap20 s1 bootstrap [0.93, 2.70], t [0.27, 3.21], mean 1.74 < 2 → NO_DETECTED_CHANGE; s2 [1.63, 3.26], [0.95, 3.87], mean 2.41 → WORSE. cap25 s1: t upper 2.09 > 2, so not NEGLIGIBLE → NO_DETECTED_CHANGE; s2 both inside ±2 → NEGLIGIBLE. Reproduced under two bootstraps (table below). |
| R3 | P2 | **The cap arms change only riskcap's FREE end.** In every arm, native included, commands with corridor clearance ≤ 3 m or outside the camera FOV are held at FIXED = 2.0 m/s. "Cap 2.0" therefore removes open-direction speeds above 2.0 m/s; it does not slow the vehicle near bars relative to native. The record should say what the cap changed. | `speed_governor.apply_speed_governor` (riskcap: cap = FIXED + release × (FREE − FIXED); direction never changed, vector scaled). Every receipt's snapshot has FIXED 2.0 and MAX_VELOCITY 2.5; FREE is 2.0 / 2.5 / 3.53553390593 as intended. Commanded maxima: cap20 2.0000002, cap25 2.5000005, native 2.948–2.961 m/s. Mean actual speed: native 2.08/2.14, cap25 2.01/2.05, cap20 1.78/1.79 m/s (s1/s2). |
| R4 | P2 | **cap20's crash increase comes with longer exposure (descriptive, not a mechanism test).** If the record discusses a mechanism, it should cite the preregistered median capture time (+1.10 s s1, +1.20 s s2) and mean actual speed. The reviewer's exposure normalisation may be added only as reviewer-exploratory: per flight-second the cap20 crash rate is lower than native, and per metre flown it is about unchanged; capture per second and per metre fall. That is "consistent with longer pursuit exposure", not "explained by". | Exploratory, not preregistered, own code: crashes per 100 s of quota flight, pooled cap20 − native **−0.36** (t [−0.62, −0.10]); capture per 100 s −3.87 (t [−4.08, −3.65]). Per 100 m flown: native 1.44/1.62, cap20 1.48/1.72; block differences change sign (s1 −0.06 … +0.28, s2 −0.06 … +0.42). Mean episode length: native 5.34/5.03 s, cap20 6.79/6.51 s. Median crash time: native 3.2–3.9 s, cap20 4.3–4.9 s. Timeouts 0, so there is no truncation. Quota = env episodes 0–15 (checked), so there is no completion-order selection. Caveat: competing risks, non-uniform hazard, and episode-mean speed × duration as a distance proxy. |
| R5 | P2 | **d1 is the one unambiguous gated result (WORSE, both seeds), with a narrow scope.** It is one 100 ms step of *command* delay, with the zero (hover) action on each episode's first step, added to a native baseline whose render delivery has zero latency. Untested: latencies between 0 and 100 ms, jitter, and observation-side delay models. | Hook `act()`: the previous step's action is returned, and zeros at episode start (read). The sidecar counts only the first-step zeros (`delayed_zero_env_steps` 2,606 in d1_smoke). The d1 ledgers differ from native (run). Trace metadata: `render_alignment: ZERO_LATENCY_RENDER_DELIVERY`. Exploratory: crashes per 100 s +0.70 pooled (t [0.53, 0.87]) and per 100 m higher in all 12 blocks (+0.16 to +0.71), so the increase is not an exposure artefact. |
| R6 | P3 | The `eye_nearest_point` profile does not apply the "eye inside the footprint counts as in view" rule. For eye-inside frames it uses G3's arithmetic (the bearing of a zero vector, which depends on yaw). | 2 V3s2 episodes with the eye inside the footprint. My rule moves them BLIND → IN_VIEW: nearest-point pooled BLIND 42.45 % (mine) vs 42.48 % (analysis). The reading is unchanged. |
| R7 | P3 | Approval provenance. The file names SCOPE (string-equal to `camt1_run.SCOPE`) and HEAD. It is the agent's transcription of the user's message, which this review cannot see. Its mtime, 16:20:30.77, falls in the same second as CHAIN START, after the CPU gate (16:20:15) and before the first cell started (16:20:31). | `execution_approval.json`, stat, chain.log, identity_01.log. The chain script reads the approval but does not write it. |
| R8 | P3 | Stage B pairing is by cell (block), as preregistered, not by episode. Only episode 0 of each env shares its scene and initial state across arms, so contrasts are distribution-level within a block. | Scene and initial-state hashes in the kroc rows: 128–135 of 2,048 quota keys match between native and each arm at 205/4501, including all 128 episode-0 keys. |
| R9 | P3 | Seed heterogeneity in Stage A. The contact side is LEFT in 52 % (s1) vs 71 % (s2), and nearest-point BLIND is 37 % vs 48 %. The record should not state a lineage-level side bias. | Per-seed shares in the table below (analysis and recompute agree). |
| R10 | P3 (positive) | Stage B's native references are reproduced on this commit for all of its 12 blocks, not only V3s1 205/4501. All 24 traced cells, which include the 115/205 blocks of both seeds, have both ledgers byte-identical to E3's. | SHA-256 of `episodes.jsonl` and `kroc2027_episodes.jsonl`, 24 × 2 files, 0 mismatches (run). |
| R11 | P3 | The trace metadata records the camera's `max_range_m` as 20 m; the analysis uses 10 m (`camera_obstacle_max_range`). | This does not bind: the eye was within 0.4 m of the contacted bar at L, and the 1 s window spans about 2 m. |
| R12 | P3 | The CPU gate is BASELINE_EQUIVALENT, not 0/0. Record it as "PUBLIC_CPU not 0/0 (baseline-equivalent): 15 pre-existing environment failures, identical to base `eefb832`". | `cpu_gate_b3b9f0b…json`: 3,978 run, 7 failures + 8 errors, 18 skipped, set equal to the baseline; log SHA 02fc7c13… matches. |
| R13 | P3 | EARLY is 11.3 %, so no placement note is triggered (< 20 %). 92 % of EARLY crashes are primary-IN_VIEW (690 of 746). | Analysis and recompute agree. |

## Process validity

| Check | Result | How |
| --- | --- | --- |
| IDENTITY_PASS | Holds: native_identity's two ledgers are byte-equal to E3 V3s1 205/4501; outcome and `result_equal` true | report, plus my SHA-256 (run) |
| NONINTERFERENCE_PASS | Holds: 24/24 cells, both ledgers byte-identical to E3 | my SHA-256 of all 48 files (run) |
| CONTROL_PASS | Holds: k1d0_identity is byte-equal to E3; d1_smoke and cap20_smoke differ (crash 566 and 550 vs 450) | my SHA-256 (run), report |
| Determinism | d1s1 = d1_smoke and cap20s1 = cap20_smoke at 205/4501, byte for byte | run |
| Envelope COMPLETE | 36/36 cells in plan order, all OK, no stop flags | report (read) |
| POSITIVE_CONTROL_PASS | All 5 metrics × 3 conditions equal the committed `c2trace_analysis.json` to < 1e-9; the negative control at 33° differs (56.0 / 80.0 / 80.2 %) | compared the two JSONs (run) |
| VOID rules | Fault count 0, boot id `407ae422…` and online CPUs `0-3,6-23` are identical at start and end of all 4 stages and still today. 64/64 receipts: `git_commit` b3b9f0b, 0 dirty files, status OK. Trace `source_dirty` false. Tree clean now; HEAD unchanged | reports (read); journalctl, `/proc`, `git status`, receipts (run) |
| Approval | SCOPE string-equal; commit = HEAD; cells 1 + 24 + 3 + 36 = 64 | run |
| References | E3 eval report COMPLETE at aaca40e; 96 reference files at the manifest's SHA-256 (manifest read from HEAD) | run |
| Script binding | `tools/camt1_analyse.py` in the worktree = HEAD; its SHA (4728d985…) is in all three analysis outputs; launcher SHA = pin c166f560… | run |
| Arm manipulations | Snapshots differ only in the arm key (trace = 1 for 24 cells; FREE 2.0 / 2.5 for 13 / 12 cells); hook sidecars only for d1 and k1d0, patch applied, calls > 0; caps never exceeded (+1e-3) | run |
| Stage A validity gates | Coverage 6,643/6,643; actions_match 100 %; frames contiguous 100 %; VALID 6,589 (non-VALID 42 s1, 12 s2; ≥ 90 % per seed); boxes axis-aligned; vehicle quaternion yaw-only (max \|x\|+\|y\| = 0) | run |
| CPU analyses after an idle GPU | "GPU idle after 0s" before g3-control; no compute process now | chain.log (read), nvidia-smi (run) |

## Recomputed numbers next to the analysis

### Stage B: quota rates, mean of 6 blocks (%)

| Arm | Crash (recompute / analysis) | Capture | Timeout |
| --- | --- | --- | --- |
| native s1 / s2 (E3) | 15.967 / 15.967 · 17.367 / 17.367 | 84.033 · 82.633 | 0 · 0 |
| cap25 s1 / s2 | 16.292 / 16.292 · 17.952 / 17.952 | 83.708 · 82.048 | 0 · 0 |
| cap20 s1 / s2 | 17.708 / 17.708 · 19.775 / 19.775 | 82.292 · 80.225 | 0 · 0 |
| d1 s1 / s2 | 20.915 / 20.915 · 21.037 / 21.037 | 79.077 · 78.963 | 0.008 · 0 |

### Stage B: crash contrast arm − native (pp), with labels

Bootstrap R is my re-implementation of the preregistered scheme (stdlib RNG 20,261,108, same call order); it equals the
analysis to 1e-10. Bootstrap I is my numpy bootstrap (seed 777). The t interval is stratified, df 4; I checked
t₀.₉₇₅ = 2.776445 by integrating the pdf (0.47500).

| Contrast | Mean | Bootstrap R (= analysis) | Bootstrap I | t (= analysis) | Label (analysis / R / I) |
| --- | --- | --- | --- | --- | --- |
| cap25 s1 | +0.326 | [−0.53, 1.53] | [−0.53, 1.46] | [−1.44, 2.09] | NO_DETECTED_CHANGE ×3 |
| cap25 s2 | +0.586 | [−0.20, 1.25] | [−0.20, 1.25] | [−0.65, 1.82] | NEGLIGIBLE ×3 |
| cap25 pooled | +0.456 | [0.04, 0.87] | [0.04, 0.89] | [−0.31, 1.22] | UNRESOLVED ×3 (descriptive) |
| **cap25 reading** | | | | | **SEEDS_DISAGREE ×3** |
| cap20 s1 | +1.742 | [0.93, 2.70] | [0.93, 2.69] | [0.27, 3.21] | NO_DETECTED_CHANGE ×3 |
| cap20 s2 | +2.409 | [1.63, 3.26] | [1.63, 3.26] | [0.95, 3.87] | WORSE ×3 |
| cap20 pooled | +2.075 | [1.45, 2.98] | [1.45, 2.88] | [0.74, 3.41] | WORSE ×3 (descriptive) |
| **cap20 reading** | | | | | **SEEDS_DISAGREE ×3** |
| d1 s1 | +4.948 | [4.30, 5.60] | [4.30, 5.60] | [3.79, 6.11] | WORSE ×3 |
| d1 s2 | +3.670 | [3.12, 4.17] | [3.12, 4.17] | [2.74, 4.60] | WORSE ×3 |
| d1 pooled | +4.309 | [3.93, 4.68] | [3.93, 4.68] | [3.64, 4.98] | WORSE ×3 (descriptive) |
| **d1 reading** | | | | | **WORSE ×3** |

- **Gate controls on CAM-K2-E** (recomputed from the raw ledgers):
  - K1s1 − K2s1 +3.54 pp, WORSE; K2s1 − K1s1, SMALL_REDUCTION;
  - K1s2 − K2s2 +4.93 pp, WORSE; K2s2 − K1s2, SMALL_REDUCTION;
  - null K2s1 − K2s2 +0.415 pp, NEGLIGIBLE.
  - All five as required; equal to the analysis.
- **Median capture time contrast, s1 / s2 / pooled** (equal to the analysis to 1e-11):
  - cap25 +0.28 / +0.23 / +0.26 s;
  - cap20 +1.10 / +1.20 / +1.15 s;
  - d1 +0.30 / +0.275 / +0.29 s.
- **Commanded speed maxima over all kroc rows:**
  - cap20 2.0000002 m/s; share > 2.0 m/s 0 %;
  - cap25 2.5000005 m/s; share > 2.5 m/s 0 %, share > 2.0 m/s 96–98 %;
  - native 2.948 / 2.961 m/s; share > 2.5 m/s 96–98 %, which agrees with the prereg's "96–98 %, max 2.95";
  - d1 2.948 / 2.960 m/s.
  - Requested (pre-governor) maxima are 3.43–3.47 m/s in every arm.

### Stage A: category shares over VALID quota crash episodes (%), cell-cluster 95 % interval

| Profile / group | n | BLIND (mine / analysis) | LEFT_FOV | IN_VIEW |
| --- | --- | --- | --- | --- |
| primary, pooled | 6,589 / 6,589 | 10.82 [9.4, 12.0] / 10.82 [9.4, 12.0] | 21.51 / 21.51 [20.4, 22.6] | **67.67** / 67.67 [65.7, 69.7] |
| primary, V3s1 | 3,236 | 8.84 / 8.84 | 19.41 / 19.41 | 71.76 / 71.76 |
| primary, V3s2 | 3,353 | 12.73 / 12.73 | 23.53 / 23.53 | 63.73 / 63.73 |
| eye_nearest_point, pooled | 6,589 | **42.45** [39.7, 45.1] / 42.48 [39.8, 45.1] | 26.74 / 26.74 [25.3, 28.3] | 30.81 / 30.78 [29.3, 32.4] |
| eye_nearest_point, V3s1 | 3,236 | 36.74 / 36.74 | 29.57 / 29.57 | 33.68 / 33.68 |
| eye_nearest_point, V3s2 | 3,353 | 47.96 / 48.02 | 24.01 / 24.01 | 28.03 / 27.97 |

- **Dominant category:**
  - primary: IN_VIEW CLEAR (lower end 65.7 > 22.6 and 12.0);
  - nearest point: BLIND CLEAR (lower end 39.7 > 32.4 and 28.3);
  - so the guidance reads **DEFINITION_SENSITIVE** (mine = analysis).
- **Per-density category counts:** identical for all 8 seed × density groups.
- **Other quantities (mine = analysis):**
  - EARLY 11.32 %;
  - below_minz 100 %;
  - median contact distance from the centre 0.223 m;
  - median \|nearest-point bearing\| 51.1°;
  - LEFT side 62 % (52 % s1, 71 % s2);
  - non-VALID crashes 42 (s1) and 12 (s2).
- **Exploratory, not preregistered** (bar in FOV and at least MinZ from the eye at some frame of the last 1 s):
  - footprint test 88.2 %; nearest-point test 54.4 %;
  - footprint test at any frame of the episode: 99.6 %.
  - So under both definitions most contacted bars entered the geometric FOV beyond MinZ before contact. Occlusion, the
    40-column depth sampling and the validity rule are not modelled, so this is not "seen by the policy".

### Exploratory exposure normalisation (reviewer only; not preregistered; do not record as a result)

| Arm (s1 / s2) | Mean episode, s | Crashes per 100 s | Captures per 100 s | Crashes per 100 m | Captures per 100 m |
| --- | --- | --- | --- | --- | --- |
| native | 5.34 / 5.03 | 3.05 / 3.50 | 15.7 / 16.4 | 1.44 / 1.62 | 7.38 / 7.53 |
| cap25 | 5.64 / 5.32 | 2.93 / 3.42 | 14.8 / 15.4 | 1.43 / 1.65 | 7.18 / 7.36 |
| cap20 | 6.79 / 6.51 | 2.69 / 3.15 | 12.1 / 12.3 | 1.48 / 1.72 | 6.62 / 6.71 |
| d1 | 5.52 / 5.21 | 3.86 / 4.09 | 14.3 / 15.1 | 1.90 / 1.98 | 6.98 / 7.24 |

## Interpretation limits

- **Causal wording.** Stage A is descriptive and "consistent with" only; nothing in it says why the policy crashes.
  Stage B is an intervention on the arm, so "the d1 arm had a higher crash rate than native in both seeds" is fair.
  - Not allowed: "latency causes the crashes on the vehicle".
  - Not allowed: "cap20 increases crashes because of longer exposure"; R4 is exploratory and untested.
- **"Speed cap makes it worse."** Not allowed as a reading. cap20 is SEEDS_DISAGREE: s1 is NO_DETECTED_CHANGE
  (+1.74 pp, below the floor), and s2 is WORSE.
  - Allowed: "no lower crash rate at either cap in either seed".
  - Allowed: "cap20's point estimates were higher in both seeds, materially so only in seed 2".
- **"Speed is not the bottleneck."** Not allowed either. No cap reached NEGLIGIBLE in both seeds; only cap25 s2 did.
- **IN_VIEW ≠ visible.**
  - Contact distances at L are below the depth model's MinZ for every crash.
  - The FOV test is geometric, without occlusion.
  - The footprint test is lenient for 0.4–0.8 m bars seen from 0.17 m.
  - The category split depends on the definition (DEFINITION_SENSITIVE). No retraining lever is indicated by Stage A.
  - Not allowed: "BLIND is dominant", which holds under the nearest-point profile only.
- **Lineage limits.** Two seeds of one policy (CAM-K2 v3); one condition (E3's V3 arm, the E2 target); three evaluation
  seeds; Stage B at 115/205 bars only. The intervals are coarse (df 4; 3 cells per density per seed).

## What can and cannot be said for the real vehicle's first flight

**Can be said** (simulation, CAM-K2 v3, this condition):
- **Speed cap.** Lowering the governor's free-speed cap from 3.54 to 2.5 or 2.0 m/s did not lower the simulated crash
  rate in either policy seed, so the simulation gives no support for a lower cap as a crash-reduction lever for this
  policy.
  - Near bars and outside the FOV, the governor already held commands at 2.0 m/s in every arm.
  - The 2.0 m/s cap mainly lengthened pursuit: median capture +1.1–1.2 s, mean speed about 2.1 → 1.8 m/s.
- **First-flight cap.** A conservative cap chosen for test-safety reasons, such as pilot takeover margin, unmodelled
  dynamics, the 1.1 kg vehicle's inferred T/W or OAK-D Pro behaviour, is a separate engineering decision. These results
  neither justify nor forbid it. If one is used, expect slower captures; do not expect fewer contacts from the cap alone.
- **Latency.** A 100 ms command delay raised the simulated crash rate by 3.7–4.9 pp in both seeds. Exposure does not
  explain it (exploratory).
  - The native baseline has zero render latency.
  - The policy is latency-sensitive at one control step in simulation.
  - The vehicle's end-to-end latency, from camera exposure through ROS2 to MAVLink and the controller, should be measured
    and bounded before relying on simulated crash rates, which are optimistic for any nonzero latency.

**Cannot be said:**
- any crash rate, or change in crash rate, for the real vehicle;
- that a cap raises or lowers real crashes;
- that any latency below 100 ms is safe or tolerable (only 0 and 1 step were tested; no jitter, no sensor-side delay);
- that the simulated MinZ (0.4 m) or depth model matches the OAK-D Pro;
- anything about trained-for-latency or trained-at-cap policies (retraining is not approved).

## Recommended wording for the record (allowed sentences)

- CAM-T1 v1 ran once at `b3b9f0b` (64 cells; host faults 0, with boot id and online CPUs unchanged; tree clean).
  Readings: IDENTITY_PASS, NONINTERFERENCE_PASS (24/24 traced cells byte-identical to CAM-K2-E3), CONTROL_PASS,
  envelope COMPLETE, POSITIVE_CONTROL_PASS; Stage A DESCRIBED; Stage B REPORTED (gate controls as required).
- PUBLIC_CPU not 0/0 (baseline-equivalent): 15 pre-existing environment failures, identical to base `eefb832` in a
  fresh worktree.
- Stage A (descriptive, simulation, geometric FOV without occlusion, eye at the trace's mount):
  - Over 6,589 VALID quota crash episodes of CAM-K2 v3, the contacted bar at the last frame before contact was IN_VIEW
    67.7 % [65.7, 69.7], LEFT_FOV 21.5 % [20.4, 22.6] and BLIND 10.8 % [9.4, 12.0] under the preregistered footprint
    profile.
  - Under the `eye_nearest_point` profile it was BLIND 42.5 % [39.8, 45.1], IN_VIEW 30.8 % [29.3, 32.4] and LEFT_FOV
    26.7 % [25.3, 28.3].
  - The guidance reads DEFINITION_SENSITIVE: the two definitions give different clear dominant categories, so Stage A
    indicates no single retraining lever.
- At the last frame the camera eye was closer to the contacted bar than the depth model's MinZ (0.4 m) in all of these
  crashes (median 0.17 m). IN_VIEW therefore means inside the camera's geometric field of view, not visible in depth.
- EARLY crashes (contact within 1 s) were 11.3 %. Non-VALID crashes, 42 (V3s1) and 12 (V3s2), are reported
  separately. Contact side and category shares differ between the two seeds (LEFT 52 % vs 71 %).
- Stage B, per lineage (both seeds), against CAM-K2-E3's native cells at 115/205 bars:
  - **d1 (one-step, 100 ms, action delay) reads WORSE:** s1 +4.95 pp (bootstrap [4.30, 5.60], t [3.79, 6.11]), s2
    +3.67 pp ([3.12, 4.17], [2.74, 4.60]). In simulation, CAM-K2 v3 is latency-sensitive at one control step.
  - **cap25 reads SEEDS_DISAGREE:** s1 NO_DETECTED_CHANGE, +0.33 pp; s2 NEGLIGIBLE, +0.59 pp.
  - **cap20 reads SEEDS_DISAGREE:** s1 NO_DETECTED_CHANGE, +1.74 pp, with both intervals above 0 but the mean below the
    2 pp floor; s2 WORSE, +2.41 pp. The pooled label (WORSE, +2.08 pp) is descriptive and not read.
  - Neither cap lowered the crash rate in either seed. No cap reading reached the guidance's REDUCTION or NEGLIGIBLE
    branch, so the guidance names no speed direction.
- The caps changed only the governor's free-speed end. Commands with corridor clearance ≤ 3 m or outside the camera FOV
  were held at 2.0 m/s in every arm, native included. At cap20 the median capture time was 1.10 s (s1) and 1.20 s (s2)
  longer, and the mean actual speed fell from about 2.1 to 1.8 m/s.
- If R4 is mentioned at all, it is a separate, labelled sentence: "Reviewer-exploratory, not preregistered: per flight
  second, cap20's crash rate was lower than native and per metre about unchanged, consistent with longer pursuit
  exposure; not tested."
- No real-flight claim. The OAK-D model, the vehicle randomization, the 0.4 m MinZ and the validity rule are simulation
  models.

## Verified by running vs by reading

**By running (own code, CPU only; `nice`, `ulimit -v`, `CUDA_VISIBLE_DEVICES=""`, `python3 -B -I`):**
- `recompute_stage_b.py`:
  - every Stage B rate, contrast, interval and label from the raw ledgers;
  - an exact replica of the preregistered bootstrap plus an independent numpy bootstrap;
  - the t constant check;
  - the gate controls on CAM-K2-E's raw ledgers;
  - the determinism and arms-act byte checks;
  - commanded and requested speed maxima;
  - the exploratory exposure rates.
- `recompute_stage_a.py`:
  - own trace parser (dtype from each summary; SHA-256 of both files) and own episode grouping;
  - own contacted-bar, eye and footprint geometry: a wedge/rectangle intersection by corners and boundary rays, a
    different algorithm from the analysis's corner-angle interval;
  - own cell-cluster bootstrap and dominance rule;
  - all 6,643 quota crash episodes of the 24 cells, plus a 1-in-10 smoke run.
- `explore_extra.py`: nearest-point depth-range visibility, contacted-bar sizes, per-metre rates.
- Inline checks:
  - all 24 traced ledgers vs E3 (48 SHA-256);
  - 64 receipts (commit, dirty files, status, snapshot keys, checkpoint pins);
  - hook sidecars;
  - E3's 96 reference files vs the HEAD manifest, and E3's eval report;
  - analysis and runner SHA vs HEAD, and the launcher pin;
  - the G3 positive control vs the committed record;
  - the approval scope's string equality;
  - quota-by-index, and scene-hash pairing;
  - host state now (fault count, boot id, online CPUs), `git status`, `nvidia-smi`.

**By reading only:**
- the prereg;
- `camt1_run.py` and `camt1_analyse.py`;
- the gate helpers (`gate_a`, `strat_t`, `strat_ci`, `lineage`, `rates`);
- `apply_speed_governor`; the hook's `act()`; `run_camt1.sh`'s gate and approval lines;
- the stage reports (host_start / host_end, stop flags, rate gate) and chain.log;
- the stage logs.

**Not verified:**
- the user's original approval message;
- the velocity-bearing, roll/pitch, speed and target-visibility shares, and the env/episode-level bootstrap intervals
  (not recomputed);
- the riskcap clearance source in `no_lidar` (taken from the prereg);
- that no tracked file was edited and reverted between cells, beyond each cell receipt's 0 dirty files;
- the test suite (not rerun);
- `camt1_analyse.py` itself (not rerun).

## Files written by this review (`~/motar_tmp/camt1_20261112/review_results_r1/`)

- `REVIEW.md` (this file);
- `recompute_stage_b.py`, `stage_b_recompute.json`;
- `recompute_stage_a.py`, `stage_a_recompute.json`, `stage_a_recompute.log`, `episodes_recompute.jsonl`;
- `stage_a_recompute_every10.json`, `episodes_recompute_every10.jsonl` (smoke);
- `explore_extra.py`, `explore_extra.json`.
