# B1-R publication evidence and mechanism audit (2026-10-24)

This audit was read-only and ran on the CPU. It started no simulation, did no re-tuning, and changed
no raw data, preregistration or existing evidence.

- **Script:** [`tools/b1r_publication_audit.py`](../../../tools/b1r_publication_audit.py).
- **Machine-readable output:** [`audit.json`](audit.json).
- **Sources the numbers were recomputed from:**
  - the B1-R raw cells;
  - their sidecars and manifests;
  - the committed Stage B record [`stage_B_selection.json`](../b1r_v1/stage_B_selection.json),
    for the tuning table.
- **Preregistration:** [`prereg_b1r_classical_baseline_v1.md`](../../plans/prereg_b1r_classical_baseline_v1.md).
- **Review.** Opus 5.5 reviewed the report independently and read-only, with verdict
  PASS_WITH_CHANGES. Every finding was adopted (see the end of this report).

The report has three parts:
- **§1 Verification:** an independent recomputation of the preregistered result. It adds no claim.
- **§2 Exploratory:** descriptive and observational diagnostics. They are not causal and are not
  new verdicts.
- **§3 TF32:** a technical sensitivity check. It does **not** change the preregistered C1/C2
  reading.

## 1. Verification of the preregistered result

**Integrity.** Each of the four stage raw manifests verified file by file, and each manifest SHA
equals its committed stage record:

| Stage | Files | Manifest SHA |
| --- | ---: | --- |
| A | 395 | `31fbb9b8…` |
| B | 451 | `2a01b676…` |
| C-PN | 395 | `4e80dbaa…` |
| C-PP | 395 | `6308506d…` |

`analysis.json` and `selection.json` also match their committed SHA bindings.

**Primary endpoint, recomputed from ledger rows.**
- In-quota rows were counted directly, not read from the summaries. They equal the summary counts,
  with 512 rows in every cell.
- Every quota row carries exactly one outcome flag.
- The recomputed seed-level values equal `analysis.json` exactly (maximum absolute difference 0).

| Seed | F | B-PP | B-PN |
| --- | ---: | ---: | ---: |
| 4301 | 0.9487 | 0.8647 | 0.6421 |
| 4302 | 0.9497 | 0.8613 | 0.6499 |
| 4303 | 0.9517 | 0.8672 | 0.6543 |
| 4304 | 0.9556 | 0.8726 | 0.6597 |
| 4305 | 0.9565 | 0.8618 | 0.6528 |
| 4306 | 0.9561 | 0.8628 | 0.6455 |
| 4307 | 0.9551 | 0.8618 | 0.6426 |

**Contrasts, recomputed with independent code.** The checks used their own sign-flip enumeration,
Holm over C1–C2, and `scipy.stats.bootstrap` BCa.

| Contrast | Mean | Same sign | Exact p | Holm p | BCa 95 % (scipy, seed 20261023 / seed 7) | Recorded reading |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| C1 F − B-PN | +0.3038 | 7/7 | 0.015625 | 0.03125 | [0.2995, 0.3084] / [0.2995, 0.3083] | SUPERIOR |
| C2 F − B-PP | +0.0887 | 7/7 | 0.015625 | 0.03125 | [0.0854, 0.0923] / [0.0853, 0.0922] | SUPERIOR |
| C3 B-PN − B-PP | −0.2151 | 7/7 | 0.015625 | — | [−0.2187, −0.2121] / [−0.2188, −0.2121] | descriptive |

The mean, exact p and Holm p equal `analysis.json` exactly. The BCa bounds change by about 1e-4
when the bootstrap seed changes.

**Density, pooled over 7 seeds (3,584 quota episodes per arm and density).** Each entry is capture /
crash / timeout.

| Arm | 70 bars | 115 bars | 160 bars | 205 bars |
| --- | --- | --- | --- | --- |
| F | .9855 / .0145 / 0 | .9760 / .0237 / .0003 | .9473 / .0527 / 0 | .9046 / .0954 / 0 |
| B-PP | .9754 / .0226 / .0020 | .9333 / .0625 / .0042 | .8368 / .1565 / .0067 | .7129 / .2768 / .0103 |
| B-PN | .8281 / .0926 / .0792 | .7115 / .1521 / .1364 | .5837 / .2723 / .1440 | .4749 / .3610 / .1641 |

**Capture and crash.**
- **Precedence.** Capture is tested after crash on the same step, and capture wins
  (`navrl_task.py:6123-6124`, `crashed_out & ~captured`).
- **Not recorded.** Whether a captured episode also touched a bar *on its capture step* is
  NOT_RECORDED. There is no per-episode contact record. The frame trace also ends one step before
  termination: frames per episode equal actions, and frame 0 is the reset observation.
- **Consequence.** Capture must therefore not be called collision-free.

**Tuning limits.**
- **B-PN gain sits on the grid edge.** B-PN capture rises strictly with N at every v_nom. At
  v_nom = 1.5 m/s, the capture totals over the two tuning cells (2 × 256 = 512 quota episodes) are
  160, 233 and 312 for N = 3, 5 and 8. The selected N = 8 therefore lies on the edge of the grid.
  This is a limit, not a defect; re-tuning is forbidden.
- **B-PP selection is a near-tie.** The best two B-PP settings, v_nom = 2.5 and 3.0, captured 425
  and 422.
- **The single tuning seed was representative.** It was slightly optimistic at 205 bars:

| Arm | Density | Tuning seed 4390 | Evaluation mean |
| --- | ---: | ---: | ---: |
| B-PN | 115 | 0.695 | 0.711 |
| B-PN | 205 | 0.523 | 0.475 |
| B-PP | 115 | 0.930 | 0.933 |
| B-PP | 205 | 0.730 | 0.713 |

## 2. Exploratory: descriptive differences between F and the baselines (not causal)

**Windows used in this section:**
- **Quota per-episode means:** `kroc2027_episodes.jsonl` joined to the in-quota ledger rows.
- **`LEGACY_STEP_WINDOW`:** the `result.json` aggregates, covering all env-steps up to
  `cutoff_step`, including episodes still in progress.
- **Frame states:** `occlusion_frames.bin` restricted to quota episodes. "Post-acquisition" means
  after the first fused detection (`ACQUIRED_BEFORE`).

**Crash causes** (`LEGACY_STEP_WINDOW`, 7 seeds pooled). There is no per-quota-episode cause
record, so the quota-level cause is NOT_RECORDED.

| Arm | Out-of-arena share of crashes, 70 → 205 bars | Remainder |
| --- | --- | --- |
| F | 0/58, 0/84, 1/193, 2/352 | almost all bar contact |
| B-PP | 3/76, 4/232, 10/550, 7/988 (≤ 0.04) | bar contact |
| B-PN | **253/301 (0.84), 367/563 (0.65), 507/1012 (0.50), 474/1380 (0.34)** | bar contact |

B-PN's quota crashes have a median centre-to-bar distance at the last recorded frame of 2.79 m at
70 bars and 1.97 m at 115 bars. Most of those episodes were nowhere near a bar.

Two facts matter here:
- Neither B arm has an arena-boundary input. The LiDAR ray-casts only the bars and the target.
- C1 (F − B-PN) therefore partly reflects B-PN leaving the arena, not obstacle handling.

**C2 (F − B-PP) is the obstacle-relevant classical contrast.** B-PP's crashes are 96–99 % bar
contacts.

**Speed chain.** Quota per-episode means in m/s, 70 → 205 bars:

| Arm | Requested | Executed (governed) | Actual |
| --- | --- | --- | --- |
| F | 3.23 → 3.23 | 3.09 → 2.82 | 2.71 → 2.45 |
| B-PP | 2.26 → 1.93 | 2.22 → 1.84 | 2.02 → 1.65 |
| B-PN | 1.41 → 1.20 | 1.41 → 1.20 | 1.35 → 1.14 |

Governor intervention (`LEGACY_STEP_WINDOW`), 70 → 205 bars:
- **F:** 0.14 → 0.41, with mean scale 0.96 → 0.87.
- **B-PP:** 0.11 → 0.24.
- **B-PN:** 0. Its request of about 1.4 m/s is below riskcap's 2.0 m/s floor, so the shared
  governor never acts on B-PN.

**Reading.** F requests and flies faster than both baselines, and it is governed more often.

**State just before a crash.** Last recorded frame of quota crash episodes, one action before
termination:

| Arm | Speed median (p90) | Centre-to-bar-surface median | n |
| --- | --- | --- | --- |
| F | 1.94–1.98 m/s (2.15–2.27) | 0.21–0.22 m | 668 |
| B-PP | 1.29–1.32 m/s (1.71–2.01) | 0.21–0.23 m | 1,858 |
| B-PN | 0.97–1.49 m/s | 0.23–2.79 m | 3,147 |

`LEGACY_STEP_WINDOW` contact steps, measured after impact:

| Arm | Actual speed | Requested | Executed | Executed stopping margin |
| --- | --- | --- | --- | --- |
| F | 1.15–1.33 m/s | ≈ 3.2 m/s | ≈ 2.0 m/s | −0.36 to −0.81 m |
| B-PP | 0.73–0.80 m/s | 1.37–1.73 m/s | 1.29–1.53 m/s | −0.41 to +0.03 m |

**Hypothesis H1 (not tested).** B-PP crashes more often than F, but at lower pre-contact speed. Its
excess crashes are therefore not explained by higher speed. They are consistent with geometric
failure modes of a reactive line-of-sight follower, such as side contact while following the belief.
The stopping margin is measured along the command direction, so it cannot show a side contact.

**Target loss.** Shares of post-acquisition frames (frame counts in `audit.json`):

| Arm | Camera lost, tracker active ("track-only") | No track | Speed with no track |
| --- | --- | --- | --- |
| F | 0.12 → 0.31 | ≤ 0.003 | 1.6–2.2 m/s (26–505 frames per density) |
| B-PP | 0.10 → 0.25 | 0.02 → 0.08 | 0.02–0.06 m/s |
| B-PN | 0.16 → 0.33 | 0.07 → 0.20 | ≈ 0.04 m/s |

With no track, both baselines stop and scan by design. F keeps flying and almost never loses the
track.

**Crashes near camera loss.** The share of crashes whose last recorded frame was in the track-only
state, and how many times larger that is than the arm's track-only frame share:

| Arm | Share of crashes | Ratio to frame share |
| --- | --- | --- |
| F | 0.45–0.58 | 1.8–4.9× |
| B-PP | 0.69–0.72 | 2.8–7.1× |
| B-PN | 0.53–0.65 | 2.0–3.4× |

- **Every arm shows the enrichment.** It is therefore not specific to line-of-sight following.
- **The direction is unknown.** Losing the camera may lead to a crash, or nearness to a bar may block
  the camera.
- **Status of this table.** It is a post-hoc look at the same question as the preregistered R01
  exploratory endpoint, with ad-hoc state definitions and no hazard model. It is **not** the R01
  analysis, which remains NOT_ANALYSED. Because this table now exists, any later R01 amendment is no
  longer blind.

**Time and crash.**
- The preregistered RMTC60 (restricted mean time to capture, non-captured episodes counted as 60 s)
  over all quota episodes, 70 → 205 bars:
  - F: 4.84 → 10.05 s;
  - B-PP: 8.33 → 24.01 s.
- F has the lower RMTC60 and the lower crash rate at every density, so no speed–crash trade-off is
  visible *between* arms.
- A within-arm split by actual speed is in `audit.json` but is confounded and not interpretable.
  Crash episodes include braking, which lowers their mean speed.

**Density pattern** (exploratory, uncorrected). The F − B-PP capture gap:

| 70 bars | 115 bars | 160 bars | 205 bars |
| ---: | ---: | ---: | ---: |
| +1.00 pp | +4.27 pp | +11.05 pp | +19.17 pp |

The gap is almost entirely crashes. At 205 bars, F crashes in 9.54 % of episodes and B-PP in
27.68 %.

## 3. TF32 sensitivity (technical; the preregistered reading is unchanged)

**Known exposure** ([`moving_target_tf32_readonly_2026-10-16.md`](../../audits/moving_target_tf32_readonly_2026-10-16.md)):
- `torch.cdist` can take a matmul path, and TF32 matmul is on.
- On the RTX 3070, the kernel selection erred by about 2 m when the second point set had a size
  divisible by 4 and the batch was at least 2. Here that is 160 bars.
- That audit also recorded nonzero errors at other densities and calls the pattern "not a
  safe-density whitelist".
- TF32 stays on for every float32 matmul, including the policy, the tracker and the task, in all
  arms. This is part of the shared contract.

**Target motion.**
- B1-R uses `direct_l2_v1` difference norms, so the audited target-motion defect path is not used.
- The recorder judges swept clearance with exact geometry (`swept_proxy_clearance`).
- Across all 84 evaluation cells: 0 swept violations, 0 wall violations, motion gate PASS in every
  cell, and minimum swept clearance 0.2216 m.

**Placement.** In this contract, both the target (initial view) and the pursuer are placed through
`_spawn_footprint_clearance_accepted` (`navrl_task.py:133`), via
`kroc_eval_adapter.sample_initial_view_for_task` and `navrl_task.py:2398`.
- That `cdist` has **one query point per batch row**. The earlier TF32 measurements found
  millimetre-to-centimetre error for this shape.
- By source reading, `navrl_task.py:2745` is bypassed when the KROC initial view is set
  (`navrl_task.py:4742`).
- `:4813` and `:4827` are inferred not to run on this path.
- `kroc_eval_adapter.py:1069` runs only when a violation occurs, and none did.

**Empirical check of placement.**
- *Method.* Every quota episode's start clearance was recomputed with exact rotated-box geometry,
  from frame 0 and the recorded layouts: 3,584 starts per arm and density. The bars are yaw-only,
  full-height boxes.
- *Result.*
  - Minimum surface clearance is 0.7245–0.7318 m for the pursuer and 0.7253–0.7394 m for the target,
    with 0 starts below 0.4 m.
  - The quantiles agree across arms (160-bar pursuer medians 1.343 / 1.338 / 1.352 m for F / B-PP /
    B-PN).
  - 160 bars is not anomalous, and crashes within 5 actions are 0 except one F episode at 205 bars.
- *Arm independence.* Episode 0 of every environment is placed identically in all arms. Later reset
  batches differ by arm, because episode lengths differ. Arm independence is therefore supported
  empirically, not by construction.
- *Limits.* The minimum-clearance check detects false acceptances only; a false rejection would
  shift the distribution without lowering the minimum. **There is no sign of a TF32 placement
  defect.** This does not prove bitwise equality with a TF32-off run.

**Sensitivity with 160 bars excluded** (partial; mean over 70, 115 and 205 bars):

| Contrast | Mean | Same sign | Holm p | BCa 95 % |
| --- | ---: | ---: | ---: | --- |
| C1 | +0.2839 | 7/7 | 0.03125 | [0.2776, 0.2897] |
| C2 | +0.0815 | 7/7 | 0.03125 | [0.0763, 0.0870] |

The direction and the resolution are unchanged. The preregistered reading stands as recorded.

## 4. Limits that travel with the result

- **The baselines are internal and reactive.** They are online-belief guidance plus a LiDAR approach
  filter, with no map, no trajectory optimisation and no arena-boundary input. They are not a
  published system: Class A = 0 is unchanged, and no planner baseline has run.
- **C1 partly reflects B-PN leaving the arena,** at 34–84 % of its crashes. **C2 is the
  obstacle-relevant contrast.**
- **The shared riskcap governor never acts on B-PN,** whose request is below its 2.0 m/s floor.
- **Speed asymmetry,** disclosed in the preregistration. The baselines fly the line of sight at
  ≤ 2.5 m/s per axis. F requests about 3.2 m/s with lateral components.
- **Tuning.** One tuning seed; B-PN's N = 8 lies on the grid edge.
- **Scope.** Simulation only, one arena, four densities, frozen policy F. Capture is the swept
  < 0.5 m test, and a contact on the capture step is NOT_RECORDED. There is no physical-interception
  or real-flight claim.
- **R01** is NOT_ANALYSED, and the §2 table above means any later amendment is no longer blind.

## 5. Baseline value vs novelty value

- **Baseline value: present within the contract.** This is the first matched comparison in MOTAR in
  which policy F faces simple classical guidance under an identical task path.
  - F beats both reactive baselines on every seed. This comes from the preregistered C1/C2 result.
  - C2, against B-PP, is the obstacle-relevant part.
  - This addresses the "trivial controller" objection *for these two tuned reactive controllers*. It
    is bounded by the tuning, speed and arena-exit limits in §4.
- **Novelty value: not established.** Comparing learned policies against PN or pure pursuit is an
  established type of experiment. B1-R adds no new method and no evidence about planners, real
  flight or physical capture.

**Hypotheses for further work.** Each is grounded only in what was observed here, and each would
need its own preregistration:
- **H1.** B-PP's excess crashes are low-speed geometric contacts.
- **H2.** Camera-loss frames are over-represented before crashes in every arm. Whether loss precedes
  the crash or proximity to a bar causes the loss is untested.
- **H3.** B-PN capture may keep rising beyond N = 8, given the monotone trend over N = 3, 5 and 8.
  Testing this would need new GPU work.
- **H4.** B-PN's lead pursuit leaves the arena because neither baseline has a boundary input.

## Review disposition (Opus 5.5, PASS_WITH_CHANGES)

| Finding | Disposition |
| --- | --- |
| P1: B-PN arena exits omitted | Adopted: crash-cause table, C1 caveat, H1 limited to B-PP, H4 added |
| P1: the README trim removes another result's caveat | Moved to a separate optional patch for a user decision |
| P2: unbound numbers | Adopted: medians replaced by the preregistered RMTC60, and numeric assertions added in the proposed test |
| P2: registry sentence | Adopted (reworded) |
| P2: wording | Adopted: "descriptive differences", "speed–crash", scoped baseline value |
| P2: contact speed measured after impact | Adopted: pre-crash last-frame speed added |
| P2: tracking-state table selective and post hoc | Adopted: B-PN added, range corrected to 0.69–0.72, R01 blinding note |
| P2: TF32 reasoning | Adopted: active-site correction, empirical arm independence, detection limit, partial exclusion, scope |
| P3 | Adopted. On the "out of 512" point: the tuning totals are sums over two cells of 256 quota episodes each, so 512 is correct |
