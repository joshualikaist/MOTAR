# Independent review r1: camera-only G3 trace analysis (c2trace)

- **Reviewer:** fresh-context Claude Code subagent, model **Opus 5.5** (`claude-opus-5-5`), read-only on every repository.
- **Date:** 2026-11-05. CPU work 09:53–10:03 KST on cores 6,7,12,13 (`nice -n 10`, `ulimit -v 16000000`,
  `CUDA_VISIBLE_DEVICES=""`).
- **Audit directory (only write location):** `/home/fair/motar_tmp/reviews/g3_c2trace_r1/`.

## Verdict: **PASS_WITH_LIMITS**

- The analysis follows the preregistration's definitions, gate and reading order.
- Every number in `c2trace_analysis.json` reproduces exactly from the raw traces with an independent script, CIs
  included (bit-identical once episode order matches).
- The reading **BLIND_SIDE_SUPPORTED** is correct under the prereg rules. It also holds under every sensitivity I ran:
  quota-only episodes, valid-only episodes, camera-eye origin, body yaw, a two-stage cell bootstrap and per density.
- The run is complete (12/12 OK) and NONINTERFERENCE_PASS, which I verified independently. Host faults stayed 0 on the
  same boot.
- **Why the limits:**
  - one preregistered "reported, not gated" item, the per-density counts, is missing from the output JSON (F1);
  - the scope and interpretation limits in F3, F4 and F12 should travel with the result.
- **No BLOCKER or MAJOR finding.**

## Inputs (SHA-256)

| File | SHA-256 |
| --- | --- |
| `MOTAR-camonly/docs/plans/prereg_camonly_c2trace_v1.md` | `ae09ad5ebaea318012afd9e409c3103ea767ed0e441ccff61df6dd15399a6e0c` |
| `MOTAR-camonly/tools/camonly_c2trace_analyse.py` | `64a42a673d631e9ec3af7576ddc4e788685dce066cc3285f4af474f38a5de518` |
| `MOTAR-camonly/docs/evidence/camonly_2026-11-04/c2trace/c2trace_analysis.json` | `decd64eacaa81f2403f78ae1586ee31eb2c0809b852a26da62a771895a99bdc8` |
| `results/camonly_c2trace_v1/c2trace_run_report.json` | `8710c3eb1cb38bc48019d3732f6506aad88510f62d387e52dc2494a0b32cbeb8` |

**Provenance:**
- Prereg, runner and analysis were committed together in `90c00b4` (2026-11-05 02:57).
- None of the three changed through HEAD `9882889`, and the worktree copy of the analysis equals HEAD.
- `90c00b4` is an ancestor of the run commit `c4bc416`.
- The run started at 09:16:13.
- `queue.log` shows G3 started exactly once (09:16:13 → exit 0 at 09:48:43). The earlier queue scripts stopped at G8,
  before G3, so there was no selective rerun.
- All 36 `inputs_sha256` entries in the analysis JSON match the raw files on disk.

## Recomputed numbers vs the JSON

Independent script: `recompute.py` / `recompute_v2.py`.
- It has its own frame parser (dtype taken from each cell's summary, both SHA-256 values checked) and its own grouping
  and geometry.
- It does not import the analysis tool or `motar_eval`.
- `recompute_v2.py` differs only in sorting crash episodes by (env, episode) inside each cell, which is the tool's order.

| Quantity | JSON | Recomputed |
| --- | --- | --- |
| A crash episodes | 475 | 475 |
| B_out crash episodes | 5138 | 5138 |
| C crash episodes | 5109 | 5109 |
| valid_share A / B_out / C (%) | 97.263 / 99.825 / 99.746 | 97.263 / 99.825 / 99.746 |
| **out_last A (%)** | **48.842** | **48.842** |
| **out_last B_out (%)** | **72.071** | **72.071** |
| **out_last C (%)** | **72.852** | **72.852** |
| never_in_1s A / B_out / C (%) | 33.684 / 56.462 / 56.743 | 33.684 / 56.462 / 56.743 |
| left A / B_out / C (%) | 63.579 / 93.324 / 93.384 | 63.579 / 93.324 / 93.384 |
| mean abs bearing A / B_out / C (deg) | 54.335 / 65.983 / 66.196 | 54.335 / 65.983 / 66.196 |
| travel_out A / B_out / C (%) | 52.064 / 70.248 / 70.678 | 52.064 / 70.248 / 70.678 |
| moving frames A / B_out / C | 435374 / 548015 / 539717 | 435374 / 548015 / 539717 |
| **Δ C−A (pp)** | **24.010, CI [19.324, 28.650]** | **24.010, CI [19.324, 28.650]** (v2; v1 ledger order: [19.345, 28.788]) |
| Δ B_out−A (pp) | 23.229, CI [18.597, 27.878] | 23.229, CI [18.597, 27.878] (v2) |
| Reading | BLIND_SIDE_SUPPORTED | BLIND_SIDE_SUPPORTED: Δ 24.0 ≥ 15, lower bound 19.3 > 0, C 72.9 ≥ 60 |

Differences in the last digit are float summation order (≤ 2e-13).

### Per-density counts (prereg §3 "reported, not gated"; absent from the JSON, see F1)

| Cond / bars | Crash (all traced) | Crash (in quota) | out_last all (%) | out_last quota (%) | valid (%) |
| --- | --- | --- | --- | --- | --- |
| A 70 | 38 | 28 | 42.1 | 39.3 | 100.0 |
| A 115 | 66 | 51 | 56.1 | 52.9 | 93.9 |
| A 160 | 145 | 103 | 44.1 | 35.9 | 97.2 |
| A 205 | 226 | 181 | 50.9 | 51.9 | 97.8 |
| B_out 70 | 775 | 549 | 72.6 | 70.5 | 99.6 |
| B_out 115 | 974 | 782 | 72.3 | 72.9 | 99.8 |
| B_out 160 | 1405 | 979 | 72.2 | 73.5 | 99.9 |
| B_out 205 | 1984 | 1227 | 71.6 | 72.3 | 99.9 |
| C 70 | 672 | 522 | 74.7 | 73.8 | 99.9 |
| C 115 | 1159 | 828 | 74.1 | 74.5 | 99.9 |
| C 160 | 1490 | 1068 | 72.3 | 71.9 | 99.6 |
| C 205 | 1788 | 1244 | 71.8 | 71.3 | 99.7 |

The quota crash counts equal `result.json` and the run log for every cell (for example A: 28/51/103/181).

### Sensitivity (reviewer-only; not preregistered, not a relabel)

| Variant | out_last A | out_last C | Δ C−A (pp), 95 % CI |
| --- | --- | --- | --- |
| As preregistered | 48.8 | 72.9 | 24.0 [19.3, 28.7] |
| Quota episodes only (`in_quota`) | 46.6 | 72.6 | 26.0 [20.7, 31.3] |
| Valid episodes only (dist ≤ 0.6 m) | 47.8 | 72.9 | ≈ 25.0 |
| Excluding ambiguous contacts (second bar within 0.1 m) | 48.8 | 72.9 | ≈ 24.1 |
| Body yaw (`p_quat`) instead of `p_vehicle_quat` | 48.8 | 72.9 | 24.0 [19.3, 28.6] |
| Bearing from camera eye (0.1 m forward), body yaw | 62.7 | 84.0 | 21.3 [16.7, 25.8] |
| Two-stage cell bootstrap (cells, then episodes; 2,000) | — | — | CI [17.7, 31.3] |
| Per-density Δ (70 / 115 / 160 / 205) | — | — | 32.6 / 18.1 / 28.2 / 20.9 |
| Density-weighted Δ (A's crash weights) | — | — | 23.7 |
| Other bootstrap seed (12345) | — | — | CI [19.5, 28.8] |

Every variant still meets all three BLIND_SIDE_SUPPORTED conditions.

## Definition-by-definition check (task item 1)

| Item | Prereg | Code | Status |
| --- | --- | --- | --- |
| FOV | \|bearing\| ≤ 43.5°, yaw-only vehicle frame, x = camera axis (prereg:45) | `HALF = radians(87)/2` (analyse.py:29); outside = `abs(b) > HALF` (analyse.py:93, 102, 103); yaw from `p_vehicle_quat` (analyse.py:86) | MATCH |
| Bearing frame | yaw-only vehicle frame | `atan2(off_y, off_x) − yaw`, wrapped (analyse.py:67-68). World offsets: `p_pos` = `robot_position` and obstacle positions share a frame. Capture positive control: last-frame pursuer–target distance median 0.58 m, max 0.95 m | MATCH |
| Bars | axis-aligned boxes spanning the vehicle's height, as `f_trace_geometry.py` | `surface()` (analyse.py:57-64) is byte-equivalent to f_trace_geometry.py:55-63. Data: 4,837,630 layout rows, all quats identity; half_z 1.5, centre z 1.5 (0–3 m); half_xy 0.201–0.395 m; box URDFs (env_object_config.py:795-818) | MATCH |
| Crash episodes | `episodes.jsonl` outcome crash, every traced episode (prereg:47) | `outcome.get(key) == "crash"` over all trace keys (analyse.py:94). This includes `in_quota=false` rows (docstring analyse.py:7). 0 crash rows lack frames or a layout | MATCH (see F3) |
| Contact bar | nearest by surface distance at the last recorded frame (prereg:48) | `argmin(surface(pos[-1]))` (analyse.py:96-97). The last frame is the last consumed observation, i.e. the state after N−1 of N actions: frames == `actions` and contiguous from 0 in all 33,117 ledger episodes | MATCH |
| out_last | contact bar outside the FOV at the last frame | analyse.py:98, 103 | MATCH |
| never_in_1s | outside in each of the last 11 frames (prereg:53) | `range(max(0, len−11), len)`, same bar j (analyse.py:99-102) | MATCH (see F7 for episodes under 11 frames) |
| left | contact bar on the left | `b_last > 0` (y-left, analyse.py:104) | MATCH |
| travel_out | share of moving frames whose velocity bearing is outside the FOV (prereg:55) | horizontal speed > 0.3 m/s (analyse.py:30, 90-93). `p_vel` = `robot_linvel`, world frame (kroc_eval_adapter.py:391; IGE_env_manager.py:448), minus yaw | MATCH; threshold only in code (F2) |
| Validity gate | ≥ 90 % of crash episodes with last-frame dist ≤ 0.6 m, each condition (prereg:57-58) | `valid_share_pct < 90.0` → INVALID (analyse.py:136, 151). Extra `n == 0` clause (F9) | MATCH |
| Contrast | Δ = out_last(C) − out_last(A) in pp (prereg:59) | analyse.py:146 | MATCH |
| Bootstrap | 95 % percentile, over crash episodes, within each condition, 10,000, seed 20261105 (prereg:59-60) | independent `integers` resamples of x and y, `NBOOT = 10000`, `default_rng(20261105)`, percentiles 2.5 / 97.5 (analyse.py:34, 108-113, 142) | MATCH (see F8) |
| Reading order | INVALID → SUPPORTED → NOT_SUPPORTED → INCONCLUSIVE, first match (prereg:61-68) | analyse.py:151-158; thresholds `>= 15.0`, `> 0`, `>= 60.0`, `< 15.0`, `< 50.0` | MATCH |
| Reported, not gated | B_out − A; never_in_1s, left, travel_out per condition; **per-density counts** (prereg:75-78) | first two present; per-density counts **absent** (analyse.py:122-141) | **MISMATCH (F1)** |

## Run report and noninterference (task item 2)

- `c2trace_run_report.json`:
  - reading `NONINTERFERENCE_PASS`; 12 cells, all `verdict OK`, `exit 0`, `problems []`;
  - `preflight_problems []`; import origins inside `MOTAR-camonly`; commit `c4bc416`;
  - cell order density then condition, as preregistered (prereg:25).
- **Independent byte check:**
  - all 12 traced `episodes.jsonl` files are SHA-256-identical to their references (A: GR-0 S00; B_out and C: C2);
  - so are all 12 `kroc2027_episodes.jsonl`;
  - the runner's logic is camonly_trace_run.py:51-63 and 125-131.
- **Host:**
  - `fault_count` was 0 at start, after each of the 12 cells and at end;
  - boot_id `407ae422…` and online CPUs `0-3,6-23` were the same throughout;
  - `journalctl -k -b 0` still counts 0 now on the same boot;
  - the queue log's shell counter reads `faults=0` before and after G3.
- Every trace summary reads `reason complete`, `source_dirty false`, `render_alignment ZERO_LATENCY_RENDER_DELIVERY`,
  with obstacle slots equal to the bar count.
- Wall time was 32.5 min, inside the 1.5 h budget.

## Findings

**F1 — MINOR — Preregistered per-density counts are not in the analysis output.**
- Prereg:78 lists "per-density counts" under "Reported, not gated".
- `camonly_c2trace_analyse.py:122-141` pools the four densities and writes only per-condition values.
- No reading depends on them. They are supplied above, from the reviewer's recomputation.
- **Fix:** add them in a separate dated supplement. Do not overwrite `c2trace_analysis.json`; the tool refuses to
  overwrite (analyse.py:117-118).

**F2 — NOTE — travel_out's moving threshold and frame set are fixed in code, not in the prereg text.**
- Prereg:55 gives no speed threshold.
- The tool, committed in the same prereg commit `90c00b4` and named "fixed here" (prereg:43), uses horizontal speed
  > 0.3 m/s (analyse.py:11, 30), as the reference does (f_trace_geometry.py:31).
- travel_out counts frames of every traced episode, including the 128 in-flight episodes per cell that have no ledger
  row. Ledger-only values are 52.20 / 70.22 / 70.67 %, against 52.06 / 70.25 / 70.68 %.

**F3 — NOTE — The crash set includes episodes outside the quota.**
- Out-of-quota crash episodes:
  - A: 112 of 475;
  - B_out: 1,601 of 5,138;
  - C: 1,447 of 5,109.
- This matches prereg:47 ("every traced episode") and the tool docstring (analyse.py:7).
- These rows carry the per-environment length bias that `navrl_episode_ledger.py:7-14` describes: each environment's
  in-flight episode is dropped, so short episodes are over-represented.
- They are not part of C2's quota metrics, although they are byte-identical to C2's ledger rows.
- Quota-only: Δ 26.0 pp [20.7, 31.3]. The reading is unchanged.
- **Fix:** state the episode set in the record ("all traced crash episodes, quota and post-quota").

**F4 — MINOR (interpretation of absolute levels) — The FOV origin is the vehicle centre, not the camera eye.**
- Bearings are taken from `p_pos`. The camera sits 0.1 m forward (trace metadata `translation_m [0.1, 0, 0.03]`,
  kroc_eval_adapter.py:351), and the adapter's own frustum test uses the eye (kroc_eval_adapter.py:881-882).
- At the median last-frame surface distance (0.21 m for A, 0.23 m for C), the origin matters:
  - out_last from the eye is 62.7 % for A and 84.0 % for C;
  - Δ from the eye is 21.3 pp [16.7, 25.8].
- The prereg's "yaw-only vehicle frame" and the reference both imply the centre, so this is not a deviation.
- However, "outside the camera FOV" at 0.1 s before contact is not the camera's literal view. In particular, A's 48.8 %
  is not "F's camera saw the bar in 51 % of A crashes".
- The contrast and the reading are robust to the choice.

**F5 — NOTE — The yaw source lags the body by up to one physics substep.**
- `robot_vehicle_orientation` is refreshed in `update_states()` at the start of a step (base_multirotor.py:287-289).
  `robot_orientation` is live.
- The yaw gap between `p_vehicle_quat` and `p_quat` is median ≈ 0°, p99 0.8°, max 5.7°. Frame 0 is exact.
- Using body yaw flips 1 of 10,722 contact classifications (in B_out). Negligible.

**F6 — NOTE — Validity-gate failures stay in out_last.**
- Crash episodes with last-frame distance > 0.6 m: A 13, B_out 9, C 13.
- Most are at z = 0.1–0.4 m within 7–11 frames, likely floor contacts during the climb. A few are at about 1 m height
  with 1–3 m clearance (cause NOT_MEASURED).
- Their "contact bar" is not a contact. Per the prereg they count in out_last; the gate only bounds them.
- Valid-only out_last: A 47.8 %, C 72.9 %.

**F7 — NOTE — never_in_1s uses fewer than 11 frames when an episode is shorter.**
- Episodes under 11 frames: A 41, B_out 191, C 208. The code (analyse.py:100) and the reference
  (f_trace_geometry.py:167) use all available frames. The prereg is silent.
- Restricted to episodes of 11 frames or more: A 35.5 %, B_out 58.4 %, C 58.7 %, against 33.7 / 56.5 / 56.7 %.
- Reported only, not gated.

**F8 — NOTE — B_out−A's bootstrap continues C−A's RNG stream.**
- analyse.py:142-147: B_out−A is drawn from the same generator after C−A, not from a fresh seed.
- It is reproducible and reported only.
- The CIs also depend on episode iteration order (dict order of `episode_frames`). With ledger order the C−A CI is
  [19.345, 28.788]: Monte Carlo noise, not an error.

**F9 — NOTE — Extra guard: zero crash episodes reads MEASUREMENT_INVALID.**
- analyse.py:151 adds `crash_episodes_traced == 0` to the invalid rule.
- It is stricter than the prereg and did not trigger.

**F10 — NOTE — The CI's independence assumption.**
- As prereg §4 states (prereg:84-85), episodes share a cell's layout seed.
- A crude two-stage cell bootstrap (only 4 cells per condition) gives [17.7, 31.3].
- Per-density Δ ranges from 18.1 to 32.6 pp, so every cell individually exceeds 15 pp.

**F11 — NOTE — Provenance fields.**
- The analysis JSON records input hashes but not the analysis tool's SHA or commit, nor the run report's reading.
- The tool does not check that the run read NONINTERFERENCE_PASS before reading traces. Prereg:39-40 wants the analysis
  to run under TRACE_INTERFERES too, so this is acceptable, but the record should carry the run reading.
- Each cell's `occlusion_trace_summary.json` keeps the static label `gpu_trajectory_noninterference: NOT_TESTED`
  (kroc_eval_adapter.py:365). The run report supersedes it. A record that cites the traces should say so.

**F12 — NOTE (interpretation, not a deviation) — out_last largely tracks the travel direction.**
- In each condition out_last is close to travel_out:
  - A: 48.8 vs 52.1;
  - B_out: 72.1 vs 70.2;
  - C: 72.9 vs 70.7.
- At about 0.2 m from the surface, the bearing of the nearest bar mostly reflects which way the vehicle was moving.
  out_last alone does not separate "a bar never seen" from "a bar hit along an outside-FOV direction of travel".
- never_in_1s (A 33.7 % → C 56.7 %) is the more specific evidence for the blind-side claim.
- B_out reproduces C almost exactly (72.1 vs 72.9). Removing the out-of-FOV LiDAR alone accounts for the shift.
- The wording should stay "consistent with" (AGENTS.md wording rules).

## Bias checks (task item 4), summary

| Check | Result |
| --- | --- |
| Frame before contact missing | Not observed: frames == `actions` and contiguous from 0 in all 33,117 ledger episodes |
| Episodes without traces or layouts | 0 crash and 0 ledger rows without frames or layout. Ledger and KROC outcomes agree on all rows |
| Episode index alignment (trace vs ledger) | Captures: last-frame pursuer–target distance median 0.58 m, max 0.95 m. Crashes: median surface distance 0.21–0.23 m |
| Bars of different heights or orientations | All bars 0–3 m, identity orientation, boxes; footprints vary (half 0.20–0.40 m), handled exactly by the box model |
| No bar spans the vehicle height (argmin over all-inf would pick bar 0) | 0 cases |
| Ambiguous contact (second bar within 0.1 m) | A 2, B_out 84, C 60; excluding them changes out_last by < 0.1 pp |
| Yaw and bearing sign | xyzw quaternion yaw; atan2(y, x), z-up, so left is positive; consistent with the ablation's own FOV mask (navrl_sensor_ablation.py:61) |
| Yaw source lag, camera origin, quota set, short episodes | F5, F4, F3, F7: none changes the reading |

## Reviewer files

| File | SHA-256 |
| --- | --- |
| `recompute.py` | `6e4480d4aa52b60c57d0905d99e109cf6e2a9f71fde65aa78aa2696b59fe6cdf` |
| `recompute_v2.py` | `6c472166696c82de5847df8f23d4ee822779a36665b35c40430f33198686fe83` |
| `supplement.py` | `00463fe500f8df9a3fa7147dd7fd32f76cbfe1f2391acfc833b2927360bfb6bd` |
| `recompute_out.json` | `78fe0657b482f8b73e3b1b73c640b83a94268de325d2116eaf17a0c732e27a18` |
| `recompute_out_v2.json` | `91bcbfe9b8d3c8e2831008c3b40d4c7f816f7587ef7bd238e5b85f3ac45b91a9` |
| `supplement_out.json` | `07cd1d32d4b14cc01676fbd4b8146294d466c5726b65dd28ae4f41a0833a1835` |

No file inside `/home/fair/workspaces/` was created, modified or deleted. Only read-only git commands were run
(`log`, `show`, `diff`, `status`, `merge-base`).
