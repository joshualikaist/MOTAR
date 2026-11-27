# Independent Fable 5.1 read-only review

Verified CLI canonical model `claude-fable-5-1`; source `e45794e`, result `d063e7d`.
Full returned text follows verbatim. The caller verified model metadata separately; see
[fable_review_metadata.json](fable_review_metadata.json) and [review raw manifest](FABLE_RAW_MANIFEST.json).
Follow-up disposition is recorded in [QUALIFICATION.md](QUALIFICATION.md).

---

**Verdict: SUBSTANTIATED_GO** for the preregistered, precision-limited simulator scope of S-L1C v2 (candidate V2-A), with no unresolved blocker. One disclosure gap should be fixed in the evidence text (finding 1), but it does not change any gate outcome.

## What I inspected and ran

Commits: base `aed6aff`, preregistration `51a9015`, oracle `396ca61`, CPU result `483e0c2`, amendments `d66acd6`/`0c2bddc`, runtime `5b57021`, GPU source `e45794e`, result `d063e7d` (HEAD). The plan JSON, oracle and CPU tool are byte-identical from their first commit to HEAD. Production, tools and tests are unchanged between the GPU source commit and HEAD apart from the post-run report script. The commit timestamps place the oracle before the CPU run and the GPU source before the GPU run.

Files read in full: the three production diffs, `navrl_sensor_lineage.py` v1 association/covariance, the perception call site and clock block, `s_l1c_v2_reference.py`, `s_l1c_reference.py`, `s_l1c_v2_cpu.py`, launcher, run, analysis, report tools, all three `test_s_l1c_v2*.py`, plan/amendments, CPU_SELECTION, RESULTS, cpu_validation, preservation, and the JSON fields of gpu_metrics, cpu_selection_summary, GPU_RUN_REPORT and the analysis.

Independent runs on CPU (all outputs under `/tmp/motar_s_l1c_v2_fable_review`, originals untouched):

| Check | Observed |
| --- | --- |
| Targeted `test_s_l1c_v2*` suite | 22 ran, OK |
| Full PUBLIC_CPU profile | 2593 ran, 0 failures, 0 errors, 10 skipped, 5 excluded |
| Fresh rerun of `s_l1c_v2_analysis.py` on GPU raw | SHA-256 identical to committed `s_l1c_v2_analysis.json` (15149c88…), all nine gates PASS |
| GPU and CPU RAW_MANIFEST members rehashed | 284/284 and 2/2 match; committed manifest copies and run report hash-identical to raw |
| CPU ledger `matched.csv.gz` streamed and recounted | 426,240 rows = 106,560 matched keys × 4 candidates; paired true losses 0 for every candidate; new false accepts 0 for V2-A/V2-B, 17,919 for V2-C |
| Own NumPy sphere-intersection truth on GPU positive packets (not the repo oracle) | max deviation from rendered scan 2.77e-5 m; semantic IDs consistent with analytic hits in all 144 scenes |
| Own recount of GPU positives | near-range 140/288 v1, 264/288 v2; 0 paired losses; 0 new false accepts; every v1 acceptance kept identical row and centre range |
| GPU negatives, both gate priors (0.01 and 1.0) | 0 acceptances of 186 probes per lineage; the seven EQ1D replay packets are bit-identical to the frozen `d2_renders.npz` |
| Covariance ratios recomputed with my own basis | radial posterior/prior ≤ 0.00682, orthogonal ≥ 0.99960 on all 548 accepted updates |
| Clock arrays | identity age 0.1…1.0 s over ten LiDAR-only steps, active after 50 misses, expired on the 51st, camera reset to 0 |
| Firewall replays | 0 mismatches across all GT/semantic replay arrays; actor firewall 5 branches incl. two reset boundaries, all actions equal, positive controls changed observations |

The test count of 2593 exceeds the recorded 2587 because the five canary tests and one D API test were added in `e45794e`; this matches cpu_validation's note.

## Assessment against the user's requirements

**Positive recall.** Improvement is real, not a denominator artefact. Opportunities are computed from first-hit truth per column independent of the decision; the 72 no-hit probes at 2.3/3 m inter-row elevations are reported, not counted. All 24 remaining v2 misses are the 1.16 m probes with the inward 0.15 m offset, whose 1.01 m distance is below the unchanged 1.1518 m conclusive threshold in both `navrl_sensor_lineage.py:255` and the oracle. CPU: V2-A adds 1,106 (zero noise) and 2,212 (±0.1 mm) correct primary acceptances, 2,084 at ±1 mm, 23 at ±1 cm, 0 at ±3 cm.

**Hard negatives.** No candidate except V2-C adds a false accept at any noise bound on tall, narrow (incl. 0.02 m), foreground or EQ1D sets. The 135 inherited accepts are entirely the 11.5 m tall-bar geometry (5 bearings × 3 offsets × 9 noise patterns), where only one row hits the 3 m bar so v1's vertical-extent test cannot fire; V2-A keeps them as ACCEPT_V1 per the plan's "keep every v1 decision". They are correctly disclosed as unsolved. In the GPU foreground scenes the selected column contains only bar returns, so they exercise the near-surface veto (`navrl_lidar_association_v2.py:56`), which rejected 46 and abstained 47.

**Abstention and noise.** Abstention is explicit and conservative: ±1 cm recovers almost nothing and ±3 cm nothing; existing v1 decisions themselves shift with noise. The GO is correctly bounded to the simulator's precision.

**Range and covariance.** New acceptances use the fitted centre distance, not the prediction (CPU max error 1.8e-14 m at zero noise, 2.1e-5 m at ±0.1 mm; GPU max 1.29e-5 m). The recovered measurement is `fitted*unit` along the track direction (`navrl_lidar_association_v2.py:46,67`), and R is still built by the unchanged `range_only_covariance` from the base `unit_vehicle`, so the innovation is purely radial; the AST test confirms v1 functions are unchanged.

**GT/semantic firewall.** The helper receives only ranges, ray table, prediction, camera flag, active flag and gate (`navrl_perception.py:2071-2073`); semantic labels are used only in evaluation. Confirmed by replay equality on all probes and the task-level actor firewall.

**Clocks.** Identity age resets only on camera (`navrl_perception.py:2276-2279`); v2 touches only the association dispatch (`navrl_perception.py:2067-2070`), and fused age reset flows through the same `tracker.correct(reset_age=…)` call as v1 (`:2078-2080`). Expiry is `age > memory_s` with 5 s (`:692`), i.e. the 51st miss.

**Regression and lineage.** Ten historical 600-step/14-key digest comparisons identical; v2 P1=P2 identical; CPU `observe` equality versus base for off/A/B/C/D passes. `sensor_lineage_level` returns 3 for `s_l1c_v2` (`navrl_sensor_lineage.py:30-31`), so A/B/C are inherited and D stays off, confirmed by the GPU process flags.

## Findings, severity-ranked

1. **Medium, disclosure gap.** The preregistered 1,152-case angular-error diagnostic (±1° bearing or elevation) is only mentioned by count in RESULTS.md and CPU_SELECTION.md. Its numbers matter: V2-0 312/864 opportunities (36.1%), V2-A 504/864 (58.3%) with 648 abstentions and 0 false accepts, new max centre error 5.2 mm. The GPU fixtures vary only the radial offset, so the headline 91.67% is for angularly exact predictions, and the benefit roughly halves at 1° direction error. The plan already forbids claiming this as qualified, but the figures should appear in the evidence text. Not a gate failure.
2. **Low.** Negative coverage is vertical box faces and the EQ1D scenes only. Non-vertical returns (ground, box top edges, adjacent rows hitting different objects) are untested; the 1 mm residual and 1 mm margin make coincidental sphere fits unlikely but unmeasured. The closed-loop gate is adaptive up to 1.0 m (`NAVRL_LIDAR_ASSOC_GATE_M` default 0) while the CPU oracle used 0.696 m; the GPU prior-1.0 probes (gate 1.0 m) showed 0/186 negative acceptances, which partly covers this.
3. **Low.** In the moving closed-loop arm v2 made 431 LiDAR updates against 362 for v1 and trajectories diverge from frame 105; these extra associations are not scored against truth. RESULTS.md labels them robustness observations, which is the right reading.
4. **Informational.** BAR/FG negatives prime the prediction from the 10° row rather than 0° at 5° elevation (argmin tie picks the first index) in both CPU and GPU tools; the scenes remain valid hard negatives.
5. **Informational.** Inherited max centre bias 0.1687 m and the camera centre-range bias persist in v2 because v1 acceptances are unchanged; correctly disclosed.

## Limitations of this review

No GPU execution was possible or attempted; GPU claims rest on raw packets, manifests and my CPU recomputation. Several shell commands were denied by the sandbox and were rerun in equivalent forms; nothing was skipped. I cannot verify the model identity of this session; the caller retains CLI metadata. No hardware realism, Recovery performance or S-L1D-on-v2 validity is implied.

**Blockers:** none. Recommended non-blocking follow-up: add the angular-error diagnostic figures to RESULTS.md before any downstream citation.
