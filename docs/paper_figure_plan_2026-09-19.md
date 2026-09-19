# Paper figure plan — 2026-09-19

Six figures maximum. Each names the claim it carries, the exact data file behind it, whether the
asset exists today, and the section it belongs to. A figure with no data file is not planned; it is
a request for an experiment.

| # | Claim it carries | Data source | Asset status | Section |
|---|---|---|---|---|
| **Fig 1** | The evidence chain is a chain: measurement → injection → policy → safety → observation, with each link measured | `docs/quantitative_positioning_registry.json` | **EXISTS** — `docs/assets/paper/quantitative-positioning-2026-09-18.svg` (generated, `--check`) | §1, §9 |
| **Fig 2** | Perception error is measured on real imagery and its policy cost is quantified | `results/eth_ds5_e3s_2026-09-10/`, `results/perception_p10_seed_replication_2026-09-10/` | **NEEDS BUILD** — no single figure combines E3-S with the P9/P10 chain | §4, §5 |
| **Fig 3** | Safety-filter geometry changes crash rate under matched arms, and the sibling geometry moves the other way | `results/independent_verification_2026-09-07/` | **NEEDS BUILD** | §6 |
| **Fig 4** | Target behaviour is not a monotonic difficulty ladder; H-relative effects with all three seed points | `results/target_motion_e0_e2_2026-09-18/canonical_summary.json` | **EXISTS** — `figures/F1_capture_by_arm_density.png`, `figures/F2_paired_effect_vs_H.png` (generated) | §7 |
| **Fig 5** | Acquisition and visibility order the arms the same way performance does; the E0 failure is at first contact | `docs/results/target_motion_visibility_reacquisition_audit_2026-09-19.json`, `figures/F3_visibility_and_timeout.png`, `figures/F4_density_attenuation.png` | **PARTIAL** — F3/F4 exist; the never-acquired panel is new and needs building | §7 |
| **Fig 6** | Which axes each published system studies, and that matched comparisons are zero — a scope matrix, not a leaderboard | `docs/literature_quantitative_ledger_2026-09-18.json` | **EXISTS** — right panel of the positioning figure | §2, §9 |

## Rules these figures follow

* **Fig 6 is not a leaderboard.** A mark means the axis is studied, never that the work is better.
  The figure states this on its face, and an absent mark explicitly does not mean the work would
  perform poorly.
* **No `closest_nocrash_mean_m` as a primary distance figure.** It is a
  `POST_HOC_CONDITIONAL_DIAGNOSTIC` and may appear only with that label, never as the preregistered
  distance primary — which is `NOT_RECORDED`.
* **Seed points are shown, not hidden.** At n = 3 the three paired effects are the evidence; Fig 4
  plots them individually beside the interval.
* **No significance stars anywhere.** Exact permutation inference has insufficient resolution at
  n = 3; figures carry intervals and effect sizes.
* **Exploratory panels are labelled on the figure**, not only in the caption. The density
  attenuation panel carries `EXPLORATORY` in its title.
* **Generated, not drawn.** Every existing asset is produced by a tool with a `--check` mode, so a
  stale figure fails rather than misleads.

## Build order if the manuscript proceeds

Fig 2 and Fig 3 are the only genuine gaps, and both are plotting jobs over committed result files —
**no new experiment is required to complete the figure set.** Fig 5's third panel is a bar chart
over the acquisition audit JSON.
