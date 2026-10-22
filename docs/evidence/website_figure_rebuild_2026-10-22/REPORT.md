# MOTAR full-page and publication-figure rebuild

Completed 23 October 2026; research snapshot remains 22 October 2026.
Base: `7689664bbaceae8c1ae6f349e41dbb6871eda3ab`.
Branch: `codex/motar-figure-redesign-20261022`.

## Delivered scope

The project page was rebuilt throughout, replacing the prior incremental BOIA addition.
A new evidence ledger, independent figure atlas and publication stylesheet use the same
six newly drawn figures. README and architecture text now point to the verified family.
Earlier measurements and immutable historical figure packages are preserved.

The six figures are: sensor-to-action system, actor data structure, seven-seed target-motion
replication, exploratory density response, separate sensitivity experiments, and BOIA's
passive research branch. Each has editable SVG, embedded-font PDF and 300 dpi PNG; a manifest
and ZIP provide all 20 artifacts. Neither a fabricated scene nor an AI-generated raster is
used to represent scientific structure or results.

## Corrected errors

- Flat 898-D observation precedes the network's 17 × 64 token sequence.
- The 288-D static scan fuses RGB-D geometry and LiDAR; it is not LiDAR-only.
- Actor z is replaced by task PI altitude hold. Previous-action history is actor action with
  governed XY, not the complete executed altitude command.
- Policy F's baseline is riskcap; arc-clearance is an experimental alternative.
- The feedback loop ends in Lee control / UAV PhysX and next sensor delivery, not an arm grid.
- LiDAR assists target association; camera depth also supports obstacle features.
- Five-slot history and five-second tracker expiry are distinct.
- The safety plot uses seed mean with seed-t interval; the separate pooled historical
  estimate remains in the ledger.
- Density-specific seed effects are shown without inventing new CIs or confirmatory tests.
- BOIA remains a passive branch with no action executor; review FAIL/HOLD remains visible.

Detailed source/function references and limits:
[`figure-redesign-2026-10-22.md`](../../research/figure-redesign-2026-10-22.md).
Two read-only source reviews checked architecture and numerical/statistical provenance;
they found the fused-scan / previous-action wording and interval-label issues, which were
corrected before final verification. This was not a Fable review or an algorithm gate.

## Verification

| Check | Result | Evidence |
| --- | --- | --- |
| Documentation + current/archived page + figure tests | 158 run, 0 failures, 0 errors, 0 skips | `document_tests.json` |
| Public documentation checker | PASS | `python -B tools/check_public_docs.py` |
| Browser preview contracts | PASS | `node tests/test_status_site.js`; `node tests/test_status_interception_episode.js` |
| Desktop/tablet/mobile, three current pages | 9/9 PASS | `browser_preview.json` |
| SVG text clipping and overlap | 6/6 PASS | `browser_preview.json` |
| Independent primary-count comparison | All 112 cell rates equal original counts | current page tests / manifest |
| Re-rendering | 20/20 files byte-identical | `regeneration.json` |
| Historical figure generator | Byte-exact PASS | `tools/build_paper_figures.py --check` |
| Preserved historical assets / preview sources | 108/108 hashes unchanged | `preservation_after.json` |
| Main WIP / stash / research worktree | PASS; main still has 82 changes | preservation receipts |

The historical evidence-layout tests now read an explicitly superseded archived page.
Current-page tests independently verify current links, all six formats/hashes, all 112
cell rates, interval estimands, observation dimensions and GT/shadow boundaries. No old
scientific guard was relaxed to make the new presentation pass.

The browser uses CPU software rendering and no simulator/policy imports. Figure panels
scroll within their own containers on mobile; the page body does not overflow. Public
pages and the main figure atlas work without JavaScript; only the preserved explanatory
browser preview requires it.

## Preservation and limits

Main checkout stays at `f74390a1d44ca264960d39018f5e51710e9b7b0a`, with its 82 changes and
stash unchanged. Research stays at `fde21cdf969a7df380aa1c4deddc1824cb524506`, clean.
No simulator, controller, algorithm, checkpoint, PPO, raw result or preregistration was
written, moved or deleted. Full raw/checkpoint rehash was not repeated; no external raw
backup was created. Disk stayed above the user's 3 GB stopping threshold.

This is a publication-presentation correction. It does not qualify physical safety,
actual occlusion accuracy, persistent identity, recovery efficacy, real flight, or a new
H-R experiment. Q02-B/Q03 and G5 remain HOLD.
