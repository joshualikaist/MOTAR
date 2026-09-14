# Public release status — 2026-09-14

Release-layer follow-up to [the earlier audit](public_release_audit_2026-09-14.md), not a new
scientific result or legal clearance. Final machine audit and test receipts are linked below
when completed. No policy, detector, controller, threshold or scientific result is changed.

| Axis | Status / boundary |
| --- | --- |
| CURRENT_TREE_DATASET_REDISTRIBUTION | Removal prepared: exact 13 ETH JPEGs and one ZIP in the [inventory](eth_ds5_public_release_inventory_2026-09-14.md) |
| HISTORICAL_GIT_DATASET_REACHABILITY | OPEN: removal does not rewrite history |
| LICENSE_STATUS | ETH ds5 CC BY-NC-SA 4.0; not covered by root BSD. No blanket clearance |
| REPRODUCIBILITY_STATUS | EXTERNAL_DATA_REQUIRED; numerical evidence retained; exact old review pack remains unreproducible due to missing complete centres |
| OPEN_CONFIRMATIONS | NavRL copy/adaptation provenance, fork asset ownership, Det-Fly hosted-data terms, NPS derived-content provenance |

## Retained confirmations, not inferred away

- **NavRL:** source-name/comment/history searches show numerous NavRL-named local adaptations,
  but a name is not proof of copying or its absence. No reliable line-by-line upstream comparison
  or complete copied-file inventory has been established. **NEEDS_CONFIRMATION**, not NOT_REDISTRIBUTED.
- **Fork copyright:** root LICENSE still names Autonomous Robots Lab, NTNU. NOTICE credits the
  fork without inventing a legal owner. Git authorship is not ownership certification.
  Asset-level and contributor ownership remains **NEEDS_CONFIRMATION**.
- **Det-Fly:** the repository MIT licence does not establish terms for separately hosted images
  or annotations. No new permission obtained: **NEEDS_CONFIRMATION / BLOCKED_BY_LICENSE** remains.
- **NPS-Drones:** no tracked JPEGs remain after ETH removal; tracked PNGs are diagram/plot/site
  screenshot paths, not a tracked raw NPS directory. Arrays, models and adapted/embedded content
  are not cleared by a filename search. Exact-content ZIP inspection is documented separately;
  full dataset-derived provenance remains **NEEDS_CONFIRMATION**.

Corrections to the earlier audit: its claim that all 18 weights were project-trained was too
broad. The existing third-party inventory distinguishes seven project weights from eleven
inherited examples. Project generation alone does not clear training-data or third-party rights.
Archive totals below include `.zip/.tar/.gz/.7z`, not ZIP alone; no Git-history size reduction is claimed.

## Release choices when historical blobs remain

A. Preserve this history and state that only the current tree excludes data.
B. With separate approval, export permitted files to a **clean public-release repository**.
   This is the recommended choice if distribution of historical image blobs must stop while
   preserving the research repository.
C. Rewrite history only with explicit approval and coordinated migration.

Neither B nor C is executed here. A normal push preserves historical reachability.

## Verification

Pending final current-HEAD inventory and full repository regression. Historical receipts and
summaries are not rewritten to reflect availability; [external_data_manifest.json](external_data_manifest.json)
is the release-layer source of truth. Missing unrelated required evidence must remain an error.
